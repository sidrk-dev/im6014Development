"""Intermediate SA03 kinematic and provenance verifier; never physics acceptance."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import posixpath
import re
import sys
from datetime import datetime, timezone
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Path(__file__).resolve().parent))
CONTRACT = ROOT / "docs/single-arm-contract.json"
COMPILER = {"fusestatic": "false", "discardvisual": "false", "strippath": "false"}
TOLERANCE_M = 1e-8
TOLERANCE_FRO = 1e-8
POSE_COUNT = 32
SEED = 0
VERDICT = "PASS: asset integrity and kinematic import only"


def require(condition, message):
    if not condition:
        raise SystemExit(f"FAIL: {message}")


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _norm_posix(value, label):
    require(isinstance(value, str) and value and value == value.strip(), f"{label}: invalid path {value!r}")
    require("\\" not in value, f"{label}: backslash path rejected: {value}")
    require(not value.startswith("/") and not re.match(r"^[A-Za-z][A-Za-z0-9+.-]*://", value), f"{label}: absolute or URI path rejected: {value}")
    require(not re.match(r"^[A-Za-z]:", value), f"{label}: drive path rejected: {value}")
    normalized = posixpath.normpath(value)
    require(normalized not in {"", ".", ".."} and not normalized.startswith("../"), f"{label}: traversal escape rejected: {value}")
    return normalized


def _safe_source(base, value, label):
    normalized = _norm_posix(value, label)
    try:
        target = (base / normalized).resolve(strict=True)
    except OSError:
        raise SystemExit(f"FAIL: {label}: missing file: {value}") from None
    require(target.is_relative_to(base), f"{label}: symlink escape rejected: {value}")
    require(target.is_file(), f"{label}: missing file: {value}")
    return normalized, target


def _display(path):
    resolved = Path(path).resolve()
    try:
        return resolved.relative_to(ROOT.resolve()).as_posix()
    except ValueError:
        return str(resolved)


def _overlaps(left, right):
    return left == right or left in right.parents or right in left.parents


def _check_hash_syntax(value, label):
    require(isinstance(value, str) and re.fullmatch(r"[0-9a-f]{64}", value) is not None, f"{label}: invalid SHA-256")


def verify_assets(prototype_dir, contract):
    base = Path(prototype_dir).resolve()
    require(base.is_dir(), f"missing prototype directory: {base}")
    asset = contract["profiles"]["prototype"]["asset"]
    _, urdf = _safe_source(base, asset["cleaned_urdf"]["file"], "cleaned_urdf")
    _, model_path = _safe_source(base, asset["model_json"]["file"], "model_json")
    require(digest(urdf) == asset["cleaned_urdf"]["sha256"], "prototype URDF hash changed")
    require(digest(model_path) == asset["model_json"]["sha256"], "prototype model JSON hash changed")

    model = json.loads(model_path.read_text())
    cleaned = model["cleaned_urdf"]
    source = model["source"]
    require(cleaned["file"] == asset["cleaned_urdf"]["file"], "model cleaned URDF filename disagrees with contract")
    require(cleaned["sha256"] == asset["cleaned_urdf"]["sha256"], "model cleaned URDF hash disagrees with contract")
    require(source["urdf_sha256"] == asset["source_urdf"]["sha256"], "model source URDF hash disagrees with contract")
    require(source["archive_sha256"] == asset["source_archive"]["sha256"], "model source archive hash disagrees with contract")

    root = ET.parse(urdf).getroot()
    visual_refs = [(visual, mesh.get("filename")) for visual in root.findall(".//visual") for mesh in visual.findall(".//mesh")]
    collision_refs = [(collision, mesh.get("filename")) for collision in root.findall(".//collision") for mesh in collision.findall(".//mesh")]

    hashes = {}
    portable = {}
    require(isinstance(model["portable_meshes"], dict) and model["portable_meshes"], "portable_meshes must be a nonempty object")
    for key, record in model["portable_meshes"].items():
        normalized = _norm_posix(key, "portable_meshes key")
        require(isinstance(record, dict), f"portable_meshes {key}: record is not an object")
        require(_norm_posix(record["file"], f"portable_meshes {key} file") == normalized, f"portable_meshes {key}: key/file disagree")
        _check_hash_syntax(record["sha256"], f"portable_meshes {key}")
        _, path = _safe_source(base, normalized, f"portable_meshes {key}")
        actual = hashes.setdefault(normalized, digest(path))
        require(actual == record["sha256"], f"portable_meshes {key}: digest mismatch")
        portable[normalized] = {"path": normalized, "absolute": path, "sha256": actual, "record": record}

    meshes = {}
    mesh_map = {}
    for role, references in (("urdf_visual", visual_refs), ("urdf_collision", collision_refs)):
        for element, filename in references:
            normalized = _norm_posix(filename, f"{role} mesh")
            require(normalized in portable, f"{role}: mesh missing from portable_meshes: {normalized}")
            expected = portable[normalized]
            require(digest(expected["absolute"]) == expected["sha256"], f"{role}: digest changed: {normalized}")
            row = meshes.setdefault(normalized, {"path": normalized, "absolute": expected["absolute"], "sha256": expected["sha256"], "roles": set(), "reference_count": 0})
            row["roles"].add(role)
            row["reference_count"] += 1
            mesh_map[filename] = str(expected["absolute"])

    body_blobs = []
    require(isinstance(model["bodies"], dict) and model["bodies"], "bodies must be a nonempty object")
    for body, roles in model["bodies"].items():
        require(isinstance(roles, dict) and roles, f"bodies {body}: expected nonempty role records")
        for role, record in roles.items():
            normalized = _norm_posix(record["file"], f"body {body}.{role}")
            _check_hash_syntax(record["sha256"], f"body {body}.{role}")
            _, path = _safe_source(base, normalized, f"body {body}.{role}")
            actual = hashes.setdefault(normalized, digest(path))
            require(actual == record["sha256"], f"body {body}.{role}: digest mismatch")
            body_blobs.append({"body": body, "role": role, "path": normalized, "sha256": actual})

    metadata = {"bytes_verified": False, "status": "metadata only; source file not supplied"}
    return {
        "urdf_path": urdf, "model_path": model_path,
        "urdf_sha256": digest(urdf), "model_json_sha256": digest(model_path), "model": model,
        "mesh_map": mesh_map,
        "meshes": [{"path": row["path"], "sha256": row["sha256"], "roles": sorted(row["roles"]), "reference_count": row["reference_count"]} for row in sorted(meshes.values(), key=lambda item: item["path"])],
        "portable_meshes": [{"path": row["path"], "sha256": row["sha256"]} for row in sorted(portable.values(), key=lambda item: item["path"])],
        "body_blobs": sorted(body_blobs, key=lambda item: (item["body"], item["role"])),
        "visual_elements": len(root.findall(".//visual")), "collision_elements": len(root.findall(".//collision")),
        "visual_mesh_references": len(visual_refs), "collision_mesh_references": len(collision_refs),
        "source_hashes": {
            "urdf": {"file": asset["source_urdf"]["file"], "sha256": asset["source_urdf"]["sha256"], **metadata},
            "archive": {"file": asset["source_archive"]["file"], "sha256": asset["source_archive"]["sha256"], **metadata},
        },
        "model_status": {"schema": model.get("schema"), "units": model.get("units"), "placeholder_inertials": model.get("placeholder_inertials"),
                         "dynamics_validated": model.get("dynamics_validated"), "collision_validated": model.get("collision_validated")},
    }


def _visual_name(link_index, visual_index):
    return f"sa03_visual_{link_index:03d}_{visual_index:02d}"


def create_import(verified, output_dir):
    source = Path(verified["urdf_path"]).resolve(strict=True)
    destination = Path(output_dir).resolve() / "arm.sa03.urdf"
    require(destination.parent.is_dir(), f"missing import output directory: {destination.parent}")
    tree = ET.parse(source)
    root = tree.getroot()
    require(root.find("mujoco") is None, "source URDF already contains a mujoco block")
    names = 0
    for link_index, link in enumerate(root.findall("link")):
        for visual_index, visual in enumerate(link.findall("visual")):
            visual.set("name", _visual_name(link_index, visual_index))
            names += 1
    rewrites = []
    for mesh in root.findall(".//mesh"):
        original = mesh.get("filename")
        require(original in verified["mesh_map"], f"unverified mesh reference: {original}")
        mesh.set("filename", verified["mesh_map"][original])
        rewrites.append({"from": original, "to": verified["mesh_map"][original]})
    mujoco = ET.SubElement(root, "mujoco")
    ET.SubElement(mujoco, "compiler", dict(COMPILER))
    payload = ET.tostring(root, encoding="utf-8", xml_declaration=True)
    temporary = destination.with_name(destination.name + ".tmp")
    if temporary.exists() or temporary.is_symlink():
        temporary.unlink()
    with temporary.open("xb") as handle:
        handle.write(payload)
    os.replace(temporary, destination)
    generated_digest = hashlib.sha256(payload).hexdigest()
    require(digest(destination) == generated_digest, "generated import digest mismatch")
    return {
        "urdf_path": destination,
        "sha256": generated_digest,
        "compiler": dict(COMPILER),
        "visual_names_assigned": names,
        "mesh_path_rewrites": rewrites,
    }


def _urdf_root(root):
    children = {joint.find("child").get("link") for joint in root.findall("joint")}
    candidates = [link.get("name") for link in root.findall("link") if link.get("name") not in children]
    require(len(candidates) == 1, f"expected one URDF root link, found {candidates}")
    return candidates[0]


def inspect_import(original_urdf, imported_urdf, contract):
    import mujoco

    original_root = ET.parse(Path(original_urdf)).getroot()
    links = [link.get("name") for link in original_root.findall("link")]
    require(len(links) == len(set(links)), "duplicate original link names")
    moving = {joint.get("name") for joint in original_root.findall("joint") if joint.get("type") in {"revolute", "continuous"}}
    expected_joints = [row["urdf_joint"] for row in contract["joints"]]
    require(len(expected_joints) == 3 and set(expected_joints) == moving, "contract joints disagree with original URDF")
    require(not original_root.findall(".//collision"), "collision geometry is outside this visual-only import probe")
    root_name = _urdf_root(original_root)

    try:
        model = mujoco.MjModel.from_xml_path(str(Path(imported_urdf).resolve(strict=True)))
    except Exception as error:
        raise SystemExit(f"FAIL: MuJoCo import failed: {error}") from error
    require(model.nq == model.nv == 3, f"imported model must have nq=nv=3, got {model.nq}/{model.nv}")
    require(model.njnt == 3, f"imported model must have exactly three joints, got {model.njnt}")

    joint_map = {}
    for index in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, index)
        require(name in expected_joints, f"unexpected imported joint: {name}")
        require(name not in joint_map, f"duplicate imported joint: {name}")
        require(model.jnt_type[index] == mujoco.mjtJoint.mjJNT_HINGE, f"joint {name} is not a hinge")
        joint_map[name] = {"qposadr": int(model.jnt_qposadr[index]), "dofadr": int(model.jnt_dofadr[index])}
    require(set(joint_map) == set(expected_joints), "missing named imported joints")

    body_names = {}
    for index in range(1, model.nbody):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, index)
        require(name and name not in body_names, f"unnamed or duplicate imported body at index {index}")
        body_names[name] = index
    link_map = {}
    root_mapping = "named body"
    for link in links:
        if link in body_names:
            link_map[link] = body_names[link]
        elif link == root_name:
            link_map[link] = 0
            root_mapping = "world"
        else:
            raise SystemExit(f"FAIL: imported body missing for link {link}")
    expected_bodies = set(links) - ({root_name} if root_mapping == "world" else set())
    require(set(body_names) == expected_bodies, f"imported body set mismatch; extra={sorted(set(body_names) - expected_bodies)} missing={sorted(expected_bodies - set(body_names))}")
    link_bodies = {link: ("world" if body_id == 0 else mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)) for link, body_id in link_map.items()}

    expected_visuals = {}
    expected_counts = {}
    for link_index, link in enumerate(original_root.findall("link")):
        link_name = link.get("name")
        expected_counts[link_name] = len(link.findall("visual"))
        for visual_index in range(expected_counts[link_name]):
            expected_visuals[_visual_name(link_index, visual_index)] = link_name
    require(model.ngeom == len(expected_visuals), f"compiled visual count {model.ngeom} != source {len(expected_visuals)}")
    compiled = {}
    for index in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, index)
        require(name in expected_visuals, f"unexpected or unnamed compiled geom: {name}")
        require(name not in compiled, f"duplicate compiled geom name: {name}")
        body_id = int(model.geom_bodyid[index])
        body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
        require(body_name == expected_visuals[name], f"visual {name} belongs to {body_name}, expected {expected_visuals[name]}")
        require(int(model.geom_contype[index]) == 0 and int(model.geom_conaffinity[index]) == 0, f"visual {name} has contacts enabled")
        compiled[name] = {"link": body_name, "body_id": body_id}
    require(set(compiled) == set(expected_visuals), "compiled visuals do not match source visual set")
    compiled_counts = {link: 0 for link in links}
    for entry in compiled.values():
        compiled_counts[entry["link"]] += 1
    require(compiled_counts == expected_counts, "compiled per-link visual ownership mismatch")

    return {"model": model, "joint_map": joint_map, "link_map": link_map, "link_bodies": link_bodies,
            "root_mapping": root_mapping, "root_name": root_name, "links": links, "visual_counts": expected_counts,
            "compiled_visual_count": model.ngeom, "original_visual_count": len(expected_visuals), "original_collision_count": 0,
            "nq": int(model.nq), "nv": int(model.nv), "njnt": int(model.njnt)}


def _transform(position, matrix):
    import numpy as np

    transform = np.eye(4)
    transform[:3, :3] = matrix
    transform[:3, 3] = position
    return transform


def compare_fk(original_urdf, imported_urdf, contract):
    import numpy as np
    import yourdfpy

    imported = inspect_import(original_urdf, imported_urdf, contract)
    model = imported["model"]
    original = yourdfpy.URDF.load(str(Path(original_urdf)), load_meshes=False, build_scene_graph=True, build_collision_scene_graph=False, load_collision_meshes=False)
    require(set(original.link_map) == set(imported["links"]), "yourdfpy link set disagrees with the original URDF")
    actuated = set(original.actuated_joint_names)
    expected = {row["urdf_joint"] for row in contract["joints"]}
    require(actuated == expected, f"yourdfpy moving joints {sorted(actuated)} != contract {sorted(expected)}")

    import mujoco

    rng = np.random.default_rng(SEED)
    samples = np.vstack([np.zeros((1, 3)), rng.uniform(-0.5, 0.5, size=(POSE_COUNT, 3))])
    require(len(samples) == POSE_COUNT + 1, "pose count mismatch")
    data = mujoco.MjData(model)
    root_id = imported["link_map"][imported["root_name"]]
    maxima = {"translation_m": 0.0, "rotation_fro": 0.0}
    worst = {"translation_m": None, "rotation_fro": None}
    per_pose = []
    poses = []
    for pose_index, values in enumerate(samples):
        configuration = {row["urdf_joint"]: float(value) for row, value in zip(contract["joints"], values)}
        original.update_cfg(configuration)
        data.qpos[:] = 0.0
        for row, value in zip(contract["joints"], values):
            data.qpos[imported["joint_map"][row["urdf_joint"]]["qposadr"]] = float(value)
        mujoco.mj_forward(model, data)
        root_transform = _transform(data.xpos[root_id], data.xmat[root_id].reshape(3, 3))
        root_inverse = np.linalg.inv(root_transform)
        pose_max = {"translation_m": 0.0, "rotation_fro": 0.0}
        for link in imported["links"]:
            body_id = imported["link_map"][link]
            matrix = _transform(data.xpos[body_id], data.xmat[body_id].reshape(3, 3))
            matrix = root_inverse @ matrix
            reference = original.get_transform(link, imported["root_name"])
            translation = float(np.linalg.norm(matrix[:3, 3] - reference[:3, 3]))
            rotation = float(np.linalg.norm(matrix[:3, :3] - reference[:3, :3], ord="fro"))
            require(np.isfinite(translation) and np.isfinite(rotation), f"non-finite FK error at pose {pose_index}, link {link}")
            if translation > maxima["translation_m"]:
                maxima["translation_m"] = translation
                worst["translation_m"] = {"link": link, "pose": pose_index}
            if rotation > maxima["rotation_fro"]:
                maxima["rotation_fro"] = rotation
                worst["rotation_fro"] = {"link": link, "pose": pose_index}
            pose_max["translation_m"] = max(pose_max["translation_m"], translation)
            pose_max["rotation_fro"] = max(pose_max["rotation_fro"], rotation)
            require(translation <= TOLERANCE_M, f"FK translation mismatch at pose {pose_index}, link {link}: {translation} > {TOLERANCE_M}")
            require(rotation <= TOLERANCE_FRO, f"FK rotation mismatch at pose {pose_index}, link {link}: {rotation} > {TOLERANCE_FRO}")
        per_pose.append({"pose": pose_index, **pose_max})
        poses.append({"pose": pose_index, "values_rad": {row["name"]: float(value) for row, value in zip(contract["joints"], values)}})
    require(len(per_pose) == POSE_COUNT + 1, "per-pose evidence count mismatch")
    return {"joint_order": [row["name"] for row in contract["joints"]], "urdf_joint_order": [row["urdf_joint"] for row in contract["joints"]],
            "joint_map": imported["joint_map"], "pose_count": len(poses), "seed": SEED, "pose_bounds_rad": [-0.5, 0.5],
            "poses": poses, "pose_note": "mathematical probe values; not commissioned physical envelopes", "tolerances": {"translation_m": TOLERANCE_M, "rotation_fro": TOLERANCE_FRO},
            "maxima": maxima, "worst": worst, "per_pose_maxima": per_pose}


def _prepare_output(prototype_dir, output_dir):
    source = Path(prototype_dir).resolve()
    candidate = Path(output_dir)
    if not candidate.is_absolute():
        candidate = ROOT / candidate
    target = candidate.resolve()
    checkout = ROOT.resolve()
    results = (checkout / "results").resolve()
    require(results == checkout / "results", "results directory must not redirect to another directory")
    require(target.is_relative_to(results), f"output directory must resolve under {results}")
    require(not _overlaps(target, source), "output directory overlaps prototype source")
    require(not target.exists() or target.is_dir(), f"output path is not a directory: {target}")
    target.mkdir(parents=True, exist_ok=True)
    report = target / "report.json"
    if report.is_symlink() or report.exists():
        require(report.is_file() or report.is_symlink(), f"stale report path is not a file: {report}")
        report.unlink()
    return source, target, report


def _atomic_write(path, text):
    path = Path(path)
    temporary = path.with_name(path.name + ".tmp")
    if temporary.exists() or temporary.is_symlink():
        temporary.unlink()
    with temporary.open("xb") as handle:
        handle.write(text.encode("utf-8"))
    os.replace(temporary, path)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--prototype-dir", type=Path, required=True, help="Directory containing the local arm.urdf and model.json")
    parser.add_argument("--output-dir", type=Path, required=True, help="Ignored results directory for the generated import and report")
    args = parser.parse_args(argv)
    source, output, report = _prepare_output(args.prototype_dir, args.output_dir)
    try:
        require(source.is_dir(), f"missing prototype directory: {source}")
        require(CONTRACT.is_file(), f"missing contract: {CONTRACT}")
        contract = json.loads(CONTRACT.read_text())
        from check_single_arm_contract import check_contract, check_prototype

        checks = check_contract(contract)
        prototype = check_prototype(source, contract)
        assets = verify_assets(source, contract)
        generated = create_import(assets, output)
        imported = inspect_import(assets["urdf_path"], generated["urdf_path"], contract)
        kinematics = compare_fk(assets["urdf_path"], generated["urdf_path"], contract)
        import numpy
        import mujoco
        import yourdfpy

        blockers = [
            "placeholder inertials: the imported inertia values are not validated physics",
            "no collision geometry or collision validation", "tool reference frame is not frozen",
            "overhead camera extrinsics are not frozen", "marker geometry is not frozen", "SA03 physics acceptance remains pending",
        ]
        result = {
            "schema_version": 1, "recorded_at_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
            "verdict": VERDICT, "scope": "intermediate asset provenance and kinematic import probe; not physics acceptance",
            "contract": {"path": _display(CONTRACT), "sha256": digest(CONTRACT), "checks": checks},
            "prototype": prototype,
            "assets": {
                "urdf": {"path": _display(assets["urdf_path"]), "sha256": assets["urdf_sha256"], "bytes_verified": True},
                "model_json": {"path": _display(assets["model_path"]), "sha256": assets["model_json_sha256"], "bytes_verified": True},
                "source": assets["source_hashes"], "visual_elements": assets["visual_elements"],
                "collision_elements": assets["collision_elements"], "visual_mesh_references": assets["visual_mesh_references"],
                "collision_mesh_references": assets["collision_mesh_references"], "meshes": assets["meshes"],
                "portable_meshes": assets["portable_meshes"], "body_blobs": assets["body_blobs"], "model_status": assets["model_status"],
            },
            "import": {"path": _display(generated["urdf_path"]), "sha256": generated["sha256"], "compiler": generated["compiler"],
                       "visual_names_assigned": generated["visual_names_assigned"], "mesh_path_rewrites": generated["mesh_path_rewrites"]},
            "structure": {
                "nq": imported["nq"], "nv": imported["nv"], "njnt": imported["njnt"], "root_mapping": imported["root_mapping"],
                "joint_map": imported["joint_map"], "link_count": len(imported["links"]), "link_map": imported["link_bodies"],
                "original_visual_count": imported["original_visual_count"], "compiled_visual_count": imported["compiled_visual_count"],
                "original_collision_count": imported["original_collision_count"], "visual_counts_per_link": imported["visual_counts"],
            },
            "kinematics": kinematics,
            "versions": {"python": platform.python_version(), "python_implementation": platform.python_implementation(),
                         "numpy": numpy.__version__, "mujoco": mujoco.__version__, "yourdfpy": yourdfpy.__version__},
            "sa03_accepted": False, "dynamics_validated": False, "collision_validated": False,
            "tool_frame_frozen": False, "camera_extrinsics_frozen": False, "marker_geometry_frozen": False,
            "hosted_calls": 0, "motor_calls": 0, "unresolved_blockers": blockers,
        }
        _atomic_write(report, json.dumps(result, indent=2) + "\n")
    except BaseException:
        if report.is_symlink() or report.exists():
            report.unlink()
        raise
    print(f"{VERDICT}; links={len(imported['links'])}, poses={kinematics['pose_count']}, max_translation={kinematics['maxima']['translation_m']:.3e}, max_rotation_fro={kinematics['maxima']['rotation_fro']:.3e}")


if __name__ == "__main__":
    main()
