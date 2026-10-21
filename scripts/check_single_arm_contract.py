"""Check and render the frozen single-arm contract."""

import argparse
import hashlib
import json
import math
from datetime import datetime, timezone
from pathlib import Path
import re
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
CONTRACT = ROOT / "docs/single-arm-contract.json"
REFERENCE = ROOT / "docs/single-arm-contract.md"
EXPECTED_JOINTS = [
    ("J1", "revolute_2", "base", 0),
    ("J2", "revolute_1", "shoulder", 1),
    ("J3", "revolute_3", "elbow", 2),
]
EXPECTED_JOINT_DIRECTION = "right-hand rotation about the URDF joint axis"
EXPECTED_JOINT_ZERO = "URDF q=0; physical zero is uncommissioned"
EXPECTED_ASSET_KEYS = {
    "kind",
    "cleaned_urdf",
    "model_json",
    "source_urdf",
    "source_archive",
}
ASSET_HASH_KEYS = ("cleaned_urdf", "model_json", "source_urdf", "source_archive")
EXPECTED_EXCLUSIONS = {
    "passive_coordinates_as_policy_actions",
    "left_arm_channels",
    "absent_gripper_channels",
    "wrist_camera",
    "eef_actions",
    "evaluator_coordinates_in_policy",
    "scripted_task_paths_in_policy",
    "cross_machine_monotonic_comparison",
}


def require(condition, message):
    if not condition:
        raise SystemExit(f"FAIL: {message}")


def require_exact_keys(value, expected, label):
    require(isinstance(value, dict), f"{label} must be an object")
    missing = sorted(expected - set(value))
    unexpected = sorted(set(value) - expected)
    require(not missing and not unexpected, f"{label} keys changed (missing: {missing}, unexpected: {unexpected})")


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def nested_keys(value):
    if isinstance(value, dict):
        for key, child in value.items():
            yield key
            yield from nested_keys(child)
    elif isinstance(value, list):
        for child in value:
            yield from nested_keys(child)


def nested_strings(value):
    if isinstance(value, dict):
        for child in value.values():
            yield from nested_strings(child)
    elif isinstance(value, list):
        for child in value:
            yield from nested_strings(child)
    elif isinstance(value, str):
        yield value


def check_contract(data):
    checks = []
    require(data["schema_version"] == 1, "unsupported contract schema")
    require(data["contract_id"] == "sa01-single-arm-contract-v1", "unexpected contract ID")
    checks.append("contract identity")

    profiles = data["profiles"]
    prototype = profiles["prototype"]
    final = profiles["final"]
    sample = profiles["illustrative_sample"]
    require(profiles["selected"] == prototype["id"] == "sim_prototype_3", "selected profile is not the prototype")
    require(prototype["moving_coordinate_count"] == 3, "prototype must have three moving coordinates")
    require(final["id"] == "sim_final_rolling", "final profile identity changed")
    require(final["moving_coordinate_count"] is None and final["geometry"] is None, "final profile must stay undefined until RC01")
    require(sample["id"] == "sim_rolling_sample" and sample["moving_coordinate_count"] == 2, "sample profile identity changed")
    checks.append("profile separation")

    asset = prototype.get("asset")
    require_exact_keys(asset, EXPECTED_ASSET_KEYS, "prototype asset")
    require(isinstance(asset["kind"], str) and asset["kind"], "prototype asset kind must be recorded")
    for key in ASSET_HASH_KEYS:
        item = asset[key]
        require_exact_keys(item, {"file", "sha256"}, f"prototype asset {key}")
        require(isinstance(item["file"], str) and item["file"], f"prototype asset {key} file must be recorded")
        require(re.fullmatch(r"[0-9a-f]{64}", item["sha256"]) is not None, f"invalid asset hash for {item['file']}")
    checks.append("prototype asset hashes recorded")

    joints = data["joints"]
    actual_joints = [(j["name"], j["urdf_joint"], j["role"], j["motor_id"]) for j in joints]
    require(actual_joints == EXPECTED_JOINTS, "joint order or roles changed")
    require(len(joints) == prototype["moving_coordinate_count"], "joint count does not match the prototype")
    require(all(j["unit"] == "rad" for j in joints), "joint units must be radians")
    require([j["index"] for j in joints] == [0, 1, 2], "joint indexes must be contiguous")
    for joint in joints:
        require(joint["positive_direction"] == EXPECTED_JOINT_DIRECTION, f"{joint['name']} positive direction changed")
        require(joint["zero"] == EXPECTED_JOINT_ZERO, f"{joint['name']} zero changed")
    j1_limits = joints[0]["limits_rad"]
    require(isinstance(j1_limits, list) and len(j1_limits) == 2, "J1 limits must be a two-value provisional interval")
    require(
        all(isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value) for value in j1_limits),
        "J1 limits must be finite numbers",
    )
    lower, upper = j1_limits
    require(lower < 0 < upper, "J1 limits must include zero with a negative lower bound and positive upper bound")
    require(
        math.isclose(lower, -math.pi / 2, rel_tol=0, abs_tol=1e-12)
        and math.isclose(upper, math.pi / 2, rel_tol=0, abs_tol=1e-12),
        "J1 provisional limits must be +/-pi/2",
    )
    require(all(joint["limits_rad"] is None for joint in joints[1:]), "J2 and J3 limits must stay uncommissioned")
    require(data["excluded_coordinates"] == [
        {"urdf_joint": "revolute_4", "reason": "passive joint fixed in the generated prototype; no policy coordinate"}
    ], "passive coordinate must stay excluded")
    checks.append("joint order, direction, limits and units")

    policy = data["policy"]
    embodiment = policy["embodiment"]
    require(policy["model_revision"] == "prototype-cad-v1", "unexpected model revision")
    require("72444bc6" in policy["model_revision_provenance"], "model revision provenance lost")
    require(policy["state_order"] == policy["action_order"] == [j["name"] for j in joints], "policy order does not match joints")
    require(policy["state_units"] == policy["action_units"] == "rad", "policy units must be radians")
    require(policy["action_meaning"] == "absolute joint targets for the next control interval", "action meaning changed")
    require(embodiment["state"] == {"arm": [0, 3]}, "state shape must be one three-wide arm part")
    require(embodiment["action"] == {"arm": [0, 3]}, "action shape must be one three-wide arm part")
    require(embodiment["parts"]["state"]["arm"] == {"kind": "arm", "side": "single"}, "state must be a single arm")
    require(embodiment["parts"]["action"]["arm"] == {"kind": "arm", "side": "single"}, "action must be a single arm")
    require(embodiment["actions"] == "absolute", "actions must be absolute")
    require(policy["internal_mapping"]["public_state_key"] == "state.arm", "public state key changed")
    require(policy["internal_mapping"]["public_action_key"] == "per_key.action.arm", "public action key changed")
    keys = [key.lower() for key in nested_keys(embodiment)]
    values = [value.lower() for value in nested_strings(embodiment)]
    require(not any("left" in text for text in keys + values), "left-arm channel found")
    require(not any("grip" in text for text in keys + values), "gripper channel found")
    checks.append("single-arm policy interface")

    camera = data["camera"]
    timing = data["timing"]
    require(camera["name"] == camera["role"] == "overhead", "camera role changed")
    require(camera["count"] == 1 and camera["policy_visible"] is True, "exactly one visible policy camera is required")
    require(camera["modality"] == "RGB" and camera["encoding"] == "JPEG", "camera format changed")
    require(embodiment["video"] == {"overhead": "observation.images.overhead"}, "camera mapping changed")
    require(camera["rate_hz"] == timing["camera_rate_hz"] == 30, "camera rate is inconsistent")
    require(camera["history_frames"] == timing["observation_history_frames"] == 1, "camera history is inconsistent")
    require(camera["resolution_px"] == [320, 240], "camera resolution changed")
    require(camera["pose_extrinsics"] is None, "camera pose must stay pending SA03")
    checks.append("overhead camera")

    require(timing["timebase"] == "simulation monotonic clock", "timebase changed")
    require(timing["control_rate_hz"] == 100, "control rate changed")
    require(timing["policy_rate_hz"] == timing["dataset_fps"] == 30, "policy and dataset rates are inconsistent")
    require(timing["max_observation_age_ms"] == 250, "observation age limit changed")
    require(timing["action_chunk_rows"] == timing["execution_prefix_rows"] == 1, "action chunk contract changed")
    require(timing["task_timeout_sim_s"] == 30.0, "task timeout changed")
    checks.append("timing")

    task = data["task"]
    require(task["instruction"] == "Reach the red marker with the fixed tool.", "task instruction changed")
    require(task["tool"]["kind"] == "fixed, non-gripper tool", "tool must stay fixed and gripperless")
    require(task["tool"]["frame"] is None, "tool frame must stay pending SA03")
    require(task["marker"]["geometry"] is None and task["marker"]["geometry_frozen_by"] == "SA03", "marker geometry must stay pending SA03")
    require(task["marker"]["policy_sees_image"] is True, "marker image must be visible to the policy")
    require(task["marker"]["policy_sees_coordinates"] is False, "marker coordinates must stay outside policy")
    success = task["success"]
    require("tool reference point" in success["metric"] and "marker center" in success["metric"], "success metric changed")
    require(success["threshold_m"] == 0.02, "success threshold changed")
    require(success["required_settle_s"] == 1.0, "settle interval changed")
    require(success["requires_no_human_intervention"] is True, "human intervention must invalidate success")
    require(success["requires_reobservation"] is True, "policy must reobserve")
    require(success["count_all_attempts"] is True, "all attempts must be counted")
    checks.append("task and success rule")

    budget = task["training_budget"]
    evaluation = task["evaluation"]
    require(budget["successful_episodes"] == 20 and budget["validation_episodes"] == 5, "training budget changed")
    require("multiple" in budget["starts"] and "multiple" in budget["marker_locations"], "training must vary starts and marker locations")
    require(evaluation["held_out_scenes"] == 10, "evaluation scene count changed")
    require(evaluation["static_scenes"] + evaluation["target_shift_scenes"] == evaluation["held_out_scenes"], "scene split is inconsistent")
    require((evaluation["static_scenes"], evaluation["target_shift_scenes"]) == (6, 4), "scene split changed")
    require((evaluation["successes_required"], evaluation["shifted_successes_required"]) == (8, 3), "evaluation screen changed")
    require(evaluation["frozen_before_training"] is True, "evaluation must be frozen before training")
    require(evaluation["scripted_paths_allowed"] is False, "scripted paths cannot pass the task")
    require(evaluation["hidden_coordinates_outside_policy"] is True, "hidden coordinates must stay outside policy")
    checks.append("training and evaluation budget")

    exclusions = data.get("exclusions")
    require_exact_keys(exclusions, EXPECTED_EXCLUSIONS, "exclusions")
    require(all(value is False for value in exclusions.values()), "an excluded channel was enabled")
    limits = data["evidence_limits"]
    require(limits["contract_only"] is True, "contract scope changed")
    require(not any(limits[key] for key in ("simulation_validated", "hosted_validated", "physical_validated")), "contract cannot claim runtime evidence")
    checks.append("exclusions and evidence limits")
    return checks


def check_prototype(path, data):
    asset = data["profiles"]["prototype"]["asset"]
    urdf = path / asset["cleaned_urdf"]["file"]
    model_path = path / asset["model_json"]["file"]
    require(urdf.is_file(), f"missing prototype URDF: {urdf}")
    require(model_path.is_file(), f"missing prototype model JSON: {model_path}")
    require(digest(urdf) == asset["cleaned_urdf"]["sha256"], "prototype URDF hash changed")
    require(digest(model_path) == asset["model_json"]["sha256"], "prototype model JSON hash changed")

    model = json.loads(model_path.read_text())
    actual = [(j["urdf"], j["role"], j["motor_id"]) for j in model["joints"]]
    expected = [(j["urdf_joint"], j["role"], j["motor_id"]) for j in data["joints"]]
    require(actual == expected, "model joint order does not match the contract")
    require([j["name"].split()[0] for j in model["joints"]] == [j["name"] for j in data["joints"]], "model joint names do not match the policy names")
    require(model["source"]["urdf_sha256"] == asset["source_urdf"]["sha256"], "source URDF hash changed")
    require(model["source"]["archive_sha256"] == asset["source_archive"]["sha256"], "source archive hash changed")
    require(model["placeholder_inertials"] is True, "prototype physics status changed")
    require(model["dynamics_validated"] is False and model["collision_validated"] is False, "prototype physics was silently accepted")

    root = ET.parse(urdf).getroot()
    joints = {j.get("name"): j for j in root.findall("joint")}
    for row in data["joints"]:
        require(row["urdf_joint"] in joints, f"missing URDF joint {row['urdf_joint']}")
        require(joints[row["urdf_joint"]].get("type") in {"revolute", "continuous"}, f"{row['urdf_joint']} is not moving")
    require(joints["revolute_4"].get("type") == "fixed", "passive joint revolute_4 must stay fixed")
    moving = [j for j in root.findall("joint") if j.get("type") in {"revolute", "continuous"}]
    require(len(moving) == 3, "URDF must have exactly three moving coordinates")
    return {
        "verified": True,
        "arm_urdf_sha256": digest(urdf),
        "model_json_sha256": digest(model_path),
        "moving_coordinate_count": len(moving),
    }


def render_doc(data):
    prototype = data["profiles"]["prototype"]
    final = data["profiles"]["final"]
    sample = data["profiles"]["illustrative_sample"]
    policy = data["policy"]
    camera = data["camera"]
    timing = data["timing"]
    task = data["task"]
    lines = [
        "# Single-arm contract",
        "",
        "<!-- Generated by scripts/check_single_arm_contract.py from single-arm-contract.json. -->",
        "",
        f"This contract freezes the first reaching task and the selected prototype profile for SA01. The [JSON source](single-arm-contract.json) is canonical. Change it in a new contract revision, then regenerate this page. Recorded on {data['recorded_on']}.",
        "",
        "## Selected profiles",
        "",
        "| Profile | Status | Moving coordinates |",
        "| --- | --- | --- |",
        f"| `{prototype['id']}` | {prototype['status']} | {prototype['moving_coordinate_count']} |",
        f"| `{final['id']}` | {final['status']} | not defined |",
        f"| `{sample['id']}` | {sample['status']} | {sample['moving_coordinate_count']} |",
        "",
        "## Joints",
        "",
        "| Index | Name | URDF joint | Role | Motor ID | Unit | Limits |",
        "| --- | --- | --- | --- | --- | --- | --- |",
    ]
    for joint in data["joints"]:
        limits = "not commissioned" if joint["limits_rad"] is None else ", ".join(f"{value:.6f}" for value in joint["limits_rad"])
        lines.append(f"| {joint['index']} | {joint['name']} | `{joint['urdf_joint']}` | {joint['role']} | {joint['motor_id']} | {joint['unit']} | {limits} |")
    lines.extend([
        "",
        "Positive values follow right-hand rotation about each URDF joint axis. J1's pair is a provisional display bound centered on an uncommissioned zero, not physical clearance. J2 and J3 are continuous CAD joints with no commissioned limits. `revolute_4` is passive and fixed, so it does not become a policy coordinate.",
        "",
        "## Policy interface",
        "",
        f"The Dream joint-action embodiment uses `state.arm` and `action.arm` with width 3 in the order {', '.join(policy['state_order'])}. Both use radians, and actions are {policy['action_meaning']}.",
        "",
        "The client maps `state.arm` to its internal right-arm slot and returns `per_key.action.arm`. This mapping does not add a left arm or a gripper. The policy observes measured joints, one RGB stream named `overhead`, and this instruction:",
        "",
        f"> {task['instruction']}",
        "",
        "## Camera",
        "",
        "| Setting | Value |",
        "| --- | --- |",
        f"| Name and role | `{camera['name']}` |",
        f"| Count | {camera['count']} |",
        f"| Modality | {camera['modality']}, {camera['encoding']} |",
        f"| Resolution | {camera['resolution_px'][0]}x{camera['resolution_px'][1]} |",
        f"| Rate | {camera['rate_hz']} Hz |",
        f"| History | {camera['history_frames']} frame |",
        f"| Mount | {camera['mount']} |",
        "",
        "## Timing",
        "",
        "| Setting | Value |",
        "| --- | --- |",
        f"| Timebase | {timing['timebase']} |",
        f"| Control rate | {timing['control_rate_hz']} Hz |",
        f"| Policy rate | {timing['policy_rate_hz']} Hz |",
        f"| Camera rate | {timing['camera_rate_hz']} Hz |",
        f"| Dataset rate | {timing['dataset_fps']} Hz |",
        f"| Observation age limit | {timing['max_observation_age_ms']} ms |",
        f"| Action chunk | {timing['action_chunk_rows']} row, execute {timing['execution_prefix_rows']} row |",
        f"| Task timeout | {timing['task_timeout_sim_s']:.0f} s simulation time |",
        "",
        "## Task and success",
        "",
        f"The first task is {task['name'].lower()}. The tool is fixed and has no gripper. The marker is a {task['marker']['appearance']}. Its image is visible to the policy, but its coordinates are not.",
        "",
        f"Success means the {task['success']['metric']} is at most {task['success']['threshold_m'] * 1000:.0f} mm for {task['success']['required_settle_s']:.1f} s of simulation time before the {timing['task_timeout_sim_s']:.0f} s timeout. Human intervention invalidates the trial. All attempts and interventions are reported.",
        "",
        f"Training uses {task['training_budget']['successful_episodes']} successful episodes and {task['training_budget']['validation_episodes']} validation episodes. Evaluation uses {task['evaluation']['held_out_scenes']} held-out scenes: {task['evaluation']['static_scenes']} static and {task['evaluation']['target_shift_scenes']} with a target shift. The screen requires {task['evaluation']['successes_required']} successes, including {task['evaluation']['shifted_successes_required']} shifted cases. SA05 freezes the training scenes and SA10 freezes the held-out cases before training.",
        "",
        "## Exclusions",
        "",
        "- Passive coordinates do not become policy actions.",
        "- No left-arm, absent-gripper, wrist-camera, or EEF channels appear.",
        "- Evaluator coordinates and scripted paths stay outside policy observations.",
        "- Local deadlines use the simulation monotonic clock. Different machines' monotonic clocks are not compared.",
        "",
        "## Evidence limits",
        "",
        "This is a contract-level record. It does not establish simulation physics, hosted model behavior, or physical readiness. SA03 must validate the prototype model and freeze the tool frame and overhead camera pose.",
        "",
    ])
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--prototype-dir", type=Path, help="Directory containing the local arm.urdf and model.json")
    parser.add_argument("--write-doc", action="store_true", help="Regenerate the reference page from the contract")
    parser.add_argument("--report", type=Path, help="Write a JSON verification report")
    args = parser.parse_args()

    require(CONTRACT.is_file(), f"missing contract: {CONTRACT}")
    data = json.loads(CONTRACT.read_text())
    checks = check_contract(data)

    prototype = {"verified": False, "reason": "prototype asset not supplied"}
    if args.prototype_dir:
        prototype = check_prototype(args.prototype_dir, data)
        checks.append("prototype hashes and joints")

    rendered = render_doc(data)
    if args.write_doc:
        REFERENCE.write_text(rendered)
        checks.append("reference page regenerated")
    else:
        require(REFERENCE.is_file(), f"missing reference page: {REFERENCE}")
        require(REFERENCE.read_text() == rendered, "regenerate docs/single-arm-contract.md with --write-doc")
        checks.append("reference page current")

    if args.report:
        result = {
            "recorded_at_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
            "verdict": "PASS",
            "scope": "contract-level only; no simulation, hosted, or physical result",
            "contract": CONTRACT.relative_to(ROOT).as_posix(),
            "contract_sha256": digest(CONTRACT),
            "reference_doc": REFERENCE.relative_to(ROOT).as_posix(),
            "reference_doc_sha256": digest(REFERENCE),
            "checks": checks,
            "prototype_asset": prototype,
        }
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(result, indent=2) + "\n")

    print(f"PASS: SA01 contract, {len(data['joints'])} joints, {data['camera']['count']} overhead camera, {len(checks)} checks")


if __name__ == "__main__":
    main()
