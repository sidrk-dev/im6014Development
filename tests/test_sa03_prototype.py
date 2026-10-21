"""Tests for the intermediate SA03 kinematic/provenance checker."""

import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import importlib.util
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import check_sa03_prototype as sa03


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def contract_for(joints):
    return {"joints": [
        {"index": index, "name": f"J{index + 1}", "urdf_joint": joint, "role": role, "motor_id": index, "unit": "rad"}
        for index, (joint, role) in enumerate(joints)
    ]}


def inertial():
    return '<inertial><mass value="1"/><inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/></inertial>'


def synthetic_urdf(visuals=True):
    visual = '<visual><geometry><box size="0.02 0.02 0.02"/></geometry></visual>' if visuals else ""
    return f"""<?xml version="1.0"?>
<robot name="fixture">
  <link name="base">{inertial()}</link>
  <link name="l1">{inertial()}{visual}</link>
  <link name="l2">{inertial()}{visual}</link>
  <link name="l3">{inertial()}{visual}</link>
  <link name="fixed">{inertial()}{visual}</link>
  <joint name="j1" type="revolute"><parent link="base"/><child link="l1"/><origin xyz="0 0 0.1"/><axis xyz="0 0 1"/><limit lower="-1" upper="1"/></joint>
  <joint name="j2" type="revolute"><parent link="l1"/><child link="l2"/><origin xyz="0 0 0.1"/><axis xyz="0 1 0"/><limit lower="-1" upper="1"/></joint>
  <joint name="j3" type="revolute"><parent link="l2"/><child link="l3"/><origin xyz="0.1 0 0"/><axis xyz="1 0 0"/><limit lower="-1" upper="1"/></joint>
  <joint name="j4" type="fixed"><parent link="l3"/><child link="fixed"/><origin xyz="0 0.05 0"/></joint>
</robot>
"""


def synthetic_fixture(tmp_path):
    urdf = tmp_path / "arm.urdf"
    urdf.write_text(synthetic_urdf())
    return urdf, contract_for([("j1", "base"), ("j2", "shoulder"), ("j3", "elbow")])


def runtime():
    for name in ("numpy", "mujoco", "yourdfpy"):
        if importlib.util.find_spec(name) is None:
            raise unittest.SkipTest(f"run with the pinned simulator environment: missing {name}")


def write_asset_fixture(base, mesh_name="meshes/a.stl", mesh_bytes=b"solid a\nendsolid a\n"):
    (base / "meshes").mkdir(parents=True, exist_ok=True)
    mesh = base / mesh_name
    mesh.parent.mkdir(parents=True, exist_ok=True)
    mesh.write_bytes(mesh_bytes)
    blob = base / "meshes" / "base.bin"
    blob.write_bytes(b"body-blob")
    urdf = base / "arm.urdf"
    urdf.write_text(f"""<robot name="asset"><link name="base"><visual><geometry><mesh filename="{mesh_name}"/></geometry></visual></link></robot>""")
    source_urdf = "0" * 64
    source_archive = "1" * 64
    model = {
        "schema": 1,
        "units": "mm",
        "cleaned_urdf": {"file": "arm.urdf", "sha256": sha(urdf)},
        "source": {"urdf_sha256": source_urdf, "archive_sha256": source_archive},
        "bodies": {"base": {"frame": {"file": "meshes/base.bin", "sha256": sha(blob)}}},
        "portable_meshes": {mesh_name: {"file": mesh_name, "sha256": sha(mesh)}},
        "placeholder_inertials": True,
        "dynamics_validated": False,
        "collision_validated": False,
    }
    model_path = base / "model.json"
    model_path.write_text(json.dumps(model))
    contract = {"profiles": {"prototype": {"asset": {
        "cleaned_urdf": {"file": "arm.urdf", "sha256": sha(urdf)},
        "model_json": {"file": "model.json", "sha256": sha(model_path)},
        "source_urdf": {"file": "mainassem.urdf", "sha256": source_urdf},
        "source_archive": {"file": "mainAssem.zip", "sha256": source_archive},
    }}}}
    return contract, urdf, model_path, mesh, blob

class PrototypeCheckerTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.tmp_path = Path(temporary.name)

    def test_synthetic_import_kinematics_and_repeatability(self):
        tmp_path = self.tmp_path
        runtime()
        urdf, contract = synthetic_fixture(tmp_path)
        out = tmp_path / "out"
        out.mkdir()
        generated = sa03.create_import({"urdf_path": urdf, "mesh_map": {}}, out)
        first = sa03.compare_fk(urdf, generated["urdf_path"], contract)
        second = sa03.compare_fk(urdf, generated["urdf_path"], contract)
        assert first["pose_count"] == 33
        assert first["poses"] == second["poses"]
        assert first["maxima"] == second["maxima"]
        assert first["joint_order"] == ["J1", "J2", "J3"]
        assert first["urdf_joint_order"] == ["j1", "j2", "j3"]
        assert len(first["per_pose_maxima"]) == 33

    def test_mutated_generated_joint_origin_and_axis_fail_independent_fk(self):
        tmp_path = self.tmp_path
        runtime()
        urdf, contract = synthetic_fixture(tmp_path)
        out = tmp_path / "out"
        out.mkdir()
        generated = sa03.create_import({"urdf_path": urdf, "mesh_map": {}}, out)
        tree = ET.parse(generated["urdf_path"])
        joint = next(item for item in tree.getroot().findall("joint") if item.get("name") == "j2")
        joint.find("axis").set("xyz", "0 1 0.001")
        tree.write(generated["urdf_path"], encoding="utf-8", xml_declaration=True)
        with self.assertRaisesRegex(SystemExit, "FK (translation|rotation) mismatch.*pose [1-9]") as error:
            sa03.compare_fk(urdf, generated["urdf_path"], contract)
        assert "l2" in str(error.exception) or "l3" in str(error.exception)

        out_origin = tmp_path / "out-origin"
        out_origin.mkdir()
        generated = sa03.create_import({"urdf_path": urdf, "mesh_map": {}}, out_origin)
        tree = ET.parse(generated["urdf_path"])
        joint = next(item for item in tree.getroot().findall("joint") if item.get("name") == "j2")
        joint.find("origin").set("xyz", "0 0 0.100001")
        tree.write(generated["urdf_path"], encoding="utf-8", xml_declaration=True)
        with self.assertRaisesRegex(SystemExit, "FK (translation|rotation) mismatch") as error:
            sa03.compare_fk(urdf, generated["urdf_path"], contract)
        assert "l2" in str(error.exception) or "l3" in str(error.exception)

    def test_visual_removal_and_missing_link_detected(self):
        tmp_path = self.tmp_path
        runtime()
        urdf, contract = synthetic_fixture(tmp_path)
        out = tmp_path / "out"
        out.mkdir()
        generated = sa03.create_import({"urdf_path": urdf, "mesh_map": {}}, out)
        tree = ET.parse(generated["urdf_path"])
        link = next(item for item in tree.getroot().findall("link") if item.get("name") == "l2")
        link.remove(link.find("visual"))
        tree.write(generated["urdf_path"], encoding="utf-8", xml_declaration=True)
        with self.assertRaisesRegex(SystemExit, "visual count"):
            sa03.inspect_import(urdf, generated["urdf_path"], contract)

        out2 = tmp_path / "out2"
        out2.mkdir()
        generated = sa03.create_import({"urdf_path": urdf, "mesh_map": {}}, out2)
        tree = ET.parse(generated["urdf_path"])
        root = tree.getroot()
        root.remove(next(item for item in root.findall("link") if item.get("name") == "fixed"))
        root.remove(next(item for item in root.findall("joint") if item.get("name") == "j4"))
        tree.write(generated["urdf_path"], encoding="utf-8", xml_declaration=True)
        with self.assertRaisesRegex(SystemExit, "imported body missing for link fixed"):
            sa03.inspect_import(urdf, generated["urdf_path"], contract)

    def test_invalid_inertials_fail_import_without_repair(self):
        tmp_path = self.tmp_path
        runtime()
        urdf = tmp_path / "arm.urdf"
        urdf.write_text(synthetic_urdf().replace('mass value="1"', 'mass value="0"').replace('0.01', '0'))
        contract = contract_for([("j1", "base"), ("j2", "shoulder"), ("j3", "elbow")])
        out = tmp_path / "out"
        out.mkdir()
        generated = sa03.create_import({"urdf_path": urdf, "mesh_map": {}}, out)
        with self.assertRaisesRegex(SystemExit, "MuJoCo import failed.*inertia"):
            sa03.inspect_import(urdf, generated["urdf_path"], contract)

    def test_invalid_mesh_and_body_digest_rejected(self):
        tmp_path = self.tmp_path
        contract, urdf, model_path, mesh, blob = write_asset_fixture(tmp_path)
        assert sa03.verify_assets(tmp_path, contract)["visual_elements"] == 1
        mesh.write_bytes(mesh.read_bytes() + b"x")
        with self.assertRaisesRegex(SystemExit, "meshes/a.stl.*digest mismatch"):
            sa03.verify_assets(tmp_path, contract)

        contract, urdf, model_path, mesh, blob = write_asset_fixture(tmp_path / "second")
        blob.write_bytes(blob.read_bytes() + b"x")
        with self.assertRaisesRegex(SystemExit, "frame.*digest mismatch"):
            sa03.verify_assets(tmp_path / "second", contract)

    def test_mesh_traversal_and_symlink_escape_rejected(self):
        tmp_path = self.tmp_path
        base = tmp_path / "asset"
        contract, urdf, model_path, mesh, blob = write_asset_fixture(base)
        outside = tmp_path / "outside.stl"
        outside.write_bytes(b"outside")
        (base / "meshes" / "escape.stl").symlink_to(outside)
        model = json.loads(model_path.read_text())
        model["portable_meshes"]["meshes/escape.stl"] = {"file": "meshes/escape.stl", "sha256": sha(outside)}
        model_path.write_text(json.dumps(model))
        contract["profiles"]["prototype"]["asset"]["model_json"]["sha256"] = sha(model_path)
        with self.assertRaisesRegex(SystemExit, "symlink escape"):
            sa03.verify_assets(base, contract)

        del model["portable_meshes"]["meshes/escape.stl"]
        (base / "meshes" / "escape.stl").unlink()
        model["portable_meshes"]["../escape.stl"] = {"file": "../escape.stl", "sha256": sha(outside)}
        model_path.write_text(json.dumps(model))
        contract["profiles"]["prototype"]["asset"]["model_json"]["sha256"] = sha(model_path)
        with self.assertRaisesRegex(SystemExit, "traversal escape"):
            sa03.verify_assets(base, contract)

    def test_stale_report_cannot_survive_failed_rerun(self):
        tmp_path = self.tmp_path
        self.enterContext(patch.object(sa03, "ROOT", tmp_path))
        output = tmp_path / "results" / "out"
        output.mkdir(parents=True)
        stale = output / "report.json"
        stale.write_text('{"verdict": "PASS"}')
        with self.assertRaises(SystemExit):
            sa03.main(["--prototype-dir", str(tmp_path / "missing"), "--output-dir", str(output)])
        assert not stale.exists()

    def test_output_symlink_escape_rejected(self):
        tmp_path = self.tmp_path
        self.enterContext(patch.object(sa03, "ROOT", tmp_path))
        (tmp_path / "results").mkdir()
        outside = tmp_path / "outside"
        outside.mkdir()
        (tmp_path / "results" / "escape").symlink_to(outside)
        with self.assertRaisesRegex(SystemExit, "must resolve under"):
            sa03.main(["--prototype-dir", str(tmp_path / "missing"), "--output-dir", "results/escape/run"])
        assert not (outside / "run" / "report.json").exists()

    def test_results_root_cannot_redirect_inside_checkout(self):
        self.assert_root_redirect_rejected(self.tmp_path / "docs")

    def test_results_root_cannot_redirect_outside_checkout(self):
        with tempfile.TemporaryDirectory() as outside:
            self.assert_root_redirect_rejected(Path(outside))

    def assert_root_redirect_rejected(self, destination):
        destination.mkdir(exist_ok=True)
        protected = destination / "existing"
        protected.mkdir()
        report = protected / "report.json"
        report.write_text("protected content")
        (self.tmp_path / "results").symlink_to(destination)
        with patch.object(sa03, "ROOT", self.tmp_path):
            with self.assertRaisesRegex(SystemExit, "must not redirect"):
                sa03.main(["--prototype-dir", str(self.tmp_path / "missing"),
                           "--output-dir", "results/existing"])
            with self.assertRaisesRegex(SystemExit, "must not redirect"):
                sa03.main(["--prototype-dir", str(self.tmp_path / "missing"),
                           "--output-dir", "results/new"])
        self.assertEqual(report.read_text(), "protected content")
        self.assertFalse((destination / "new").exists())


if __name__ == "__main__":
    unittest.main()
