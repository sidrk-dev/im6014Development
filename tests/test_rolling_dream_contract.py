"""Regression tests for the selected rolling-reach Dream profile boundary."""

import copy
from dataclasses import replace
import importlib.util
import json
import math
from pathlib import Path
from unittest.mock import patch
import sys
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

SPEC = importlib.util.spec_from_file_location(
    "rolling_reach_verify_dream", ROOT / "examples/rolling_reach_arm/verify_dream.py"
)
verify_dream = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = verify_dream
SPEC.loader.exec_module(verify_dream)


def frame():
    row = np.arange(320, dtype=np.uint8)
    rgb = np.zeros((240, 320, 3), dtype=np.uint8)
    rgb[:, :, 0] = row
    rgb[:, :, 1] = row[::-1]
    rgb[:, :, 2] = 96
    return rgb


def observation(**changes):
    values = {
        "rgb": frame(),
        "joints": (0.10, 0.20, -0.50),
        "prompt": verify_dream.scene.SCENARIO.instruction,
        "timestamp": 0.5,
    }
    values.update(changes)
    return verify_dream.Observation(**values)


class RollingDreamContractTests(unittest.TestCase):
    def setUp(self):
        self.manifest = verify_dream.scene.effective_manifest()

    def test_canonical_profile_manifest_and_registration_pass(self):
        verify_dream.check_profile(verify_dream.PROFILE)
        verify_dream.check_manifest(verify_dream.PROFILE, self.manifest)
        config = verify_dream.embodiment()
        body = verify_dream.registered_body(config)
        verify_dream.check_registration(verify_dream.PROFILE, body)
        self.assertEqual([slot.name for slot in body.plan.state], ["right_arm"])
        self.assertEqual([slot.width for slot in body.plan.action], [3])
        self.assertIsNone(body.head_camera)
        self.assertFalse(body.plan.use_eef)

    def test_wrong_model_and_revision_are_rejected(self):
        for key, value in (("model", "other_body"),
                           ("model_revision", "other-revision")):
            with self.subTest(key=key):
                changed = copy.deepcopy(self.manifest)
                changed[key] = value
                with self.assertRaises(verify_dream.ContractError):
                    verify_dream.check_manifest(verify_dream.PROFILE, changed)

    def test_reordered_same_width_names_are_rejected(self):
        logical = copy.deepcopy(self.manifest)
        logical["logical_coordinates"] = list(reversed(logical["logical_coordinates"]))
        with self.assertRaises(verify_dream.ContractError):
            verify_dream.check_manifest(verify_dream.PROFILE, logical)

        internal = copy.deepcopy(self.manifest)
        internal["internal_coordinates"] = [
            "yaw", "shoulder_b", "shoulder_a", "elbow_a", "elbow_b"
        ]
        with self.assertRaises(verify_dream.ContractError):
            verify_dream.check_manifest(verify_dream.PROFILE, internal)

        five = copy.deepcopy(self.manifest)
        five["internal_coordinates"] = [*five["internal_coordinates"], "passive_wheel"]
        with self.assertRaises(verify_dream.ContractError):
            verify_dream.check_manifest(verify_dream.PROFILE, five)

    def test_wrong_semantics_units_camera_and_timing_are_rejected(self):
        for key, value in (("action_semantics", "delta joint velocities"),
                           ("units", "meters, kilograms, degrees, seconds")):
            with self.subTest(key=key):
                changed = copy.deepcopy(self.manifest)
                changed[key] = value
                with self.assertRaises(verify_dream.ContractError):
                    verify_dream.check_manifest(verify_dream.PROFILE, changed)

        for name, width in (("wrist", 320), ("overhead", 640)):
            with self.subTest(name=name, width=width):
                changed = copy.deepcopy(self.manifest)
                changed["camera"]["name"] = name
                changed["camera"]["width"] = width
                with self.assertRaises(verify_dream.ContractError):
                    verify_dream.check_manifest(verify_dream.PROFILE, changed)

        for timing in (replace(verify_dream.PROFILE.timing, action_hz=100),
                       replace(verify_dream.PROFILE.timing, history=2),
                       replace(verify_dream.PROFILE.timing, chunk_rows=2)):
            with self.subTest(timing=timing):
                with self.assertRaises(verify_dream.ContractError):
                    verify_dream.check_profile(replace(verify_dream.PROFILE, timing=timing))

    def test_five_wide_or_extra_parts_fail_the_selected_plan(self):
        two_wide = verify_dream.embodiment()
        two_wide["state"] = {"arm": [0, 2]}
        two_wide["action"] = {"arm": [0, 2]}
        with self.assertRaises(verify_dream.ContractError):
            verify_dream.check_registration(
                verify_dream.PROFILE, verify_dream.registered_body(two_wide)
            )

        five_wide = verify_dream.embodiment()
        five_wide["state"] = {"arm": [0, 3], "passive": [3, 5]}
        five_wide["action"] = {"arm": [0, 3], "passive": [3, 5]}
        five_wide["parts"]["state"]["passive"] = {"kind": "other"}
        five_wide["parts"]["action"]["passive"] = {"kind": "other"}
        body = verify_dream.registered_body(five_wide)
        with self.assertRaises(verify_dream.ContractError):
            verify_dream.check_registration(verify_dream.PROFILE, body)

        left = verify_dream.embodiment()
        left["state"]["left_arm"] = [3, 6]
        left["action"]["left_arm"] = [3, 6]
        left["parts"]["state"]["left_arm"] = {"kind": "arm", "side": "left"}
        left["parts"]["action"]["left_arm"] = {"kind": "arm", "side": "left"}
        with self.assertRaises((verify_dream.ContractError, verify_dream.EmbodimentInvalidError)):
            verify_dream.check_registration(
                verify_dream.PROFILE, verify_dream.registered_body(left)
            )

        gripper = verify_dream.embodiment()
        gripper["state"]["gripper"] = [3, 4]
        gripper["action"]["gripper"] = [3, 4]
        gripper["parts"]["state"]["gripper"] = {
            "kind": "gripper", "side": "single", "closed": 0., "open": 1.
        }
        gripper["parts"]["action"]["gripper"] = {
            "kind": "gripper", "side": "single", "closed": 0., "open": 1.
        }
        with self.assertRaises((verify_dream.ContractError, verify_dream.EmbodimentInvalidError)):
            verify_dream.check_registration(
                verify_dream.PROFILE, verify_dream.registered_body(gripper)
            )

        head = verify_dream.embodiment()
        head["head_camera"] = "overhead"
        with self.assertRaises(verify_dream.ContractError):
            verify_dream.check_registration(
                verify_dream.PROFILE, verify_dream.registered_body(head)
            )

        eef = verify_dream.embodiment()
        eef["state"]["right_eef_xyz"] = [3, 6]
        eef["parts"]["state"]["right_eef_xyz"] = {
            "kind": "position", "side": "right", "point": "grasp", "frame": "arm_base"
        }
        with self.assertRaises((verify_dream.ContractError, verify_dream.EmbodimentInvalidError)):
            verify_dream.check_registration(
                verify_dream.PROFILE, verify_dream.registered_body(eef)
            )

    def test_observation_boundary_rejects_bad_state_frame_timestamp_and_text(self):
        state, encoded, payload = verify_dream.check_observation(
            verify_dream.PROFILE, self.manifest, observation()
        )
        self.assertEqual(state["state.arm"], [0.10, 0.20, -0.50])
        self.assertIsInstance(encoded, str)
        self.assertGreater(len(payload), 0)

        cases = {
            "two-wide state": observation(joints=(0., 0.)),
            "five-wide state": observation(joints=(0., 0., 0., 0., 0.)),
            "nonfinite state": observation(joints=(0., math.nan, 0.)),
            "numeric string state": observation(joints=("0.1", 0.2, -0.5)),
            "boolean state": observation(joints=(True, 0.2, -0.5)),
            "out-of-range state": observation(joints=(0., 0., math.pi)),
            "float frame": observation(rgb=frame().astype(float)),
            "constant frame": observation(rgb=np.zeros((240, 320, 3), dtype=np.uint8)),
            "wrong frame shape": observation(rgb=frame()[:120]),
            "nonfinite timestamp": observation(timestamp=math.nan),
            "negative timestamp": observation(timestamp=-1.),
            "wrong prompt": observation(prompt="reach elsewhere"),
        }
        for label, changed in cases.items():
            with self.subTest(label=label):
                with self.assertRaises(verify_dream.ContractError):
                    verify_dream.check_observation(
                        verify_dream.PROFILE, self.manifest, changed
                    )

    def test_action_boundary_requires_exact_keys_one_finite_bounded_row(self):
        good = {"per_key": {"action.arm": [[0.20, 0.35, -0.45]]}}
        self.assertEqual(
            verify_dream.check_action(verify_dream.PROFILE, self.manifest, good),
            [[0.20, 0.35, -0.45]],
        )

        cases = {
            "extra key": {"per_key": {
                "action.arm": [[0.20, 0.35, -0.45]],
                "action.left_arm": [[0.20, 0.35, -0.45]],
            }},
            "two wide": {"per_key": {"action.arm": [[0., 0.]]}},
            "empty": {"per_key": {"action.arm": []}},
            "multirow": {"per_key": {"action.arm": [
                [0.20, 0.35, -0.45], [0.20, 0.35, -0.45]
            ]}},
            "ragged": {"per_key": {"action.arm": [
                [0., 0., 0.], [0., 0.]
            ]}},
            "nan": {"per_key": {"action.arm": [[math.nan, 0., 0.]]}},
            "numeric string": {"per_key": {"action.arm": [["0.1", 0.2, -0.5]]}},
            "boolean": {"per_key": {"action.arm": [[True, 0.2, -0.5]]}},
            "infinity": {"per_key": {"action.arm": [[0., math.inf, 0.]]}},
            "out of range": {"per_key": {"action.arm": [[1.21, 0., 0.]]}},
            "missing key": {"per_key": {"action.right_arm": [[0., 0., 0.]]}},
        }
        for label, reply in cases.items():
            with self.subTest(label=label):
                with self.assertRaises(verify_dream.ContractError):
                    verify_dream.check_action(
                        verify_dream.PROFILE, self.manifest, reply
                    )

    def test_resolve_output_rejects_symlink_escape(self):
        with tempfile.TemporaryDirectory() as temporary:
            base = Path(temporary)
            results = base / "results"
            results.mkdir()
            outside = base / "outside.json"
            outside.write_text("{}")
            link = results / "escape.json"
            link.symlink_to(outside)
            original = verify_dream.RESULTS
            verify_dream.RESULTS = results
            self.addCleanup(setattr, verify_dream, "RESULTS", original)
            old_here = verify_dream.HERE
            verify_dream.HERE = base
            self.addCleanup(setattr, verify_dream, "HERE", old_here)
            with self.assertRaises(verify_dream.ContractError):
                verify_dream.resolve_output(link)
            self.assertEqual(
                verify_dream.resolve_output(results / "nested" / "report.json"),
                (results / "nested" / "report.json").resolve(),
            )

    def test_results_directory_symlink_is_rejected(self):
        with tempfile.TemporaryDirectory() as temporary:
            base = Path(temporary)
            outside = base / "outside"
            outside.mkdir()
            redirected = base / "results"
            redirected.symlink_to(outside, target_is_directory=True)
            with patch.object(verify_dream, "HERE", base), patch.object(verify_dream, "RESULTS", redirected):
                with self.assertRaises(verify_dream.ContractError):
                    verify_dream.resolve_output()

    def test_changed_limits_and_camera_extrinsics_are_rejected(self):
        for section, name, value in (("limits", "yaw_rad", 100.0),
                                      ("camera", "pos_m", [0, 0, 2]),
                                      ("camera", "fovy_deg", 120.0)):
            changed = copy.deepcopy(self.manifest)
            changed[section][name] = value
            with self.subTest(section=section, name=name):
                with self.assertRaises(verify_dream.ContractError):
                    verify_dream.check_manifest(verify_dream.PROFILE, changed)

    def test_positive_report_keeps_contract_scope(self):
        self.assertEqual(verify_dream.PROFILE.timing.action_hz, 30)
        self.assertNotEqual(verify_dream.PROFILE.timing.action_hz,
                            verify_dream.PROFILE.timing.control_hz)
        self.assertEqual(verify_dream.PROFILE.logical_names,
                         ("base_yaw", "shoulder_bend", "elbow_bend"))
        self.assertEqual(verify_dream.PROFILE.internal_names,
                         ("yaw", "shoulder_a", "shoulder_b", "elbow_a", "elbow_b"))


if __name__ == "__main__":
    unittest.main()
