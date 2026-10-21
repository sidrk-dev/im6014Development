"""Regression tests for the frozen single-arm contract checker."""

import copy
import importlib.util
import json
import math
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
CHECKER_PATH = ROOT / "scripts" / "check_single_arm_contract.py"
CONTRACT_PATH = ROOT / "docs" / "single-arm-contract.json"

SPEC = importlib.util.spec_from_file_location("check_single_arm_contract", CHECKER_PATH)
checker = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(checker)


class SingleArmContractCheckerTests(unittest.TestCase):
    def setUp(self):
        self.data = json.loads(CONTRACT_PATH.read_text())

    def assert_rejected(self):
        with self.assertRaises(SystemExit):
            checker.check_contract(self.data)

    def test_canonical_contract_passes(self):
        checks = checker.check_contract(self.data)
        self.assertIn("joint order, direction, limits and units", checks)
        self.assertIn("exclusions and evidence limits", checks)

    def test_missing_prototype_asset_is_rejected(self):
        del self.data["profiles"]["prototype"]["asset"]
        self.assert_rejected()

    def test_empty_prototype_asset_is_rejected(self):
        self.data["profiles"]["prototype"]["asset"] = {}
        self.assert_rejected()

    def test_missing_prototype_asset_entry_is_rejected(self):
        del self.data["profiles"]["prototype"]["asset"]["source_archive"]
        self.assert_rejected()

    def test_unexpected_prototype_asset_key_is_rejected(self):
        self.data["profiles"]["prototype"]["asset"]["unexpected"] = {}
        self.assert_rejected()

    def test_missing_exclusions_are_rejected(self):
        del self.data["exclusions"]
        self.assert_rejected()

    def test_empty_exclusions_are_rejected(self):
        self.data["exclusions"] = {}
        self.assert_rejected()

    def test_missing_exclusion_key_is_rejected(self):
        del self.data["exclusions"]["eef_actions"]
        self.assert_rejected()

    def test_unexpected_exclusion_key_is_rejected(self):
        self.data["exclusions"]["unexpected"] = False
        self.assert_rejected()

    def test_altered_joint_positive_direction_is_rejected_without_rendering_change(self):
        rendered = checker.render_doc(copy.deepcopy(self.data))
        self.data["joints"][0]["positive_direction"] = "left-hand rotation about the URDF joint axis"
        self.assertEqual(checker.render_doc(copy.deepcopy(self.data)), rendered)
        self.assert_rejected()

    def test_altered_joint_zero_is_rejected(self):
        self.data["joints"][1]["zero"] = "commissioned physical zero"
        self.assert_rejected()

    def test_reversed_j1_limits_are_rejected(self):
        self.data["joints"][0]["limits_rad"] = [math.pi / 2, -math.pi / 2]
        self.assert_rejected()

    def test_changed_j1_limit_magnitude_is_rejected(self):
        self.data["joints"][0]["limits_rad"] = [-1.0, 1.0]
        self.assert_rejected()

    def test_nonfinite_j1_limits_are_rejected(self):
        cases = [
            [float("-inf"), math.pi / 2],
            [math.nan, math.pi / 2],
            [-math.pi / 2, float("inf")],
        ]
        for limits in cases:
            with self.subTest(limits=limits):
                self.data["joints"][0]["limits_rad"] = list(limits)
                self.assert_rejected()

    def test_j2_and_j3_limits_must_stay_null(self):
        for index in (1, 2):
            with self.subTest(joint=self.data["joints"][index]["name"]):
                self.data = json.loads(CONTRACT_PATH.read_text())
                self.data["joints"][index]["limits_rad"] = [-math.pi / 2, math.pi / 2]
                self.assert_rejected()

    def test_removed_exclusion_is_rejected_without_rendering_change(self):
        rendered = checker.render_doc(copy.deepcopy(self.data))
        del self.data["exclusions"]["eef_actions"]
        self.assertEqual(checker.render_doc(copy.deepcopy(self.data)), rendered)
        self.assert_rejected()


if __name__ == "__main__":
    unittest.main()
