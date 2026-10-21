from pathlib import Path
import sys
import unittest

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from examples.rolling_reach_arm import scene  # noqa: E402


def seeded_states(count=200):
    rng = np.random.default_rng(421)
    q = rng.uniform(-.3, .3, (count, 3))
    dq = rng.uniform(-.4, .4, (count, 3))
    ctrl = q + rng.uniform(-.2, .2, (count, 3))
    return q, dq, ctrl


class RollingDriveSemanticsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = mujoco.MjModel.from_xml_string(scene.generated_xml())

    def setUp(self):
        self.data = mujoco.MjData(self.model)

    def drive(self, q, dq, ctrl):
        self.data.qpos[:] = scene.logical_to_internal(q)
        self.data.qvel[:] = scene.mapping() @ dq
        self.data.ctrl[:] = ctrl
        mujoco.mj_forward(self.model, self.data)
        return self.data

    def expected_force(self, q, dq, ctrl):
        kp = self.model.actuator_gainprm[:, 0]
        kv = -self.model.actuator_biasprm[:, 2]
        raw = kp * (ctrl - q) - kv * dq
        limits = self.model.actuator_forcerange
        return raw, np.clip(raw, limits[:, 0], limits[:, 1])

    def test_loaded_gains_damping_and_force_caps(self):
        np.testing.assert_array_equal(self.model.actuator_gainprm[:, 0],
                                      [180., 420., 420.])
        np.testing.assert_array_equal(self.model.actuator_biasprm[:, 0],
                                      [0., 0., 0.])
        np.testing.assert_array_equal(self.model.actuator_biasprm[:, 1],
                                      [-180., -420., -420.])
        np.testing.assert_array_equal(self.model.actuator_biasprm[:, 2],
                                      [-18., -38., -38.])
        np.testing.assert_array_equal(self.model.actuator_forcerange,
                                      [[-14., 14.], [-25., 25.], [-25., 25.]])

    def test_seeded_states_match_position_servo_and_rolling_map(self):
        q, dq, ctrl = seeded_states()
        self.assertTrue((q < 0).any(axis=0).all() and (q > 0).any(axis=0).all())
        self.assertTrue((dq < 0).any(axis=0).all() and (dq > 0).any(axis=0).all())
        self.assertTrue(np.all(dq != 0.))
        peak_force = peak_length = peak_velocity = peak_map = 0.
        saturated = unsaturated = 0
        for desired, velocity, command in zip(q, dq, ctrl):
            data = self.drive(desired, velocity, command)
            raw, expected = self.expected_force(desired, velocity, command)
            peak_force = max(peak_force, float(np.max(np.abs(
                data.actuator_force - expected))))
            peak_map = max(peak_map, float(np.max(np.abs(
                scene.mapping().T @ data.qfrc_actuator - data.actuator_force))))
            peak_length = max(peak_length, float(np.max(np.abs(
                data.actuator_length - desired))))
            peak_velocity = max(peak_velocity, float(np.max(np.abs(
                data.actuator_velocity - velocity))))
            clipped = bool(np.any(np.abs(raw) > self.model.actuator_forcerange[:, 1]))
            saturated += int(clipped)
            unsaturated += int(not clipped)
        self.assertEqual((saturated, unsaturated), (188, 12))
        self.assertLess(peak_force, 1e-12)
        self.assertLess(peak_map, 1e-12)
        self.assertLess(peak_length, 1e-12)
        self.assertLess(peak_velocity, 1e-12)

    def test_nonzero_desired_velocity_formula_is_not_the_loaded_drive(self):
        q = np.array([.1, -.2, .15])
        dq = np.array([.05, -.1, .2])
        ctrl = q + np.array([.01, -.02, .03])
        data = self.drive(q, dq, ctrl)
        kp = self.model.actuator_gainprm[:, 0]
        kv = -self.model.actuator_biasprm[:, 2]
        with_feedthrough = kp * (ctrl - q) - kv * (dq - np.array([.3, -.25, .2]))
        self.assertFalse(np.allclose(data.actuator_force, with_feedthrough,
                                     atol=1e-9, rtol=0.))
        self.assertGreater(float(np.max(np.abs(
            data.actuator_force - with_feedthrough))), 1.)

    def test_stop_zeroes_force_and_explicit_enable_restores_it(self):
        q = np.array([.1, -.2, .15])
        dq = np.array([.05, -.1, .2])
        ctrl = q + np.array([.01, -.02, .03])
        data = self.drive(q, dq, ctrl)
        before = data.actuator_force.copy()
        _, expected = self.expected_force(q, dq, ctrl)
        np.testing.assert_allclose(before, expected, atol=1e-12)
        self.assertGreater(float(np.max(np.abs(before))), 0.)
        bit = int(mujoco.mjtDisableBit.mjDSBL_ACTUATION)
        self.model.opt.disableflags |= bit
        try:
            mujoco.mj_forward(self.model, data)
            np.testing.assert_array_equal(data.actuator_force, np.zeros(3))
            np.testing.assert_array_equal(data.qfrc_actuator, np.zeros(5))
        finally:
            self.model.opt.disableflags &= ~bit
        mujoco.mj_forward(self.model, data)
        np.testing.assert_array_equal(data.actuator_force, before)


if __name__ == "__main__":
    unittest.main()
