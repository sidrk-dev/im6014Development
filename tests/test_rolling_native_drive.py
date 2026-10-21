from pathlib import Path
import sys
import unittest

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from examples.rolling_reach_arm import drive, scene  # noqa: E402
from im6014_control.core import JointCore  # noqa: E402
from im6014_control.mapping import JointMap, JointSample  # noqa: E402
from im6014_control.records import (  # noqa: E402
    MotorHealth,
    NativePositionServo,
    SourceIdentity,
    State,
    Target,
)
from im6014_control.reference import (  # noqa: E402
    _applied_reference,
    _predicted_effort,
)
from tests.control.test_controller import DeterministicClock  # noqa: E402


STOP = MotorHealth(0, 0, 0, False)


def loaded_model():
    return mujoco.MjModel.from_xml_string(scene.generated_xml())


def native_core(specs, *, positions=None, velocities=None, max_step=0.02):
    count = len(specs)
    positions = (0.0,) * count if positions is None else positions
    velocities = (0.0,) * count if velocities is None else velocities
    clock = DeterministicClock(0.0)
    core = JointCore(
        specs,
        clock=clock,
        feedback_max_age_s=10.0,
        max_reference_step_s=max_step,
    )
    assert core.begin_connect().accepted
    assert core.connected(SourceIdentity("fixture", "rolling-native", 1)).accepted
    assert core.begin_check().accepted
    for spec, q_rad, dq_rad_s in zip(specs, positions, velocities):
        assert core.receive(
            spec.motor_id, JointSample(q_rad, dq_rad_s, 0.0), STOP
        ).accepted
    assert core.checked().accepted
    grant = core.arm(owner="native")
    assert grant.accepted
    return core, clock, grant


def target(core, joints, sequence=0, expiry=10.0):
    return Target(sequence, tuple(joints), 0.0, expiry, core.snapshot().ownership)


class NativeDriveFactoryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = loaded_model()

    def test_factory_derives_three_identity_native_specs(self):
        specs = drive.joint_specs_from_model(
            self.model, max_measured_lead_rad=0.05
        )
        self.assertEqual(len(specs), 3)
        self.assertEqual(
            tuple(spec.name for spec in specs), scene.SCENARIO.logical_names
        )
        self.assertEqual(tuple(spec.motor_id for spec in specs), (0, 1, 2))
        for spec in specs:
            self.assertIsInstance(spec.drive, NativePositionServo)
            self.assertEqual(spec.mapping, JointMap(1.0, 1, 0.0))
            self.assertEqual(spec.max_measured_lead_rad, 0.05)
            self.assertEqual(spec.max_velocity_rad_s, scene.SCENARIO.speed_limit)
            self.assertEqual(
                spec.max_acceleration_rad_s2, scene.SCENARIO.accel_limit
            )

        self.assertEqual(
            [spec.drive.kp for spec in specs], [180.0, 420.0, 420.0]
        )
        self.assertEqual(
            [spec.drive.kd for spec in specs], [18.0, 38.0, 38.0]
        )
        self.assertEqual(
            [(spec.position_limits.minimum_rad, spec.position_limits.maximum_rad)
             for spec in specs],
            [(-1.2, 1.2), (-1.2, 1.2), (-1.4, 1.4)],
        )
        self.assertEqual(
            [(spec.drive.force_min_nm, spec.drive.force_max_nm)
             for spec in specs],
            [(-14.0, 14.0), (-25.0, 25.0), (-25.0, 25.0)],
        )
        self.assertEqual(
            [spec.max_effort_nm for spec in specs], [14.0, 25.0, 25.0]
        )

    def test_factory_rejects_mutated_drive_metadata(self):
        def mutate_gain(model):
            model.actuator_gainprm[0, 0] += 1.0

        def mutate_damping(model):
            model.actuator_biasprm[1, 2] += 1.0

        def mutate_gear(model):
            model.actuator_gear[1, 0] = 2.0

        def mutate_ctrl_range(model):
            model.actuator_ctrlrange[0, 1] += 0.1

        def mutate_force_range(model):
            model.actuator_forcerange[2, 1] += 1.0

        def disable_ctrl_limit(model):
            model.actuator_ctrllimited[0] = False

        def disable_force_limit(model):
            model.actuator_forcelimited[1] = False

        def mutate_delay(model):
            model.actuator_delay[0] = 0.01

        def mutate_ctrl_spec(model):
            model.actuator_ctrlspec[1] = 0

        def mutate_tendon_wrap(model):
            model.wrap_prm[0] += 0.1

        def drift_order(model):
            first = float(model.actuator_gainprm[0, 0])
            second = float(model.actuator_gainprm[1, 0])
            model.actuator_gainprm[0, 0] = second
            model.actuator_gainprm[1, 0] = first
            model.actuator_trnid[0, 0] = 1

        cases = (
            ("gain", mutate_gain),
            ("damping", mutate_damping),
            ("gear", mutate_gear),
            ("ctrl_range", mutate_ctrl_range),
            ("force_range", mutate_force_range),
            ("ctrl_limiting", disable_ctrl_limit),
            ("force_limiting", disable_force_limit),
            ("delay", mutate_delay),
            ("ctrl_spec", mutate_ctrl_spec),
            ("tendon_wrap", mutate_tendon_wrap),
            ("order", drift_order),
        )
        for label, mutate in cases:
            with self.subTest(label=label):
                model = loaded_model()
                mutate(model)
                with self.assertRaises(ValueError):
                    drive.joint_specs_from_model(
                        model, max_measured_lead_rad=0.05
                    )

    def test_wrong_physical_high_gains_are_still_rejected(self):
        model = loaded_model()
        model.actuator_gainprm[:, 0] = [1000.0, 2000.0, 2000.0]
        with self.assertRaises(ValueError):
            drive.joint_specs_from_model(model, max_measured_lead_rad=0.05)

    def test_factory_rejects_loaded_name_drift(self):
        xml = scene.generated_xml().replace(
            'name="yaw_target"', 'name="yaw_target_drifted"'
        )
        model = mujoco.MjModel.from_xml_string(xml)
        with self.assertRaises(ValueError):
            drive.joint_specs_from_model(model, max_measured_lead_rad=0.05)

    def test_stop_mode_is_allowed_at_the_factory_boundary(self):
        model = loaded_model()
        bit = int(mujoco.mjtDisableBit.mjDSBL_ACTUATION)
        model.opt.disableflags |= bit
        specs = drive.joint_specs_from_model(
            model, max_measured_lead_rad=0.05
        )
        self.assertEqual(len(specs), 3)

    def test_factory_rejects_invalid_measured_lead(self):
        for bad in (True, False, 0, -1.0, float("nan"), float("inf"), 10**1000):
            with self.subTest(bad=repr(bad)), self.assertRaises(ValueError):
                drive.joint_specs_from_model(
                    self.model, max_measured_lead_rad=bad
                )


class LoadedNativePredictionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = loaded_model()

    def test_issued_references_match_loaded_unsaturated_pd(self):
        specs = drive.joint_specs_from_model(
            self.model, max_measured_lead_rad=0.05
        )
        core, clock, _grant = native_core(specs, max_step=0.02)
        goal = tuple(spec.drive.target(0.01) for spec in specs)
        pending = target(core, goal)
        self.assertTrue(core.submit_target(pending).accepted)
        clock.value = 0.02
        reference = core.consume_target()
        self.assertIsNotNone(reference)
        self.assertTrue(all(joint.q_rad > 0.0 for joint in reference.joints))

        measured = np.zeros(3)
        velocity = np.zeros(3)
        data = mujoco.MjData(self.model)
        data.qpos[:] = scene.logical_to_internal(measured)
        data.qvel[:] = scene.mapping() @ velocity
        data.ctrl[:] = [joint.q_rad for joint in reference.joints]
        mujoco.mj_forward(self.model, data)

        kp = self.model.actuator_gainprm[:, 0]
        kd = -self.model.actuator_biasprm[:, 2]
        raw = kp * (
            np.asarray([joint.q_rad for joint in reference.joints]) - measured
        ) - kd * velocity
        limits = self.model.actuator_forcerange
        self.assertTrue(np.all(raw >= limits[:, 0]))
        self.assertTrue(np.all(raw <= limits[:, 1]))
        np.testing.assert_allclose(data.actuator_force, raw, atol=1e-12, rtol=0.0)
        np.testing.assert_allclose(data.actuator_length, measured, atol=1e-12, rtol=0.0)
        np.testing.assert_allclose(data.actuator_velocity, velocity, atol=1e-12, rtol=0.0)

        for index, (spec, joint) in enumerate(zip(specs, reference.joints)):
            applied = _applied_reference(spec, joint)
            self.assertEqual((applied.kp, applied.kd), (spec.drive.kp, spec.drive.kd))
            self.assertAlmostEqual(
                _predicted_effort(applied, measured[index], velocity[index]),
                float(raw[index]),
                places=11,
            )

    def test_issued_references_match_loaded_pd_under_measured_motion(self):
        specs = drive.joint_specs_from_model(
            self.model, max_measured_lead_rad=0.05
        )
        measured = np.array([0.02, -0.01, 0.015])
        velocity = np.array([0.01, -0.02, 0.008])
        core, clock, _grant = native_core(
            specs,
            positions=tuple(measured),
            velocities=tuple(velocity),
            max_step=0.05,
        )
        goal = tuple(
            spec.drive.target(float(q_rad + 0.002))
            for spec, q_rad in zip(specs, measured)
        )
        pending = target(core, goal)
        self.assertTrue(core.submit_target(pending).accepted)
        clock.value = 0.05
        reference = core.consume_target()
        self.assertIsNotNone(reference)
        self.assertTrue(
            all(
                joint.q_rad > q_rad
                for joint, q_rad in zip(reference.joints, measured)
            )
        )

        data = mujoco.MjData(self.model)
        data.qpos[:] = scene.logical_to_internal(measured)
        data.qvel[:] = scene.mapping() @ velocity
        data.ctrl[:] = [joint.q_rad for joint in reference.joints]
        mujoco.mj_forward(self.model, data)

        commanded = np.asarray([joint.q_rad for joint in reference.joints])
        kp = self.model.actuator_gainprm[:, 0]
        kd = -self.model.actuator_biasprm[:, 2]
        raw = kp * (commanded - measured) - kd * velocity
        limits = self.model.actuator_forcerange
        self.assertTrue(np.all(raw >= limits[:, 0]))
        self.assertTrue(np.all(raw <= limits[:, 1]))
        np.testing.assert_allclose(data.actuator_force, raw, atol=1e-12, rtol=0.0)
        np.testing.assert_allclose(data.actuator_length, measured, atol=1e-12, rtol=0.0)
        np.testing.assert_allclose(data.actuator_velocity, velocity, atol=1e-12, rtol=0.0)

        for index, (spec, joint) in enumerate(zip(specs, reference.joints)):
            applied = _applied_reference(spec, joint)
            self.assertAlmostEqual(
                _predicted_effort(applied, measured[index], velocity[index]),
                float(raw[index]),
                places=11,
            )

    def test_unsafe_constraint_veto_is_whole_vector_and_atomic(self):
        specs = drive.joint_specs_from_model(
            self.model, max_measured_lead_rad=0.5
        )
        core, clock, _grant = native_core(specs, max_step=0.2)
        goal = tuple(spec.drive.target(0.1) for spec in specs)
        pending = target(core, goal)
        self.assertTrue(core.submit_target(pending).accepted)
        history = core._history
        clock.value = 0.2
        self.assertIsNone(core.consume_target())
        self.assertIs(core._history, history)
        self.assertIs(core.snapshot().pending_target, pending)

        safe = target(
            core,
            tuple(spec.drive.target(0.01) for spec in specs),
            sequence=1,
        )
        self.assertTrue(core.submit_target(safe).accepted)
        clock.value = 0.4
        reference = core.consume_target()
        self.assertIsNotNone(reference)
        self.assertEqual(len(reference.joints), 3)


class LoadedNativeClosedLoopTests(unittest.TestCase):
    def test_ten_second_multi_axis_progress_reaches_exact_reference_goal(self):
        model = loaded_model()
        data = mujoco.MjData(model)
        stop = int(mujoco.mjtDisableBit.mjDSBL_ACTUATION)
        model.opt.disableflags |= stop
        data.qpos[:] = scene.logical_to_internal(scene.SCENARIO.home)
        data.ctrl[:] = scene.SCENARIO.home
        mujoco.mj_forward(model, data)
        specs = drive.joint_specs_from_model(
            model, max_measured_lead_rad=0.1
        )
        step = 0

        def now():
            return step * float(model.opt.timestep)

        core = JointCore(
            specs,
            clock=now,
            feedback_max_age_s=0.05,
            max_reference_step_s=0.01,
        )
        self.assertTrue(core.begin_connect().accepted)
        self.assertTrue(
            core.connected(
                SourceIdentity("demo", "native-progress", 1)
            ).accepted
        )
        self.assertTrue(core.begin_check().accepted)

        def feedback():
            mujoco.mj_forward(model, data)
            q = scene.observer() @ data.qpos
            dq = scene.observer() @ data.qvel
            tau = scene.mapping().T @ data.qfrc_actuator
            health = MotorHealth(
                0 if model.opt.disableflags & stop else 1, 0, 0, False
            )
            for spec, qi, vi, ti in zip(specs, q, dq, tau):
                self.assertTrue(
                    core.receive(
                        spec.motor_id,
                        JointSample(float(qi), float(vi), float(ti)),
                        health,
                    ).accepted
                )
            return q

        feedback()
        self.assertTrue(core.checked().accepted)
        arm = core.arm(owner="native-progress")
        self.assertTrue(arm.accepted, arm)
        initial = np.array(scene.SCENARIO.home)
        goal = initial + np.array([0.05, 0.04, 0.03])
        issued = []
        last_reference = initial.copy()

        for step in range(30000):
            if step % scene.SCENARIO.control_decimation == 0:
                measured = feedback()
                measured_velocity = scene.observer() @ data.qvel
                self.assertIsNot(core.snapshot().state, State.FAULT)
                pending = Target(
                    step // scene.SCENARIO.control_decimation,
                    tuple(
                        spec.drive.target(float(qi))
                        for spec, qi in zip(specs, goal)
                    ),
                    now(),
                    now() + 0.05,
                    arm.ownership,
                )
                self.assertTrue(core.submit_target(pending).accepted)
                reference = core.consume_target()
                if reference is not None:
                    last_reference = np.array(
                        [joint.q_rad for joint in reference.joints]
                    )
                    issued.append((
                        now(),
                        last_reference.copy(),
                        measured.copy(),
                        measured_velocity.copy(),
                        np.array([
                            joint.dq_rad_s for joint in reference.joints
                        ]),
                    ))
                    data.ctrl[:] = last_reference
                    model.opt.disableflags &= ~stop
            mujoco.mj_step(model, data)

        self.assertGreater(len(issued), 900)
        np.testing.assert_array_equal(last_reference, goal)

        previous_stamp = 0.0
        previous_reference = initial.copy()
        previous_rate = np.zeros(3)
        for stamp, reference, measured, velocity, desired_rate in issued:
            for spec, q, dq in zip(specs, reference, desired_rate):
                self.assertGreaterEqual(q, spec.position_limits.minimum_rad)
                self.assertLessEqual(q, spec.position_limits.maximum_rad)
                self.assertLessEqual(abs(dq), spec.max_velocity_rad_s)
            for index, (spec, q) in enumerate(zip(specs, reference)):
                applied = _applied_reference(
                    spec, spec.drive.target(float(q))
                )
                self.assertLessEqual(
                    abs(_predicted_effort(
                        applied, float(measured[index]),
                        float(velocity[index]),
                    )),
                    spec.max_effort_nm,
                )
            elapsed = stamp - previous_stamp
            if elapsed == 0.0:
                np.testing.assert_array_equal(reference, previous_reference)
            else:
                rate = (reference - previous_reference) / elapsed
                for spec, value, prior in zip(specs, rate, previous_rate):
                    self.assertLessEqual(abs(value), spec.max_velocity_rad_s)
                    self.assertLessEqual(
                        abs(value - prior),
                        spec.max_acceleration_rad_s2 * elapsed,
                    )
                previous_rate = rate
            previous_stamp = stamp
            previous_reference = reference


if __name__ == "__main__":
    unittest.main()
