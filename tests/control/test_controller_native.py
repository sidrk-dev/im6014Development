import math
import unittest
from unittest.mock import patch

from im6014_control import core as core_module
from im6014_control.core import JointCore
from im6014_control.mapping import JointMap, JointSample, JointTarget
from im6014_control.records import (
    DecisionCode,
    JointSpec,
    MotorHealth,
    NativePositionServo,
    PhysicalIM6014,
    PositionLimits,
    SourceIdentity,
    State,
    Target,
)
from im6014_control.reference import (
    _applied_reference,
    _predicted_effort,
    _seed_reference,
    plan_reference,
    project_joint_target,
)
from tests.control.test_controller import DeterministicClock


STOP = MotorHealth(0, 0, 0, False)


def servo(kp=10.0, kd=1.0, force_min=-5.0, force_max=5.0):
    return NativePositionServo(kp, kd, force_min, force_max)


def native_spec(
    name="J1",
    motor_id=0,
    *,
    kp=10.0,
    kd=1.0,
    cap=None,
    force_min=-5.0,
    force_max=5.0,
    limits=None,
    velocity=1.0,
    acceleration=10.0,
    lead=1.0,
):
    drive = servo(kp, kd, force_min, force_max)
    if cap is None:
        cap = min(-force_min, force_max)
    return JointSpec(
        name,
        motor_id,
        JointMap(1.0, 1, 0.0),
        PositionLimits(-2.0, 2.0) if limits is None else limits,
        velocity,
        acceleration,
        lead,
        cap,
        drive,
    )


def physical_spec(name="P1", motor_id=0, *, cap=100.0):
    return JointSpec(
        name,
        motor_id,
        JointMap(1.0, 1, 0.0),
        PositionLimits(-2.0, 2.0),
        1.0,
        10.0,
        1.0,
        cap,
    )


def armed(
    specs,
    *,
    positions=None,
    velocities=None,
    feedback_age=10.0,
    max_step=0.1,
):
    specs = tuple(specs)
    count = len(specs)
    positions = (0.0,) * count if positions is None else positions
    velocities = (0.0,) * count if velocities is None else velocities
    clock = DeterministicClock(0.0)
    core = JointCore(
        specs,
        clock=clock,
        feedback_max_age_s=feedback_age,
        max_reference_step_s=max_step,
    )
    assert core.begin_connect().accepted
    assert core.connected(SourceIdentity("fixture", "native", 1)).accepted
    assert core.begin_check().accepted
    for spec, q_rad, dq_rad_s in zip(specs, positions, velocities):
        assert core.receive(spec.motor_id, JointSample(q_rad, dq_rad_s, 0.0), STOP).accepted
    assert core.checked().accepted
    return core, clock


def target(core, joints, sequence=0, expiry=10.0):
    return Target(sequence, tuple(joints), 0.0, expiry, core.snapshot().ownership)


class NativeRecordTests(unittest.TestCase):
    def test_servo_rejects_invalid_configuration(self):
        for field, bad in (
            ("kp", -1.0),
            ("kp", True),
            ("kp", math.nan),
            ("kp", math.inf),
            ("kp", 10**1000),
            ("kd", -0.1),
            ("kd", False),
            ("force_min_nm", 0.0),
            ("force_min_nm", 1.0),
            ("force_min_nm", math.nan),
            ("force_max_nm", 0.0),
            ("force_max_nm", -1.0),
            ("force_max_nm", 10**1000),
        ):
            kwargs = dict(kp=10.0, kd=1.0, force_min_nm=-5.0, force_max_nm=5.0)
            kwargs[field] = bad
            with self.subTest(field=field, bad=repr(bad)), self.assertRaises(ValueError):
                NativePositionServo(**kwargs)

        valid = NativePositionServo(10, 1, -5, 5)
        self.assertEqual(
            (valid.kp, valid.kd, valid.force_min_nm, valid.force_max_nm),
            (10.0, 1.0, -5.0, 5.0),
        )
        self.assertEqual(
            valid.target(0.25), JointTarget(0.25, 0.0, 0.0, 10.0, 1.0)
        )

    def test_native_spec_requires_identity_map_and_bounded_effort(self):
        drive = servo()
        for mapping in (
            JointMap(2.0, 1, 0.0),
            JointMap(1.0, -1, 0.0),
            JointMap(1.0, 1, 0.001),
        ):
            with self.subTest(mapping=mapping), self.assertRaises(ValueError):
                JointSpec("J", 0, mapping, PositionLimits(-1, 1), 1, 1, 0.1, 5.0, drive)

        for cap in (5.0001, 6.0):
            with self.subTest(cap=cap), self.assertRaises(ValueError):
                JointSpec(
                    "J", 0, JointMap(1, 1, 0), PositionLimits(-1, 1),
                    1, 1, 0.1, cap, drive,
                )

        self.assertIsInstance(
            JointSpec(
                "J", 0, JointMap(1, 1, 0), PositionLimits(-1, 1),
                1, 1, 0.1, 5.0, drive,
            ).drive,
            NativePositionServo,
        )
        self.assertIsInstance(physical_spec().drive, PhysicalIM6014)
        with self.assertRaises(ValueError):
            JointSpec(
                "J", 0, JointMap(1, 1, 0), PositionLimits(-1, 1),
                1, 1, 0.1, 5.0, object(),
            )

    def test_native_drive_rejects_subclasses(self):
        class DerivedServo(NativePositionServo):
            pass

        with self.assertRaises(ValueError):
            JointSpec(
                "J", 0, JointMap(1, 1, 0), PositionLimits(-1, 1),
                1, 1, 0.1, 5.0, DerivedServo(10, 1, -5, 5),
            )

    def test_default_physical_drive_keeps_strict_codec_caps(self):
        spec = physical_spec(cap=1000.0)
        self.assertIsInstance(spec.drive, PhysicalIM6014)
        with self.assertRaises(ValueError):
            _applied_reference(spec, JointTarget(0, 0, 0, 30.0001, 0))
        native = native_spec(kp=180, kd=18, force_min=-14, force_max=14, cap=14)
        applied = _applied_reference(native, native.drive.target(0.1))
        self.assertEqual((applied.kp, applied.kd), (180.0, 18.0))

    def test_seed_is_representation_specific(self):
        native = native_spec(kp=12, kd=3, force_min=-9, force_max=9, cap=9)
        physical = physical_spec()
        sample = JointSample(0.3, 0.2, 0.0)
        self.assertEqual(
            _seed_reference(native, sample), JointTarget(0.3, 0.0, 0.0, 12.0, 3.0)
        )
        self.assertEqual(
            _seed_reference(physical, sample), JointTarget(0.3, 0.0, 0.0, 0.0, 0.0)
        )


class NativeArmTests(unittest.TestCase):
    def test_arm_stores_actual_native_seed_history_without_issuing(self):
        spec = native_spec(kp=10, kd=2, cap=5)
        core, _clock = armed((spec,), positions=(0.25,), velocities=(1.0,))
        grant = core.arm(owner="native")
        self.assertTrue(grant.accepted)
        self.assertEqual(core._history.kind, "seed")
        self.assertEqual(core._history.joints[0], JointTarget(0.25, 0.0, 0.0, 10.0, 2.0))
        self.assertEqual(core._history.position_rates_rad_s, (0.0,))
        self.assertIsNone(core.snapshot().pending_target)
        self.assertIs(core.snapshot().state, State.ARMED)

    def test_arm_accepts_seed_at_safe_effort_boundary(self):
        spec = native_spec(kp=10, kd=2, cap=5)
        core, _clock = armed((spec,), velocities=(2.5,))
        self.assertTrue(core.arm(owner="native").accepted)

    def test_arm_rejects_unsafe_native_seed_damping(self):
        spec = native_spec(kp=10, kd=2, cap=5)
        core, _clock = armed((spec,), velocities=(2.5000001,))
        decision = core.arm(owner="native")
        self.assertEqual(decision.code, DecisionCode.EFFORT_LIMIT)
        self.assertIs(core.snapshot().state, State.READY)
        self.assertIsNone(core.snapshot().ownership)
        self.assertIsNone(core._history)

    def test_physical_seed_is_unaffected_by_measured_velocity(self):
        core, _clock = armed((physical_spec(cap=0.1),), velocities=(100.0,))
        self.assertTrue(core.arm(owner="physical").accepted)
        self.assertEqual(
            core._history.joints[0], JointTarget(0.0, 0.0, 0.0, 0.0, 0.0)
        )


class NativeAdmissionTests(unittest.TestCase):
    def test_exact_native_intent_is_admitted(self):
        spec = native_spec()
        core, _clock = armed((spec,))
        core.arm(owner="native")
        pending = target(core, (spec.drive.target(0.1),))
        self.assertTrue(core.submit_target(pending).accepted)
        self.assertIs(core.snapshot().pending_target, pending)
        self.assertEqual(core.snapshot().last_sequence, 0)

    def test_semantic_mismatch_rejects_before_pending_or_sequence_mutation(self):
        spec = native_spec()
        core, _clock = armed((spec,))
        core.arm(owner="native")
        prior = target(core, (spec.drive.target(0.1),), sequence=3)
        self.assertTrue(core.submit_target(prior).accepted)
        drive = spec.drive
        mismatches = (
            JointTarget(0.1, 0.0, 0.0, drive.kp + 1.0, drive.kd),
            JointTarget(0.1, 0.0, 0.0, drive.kp, drive.kd + 1.0),
            JointTarget(0.1, 0.1, 0.0, drive.kp, drive.kd),
            JointTarget(0.1, 0.0, 0.1, drive.kp, drive.kd),
        )
        for index, bad in enumerate(mismatches):
            with self.subTest(index=index):
                decision = core.submit_target(
                    target(core, (bad,), sequence=4)
                )
                self.assertEqual(decision.code, DecisionCode.INVALID_ARGUMENT)
                self.assertIs(core.snapshot().pending_target, prior)
                self.assertEqual(core.snapshot().last_sequence, 3)


class NativeProjectionTests(unittest.TestCase):
    def test_projection_clamps_closed_bounds_and_requires_zero_speed(self):
        spec = native_spec(limits=PositionLimits(-0.5, 0.5))
        drive = spec.drive
        for wanted, expected in (
            (-1.0, math.nextafter(-0.5, 0.5)),
            (1.0, math.nextafter(0.5, -0.5)),
            (-0.5, -0.5),
            (0.5, 0.5),
            (0.1, 0.1),
        ):
            with self.subTest(wanted=wanted):
                projected = project_joint_target(
                    spec, drive.target(wanted), -0.5, 0.5, -1.0, 1.0
                )
                self.assertEqual(
                    projected, JointTarget(expected, 0.0, 0.0, drive.kp, drive.kd)
                )
        for dq_min, dq_max in ((0.1, 1.0), (-1.0, -0.1)):
            with self.subTest(dq_min=dq_min), self.assertRaises(ValueError):
                project_joint_target(spec, drive.target(0.0), -0.5, 0.5, dq_min, dq_max)
        self.assertEqual(
            project_joint_target(spec, drive.target(0.0), -0.5, 0.5, 0.0, 0.0),
            JointTarget(0.0, 0.0, 0.0, drive.kp, drive.kd),
        )
        for bad in (
            JointTarget(0.0, 0.0, 0.0, drive.kp + 1.0, drive.kd),
            JointTarget(0.0, 0.0, 0.0, drive.kp, drive.kd + 1.0),
            JointTarget(0.0, 0.1, 0.0, drive.kp, drive.kd),
            JointTarget(0.0, 0.0, 0.1, drive.kp, drive.kd),
        ):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                project_joint_target(spec, bad, -0.5, 0.5, -1.0, 1.0)

    def test_rounded_pacing_endpoint_allows_legal_progress(self):
        spec = native_spec(
            kp=180, kd=18, force_min=-14, force_max=14, cap=14,
            limits=PositionLimits(-1.2, 1.2),
            velocity=1.0, acceleration=2.0, lead=0.1,
        )
        previous = spec.drive.target(0.022399999999955192)
        result = plan_reference(
            (spec,),
            (spec.drive.target(0.05),),
            (previous,),
            (0.0,),
            (previous.q_rad,),
            1.0,
            0.01,
            False,
            measured_velocities=(0.0,),
        )
        self.assertIsNotNone(result)
        self.assertGreater(result[0].q_rad, previous.q_rad)
        self.assertLessEqual(result[0].q_rad - previous.q_rad, 0.01)

    def test_rounded_pacing_endpoint_allows_legal_progress_in_both_directions(self):
        spec = native_spec(
            kp=180, kd=18, force_min=-14, force_max=14, cap=14,
            limits=PositionLimits(-1.2, 1.2),
            velocity=1.0, acceleration=2.0, lead=0.1,
        )
        for start, goal in (
            (0.022399999999955192, 0.05),
            (-0.022399999999955192, -0.05),
        ):
            with self.subTest(start=start):
                previous = spec.drive.target(start)
                result = plan_reference(
                    (spec,),
                    (spec.drive.target(goal),),
                    (previous,),
                    (0.0,),
                    (previous.q_rad,),
                    1.0,
                    0.01,
                    False,
                    measured_velocities=(0.0,),
                )
                self.assertIsNotNone(result)
                progress = result[0].q_rad - previous.q_rad
                if goal > start:
                    self.assertGreater(progress, 0.0)
                else:
                    self.assertLess(progress, 0.0)
                self.assertLessEqual(abs(progress), 0.01)

    def test_projection_preserves_interior_and_exact_endpoint_goals(self):
        spec = native_spec(
            kp=180, kd=18, force_min=-14, force_max=14, cap=14,
            limits=PositionLimits(-1.2, 1.2),
        )
        for wanted in (-0.02, -0.005, 0.0, 0.005, 0.02):
            with self.subTest(wanted=wanted):
                projected = project_joint_target(
                    spec, spec.drive.target(wanted), -0.02, 0.02, -1.0, 1.0
                )
                self.assertEqual(projected.q_rad, wanted)

    def test_continuous_subcodec_grid_progress(self):
        spec = native_spec(
            kp=10, kd=1, force_min=-100, force_max=100, cap=100,
            velocity=1.0, acceleration=10.0, lead=1.0,
        )
        core, clock = armed((spec,), max_step=0.1)
        core.arm(owner="native")
        wanted = (5e-7, 1.1e-6, 1.7e-6)
        issued = []
        for sequence, q_rad in enumerate(wanted, start=1):
            pending = target(core, (spec.drive.target(q_rad),), sequence=sequence)
            self.assertTrue(core.submit_target(pending).accepted)
            clock.value = 0.1 * sequence
            reference = core.consume_target()
            self.assertIsNotNone(reference)
            issued.append(reference.joints[0].q_rad)
        self.assertEqual(issued, list(wanted))

        physical = physical_spec(cap=1000.0)
        quantized = project_joint_target(
            physical, JointTarget(5e-7, 0.0, 0.0, 0.0, 0.0), -0.1, 0.1, -1.0, 1.0
        )
        self.assertNotEqual(quantized.q_rad, 5e-7)

    def test_zero_elapsed_requires_unchanged_reference(self):
        spec = native_spec(kp=10, kd=1, force_min=-100, force_max=100, cap=100)
        core, _clock = armed((spec,))
        core.arm(owner="native")
        seed_pending = target(core, (spec.drive.target(0.01),))
        self.assertTrue(core.submit_target(seed_pending).accepted)
        seed_reference = core.consume_target()
        self.assertIsNotNone(seed_reference)
        self.assertEqual(seed_reference.joints[0], JointTarget(0.0, 0.0, 0.0, 10.0, 1.0))

        pending = target(core, (spec.drive.target(0.01),), sequence=1)
        self.assertTrue(core.submit_target(pending).accepted)
        history = core._history
        self.assertIsNone(core.consume_target())
        self.assertIs(core._history, history)
        self.assertIs(core.snapshot().pending_target, pending)

    def test_acceleration_lead_and_velocity_corridors(self):
        accel = native_spec(
            kp=10, kd=1, force_min=-100, force_max=100, cap=100,
            velocity=1.0, acceleration=0.5, lead=1.0,
        )
        candidate = plan_reference(
            (accel,),
            (accel.drive.target(0.05),),
            (accel.drive.target(0.0),),
            (0.0,),
            (0.0,),
            0.01,
            0.01,
            False,
            measured_velocities=(0.0,),
        )
        self.assertIsNotNone(candidate)
        self.assertAlmostEqual(candidate[0].q_rad, 5e-5, places=15)

        lead = native_spec(
            kp=10, kd=1, force_min=-100, force_max=100, cap=100,
            velocity=1.0, acceleration=100.0, lead=0.001,
        )
        candidate = plan_reference(
            (lead,),
            (lead.drive.target(0.05),),
            (lead.drive.target(0.0),),
            (0.0,),
            (0.0,),
            0.1,
            0.1,
            False,
            measured_velocities=(0.0,),
        )
        self.assertAlmostEqual(candidate[0].q_rad, 0.001, places=15)

        velocity = native_spec(
            kp=10, kd=1, force_min=-100, force_max=100, cap=100,
            velocity=0.1, acceleration=100.0, lead=1.0,
        )
        candidate = plan_reference(
            (velocity,),
            (velocity.drive.target(0.05),),
            (velocity.drive.target(0.0),),
            (0.0,),
            (0.0,),
            0.01,
            0.01,
            False,
            measured_velocities=(0.0,),
        )
        self.assertAlmostEqual(candidate[0].q_rad, 0.001, places=15)

    def test_whole_vector_failure_is_atomic_and_recoverable(self):
        first = native_spec(
            "J1", 0, kp=10, kd=1, force_min=-100, force_max=100, cap=100,
            velocity=1.0, acceleration=100.0, lead=1.0,
        )
        second = native_spec(
            "J2", 1, kp=100, kd=1, force_min=-3, force_max=3, cap=3,
            velocity=1.0, acceleration=100.0, lead=1.0,
        )
        core, clock = armed((first, second), max_step=0.1)
        core.arm(owner="native")
        pending = target(core, (first.drive.target(0.01), second.drive.target(0.1)))
        self.assertTrue(core.submit_target(pending).accepted)
        history = core._history
        clock.value = 0.1
        self.assertIsNone(core.consume_target())
        self.assertIs(core._history, history)
        self.assertIs(core.snapshot().pending_target, pending)

        safe = target(
            core, (first.drive.target(0.01), second.drive.target(0.01)),
            sequence=1,
        )
        self.assertTrue(core.submit_target(safe).accepted)
        clock.value = 0.2
        reference = core.consume_target()
        self.assertIsNotNone(reference)
        self.assertIsNot(reference.requested, pending)
        self.assertEqual(len(reference.joints), 2)


class NativeEffortTests(unittest.TestCase):
    def test_applied_uses_actual_fixed_gains_and_signed_damping(self):
        spec = native_spec(
            kp=10, kd=3, force_min=-20, force_max=20, cap=20,
            velocity=1.0, acceleration=100.0, lead=1.0,
        )
        applied = _applied_reference(spec, spec.drive.target(0.2))
        self.assertEqual(
            (applied.q_rad, applied.dq_rad_s, applied.tau_nm, applied.kp, applied.kd),
            (0.2, 0.0, 0.0, 10.0, 3.0),
        )
        self.assertAlmostEqual(
            _predicted_effort(applied, 0.1, 0.5), 10.0 * 0.1 - 3.0 * 0.5
        )
        self.assertAlmostEqual(
            _predicted_effort(applied, 0.1, -0.5), 10.0 * 0.1 + 3.0 * 0.5
        )

        core, clock = armed((spec,), velocities=(1.5,), max_step=0.1)
        core.arm(owner="native")
        pending = target(core, (spec.drive.target(0.0),))
        self.assertTrue(core.submit_target(pending).accepted)
        clock.value = 0.1
        reference = core.consume_target()
        self.assertIsNotNone(reference)
        issued = _applied_reference(spec, reference.joints[0])
        self.assertEqual((issued.kp, issued.kd), (10.0, 3.0))
        self.assertAlmostEqual(
            _predicted_effort(issued, 0.0, 1.5), -3.0 * 1.5
        )

    def test_signed_damping_can_push_the_whole_vector_over_the_limit(self):
        spec = native_spec(
            kp=10, kd=3, force_min=-5, force_max=5, cap=5,
            velocity=1.0, acceleration=100.0, lead=1.0,
        )
        core, clock = armed((spec,), velocities=(-1.5,))
        self.assertTrue(core.arm(owner="native").accepted)
        pending = target(core, (spec.drive.target(0.1),))
        self.assertTrue(core.submit_target(pending).accepted)
        history = core._history
        clock.value = 0.1
        self.assertIsNone(core.consume_target())
        self.assertIs(core._history, history)
        self.assertIs(core.snapshot().pending_target, pending)


class NativeReleaseBarrierTests(unittest.TestCase):
    def test_release_effort_veto_is_independent_of_planner(self):
        spec = native_spec(
            kp=10, kd=1, force_min=-5, force_max=5, cap=5,
            velocity=10.0, acceleration=1000.0, lead=1.0,
        )
        core, clock = armed((spec,), max_step=0.1)
        core.arm(owner="native")
        pending = target(core, (spec.drive.target(0.0),))
        self.assertTrue(core.submit_target(pending).accepted)
        clock.value = 0.1
        unsafe = (
            JointTarget(0.6, 0.0, 0.0, spec.drive.kp, spec.drive.kd),
        )
        with patch.object(core_module, "plan_reference", return_value=unsafe):
            self.assertIsNone(core.consume_target())
        self.assertIs(core.snapshot().pending_target, pending)

    def _release_mutation(self, action, *, expiry=10.0, feedback_age=10.0):
        spec = native_spec(
            kp=10, kd=1, force_min=-100, force_max=100, cap=100,
            velocity=1.0, acceleration=100.0, lead=1.0,
        )
        core, clock = armed(
            (spec,), feedback_age=feedback_age, max_step=0.1
        )
        core.arm(owner="native")
        pending = target(core, (spec.drive.target(0.01),), expiry=expiry)
        self.assertTrue(core.submit_target(pending).accepted)
        history = core._history
        clock.value = 0.1
        original = core_module._applied_reference
        fired = False

        def mutate_after_applying(*args):
            nonlocal fired
            applied = original(*args)
            if not fired:
                fired = True
                action(core, clock)
            return applied

        with patch.object(
            core_module, "_applied_reference", side_effect=mutate_after_applying
        ):
            self.assertIsNone(core.consume_target())
        self.assertTrue(fired)
        return core, pending, history, spec

    def test_final_encoding_expiry_preserves_history_and_clears_pending(self):
        core, _pending, history, _spec = self._release_mutation(
            lambda core, clock: setattr(clock, "value", 0.2), expiry=0.2
        )
        self.assertIsNone(core.snapshot().pending_target)
        self.assertIs(core._history, history)

    def test_final_encoding_staleness_latches_fault(self):
        core, _pending, _history, _spec = self._release_mutation(
            lambda core, clock: setattr(clock, "value", 0.3), feedback_age=0.2
        )
        self.assertIs(core.snapshot().state, State.FAULT)
        self.assertTrue(core.snapshot().faults[0].stale)
        self.assertIsNone(core.snapshot().ownership)

    def test_final_encoding_feedback_shift_vetoes_and_retries(self):
        def shift(core, clock):
            core.receive(0, JointSample(0.0, 0.5, 0.0), STOP)

        core, pending, history, _spec = self._release_mutation(shift)
        self.assertIs(core.snapshot().pending_target, pending)
        self.assertIs(core._history, history)
        self.assertIsNotNone(core.consume_target())

    def test_final_encoding_stop_preserves_new_state(self):
        core, _pending, _history, _spec = self._release_mutation(
            lambda core, clock: core.stop()
        )
        self.assertIs(core.snapshot().state, State.STOPPED)
        self.assertIsNone(core.snapshot().pending_target)
        self.assertIsNone(core._history)

    def test_final_encoding_owner_takeover_preserves_new_work(self):
        def takeover(core, clock):
            spec = core.joint_specs[0]
            core.stop()
            core.begin_check()
            core.receive(0, JointSample(0.0, 0.0, 0.0), STOP)
            core.checked()
            core.arm(owner="replacement")
            replacement = Target(
                1,
                (spec.drive.target(0.005),),
                0.0,
                10.0,
                core.snapshot().ownership,
            )
            self.assertTrue(core.submit_target(replacement).accepted)

        core, old, history, _spec = self._release_mutation(takeover)
        self.assertEqual(core.snapshot().ownership.owner, "replacement")
        self.assertIsNot(core.snapshot().pending_target, old)
        self.assertIsNot(core._history, history)
        self.assertIs(core.snapshot().state, State.ARMED)


class NativeSourceIdentityTests(unittest.TestCase):
    def test_native_core_rejects_physical_source(self):
        specs = (
            native_spec(),
            native_spec("J2", 1, kp=20, kd=2, force_min=-5, force_max=5),
        )
        core = JointCore(
            specs, clock=DeterministicClock(0.0), max_reference_step_s=0.1
        )
        self.assertTrue(core.begin_connect().accepted)
        decision = core.connected(SourceIdentity("physical", "arm", 1))
        self.assertEqual(decision.code, DecisionCode.INVALID_SOURCE)
        self.assertIs(core.snapshot().state, State.CONNECTING)
        self.assertIsNone(core.snapshot().source)
        self.assertTrue(
            core.connected(SourceIdentity("demo", "arm", 1)).accepted
        )

    def test_mixed_physical_core_still_accepts_physical_source(self):
        core = JointCore(
            (physical_spec(),), clock=DeterministicClock(0.0),
            max_reference_step_s=0.1,
        )
        self.assertTrue(core.begin_connect().accepted)
        self.assertTrue(
            core.connected(SourceIdentity("physical", "arm", 1)).accepted
        )


if __name__ == "__main__":
    unittest.main()
