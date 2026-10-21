import math
import unittest
from unittest.mock import patch

from im6014_control import reference
from im6014_control.codec import decode_command, raw_to_position, raw_to_rpm
from im6014_control.core import JointCore
from im6014_control.records import (
    DecisionCode,
    JointSpec,
    MotorHealth,
    PositionLimits,
    Reference,
    SourceIdentity,
    State,
    Target,
)
from im6014_control.mapping import JointMap, JointSample, JointTarget, OutputSample
from tests.control.test_controller import DeterministicClock, FakeStation


SOURCE = SourceIdentity("fixture", "controller-velocity", 1)
STOP = MotorHealth(0, 0, 0, False)


def armed_core(specs, seeds, *, clock=None, max_step=0.02, feedback_max_age=1.0):
    if clock is None:
        clock = DeterministicClock(0.0)
    core = JointCore(
        specs,
        clock=clock,
        feedback_max_age_s=feedback_max_age,
        max_reference_step_s=max_step,
    )
    assert core.begin_connect()
    assert core.connected(SOURCE)
    assert core.begin_check()
    for spec, position in zip(specs, seeds):
        assert core.receive(spec.motor_id, JointSample(position, 0.0, 0.0), STOP)
    assert core.checked()
    assert core.arm(owner="test").accepted
    return core, FakeStation(core), clock


def target(core, clock, sequence, joints, *, created_at_s=None, expires_at_s=100.0):
    if created_at_s is None:
        created_at_s = clock.value
    return Target(
        sequence,
        tuple(joints),
        created_at_s=created_at_s,
        expires_at_s=expires_at_s,
        ownership=core.snapshot().ownership,
    )


class ReferenceVelocityTests(unittest.TestCase):
    def test_velocity_and_step_limits_are_required_finite_and_positive(self):
        mapping = JointMap(1, 1, 0.0)
        limits = PositionLimits(-1.0, 1.0)
        for value in (True, False, 0, -1.0, math.nan, math.inf, -math.inf, 10**400, "1"):
            with self.subTest(value=repr(value)), self.assertRaises(ValueError):
                JointSpec("J1", 0, mapping, limits, value, max_acceleration_rad_s2=1000000.0, max_measured_lead_rad=1000000.0, max_effort_nm=1e100)
            with self.subTest(step=value), self.assertRaises(ValueError):
                JointCore(
                    (JointSpec("J1", 0, mapping, limits, 1.0, 1000000.0, 1000000.0, max_effort_nm=1e100),),
                    max_reference_step_s=value,
                )
        with self.assertRaises(TypeError):
            JointSpec("J1", 0, mapping, limits, max_effort_nm=1e100)
        with self.assertRaises(TypeError):
            JointCore((JointSpec("J1", 0, mapping, limits, 1.0, 1000000.0, 1000000.0, max_effort_nm=1e100),))

    def test_arm_rejects_measured_seed_outside_codec_range_without_grant(self):
        spec = JointSpec(
            "J1", 0, JointMap(1, 1, 0.0), PositionLimits(-20_000_000.0, 20_000_000.0), 1.0, 1000000.0, 1000000.0,
            max_effort_nm=1e100,
        )
        core = JointCore((spec,), max_reference_step_s=0.02)
        self.assertTrue(core.begin_connect())
        self.assertTrue(core.connected(SOURCE))
        self.assertTrue(core.begin_check())
        self.assertTrue(core.receive(0, JointSample(10_000_000.0, 0.0, 0.0), STOP))
        self.assertTrue(core.checked())

        decision = core.arm(owner="test")

        self.assertFalse(decision.accepted)
        self.assertIs(decision.code, DecisionCode.INVALID_ARGUMENT)
        snapshot = core.snapshot()
        self.assertIs(snapshot.state, State.READY)
        self.assertIsNone(snapshot.ownership)
        self.assertEqual(snapshot.ownership_generation, 0)
        self.assertIsNone(snapshot.pending_target)

    def test_arm_surfaces_projection_type_error_before_grant_or_reference(self):
        spec = JointSpec("J1", 0, JointMap(1, 1, 0.0), PositionLimits(-1.0, 1.0), 1.0, 1000000.0, 1000000.0, max_effort_nm=1e100)
        clock = DeterministicClock(0.0)
        core = JointCore((spec,), clock=clock, max_reference_step_s=0.02)
        station = FakeStation(core)
        self.assertTrue(station.connect().accepted)
        self.assertTrue(station.check().accepted)
        station.writes.clear()

        with patch.object(
            reference,
            "project_position_raw",
            side_effect=TypeError("injected projection programming error"),
        ):
            with self.assertRaisesRegex(TypeError, "injected projection programming error"):
                station.arm(owner="test")

        snapshot = core.snapshot()
        self.assertIs(snapshot.state, State.READY)
        self.assertIsNone(snapshot.ownership)
        self.assertIsNone(snapshot.pending_target)
        self.assertIsNone(station.consume())
        self.assertEqual(station.writes, [])

    def test_consume_surfaces_projection_type_error_without_reference_or_write(self):
        specs = (JointSpec("J1", 0, JointMap(1, 1, 0.0), PositionLimits(-1.0, 1.0), 1.0, 1000000.0, 1000000.0, max_effort_nm=1e100),)
        core, station, clock = armed_core(specs, (0.0,))
        request = target(core, clock, 0, (JointTarget(0.5, 0.0, 0.0, 1.0, 0.1),))
        self.assertTrue(station.submit(request))
        clock.value = 0.01

        with patch.object(
            reference,
            "project_position_raw",
            side_effect=TypeError("injected projection programming error"),
        ):
            with self.assertRaisesRegex(TypeError, "injected projection programming error"):
                station.consume()

        snapshot = core.snapshot()
        self.assertIs(snapshot.state, State.ARMED)
        self.assertIs(snapshot.pending_target, request)
        self.assertEqual(snapshot.last_sequence, request.sequence)
        self.assertIsNotNone(snapshot.ownership)
        self.assertEqual(station.writes, [])

    def test_nonzero_measured_seed_per_joint_steps_reversal_and_feedforward_clipping(self):
        specs = (
            JointSpec("J1", 0, JointMap(1, 1, 0.0), PositionLimits(-2.0, 2.0), 0.4, 1000000.0, 1000000.0, max_effort_nm=1e100),
            JointSpec("J2", 1, JointMap(1, -1, 0.0), PositionLimits(-2.0, 2.0), 0.8, 1000000.0, 1000000.0, max_effort_nm=1e100),
        )
        core, station, clock = armed_core(specs, (0.2, -0.3), max_step=0.2)
        first = target(core, clock, 0, (
            JointTarget(1.0, 1.0, 0.0, 1.0, 0.1),
            JointTarget(-1.0, -1.5, 0.0, 1.0, 0.1),
        ))
        clock.value = 0.1
        self.assertTrue(station.submit(first))
        issued = station.consume()
        self.assertIsInstance(issued, Reference)
        self.assertEqual(issued.requested, first)
        self.assertGreater(issued.joints[0].q_rad, 0.2)
        self.assertLessEqual(issued.joints[0].q_rad - 0.2, 0.04)
        self.assertLess(issued.joints[1].q_rad, -0.3)
        self.assertGreaterEqual(issued.joints[1].q_rad + 0.3, -0.08)
        self.assertLessEqual(issued.joints[0].dq_rad_s, 0.4)
        self.assertGreaterEqual(issued.joints[0].dq_rad_s, -0.4)
        self.assertLessEqual(issued.joints[1].dq_rad_s, 0.8)
        self.assertGreaterEqual(issued.joints[1].dq_rad_s, -0.8)

        reverse = target(core, clock, 1, (
            JointTarget(-1.0, -1.0, 0.0, 1.0, 0.1),
            JointTarget(1.0, 1.5, 0.0, 1.0, 0.1),
        ))
        clock.value = 0.2
        self.assertTrue(station.submit(reverse))
        reversed_reference = station.consume()
        self.assertLess(reversed_reference.joints[0].q_rad, issued.joints[0].q_rad)
        self.assertGreaterEqual(
            reversed_reference.joints[0].q_rad - issued.joints[0].q_rad, -0.04
        )
        self.assertGreater(reversed_reference.joints[1].q_rad, issued.joints[1].q_rad)
        self.assertLessEqual(
            reversed_reference.joints[1].q_rad - issued.joints[1].q_rad, 0.08
        )

    def test_zero_elapsed_is_stationary_but_same_tick_streaming_advances(self):
        specs = (JointSpec("J1", 0, JointMap(1, 1, 0.0), PositionLimits(-1, 1), 1.0, 1000000.0, 1000000.0, max_effort_nm=1e100),)
        core, station, clock = armed_core(specs, (0.0,), max_step=0.02)
        first = target(core, clock, 0, (JointTarget(0.5, 2.0, 0.0, 1.0, 0.1),))
        self.assertTrue(station.submit(first))
        at_seed = station.consume()
        self.assertIsNotNone(at_seed)
        self.assertEqual(at_seed.joints[0].q_rad, 0.0)
        self.assertLessEqual(abs(at_seed.joints[0].dq_rad_s), 1.0)
        self.assertEqual((at_seed.accepted_at_s, at_seed.issued_at_s), (0.0, 0.0))

        clock.value = 0.01
        streamed = target(core, clock, 1, (JointTarget(0.5, 0.0, 0.0, 1.0, 0.1),))
        self.assertTrue(station.submit(streamed))
        advanced = station.consume()
        self.assertEqual(advanced.accepted_at_s, advanced.issued_at_s)
        self.assertGreater(advanced.joints[0].q_rad, at_seed.joints[0].q_rad)
        self.assertLessEqual(advanced.joints[0].q_rad, 0.01)

        clock.value = 0.02
        next_request = target(core, clock, 2, (JointTarget(0.5, 0.0, 0.0, 1.0, 0.1),))
        self.assertTrue(station.submit(next_request))
        next_reference = station.consume()
        self.assertGreater(next_reference.joints[0].q_rad, advanced.joints[0].q_rad)
        self.assertLessEqual(next_reference.joints[0].q_rad - advanced.joints[0].q_rad, 0.01)

    def test_acceptance_queue_and_idle_time_never_bank_more_than_step_cap(self):
        specs = (JointSpec("J1", 0, JointMap(1, 1, 0.0), PositionLimits(-2, 2), 1.0, 1000000.0, 1000000.0, max_effort_nm=1e100),)
        core, station, clock = armed_core(
            specs, (0.0,), max_step=0.02, feedback_max_age=20.0
        )
        delayed = target(
            core, clock, 0, (JointTarget(1.0, 0.0, 0.0, 1.0, 0.1),),
            created_at_s=0.0, expires_at_s=20.0,
        )
        clock.value = 0.5
        self.assertTrue(station.submit(delayed))
        clock.value = 0.6
        issued = station.consume()
        self.assertEqual(issued.accepted_at_s, 0.5)
        self.assertLessEqual(issued.joints[0].q_rad, 0.02)

        clock.value = 10.0
        idle_target = target(core, clock, 1, (JointTarget(1.0, 0.0, 0.0, 1.0, 0.1),))
        self.assertTrue(station.submit(idle_target))
        idle_issue = station.consume()
        self.assertLessEqual(idle_issue.joints[0].q_rad - issued.joints[0].q_rad, 0.02)

    def test_absent_expired_invalid_replacement_and_clock_regression_preserve_bounds(self):
        specs = (JointSpec("J1", 0, JointMap(1, 1, 0.0), PositionLimits(-1, 1), 1.0, 1000000.0, 1000000.0, max_effort_nm=1e100),)
        core, station, clock = armed_core(specs, (0.0,))
        self.assertIsNone(station.consume())
        self.assertEqual(station.writes, [])
        seed_status = core.snapshot().joints[0]

        accepted = target(core, clock, 0, (JointTarget(0.5, 0.0, 0.0, 1.0, 0.1),), expires_at_s=0.1)
        self.assertTrue(station.submit(accepted))
        pending_before = core.snapshot().pending_target
        invalid = target(core, clock, 1, (JointTarget(2.0, 0.0, 0.0, 1.0, 0.1),))
        self.assertEqual(station.submit(invalid).code, DecisionCode.POSITION_LIMIT)
        snapshot = core.snapshot()
        self.assertIs(snapshot.pending_target, pending_before)
        self.assertEqual(snapshot.last_sequence, accepted.sequence)
        self.assertEqual(station.writes, [])

        clock.value = 0.1
        self.assertIsNone(station.consume())
        self.assertIsNone(core.snapshot().pending_target)
        self.assertEqual(core.snapshot().last_sequence, 0)
        self.assertEqual(station.writes, [])

        newer = target(core, clock, 1, (JointTarget(0.5, 0.0, 0.0, 1.0, 0.1),), expires_at_s=1.0)
        self.assertTrue(station.submit(newer))
        clock.value = 0.05
        with self.assertRaises(ValueError):
            station.consume()
        clock.value = 0.1
        snapshot = core.snapshot()
        self.assertIs(snapshot.pending_target, newer)
        self.assertEqual(snapshot.last_sequence, newer.sequence)
        self.assertEqual(station.writes, [])

        issued = station.consume()
        self.assertIsInstance(issued, Reference)
        self.assertIs(issued.requested, newer)
        self.assertGreater(issued.joints[0].q_rad, seed_status.sample.q_rad)
        self.assertLessEqual(
            issued.joints[0].q_rad - seed_status.sample.q_rad,
            specs[0].max_velocity_rad_s
            * min(issued.issued_at_s - seed_status.received_at, 0.02),
        )

    def test_failed_second_joint_projection_keeps_pending_and_history_atomically(self):
        specs = (
            JointSpec("J1", 0, JointMap(1, 1, 0.0), PositionLimits(-1, 1), 1.0, 1000000.0, 1000000.0, max_effort_nm=1e100),
            JointSpec("J2", 1, JointMap(1, -1, 0.0), PositionLimits(-1, 1), 1.0, 1000000.0, 1000000.0, max_effort_nm=1e100),
        )
        core, station, clock = armed_core(specs, (0.0, 0.0))
        seed_target = target(core, clock, 0, (
            JointTarget(0.0, 0.0, 0.0, 1.0, 0.1),
            JointTarget(0.0, 0.0, 0.0, 1.0, 0.1),
        ))
        self.assertTrue(station.submit(seed_target))
        seed_reference = station.consume()
        self.assertIsInstance(seed_reference, Reference)

        request = target(core, clock, 1, (
            JointTarget(0.5, 0.0, 0.0, 1.0, 0.1),
            JointTarget(-0.5, 0.0, 0.0, 1.0, 0.1),
        ))
        self.assertTrue(station.submit(request))
        clock.value = 0.01
        original = reference.project_position_raw

        def fail_second(*args, **kwargs):
            wanted_q = args[0] if args else kwargs.get("wanted_q")
            q_min = args[1] if len(args) > 1 else kwargs.get("q_min")
            q_max = args[2] if len(args) > 2 else kwargs.get("q_max")
            if wanted_q == -0.5 and (q_min > -1.0 or q_max < 1.0):
                raise ValueError("injected second-joint projection failure")
            return original(*args, **kwargs)

        writes_before = len(station.writes)
        with patch.object(reference, "project_position_raw", side_effect=fail_second):
            self.assertIsNone(station.consume())
        snapshot = core.snapshot()
        self.assertEqual(len(station.writes), writes_before)
        self.assertIs(snapshot.pending_target, request)
        self.assertEqual(snapshot.last_sequence, request.sequence)
        self.assertIs(snapshot.state, State.MOVING)

        opposite = target(core, clock, 2, (
            JointTarget(-0.5, 0.0, 0.0, 1.0, 0.1),
            JointTarget(0.5, 0.0, 0.0, 1.0, 0.1),
        ))
        self.assertTrue(station.submit(opposite))
        clock.value = 0.02
        recovered = station.consume()
        self.assertIsInstance(recovered, Reference)
        self.assertIs(recovered.requested, opposite)
        self.assertLess(recovered.joints[0].q_rad, seed_reference.joints[0].q_rad - 0.015)
        self.assertGreater(recovered.joints[1].q_rad, seed_reference.joints[1].q_rad + 0.015)
        for issued, seed in zip(recovered.joints, seed_reference.joints):
            self.assertLessEqual(abs(issued.q_rad - seed.q_rad), 0.02)

    def test_projection_time_passing_target_deadline_discards_without_release(self):
        specs = (JointSpec("J1", 0, JointMap(1, 1, 0.0), PositionLimits(-1, 1), 1.0, 1000000.0, 1000000.0, max_effort_nm=1e100),)
        core, station, clock = armed_core(specs, (0.0,), max_step=0.1)
        seed_target = target(core, clock, 0, (JointTarget(0.0, 0.0, 0.0, 1.0, 0.1),))
        self.assertTrue(station.submit(seed_target))
        seed_reference = station.consume()
        self.assertIsInstance(seed_reference, Reference)
        request = target(
            core, clock, 1, (JointTarget(0.5, 0.0, 0.0, 1.0, 0.1),),
            created_at_s=0.0, expires_at_s=0.1,
        )
        self.assertTrue(station.submit(request))
        clock.value = 0.05
        before = core.snapshot()
        writes_before = len(station.writes)
        calls_before = clock.calls
        original = reference.project_position_raw

        def pass_deadline_during_projection(*args, **kwargs):
            projected = original(*args, **kwargs)
            clock.value = 0.2
            return projected

        with patch.object(
            reference,
            "project_position_raw",
            side_effect=pass_deadline_during_projection,
        ):
            self.assertIsNone(station.consume())

        self.assertEqual(clock.calls - calls_before, 2)
        snapshot = core.snapshot()
        self.assertEqual(len(station.writes), writes_before)
        self.assertIsNone(snapshot.pending_target)
        self.assertEqual(snapshot.last_sequence, request.sequence)
        self.assertIs(snapshot.state, State.MOVING)
        self.assertIs(snapshot.ownership, before.ownership)
        self.assertEqual(snapshot.ownership_generation, before.ownership_generation)

        opposite = target(core, clock, 2, (JointTarget(-0.5, 0.0, 0.0, 1.0, 0.1),))
        self.assertTrue(station.submit(opposite))
        recovered = station.consume()
        self.assertIsInstance(recovered, Reference)
        self.assertIs(recovered.requested, opposite)
        self.assertLess(recovered.joints[0].q_rad, seed_reference.joints[0].q_rad - 0.075)
        self.assertGreaterEqual(
            recovered.joints[0].q_rad - seed_reference.joints[0].q_rad,
            -specs[0].max_velocity_rad_s
            * min(recovered.issued_at_s - seed_reference.issued_at_s, 0.1),
        )

    def test_projection_time_staling_feedback_revokes_without_release(self):
        specs = (JointSpec("J1", 0, JointMap(1, 1, 0.0), PositionLimits(-1, 1), 1.0, 1000000.0, 1000000.0, max_effort_nm=1e100),)
        core, station, clock = armed_core(
            specs, (0.0,), max_step=0.1, feedback_max_age=0.1
        )
        request = target(
            core, clock, 1, (JointTarget(0.5, 0.0, 0.0, 1.0, 0.1),),
            created_at_s=0.0, expires_at_s=1.0,
        )
        self.assertTrue(station.submit(request))
        clock.value = 0.05
        before = core.snapshot()
        calls_before = clock.calls
        original = reference.project_position_raw

        def stale_feedback_during_projection(*args, **kwargs):
            projected = original(*args, **kwargs)
            clock.value = 0.2
            return projected

        with patch.object(
            reference,
            "project_position_raw",
            side_effect=stale_feedback_during_projection,
        ):
            self.assertIsNone(station.consume())

        self.assertEqual(clock.calls - calls_before, 2)
        self.assertEqual(station.writes, [])
        snapshot = core.snapshot()
        self.assertIsNone(snapshot.pending_target)
        self.assertEqual(snapshot.last_sequence, request.sequence)
        self.assertIs(snapshot.state, State.FAULT)
        self.assertTrue(snapshot.fault_latched)
        self.assertEqual(snapshot.affected_joints, ("J1",))
        self.assertTrue(snapshot.faults[0].stale)
        self.assertIsNone(snapshot.ownership)
        self.assertEqual(snapshot.ownership_generation, before.ownership_generation + 1)

    def test_projection_compute_delay_stamps_final_release_time(self):
        specs = (JointSpec("J1", 0, JointMap(1, 1, 0.0), PositionLimits(-1, 1), 1.0, 1000000.0, 1000000.0, max_effort_nm=1e100),)
        core, station, clock = armed_core(specs, (0.0,), max_step=0.1)
        request = target(
            core, clock, 1, (JointTarget(0.5, 0.0, 0.0, 1.0, 0.1),),
            created_at_s=0.0, expires_at_s=1.0,
        )
        self.assertTrue(station.submit(request))
        clock.value = 0.05
        calls_before = clock.calls
        original = reference.project_position_raw

        def small_compute_delay(*args, **kwargs):
            projected = original(*args, **kwargs)
            clock.value = 0.06
            return projected

        with patch.object(
            reference,
            "project_position_raw",
            side_effect=small_compute_delay,
        ):
            issued = station.consume()

        self.assertIsNotNone(issued)
        self.assertIs(issued.requested, request)
        self.assertEqual(issued.issued_at_s, 0.06)
        self.assertEqual(clock.calls - calls_before, 2)
        snapshot = core.snapshot()
        self.assertIs(snapshot.state, State.MOVING)
        self.assertIsNone(snapshot.pending_target)
        self.assertEqual(snapshot.last_sequence, request.sequence)
        self.assertEqual(len(station.writes), 1)

        next_request = target(
            core, clock, 2, (JointTarget(0.5, 0.0, 0.0, 1.0, 0.1),),
            expires_at_s=1.0,
        )
        self.assertTrue(station.submit(next_request))
        clock.value = 0.07
        next_issue = station.consume()
        self.assertIsNotNone(next_issue)
        self.assertLessEqual(
            abs(next_issue.joints[0].q_rad - issued.joints[0].q_rad),
            specs[0].max_velocity_rad_s * (next_issue.issued_at_s - issued.issued_at_s),
        )
        self.assertGreater(next_issue.joints[0].q_rad, issued.joints[0].q_rad)
        self.assertEqual(next_issue.issued_at_s, 0.07)
        self.assertEqual(core.snapshot().last_sequence, next_request.sequence)
        self.assertEqual(len(station.writes), 2)

    def test_clock_regression_during_projection_preserves_history_and_writes_nothing(self):
        specs = (JointSpec("J1", 0, JointMap(1, 1, 0.0), PositionLimits(-1, 1), 1.0, 1000000.0, 1000000.0, max_effort_nm=1e100),)
        core, station, clock = armed_core(specs, (0.0,), max_step=0.1)
        seed_target = target(core, clock, 0, (JointTarget(0.0, 0.0, 0.0, 1.0, 0.1),))
        self.assertTrue(station.submit(seed_target))
        seed_reference = station.consume()
        self.assertIsInstance(seed_reference, Reference)
        request = target(
            core, clock, 1, (JointTarget(0.5, 0.0, 0.0, 1.0, 0.1),),
            created_at_s=0.0, expires_at_s=1.0,
        )
        self.assertTrue(station.submit(request))
        clock.value = 0.05
        writes_before = len(station.writes)
        before = core.snapshot()
        original = reference.project_position_raw

        def regress_clock_during_projection(*args, **kwargs):
            projected = original(*args, **kwargs)
            clock.value = 0.04
            return projected

        with patch.object(
            reference,
            "project_position_raw",
            side_effect=regress_clock_during_projection,
        ):
            with self.assertRaises(ValueError):
                station.consume()

        clock.value = 0.05
        snapshot = core.snapshot()
        self.assertEqual(len(station.writes), writes_before)
        self.assertIs(snapshot.pending_target, request)
        self.assertEqual(snapshot.last_sequence, before.last_sequence)
        self.assertIs(snapshot.state, State.MOVING)
        self.assertIs(snapshot.ownership, before.ownership)

        opposite = target(core, clock, 2, (JointTarget(-0.5, 0.0, 0.0, 1.0, 0.1),))
        self.assertTrue(station.submit(opposite))
        recovered = station.consume()
        self.assertIsInstance(recovered, Reference)
        self.assertIs(recovered.requested, opposite)
        self.assertLess(recovered.joints[0].q_rad, seed_reference.joints[0].q_rad - 0.04)
        self.assertGreaterEqual(
            recovered.joints[0].q_rad - seed_reference.joints[0].q_rad,
            -specs[0].max_velocity_rad_s
            * min(recovered.issued_at_s - seed_reference.issued_at_s, 0.1),
        )

    def test_subgrid_zero_elapsed_corridor_is_retained_until_feasible(self):
        specs = (JointSpec("J1", 0, JointMap(1, 1, 0.0), PositionLimits(-1, 1), 0.1, 1000000.0, 1000000.0, max_effort_nm=1e100),)
        core, station, clock = armed_core(specs, (0.0001,), max_step=0.02)
        seed_status = core.snapshot().joints[0]
        request = target(core, clock, 0, (JointTarget(0.2, 0.0, 0.0, 1.0, 0.1),))
        self.assertTrue(station.submit(request))
        self.assertIsNone(station.consume())
        self.assertEqual(core.snapshot().pending_target, request)
        self.assertEqual(station.writes, [])

        clock.value = 0.01
        later = station.consume()
        self.assertIsInstance(later, Reference)
        self.assertEqual(later.requested, request)
        self.assertIsNone(core.snapshot().pending_target)
        self.assertLessEqual(
            abs(later.joints[0].q_rad - seed_status.sample.q_rad),
            specs[0].max_velocity_rad_s
            * (later.issued_at_s - seed_status.received_at),
        )

    def test_lifecycle_rearm_seeds_new_measured_pose_and_requires_new_grant(self):
        specs = (JointSpec("J1", 0, JointMap(1, 1, 0.0), PositionLimits(-1, 1), 0.5, 1000000.0, 1000000.0, max_effort_nm=1e100),)
        core, station, clock = armed_core(specs, (0.0,), max_step=0.1)
        old_grant = core.snapshot().ownership
        self.assertTrue(station.stop())
        stopped = core.snapshot()
        self.assertIs(stopped.state, State.STOPPED)
        self.assertIsNone(stopped.ownership)
        self.assertIsNone(stopped.pending_target)
        self.assertTrue(core.begin_check())
        self.assertTrue(core.receive(0, JointSample(0.5, 0.0, 0.0), STOP))
        self.assertTrue(core.checked())
        new_grant = core.arm(owner="test")
        self.assertTrue(new_grant.accepted)
        self.assertIsNot(new_grant.ownership, old_grant)
        armed = core.snapshot()
        self.assertIs(armed.state, State.ARMED)
        self.assertIs(armed.ownership, new_grant.ownership)
        self.assertEqual(armed.joints[0].sample.q_rad, 0.5)

        stale_grant_target = Target(
            0, (JointTarget(0.8, 0.0, 0.0, 1.0, 0.1),),
            created_at_s=0.0, expires_at_s=10.0, ownership=old_grant,
        )
        self.assertEqual(station.submit(stale_grant_target).code, DecisionCode.INVALID_OWNERSHIP)
        current = target(core, clock, 0, (JointTarget(0.8, 0.0, 0.0, 1.0, 0.1),))
        self.assertTrue(station.submit(current))
        clock.value = 0.1
        issued = station.consume()
        self.assertIsInstance(issued, Reference)
        self.assertIs(issued.requested, current)
        self.assertGreaterEqual(issued.joints[0].q_rad, 0.5)
        self.assertLessEqual(
            issued.joints[0].q_rad - 0.5,
            specs[0].max_velocity_rad_s
            * min(issued.issued_at_s - armed.joints[0].received_at, 0.1),
        )

    def test_inward_codec_grid_projection_for_position_speed_reductions_and_directions(self):
        for reduction in (1, 3, 6):
            for direction in (-1, 1):
                with self.subTest(reduction=reduction, direction=direction):
                    mapping = JointMap(reduction, direction, -direction * 0.37)
                    position_code = 100
                    q_grid = mapping.from_output(
                        OutputSample(raw_to_position(position_code), 0.0, 0.0)
                    ).q_rad
                    upper = math.nextafter(q_grid, -math.inf)
                    speed_code = direction * 2000
                    speed_grid = mapping.from_output(
                        OutputSample(0.0, raw_to_rpm(speed_code), 0.0)
                    ).dq_rad_s
                    max_velocity = math.nextafter(abs(speed_grid), 0.0)
                    limits = PositionLimits(-1.0, upper)
                    spec = JointSpec("J1", 0, mapping, limits, max_velocity, 1000000.0, 1000000.0, max_effort_nm=1e100)
                    core, station, clock = armed_core((spec,), (0.0,), max_step=1.0)

                    for speed_sign in (-1, 1):
                        sequence = 0 if speed_sign < 0 else 1
                        request = target(core, clock, sequence, (
                            JointTarget(upper, speed_sign * 2 * max_velocity, 0.0, 1.0, 0.1),
                        ))
                        self.assertTrue(station.submit(request))
                        clock.value += 0.1
                        reference = station.consume()
                        self.assertIsNotNone(reference)
                        self.assertLessEqual(reference.joints[0].q_rad, upper)
                        self.assertGreaterEqual(reference.joints[0].q_rad, limits.minimum_rad)
                        self.assertLessEqual(abs(reference.joints[0].dq_rad_s), max_velocity)
                        packet = station.writes[-1]
                        decoded = decode_command(packet)
                        packet_joint = mapping.from_output(
                            OutputSample(decoded["position_rad"], decoded["rpm"], decoded["torque_nm"])
                        )
                        self.assertLessEqual(packet_joint.q_rad, upper)
                        self.assertGreaterEqual(packet_joint.q_rad, limits.minimum_rad)
                        self.assertLessEqual(abs(packet_joint.dq_rad_s), max_velocity)
                        self.assertEqual(packet_joint.q_rad, reference.joints[0].q_rad)
                        self.assertEqual(packet_joint.dq_rad_s, reference.joints[0].dq_rad_s)
                        self.assertNotEqual(decoded["raw_position"], position_code)
                        self.assertNotEqual(decoded["raw_rpm"], speed_sign * speed_code)


if __name__ == "__main__":
    unittest.main()
