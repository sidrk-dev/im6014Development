"""Acceptance and integration tests for C11 acceleration limits.

Tests verify required finite positive max_acceleration_rad_s2 on JointSpec,
independent q and dq corridors, strict closed bounds without outward epsilon,
decoded inward grids, complete-vector suppression on subgrid steps, bootstrap
waiver, statically settled updates, off-grid seed rate preservation, delay-induced
deceleration rate veto with retained epoch retry, queued/idle caps, and lifecycle
invalidation.
"""

import math
import unittest
from unittest.mock import patch

from im6014_control import reference
from im6014_control.codec import (
    command,
    decode_command,
    raw_to_position,
    raw_to_rpm,
)
from im6014_control.core import JointCore
from im6014_control.mapping import JointMap, JointSample, JointTarget, OutputSample
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


SOURCE = SourceIdentity("fixture", "controller-acceleration", 1)
STOP = MotorHealth(0, 0, 0, False)


class DeterministicClock:
    def __init__(self, value=0.0):
        self.value = value
        self.calls = 0

    def __call__(self):
        self.calls += 1
        return self.value


class AccelerationStation:
    def __init__(self, core):
        self.core = core
        self.writes = []
        self.ownership = None

    def _write_stops(self):
        self.writes.extend(command(spec.motor_id, mode=0) for spec in self.core.joint_specs)

    def connect(self):
        decision = self.core.begin_connect()
        if not decision.accepted:
            return decision
        decision = self.core.connected(SOURCE)
        if decision.accepted:
            self._write_stops()
        return decision

    def check(self, *, acknowledge_fault=False):
        decision = self.core.begin_check(acknowledge_fault=acknowledge_fault)
        if not decision.accepted:
            return decision
        self._write_stops()
        for spec in self.core.joint_specs:
            decision = self.core.receive(spec.motor_id, JointSample(0.0, 0.0, 0.0), STOP)
            if not decision.accepted:
                return decision
        return self.core.checked()

    def arm(self, *, owner="test"):
        decision = self.core.arm(owner=owner)
        if decision.accepted:
            self.ownership = decision.ownership
        return decision

    def submit(self, target):
        return self.core.submit_target(target)

    def consume(self):
        reference = self.core.consume_target()
        if reference is not None:
            for spec, joint_target in zip(self.core.joint_specs, reference.joints):
                output = spec.mapping.to_output(joint_target)
                self.writes.append(
                    command(
                        spec.motor_id,
                        mode=1,
                        rpm=output.rpm,
                        kd=output.kd,
                        torque=output.tau_nm,
                        watchdog=True,
                        position=output.q_rad,
                        kp=output.kp,
                    )
                )
        return reference

    def stop(self):
        decision = self.core.stop()
        if decision.accepted:
            self._write_stops()
        return decision

    def disconnect(self):
        return self.core.disconnect()


def armed_core(
    specs,
    seeds,
    *,
    clock=None,
    max_step=0.05,
    feedback_max_age=1.0,
):
    if clock is None:
        clock = DeterministicClock(0.0)
    core = JointCore(
        specs,
        clock=clock,
        feedback_max_age_s=feedback_max_age,
        max_reference_step_s=max_step,
    )
    assert core.begin_connect().accepted
    assert core.connected(SOURCE).accepted
    assert core.begin_check().accepted
    for spec, position in zip(specs, seeds):
        assert core.receive(spec.motor_id, JointSample(position, 0.0, 0.0), STOP).accepted
    assert core.checked().accepted
    grant = core.arm(owner="test")
    assert grant.accepted
    station = AccelerationStation(core)
    station.ownership = grant.ownership
    return core, station, clock


def make_target(
    core,
    clock,
    sequence,
    joints,
    *,
    created_at_s=None,
    expires_at_s=100.0,
    ownership=None,
):
    if created_at_s is None:
        created_at_s = clock.value
    if ownership is None:
        ownership = core.snapshot().ownership
    return Target(
        sequence,
        tuple(joints),
        created_at_s=created_at_s,
        expires_at_s=expires_at_s,
        ownership=ownership,
    )


class ReferenceAccelerationTests(unittest.TestCase):
    def test_acceleration_is_required_finite_and_positive_without_default(self):
        mapping = JointMap(1, 1, 0.0)
        limits = PositionLimits(-1.0, 1.0)
        with self.assertRaises(TypeError):
            JointSpec("J1", 0, mapping, limits, 1.0, max_effort_nm=1e100)
        for bad in (True, False, 0, 0.0, -1.0, -0.001, math.nan, math.inf, -math.inf, "1.0", None, 10**400):
            with self.subTest(bad=repr(bad)), self.assertRaises(ValueError):
                JointSpec("J1", 0, mapping, limits, 1.0, max_acceleration_rad_s2=bad, max_measured_lead_rad=1000000.0, max_effort_nm=1e100)
        for good in (0.01, 1.0, 50.0, 1000.0):
            spec = JointSpec("J1", 0, mapping, limits, 1.0, max_acceleration_rad_s2=good, max_measured_lead_rad=1000000.0, max_effort_nm=1e100)
            self.assertEqual(spec.max_acceleration_rad_s2, float(good))

    def test_dq_ramp_and_q_step_with_dq0_obey_acceleration_and_velocity_bounds(self):
        spec = JointSpec(
            "J1", 0, JointMap(1, 1, 0.0), PositionLimits(-2.0, 2.0),
            max_velocity_rad_s=1.0, max_acceleration_rad_s2=2.0, max_measured_lead_rad=1000000.0,
            max_effort_nm=1e100,
        )
        core, station, clock = armed_core((spec,), (0.0,), max_step=0.05)

        clock.value = 0.02
        tgt0 = make_target(core, clock, 0, (JointTarget(0.0, 1.0, 0.0, 1.0, 0.1),))
        self.assertTrue(station.submit(tgt0).accepted)
        ref0 = station.consume()
        self.assertIsNotNone(ref0)
        self.assertGreater(ref0.joints[0].dq_rad_s, 0.0)
        self.assertLessEqual(ref0.joints[0].dq_rad_s, 0.04000000001)
        self.assertEqual(ref0.joints[0].q_rad, 0.0)

        clock.value = 0.04
        tgt1 = make_target(core, clock, 1, (JointTarget(0.0, 1.0, 0.0, 1.0, 0.1),))
        self.assertTrue(station.submit(tgt1).accepted)
        ref1 = station.consume()
        self.assertIsNotNone(ref1)
        self.assertGreater(ref1.joints[0].dq_rad_s, ref0.joints[0].dq_rad_s)
        self.assertLessEqual(ref1.joints[0].dq_rad_s - ref0.joints[0].dq_rad_s, 0.04000000001)

        clock.value = 0.06
        tgt2 = make_target(core, clock, 2, (JointTarget(0.0, -1.0, 0.0, 1.0, 0.1),))
        self.assertTrue(station.submit(tgt2).accepted)
        ref2 = station.consume()
        self.assertIsNotNone(ref2)
        self.assertLess(ref2.joints[0].dq_rad_s, ref1.joints[0].dq_rad_s)
        self.assertGreaterEqual(ref2.joints[0].dq_rad_s - ref1.joints[0].dq_rad_s, -0.04000000001)

        clock.value = 0.08
        tgt3 = make_target(core, clock, 3, (JointTarget(0.5, 0.0, 0.0, 1.0, 0.1),))
        self.assertTrue(station.submit(tgt3).accepted)
        ref3 = station.consume()
        self.assertIsNotNone(ref3)
        self.assertGreater(ref3.joints[0].q_rad, 0.0)
        self.assertLessEqual(ref3.joints[0].q_rad, 0.02000000001)
        elapsed = ref3.issued_at_s - ref2.issued_at_s
        rate = (ref3.joints[0].q_rad - ref2.joints[0].q_rad) / elapsed
        self.assertLessEqual(abs(rate), 1.0)
        self.assertLessEqual(abs(rate - 0.0), 2.0 * elapsed)

    def test_independent_q_and_desired_dq_channels(self):
        spec = JointSpec(
            "J1", 0, JointMap(1, 1, 0.0), PositionLimits(-2.0, 2.0),
            max_velocity_rad_s=1.0, max_acceleration_rad_s2=2.0, max_measured_lead_rad=1000000.0,
            max_effort_nm=1e100,
        )
        core, station, clock = armed_core((spec,), (0.0,), max_step=0.05)

        clock.value = 0.02
        tgt0 = make_target(core, clock, 0, (JointTarget(0.0, 0.8, 0.0, 1.0, 0.1),))
        self.assertTrue(station.submit(tgt0).accepted)
        ref0 = station.consume()
        self.assertIsNotNone(ref0)
        self.assertEqual(ref0.joints[0].q_rad, 0.0)
        self.assertGreater(ref0.joints[0].dq_rad_s, 0.0)
        self.assertLessEqual(ref0.joints[0].dq_rad_s, 0.04000000001)

        clock.value = 0.04
        tgt1 = make_target(core, clock, 1, (JointTarget(0.4, 0.0, 0.0, 1.0, 0.1),))
        self.assertTrue(station.submit(tgt1).accepted)
        ref1 = station.consume()
        self.assertIsNotNone(ref1)
        self.assertGreater(ref1.joints[0].q_rad, 0.0)
        self.assertLess(ref1.joints[0].dq_rad_s, ref0.joints[0].dq_rad_s)

    def test_both_reversals_bound_dq_slew_and_position_rates(self):
        spec = JointSpec(
            "J1", 0, JointMap(1, 1, 0.0), PositionLimits(-2.0, 2.0),
            max_velocity_rad_s=1.0, max_acceleration_rad_s2=4.0, max_measured_lead_rad=1000000.0,
            max_effort_nm=1e100,
        )
        core, station, clock = armed_core((spec,), (0.0,), max_step=0.05)

        clock.value = 0.05
        tgt0 = make_target(core, clock, 0, (JointTarget(0.0, 1.0, 0.0, 1.0, 0.1),))
        self.assertTrue(station.submit(tgt0).accepted)
        ref0 = station.consume()
        self.assertIsNotNone(ref0)

        clock.value = 0.10
        tgt1 = make_target(core, clock, 1, (JointTarget(0.0, 1.0, 0.0, 1.0, 0.1),))
        self.assertTrue(station.submit(tgt1).accepted)
        ref1 = station.consume()
        self.assertIsNotNone(ref1)
        self.assertGreater(ref1.joints[0].dq_rad_s, ref0.joints[0].dq_rad_s)

        clock.value = 0.15
        tgt2 = make_target(core, clock, 2, (JointTarget(0.0, -1.0, 0.0, 1.0, 0.1),))
        self.assertTrue(station.submit(tgt2).accepted)
        ref2 = station.consume()
        self.assertIsNotNone(ref2)
        slew = ref2.joints[0].dq_rad_s - ref1.joints[0].dq_rad_s
        self.assertLess(slew, 0.0)
        self.assertGreaterEqual(slew, -4.0 * 0.05)

        clock.value = 0.20
        tgt3 = make_target(core, clock, 3, (JointTarget(0.5, 0.0, 0.0, 1.0, 0.1),))
        self.assertTrue(station.submit(tgt3).accepted)
        ref3 = station.consume()
        self.assertIsNotNone(ref3)
        rate_fwd = (ref3.joints[0].q_rad - ref2.joints[0].q_rad) / 0.05
        self.assertGreater(rate_fwd, 0.0)

        clock.value = 0.26
        tgt4 = make_target(core, clock, 4, (JointTarget(-0.5, 0.0, 0.0, 1.0, 0.1),))
        self.assertTrue(station.submit(tgt4).accepted)
        ref4 = station.consume()
        self.assertIsNotNone(ref4)
        elapsed_rev = ref4.issued_at_s - ref3.issued_at_s
        rate_rev = (ref4.joints[0].q_rad - ref3.joints[0].q_rad) / elapsed_rev
        self.assertLessEqual(abs(rate_rev - rate_fwd), 4.0 * elapsed_rev)

    def test_both_directions_and_reductions_1_3_6_roundtrip_codec_and_respect_bounds(self):
        for reduction in (1, 3, 6):
            for direction in (1, -1):
                with self.subTest(reduction=reduction, direction=direction):
                    spec = JointSpec(
                        f"J_{reduction}_{direction}", 0,
                        JointMap(reduction=reduction, direction=direction, zero_output_rad=0.05),
                        PositionLimits(-1.5, 1.5),
                        max_velocity_rad_s=0.8,
                        max_acceleration_rad_s2=2.0, max_measured_lead_rad=1000000.0,
                        max_effort_nm=1e100,
                    )
                    core, station, clock = armed_core((spec,), (0.0,), max_step=0.05)
                    clock.value = 0.02
                    tgt = make_target(core, clock, 0, (JointTarget(0.3, 0.5, 0.0, 1.0, 0.1),))
                    self.assertTrue(station.submit(tgt).accepted)
                    ref = station.consume()
                    self.assertIsNotNone(ref)
                    jt = ref.joints[0]
                    self.assertGreater(jt.q_rad, 0.0)
                    self.assertGreater(jt.dq_rad_s, 0.0)
                    self.assertLessEqual(jt.q_rad, 0.8 * 0.02)
                    self.assertLessEqual(jt.dq_rad_s, 2.0 * 0.02)

                    output = spec.mapping.to_output(jt)
                    frame = command(
                        spec.motor_id, mode=1, rpm=output.rpm, kd=output.kd,
                        torque=output.tau_nm, watchdog=True, position=output.q_rad, kp=output.kp,
                    )
                    decoded = decode_command(frame)
                    recovered = spec.mapping.from_output(
                        OutputSample(decoded["position_rad"], decoded["rpm"], decoded["torque_nm"])
                    )
                    self.assertEqual(recovered.q_rad, jt.q_rad)
                    self.assertEqual(recovered.dq_rad_s, jt.dq_rad_s)

    def test_nextafter_endpoints_and_strict_closed_corridor_without_outward_epsilon(self):
        spec = JointSpec(
            "J1", 0, JointMap(1, 1, 0.0), PositionLimits(-0.1, 0.1),
            max_velocity_rad_s=0.5, max_acceleration_rad_s2=1.0, max_measured_lead_rad=1000000.0,
            max_effort_nm=1e100,
        )
        core, station, clock = armed_core((spec,), (0.0,), max_step=0.05)
        clock.value = 0.01
        tgt = make_target(core, clock, 0, (JointTarget(0.1, 0.5, 0.0, 1.0, 0.1),))
        self.assertTrue(station.submit(tgt).accepted)
        ref = station.consume()
        self.assertIsNotNone(ref)
        self.assertLessEqual(ref.joints[0].q_rad, 0.00500000001)
        self.assertGreaterEqual(ref.joints[0].q_rad, -0.00500000001)
        self.assertLessEqual(ref.joints[0].dq_rad_s, 0.01000000001)
        self.assertGreaterEqual(ref.joints[0].dq_rad_s, -0.01000000001)

    def test_one_bootstrap_progress_waiver_vs_repeated_subgrid_ticks_and_progress(self):
        spec = JointSpec(
            "J1", 0, JointMap(1, 1, 0.0), PositionLimits(-1.0, 1.0),
            max_velocity_rad_s=0.01, max_acceleration_rad_s2=0.05, max_measured_lead_rad=1000000.0,
            max_effort_nm=1e100,
        )
        core, station, clock = armed_core((spec,), (0.0,), max_step=0.05)

        clock.value = 0.0001
        tgt0 = make_target(core, clock, 0, (JointTarget(0.1, 0.0, 0.0, 1.0, 0.1),))
        self.assertTrue(station.submit(tgt0).accepted)
        ref0 = station.consume()
        self.assertIsNotNone(ref0)
        self.assertEqual(ref0.joints[0].q_rad, 0.0)

        clock.value = 0.0002
        tgt1 = make_target(core, clock, 1, (JointTarget(0.1, 0.0, 0.0, 1.0, 0.1),))
        self.assertTrue(station.submit(tgt1).accepted)
        self.assertIsNone(station.consume())
        self.assertIs(core.snapshot().pending_target, tgt1)

        clock.value = 0.0003
        self.assertIsNone(station.consume())
        self.assertIs(core.snapshot().pending_target, tgt1)

        clock.value = 0.02
        ref_progress = station.consume()
        self.assertIsNotNone(ref_progress)
        self.assertGreater(ref_progress.joints[0].q_rad, 0.0)

    def test_fast_and_slow_two_joint_coupling_suppresses_complete_vector(self):
        spec_fast = JointSpec(
            "J1", 0, JointMap(1, 1, 0.0), PositionLimits(-1.0, 1.0),
            max_velocity_rad_s=1.0, max_acceleration_rad_s2=10.0, max_measured_lead_rad=1000000.0,
            max_effort_nm=1e100,
        )
        spec_slow = JointSpec(
            "J2", 1, JointMap(1, 1, 0.0), PositionLimits(-1.0, 1.0),
            max_velocity_rad_s=0.001, max_acceleration_rad_s2=0.01, max_measured_lead_rad=1000000.0,
            max_effort_nm=1e100,
        )
        core, station, clock = armed_core((spec_fast, spec_slow), (0.0, 0.0), max_step=0.05)

        tgt0 = make_target(
            core, clock, 0,
            (JointTarget(0.0, 0.0, 0.0, 1.0, 0.1), JointTarget(0.0, 0.0, 0.0, 1.0, 0.1)),
        )
        self.assertTrue(station.submit(tgt0).accepted)
        self.assertIsNotNone(station.consume())

        clock.value = 0.001
        tgt1 = make_target(
            core, clock, 1,
            (JointTarget(0.5, 0.0, 0.0, 1.0, 0.1), JointTarget(0.5, 0.0, 0.0, 1.0, 0.1)),
        )
        self.assertTrue(station.submit(tgt1).accepted)
        self.assertIsNone(station.consume())
        self.assertIs(core.snapshot().pending_target, tgt1)

        clock.value = 0.05
        ref = station.consume()
        self.assertIsNotNone(ref)
        self.assertGreater(ref.joints[0].q_rad, 0.0)
        self.assertGreater(ref.joints[1].q_rad, 0.0)

    def test_statically_settled_subgrid_gain_updates_allowed(self):
        spec = JointSpec(
            "J1", 0, JointMap(1, 1, 0.0), PositionLimits(-1.0, 1.0),
            max_velocity_rad_s=1.0, max_acceleration_rad_s2=2.0, max_measured_lead_rad=1000000.0,
            max_effort_nm=1e100,
        )
        core, station, clock = armed_core((spec,), (0.0,), max_step=0.05)

        tgt0 = make_target(core, clock, 0, (JointTarget(0.0, 0.0, 0.0, 1.0, 0.1),))
        self.assertTrue(station.submit(tgt0).accepted)
        self.assertIsNotNone(station.consume())

        clock.value = 0.01
        tgt1 = make_target(core, clock, 1, (JointTarget(0.0, 0.0, 0.0, 25.0, 2.0),))
        self.assertTrue(station.submit(tgt1).accepted)
        ref1 = station.consume()
        self.assertIsNotNone(ref1)
        self.assertEqual(ref1.joints[0].q_rad, 0.0)
        self.assertEqual(ref1.joints[0].kp, 25.0)
        self.assertEqual(ref1.joints[0].kd, 2.0)

    def test_off_grid_seed_zero_time_hold_preserves_rate_and_bounds_subsequent_ticks(self):
        spec = JointSpec(
            "seed", 0, JointMap(1, 1, 0.0), PositionLimits(-1.0, 1.0),
            max_velocity_rad_s=1.0, max_acceleration_rad_s2=1.0, max_measured_lead_rad=1000000.0,
            max_effort_nm=1e100,
        )
        q_seed = raw_to_position(1) / 2.0
        core, station, clock = armed_core((spec,), (q_seed,), max_step=0.05)

        clock.value = 0.1
        tgt0 = make_target(core, clock, 0, (JointTarget(q_seed, 0.0, 0.0, 1.0, 0.1),))
        self.assertTrue(station.submit(tgt0).accepted)
        first = station.consume()
        self.assertIsNotNone(first)
        self.assertNotEqual(first.joints[0].q_rad, q_seed)
        seed_rate = (first.joints[0].q_rad - q_seed) / first.issued_at_s
        self.assertNotEqual(seed_rate, 0.0)

        tgt1 = make_target(core, clock, 1, (JointTarget(q_seed, 0.0, 0.0, 2.0, 0.1),))
        self.assertTrue(station.submit(tgt1).accepted)
        same_time = station.consume()
        self.assertIsNotNone(same_time)
        self.assertEqual(same_time.issued_at_s, first.issued_at_s)
        self.assertEqual(same_time.joints[0].q_rad, first.joints[0].q_rad)

        clock.value = 0.10001
        tgt2 = make_target(core, clock, 2, (JointTarget(q_seed, 0.0, 0.0, 3.0, 0.1),))
        self.assertTrue(station.submit(tgt2).accepted)
        self.assertGreater(abs(seed_rate), clock.value - first.issued_at_s)
        self.assertIsNone(station.consume())
        snapshot = core.snapshot()
        self.assertIs(snapshot.pending_target, tgt2)
        self.assertEqual(snapshot.last_sequence, 2)

        clock.value = 0.101
        final = station.consume()
        self.assertIsNotNone(final)
        self.assertEqual(final.joints[0].q_rad, first.joints[0].q_rad)

    def test_queued_and_idle_time_capped_by_max_reference_step(self):
        spec = JointSpec(
            "J1", 0, JointMap(1, 1, 0.0), PositionLimits(-2.0, 2.0),
            max_velocity_rad_s=1.0, max_acceleration_rad_s2=2.0, max_measured_lead_rad=1000000.0,
            max_effort_nm=1e100,
        )
        core, station, clock = armed_core(
            (spec,), (0.0,), max_step=0.02, feedback_max_age=20.0,
        )
        delayed = make_target(
            core, clock, 0, (JointTarget(1.0, 1.0, 0.0, 1.0, 0.1),),
            created_at_s=0.0, expires_at_s=20.0,
        )
        clock.value = 5.0
        self.assertTrue(station.submit(delayed).accepted)
        clock.value = 5.5
        ref0 = station.consume()
        self.assertIsNotNone(ref0)
        self.assertLessEqual(ref0.joints[0].q_rad, 0.02000000001)
        self.assertLessEqual(ref0.joints[0].dq_rad_s, 0.04000000001)

        clock.value = 10.0
        idle_tgt = make_target(core, clock, 1, (JointTarget(1.0, 1.0, 0.0, 1.0, 0.1),), expires_at_s=30.0)
        self.assertTrue(station.submit(idle_tgt).accepted)
        ref1 = station.consume()
        self.assertIsNotNone(ref1)
        self.assertLessEqual(ref1.joints[0].q_rad - ref0.joints[0].q_rad, 0.02000000001)
        self.assertLessEqual(ref1.joints[0].dq_rad_s - ref0.joints[0].dq_rad_s, 0.04000000001)

    def test_valid_and_rejected_replacement_and_sequence_watermark(self):
        spec = JointSpec(
            "J1", 0, JointMap(1, 1, 0.0), PositionLimits(-1.0, 1.0),
            max_velocity_rad_s=1.0, max_acceleration_rad_s2=2.0, max_measured_lead_rad=1000000.0,
            max_effort_nm=1e100,
        )
        core, station, clock = armed_core((spec,), (0.0,))
        tgt0 = make_target(core, clock, 0, (JointTarget(0.5, 0.0, 0.0, 1.0, 0.1),))
        self.assertTrue(station.submit(tgt0).accepted)
        self.assertIs(core.snapshot().pending_target, tgt0)
        self.assertEqual(core.snapshot().last_sequence, 0)

        invalid_tgt = make_target(core, clock, 1, (JointTarget(2.0, 0.0, 0.0, 1.0, 0.1),))
        decision = station.submit(invalid_tgt)
        self.assertFalse(decision.accepted)
        self.assertIs(decision.code, DecisionCode.POSITION_LIMIT)
        self.assertIs(core.snapshot().pending_target, tgt0)
        self.assertEqual(core.snapshot().last_sequence, 0)

        tgt2 = make_target(core, clock, 2, (JointTarget(0.3, 0.0, 0.0, 1.0, 0.1),))
        self.assertTrue(station.submit(tgt2).accepted)
        self.assertIs(core.snapshot().pending_target, tgt2)
        self.assertEqual(core.snapshot().last_sequence, 2)

    def test_second_joint_projection_failure_preserves_pending_and_history_atomically(self):
        specs = (
            JointSpec("J1", 0, JointMap(1, 1, 0.0), PositionLimits(-1.0, 1.0), 1.0, 2.0, 1000000.0, max_effort_nm=1e100),
            JointSpec("J2", 1, JointMap(1, -1, 0.0), PositionLimits(-1.0, 1.0), 1.0, 2.0, 1000000.0, max_effort_nm=1e100),
        )
        core, station, clock = armed_core(specs, (0.0, 0.0))
        seed_tgt = make_target(
            core, clock, 0,
            (JointTarget(0.0, 0.0, 0.0, 1.0, 0.1), JointTarget(0.0, 0.0, 0.0, 1.0, 0.1)),
        )
        self.assertTrue(station.submit(seed_tgt).accepted)
        seed_ref = station.consume()
        self.assertIsNotNone(seed_ref)

        request = make_target(
            core, clock, 1,
            (JointTarget(0.5, 0.0, 0.0, 1.0, 0.1), JointTarget(-0.5, 0.0, 0.0, 1.0, 0.1)),
        )
        self.assertTrue(station.submit(request).accepted)
        clock.value = 0.01

        original = reference.project_position_raw

        def fail_second(*args, **kwargs):
            wanted_q = args[0] if args else kwargs.get("wanted_q")
            if wanted_q == -0.5:
                raise ValueError("injected second-joint projection failure")
            return original(*args, **kwargs)

        writes_before = len(station.writes)
        with patch.object(reference, "project_position_raw", side_effect=fail_second):
            self.assertIsNone(station.consume())

        snapshot = core.snapshot()
        self.assertEqual(len(station.writes), writes_before)
        self.assertIs(snapshot.pending_target, request)
        self.assertEqual(snapshot.last_sequence, 1)

        opposite = make_target(
            core, clock, 2,
            (JointTarget(-0.5, 0.0, 0.0, 1.0, 0.1), JointTarget(0.5, 0.0, 0.0, 1.0, 0.1)),
        )
        self.assertTrue(station.submit(opposite).accepted)
        clock.value = 0.02
        recovered = station.consume()
        self.assertIsNotNone(recovered)
        self.assertIs(recovered.requested, opposite)

    def test_final_delay_acceleration_rate_veto_and_retained_epoch_retry(self):
        spec = JointSpec(
            "J1", 0, JointMap(1, 1, 0.0), PositionLimits(-2.0, 2.0),
            max_velocity_rad_s=1.0, max_acceleration_rad_s2=1.0, max_measured_lead_rad=1000000.0,
            max_effort_nm=1e100,
        )
        core, station, clock = armed_core((spec,), (0.0,), max_step=0.5, feedback_max_age=1000.0)

        clock.value = 1.0
        tgt0 = make_target(core, clock, 0, (JointTarget(0.5, 0.0, 0.0, 1.0, 0.1),))
        self.assertTrue(station.submit(tgt0).accepted)
        ref0 = station.consume()
        self.assertIsNotNone(ref0)
        self.assertEqual(ref0.issued_at_s, 1.0)

        clock.value = 1.02
        tgt1 = make_target(core, clock, 1, (JointTarget(0.51, 0.0, 0.0, 1.0, 0.1),))
        self.assertTrue(station.submit(tgt1).accepted)

        original_project = reference.project_joint_target

        def delay_during_projection(*args, **kwargs):
            projected = original_project(*args, **kwargs)
            clock.value = 1.10
            return projected

        with patch.object(reference, "project_joint_target", side_effect=delay_during_projection):
            self.assertIsNone(station.consume())

        snapshot = core.snapshot()
        self.assertIs(snapshot.pending_target, tgt1)
        self.assertEqual(snapshot.last_sequence, 1)

        clock.value = 1.50
        ref1 = station.consume()
        self.assertIsNotNone(ref1)
        self.assertIs(ref1.requested, tgt1)
        self.assertGreater(ref1.joints[0].q_rad, ref0.joints[0].q_rad)

    def test_expiry_freshness_and_clock_regression_final_guards(self):
        spec = JointSpec(
            "J1", 0, JointMap(1, 1, 0.0), PositionLimits(-1.0, 1.0),
            max_velocity_rad_s=1.0, max_acceleration_rad_s2=2.0, max_measured_lead_rad=1000000.0,
            max_effort_nm=1e100,
        )
        core, station, clock = armed_core(
            (spec,), (0.0,), max_step=0.1, feedback_max_age=0.1,
        )
        clock.value = 0.05
        tgt = make_target(
            core, clock, 0, (JointTarget(0.5, 0.0, 0.0, 1.0, 0.1),),
            created_at_s=0.0, expires_at_s=0.10,
        )
        self.assertTrue(station.submit(tgt).accepted)

        original_project = reference.project_joint_target

        def expire_in_projection(*args, **kwargs):
            proj = original_project(*args, **kwargs)
            clock.value = 0.10
            return proj

        with patch.object(reference, "project_joint_target", side_effect=expire_in_projection):
            self.assertIsNone(station.consume())
        self.assertIsNone(core.snapshot().pending_target)
        self.assertEqual(core.snapshot().last_sequence, 0)

        clock.value = 0.12
        self.assertTrue(core.receive(0, JointSample(0.0, 0.0, 0.0), STOP).accepted)
        fresh_tgt = make_target(
            core, clock, 1, (JointTarget(0.5, 0.0, 0.0, 1.0, 0.1),),
            created_at_s=0.12, expires_at_s=1.0,
        )
        self.assertTrue(station.submit(fresh_tgt).accepted)

        def stale_in_projection(*args, **kwargs):
            proj = original_project(*args, **kwargs)
            clock.value = 0.30
            return proj

        with patch.object(reference, "project_joint_target", side_effect=stale_in_projection):
            self.assertIsNone(station.consume())
        snap_fault = core.snapshot()
        self.assertIs(snap_fault.state, State.FAULT)
        self.assertIsNone(snap_fault.ownership)

        core2, station2, clock2 = armed_core((spec,), (0.0,), max_step=0.1)
        clock2.value = 0.05
        tgt2 = make_target(core2, clock2, 0, (JointTarget(0.5, 0.0, 0.0, 1.0, 0.1),))
        self.assertTrue(station2.submit(tgt2).accepted)

        def regress_in_projection(*args, **kwargs):
            proj = original_project(*args, **kwargs)
            clock2.value = 0.02
            return proj

        with patch.object(reference, "project_joint_target", side_effect=regress_in_projection):
            with self.assertRaises(ValueError):
                station2.consume()
        clock2.value = 0.05
        self.assertIs(core2.snapshot().pending_target, tgt2)

    def test_stop_fault_disconnect_and_rearm_with_fresh_seed_and_old_owner_rejection(self):
        spec = JointSpec(
            "J1", 0, JointMap(1, 1, 0.0), PositionLimits(-1.0, 1.0),
            max_velocity_rad_s=1.0, max_acceleration_rad_s2=2.0, max_measured_lead_rad=1000000.0,
            max_effort_nm=1e100,
        )
        core, station, clock = armed_core((spec,), (0.0,))
        old_ownership = core.snapshot().ownership
        self.assertEqual(core.snapshot().ownership_generation, 1)

        self.assertTrue(station.stop().accepted)
        self.assertIs(core.snapshot().state, State.STOPPED)
        self.assertIsNone(core.snapshot().ownership)
        self.assertEqual(core.snapshot().ownership_generation, 2)

        tgt_old = make_target(
            core, clock, 0, (JointTarget(0.1, 0.0, 0.0, 1.0, 0.1),),
            ownership=old_ownership,
        )
        decision = station.submit(tgt_old)
        self.assertFalse(decision.accepted)

        self.assertTrue(station.check().accepted)
        grant = station.arm(owner="new_owner")
        self.assertTrue(grant.accepted)
        self.assertEqual(core.snapshot().ownership_generation, 3)

        decision_old = station.submit(tgt_old)
        self.assertFalse(decision_old.accepted)
        self.assertIs(decision_old.code, DecisionCode.INVALID_OWNERSHIP)

        tgt_new = make_target(core, clock, 1, (JointTarget(0.1, 0.0, 0.0, 1.0, 0.1),))
        self.assertTrue(station.submit(tgt_new).accepted)
        clock.value = 0.02
        ref = station.consume()
        self.assertIsNotNone(ref)

    def test_huge_finite_acceleration_overflow_and_tiny_cap_infeasibility(self):
        huge = JointSpec(
            "huge", 0, JointMap(1, 1, 0.0), PositionLimits(-1.0, 1.0),
            max_velocity_rad_s=1.0, max_acceleration_rad_s2=1e308, max_measured_lead_rad=1000000.0,
            max_effort_nm=1e100,
        )
        core, station, clock = armed_core((huge,), (0.0,), feedback_max_age=1000.0)
        clock.value = 10.0
        tgt = make_target(core, clock, 0, (JointTarget(0.2, 0.1, 0.0, 1.0, 0.1),))
        self.assertTrue(station.submit(tgt).accepted)
        self.assertIsNone(station.consume())
        self.assertIs(core.snapshot().pending_target, tgt)
        self.assertEqual(core.snapshot().last_sequence, 0)

        tiny = JointSpec(
            "tiny", 0, JointMap(1, 1, 0.0), PositionLimits(-1.0, 1.0),
            max_velocity_rad_s=1e-12, max_acceleration_rad_s2=1e-12, max_measured_lead_rad=1000000.0,
            max_effort_nm=1e100,
        )
        core2, station2, clock2 = armed_core((tiny,), (0.0,), max_step=1e-6)
        clock2.value = 0.01
        tgt2 = make_target(core2, clock2, 0, (JointTarget(0.1, 0.0, 0.0, 1.0, 0.1),))
        self.assertTrue(station2.submit(tgt2).accepted)
        bootstrap = station2.consume()
        self.assertIsNotNone(bootstrap)
        self.assertEqual(bootstrap.joints[0].q_rad, 0.0)
        tgt3 = make_target(core2, clock2, 1, (JointTarget(0.1, 0.0, 0.0, 1.0, 0.1),))
        self.assertTrue(station2.submit(tgt3).accepted)
        self.assertIsNone(station2.consume())
        self.assertIs(core2.snapshot().pending_target, tgt3)
        self.assertEqual(core2.snapshot().last_sequence, 1)


if __name__ == "__main__":
    unittest.main()
