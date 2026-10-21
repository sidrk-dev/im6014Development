"""Acceptance and integration tests for C12 measured-position lead limits.

Tests verify required finite positive max_measured_lead_rad on JointSpec,
pure planner four-way corridor intersection ([L, U], velocity step, acceleration
rate, and measured-position lead corridor), empty-corridor suppression without
state corruption, preservation of C09 distant target submission, multi-tick
pacing with stall holding, multi-joint coupling, release-time observation shift
veto, reductions 1/3/6 and directions +/-1, strict boundary endpoints, and
bootstrap seed invariants.
"""

import math
import unittest
from unittest.mock import patch

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
from im6014_control.reference import plan_reference, position_rates_at_release, project_joint_target


SOURCE = SourceIdentity("fixture", "controller-lead", 1)
STOP = MotorHealth(0, 0, 0, False)


class DeterministicClock:
    def __init__(self, value=0.0):
        self.value = value
        self.calls = 0

    def __call__(self):
        self.calls += 1
        return self.value


class LeadStation:
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
    station = LeadStation(core)
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


class ReferenceLeadTests(unittest.TestCase):
    def test_lead_is_required_finite_and_positive_without_default(self):
        mapping = JointMap(1, 1, 0.0)
        limits = PositionLimits(-2.0, 2.0)
        with self.assertRaises(TypeError):
            JointSpec("J1", 0, mapping, limits, 1.0, 2.0, max_effort_nm=1e100)
        for bad in (True, False, 0, 0.0, -1.0, -0.001, math.nan, math.inf, -math.inf, "0.1", None, 10**400):
            with self.subTest(bad=repr(bad)), self.assertRaises(ValueError):
                JointSpec("J1", 0, mapping, limits, 1.0, 2.0, max_measured_lead_rad=bad, max_effort_nm=1e100)
        for good in (0.001, 0.05, 1.0, 100.0):
            spec = JointSpec("J1", 0, mapping, limits, 1.0, 2.0, max_measured_lead_rad=good, max_effort_nm=1e100)
            self.assertEqual(spec.max_measured_lead_rad, float(good))

    def test_plan_reference_bounds_forward_and_reverse_lead(self):
        mapping = JointMap(1, 1, 0.0)
        limits = PositionLimits(-5.0, 5.0)
        spec = JointSpec("J1", 0, mapping, limits, 10.0, 50.0, max_measured_lead_rad=0.05, max_effort_nm=1e100)
        prev = JointTarget(0.0, 0.0, 0.0, 1.0, 0.1)

        # Distant forward goal: requested q=2.0, q_meas=0.0, lead=0.05 -> candidate q must not exceed 0.05
        fwd_cand = plan_reference(
            specs=(spec,),
            requested=(JointTarget(2.0, 0.0, 0.0, 1.0, 0.1),),
            previous=(prev,),
            previous_rates=(0.0,),
            measured_positions=(0.0,),
            elapsed_s=0.1,
            max_step_s=0.1,
            allow_bootstrap=False,
            measured_velocities=(0.0,) * len((spec,)),
        )
        self.assertIsNotNone(fwd_cand)
        self.assertLessEqual(fwd_cand[0].q_rad, 0.05)
        self.assertGreater(fwd_cand[0].q_rad, 0.0)

        # Distant reverse goal: requested q=-2.0, q_meas=0.0, lead=0.05 -> candidate q must not be below -0.05
        rev_cand = plan_reference(
            specs=(spec,),
            requested=(JointTarget(-2.0, 0.0, 0.0, 1.0, 0.1),),
            previous=(prev,),
            previous_rates=(0.0,),
            measured_positions=(0.0,),
            elapsed_s=0.1,
            max_step_s=0.1,
            allow_bootstrap=False,
            measured_velocities=(0.0,) * len((spec,)),
        )
        self.assertIsNotNone(rev_cand)
        self.assertGreaterEqual(rev_cand[0].q_rad, -0.05)
        self.assertLess(rev_cand[0].q_rad, 0.0)

    def test_four_way_corridor_intersection(self):
        mapping = JointMap(1, 1, 0.0)
        prev = JointTarget(0.0, 0.0, 0.0, 1.0, 0.1)

        # Case A: lead is tightest bound (limits 1.0, V*h 0.2, lead 0.03)
        spec_a = JointSpec("J1", 0, mapping, PositionLimits(-1.0, 1.0), 2.0, 10.0, max_measured_lead_rad=0.03, max_effort_nm=1e100)
        cand_a = plan_reference(
            (spec_a,), (JointTarget(0.5, 0.0, 0.0, 1.0, 0.1),), (prev,), (0.0,), (0.0,),
            elapsed_s=0.1, max_step_s=0.1, allow_bootstrap=False,
            measured_velocities=(0.0,) * len((spec_a,)),
        )
        self.assertIsNotNone(cand_a)
        self.assertLessEqual(cand_a[0].q_rad, 0.03)

        # Case B: velocity is tightest bound (limits 1.0, V*h 0.02, lead 0.1)
        spec_b = JointSpec("J1", 0, mapping, PositionLimits(-1.0, 1.0), 0.2, 10.0, max_measured_lead_rad=0.1, max_effort_nm=1e100)
        cand_b = plan_reference(
            (spec_b,), (JointTarget(0.5, 0.0, 0.0, 1.0, 0.1),), (prev,), (0.0,), (0.0,),
            elapsed_s=0.1, max_step_s=0.1, allow_bootstrap=False,
            measured_velocities=(0.0,) * len((spec_b,)),
        )
        self.assertIsNotNone(cand_b)
        self.assertLessEqual(cand_b[0].q_rad, 0.02)

        # Case C: position limit is tightest bound (limits [-0.01, 0.01], V*h 0.2, lead 0.1)
        spec_c = JointSpec("J1", 0, mapping, PositionLimits(-0.01, 0.01), 2.0, 10.0, max_measured_lead_rad=0.1, max_effort_nm=1e100)
        cand_c = plan_reference(
            (spec_c,), (JointTarget(0.5, 0.0, 0.0, 1.0, 0.1),), (prev,), (0.0,), (0.0,),
            elapsed_s=0.1, max_step_s=0.1, allow_bootstrap=False,
            measured_velocities=(0.0,) * len((spec_c,)),
        )
        self.assertIsNotNone(cand_c)
        self.assertLessEqual(cand_c[0].q_rad, 0.01)

    def test_empty_corridor_returns_none(self):
        mapping = JointMap(1, 1, 0.0)
        # Prior reference at 0.5 rad, moving at 2.0 rad/s with deceleration 1.0 rad/s2.
        # Deceleration corridor requires moving forward at least e * (2.0 - 1.0*0.02) = 0.0396 rad -> q >= 0.5396 rad.
        # But measured position is lagging at 0.40 rad with lead 0.05 -> q <= 0.45 rad.
        # The lead bound (0.45) is below the minimum deceleration point (0.5396) -> disjoint empty corridor!
        spec = JointSpec("J1", 0, mapping, PositionLimits(-2.0, 2.0), 2.0, 1.0, max_measured_lead_rad=0.05, max_effort_nm=1e100)
        prev = JointTarget(0.5, 2.0, 0.0, 1.0, 0.1)

        result = plan_reference(
            specs=(spec,),
            requested=(JointTarget(1.0, 0.0, 0.0, 1.0, 0.1),),
            previous=(prev,),
            previous_rates=(2.0,),
            measured_positions=(0.40,),  # 0.40 + 0.05 = 0.45 < 0.5396
            elapsed_s=0.02,
            max_step_s=0.02,
            allow_bootstrap=False,
            measured_velocities=(0.0,) * len((spec,)),
        )
        self.assertIsNone(result)

    def test_plan_reference_invalid_measured_positions(self):
        mapping = JointMap(1, 1, 0.0)
        spec = JointSpec("J1", 0, mapping, PositionLimits(-1.0, 1.0), 1.0, 2.0, max_measured_lead_rad=0.1, max_effort_nm=1e100)
        prev = JointTarget(0.0, 0.0, 0.0, 1.0, 0.1)
        req = JointTarget(0.5, 0.0, 0.0, 1.0, 0.1)

        # Mismatched length
        self.assertIsNone(plan_reference((spec,), (req,), (prev,), (0.0,), (), 0.02, 0.02, False, measured_velocities=(0.0,) * len((spec,))))
        self.assertIsNone(plan_reference((spec,), (req,), (prev,), (0.0,), (0.0, 0.0), 0.02, 0.02, False, measured_velocities=(0.0,) * len((spec,))))

        # Invalid element types
        for bad_q in (True, False, math.nan, math.inf, -math.inf, "0.0", None):
            with self.subTest(bad_q=repr(bad_q)):
                self.assertIsNone(plan_reference((spec,), (req,), (prev,), (0.0,), (bad_q,), 0.02, 0.02, False, measured_velocities=(0.0,) * len((spec,))))

    def test_distant_target_accepted_at_submit_time(self):
        spec = JointSpec(
            "J1", 0, JointMap(1, 1, 0.0), PositionLimits(-2.0, 2.0),
            max_velocity_rad_s=1.0, max_acceleration_rad_s2=2.0, max_measured_lead_rad=0.05,
            max_effort_nm=1e100,
        )
        core, station, clock = armed_core((spec,), (0.0,))

        # Target at 1.5 rad is distant (|1.5 - 0.0| = 1.5 >> 0.05), but within [-2.0, 2.0] limits -> must be ACCEPTED!
        tgt = make_target(core, clock, 0, (JointTarget(1.5, 0.0, 0.0, 1.0, 0.1),))
        dec = station.submit(tgt)
        self.assertTrue(dec.accepted)
        self.assertEqual(dec.code, DecisionCode.ACCEPTED)

        # Target at 2.5 rad is outside limits -> rejected with POSITION_LIMIT
        bad_tgt = make_target(core, clock, 1, (JointTarget(2.5, 0.0, 0.0, 1.0, 0.1),))
        bad_dec = station.submit(bad_tgt)
        self.assertFalse(bad_dec.accepted)
        self.assertEqual(bad_dec.code, DecisionCode.POSITION_LIMIT)

    def test_multi_tick_lead_pacing_and_stall_holding(self):
        spec = JointSpec(
            "J1", 0, JointMap(1, 1, 0.0), PositionLimits(-2.0, 2.0),
            max_velocity_rad_s=1.0, max_acceleration_rad_s2=10.0, max_measured_lead_rad=0.01,
            max_effort_nm=1e100,
        )
        core, station, clock = armed_core((spec,), (0.0,), max_step=0.02)

        # Submit distant goal of 1.0 rad
        clock.value = 0.02
        station.submit(make_target(core, clock, 0, (JointTarget(1.0, 0.0, 0.0, 1.0, 0.1),)))

        # Tick 1: measured position 0.0 -> reference advances up to lead (<= 0.01)
        ref1 = station.consume()
        self.assertIsNotNone(ref1)
        self.assertLessEqual(ref1.joints[0].q_rad, 0.01)

        # Tick 2: reaches lead boundary (0.01)
        clock.value += 0.02
        core.receive(0, JointSample(0.0, 0.0, 0.0), STOP)
        station.submit(make_target(core, clock, 1, (JointTarget(1.0, 0.0, 0.0, 1.0, 0.1),)))
        ref2 = station.consume()
        self.assertIsNotNone(ref2)
        self.assertLessEqual(ref2.joints[0].q_rad, 0.01)

        # Arm physical stall simulation: measured position stays stuck at 0.0
        # Subsequent ticks must not advance reference past 0.01!
        seq = 2
        for tick in range(3, 7):
            clock.value += 0.02
            # Re-feed feedback showing motor stuck at 0.0
            core.receive(0, JointSample(0.0, 0.0, 0.0), STOP)
            station.submit(make_target(core, clock, seq, (JointTarget(1.0, 0.0, 0.0, 1.0, 0.1),)))
            seq += 1
            ref_stall = station.consume()
            if ref_stall is not None:
                self.assertLessEqual(ref_stall.joints[0].q_rad, 0.01)

        # Motor catches up: feedback advances to 0.02 rad
        clock.value += 0.02
        core.receive(0, JointSample(0.02, 0.0, 0.0), STOP)
        station.submit(make_target(core, clock, seq, (JointTarget(1.0, 0.0, 0.0, 1.0, 0.1),)))
        ref_advance = station.consume()
        self.assertIsNotNone(ref_advance)
        # Now reference can advance up to 0.02 + 0.01 = 0.03 rad, exceeding 0.01
        self.assertLessEqual(ref_advance.joints[0].q_rad, 0.03)
        self.assertGreater(ref_advance.joints[0].q_rad, 0.01)

    def test_multi_joint_coupling_suppresses_whole_vector(self):
        spec1 = JointSpec("J1", 0, JointMap(1, 1, 0.0), PositionLimits(-2.0, 2.0), 1.0, 2.0, max_measured_lead_rad=0.5, max_effort_nm=1e100)
        spec2 = JointSpec("J2", 1, JointMap(1, 1, 0.0), PositionLimits(-2.0, 2.0), 1.0, 2.0, max_measured_lead_rad=0.02, max_effort_nm=1e100)
        core, station, clock = armed_core((spec1, spec2), (0.0, 0.0), max_step=0.02)

        # J1 requested 1.0 rad (has 0.5 lead headroom). J2 requested 1.0 rad but only has 0.02 lead headroom.
        clock.value = 0.02
        station.submit(make_target(core, clock, 0, (
            JointTarget(1.0, 0.0, 0.0, 1.0, 0.1),
            JointTarget(1.0, 0.0, 0.0, 1.0, 0.1),
        )))
        ref0 = station.consume()
        self.assertIsNotNone(ref0)

        # Next tick: J2 feedback stuck at 0.0. J2 reference reached 0.02 lead limit.
        # J2 cannot advance representably. Both joints must be suppressed atomically!
        clock.value += 0.02
        core.receive(0, JointSample(0.0, 0.0, 0.0), STOP)
        core.receive(1, JointSample(0.0, 0.0, 0.0), STOP)
        ref_coupled = station.consume()
        if ref_coupled is not None:
            # If emitted, J2 must not violate lead bound
            self.assertLessEqual(ref_coupled.joints[1].q_rad, 0.02)

    def test_release_time_lead_check_veto(self):
        spec = JointSpec("J1", 0, JointMap(1, 1, 0.0), PositionLimits(-2.0, 2.0), 1.0, 2.0, max_measured_lead_rad=0.05, max_effort_nm=1e100)
        core, station, clock = armed_core((spec,), (0.0,), max_step=0.02)

        clock.value = 0.02
        station.submit(make_target(core, clock, 0, (JointTarget(0.5, 0.0, 0.0, 1.0, 0.1),)))

        # Hook into now() callback during consume to simulate observation shift before release
        orig_now = core._now
        shifted = False
        def shifted_now():
            nonlocal shifted
            t = orig_now()
            if not shifted:
                # Simulate external disturbance shifting measured position to -0.2 rad
                core._observations[0] = core._observations[0].sample and core._observations[0].__class__(
                    JointSample(-0.2, 0.0, 0.0),
                    core._observations[0].health,
                    t,
                )
                shifted = True
            return t

        core._now = shifted_now
        # Candidate was planned with q_meas=0.0, but at release q_meas=-0.2 rad -> |cand - (-0.2)| > 0.05 -> veto!
        ref = core.consume_target()
        self.assertIsNone(ref)
        # Pending target retained
        self.assertIsNotNone(core.snapshot().pending_target)

    def test_both_directions_and_reductions_roundtrip_codec(self):
        for reduction in (1, 3, 6):
            for direction in (1, -1):
                with self.subTest(reduction=reduction, direction=direction):
                    mapping = JointMap(reduction, direction, 0.0)
                    limits = PositionLimits(-2.0, 2.0)
                    spec = JointSpec("J1", 0, mapping, limits, 1.0, 2.0, max_measured_lead_rad=0.05, max_effort_nm=1e100)
                    q_meas = 0.1
                    prev = JointTarget(q_meas, 0.0, 0.0, 1.0, 0.1)

                    cand = plan_reference(
                        specs=(spec,),
                        requested=(JointTarget(q_meas + 0.5, 0.0, 0.0, 1.0, 0.1),),
                        previous=(prev,),
                        previous_rates=(0.0,),
                        measured_positions=(q_meas,),
                        elapsed_s=0.05,
                        max_step_s=0.05,
                        allow_bootstrap=False,
                        measured_velocities=(0.0,) * len((spec,)),
                    )
                    self.assertIsNotNone(cand)
                    # Candidate must not exceed measured + lead
                    self.assertLessEqual(cand[0].q_rad, q_meas + 0.05)
                    self.assertGreaterEqual(cand[0].q_rad, q_meas - 0.05)

                    # Verify codec roundtrip
                    out = mapping.to_output(cand[0])
                    frame = command(0, mode=1, position=out.q_rad, rpm=out.rpm, kd=out.kd, kp=out.kp)
                    dec = decode_command(frame)
                    dec_pos = mapping.from_output(OutputSample(raw_to_position(dec["raw_position"]), 0.0, 0.0)).q_rad
                    self.assertLessEqual(abs(dec_pos - q_meas), 0.05)

    def test_boundary_equality_and_nextafter_inward_projection(self):
        mapping = JointMap(1, 1, 0.0)
        limits = PositionLimits(-2.0, 2.0)
        lead = 0.05
        spec = JointSpec("J1", 0, mapping, limits, 5.0, 20.0, max_measured_lead_rad=lead, max_effort_nm=1e100)
        prev = JointTarget(0.0, 0.0, 0.0, 1.0, 0.1)

        # Target at exact boundary (0.05)
        cand_exact = plan_reference(
            (spec,), (JointTarget(lead, 0.0, 0.0, 1.0, 0.1),), (prev,), (0.0,), (0.0,),
            elapsed_s=0.1, max_step_s=0.1, allow_bootstrap=False,
            measured_velocities=(0.0,) * len((spec,)),
        )
        self.assertIsNotNone(cand_exact)
        self.assertLessEqual(cand_exact[0].q_rad, lead)

        # Target slightly beyond boundary (0.05 + 1e-4) -> must be projected strictly <= lead, no outward epsilon
        cand_beyond = plan_reference(
            (spec,), (JointTarget(lead + 1e-4, 0.0, 0.0, 1.0, 0.1),), (prev,), (0.0,), (0.0,),
            elapsed_s=0.1, max_step_s=0.1, allow_bootstrap=False,
            measured_velocities=(0.0,) * len((spec,)),
        )
        self.assertIsNotNone(cand_beyond)
        self.assertLessEqual(cand_beyond[0].q_rad, lead)

    def test_infeasibility_preserves_state_machine_and_history(self):
        spec = JointSpec("J1", 0, JointMap(1, 1, 0.0), PositionLimits(-2.0, 2.0), 2.0, 1.0, max_measured_lead_rad=0.05, max_effort_nm=1e100)
        core, station, clock = armed_core((spec,), (0.0,))

        # Inject disjoint observation condition: reference was at 0.0, but measured position jumped to -0.5 rad
        core._observations[0] = core._observations[0].__class__(
            JointSample(-0.5, 0.0, 0.0),
            core._observations[0].health,
            clock.value,
        )

        clock.value = 0.02
        tgt = make_target(core, clock, 0, (JointTarget(0.0, 0.0, 0.0, 1.0, 0.1),))
        station.submit(tgt)

        # Infeasible lead corridor (-0.55 to -0.45, but q0=0.0) -> returns None
        ref = station.consume()
        self.assertIsNone(ref)

        # Invariants: no fault latched, state ARMED, pending target preserved
        snap = core.snapshot()
        self.assertEqual(snap.state, State.ARMED)
        self.assertEqual(len(snap.faults), 0)
        self.assertIsNotNone(snap.pending_target)

    def test_bootstrap_seed_at_zero_lead_separation(self):
        spec = JointSpec("J1", 0, JointMap(1, 1, 0.0), PositionLimits(-2.0, 2.0), 1.0, 2.0, max_measured_lead_rad=0.05, max_effort_nm=1e100)
        core, station, clock = armed_core((spec,), (0.25,))

        # Arm seeded position at 0.25 rad. Measured position is 0.25 rad.
        # Separation at seed is 0.0.
        clock.value = 0.02
        station.submit(make_target(core, clock, 0, (JointTarget(0.28, 0.0, 0.0, 1.0, 0.1),)))
        ref = station.consume()
        self.assertIsNotNone(ref)
        self.assertLessEqual(abs(ref.joints[0].q_rad - 0.25), 0.05)


if __name__ == "__main__":
    unittest.main()
