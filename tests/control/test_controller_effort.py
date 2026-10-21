from dataclasses import replace
import math
import unittest
from unittest.mock import patch

from im6014_control import core as core_module
from im6014_control.codec import command, decode_command
from im6014_control.core import JointCore
from im6014_control.mapping import JointMap, JointSample, JointTarget
from im6014_control.records import (
    DecisionCode, JointFault, JointSpec, MotorHealth, PositionLimits,
    SourceIdentity, State, Target,
)
from im6014_control.reference import _applied_reference, _predicted_effort, plan_reference
from tests.control.test_controller import DeterministicClock, FakeStation


STOP = MotorHealth(0, 0, 0, False)


def spec(name="J1", motor_id=0, cap=1.0, mapping=None, lead=2.0):
    return JointSpec(
        name, motor_id, mapping or JointMap(1.0, 1, 0.0),
        PositionLimits(-2.0, 2.0), 10.0, 1000.0, lead, max_effort_nm=cap,
    )


def armed(specs=None, *, position=0.0, velocity=0.0, feedback_age=10.0, max_step=0.1):
    specs = (spec(),) if specs is None else specs
    clock = DeterministicClock(0.0)
    core = JointCore(specs, clock=clock, feedback_max_age_s=feedback_age,
                     max_reference_step_s=max_step)
    assert core.begin_connect().accepted
    assert core.connected(SourceIdentity("fixture", "effort", 1)).accepted
    assert core.begin_check().accepted
    for joint in specs:
        assert core.receive(joint.motor_id, JointSample(position, velocity, 0.0), STOP).accepted
    assert core.checked().accepted
    grant = core.arm(owner="test")
    assert grant.accepted
    station = FakeStation(core)
    station.ownership = grant.ownership
    return core, station, clock


def target(core, joints, sequence=0, expiry=10.0):
    return Target(sequence, tuple(joints), 0.0, expiry, core.snapshot().ownership)


def submit(core, joints, *, sequence=0, expiry=10.0):
    proposed = target(core, joints, sequence, expiry)
    assert core.submit_target(proposed).accepted
    return proposed


class EffortTests(unittest.TestCase):
    def test_effort_limit_is_required_finite_positive_nonboolean(self):
        args = ("J1", 0, JointMap(1, 1, 0), PositionLimits(-1, 1), 1, 2, 0.1)
        with self.assertRaises(TypeError):
            JointSpec(*args)
        for bad in (True, False, 0, -1, math.nan, math.inf, -math.inf, None, "1", 10**1000):
            with self.subTest(bad=repr(bad)), self.assertRaises(ValueError):
                JointSpec(*args, max_effort_nm=bad)
        self.assertEqual(JointSpec(*args, max_effort_nm=2).max_effort_nm, 2.0)
        for bad in (1, None, "yes"):
            with self.subTest(flag=bad), self.assertRaises(ValueError):
                JointFault("J1", 0, effort_limit_exceeded=bad)

    def test_planner_requires_explicit_finite_measured_velocities(self):
        joint = spec()
        hold = JointTarget(0, 0, 0, 1, 1)
        args = ((joint,), (hold,), (hold,), (0.0,), (0.0,), 0.1, 0.1, True)
        with self.assertRaises(TypeError):
            plan_reference(*args)
        for bad in (None, (), (0.0, 0.0), (True,), (math.nan,), (math.inf,), (10**1000,), ("0",)):
            with self.subTest(velocity=repr(bad)):
                self.assertIsNone(plan_reference(*args, measured_velocities=bad))
        bad_positions = (*args[:4], (10**1000,), *args[5:])
        self.assertIsNone(plan_reference(*bad_positions, measured_velocities=(0.0,)))

    def test_applied_prediction_matches_independent_wire_formula(self):
        for reduction in (1, 3, 6):
            for direction in (1, -1):
                for zero in (0.0, 0.37):
                    with self.subTest(reduction=reduction, direction=direction, zero=zero):
                        mapping = JointMap(reduction, direction, zero)
                        joint = spec(cap=1000.0, mapping=mapping)
                        requested = JointTarget(0.4139, -0.2373, 0.1279 * reduction,
                                                2.137 * reduction**2, 0.235 * reduction**2)
                        core, station, clock = armed((joint,), position=0.2, velocity=0.17)
                        submit(core, (requested,))
                        clock.value = 0.1
                        issued = station.consume()
                        self.assertIsNotNone(issued)
                        decoded = decode_command(station.writes[-1])
                        q_eff = direction * (decoded["position_rad"] - zero) / reduction
                        dq_eff = direction * decoded["rpm"] * math.tau / 60 / reduction
                        ff_eff = direction * reduction * decoded["torque_nm"]
                        expected = (decoded["kp"] * reduction**2 * (q_eff - 0.2)
                                    + decoded["kd"] * reduction**2 * (dq_eff - 0.17) + ff_eff)
                        applied = _applied_reference(joint, issued.joints[0])
                        self.assertAlmostEqual(_predicted_effort(applied, 0.2, 0.17), expected, places=11)
                        self.assertGreater(abs(expected), abs(requested.tau_nm))
                        tight = replace(joint, max_effort_nm=abs(expected) * 0.9)
                        rejected, writer, timer = armed((tight,), position=0.2, velocity=0.17)
                        pending = submit(rejected, (requested,))
                        timer.value = 0.1
                        self.assertIsNone(writer.consume())
                        self.assertIs(rejected.snapshot().pending_target, pending)
                        self.assertEqual(writer.writes, [])

    def test_kd_cap_quantization_is_applied_without_reencoding_decoded_gain(self):
        requested = JointTarget(0, 0, 0, 0, 6)
        decoded = decode_command(command(0, mode=1, kd=6))
        self.assertGreater(decoded["kd"], 6.0005)
        for cap, allowed in ((6.0005, False), (7.0, True)):
            with self.subTest(cap=cap):
                core, station, clock = armed((spec(cap=cap),), velocity=1.0)
                submit(core, (requested,))
                clock.value = 0.1
                issued = station.consume()
                self.assertEqual(issued is not None, allowed)
                if allowed:
                    self.assertEqual(issued.joints[0].kd, 6.0)
                    self.assertEqual(decode_command(station.writes[-1])["kd"], decoded["kd"])

    def test_requested_and_decoded_feedforward_reject_without_poisoning_pending(self):
        for torque, cap in ((1.01, 1.0), (0.009, 0.009), (-0.009, 0.009)):
            with self.subTest(torque=torque):
                core, station, _ = armed((spec(cap=cap),))
                prior = submit(core, (JointTarget(0, 0, 0, 0, 0),), sequence=3)
                decision = core.submit_target(target(core, (JointTarget(0, 0, torque, 0, 0),), 4))
                self.assertEqual(decision.code, DecisionCode.EFFORT_LIMIT)
                self.assertIs(core.snapshot().pending_target, prior)
                self.assertEqual(core.snapshot().last_sequence, 3)
                self.assertEqual(station.writes, [])

    def test_distant_pd_macro_is_accepted_when_existing_pacing_makes_it_safe(self):
        core, station, clock = armed((replace(spec(), max_velocity_rad_s=1.0),), max_step=0.01)
        pending = submit(core, (JointTarget(1.5, 0, 0, 30, 0),))
        clock.value = 0.01
        issued = station.consume()
        self.assertIsNotNone(issued)
        self.assertIs(issued.requested, pending)
        self.assertLess(issued.joints[0].q_rad, 0.011)

    def test_combined_pd_feedforward_suppression_preserves_pending_and_history(self):
        core, station, clock = armed()
        pending = submit(core, (JointTarget(0.1, 0, 0.3, 8, 0),))
        history = core._history
        clock.value = 0.1
        self.assertIsNone(station.consume())
        self.assertIs(core._history, history)
        self.assertIs(core.snapshot().pending_target, pending)
        self.assertEqual(station.writes, [])

    def test_nonzero_measured_velocity_and_signed_damping_cancellation(self):
        for velocity, allowed in ((1.0, True), (-1.0, False)):
            with self.subTest(velocity=velocity):
                core, station, clock = armed((spec(cap=1.1),), velocity=velocity)
                submit(core, (JointTarget(0, 0, 1.0, 0, 1.0),))
                clock.value = 0.1
                self.assertEqual(station.consume() is not None, allowed)

    def test_zero_stiffness_valid_feedforward_can_issue(self):
        core, station, clock = armed()
        submit(core, (JointTarget(0.5, 0, 0.25, 0, 0),))
        clock.value = 0.1
        self.assertIsNotNone(station.consume())

    def test_strict_effort_threshold_and_adjacent_floats(self):
        kp = decode_command(command(0, mode=1, kp=1))["kp"]
        boundary = kp * 0.25
        for cap, allowed in ((math.nextafter(boundary, 0), False), (boundary, True),
                             (math.nextafter(boundary, math.inf), True)):
            with self.subTest(cap=cap):
                core, station, clock = armed((spec(cap=cap),), position=-0.25)
                submit(core, (JointTarget(0, 0, 0, 1, 0),))
                clock.value = 0.1
                self.assertEqual(station.consume() is not None, allowed)

    def test_prediction_overflow_suppresses_without_consuming(self):
        core, station, clock = armed((spec(cap=1e308, lead=1e308),))
        pending = submit(core, (JointTarget(0, 0, 0, 30, 0),))
        core.receive(0, JointSample(-1e307, 0, 0), STOP)
        clock.value = 0.1
        self.assertIsNone(station.consume())
        self.assertIs(core.snapshot().pending_target, pending)
        self.assertEqual(station.writes, [])

    def test_one_unsafe_joint_suppresses_whole_vector_and_can_recover(self):
        core, station, clock = armed((spec(), spec("J2", 1)))
        pending = submit(core, (JointTarget(0.02, 0, 0, 10, 0), JointTarget(0.2, 0, 0, 10, 0)))
        history = core._history
        clock.value = 0.1
        self.assertIsNone(station.consume())
        self.assertIs(core._history, history)
        self.assertIs(core.snapshot().pending_target, pending)
        self.assertEqual(station.writes, [])
        core.receive(1, JointSample(0.15, 0, 0), STOP)
        clock.value = 0.2
        self.assertIsNotNone(station.consume())
        self.assertEqual(len(station.writes), 2)

    def test_measured_limit_boundary_is_strict_in_both_directions(self):
        for sign in (-1, 1):
            with self.subTest(sign=sign):
                core, _, _ = armed()
                core.receive(0, JointSample(0, 0, sign), STOP)
                self.assertFalse(core.snapshot().fault_latched)
                core.receive(0, JointSample(0, 0, sign * math.nextafter(1.0, math.inf)), STOP)
                self.assertTrue(core.snapshot().faults[0].effort_limit_exceeded)

    def test_measured_fault_is_attributed_merged_and_persistent_until_explicit_recovery(self):
        core, station, clock = armed((spec(), spec("J2", 1)))
        old_owner = core.snapshot().ownership
        submit(core, (JointTarget(0, 0, 0, 0, 0),) * 2)
        core.receive(1, JointSample(0, 0, 1.01), MotorHealth(0, 2, 1, False))
        fault = core.snapshot().faults[0]
        self.assertEqual((fault.name, fault.motor_id, fault.errors, fault.warnings), ("J2", 1, 2, 1))
        self.assertTrue(fault.effort_limit_exceeded)
        self.assertIsNone(core._history)
        self.assertIsNone(core.snapshot().pending_target)
        self.assertIsNone(core.snapshot().ownership)
        core.receive(1, JointSample(0, 0, 0), STOP)
        self.assertIs(core.snapshot().state, State.FAULT)
        station.stop()
        core.disconnect()
        self.assertTrue(core.snapshot().faults[0].effort_limit_exceeded)
        core.begin_connect()
        core.connected(SourceIdentity("fixture", "effort", 2))
        self.assertEqual(core.begin_check().code, DecisionCode.FAULT_ACK_REQUIRED)
        self.assertTrue(core.begin_check(acknowledge_fault=True))
        self.assertEqual(core.checked().code, DecisionCode.OBSERVATIONS_INCOMPLETE)
        for joint in core.joint_specs:
            core.receive(joint.motor_id, JointSample(0, 0, 0), MotorHealth(1, 0, 0, False))
        self.assertEqual(core.checked().code, DecisionCode.NOT_STOPPED)
        for joint in core.joint_specs:
            core.receive(joint.motor_id, JointSample(0, 0, 0), STOP)
        clock.value = 11
        self.assertEqual(core.checked().code, DecisionCode.STALE_FEEDBACK)
        for joint in core.joint_specs:
            core.receive(joint.motor_id, JointSample(0, 0, 0), STOP)
        self.assertTrue(core.checked())
        self.assertFalse(core.snapshot().fault_latched)
        self.assertIsNone(core.snapshot().ownership)
        grant = core.arm(owner="test")
        self.assertTrue(grant)
        self.assertIsNot(grant.ownership, old_owner)
        stale = Target(5, (JointTarget(0, 0, 0, 0, 0),) * 2, 11, 20, old_owner)
        self.assertEqual(core.submit_target(stale).code, DecisionCode.INVALID_OWNERSHIP)

    def test_over_limit_feedback_during_recovery_relatches(self):
        core, _, _ = armed()
        core.receive(0, JointSample(0, 0, 2), STOP)
        self.assertTrue(core.begin_check(acknowledge_fault=True))
        core.receive(0, JointSample(0, 0, 2), STOP)
        self.assertIs(core.snapshot().state, State.FAULT)
        self.assertEqual(core.checked().code, DecisionCode.INVALID_STATE)

    def _release_mutation(self, action, *, expiry=10, feedback_age=10):
        core, station, clock = armed(feedback_age=feedback_age)
        pending = submit(core, (JointTarget(0.02, 0, 0, 1, 0),), expiry=expiry)
        history = core._history
        clock.value = 0.1
        original = core_module._applied_reference
        fired = False

        def mutate_after_encoding(*args):
            nonlocal fired
            applied = original(*args)
            if not fired:
                fired = True
                action(core, clock)
            return applied

        with patch.object(core_module, "_applied_reference", side_effect=mutate_after_encoding):
            self.assertIsNone(station.consume())
        self.assertTrue(fired)
        self.assertEqual(station.writes, [])
        return core, pending, history

    def test_release_encoding_delay_expiry_and_staleness(self):
        core, _, history = self._release_mutation(lambda core, clock: setattr(clock, "value", 0.2), expiry=0.2)
        self.assertIsNone(core.snapshot().pending_target)
        self.assertIs(core._history, history)
        core, _, _ = self._release_mutation(lambda core, clock: setattr(clock, "value", 0.3), feedback_age=0.2)
        self.assertIs(core.snapshot().state, State.FAULT)
        self.assertTrue(core.snapshot().faults[0].stale)
        self.assertIsNone(core.snapshot().ownership)

    def test_release_encoding_feedback_shift_veto_and_retry(self):
        def shift(core, clock):
            core.receive(0, JointSample(0, 0.5, 0), STOP)
        core, pending, history = self._release_mutation(shift)
        self.assertIs(core.snapshot().pending_target, pending)
        self.assertIs(core._history, history)
        self.assertIsNotNone(core.consume_target())

    def test_release_encoding_stop_and_disconnect_preserve_new_state(self):
        for method, expected in (("stop", State.STOPPED), ("disconnect", State.DISCONNECTED)):
            with self.subTest(method=method):
                core, _, _ = self._release_mutation(lambda core, clock: getattr(core, method)())
                self.assertIs(core.snapshot().state, expected)
                self.assertIsNone(core.snapshot().pending_target)
                self.assertIsNone(core._history)

    def test_release_encoding_target_replacement_and_takeover_preserve_new_work(self):
        def replace_target(core, clock):
            submit(core, (JointTarget(-0.02, 0, 0, 1, 0),), sequence=1)
        core, old, history = self._release_mutation(replace_target)
        self.assertIsNot(core.snapshot().pending_target, old)
        self.assertEqual(core.snapshot().last_sequence, 1)
        self.assertIs(core._history, history)

        def takeover(core, clock):
            core.stop()
            core.begin_check()
            core.receive(0, JointSample(0, 0, 0), STOP)
            core.checked()
            core.arm(owner="replacement")
            submit(core, (JointTarget(0.01, 0, 0, 1, 0),), sequence=1)
        core, old, history = self._release_mutation(takeover)
        self.assertEqual(core.snapshot().ownership.owner, "replacement")
        self.assertIsNot(core.snapshot().pending_target, old)
        self.assertIsNot(core._history, history)
        self.assertIs(core.snapshot().state, State.ARMED)

    def test_release_encoding_source_and_history_identity_changes_veto(self):
        def new_history(core, clock):
            core._history = replace(core._history)
        core, pending, old_history = self._release_mutation(new_history)
        self.assertIs(core.snapshot().pending_target, pending)
        self.assertIsNot(core._history, old_history)
        def new_source(core, clock):
            core._source = SourceIdentity("fixture", "different", 2)
        core, pending, history = self._release_mutation(new_source)
        self.assertIs(core.snapshot().pending_target, pending)
        self.assertIs(core._history, history)

    def test_terminal_clock_callback_stop_is_observed(self):
        core, station, clock = armed()
        submit(core, (JointTarget(0.02, 0, 0, 1, 0),))
        clock.value = 0.1
        calls = 0
        def stopping_clock():
            nonlocal calls
            calls += 1
            if calls == 2:
                core.stop()
            return clock.value
        core._clock = stopping_clock
        self.assertIsNone(station.consume())
        self.assertEqual(station.writes, [])
        self.assertIs(core.snapshot().state, State.STOPPED)

    def test_release_effort_veto_is_independent_of_planner_check(self):
        core, station, clock = armed((spec(cap=6.0005),), velocity=1.0)
        unsafe = (JointTarget(0, 0, 0, 0, 6),)
        pending = submit(core, unsafe)
        clock.value = 0.1
        with patch.object(core_module, "plan_reference", return_value=unsafe):
            self.assertIsNone(station.consume())
        self.assertIs(core.snapshot().pending_target, pending)
        self.assertEqual(station.writes, [])


if __name__ == "__main__":
    unittest.main()
