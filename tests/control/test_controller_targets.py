import unittest
from dataclasses import FrozenInstanceError
import math
from im6014_control.codec import command
from im6014_control.core import JointCore
from im6014_control.records import (
    CommunicationFailure,
    DecisionCode,
    JointSpec,
    MotorHealth,
    Ownership,
    PositionLimits,
    State,
    Target,
)
from im6014_control.mapping import JointMap, JointTarget

from tests.control.test_controller import DeterministicClock, FakeStation, SAMPLE, HEALTHY_STOP, core_at

WIDE_LIMITS = PositionLimits(-1e12, 1e12)


def owned_target(core, *args, **kwargs):
    ownership = core.snapshot().ownership or Ownership("test-fixture", 1)
    return Target(*args, ownership=ownership, **kwargs)


class LatestTargetTests(unittest.TestCase):
    def setUp(self):
        self.specs = (
            JointSpec("J1", 0, JointMap(1.0, 1, 0.0), WIDE_LIMITS, max_velocity_rad_s=1000000.0, max_acceleration_rad_s2=1000000.0, max_measured_lead_rad=1000000.0),
            JointSpec("J2", 1, JointMap(1.0, 1, 0.0), WIDE_LIMITS, max_velocity_rad_s=1000000.0, max_acceleration_rad_s2=1000000.0, max_measured_lead_rad=1000000.0),
        )

    def test_ten_thousand_accepted_updates_leave_one_newest_target_and_one_consume(self):
        clock = DeterministicClock(0.0)
        core = JointCore(self.specs, clock=clock, max_reference_step_s=1000000.0)
        station = FakeStation(core)
        self.assertTrue(station.connect().accepted)
        self.assertTrue(station.check().accepted)
        self.assertTrue(station.arm(owner="test").accepted)
        self.assertIsNone(core.snapshot().pending_target)

        last_target = None
        for i in range(10_000):
            target = owned_target(core,
                i,
                (
                    JointTarget(q_rad=i * 0.0001, dq_rad_s=0.0, tau_nm=0.0, kp=1.0, kd=0.1),
                    JointTarget(q_rad=-i * 0.0001, dq_rad_s=0.0, tau_nm=0.0, kp=1.0, kd=0.1),
                ), created_at_s=0.0, expires_at_s=1000000.0
            )
            decision = station.submit(target)
            self.assertTrue(decision.accepted)
            last_target = target

        snap = core.snapshot()
        self.assertEqual(snap.pending_target, last_target)
        self.assertEqual(snap.last_sequence, 9999)
        self.assertIs(snap.state, State.ARMED)

        clock.value = 0.01
        consumed = station.consume()
        self.assertEqual(consumed.requested, last_target)
        self.assertIs(core.snapshot().state, State.MOVING)
        self.assertIsNone(core.snapshot().pending_target)

        second_consumed = station.consume()
        self.assertIsNone(second_consumed)
        self.assertIs(core.snapshot().state, State.MOVING)
        self.assertIsNone(core.snapshot().pending_target)

    def test_immutable_owned_vector(self):
        clock = DeterministicClock(0.0)
        core = JointCore(self.specs, clock=clock, max_reference_step_s=1000000.0)
        station = FakeStation(core)
        self.assertTrue(station.connect().accepted)
        self.assertTrue(station.check().accepted)
        self.assertTrue(station.arm(owner="test").accepted)

        jt1 = JointTarget(0.1, 0.0, 0.0, 1.0, 0.1)
        jt2 = JointTarget(0.2, 0.0, 0.0, 1.0, 0.1)

        for bad_joints in (
            [jt1, jt2],
            "not a tuple",
            123,
            None,
            {jt1, jt2},
        ):
            with self.subTest(bad_joints=repr(bad_joints)), self.assertRaises(ValueError):
                owned_target(core, 0, bad_joints, created_at_s=0.0, expires_at_s=1000000.0)

        with self.assertRaises(ValueError):
            owned_target(core, 0, (), created_at_s=0.0, expires_at_s=1000000.0)

        for bad_elements in (
            (jt1, "not a joint target"),
            (1, 2),
            (jt1, None),
        ):
            with self.subTest(bad_elements=repr(bad_elements)), self.assertRaises(ValueError):
                owned_target(core, 0, bad_elements, created_at_s=0.0, expires_at_s=1000000.0)

        target = owned_target(core, 0, (jt1, jt2), created_at_s=0.0, expires_at_s=1000000.0)
        with self.assertRaises(FrozenInstanceError):
            target.sequence = 1
        with self.assertRaises(FrozenInstanceError):
            target.joints = (jt2, jt1)
        with self.assertRaises(TypeError):
            target.joints[0] = jt2
        with self.assertRaises(FrozenInstanceError):
            target.joints[0].q_rad = 99.0

        self.assertTrue(station.submit(target).accepted)

        snap = core.snapshot()
        self.assertEqual(snap.pending_target, target)
        self.assertEqual(snap.last_sequence, 0)
        with self.assertRaises(FrozenInstanceError):
            snap.pending_target = None
        with self.assertRaises(FrozenInstanceError):
            snap.last_sequence = 1
        with self.assertRaises(TypeError):
            snap.pending_target.joints[0] = jt1
        with self.assertRaises(FrozenInstanceError):
            snap.pending_target.joints[0].tau_nm = 0.5

    def test_malformed_incomplete_and_invalid_mapped_outputs_preserve_prior_target(self):
        clock = DeterministicClock(0.0)
        core = JointCore(self.specs, clock=clock, max_reference_step_s=1000000.0)
        station = FakeStation(core)
        self.assertTrue(station.connect().accepted)
        self.assertTrue(station.check().accepted)
        self.assertTrue(station.arm(owner="test").accepted)

        jt1 = JointTarget(0.1, 0.0, 0.0, 1.0, 0.1)
        jt2 = JointTarget(0.2, 0.0, 0.0, 1.0, 0.1)
        prior_target = owned_target(core, 1, (jt1, jt2), created_at_s=0.0, expires_at_s=1000000.0)
        self.assertTrue(station.submit(prior_target).accepted)
        self.assertEqual(core.snapshot().pending_target, prior_target)
        self.assertEqual(core.snapshot().last_sequence, 1)

        for malformed in (None, "target", 42, object(), [jt1, jt2]):
            with self.subTest(malformed=repr(malformed)):
                decision = core.submit_target(malformed)
                self.assertFalse(decision.accepted)
                self.assertEqual(decision.code, DecisionCode.INVALID_ARGUMENT)
                self.assertEqual(core.snapshot().pending_target, prior_target)
                self.assertEqual(core.snapshot().last_sequence, 1)

        for wrong_vector in (
            owned_target(core, 2, (jt1,), created_at_s=0.0, expires_at_s=1000000.0),
            owned_target(core, 2, (jt1, jt2, jt1), created_at_s=0.0, expires_at_s=1000000.0),
        ):
            with self.subTest(wrong_vector=wrong_vector):
                decision = core.submit_target(wrong_vector)
                self.assertFalse(decision.accepted)
                self.assertEqual(decision.code, DecisionCode.INVALID_ARGUMENT)
                self.assertEqual(core.snapshot().pending_target, prior_target)
                self.assertEqual(core.snapshot().last_sequence, 1)

        huge_speed = owned_target(core, 2, (JointTarget(0.0, 1e7, 0.0, 1.0, 0.1), jt2), created_at_s=0.0, expires_at_s=1000000.0)
        rej = core.submit_target(huge_speed)
        self.assertFalse(rej.accepted)
        self.assertEqual(rej.code, DecisionCode.INVALID_ARGUMENT)
        self.assertEqual(core.snapshot().pending_target, prior_target)
        self.assertEqual(core.snapshot().last_sequence, 1)

        huge_kp = owned_target(core, 2, (JointTarget(0.0, 0.0, 0.0, 31.0, 0.1), jt2), created_at_s=0.0, expires_at_s=1000000.0)
        rej = core.submit_target(huge_kp)
        self.assertFalse(rej.accepted)
        self.assertEqual(rej.code, DecisionCode.INVALID_ARGUMENT)
        self.assertEqual(core.snapshot().pending_target, prior_target)
        self.assertEqual(core.snapshot().last_sequence, 1)

        huge_kd = owned_target(core, 2, (JointTarget(0.0, 0.0, 0.0, 1.0, 6.1), jt2), created_at_s=0.0, expires_at_s=1000000.0)
        rej = core.submit_target(huge_kd)
        self.assertFalse(rej.accepted)
        self.assertEqual(rej.code, DecisionCode.INVALID_ARGUMENT)
        self.assertEqual(core.snapshot().pending_target, prior_target)
        self.assertEqual(core.snapshot().last_sequence, 1)

        huge_pos = owned_target(core, 2, (JointTarget(1e7, 0.0, 0.0, 1.0, 0.1), jt2), created_at_s=0.0, expires_at_s=1000000.0)
        rej = core.submit_target(huge_pos)
        self.assertFalse(rej.accepted)
        self.assertEqual(rej.code, DecisionCode.INVALID_ARGUMENT)
        self.assertEqual(core.snapshot().pending_target, prior_target)
        self.assertEqual(core.snapshot().last_sequence, 1)

    def test_each_fault_path_clears_pending_without_fake_foc_writes(self):
        for action in ('packet', 'communication', 'tick'):
            with self.subTest(action=action):
                clock = DeterministicClock(0.0)
                core = JointCore(self.specs, clock=clock, max_reference_step_s=1000000.0)
                station = FakeStation(core)
                self.assertTrue(station.connect())
                self.assertTrue(station.check())
                self.assertTrue(station.arm(owner="test"))
                target = owned_target(core, 0, (JointTarget(0, 0, 0, 1, .1),) * 2, created_at_s=0.0, expires_at_s=1000000.0)
                self.assertTrue(station.submit(target))
                self.assertEqual(core.snapshot().last_sequence, 0)
                station.writes.clear()
                if action == 'packet':
                    self.assertTrue(core.receive(1, SAMPLE, MotorHealth(0, 4, 0, False)))
                elif action == 'communication':
                    self.assertTrue(core.communication_failed(1, CommunicationFailure.TIMEOUT))
                else:
                    clock.value = .6
                    self.assertTrue(core.tick())
                self.assertIs(core.snapshot().state, State.FAULT)
                self.assertIsNone(core.snapshot().pending_target)
                self.assertEqual(core.snapshot().last_sequence, 0)
                self.assertIsNone(station.consume())
                self.assertEqual(station.writes, [])

    def test_mapped_torque_outside_codec_bounds_rejects(self):
        clock = DeterministicClock(0.0)
        core = JointCore(self.specs, clock=clock, max_reference_step_s=1000000.0)
        station = FakeStation(core)
        self.assertTrue(station.connect().accepted)
        self.assertTrue(station.check().accepted)
        self.assertTrue(station.arm(owner="test").accepted)

        jt1 = JointTarget(0.1, 0.0, 0.0, 1.0, 0.1)
        jt2 = JointTarget(0.2, 0.0, 0.0, 1.0, 0.1)
        prior_target = owned_target(core, 1, (jt1, jt2), created_at_s=0.0, expires_at_s=1000000.0)
        self.assertTrue(station.submit(prior_target).accepted)
        self.assertEqual(core.snapshot().last_sequence, 1)

        too_high = owned_target(core, 2, (JointTarget(0.0, 0.0, 3.501, 1.0, 0.1), jt2), created_at_s=0.0, expires_at_s=1000000.0)
        rej = core.submit_target(too_high)
        self.assertFalse(rej.accepted)
        self.assertEqual(rej.code, DecisionCode.INVALID_ARGUMENT)
        self.assertEqual(core.snapshot().pending_target, prior_target)
        self.assertEqual(core.snapshot().last_sequence, 1)

        too_low = owned_target(core, 2, (JointTarget(0.0, 0.0, -3.501, 1.0, 0.1), jt2), created_at_s=0.0, expires_at_s=1000000.0)
        rej = core.submit_target(too_low)
        self.assertFalse(rej.accepted)
        self.assertEqual(rej.code, DecisionCode.INVALID_ARGUMENT)
        self.assertEqual(core.snapshot().pending_target, prior_target)
        self.assertEqual(core.snapshot().last_sequence, 1)

        specs_ratio = (
            JointSpec("J1", 0, JointMap(reduction=3.0, direction=-1, zero_output_rad=0.0), WIDE_LIMITS, max_velocity_rad_s=1000000.0, max_acceleration_rad_s2=1000000.0, max_measured_lead_rad=1000000.0),
            JointSpec("J2", 1, JointMap(reduction=1.0, direction=1, zero_output_rad=0.0), WIDE_LIMITS, max_velocity_rad_s=1000000.0, max_acceleration_rad_s2=1000000.0, max_measured_lead_rad=1000000.0),
        )
        core_ratio = JointCore(specs_ratio, clock=clock, max_reference_step_s=1000000.0)
        station_ratio = FakeStation(core_ratio)
        self.assertTrue(station_ratio.connect().accepted)
        self.assertTrue(station_ratio.check().accepted)
        self.assertTrue(station_ratio.arm(owner="test").accepted)

        rej_ratio = core_ratio.submit_target(
            owned_target(core_ratio, 1, (JointTarget(0.0, 0.0, 12.0, 9.0, 0.9), jt2), created_at_s=0.0, expires_at_s=1000000.0)
        )
        self.assertFalse(rej_ratio.accepted)
        self.assertEqual(rej_ratio.code, DecisionCode.INVALID_ARGUMENT)
        self.assertIsNone(core_ratio.snapshot().pending_target)
        self.assertEqual(core_ratio.snapshot().last_sequence, -1)

    def test_successful_fake_adapter_uses_actual_codec_foc_packets_with_independent_expected_mappings(self):
        clock = DeterministicClock(0.0)
        specs = (
            JointSpec("J1", 0, JointMap(reduction=3.0, direction=-1, zero_output_rad=0.25), WIDE_LIMITS, max_velocity_rad_s=1000000.0, max_acceleration_rad_s2=1000000.0, max_measured_lead_rad=1000000.0),
            JointSpec("J2", 1, JointMap(reduction=2.0, direction=1, zero_output_rad=0.1), WIDE_LIMITS, max_velocity_rad_s=1000000.0, max_acceleration_rad_s2=1000000.0, max_measured_lead_rad=1000000.0),
        )
        core = JointCore(specs, clock=clock, max_reference_step_s=1000000.0)
        station = FakeStation(core)
        self.assertTrue(station.connect().accepted)
        self.assertTrue(station.check().accepted)
        self.assertTrue(station.arm(owner="test").accepted)
        station.writes.clear()

        jt1 = JointTarget(q_rad=0.1, dq_rad_s=1.0, tau_nm=-6.0, kp=18.0, kd=4.5)
        jt2 = JointTarget(q_rad=0.2, dq_rad_s=-1.0, tau_nm=3.0, kp=8.0, kd=2.0)
        target = owned_target(core, 1, (jt1, jt2), created_at_s=0.0, expires_at_s=1000000.0)

        decision = station.submit(target)
        self.assertTrue(decision.accepted)
        self.assertEqual(core.snapshot().last_sequence, 1)
        self.assertEqual(station.writes, [])

        expected_pkt_1 = command(
            0,
            mode=1,
            rpm=-180.0 / math.tau,
            kd=0.5,
            torque=2.0,
            watchdog=True,
            position=-0.05,
            kp=2.0,
        )

        expected_pkt_2 = command(
            1,
            mode=1,
            rpm=-120.0 / math.tau,
            kd=0.5,
            torque=1.5,
            watchdog=True,
            position=0.5,
            kp=2.0,
        )

        clock.value = 0.01
        consumed = station.consume()
        self.assertEqual(consumed.requested, target)
        self.assertEqual(station.writes, [expected_pkt_1, expected_pkt_2])
        self.assertIs(core.snapshot().state, State.MOVING)
        self.assertEqual(core.snapshot().last_sequence, 1)

    def test_no_writes_on_rejection_stale_fault_stop_disconnect(self):
        clock = DeterministicClock(0.0)
        core = JointCore(self.specs, clock=clock, max_reference_step_s=1000000.0)
        station = FakeStation(core)
        self.assertTrue(station.connect().accepted)
        self.assertTrue(station.check().accepted)
        self.assertTrue(station.arm(owner="test").accepted)
        station.writes.clear()

        jt1 = JointTarget(0.1, 0.0, 0.0, 1.0, 0.1)
        jt2 = JointTarget(0.2, 0.0, 0.0, 1.0, 0.1)
        valid_target = owned_target(core, 1, (jt1, jt2), created_at_s=0.0, expires_at_s=1000000.0)

        bad_target = owned_target(core, 2, (JointTarget(0.0, 0.0, 10.0, 1.0, 0.1), jt2), created_at_s=0.0, expires_at_s=1000000.0)
        self.assertFalse(station.submit(bad_target).accepted)
        self.assertEqual(station.writes, [])

        self.assertIsNone(station.consume())
        self.assertEqual(station.writes, [])

        self.assertTrue(station.submit(valid_target).accepted)
        self.assertEqual(core.snapshot().last_sequence, 1)
        clock.value = 1.000001
        valid_target_newer = owned_target(core, 2, (jt1, jt2), created_at_s=0.0, expires_at_s=1000000.0)
        self.assertEqual(station.submit(valid_target_newer).code, DecisionCode.STALE_FEEDBACK)
        self.assertEqual(station.writes, [])

        self.assertIsNone(station.consume())
        self.assertEqual(station.writes, [])
        self.assertIs(core.snapshot().state, State.FAULT)
        self.assertIsNone(core.snapshot().pending_target)
        self.assertEqual(core.snapshot().last_sequence, 1)

        revoked_target_3 = owned_target(core, 3, (jt1, jt2), created_at_s=0.0, expires_at_s=1000000.0)
        self.assertEqual(station.submit(revoked_target_3).code, DecisionCode.INVALID_STATE)
        self.assertIsNone(station.consume())
        self.assertEqual(station.writes, [])

        station.writes.clear()
        self.assertTrue(station.check(acknowledge_fault=True).accepted)
        station.writes.clear()
        self.assertTrue(station.arm(owner="test").accepted)
        self.assertEqual(core.snapshot().last_sequence, 1)
        valid_target_3 = owned_target(core, 3, (jt1, jt2), created_at_s=0.0, expires_at_s=1000000.0)
        self.assertTrue(station.submit(valid_target_3).accepted)
        self.assertEqual(core.snapshot().last_sequence, 3)

        station.writes.clear()
        self.assertTrue(station.stop().accepted)
        self.assertEqual(station.writes, [command(spec.motor_id, mode=0) for spec in self.specs])
        self.assertIsNone(core.snapshot().pending_target)
        self.assertEqual(core.snapshot().last_sequence, 3)
        station.writes.clear()
        self.assertIsNone(station.consume())
        self.assertEqual(station.writes, [])

        core2 = JointCore(self.specs, clock=clock, max_reference_step_s=1000000.0)
        station2 = FakeStation(core2)
        self.assertTrue(station2.connect().accepted)
        self.assertTrue(station2.check().accepted)
        self.assertTrue(station2.arm(owner="test").accepted)
        self.assertEqual(core2.snapshot().last_sequence, -1)
        independent_target = owned_target(
            core2, 1, (jt1, jt2), created_at_s=0.0, expires_at_s=1000000.0,
        )
        self.assertTrue(station2.submit(independent_target).accepted)
        self.assertEqual(core2.snapshot().last_sequence, 1)
        station2.writes.clear()

        self.assertTrue(station2.disconnect().accepted)
        self.assertEqual(station2.writes, [])
        self.assertIsNone(core2.snapshot().pending_target)
        self.assertEqual(core2.snapshot().last_sequence, 1)
        self.assertIsNone(station2.consume())
        self.assertEqual(station2.writes, [])

    def test_stale_feedback_discovered_on_consume_clears_pending_and_latches_fault(self):
        clock = DeterministicClock(0.0)
        core = JointCore(self.specs, clock=clock, feedback_max_age_s=0.5, max_reference_step_s=1000000.0)
        station = FakeStation(core)
        self.assertTrue(station.connect().accepted)
        self.assertTrue(station.check().accepted)
        self.assertTrue(station.arm(owner="test").accepted)

        target = owned_target(core, 1, (
            JointTarget(0.1, 0.0, 0.0, 1.0, 0.1),
            JointTarget(0.2, 0.0, 0.0, 1.0, 0.1),
        ), created_at_s=0.0, expires_at_s=1000000.0)
        self.assertTrue(station.submit(target).accepted)
        self.assertEqual(core.snapshot().pending_target, target)
        self.assertEqual(core.snapshot().last_sequence, 1)

        clock.value = 0.4
        self.assertTrue(core.receive(self.specs[0].motor_id, SAMPLE, HEALTHY_STOP).accepted)

        clock.value = 0.6
        consumed = station.consume()
        self.assertIsNone(consumed)

        snap = core.snapshot()
        self.assertIs(snap.state, State.FAULT)
        self.assertTrue(snap.fault_latched)
        self.assertIsNone(snap.pending_target)
        self.assertEqual(snap.last_sequence, 1)
        self.assertEqual(snap.affected_joints, ("J2",))
        self.assertEqual(len(snap.faults), 1)
        self.assertTrue(snap.faults[0].stale)
        self.assertEqual(snap.faults[0].name, "J2")

        self.assertEqual(station.check().code, DecisionCode.FAULT_ACK_REQUIRED)

    def test_only_armed_and_moving_may_submit_and_consume(self):
        rejected_states = (
            State.DISCONNECTED,
            State.CONNECTING,
            State.CONNECTED,
            State.CHECKING,
            State.READY,
            State.STOPPED,
            State.FAULT,
        )
        for state in rejected_states:
            with self.subTest(state=state):
                core = core_at(self, state)
                target = owned_target(core, 1, (
                    JointTarget(0.1, 0.0, 0.0, 1.0, 0.1),
                    JointTarget(0.2, 0.0, 0.0, 1.0, 0.1),
                ), created_at_s=0.0, expires_at_s=1000000.0)
                before = core.snapshot()
                self.assertEqual(core.submit_target(target).code, DecisionCode.INVALID_STATE)
                self.assertIsNone(core.consume_target())
                self.assertEqual(core.snapshot(), before)

        clock = DeterministicClock(0.0)
        core = core_at(self, State.ARMED, clock=clock)
        clock.value += 0.01
        target = owned_target(core, 1, (
            JointTarget(0.1, 0.0, 0.0, 1.0, 0.1),
            JointTarget(0.2, 0.0, 0.0, 1.0, 0.1),
        ), created_at_s=0.0, expires_at_s=1000000.0)
        self.assertTrue(core.submit_target(target).accepted)
        self.assertEqual(core.snapshot().pending_target, target)
        self.assertEqual(core.snapshot().last_sequence, 1)
        self.assertEqual(core.consume_target().requested, target)
        self.assertIs(core.snapshot().state, State.MOVING)
        self.assertEqual(core.snapshot().last_sequence, 1)

        clock.value += 0.01
        target2 = owned_target(core, 2, (
            JointTarget(0.3, 0.0, 0.0, 1.0, 0.1),
            JointTarget(0.4, 0.0, 0.0, 1.0, 0.1),
        ), created_at_s=0.0, expires_at_s=1000000.0)
        self.assertTrue(core.submit_target(target2).accepted)
        self.assertEqual(core.snapshot().pending_target, target2)
        self.assertEqual(core.snapshot().last_sequence, 2)
        self.assertEqual(core.consume_target().requested, target2)
        self.assertIs(core.snapshot().state, State.MOVING)
        self.assertIsNone(core.snapshot().pending_target)
        self.assertEqual(core.snapshot().last_sequence, 2)


class CommandSequenceTests(unittest.TestCase):
    def setUp(self):
        self.specs = (
            JointSpec("J1", 0, JointMap(1.0, 1, 0.0), WIDE_LIMITS, max_velocity_rad_s=1000000.0, max_acceleration_rad_s2=1000000.0, max_measured_lead_rad=1000000.0),
            JointSpec("J2", 1, JointMap(1.0, 1, 0.0), WIDE_LIMITS, max_velocity_rad_s=1000000.0, max_acceleration_rad_s2=1000000.0, max_measured_lead_rad=1000000.0),
        )
        self.valid_joints = (
            JointTarget(0.1, 0.0, 0.0, 1.0, 0.1),
            JointTarget(0.2, 0.0, 0.0, 1.0, 0.1),
        )

    def _armed_station(self, clock=None):
        if clock is None:
            clock = DeterministicClock(0.0)
        core = JointCore(self.specs, clock=clock, max_reference_step_s=1000000.0)
        station = FakeStation(core)
        self.assertTrue(station.connect().accepted)
        self.assertTrue(station.check().accepted)
        self.assertTrue(station.arm(owner="test").accepted)
        return core, station, clock

    def test_actual_order_1_5_5_2_6_ends_at_6(self):
        core, station, clock = self._armed_station()
        self.assertEqual(core.snapshot().last_sequence, -1)
        self.assertIsNone(core.snapshot().pending_target)

        t1 = owned_target(core, 1, self.valid_joints, created_at_s=0.0, expires_at_s=1000000.0)
        d1 = station.submit(t1)
        self.assertTrue(d1.accepted)
        self.assertEqual(core.snapshot().last_sequence, 1)
        self.assertEqual(core.snapshot().pending_target, t1)

        t5 = owned_target(core, 5, self.valid_joints, created_at_s=0.0, expires_at_s=1000000.0)
        d5 = station.submit(t5)
        self.assertTrue(d5.accepted)
        self.assertEqual(core.snapshot().last_sequence, 5)
        self.assertEqual(core.snapshot().pending_target, t5)

        station.writes.clear()
        t5_dup = owned_target(core, 5, self.valid_joints, created_at_s=0.0, expires_at_s=1000000.0)
        d5_dup = station.submit(t5_dup)
        self.assertFalse(d5_dup.accepted)
        self.assertEqual(d5_dup.code, DecisionCode.DUPLICATE_OR_OLDER)
        self.assertEqual(core.snapshot().last_sequence, 5)
        self.assertEqual(core.snapshot().pending_target, t5)
        self.assertEqual(station.writes, [])

        t2 = owned_target(core, 2, self.valid_joints, created_at_s=0.0, expires_at_s=1000000.0)
        d2 = station.submit(t2)
        self.assertFalse(d2.accepted)
        self.assertEqual(d2.code, DecisionCode.DUPLICATE_OR_OLDER)
        self.assertEqual(core.snapshot().last_sequence, 5)
        self.assertEqual(core.snapshot().pending_target, t5)
        self.assertEqual(station.writes, [])

        t6 = owned_target(core, 6, self.valid_joints, created_at_s=0.0, expires_at_s=1000000.0)
        d6 = station.submit(t6)
        self.assertTrue(d6.accepted)
        self.assertEqual(core.snapshot().last_sequence, 6)
        self.assertEqual(core.snapshot().pending_target, t6)

        station.writes.clear()
        clock.value = 0.01
        consumed = station.consume()
        self.assertEqual(consumed.requested, t6)
        self.assertIs(core.snapshot().state, State.MOVING)
        self.assertIsNone(core.snapshot().pending_target)
        self.assertEqual(core.snapshot().last_sequence, 6)
        self.assertEqual(len(station.writes), 2)

    def test_boundaries_0_and_uint64max(self):
        core, station, _ = self._armed_station()
        self.assertEqual(core.snapshot().last_sequence, -1)

        t0 = owned_target(core, 0, self.valid_joints, created_at_s=0.0, expires_at_s=1000000.0)
        self.assertTrue(station.submit(t0).accepted)
        self.assertEqual(core.snapshot().last_sequence, 0)
        self.assertEqual(core.snapshot().pending_target, t0)

        d0_dup = station.submit(t0)
        self.assertFalse(d0_dup.accepted)
        self.assertEqual(d0_dup.code, DecisionCode.DUPLICATE_OR_OLDER)
        self.assertEqual(core.snapshot().last_sequence, 0)
        self.assertEqual(core.snapshot().pending_target, t0)

        uint64_max = (1 << 64) - 1
        t_max = owned_target(core, uint64_max, self.valid_joints, created_at_s=0.0, expires_at_s=1000000.0)
        self.assertTrue(station.submit(t_max).accepted)
        self.assertEqual(core.snapshot().last_sequence, uint64_max)
        self.assertEqual(core.snapshot().pending_target, t_max)

        d_max_dup = station.submit(t_max)
        self.assertFalse(d_max_dup.accepted)
        self.assertEqual(d_max_dup.code, DecisionCode.DUPLICATE_OR_OLDER)
        self.assertEqual(core.snapshot().last_sequence, uint64_max)

        t_smaller = owned_target(core, uint64_max - 1, self.valid_joints, created_at_s=0.0, expires_at_s=1000000.0)
        d_smaller = station.submit(t_smaller)
        self.assertFalse(d_smaller.accepted)
        self.assertEqual(d_smaller.code, DecisionCode.DUPLICATE_OR_OLDER)
        self.assertEqual(core.snapshot().last_sequence, uint64_max)

    def test_sequence_type_and_range_rejections(self):
        core, _station, _clock = self._armed_station()
        invalid_sequences = (
            True,
            False,
            -1,
            -100,
            0.0,
            1.5,
            math.nan,
            math.inf,
            -math.inf,
            "0",
            "1",
            "100",
            1 << 64,
            (1 << 64) + 1,
            None,
            object(),
        )
        for bad_seq in invalid_sequences:
            with self.subTest(bad_seq=repr(bad_seq)), self.assertRaises(ValueError):
                owned_target(core, bad_seq, self.valid_joints, created_at_s=0.0, expires_at_s=1000000.0)

        target = owned_target(core, 1, self.valid_joints, created_at_s=0.0, expires_at_s=1000000.0)
        with self.assertRaises(FrozenInstanceError):
            target.sequence = 2

    def test_invalid_higher_vector_or_mapping_preserves_pending_and_watermark_and_never_blocks_smaller(self):
        core, station, _ = self._armed_station()
        t5 = owned_target(core, 5, self.valid_joints, created_at_s=0.0, expires_at_s=1000000.0)
        self.assertTrue(station.submit(t5).accepted)
        self.assertEqual(core.snapshot().last_sequence, 5)
        self.assertEqual(core.snapshot().pending_target, t5)

        bad_len = owned_target(core, 10, (self.valid_joints[0],), created_at_s=0.0, expires_at_s=1000000.0)
        d_bad_len = station.submit(bad_len)
        self.assertFalse(d_bad_len.accepted)
        self.assertEqual(d_bad_len.code, DecisionCode.INVALID_ARGUMENT)
        self.assertEqual(core.snapshot().last_sequence, 5)
        self.assertEqual(core.snapshot().pending_target, t5)

        bad_map = owned_target(core, 10, (JointTarget(0.0, 0.0, 10.0, 1.0, 0.1), self.valid_joints[1]), created_at_s=0.0, expires_at_s=1000000.0)
        d_bad_map = station.submit(bad_map)
        self.assertFalse(d_bad_map.accepted)
        self.assertEqual(d_bad_map.code, DecisionCode.INVALID_ARGUMENT)
        self.assertEqual(core.snapshot().last_sequence, 5)
        self.assertEqual(core.snapshot().pending_target, t5)

        bad_second = owned_target(core, 10, (self.valid_joints[0], JointTarget(0, 0, 10, 1, .1)), created_at_s=0.0, expires_at_s=1000000.0)
        before = core.snapshot()
        self.assertEqual(station.submit(bad_second).code, DecisionCode.INVALID_ARGUMENT)
        self.assertEqual(core.snapshot(), before)

        t7 = owned_target(core, 7, self.valid_joints, created_at_s=0.0, expires_at_s=1000000.0)
        d7 = station.submit(t7)
        self.assertTrue(d7.accepted)
        self.assertEqual(core.snapshot().last_sequence, 7)
        self.assertEqual(core.snapshot().pending_target, t7)

    def test_watermark_survives_consumption_stop_fault_disconnect_reconnect_and_recovery(self):
        clock = DeterministicClock(0.0)
        core = JointCore(self.specs, clock=clock, feedback_max_age_s=1.0, max_reference_step_s=1000000.0)
        station = FakeStation(core)
        self.assertTrue(station.connect().accepted)
        self.assertTrue(station.check().accepted)
        self.assertTrue(station.arm(owner="test").accepted)

        t10 = owned_target(core, 10, self.valid_joints, created_at_s=0.0, expires_at_s=1000000.0)
        self.assertTrue(station.submit(t10).accepted)
        self.assertEqual(core.snapshot().last_sequence, 10)

        clock.value = 0.01
        consumed = station.consume()
        self.assertEqual(consumed.requested, t10)
        self.assertIs(core.snapshot().state, State.MOVING)
        self.assertIsNone(core.snapshot().pending_target)
        self.assertEqual(core.snapshot().last_sequence, 10)

        self.assertTrue(station.stop().accepted)
        self.assertIs(core.snapshot().state, State.STOPPED)
        self.assertEqual(core.snapshot().last_sequence, 10)

        self.assertTrue(station.check().accepted)
        self.assertTrue(station.arm(owner="test").accepted)
        self.assertEqual(core.snapshot().last_sequence, 10)

        bad_health = MotorHealth(mode=0, errors=1, warnings=0, timeout=False)
        self.assertTrue(core.receive(self.specs[0].motor_id, SAMPLE, bad_health).accepted)
        self.assertIs(core.snapshot().state, State.FAULT)
        self.assertEqual(core.snapshot().last_sequence, 10)

        self.assertTrue(station.stop().accepted)
        self.assertIs(core.snapshot().state, State.FAULT)
        self.assertEqual(core.snapshot().last_sequence, 10)

        self.assertTrue(station.disconnect().accepted)
        self.assertIs(core.snapshot().state, State.DISCONNECTED)
        self.assertEqual(core.snapshot().last_sequence, 10)

        self.assertTrue(station.connect().accepted)
        self.assertIs(core.snapshot().state, State.CONNECTED)
        self.assertEqual(core.snapshot().last_sequence, 10)

        self.assertTrue(station.check(acknowledge_fault=True).accepted)
        self.assertIs(core.snapshot().state, State.READY)
        self.assertEqual(core.snapshot().last_sequence, 10)

        self.assertTrue(station.arm(owner="test").accepted)
        self.assertIs(core.snapshot().state, State.ARMED)
        self.assertEqual(core.snapshot().last_sequence, 10)

        current_t10 = owned_target(
            core, 10, self.valid_joints,
            created_at_s=0.0, expires_at_s=1000000.0,
        )
        d10_dup = station.submit(current_t10)
        self.assertFalse(d10_dup.accepted)
        self.assertEqual(d10_dup.code, DecisionCode.DUPLICATE_OR_OLDER)
        self.assertEqual(core.snapshot().last_sequence, 10)

        t9 = owned_target(core, 9, self.valid_joints, created_at_s=0.0, expires_at_s=1000000.0)
        d9 = station.submit(t9)
        self.assertFalse(d9.accepted)
        self.assertEqual(d9.code, DecisionCode.DUPLICATE_OR_OLDER)
        self.assertEqual(core.snapshot().last_sequence, 10)

        t11 = owned_target(core, 11, self.valid_joints, created_at_s=0.0, expires_at_s=1000000.0)
        d11 = station.submit(t11)
        self.assertTrue(d11.accepted)
        self.assertEqual(core.snapshot().last_sequence, 11)
        self.assertEqual(core.snapshot().pending_target, t11)

    def test_rejected_state_and_stale_submissions_preserve_watermark_and_pending(self):
        core, station, clock = self._armed_station()
        t5 = owned_target(core, 5, self.valid_joints, created_at_s=0.0, expires_at_s=1000000.0)
        self.assertTrue(station.submit(t5).accepted)
        self.assertEqual(core.snapshot().last_sequence, 5)
        self.assertEqual(core.snapshot().pending_target, t5)

        clock.value = 1.000001
        t10 = owned_target(core, 10, self.valid_joints, created_at_s=0.0, expires_at_s=1000000.0)
        d_stale = station.submit(t10)
        self.assertFalse(d_stale.accepted)
        self.assertEqual(d_stale.code, DecisionCode.STALE_FEEDBACK)
        self.assertEqual(core.snapshot().last_sequence, 5)
        self.assertEqual(core.snapshot().pending_target, t5)

        rejected_states = (
            State.DISCONNECTED,
            State.CONNECTING,
            State.CONNECTED,
            State.CHECKING,
            State.READY,
            State.STOPPED,
            State.FAULT,
        )
        for state in rejected_states:
            with self.subTest(state=state):
                state_core = core_at(self, state)
                self.assertIs(state_core.snapshot().state, state)
                self.assertEqual(state_core.snapshot().last_sequence, -1)
                d = state_core.submit_target(owned_target(state_core, 20, self.valid_joints, created_at_s=0.0, expires_at_s=1000000.0))
                self.assertFalse(d.accepted)
                self.assertEqual(d.code, DecisionCode.INVALID_STATE)
                self.assertEqual(state_core.snapshot().last_sequence, -1)
                self.assertIsNone(state_core.snapshot().pending_target)

    def test_new_core_starts_negative_one_and_independent_watermarks(self):
        core1 = JointCore(self.specs, clock=DeterministicClock(0.0), max_reference_step_s=1000000.0)
        core2 = JointCore(self.specs, clock=DeterministicClock(0.0), max_reference_step_s=1000000.0)
        self.assertEqual(core1.snapshot().last_sequence, -1)
        self.assertEqual(core2.snapshot().last_sequence, -1)

        station1 = FakeStation(core1)
        self.assertTrue(station1.connect().accepted)
        self.assertTrue(station1.check().accepted)
        self.assertTrue(station1.arm(owner="test").accepted)

        t10 = owned_target(core1, 10, self.valid_joints, created_at_s=0.0, expires_at_s=1000000.0)
        self.assertTrue(station1.submit(t10).accepted)
        self.assertEqual(core1.snapshot().last_sequence, 10)
        self.assertEqual(core2.snapshot().last_sequence, -1)


class CommandExpiryTests(unittest.TestCase):
    def setUp(self):
        self.specs = (
            JointSpec("J1", 0, JointMap(1.0, 1, 0.0), WIDE_LIMITS, max_velocity_rad_s=1000000.0, max_acceleration_rad_s2=1000000.0, max_measured_lead_rad=1000000.0),
            JointSpec("J2", 1, JointMap(1.0, 1, 0.0), WIDE_LIMITS, max_velocity_rad_s=1000000.0, max_acceleration_rad_s2=1000000.0, max_measured_lead_rad=1000000.0),
        )
        self.joints = (
            JointTarget(0.1, 0.0, 0.0, 1.0, 0.1),
            JointTarget(-0.2, 0.0, 0.0, 2.0, 0.2),
        )

    def _armed_station(self, clock):
        core = JointCore(self.specs, clock=clock, max_reference_step_s=1000000.0)
        station = FakeStation(core)
        self.assertTrue(station.connect().accepted)
        self.assertTrue(station.check().accepted)
        self.assertTrue(station.arm(owner="test").accepted)
        station.writes.clear()
        return core, station

    def test_target_requires_finite_ordered_timestamps_and_accepts_zero(self):
        core, _station = self._armed_station(DeterministicClock(0.0))
        with self.assertRaises(TypeError):
            owned_target(core, 0, self.joints)
        target = owned_target(core, 0, self.joints, created_at_s=0, expires_at_s=1)
        self.assertEqual(target.created_at_s, 0.0)
        self.assertEqual(target.expires_at_s, 1.0)

        for created_at_s in (True, False, None, "0", math.nan, math.inf, -math.inf):
            with self.subTest(created_at_s=created_at_s), self.assertRaises(ValueError):
                owned_target(core, 0, self.joints, created_at_s, 1.0)
        for expires_at_s in (True, False, None, "1", math.nan, math.inf, -math.inf):
            with self.subTest(expires_at_s=expires_at_s), self.assertRaises(ValueError):
                owned_target(core, 0, self.joints, 0.0, expires_at_s)
        for expires_at_s in (-1.0, 0.0):
            with self.subTest(reversed_deadline=expires_at_s), self.assertRaises(ValueError):
                owned_target(core, 0, self.joints, 0.0, expires_at_s)

    def test_future_creation_and_producer_queue_expiry_reject_without_storage(self):
        clock = DeterministicClock(0.0)
        core, station = self._armed_station(clock)
        future = owned_target(core, 1, self.joints, created_at_s=0.1, expires_at_s=1.0)
        self.assertEqual(station.submit(future).code, DecisionCode.INVALID_ARGUMENT)
        self.assertIsNone(core.snapshot().pending_target)
        self.assertEqual(core.snapshot().last_sequence, -1)

        clock.value = 0.25
        queued_too_long = owned_target(core, 2, self.joints, created_at_s=0.0, expires_at_s=0.25)
        self.assertEqual(station.submit(queued_too_long).code, DecisionCode.EXPIRED)
        self.assertIsNone(core.snapshot().pending_target)
        self.assertEqual(core.snapshot().last_sequence, -1)
        self.assertEqual(station.writes, [])

    def test_expired_high_sequence_replacement_preserves_pending_and_watermark(self):
        clock = DeterministicClock(0.0)
        core, station = self._armed_station(clock)
        pending = owned_target(core, 5, self.joints, created_at_s=0.0, expires_at_s=0.5)
        self.assertTrue(station.submit(pending).accepted)

        clock.value = 0.2
        expired = owned_target(core, 10, self.joints, created_at_s=0.0, expires_at_s=0.2)
        self.assertEqual(station.submit(expired).code, DecisionCode.EXPIRED)
        self.assertEqual(core.snapshot().pending_target, pending)
        self.assertEqual(core.snapshot().last_sequence, 5)
        self.assertEqual(station.writes, [])

    def test_valid_target_dispatches_expected_fake_packet_bytes(self):
        clock = DeterministicClock(0.0)
        core, station = self._armed_station(clock)
        target = owned_target(core, 1, self.joints, created_at_s=0.0, expires_at_s=1.0)
        self.assertTrue(station.submit(target).accepted)
        self.assertEqual(station.writes, [])

        clock.value = 0.01
        self.assertEqual(station.consume().requested, target)
        self.assertEqual(station.writes, [
            command(0, mode=1, rpm=0.0, kd=0.1, torque=0.0, watchdog=True, position=0.1, kp=1.0),
            command(1, mode=1, rpm=0.0, kd=0.2, torque=0.0, watchdog=True, position=-0.2, kp=2.0),
        ])

    def test_expiry_at_consume_discards_once_retains_sequence_and_uses_one_clock_sample(self):
        clock = DeterministicClock(0.0)
        core, station = self._armed_station(clock)
        target = owned_target(core, 7, self.joints, created_at_s=0.0, expires_at_s=0.1)
        self.assertTrue(station.submit(target).accepted)
        station.writes.clear()

        clock.value = 0.1
        calls_before_consume = clock.calls
        self.assertIsNone(station.consume())
        self.assertEqual(clock.calls, calls_before_consume + 1)
        snapshot = core.snapshot()
        self.assertIsNone(snapshot.pending_target)
        self.assertEqual(snapshot.last_sequence, 7)
        self.assertIs(snapshot.state, State.ARMED)
        self.assertFalse(snapshot.fault_latched)
        self.assertEqual(station.writes, [])

        same_sequence = owned_target(core, 7, self.joints, created_at_s=0.1, expires_at_s=0.5)
        self.assertEqual(station.submit(same_sequence).code, DecisionCode.DUPLICATE_OR_OLDER)
        newer = owned_target(core, 8, self.joints, created_at_s=0.1, expires_at_s=0.5)
        self.assertTrue(station.submit(newer).accepted)

    def test_clock_regression_raises_before_pending_target_can_be_written(self):
        clock = DeterministicClock(0.0)
        core, station = self._armed_station(clock)
        target = owned_target(core, 1, self.joints, created_at_s=0.0, expires_at_s=1.0)
        self.assertTrue(station.submit(target).accepted)
        station.writes.clear()

        clock.value = -0.1
        with self.assertRaises(ValueError):
            station.consume()
        clock.value = 0.0
        self.assertEqual(core.snapshot().pending_target, target)
        self.assertEqual(station.writes, [])

    def test_timestamp_guards_precede_codec_and_sequence_metadata(self):
        clock = DeterministicClock(0.0)
        core, station = self._armed_station(clock)
        accepted = owned_target(core, 5, self.joints, created_at_s=0.0, expires_at_s=1.0)
        self.assertTrue(station.submit(accepted).accepted)

        future_older = owned_target(core, 4, self.joints, created_at_s=0.1, expires_at_s=1.0)
        self.assertEqual(station.submit(future_older).code, DecisionCode.INVALID_ARGUMENT)
        expired_bad_shape = owned_target(core, 11, self.joints[:1], created_at_s=0.0, expires_at_s=0.01)
        clock.value = 0.01
        self.assertEqual(station.submit(expired_bad_shape).code, DecisionCode.INVALID_ARGUMENT)
        expired_unencodable = owned_target(core,
            10,
            (JointTarget(0.0, 0.0, 10.0, 1.0, 0.1), self.joints[1]),
            created_at_s=0.0,
            expires_at_s=0.01,
        )
        self.assertEqual(station.submit(expired_unencodable).code, DecisionCode.EXPIRED)
        self.assertEqual(core.snapshot().pending_target, accepted)
        self.assertEqual(core.snapshot().last_sequence, 5)


if __name__ == "__main__":
    unittest.main()
