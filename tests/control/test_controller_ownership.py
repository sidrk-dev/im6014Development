import unittest
from dataclasses import FrozenInstanceError

from im6014_control.core import JointCore
from im6014_control.records import (
    ArmDecision,
    CommunicationFailure,
    DecisionCode,
    JointSpec,
    JointStatus,
    MotorHealth,
    Ownership,
    PositionLimits,
    Snapshot,
    State,
    Target,
)
from im6014_control.mapping import JointMap, JointTarget
from tests.control.test_controller import DeterministicClock, FakeStation, SAMPLE

WIDE_LIMITS = PositionLimits(-1e12, 1e12)


class OwnershipTests(unittest.TestCase):
    def setUp(self):
        self.specs = (
            JointSpec("J1", 0, JointMap(1.0, 1, 0.0), WIDE_LIMITS, max_velocity_rad_s=1000000.0, max_acceleration_rad_s2=1000000.0, max_measured_lead_rad=1000000.0, max_effort_nm=1e100),
            JointSpec("J2", 1, JointMap(1.0, 1, 0.0), WIDE_LIMITS, max_velocity_rad_s=1000000.0, max_acceleration_rad_s2=1000000.0, max_measured_lead_rad=1000000.0, max_effort_nm=1e100),
        )
        self.joints = (
            JointTarget(0.1, 0.0, 0.0, 1.0, 0.1),
            JointTarget(-0.2, 0.0, 0.0, 2.0, 0.2),
        )

    def _armed_station(self, *, owner="teleop", clock=None):
        if clock is None:
            clock = DeterministicClock(0.0)
        core = JointCore(self.specs, clock=clock, feedback_max_age_s=1.0, max_reference_step_s=1000000.0)
        station = FakeStation(core)
        self.assertTrue(station.connect().accepted)
        self.assertTrue(station.check().accepted)
        decision = station.arm(owner=owner)
        self.assertTrue(decision.accepted)
        return core, station, clock, decision

    def _target(self, sequence, ownership, *, created_at_s=0.0, expires_at_s=10.0, joints=None):
        return Target(
            sequence,
            self.joints if joints is None else joints,
            created_at_s=created_at_s,
            expires_at_s=expires_at_s,
            ownership=ownership,
        )

    def test_records_are_frozen_and_ownership_is_mandatory(self):
        for owner, generation in (("", 1), ("  ", 1), (1, 1), ("operator", 0), ("operator", -1), ("operator", True)):
            with self.subTest(owner=owner, generation=generation), self.assertRaises(ValueError):
                Ownership(owner, generation)

        ownership = Ownership("operator", 1)
        with self.assertRaises(FrozenInstanceError):
            ownership.owner = "other"
        with self.assertRaises(FrozenInstanceError):
            ownership.generation = 2

        with self.assertRaises(ValueError):
            ArmDecision(True, DecisionCode.ACCEPTED, None)
        with self.assertRaises(ValueError):
            ArmDecision(False, DecisionCode.INVALID_STATE, ownership)
        accepted = ArmDecision(True, DecisionCode.ACCEPTED, ownership)
        rejected = ArmDecision(False, DecisionCode.INVALID_STATE, None)
        self.assertTrue(accepted)
        self.assertFalse(rejected)
        self.assertIs(accepted.ownership, ownership)

        with self.assertRaises(TypeError):
            Target(0, self.joints, created_at_s=0.0, expires_at_s=1.0)
        with self.assertRaises(ValueError):
            Target(0, self.joints, created_at_s=0.0, expires_at_s=1.0, ownership="operator")
        with self.assertRaises(ValueError):
            Snapshot(
                state=State.READY,
                joint_names=("J1",),
                joints=(JointStatus(),),
                source=None,
                fault_latched=False,
                feedback_ages_s=(None,),
                ownership=Ownership("operator", 1),
                ownership_generation=2,
            )

    def test_arm_mints_snapshot_capability_and_revokes_it_on_stop(self):
        core, station, clock, decision = self._armed_station(owner="teleop")
        grant = decision.ownership
        snapshot = core.snapshot()
        self.assertIsInstance(decision, ArmDecision)
        self.assertEqual(grant, Ownership("teleop", 1))
        self.assertIs(snapshot.ownership, grant)
        self.assertEqual(snapshot.ownership_generation, 1)
        self.assertIs(snapshot.state, State.ARMED)

        target = self._target(0, grant)
        self.assertTrue(station.submit(target).accepted)
        station.writes.clear()
        clock.value = 0.01
        self.assertEqual(station.consume().requested, target)
        self.assertEqual(len(station.writes), len(self.specs))
        self.assertIs(core.snapshot().state, State.MOVING)

        self.assertTrue(station.stop().accepted)
        stopped = core.snapshot()
        self.assertIsNone(stopped.ownership)
        self.assertEqual(stopped.ownership_generation, 2)
        self.assertIsNone(stopped.pending_target)
        self.assertTrue(station.stop().accepted)
        self.assertEqual(core.snapshot().ownership_generation, 2)
        self.assertTrue(station.check().accepted)
        replacement = station.arm(owner="teleop")
        self.assertTrue(replacement.accepted)
        self.assertIsNot(replacement.ownership, grant)
        self.assertEqual(replacement.ownership.generation, 3)

    def test_failed_arm_never_mints_and_repeated_arm_does_not_advance(self):
        clock = DeterministicClock(0.0)
        core = JointCore(self.specs, clock=clock, feedback_max_age_s=1.0, max_reference_step_s=1000000.0)
        station = FakeStation(core)
        self.assertTrue(station.connect().accepted)
        self.assertTrue(station.check().accepted)

        invalid_owner = station.arm(owner=" ")
        self.assertEqual(invalid_owner.code, DecisionCode.INVALID_ARGUMENT)
        self.assertIsNone(invalid_owner.ownership)
        self.assertIsNone(core.snapshot().ownership)
        self.assertEqual(core.snapshot().ownership_generation, 0)

        clock.value = 1.1
        stale = station.arm(owner="teleop")
        self.assertEqual(stale.code, DecisionCode.STALE_FEEDBACK)
        self.assertIsNone(stale.ownership)
        self.assertEqual(core.snapshot().ownership_generation, 0)

        clock = DeterministicClock(0.0)
        core, station, _clock, decision = self._armed_station(clock=clock)
        grant = decision.ownership
        repeated = station.arm(owner="teleop")
        self.assertEqual(repeated.code, DecisionCode.INVALID_STATE)
        self.assertIsNone(repeated.ownership)
        self.assertIs(core.snapshot().ownership, grant)
        self.assertEqual(core.snapshot().ownership_generation, 1)

        core, station, _clock, _decision = self._armed_station()
        self.assertTrue(station.stop().accepted)
        self.assertTrue(station.check().accepted)
        self.assertTrue(core.receive(0, SAMPLE, MotorHealth(1, 0, 0, False)).accepted)
        not_stopped = station.arm(owner="teleop")
        self.assertEqual(not_stopped.code, DecisionCode.NOT_STOPPED)
        self.assertIsNone(not_stopped.ownership)
        self.assertIsNone(core.snapshot().ownership)
        self.assertEqual(core.snapshot().ownership_generation, 2)

    def test_fabricated_and_foreign_capabilities_reject_before_payload_or_sequence(self):
        core, station, clock, decision = self._armed_station(owner="same-owner")
        other, _other_station, _other_clock, other_decision = self._armed_station(owner="same-owner")
        grant = decision.ownership
        foreign = other_decision.ownership
        fabricated = Ownership(grant.owner, grant.generation)
        self.assertEqual(fabricated, grant)
        self.assertIsNot(fabricated, grant)
        self.assertEqual(foreign, grant)
        self.assertIsNot(foreign, grant)

        prior = self._target(7, grant)
        self.assertTrue(station.submit(prior).accepted)
        station.writes.clear()
        before = core.snapshot()

        unencodable = (JointTarget(0.0, 0.0, 10.0, 1.0, 0.1), self.joints[1])
        bad_capability = self._target(0, fabricated, joints=unencodable)
        self.assertEqual(station.submit(bad_capability).code, DecisionCode.INVALID_OWNERSHIP)
        self.assertEqual(core.snapshot().pending_target, before.pending_target)
        self.assertEqual(core.snapshot().last_sequence, before.last_sequence)
        self.assertEqual(station.writes, [])

        foreign_target = self._target(8, foreign)
        self.assertEqual(station.submit(foreign_target).code, DecisionCode.INVALID_OWNERSHIP)
        self.assertEqual(core.snapshot().pending_target, prior)
        self.assertEqual(core.snapshot().last_sequence, 7)
        self.assertEqual(station.writes, [])

        clock.value = 0.2
        expired = self._target(8, fabricated, expires_at_s=0.1)
        future = self._target(8, fabricated, created_at_s=0.3)
        self.assertEqual(station.submit(expired).code, DecisionCode.EXPIRED)
        self.assertEqual(station.submit(future).code, DecisionCode.INVALID_ARGUMENT)
        self.assertEqual(core.snapshot().pending_target, prior)
        self.assertEqual(core.snapshot().last_sequence, 7)
        self.assertIs(other.snapshot().ownership, foreign)

    def test_revocation_paths_require_a_fresh_explicit_arm(self):
        for path in ("stop", "fault", "communication", "stale", "disconnect"):
            with self.subTest(path=path):
                clock = DeterministicClock(0.0)
                core, station, clock, decision = self._armed_station(clock=clock)
                old_grant = decision.ownership
                old_target = self._target(100, old_grant)
                self.assertTrue(station.submit(old_target).accepted)

                if path == "stop":
                    self.assertTrue(station.stop().accepted)
                    generation_after_revoke = 2
                    self.assertTrue(station.stop().accepted)
                    self.assertEqual(core.snapshot().ownership_generation, generation_after_revoke)
                    self.assertTrue(station.check().accepted)
                elif path == "fault":
                    self.assertTrue(core.receive(0, SAMPLE, MotorHealth(0, 1, 0, False)).accepted)
                    generation_after_revoke = 2
                    self.assertIsNone(core.snapshot().ownership)
                    self.assertTrue(core.communication_failed(1, CommunicationFailure.TIMEOUT).accepted)
                    self.assertEqual(core.snapshot().ownership_generation, generation_after_revoke)
                    self.assertTrue(core.receive(0, SAMPLE, MotorHealth(0, 0, 0, False)).accepted)
                    self.assertIsNone(core.snapshot().ownership)
                    self.assertTrue(station.check(acknowledge_fault=True).accepted)
                elif path == "communication":
                    self.assertTrue(core.communication_failed(0, CommunicationFailure.TIMEOUT).accepted)
                    generation_after_revoke = 2
                    self.assertIsNone(core.snapshot().ownership)
                    self.assertEqual(core.snapshot().ownership_generation, generation_after_revoke)
                    self.assertTrue(station.check(acknowledge_fault=True).accepted)
                elif path == "stale":
                    clock.value = 1.1
                    self.assertTrue(core.tick().accepted)
                    generation_after_revoke = 2
                    self.assertIsNone(core.snapshot().ownership)
                    self.assertTrue(core.tick().accepted)
                    self.assertEqual(core.snapshot().ownership_generation, generation_after_revoke)
                    self.assertTrue(station.check(acknowledge_fault=True).accepted)
                else:
                    self.assertTrue(station.disconnect().accepted)
                    generation_after_revoke = 2
                    self.assertIsNone(core.snapshot().ownership)
                    self.assertTrue(station.connect().accepted)
                    self.assertTrue(station.check().accepted)

                self.assertIsNone(core.snapshot().ownership)
                self.assertEqual(core.snapshot().ownership_generation, generation_after_revoke)
                new_owner = "human" if path == "stop" else "teleop"
                self.assertTrue(station.arm(owner=new_owner).accepted)
                fresh_grant = core.snapshot().ownership
                self.assertIsNot(fresh_grant, old_grant)
                self.assertEqual(fresh_grant.owner, new_owner)
                self.assertEqual(fresh_grant.generation, generation_after_revoke + 1)
                station.writes.clear()

                old_high_sequence = self._target(101, old_grant)
                self.assertEqual(station.submit(old_high_sequence).code, DecisionCode.INVALID_OWNERSHIP)
                self.assertEqual(core.snapshot().last_sequence, 100)
                self.assertIsNone(core.snapshot().pending_target)
                self.assertEqual(station.writes, [])

    def test_lifecycle_discards_old_pending_and_requires_current_grant_for_dispatch(self):
        core, station, clock, decision = self._armed_station()
        old_target = self._target(5, decision.ownership)
        self.assertTrue(station.submit(old_target).accepted)
        self.assertIs(core.snapshot().pending_target, old_target)

        self.assertTrue(station.stop().accepted)
        self.assertIsNone(core.snapshot().pending_target)
        self.assertTrue(station.check().accepted)
        current = station.arm(owner="teleop")
        self.assertTrue(current.accepted)
        station.writes.clear()

        self.assertIsNone(station.consume())
        self.assertIsNone(core.snapshot().pending_target)
        self.assertEqual(core.snapshot().last_sequence, 5)
        self.assertIs(core.snapshot().state, State.ARMED)
        self.assertEqual(station.writes, [])

        delayed_old_target = self._target(6, decision.ownership)
        self.assertEqual(station.submit(delayed_old_target).code, DecisionCode.INVALID_OWNERSHIP)
        self.assertEqual(core.snapshot().last_sequence, 5)
        self.assertIsNone(core.snapshot().pending_target)

        fresh_target = self._target(6, current.ownership)
        self.assertTrue(station.submit(fresh_target).accepted)
        clock.value += 0.01
        self.assertEqual(station.consume().requested, fresh_target)
        self.assertEqual(len(station.writes), len(self.specs))


if __name__ == "__main__":
    unittest.main()
