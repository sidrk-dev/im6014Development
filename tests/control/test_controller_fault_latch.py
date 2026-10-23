from dataclasses import replace
import unittest

from im6014_control.mapping import JointSample, JointTarget
from im6014_control.records import DecisionCode, MotorHealth, State
from tests.control.test_controller_effort import (
    STOP, armed, spec, target,
)


class FaultLatchAcceptanceTests(unittest.TestCase):
    def test_thermal_and_measured_effort_faults_latch_per_affected_joint(self):
        cases = [
            (f"thermal-bit-{bit}", 1 << bit, 0.0)
            for bit in (4, 5, 6, 8, 9)
        ]
        cases += [
            ("repeated-measured-over-effort", 0, 1.1),
            ("combined-thermal-effort", sum(1 << bit for bit in (4, 5, 6, 8, 9)), 1.1),
        ]
        for label, bits, effort in cases:
            for affected in (0, 1):
                with self.subTest(case=label, affected_motor_id=affected):
                    core, station, clock = armed((spec("J1", 0), spec("J2", 1)))
                    old = target(core, (JointTarget(0, 0, 0, 1, 1),) * 2)
                    self.assertTrue(core.submit_target(old).accepted)
                    self.assertIs(core.snapshot().pending_target, old)
                    bad = MotorHealth(0, bits, 0, False)
                    for attempt in range(3):
                        self.assertTrue(
                            core.receive(affected, JointSample(0, 0, effort), bad).accepted
                        )
                        before = core.snapshot()
                        self.assertIs(before.state, State.FAULT)
                        self.assertIsNone(before.ownership)
                        self.assertIsNone(before.pending_target)
                        fresh = replace(old, sequence=attempt + 10)
                        self.assertIsNot(fresh, old)
                        decision = core.submit_target(fresh)
                        self.assertIs(decision.code, DecisionCode.INVALID_STATE)
                        self.assertEqual(core.snapshot(), before)
                        self.assertIsNone(station.consume())
                    fault = core.snapshot().faults[0]
                    self.assertEqual((fault.name, fault.motor_id), (f"J{affected + 1}", affected))
                    self.assertEqual(fault.errors, bits)
                    self.assertIs(fault.effort_limit_exceeded, effort > 1)

                    self.assertTrue(
                        core.receive(affected, JointSample(0, 0, 0), STOP).accepted
                    )
                    self.assertTrue(core.snapshot().fault_latched)
                    self.assertTrue(station.stop().accepted)
                    self.assertTrue(core.snapshot().fault_latched)
                    self.assertTrue(station.disconnect().accepted)
                    self.assertTrue(core.snapshot().fault_latched)
                    self.assertTrue(station.connect().accepted)
                    self.assertIs(
                        station.check().code, DecisionCode.FAULT_ACK_REQUIRED
                    )
                    self.assertFalse(station.arm(owner="automatic").accepted)
                    self.assertTrue(core.begin_check(acknowledge_fault=True).accepted)
                    self.assertTrue(
                        core.receive(affected, JointSample(0, 0, effort), bad).accepted
                    )
                    self.assertFalse(core.checked().accepted)
                    self.assertTrue(core.snapshot().fault_latched)

                    self.assertTrue(station.check(acknowledge_fault=True).accepted)
                    recovered = core.snapshot()
                    self.assertIs(recovered.state, State.READY)
                    self.assertFalse(recovered.fault_latched)
                    self.assertIsNone(recovered.ownership)
                    self.assertFalse(core.submit_target(old).accepted)
                    self.assertIsNone(station.consume())

                    station.writes.clear()
                    grant = station.arm(owner="explicit-recovery")
                    self.assertTrue(grant.accepted)
                    self.assertIsNot(grant.ownership, old.ownership)
                    self.assertEqual(station.writes, [])
                    stale = core.submit_target(old)
                    self.assertIs(stale.code, DecisionCode.INVALID_OWNERSHIP)
                    fresh = target(core, (JointTarget(0.1, 0, 0, 1, 1),) * 2, sequence=100)
                    self.assertTrue(core.submit_target(fresh).accepted)
                    clock.value = 0.01
                    self.assertIsNotNone(station.consume())
                    self.assertEqual(len(station.writes), 2)


if __name__ == "__main__":
    unittest.main()
