import math
import unittest

from im6014_control.codec import command
from im6014_control.core import JointCore
from im6014_control.records import DecisionCode, JointSpec, PositionLimits, State, Target
from im6014_control.mapping import JointMap, JointTarget

from tests.control.test_controller import DeterministicClock, FakeStation, WIDE_LIMITS


class PositionLimitTests(unittest.TestCase):
    def _armed_station(self, specs, clock=None):
        if clock is None:
            clock = DeterministicClock(0.0)
        core = JointCore(specs, clock=clock, max_reference_step_s=1000000.0)
        station = FakeStation(core)
        self.assertTrue(station.connect().accepted)
        self.assertTrue(station.check().accepted)
        self.assertTrue(station.arm(owner="test").accepted)
        station.writes.clear()
        return core, station, clock

    @staticmethod
    def _target(core, sequence, joints):
        return Target(
            sequence,
            tuple(joints),
            created_at_s=0.0,
            expires_at_s=1.0,
            ownership=core.snapshot().ownership,
        )

    def test_limits_require_finite_ordered_real_values_and_allow_equal_bounds(self):
        invalid = (
            (True, 1.0),
            (0.0, False),
            ("0", 1.0),
            (0.0, None),
            (math.nan, 1.0),
            (0.0, math.inf),
            (-math.inf, 0.0),
            (10**1000, 10**1000),
            (1.0, 0.0),
        )
        for minimum, maximum in invalid:
            with self.subTest(minimum=type(minimum).__name__, maximum=type(maximum).__name__), self.assertRaises(ValueError):
                PositionLimits(minimum, maximum)

        equal = PositionLimits(0.25, 0.25)
        self.assertEqual((equal.minimum_rad, equal.maximum_rad), (0.25, 0.25))
        with self.assertRaises(AttributeError):
            equal.minimum_rad = 0.0
        with self.assertRaises(TypeError):
            JointSpec("J1", 0, JointMap(1, 1, 0.0))
        with self.assertRaises(ValueError):
            JointSpec("J1", 0, JointMap(1, 1, 0.0), object(), max_velocity_rad_s=1000000.0, max_acceleration_rad_s2=1000000.0, max_measured_lead_rad=1000000.0)

    def test_endpoints_and_zero_width_are_accepted_but_adjacent_values_reject(self):
        limits = (PositionLimits(-0.4, 0.2), PositionLimits(0.1, 0.1))
        specs = (
            JointSpec("J1", 0, JointMap(1, 1, 0.0), limits[0], max_velocity_rad_s=1000000.0, max_acceleration_rad_s2=1000000.0, max_measured_lead_rad=1000000.0),
            JointSpec("J2", 1, JointMap(1, 1, -0.1), limits[1], max_velocity_rad_s=1000000.0, max_acceleration_rad_s2=1000000.0, max_measured_lead_rad=1000000.0),
        )
        core, station, clock = self._armed_station(specs)
        endpoint_target = self._target(core, 0, (
            JointTarget(-0.4, 0.0, 0.0, 1.0, 0.1),
            JointTarget(0.1, 0.0, 0.0, 1.0, 0.1),
        ))
        self.assertTrue(station.submit(endpoint_target).accepted)
        clock.value = 0.01
        first_reference = station.consume()
        self.assertEqual(first_reference.requested, endpoint_target)
        accepted_writes = list(station.writes)
        self.assertEqual(len(accepted_writes), 2)
        other_endpoint = self._target(core, 1, (
            JointTarget(0.2, 0.0, 0.0, 1.0, 0.1),
            JointTarget(0.1, 0.0, 0.0, 1.0, 0.1),
        ))
        self.assertTrue(station.submit(other_endpoint).accepted)
        clock.value = 0.02
        second_reference = station.consume()
        self.assertEqual(second_reference.requested, other_endpoint)
        accepted_writes = list(station.writes)
        self.assertEqual(len(accepted_writes), 4)

        outside = (
            (0, math.nextafter(-0.4, -math.inf)),
            (0, math.nextafter(0.2, math.inf)),
            (1, math.nextafter(0.1, -math.inf)),
            (1, math.nextafter(0.1, math.inf)),
        )
        for joint_index, q_rad in outside:
            with self.subTest(joint=joint_index, q_rad=q_rad):
                joints = [
                    JointTarget(-0.4, 0.0, 0.0, 1.0, 0.1),
                    JointTarget(0.1, 0.0, 0.0, 1.0, 0.1),
                ]
                joints[joint_index] = JointTarget(q_rad, 0.0, 0.0, 1.0, 0.1)
                before = core.snapshot()
                rejected = station.submit(self._target(core, 2, joints))
                self.assertEqual(rejected.code, DecisionCode.POSITION_LIMIT)
                self.assertEqual(core.snapshot(), before)
                self.assertEqual(station.writes, accepted_writes)

    def test_bad_second_joint_rejects_whole_vector_and_does_not_advance_sequence(self):
        specs = (
            JointSpec("J1", 0, JointMap(1, 1, 0.0), PositionLimits(-0.5, 0.5), max_velocity_rad_s=1000000.0, max_acceleration_rad_s2=1000000.0, max_measured_lead_rad=1000000.0),
            JointSpec("J2", 1, JointMap(1, 1, 0.0), PositionLimits(-0.25, 0.25), max_velocity_rad_s=1000000.0, max_acceleration_rad_s2=1000000.0, max_measured_lead_rad=1000000.0),
        )
        core, station, _ = self._armed_station(specs)
        pending = self._target(core, 3, (
            JointTarget(0.1, 0.0, 0.0, 1.0, 0.1),
            JointTarget(0.2, 0.0, 0.0, 1.0, 0.1),
        ))
        self.assertTrue(station.submit(pending).accepted)
        before = core.snapshot()

        invalid_higher = self._target(core, 5, (
            JointTarget(0.3, 0.0, 0.0, 1.0, 0.1),
            JointTarget(math.nextafter(0.25, math.inf), 0.0, 0.0, 1.0, 0.1),
        ))
        self.assertEqual(station.submit(invalid_higher).code, DecisionCode.POSITION_LIMIT)
        self.assertEqual(core.snapshot(), before)
        self.assertEqual(station.writes, [])

        valid_lower = self._target(core, 4, (
            JointTarget(-0.3, 0.0, 0.0, 1.0, 0.1),
            JointTarget(-0.2, 0.0, 0.0, 1.0, 0.1),
        ))
        self.assertTrue(station.submit(valid_lower).accepted)
        self.assertEqual(core.snapshot().pending_target, valid_lower)
        self.assertEqual(core.snapshot().last_sequence, 4)
        self.assertEqual(station.writes, [])

    def test_codec_invalid_position_inside_configured_interval_keeps_codec_rejection(self):
        specs = (JointSpec("J1", 0, JointMap(1, 1, 0.0), WIDE_LIMITS, max_velocity_rad_s=1000000.0, max_acceleration_rad_s2=1000000.0, max_measured_lead_rad=1000000.0),)
        core, station, _ = self._armed_station(specs)
        valid = self._target(core, 1, (JointTarget(0.1, 0.0, 0.0, 1.0, 0.1),))
        self.assertTrue(station.submit(valid).accepted)
        before = core.snapshot()

        codec_invalid = self._target(core, 2, (JointTarget(1e7, 0.0, 0.0, 1.0, 0.1),))
        self.assertGreaterEqual(codec_invalid.joints[0].q_rad, specs[0].position_limits.minimum_rad)
        self.assertLessEqual(codec_invalid.joints[0].q_rad, specs[0].position_limits.maximum_rad)
        self.assertEqual(station.submit(codec_invalid).code, DecisionCode.INVALID_ARGUMENT)
        self.assertEqual(core.snapshot(), before)
        self.assertEqual(station.writes, [])

    def test_arm_rejects_measured_pose_outside_interval_without_grant_or_generation_change(self):
        specs = (JointSpec("J1", 0, JointMap(1, 1, 0.0), PositionLimits(-0.05, 0.05), max_velocity_rad_s=1000000.0, max_acceleration_rad_s2=1000000.0, max_measured_lead_rad=1000000.0),)
        core = JointCore(specs, clock=DeterministicClock(0.0), max_reference_step_s=1000000.0)
        station = FakeStation(core)
        self.assertTrue(station.connect().accepted)
        self.assertTrue(station.check().accepted)
        before = core.snapshot()
        station.writes.clear()

        rejected = station.arm(owner="test")
        self.assertEqual(rejected.code, DecisionCode.POSITION_LIMIT)
        after = core.snapshot()
        self.assertIs(after.state, State.READY)
        self.assertIsNone(after.ownership)
        self.assertEqual(after.ownership_generation, before.ownership_generation)
        self.assertEqual(after, before)
        self.assertEqual(station.writes, [])

    def test_requested_limits_are_joint_domain_for_all_reductions_directions_and_offsets(self):
        for reduction in (1, 3, 6):
            for direction in (-1, 1):
                with self.subTest(reduction=reduction, direction=direction):
                    mapping = JointMap(reduction, direction, 0.37)
                    spec = JointSpec("J1", 0, mapping, PositionLimits(-0.5, 0.5), max_velocity_rad_s=1000000.0, max_acceleration_rad_s2=1000000.0, max_measured_lead_rad=1000000.0)
                    core, station, clock = self._armed_station((spec,))
                    requested = JointTarget(0.2, 0.0, 0.0, 1.0, 0.1)
                    target = self._target(core, 0, (requested,))
                    expected_output = mapping.to_output(requested)
                    expected_packet = command(
                        0,
                        mode=1,
                        rpm=expected_output.rpm,
                        kd=expected_output.kd,
                        torque=expected_output.tau_nm,
                        watchdog=True,
                        position=expected_output.q_rad,
                        kp=expected_output.kp,
                    )

                    self.assertTrue(station.submit(target).accepted)
                    clock.value = 0.01
                    reference = station.consume()
                    self.assertEqual(reference.requested, target)
                    self.assertEqual(station.writes, [expected_packet])


if __name__ == "__main__":
    unittest.main()
