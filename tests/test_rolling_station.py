import functools
import os
from pathlib import Path
import sys
import unittest

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from examples.rolling_reach_arm import scene  # noqa: E402
from examples.rolling_reach_arm.station import (  # noqa: E402
    Capture,
    Reply,
    RollingStation,
)
from im6014_control.records import (  # noqa: E402
    CommunicationFailure,
    Decision,
    DecisionCode,
    Reference,
    State,
)


HOME = np.asarray(scene.SCENARIO.home, dtype=float)
KP = np.array([scene.SCENARIO.yaw_kp, scene.SCENARIO.bend_kp, scene.SCENARIO.bend_kp])
KD = np.array([scene.SCENARIO.yaw_kv, scene.SCENARIO.bend_kv, scene.SCENARIO.bend_kv])
FORCE = np.array([scene.SCENARIO.yaw_force, scene.SCENARIO.bend_force, scene.SCENARIO.bend_force])


def ready_station(owner="station-test"):
    station = RollingStation()
    if not station.check().accepted:
        station.close()
        raise AssertionError("check failed")
    if not station.arm(owner=owner).accepted:
        station.close()
        raise AssertionError("arm failed")
    return station


def target_from(capture, delta):
    return tuple((np.asarray(capture.q) + np.asarray(delta)).tolist())


def first_capture(station, advances=20):
    for _ in range(advances):
        captures = station.advance_control_tick()
        if captures:
            return captures[0]
    raise AssertionError("no capture emitted")


class LifecycleTests(unittest.TestCase):
    def test_explicit_check_and_arm_never_enable_actuation(self):
        station = RollingStation()
        self.addCleanup(station.close)

        status = station.status()
        self.assertIs(status.state, State.DISCONNECTED)
        self.assertFalse(status.actuation_enabled)
        self.assertEqual(station.advance_control_tick(), ())
        self.assertEqual(station.status().captures_emitted, 0)
        for forbidden in ("ownership", "core", "model", "data"):
            self.assertFalse(hasattr(status, forbidden))

        self.assertTrue(station.check().accepted)
        status = station.status()
        self.assertIs(status.state, State.READY)
        self.assertFalse(status.actuation_enabled)
        self.assertEqual(status.source.backend, "demo")
        self.assertIn(scene.SCENARIO.name, status.source.identity)

        decision = station.arm(owner="first")
        self.assertIs(type(decision), Decision)
        self.assertTrue(decision.accepted)
        self.assertFalse(hasattr(decision, "ownership"))
        status = station.status()
        self.assertIs(status.state, State.ARMED)
        self.assertFalse(status.actuation_enabled)

    def test_rejected_check_does_not_disable_active_control(self):
        station = ready_station()
        self.addCleanup(station.close)
        capture = first_capture(station)
        station.advance_control_tick(
            replies=(Reply(capture, target_from(capture, (0.02, 0.02, 0.02))),)
        )
        self.assertTrue(station.status().actuation_enabled)
        self.assertIs(station.status().state, State.MOVING)

        rejected = station.check()
        self.assertFalse(rejected.accepted)
        self.assertIs(rejected.code, DecisionCode.INVALID_STATE)
        status = station.status()
        self.assertIs(status.state, State.MOVING)
        self.assertTrue(status.actuation_enabled)
        station.advance_control_tick()
        self.assertTrue(station.status().actuation_enabled)


class SchedulingTests(unittest.TestCase):
    def test_one_second_schedules_exact_integer_counts(self):
        station = ready_station()
        self.addCleanup(station.close)
        captures = []
        for _ in range(100):
            captures.extend(station.advance_control_tick())
        status = station.status()
        self.assertEqual(status.step, 3000)
        self.assertEqual(status.control_ticks, 100)
        self.assertEqual(status.camera_ticks, 30)
        self.assertEqual(status.captures_emitted, 30)
        self.assertEqual(len(captures), 30)
        self.assertEqual([capture.sequence for capture in captures], list(range(30)))
        self.assertEqual(
            [capture.captured_at_s for capture in captures],
            [index * 100 / 3000 for index in range(30)],
        )
        self.assertEqual(len(captures[0].rgb), 240 * 320 * 3)
        self.assertEqual(status.captures_buffered, 8)
        self.assertEqual(status.captures_dropped, 22)
        self.assertIs(status.state, State.ARMED)
        self.assertFalse(status.actuation_enabled)

    def test_reply_limit_rejects_before_any_physics_step(self):
        station = ready_station()
        self.addCleanup(station.close)
        dummy = Capture(0, 0.0, (0.0, 0.0, 0.0), b"x")
        replies = tuple(Reply(dummy, (0.0, 0.0, 0.0)) for _ in range(9))
        with self.assertRaises(ValueError):
            station.advance_control_tick(replies=replies)
        self.assertEqual(station.status().step, 0)
        with self.assertRaises(ValueError):
            station.advance_control_tick(replies=[Reply(dummy, (0.0, 0.0, 0.0))])
        self.assertEqual(station.status().step, 0)


class ClosedLoopTests(unittest.TestCase):
    def test_unprimed_fifty_ms_reply_does_not_issue_enable_or_reseed(self):
        station = ready_station()
        self.addCleanup(station.close)
        seed = station._core._history.joints
        self.assertEqual(station._core._history.kind, "seed")
        capture = first_capture(station)
        for _ in range(4):
            station.advance_control_tick()
        self.assertEqual(station.status().step, 150)

        station.advance_control_tick(
            replies=(Reply(capture, (0.05, 0.04, -0.87)),)
        )
        status = station.status()
        self.assertIs(status.state, State.ARMED)
        self.assertFalse(status.actuation_enabled)
        self.assertEqual(status.references_issued, 0)
        self.assertEqual(status.replies_accepted, 1)
        self.assertIsNone(status.last_issued)
        self.assertFalse(status.fault_latched)
        self.assertIs(station._core._history.joints, seed)
        self.assertEqual(station._core._history.kind, "seed")

        station.advance_control_tick()
        status = station.status()
        self.assertIs(status.state, State.ARMED)
        self.assertFalse(status.actuation_enabled)
        self.assertEqual(status.references_issued, 0)
        self.assertEqual(status.replies_rejected, 0)

    def test_ten_second_multiaxis_progress_with_delayed_fresh_replies(self):
        station = ready_station()
        self.addCleanup(station.close)
        goal = (0.05, 0.04, -0.87)
        pending = {}
        records = []
        original_release = station._release

        @functools.wraps(original_release)
        def release(reference, intent, target, tick):
            measured = tuple(
                float(value) for value in station._observer @ station._data.qpos
            )
            velocity = tuple(
                float(value) for value in station._observer @ station._data.qvel
            )
            records.append((
                tick,
                reference,
                intent,
                target,
                measured,
                velocity,
                station._core.snapshot(),
            ))
            return original_release(reference, intent, target, tick)

        station._release = release
        for index in range(1000):
            due = pending.pop(index, ())
            captures = station.advance_control_tick(replies=due)
            for capture in captures:
                q = capture.q if capture.sequence == 0 else goal
                pending[index + (1 if capture.sequence == 0 else 5)] = (
                    Reply(capture, q),
                )

        status = station.status()
        self.assertEqual(status.step, 30000)
        self.assertEqual(status.control_ticks, 1000)
        self.assertEqual(status.camera_ticks, 300)
        self.assertEqual(status.captures_emitted, 300)
        self.assertEqual(status.replies_rejected, 0)
        self.assertIs(status.state, State.MOVING)
        self.assertFalse(status.fault_latched)
        self.assertTrue(status.actuation_enabled)
        self.assertGreater(status.references_issued, 900)
        self.assertEqual(len(records), status.references_issued)

        measured = np.asarray(status.measured_q)
        np.testing.assert_allclose(measured[0], goal[0], atol=1e-3)
        np.testing.assert_allclose(measured[1:], goal[1:], atol=0.02)
        self.assertGreater(abs(measured[1] - HOME[1]), 0.02)
        self.assertGreater(abs(measured[2] - HOME[2]), 0.02)
        self.assertLess(np.max(np.abs(status.measured_dq)), 1e-6)

        velocity_limits = np.array([
            spec.max_velocity_rad_s for spec in station._specs
        ])
        acceleration_limits = np.array([
            spec.max_acceleration_rad_s2 for spec in station._specs
        ])
        previous_issued = None
        previous_rate = None
        previous_tick = None
        for tick, reference, intent, target, measured, velocity, snapshot in records:
            entry = intent.entry
            self.assertIsInstance(reference, Reference)
            self.assertIs(reference.requested, target)
            self.assertIs(target.ownership, entry.owner)
            self.assertIs(snapshot.ownership, entry.owner)
            self.assertIs(snapshot.source, entry.source)
            self.assertIs(snapshot.state, State.MOVING)
            self.assertEqual(target.created_at_s, entry.created_at_s)
            self.assertEqual(target.expires_at_s, entry.expires_at_s)
            self.assertEqual(entry.expires_tick, entry.tick + 750)
            self.assertLess(tick, entry.expires_tick)
            self.assertLess(tick / 3000, entry.expires_at_s)

            issued = np.array([joint.q_rad for joint in reference.joints])
            self.assertTrue(np.isfinite(issued).all())
            for spec, q_issued, q_measured, dq_measured, joint in zip(
                station._specs, issued, measured, velocity, reference.joints
            ):
                self.assertLessEqual(
                    abs(q_issued - q_measured),
                    spec.max_measured_lead_rad + 1e-12,
                )
                self.assertEqual(joint.dq_rad_s, 0.0)
                self.assertEqual(joint.tau_nm, 0.0)
                self.assertEqual(joint.kp, spec.drive.kp)
                self.assertEqual(joint.kd, spec.drive.kd)
                effort = _native_effort(
                    spec.drive, q_issued, q_measured, dq_measured
                )
                self.assertLessEqual(abs(effort), spec.max_effort_nm + 1e-12)

            if previous_issued is not None:
                elapsed = (tick - previous_tick) / 3000
                self.assertGreater(elapsed, 0.0)
                rate = (issued - previous_issued) / elapsed
                self.assertTrue(
                    np.all(np.abs(rate) <= velocity_limits + 1e-9)
                )
                if previous_rate is not None:
                    self.assertTrue(
                        np.all(
                            np.abs(rate - previous_rate)
                            <= acceleration_limits * elapsed + 1e-9
                        )
                    )
                previous_rate = rate
            previous_issued = issued
            previous_tick = tick

    def test_measured_observer_mapping_and_native_pd_force_match(self):
        station = ready_station()
        self.addCleanup(station.close)
        capture = first_capture(station)
        station.advance_control_tick(
            replies=(Reply(capture, target_from(capture, (0.02, 0.015, 0.02))),)
        )
        for _ in range(4):
            station.advance_control_tick()

        status = station.status()
        self.assertTrue(status.actuation_enabled)
        self.assertIsNotNone(status.last_issued)
        measured_q = np.asarray(status.measured_q)
        measured_dq = np.asarray(status.measured_dq)
        measured_tau = np.asarray(status.measured_tau)
        observed_q = np.asarray(station._observer @ station._data.qpos)
        observed_dq = np.asarray(station._observer @ station._data.qvel)
        np.testing.assert_allclose(measured_q, observed_q, atol=1e-15)
        np.testing.assert_allclose(measured_dq, observed_dq, atol=1e-15)
        np.testing.assert_allclose(
            measured_tau,
            np.asarray(station._mapping.T @ station._data.qfrc_actuator),
            atol=1e-15,
        )
        issued = np.asarray(status.last_issued.q)
        np.testing.assert_allclose(station._data.ctrl, issued, atol=1e-15)

        raw = KP * (issued - measured_q) - KD * measured_dq
        expected = np.clip(raw, -FORCE, FORCE)
        np.testing.assert_allclose(measured_tau, expected, atol=1e-9)
        self.assertTrue(np.any(np.abs(raw) < FORCE))
        self.assertGreater(np.max(np.abs(measured_dq)), 1e-4)

    def test_issued_references_respect_lead_rate_accel_and_fixed_native_intent(self):
        station = ready_station()
        self.addCleanup(station.close)
        capture = first_capture(station)
        reply = Reply(capture, target_from(capture, (0.05, 0.04, 0.05)))
        records = []

        original_release = station._release

        @functools.wraps(original_release)
        def release(reference, intent, target, tick):
            history = station._core._history
            issued = tuple(joint.q_rad for joint in history.joints)
            measured = tuple(float(value) for value in station._observer @ station._data.qpos)
            velocity = tuple(float(value) for value in station._observer @ station._data.qvel)
            records.append((station._n, issued, measured, velocity, history.joints))
            return original_release(reference, intent, target, tick)

        station._release = release
        station.advance_control_tick(replies=(reply,))
        for _ in range(11):
            station.advance_control_tick()

        self.assertGreaterEqual(len(records), 10)
        for tick, issued, measured, velocity, joints in records:
            self.assertEqual(len(issued), 3)
            self.assertTrue(np.isfinite(issued).all())
            for spec, q_issued, q_measured, dq_measured, joint in zip(
                station._specs, issued, measured, velocity, joints
            ):
                self.assertLessEqual(
                    abs(q_issued - q_measured),
                    spec.max_measured_lead_rad + 1e-12,
                )
                self.assertEqual(joint.dq_rad_s, 0.0)
                self.assertEqual(joint.tau_nm, 0.0)
                self.assertEqual(joint.kp, spec.drive.kp)
                self.assertEqual(joint.kd, spec.drive.kd)
                effort = _native_effort(
                    spec.drive, q_issued, q_measured, dq_measured
                )
                self.assertLessEqual(abs(effort), spec.max_effort_nm + 1e-12)

        rates = []
        for previous, current in zip(records, records[1:]):
            elapsed = (current[0] - previous[0]) / 3000
            self.assertGreater(elapsed, 0.0)
            rate = (np.asarray(current[1]) - np.asarray(previous[1])) / elapsed
            self.assertTrue(
                np.all(np.abs(rate) <= np.array(
                    [spec.max_velocity_rad_s for spec in station._specs]
                ) + 1e-9)
            )
            rates.append(rate)
        limits = np.array([
            spec.max_acceleration_rad_s2 for spec in station._specs
        ])
        for previous, current in zip(rates, rates[1:]):
            self.assertTrue(np.all(np.abs(current - previous) <= limits * 0.01 + 1e-9))


def _native_effort(drive, q_issued, q_measured, dq_measured):
    return drive.kp * (q_issued - q_measured) - drive.kd * dq_measured


class ReplyAuthorityTests(unittest.TestCase):
    def test_reorder_duplicate_copy_and_malformed_vectors_keep_newest_valid(self):
        station = ready_station()
        self.addCleanup(station.close)
        captures = []
        while len(captures) < 3:
            captures.extend(station.advance_control_tick())
        first, second, third = captures[:3]

        newest = target_from(second, (0.01, 0.01, 0.01))
        copied = Capture(first.sequence, first.captured_at_s, first.q, first.rgb)
        replies = (
            Reply(second, newest),
            Reply(copied, target_from(first, (0.03, 0.03, 0.03))),
            Reply(second, newest),
            Reply(first, target_from(first, (0.04, 0.04, 0.04))),
        )
        station.advance_control_tick(replies=replies)
        status = station.status()
        self.assertEqual(status.last_requested.capture_sequence, second.sequence)
        self.assertEqual(status.last_requested.q, newest)
        self.assertEqual(status.replies_rejected, 3)

        while True:
            captures = station.advance_control_tick()
            if captures:
                fourth = captures[0]
                break
        before = station.status().last_requested
        for bad in (
            (True, 0.0, 0.0),
            (0.0, float("nan"), 0.0),
            (0.0, 0.0, float("inf")),
            [0.0, 0.0, 0.0],
            (0.0, 0.0),
        ):
            station.advance_control_tick(replies=(Reply(fourth, bad),))
            self.assertEqual(station.status().last_requested, before)

        replacement = target_from(fourth, (0.01, 0.01, 0.01))
        station.advance_control_tick(replies=(Reply(fourth, replacement),))
        status = station.status()
        self.assertEqual(status.last_requested.capture_sequence, fourth.sequence)
        self.assertEqual(status.last_requested.q, replacement)
        self.assertGreater(status.replies_rejected, 3)

    def test_disconnect_new_connection_and_old_capture_cannot_reissue(self):
        station = ready_station(owner="first")
        self.addCleanup(station.close)
        capture = first_capture(station)
        station.advance_control_tick(
            replies=(Reply(capture, target_from(capture, (0.01, 0.01, 0.01))),)
        )
        before = station.status()
        self.assertIsNotNone(before.last_issued)
        first_issue = before.last_issued.issue_sequence
        first_connection = before.source.connection
        first_step = before.step

        station.disconnect()
        status = station.status()
        self.assertIs(status.state, State.DISCONNECTED)
        self.assertFalse(status.actuation_enabled)
        self.assertEqual(status.step, first_step)

        self.assertTrue(station.check().accepted)
        self.assertTrue(station.arm(owner="second").accepted)
        status = station.status()
        self.assertGreater(status.source.connection, first_connection)
        self.assertGreaterEqual(status.step, first_step)

        station.advance_control_tick(
            replies=(Reply(capture, target_from(capture, (0.01, 0.01, 0.01))),)
        )
        status = station.status()
        self.assertIsNone(status.last_requested)
        self.assertIsNone(status.last_issued)
        self.assertFalse(status.actuation_enabled)
        self.assertGreaterEqual(
            station._next_issue_sequence, first_issue + 1
        )


class DeadlineTests(unittest.TestCase):
    def test_capture_at_off_boundary_step_100_expires_at_step_850(self):
        station = ready_station()
        self.addCleanup(station.close)
        capture = None
        while capture is None:
            for emitted in station.advance_control_tick():
                if emitted.captured_at_s == 100 / 3000:
                    capture = emitted
        self.assertIsNotNone(capture)
        self.assertEqual(station.status().step, 120)
        self.assertEqual(station._captures[capture.sequence].expires_tick, 850)
        while station.status().step < 810:
            station.advance_control_tick()
        self.assertEqual(station.status().step, 810)

        station.advance_control_tick(
            replies=(Reply(capture, target_from(capture, (0.01, 0.01, 0.01))),)
        )
        status = station.status()
        self.assertEqual(status.step, 840)
        self.assertIs(status.state, State.ARMED)
        self.assertFalse(status.actuation_enabled)
        self.assertEqual(status.references_issued, 0)
        self.assertEqual(status.replies_accepted, 1)
        self.assertIsNotNone(status.last_requested)

        station.advance_control_tick()
        status = station.status()
        self.assertEqual(status.step, 870)
        self.assertIs(status.state, State.STOPPED)
        self.assertFalse(status.actuation_enabled)
        self.assertEqual(status.references_issued, 0)
        self.assertIsNone(status.last_issued)
        self.assertIsNone(status.last_requested)

    def test_receipt_at_exact_capture_expiry_is_rejected(self):
        station = ready_station()
        self.addCleanup(station.close)
        capture = first_capture(station)
        self.assertEqual(capture.captured_at_s + 0.25, 0.25)
        while station.status().step < 750:
            station.advance_control_tick()
        station.advance_control_tick(
            replies=(Reply(capture, target_from(capture, (0.01, 0.01, 0.01))),)
        )
        status = station.status()
        self.assertIsNone(status.last_requested)
        self.assertEqual(status.replies_rejected, 1)
        self.assertIs(status.state, State.ARMED)
        self.assertFalse(status.actuation_enabled)

    def test_fresh_retained_intent_cannot_extend_last_issued_deadline(self):
        station = ready_station()
        self.addCleanup(station.close)
        first = first_capture(station)
        station.advance_control_tick(
            replies=(Reply(first, target_from(first, (0.02, 0.02, 0.02))),)
        )
        status = station.status()
        self.assertIsNotNone(status.last_issued)
        self.assertEqual(status.last_issued.capture_sequence, first.sequence)
        self.assertEqual(status.last_issued.expires_tick, 750)
        requested = status.last_requested
        self.assertEqual(requested.created_at_s, first.captured_at_s)
        self.assertEqual(requested.expires_tick, 750)

        late = None
        while station.status().step < 720:
            for capture in station.advance_control_tick():
                if capture.captured_at_s == 700 / 3000:
                    late = capture
        self.assertIsNotNone(late)
        status = station.status()
        self.assertIs(status.state, State.MOVING)
        self.assertTrue(status.actuation_enabled)
        self.assertEqual(status.last_requested, requested)
        self.assertEqual(status.last_issued.expires_tick, 750)

        station.advance_control_tick(
            replies=(Reply(late, (2.0, 2.0, 2.0)),)
        )
        status = station.status()
        self.assertEqual(status.step, 750)
        self.assertIs(status.state, State.MOVING)
        self.assertTrue(status.actuation_enabled)
        self.assertEqual(status.last_issued.capture_sequence, first.sequence)
        self.assertEqual(status.last_requested.capture_sequence, late.sequence)
        self.assertFalse(status.fault_latched)

        station.advance_control_tick(
            replies=(Reply(late, target_from(late, (0.01, 0.01, 0.01))),)
        )
        status = station.status()
        self.assertIs(status.state, State.STOPPED)
        self.assertFalse(status.actuation_enabled)
        self.assertIsNone(status.last_issued)
        self.assertIsNone(status.last_requested)
        self.assertFalse(status.fault_latched)
        self.assertTrue(station.check().accepted)
        self.assertTrue(station.arm(owner="after-expiry").accepted)
        self.assertFalse(station.status().actuation_enabled)


class StopAndFaultTests(unittest.TestCase):
    def test_stop_is_idempotent_zero_force_sags_and_never_autoarms(self):
        station = ready_station()
        self.addCleanup(station.close)
        capture = first_capture(station)
        station.advance_control_tick(
            replies=(Reply(capture, target_from(capture, (0.03, 0.03, 0.03))),)
        )
        for _ in range(8):
            station.advance_control_tick()
        before = station.status()
        self.assertIs(before.state, State.MOVING)
        self.assertTrue(before.actuation_enabled)
        held_q = np.asarray(before.measured_q)

        station.stop()
        station.stop()
        status = station.status()
        self.assertIs(status.state, State.STOPPED)
        self.assertFalse(status.actuation_enabled)
        self.assertIsNone(status.last_issued)
        self.assertIsNone(status.last_requested)

        station.advance_control_tick()
        status = station.status()
        np.testing.assert_allclose(status.measured_tau, 0.0, atol=1e-12)
        for _ in range(30):
            station.advance_control_tick()
        status = station.status()
        self.assertIs(status.state, State.STOPPED)
        self.assertFalse(status.actuation_enabled)
        np.testing.assert_allclose(status.measured_tau, 0.0, atol=1e-12)
        self.assertGreater(np.max(np.abs(np.asarray(status.measured_q) - held_q)), 1e-3)

        self.assertTrue(station.check().accepted)
        self.assertTrue(station.arm(owner="explicit-recovery").accepted)
        self.assertFalse(station.status().actuation_enabled)

    def test_takeover_stop_at_release_writes_nothing(self):
        station = ready_station()
        self.addCleanup(station.close)
        capture = first_capture(station)
        original_release = station._release

        @functools.wraps(original_release)
        def release(*args, **kwargs):
            station.stop()
            return original_release(*args, **kwargs)

        station._release = release
        station.advance_control_tick(
            replies=(Reply(capture, target_from(capture, (0.01, 0.01, 0.01))),)
        )
        status = station.status()
        self.assertIs(status.state, State.STOPPED)
        self.assertFalse(status.actuation_enabled)
        self.assertIsNone(status.last_issued)
        self.assertEqual(status.references_issued, 0)


class TerminalOwnershipTests(unittest.TestCase):
    def test_actual_core_takeover_between_consume_and_release_writes_nothing(self):
        station = ready_station(owner="original")
        self.addCleanup(station.close)
        capture = first_capture(station)
        original_ownership = station._ownership
        original_source = station._source
        before_ctrl = np.array(station._data.ctrl, copy=True)
        original_release = station._release
        observed = {}

        @functools.wraps(original_release)
        def release(reference, intent, target, tick):
            self.assertIsInstance(reference, Reference)
            self.assertIs(reference.requested, target)
            self.assertIs(reference.requested.ownership, original_ownership)
            self.assertIs(intent.entry.owner, original_ownership)
            self.assertIs(intent.entry.source, original_source)
            self.assertIs(station._ownership, original_ownership)
            self.assertIs(station._source, original_source)
            self.assertIs(station._core.snapshot().ownership, original_ownership)
            self.assertTrue(station._core.stop().accepted)
            self.assertTrue(station._core.begin_check().accepted)
            self.assertTrue(station._receive_feedback())
            self.assertTrue(station._core.checked().accepted)
            grant = station._core.arm(owner="replacement")
            self.assertTrue(grant.accepted)
            snapshot = station._core.snapshot()
            self.assertIsNot(snapshot.ownership, original_ownership)
            self.assertIs(snapshot.source, original_source)
            self.assertIs(snapshot.state, State.ARMED)
            self.assertIs(station._ownership, original_ownership)
            observed["reference"] = reference
            observed["ctrl"] = np.array(station._data.ctrl, copy=True)
            return original_release(reference, intent, target, tick)

        station._release = release
        station.advance_control_tick(
            replies=(Reply(capture, target_from(capture, (0.01, 0.01, 0.01))),)
        )

        status = station.status()
        self.assertIsInstance(observed["reference"], Reference)
        np.testing.assert_array_equal(observed["ctrl"], before_ctrl)
        np.testing.assert_array_equal(station._data.ctrl, before_ctrl)
        self.assertEqual(status.references_issued, 0)
        self.assertFalse(status.actuation_enabled)
        self.assertIs(status.state, State.STOPPED)
        self.assertIsNone(status.last_issued)
        self.assertIsNone(station._last_issued)
        self.assertIsNone(station._core.snapshot().pending_target)
        self.assertIsNone(station._core._history)

    def test_private_grant_change_between_consume_and_release_writes_nothing(self):
        station = ready_station(owner="original")
        self.addCleanup(station.close)
        capture = first_capture(station)
        original_ownership = station._ownership
        original_source = station._source
        before_ctrl = np.array(station._data.ctrl, copy=True)
        original_release = station._release
        observed = {}

        @functools.wraps(original_release)
        def release(reference, intent, target, tick):
            self.assertIsInstance(reference, Reference)
            self.assertIs(intent.entry.owner, original_ownership)
            self.assertTrue(station._core.stop().accepted)
            self.assertTrue(station._core.begin_check().accepted)
            self.assertTrue(station._receive_feedback())
            self.assertTrue(station._core.checked().accepted)
            grant = station._core.arm(owner="replacement")
            self.assertTrue(grant.accepted)
            station._ownership = grant.ownership
            snapshot = station._core.snapshot()
            self.assertIs(snapshot.ownership, station._ownership)
            self.assertIsNot(station._ownership, original_ownership)
            self.assertIs(snapshot.source, original_source)
            self.assertIs(intent.entry.source, original_source)
            observed["ctrl"] = np.array(station._data.ctrl, copy=True)
            return original_release(reference, intent, target, tick)

        station._release = release
        station.advance_control_tick(
            replies=(Reply(capture, target_from(capture, (0.01, 0.01, 0.01))),)
        )

        status = station.status()
        np.testing.assert_array_equal(observed["ctrl"], before_ctrl)
        np.testing.assert_array_equal(station._data.ctrl, before_ctrl)
        self.assertEqual(status.references_issued, 0)
        self.assertFalse(status.actuation_enabled)
        self.assertIs(status.state, State.STOPPED)
        self.assertIsNone(status.last_issued)
        self.assertIsNone(station._core.snapshot().pending_target)
        self.assertIsNone(station._core._history)

    def test_terminal_clock_at_capture_expiry_between_consume_and_release_writes_nothing(self):
        station = ready_station()
        self.addCleanup(station.close)
        capture = first_capture(station)
        entry = station._captures[capture.sequence]
        self.assertEqual(entry.expires_tick, 750)
        before_ctrl = np.array(station._data.ctrl, copy=True)
        original_release = station._release
        observed = {}

        @functools.wraps(original_release)
        def release(reference, intent, target, tick):
            self.assertIsInstance(reference, Reference)
            self.assertIs(intent.entry, entry)
            self.assertLess(station._n, entry.expires_tick)
            self.assertLess(tick, entry.expires_tick)
            station._n = entry.expires_tick
            result = original_release(reference, intent, target, tick)
            observed["reference"] = reference
            observed["tick"] = tick
            observed["after"] = (
                station._references_issued,
                station._authorized,
                station._disabled(),
            )
            return result

        station._release = release
        station.advance_control_tick(
            replies=(Reply(capture, target_from(capture, (0.01, 0.01, 0.01))),)
        )

        status = station.status()
        self.assertIsInstance(observed["reference"], Reference)
        self.assertLess(observed["tick"], entry.expires_tick)
        self.assertEqual(observed["after"], (0, False, True))
        np.testing.assert_array_equal(station._data.ctrl, before_ctrl)
        self.assertEqual(status.references_issued, 0)
        self.assertFalse(status.actuation_enabled)
        self.assertIs(status.state, State.STOPPED)
        self.assertIsNone(status.last_issued)
        self.assertIsNone(station._core.snapshot().pending_target)
        self.assertIsNone(station._core._history)


class FaultTests(unittest.TestCase):
    def test_invalid_feedback_latches_communication_fault_and_recovers_explicitly(self):
        station = ready_station()
        self.addCleanup(station.close)
        original = station._feedback
        station._feedback = lambda: (
            (0.0, 0.0, float("nan")),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
        )
        station.advance_control_tick()
        status = station.status()
        self.assertIs(status.state, State.FAULT)
        self.assertTrue(status.fault_latched)
        self.assertFalse(status.actuation_enabled)
        self.assertTrue(
            all(
                CommunicationFailure.INVALID_FEEDBACK in fault.communication_errors
                for fault in status.faults
            )
        )

        station._feedback = original
        self.assertIs(station.check().code, DecisionCode.FAULT_ACK_REQUIRED)
        self.assertTrue(station.check(acknowledge_fault=True).accepted)
        self.assertFalse(station.status().actuation_enabled)
        self.assertTrue(station.arm(owner="recovered").accepted)
        self.assertFalse(station.status().actuation_enabled)

    def test_measured_effort_fault_latches(self):
        station = ready_station()
        self.addCleanup(station.close)
        station._feedback = lambda: (
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 1e6),
        )
        station.advance_control_tick()
        status = station.status()
        self.assertIs(status.state, State.FAULT)
        self.assertTrue(status.fault_latched)
        self.assertFalse(status.actuation_enabled)
        self.assertTrue(any(fault.effort_limit_exceeded for fault in status.faults))

    def test_stale_feedback_faults_without_refresh(self):
        station = ready_station()
        self.addCleanup(station.close)
        station._receive_feedback = lambda: True
        for _ in range(8):
            station.advance_control_tick()
        status = station.status()
        self.assertIs(status.state, State.FAULT)
        self.assertTrue(status.fault_latched)
        self.assertFalse(status.actuation_enabled)
        self.assertTrue(any(fault.stale for fault in status.faults))


class IdentityTests(unittest.TestCase):
    def test_model_mutations_veto_active_release(self):
        def mutate_mass(station):
            station._model.body_mass[1] += 0.05

        def mutate_camera(station):
            station._model.cam_pos[0, 0] += 0.01

        def mutate_marker_z(station):
            station._model.geom_pos[station._marker_id, 2] += 0.001

        def mutate_gain(station):
            station._model.actuator_gainprm[0, 0] += 1.0

        def mutate_timestep(station):
            station._model.opt.timestep *= 1.01

        def mutate_order(station):
            station._model.actuator_trnid[[0, 2]] = station._model.actuator_trnid[[2, 0]]

        def mutate_other_disable_flag(station):
            station._model.opt.disableflags ^= int(
                mujoco.mjtDisableBit.mjDSBL_GRAVITY
            )

        def mutate_payload_geometry(station):
            station._model.geom_size[station._marker_id, 0] += 0.001

        cases = (
            ("mass", mutate_mass),
            ("camera", mutate_camera),
            ("marker_z", mutate_marker_z),
            ("gain", mutate_gain),
            ("timestep", mutate_timestep),
            ("order", mutate_order),
            ("other_disable_flag", mutate_other_disable_flag),
            ("payload_geometry", mutate_payload_geometry),
        )
        for label, mutate in cases:
            with self.subTest(label=label):
                station = ready_station(owner="identity")
                try:
                    mutate(station)
                    station.advance_control_tick()
                    status = station.status()
                    self.assertIs(status.state, State.FAULT)
                    self.assertTrue(status.fault_latched)
                    self.assertFalse(status.actuation_enabled)
                    self.assertTrue(status.faults)
                finally:
                    station.close()

    def test_marker_xy_and_stop_bit_are_normalized_but_enable_is_vetoed(self):
        station = RollingStation()
        try:
            identity = station.status().identity
            self.assertTrue(station.check().accepted)
            self.assertTrue(station.arm(owner="normalized").accepted)
            station.set_marker(scene.SCENARIO.probes[1].marker_xy)
            station.advance_control_tick()
            status = station.status()
            self.assertFalse(status.fault_latched)
            self.assertIs(status.state, State.ARMED)
            self.assertEqual(status.identity, identity)

            station.stop()
            self.assertTrue(station.check().accepted)
            self.assertTrue(station.arm(owner="stop-bit").accepted)
            station.advance_control_tick()
            status = station.status()
            self.assertFalse(status.fault_latched)
            self.assertEqual(status.identity, identity)

            bit = int(mujoco.mjtDisableBit.mjDSBL_ACTUATION)
            station._model.opt.disableflags &= ~bit
            station.advance_control_tick()
            status = station.status()
            self.assertIs(status.state, State.STOPPED)
            self.assertFalse(status.actuation_enabled)
            self.assertTrue(int(station._model.opt.disableflags) & bit)
        finally:
            station.close()

    def test_set_marker_rejects_bad_and_out_of_workspace_values(self):
        station = RollingStation()
        try:
            for bad in (
                (0.1, 0.1),
                (float("nan"), 0.0),
                (True, 0.0),
                [0.60, 0.0],
                (0.60,),
            ):
                with self.subTest(bad=bad), self.assertRaises(ValueError):
                    station.set_marker(bad)
        finally:
            station.close()


class CloseTests(unittest.TestCase):
    def test_close_is_idempotent_and_disconnects(self):
        station = ready_station()
        station.close()
        status = station.status()
        self.assertIs(status.state, State.DISCONNECTED)
        self.assertFalse(status.actuation_enabled)
        station.close()
        with self.assertRaises(RuntimeError):
            station.advance_control_tick()
        with self.assertRaises(RuntimeError):
            station.check()


if __name__ == "__main__":
    unittest.main()
