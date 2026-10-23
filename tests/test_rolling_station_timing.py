import dataclasses
import math
import os
from pathlib import Path
import sys
import unittest

os.environ.setdefault("MUJOCO_GL", "egl")

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from examples.rolling_reach_arm import scene  # noqa: E402
from examples.rolling_reach_arm.station import (  # noqa: E402
    FEEDBACK_MAX_AGE_S,
    Capture,
    Reply,
    RollingStation,
)
from examples.rolling_reach_arm.timing import (  # noqa: E402
    StationTiming,
    TimingStats,
)
from im6014_control.records import State  # noqa: E402


def target_from(capture, delta):
    return tuple((np.asarray(capture.q) + np.asarray(delta)).tolist())


def first_capture(station):
    for _ in range(20):
        captures = station.advance_control_tick()
        if captures:
            return captures[0]
    raise AssertionError("no capture emitted")


def ready_station(wall_clock=None):
    station = (
        RollingStation()
        if wall_clock is None
        else RollingStation(wall_clock=wall_clock)
    )
    if not station.check().accepted:
        station.close()
        raise AssertionError("check failed")
    if not station.arm(owner="timing-test").accepted:
        station.close()
        raise AssertionError("arm failed")
    return station


class ManualClock:
    def __init__(self, value=0.0):
        self.value = float(value)

    def __call__(self):
        return self.value


class StepClock:
    def __init__(self, step=0.001, start=0.0):
        self.step = float(step)
        self.value = float(start)

    def __call__(self):
        value = self.value
        self.value += self.step
        return value


class QueueClock:
    def __init__(self, values):
        self.values = list(values)
        self.index = 0

    def __call__(self):
        if self.index >= len(self.values):
            raise AssertionError("unexpected timing clock sample")
        value = self.values[self.index]
        self.index += 1
        return value


class TimingSnapshotTests(unittest.TestCase):
    def test_initial_snapshot_is_empty_and_station_status_is_compatible(self):
        station = RollingStation(wall_clock=ManualClock())
        self.addCleanup(station.close)
        status = station.status()
        self.assertIs(status.state, State.DISCONNECTED)
        self.assertFalse(status.actuation_enabled)
        timing = status.timing
        self.assertIsInstance(timing, StationTiming)
        for stats in (
            timing.callback_period_wall_s,
            timing.advance_wall_s,
            timing.control_wall_s,
            timing.accepted_reply_sim_s,
            timing.rejected_reply_sim_s,
            timing.accepted_reply_wall_s,
            timing.rejected_reply_wall_s,
        ):
            self.assertIsInstance(stats, TimingStats)
            self.assertEqual((stats.count, stats.missing), (0, 0))
            for name in ("last_s", "min_s", "max_s", "mean_s"):
                self.assertIsNone(getattr(stats, name))
        self.assertEqual(
            (
                timing.advances_started,
                timing.advances_completed,
                timing.advances_failed,
                timing.replies_offered,
                timing.replies_unprocessed,
                timing.clock_errors,
                timing.advance_overruns,
            ),
            (0, 0, 0, 0, 0, 0, 0),
        )
        self.assertEqual(timing.advance_budget_wall_s, 0.010)
        self.assertEqual(
            timing.feedback_age_sim_s,
            (None, None, None),
        )
        self.assertIsNone(timing.feedback_peak_observed_sim_s)
        for forbidden in ("model", "data", "core", "ownership", "pending_target"):
            self.assertFalse(hasattr(timing, forbidden))

    def test_lifecycle_and_status_do_not_sample_the_wall_clock(self):
        calls = []

        def clock():
            calls.append(None)
            return 0.0

        station = RollingStation(wall_clock=clock)
        self.addCleanup(station.close)
        self.assertTrue(station.check().accepted)
        self.assertTrue(station.arm(owner="no-sample").accepted)
        station.status()
        station.stop()
        station.disconnect()
        station.status()
        station.close()
        self.assertEqual(calls, [])

    def test_snapshots_are_frozen_and_retained(self):
        station = ready_station(ManualClock())
        self.addCleanup(station.close)
        station.advance_control_tick()
        first = station.status().timing
        station.advance_control_tick()
        second = station.status().timing
        self.assertEqual(first.advances_started, 1)
        self.assertEqual(second.advances_started, 2)
        with self.assertRaises(dataclasses.FrozenInstanceError):
            first.advances_started = 99
        with self.assertRaises(dataclasses.FrozenInstanceError):
            first.advance_wall_s.count = 99
        self.assertEqual(first.advances_started, 1)
        self.assertEqual(first.advance_wall_s.count, 1)


class TimingScopeTests(unittest.TestCase):
    def test_advance_control_capture_and_callback_scopes(self):
        clock = QueueClock(
            [0.0, 1.0, 3.0, 6.0, 10.0, 20.0, 21.0, 22.0, 25.0]
        )
        station = ready_station(clock)
        self.addCleanup(station.close)
        first = station.advance_control_tick()
        self.assertEqual(len(first), 1)
        station.advance_control_tick()
        timing = station.status().timing

        self.assertEqual(clock.index, len(clock.values))
        self.assertEqual(timing.advances_started, 2)
        self.assertEqual(timing.advances_completed, 2)
        self.assertEqual(timing.advances_failed, 0)
        self.assertEqual(timing.advance_wall_s.count, 2)
        self.assertEqual(timing.advance_wall_s.missing, 0)
        self.assertEqual(timing.advance_wall_s.last_s, 5.0)
        self.assertEqual(timing.advance_wall_s.min_s, 5.0)
        self.assertEqual(timing.advance_wall_s.max_s, 10.0)
        self.assertEqual(timing.advance_wall_s.mean_s, 7.5)
        self.assertEqual(timing.control_wall_s.count, 2)
        self.assertEqual(timing.control_wall_s.last_s, 1.0)
        self.assertEqual(timing.control_wall_s.min_s, 1.0)
        self.assertEqual(timing.control_wall_s.max_s, 2.0)
        self.assertEqual(timing.control_wall_s.mean_s, 1.5)
        self.assertEqual(timing.callback_period_wall_s.count, 1)
        self.assertEqual(timing.callback_period_wall_s.missing, 0)
        self.assertEqual(timing.callback_period_wall_s.last_s, 20.0)
        self.assertEqual(timing.advance_overruns, 2)

    def test_zero_durations_equal_reads_and_retained_counts(self):
        station = ready_station(ManualClock(5.0))
        self.addCleanup(station.close)
        station.advance_control_tick()
        station.advance_control_tick()
        timing = station.status().timing
        self.assertEqual(timing.advance_wall_s.count, 2)
        self.assertEqual(timing.advance_wall_s.last_s, 0.0)
        self.assertEqual(timing.advance_wall_s.min_s, 0.0)
        self.assertEqual(timing.advance_wall_s.max_s, 0.0)
        self.assertEqual(timing.control_wall_s.count, 2)
        self.assertEqual(timing.control_wall_s.last_s, 0.0)
        self.assertEqual(timing.callback_period_wall_s.count, 1)
        self.assertEqual(timing.callback_period_wall_s.last_s, 0.0)
        self.assertEqual(timing.advance_overruns, 0)

    def test_overrun_is_strict_and_failed_spans_are_not_fast(self):
        clock = QueueClock(
            [
                0.0, 0.001, 0.002, 0.003, 0.010,
                0.010, 0.011, 0.012, 0.020,
                0.020, 0.021, 0.022, 0.0301,
                0.040, 0.041, 0.042, 0.043,
            ]
        )
        station = ready_station(clock)
        self.addCleanup(station.close)
        for _ in range(3):
            station.advance_control_tick()
        timing = station.status().timing
        self.assertEqual(timing.advance_wall_s.count, 3)
        self.assertEqual(timing.advance_wall_s.missing, 0)
        self.assertEqual(timing.advance_wall_s.min_s, 0.010)
        self.assertEqual(timing.advance_wall_s.max_s, 0.0301 - 0.020)
        self.assertEqual(timing.advance_overruns, 1)

        class Boom(RuntimeError):
            pass

        def boom(*args):
            raise Boom("control failed")

        station._control = boom
        with self.assertRaises(Boom):
            station.advance_control_tick()
        timing = station.status().timing
        self.assertEqual(timing.advances_started, 4)
        self.assertEqual(timing.advances_completed, 3)
        self.assertEqual(timing.advances_failed, 1)
        self.assertEqual(timing.advance_wall_s.count, 3)
        self.assertEqual(timing.advance_wall_s.missing, 1)
        self.assertEqual(timing.control_wall_s.count, 3)
        self.assertEqual(timing.control_wall_s.missing, 1)
        self.assertEqual(timing.advance_overruns, 1)

    def test_failed_advance_preserves_partial_step_progress(self):
        station = ready_station(StepClock())
        self.addCleanup(station.close)
        for _ in range(3):
            station.advance_control_tick()
        self.assertEqual(station.status().step, 90)

        class Boom(RuntimeError):
            pass

        def boom(tick):
            raise Boom("capture failed")

        station._capture = boom
        with self.assertRaises(Boom):
            station.advance_control_tick()
        status = station.status()
        self.assertEqual(status.step, 100)
        timing = status.timing
        self.assertEqual(timing.advances_started, 4)
        self.assertEqual(timing.advances_completed, 3)
        self.assertEqual(timing.advances_failed, 1)
        self.assertEqual(timing.advance_wall_s.missing, 1)
        self.assertEqual(timing.control_wall_s.count, 4)
        self.assertEqual(timing.control_wall_s.missing, 0)

    def test_reply_is_unprocessed_when_control_exits_before_validation(self):
        station = ready_station(ManualClock())
        self.addCleanup(station.close)
        station.stop()
        dummy = Capture(0, 0.0, (0.0, 0.0, 0.0), b"x")
        station.advance_control_tick(
            replies=(Reply(dummy, (0.0, 0.0, 0.0)),)
        )
        timing = station.status().timing
        self.assertEqual(timing.replies_offered, 1)
        self.assertEqual(timing.replies_unprocessed, 1)
        self.assertEqual(timing.accepted_reply_sim_s.count, 0)
        self.assertEqual(timing.rejected_reply_sim_s.count, 0)
        self.assertEqual(timing.accepted_reply_sim_s.missing, 0)
        self.assertEqual(timing.rejected_reply_sim_s.missing, 0)


class ReplyTimingTests(unittest.TestCase):
    def test_accepted_and_repeated_reply_ages_use_stored_capture(self):
        clock = ManualClock(100.0)
        station = ready_station(clock)
        self.addCleanup(station.close)
        capture = first_capture(station)
        clock.value = 102.0
        station.advance_control_tick(
            replies=(Reply(capture, target_from(capture, (0.01, 0.01, 0.01))),)
        )
        timing = station.status().timing
        self.assertEqual(timing.accepted_reply_sim_s.count, 1)
        self.assertAlmostEqual(timing.accepted_reply_sim_s.last_s, 30 / 3000)
        self.assertEqual(timing.accepted_reply_wall_s.count, 1)
        self.assertEqual(timing.accepted_reply_wall_s.last_s, 2.0)
        self.assertEqual(timing.rejected_reply_sim_s.count, 0)
        self.assertEqual(timing.callback_period_wall_s.last_s, 2.0)

        clock.value = 103.0
        station.advance_control_tick(
            replies=(Reply(capture, target_from(capture, (0.01, 0.01, 0.01))),)
        )
        status = station.status()
        timing = status.timing
        self.assertEqual(status.replies_accepted, 1)
        self.assertEqual(status.replies_rejected, 1)
        self.assertEqual(timing.rejected_reply_sim_s.count, 1)
        self.assertAlmostEqual(timing.rejected_reply_sim_s.last_s, 60 / 3000)
        self.assertEqual(timing.rejected_reply_wall_s.count, 1)
        self.assertEqual(timing.rejected_reply_wall_s.last_s, 3.0)
        self.assertEqual(
            timing.accepted_reply_sim_s.count
            + timing.accepted_reply_sim_s.missing,
            status.replies_accepted,
        )
        self.assertEqual(
            timing.rejected_reply_sim_s.count
            + timing.rejected_reply_sim_s.missing,
            status.replies_rejected,
        )

    def test_forged_unknown_and_expired_captures(self):
        clock = ManualClock()
        station = ready_station(clock)
        self.addCleanup(station.close)
        capture = first_capture(station)
        forged = Capture(
            capture.sequence, capture.captured_at_s, capture.q, b"forged"
        )
        unknown = Capture(999, 0.0, (0.0, 0.0, 0.0), b"unknown")
        clock.value = 2.0
        station.advance_control_tick(
            replies=(
                Reply(forged, (0.0, 0.0, 0.0)),
                Reply(unknown, (0.0, 0.0, 0.0)),
            )
        )
        timing = station.status().timing
        self.assertEqual(timing.replies_offered, 2)
        self.assertEqual(timing.replies_unprocessed, 0)
        self.assertEqual(timing.rejected_reply_sim_s.count, 0)
        self.assertEqual(timing.rejected_reply_sim_s.missing, 2)
        self.assertEqual(timing.rejected_reply_wall_s.count, 0)
        self.assertEqual(timing.rejected_reply_wall_s.missing, 2)

        expired_clock = ManualClock()
        expired_station = ready_station(expired_clock)
        self.addCleanup(expired_station.close)
        expired_capture = first_capture(expired_station)
        for _ in range(24):
            expired_station.advance_control_tick()
        self.assertEqual(expired_station.status().step, 750)
        expired_clock.value = 2.0
        expired_station.advance_control_tick(
            replies=(
                Reply(
                    expired_capture,
                    target_from(expired_capture, (0.01, 0.01, 0.01)),
                ),
            )
        )
        status = expired_station.status()
        timing = status.timing
        self.assertIs(status.state, State.ARMED)
        self.assertEqual(status.replies_rejected, 1)
        self.assertEqual(timing.rejected_reply_sim_s.count, 1)
        self.assertAlmostEqual(timing.rejected_reply_sim_s.last_s, 750 / 3000)
        self.assertEqual(timing.rejected_reply_wall_s.count, 1)
        self.assertEqual(timing.rejected_reply_wall_s.last_s, 2.0)


class FeedbackAgeTests(unittest.TestCase):
    def test_peak_is_observed_before_refresh_not_configured_maximum(self):
        station = ready_station(ManualClock())
        self.addCleanup(station.close)
        station.advance_control_tick()
        original_receive = station._receive_feedback
        skipped = [False]

        def skip_once():
            if not skipped[0]:
                skipped[0] = True
                return True
            return original_receive()

        station._receive_feedback = skip_once
        station.advance_control_tick()
        station.advance_control_tick()
        timing = station.status().timing
        self.assertAlmostEqual(
            timing.feedback_peak_observed_sim_s, 60 / 3000, places=12
        )
        self.assertNotEqual(timing.feedback_peak_observed_sim_s, FEEDBACK_MAX_AGE_S)
        self.assertAlmostEqual(timing.feedback_age_sim_s[0], 30 / 3000, places=12)
        self.assertGreater(
            timing.feedback_peak_observed_sim_s, timing.feedback_age_sim_s[0]
        )
        self.assertEqual(len(timing.feedback_age_sim_s), 3)
        self.assertTrue(
            all(age is not None for age in timing.feedback_age_sim_s)
        )

        station.disconnect()
        timing = station.status().timing
        self.assertEqual(timing.feedback_age_sim_s, (None, None, None))
        self.assertAlmostEqual(
            timing.feedback_peak_observed_sim_s, 60 / 3000, places=12
        )


class ClockSafetyTests(unittest.TestCase):
    def test_throwing_clock_keeps_stepping_and_authority_unchanged(self):
        def clock():
            raise RuntimeError("clock unavailable")

        station = ready_station(clock)
        self.addCleanup(station.close)
        station.advance_control_tick()
        timing = station.status().timing
        self.assertEqual(timing.clock_errors, 5)
        self.assertEqual(timing.advance_wall_s.count, 0)
        self.assertEqual(timing.advance_wall_s.missing, 1)
        self.assertEqual(timing.control_wall_s.count, 0)
        self.assertEqual(timing.control_wall_s.missing, 1)
        self.assertIs(station.status().state, State.ARMED)
        self.assertFalse(station.status().actuation_enabled)

        for _ in range(99):
            station.advance_control_tick()
        status = station.status()
        self.assertEqual(status.step, 3000)
        self.assertEqual(status.control_ticks, 100)
        self.assertEqual(status.camera_ticks, 30)
        self.assertEqual(status.captures_emitted, 30)
        self.assertIs(status.state, State.ARMED)
        self.assertFalse(status.actuation_enabled)
        self.assertEqual(status.timing.advances_started, 100)
        self.assertEqual(status.timing.advances_completed, 100)
        self.assertEqual(status.timing.advances_failed, 0)
        self.assertEqual(status.timing.advance_wall_s.count, 0)
        self.assertEqual(status.timing.advance_wall_s.missing, 100)
        self.assertEqual(status.timing.control_wall_s.count, 0)
        self.assertEqual(status.timing.control_wall_s.missing, 100)

    def test_invalid_clock_values_invalidate_but_do_not_halt(self):
        for bad in (True, float("nan"), float("inf"), -1.0, "bad"):
            with self.subTest(bad=bad):
                station = ready_station(lambda bad=bad: bad)
                try:
                    station.advance_control_tick()
                    timing = station.status().timing
                    self.assertEqual(timing.clock_errors, 5)
                    self.assertEqual(timing.advance_wall_s.count, 0)
                    self.assertEqual(timing.advance_wall_s.missing, 1)
                    self.assertEqual(timing.control_wall_s.count, 0)
                    self.assertEqual(timing.control_wall_s.missing, 1)
                    self.assertIs(station.status().state, State.ARMED)
                    self.assertEqual(station.status().step, 30)
                finally:
                    station.close()

    def test_regressing_clock_invalidates_then_recovers_adjacency(self):
        clock = ManualClock(10.0)
        station = ready_station(clock)
        self.addCleanup(station.close)
        station.advance_control_tick()
        clock.value = 5.0
        station.advance_control_tick()
        timing = station.status().timing
        self.assertEqual(timing.clock_errors, 1)
        self.assertEqual(timing.callback_period_wall_s.missing, 1)
        self.assertEqual(timing.advance_wall_s.missing, 1)
        self.assertEqual(timing.control_wall_s.count, 2)
        self.assertIs(station.status().state, State.ARMED)

        clock.value = 6.0
        station.advance_control_tick()
        self.assertEqual(
            station.status().timing.callback_period_wall_s.missing, 2
        )
        clock.value = 7.0
        station.advance_control_tick()
        timing = station.status().timing
        self.assertEqual(timing.callback_period_wall_s.count, 1)
        self.assertEqual(timing.callback_period_wall_s.last_s, 1.0)
        self.assertEqual(timing.advance_wall_s.count, 3)
        self.assertEqual(timing.clock_errors, 1)
        self.assertIs(station.status().state, State.ARMED)

    def test_epoch_recovery_durations_keep_snapshot_mean_finite(self):
        clock = QueueClock(
            [0.0, 0.0, 0.0, 0.0, 1e308]
            + [float("nan")] * 4
            + [0.0, 0.0, 0.0, 1e308]
        )
        station = ready_station(clock)
        self.addCleanup(station.close)
        for _ in range(3):
            station.advance_control_tick()
        self.assertEqual(clock.index, len(clock.values))
        stats = station.status().timing.advance_wall_s
        self.assertEqual(stats.count, 2)
        self.assertEqual(stats.missing, 1)
        self.assertEqual(stats.min_s, 1e308)
        self.assertEqual(stats.max_s, 1e308)
        self.assertEqual(stats.mean_s, 1e308)
        self.assertTrue(math.isfinite(stats.mean_s))

    def test_reentrant_clock_calls_are_rejected_and_invalidate_sample(self):
        holder = {}
        attempts = []

        def clock():
            for name, call in (
                ("status", lambda: holder["station"].status()),
                ("check", lambda: holder["station"].check()),
                ("arm", lambda: holder["station"].arm(owner="intruder")),
                ("stop", lambda: holder["station"].stop()),
                ("disconnect", lambda: holder["station"].disconnect()),
                (
                    "set_marker",
                    lambda: holder["station"].set_marker(
                        scene.SCENARIO.probes[0].marker_xy
                    ),
                ),
                ("close", lambda: holder["station"].close()),
            ):
                try:
                    call()
                except RuntimeError:
                    attempts.append(name)
                else:
                    attempts.append("missed:" + name)
            return 1.0

        station = ready_station(clock)
        holder["station"] = station
        self.addCleanup(station.close)
        station.advance_control_tick()
        status = station.status()
        self.assertTrue(attempts)
        self.assertFalse(any(item.startswith("missed:") for item in attempts))
        self.assertIs(status.state, State.ARMED)
        self.assertFalse(status.actuation_enabled)
        self.assertEqual(status.step, 30)
        self.assertEqual(status.timing.clock_errors, 5)
        self.assertEqual(status.timing.advance_wall_s.count, 0)
        self.assertEqual(status.timing.advance_wall_s.missing, 1)
        self.assertEqual(status.timing.control_wall_s.count, 0)
        self.assertEqual(status.timing.control_wall_s.missing, 1)
        self.assertEqual(station.advance_control_tick(), ())

    def test_bad_clock_does_not_mask_expiry_or_final_release_veto(self):
        def clock():
            raise RuntimeError("clock unavailable")

        station = ready_station(clock)
        self.addCleanup(station.close)
        capture = first_capture(station)
        for _ in range(24):
            station.advance_control_tick()
        self.assertEqual(station.status().step, 750)
        station.advance_control_tick(
            replies=(Reply(capture, target_from(capture, (0.01, 0.01, 0.01))),)
        )
        status = station.status()
        self.assertIs(status.state, State.ARMED)
        self.assertEqual(status.replies_rejected, 1)
        self.assertEqual(status.references_issued, 0)
        self.assertFalse(status.actuation_enabled)
        self.assertEqual(status.timing.rejected_reply_wall_s.count, 0)
        self.assertEqual(status.timing.rejected_reply_wall_s.missing, 1)

        veto_station = ready_station(clock)
        self.addCleanup(veto_station.close)
        veto_capture = first_capture(veto_station)
        original_release = veto_station._release

        def release(reference, intent, target, tick):
            veto_station._model.body_mass[1] += 0.05
            return original_release(reference, intent, target, tick)

        veto_station._release = release
        veto_station.advance_control_tick(
            replies=(
                Reply(
                    veto_capture,
                    target_from(veto_capture, (0.01, 0.01, 0.01)),
                ),
            )
        )
        status = veto_station.status()
        self.assertIs(status.state, State.FAULT)
        self.assertTrue(status.fault_latched)
        self.assertEqual(status.references_issued, 0)
        self.assertFalse(status.actuation_enabled)
        self.assertEqual(status.timing.accepted_reply_sim_s.count, 1)


class ClockConversionTests(unittest.TestCase):
    def test_value_conversion_cannot_reenter_or_mutate_station(self):
        marker_xy = scene.SCENARIO.probes[0].marker_xy
        for base in (float, int):
            with self.subTest(base=base.__name__):
                holder = {}
                attempts = []
                busy = [False]

                def action():
                    if busy[0]:
                        return
                    busy[0] = True
                    try:
                        station = holder["station"]
                        calls = (
                            ("status", station.status),
                            ("check", station.check),
                            ("arm", lambda: station.arm(owner="conversion")),
                            ("advance", station.advance_control_tick),
                            ("stop", station.stop),
                            ("disconnect", station.disconnect),
                            ("set_marker", lambda: station.set_marker(marker_xy)),
                            ("close", station.close),
                        )
                        for name, call in calls:
                            try:
                                call()
                            except RuntimeError:
                                attempts.append(name)
                            else:
                                attempts.append("missed:" + name)
                    finally:
                        busy[0] = False

                class Value(base):
                    def __new__(cls):
                        obj = base.__new__(cls, 1.0)
                        obj._value = 1.0
                        return obj

                    def __float__(self):
                        try:
                            action()
                        except RuntimeError:
                            pass
                        return self._value

                value = Value()
                station = ready_station(lambda: value)
                holder["station"] = station
                self.addCleanup(station.close)
                marker_before = tuple(
                    float(item)
                    for item in station._model.geom_pos[station._marker_id, :2]
                )
                station.advance_control_tick()
                status = station.status()
                marker_after = tuple(
                    float(item)
                    for item in station._model.geom_pos[station._marker_id, :2]
                )
                self.assertIs(status.state, State.ARMED)
                self.assertFalse(status.actuation_enabled)
                self.assertFalse(station._closed)
                self.assertEqual(marker_after, marker_before)
                self.assertEqual(status.step, 30)
                self.assertTrue(attempts)
                self.assertFalse(
                    any(item.startswith("missed:") for item in attempts)
                )
                self.assertEqual(status.timing.clock_errors, 5)
                self.assertEqual(status.timing.advance_wall_s.missing, 1)
                self.assertEqual(status.timing.control_wall_s.missing, 1)

    def test_throwing_conversion_invalidates_but_keeps_stepping(self):
        for base in (float, int):
            with self.subTest(base=base.__name__):

                class Value(base):
                    def __new__(cls):
                        return base.__new__(cls, 1.0)

                    def __float__(self):
                        raise RuntimeError("diagnostic conversion failed")

                value = Value()
                station = ready_station(lambda: value)
                self.addCleanup(station.close)
                station.advance_control_tick()
                status = station.status()
                self.assertIs(status.state, State.ARMED)
                self.assertFalse(status.actuation_enabled)
                self.assertEqual(status.step, 30)
                self.assertEqual(status.timing.clock_errors, 5)
                self.assertEqual(status.timing.advance_wall_s.count, 0)
                self.assertEqual(status.timing.advance_wall_s.missing, 1)
                self.assertEqual(status.timing.control_wall_s.missing, 1)
                self.assertEqual(status.timing.advances_completed, 1)
                self.assertEqual(status.timing.advances_failed, 0)

    def test_plain_int_and_float_clock_values_remain_valid(self):
        for base in (int, float):
            with self.subTest(base=base.__name__):
                counter = [0]

                def clock(base=base, counter=counter):
                    counter[0] += 1
                    return base(counter[0])

                station = ready_station(clock)
                self.addCleanup(station.close)
                station.advance_control_tick()
                status = station.status()
                self.assertEqual(status.timing.clock_errors, 0)
                self.assertEqual(status.timing.advance_wall_s.count, 1)
                self.assertEqual(status.timing.control_wall_s.count, 1)
                self.assertEqual(status.step, 30)


class ReleaseBoundaryTests(unittest.TestCase):
    def test_no_clock_callback_between_release_validation_and_actuation(self):
        inside = {"release": False, "violation": False}
        raw = ManualClock()

        def clock():
            if inside["release"]:
                inside["violation"] = True
            return raw()

        station = ready_station(clock)
        self.addCleanup(station.close)
        capture = first_capture(station)
        original_release = station._release

        def release(*args):
            inside["release"] = True
            try:
                return original_release(*args)
            finally:
                inside["release"] = False

        station._release = release
        station.advance_control_tick(
            replies=(Reply(capture, target_from(capture, (0.01, 0.01, 0.01))),)
        )
        status = station.status()
        self.assertIsNotNone(status.last_issued)
        self.assertGreaterEqual(status.references_issued, 1)
        self.assertFalse(inside["violation"])


if __name__ == "__main__":
    unittest.main()
