import dataclasses
import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from unittest import mock

os.environ.setdefault("MUJOCO_GL", "egl")

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from examples.rolling_reach_arm import process_runtime  # noqa: E402
from examples.rolling_reach_arm.process_runtime import (  # noqa: E402
    CHILD_EXIT_GRACE_S,
    LEASE_S,
    ProcessStation,
    ProcessStatus,
)
from im6014_control.records import State  # noqa: E402


FIXTURE = ROOT / "tests" / "fixtures" / "rolling_process_client.py"


def child_argv(mode="normal"):
    return [sys.executable, "-B", str(FIXTURE), "--mode", mode]


def assert_reaped(test, pid):
    with test.assertRaises(ProcessLookupError):
        os.kill(pid, 0)


class ShortSendSocket:

    def __init__(self, sock, limit=8):
        self._sock = sock
        self._limit = limit
        self.partial_writes = 0

    def send(self, data):
        if self.partial_writes == 0 and len(data) > self._limit:
            self.partial_writes += 1
            return self._sock.send(bytes(data[:self._limit]))
        return self._sock.send(data)

    def __getattr__(self, name):
        return getattr(self._sock, name)


class ProcessRuntimeTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="rolling-process-")
        self.addCleanup(self.temp.cleanup)
        self.lock = Path(self.temp.name) / "owner.lock"

    def runtime(self):
        runtime = ProcessStation(lock_path=self.lock)
        self.addCleanup(runtime.close)
        return runtime

    def armed_runtime(self, mode="normal"):
        runtime = self.runtime()
        self.assertTrue(runtime.spawn_client(child_argv(mode)))
        self.assertTrue(runtime.check().accepted)
        self.assertTrue(runtime.arm("process-test").accepted)
        return runtime

    def test_explicit_lifecycle_and_capture_echo(self):
        runtime = self.runtime()
        initial = runtime.status()
        self.assertIsInstance(initial, ProcessStatus)
        self.assertIs(initial.station.state, State.DISCONNECTED)
        self.assertFalse(initial.connected)
        self.assertFalse(initial.handshaken)
        self.assertIsNone(initial.session_id)
        self.assertIsNone(initial.child_pid)
        self.assertFalse(initial.pending_notice)
        with self.assertRaises(dataclasses.FrozenInstanceError):
            initial.arm_epoch = 99
        for forbidden in ("model", "core", "ownership", "_station"):
            self.assertFalse(hasattr(initial, forbidden))

        self.assertTrue(runtime.spawn_client(child_argv()))
        status = runtime.status()
        self.assertTrue(status.connected)
        self.assertTrue(status.handshaken)
        self.assertTrue(status.child_running)
        self.assertIsInstance(status.child_pid, int)
        self.assertIsInstance(status.session_id, str)
        self.assertGreaterEqual(status.acknowledgements, 1)

        self.assertTrue(runtime.check().accepted)
        self.assertIs(runtime.status().station.state, State.READY)
        self.assertTrue(runtime.arm("capture").accepted)
        self.assertIs(runtime.status().station.state, State.ARMED)

        runtime.run_for(0.25)
        status = runtime.status()
        self.assertTrue(status.connected)
        self.assertGreater(status.captures_exported, 0)
        self.assertGreater(status.replies_submitted, 0)
        self.assertGreater(status.station.references_issued, 0)
        self.assertTrue(status.station.actuation_enabled)
        self.assertIsNotNone(status.last_child_q)
        self.assertEqual(len(status.last_child_q), 3)
        self.assertLessEqual(status.capture_correlations, 8)

    def test_second_child_rejected_and_killed_child_replaced(self):
        runtime = self.runtime()
        self.assertTrue(runtime.spawn_client(child_argv()))
        first = runtime.status()
        self.assertFalse(runtime.spawn_client(child_argv()))
        self.assertEqual(runtime.status().child_pid, first.child_pid)

        self.assertTrue(runtime.check().accepted)
        self.assertTrue(runtime.arm("first").accepted)
        runtime.run_for(0.05)
        os.kill(first.child_pid, signal.SIGKILL)
        time.sleep(0.02)
        self.assertTrue(runtime.spawn_client(child_argv()))
        assert_reaped(self, first.child_pid)

        replaced = runtime.status()
        self.assertTrue(replaced.connected)
        self.assertNotEqual(replaced.session_id, first.session_id)
        self.assertTrue(runtime.check().accepted)
        self.assertTrue(runtime.arm("second").accepted)
        runtime.run_for(0.05)
        self.assertTrue(runtime.status().connected)

    def test_expired_child_is_replaced_without_auto_restart(self):
        runtime = self.runtime()
        self.assertTrue(runtime.spawn_client(child_argv("silent")))
        first = runtime.status()
        time.sleep(LEASE_S + 0.05)
        self.assertTrue(runtime.spawn_client(child_argv()))
        assert_reaped(self, first.child_pid)

        second = runtime.status()
        self.assertTrue(second.connected)
        self.assertNotEqual(second.session_id, first.session_id)
        self.assertGreaterEqual(second.lease_expirations, 1)
        self.assertTrue(runtime.check().accepted)
        self.assertTrue(runtime.arm("expired").accepted)

    def test_killed_child_disconnects_and_is_reaped(self):
        runtime = self.armed_runtime()
        runtime.run_for(0.05)
        pid = runtime.status().child_pid
        os.kill(pid, signal.SIGKILL)
        time.sleep(0.02)
        runtime.run_for(1e-9)

        status = runtime.status()
        self.assertFalse(status.connected)
        self.assertIsNone(status.session_id)
        self.assertIsNone(status.child_pid)
        self.assertFalse(status.child_running)
        self.assertGreaterEqual(status.disconnects, 1)
        assert_reaped(self, pid)
        runtime.run_for(1e-9)
        self.assertFalse(runtime.status().connected)

    def test_sigstop_lease_expires_and_explicit_restart_is_disconnected(self):
        runtime = self.armed_runtime()
        runtime.run_for(0.05)
        pid = runtime.status().child_pid
        os.kill(pid, signal.SIGSTOP)

        started = time.monotonic()
        runtime.run_for(LEASE_S + 0.3)
        elapsed = time.monotonic() - started
        status = runtime.status()
        self.assertLess(elapsed, 3.0)
        self.assertFalse(status.connected)
        self.assertIsNone(status.session_id)
        self.assertFalse(status.child_running)
        self.assertGreaterEqual(status.lease_expirations, 1)
        self.assertIs(status.station.state, State.DISCONNECTED)
        try:
            os.kill(pid, signal.SIGCONT)
        except ProcessLookupError:
            pass

        runtime.run_for(0.05)
        self.assertFalse(runtime.status().connected)
        self.assertGreaterEqual(runtime.status().disconnects, 1)

        self.assertTrue(runtime.spawn_client(child_argv()))
        self.assertTrue(runtime.check().accepted)
        self.assertTrue(runtime.arm("restart").accepted)
        runtime.run_for(0.05)
        self.assertTrue(runtime.status().connected)

    def test_replayed_heartbeats_cannot_renew_a_hung_lease(self):
        runtime = self.runtime()
        self.assertTrue(runtime.spawn_client(child_argv("replay")))
        runtime.run_for(LEASE_S + 0.3)

        status = runtime.status()
        self.assertFalse(status.connected)
        self.assertGreaterEqual(status.lease_expirations, 1)
        self.assertGreaterEqual(status.frames_stale, 1)
        self.assertFalse(status.child_running)

    def test_rearmed_ticket_replay_is_rejected_without_authority(self):
        runtime = self.armed_runtime("ticket-replay")
        runtime.run_for(0.2)
        before = runtime.status()
        self.assertGreater(before.captures_exported, 0)

        runtime.stop()
        self.assertIsNotNone(runtime.status().session_id)
        self.assertTrue(runtime.check().accepted)
        self.assertTrue(runtime.arm("rearmed").accepted)
        runtime.run_for(0.2)

        after = runtime.status()
        self.assertTrue(after.connected)
        self.assertGreater(after.replies_rejected, before.replies_rejected)
        self.assertGreater(after.replies_submitted, 0)
        self.assertLessEqual(after.capture_correlations, 8)

    def test_stalled_reader_and_flood_do_not_block_the_owner(self):
        stalled = self.runtime()
        self.assertTrue(stalled.spawn_client(child_argv("stall")))
        started = time.monotonic()
        stalled.run_for(LEASE_S + 0.3)
        self.assertLess(time.monotonic() - started, 3.0)
        stalled_status = stalled.status()
        self.assertFalse(stalled_status.connected)
        self.assertGreaterEqual(stalled_status.lease_expirations, 1)
        self.assertFalse(stalled_status.child_running)

        flooded = ProcessStation(lock_path=Path(self.temp.name) / "flood.lock")
        self.addCleanup(flooded.close)
        self.assertTrue(flooded.spawn_client(child_argv("flood")))
        started = time.monotonic()
        flooded.run_for(0.1)
        self.assertLess(time.monotonic() - started, 3.0)
        flooded_status = flooded.status()
        self.assertFalse(flooded_status.connected)
        self.assertGreaterEqual(flooded_status.protocol_faults, 1)

    def test_lifecycle_frames_from_the_child_are_rejected(self):
        runtime = self.runtime()
        self.assertTrue(runtime.spawn_client(child_argv("lifecycle")))
        runtime.run_for(0.05)
        status = runtime.status()
        self.assertFalse(status.connected)
        self.assertGreaterEqual(status.protocol_faults, 1)

    def test_duplicate_profile_owner_is_rejected_before_model_construction(self):
        owner = self.runtime()
        with mock.patch.object(process_runtime, "RollingStation") as constructor:
            constructor.side_effect = AssertionError("model was constructed")
            with self.assertRaises(RuntimeError):
                ProcessStation(lock_path=self.lock)
            constructor.assert_not_called()
        self.assertIs(owner.status().station.state, State.DISCONNECTED)

    def test_profile_lock_symlink_fails_safely(self):
        target = Path(self.temp.name) / "real.lock"
        target.write_text("", encoding="utf-8")
        link = Path(self.temp.name) / "link.lock"
        link.symlink_to(target)
        with self.assertRaises(RuntimeError):
            ProcessStation(lock_path=link)

    def test_close_reaps_child_and_explicit_restart_starts_disconnected(self):
        first = ProcessStation(lock_path=self.lock)
        self.addCleanup(first.close)
        self.assertTrue(first.spawn_client(child_argv()))
        self.assertTrue(first.check().accepted)
        self.assertTrue(first.arm("before-close").accepted)
        first.run_for(0.05)
        pid = first.status().child_pid
        first.close()
        assert_reaped(self, pid)

        second = ProcessStation(lock_path=self.lock)
        self.addCleanup(second.close)
        status = second.status()
        self.assertIs(status.station.state, State.DISCONNECTED)
        self.assertFalse(status.connected)
        self.assertIsNone(status.session_id)
        self.assertIsNone(status.child_pid)
        second.run_for(1e-9)
        self.assertGreater(second.status().station.step, 0)
        self.assertFalse(second.status().connected)

        self.assertTrue(second.spawn_client(child_argv()))
        self.assertTrue(second.check().accepted)
        self.assertTrue(second.arm("after-close").accepted)
        second.run_for(0.05)
        self.assertTrue(second.status().connected)

    def test_unhashable_wire_type_disconnects_without_escaping_owner_loop(self):
        for mode in ("bad-type-list", "bad-type-dict"):
            with self.subTest(mode=mode):
                runtime = self.armed_runtime(mode)
                try:
                    before = runtime.status()
                    self.assertIs(before.station.state, State.ARMED)
                    runtime.run_for(0.2)
                    status = runtime.status()
                    self.assertIs(status.station.state, State.DISCONNECTED)
                    self.assertFalse(status.connected)
                    self.assertFalse(status.handshaken)
                    self.assertIsNone(status.session_id)
                    self.assertGreaterEqual(status.protocol_faults, 1)
                    self.assertGreater(status.station.step, before.station.step)
                    runtime.run_for(1e-9)
                finally:
                    runtime.close()

    def test_handshake_rejects_boolean_epoch(self):
        runtime = self.runtime()
        with self.assertRaises(RuntimeError):
            runtime.spawn_client(child_argv("bool-epoch"))
        status = runtime.status()
        self.assertFalse(status.connected)
        self.assertFalse(status.handshaken)
        self.assertIsNone(status.session_id)
        self.assertIsNone(status.child_pid)

    def test_partial_control_frame_is_discarded_when_link_is_lost(self):
        runtime = self.runtime()
        self.assertTrue(runtime.spawn_client(child_argv()))
        self.assertTrue(runtime.check().accepted)
        self.assertTrue(runtime.arm("partial").accepted)
        first = runtime.status()
        session = runtime._active
        self.assertIsNotNone(session)
        wrapper = ShortSendSocket(session.sock, limit=8)
        session.sock = wrapper

        runtime.stop()
        self.assertGreaterEqual(wrapper.partial_writes, 1)
        self.assertGreater(len(session.outgoing), 0)

        os.kill(first.child_pid, signal.SIGKILL)
        time.sleep(0.02)
        self.assertTrue(runtime.spawn_client(child_argv()))
        replaced = runtime.status()
        self.assertTrue(replaced.connected)
        self.assertTrue(replaced.handshaken)
        self.assertIsNotNone(replaced.session_id)
        self.assertNotEqual(replaced.session_id, first.session_id)
        self.assertNotEqual(replaced.child_pid, first.child_pid)
        assert_reaped(self, first.child_pid)
        self.assertTrue(runtime.check().accepted)
        self.assertTrue(runtime.arm("replaced").accepted)
        runtime.run_for(0.05)
        self.assertTrue(runtime.status().connected)

    def test_partial_control_frame_completes_on_a_live_link(self):
        runtime = self.armed_runtime()
        session = runtime._active
        self.assertIsNotNone(session)
        wrapper = ShortSendSocket(session.sock, limit=8)
        session.sock = wrapper

        runtime.stop()
        self.assertGreaterEqual(wrapper.partial_writes, 1)
        self.assertGreater(len(session.outgoing), 0)
        before = runtime.status()

        runtime.run_for(0.1)
        status = runtime.status()
        self.assertTrue(status.connected)
        self.assertTrue(status.handshaken)
        self.assertIs(status.station.state, State.STOPPED)
        self.assertEqual(status.session_id, before.session_id)
        self.assertGreater(status.frames_sent, before.frames_sent)

    def test_ignore_term_child_is_reaped_while_advances_continue(self):
        runtime = self.runtime()
        self.assertTrue(runtime.spawn_client(child_argv("ignore-term")))
        process = runtime._active.child
        self.assertTrue(runtime.check().accepted)
        self.assertTrue(runtime.arm("ignore-term").accepted)
        runtime.run_for(0.05)

        advances = []
        terminate_at = []
        original_advance = runtime._station.advance_control_tick

        def observe_advance(*args, **kwargs):
            result = original_advance(*args, **kwargs)
            advances.append((time.monotonic(), process.poll() is None))
            return result

        runtime._station.advance_control_tick = observe_advance
        real_kill = os.kill

        def record_kill(pid, sig):
            if pid == process.pid and sig == signal.SIGTERM:
                terminate_at.append(time.monotonic())
            return real_kill(pid, sig)

        started = time.monotonic()
        with mock.patch.object(process_runtime.os, "kill", side_effect=record_kill):
            runtime.run_for(LEASE_S + CHILD_EXIT_GRACE_S + 0.25)
        elapsed = time.monotonic() - started
        self.assertLess(elapsed, 3.0)
        self.assertEqual(len(terminate_at), 1)

        alive_during_grace = [
            stamp for stamp, alive in advances
            if alive and stamp >= terminate_at[0]
        ]
        self.assertGreaterEqual(len(alive_during_grace), 2)

        status = runtime.status()
        self.assertFalse(status.connected)
        self.assertFalse(status.handshaken)
        self.assertIsNone(status.child_pid)
        self.assertFalse(status.child_running)
        self.assertGreaterEqual(status.lease_expirations, 1)
        assert_reaped(self, process.pid)

    def test_close_reaps_live_stopped_and_term_ignoring_children(self):
        for case in ("alive", "sigstop", "ignore-term"):
            with self.subTest(case=case):
                lock = Path(self.temp.name) / f"close-{case}.lock"
                runtime = ProcessStation(lock_path=lock)
                try:
                    mode = "ignore-term" if case == "ignore-term" else "normal"
                    self.assertTrue(runtime.spawn_client(child_argv(mode)))
                    pid = runtime.status().child_pid
                    if case == "sigstop":
                        os.kill(pid, signal.SIGSTOP)
                    started = time.monotonic()
                    runtime.close()
                    self.assertLess(time.monotonic() - started, 3.0)
                    assert_reaped(self, pid)
                finally:
                    runtime.close()

    def test_process_level_profile_lock_and_abrupt_owner_death(self):
        script = (
            "import sys, time\n"
            "sys.path.insert(0, sys.argv[2])\n"
            "from examples.rolling_reach_arm.process_runtime import ProcessStation\n"
            "station = ProcessStation(lock_path=sys.argv[1])\n"
            "print('ready', flush=True)\n"
            "while True:\n"
            "    time.sleep(0.05)\n"
        )
        owner = subprocess.Popen(
            [sys.executable, "-c", script, str(self.lock), str(ROOT)],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        try:
            ready, _, _ = select.select([owner.stdout], [], [], 30.0)
            if not ready:
                owner.kill()
                owner.wait(timeout=5.0)
                self.fail(f"inline owner did not start: {owner.stderr.read()}")
            self.assertEqual(owner.stdout.readline().strip(), "ready")
            with self.assertRaises(RuntimeError):
                ProcessStation(lock_path=self.lock)

            os.kill(owner.pid, signal.SIGKILL)
            owner.wait(timeout=5.0)

            restarted = ProcessStation(lock_path=self.lock)
            try:
                status = restarted.status()
                self.assertIs(status.station.state, State.DISCONNECTED)
                self.assertFalse(status.connected)
                self.assertFalse(status.handshaken)
                self.assertIsNone(status.session_id)
                self.assertIsNone(status.child_pid)
                restarted.run_for(1e-9)
                self.assertGreater(restarted.status().station.step, 0)
            finally:
                restarted.close()
        finally:
            if owner.poll() is None:
                owner.kill()
                owner.wait(timeout=5.0)
            for stream in (owner.stdout, owner.stderr):
                if stream is not None:
                    stream.close()


if __name__ == "__main__":
    unittest.main()
