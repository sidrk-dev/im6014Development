"""Scoped local process fixture for the single-writer rolling station.

The owning process keeps sole authority over one :class:`RollingStation`. A
disposable child only receives metadata-only capture notices and returns
proposed vectors. The child wire has no check, arm, or other lifecycle
commands, and every frame is a bounded length-prefixed JSON object.

The initial handshake is a deliberately synchronous, bounded startup step. It
is not part of uninterrupted stepping; callers should not expect physics to
advance while a new child is being started and authenticated.

This is a local fake/simulation fixture. It makes no hosted, Qt, learned, or
physical-motor claim.
"""

from __future__ import annotations

from collections.abc import Sequence
from contextlib import suppress
from dataclasses import dataclass, field
import fcntl
import json
import math
import os
from pathlib import Path
import secrets
import select
import signal
import socket
import stat
import subprocess
import time

from im6014_control.records import Decision, DecisionCode, State

from .station import Capture, Reply, RollingStation, StationStatus


MAX_BODY_BYTES = 1024
HEADER_BYTES = 4
MAX_INPUT_BYTES = HEADER_BYTES + MAX_BODY_BYTES
MAX_FRAMES_PER_BOUNDARY = 8
MAX_READ_BYTES_PER_BOUNDARY = 8 * 1024
MAX_OUTPUT_FRAMES_PER_FLUSH = 2
MAX_CAPTURE_CORRELATIONS = 8
LEASE_S = 0.300
ADVANCE_OPPORTUNITY_S = 0.010
HANDSHAKE_TIMEOUT_S = 2.0
CHILD_EXIT_GRACE_S = 0.250
CHILD_KILL_GRACE_S = 1.0

_ALLOWED_CHILD_TYPES = frozenset({"hello_ack", "ack", "reply"})


class _ProtocolError(Exception):
    pass


@dataclass(frozen=True)
class ProcessStatus:
    """Frozen process diagnostics without a raw station, model, or core."""

    station: StationStatus
    session_id: str | None
    arm_epoch: int
    connected: bool
    handshaken: bool
    child_pid: int | None
    child_running: bool
    last_child_q: tuple | None
    frames_received: int
    frames_sent: int
    frames_stale: int
    protocol_faults: int
    acknowledgements: int
    captures_exported: int
    capture_correlations: int
    replies_submitted: int
    replies_rejected: int
    lease_expirations: int
    disconnects: int
    child_replacements: int
    pending_notice: bool


def _private_lock_directory() -> Path:
    base = os.environ.get("XDG_STATE_HOME")
    root = Path(base) if base else Path.home() / ".local" / "state"
    return root / "im6014" / f"rolling-{os.getuid()}"


def _ensure_private_directory(path: Path) -> None:
    try:
        info = os.lstat(path)
    except FileNotFoundError:
        try:
            path.mkdir(mode=0o700, parents=True)
        except FileExistsError:
            pass
        info = os.lstat(path)
    if stat.S_ISLNK(info.st_mode) or not stat.S_ISDIR(info.st_mode):
        raise RuntimeError("default profile directory must be a real directory")
    if info.st_uid != os.getuid():
        raise RuntimeError("default profile directory is owned by another user")
    if info.st_mode & 0o022:
        raise RuntimeError("default profile directory is group or world writable")


def _open_profile_lock(lock_path: str | os.PathLike[str] | None) -> int:
    if lock_path is None:
        directory = _private_lock_directory()
        _ensure_private_directory(directory)
        path = directory / "owner.lock"
    else:
        path = Path(lock_path)
    flags = os.O_RDWR | os.O_CREAT | os.O_CLOEXEC
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    try:
        fd = os.open(path, flags, 0o600)
    except OSError as exc:
        raise RuntimeError(f"cannot open profile lock {path}: {exc}") from exc
    try:
        info = os.fstat(fd)
        if not stat.S_ISREG(info.st_mode) or info.st_uid != os.getuid():
            raise RuntimeError("profile lock must be a regular file owned by this user")
        os.fchmod(fd, 0o600)
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as exc:
        os.close(fd)
        raise RuntimeError("rolling profile is already owned by another runtime") from exc
    except BaseException:
        os.close(fd)
        raise
    return fd


def _release_profile_lock(fd: int) -> None:
    with suppress(OSError):
        fcntl.flock(fd, fcntl.LOCK_UN)
    with suppress(OSError):
        os.close(fd)


@dataclass
class _Session:

    sock: socket.socket
    child: subprocess.Popen
    nonce: str
    challenge: str
    epoch: int = 0
    handshaken: bool = False
    last_receipt_s: float | None = None
    last_child_q: tuple | None = None
    correlations: dict = field(default_factory=dict)
    inbuf: bytearray = field(default_factory=bytearray)
    outgoing: bytearray = field(default_factory=bytearray)
    outgoing_started: bool = False
    pending_control: bytes | None = None
    pending_notice: bytes | None = None

    def close(self) -> None:
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None
        self.handshaken = False
        self.last_receipt_s = None
        self.last_child_q = None
        self.correlations.clear()
        self.inbuf.clear()
        self.outgoing.clear()
        self.outgoing_started = False
        self.pending_control = None
        self.pending_notice = None


@dataclass
class _RetiringChild:

    proc: subprocess.Popen
    kill_at: float
    killed: bool = False


class ProcessStation:
    """One foreground station owner plus one disposable producer child."""

    def __init__(self, *, lock_path: str | os.PathLike[str] | None = None) -> None:
        self._closed = False
        self._lock_fd = _open_profile_lock(lock_path)
        try:
            self._station = RollingStation()
        except BaseException:
            _release_profile_lock(self._lock_fd)
            self._lock_fd = None
            raise
        self._active: _Session | None = None
        self._retiring: _RetiringChild | None = None
        self._last_child_q = None
        self._frames_received = self._frames_sent = self._frames_stale = 0
        self._protocol_faults = self._acknowledgements = 0
        self._captures_exported = self._replies_submitted = 0
        self._replies_rejected = self._lease_expirations = 0
        self._disconnects = self._child_replacements = 0

    def spawn_client(self, argv: Sequence[str]) -> bool:
        """Start and handshake one disposable child."""
        self._ensure_open()
        if not self._reap_retiring_blocking():
            return False
        active = self._active
        if active is not None:
            if active.child.poll() is None:
                expired = (active.last_receipt_s is None or
                           time.monotonic() - active.last_receipt_s > LEASE_S)
                if not expired:
                    return False
                self._lease_expirations += 1
            self._child_replacements += 1
            self._disconnect()
            if not self._reap_retiring_blocking():
                return False
        arguments = self._validated_argv(argv)
        parent, child = socket.socketpair(socket.AF_UNIX, socket.SOCK_STREAM)
        parent.setblocking(False)
        child.setblocking(False)
        try:
            proc = subprocess.Popen(
                [*arguments, "--fd", str(child.fileno())],
                pass_fds=(child.fileno(),), close_fds=True, shell=False,
                stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL)
        except BaseException:
            parent.close()
            child.close()
            raise
        child.close()
        session = _Session(
            parent, proc, secrets.token_hex(16), secrets.token_hex(16))
        self._active = session
        self._queue_control(session, {
            "type": "hello", "session": session.nonce,
            "challenge": session.challenge, "epoch": session.epoch,
            "q": list(self._station.status().measured_q)})
        try:
            handshaken = self._await_handshake(session)
        except BaseException:
            self._disconnect()
            raise
        if not handshaken:
            self._disconnect()
            raise RuntimeError("child did not complete the station handshake")
        return True

    def run_for(self, wall_seconds: float) -> None:
        """Pace one advance per 10 ms opportunity and end on a boundary."""
        self._ensure_open()
        if isinstance(wall_seconds, bool) or not isinstance(
                wall_seconds, (int, float)):
            raise ValueError("wall_seconds must be a finite nonnegative number")
        wall_seconds = float(wall_seconds)
        if not math.isfinite(wall_seconds) or wall_seconds < 0.0:
            raise ValueError("wall_seconds must be a finite nonnegative number")
        if wall_seconds == 0.0:
            return
        start = time.monotonic()
        deadline = start + wall_seconds
        next_due = start
        completed = False
        while True:
            now = time.monotonic()
            if completed and now >= deadline:
                break
            if completed and now < next_due:
                time.sleep(min(next_due - now, deadline - now))
                continue
            self._service_boundary()
            completed = True
            after = time.monotonic()
            next_due += ADVANCE_OPPORTUNITY_S
            if next_due <= after:
                next_due = after + ADVANCE_OPPORTUNITY_S

    def check(self, acknowledge_fault: bool = False) -> Decision:
        self._ensure_open()
        if type(acknowledge_fault) is not bool:
            return Decision(False, DecisionCode.INVALID_ARGUMENT)
        if not self._refresh_session():
            return Decision(False, DecisionCode.INVALID_STATE)
        decision = self._station.check(acknowledge_fault=acknowledge_fault)
        if decision.accepted:
            self._start_epoch()
        return decision

    def arm(self, owner: str) -> Decision:
        self._ensure_open()
        if not self._refresh_session():
            return Decision(False, DecisionCode.INVALID_STATE)
        decision = self._station.arm(owner=owner)
        if decision.accepted:
            self._start_epoch()
        return decision

    def stop(self) -> None:
        self._ensure_open()
        self._station.stop()
        session = self._active
        if session is not None:
            session.epoch += 1
            self._clear_exports(session)
            if session.handshaken:
                self._queue_control(session, self._ping_payload(session))
                self._flush_output()

    def status(self) -> ProcessStatus:
        self._ensure_open()
        session = self._active
        child = None if session is None else session.child
        child_running = child is not None and child.poll() is None
        return ProcessStatus(
            self._station.status(),
            None if session is None else session.nonce,
            session.epoch if session is not None else 0,
            session is not None,
            session.handshaken if session is not None else False,
            None if child is None else child.pid,
            child_running,
            None if session is None else session.last_child_q,
            self._frames_received, self._frames_sent,
            self._frames_stale, self._protocol_faults,
            self._acknowledgements, self._captures_exported,
            0 if session is None else len(session.correlations),
            self._replies_submitted, self._replies_rejected,
            self._lease_expirations, self._disconnects,
            self._child_replacements,
            session is not None and session.pending_notice is not None)

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            with suppress(Exception):
                self._station.stop()
                self._station.disconnect()
            self._reap_retiring_blocking()
            session = self._active
            self._active = None
            if session is not None:
                session.close()
                self._begin_retirement(session.child, time.monotonic())
                self._reap_retiring_blocking()
            self._station.close()
        finally:
            if self._lock_fd is not None:
                _release_profile_lock(self._lock_fd)
                self._lock_fd = None

    def __enter__(self) -> "ProcessStation":
        self._ensure_open()
        return self

    def __exit__(self, exc_type, exc, traceback) -> bool:
        self.close()
        return False

    def _service_boundary(self) -> None:
        now = time.monotonic()
        self._reap_retiring(now)
        live = self._check_child(now)
        replies: list[Reply] = []
        if live:
            if not self._drain(now, replies) or not self._flush_output():
                replies.clear()
        if not self._check_child(time.monotonic()):
            replies.clear()
        captures = self._station.advance_control_tick(replies=tuple(replies))
        state = self._settle_station_state()
        session = self._active
        if (captures and state in (State.ARMED, State.MOVING)
                and session is not None and session.handshaken):
            self._export_captures(captures, session)
            self._flush_output()
        self._reap_retiring(time.monotonic())

    def _settle_station_state(self) -> State:
        state = self._station.status().state
        if state not in (State.ARMED, State.MOVING):
            session = self._active
            if session is not None and (
                    session.correlations or session.pending_notice is not None):
                self._start_epoch()
        return state

    def _export_captures(
            self, captures: tuple[Capture, ...], session: _Session) -> None:
        for capture in captures:
            session.correlations[capture.sequence] = capture
            while len(session.correlations) > MAX_CAPTURE_CORRELATIONS:
                session.correlations.pop(next(iter(session.correlations)))
            session.pending_notice = self._encode_message({
                "type": "capture", "session": session.nonce,
                "epoch": session.epoch, "sequence": capture.sequence,
                "captured_at_s": capture.captured_at_s,
                "q": list(capture.q)})
            self._captures_exported += 1

    def _await_handshake(self, session: _Session) -> bool:
        sock = session.sock
        if sock is None:
            return False
        deadline = time.monotonic() + HANDSHAKE_TIMEOUT_S
        while True:
            if session.child.poll() is not None or not self._flush_output():
                return False
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return False
            try:
                readable, _, _ = select.select(
                    [sock], [], [], min(remaining, 0.020))
            except (OSError, ValueError):
                return False
            if not readable:
                continue
            space = MAX_INPUT_BYTES - len(session.inbuf)
            if space <= 0:
                return False
            try:
                data = sock.recv(space)
            except (BlockingIOError, InterruptedError):
                continue
            except OSError:
                return False
            if not data:
                return False
            session.inbuf.extend(data)
            try:
                message = self._decode_frame(session)
            except _ProtocolError:
                return False
            if message is None:
                continue
            self._frames_received += 1
            if not self._accept_handshake(session, message):
                return False
            self._flush_output()
            return True

    def _accept_handshake(self, session: _Session, message: dict) -> bool:
        epoch = message.get("epoch")
        if (message.get("type") != "hello_ack"
                or not isinstance(message.get("session"), str)
                or message.get("session") != session.nonce
                or not isinstance(message.get("challenge"), str)
                or message.get("challenge") != session.challenge
                or type(epoch) is not int
                or epoch != 0):
            return False
        q = self._validated_vector(message.get("q"))
        if q is None:
            return False
        session.last_child_q = q
        session.last_receipt_s = time.monotonic()
        self._acknowledgements += 1
        session.handshaken = True
        self._rotate_challenge(session)
        self._queue_control(session, self._ping_payload(session))
        return True

    def _refresh_session(self) -> bool:
        now = time.monotonic()
        self._reap_retiring(now)
        if not self._check_child(now):
            return False
        ignored: list[Reply] = []
        if not self._drain(now, ignored) or not self._flush_output():
            return False
        return self._check_child(time.monotonic())

    def _check_child(self, now: float) -> bool:
        session = self._active
        if session is None or not session.handshaken:
            return False
        if session.child.poll() is not None:
            self._disconnect()
            return False
        if session.last_receipt_s is None or now - session.last_receipt_s > LEASE_S:
            self._lease_expirations += 1
            self._disconnect()
            return False
        return True

    def _disconnect(self) -> bool:
        session = self._active
        if session is None:
            return False
        self._active = None
        session.close()
        self._station.disconnect()
        self._begin_retirement(session.child, time.monotonic())
        self._disconnects += 1
        return False

    def _begin_retirement(
            self, proc: subprocess.Popen, now: float) -> None:
        if self._retiring is not None:
            raise RuntimeError("previous retiring child has not been reaped")
        if proc.poll() is not None:
            with suppress(subprocess.TimeoutExpired):
                proc.wait(timeout=0.0)
            return
        with suppress(ProcessLookupError, PermissionError):
            os.kill(proc.pid, signal.SIGCONT)
        with suppress(ProcessLookupError):
            proc.terminate()
        self._retiring = _RetiringChild(proc, now + CHILD_EXIT_GRACE_S)

    def _reap_retiring(self, now: float, *, wait: bool = False) -> bool:
        record = self._retiring
        if record is None:
            return True
        proc = record.proc
        if proc.poll() is not None:
            with suppress(subprocess.TimeoutExpired):
                proc.wait(timeout=0.0)
            self._retiring = None
            return True
        if not record.killed:
            if wait and now < record.kill_at:
                with suppress(subprocess.TimeoutExpired):
                    proc.wait(timeout=record.kill_at - now)
            if proc.poll() is not None:
                with suppress(subprocess.TimeoutExpired):
                    proc.wait(timeout=0.0)
                self._retiring = None
                return True
            if wait or now >= record.kill_at:
                with suppress(ProcessLookupError, PermissionError):
                    os.kill(proc.pid, signal.SIGKILL)
                record.killed = True
        if wait:
            with suppress(subprocess.TimeoutExpired):
                proc.wait(timeout=CHILD_KILL_GRACE_S)
        if proc.poll() is not None:
            with suppress(subprocess.TimeoutExpired):
                proc.wait(timeout=0.0)
            self._retiring = None
            return True
        return False

    def _reap_retiring_blocking(self) -> bool:
        if self._retiring is None:
            return True
        self._reap_retiring(time.monotonic(), wait=True)
        return self._retiring is None

    @staticmethod
    def _validated_argv(argv: Sequence[str]) -> list[str]:
        if isinstance(argv, (str, bytes)) or not isinstance(argv, Sequence):
            raise TypeError("argv must be a sequence of strings")
        arguments = list(argv)
        if not arguments or any(not isinstance(item, str) for item in arguments):
            raise ValueError("argv must be a non-empty sequence of strings")
        if "--fd" in arguments:
            raise ValueError("argv must not supply the inherited client fd")
        return arguments

    @staticmethod
    def _encode_message(payload: dict) -> bytes:
        body = json.dumps(payload, separators=(",", ":"),
                          allow_nan=False, sort_keys=True).encode("utf-8")
        if not body or len(body) > MAX_BODY_BYTES:
            raise RuntimeError("child frame body exceeds the bounded size")
        return len(body).to_bytes(HEADER_BYTES, "big") + body

    def _queue_control(self, session: _Session, payload: dict) -> None:
        session.pending_control = self._encode_message(payload)

    def _ping_payload(self, session: _Session) -> dict:
        return {"type": "ping", "session": session.nonce,
                "challenge": session.challenge, "epoch": session.epoch}

    def _rotate_challenge(self, session: _Session) -> None:
        session.challenge = secrets.token_hex(16)

    def _flush_output(self) -> bool:
        session = self._active
        if session is None or session.sock is None:
            return False
        frames = 0
        while frames < MAX_OUTPUT_FRAMES_PER_FLUSH:
            if not session.outgoing:
                if session.pending_control is not None:
                    session.outgoing.extend(session.pending_control)
                    session.pending_control = None
                elif session.pending_notice is not None:
                    session.outgoing.extend(session.pending_notice)
                    session.pending_notice = None
                else:
                    break
                session.outgoing_started = False
            try:
                sent = session.sock.send(session.outgoing)
            except (BlockingIOError, InterruptedError):
                return True
            except OSError:
                return self._disconnect()
            if sent <= 0:
                return self._disconnect()
            session.outgoing_started = True
            del session.outgoing[:sent]
            if session.outgoing:
                return True
            session.outgoing_started = False
            self._frames_sent += 1
            frames += 1
        return True

    def _fail(self) -> bool:
        self._protocol_faults += 1
        return self._disconnect()

    def _decode_frame(self, session: _Session) -> dict | None:
        if len(session.inbuf) < HEADER_BYTES:
            return None
        length = int.from_bytes(session.inbuf[:HEADER_BYTES], "big")
        if length <= 0 or length > MAX_BODY_BYTES:
            raise _ProtocolError("invalid frame length")
        total = HEADER_BYTES + length
        if len(session.inbuf) < total:
            return None
        body = bytes(session.inbuf[HEADER_BYTES:total])
        del session.inbuf[:total]
        try:
            message = json.loads(body)
        except (ValueError, RecursionError) as exc:
            raise _ProtocolError("invalid frame body") from exc
        if not isinstance(message, dict):
            raise _ProtocolError("frame body must be an object")
        return message

    def _drain(self, now: float, replies: list[Reply]) -> bool:
        session = self._active
        if session is None or not session.handshaken:
            return True
        messages = read_bytes = 0
        while True:
            while messages < MAX_FRAMES_PER_BOUNDARY:
                try:
                    message = self._decode_frame(session)
                except _ProtocolError:
                    return self._fail()
                if message is None:
                    break
                messages += 1
                self._frames_received += 1
                if not self._process_message(session, message, now, replies):
                    return False
            if messages >= MAX_FRAMES_PER_BOUNDARY:
                if len(session.inbuf) >= MAX_INPUT_BYTES:
                    return self._fail()
                break
            space = MAX_INPUT_BYTES - len(session.inbuf)
            budget = MAX_READ_BYTES_PER_BOUNDARY - read_bytes
            if space <= 0:
                return self._fail()
            if budget <= 0:
                break
            try:
                data = session.sock.recv(min(space, budget))
            except (BlockingIOError, InterruptedError):
                break
            except OSError:
                return self._fail()
            if not data:
                return self._disconnect()
            read_bytes += len(data)
            session.inbuf.extend(data)
        return True

    def _process_message(
            self, session: _Session, message: dict, now: float,
            replies: list[Reply]) -> bool:
        kind = message.get("type")
        if not isinstance(kind, str) or kind not in _ALLOWED_CHILD_TYPES:
            return self._fail()
        if message.get("session") != session.nonce:
            return self._fail()
        if kind in ("hello_ack", "ack"):
            challenge = message.get("challenge")
            epoch = message.get("epoch")
            if not isinstance(challenge, str) or type(epoch) is not int:
                return self._fail()
            if epoch != session.epoch or challenge != session.challenge:
                self._frames_stale += 1
                return True
            q = self._validated_vector(message.get("q"))
            if q is None:
                return self._fail()
            session.last_child_q = q
            session.last_receipt_s = now
            self._acknowledgements += 1
            self._rotate_challenge(session)
            self._queue_control(session, self._ping_payload(session))
            return True
        epoch = message.get("epoch")
        sequence = message.get("sequence")
        if type(epoch) is not int or type(sequence) is not int:
            return self._fail()
        if epoch != session.epoch or sequence < 0:
            self._frames_stale += 1
            self._replies_rejected += 1
            return True
        capture = session.correlations.get(sequence)
        if capture is None:
            self._replies_rejected += 1
            return True
        q = self._validated_vector(message.get("q"))
        if q is None:
            return self._fail()
        replies.append(Reply(capture, q))
        self._replies_submitted += 1
        return True

    @staticmethod
    def _validated_vector(value) -> tuple | None:
        if not isinstance(value, (list, tuple)) or len(value) != 3:
            return None
        result = []
        for item in value:
            if isinstance(item, bool) or not isinstance(item, (int, float)):
                return None
            try:
                item = float(item)
            except (OverflowError, TypeError, ValueError):
                return None
            if not math.isfinite(item):
                return None
            result.append(item)
        return tuple(result)

    def _start_epoch(self) -> None:
        session = self._active
        if session is None:
            return
        session.epoch += 1
        self._clear_exports(session)
        if session.handshaken:
            self._queue_control(session, self._ping_payload(session))
            self._flush_output()

    def _clear_exports(self, session: _Session) -> None:
        session.correlations.clear()
        session.pending_notice = session.pending_control = None
        if not session.outgoing_started:
            session.outgoing.clear()

    def _ensure_open(self) -> None:
        if self._closed:
            raise RuntimeError("process station is closed")
