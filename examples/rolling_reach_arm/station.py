"""Single-owner simulation station for the selected rolling-reach scene.

The station owns one model, data, renderer, and JointCore.  Callers exchange
immutable captures and replies; no policy or transport runs inside a physics
chunk.
"""

from __future__ import annotations

import copy
from dataclasses import dataclass
import hashlib
import math
import os

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

from im6014_control.core import JointCore  # noqa: E402
from im6014_control.mapping import JointSample  # noqa: E402
from im6014_control.records import (  # noqa: E402
    CommunicationFailure,
    Decision,
    DecisionCode,
    JointFault,
    MotorHealth,
    Ownership,
    SourceIdentity,
    State,
    Target,
)

from . import drive, scene  # noqa: E402


TICKS_PER_SECOND = 3000
CONTROL_DECIMATION = 30
CAMERA_DECIMATION = 100
STEPS_PER_ADVANCE = 30
CAPTURE_TTL_TICKS = 750
CAPTURE_TABLE_SIZE = 8
MAX_REPLIES_PER_BOUNDARY = 8
FEEDBACK_MAX_AGE_S = 0.05
MAX_REFERENCE_STEP_S = 0.01
MEASURED_LEAD_RAD = 0.1


@dataclass(frozen=True)
class Capture:
    """One authenticated camera observation emitted by the station."""

    sequence: int
    captured_at_s: float
    q: tuple
    rgb: bytes

    def __post_init__(self) -> None:
        if type(self.sequence) is not int or self.sequence < 0:
            raise ValueError("sequence must be a nonnegative integer")
        if isinstance(self.captured_at_s, bool) or not isinstance(
            self.captured_at_s, (int, float)
        ):
            raise ValueError("captured_at_s must be a finite real number")
        captured_at_s = float(self.captured_at_s)
        if not math.isfinite(captured_at_s) or captured_at_s < 0.0:
            raise ValueError("captured_at_s must be finite and nonnegative")
        object.__setattr__(self, "captured_at_s", captured_at_s)
        if type(self.q) is not tuple or len(self.q) != 3:
            raise ValueError("q must be a three-element tuple")
        if type(self.rgb) is not bytes or not self.rgb:
            raise ValueError("rgb must be non-empty bytes")


@dataclass(frozen=True)
class Reply:
    """A producer's proposed absolute logical joint vector for one capture."""

    capture: Capture
    q: tuple


@dataclass(frozen=True)
class RequestedReference:
    capture_sequence: int
    q: tuple
    created_at_s: float
    expires_at_s: float
    created_tick: int
    expires_tick: int


@dataclass(frozen=True)
class IssuedReference:
    capture_sequence: int
    issue_sequence: int
    q: tuple
    created_at_s: float
    expires_at_s: float
    created_tick: int
    expires_tick: int
    issued_at_s: float
    issued_tick: int


@dataclass(frozen=True)
class StationStatus:
    """Immutable diagnostic view without ownership, model, data, or core."""

    state: State
    identity: str
    source: SourceIdentity | None
    fault_latched: bool
    faults: tuple[JointFault, ...]
    actuation_enabled: bool
    measured_q: tuple
    measured_dq: tuple
    measured_tau: tuple
    step: int
    control_ticks: int
    camera_ticks: int
    captures_emitted: int
    captures_dropped: int
    captures_buffered: int
    replies_accepted: int
    replies_rejected: int
    references_issued: int
    last_requested: RequestedReference | None
    last_issued: IssuedReference | None


@dataclass(frozen=True)
class _CaptureEntry:
    capture: Capture
    owner: Ownership
    source: SourceIdentity
    tick: int
    expires_tick: int

    @property
    def created_at_s(self) -> float:
        return self.tick / TICKS_PER_SECOND

    @property
    def expires_at_s(self) -> float:
        return self.expires_tick / TICKS_PER_SECOND


@dataclass(frozen=True)
class _Intent:
    q: tuple
    entry: _CaptureEntry


@dataclass(frozen=True)
class _Issued:
    q: tuple
    entry: _CaptureEntry
    issue_sequence: int
    issued_tick: int


class RollingStation:
    """One writer for the canonical model, core, captures, and release."""

    def __init__(self) -> None:
        cfg = scene.SCENARIO
        self._closed = False
        self._model = mujoco.MjModel.from_xml_string(scene.generated_xml())
        self._marker_id = int(self._model.geom("task_marker").id)
        self._canonical_marker_xy = (
            float(self._model.geom_pos[self._marker_id, 0]),
            float(self._model.geom_pos[self._marker_id, 1]),
        )
        self._actuation_bit = int(mujoco.mjtDisableBit.mjDSBL_ACTUATION)
        self._expected_digest = self._normalized_digest(self._model)
        self._identity = (
            f"{cfg.name}:{cfg.revision}:{self._expected_digest}:"
            f"mujoco-{mujoco.__version__}"
        )
        self._set_actuation(False)

        self._specs = drive.joint_specs_from_model(
            self._model, max_measured_lead_rad=MEASURED_LEAD_RAD
        )
        self._observer = scene.observer()
        self._mapping = scene.mapping()
        self._data = mujoco.MjData(self._model)
        self._data.qpos[:] = scene.logical_to_internal(cfg.home)
        self._data.qvel[:] = 0.0
        self._data.ctrl[:] = cfg.home
        mujoco.mj_forward(self._model, self._data)
        self._renderer = mujoco.Renderer(
            self._model, height=cfg.image_height, width=cfg.image_width
        )
        self._core = JointCore(
            self._specs,
            feedback_max_age_s=FEEDBACK_MAX_AGE_S,
            clock=lambda: self._n / TICKS_PER_SECOND,
            max_reference_step_s=MAX_REFERENCE_STEP_S,
        )

        self._n = 0
        self._connection = 0
        self._source: SourceIdentity | None = None
        self._ownership: Ownership | None = None
        self._authorized = False
        self._captures: dict[int, _CaptureEntry] = {}
        self._next_capture_sequence = 0
        self._next_issue_sequence = 0
        self._intent: _Intent | None = None
        self._last_issued: _Issued | None = None
        self._last_requested: RequestedReference | None = None
        self._control_ticks = 0
        self._camera_ticks = 0
        self._captures_emitted = 0
        self._captures_dropped = 0
        self._replies_accepted = 0
        self._replies_rejected = 0
        self._references_issued = 0

    def _normalized_digest(self, model: mujoco.MjModel) -> str:
        clone = copy.deepcopy(model)
        try:
            marker_id = int(clone.geom("task_marker").id)
        except Exception:
            return ""
        clone.geom_pos[marker_id, 0] = self._canonical_marker_xy[0]
        clone.geom_pos[marker_id, 1] = self._canonical_marker_xy[1]
        clone.opt.disableflags = (
            int(clone.opt.disableflags) & ~self._actuation_bit
        )
        try:
            buffer = np.empty(mujoco.mj_sizeModel(clone), dtype=np.uint8)
            mujoco.mj_saveModel(clone, None, buffer)
        except Exception:
            return ""
        return hashlib.sha256(buffer.tobytes()).hexdigest()

    def _identity_ok(self) -> bool:
        return bool(self._expected_digest) and (
            self._normalized_digest(self._model) == self._expected_digest
        )

    def _disabled(self) -> bool:
        return bool(int(self._model.opt.disableflags) & self._actuation_bit)

    def _set_actuation(self, enabled: bool) -> None:
        if enabled:
            self._model.opt.disableflags = (
                int(self._model.opt.disableflags) & ~self._actuation_bit
            )
        else:
            self._model.opt.disableflags = (
                int(self._model.opt.disableflags) | self._actuation_bit
            )

    def _clear_work(self) -> None:
        self._captures.clear()
        self._intent = None
        self._last_issued = None
        self._last_requested = None

    def _halt(self) -> None:
        self._core.stop()
        self._ownership = None
        self._authorized = False
        self._clear_work()
        self._set_actuation(False)

    def _identity_fault(self) -> None:
        if self._core.snapshot().state not in (
            State.DISCONNECTED,
            State.CONNECTING,
        ):
            for spec in self._specs:
                self._core.communication_failed(
                    spec.motor_id, CommunicationFailure.DRIVER_FAILURE
                )
        self._halt()

    def _communication_fault(self) -> None:
        for spec in self._specs:
            self._core.communication_failed(
                spec.motor_id, CommunicationFailure.INVALID_FEEDBACK
            )
        self._halt()

    def _feedback(self):
        mujoco.mj_forward(self._model, self._data)
        q = tuple(float(value) for value in self._observer @ self._data.qpos)
        dq = tuple(float(value) for value in self._observer @ self._data.qvel)
        tau = tuple(
            float(value) for value in self._mapping.T @ self._data.qfrc_actuator
        )
        return q, dq, tau

    def _receive_feedback(self) -> bool:
        try:
            feedback = self._feedback()
        except (ArithmeticError, OverflowError, TypeError, ValueError):
            self._communication_fault()
            return False
        if (
            not isinstance(feedback, tuple)
            or len(feedback) != 3
            or any(
                not isinstance(values, tuple)
                or len(values) != 3
                or any(
                    isinstance(value, bool)
                    or not isinstance(value, (int, float))
                    or not math.isfinite(float(value))
                    for value in values
                )
                for values in feedback
            )
        ):
            self._communication_fault()
            return False
        q, dq, tau = feedback
        health = MotorHealth(0 if self._disabled() else 1, 0, 0, False)
        for spec, qi, vi, ti in zip(self._specs, q, dq, tau):
            try:
                sample = JointSample(float(qi), float(vi), float(ti))
            except ValueError:
                self._communication_fault()
                return False
            if not self._core.receive(spec.motor_id, sample, health).accepted:
                self._halt()
                return False
        if self._core.snapshot().state is State.FAULT:
            self._halt()
            return False
        return True

    def _enforce_authority_bit(self) -> bool:
        if self._authorized != (not self._disabled()):
            self._halt()
            return False
        return True

    def _enforce_deadlines(self, tick: int) -> bool:
        expired = (
            self._last_issued is not None
            and tick >= self._last_issued.entry.expires_tick
        ) or (
            self._intent is not None
            and tick >= self._intent.entry.expires_tick
        )
        if expired:
            self._halt()
        return expired

    @staticmethod
    def _validated_vector(value):
        if not isinstance(value, tuple) or len(value) != 3:
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

    def _purge_captures(self, tick: int) -> None:
        expired = [
            sequence
            for sequence, entry in self._captures.items()
            if tick >= entry.expires_tick
        ]
        for sequence in expired:
            del self._captures[sequence]
            self._captures_dropped += 1

    def _accept_reply(self, reply: Reply, tick: int) -> None:
        capture = reply.capture
        if not isinstance(capture, Capture) or type(capture.sequence) is not int:
            self._replies_rejected += 1
            return
        entry = self._captures.get(capture.sequence)
        if entry is None or entry.capture is not capture:
            self._replies_rejected += 1
            return
        if (
            self._ownership is None
            or entry.owner is not self._ownership
            or self._source is None
            or entry.source is not self._source
        ):
            self._replies_rejected += 1
            return
        if tick >= entry.expires_tick:
            del self._captures[capture.sequence]
            self._captures_dropped += 1
            self._replies_rejected += 1
            return
        if (
            self._intent is not None
            and capture.sequence <= self._intent.entry.capture.sequence
        ):
            self._replies_rejected += 1
            return
        q = self._validated_vector(reply.q)
        if q is None:
            self._replies_rejected += 1
            return
        self._intent = _Intent(q, entry)
        self._last_requested = RequestedReference(
            capture.sequence,
            q,
            entry.created_at_s,
            entry.expires_at_s,
            entry.tick,
            entry.expires_tick,
        )
        self._replies_accepted += 1

    def _make_target(self, intent: _Intent) -> Target:
        sequence = self._next_issue_sequence
        self._next_issue_sequence += 1
        entry = intent.entry
        return Target(
            sequence,
            tuple(spec.drive.target(q) for spec, q in zip(self._specs, intent.q)),
            entry.created_at_s,
            entry.expires_at_s,
            self._ownership,
        )

    def _release(self, reference, intent: _Intent, target: Target, tick: int) -> None:
        ownership = self._ownership
        source = self._source
        entry = intent.entry
        snapshot = self._core.snapshot()
        if (
            ownership is None
            or source is None
            or snapshot.ownership is not ownership
            or snapshot.ownership is not entry.owner
            or snapshot.source is not source
            or snapshot.source is not entry.source
            or reference.requested is not target
            or reference.requested.ownership is not ownership
            or self._n >= entry.expires_tick
            or self._n / TICKS_PER_SECOND >= entry.expires_at_s
        ):
            self._halt()
            return
        if not self._identity_ok():
            self._identity_fault()
            return
        if snapshot.state not in (State.ARMED, State.MOVING):
            self._halt()
            return
        q = tuple(float(joint.q_rad) for joint in reference.joints)
        if len(q) != len(self._specs) or any(
            not math.isfinite(value)
            or value < spec.position_limits.minimum_rad
            or value > spec.position_limits.maximum_rad
            for spec, value in zip(self._specs, q)
        ):
            self._halt()
            return

        self._data.ctrl[:] = q
        self._last_issued = _Issued(
            q,
            entry,
            target.sequence,
            tick,
        )
        self._references_issued += 1
        self._authorized = True
        self._set_actuation(True)

    def _control(self, tick: int, replies: tuple[Reply, ...]) -> None:
        state = self._core.snapshot().state
        if state in (State.DISCONNECTED, State.CONNECTING):
            return
        if not self._identity_ok():
            self._identity_fault()
            return
        if state not in (State.ARMED, State.MOVING):
            return
        if self._ownership is None:
            self._halt()
            return
        if not self._identity_ok():
            self._identity_fault()
            return
        if not self._receive_feedback():
            return
        decision = self._core.tick()
        if (
            not decision.accepted
            or self._core.snapshot().state is State.FAULT
        ):
            self._halt()
            return

        for reply in replies:
            self._accept_reply(reply, tick)

        intent = self._intent
        if intent is None:
            return
        if tick >= intent.entry.expires_tick:
            self._halt()
            return
        entry = intent.entry
        if (
            self._ownership is None
            or entry.owner is not self._ownership
            or self._source is None
            or entry.source is not self._source
            or self._captures.get(entry.capture.sequence) is not entry
        ):
            self._halt()
            return
        if not self._identity_ok():
            self._identity_fault()
            return
        try:
            target = self._make_target(intent)
        except (ArithmeticError, TypeError, ValueError):
            self._halt()
            return
        if not self._core.submit_target(target).accepted:
            return
        reference = self._core.consume_target()
        if reference is None:
            return
        self._release(reference, intent, target, tick)

    def _capture(self, tick: int) -> Capture | None:
        self._purge_captures(tick)
        state = self._core.snapshot().state
        if (
            self._ownership is None
            or self._source is None
            or state not in (State.ARMED, State.MOVING)
        ):
            return None
        self._renderer.update_scene(self._data, camera="overhead")
        rgb = np.ascontiguousarray(self._renderer.render())
        q = tuple(float(value) for value in self._observer @ self._data.qpos)
        capture = Capture(
            self._next_capture_sequence,
            tick / TICKS_PER_SECOND,
            q,
            rgb.tobytes(),
        )
        self._next_capture_sequence += 1
        entry = _CaptureEntry(
            capture,
            self._ownership,
            self._source,
            tick,
            tick + CAPTURE_TTL_TICKS,
        )
        if len(self._captures) >= CAPTURE_TABLE_SIZE:
            oldest = next(iter(self._captures))
            del self._captures[oldest]
            self._captures_dropped += 1
        self._captures[capture.sequence] = entry
        self._captures_emitted += 1
        return capture

    def check(self, *, acknowledge_fault: bool = False) -> Decision:
        if self._closed:
            raise RuntimeError("station is closed")
        if type(acknowledge_fault) is not bool:
            return Decision(False, DecisionCode.INVALID_ARGUMENT)
        state = self._core.snapshot().state
        if state not in (
            State.DISCONNECTED,
            State.CONNECTED,
            State.STOPPED,
            State.FAULT,
        ):
            return Decision(False, DecisionCode.INVALID_STATE)
        if state is State.DISCONNECTED:
            if not self._core.begin_connect().accepted:
                return Decision(False, DecisionCode.INVALID_STATE)
            self._connection += 1
            source = SourceIdentity("demo", self._identity, self._connection)
            if not self._core.connected(source).accepted:
                return Decision(False, DecisionCode.INVALID_SOURCE)
            self._source = self._core.snapshot().source
        if not self._identity_ok():
            self._identity_fault()
            return Decision(False, DecisionCode.INVALID_SOURCE)
        decision = self._core.begin_check(
            acknowledge_fault=acknowledge_fault
        )
        if not decision.accepted:
            return decision
        self._set_actuation(False)
        self._authorized = False
        self._ownership = None
        self._source = self._core.snapshot().source
        self._clear_work()
        if not self._receive_feedback():
            return Decision(False, DecisionCode.INVALID_OBSERVATION)
        decision = self._core.checked()
        if not decision.accepted:
            self._halt()
            return decision
        self._source = self._core.snapshot().source
        return decision

    def arm(self, *, owner: str) -> Decision:
        if self._closed:
            raise RuntimeError("station is closed")
        if self._core.snapshot().state is State.READY:
            if not self._identity_ok():
                self._identity_fault()
                return Decision(False, DecisionCode.INVALID_SOURCE)
        decision = self._core.arm(owner=owner)
        if decision.accepted:
            self._ownership = decision.ownership
            self._source = self._core.snapshot().source
            self._authorized = False
            self._set_actuation(False)
        return Decision(decision.accepted, decision.code)

    def advance_control_tick(
        self, *, replies: tuple[Reply, ...] = ()
    ) -> tuple[Capture, ...]:
        if self._closed:
            raise RuntimeError("station is closed")
        if not isinstance(replies, tuple):
            raise ValueError("replies must be a tuple")
        if len(replies) > MAX_REPLIES_PER_BOUNDARY:
            raise ValueError("too many replies for one control boundary")
        if any(not isinstance(reply, Reply) for reply in replies):
            raise ValueError("every reply must be a Reply")

        captures: list[Capture] = []
        for _ in range(STEPS_PER_ADVANCE):
            tick = self._n
            if tick % CONTROL_DECIMATION == 0:
                self._control_ticks += 1
            if tick % CAMERA_DECIMATION == 0:
                self._camera_ticks += 1
            ready = self._enforce_authority_bit()
            if ready:
                ready = not self._enforce_deadlines(tick)
            if ready and tick % CONTROL_DECIMATION == 0:
                self._control(tick, replies)
            if ready and tick % CAMERA_DECIMATION == 0:
                capture = self._capture(tick)
                if capture is not None:
                    captures.append(capture)
            self._enforce_authority_bit()
            mujoco.mj_step(self._model, self._data)
            self._n += 1
        return tuple(captures)

    def stop(self) -> None:
        if self._closed:
            return
        self._halt()

    def disconnect(self) -> None:
        if self._closed:
            return
        self._core.disconnect()
        self._ownership = None
        self._source = None
        self._authorized = False
        self._clear_work()
        self._set_actuation(False)

    def set_marker(self, xy: tuple[float, float]) -> None:
        if self._closed:
            raise RuntimeError("station is closed")
        if not isinstance(xy, tuple) or len(xy) != 2:
            raise ValueError("xy must be a two-element tuple")
        values = []
        for item in xy:
            if isinstance(item, bool) or not isinstance(item, (int, float)):
                raise ValueError("xy must contain finite real numbers")
            try:
                item = float(item)
            except (OverflowError, TypeError, ValueError) as exc:
                raise ValueError("xy must contain finite real numbers") from exc
            if not math.isfinite(item):
                raise ValueError("xy must contain finite real numbers")
            values.append(item)
        if not scene.marker_in_workspace(tuple(values)):
            raise ValueError("marker is outside the declared workspace")
        self._model.geom_pos[self._marker_id, 0] = values[0]
        self._model.geom_pos[self._marker_id, 1] = values[1]

    def status(self) -> StationStatus:
        snapshot = self._core.snapshot()
        mujoco.mj_forward(self._model, self._data)
        q = tuple(float(value) for value in self._observer @ self._data.qpos)
        dq = tuple(float(value) for value in self._observer @ self._data.qvel)
        tau = tuple(
            float(value) for value in self._mapping.T @ self._data.qfrc_actuator
        )
        last_issued = None
        if self._last_issued is not None:
            issued = self._last_issued
            last_issued = IssuedReference(
                issued.entry.capture.sequence,
                issued.issue_sequence,
                issued.q,
                issued.entry.created_at_s,
                issued.entry.expires_at_s,
                issued.entry.tick,
                issued.entry.expires_tick,
                issued.issued_tick / TICKS_PER_SECOND,
                issued.issued_tick,
            )
        return StationStatus(
            snapshot.state,
            self._identity,
            snapshot.source,
            snapshot.fault_latched,
            snapshot.faults,
            self._authorized and not self._disabled(),
            q,
            dq,
            tau,
            self._n,
            self._control_ticks,
            self._camera_ticks,
            self._captures_emitted,
            self._captures_dropped,
            len(self._captures),
            self._replies_accepted,
            self._replies_rejected,
            self._references_issued,
            self._last_requested,
            last_issued,
        )

    def close(self) -> None:
        if self._closed:
            return
        self._core.disconnect()
        self._ownership = None
        self._source = None
        self._authorized = False
        self._clear_work()
        self._set_actuation(False)
        self._renderer.close()
        self._closed = True
