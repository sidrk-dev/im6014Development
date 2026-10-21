"""Pure reference planning, codec projection, and release rate calculations.

This module owns pure mathematical and codec planning for references. It owns
no lifecycle, clock, serial transport, pending targets, or mutable state.
"""

import math
from dataclasses import dataclass

from .codec import (
    command,
    decode_command,
    project_position_raw,
    project_speed_raw,
    raw_to_position,
    raw_to_rpm,
)
from .mapping import JointSample, JointTarget, OutputSample
from .records import JointSpec, NativePositionServo


@dataclass(frozen=True)
class _AppliedJointReference:
    q_rad: float
    dq_rad_s: float
    tau_nm: float
    kp: float
    kd: float


def _seed_reference(spec: JointSpec, sample: JointSample) -> JointTarget:
    if isinstance(spec.drive, NativePositionServo):
        return spec.drive.target(sample.q_rad)
    return JointTarget(sample.q_rad, 0.0, 0.0, 0.0, 0.0)


def _applied_reference(spec: JointSpec, target: JointTarget) -> _AppliedJointReference:
    if isinstance(spec.drive, NativePositionServo):
        drive = spec.drive
        if target.kp != drive.kp or target.kd != drive.kd:
            raise ValueError("native target gains must match the fixed drive")
        if target.dq_rad_s != 0.0 or target.tau_nm != 0.0:
            raise ValueError("native desired velocity and feedforward must be zero")
        return _AppliedJointReference(
            target.q_rad, 0.0, 0.0, drive.kp, drive.kd,
        )

    output = spec.mapping.to_output(target)
    decoded = decode_command(command(
        spec.motor_id, mode=1, position=output.q_rad, rpm=output.rpm,
        torque=output.tau_nm, kp=output.kp, kd=output.kd, watchdog=True,
    ))
    sample = spec.mapping.from_output(OutputSample(
        decoded["position_rad"], decoded["rpm"], decoded["torque_nm"],
    ))
    reduction = spec.mapping.reduction
    kp = decoded["kp"] * reduction * reduction
    kd = decoded["kd"] * reduction * reduction
    if not math.isfinite(kp) or not math.isfinite(kd):
        raise ValueError("applied gains must be finite")
    return _AppliedJointReference(sample.q_rad, sample.dq_rad_s, sample.tau_nm, kp, kd)


def _reference_key(
    spec: JointSpec, target: JointTarget
) -> tuple[int, int] | tuple[float, float]:
    if isinstance(spec.drive, NativePositionServo):
        return target.q_rad, 0.0
    output = spec.mapping.to_output(target)
    decoded = decode_command(command(
        spec.motor_id, mode=1, rpm=output.rpm, kd=output.kd,
        torque=output.tau_nm, watchdog=True, position=output.q_rad, kp=output.kp,
    ))
    return decoded["raw_position"], decoded["raw_rpm"]


def _predicted_effort(applied: _AppliedJointReference, q_rad: float, dq_rad_s: float) -> float:
    effort = (applied.kp * (applied.q_rad - q_rad)
              + applied.kd * (applied.dq_rad_s - dq_rad_s) + applied.tau_nm)
    if not math.isfinite(effort):
        raise ValueError("predicted effort must be finite")
    return effort


def project_joint_target(
    spec: JointSpec,
    target: JointTarget,
    q_min: float,
    q_max: float,
    dq_min: float,
    dq_max: float,
) -> JointTarget:
    """Project requested joint values into the representation's closed corridor."""
    if isinstance(spec.drive, NativePositionServo):
        applied = _applied_reference(spec, target)
        for value in (q_min, q_max, dq_min, dq_max):
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError("native projection bounds must be finite")
            try:
                finite = math.isfinite(value)
            except OverflowError as exc:
                raise ValueError("native projection bounds must be finite") from exc
            if not finite:
                raise ValueError("native projection bounds must be finite")
        if q_min > q_max:
            raise ValueError("native position corridor is empty")
        if dq_min > dq_max or not dq_min <= 0.0 <= dq_max:
            raise ValueError("native desired velocity corridor must contain zero")
        q_rad = applied.q_rad
        if q_rad < q_min:
            q_rad = math.nextafter(q_min, q_max)
        elif q_rad > q_max:
            q_rad = math.nextafter(q_max, q_min)
        return JointTarget(q_rad, 0.0, 0.0, applied.kp, applied.kd)

    mapping = spec.mapping
    direction = mapping.direction
    wanted_speed = max(dq_min, min(dq_max, target.dq_rad_s))

    def position_to_joint(output_rad: float) -> float:
        return mapping.from_output(OutputSample(output_rad, 0.0, 0.0)).q_rad

    def speed_to_joint(output_rpm: float) -> float:
        return mapping.from_output(OutputSample(0.0, output_rpm, 0.0)).dq_rad_s

    raw_position = project_position_raw(
        target.q_rad, q_min, q_max, position_to_joint, direction
    )
    raw_speed = project_speed_raw(
        wanted_speed, dq_min, dq_max, speed_to_joint, direction
    )
    if raw_position is None or raw_speed is None:
        raise ValueError("no representable reference fits the configured bounds")

    safe_target = JointTarget(
        position_to_joint(raw_to_position(raw_position)),
        speed_to_joint(raw_to_rpm(raw_speed)),
        target.tau_nm,
        target.kp,
        target.kd,
    )
    output = mapping.to_output(safe_target)
    frame = command(
        spec.motor_id,
        mode=1,
        rpm=output.rpm,
        kd=output.kd,
        torque=output.tau_nm,
        watchdog=True,
        position=output.q_rad,
        kp=output.kp,
    )
    decoded = decode_command(frame)
    if decoded["raw_position"] != raw_position or decoded["raw_rpm"] != raw_speed:
        raise ValueError("normal command encoding changed a projected grid code")
    if not (q_min <= safe_target.q_rad <= q_max):
        raise ValueError("projected position violates its closed bounds")
    if not (dq_min <= safe_target.dq_rad_s <= dq_max):
        raise ValueError("projected speed violates its closed bounds")
    return safe_target


def plan_reference(
    specs: tuple[JointSpec, ...],
    requested: tuple[JointTarget, ...],
    previous: tuple[JointTarget, ...],
    previous_rates: tuple[float, ...],
    measured_positions: tuple[float, ...],
    elapsed_s: float,
    max_step_s: float,
    allow_bootstrap: bool,
    *,
    measured_velocities: tuple[float, ...],
) -> tuple[JointTarget, ...] | None:
    """Plan a complete bounded candidate vector under S2 semantic contract.

    Computes independent dq and q corridors, intersects with position limits,
    velocity caps, acceleration rate bounds, and measured-position lead limits.
    Projects to inward representation-specific values, validates closed bounds,
    and checks progress against the static bounded goal. Physical joints also
    retain codec roundtrip and raw-grid checks; native joints use the exact
    continuous reference key. A candidate still needs final release validation;
    a tick need not progress.
    """
    if (
        isinstance(elapsed_s, bool)
        or not isinstance(elapsed_s, (int, float))
        or not math.isfinite(elapsed_s)
        or elapsed_s < 0.0
    ):
        return None
    if (
        isinstance(max_step_s, bool)
        or not isinstance(max_step_s, (int, float))
        or not math.isfinite(max_step_s)
        or max_step_s <= 0.0
    ):
        return None
    if not isinstance(measured_velocities, tuple):
        return None
    if (
        len(specs) != len(requested)
        or len(specs) != len(previous)
        or len(specs) != len(previous_rates)
        or len(specs) != len(measured_positions)
        or len(specs) != len(measured_velocities)
    ):
        return None
    try:
        for measured in (*measured_positions, *measured_velocities):
            if (
                isinstance(measured, bool)
                or not isinstance(measured, (int, float))
                or not math.isfinite(measured)
            ):
                return None
    except OverflowError:
        return None

    h = min(elapsed_s, max_step_s)
    candidate_joints: list[JointTarget] = []

    for i, spec in enumerate(specs):
        req = requested[i]
        prev = previous[i]
        w0 = previous_rates[i]
        q_meas = float(measured_positions[i])
        mapping = spec.mapping
        direction = mapping.direction
        limits = spec.position_limits
        L = limits.minimum_rad
        U = limits.maximum_rad
        V = spec.max_velocity_rad_s
        A = spec.max_acceleration_rad_s2
        lead = spec.max_measured_lead_rad
        q0 = prev.q_rad
        d0 = prev.dq_rad_s

        try:
            V_h = V * h
            A_h = A * h
            A_e = A * elapsed_s
            q_lead_min = q_meas - lead
            q_lead_max = q_meas + lead
        except (ArithmeticError, ValueError):
            return None
        if not (
            math.isfinite(V_h)
            and math.isfinite(A_h)
            and math.isfinite(A_e)
            and math.isfinite(q_lead_min)
            and math.isfinite(q_lead_max)
        ):
            return None

        try:
            d_min_cand = d0 - A_h
            d_max_cand = d0 + A_h
            w_min_cand = w0 - A_e
            w_max_cand = w0 + A_e
        except (ArithmeticError, ValueError):
            return None
        if not (
            math.isfinite(d_min_cand)
            and math.isfinite(d_max_cand)
            and math.isfinite(w_min_cand)
            and math.isfinite(w_max_cand)
        ):
            return None

        dq_min = max(-V, d_min_cand)
        dq_max = min(V, d_max_cand)
        if dq_min > dq_max:
            return None

        w_min = max(-V, w_min_cand)
        w_max = min(V, w_max_cand)
        if w_min > w_max:
            return None

        try:
            ew_min = elapsed_s * w_min
            ew_max = elapsed_s * w_max
            q_v_min = q0 - V_h
            q_v_max = q0 + V_h
            q_w_min = q0 + ew_min
            q_w_max = q0 + ew_max
        except (ArithmeticError, ValueError):
            return None
        if not (
            math.isfinite(ew_min)
            and math.isfinite(ew_max)
            and math.isfinite(q_v_min)
            and math.isfinite(q_v_max)
            and math.isfinite(q_w_min)
            and math.isfinite(q_w_max)
        ):
            return None

        q_min = max(L, q_v_min, q_w_min, q_lead_min)
        q_max = min(U, q_v_max, q_w_max, q_lead_max)
        if q_min > q_max:
            return None

        if not (
            math.isfinite(q_min)
            and math.isfinite(q_max)
            and math.isfinite(dq_min)
            and math.isfinite(dq_max)
        ):
            return None

        try:
            cand = project_joint_target(spec, req, q_min, q_max, dq_min, dq_max)
        except (ArithmeticError, ValueError):
            return None

        if abs(cand.q_rad - q0) > V_h:
            return None
        if abs(cand.dq_rad_s - d0) > A_h:
            return None
        if abs(cand.q_rad - q_meas) > lead:
            return None

        if isinstance(spec.drive, NativePositionServo):
            try:
                static_goal = project_joint_target(spec, req, L, U, -V, V)
                candidate_key = _reference_key(spec, cand)
                previous_key = _reference_key(spec, prev)
                static_key = _reference_key(spec, static_goal)
            except (ArithmeticError, ValueError):
                return None
            if not allow_bootstrap:
                if static_key[0] != previous_key[0] and candidate_key[0] == previous_key[0]:
                    return None
        else:
            try:
                candidate_key = _reference_key(spec, cand)
                previous_key = _reference_key(spec, prev)
            except (ArithmeticError, ValueError):
                return None

            def position_to_joint(output_rad: float) -> float:
                return mapping.from_output(OutputSample(output_rad, 0.0, 0.0)).q_rad

            def speed_to_joint(output_rpm: float) -> float:
                return mapping.from_output(OutputSample(0.0, output_rpm, 0.0)).dq_rad_s

            try:
                static_pos_raw = project_position_raw(
                    req.q_rad, L, U, position_to_joint, direction
                )
                wanted_static_speed = max(-V, min(V, req.dq_rad_s))
                static_speed_raw = project_speed_raw(
                    wanted_static_speed, -V, V, speed_to_joint, direction
                )
            except (ArithmeticError, ValueError):
                return None

            if static_pos_raw is None or static_speed_raw is None:
                return None

            if not allow_bootstrap:
                if static_pos_raw != previous_key[0] and candidate_key[0] == previous_key[0]:
                    return None
                if static_speed_raw != previous_key[1] and candidate_key[1] == previous_key[1]:
                    return None

        try:
            effort = _predicted_effort(
                _applied_reference(spec, cand), q_meas, measured_velocities[i],
            )
        except (ArithmeticError, ValueError):
            return None
        if abs(effort) > spec.max_effort_nm:
            return None

        candidate_joints.append(cand)

    return tuple(candidate_joints)


def position_rates_at_release(
    specs: tuple[JointSpec, ...],
    previous: tuple[JointTarget, ...],
    previous_rates: tuple[float, ...],
    candidate: tuple[JointTarget, ...],
    elapsed_s: float,
) -> tuple[float, ...] | None:
    """Compute position finite-difference rates at release and enforce acceleration limits.

    Uses uncapped elapsed_s (E = tr - t0). If E == 0, requires exact unchanged
    positions and desired speeds and returns previous_rates. If E > 0, verifies
    strict |w_new| <= V and |w_new - w0| <= A * E for every joint.
    """
    if (
        isinstance(elapsed_s, bool)
        or not isinstance(elapsed_s, (int, float))
        or not math.isfinite(elapsed_s)
        or elapsed_s < 0.0
    ):
        return None
    if len(specs) != len(previous) or len(specs) != len(previous_rates) or len(specs) != len(candidate):
        return None

    if elapsed_s == 0.0:
        for prev, cand in zip(previous, candidate):
            if cand.q_rad != prev.q_rad or cand.dq_rad_s != prev.dq_rad_s:
                return None
        return previous_rates

    w_news: list[float] = []
    for spec, prev, w0, cand in zip(specs, previous, previous_rates, candidate):
        q0 = prev.q_rad
        V = spec.max_velocity_rad_s
        A = spec.max_acceleration_rad_s2

        try:
            A_E = A * elapsed_s
        except (ArithmeticError, ValueError):
            return None
        if not math.isfinite(A_E):
            return None

        try:
            delta_q = cand.q_rad - q0
            w_new = delta_q / elapsed_s
            delta_w = abs(w_new - w0)
        except (ArithmeticError, ValueError):
            return None

        if not math.isfinite(delta_q) or not math.isfinite(w_new) or not math.isfinite(delta_w):
            return None
        if abs(w_new) > V:
            return None
        if delta_w > A_E:
            return None
        w_news.append(w_new)

    return tuple(w_news)
