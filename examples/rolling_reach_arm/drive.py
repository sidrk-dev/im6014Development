"""Loaded-model factory for the selected rolling-reach native position drive.

This is the selected-profile example boundary, not a general plugin surface.
The factory validates the drive relation only: the loaded actuator, tendon, and
joint metadata used to derive three logical native position servos must exactly
match a freshly compiled canonical scene model.
"""

import math

import mujoco
import numpy as np

from im6014_control.mapping import JointMap
from im6014_control.records import (
    JointSpec,
    NativePositionServo,
    PositionLimits,
)

from . import scene


_METADATA_PREFIXES = ("actuator_", "tendon_", "jnt_", "wrap_")
_DIMENSIONS = ("nq", "nv", "nu", "njnt", "ntendon", "nwrap")
_OBJECTS = (
    (mujoco.mjtObj.mjOBJ_ACTUATOR, "nu"),
    (mujoco.mjtObj.mjOBJ_JOINT, "njnt"),
    (mujoco.mjtObj.mjOBJ_TENDON, "ntendon"),
)


def _names(model, obj_type, count):
    return tuple(
        mujoco.mj_id2name(model, obj_type, index) for index in range(count)
    )


def _validate_loaded_metadata(model):
    canonical = mujoco.MjModel.from_xml_string(scene.generated_xml())

    for name in _DIMENSIONS:
        if getattr(model, name) != getattr(canonical, name):
            raise ValueError(
                f"loaded model {name} does not match the canonical rolling scene"
            )

    for obj_type, count_name in _OBJECTS:
        count = getattr(model, count_name)
        if _names(model, obj_type, count) != _names(canonical, obj_type, count):
            raise ValueError(
                f"loaded model {count_name} names do not match the canonical rolling scene"
            )

    fields = sorted(
        name for name in dir(canonical)
        if name.startswith(_METADATA_PREFIXES)
    )
    for name in fields:
        if not np.array_equal(
            np.asarray(getattr(model, name)),
            np.asarray(getattr(canonical, name)),
        ):
            raise ValueError(
                f"loaded model {name} does not match the canonical rolling scene"
            )


def joint_specs_from_model(model, *, max_measured_lead_rad):
    """Build the three logical native specs from a validated loaded model."""
    if isinstance(max_measured_lead_rad, bool) or not isinstance(
        max_measured_lead_rad, (int, float)
    ):
        raise ValueError("max_measured_lead_rad must be finite and positive")
    try:
        lead = float(max_measured_lead_rad)
    except (OverflowError, TypeError, ValueError) as exc:
        raise ValueError("max_measured_lead_rad must be finite and positive") from exc
    if not math.isfinite(lead) or lead <= 0.0:
        raise ValueError("max_measured_lead_rad must be finite and positive")
    if not isinstance(model, mujoco.MjModel):
        raise ValueError("model must be a compiled MuJoCo model")
    _validate_loaded_metadata(model)

    specs = []
    for index, name in enumerate(scene.SCENARIO.logical_names):
        kp = float(model.actuator_gainprm[index, 0])
        kd = float(-model.actuator_biasprm[index, 2])
        ctrl_min, ctrl_max = (
            float(value) for value in model.actuator_ctrlrange[index]
        )
        force_min, force_max = (
            float(value) for value in model.actuator_forcerange[index]
        )
        drive = NativePositionServo(kp, kd, force_min, force_max)
        specs.append(JointSpec(
            name=name,
            motor_id=index,
            mapping=JointMap(1.0, 1, 0.0),
            position_limits=PositionLimits(ctrl_min, ctrl_max),
            max_velocity_rad_s=scene.SCENARIO.speed_limit,
            max_acceleration_rad_s2=scene.SCENARIO.accel_limit,
            max_measured_lead_rad=lead,
            max_effort_nm=min(-force_min, force_max),
            drive=drive,
        ))
    return tuple(specs)
