"""Two logical bend coordinates between Dream's joint-action API and this simulator."""

import base64
import io
import json

import mujoco
import numpy as np
from PIL import Image

from model import HERE, JOINTS, internal_to_logical, logical_to_internal

EMBODIMENT = json.loads((HERE / "dream_embodiment.json").read_text())
ACTION_HZ = 30


def jpeg_frame(rgb):
    buffer = io.BytesIO()
    Image.fromarray(rgb).save(buffer, format="JPEG", quality=85)
    return base64.b64encode(buffer.getvalue()).decode("ascii")


def observe(data, rgb, task):
    return {"video.overhead": jpeg_frame(rgb)}, {
        "state.arm": internal_to_logical(data.qpos).tolist(), "prompt": task}


def targets(reply):
    try:
        rows = np.asarray(reply["per_key"]["action.arm"], dtype=float)
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError("Missing or invalid action.arm chunk") from error
    if rows.ndim != 2 or rows.shape[1] != 2 or len(rows) == 0:
        raise ValueError("Expected a nonempty H by 2 action.arm chunk")
    for row in rows:
        logical_to_internal(row)
    return rows


def apply_chunk(model, data, reply, action_hz=ACTION_HZ):
    rows = targets(reply)
    if not np.isfinite(action_hz) or action_hz <= 0:
        raise ValueError("Action frequency must be finite and positive")
    steps = 1 / (action_hz * model.opt.timestep)
    if not np.isfinite(steps) or steps < 1 or abs(steps-round(steps)) > 1e-9:
        raise ValueError("Action period must be a positive integer number of simulation steps")
    if model.nu != len(JOINTS):
        raise ValueError("Sample requires exactly two logical-bend actuators")
    for row in rows:
        data.ctrl[:] = row
        for _ in range(round(steps)):
            mujoco.mj_step(model, data)
    mujoco.mj_forward(model, data)
