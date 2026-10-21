"""Validate the frozen sim_rolling_reach_v1 Dream client boundary with the installed SDK offline."""

import argparse
import base64
from dataclasses import dataclass, replace
from datetime import datetime, timezone
import hashlib
import importlib.metadata as metadata
import io
import json
import math
import os
from pathlib import Path
import platform
import subprocess
import sys

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image  # noqa: E402

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from examples.rolling_reach_arm import scene, reach  # noqa: E402
from scripts.dream_client_fixture import offline_client, registered_body  # noqa: E402
from shared.datasets.embodiments import EmbodimentInvalidError  # noqa: E402

try:
    from dream.errors import DreamActionError, DreamObservationError
except ImportError as error:
    raise SystemExit(f"FAIL: run with the pinned Dream client environment: {error}") from error

RESULTS = HERE / "results"
REQUIREMENTS = ROOT / "examples/rolling_contact_arm/requirements.txt"
PACKAGES = ("mujoco", "yourdfpy", "numpy", "Pillow", "dreamlabs-client", "av", "pyarrow")
SOURCE_FILES = {
    "fixture": ROOT / "scripts/dream_client_fixture.py",
    "verifier": HERE / "verify_dream.py",
    "scene": HERE / "scene.py",
    "reach": HERE / "reach.py",
    "model": ROOT / "examples/rolling_contact_arm/model.py",
    "requirements": REQUIREMENTS,
}
MOCK_TARGET = [0.20, 0.35, -0.45]
FIRST_TARGET = (0.12, 0.18, -0.55)


class ContractError(ValueError):
    """A local contract violation, rejected before any transport or simulator mutation."""


@dataclass(frozen=True)
class CameraProfile:
    name: str = "overhead"
    role: str = "overhead"
    width: int = 320
    height: int = 240
    hz: int = 30


@dataclass(frozen=True)
class TimingProfile:
    physics_hz: int = 3000
    control_hz: int = 100
    action_hz: int = 30
    history: int = 1
    chunk_rows: int = 1


@dataclass(frozen=True)
class SelectedProfile:
    model: str = "sim_rolling_reach_v1"
    revision: str = "rolling-reach-reference-v1"
    checkpoint: str = "offline-sim-rolling-reach-v1"
    state_part: str = "arm"
    action_part: str = "arm"
    wire_state_slot: str = "right_arm"
    logical_names: tuple = ("base_yaw", "shoulder_bend", "elbow_bend")
    internal_names: tuple = ("yaw", "shoulder_a", "shoulder_b", "elbow_a", "elbow_b")
    units: str = "rad"
    action_meaning: str = "absolute logical joint targets in radians"
    camera: CameraProfile = CameraProfile()
    timing: TimingProfile = TimingProfile()


@dataclass(frozen=True)
class Observation:
    rgb: np.ndarray
    joints: tuple
    prompt: str
    timestamp: float


PROFILE = SelectedProfile()
CANONICAL_PROFILE = SelectedProfile()
MAPPING = np.array([
    [1., 0., 0., 0., 0.],
    [0., 1., 1., 0., 0.],
    [0., 0., 0., 1., 1.],
])


def _require(condition, message):
    if not condition:
        raise ContractError(message)


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def embodiment(profile=PROFILE):
    part = profile.state_part
    return {
        "state": {part: [0, 3]},
        "action": {profile.action_part: [0, 3]},
        "video": {profile.camera.role: f"observation.images.{profile.camera.role}"},
        "parts": {
            "state": {part: {"kind": "arm", "side": "single"}},
            "action": {profile.action_part: {"kind": "arm", "side": "single"}},
        },
        "actions": "absolute",
    }


def check_profile(profile):
    _require(profile == CANONICAL_PROFILE, "selected profile differs from the frozen contract")


def check_manifest(profile, manifest):
    selected = scene.effective_manifest()
    for section in ("camera", "timing", "limits"):
        _require(manifest[section] == selected[section], f"manifest {section} changed")
    _require(manifest["model"] == profile.model, "manifest model changed")
    _require(manifest["model_revision"] == profile.revision, "manifest model revision changed")
    _require(tuple(manifest["logical_coordinates"]) == profile.logical_names,
             "manifest logical coordinate order changed")
    _require(tuple(manifest["internal_coordinates"]) == profile.internal_names,
             "manifest internal coordinate order changed")
    _require(manifest["action_semantics"] == profile.action_meaning,
             "manifest action semantics changed")
    _require(manifest["units"] == "meters, kilograms, radians, seconds", "manifest units changed")
    _require(np.array_equal(np.asarray(manifest["internal_to_logical"], dtype=float), MAPPING),
             "manifest internal-to-logical mapping changed")

    camera = manifest["camera"]
    _require(camera["name"] == profile.camera.name, "manifest camera name changed")
    _require((camera["width"], camera["height"]) == (profile.camera.width, profile.camera.height),
             "manifest camera dimensions changed")

    timing = manifest["timing"]
    _require(round(1. / timing["timestep_s"]) == profile.timing.physics_hz,
             "manifest physics rate changed")
    _require(timing["control_hz"] == profile.timing.control_hz,
             "manifest local control rate changed")
    _require(timing["camera_hz"] == profile.camera.hz, "manifest camera rate changed")
    _require(timing["camera_decimation"] == profile.timing.physics_hz // profile.camera.hz,
             "manifest camera decimation changed")
    _require(timing["control_decimation"] == profile.timing.physics_hz // profile.timing.control_hz,
             "manifest control decimation changed")


def check_registration(profile, body):
    _require(body.plan is not None and body.plan.use_eef is False,
             "selected registration must use a joint plan without EEF")
    _require([(slot.name, slot.part, slot.width) for slot in body.plan.state]
             == [(profile.wire_state_slot, profile.state_part, 3)],
             "state plan is not one three-wide single arm")
    _require([(slot.name, slot.part, slot.width) for slot in body.plan.action]
             == [(profile.wire_state_slot, profile.action_part, 3)],
             "action plan is not one three-wide single arm")
    _require(body.video == {profile.camera.role: f"observation.images.{profile.camera.role}"},
             "selected registration must name one overhead camera")
    _require(body.head_camera is None, "selected registration must not name a head camera")


def logical_limits(manifest):
    return (manifest["limits"]["yaw_rad"], *manifest["limits"]["bend_rad"])


def encode_frame(profile, rgb):
    _require(isinstance(rgb, np.ndarray), "frame must be a numpy array")
    _require(rgb.dtype == np.uint8, "frame must be uint8 RGB")
    _require(rgb.shape == (profile.camera.height, profile.camera.width, 3),
             "frame must be 240x320x3 RGB")
    _require(float(rgb.std()) > 1., "frame must show a nonconstant scene")
    buffer = io.BytesIO()
    Image.fromarray(rgb).save(buffer, format="JPEG", quality=85)
    payload = buffer.getvalue()
    decoded = Image.open(io.BytesIO(payload))
    decoded.load()
    _require(decoded.format == "JPEG", "encoded frame is not JPEG")
    _require(decoded.mode == "RGB", "encoded frame is not RGB")
    _require(decoded.size == (profile.camera.width, profile.camera.height),
             "encoded frame dimensions changed")
    _require(float(np.asarray(decoded).std()) > 1., "encoded frame lost the scene")
    return base64.b64encode(payload).decode("ascii"), payload


def check_observation(profile, manifest, observation):
    _require(isinstance(observation, Observation), "observation record changed")
    _require(isinstance(observation.joints, (tuple, list))
             and len(observation.joints) == 3
             and all(isinstance(value, (int, float, np.integer, np.floating))
                     and not isinstance(value, (bool, np.bool_)) for value in observation.joints),
             "state must contain three real numbers")
    joints = np.asarray(observation.joints, dtype=float)
    _require(joints.shape == (3,), "state must be three logical coordinates")
    _require(np.isfinite(joints).all(), "state must be finite measured joints")
    limits = logical_limits(manifest)
    _require(all(abs(float(value)) <= limit + 1e-9 for value, limit in zip(joints, limits)),
             "state is outside the logical joint limits")
    _require(observation.prompt == scene.SCENARIO.instruction,
             "task text changed")
    _require(isinstance(observation.timestamp, (int, float))
             and not isinstance(observation.timestamp, bool)
             and math.isfinite(observation.timestamp)
             and observation.timestamp >= 0.,
             "timestamp must be finite, nonnegative seconds")
    frame, payload = encode_frame(profile, observation.rgb)
    return {
        "state.arm": joints.tolist(),
        "prompt": observation.prompt,
    }, frame, payload


def check_action(profile, manifest, reply):
    _require(isinstance(reply, dict), "action reply must be an object")
    per_key = reply.get("per_key")
    _require(isinstance(per_key, dict), "action reply must carry per_key")
    key = f"action.{profile.action_part}"
    _require(list(per_key) == [key], f"per_key must be exactly ['{key}'], got {list(per_key)}")
    rows = per_key[key]
    _require(isinstance(rows, list) and len(rows) == profile.timing.chunk_rows,
             f"action must be {profile.timing.chunk_rows} row")
    limits = logical_limits(manifest)
    result = []
    for row in rows:
        _require(isinstance(row, list) and len(row) == 3, "action row must be three wide")
        _require(all(isinstance(value, (int, float, np.integer, np.floating))
                     and not isinstance(value, (bool, np.bool_)) for value in row),
                 "action row must contain real numbers")
        values = np.asarray(row, dtype=float)
        _require(np.isfinite(values).all(), "action row must be finite")
        _require(all(abs(float(value)) <= limit + 1e-9 for value, limit in zip(values, limits)),
                 "action row is outside the logical joint limits")
        result.append([float(value) for value in values])
    return result


def advance(model, data, target, ticks):
    data.ctrl[:] = target
    for _ in range(ticks):
        mujoco.mj_step(model, data, nstep=scene.SCENARIO.camera_decimation)


def sample(renderer, data, profile):
    renderer.update_scene(data, camera=profile.camera.name)
    rgb = renderer.render().copy()
    joints = tuple(float(value) for value in scene.internal_to_logical(data.qpos))
    return Observation(rgb, joints, scene.SCENARIO.instruction, float(data.time))


def pinned_versions():
    versions = {}
    for line in REQUIREMENTS.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        name, version = line.split("==", 1)
        name = name.split("[", 1)[0]
        _require(metadata.version(name) == version, f"{name} is not pinned to {version}")
        versions[name] = version
    for name in PACKAGES:
        _require(name in versions, f"requirements do not pin {name}")
    return versions


def expect_contract(label, call, negatives):
    try:
        call()
    except ContractError as error:
        negatives.append({"check": label, "layer": "local contract", "detail": str(error)})
    else:
        raise SystemExit(f"FAIL: local contract accepted {label}")


def expect_registration(label, config, negatives):
    try:
        check_registration(PROFILE, registered_body(config))
    except (ContractError, EmbodimentInvalidError) as error:
        negatives.append({"check": label, "layer": "SDK registration/local plan", "detail": str(error)})
    else:
        raise SystemExit(f"FAIL: registration accepted {label}")


def metadata_negatives(manifest, negatives):
    cases = {
        "wrong local model": lambda m: m.update(model="other_body"),
        "wrong local model revision": lambda m: m.update(model_revision="other-revision"),
        "reordered same-width logical names": lambda m: m.update(
            logical_coordinates=list(reversed(m["logical_coordinates"]))),
        "reordered same-width internal names": lambda m: m.update(
            internal_coordinates=["yaw", "shoulder_b", "shoulder_a", "elbow_a", "elbow_b"]),
        "wrong action semantics": lambda m: m.update(action_semantics="delta joint velocities"),
        "wrong units": lambda m: m.update(units="meters, kilograms, degrees, seconds"),
        "five internal coordinates with passive part": lambda m: m.update(
            internal_coordinates=[*m["internal_coordinates"], "passive_wheel"]),
        "wrong camera role": lambda m: m["camera"].update(name="wrist"),
        "wrong camera dimensions": lambda m: m["camera"].update(width=640),
    }
    for label, mutate in cases.items():
        changed = json.loads(json.dumps(manifest))
        mutate(changed)
        expect_contract(label, lambda changed=changed: check_manifest(PROFILE, changed), negatives)

    for label, timing in (
        ("action_hz drifted to local control 100", replace(PROFILE.timing, action_hz=100)),
        ("history drifted to two frames", replace(PROFILE.timing, history=2)),
        ("action chunk drifted to two rows", replace(PROFILE.timing, chunk_rows=2)),
    ):
        changed = replace(PROFILE, timing=timing)
        expect_contract(label, lambda changed=changed: check_profile(changed), negatives)

    two_wide = embodiment()
    two_wide["state"] = {"arm": [0, 2]}
    two_wide["action"] = {"arm": [0, 2]}
    expect_registration("two-wide state/action", two_wide, negatives)

    renamed = embodiment()
    renamed["action"] = {"joints": [0, 3]}
    renamed["parts"]["action"] = {"joints": {"kind": "arm", "side": "single"}}
    expect_registration("renamed same-width action part", renamed, negatives)

    five_wide = embodiment()
    five_wide["state"] = {"arm": [0, 3], "passive": [3, 5]}
    five_wide["action"] = {"arm": [0, 3], "passive": [3, 5]}
    five_wide["parts"]["state"]["passive"] = {"kind": "other"}
    five_wide["parts"]["action"]["passive"] = {"kind": "other"}
    expect_registration("five-wide state/action with passive part", five_wide, negatives)

    left = embodiment()
    left["state"]["left_arm"] = [3, 6]
    left["action"]["left_arm"] = [3, 6]
    left["parts"]["state"]["left_arm"] = {"kind": "arm", "side": "left"}
    left["parts"]["action"]["left_arm"] = {"kind": "arm", "side": "left"}
    expect_registration("extra left-arm part", left, negatives)

    gripper = embodiment()
    gripper["state"]["gripper"] = [3, 4]
    gripper["action"]["gripper"] = [3, 4]
    gripper["parts"]["state"]["gripper"] = {"kind": "gripper", "side": "single",
                                            "closed": 0., "open": 1.}
    gripper["parts"]["action"]["gripper"] = {"kind": "gripper", "side": "single",
                                             "closed": 0., "open": 1.}
    expect_registration("extra gripper part", gripper, negatives)

    head = embodiment()
    head["head_camera"] = "overhead"
    expect_registration("head camera role", head, negatives)

    eef = embodiment()
    eef["state"]["right_eef_xyz"] = [3, 6]
    eef["parts"]["state"]["right_eef_xyz"] = {"kind": "position", "side": "right",
                                              "point": "grasp", "frame": "arm_base"}
    expect_registration("extra EEF part", eef, negatives)


def observation_negatives(manifest, observation, negatives):
    cases = {
        "two-wide state": replace(observation, joints=(0., 0.)),
        "five-wide internal state": replace(observation, joints=(0., 0., 0., 0., 0.)),
        "nonfinite measured state": replace(observation, joints=(0., math.nan, 0.)),
        "out-of-range measured state": replace(observation, joints=(0., 0., math.pi)),
        "float frame instead of uint8 RGB": replace(observation, rgb=observation.rgb.astype(float)),
        "blank constant frame": replace(observation, rgb=np.zeros_like(observation.rgb)),
        "wrong frame shape": replace(observation, rgb=observation.rgb[:120]),
        "nonfinite timestamp": replace(observation, timestamp=math.nan),
        "negative timestamp": replace(observation, timestamp=-1.),
        "wrong task text": replace(observation, prompt="reach elsewhere"),
    }
    for label, changed in cases.items():
        expect_contract(label, lambda changed=changed: check_observation(PROFILE, manifest, changed),
                        negatives)


def action_negatives(manifest, target, negatives):
    key = "action.arm"
    cases = {
        "extra left-arm action key": {"per_key": {key: [target], "action.left_arm": [target]}},
        "wrong action width": {"per_key": {key: [[0., 0.]]}},
        "empty action rows": {"per_key": {key: []}},
        "multirow action chunk": {"per_key": {key: [target, target]}},
        "nonfinite action": {"per_key": {key: [[math.nan, 0., 0.]]}},
        "out-of-range action": {"per_key": {key: [[1.21, 0., 0.]]}},
        "ragged action rows": {"per_key": {key: [[0., 0., 0.], [0., 0.]]}},
    }
    for label, reply in cases.items():
        expect_contract(label, lambda reply=reply: check_action(PROFILE, manifest, reply), negatives)


def run():
    _require(sys.version_info[:2] == (3, 14), f"unexpected Python {platform.python_version()}")
    profile = PROFILE
    check_profile(profile)
    manifest = scene.effective_manifest()
    check_manifest(profile, manifest)
    config = embodiment(profile)
    body = registered_body(config)
    check_registration(profile, body)

    versions = pinned_versions()
    pip = subprocess.run([sys.executable, "-m", "pip", "check"],
                         capture_output=True, text=True, check=False)
    _require(pip.returncode == 0, f"pip check failed: {pip.stdout}{pip.stderr}")

    negatives = []
    metadata_negatives(manifest, negatives)

    model = reach.build_model()
    _require(model.njnt == 5 and model.nu == 3, "selected native model joint/actuator width changed")
    _require(model.camera(profile.camera.name).id >= 0, "native model has no selected overhead camera")
    data = mujoco.MjData(model)
    data.qpos[:] = scene.logical_to_internal(scene.SCENARIO.home)
    data.qvel[:] = 0.
    data.ctrl[:] = scene.SCENARIO.home
    mujoco.mj_forward(model, data)

    with mujoco.Renderer(model, height=profile.camera.height, width=profile.camera.width) as renderer:
        advance(model, data, FIRST_TARGET, 8)
        first_observed = sample(renderer, data, profile)
        observation_negatives(manifest, first_observed, negatives)
        action_negatives(manifest, MOCK_TARGET, negatives)

        state_one, frame_one, jpeg_one = check_observation(profile, manifest, first_observed)
        with offline_client(
            config,
            [MOCK_TARGET],
            checkpoint=profile.checkpoint,
            part=profile.wire_state_slot,
            action_hz=profile.timing.action_hz,
            obs_window=profile.timing.history,
            execution_horizon=profile.timing.chunk_rows,
        ) as (client, exchange):
            client.set_observation({"video.overhead": frame_one}, state_one,
                                   timestamp=first_observed.timestamp)
            reply = client.get_action()
            first_request = exchange.requests[-1][1]
            _require(first_request["session_id"] is None, "first request must open a session")
            wire = first_request["observation"]
            _require(wire["state.right_arm"] == state_one["state.arm"],
                     "wire state is not the measured logical state")
            _require("state.arm" not in wire, "public state key leaked to the wire")
            _require(wire["video.overhead"] == [frame_one],
                     "wire is not the exact first rendered JPEG")
            _require(wire["prompt"] == scene.SCENARIO.instruction, "wire task text changed")
            _require(not any(fragment in key for fragment in
                             ("left_arm", "gripper", "head_camera", "right_eef", "left_eef")
                             for key in wire), "excluded wire channel found")
            decoded = check_action(profile, manifest, reply)
            _require(decoded == [MOCK_TARGET], "decoded action order/key changed")
            _require(decoded[0] != state_one["state.arm"],
                     "mocked target must differ from measured state")

            advance(model, data, decoded[0], 8)
            second_observed = sample(renderer, data, profile)
            _require(any(abs(value) > 1e-3 for value in first_observed.joints),
                     "first measured logical state is all zero")
            _require(any(abs(value) > 1e-3 for value in second_observed.joints),
                     "second measured logical state is all zero")
            _require(max(abs(a - b) for a, b in zip(first_observed.joints, second_observed.joints)) > 1e-3,
                     "measured logical state did not change after physics motion")
            _require(not np.array_equal(first_observed.rgb, second_observed.rgb),
                     "rendered scene did not change after physics motion")

            state_two, frame_two, jpeg_two = check_observation(profile, manifest, second_observed)
            _require(jpeg_one != jpeg_two, "JPEG frames did not change after physics motion")
            client.set_observation({"video.overhead": frame_two}, state_two,
                                   timestamp=second_observed.timestamp)
            reply = client.get_action()
            second_request = exchange.requests[-1][1]
            _require(second_request["session_id"] == exchange.session_id,
                     "second request did not continue the session")
            _require(second_request["observation"]["state.right_arm"] == state_two["state.arm"],
                     "continued request state is not the second measured state")
            _require(second_request["observation"]["video.overhead"] == [frame_two],
                     "continued request did not send the exact second JPEG")
            _require(check_action(profile, manifest, reply) == [MOCK_TARGET],
                     "continued action decode changed")

            before = len(exchange.requests)
            try:
                client.set_observation({"video.overhead": frame_two},
                                       {"state.arm": [0., 0.], "prompt": scene.SCENARIO.instruction},
                                       timestamp=second_observed.timestamp + 1.)
                client.get_action()
            except DreamObservationError:
                negatives.append({
                    "check": "SDK rejects two-wide state before transport",
                    "layer": "SDK observation check",
                    "detail": "DreamObservationError",
                })
            else:
                raise SystemExit("FAIL: SDK accepted a two-wide state")
            _require(len(exchange.requests) == before, "invalid observation reached transport")

            sdk_targets = {
                "SDK rejects two-wide action row": [[0., 0.]],
                "SDK rejects empty action chunk": [],
                "SDK rejects ragged action chunk": [[0., 0., 0.], [0., 0.]],
                "SDK rejects nonnumeric action row": [["bad", 0., 0.]],
            }
            for label, rows in sdk_targets.items():
                exchange.rows = rows
                snapshot = (data.qpos.copy(), data.time)
                client.set_observation({"video.overhead": frame_two}, state_two,
                                       timestamp=second_observed.timestamp + 2.)
                try:
                    client.get_action()
                except DreamActionError:
                    negatives.append({"check": label, "layer": "SDK action check",
                                      "detail": "DreamActionError"})
                else:
                    raise SystemExit(f"FAIL: {label} was accepted")
                _require(np.array_equal(data.qpos, snapshot[0]) and data.time == snapshot[1],
                         "SDK-rejected action mutated the simulator")

            local_targets = {
                "local finite check rejects NaN action": [[math.nan, 0., 0.]],
                "local range check rejects out-of-range action": [[1.21, 0., 0.]],
                "local chunk check rejects multirow action": [MOCK_TARGET, MOCK_TARGET],
            }
            for label, rows in local_targets.items():
                exchange.rows = rows
                snapshot = (data.qpos.copy(), data.time)
                client.set_observation({"video.overhead": frame_two}, state_two,
                                       timestamp=second_observed.timestamp + 3.)
                reply = client.get_action()
                try:
                    check_action(profile, manifest, reply)
                except ContractError as error:
                    negatives.append({"check": label, "layer": "local action check",
                                      "detail": str(error)})
                else:
                    raise SystemExit(f"FAIL: {label} was accepted")
                _require(np.array_equal(data.qpos, snapshot[0]) and data.time == snapshot[1],
                         "rejected action mutated the simulator")

            exchange.rows = [MOCK_TARGET]
            end_sessions_before = sum(path == "/v1/end_session"
                                      for path, _ in exchange.requests)
            client.reset_episode()
            _require(sum(path == "/v1/end_session" for path, _ in exchange.requests)
                     == end_sessions_before + 1, "reset did not end the held session")
            client.set_observation({"video.overhead": frame_two}, state_two, timestamp=0.)
            reply = client.get_action()
            _require(exchange.requests[-1][1]["session_id"] is None,
                     "reset did not start a new session")
            _require(check_action(profile, manifest, reply) == [MOCK_TARGET],
                     "post-reset action decode changed")
            end_sessions_before = sum(path == "/v1/end_session"
                                      for path, _ in exchange.requests)

        _require(sum(path == "/v1/end_session" for path, _ in exchange.requests)
                 == end_sessions_before + 1, "close did not end the held session")

    report = {
        "recorded_at_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "verdict": "PASS for the frozen local selected profile, installed SDK, and simulator boundary",
        "scope": "no hosted model, no cloud call, no motor call, no learned-behavior claim",
        "python": platform.python_version(),
        "executable": sys.executable,
        "packages": versions,
        "pip_check": pip.stdout.strip(),
        "profile": {
            "model": profile.model,
            "model_revision": profile.revision,
            "checkpoint_label": profile.checkpoint,
            "logical_coordinates": list(profile.logical_names),
            "internal_coordinates": list(profile.internal_names),
            "units": profile.units,
            "action_semantics": profile.action_meaning,
            "state_part": profile.state_part,
            "action_part": profile.action_part,
            "wire_state_key": "state.right_arm",
            "public_action_key": "per_key.action.arm",
            "camera": {
                "role": profile.camera.role,
                "name": profile.camera.name,
                "width": profile.camera.width,
                "height": profile.camera.height,
                "hz": profile.camera.hz,
                "history": profile.timing.history,
            },
            "timing": {
                "physics_hz": profile.timing.physics_hz,
                "local_control_hz": profile.timing.control_hz,
                "action_row_hz": profile.timing.action_hz,
                "note": "action_hz is 30, not the 100 Hz local control rate",
                "history": profile.timing.history,
                "chunk_rows": profile.timing.chunk_rows,
            },
        },
        "registration": {
            "state_slots": [{"name": slot.name, "part": slot.part, "width": slot.width}
                            for slot in body.plan.state],
            "action_slots": [{"name": slot.name, "part": slot.part, "width": slot.width}
                             for slot in body.plan.action],
            "use_eef": body.plan.use_eef,
            "video": body.video,
            "head_camera": body.head_camera,
            "joints_checked": True,
        },
        "hashes": {
            "sources": {name: {"path": path.relative_to(ROOT).as_posix(), "sha256": digest(path)}
                        for name, path in SOURCE_FILES.items()},
            "manifest_sha256": hashlib.sha256(
                json.dumps(manifest, sort_keys=True, separators=(",", ":")).encode()
            ).hexdigest(),
        },
        "evidence": {
            "simulated_seconds": float(data.time),
            "physics_steps": int(round(data.time / scene.SCENARIO.timestep)),
            "first_measured_state_rad": list(first_observed.joints),
            "second_measured_state_rad": list(second_observed.joints),
            "measured_state_changed": True,
            "rendered_frames_distinct": True,
            "jpeg_size": [profile.camera.width, profile.camera.height],
            "jpeg_mode": "RGB",
            "jpeg_format": "JPEG",
            "jpeg_sha256": {
                "first": hashlib.sha256(jpeg_one).hexdigest(),
                "second": hashlib.sha256(jpeg_two).hexdigest(),
            },
            "image_bytes_omitted": True,
            "wire_state_keys": list(first_request["observation"]),
            "decoded_action_rows": [MOCK_TARGET],
            "session_continued": True,
            "session_reset": True,
            "session_closed": True,
        },
        "negative_checks": negatives,
        "negative_count": len(negatives),
        "hosted_calls": 0,
        "motor_calls": 0,
        "hosted_model_behavior_verified": False,
        "learned_behavior_verified": False,
        "checkpoint_label_proves_local_serialization_only": True,
        "vector_semantic_limitation": (
            "A mocked SDK vector cannot prove hosted checkpoint training identity or learned semantics; "
            "local metadata/order/units checks bind this profile only."
        ),
    }
    return report


def resolve_output(path=None):
    _require(RESULTS.resolve() == HERE.resolve() / "results",
             "results directory must not redirect outside the example")
    output = (Path(path) if path else RESULTS / "sa02-client.json").resolve()
    _require(RESULTS.resolve() in output.parents,
             "--output must resolve under examples/rolling_reach_arm/results/")
    return output


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path,
                        help="JSON report under examples/rolling_reach_arm/results/")
    args = parser.parse_args(argv)
    output = resolve_output(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    report = run()
    output.write_text(json.dumps(report, indent=2) + "\n")
    print(
        f"PASS: {PROFILE.model}/{PROFILE.revision} on {report['python']}; "
        f"action_hz={PROFILE.timing.action_hz}, one {PROFILE.camera.width}x{PROFILE.camera.height} "
        f"overhead RGB, {report['negative_count']} negative checks"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
