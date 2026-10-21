"""Validate the pinned Dream client against the SA01 contract without cloud or motor I/O."""

import argparse
import base64
import hashlib
import importlib.metadata as metadata
import io
import json
from pathlib import Path
import platform
import subprocess
import sys
from datetime import datetime, timezone

from check_single_arm_contract import check_contract
from dream_client_fixture import offline_client, registered_body

try:
    import dream
    from dream.errors import DreamActionError, DreamObservationError
    from PIL import Image
except ImportError as error:
    raise SystemExit(f"FAIL: run with the pinned Dream client environment: {error}") from error

ROOT = Path(__file__).resolve().parents[1]
CONTRACT = ROOT / "docs/single-arm-contract.json"
REQUIREMENTS = ROOT / "examples/rolling_contact_arm/requirements.txt"
PACKAGES = ("mujoco", "yourdfpy", "numpy", "Pillow", "dreamlabs-client", "av", "pyarrow")


def require(condition, message):
    if not condition:
        raise SystemExit(f"FAIL: {message}")


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def jpeg_frame():
    buffer = io.BytesIO()
    Image.new("RGB", (320, 240), (0, 0, 0)).save(buffer, format="JPEG")
    return base64.b64encode(buffer.getvalue()).decode("ascii")


def pinned_versions():
    versions = {}
    for line in REQUIREMENTS.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        name, version = line.split("==", 1)
        name = name.split("[", 1)[0]
        require(metadata.version(name) == version, f"{name} is not pinned to {version}")
        versions[name] = version
    for name in PACKAGES:
        require(name in versions, f"requirements do not pin {name}")
    return versions


def check_registration(data):
    body_dict = data["policy"]["embodiment"]
    body = registered_body(body_dict)
    require(body.plan is not None and body.plan.use_eef is False, "joint-action plan must not use EEF")
    require([slot.name for slot in body.plan.state] == ["right_arm"], "state slot must map to right_arm")
    require([slot.name for slot in body.plan.action] == ["right_arm"], "action slot must map to right_arm")
    require([slot.width for slot in body.plan.state] == [3], "state slot width must be 3")
    require([slot.width for slot in body.plan.action] == [3], "action slot width must be 3")
    require(body.video == {"overhead": "observation.images.overhead"}, "video mapping changed")
    require(body.head_camera is None, "head camera must not be registered")
    return {
        "state_slots": [{"name": slot.name, "part": slot.part, "width": slot.width} for slot in body.plan.state],
        "action_slots": [{"name": slot.name, "part": slot.part, "width": slot.width} for slot in body.plan.action],
        "use_eef": body.plan.use_eef,
        "video": body.video,
        "head_camera": body.head_camera,
        "joints_checked": True,
    }


def check_joint_action(data):
    body_dict = data["policy"]["embodiment"]
    frame = jpeg_frame()
    rows = [[0.1, 0.2, 0.3]]
    negative = []

    with offline_client(
        body_dict, rows, checkpoint="offline-sa01-contract", action_hz=30
    ) as (client, exchange):
        client.set_observation(
            {"video.overhead": frame},
            {"state.arm": [0.0, 0.0, 0.0], "prompt": data["task"]["instruction"]},
            timestamp=0,
        )
        reply = client.get_action()
        require(list(reply["per_key"]) == ["action.arm"], "public action key changed")
        require(reply["per_key"]["action.arm"] == rows, "action rows changed")
        wire = exchange.requests[-1][1]["observation"]
        require(wire["state.right_arm"] == [0.0, 0.0, 0.0], "state width or key changed")
        require("state.arm" not in wire, "public state key leaked to the wire")
        require(wire["video.overhead"] == [frame], "overhead frame was not forwarded")
        require(not any("left" in key or "grip" in key for key in wire), "excluded wire channel found")
        require(wire["prompt"] == data["task"]["instruction"], "task text changed")
        response_rows = len(rows)
        require(response_rows == data["timing"]["action_chunk_rows"], "positive response does not match the contracted action row count")

        before = len(exchange.requests)
        client.set_observation(
            {"video.overhead": frame},
            {"state.arm": [0.0, 0.0], "prompt": data["task"]["instruction"]},
            timestamp=1,
        )
        try:
            client.get_action()
        except DreamObservationError:
            negative.append("SDK rejects two-wide state before transport")
        else:
            raise SystemExit("FAIL: SDK accepted a state width that does not match the contract")
        require(len(exchange.requests) == before, "invalid observation reached transport")

        exchange.rows = [[0.1, 0.2]]
        client.set_observation(
            {"video.overhead": frame},
            {"state.arm": [0.0, 0.0, 0.0], "prompt": data["task"]["instruction"]},
            timestamp=2,
        )
        try:
            client.get_action()
        except DreamActionError:
            negative.append("SDK rejects two-wide action after transport")
        else:
            raise SystemExit("FAIL: SDK accepted an action width that does not match the contract")

    return {
        "checkpoint": "offline-sa01-contract",
        "state_width": 3,
        "action_width": 3,
        "public_action_key": "per_key.action.arm",
        "wire_state_key": "state.right_arm",
        "wire_video_key": "video.overhead",
        "response_rows": response_rows,
        "negative_checks": negative,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", type=Path, help="Write a JSON verification report")
    args = parser.parse_args()

    require(sys.version_info[:2] == (3, 14), f"unexpected Python {platform.python_version()}")
    data = json.loads(CONTRACT.read_text())
    check_contract(data)
    versions = pinned_versions()
    pip = subprocess.run(
        [sys.executable, "-m", "pip", "check"], capture_output=True, text=True, check=False
    )
    require(pip.returncode == 0, f"pip check failed: {pip.stdout}{pip.stderr}")

    registration = check_registration(data)
    joint_action = check_joint_action(data)
    result = {
        "recorded_at_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "verdict": "PASS for installed client and offline schema only",
        "scope": "no hosted model, no cloud call, no motor call, no SA03 physics",
        "python": platform.python_version(),
        "executable": sys.executable,
        "packages": versions,
        "pip_check": pip.stdout.strip(),
        "requirements": {
            "path": REQUIREMENTS.relative_to(ROOT).as_posix(),
            "sha256": digest(REQUIREMENTS),
        },
        "contract": {
            "path": CONTRACT.relative_to(ROOT).as_posix(),
            "sha256": digest(CONTRACT),
        },
        "registration": registration,
        "joint_action": joint_action,
        "dreamlabs_client_version": dream.__version__,
        "hosted_calls": 0,
        "motor_calls": 0,
        "hosted_model_behavior_verified": False,
    }
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(result, indent=2) + "\n")
    print(
        f"PASS: SA02 client {dream.__version__} on Python {platform.python_version()}, "
        f"3-wide joint action, {joint_action['response_rows']}-row response, one overhead camera, "
        f"{len(joint_action['negative_checks'])} negative checks"
    )


if __name__ == "__main__":
    main()
