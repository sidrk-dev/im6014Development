"""Exercise the installed Dream SDK and simulated arm with only transport mocked."""

import base64
import copy
import hashlib
import io
import json
import os
from pathlib import Path
from unittest.mock import patch

os.environ["DREAM_LOG"] = "off"

import dream
from dream import arm_file, credentials, transport
from dream.errors import DreamActionError, DreamConfigError, DreamObservationError
from shared.datasets import embodiments
import mujoco
import numpy as np
from PIL import Image

from dream_bridge import EMBODIMENT, apply_chunk, observe, targets
from model import HERE, internal_to_logical


def run():
    output = HERE / "results"
    output.mkdir(exist_ok=True)
    body = embodiments.check_registration(EMBODIMENT)
    embodiments.check_state_width(body)
    embodiments.check_action_width(body)
    embodiments.check_joints(embodiments.trained(body))
    assert body.plan.use_eef is False
    try:
        arm_file._chain((HERE/"arm.urdf").read_bytes(), "base_link", "tool", (), HERE/"arm.urdf")
    except DreamConfigError as error:
        ik_rejection = str(error)
        assert "mimic" in ik_rejection
    else:
        raise AssertionError("Recheck Dream stock IK support; sample mimic chain was accepted")
    model = mujoco.MjModel.from_xml_path(str(HERE / "arm.xml"))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    with mujoco.Renderer(model, height=240, width=320) as renderer:
        renderer.update_scene(data, camera="inspection")
        rgb = renderer.render().copy()
    frames, state = observe(data, rgb, "Move the sample arm toward the marker")
    decoded = Image.open(io.BytesIO(base64.b64decode(frames["video.overhead"])))
    assert decoded.format == "JPEG" and decoded.size == (320, 240)
    requests = []
    response_rows = [[.1, .15], [.11, .16], [.12, .17]]
    creds = credentials.Credentials(gateway_url="http://127.0.0.1:9", platform_url="http://127.0.0.1:9",
                                    data_url="http://127.0.0.1:9", api_key="offline-test-key",
                                    path=Path("/tmp/unused-rolling-sample-credentials"))

    def exchange(self, payload, headers, *, path=transport.GET_ACTION_PATH, timeout_ms=None):
        raw = transport.encode(payload)
        sent = transport.decode(raw)
        requests.append((path, sent))
        if path == transport.END_SESSION_PATH:
            reply = {"status":{"code":"DA-0000"}}
        else:
            reply = {"status":{"code":"DA-0000" if sent["session_id"] else "DA-0001"},
                     "session_id":"offline-rolling-session", "actions":{"per_key":{"action.right_arm":response_rows}}}
            if not sent["session_id"]:
                reply["status"]["reason"] = "new"
        encoded = transport.encode(reply)
        self.last_exchange = transport.Exchange(rtt_ms=1, request_bytes=len(raw), response_bytes=len(encoded))
        return transport.decode(encoded)

    negative = []
    with patch.object(credentials, "load", return_value=creds), patch.object(transport.Transport, "request", exchange):
        client = dream.DreamClient(api_key="offline-test-key")
        client.policy({"checkpoint":"offline-contract-fixture", "embodiment":EMBODIMENT,
                       "strict":True, "_record":False, "action_hz":30})
        client.set_observation(frames, state, timestamp=0)
        reply = client.get_action()
        np.testing.assert_allclose(targets(reply), response_rows)
        first = requests[-1][1]
        wire = first["observation"]
        assert wire["state.right_arm"] == [0,0]
        assert "state.arm" not in wire
        assert wire["video.overhead"] == [frames["video.overhead"]]
        assert not any("left" in key or "grip" in key for key in wire)
        assert list(reply["per_key"]) == ["action.arm"]
        apply_chunk(model, data, reply)
        assert data.time > 0 and np.linalg.norm(data.qpos) > .001
        with mujoco.Renderer(model, height=240, width=320) as renderer:
            renderer.update_scene(data, camera="inspection")
            next_rgb = renderer.render().copy()
        assert not np.array_equal(rgb, next_rgb)
        rgb = next_rgb
        frames, state = observe(data, rgb, "Move the sample arm toward the marker")
        client.set_observation(frames, state, timestamp=float(data.time))
        client.get_action()
        assert requests[-1][1]["session_id"] == "offline-rolling-session"
        np.testing.assert_allclose(requests[-1][1]["observation"]["state.right_arm"], internal_to_logical(data.qpos))
        before = len(requests)
        client.set_observation(frames, {"state.arm":[0], "prompt":"wrong width"}, timestamp=float(data.time)+1)
        try:
            client.get_action()
        except DreamObservationError:
            negative.append("SDK rejects wrong observation width before transport")
        else:
            raise AssertionError("SDK accepted one-wide state")
        assert before == len(requests)
        for rows in ([[]], [[0]], [[0,0,0]], [], [[0,0],[0]]):
            response_rows = rows
            client.set_observation(frames, state, timestamp=float(data.time)+2)
            try:
                client.get_action()
            except DreamActionError:
                negative.append("SDK rejects malformed action shape")
            else:
                raise AssertionError(f"SDK accepted malformed action chunk {rows}")
        for rows in ([[float("nan"),0]], [[0,float("inf")]], [[1.21,0]], [[0,1.41]], [[0,0],[float("nan"),0]]):
            old_control, old_qpos, old_time = data.ctrl.copy(), data.qpos.copy(), data.time
            try:
                apply_chunk(model, data, {"per_key":{"action.arm":rows}})
            except ValueError:
                negative.append("adapter rejects nonfinite or out-of-range chunk atomically")
            else:
                raise AssertionError(f"Adapter accepted invalid target {rows}")
            np.testing.assert_array_equal(data.ctrl, old_control)
            np.testing.assert_array_equal(data.qpos, old_qpos)
            assert data.time == old_time
        client.reset_episode()
        response_rows = [[0,0]]
        client.set_observation(frames, state, timestamp=0)
        client.get_action()
        assert requests[-1][1]["session_id"] is None
        client.close()
    safe_request = copy.deepcopy(first)
    safe_request["observation"]["video.overhead"] = ["<base64 JPEG from MuJoCo camera>"]
    result = {"verdict":"PASS for local installed SDK contract and simulation adapter only",
              "dreamlabs_client_version":dream.__version__, "single_arm":True, "logical_width":2,
              "camera_count":1, "gripper":False, "training_mode":"joint actions, use_eef=False",
              "mocked":"credentials resolution and Transport.request only; no hosted model",
              "wire_observation_keys":list(wire), "public_action_key":"per_key.action.arm",
              "jpeg_size":[320,240], "simulated_seconds_from_sdk_chunk":float(data.time),
              "camera_rerendered_after_motion":True,
              "final_measured_logical_bends":internal_to_logical(data.qpos).tolist(),
              "negative_checks":negative, "session_continuation_and_reset":True,
              "dream_stock_ik_sample_urdf_rejected":ik_rejection,
              "hosted_calls":0, "hosted_model_behavior_verified":False,
              "request_example":safe_request,
              "sha256":{name:hashlib.sha256((HERE/name).read_bytes()).hexdigest()
                        for name in ("dream_bridge.py", "verify_dream.py", "dream_embodiment.json")}}
    (output / "dream_contract.json").write_text(json.dumps(result, indent=2)+"\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    run()
