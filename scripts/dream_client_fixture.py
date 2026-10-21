"""Small offline fixture for the installed Dream client's registered-body and request boundary."""

from contextlib import contextmanager
import os
from pathlib import Path
from unittest.mock import patch

os.environ["DREAM_LOG"] = "off"

import dream
from dream import credentials, transport
from shared.datasets import embodiments

FAKE_GATEWAY = "http://127.0.0.1:9"
FAKE_API_KEY = "offline-test-key"
CREDENTIALS_PATH = Path("/tmp/unused-dream-client-fixture-credentials")


def registered_body(config):
    """Run the installed SDK's local body, width and joint validators."""
    body = embodiments.check_registration(config)
    embodiments.check_state_width(body)
    embodiments.check_action_width(body)
    embodiments.check_joints(embodiments.trained(body))
    return body


class OfflineTransport:
    """Mock ``Transport.request`` while real encode/decode and the client's session lifecycle stay active."""

    def __init__(self, rows, *, part="right_arm", session_id="offline-dream-session"):
        self.rows = rows
        self.part = part
        self.session_id = session_id
        self.requests = []
        self.last_exchange = None

    def __call__(self, payload, headers, *, path=transport.GET_ACTION_PATH, timeout_ms=None):
        if path not in (transport.GET_ACTION_PATH, transport.END_SESSION_PATH):
            raise AssertionError(f"Unexpected offline SDK path: {path}")
        raw = transport.encode(payload)
        sent = transport.decode(raw)
        self.requests.append((path, sent))
        if path == transport.END_SESSION_PATH:
            reply = {"status": {"code": "DA-0000"}}
        else:
            reply = {
                "status": {"code": "DA-0000" if sent["session_id"] else "DA-0001"},
                "session_id": self.session_id,
                "actions": {"per_key": {f"action.{self.part}": self.rows}},
            }
            if not sent["session_id"]:
                reply["status"]["reason"] = "new"
        encoded = transport.encode(reply)
        self.last_exchange = transport.Exchange(
            rtt_ms=1, request_bytes=len(raw), response_bytes=len(encoded)
        )
        return transport.decode(encoded)


def fake_credentials():
    return credentials.Credentials(
        gateway_url=FAKE_GATEWAY,
        platform_url=FAKE_GATEWAY,
        data_url=FAKE_GATEWAY,
        api_key=FAKE_API_KEY,
        path=CREDENTIALS_PATH,
    )


@contextmanager
def offline_client(config, rows, *, checkpoint, part="right_arm", action_hz=30.0,
                   obs_window=None, execution_horizon=None):
    """Open a real DreamClient against a registered body and a loopback-only mocked transport."""
    registered_body(config)
    exchange = OfflineTransport(rows, part=part)
    policy = {
        "checkpoint": checkpoint,
        "embodiment": config,
        "strict": True,
        "_record": False,
        "action_hz": action_hz,
    }
    if obs_window is not None:
        policy["obs_window"] = obs_window
    if execution_horizon is not None:
        policy["execution_horizon"] = execution_horizon
    def request(connection, payload, headers, **kwargs):
        reply = exchange(payload, headers, **kwargs)
        connection.last_exchange = exchange.last_exchange
        return reply

    with (
        patch.object(credentials, "load", return_value=fake_credentials()),
        patch.object(transport.Transport, "request", request),
    ):
        client = dream.DreamClient(api_key=FAKE_API_KEY)
        client.policy(policy)
        try:
            yield client, exchange
        finally:
            client.close()
