#!/usr/bin/env python3
from __future__ import annotations
import argparse
import json
import select
import signal
import socket
import time
MAX_BODY = 1024
MODES = ("normal", "silent", "replay", "ticket-replay",
         "flood", "stall", "malformed", "lifecycle", "bool-epoch",
         "bad-type-list", "bad-type-dict", "ignore-term")
def valid_vector(value):
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        return None
    result = []
    for item in value:
        if isinstance(item, bool) or not isinstance(item, (int, float)):
            return None
        item = float(item)
        if not (float("-inf") < item < float("inf")):
            return None
        result.append(item)
    return tuple(result)
def encode(payload):
    body = json.dumps(payload, separators=(",", ":"),
                      allow_nan=False, sort_keys=True).encode("utf-8")
    if not body or len(body) > MAX_BODY:
        raise ValueError("frame body exceeds the bounded size")
    return len(body).to_bytes(4, "big") + body
def ack(wire, session, challenge, epoch, q):
    return wire.send({"type": "ack", "session": session,
                      "challenge": challenge, "epoch": epoch, "q": q})
def reply(wire, session, epoch, sequence, q):
    return wire.send({"type": "reply", "session": session, "epoch": epoch,
                      "sequence": sequence, "q": q})
class Wire:
    def __init__(self, sock):
        self.sock = sock
        self.inbuf = bytearray()
        self.outbuf = bytearray()
    def send(self, payload):
        self.outbuf.extend(encode(payload))
        return self.flush()
    def send_raw(self, data):
        self.outbuf.extend(data)
        return self.flush()
    def flush(self):
        while self.outbuf:
            try:
                sent = self.sock.send(self.outbuf)
            except BlockingIOError:
                return True
            except OSError:
                return False
            if sent <= 0:
                return False
            del self.outbuf[:sent]
        return True
    def receive(self, timeout):
        if not self.flush():
            return None
        try:
            readable, _, _ = select.select([self.sock], [], [], timeout)
        except (OSError, ValueError):
            return None
        if not readable:
            return []
        try:
            data = self.sock.recv(8192)
        except BlockingIOError:
            return []
        except OSError:
            return None
        if not data:
            return None
        self.inbuf.extend(data)
        messages = []
        while len(self.inbuf) >= 4:
            length = int.from_bytes(self.inbuf[:4], "big")
            if length <= 0 or length > MAX_BODY:
                return None
            total = 4 + length
            if len(self.inbuf) < total:
                break
            body = bytes(self.inbuf[4:total])
            del self.inbuf[:total]
            try:
                message = json.loads(body)
            except (ValueError, RecursionError):
                return None
            if not isinstance(message, dict):
                return None
            messages.append(message)
        return messages if self.flush() else None
def idle():
    while True:
        time.sleep(0.05)
def flood(wire, session):
    while True:
        if len(wire.outbuf) < 65536:
            wire.outbuf.extend(encode({"type": "flood", "session": session}))
        if not wire.flush():
            return
        time.sleep(0.001)
def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--fd", type=int, required=True)
    parser.add_argument("--mode", choices=MODES, default="normal")
    args = parser.parse_args(argv)
    sock = socket.socket(fileno=args.fd)
    sock.setblocking(False)
    wire = Wire(sock)
    session = challenge = None
    epoch = 0
    last_q = [0.0, 0.0, 0.0]
    deadline = time.monotonic() + 5.0
    while session is None and time.monotonic() < deadline:
        messages = wire.receive(0.05)
        if messages is None:
            return 0
        for message in messages:
            if message.get("type") != "hello":
                continue
            session = message.get("session")
            challenge = message.get("challenge")
            epoch = message.get("epoch", 0)
            q = valid_vector(message.get("q"))
            if q is not None:
                last_q = list(q)
            break
    if not isinstance(session, str) or not isinstance(challenge, str):
        return 2
    if type(epoch) is not int:
        return 2
    first_challenge = challenge
    ack_epoch = False if args.mode == "bool-epoch" else epoch
    if not wire.send({"type": "hello_ack", "session": session,
                      "challenge": challenge, "epoch": ack_epoch, "q": last_q}):
        return 0
    if args.mode == "bool-epoch":
        idle()
    if args.mode == "ignore-term":
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        idle()
    if args.mode == "malformed":
        wire.send_raw(b"\x00\x00\x00\x05notjs")
        idle()
    if args.mode == "flood":
        flood(wire, session)
    if args.mode == "stall":
        idle()
    if args.mode == "lifecycle":
        if not wire.send({"type": "check", "session": session}):
            return 0
    ticket = None
    while True:
        messages = wire.receive(0.05)
        if messages is None:
            return 0
        for message in messages:
            kind = message.get("type")
            if kind == "ping":
                epoch = message.get("epoch", epoch)
                current = message.get("challenge")
                if not isinstance(current, str) or type(epoch) is not int:
                    continue
                if args.mode in ("normal", "ticket-replay", "lifecycle",
                                 "bad-type-list", "bad-type-dict"):
                    if not ack(wire, session, current, epoch, last_q):
                        return 0
                elif args.mode == "replay":
                    if not ack(wire, session, first_challenge, epoch, last_q):
                        return 0
            elif kind == "capture":
                if args.mode in ("bad-type-list", "bad-type-dict"):
                    bad_type = [] if args.mode == "bad-type-list" else {}
                    if not wire.send({"type": bad_type, "session": session}):
                        return 0
                    idle()
                notice_epoch = message.get("epoch")
                sequence = message.get("sequence")
                q = valid_vector(message.get("q"))
                if type(notice_epoch) is not int or type(sequence) is not int:
                    continue
                if q is None:
                    continue
                last_q = list(q)
                if args.mode == "ticket-replay":
                    if ticket is None:
                        ticket = (session, notice_epoch, sequence, list(q))
                    elif not reply(wire, *ticket):
                        return 0
                if args.mode in ("normal", "ticket-replay"):
                    if not reply(wire, session, notice_epoch, sequence, list(q)):
                        return 0
        if not wire.flush():
            return 0
if __name__ == "__main__":
    raise SystemExit(main())
