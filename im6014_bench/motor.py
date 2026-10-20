"""Bounded, single-motor IM6014 bench test. Output-shaft units throughout.

No automatic fault clearing, reset, reconnect, or motion restart.
"""
import argparse
import json
import math
from pathlib import Path
import struct
import time

import serial

RATIO = 38 / 3
RPM_MIN = -32768 / 64 / RATIO * 60
RPM_MAX = 32767 / 64 / RATIO * 60
POSITION_MIN = -2147483648 / 32768 / RATIO * math.tau
POSITION_MAX = 2147483647 / 32768 / RATIO * math.tau
PORT = '/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTC2RQ7H-if03-port0'


def crc32(data):
    if len(data) % 4:
        raise ValueError('CRC input must contain complete four-byte words')
    crc = 0xFFFFFFFF
    for start in range(0, len(data), 4):
        for byte in data[start:start + 4][::-1]:
            crc ^= byte << 24
            for _ in range(8):
                crc = ((crc << 1) ^ (0x04C11DB7 if crc & 0x80000000 else 0)) & 0xFFFFFFFF
    return crc


def command(motor_id, mode=0, rpm=0, kd=0, torque=0, watchdog=True, position=0, kp=0):
    if not 0 <= motor_id <= 14 or mode not in (0, 1):
        raise ValueError('Individual ID 0–14 and Stop/FOC modes only')
    if not all(math.isfinite(v) for v in (rpm, kd, torque, position, kp)):
        raise ValueError('Nonfinite control input')
    if not RPM_MIN <= rpm <= RPM_MAX:
        raise ValueError('Speed cannot be encoded in the motor protocol; this is not a rated-speed limit')
    if not 0 <= kd <= 6 or not 0 <= kp <= 30 or not -3.5 <= torque <= 3.5:
        raise ValueError('Bench limits: Kd 0–6, Kp 0–30, feedforward torque ±3.5 Nm')
    if not POSITION_MIN <= position <= POSITION_MAX:
        raise ValueError('Position cannot be encoded in the motor protocol')
    position_raw = round(position / math.tau * RATIO * 32768)
    if mode == 0 and (rpm or kd or torque or position or kp) or mode == 1 and not watchdog:
        raise ValueError('Stop must have zero targets; FOC requires watchdog')
    raw = b'\xfe\xee' + bytes([motor_id | mode << 4 | int(watchdog) << 7, 0])
    raw += struct.pack('<hhihh', round(torque / RATIO * 2560),
                       round(rpm / 60 * RATIO * 64), position_raw, round(kp / RATIO ** 2 * 12800),
                       round(kd / RATIO ** 2 * 51200))
    return raw + struct.pack('<I', crc32(raw))


def feedback(raw, motor_id):
    if len(raw) != 26 or raw[:2] != b'\xfc\xee':
        raise RuntimeError('Invalid feedback length/header')
    if struct.unpack('<I', raw[22:])[0] != crc32(raw[2:22]):
        raise RuntimeError('Feedback CRC mismatch')
    m, td, tw, v, tq, sp, pos, err, flags, _, _ = struct.unpack('<BbBBhhiIHBB', raw[2:22])
    if m & 15 != motor_id or (m >> 4) & 7 not in (0, 1):
        raise RuntimeError('Feedback ID/mode mismatch')
    return dict(id=m & 15, mode=(m >> 4) & 7, timeout=m >> 7,
                driver_c=td, winding_c=tw, voltage=v / 2,
                torque_nm=tq / 2560 * RATIO, rpm=sp / 64 / RATIO * 60,
                position_rad=pos / 32768 / RATIO * math.tau,
                errors=err, warnings=(flags >> 13) & 7, raw=raw.hex())


def healthy(f, mode, allow_timeout=False, speed_limit=0):
    if f['mode'] != mode or f['errors'] or f['warnings'] or f['timeout'] and not allow_timeout:
        raise RuntimeError('Motor mode/fault/warning/timeout gate: ' + json.dumps(f))
    if not 24 <= f['voltage'] <= 72 or not -10 <= f['driver_c'] <= 70 or f['winding_c'] > 90:
        raise RuntimeError('Voltage/temperature outside bench limits: ' + json.dumps(f))
    if speed_limit and abs(f['rpm']) > speed_limit:
        raise RuntimeError('Measured speed exceeds configured limit: ' + json.dumps(f))
    if abs(f['torque_nm']) > 4:
        raise RuntimeError('Torque exceeds bench limit: ' + json.dumps(f))


def exchange(bus, packet, motor_id, deadline_s=0.05):
    # One owner and one outstanding transaction; discard data older than this request.
    bus.reset_input_buffer()
    deadline = time.monotonic() + deadline_s
    if bus.write(packet) != len(packet):
        raise RuntimeError('Incomplete serial write')
    data = bytearray()
    while time.monotonic() < deadline:
        data.extend(bus.read(min(26, max(1, bus.in_waiting))))
        start = data.find(b'\xfc\xee')
        if start >= 0 and len(data) - start >= 26:
            if time.monotonic() >= deadline:
                break
            return feedback(bytes(data[start:start + 26]), motor_id)
    raise RuntimeError('No validated motor feedback within deadline; received ' + data.hex())


def run(bus, args, log):
    samples = log['samples']

    def transact(mode=0, rpm=0, kd=0, torque=0, watchdog=True, allow_timeout=False, entering_foc=False):
        f = exchange(bus, command(args.id, mode, rpm, kd, torque, watchdog), args.id)
        f.update(elapsed_s=time.monotonic() - started, target_rpm=rpm)
        samples.append(f)
        healthy(f, f['mode'] if entering_foc else mode, allow_timeout, args.speed_limit)
        return f

    started = time.monotonic()
    first = transact(allow_timeout=args.ack_timeout)
    if first['timeout']:
        # Explicit startup-only acknowledgment while stopped, never during motion/recovery.
        transact(watchdog=False, allow_timeout=True)
    initial = transact()
    print('STOP/status verified:', json.dumps(initial), flush=True)
    if not args.arm:
        return
    # Firmware reports the previous state on its first mode-transition reply.
    # Permit this only for the zero-torque arming exchange, then require FOC.
    transact(1, entering_foc=True)
    initial = transact(1)
    start_position = initial['position_rad']
    start = time.monotonic()
    last_reply = start
    next_print = 0
    while True:
        now = time.monotonic()
        elapsed = now - start
        if elapsed >= args.duration:
            break
        if now - last_reply > 0.1:
            raise RuntimeError('Host feedback watchdog expired; motion remains disarmed')
        # 0.75-second acceleration/deceleration ramps, with zero position stiffness.
        ramp = min(1, elapsed / 0.75, (args.duration - elapsed) / 0.75)
        f = transact(1, args.rpm * ramp, args.kd, args.torque_ff * ramp * (1 if args.rpm >= 0 else -1))
        last_reply = time.monotonic()
        if args.travel_limit and abs(f['position_rad'] - start_position) > math.radians(args.travel_limit):
            raise RuntimeError('Exceeded configured output travel limit')
        if elapsed >= next_print:
            print(f"t={elapsed:.2f}s target={args.rpm * ramp:.2f} rpm measured={f['rpm']:.3f} rpm torque={f['torque_nm']:.3f} Nm", flush=True)
            next_print += 0.5
        time.sleep(max(0, 0.01 - (time.monotonic() - now)))
    plateau = [f['rpm'] for f in samples if f.get('target_rpm') == args.rpm and f['mode'] == 1]
    mean_rpm = sum(plateau) / len(plateau) if plateau else 0
    log['mean_rpm'] = mean_rpm
    if args.rpm and abs(mean_rpm - args.rpm) > 0.5:
        raise RuntimeError(f'Measured speed {mean_rpm:.3f} rpm did not reach the requested speed')
    print(f'Measured plateau average: {mean_rpm:.3f} rpm', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default=PORT)
    parser.add_argument('--id', type=int, default=0)
    parser.add_argument('--baud', type=int, choices=(4000000, 6000000), default=6000000)
    parser.add_argument('--rpm', type=float, default=5)
    parser.add_argument('--kd', type=float, default=6)
    # Measured unloaded-bench friction compensation; retune for a different motor/load.
    parser.add_argument('--torque-ff', type=float, default=0.16)
    parser.add_argument('--duration', type=float, default=5)
    parser.add_argument('--speed-limit', type=float, default=0, help='Optional measured-speed trip in rpm; 0 disables it')
    parser.add_argument('--travel-limit', type=float, default=0, help='Optional travel trip in degrees from test start; 0 disables it')
    parser.add_argument('--ack-timeout', action='store_true', help='Explicitly acknowledge startup timeout only in Stop, then enable watchdog')
    parser.add_argument('--arm', action='store_true', help='Allow this bounded unloaded-bench motion test')
    parser.add_argument('--log', default='motor_run.json')
    args = parser.parse_args()
    command(args.id, 1, args.rpm, args.kd, args.torque_ff)
    if args.torque_ff < 0:
        parser.error('Friction compensation must be nonnegative; direction follows --rpm')
    if not math.isfinite(args.duration) or not 2 <= args.duration <= 10:
        parser.error('Duration must be 2–10 seconds')
    if not all(math.isfinite(x) and x >= 0 for x in (args.speed_limit, args.travel_limit)):
        parser.error('Optional speed/travel limits must be finite and nonnegative')
    if args.speed_limit and abs(args.rpm) > args.speed_limit:
        parser.error('Requested speed exceeds the configured measured-speed limit')
    # Confirm the result location is writable before commanding hardware.
    path = Path(args.log)
    log = dict(settings=vars(args), samples=[], stopped=False, error=None)
    path.write_text(json.dumps(log, indent=2) + '\n')
    bus = serial.Serial(port=None, baudrate=args.baud, timeout=0.005, write_timeout=0.05, exclusive=True)
    bus.port = args.port
    bus.dtr = False
    bus.rts = False
    try:
        bus.open()
        run(bus, args, log)
    except (Exception, KeyboardInterrupt) as error:
        log['error'] = str(error) or type(error).__name__
        print('STOPPING:', log['error'], flush=True)
    finally:
        if bus.is_open:
            try:
                for _ in range(3):
                    f = exchange(bus, command(args.id), args.id)
                    log['samples'].append(dict(f, final_stop=True))
                    log['stopped'] = f['mode'] == 0
                    time.sleep(0.02)
                print('Final Stop feedback:', json.dumps(f), flush=True)
            except Exception as error:
                log['stopped'] = False
                log['stop_error'] = str(error)
                print('Stop confirmation unavailable:', error, flush=True)
            bus.close()
        path.write_text(json.dumps(log, indent=2) + '\n')
    if log['error'] or not log['stopped']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
