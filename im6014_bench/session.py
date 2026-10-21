"""One serial owner for the desktop dashboard; packet functions live in motor.

No motor operation occurs on import or construction. UI requests are serialized.
"""
from collections import deque
import copy
import math
import queue
import struct
import threading
import time

import serial

from .motor import PORT, RATIO, command, crc32, exchange, healthy

ERRORS = {0: 'Overcurrent', 1: 'Transient overvoltage', 2: 'Sustained overvoltage',
          3: 'Undervoltage', 4: 'Chip overheating', 5: 'MOS temperature outside range',
          6: 'MOS temperature sensor', 8: 'Housing temperature sensor', 9: 'Winding overheating',
          10: 'Rotor encoder', 12: 'Output encoder', 13: 'Stored data', 14: 'Abnormal reset',
          16: 'Chip verification', 17: 'Calibration mode', 19: 'Driver version too low',
          20: 'Firmware model mismatch', 22: 'Hardware overcurrent'}


def fault_text(errors, warnings):
    result = [name for bit, name in ERRORS.items() if errors & (1 << bit)]
    unknown = errors & ~sum(1 << bit for bit in ERRORS)
    if unknown:
        result.append(f'Unknown error bits 0x{unknown:08x}')
    if warnings & 1:
        result.append('Low-voltage power supply warning')
    if warnings & ~1:
        result.append(f'Unknown warning bits 0x{warnings & ~1:x}')
    return '; '.join(result) or 'No faults or warnings'


def validate_test(values):
    kind = values.get('kind')
    if kind not in ('Speed', 'Position', 'Torque pulse', 'Damping'):
        raise ValueError('Choose Speed, Position, Torque pulse, or Damping')
    v = {k: float(values[k]) for k in ('rpm', 'kd', 'ff', 'torque', 'duration')}
    v.update({k: float(values.get(k, default)) for k, default in
              [('position_deg', 10), ('kp', 3), ('speed_limit', 0), ('travel_limit', 0)]})
    if not all(math.isfinite(x) for x in v.values()):
        raise ValueError('Test values must be finite')
    packet = command(0, 1, rpm=v['rpm'], kd=v['kd'], kp=v['kp'], position=math.radians(v['position_deg']))
    if not 0 <= v['ff'] <= 0.5:
        raise ValueError('Friction compensation must be 0–0.5 Nm')
    if not -1 <= v['torque'] <= 1:
        raise ValueError('Torque pulse must be within ±1 Nm')
    if v['speed_limit'] < 0 or v['travel_limit'] < 0:
        raise ValueError('Optional speed/travel limits must be nonnegative; 0 means Off')
    if kind in ('Speed', 'Position') and v['speed_limit'] and abs(v['rpm']) > v['speed_limit']:
        raise ValueError('Requested speed exceeds the configured measured-speed limit')
    reference = values.get('position_reference', 'Relative')
    if reference not in ('Relative', 'Absolute'):
        raise ValueError('Position reference must be Relative or Absolute')
    if kind == 'Position' and (v['rpm'] <= 0 or packet[12:14] == b'\0\0'):
        raise ValueError('Position requires a positive move rate and an encodable nonzero Kp')
    lo, hi = (0.1, 0.5) if kind == 'Torque pulse' else (2, 60 if kind == 'Position' else 10)
    if not lo <= v['duration'] <= hi:
        raise ValueError(f'{kind} duration must be {lo}–{hi} seconds')
    return dict(v, kind=kind, position_reference=reference)


class DemoSerial:
    """Packet-level simulator; ID 0/1 exist. Never accesses a real device."""
    is_open = True
    in_waiting = 26
    timeout = 0.005

    def __init__(self):
        self.data = b''
        self.states = {i: dict(mode=0, timeout=1, rpm=0., pos=0., torque=0.) for i in (0, 1)}
        self.previous = time.monotonic()
        self.inject = None

    def reset_input_buffer(self):
        self.data = b''

    def write(self, packet):
        now = time.monotonic()
        dt = min(0.1, now - self.previous)
        self.previous = now
        motor_id, mode, wd = packet[2] & 15, (packet[2] >> 4) & 7, packet[2] >> 7
        if motor_id not in self.states or self.inject == 'timeout':
            return len(packet)
        state = self.states[motor_id]
        old_mode, old_timeout = state['mode'], state['timeout']
        torque, speed, position, kp, kd = struct.unpack('<hhihh', packet[4:16])
        state['mode'] = mode
        if not wd and mode == 0:
            state['timeout'] = 0
        target = speed / 64 / RATIO * 60
        if not mode:
            state['rpm'] = 0.
            state['torque'] = 0.
        elif not state['timeout']:
            if kp:
                # ponytail: unloaded toy dynamics, not a calibrated model of a real arm.
                error = position / 32768 / RATIO * math.tau - state['pos']
                state['torque'] = kp / 12800 * RATIO ** 2 * error - kd / 51200 * RATIO ** 2 * state['rpm'] * math.tau / 60
                state['rpm'] += state['torque'] / .05 * dt * 60 / math.tau
            elif kd:
                state['rpm'] += (target - state['rpm']) * min(1, dt * 12)
            else:
                state['rpm'] += torque / 2560 * RATIO * dt * 2
            if not kp:
                state['torque'] = torque / 2560 * RATIO + kd / 51200 * RATIO ** 2 * (target - state['rpm']) * math.tau / 60
        state['pos'] += state['rpm'] * math.tau / 60 * dt
        body = struct.pack('<BbBBhhiIHBB', motor_id | old_mode << 4 | old_timeout << 7,
                           35, 31, 72, round(state['torque'] / RATIO * 2560),
                           round(state['rpm'] / 60 * RATIO * 64), round(state['pos'] / math.tau * RATIO * 32768),
                           1 if self.inject == 'fault' else 0, 0, 0, 0)
        self.data = b'\xfc\xee' + body + struct.pack('<I', crc32(body))
        if self.inject == 'crc':
            self.data = self.data[:-1] + bytes([self.data[-1] ^ 1])
        return len(packet)

    def read(self, size):
        data, self.data = self.data[:size], self.data[size:]
        if not data:
            time.sleep(0.001)
        return data

    def close(self):
        self.is_open = False


class MotorSession:
    def __init__(self):
        self.lock = threading.RLock()
        self.jobs = queue.Queue()
        self.stop_requested = threading.Event()
        self.closing = threading.Event()
        self.generation = 0
        self.bus = None
        self.test = None
        self.started = time.monotonic()
        self.last_ui = self.started
        self.last_reply = 0.
        self.last_poll = 0.
        self.arm_until = 0.
        # ponytail: rolling session buffer; stream to disk if longer captures are needed.
        self.samples = deque(maxlen=10000)
        self.events = deque(maxlen=500)
        self.data = dict(state='Disconnected', connected=False, demo=False, id=0, port=PORT,
                         baud=6000000, armed=False, latest=None, reason='Select an adapter and connect.',
                         tx=0, rx=0, crc_errors=0, reply_timeouts=0, rtt_ms=0., target_rpm=0.,
                         last_tx='', last_rx='', discovered=[], test_remaining=0., total_samples=0,
                         stopped=False, connection=0, last_test=None)
        self.thread = threading.Thread(target=self._loop, daemon=True, name='IM6014 serial owner')
        self.thread.start()

    def ping(self):
        self.last_ui = time.monotonic()

    def snapshot(self):
        with self.lock:
            return dict(copy.deepcopy(self.data), samples=list(self.samples), events=list(self.events),
                        age_s=time.monotonic() - self.last_reply if self.last_reply else None)

    def request(self, action, **values):
        if action == 'stop':
            with self.lock:
                self.generation += 1
            self.stop_requested.set()
            return
        self.jobs.put((self.generation, action, values))

    def close(self):
        self.request('stop')
        self.closing.set()
        self.thread.join(2)
        return not self.thread.is_alive()

    def _event(self, text):
        with self.lock:
            self.events.append(f'{time.monotonic() - self.started:8.2f}s  {text}')

    def _set(self, **values):
        with self.lock:
            self.data.update(values)

    def _transact(self, mode=0, rpm=0, kd=0, torque=0, watchdog=True, motor_id=None, position=0, kp=0):
        if mode and (self.stop_requested.is_set() or self.closing.is_set()):
            raise InterruptedError('Stop requested')
        motor_id = self.data['id'] if motor_id is None else motor_id
        packet = command(motor_id, mode, rpm, kd, torque, watchdog, position, kp)
        before = time.monotonic()
        self._set(tx=self.data['tx'] + 1, last_tx=packet.hex())
        try:
            f = exchange(self.bus, packet, motor_id)
        except Exception as error:
            if 'CRC' in str(error):
                self._set(crc_errors=self.data['crc_errors'] + 1)
            if 'deadline' in str(error):
                self._set(reply_timeouts=self.data['reply_timeouts'] + 1)
            raise
        now = time.monotonic()
        f.update(time_s=now - self.started, target_rpm=rpm, rtt_ms=(now - before) * 1000,
                 target_position_rad=position if kp else None,
                 demo=self.data['demo'], connection=self.data['connection'],
                 port=self.data['port'], baud=self.data['baud'])
        with self.lock:
            self.data.update(rx=self.data['rx'] + 1, last_rx=f['raw'], rtt_ms=f['rtt_ms'])
            if motor_id == self.data['id']:
                self.last_reply = now
                self.data.update(latest=f, target_rpm=rpm)
                self.samples.append(f)
                self.data['total_samples'] += 1
        return f

    def _stop(self):
        self.test = None
        self._set(armed=False, target_rpm=0., test_remaining=0., stopped=False)
        if not self.bus:
            return False
        try:
            f = self._transact()
            if f['mode'] != 0:
                f = self._transact()
            if f['mode'] != 0:
                raise RuntimeError('Motor has not confirmed Stop')
            self._set(stopped=True)
            return True
        except Exception as error:
            self._event('STOP NOT CONFIRMED: ' + str(error))
            return False

    def _fault(self, error):
        message = str(error)
        self._stop()
        if self.bus and not self.bus.is_open:
            self.bus.close()
            self.bus = None
            self._set(connected=False)
        self._set(state='Fault — disarmed', reason=message)
        self._event('DISARMED: ' + message)

    def _disconnect(self):
        if self.bus:
            self._stop()
            self.bus.close()
            self.bus = None
        self._set(connected=False, armed=False, state='Disconnected', latest=None,
                  reason='Disconnected. Reconnect requires a new check and arm action.', last_tx='', last_rx='')
        self.last_reply = 0.

    def _check(self, acknowledge=False):
        if not self.bus:
            raise ValueError('Connect first')
        self.test = None
        self._set(armed=False)
        if not self._stop():
            raise RuntimeError('Cannot verify Stop; check adapter, ID, baud and motor power')
        f = self._transact()
        healthy(f, 0, allow_timeout=True)
        if f['timeout']:
            if not acknowledge:
                self._set(state='Timeout — disarmed', reason='Click Acknowledge timeout while the motor is supported.')
                return
            self._transact(watchdog=False)
            f = self._transact()
            healthy(f, 0)
            self._event('Startup timeout acknowledged in Stop; firmware watchdog restored.')
        self._set(state='Ready · Stop', reason='Healthy feedback. Motor stopped. Arm before testing.', stopped=True)

    def _handle(self, action, v):
        if action == 'connect':
            if self.bus:
                raise ValueError('Disconnect before changing adapter or motor ID')
            motor_id, baud = v.get('id'), v.get('baud')
            if type(motor_id) is not int or not 0 <= motor_id <= 14 or baud not in (4000000, 6000000):
                raise ValueError('Individual motor ID 0–14; baud 4 or 6 Mbps')
            self._set(state='Connecting', reason='Opening adapter and sending Stop only.',
                      id=motor_id, baud=baud, port=v['port'], demo=bool(v.get('demo')), discovered=[],
                      connection=self.data['connection'] + 1, latest=None, armed=False, stopped=False,
                      tx=0, rx=0, crc_errors=0, reply_timeouts=0, last_tx='', last_rx='', last_test=None)
            self.last_reply = 0.
            if v.get('demo'):
                self.bus = DemoSerial()
            else:
                self.bus = serial.Serial(port=None, baudrate=baud, timeout=0.005,
                                         write_timeout=0.05, exclusive=True)
                self.bus.port = v['port']
                self.bus.dtr = self.bus.rts = False
                self.bus.open()
            self._set(connected=True)
            self._event(f"{'DEMO' if v.get('demo') else 'REAL'} adapter open: {v['port']} · ID {motor_id} · {baud}")
            self._check()
        elif action == 'disconnect':
            self._disconnect()
            self._event('Disconnected.')
        elif action in ('check', 'ack'):
            self._check(acknowledge=action == 'ack')
        elif action == 'arm':
            if self.data['state'] != 'Ready · Stop' or v.get('supported') is not True:
                raise ValueError('Healthy Stop and secured/unloaded setup acknowledgment required')
            if not self.last_reply or time.monotonic() - self.last_reply > .25:
                raise ValueError('Feedback stale; run Check connection again')
            if time.monotonic() - self.last_ui > .5:
                raise ValueError('Dashboard heartbeat stale; cannot arm')
            healthy(self.data['latest'], 0)
            self.arm_until = time.monotonic() + 10
            self._set(armed=True, state='Armed · motor stopped', reason='Ready for one bounded test. Arm expires in 10 seconds.')
            self._event('Armed for one test; motor remains stopped.')
        elif action == 'test':
            t = validate_test(v)
            if self.test or self.data['state'] == 'Testing':
                raise ValueError('A test is already running; Stop before changing it')
            if not self.data['armed'] or time.monotonic() > self.arm_until:
                raise ValueError('Arm before each test')
            if time.monotonic() - self.last_reply > .1:
                raise ValueError('Feedback stale; cannot start test')
            if time.monotonic() - self.last_ui > .5:
                raise ValueError('Dashboard heartbeat stale; cannot start test')
            healthy(self.data['latest'], 0)
            initial_position = self.data['latest']['position_rad']
            if t['kind'] == 'Position':
                target = math.radians(t['position_deg']) + (initial_position if t['position_reference'] == 'Relative' else 0)
                command(self.data['id'], 1, position=target, kp=t['kp'], kd=t['kd'])
                delta = abs(target - initial_position)
                if t['travel_limit'] and delta > math.radians(t['travel_limit']):
                    raise ValueError('Position target exceeds the configured travel limit')
                # Smoothstep trajectory has peak rate 1.5 * distance / time.
                move_s = max(.75, 1.5 * delta / (t['rpm'] * math.tau / 60))
                if move_s > t['duration']:
                    raise ValueError(f'Position move needs at least {move_s:.2f}s at this rate; increase duration or move rate')
                t.update(target_position=target, move_s=move_s)
            first = self._transact(1)
            healthy(first, first['mode'])
            f = self._transact(1)
            healthy(f, 1)
            self.test = dict(t, started=time.monotonic(), initial_position=f['position_rad'])
            self._set(state='Testing', reason=f"{t['kind']} test · {t['duration']:g}s maximum", stopped=False, last_test=t)
            self._event(f"TEST START: {t}")
        elif action == 'scan':
            if not self.bus or self.data['armed'] or self.test or v.get('supported') is not True:
                raise ValueError('Scan requires a connected adapter, disarmed state and supported motors')
            found = []
            self._set(state='Scanning IDs', reason='Sending Stop/status to IDs 0–14; never broadcasting.')
            for motor_id in range(15):
                if self.stop_requested.is_set() or self.closing.is_set():
                    raise InterruptedError('Scan cancelled')
                try:
                    f = self._transact(motor_id=motor_id)
                    found.append(dict(id=motor_id, voltage=f['voltage'], faults=fault_text(f['errors'], f['warnings'])))
                except RuntimeError as error:
                    if 'deadline' not in str(error):
                        raise
            self._set(discovered=found)
            self._event('Found motor IDs: ' + (', '.join(str(x['id']) for x in found) or 'none'))
            self._check()
        else:
            raise ValueError('Unknown action')

    def _tick(self):
        now = time.monotonic()
        if self.test:
            if now - self.last_ui > .5 or now - self.last_reply > .1:
                raise RuntimeError('Dashboard/feedback heartbeat lost. Re-check and arm again.')
            t = self.test
            elapsed = now - t['started']
            if elapsed >= t['duration']:
                kind = t['kind']
                if not self._stop():
                    raise RuntimeError('Test ended, but Stop not confirmed; isolate motor power')
                self._check()
                self._event(f'{kind} test complete. Stop confirmed; disarmed.')
                return
            ramp = min(1., elapsed / .75, (t['duration'] - elapsed) / .75) if t['kind'] == 'Speed' else 1.
            rpm = t['rpm'] * ramp if t['kind'] == 'Speed' else 0.
            torque = t['ff'] * ramp * (1 if t['rpm'] > 0 else -1) if t['kind'] == 'Speed' and t['rpm'] else 0.
            if t['kind'] == 'Torque pulse':
                torque = t['torque']
            kd = t['kd'] if t['kind'] != 'Torque pulse' else 0.
            position, kp = 0., 0.
            if t['kind'] == 'Position':
                u = min(1., elapsed / t['move_s'])
                position = t['initial_position'] + (t['target_position'] - t['initial_position']) * u ** 2 * (3 - 2 * u)
                kp = t['kp']
            f = self._transact(1, rpm, kd, torque, position=position, kp=kp)
            healthy(f, 1, speed_limit=t['speed_limit'])
            travel = abs(f['position_rad'] - t['initial_position'])
            limit = 15 if t['kind'] == 'Torque pulse' else t['travel_limit']
            if limit and travel > math.radians(limit):
                raise RuntimeError('Test exceeded travel limit')
            self._set(test_remaining=max(0., t['duration'] - elapsed))
        elif self.bus and self.bus.is_open and now - self.last_poll >= .05:
            self.last_poll = now
            f = self._transact()
            if not self.data['state'].startswith(('Fault', 'Timeout')):
                healthy(f, 0)
            if self.data['armed'] and (now > self.arm_until or now - self.last_ui > .5):
                self._set(armed=False, state='Ready · Stop', reason='Arm expired. Motor remains stopped.')
                self._event('Arm expired / dashboard heartbeat lost; disarmed.')

    def _loop(self):
        try:
            while not self.closing.is_set():
                if self.stop_requested.is_set():
                    confirmed = self._stop()
                    self.stop_requested.clear()
                    self._event('Operator Stop requested; ' + ('confirmed.' if confirmed else 'not confirmed.'))
                    if self.bus:
                        self._set(state='Stopped · re-check required', reason='Click Check connection before re-arming.')
                try:
                    generation, action, values = self.jobs.get_nowait()
                except queue.Empty:
                    action = None
                try:
                    if action and generation == self.generation:
                        self._handle(action, values)
                    self._tick()
                except InterruptedError:
                    self._stop()
                except ValueError as error:
                    self._event('Request rejected: ' + str(error))
                    self._set(reason=str(error))
                except Exception as error:
                    if not self.data['state'].startswith('Fault'):
                        self._fault(error)
                    time.sleep(.1)
                time.sleep(.01)
        finally:
            self._disconnect()
