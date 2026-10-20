"""Runnable offline lifecycle and Qt UI checks; never opens a physical serial port.

Run: QT_QPA_PLATFORM=offscreen python -m tests.bench.test_dashboard
"""
import csv
import json
import math
import os
from pathlib import Path
import tempfile
import struct
import threading
import time
from unittest.mock import patch

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
from PySide6.QtWidgets import QApplication, QFileDialog
from PySide6.QtGui import QColor, QPalette
from im6014_bench.dashboard import Dashboard, ROOT
from im6014_bench.session import DemoSerial, MotorSession, fault_text, validate_test

OUTPUT = ROOT / 'results/bench'
OUTPUT.mkdir(parents=True, exist_ok=True)

checks = []


def passed(name):
    checks.append(name)
    print('PASS:', name, flush=True)


def wait(session, predicate, timeout=4, heartbeat=True):
    until = time.monotonic() + timeout
    while time.monotonic() < until:
        if heartbeat:
            session.ping()
        snapshot = session.snapshot()
        if predicate(snapshot):
            return snapshot
        time.sleep(.01)
    snapshot = session.snapshot()
    raise AssertionError({k: snapshot[k] for k in ('state', 'reason', 'events')})


def ready(session):
    replies = session.snapshot()['rx']
    session.request('check')
    return wait(session, lambda x: x['state'] == 'Ready · Stop' and x['rx'] >= replies + 2)


def arm(session):
    session.request('arm', supported=True)
    wait(session, lambda x: x['armed'])


def start(session, kind='Speed', **updates):
    values = dict(kind=kind, rpm=5, kd=6, ff=.16, torque=.1, duration=2)
    values.update(updates)
    session.request('test', **values)
    return wait(session, lambda x: x['state'] == 'Testing')


def fails(call):
    try:
        call()
    except ValueError:
        return
    raise AssertionError('Invalid request accepted')


class TrackedDemo(DemoSerial):
    def __init__(self):
        super().__init__()
        self.writes = []

    def write(self, packet):
        self.writes.append((packet, threading.current_thread().name))
        return super().write(packet)


assert 'Overcurrent' in fault_text(3, 0) and 'overvoltage' in fault_text(3, 0)
assert 'Unknown error' in fault_text(1 << 31, 0) and 'Unknown warning' in fault_text(0, 4)
base = dict(kind='Speed', rpm=5, kd=6, ff=.16, torque=.1, duration=5)
for change in [dict(rpm=float('nan')), dict(rpm=3000), dict(kd=7), dict(ff=.6), dict(torque=1.1), dict(duration=11), dict(kind='Raw'),
               dict(position_deg=float('inf')), dict(position_deg=1e9), dict(position_deg=1e308), dict(kp=31), dict(speed_limit=-1),
               dict(travel_limit=float('nan')), dict(position_reference='Unknown'), dict(speed_limit=4)]:
    fails(lambda: validate_test(dict(base, **change)))
fails(lambda: validate_test(dict(base, kind='Torque pulse', duration=1)))
assert validate_test(dict(base, rpm=60))['rpm'] == 60
for change in [dict(rpm=0), dict(rpm=-1), dict(kp=0), dict(kp=.001), dict(duration=61)]:
    fails(lambda: validate_test(dict(base, kind='Position', **change)))
passed('test input boundaries, nonfinite rejection, simultaneous/unknown fault decoding')

with patch('im6014_bench.session.DemoSerial', TrackedDemo), patch('im6014_bench.session.serial.Serial') as real_serial:
    session = MotorSession()
    try:
        assert session.snapshot()['state'] == 'Disconnected' and not real_serial.called
        session.request('connect', demo=True, port='demo', id=0, baud=6000000)
        wait(session, lambda x: x['state'] == 'Timeout — disarmed')
        bus = session.bus
        assert not real_serial.called and all((p[2] >> 4) & 7 == 0 for p, _ in bus.writes)
        session.request('arm', supported=True)
        wait(session, lambda x: 'rejected' in x['events'][-1].lower())
        assert not session.snapshot()['armed']
        session.request('ack')
        wait(session, lambda x: x['state'] == 'Ready · Stop')
        assert not session.snapshot()['latest']['timeout']
        assert any(not (p[2] & 128) for p, _ in bus.writes)
        passed('demo never opens hardware; connect remains stopped; timeout blocks arm and explicit acknowledgment restores watchdog')

        before = len(bus.writes)
        session.request('test', **base)
        wait(session, lambda x: 'Arm before' in x['reason'])
        assert all((p[2] >> 4) & 7 == 0 for p, _ in bus.writes[before:])
        arm(session)
        start(session)
        test_started = session.test['started']
        session.request('test', **base)
        wait(session, lambda x: 'already running' in x['reason'])
        assert session.test['started'] == test_started
        wait(session, lambda x: x['state'] == 'Ready · Stop', timeout=3)
        assert not session.snapshot()['armed'] and session.snapshot()['stopped']
        assert max(x['rpm'] for x in session.snapshot()['samples']) > 3
        assert all(p[2] & 128 for p, _ in bus.writes if (p[2] >> 4) & 7 == 1)
        assert {owner for _, owner in bus.writes} == {'IM6014 serial owner'}
        passed('one serial owner, arm per test, active-test replacement rejected, delayed FOC acknowledgment, ramps, watchdog and final Stop')

        arm(session)
        start(session, rpm=-5)
        wait(session, lambda x: x['latest']['rpm'] < -1)
        session.request('stop')
        wait(session, lambda x: x['state'] == 'Stopped · re-check required')
        assert not session.snapshot()['armed'] and bus.states[0]['mode'] == 0
        ready(session)
        arm(session)
        start(session, 'Torque pulse', duration=.25, torque=-.1)
        wait(session, lambda x: x['state'] == 'Ready · Stop')
        arm(session)
        start(session, 'Damping', kd=1)
        session.request('stop')
        wait(session, lambda x: x['state'] == 'Stopped · re-check required')
        passed('reverse speed, bounded negative torque pulse, damping and operator Stop')

        ready(session)
        arm(session)
        sample_start = session.snapshot()['total_samples']
        start(session, rpm=60, kd=.5)
        result = wait(session, lambda x: x['state'] == 'Ready · Stop', timeout=3)
        samples = result['samples'][sample_start:]
        assert max(x['rpm'] for x in samples) > 50
        assert abs(samples[-1]['position_rad'] - samples[0]['position_rad']) > math.tau
        assert result['stopped'] and not result['armed']
        passed('60 rpm completes with speed/travel trips Off; no hidden 5/8 rpm or one-revolution ceiling')

        for limit in [dict(speed_limit=8), dict(travel_limit=2)]:
            arm(session)
            start(session, rpm=5, kd=1, **limit)
            if 'speed_limit' in limit:
                bus.states[0]['rpm'] = 30
            wait(session, lambda x: x['state'].startswith('Fault'))
            assert not session.snapshot()['armed'] and bus.states[0]['mode'] == 0
            ready(session)
        passed('configured measured-speed and travel trips still stop and disarm')

        for reference, degrees in [('Relative', 10), ('Absolute', -5)]:
            # Reset this simulator's angle while stopped to test signed absolute motion.
            bus.states[0]['pos'] = 0
            ready(session)
            initial = session.snapshot()['latest']['position_rad']
            arm(session)
            before = len(bus.writes)
            start(session, 'Position', kd=1, kp=3, position_deg=degrees, position_reference=reference, duration=3)
            wait(session, lambda x: x['state'] == 'Ready · Stop', timeout=4)
            target = math.radians(degrees) + (initial if reference == 'Relative' else 0)
            assert abs(session.snapshot()['latest']['position_rad'] - target) < math.radians(.5)
            packets = [struct.unpack('<hhihh', p[4:16]) for p, _ in bus.writes[before:] if (p[2] >> 4) & 7 == 1]
            assert packets[0] == packets[1] == (0, 0, 0, 0, 0)
            targets = [p[2] / 32768 / (38 / 3) * math.tau for p in packets if p[3]]
            assert abs(targets[0] - initial) < math.radians(.1)
            assert abs(targets[-1] - target) < math.radians(.01)
            assert all(p[0] == p[1] == 0 for p in packets)
            assert session.snapshot()['stopped'] and not session.snapshot()['armed']
        passed('relative and signed absolute position: current-angle start, smooth targets, gains, hold, timed Stop')

        for updates in [dict(position_deg=30, travel_limit=10), dict(position_deg=180, rpm=1, duration=2)]:
            arm(session)
            before = len(bus.writes)
            session.request('test', **dict(base, kind='Position', **updates))
            wait(session, lambda x: 'Position' in x['reason'] and 'test' not in x['reason'])
            assert all((p[2] >> 4) & 7 == 0 for p, _ in bus.writes[before:])
            session.request('stop')
            wait(session, lambda x: x['state'] == 'Stopped · re-check required')
            ready(session)
        passed('position target/travel and impossible move duration rejected before enabling FOC')

        ready(session)
        arm(session)
        session.arm_until = time.monotonic() - 1
        wait(session, lambda x: not x['armed'])
        assert session.snapshot()['state'] == 'Ready · Stop'
        with session.lock:
            session.request('arm', supported=True)
            session.request('test', **base)
            before = len(bus.writes)
            session.request('stop')
        wait(session, lambda x: x['state'] == 'Stopped · re-check required')
        time.sleep(.1)
        assert all((p[2] >> 4) & 7 == 0 for p, _ in bus.writes[before:])
        passed('arm expiry and priority Stop invalidate queued arm/motion requests')

        for injection in ('fault', 'crc', 'timeout'):
            ready(session)
            arm(session)
            start(session)
            bus.inject = injection
            wait(session, lambda x: x['state'].startswith('Fault'))
            assert not session.snapshot()['armed'] and session.test is None
            bus.inject = None
            wait(session, lambda x: x['latest']['mode'] == 0)
            assert session.snapshot()['state'].startswith('Fault')
            before = len(bus.writes)
            session.request('test', **base)
            time.sleep(.15)
            assert all((p[2] >> 4) & 7 == 0 for p, _ in bus.writes[before:])
        passed('motor fault, corrupt CRC and lost replies latch disarmed; restored communications never resumes motion')

        ready(session)
        arm(session)
        start(session)
        session.last_ui = time.monotonic() - 1
        wait(session, lambda x: x['state'].startswith('Fault'), heartbeat=False)
        assert bus.states[0]['mode'] == 0
        ready(session)
        arm(session)
        start(session)
        session.last_reply = time.monotonic() - 1
        wait(session, lambda x: x['state'].startswith('Fault'))
        assert not session.snapshot()['armed']
        passed('stalled UI heartbeat and stale motor feedback abort motion and confirm Stop')

        ready(session)
        session.request('scan', supported=True)
        result = wait(session, lambda x: len(x['discovered']) == 2 and x['state'] == 'Ready · Stop')
        assert [x['id'] for x in result['discovered']] == [0, 1]
        assert all(v['mode'] == 0 for v in bus.states.values())
        session.request('disconnect')
        wait(session, lambda x: not x['connected'])
        session.request('connect', demo=True, port='demo', id=1, baud=6000000)
        wait(session, lambda x: x['state'] == 'Timeout — disarmed')
        assert not session.snapshot()['armed']
        assert {x['connection'] for x in session.snapshot()['samples']} == {1, 2}
        assert all(x['demo'] for x in session.snapshot()['samples'])
        passed('individual-ID Stop scan, disconnect/reconnect disarming and unambiguous per-connection capture tags')
    finally:
        assert session.close()

with patch('im6014_bench.session.serial.Serial') as serial_constructor:
    serial_constructor.return_value.is_open = False
    serial_constructor.return_value.open.side_effect = OSError('permission denied')
    session = MotorSession()
    try:
        session.request('connect', demo=False, port='/not-real', id=0, baud=6000000)
        wait(session, lambda x: x['state'].startswith('Fault'))
        assert not session.snapshot()['connected'] and session.bus is None
        session.request('connect', demo=True, port='demo', id=0, baud=6000000)
        wait(session, lambda x: x['connected'])
        passed('failed adapter open releases ownership and permits a fresh connection')
    finally:
        session.close()

app = QApplication([])
app.setStyle('Fusion')
light = QPalette(app.palette())
dark = QPalette(light)
for role, color in [(QPalette.ColorRole.Window, '#202326'), (QPalette.ColorRole.Base, '#17191b'),
                    (QPalette.ColorRole.Button, '#353a40'), (QPalette.ColorRole.WindowText, '#edf2f7'),
                    (QPalette.ColorRole.Text, '#edf2f7'), (QPalette.ColorRole.ButtonText, '#edf2f7')]:
    dark.setColor(role, QColor(color))
app.setPalette(dark)
window = Dashboard(True)
window.show()


def ui_wait(predicate, timeout=4):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        app.processEvents()
        if predicate():
            return
        time.sleep(.01)
    snapshot = window.session.snapshot()
    raise AssertionError({k: snapshot[k] for k in ('state', 'reason', 'events')})


try:
    assert not window.arm_button.isEnabled() and not window.run_button.isEnabled()
    window.connect_button.click()
    ui_wait(lambda: window.ack_button.isEnabled())
    window.ack_button.click()
    ui_wait(lambda: window.session.snapshot()['state'] == 'Ready · Stop')
    assert not window.arm_button.isEnabled()
    window.secured.setChecked(True)
    ui_wait(window.arm_button.isEnabled)
    window.arm_button.click()
    ui_wait(window.run_button.isEnabled)
    window.duration.setValue(2)
    window.run_button.click()
    ui_wait(lambda: window.session.snapshot()['state'] == 'Testing')
    ui_wait(lambda: window.session.snapshot()['latest']['rpm'] > 3)
    window.grab().save(str(OUTPUT / 'dashboard_preview.png'))
    window.stop_button.click()
    ui_wait(lambda: window.session.snapshot()['state'] == 'Stopped · re-check required')
    assert not window.run_button.isEnabled()
    window.kind.setCurrentText('Torque pulse')
    assert window.duration.maximum() == .5 and not window.rpm.isVisible()
    window.kind.setCurrentText('Damping')
    assert window.kd.value() == 1 and window.duration.maximum() == 10
    window.console.setText('speed nan 2')
    window.console_command()
    ui_wait(lambda: 'finite' in window.session.snapshot()['events'][-1])
    passed('native UI connect/ack/checkbox/arm/run/Stop flow, live chart rendering, mode-specific controls and console validation')

    assert window.rpm.maximum() > 2000 and window.rpm.minimum() < -2000
    window.rpm.setValue(60)
    assert window.test_values()['rpm'] == 60
    window.rpm.setValue(5)
    window.kind.setCurrentText('Position')
    assert window.position_deg.isVisible() and window.kp.isVisible() and not window.ff.isVisible()
    assert window.duration.maximum() == 60 and window.kd.value() == 1
    assert window.plots[0].signal_label == 'Position (degrees)'
    ui_wait(window.use_current.isEnabled)
    window.use_current.click()
    assert window.position_reference.currentText() == 'Absolute'
    assert abs(window.position_deg.value() - math.degrees(window.session.snapshot()['latest']['position_rad'])) < .002
    window.position_reference.setCurrentText('Relative')
    window.position_deg.setValue(10)
    window.duration.setValue(3)
    window.check_button.click()
    ui_wait(lambda: window.session.snapshot()['state'] == 'Ready · Stop')
    ui_wait(window.arm_button.isEnabled)
    window.arm_button.click()
    ui_wait(window.run_button.isEnabled)
    window.run_button.click()
    ui_wait(lambda: window.session.snapshot()['latest'].get('target_position_rad') is not None)
    ui_wait(lambda: window.session.snapshot()['test_remaining'] < 2)
    window.resize(1280, 1024)
    app.processEvents()
    for widget in (window.banner, window.source_badge):
        assert widget.palette().color(QPalette.ColorRole.WindowText).name() == '#273747'
    assert window.fields['rpm'].foreground(1).color().name() == '#edf2f7'
    window.grab().save(str(OUTPUT / 'dashboard_dark_preview.png'))
    ui_wait(lambda: window.session.snapshot()['state'] == 'Ready · Stop', timeout=4)
    window.session._set(state='Fault — disarmed', reason='Offline contrast check')
    window.refresh()
    assert window.banner.palette().color(QPalette.ColorRole.WindowText).name() == '#273747'
    with patch.object(window.session, 'request') as request:
        for text, reference in [('position 10 3', 'Relative'), ('goto -5 3', 'Absolute')]:
            window.console.setText(text)
            window.console_command()
            assert request.call_args.kwargs['kind'] == 'Position'
            assert request.call_args.kwargs['position_reference'] == reference
    window.session.request('disconnect')
    ui_wait(lambda: not window.session.snapshot()['connected'])
    window.source.setCurrentIndex(0)
    window.refresh()
    assert window.source_badge.text().startswith('REAL HARDWARE')
    assert window.source_badge.palette().color(QPalette.ColorRole.WindowText).name() == '#273747'
    window.source.setCurrentIndex(1)
    window.connect_button.click()
    ui_wait(lambda: window.ack_button.isEnabled())
    window.ack_button.click()
    ui_wait(lambda: window.session.snapshot()['state'] == 'Ready · Stop')
    passed('higher-speed input, position controls/current-angle copy/target plot/console and dark-theme demo/real/fault contrast')

    with tempfile.TemporaryDirectory() as directory:
        for suffix in ('json', 'csv'):
            path = Path(directory) / ('session.' + suffix)
            with patch.object(QFileDialog, 'getSaveFileName', return_value=(str(path), '')):
                window.export_session()
            if suffix == 'json':
                assert json.loads(path.read_text())['samples'][0]['demo'] is True
            else:
                with path.open() as file:
                    records = list(csv.DictReader(file))
                assert records and records[0]['demo'] == 'True' and 'raw' in records[0]
                assert 'target_position_rad' in records[0]
                assert any(x['target_position_rad'] for x in records)
    captured = []
    with patch.object(window.session, 'request', side_effect=lambda action, **v: captured.append((action, v))):
        window.port.setEditText('/custom/adapter')
        window.connect_motor()
    assert captured[-1][1]['port'] == '/custom/adapter'
    window.resize(1024, 768)
    app.processEvents()
    assert window.width() == 1024 and window.height() == 768
    window.grab().save('/tmp/dashboard_small.png')
    passed('CSV/JSON round trips, typed adapter path honored, and responsive 1024×768 layout')
    window.session.request('check')
    ui_wait(lambda: window.session.snapshot()['state'] == 'Ready · Stop')
    window.secured.setChecked(True)
    window.session.request('arm', supported=True)
    ui_wait(lambda: window.session.snapshot()['armed'])
    window.session.request('test', **base)
    ui_wait(lambda: window.session.snapshot()['state'] == 'Testing')
    bus_at_close = window.session.bus
finally:
    window.close()
    app.processEvents()
assert not window.session.thread.is_alive()
assert bus_at_close.states[0]['mode'] == 0 and not bus_at_close.is_open
passed('closing the native window during motion confirms Stop, releases adapter and joins its worker')
app.setPalette(light)
light_window = Dashboard()
try:
    light_window.show()
    app.processEvents()
    for widget in (light_window.banner, light_window.source_badge):
        assert widget.palette().color(QPalette.ColorRole.WindowText).name() == '#273747'
    assert light_window.source_badge.text().startswith('REAL HARDWARE')
finally:
    light_window.close()
passed('light-theme real-hardware badge and status banner contrast; starts disconnected')
(OUTPUT / 'dashboard_test_results.json').write_text(json.dumps(dict(motor_io_performed=False, checks=checks), indent=2) + '\n')
print(f'{len(checks)} grouped checks passed. No real motor I/O.', flush=True)
