"""Local IM6014 desktop bench dashboard. Starts disconnected; --demo never opens USB."""
import argparse
import csv
import json
import math
from pathlib import Path
import shlex
import sys
import time

from PySide6.QtCore import QPointF, Qt, QTimer
from PySide6.QtGui import QColor, QKeySequence, QPainter, QPainterPath, QPalette, QPen, QShortcut
from PySide6.QtWidgets import (QApplication, QCheckBox, QComboBox, QDoubleSpinBox, QFileDialog,
                              QFormLayout, QGroupBox, QHBoxLayout, QLabel, QLineEdit, QMainWindow,
                              QMessageBox, QPlainTextEdit, QProgressBar, QPushButton, QSplitter,
                              QScrollArea, QSpinBox, QTabWidget, QTextBrowser, QTreeWidget, QTreeWidgetItem,
                              QVBoxLayout, QWidget)
from serial.tools import list_ports

from .session import MotorSession, fault_text
from .motor import PORT, POSITION_MIN, POSITION_MAX, RPM_MIN, RPM_MAX

ROOT = Path(__file__).resolve().parents[1]
METRICS = {'Speed (rpm)': 'rpm', 'Torque (Nm)': 'torque_nm', 'Position (degrees)': 'position_rad',
           'Bus voltage (V)': 'voltage', 'Driver temperature (°C)': 'driver_c',
           'Winding temperature (°C)': 'winding_c', 'Reply time (ms)': 'rtt_ms'}


class Plot(QWidget):
    def __init__(self, label):
        super().__init__()
        self.signal_label = label
        self.samples = []
        self.setMinimumSize(300, 120)
        self.setAccessibleName(label + ' live plot')

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.fillRect(self.rect(), QColor('#ffffff'))
        p.setPen(QColor('#273747'))
        p.drawText(12, 22, self.signal_label + ' · last 15 seconds')
        box = self.rect().adjusted(65, 38, -18, -30)
        values = [(x['time_s'], math.degrees(x['position_rad']) if self.signal_label == 'Position (degrees)' else x[METRICS[self.signal_label]]) for x in self.samples]
        if not values:
            p.drawText(box, Qt.AlignmentFlag.AlignCenter, 'Connect to see live feedback')
            p.end()
            return
        end = values[-1][0]
        values = [(t, y) for t, y in values if t >= end - 15]
        targets = [(x['time_s'], x['target_rpm']) for x in self.samples if x['time_s'] >= end - 15] if self.signal_label == 'Speed (rpm)' else []
        if self.signal_label == 'Position (degrees)':
            targets = [(x['time_s'], math.degrees(x['target_position_rad'])) for x in self.samples
                       if x['time_s'] >= end - 15 and x.get('target_position_rad') is not None]
        ys = [y for _, y in values + targets]
        lo, hi = min(0, min(ys)), max(0, max(ys))
        pad = max((hi - lo) * .15, .1)
        lo -= pad
        hi += pad
        for fraction in (0., .25, .5, .75, 1.):
            y = box.bottom() - box.height() * fraction
            p.setPen(QColor('#e4e9ee'))
            p.drawLine(QPointF(box.left(), y), QPointF(box.right(), y))
            p.setPen(QColor('#617284'))
            p.drawText(3, int(y + 4), f'{lo + (hi - lo) * fraction:.2f}')
        p.drawText(box.left(), self.height() - 9, '−15 s')
        p.drawText(box.right() - 27, self.height() - 9, 'now')
        for trace, color, style in [(values, '#2463a0', Qt.PenStyle.SolidLine), (targets, '#ad7620', Qt.PenStyle.DashLine)]:
            if not trace:
                continue
            path = QPainterPath()
            for index, (t, y) in enumerate(trace):
                point = QPointF(box.left() + (t - (end - 15)) / 15 * box.width(),
                                box.bottom() - (y - lo) / (hi - lo) * box.height())
                path.moveTo(point) if not index else path.lineTo(point)
            p.setPen(QPen(QColor(color), 2, style))
            p.drawPath(path)
        p.setPen(QColor('#2463a0'))
        p.drawText(box.left(), 22, '')
        p.drawText(box.right() - 170, 22, 'Measured' + ('   - - Target' if targets else ''))
        p.end()


class Dashboard(QMainWindow):
    def __init__(self, demo=False):
        super().__init__()
        self.session = MotorSession()
        self.setWindowTitle('IM6014 Motor Bench')
        self.resize(1200, 970)
        self.last_events = None
        self.setStyleSheet('QGroupBox {font-weight:600; margin-top:12px; padding-top:12px;} '
                           'QGroupBox::title {subcontrol-origin:margin; left:10px;} '
                           'QPushButton {padding:6px 10px;} QTabWidget::pane {border:1px solid #cbd3dc;}')
        central = QWidget()
        layout = QVBoxLayout(central)
        self.setCentralWidget(central)
        header = QHBoxLayout()
        title = QLabel('<b>IM6014 Motor Bench</b>  ·  connect, inspect, test')
        title.setStyleSheet('font-size:18px;')
        self.source_badge = QLabel('REAL HARDWARE · disconnected')
        self.source_badge.setStyleSheet('padding:8px; background:#e9eef5; color:#273747; font-weight:600;')
        self.stop_button = QPushButton('STOP / DISARM    Esc')
        self.stop_button.setStyleSheet('background:#b72c2c;color:white;font-weight:700;padding:10px;')
        self.stop_button.clicked.connect(lambda: self.session.request('stop'))
        header.addWidget(title)
        header.addStretch()
        header.addWidget(self.source_badge)
        header.addWidget(self.stop_button)
        layout.addLayout(header)
        self.banner = QLabel('Select an adapter and connect. Connecting sends Stop/status only.')
        self.banner.setWordWrap(True)
        self.banner.setMinimumHeight(44)
        self.banner.setMaximumHeight(76)
        layout.addWidget(self.banner)
        self.vertical = QSplitter(Qt.Orientation.Vertical)
        top = QSplitter(Qt.Orientation.Horizontal)
        left = QTabWidget()
        left.setMinimumWidth(410)
        left.addTab(self._controls(demo), 'Connection & tests')
        left.addTab(self._telemetry(), 'Telemetry tree')
        top.addWidget(left)
        right = QTabWidget()
        right.addTab(self._plots(), 'Live plots')
        right.addTab(self._diagnostics(), 'Diagnose')
        manual = QTextBrowser()
        manual.setOpenExternalLinks(True)
        manual.setMarkdown((ROOT / 'docs/bench/user-manual.md').read_text() if (ROOT / 'docs/bench/user-manual.md').exists() else 'User manual is being prepared.')
        right.addTab(manual, 'User manual')
        top.addWidget(right)
        top.setSizes([415, 785])
        self.vertical.addWidget(top)
        self.vertical.addWidget(self._console())
        self.vertical.setSizes([760, 130])
        layout.addWidget(self.vertical)
        self.statusBar().showMessage('Output-shaft units · One selected motor · No automatic motion')
        self.shortcut = QShortcut(QKeySequence('Esc'), self)
        self.shortcut.activated.connect(lambda: self.session.request('stop'))
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(100)
        self.refresh()

    def _button(self, text, action, tip):
        button = QPushButton(text)
        button.setToolTip(tip)
        button.clicked.connect(action)
        return button

    def _controls(self, demo):
        page = QWidget()
        layout = QVBoxLayout(page)
        group = QGroupBox('1  Connect')
        form = QFormLayout(group)
        self.source = QComboBox()
        self.source.addItems(['Real motor', 'Demo simulator'])
        self.source.setCurrentIndex(int(demo))
        self.source.currentIndexChanged.connect(lambda: self.refresh_ports())
        form.addRow('Source', self.source)
        port_row = QWidget()
        port_layout = QHBoxLayout(port_row)
        port_layout.setContentsMargins(0, 0, 0, 0)
        self.port = QComboBox()
        self.port.setEditable(True)
        self.port.setMinimumContentsLength(12)
        self.port.setSizeAdjustPolicy(QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon)
        self.rescan = self._button('Refresh', self.refresh_ports, 'List USB/serial adapters; does not send motor packets.')
        port_layout.addWidget(self.port, 1)
        port_layout.addWidget(self.rescan)
        form.addRow('Adapter / channel', port_row)
        self.baud = QComboBox()
        self.baud.addItems(['6 Mbps', '4 Mbps'])
        form.addRow('Baud', self.baud)
        self.motor_id = QSpinBox()
        self.motor_id.setRange(0, 14)
        form.addRow('Motor ID', self.motor_id)
        buttons = QWidget()
        row = QHBoxLayout(buttons)
        row.setContentsMargins(0, 0, 0, 0)
        self.connect_button = self._button('Connect', self.connect_motor, 'Open this channel and verify the selected motor using Stop/status packets.')
        self.disconnect_button = self._button('Disconnect', lambda: self.session.request('disconnect'), 'Request Stop, then release the adapter.')
        row.addWidget(self.connect_button)
        row.addWidget(self.disconnect_button)
        form.addRow(buttons)
        layout.addWidget(group)
        group = QGroupBox('2  Check & prepare')
        prepare = QVBoxLayout(group)
        self.check_button = self._button('Check connection', lambda: self.session.request('check'), 'Send Stop and re-check fresh feedback. No fault clearing or motion.')
        self.ack_button = self._button('Acknowledge timeout', lambda: self.session.request('ack'), 'In Stop only: acknowledge the startup timeout, then immediately restore the watchdog.')
        check_row = QHBoxLayout()
        check_row.addWidget(self.check_button)
        check_row.addWidget(self.ack_button)
        prepare.addLayout(check_row)
        self.secured = QCheckBox('Motor secured / unloaded\nPower cut within reach')
        self.secured.setToolTip('Required before arming or scanning IDs. Stop can release a gravity-loaded joint.')
        self.secured.toggled.connect(self.support_changed)
        prepare.addWidget(self.secured)
        self.arm_button = self._button('Arm one test', lambda: self.session.request('arm', supported=self.secured.isChecked()), 'Motor remains stopped. Arm expires after 10 seconds and is consumed by one test.')
        prepare.addWidget(self.arm_button)
        layout.addWidget(group)
        group = QGroupBox('3  Bounded test')
        form = QFormLayout(group)
        self.test_form = form
        self.kind = QComboBox()
        self.kind.addItems(['Speed', 'Position', 'Torque pulse', 'Damping'])
        self.kind.currentTextChanged.connect(self.test_kind_changed)
        form.addRow('Test', self.kind)
        self.rpm = self._number(math.ceil(RPM_MIN * 100) / 100, math.floor(RPM_MAX * 100) / 100, 5, 2, .25)
        self.rpm.setToolTip('Output rpm. No 5 rpm cap. Only the packet format bounds this input; it is not the motor\'s rated speed. Position uses a positive peak target move rate.')
        self.position_reference = QComboBox()
        self.position_reference.addItems(['Relative', 'Absolute'])
        self.position_reference.currentTextChanged.connect(self.position_reference_changed)
        self.position_deg = self._number(math.ceil(math.degrees(POSITION_MIN) * 1000) / 1000,
                                         math.floor(math.degrees(POSITION_MAX) * 1000) / 1000, 10, 3, 1)
        self.position_deg.setToolTip('Relative: offset from fresh position at Run. Absolute: powered-session encoder angle. No automatic homing or shortest-path wrap.')
        self.use_current = self._button('Use current position', self.use_current_position, 'Copy fresh encoder angle to an absolute target. Does not command motion.')
        self.kp = self._number(0, 30, 3, 2, .25)
        self.kd = self._number(0, 6, 6, 2, .25)
        self.ff = self._number(0, .5, .16, 3, .01)
        self.torque = self._number(-1, 1, .1, 3, .05)
        self.duration = self._number(2, 10, 5, 2, .5)
        self.speed_limit = self._number(0, math.ceil(RPM_MAX), 0, 2, 1)
        self.travel_limit = self._number(0, math.degrees(POSITION_MAX - POSITION_MIN), 0, 2, 15)
        for field in (self.speed_limit, self.travel_limit):
            field.setSpecialValueText('Off')
        self.speed_limit.setToolTip('Optional trip on absolute measured rpm. Off removes the old 8 rpm trip. Other fault/torque/temperature/watchdog checks still apply.')
        self.travel_limit.setToolTip('Optional measured travel from test start, in either direction. Off disables this trip. Torque pulse retains its 15° trip.')
        form.addRow('Speed · rpm', self.rpm)
        form.addRow('Position reference', self.position_reference)
        form.addRow('Move by · degrees', self.position_deg)
        form.addRow(self.use_current)
        form.addRow('Position stiffness · Kp', self.kp)
        form.addRow('Speed stiffness · Kd', self.kd)
        form.addRow('Friction torque · Nm', self.ff)
        self.ff.setToolTip('Feed-forward friction compensation, output-side Nm. Sign follows the selected speed.')
        form.addRow('Pulse torque · Nm', self.torque)
        form.addRow('Duration · seconds', self.duration)
        form.addRow('Measured speed trip · rpm', self.speed_limit)
        form.addRow('Travel trip · degrees', self.travel_limit)
        self.run_button = self._button('Run test', self.run_test, 'Run one bounded test and automatically return to Stop/disarmed.')
        form.addRow(self.run_button)
        self.progress = QProgressBar()
        self.progress.setRange(0, 100)
        self.progress.setFormat('No test running')
        form.addRow(self.progress)
        hint = QLabel('Speed ramps gently. Position ramps, then holds until the test ends. Optional trips: Off = no speed/travel trip. Torque pulse: ±1 Nm, 0.5 s / 15° maximum.')
        hint.setWordWrap(True)
        form.addRow(hint)
        layout.addWidget(group)
        layout.addStretch()
        self.refresh_ports()
        self.test_kind_changed('Speed')
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QScrollArea.Shape.NoFrame)
        scroll.setWidget(page)
        return scroll

    def _number(self, low, high, value, decimals, step):
        widget = QDoubleSpinBox()
        widget.setRange(low, high)
        widget.setDecimals(decimals)
        widget.setSingleStep(step)
        widget.setValue(value)
        return widget

    def _telemetry(self):
        self.tree = QTreeWidget()
        self.tree.setHeaderLabels(['Signal · output shaft', 'Value'])
        self.fields = {}
        for group, fields in [('Motion', [('rpm', 'Speed · rpm'), ('position_rad', 'Position · degrees'), ('torque_nm', 'Torque · Nm')]),
                              ('Power & temperature', [('voltage', 'Bus · V'), ('driver_c', 'Driver · °C'), ('winding_c', 'Winding · °C')]),
                              ('Motor state', [('id', 'Motor ID'), ('mode', 'Motor mode'), ('timeout', 'Timeout latched'), ('errors', 'Error bits'), ('warnings', 'Warning bits')])]:
            parent = QTreeWidgetItem(self.tree, [group])
            for key, label in fields:
                self.fields[key] = QTreeWidgetItem(parent, [label, '—'])
        self.tree.expandAll()
        self.tree.setColumnWidth(0, 220)
        return self.tree

    def _plots(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        cards = QHBoxLayout()
        self.cards = {}
        for key, label in [('rpm', 'SPEED · rpm'), ('torque_nm', 'TORQUE · Nm'), ('voltage', 'BUS · V')]:
            group = QGroupBox(label)
            inner = QVBoxLayout(group)
            value = QLabel('—')
            value.setStyleSheet('font-size:28px;font-weight:600;')
            inner.addWidget(value)
            cards.addWidget(group)
            self.cards[key] = value
        layout.addLayout(cards)
        self.plots = []
        self.plot_selectors = []
        for initial in ['Speed (rpm)', 'Torque (Nm)']:
            selector = QComboBox()
            selector.addItems(METRICS)
            selector.setCurrentText(initial)
            plot = Plot(initial)
            selector.currentTextChanged.connect(lambda text, p=plot: self.change_plot(p, text))
            layout.addWidget(selector)
            layout.addWidget(plot, 1)
            self.plots.append(plot)
            self.plot_selectors.append(selector)
        self.link_status = QLabel('No valid feedback yet.')
        self.link_status.setWordWrap(True)
        layout.addWidget(self.link_status)
        return page

    def _diagnostics(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        self.diagnosis = QTextBrowser()
        layout.addWidget(self.diagnosis)
        self.scan_button = self._button('Discover IDs 0–14 · sends Stop', self.scan_ids, 'Explicit Stop/status scan on this channel. This may release every supported motor on the bus.')
        layout.addWidget(self.scan_button)
        self.export_button = self._button('Export session · JSON / CSV', self.export_session, 'Save retained feedback samples and diagnostics. JSON includes raw packets and the event log.')
        layout.addWidget(self.export_button)
        return page

    def _console(self):
        tabs = QTabWidget()
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(6, 6, 6, 6)
        self.event_log = QPlainTextEdit()
        self.event_log.setReadOnly(True)
        self.event_log.setMaximumBlockCount(500)
        self.event_log.setStyleSheet('font-family:monospace;')
        layout.addWidget(self.event_log)
        self.console = QLineEdit()
        self.console.setPlaceholderText('Optional console: help, status, stop, check, ack-timeout, arm, speed 5 5')
        self.console.returnPressed.connect(self.console_command)
        layout.addWidget(self.console)
        tabs.addTab(page, 'Event log & console')
        self.packets = QPlainTextEdit()
        self.packets.setReadOnly(True)
        self.packets.setStyleSheet('font-family:monospace;')
        tabs.addTab(self.packets, 'Latest packets')
        return tabs

    def refresh_ports(self):
        if not hasattr(self, 'port'):
            return
        current = self.port.currentData() or self.port.currentText()
        self.port.clear()
        if self.source.currentIndex() == 1:
            self.port.addItem('Demo bus · IDs 0 and 1', 'demo')
            return
        ports = sorted(list_ports.comports(), key=lambda p: (not (p.serial_number == 'FTC2RQ7H' and p.location and p.location.endswith('.3')), p.device))
        for p in ports:
            path = PORT if Path(PORT).exists() and Path(PORT).resolve() == Path(p.device).resolve() else p.device
            self.port.addItem(f'{p.device} · {p.description}', path)
            self.port.setItemData(self.port.count() - 1, f'{path}\nSerial: {p.serial_number or "unknown"}\nUSB location: {p.location or "unknown"}', Qt.ItemDataRole.ToolTipRole)
        if not ports:
            self.port.addItem(PORT, PORT)
        index = self.port.findData(current)
        if index >= 0:
            self.port.setCurrentIndex(index)

    def connect_motor(self):
        matched = self.port.currentIndex() >= 0 and self.port.currentText() == self.port.itemText(self.port.currentIndex())
        port = self.port.currentData() if matched else self.port.currentText().strip()
        if not port:
            return
        self.secured.setChecked(False)
        self.session.request('connect', port=port, id=self.motor_id.value(),
                             baud=6000000 if self.baud.currentIndex() == 0 else 4000000,
                             demo=self.source.currentIndex() == 1)

    def test_kind_changed(self, kind):
        pulse = kind == 'Torque pulse'
        position = kind == 'Position'
        self.duration.setRange(.1 if pulse else 2, .5 if pulse else 60 if position else 10)
        self.duration.setSingleStep(.05 if pulse else .5)
        self.duration.setValue(.25 if pulse else 5)
        self.rpm.setEnabled(kind in ('Speed', 'Position'))
        self.test_form.labelForField(self.rpm).setText('Target move rate · rpm' if position else 'Speed · rpm')
        self.ff.setEnabled(kind == 'Speed')
        self.kd.setEnabled(not pulse)
        self.torque.setEnabled(pulse)
        if kind == 'Damping':
            self.kd.setValue(1)
        elif position:
            self.kd.setValue(1)
            self.rpm.setValue(abs(self.rpm.value()) or 5)
        if hasattr(self, 'plot_selectors') and kind in ('Speed', 'Position'):
            self.plot_selectors[0].setCurrentText('Position (degrees)' if position else 'Speed (rpm)')
        for field, visible in [(self.rpm, kind in ('Speed', 'Position')), (self.ff, kind == 'Speed'),
                               (self.kd, not pulse), (self.torque, pulse), (self.position_reference, position),
                               (self.position_deg, position), (self.use_current, position), (self.kp, position),
                               (self.travel_limit, not pulse)]:
            self.test_form.setRowVisible(field, visible)

    def position_reference_changed(self, reference):
        self.test_form.labelForField(self.position_deg).setText('Move by · degrees' if reference == 'Relative' else 'Go to · degrees')

    def use_current_position(self):
        snap = self.session.snapshot()
        if snap['latest'] and snap['age_s'] is not None and snap['age_s'] < .25:
            self.position_reference.setCurrentText('Absolute')
            self.position_deg.setValue(math.degrees(snap['latest']['position_rad']))

    def support_changed(self, checked):
        if not checked and self.session.snapshot()['armed']:
            self.session.request('stop')

    def test_values(self):
        return dict(kind=self.kind.currentText(), rpm=self.rpm.value(), kd=self.kd.value(),
                    ff=self.ff.value(), torque=self.torque.value(), duration=self.duration.value(),
                    position_deg=self.position_deg.value(), position_reference=self.position_reference.currentText(),
                    kp=self.kp.value(), speed_limit=self.speed_limit.value(), travel_limit=self.travel_limit.value())

    def run_test(self):
        self.session.request('test', **self.test_values())

    def scan_ids(self):
        if QMessageBox.question(self, 'Stop/status scan', 'This sends Stop to IDs 0–14 on the selected bus. Stop can release torque. Are all motors on this bus supported?') == QMessageBox.StandardButton.Yes:
            self.session.request('scan', supported=self.secured.isChecked())

    def change_plot(self, plot, text):
        plot.signal_label = text
        plot.update()

    def console_command(self):
        text = self.console.text().strip()
        self.console.clear()
        self.session._event('> ' + text)
        try:
            tokens = shlex.split(text)
            if tokens == ['help']:
                self.session._event('Commands: status | stop | check | ack-timeout | arm | speed <rpm> <seconds> | position <relative-degrees> <seconds> | goto <absolute-degrees> <seconds> | torque <Nm> <seconds> | damping <Kd> <seconds>. No shell/raw commands.')
            elif tokens == ['status']:
                snap = self.session.snapshot()
                self.session._event(json.dumps({k: snap[k] for k in ('state', 'id', 'latest', 'reason')}))
            elif len(tokens) == 1 and tokens[0] in ('stop', 'check', 'ack-timeout', 'arm'):
                self.session.request('ack' if tokens[0] == 'ack-timeout' else tokens[0], supported=self.secured.isChecked())
            elif len(tokens) == 3 and tokens[0] in ('speed', 'position', 'goto', 'torque', 'damping'):
                values = self.test_values()
                kind, field = {'speed': ('Speed', 'rpm'), 'position': ('Position', 'position_deg'),
                               'goto': ('Position', 'position_deg'), 'torque': ('Torque pulse', 'torque'), 'damping': ('Damping', 'kd')}[tokens[0]]
                values.update(kind=kind, duration=float(tokens[2]))
                values[field] = float(tokens[1])
                if kind == 'Position':
                    values['position_reference'] = 'Absolute' if tokens[0] == 'goto' else 'Relative'
                self.session.request('test', **values)
            else:
                raise ValueError('Unknown command. Type help.')
        except ValueError as error:
            self.session._event(str(error))

    def export_session(self):
        name, _ = QFileDialog.getSaveFileName(self, 'Export retained session', 'im6014-session.json', 'JSON (*.json);;CSV (*.csv)')
        if not name:
            return
        try:
            snap = self.session.snapshot()
            if Path(name).suffix.lower() == '.csv':
                with open(name, 'w', newline='') as file:
                    fields = ['time_s', 'demo', 'connection', 'port', 'baud', 'id', 'mode', 'timeout', 'rpm', 'target_rpm', 'target_position_rad', 'position_rad', 'torque_nm', 'voltage', 'driver_c', 'winding_c', 'errors', 'warnings', 'rtt_ms', 'raw']
                    writer = csv.DictWriter(file, fieldnames=fields, extrasaction='ignore')
                    writer.writeheader()
                    writer.writerows(snap['samples'])
            else:
                Path(name).write_text(json.dumps(snap, indent=2) + '\n')
            self.session._event(f'Exported {len(snap["samples"])} retained samples to {name}')
        except OSError as error:
            QMessageBox.warning(self, 'Export failed', str(error))

    def refresh(self):
        self.session.ping()
        snap = self.session.snapshot()
        connected = snap['connected']
        testing = snap['state'] == 'Testing'
        busy = snap['state'] in ('Connecting', 'Scanning IDs')
        selected_demo = self.source.currentIndex() == 1
        self.source_badge.setText(('DEMO · no hardware' if (snap['demo'] if connected else selected_demo) else 'REAL HARDWARE') + '  ·  ' + snap['state'])
        self.source_badge.setStyleSheet('padding:8px;font-weight:600;color:#273747;background:' + ('#fff0c7;' if (snap['demo'] if connected else selected_demo) else '#e9eef5;'))
        self.banner.setText(snap['reason'])
        self.banner.setToolTip(snap['reason'])
        self.banner.setStyleSheet('padding:8px;border:1px solid #cbd3dc;color:#273747;background:' + ('#ffe6e6;' if snap['state'].startswith(('Fault', 'Timeout')) else '#f3f6fa;'))
        for widget in (self.source, self.port, self.baud, self.motor_id, self.rescan):
            widget.setEnabled(not connected and not busy)
        self.connect_button.setEnabled(not connected and not busy)
        self.disconnect_button.setEnabled(connected and not busy)
        self.check_button.setEnabled(connected and not testing and not busy)
        self.ack_button.setEnabled(connected and not testing and not busy and bool(snap['latest'] and snap['latest']['timeout']))
        self.arm_button.setEnabled(snap['state'] == 'Ready · Stop' and self.secured.isChecked())
        self.run_button.setEnabled(snap['armed'] and not testing and not busy)
        self.scan_button.setEnabled(connected and not snap['armed'] and not testing and not busy and self.secured.isChecked())
        for widget in (self.kind, self.duration, self.speed_limit, self.travel_limit):
            widget.setEnabled(not testing and not busy)
        position_mode = self.kind.currentText() == 'Position'
        for widget, active in [(self.rpm, self.kind.currentText() in ('Speed', 'Position')), (self.ff, self.kind.currentText() == 'Speed'),
                               (self.kd, self.kind.currentText() != 'Torque pulse'), (self.torque, self.kind.currentText() == 'Torque pulse'),
                               (self.position_deg, position_mode), (self.position_reference, position_mode), (self.kp, position_mode)]:
            widget.setEnabled(active and not testing and not busy)
        self.progress.setValue(round(100 * (1 - snap['test_remaining'] / max(.1, self.duration.value()))) if testing else 0)
        self.progress.setFormat(f"{snap['test_remaining']:.1f} seconds remaining" if testing else 'No test running')
        f = snap['latest']
        fresh = snap['age_s'] is not None and snap['age_s'] < .25
        self.use_current.setEnabled(position_mode and fresh and not testing and not busy)
        for key, item in self.fields.items():
            val = f[key] if f else None
            if key == 'position_rad' and val is not None:
                val = math.degrees(val)
            if key == 'mode' and val is not None:
                val = 'Stop' if val == 0 else 'FOC'
            elif key in ('errors', 'warnings') and val is not None:
                val = f'0x{val:08x}'
            item.setText(1, '—' if val is None else f'{val:.3f}' if isinstance(val, float) else str(val))
            item.setForeground(1, self.tree.palette().brush(QPalette.ColorGroup.Active if fresh else QPalette.ColorGroup.Disabled, QPalette.ColorRole.Text))
        for key, card in self.cards.items():
            card.setText(f'{f[key]:.3f}' if f and fresh else '—')
        age = f"{snap['age_s'] * 1000:.0f} ms" if snap['age_s'] is not None else '—'
        self.link_status.setText(f"Reply age {age} · Round trip {snap['rtt_ms']:.1f} ms · Valid replies {snap['rx']} · CRC errors {snap['crc_errors']} · Missed replies {snap['reply_timeouts']}")
        for plot in self.plots:
            plot.samples = [x for x in snap['samples'] if x['connection'] == snap['connection']] if connected else []
            plot.update()
        diag = ['# Connection diagnosis', f"**State:** {snap['state']}", f"**Feedback age:** {age}",
                f"**Selected motor:** ID {snap['id']} · {snap['baud']:,} baud", f"**Adapter:** `{snap['port']}`",
                f"**Stop confirmed:** {'yes' if snap['stopped'] else 'no'}", f"**Link:** {snap['tx']} requests / {snap['rx']} valid replies",
                f"**Packet failures:** {snap['crc_errors']} CRC / {snap['reply_timeouts']} deadlines"]
        if f:
            diag += ['', '## Motor report', fault_text(f['errors'], f['warnings']),
                     f"Timeout latch: {'triggered' if f['timeout'] else 'normal'}", f"Voltage: {f['voltage']:.1f} V · Driver {f['driver_c']} °C · Winding {f['winding_c']} °C"]
        diag += ['', '## Next checks', 'No replies: confirm motor power, Channel D, matching baud and correct ID. Adapter detection alone is not motor communication.',
                 'Timeout latch: support the motor, then explicitly acknowledge timeout in Stop. Fault bits: resolve the reported cause before re-checking; this app does not clear faults.',
                 'CRC / deadline failures: inspect cable and adapter, reconnect while stopped, then re-check. No motion resumes automatically.',
                 '', '## Discovered motors', '\n'.join(f"- ID {x['id']}: {x['voltage']:.1f} V · {x['faults']}" for x in snap['discovered']) or 'Use Discover IDs on a supported, disarmed bus.',
                 '', f"Capture: {len(snap['samples'])} retained of {snap['total_samples']} samples. Last 10,000 samples / 500 events are retained."]
        self.diagnosis.setMarkdown('\n\n'.join(diag))
        if snap['events'] != self.last_events:
            self.event_log.setPlainText('\n'.join(snap['events']))
            self.event_log.verticalScrollBar().setValue(self.event_log.verticalScrollBar().maximum())
            self.last_events = snap['events']
        self.packets.setPlainText(f"TX · 20 bytes\n{snap['last_tx']}\n\nRX · 26 bytes · last validated packet\n{snap['last_rx']}\n\nCRC32: request bytes 0–15; feedback bytes 2–21.\nLast packets remain visible after a fault; check reply age.")

    def closeEvent(self, event):
        self.timer.stop()
        self.session.close()
        event.accept()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--demo', action='store_true', help='Select demo; still starts disconnected')
    args = parser.parse_args()
    app = QApplication(sys.argv[:1])
    app.setStyle('Fusion')
    window = Dashboard(args.demo)
    window.show()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
