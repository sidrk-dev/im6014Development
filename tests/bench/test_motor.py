"""Offline packet, transport, and safety-gate checks. No real serial ports."""
import math
import struct

from im6014_bench import motor as s


def must_fail(call):
    try:
        call()
    except (RuntimeError, ValueError):
        return
    raise AssertionError('Unsafe input was accepted')


def frame(mode=0, winding=200, volts=150, errors=0):
    body = struct.pack('<BbBBhhiIHBB', mode << 4, 35, winding, volts, -11, 0, -9945, errors, 0, 0, 0)
    return b'\xfc\xee' + body + struct.pack('<I', s.crc32(body))


assert s.command(0).hex() == 'feee80000000000000000000000000001d99fb6e'
p = s.command(0, 1, 5, 6)
assert p[2] == 0x90
assert struct.unpack('<hhihh', p[4:16]) == (0, 68, 0, 0, 1915)
f = s.feedback(frame(), 0)
assert f['voltage'] == 75 and f['winding_c'] == 200 and f['position_rad'] < 0
must_fail(lambda: s.healthy(f, 0))
for raw in [frame()[:-1], frame()[:10] + b'\0' + frame()[11:]]:
    must_fail(lambda: s.feedback(raw, 0))
must_fail(lambda: s.feedback(frame(), 1))
for rpm in [math.nan, math.inf, s.RPM_MIN - .1, s.RPM_MAX + .1]:
    must_fail(lambda: s.command(0, 1, rpm))
must_fail(lambda: s.command(15))
assert struct.unpack('<h', s.command(0, 1, -5)[6:8])[0] == -68
for rpm, encoded in [(60, 811), (-60, -811), (s.RPM_MIN, -32768), (s.RPM_MAX, 32767)]:
    assert struct.unpack('<h', s.command(0, 1, rpm)[6:8])[0] == encoded
position = s.command(0, 1, position=-math.pi / 2, kp=3, kd=1)
assert struct.unpack('<hhihh', position[4:16]) == (0, 0, -103765, 239, 319)
assert struct.unpack('<I', position[16:])[0] == s.crc32(position[:16])
for kwargs in [dict(position=math.nan), dict(position=1e9), dict(position=1e308), dict(kp=31), dict(kp=-1)]:
    must_fail(lambda: s.command(0, 1, **kwargs))
for value, encoded in [(s.POSITION_MIN, -2147483648), (s.POSITION_MAX, 2147483647)]:
    assert struct.unpack('<i', s.command(0, 1, position=value)[8:12])[0] == encoded
must_fail(lambda: s.command(0, 1, watchdog=False))
must_fail(lambda: s.command(0, 0, rpm=1))
must_fail(lambda: s.command(0, 0, position=1, kp=1))
must_fail(lambda: s.crc32(b'abc'))
healthy = s.feedback(frame(winding=30, volts=72), 0)
s.healthy(healthy, 0)
for key, val in [('mode', 1), ('errors', 1), ('warnings', 1), ('timeout', 1), ('torque_nm', 4.1)]:
    must_fail(lambda: s.healthy(dict(healthy, **{key: val}), 0))
s.healthy(dict(healthy, rpm=60), 0)
must_fail(lambda: s.healthy(dict(healthy, rpm=60), 0, speed_limit=40))


class FakeSerial:
    timeout = 0
    in_waiting = 1

    def __init__(self, partial=False):
        self.partial = partial
        self.data = bytearray()

    def reset_input_buffer(self):
        self.data.clear()

    def write(self, packet):
        self.data = bytearray(b'noise' + frame(winding=30, volts=72))
        return len(packet) - int(self.partial)

    def read(self, size):
        chunk = self.data[:1]
        del self.data[:1]
        return chunk


assert s.exchange(FakeSerial(), s.command(0), 0)['voltage'] == 36
must_fail(lambda: s.exchange(FakeSerial(True), s.command(0), 0))
fake = FakeSerial()
fake.write = lambda packet: len(packet)
must_fail(lambda: s.exchange(fake, s.command(0), 0, .002))
print('PASS: packet units/CRC, signed and unsigned parsing, fragmented/noisy replies, partial-write/deadline and safety rejection checks; no motor I/O')
