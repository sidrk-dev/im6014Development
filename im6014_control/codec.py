import math
import struct
from collections.abc import Callable

RATIO = 38 / 3
RPM_MIN = -32768 / 64 / RATIO * 60
RPM_MAX = 32767 / 64 / RATIO * 60
POSITION_MIN = -2147483648 / 32768 / RATIO * math.tau
POSITION_MAX = 2147483647 / 32768 / RATIO * math.tau
RAW_POSITION_MIN = -2147483648
RAW_POSITION_MAX = 2147483647
RAW_RPM_MIN = -32768
RAW_RPM_MAX = 32767


def raw_to_position(raw_position):
    if type(raw_position) is not int or not RAW_POSITION_MIN <= raw_position <= RAW_POSITION_MAX:
        raise ValueError('raw_position must be a signed 32-bit integer')
    return raw_position / 32768 / RATIO * math.tau


def position_to_raw(position):
    position = _finite_control(position, 'Position')
    return _signed_raw(position / math.tau * RATIO * 32768,
                       RAW_POSITION_MIN, RAW_POSITION_MAX, 'Position')


def raw_to_rpm(raw_rpm):
    if type(raw_rpm) is not int or not RAW_RPM_MIN <= raw_rpm <= RAW_RPM_MAX:
        raise ValueError('raw_rpm must be a signed 16-bit integer')
    return raw_rpm / 64 / RATIO * 60


def rpm_to_raw(rpm):
    rpm = _finite_control(rpm, 'Speed')
    return _signed_raw(rpm / 60 * RATIO * 64, RAW_RPM_MIN, RAW_RPM_MAX, 'Speed')


def _motor_id(value, name='Individual ID'):
    if type(value) is not int or not 0 <= value <= 14:
        raise ValueError(f'{name} must be an integer from 0 to 14')
    return value


def _finite_control(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{name} must be a finite int or float')
    try:
        result = float(value)
    except (OverflowError, TypeError, ValueError) as error:
        raise ValueError(f'{name} must be finite') from error
    if not math.isfinite(result):
        raise ValueError(f'{name} must be finite')
    return result


def _signed_raw(value, minimum, maximum, name):
    raw = round(value)
    if not minimum <= raw <= maximum:
        raise ValueError(f'{name} cannot be represented by the motor protocol')
    return raw


def _torque_raw(value):
    return _signed_raw(value / RATIO * 2560, -32768, 32767, 'Feedforward torque')


def _kp_raw(value):
    return _signed_raw(value / RATIO ** 2 * 12800, -32768, 32767, 'Kp')


def _kd_raw(value):
    return _signed_raw(value / RATIO ** 2 * 51200, -32768, 32767, 'Kd')


def _raw_position(value):
    if type(value) is not int or not RAW_POSITION_MIN <= value <= RAW_POSITION_MAX:
        raise ValueError('raw_position must be a signed 32-bit integer')
    return value


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
    position = _finite_control(position, 'Position')
    if not POSITION_MIN <= position <= POSITION_MAX:
        raise ValueError('Position cannot be encoded in the motor protocol')
    if mode == 0 and position != 0.0:
        raise ValueError('Stop must have zero targets; FOC requires watchdog')
    position_raw = _signed_raw(position / math.tau * RATIO * 32768,
                               RAW_POSITION_MIN, RAW_POSITION_MAX, 'Position')
    return command_with_raw_position(
        motor_id,
        position_raw,
        mode=mode,
        rpm=rpm,
        kd=kd,
        torque=torque,
        watchdog=watchdog,
        kp=kp,
    )


def command_with_raw_position(
    motor_id, raw_position, mode=0, rpm=0, kd=0, torque=0, watchdog=True, kp=0, raw_rpm=None
):
    """Encode a command while preserving an already selected position grid code."""
    motor_id = _motor_id(motor_id)
    raw_position = _raw_position(raw_position)
    if type(mode) is not int or mode not in (0, 1):
        raise ValueError('Individual ID 0–14 and Stop/FOC modes only')
    if type(watchdog) is not bool:
        raise ValueError('Watchdog must be a bool')
    rpm, kd, torque, kp = (
        _finite_control(value, name) for value, name in
        ((rpm, 'Speed'), (kd, 'Kd'), (torque, 'Feedforward torque'),
         (kp, 'Kp'))
    )
    if not RPM_MIN <= rpm <= RPM_MAX:
        raise ValueError('Speed cannot be encoded in the motor protocol; this is not a rated-speed limit')
    if not 0 <= kd <= 6 or not 0 <= kp <= 30 or not -3.5 <= torque <= 3.5:
        raise ValueError('Bench limits: Kd 0–6, Kp 0–30, feedforward torque ±3.5 Nm')
    torque_raw = _torque_raw(torque)
    if raw_rpm is not None:
        if type(raw_rpm) is not int or not RAW_RPM_MIN <= raw_rpm <= RAW_RPM_MAX:
            raise ValueError('raw_rpm must be a signed 16-bit integer')
        rpm_raw = raw_rpm
    else:
        rpm_raw = _signed_raw(rpm / 60 * RATIO * 64, RAW_RPM_MIN, RAW_RPM_MAX, 'Speed')
    kp_raw = _kp_raw(kp)
    kd_raw = _kd_raw(kd)
    if mode == 0 and (rpm or kd or torque or raw_position or kp or (raw_rpm is not None and raw_rpm != 0)) or mode == 1 and not watchdog:
        raise ValueError('Stop must have zero targets; FOC requires watchdog')
    raw = b'\xfe\xee' + bytes([motor_id | mode << 4 | int(watchdog) << 7, 0])
    raw += struct.pack('<hhihh', torque_raw, rpm_raw, raw_position, kp_raw, kd_raw)
    return raw + struct.pack('<I', crc32(raw))


def project_position_raw(
    wanted_q,
    q_min,
    q_max,
    map_decoded_position: Callable[[float], float],
    direction: int,
) -> int | None:
    """Choose the nearest raw position whose mapped decoded value is in bounds.

    ``map_decoded_position`` maps decoded actuator radians to the caller's
    position domain. ``direction`` declares whether that mapping increases or
    decreases as the raw position code increases. Bounds are closed.
    """
    wanted_q = _finite_control(wanted_q, 'Wanted position')
    q_min = _finite_control(q_min, 'Minimum position')
    q_max = _finite_control(q_max, 'Maximum position')
    if q_min > q_max:
        raise ValueError('Minimum position must not exceed maximum position')
    if not callable(map_decoded_position):
        raise ValueError('Position mapping must be callable')
    if type(direction) is not int or direction not in (-1, 1):
        raise ValueError('Position mapping direction must be -1 or 1')

    ordered_min = direction * (q_min if direction == 1 else q_max)
    ordered_max = direction * (q_max if direction == 1 else q_min)
    ordered_wanted = direction * wanted_q

    def mapped(raw_position):
        decoded = raw_to_position(raw_position)
        try:
            result = map_decoded_position(decoded)
        except (ArithmeticError, TypeError, ValueError) as error:
            raise ValueError('Position mapping is not representable') from error
        return direction * _finite_control(result, 'Mapped position')

    def first_raw_satisfying(bound, *, strict):
        left = RAW_POSITION_MIN
        right = RAW_POSITION_MAX + 1
        while left < right:
            mid = (left + right) // 2
            value = mapped(mid)
            matches = value > bound if strict else value >= bound
            if matches:
                right = mid
            else:
                left = mid + 1
        return left

    first = first_raw_satisfying(ordered_min, strict=False)
    last = first_raw_satisfying(ordered_max, strict=True) - 1
    if first > RAW_POSITION_MAX or last < first:
        return None

    candidate = first_raw_satisfying(ordered_wanted, strict=False)
    if candidate < first:
        return first
    if candidate > last:
        return last
    if candidate == first:
        return candidate

    current_delta = abs(mapped(candidate) - ordered_wanted)
    prior_delta = abs(mapped(candidate - 1) - ordered_wanted)
    return candidate - 1 if prior_delta <= current_delta else candidate


def project_speed_raw(
    wanted_dq,
    v_min,
    v_max,
    map_decoded_speed: Callable[[float], float],
    direction: int,
) -> int | None:
    """Choose the nearest raw speed whose mapped decoded value is in bounds.

    ``map_decoded_speed`` maps decoded actuator RPM to the caller's
    speed domain. ``direction`` declares whether that mapping increases or
    decreases as the raw speed code increases. Bounds are closed.
    """
    wanted_dq = _finite_control(wanted_dq, 'Wanted speed')
    v_min = _finite_control(v_min, 'Minimum speed')
    v_max = _finite_control(v_max, 'Maximum speed')
    if v_min > v_max:
        raise ValueError('Minimum speed must not exceed maximum speed')
    if not callable(map_decoded_speed):
        raise ValueError('Speed mapping must be callable')
    if type(direction) is not int or direction not in (-1, 1):
        raise ValueError('Speed mapping direction must be -1 or 1')

    ordered_min = direction * (v_min if direction == 1 else v_max)
    ordered_max = direction * (v_max if direction == 1 else v_min)
    ordered_wanted = direction * wanted_dq

    def mapped(raw_speed):
        decoded = raw_to_rpm(raw_speed)
        try:
            result = map_decoded_speed(decoded)
        except (ArithmeticError, TypeError, ValueError) as error:
            raise ValueError('Speed mapping is not representable') from error
        return direction * _finite_control(result, 'Mapped speed')

    def first_raw_satisfying(bound, *, strict):
        left = RAW_RPM_MIN
        right = RAW_RPM_MAX + 1
        while left < right:
            mid = (left + right) // 2
            value = mapped(mid)
            matches = value > bound if strict else value >= bound
            if matches:
                right = mid
            else:
                left = mid + 1
        return left

    first = first_raw_satisfying(ordered_min, strict=False)
    last = first_raw_satisfying(ordered_max, strict=True) - 1
    if first > RAW_RPM_MAX or last < first:
        return None

    candidate = first_raw_satisfying(ordered_wanted, strict=False)
    if candidate < first:
        return first
    if candidate > last:
        return last
    if candidate == first:
        return candidate

    current_delta = abs(mapped(candidate) - ordered_wanted)
    prior_delta = abs(mapped(candidate - 1) - ordered_wanted)
    return candidate - 1 if prior_delta <= current_delta else candidate


def feedback(raw, motor_id):
    motor_id = _motor_id(motor_id, 'Expected motor ID')
    if not isinstance(raw, (bytes, bytearray, memoryview)):
        raise ValueError('Feedback frame must be a bytes-like buffer')
    try:
        raw = bytes(raw)
    except (TypeError, ValueError) as error:
        raise ValueError('Feedback frame must be a bytes-like buffer') from error
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


def decode_command(raw):
    if not isinstance(raw, (bytes, bytearray, memoryview)):
        raise ValueError('Command frame must be a bytes-like buffer')
    try:
        raw = bytes(raw)
    except (TypeError, ValueError) as error:
        raise ValueError('Command frame must be a bytes-like buffer') from error
    if len(raw) != 20 or raw[:2] != b'\xfe\xee':
        raise ValueError('Invalid command length/header')
    if struct.unpack('<I', raw[16:])[0] != crc32(raw[:16]):
        raise ValueError('Command CRC mismatch')
    m = raw[2]
    motor_id = m & 0x0F
    mode = (m >> 4) & 0x07
    watchdog = bool(m & 0x80)
    if motor_id > 14 or mode not in (0, 1) or raw[3] != 0:
        raise ValueError('Invalid command ID, mode, or reserved byte')
    if mode == 1 and not watchdog:
        raise ValueError('FOC command requires watchdog')
    tq, sp, pos, kp, kd = struct.unpack('<hhihh', raw[4:16])
    if mode == 0 and (tq or sp or pos or kp or kd):
        raise ValueError('Stop command must have zero targets')
    decoded = dict(
        motor_id=motor_id,
        mode=mode,
        watchdog=watchdog,
        raw_torque=tq,
        raw_rpm=sp,
        raw_position=pos,
        raw_kp=kp,
        raw_kd=kd,
        torque_nm=tq / 2560 * RATIO,
        rpm=sp / 64 / RATIO * 60,
        position_rad=pos / 32768 / RATIO * math.tau,
        kp=kp * (RATIO ** 2) / 12800,
        kd=kd * (RATIO ** 2) / 51200,
        raw=raw.hex(),
    )
    if not _torque_raw(-3.5) <= tq <= _torque_raw(3.5):
        raise ValueError('Feedforward torque is outside the canonical command range')
    if not 0 <= kp <= _kp_raw(30.0) or not 0 <= kd <= _kd_raw(6.0):
        raise ValueError('Gain is outside the canonical command range')
    return decoded
