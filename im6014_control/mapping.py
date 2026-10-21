from dataclasses import dataclass
import math


def _finite(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a real int or float")
    try:
        result = float(value)
    except (OverflowError, TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be finite") from exc
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def _result(value, name, nonzero_inputs=None):
    if not math.isfinite(value):
        raise ValueError(f"{name} result must be finite")
    if nonzero_inputs and value == 0.0 and all(item != 0.0 for item in nonzero_inputs):
        raise ValueError(f"{name} result underflowed")
    return value


def _multiply(name, *values):
    result = 1.0
    for index, value in enumerate(values):
        result *= value
        _result(result, name, values[:index + 1])
    return result


def _divide(numerator, denominator, name):
    try:
        result = numerator / denominator
    except (OverflowError, ZeroDivisionError) as exc:
        raise ValueError(f"{name} result is not representable") from exc
    return _result(result, name, (numerator,))


def _add(left, right, name):
    return _result(left + right, name)


def _subtract(left, right, name):
    return _result(left - right, name)


@dataclass(frozen=True)
class JointMap:
    reduction: float
    direction: int
    zero_output_rad: float

    def __post_init__(self):
        reduction = _finite(self.reduction, "reduction")
        if reduction <= 0.0:
            raise ValueError("reduction must be positive")
        if type(self.direction) is not int or self.direction not in (-1, 1):
            raise ValueError("direction must be the integer -1 or 1")
        object.__setattr__(self, "reduction", reduction)
        object.__setattr__(self, "zero_output_rad", _finite(self.zero_output_rad, "zero_output_rad"))

    def to_output(self, target: "JointTarget") -> "OutputTarget":
        if not isinstance(target, JointTarget):
            raise ValueError("target must be a JointTarget")

        signed_reduction = _multiply("signed reduction", float(self.direction), self.reduction)
        q_rad = _add(
            self.zero_output_rad,
            _multiply("output position", signed_reduction, target.q_rad),
            "output position",
        )
        output_rad_s = _multiply("output velocity", signed_reduction, target.dq_rad_s)
        rpm = _divide(_multiply("output speed", output_rad_s, 60.0), math.tau, "output speed")
        tau_nm = _divide(
            _multiply("output torque", float(self.direction), target.tau_nm),
            self.reduction,
            "output torque",
        )
        kp = _divide(_divide(target.kp, self.reduction, "output kp"), self.reduction, "output kp")
        kd = _divide(_divide(target.kd, self.reduction, "output kd"), self.reduction, "output kd")
        return OutputTarget(q_rad=q_rad, rpm=rpm, tau_nm=tau_nm, kp=kp, kd=kd)

    def from_output(self, sample: "OutputSample") -> "JointSample":
        if not isinstance(sample, OutputSample):
            raise ValueError("sample must be an OutputSample")

        q_delta = _subtract(sample.q_rad, self.zero_output_rad, "joint position")
        q_rad = _divide(
            _multiply("joint position", float(self.direction), q_delta),
            self.reduction,
            "joint position",
        )
        output_rad_s = _divide(
            _multiply("joint velocity", sample.rpm, math.tau),
            60.0,
            "joint velocity",
        )
        dq_rad_s = _divide(
            _multiply("joint velocity", float(self.direction), output_rad_s),
            self.reduction,
            "joint velocity",
        )
        tau_nm = _multiply("joint torque", float(self.direction), self.reduction, sample.tau_nm)
        return JointSample(q_rad=q_rad, dq_rad_s=dq_rad_s, tau_nm=tau_nm)


@dataclass(frozen=True)
class JointTarget:
    q_rad: float
    dq_rad_s: float
    tau_nm: float
    kp: float
    kd: float

    def __post_init__(self):
        for name in ("q_rad", "dq_rad_s", "tau_nm", "kp", "kd"):
            object.__setattr__(self, name, _finite(getattr(self, name), name))
        if self.kp < 0.0 or self.kd < 0.0:
            raise ValueError("kp and kd must be nonnegative")


@dataclass(frozen=True)
class OutputTarget:
    q_rad: float
    rpm: float
    tau_nm: float
    kp: float
    kd: float

    def __post_init__(self):
        for name in ("q_rad", "rpm", "tau_nm", "kp", "kd"):
            object.__setattr__(self, name, _finite(getattr(self, name), name))
        if self.kp < 0.0 or self.kd < 0.0:
            raise ValueError("kp and kd must be nonnegative")


@dataclass(frozen=True)
class OutputSample:
    q_rad: float
    rpm: float
    tau_nm: float

    def __post_init__(self):
        for name in ("q_rad", "rpm", "tau_nm"):
            object.__setattr__(self, name, _finite(getattr(self, name), name))


@dataclass(frozen=True)
class JointSample:
    q_rad: float
    dq_rad_s: float
    tau_nm: float

    def __post_init__(self):
        for name in ("q_rad", "dq_rad_s", "tau_nm"):
            object.__setattr__(self, name, _finite(getattr(self, name), name))
