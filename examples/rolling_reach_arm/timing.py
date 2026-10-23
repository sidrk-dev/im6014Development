from __future__ import annotations

from dataclasses import dataclass, field
import math


ADVANCE_BUDGET_WALL_S = 0.010


@dataclass(frozen=True)
class TimingStats:
    """Aggregate of one observed duration population."""

    count: int = 0
    missing: int = 0
    last_s: float | None = None
    min_s: float | None = None
    max_s: float | None = None
    mean_s: float | None = None


@dataclass(frozen=True)
class StationTiming:
    """Frozen diagnostic snapshot; carries no model, core or accumulator."""

    callback_period_wall_s: TimingStats = field(default_factory=TimingStats)
    advance_wall_s: TimingStats = field(default_factory=TimingStats)
    control_wall_s: TimingStats = field(default_factory=TimingStats)
    accepted_reply_sim_s: TimingStats = field(default_factory=TimingStats)
    rejected_reply_sim_s: TimingStats = field(default_factory=TimingStats)
    accepted_reply_wall_s: TimingStats = field(default_factory=TimingStats)
    rejected_reply_wall_s: TimingStats = field(default_factory=TimingStats)
    advances_started: int = 0
    advances_completed: int = 0
    advances_failed: int = 0
    replies_offered: int = 0
    replies_unprocessed: int = 0
    clock_errors: int = 0
    advance_overruns: int = 0
    advance_budget_wall_s: float = ADVANCE_BUDGET_WALL_S
    feedback_age_sim_s: tuple[float | None, ...] = ()
    feedback_peak_observed_sim_s: float | None = None


class _Accumulator:
    __slots__ = ("count", "missing", "last_s", "min_s", "max_s", "mean_s")

    def __init__(self) -> None:
        self.count = 0
        self.missing = 0
        self.last_s: float | None = None
        self.min_s: float | None = None
        self.max_s: float | None = None
        self.mean_s = 0.0

    def add(self, value: float) -> None:
        self.count += 1
        self.last_s = value
        self.min_s = value if self.min_s is None else min(self.min_s, value)
        self.max_s = value if self.max_s is None else max(self.max_s, value)
        self.mean_s += (value - self.mean_s) / self.count

    def miss(self) -> None:
        self.missing += 1

    def snapshot(self) -> TimingStats:
        mean = self.mean_s if self.count else None
        return TimingStats(
            self.count,
            self.missing,
            self.last_s,
            self.min_s,
            self.max_s,
            mean,
        )


class _StationTimingState:
    def __init__(self) -> None:
        self._callback = _Accumulator()
        self._advance = _Accumulator()
        self._control = _Accumulator()
        self._accepted_sim = _Accumulator()
        self._rejected_sim = _Accumulator()
        self._accepted_wall = _Accumulator()
        self._rejected_wall = _Accumulator()
        self._callback_started = False
        self._callback_previous: tuple[int, float] | None = None
        self._advances_started = 0
        self._advances_completed = 0
        self._advances_failed = 0
        self._replies_offered = 0
        self._replies_unprocessed = 0
        self._clock_errors = 0
        self._advance_overruns = 0
        self._feedback_peak: float | None = None

    @staticmethod
    def _span(
        start: tuple[int, float] | None,
        end: tuple[int, float] | None,
    ) -> float | None:
        if start is None or end is None or start[0] != end[0]:
            return None
        value = end[1] - start[1]
        if not math.isfinite(value) or value < 0.0:
            return None
        return value

    def callback_entry(self, sample: tuple[int, float] | None) -> None:
        if not self._callback_started:
            self._callback_started = True
            self._callback_previous = sample
            return
        previous = self._callback_previous
        if (
            sample is not None
            and previous is not None
            and sample[0] == previous[0]
        ):
            self._callback.add(sample[1] - previous[1])
        else:
            self._callback.miss()
        self._callback_previous = sample

    def advance_started(self) -> None:
        self._advances_started += 1

    def advance_finished(
        self,
        start: tuple[int, float] | None,
        end: tuple[int, float] | None,
        failed: bool,
    ) -> None:
        if failed:
            self._advances_failed += 1
            self._advance.miss()
            return
        self._advances_completed += 1
        value = self._span(start, end)
        if value is None:
            self._advance.miss()
            return
        self._advance.add(value)
        if value > ADVANCE_BUDGET_WALL_S:
            self._advance_overruns += 1

    def control_finished(
        self,
        start: tuple[int, float] | None,
        end: tuple[int, float] | None,
        failed: bool,
    ) -> None:
        value = None if failed else self._span(start, end)
        if value is None:
            self._control.miss()
        else:
            self._control.add(value)

    def reply(
        self,
        accepted: bool,
        sim_age: float | None,
        wall_age: float | None,
    ) -> None:
        sim = self._accepted_sim if accepted else self._rejected_sim
        wall = self._accepted_wall if accepted else self._rejected_wall
        if sim_age is None:
            sim.miss()
        else:
            sim.add(sim_age)
        if wall_age is None:
            wall.miss()
        else:
            wall.add(wall_age)

    def replies_offered(self, count: int) -> None:
        self._replies_offered += count

    def replies_unprocessed(self, count: int) -> None:
        self._replies_unprocessed += count

    def clock_error(self) -> None:
        self._clock_errors += 1

    def observe_peak(self, ages: tuple[float | None, ...]) -> None:
        for age in ages:
            if age is None or isinstance(age, bool) or not isinstance(
                age, (int, float)
            ):
                continue
            value = float(age)
            if not math.isfinite(value) or value < 0.0:
                continue
            if self._feedback_peak is None or value > self._feedback_peak:
                self._feedback_peak = value

    def snapshot(
        self, feedback_ages: tuple[float | None, ...]
    ) -> StationTiming:
        return StationTiming(
            self._callback.snapshot(),
            self._advance.snapshot(),
            self._control.snapshot(),
            self._accepted_sim.snapshot(),
            self._rejected_sim.snapshot(),
            self._accepted_wall.snapshot(),
            self._rejected_wall.snapshot(),
            self._advances_started,
            self._advances_completed,
            self._advances_failed,
            self._replies_offered,
            self._replies_unprocessed,
            self._clock_errors,
            self._advance_overruns,
            ADVANCE_BUDGET_WALL_S,
            tuple(feedback_ages),
            self._feedback_peak,
        )
