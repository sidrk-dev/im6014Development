# Rolling station timing diagnostics

`RollingStation.status().timing` exposes frozen, bounded lifetime aggregates. Wall seconds measure caller entry spacing, complete advance work (physics, rendering and control), and control work separately. Simulation seconds measure authenticated capture-to-control age and current feedback age from the shared core. Neither diagnostic wall timestamps nor callbacks grant authority or change capture expiry.

A reply's wall turnaround starts when its original rendered capture becomes available and ends at the admitted reply call. This includes local caller delay. It is not hosted transaction latency. Accepted/rejected reply populations are separate; unknown or forged capture identity gives missing age. Offers not processed by control are counted separately. An accepted intent need not produce an issued reference.

Zero is a measured value; missing clocks/spans are counted explicitly. An overrun is a completed valid advance strictly longer than 10 ms, with valid completed advances as denominator. Failed advances rethrow the original station error, retain partial physics progress and count a missing duration. Feedback peak means the maximum age actually observed at refresh/status boundaries, not continuous monitoring or the configured age limit. Stop/check/disconnect preserve lifetime counters; disconnected current feedback ages are absent.

Diagnostic clock failures, nonfinite/retrograde values and public API reentry attempts invalidate samples and recover through a new clock epoch. The guard includes numeric return conversion. Running means remain finite for finite nonnegative duration populations across clock recovery epochs. No diagnostic callback occurs between final release validation and actuation. The station remains a sole-writer object; this is not process isolation or thread safety.

## Parent evidence

All 97 pinned simulator/SDK tests passed, including 21 timing cases and inherited 76 cases. The offline suite passed 159 controller tests and 18 bench groups; four simulator-dependent cases skipped there are covered by the pinned suite. SDK transport was mocked. The independent clock-return probe failed before the fix (silent Stop or aborted stepping), then preserved armed state and 30 steps with diagnostic errors after the fix.

Five measured unpaced local runs each executed 1,200 actual physics steps, 40 control ticks, 12 captures, 12 accepted replies and 38 releases. Constructor and one warmup run are excluded. Mean full advance work was 10.9–11.4 ms, while mean control work was 0.92–1.03 ms. Each run observed 12/40 advances above 10 ms. Rendering dominated a separate excluded profiling run. These measurements do not establish 100 Hz wall-time scheduling. Local capture echo does not establish hosted latency, learned behavior or marker-reaching success.

## Remaining acceptance

SA04 remains in progress. Actual hosted transaction timing and dashboard/process-failure integration need separate evidence. C14 fault-bit software retention is being checked separately; no physical temperature or thermal rating is measured here. No motor port, hosted account or dataset was accessed.

Verification summary and source hashes: [timing-verification.json](timing-verification.json).

## Inspect diagnostics

```python
from examples.rolling_reach_arm.station import RollingStation

station = RollingStation()
try:
    station.advance_control_tick()  # disconnected simulation steps only
    print(station.status().timing)
finally:
    station.close()
```

Arming alone still does not enable actuation. Explicit Stop disables force and permits gravity sag; it is not a holding controller.
