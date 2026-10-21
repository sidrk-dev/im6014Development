# Rolling-arm simulator station

`RollingStation` joins the shared controller to the selected
`sim_rolling_reach_v1` model: one arm, three logical coordinates, five internal
hinges, and one overhead RGB camera. One owner manages physics, feedback,
captures, commands, and lifecycle. Producers exchange immutable captures and
replies; they receive RGB, measured logical positions, sequence, and capture time.

The station advances 30 actual physics steps per call. Integer scheduling gives
3000 Hz physics, 100 Hz control, and 30 Hz camera opportunities in simulation
time. The capture/intent lifetime is 250 ms, feedback maximum age is 50 ms,
reference step is 10 ms, and measured lead is 0.1 rad. The age limits are explicit
station assumptions, rather than a previously accepted selected SDK age contract.

Every resubmission retains the original capture time, expiry, source, and actual
ownership. A newer retained proposal cannot extend an older applied command's
deadline before a replacement successfully passes release. Expiry is enforced
on every physics step, including between control boundaries. Infeasible planning
writes nothing. Terminal release rechecks the actual core grant and source,
current tick, model identity, and complete command vector.

Full normalized MJB identity protects geometry, dynamics, gains, timing, and
camera calibration. Only marker GEOM XY and the actuation disable bit are
normalized in an independent clone. Actuation authority separately checks that
bit. Model, data, core, and ownership objects remain private.

## Stop and startup

Construction is disconnected with actuation disabled. Checking and arming do
not enable servos; only a released reference can enable them. Stop disables
actuation and produces zero actuator force, allowing gravity sag or falling.
It is not a holding controller. Recovery requires explicit fresh checks and arm.

An unprimed 50 ms first reply safely issued no commands: gravity had already made
the bounded reference infeasible. It did not silently reset the seed. A separate
probe explicitly returned the first captured measured pose at the next boundary
(10 ms), then delayed later fresh replies by 50 ms. That bounded local startup
hold passed normal release and retained its original deadline.

## Parent verification

- Full offline suite: 159 controller tests and 18 bench groups passed, with no
  motor I/O. The bench environment skipped four simulator-dependent contract
  cases, covered by the separate pinned suite.
- Pinned simulator/SDK suite: 76 passed, including 25 station tests and all 51
  inherited cases. SDK transport was mocked.
- Independent connected probe: 30,000 physics steps, 1,000 control ticks, 300
  captures, 299 accepted replies, and 998 releases with the explicit startup hold
  followed by 50 ms reply delays. Final reference exactly matched the joint goal;
  measured joints exhibited gravity sag.
- A previously failing takeover regression now issues zero commands after the
  actual core owner changes between consume and release. Tests also cover exact
  expiry, original held-command lifetime, malformed/reordered/forged replies,
  changed models, force mapping, faults, Stop, and explicit recovery.
- Independent full source review found no actionable defects in the frozen
  snapshot. It ran no runtime tests. The provider receipt pins the requested
  Gemini model in argv; no returned model identifier was verified.

Selected results and source hashes are in [verification.json](verification.json).

## Remaining gates

SA04 remains in progress. Injected delays establish simulated scheduling only.
Actual hosted-worker timing, transaction latency, overruns, feedback-age
telemetry, dashboard-failure integration, and remaining C14-C16 acceptance still
need evidence. Thermal fault injection is state-machine evidence, not a thermal
model or physical drive rating. No learned marker-reaching, dataset, hosted
checkpoint, or physical outcome is established by these joint-goal probes.

## Shared-controller station

This report records the partial SA04 integration and remaining gates. Run its disconnected, physics, provenance,
expiry, fault, and delayed-reply checks with the pinned simulator environment:

```bash
MUJOCO_GL=egl .venv-rolling/bin/python -m unittest tests.test_rolling_station
```

A bounded local startup command uses the first captured measured pose:

```python
from examples.rolling_reach_arm.station import RollingStation, Reply

station = RollingStation()
try:
    assert station.check().accepted
    assert station.arm(owner="local-example").accepted
    capture = station.advance_control_tick()[0]
    station.advance_control_tick(replies=(Reply(capture, capture.q),))
    print(station.status())
    station.stop()
finally:
    station.close()
```

Arming alone does not enable actuation. This initial command retains its capture
deadline; later replies cannot renew an already applied command's lifetime until
a replacement passes release. Stop releases actuator force and can permit
gravity motion. The local example makes no hosted request or learned-task claim.
