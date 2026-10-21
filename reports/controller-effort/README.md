# Scalar effort and release checks

The shared controller now requires a finite positive logical `max_effort_nm` for
every joint. It predicts stiffness, damping, and feedforward demand from the
actual decoded physical command and measured joint position and velocity. A
violation suppresses the complete vector without consuming the pending target
or advancing reference history. Over-limit requested or decoded feedforward is
rejected before changing the target sequence watermark.

Decoded applied gains are separate from admissible encoding inputs: physical
`Kd=6` decodes slightly above 6. Re-encoding that decoded gain would fail the
strict input bound. Physical codec caps remain unchanged.

Measured effort above the scalar limit latches an attributed persistent fault,
clears pending/history, and revokes ownership. Healthy feedback, Stop, and
reconnect preserve it. Recovery requires explicit acknowledgment, fresh healthy
Stop feedback below the limit, and a new arm request. The terminal release gate
checks state, source, history, feedback, ownership, expiry, freshness, rates, and
lead after the command calculations.

## Parent verification

[Selected results and source hashes](verification.json) record the exact scope.
The standard-library suite passes all 130 tests: 108 inherited tests with explicit
effort/velocity inputs, plus 22 effort, fault, and release regressions. AST
comparison confirmed that the seven inherited test modules changed only to add
those required inputs; their 108 test identities and original assertions remain.

```bash
python -m unittest discover -s tests/control -t .
PATH="$PWD/.venv/bin:$PATH" scripts/check_offline.sh
```

The new [effort tests](../../tests/control/test_controller_effort.py) cover both
directions, reductions 1/3/6, zero offsets, decoded gain quantization, strict
adjacent-float thresholds, signed damping cancellation, combined demand,
feedforward rejection, distant paced goals, overflow, vector atomicity, fault
recovery, and changes during final encoding. Release mutation cases cover
expiry, stale feedback, changed observations, Stop, disconnect, target
replacement, takeover, source/history identity, and a reentrant clock Stop.

A separate parent oracle checked 5,400 seeded candidates using the decoded
output-unit PD equation and signed reduction, independently of the controller's
prediction helper. All 2,075 issued candidates stayed within the cap; 3,325 were
suppressed. A direct public-API probe also verified persistent measured-effort
faults through Stop and reconnect, followed by explicit recovery.

The configured Gemini reviewer inspected all five current controller modules and
reported no actionable defects. Its completed receipt used pinned-argv model
evidence; the parent confirmed matching source hashes. The reviewer ran no tests.
Parent runtime checks provide the evidence above.

## Remaining integration

This is physical-codec software verification, not a physical torque, thermal,
or continuous-duty rating. The scalar limit has no thermal-duration model.
Suppression can leave a goal pending without progress; no gain clipping or new
trajectory solver is introduced. Full C14-C16 and SA04 remain open.

The assumed rolling-arm MuJoCo position servos need a separate representation:
their gains are 180/420 and damping 18/38, with zero desired velocity and no
feedforward. A 200-state loaded-model design probe matched servo force within
1.5e-14 Nm and preserved generalized effort through the rolling mapping.
`mjDSBL_ACTUATION` produced zero actuator force in a Stop probe; this is not a hold
or gravity compensation. These probes guide the next unit and do not establish
a simulator station, policy-delay behavior, or task success through this core.

The [native-drive regressions](../../tests/test_rolling_drive_semantics.py) now
make those model checks repeatable in simulator CI. Four tests pass, including
200 seeded states (188 saturated, 12 unsaturated), actuator position/velocity
mapping, a wrong desired-velocity negative, and explicit disable/enable. The
combined pinned contract/SDK/drive suite passes all 41 tests. This vectorized
seeded matrix is separate from the earlier design probe above.

```bash
.venv-rolling/bin/python -m unittest tests.test_rolling_drive_semantics
```

No motor port, hosted request, training, upload, or physical motion was used.
The [initial C12 port record](../controller-core/README.md) remains historical;
its source-equivalence claim predates these intentional effort changes.
