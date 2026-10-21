# Native rolling-arm drive representation

The selected simulation has three logical coordinates over five internal
hinges. Its position servos apply fixed stiffness and damping, zero desired
velocity, and zero feedforward. The shared controller must predict that actual
law while retaining its timing, ownership, fault, and kinematic checks.

`JointSpec.drive` selects either the default physical IM6014 codec or a frozen
`NativePositionServo`. Native targets carry the declared gains and require zero
velocity/feedforward fields; incompatible requests are rejected rather than
silently changed. Native coordinates use an explicit logical identity map.
Rolling kinematics and virtual work use the separate scene matrices `S` and `G`.

The example-owned factory reads a loaded MuJoCo model and compares its drive
metadata with the canonical selected scene before constructing native specs.
The pure `im6014_control` package has no MuJoCo dependency. Physical codec caps
remain unchanged; no artificial motor reduction is used to fit native gains.

```python
import mujoco
from examples.rolling_reach_arm import scene
from examples.rolling_reach_arm.drive import joint_specs_from_model

model = mujoco.MjModel.from_xml_string(scene.generated_xml())
specs = joint_specs_from_model(model, max_measured_lead_rad=0.1)
goal = (0.05, 0.04, -0.87)
joints = tuple(spec.drive.target(q) for spec, q in zip(specs, goal))
```

This prepares a command representation. Connection, fresh stopped feedback,
explicit arming, target ownership/expiry, and successful reference release are
still required before a station can enable actuation. Arming seeds actual native
gains and checks the resulting damping effort at measured velocity; it does not
apply a command.

## Verification scope

Run the standard-library controller suite and the pinned simulator checks:

```bash
python -m unittest discover -s tests/control -t .
.venv-rolling/bin/python -m unittest tests.test_rolling_drive_semantics tests.test_rolling_native_drive
```

Parent verification passed 159 controller tests, including all 130 inherited
physical regressions, and 51 pinned simulator/SDK tests. All eight inherited
controller test modules remain byte-identical to the baseline. The full offline
suite also passed, including 18 bench groups.

Repeated actual physics stepping exposed a rounding stall missed by the first
unit checks: only eight references were issued over 1,000 control ticks. Moving
an out-of-corridor clamp one representable float inward restored progress while
preserving strict bounds. The corrected ten-second probe issued 999 references
over 30,000 physics steps and reached the exact requested reference vector.
Measured joints differ from references because fixed native PD exhibits gravity
sag. The connected regression checks rate, acceleration, lead, and effort bounds.
No marker-reaching or learned-policy success follows from this joint-goal probe.

An independent source review of the complete implementation and new tests found
no actionable defects. It executed no runtime tests; the provider receipt pins
the requested Gemini model in argv rather than verifying a returned model ID.
Selected counts, source hashes, and probe measurements are in
[verification.json](verification.json).

## Remaining integration

SA04 stays open for the simulator station and its continued local timing during
policy delay, full model/camera identity, stale-frame/expiry behavior, takeover,
and remaining C14-C16 acceptance. The factory validates the drive relation only;
it does not establish full scene identity or prevent later model mutation.

`mjDSBL_ACTUATION` produces zero actuator force. It is not a hold or gravity
compensation. Scalar effort limits are not thermal or physical drive ratings.
Hosted checkpoints, learned behavior, and hardware outcomes need separate gates.
The earlier [effort report](../controller-effort/README.md) records the preceding
physical-codec software unit and its source snapshot.
