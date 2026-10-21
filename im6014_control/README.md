# im6014_control

Transport-free joint-controller core, based on the reviewed C12 source. The `JointCore` lifecycle, ownership,
and fault state machine is thread-confined over frozen domain records; mapping,
reference planning, and codec functions are pure. This package imports no
serial transport and never opens a port.

## Public imports

The package intentionally has no broad re-exports or facade classes. Import the
module that owns the API you need:

```python
from im6014_control.core import JointCore
from im6014_control.records import JointSpec, PositionLimits, Target, State
from im6014_control.mapping import JointMap, JointSample, JointTarget
from im6014_control.reference import plan_reference, project_joint_target
from im6014_control.codec import command, decode_command, raw_to_position, raw_to_rpm
```

## Migration source

Migrated from committed C12 source at
`https://github.com/sidrk-dev/im6014-arm-control`, commit
`227ec4ecc712c0cee48a50c9e234e70dd065b8e3`. See `provenance.json` for the
source-to-destination map and original source/test SHA256 values.

The initial C12 port is recorded separately from subsequent effort changes.
The required `JointSpec.max_effort_nm` bounds the combined decoded stiffness,
damping, and feedforward demand at measured position and velocity. The planner
suppresses the complete vector if any joint exceeds its limit. It preserves
admissible encoding inputs separately from decoded gain values and performs a
post-computation release check before committing reference history.

Measured effort above the same scalar limit latches an attributed fault and
revokes ownership. Healthy packets, Stop, and reconnect do not clear it. Recovery
requires acknowledgment, fresh healthy Stop feedback, and a new explicit arm.
This scalar is not a continuous-duty or thermal rating.

[Effort verification](../reports/controller-effort/README.md) describes the
software checks. C14-C16 acceptance and SA04 rolling-simulator station integration
remain open. No physical drive or thermal qualification follows from these
checks; the package adds no transport owner.

Accepted C12 behavior is preserved, including one-shot reference consumption,
codec quantization and gain/torque caps, and release-time vetoes. The [native drive unit](../reports/native-drive/README.md) adds a closed drive
variant and loaded-model factory for fixed MuJoCo position servos. Native targets
require the declared gains, zero desired velocity/feedforward, and an explicit
logical identity map. A future station must still enforce model/camera identity
and measure multi-rate behavior before integration.

## Tests

From the repository root, using only the standard library:

```bash
python -m unittest discover -s tests/control -t .
```

This runs the 108 migrated C12 regression tests plus effort, fault, and release-barrier regressions against this package. Legacy fixtures use explicit wide test caps to keep their original gates active; applications must supply their own justified limits.
