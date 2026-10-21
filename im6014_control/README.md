# im6014_control

Transport-free C12 joint-controller core. The `JointCore` lifecycle, ownership,
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

This package contains the committed C12 pure controller closure only. C13 effort
prediction, C14 fault acceptance, C15 timing, C16 station independence, and SA04
rolling-simulator station integration are **not accepted**. The source review
evidence is software-only; this is not a physically verified or hardware-ready
station, and no second bench/transport owner is added. The old bench launcher is
untouched.

Accepted C12 behavior is preserved, including one-shot reference consumption,
codec quantization and gain/torque caps, and release-time vetoes. A future station
must define a reviewed backend/drive/gain relation and measure multi-rate
behavior before integration.

## Tests

From the repository root, using only the standard library:

```bash
python -m unittest discover -s tests/control -t .
```

This runs the 108 migrated C12 regression tests against this package.
