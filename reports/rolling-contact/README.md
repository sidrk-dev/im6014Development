# Rolling-contact sample results

Checked with MuJoCo 3.16.0, yourdfpy 0.0.60, Dream client 1.0.3, and Python 3.14.6 on Linux. Reproduce using the [sample instructions](../../examples/rolling_contact_arm/README.md). Generated results remain local; these JSON files are selected snapshots.

| Report | Evidence |
| --- | --- |
| [Mechanism](verification.json) | 625 poses per case, equal/unequal radii, FK/Jacobians/contact velocities, native/imported loaded dynamics, and broken-constraint controls. |
| [SDK](dream_contract.json) | Real SDK validation, one camera/two logical values, mocked transport, and targets applied in simulation. |
| [Dataset](dataset_validation.json) | 90-frame 30 Hz LeRobot v2.1 fixture passes local upload preflight; wrong action width is rejected. |

Both dynamic models ran eight seconds and 24,000 steps, checking metrics every step. Peak coupling error was 0.000118 rad; transient material slip velocity was 0.002014 m/s; tool error against ideal geometry at measured bends was below 0.00001 m. Target tracking was separately 0.07383 rad. Solver warnings were zero.

Constraints model ideal rolling, not traction or load capacity. Targets came from fixtures, not learning. Dimensions, inertias, and transmission are illustrative. Hosted learning, final CAD, and hardware remain unverified. The [independent review](../research/dreamlabs/2026-10-20-rolling-sample-independent-review.md) includes the reporting correction and timestep check.
