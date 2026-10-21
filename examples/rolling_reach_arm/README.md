# sim_rolling_reach_v1

A self-contained MuJoCo reaching scene built by composing the public
`examples/rolling_contact_arm/model.py` `mjcf()` and `urdf()` ElementTrees. The
rolling geometry is unchanged; the new scene adds a vertical yaw mount, a table,
a red tabletop marker, an overhead RGB camera, and a payload.

This is a model and vision/reference-control unit only. It is **not** a trained
Dream policy, a hardware result, a teleoperation recording, or final-CAD
calibration. The hand-coded image-feedback policy is a baseline for exercising
the simulator and observations, not a learned-task acceptance claim.

## Run

Create the environment once with Python 3.14 and the existing pinned requirements:

```bash
python3.14 -m venv .venv-rolling
.venv-rolling/bin/python -m pip install -r examples/rolling_contact_arm/requirements.txt
```

From the repository root, using the prepared pinned environment:

```bash
MUJOCO_GL=egl .venv-rolling/bin/python examples/rolling_reach_arm/verify.py
MUJOCO_GL=egl .venv-rolling/bin/python examples/rolling_reach_arm/verify.py --render
MUJOCO_GL=egl .venv-rolling/bin/python examples/rolling_reach_arm/reach.py --episodes 4 --seed 0
```

No new dependency is required. `reach.py --output` and all generated reports,
model XML/URDF, images, and GIFs are confined to the ignored
`examples/rolling_reach_arm/results/` directory. The checks need a working
offscreen renderer because the reference policy consumes real rendered images.

## Files

- `scene.py` — the single scenario/config owner, effective manifest derivation,
  native/URDF composition, analytic FK, analytic Jacobian, coordinate mapping,
  camera projection/backprojection, and red-marker extraction.
- `reach.py` — simulation owner, acceleration/speed-limited governor, RGB-only
  reference reacher, episode/evaluation loop, and CLI.
- `verify.py` — independent model, importer, detection, control, and reference
  reaching checks.
- `results/` — ignored runtime artifacts (`verification.json`,
  `probes_frozen.json`, model files, previews/GIFs).

## Mechanism and coordinates

The three logical coordinates are `[base_yaw, shoulder_bend, elbow_bend]` and
the five internal hinges are `[yaw, shoulder_a, shoulder_b, elbow_a, elbow_b]`.
Each equal-radius pair has center spacing `2r`, `b = a`, and logical bend
`a + b`. The inherited mechanics are mounted through `Rz(yaw) Rx(+pi/2)` at a
0.35 m shoulder height. Thus old local XY becomes world XZ, old local +Z becomes
world -Y at zero yaw, and world gravity is `(0, 0, -9.81)`.

The actuators are exactly `[yaw, shoulder_bend, elbow_bend]`. The bend
actuators remain full-bend tendons (`a+b`); they are not half-angle carrier
commands. The yaw actuator drives the yaw hinge directly. The mapping and
virtual-work identities (`S G = I`, `G.T actuator_moment.T = I`) are checked.

The inherited assumptions are 0.32/0.28 m links, 0.05/0.04 m equal rolling
radii, 0.7/0.4 kg links, and 0.025 kg carriers with the inherited COM/inertias.
The scene adds a 0.25 kg yaw hub with a solid-cylinder inertia and a 0.10 kg
spherical payload with a solid-sphere inertia. The optional 0.05 kg proximal
wheel was not added: the inherited wheel geoms are massless and the carriers own
the modeled rotating mass, so there is no double counting. These values are
simulation assumptions, not physical calibration.

## Task and policy boundary

Instruction: **"Move the tool above the red marker."** The virtual goal is the
marker XY at 0.040 m above the tabletop. An attempt succeeds only when the tool
site center stays within 0.020 m of that 3D goal continuously for 1 s before a
20 s timeout. Every attempt is reported; failures are not dropped.

The reference policy receives only rendered `uint8[240,320,3]` RGB, measured
logical joints, the fixed task text, and a timestamp. It knows the fixed camera
calibration and fixed goal height, but it does not receive marker/target
coordinates, an environment/model accessor, or evaluator state. Every camera
frame it filters red pixels, keeps substantial connected components, fits the
known projected marker radius to the visible outer boundary (a plain centroid is
biased when the arm occludes the marker), backprojects to the tabletop, adds the
goal height, and sends bounded absolute logical targets through analytic
FK/Jacobian damped least squares. Missing or occluded markers hold the last
target.

The evaluator separately uses ground-truth marker and tool-site positions. The
scene marker is non-colliding; the table, yaw hub, and non-excluded nonadjacent
self-collisions are active. Adjacent rolling-wheel exclusions remain body-wide
for the ideal constrained contact, so the verification audits those exclusions
and tests nonadjacent geometric clearance rather than claiming every possible
self-contact was certified.

## Seeds and declared cases

The seed camera is at `(0.35, 0, 1.2)` m, fovy 55 deg, 320x240 RGB. The home pose
`[0, 0, -0.9]` rad is non-singular and clear of the table. The marker radius is
0.060 m so the tool can occlude its center while the outer arc remains
detectable. The default marker and the four probe cases plus two marker-shift
cases are declared in `scene.py`; `verify.py` writes
`results/probes_frozen.json` before running any success evaluation. No case was
excluded from the reported run. `--seed` changes the order/subset of declared
probes; it does not generate randomized held-out scenes.

## Checks

`verify.py` covers:

- 5 internal coordinates, 3 actuators, 2 equalities, actuator order, gravity,
  camera pose/axes, timing decimators, limits, force caps, and scenario ranges.
- Independent analytic FK and Jacobians against MuJoCo and finite differences
  over 567 bounded combined poses.
- Fixed rolling-center spacing and material contact-point velocity (zero slip in
  the ideal constraint), plus mapping round trips and virtual work.
- Positive-definite, triangle-admissible inertias and exact body mass ownership,
  including native/URDF total-mass agreement.
- Table/hub collision activity, nonadjacent clearance, marker non-collision, and
  the explicit tool site.
- Projection/backprojection for positive and behind-camera points, parallel-ray
  rejection, image row direction, actual red-marker extraction with arm
  occlusion, absent-marker rejection, and safe hold on a missing marker.
- Acceleration- and speed-limited commands, loaded stability with zero solver
  warnings, force/velocity bounds, and intentional removed/reversed equality
  controls under antagonistic disturbances.
- URDF import with `yourdfpy` and MuJoCo: 3 independent coordinates across 5
  hinges and 2 mimics, FK, mass totals, and loaded behavior. URDF contact
  geometry is box/cylinder-based and its fixed-joint/body structure differs from
  the native capsule model; the report labels those representation differences.
- All declared probe attempts, live marker shifts, and frozen-image/hold
  negative controls. Live shifts change commands and recover; frozen-image and
  no-update controls fail to adapt, as expected.

`--render` additionally saves actual camera images and short motion GIFs under
`results/render/`. `results/verification.json` is the selected report and
distinguishes model checks, reference image-reaching, URDF import, and the
explicitly absent dataset/SDK/learned/hosted/physical evidence.

## Limits of this reference baseline

The task text is a fixed scenario constant; this baseline does not interpret
language. The red-circle fit is tested for the visible arcs in these declared
scenes, not severe or arbitrary occlusions. Missing detections hold the previous
target. Small discrete governor oscillations near a target are possible; SA04
must supply the reviewed shared controller before demonstrations or policy use.
The exported URDF retains a zero-inertia fixed wheel link accepted by the tested
MuJoCo/yourdfpy path. Compatibility with other dynamics engines is unverified.
