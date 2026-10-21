# Rolling-contact mechanism model

The selected [reaching simulator](../examples/rolling_reach_arm/README.md), `sim_rolling_reach_v1`, has base yaw and ideal circular rolling shoulder/elbow joints. Its three logical coordinates drive five internal hinges. It uses independently chosen geometry and masses so simulation can proceed before hardware exists. The earlier CAD prototype, two-bend [sample](../examples/rolling_contact_arm/README.md), and eventual hardware-matched model retain separate identities and evidence. Final physical geometry and drive relationships remain undefined.

## Ideal circular pair

`a` rotates the center carrier of an external circular pair; `b` rotates its distal body relative to the carrier. Radii are `rp` and `rd`, center spacing is `rp+rd`. Under the sample's same-axis zero convention:

```text
rp * a_dot = rd * b_dot
q = a + b
a = q / (1 + rp/rd)
b = q * (rp/rd) / (1 + rp/rd)
```

Equal radii give internal angles `q/2`. One independently driven elbow remains one policy coordinate. A fixed-axis hinge generally misses the moving pivot. Noncircular surfaces, offsets, or different mounting require their own relation.

The sample uses 50 mm shoulder radii, 40 mm elbow radii, and 320/280 mm links. These are illustrative, not final CAD or AMBIDEX dimensions. Constraints represent ideal rolling and exclude those surfaces from collision solving. They do not establish frictional traction, slip onset, contact stress, tendon elasticity, backlash, or physical load capacity.

## Formats and actuation

MuJoCo 3.16.0 imports both sample URDF `mimic` joints as equality constraints. Native MJCF and yourdfpy reproduce the same poses. Dream client 1.0.3's own IK parser rejects moving mimic joints. Use the [joint-action route](dream-integration.md) with local mechanics; removing mimic or adding independent action channels changes the mechanism.

The sample's fixed tendon has length `a+b`, so actuator targets are complete logical bends. Do not halve those targets. This generalized transmission is not an actual motor/tendon route. Rolling geometry does not establish motor gearing.

For the actual drive relation `q=g(m)`, use `J=dq/dm`:

```text
q_dot = J * m_dot
tau_m = transpose(J) * tau_q
```

Verify constant ratios where applicable. Nonlinear or coupled drives require the derivative in velocity, effort, gain, feedback-estimation, and release checks. A motor encoder may only estimate link angle through a compliant transmission.

## Selected reaching scene

The 320/280 mm links are mounted vertically at a 350 mm shoulder height with a yaw hub and 100 g tool. The overhead camera supplies 320×240 RGB at 30 Hz; physics runs at 3000 Hz and reference control at 100 Hz. The goal is 40 mm above the tabletop at the visible red marker XY; the tool must stay within 20 mm for one second before a 20 second timeout.

Parent-run checks cover 567 combined poses, loaded dynamics, rolling contact-point velocity, virtual work, URDF mimic import, and actual image-feedback adaptation to marker shifts. The URDF uses boxes/cylinders while native MJCF uses capsules; importer checks do not prove contact-equivalent behavior. The reference reacher is hand-coded and cannot pass learned-policy gates. See the [verification record](../reports/rolling-reach/README.md).

## Final-model acceptance

RC01 freezes final CAD, profiles/radii, centers/contact frames, axes, signs/zeros, travel, independent coordinates, drive relationships, and mass/inertia/collision provenance. RC02 compares full poses across limits and combined configurations against CAD or independent geometry, checks finite-difference Jacobians/coupling, and evaluates load behavior and timestep sensitivity. Assign every mass once.

RC03 integrates the drive mapping into the shared controller; RC04 freezes the corresponding Dream profile and data revision. The [ledger](milestones.tsv) owns dependencies and evidence. The selected assumed simulation proceeds without final CAD; RC gates apply when matching the final physical mechanism.

The [sample reports](../reports/rolling-contact/README.md) state actual numerical limits. MuJoCo documents [joint equality](https://mujoco.readthedocs.io/en/stable/XMLreference.html#equality-joint) and its [URDF workflow](https://mujoco.readthedocs.io/en/stable/modeling.html#urdf-extensions). The [general rolling-contact paper](https://arxiv.org/html/2409.00616v1) covers geometry-specific surfaces.
