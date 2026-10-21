# Rolling-contact mechanism model

The current CAD arm is a three-moving-coordinate prototype. The final shoulder and elbow will use rolling-contact joints. The [sample](../examples/rolling_contact_arm/README.md), prototype, and final model have distinct identities and datasets. Final geometry and drive relationships remain undefined.

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

## Final-model acceptance

RC01 freezes final CAD, profiles/radii, centers/contact frames, axes, signs/zeros, travel, independent coordinates, drive relationships, and mass/inertia/collision provenance. RC02 compares full poses across limits and combined configurations against CAD or independent geometry, checks finite-difference Jacobians/coupling, and evaluates load behavior and timestep sensitivity. Assign every mass once.

RC03 integrates the drive mapping into the shared controller; RC04 freezes the corresponding Dream profile and data revision. The [ledger](milestones.tsv) owns dependencies and evidence. Prototype work continues without final CAD.

The [sample reports](../reports/rolling-contact/README.md) state actual numerical limits. MuJoCo documents [joint equality](https://mujoco.readthedocs.io/en/stable/XMLreference.html#equality-joint) and its [URDF workflow](https://mujoco.readthedocs.io/en/stable/modeling.html#urdf-extensions). The [general rolling-contact paper](https://arxiv.org/html/2409.00616v1) covers geometry-specific surfaces.
