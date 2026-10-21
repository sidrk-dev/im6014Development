# Rolling reaching simulator verification

Selected profile: `sim_rolling_reach_v1`, an independently designed, assumed
single arm with base yaw and rolling shoulder/elbow joints. No private CAD or
hardware is needed. [Run instructions](../../examples/rolling_reach_arm/README.md).

The [parent review](parent-review.md) records review findings and gate scope.

The parent reran `MUJOCO_GL=egl .venv-rolling/bin/python
examples/rolling_reach_arm/verify.py --render` on 2026-10-21 with the pinned
simulation environment. The [selected report](verification.json) retains all
attempt outcomes and model metrics; only per-tick samples and duplicate scenario
metadata are omitted. Full results, generated XML/URDF and motion GIFs remain in
the ignored example results directory.

- 567 combined poses passed independent FK, Jacobian, rolling-point velocity,
  coordinate-map and virtual-work checks.
- Native and imported URDF have five hinges, two couplings and three independent
  coordinates; mass ownership totals 1.5 kg. URDF collision primitives differ
  from native MJCF, so this is not contact-equivalence evidence.
- Loaded stability passed with zero solver warnings. Removed/reversed rolling
  constraints failed the coupling check as intended.
- Four rendered-camera reference probes and two live marker shifts succeeded
  within the 20 mm / one second / 20 second rule. Four frozen-camera or
  held-command controls failed to adapt. These declared development probes are
  not an untouched learned-policy evaluation set.

The reference policy is hand-coded and receives rendered RGB and measured
logical joints with known camera calibration. This report accepts the assumed
simulation model only. Shared-controller integration, synchronized demonstrations,
selected-profile SDK validation, hosted training, learned behavior, and physical
operation remain separate milestones. Ideal no-slip constraints do not simulate
traction limits, compliance, backlash, or the final motor transmission.

## Selected-profile SDK boundary

The [SA02 review](sa02-review.md) records parent checks and review triage.
The [SA02 report](sa02-client.json) records the parent-run pinned SDK check for
this selected arm. Real rendered overhead JPEGs and measured logical joints pass
through real SDK serialization, while credentials and transport are mocked.
There are 44 layered negative checks and 37 parent-run contract/import/SDK unit
tests. Additional parent regressions caught numeric coercion, camera/limit drift,
and a redirected results directory before the fixes. Policy rows are 30 Hz;
local control remains 100 Hz. A fake checkpoint proves no hosted model identity,
training outcome, learned behavior, or SA04 control readiness.
