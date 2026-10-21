# Parent acceptance review, 2026-10-21

Scope: the assumed `sim_rolling_reach_v1` model and RGB reference-reaching
baseline. The parent ran the simulator, CLI, bench baseline, all 26 pinned
contract/import regressions, and the earlier-profile SDK fixture. There was no
motor access, hosted inference, upload, training, or learned checkpoint.

## Independent source review

The configured Gemini `gemini-3.8-flash-high` read-only lane completed with
exit code 0 and a PASS-with-caveats verdict. The runner pinned the model in its
argv; its receipt does not independently verify the model identity. The reviewer
inspected supplied source and did not execute tests. Local raw review/receipt
artifacts remain under the ignored `.audit/rolling-simulation-2026-10-21/`.

The parent accepted and corrected the tool-offset mismatch: the manifest now
records the actual elbow-local frame `[link_length, 0, lateral_offset]`.
The native payload and URDF tool joint now derive that frame from the scenario.
Regression checks cover both the selected 25 mm lateral offset and a 45 mm
variant, comparing manifest, native model, URDF origin and analytic FK.

The parent also aligned URDF elbow color with native MJCF, included the shared
model factory in source hashes, asserted unchanged held-control commands, and
removed redundant policy state, an unused centroid function, a misleading
pre-shift metric, and duplicate final-error calculation. The audit retained
style-only import-order suppressions; they are not correctness suppressions.

The source review overstated universal failure of zero-inertia fixed links in
other engines. Acceptance is limited to the actual tested MuJoCo/yourdfpy
imports; no broader compatibility conclusion is drawn. Severe-occlusion vision
robustness and governor micro-oscillation remain documented reference-baseline
limits. The marker-fit radius in the actual report is about 11.5 pixels; the
reviewer's approximate 20-pixel statement is not measurement evidence.

## Gate decision

SA03 is simulation verified for the selected assumed arm. Earlier prototype
asset/kinematic evidence remains valid only for its earlier profile. SA01 freezes
the selected simulation model/task constants; SA02 still needs selected-profile
SDK validation. SA04 controller integration, SA05-SA06 synchronized data,
SA07-SA10 hosted/learned behavior, and physical/final-hardware gates remain open.
The current reference governor cannot substitute for the shared controller.

EGL rendering passed locally. The local OSMesa attempt failed at import because
the OSMesa graphics library is unavailable; CI installs libOSMesa and runs the
same verifier. CI status is separate evidence. An initialization regression
introduced while removing redundant state was caught and corrected before the
successful final verification; it is not part of the accepted artifact.
