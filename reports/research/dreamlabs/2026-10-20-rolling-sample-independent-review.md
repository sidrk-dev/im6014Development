# Independent rolling sample review

No open blocking findings in the reviewed implementation. One evidence issue was found and accepted by the parent. The current source contains its fix, and the recorded source hashes match the current files. Root retains final acceptance and the full script reruns.

## Substantive finding identified and corrected

P2. The original `verify.py` lines 159-160 skipped nine simulation steps out of ten before updating every reported maximum. A separate 24,000-step rerun measured peak contact slip of 0.00201387259685416 m/s at step 3. The original recorded maximum was 0.0010889684220483564 m/s. Peak coupling error was 0.00011746500635918198 rad rather than the recorded 0.00011601458999946514 rad. These values still pass the existing 0.005 m/s and 0.001 rad thresholds, but the original report understated the actual peaks. The smallest remedy was to update metrics every simulation step and sample only history frames. The current `verify.py` lines 175-189 do that and explicitly record `metrics_checked_every_step=True`.

## Mathematical and simulator evidence

For external circular rolling, let a be carrier rotation relative to the proximal body, b be distal rotation relative to the carrier, and R and r the proximal and distal radii. The relative material-point tangential velocity at contact is R*a_dot - r*b_dot. Thus b=(R/r)*a and the logical bend q=a+b gives a=q/(1+R/r), b=q*(R/r)/(1+R/r). The implementation has the correct sign and radius ratio for equal and unequal radii. The same derivation remains valid when the proximal body rotates, as at the elbow.

I independently ran the 625-pose equal-radius geometry check and 625-pose unequal-radius check. Position errors were below 6e-16 m, Jacobian error below 1.33e-10, and material contact velocity mismatch below 6.3e-17 m/s in the prescribed ideal states. I inspected the contact calculation at `verify.py` lines 44-60. It evaluates both bodies' material-point Jacobians at the same instantaneous contact point, so it measures the intended relative contact velocity.

I directly imported both generated URDFs with the installed MuJoCo 3.16.0 parser. Both have four internal coordinates and two joint equalities, with zero imported actuators. The equal-radius coefficients are 1 and 1. The unequal-radius coefficients are 1.5 and 1.6666666666666667. The two constraint rows are independent, leaving a two-dimensional ideal configuration manifold. The two logical-bend actuators are subsequently added explicitly.

I independently reproduced both original 8-second native and imported dynamics runs, including gravity, a 2 N external force on the elbow body's center of mass during seconds 3-4, and the prescribed antagonistic hinge torques during seconds 5-5.4. Both had no MuJoCo warnings and reproduced the stored metrics. I separately ran all-step metric collection for the native 24,000-step run and found the reporting discrepancy above. Removing or reversing the equalities produced coupling errors above 1.6 rad, so the negative tests detect real constraint loss.

With the corrected metric implementation, an independent 48,000-step run at half the timestep passed. Peak coupling error was 5.8578492450785086e-05 rad, contact slip 0.0010319214997746608 m/s, tool reference error 4.910479263147739e-06 m, and tracking error 0.07390170892193532 rad. This is a useful numerical sensitivity check, not validation of an exact physical mechanism.

The fixed tendon has q=a+b and produces hinge forces [F,F] for each logical actuator. I independently checked G.T*qfrc_actuator=actuator_force for the logical-to-internal mapping G. I also ran the parent's current `check_actuation`, which measured a virtual-work error of 1.7763568394002505e-15 W. The reported force is the logical generalized torque. It does not establish the torque of a physical motor driving only one internal hinge.

## Dream SDK evidence

I read the installed Dream 1.0.3 implementation in `dream/facade.py`, `dream/client.py`, `dream/transport.py`, and `shared/datasets/embodiments.py`. The two-wide arm registration, explicit single-arm mapping to the trained right-arm slot, joint-action mode with use_eef=False, observation conversion, action conversion, and session semantics match the real SDK.

I independently ran the original and current `verify_dream.run()` with its existing credentials-resolution and Transport.request mocks. I intercepted only final result writes and the existing-directory creation call to preserve the read-only assignment. The SDK itself, JSON encoding/decoding, validation, key mapping, session continuation/reset, camera rendering, and simulator adapter executed. Wrong observation widths and malformed action shapes were rejected by the real SDK. The adapter rejected nonfinite and out-of-range chunks without changing control, position, or simulation time.

The parent corrected the test's originally reused RGB frame. Current `verify_dream.py` lines 88-94 rerender the moved simulator and assert changed pixels before submitting the next observation. I reran that updated test successfully. I also confirmed its new real-SDK negative check. Dream's stock arm-file IK loader rejects `shoulder_b` because the chain contains a mimic joint. This sample's logical joint-action adapter is valid locally, but the URDF is not a drop-in input to stock Dream IK.

All SHA256 entries in the latest `verification.json` and `dream_contract.json` matched current source/model files at the final read. The initial recorded hashes had also matched the initial reviewed snapshot.

## Scope and limits

The sample models ideal external circular rolling with prescribed center spacing and soft equality enforcement. Native dynamics recorded zero collision contacts. It validates neither frictional traction nor contact-force capacity, slip onset, tendon elasticity, backlash, the user's eventual CAD, calibrated inertias, real motor transmission, or hardware. Unequal radii have geometry and direct-import checks, not the same loaded dynamic campaign.

The Dream exercise contains synthetic response rows and an offline checkpoint fixture. It proves the local installed SDK contract and simulator adapter, including fresh images after simulated motion. It does not prove hosted checkpoint availability, authentication, hosted inference, fine-tuning acceptance, learned task success, or transfer to the user's future arm. There were zero hosted calls.

I scanned scoped code comments and docstrings for constraints hiding local defects. I found no substantive comment-related issue. I did not review the parent's unfinished `record_dataset.py`, README, or separate documentation. I made no checkout or Git changes. The only file I wrote was this report.
