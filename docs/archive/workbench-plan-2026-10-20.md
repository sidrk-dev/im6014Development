# IM6014 Robot Workbench Agent Prompt

Policy route selected, 2026-10-20. Use [Build the single-arm Dream workflow](docs/DREAMLABS_MILESTONES.md) for one custom arm, one overhead camera, joint-action demonstrations, hosted Dream fine-tuning, and a custom control loop. P01-P16 and T01-T12 below now describe that route. The vendor two-arm YAM example, local GPU training, and EEF integration are optional alternatives. Existing controller and physical acceptance gates still apply; historical results retain their original meaning.

This handoff contains 195 small milestones synchronized with the construction of a custom IM6014 arm. The original 124 software milestones and five RC-series milestones cover work that can be developed while only J1 works or final CAD is being defined. The remaining 66 cover physical commissioning and task validation as the arm is assembled. RC00 records only the completed illustrative rolling-arm experiment; RC01-RC04 track the final rolling-contact shoulder/elbow integration.

## Project objective

Build a robotics workbench alongside my custom IM6014 arm.

The workbench must let me configure the robot, inspect its state, teleoperate, record demonstrations, inspect datasets, fine-tune Dream on single-arm joint-action demonstrations from one overhead camera, configure hosted inference, evaluate policies, take over manually, and collect corrections.

I currently have **only the base joint, J1, functional**. Establish whether that means an actuator on a bench, an assembled base mechanism, or an installed and calibrated base. Treat these as separate physical readiness levels.

Maximize useful software development at every arm build stage. Hardware that has not arrived or has not been assembled must block only the tests that actually need it.

Implement and verify one small behavior at a time.

Develop separate physical and simulation configurations.

- The actual physical arm configuration, initially one functioning joint.
- The selected simulated build, starting with the three-moving-coordinate prototype URDF. Keep the two-coordinate rolling sample, any nominal six-joint model, and the final rolling-contact design as distinct profiles.

Keep their identities and datasets separate. A working one-joint recorder proves the recording system. A successful simulation proves only the selected simulated workflow. Physical scooping needs its own validation once the arm is assembled. The [rolling-contact and Dream integration reference](docs/ROLLING_CONTACT_DREAM_INTEGRATION.md) defines the RC-series boundary.

## Working rules

1. Begin by inspecting the current repository and its instructions. Preserve existing working bench tools and user changes.
2. Use the existing bench and keyboard/gamepad workflow first. Defer MakerMods Lab integration until the basic learned task works.
3. Use one shared custom-arm adapter for teleoperation, recording, and Dream inference. Export the pinned Dream-compatible LeRobot dataset format; add a LeRobot robot package only where an integration needs it.
4. Reuse TRLC DK1 patterns selectively for robot integration, background control, and gravity compensation. Its motor transport is different from mine.
5. Reuse MakerMods' agent harness interfaces where useful. Its controller must communicate through our existing arm controller.
6. Keep a single process responsible for physical motor communication.
7. Build the selected single-arm simulation early, with one overhead policy camera and explicitly provisional geometry where needed. Use actual independent coordinates; passive rolling hinges do not add policy controls.
8. Keep simulation, physical measurements, and generated fixtures explicitly identified.
9. Use hosted Dream joint-action fine-tuning for one arm and one overhead camera. Collect and validate demonstrations before training; verify service acceptance and real inference.
10. Implement one policy backend first. Keep local VLA training, OpenPI, and GR00T outside the first task unless Dream has a demonstrated blocking limitation.
11. Keep the Dream service client, local station, camera capture, and optional language-model agent independently configurable. No local training GPU is required by this route.
12. Start with keyboard or gamepad teleoperation. I do not necessarily own a leader arm.
13. Follow the operator's current Pstack model configuration. Do not copy stale provider/model choices into another routing sheet or silently substitute a model.
14. The parent agent must review subagent changes and independently run the relevant verification.
15. Never report a test as passed merely because a subagent says it passed.
16. Complete software tests when hardware is unavailable, then mark physical verification pending.
17. Complete protocol fixtures when cloud access is unavailable, then mark real model inference or training pending.
18. Do not implement multiple unrelated features before testing the first one.
19. Use screenshots for dashboard behavior, rendered images for simulation behavior, and plots or decoded traces for control and recording behavior.
20. Inspect those images. Saving them alone is insufficient.

## Repository reuse

Inspect these repositories at their current revisions and record the commit used.

| Repository | Intended use | Compatibility work |
|---|---|---|
| [My IM6014 development repository](https://github.com/sidrk-dev/im6014Development) | Existing protocol, serial communication, bench session, diagnostics, tests | Audit its current behavior before extending it |
| [MakerMods Lab](https://github.com/makermods-robotics/makermodslab) | Dashboard, robot records, recording, dataset inspection, training jobs, deployment controls | Register the custom arm and adapt every motion and shutdown path |
| [TRLC DK1](https://github.com/robot-learning-co/trlc-dk1) | LeRobot plugin structure and selected control patterns | Replace transport, dimensions, geometry, calibration, gains, and stop behavior |
| [Metal arm harness](https://github.com/makermods-robotics/metal-arm-harness) | Agent observation and bounded motion tools | Convert degrees explicitly and use our controller as its backend |
| [LeRobot](https://github.com/huggingface/lerobot) | Robot interface, datasets, policies, training | Pin one compatible implementation |
| [OpenPI](https://github.com/Physical-Intelligence/openpi) | Alternative π₀.₅ implementation if justified | Separate environment and explicit data adapter |
| [Isaac GR00T](https://github.com/NVIDIA/Isaac-GR00T) | Alternative model if justified | Custom embodiment, processor, dataset, and serving integration |

Do not assume that working code for another arm has validated my custom arm.

MakerMods Lab has an extensible arm registry and existing recording, training, and deployment workflows. The release reviewed for this plan specifically supported SO101 single-arm remote inference. Verify the selected revision and implement the custom IM6014 integration where necessary. Inspect its shutdown and calibration paths for compatibility with gravity-loaded joints.

LeRobot supports custom robot and teleoperator packages. MakerMods' harness uses degrees, and its simple simulator follows commands perfectly. Use that simulator for interface checks and MuJoCo for dynamics testing. Verify these implementation details against the pinned revisions.

## Known hardware and provisional assumptions

| Item | Current information | Treatment |
|---|---|---|
| Functional physical joints | J1 only | Confirm its mounting and calibration |
| Intended arm | Six rotational joints | Verify actual axis definitions |
| Motors | Unitree IM6014 | Verify protocol against current code and manufacturer material |
| Actuator mass | Previously specified as 535 g each | Confirm and assign each mass to its physical mounting body |
| Intended reach | Approximately 0.7 m | Provisional simulation value |
| Intended upper arm and forearm | Approximately 0.35 m each | Replace with measured dimensions |
| Internal reduction | Previously recorded as 38/3 | Verify and apply once in the codec |
| External shoulder reduction | Previously proposed as 6 to 1 | Provisional until physically confirmed |
| External elbow reduction | Previously proposed as 3 to 1 | Provisional until physically confirmed |
| Other external reductions | Previously proposed as 1 to 1 | Verify individually |
| Communication | RS485 | Confirm adapter, channel, baud, IDs, and topology |
| Previous bench setup | ID0, 6 Mbps, FT4232H Channel D | Historical information to verify |
| Supply | Previously intended as 36 V | Confirm actual hardware |
| Initial tool | Fixed tool, with scoop-specific work later | Independent arm dimensions follow the selected build; no gripper channel |
| Future gripper | Optional | Separate backend and dataset configuration |
| Development laptop | Previously specified as HP Envy 16 with RTX4060 | Detect actual OS, RAM, GPU, and VRAM |

Do not use the earlier 10 Nm continuous torque assumption as a validated actuator rating.

## Build identities

Create versioned profiles for these configurations.

| Build identity | Physical configuration |
|---|---|
| `bench_j1` | One actuator or base joint under bench verification |
| `arm_j1` | Mounted and calibrated base |
| `arm_j1_j2` | Base and shoulder with the actual attached link |
| `arm_j1_j2_j3` | Base, shoulder, elbow, and actual links |
| `arm_j1_j2_j3_j4` | First wrist joint added |
| `arm_j1_j2_j3_j4_j5` | Second wrist joint added |
| `arm_j1_j2_j3_j4_j5_j6` | Complete six-joint arm |
| `arm_6_tool` | Complete arm with measured tool and camera arrangement |
| `sim_6_nominal` | Provisional six-joint simulation |
| `sim_6_measured` | Simulation updated from assembled hardware |
| `sim_prototype_3` | Existing CAD-derived prototype with three moving coordinates; physics validation pending |
| `sim_rolling_sample` | Illustrative shoulder/elbow rolling mechanism with two logical bends; locally verified only |
| `sim_final_rolling` | Separate final CAD-based shoulder/elbow mechanism; independent coordinate count and geometry pending RC01 |

Each physical profile must distinguish joints that are absent, installed but uncommissioned, calibrated, and permitted to move.

Record physically installed passive or locked joints separately. Their bodies still contribute mass even when their motors are not active.

An absent joint must never appear as a fresh measured physical joint.

## Required configuration fields

Implement these as validated configuration data rather than scattered constants.

| Configuration | Required fields |
|---|---|
| Robot identity | Robot ID, build ID, backend, revision, active joint names |
| Joint identity | Joint name, motor ID, bus, physical actuator identity |
| Joint mapping | Direction, zero offset, external reduction, encoder convention |
| Limits | Position interval, velocity, acceleration, command lead, effort budget |
| Controller | Verified gain semantics, gains, reference rate, feedback deadline |
| Fault behavior | Command expiry, feedback expiry, hold eligibility, park procedure |
| Geometry | Parent body, joint origin, axis, link dimensions |
| Dynamics | Body mass, center of mass, inertia, actuator mounting body |
| Tool | Mount transform, mass, center of mass, collision geometry |
| Camera | Stable identity, resolution, frame rate, orientation, timestamps |
| Dataset | Build revision, joint order, action meaning, sample rate, camera keys |
| Policy | Model, checkpoint, processor, dimensions, units, timing, normalization |
| Cloud | Provider, endpoint, credential reference, timeout, resource limits |
| Agent | Provider, model, tools, action budget, observation requirements |

Every uncertain value must carry its source and verification status.

## Canonical state and action contracts

Use radians internally for joint position, radians per second for velocity, and joint-side Nm for effort where the underlying semantics are verified.

Maintain a fixed joint order for each build.

Every state snapshot must include

- Build and configuration revision.
- Joint names and measured positions.
- Measured velocities where available.
- Reported effort with its verified meaning.
- Per-joint receive timestamps and feedback age.
- Per-joint validity and fault status.
- Snapshot time and joint sampling skew.
- Current controller state and motion owner.
- Camera frame identifiers and capture or receive timestamps.

Every command must include

- Build and configuration revision.
- Joint names and explicit action representation.
- Target values and units.
- Command source.
- Sequence number.
- Control ownership generation.
- Creation time and expiry.
- Relevant observation identifier for policy commands.

Maintain four separate records

1. Requested action.
2. Accepted action after validation and limiting.
3. Reference actually dispatched to the actuator controller.
4. Measured resulting state.

Do not substitute measured positions for action labels without explicitly defining and validating that learning objective.

## Partial arm rules

For `arm_j1`, the physical state and action dimensions are one.

For `sim_6_nominal`, they are six.

Do not pad absent physical joints with zero and describe that as a measured six-joint robot.

A display may show a full simulated arm driven by the measured base angle, but its other joint values must remain clearly identified as simulated or assumed. That display must not silently produce six-joint physical training data.

A six-joint policy must not be deployed onto J1 by automatically discarding its other outputs.

## Reduction mapping

After verifying the conventions, implement the ideal external transmission mapping consistently.

\[
q_{\mathrm{out}} = q_0 + sNq_{\mathrm{joint}}
\]

\[
\dot q_{\mathrm{out}} = sN\dot q_{\mathrm{joint}}
\]

\[
\tau_{\mathrm{out}} = \frac{s\tau_{\mathrm{joint}}}{N}
\]

For compatible stiffness and damping definitions, their ideal output-side coefficients scale with \(1/N^2\).

Keep internal motor reduction conversion inside the protocol boundary. Apply external transmission conversion once outside it.

Record friction, backlash, compliance, and efficiency as measured properties later.

## Control architecture

Use one station process to own the physical bus and the actuator refresh loop.

The dashboard, recorder, teleoperator, cloud policy client, and agent tools submit requests to that station. They must not each open the serial port.

Separate these rates

- Actuator communication and reference refresh.
- Teleoperation input processing.
- Dataset sampling.
- Camera capture.
- Dashboard updates.
- Cloud inference.

Measure the achievable actuator rate. Do not copy a 250 Hz configuration from another robot and claim the RS485 implementation achieves it.

A reasonable initial plumbing configuration is a 10 Hz dataset and policy action grid, with faster camera capture where available. Treat that as an initial experiment. Pin the actual timing in each dataset and checkpoint.

Use bounded latest-target storage for streaming commands. Do not accumulate an unlimited FIFO of old teleoperation targets.

Human takeover must revoke automated ownership, clear pending automated actions, and reject delayed responses from the earlier ownership generation.

## Stop behavior

Implement distinct operations for

- Stop the current motion.
- Hold position when feedback and communication permit it.
- Move to a verified supported park position.
- Disable actuator torque.
- Respond to a latched fault.

Their effects depend on the physical build.

Torque-off is suitable for some unloaded bench situations. A gravity-loaded shoulder needs a verified supported shutdown procedure.

Do not claim that a software hold can preserve the arm through loss of power, lost communication, or a firmware torque-disable watchdog.

Audit calibration, startup, connect, disconnect, exception cleanup, browser abandonment, replay completion, and cloud failure paths. All must use the build-specific procedure.

Do not describe a diagnostic as read-only if it sends Stop or other control packets.

## Milestone evidence

For every milestone, record

- What changed.
- Files changed.
- Verification command or operator procedure.
- Expected and actual result.
- Relevant image, plot, or trace.
- Parent review result.
- Commit or equivalent revision.
- Remaining physical or cloud dependency.

Use separate statuses for

- Software verified.
- Physical verification pending.
- Physical verified.
- Cloud verification pending.
- Real cloud verified.
- Policy performance pending.
- Policy performance measured.

A transport fixture must never be reported as a real VLA result.

## Software milestones while only J1 works

The following software milestones should be developed while only J1 works. Prioritize useful J1 operation early, then keep advancing the selected custom-arm simulation and learning workflow while the next mechanical stage is being built.

### A01–A10 establish the actual starting point

| ID | Implement | Pass test |
|---|---|---|
| A01 | Inspect repository instructions and working tree | Record current revision and existing changes without overwriting them |
| A02 | Identify the existing working J1 entry point | Locate the actual command, configuration, and dependencies |
| A03 | Run existing hardware-independent tests | Save real output and identify failures separately from missing hardware |
| A04 | Audit startup, scan, Stop, and shutdown behavior | Document every operation that transmits a control packet |
| A05 | Detect the implementation environment | Record OS, Python, serial support, GPU, RAM, and VRAM |
| A06 | Pin the selected dependency revisions | A clean environment resolves the same package versions |
| A07 | Define repository boundaries | Each component has one purpose and a documented dependency direction |
| A08 | Create the milestone ledger | Passed, failed, and pending work survive restarting the agent |
| A09 | Verify subagent availability | Record exact working model identifiers and parent review workflow |
| A10 | Create a baseline dashboard launch | App loads without opening the motor port or energizing hardware |

### B01–B14 isolate and verify the IM6014 communication boundary

Keep the existing working bench client available during this work.

| ID | Implement | Pass test |
|---|---|---|
| B01 | Extract or wrap the existing packet encoder | Existing verified command fixtures remain identical |
| B02 | Verify the packet decoder | A captured valid packet reproduces the expected fields |
| B03 | Verify checksum handling | Corrupt payload and checksum bytes are rejected |
| B04 | Verify packet lengths and framing | Short, oversized, and unrelated frames never become valid feedback |
| B05 | Build a fake serial transport | It records writes and returns controllable fragmented responses |
| B06 | Handle fragmented input | A response arriving one byte at a time is decoded correctly |
| B07 | Handle noise and resynchronization | Noise before a valid frame does not corrupt the valid result |
| B08 | Enforce motor identity | Feedback from another motor does not refresh the requested motor |
| B09 | Enforce transaction deadlines | A missing response returns within the configured bounded deadline |
| B10 | Preserve motor-output units | Encoder position and velocity conversions match independently checked fixtures |
| B11 | Implement external joint mapping | Ratios 1, 3, and 6 round-trip correctly with both directions and nonzero offsets |
| B12 | Validate numerical inputs | NaN, infinity, invalid IDs, and out-of-range encoded values are rejected |
| B13 | Define explicit port configuration | Wrong adapter channel or unsupported baud produces a useful diagnostic |
| B14 | Add diagnostic capture | Raw packets and decoded fields can be compared without exposing credentials |

### C01–C16 establish the shared controller

These tests initially use the fake transport. Their physical counterparts are commissioned later.

| ID | Implement | Pass test |
|---|---|---|
| C01 | Enforce one bus owner | A second controller cannot independently open the same configured hardware |
| C02 | Implement controller states | Illegal transitions produce no motor command |
| C03 | Add per-joint feedback age | Fresh J1 feedback cannot conceal stale J2 feedback |
| C04 | Add per-joint error status | One motor fault identifies that motor and affects the appropriate build |
| C05 | Add bounded target storage | Ten thousand updates leave bounded storage and the newest valid target |
| C06 | Add command sequence validation | Duplicate and older commands cannot replace a newer target |
| C07 | Add command expiry | Expired commands are rejected before dispatch |
| C08 | Add ownership generations | A delayed command from a previous owner is rejected |
| C09 | Add joint position limits | Boundary values behave correctly and out-of-range requests follow documented handling |
| C10 | Add velocity limiting | A target jump cannot exceed the configured reference velocity |
| C11 | Add acceleration limiting | Reference velocity changes remain within the configured acceleration limit |
| C12 | Add measured-position lead limits | Excessive separation between measured and requested position is bounded |
| C13 | Add effort prediction and monitoring | Combined stiffness, damping, and feedforward effort is considered |
| C14 | Add latched faults | A new target cannot silently clear a temperature or persistent effort fault |
| C15 | Add timing measurements | Report actual loop period, transaction latency, overruns, and feedback age |
| C16 | Test station independence | Dashboard failure cannot create a second owner or automatically rearm the robot |

Predicted effort limits are an additional control check. Do not report them as independently certified actuator protection.

### D01–D06 track arm build progress in software

| ID | Implement | Pass test |
|---|---|---|
| D01 | Create the versioned build profile schema | Duplicate motor IDs, invalid reductions, and missing required mappings are rejected |
| D02 | Create `bench_j1` and `arm_j1` profiles | Their mechanical readiness and allowed actions remain distinct |
| D03 | Create provisional later build profiles | Unknown measurements stay explicitly unverified |
| D04 | Add absent and locked joint handling | Absent joints cannot be commanded, while installed locked bodies retain their mass |
| D05 | Bind calibration to build identity | A calibration for another revision is rejected |
| D06 | Invalidate affected validation after changes | A changed reduction, tool, or mounting transform marks relevant tests pending |

### E01–E10 adapt MakerMods Lab as the operator interface

Inspect the selected version's arm registration and session mechanisms. Extend those mechanisms rather than creating a parallel dashboard.

| ID | Implement | Pass test |
|---|---|---|
| E01 | Register the custom arm family | The dashboard recognizes the arm without falling back to SO101 |
| E02 | Display the physical build stage | The page visibly distinguishes J1 hardware from six-joint simulation |
| E03 | Display joint readiness | Absent, uncommissioned, calibrated, and movable joints are distinct |
| E04 | Display connection state | An open port alone is not shown as fresh motor feedback |
| E05 | Display measured and requested positions | The two values remain visibly distinct during tracking error |
| E06 | Display feedback age and faults | Stale state loses its live indication and shows the affected joint |
| E07 | Add explicit controller operations | Motion stop, hold, park, and torque disable have distinct effects |
| E08 | Add build configuration editing | Invalid changes fail validation and accepted changes create a revision |
| E09 | Integrate session ownership | Concurrent motion sessions are refused by the backend |
| E10 | Verify desktop and phone layouts | State, faults, and stop controls remain readable without overlapping |

Visually inspect screenshots for E02–E10. Numerical values and status colors must match the backend.

### F01–F08 establish camera capture immediately

Use one fixed overhead RGB camera as the policy input. Register its stable identity and verify that the tool and target remain visible. A wrist camera is optional future work requiring a new contract and dataset revision.

| ID | Implement | Pass test |
|---|---|---|
| F01 | Enumerate cameras | Available devices are listed without assuming camera index zero is correct |
| F02 | Store stable camera identity | Reconnecting a device does not silently swap its camera role |
| F03 | Capture timestamped frames | Frames carry increasing identifiers and usable station-side timing |
| F04 | Configure image orientation | A visible asymmetric marker confirms rotation, mirroring, and color order |
| F05 | Add bounded frame storage | A slow viewer cannot create an unlimited frame backlog |
| F06 | Display live video | Screen recording confirms actual changing frames rather than a cached image |
| F07 | Handle camera failure | A disconnected camera becomes unavailable rather than repeating a frame as live |
| F08 | Synchronize frames with joint snapshots | Measured frame age and joint sampling skew are included in observations |

Capture a visible timing event to assess synchronization. A shared software timestamp alone does not prove simultaneous camera exposure and motor sampling.

### G01–G12 implement useful J1 teleoperation and recording

The same components must work with the six-joint simulation.

| ID | Implement | Pass test |
|---|---|---|
| G01 | Add small joint jog requests | A jog references the calibrated current state and obeys limits |
| G02 | Add keyboard teleoperation | Press, release, lost focus, and deadman release behave predictably |
| G03 | Add gamepad support if a device exists | Neutral input, dead zone, reconnect, and unplug are handled |
| G04 | Integrate input on a controlled timebase | Browser key-repeat frequency does not determine joint speed |
| G05 | Start a recording episode | Episode identity and build metadata are written before samples |
| G06 | Record measured joint state | Saved state agrees with controller telemetry at the sampling times |
| G07 | Record requested, accepted, and dispatched actions | A limited request remains distinguishable from what was actually sent |
| G08 | Record synchronized video | Playback aligns a visible motion with the associated joint trace |
| G09 | Finalize an episode | Video, metadata, and sample counts reopen correctly |
| G10 | Recover an interrupted recording | Partial data is clearly marked and does not appear as a complete demonstration |
| G11 | Add an episode viewer | Scrubbing updates video and joint plots to the same episode time |
| G12 | Add annotations and exclusion | Success, failure, takeover, and exclusion survive reopening |

Do not turn all J1 commissioning motions into training demonstrations automatically. Preserve commissioning logs separately and select demonstrations deliberately.

### S01–S12 build the selected custom-arm simulation

Use provisional geometry when necessary and display that status. Avoid spending time on detailed food physics before the workbench works.

| ID | Implement | Pass test |
|---|---|---|
| S01 | Import the prototype URDF and keep later profiles separate | Prototype joint order and geometry match the retained CAD asset; nominal six-joint and final rolling-joint models have distinct provenance |
| S02 | Verify joint and rolling-mechanism transforms | Prototype axes and final coupled rolling poses match CAD or independent geometry across their intended travel |
| S03 | Add actuator and link mass ownership | Total mass matches the component ledger without counting a motor twice |
| S04 | Add simulated impedance behavior | The arm exhibits tracking error under load rather than teleporting to targets |
| S05 | Run simulation through the controller contract | The same command validation tests pass for fake, simulated, and physical adapters |
| S06 | Add one simulated overhead policy camera | The overhead role, crop, orientation, and image dimensions match the selected single-camera contract |
| S07 | Implement forward kinematics for independent coordinates | Link/tool poses include any passive rolling coupling and match independent reference calculations within frozen tolerance |
| S08 | Verify the Jacobian | Finite-difference motion agrees with Jacobian predictions |
| S09 | Add Cartesian target preview | The UI shows a proposed pose without executing motion |
| S10 | Add inverse kinematics | Reachable targets have bounded error and unreachable targets fail explicitly |
| S11 | Add a simple visual task | The simulated tool reaches a visible target region with a frozen success rule |
| S12 | Record simulated demonstrations | Teleop, recording, playback, and dataset inspection work for six joints |

Use at least two diagnostic viewing angles when checking geometry. Those evaluator-only views do not become policy cameras. The policy receives only its registered overhead stream.

The first task should use the fixed tool, such as reaching a visible marker or following a short path. Do not choose grasping as the mandatory first task unless a working gripper is part of the build.

### P01-P16 implement the single-arm Dream inference boundary

Use one custom arm, one overhead camera, measured joint state, and a joint-action fine-tune. No left-arm telemetry, absent gripper, base-model zero-shot claim, or EEF/IK prerequisite belongs in this route. The own control loop submits targets through the shared controller; it does not use the stock bimanual Dream station as a single-arm driver.

| ID | Implement | Pass test |
|---|---|---|
| P01 | Select the Dream joint-action route | Pin client, dataset format, actual service/base model, and single-arm/one-camera body; verify account access separately |
| P02 | Freeze the policy manifest | Missing model revision, independent joint order, zero/sign conventions, units, overhead role, history, absolute-target meaning, or timing is rejected; passive rolling coordinates are not extra controls |
| P03 | Build the one-camera observation adapter | Overhead image history, measured joint positions, optional actual gripper, task, and capture times match the registered model |
| P04 | Decode the fine-tune's own named action parts | Known per_key fixtures become intended absolute joint-radian targets without assuming YAM chunk layout or inventing another arm |
| P05 | Register credential references | Endpoint/key references persist without secrets in source, datasets, logs, or evidence |
| P06 | Separate service readiness checks | Transport, account/body access, dataset processing, published model, and actual inference have distinct results |
| P07 | Exchange a recorded observation with a fixture | Local observation and ownership generations remain associated with the matching response; no unsupported server echo field is assumed |
| P08 | Reject invalid responses | Wrong dimensions, nonfinite values, wrong units, extra parts, or incompatible model/build never dispatch |
| P09 | Verify hosted model publication | Record fine_tuned_model, base/service identity, dataset revision, and the returned observation/action contract; no local GPU load is required |
| P10 | Run actual inference without motion | A held-out recorded one-arm observation produces decoded targets, timing, model identity, and billed-call evidence |
| P11 | Bound action chunks and their age | Chunk rows, storage, execution prefix, action grid, and discard policy are explicit and tested |
| P12 | Execute through the controller in custom simulation | Joint mapping and timing hold while physics continues during delayed inference |
| P13 | Handle old and reordered replies | Obsolete observations and owner generations cannot replace current control |
| P14 | Implement human takeover | Pending chunks and late replies cannot restart automated execution |
| P15 | Handle camera, network, and service failures | Stale frames, timeouts, exhausted credit, authentication errors, and faults follow the build-specific stop policy while local control remains responsive |
| P16 | Report latency and actual costs | Record preparation, wall round trip, available provider timing, observation age, execution delay, and billed calls; unknown provider timings remain unknown |

Use station monotonic time for local deadlines. Do not compare monotonic clocks from different machines directly. Start with a bounded execution prefix and one inference request at a time. Camera capture and the local controller continue while inference waits; stopping does not wait for a network response. Preserve all C-series validation and physical gates.

The vendor two-arm YAM simulator, EEF inference, and local OpenPI deployment remain optional experiments. They do not block the selected joint-action route or substitute for custom-arm evidence.

### T01-T12 implement demonstration-based hosted fine-tuning

Training is required for this selected route. First establish the pipeline using the custom single-arm simulation and one overhead camera. Later repeat it on separately identified real-arm demonstrations before claiming physical task performance. Keep held-out evaluation episodes outside training uploads.

| ID | Implement | Pass test |
|---|---|---|
| T01 | Export the pinned Dream-compatible LeRobot dataset | Reader and SDK checks open overhead video, measured joints, absolute joint targets, tasks, and episode metadata |
| T02 | Verify samples and synchronization | Invalid values, missing frames, incorrect action alignment, retimed timestamps, and incomplete episodes are detected; inspected replay confirms alignment |
| T03 | Freeze provenance and dataset revision | Hashes retain simulation/physical identity, build/tool/camera revision, units, timing, outcomes, and correction labels |
| T04 | Split complete episodes | Train/validation/evaluation episodes do not overlap; independent evaluation cases are excluded from hosted training data |
| T05 | Verify model data transformations | State/action units, absolute targets, service normalization, and constant dimensions are documented; no local pi0.5 statistics or duplicate normalization is applied |
| T06 | Record a hosted job configuration and cost | Exact body, dataset, account access, actual base-model identity, allowed job count, up-front price, and authorized funding are recorded before submission |
| T07 | Verify hosted dataset processing | Register/upload actual one-arm/overhead data, inspect processed shape and timing, and keep inaccessible alignment evidence explicitly unverified |
| T08 | Run one bounded hosted joint-action job | Record submitted parameters, job ID, actual terminal result and logs; use_eef is false where exposed, and no local optimizer/GPU check is substituted |
| T09 | Verify the published model | A usable fine_tuned_model and its matching body/observation/action contract are recorded; real no-motion inference remains a separate gate |
| T10 | Recover without duplicate jobs | Persist job identity and poll existing work; a timeout never automatically submits another paid job or promises optimizer resume/cancellation the service does not support |
| T11 | Perform a small adaptation diagnostic | A tiny reviewed dataset and inspected model predictions verify plumbing; training loss and overfitting do not establish task competence |
| T12 | Evaluate held-out closed-loop behavior | Execute the trained policy on new reachable scenes and target shifts; report actual outcomes, interventions, latency, faults, and all saved trials |

Begin with a bounded data/job experiment, not an assumed demonstration count guarantee. Verify up-front job cost and funding because priced training may not be cancellable. The current inspected API does not expose an arbitrary starting-checkpoint argument; do not assume the physical model resumes from the simulation fine-tune.

When credentials, funding, hosted processing, or model access are unavailable, finish offline data and fixture checks and leave real training/inference pending. A published model or declining loss does not establish useful robot behavior.

### RC00-RC04 document and integrate rolling-contact joints

Use the [rolling-contact and Dream integration reference](docs/ROLLING_CONTACT_DREAM_INTEGRATION.md) and [canonical ledger](docs/dreamlabs_milestones.tsv) for evidence and dependencies. These are a separate final-arm branch. They do not block the prototype path or replace existing controller, hosted-model, or physical acceptance gates.

| ID | Behavior | Verification |
|---|---|---|
| RC00 | Verify the illustrative rolling arm and local Dream boundary | Recorded MJCF/mimic-URDF geometry, loaded dynamics, constraint-loss controls, actual SDK with mocked transport, and dataset preflight pass; status is locally verified only |
| RC01 | Define final shoulder/elbow geometry and independent coordinates | Version CAD, rolling profiles/radii, centers, axes, reference poses, travel, drive relationships, mass/inertia/collision provenance, and remaining calibration |
| RC02 | Build and validate the final rolling-aware model | Independent pose/Jacobian sweeps, passive coupling, collision/mass ownership, timestep sensitivity, and load checks pass frozen tolerances against final geometry |
| RC03 | Integrate the final mechanism and drive mapping locally | Shared-controller simulated/fake checks cover position, velocity, torque virtual work, gains/effort limits, feedback estimation, and final release; passive hinges are never extra independent actions |
| RC04 | Freeze the final-arm Dream contract and dataset revision | One-arm joint-action config, actual coordinate order/units/timing, one overhead camera, final-model demonstrations, replay, dataset preflight, and hashes pass before SA07-SA10 hosted work |

RC01 depends on RC00. RC02 depends on RC01. RC03 depends on RC02 and SA04. RC04 depends on RC03, SA01, and SA06. The ledger's `final_profile_requires` column additionally requires RC03 before final-profile SA05 demonstrations and RC04 before final-profile SA07 uploads. The 90-frame sample recording remains a format fixture. Final hardware calibration and policy trials retain SA11-SA14 and the H-series gates.

### R01–R08 add the optional direct-agent experiment

Begin after the conventional workflow works.

| ID | Implement | Pass test |
|---|---|---|
| R01 | Expose fresh observation tools | Returned images and measured state identify their age and source |
| R02 | Expose bounded joint-motion tools | Oversized, invalid, and unavailable-joint requests are rejected |
| R03 | Expose Cartesian preview tools | Proposed poses are checked before execution |
| R04 | Adapt the MakerMods harness | Degree-to-radian conversion is explicit and no second bus owner appears |
| R05 | Run an observe-act-observe agent task in simulation | Every move has a bounded command and an inspected subsequent observation |
| R06 | Save a validated skill and trace | The skill records build, assumptions, parameters, and actual verification |
| R07 | Invoke the selected Dream policy as an optional agent tool | The agent uses the same single-arm policy adapter and shared controller |
| R08 | Compare the joint policy and optional agent modes | Run matched held-out one-arm tasks and report outcomes, interventions, timing, and model costs |

Keep simulator ground truth available to the evaluator where necessary, but do not silently give it to an image-based policy or agent.

Agent mode remains optional. Its failure must not block teleoperation, demonstrations, or conventional VLA deployment.

## Milestones synchronized with physical assembly

Implement their supporting software in advance whenever possible. Perform their physical tests only when the corresponding mechanism exists.

### H1 covers the mounted base joint

If the current J1 already meets these requirements, capture the evidence and proceed. Do not rebuild working hardware merely to follow the numbering.

| ID | Implement or measure | Pass test |
|---|---|---|
| H1.01 | Record the base mounting and transmission | Photos and measurements identify motor, output axis, reduction, and cable path |
| H1.02 | Verify the configured motor identity | Communication addresses the intended J1 and reports the expected actuator |
| H1.03 | Calibrate direction and zero | A small positive move matches the documented physical direction |
| H1.04 | Establish the allowed base interval | Cable travel and mechanism limits are represented without assuming unlimited rotation |
| H1.05 | Verify bounded base motion | Small jogs and a slow return track the target within a predeclared tolerance |
| H1.06 | Complete the J1 operator workflow | Connect, inspect, teleop, record, replay inspection, stop, and restart all work |

For H1.05, declare the test amplitude and tolerance before executing it. Select them from the actual fixture and measurements, rather than inventing universal values.

At this point, the real J1 should be a complete workbench device.

The operator should be able to see live feedback, teleoperate, record synchronized demonstrations, inspect episodes, and compare requested actions with actual tracking.

The six-joint simulation should already exercise the full learning workflow independently.

### H2 covers J2 and the attached upper-arm mechanism

This is the first major gravity-loaded stage. Adding the shoulder changes startup, dynamics, controller gains, and shutdown behavior.

| ID | Implement or measure | Pass test |
|---|---|---|
| H2.01 | Verify J2 alone in its supported commissioning fixture | Identity and basic communication work without relying on J1 |
| H2.02 | Extend the physical profile to two active joints | Commands and observations expose exactly J1 and J2 |
| H2.03 | Verify two-motor transaction routing | Wrong-ID feedback cannot update either joint incorrectly |
| H2.04 | Measure two-joint bus timing | Capture ordinary latency and the effect of one missing response |
| H2.05 | Measure shoulder geometry and actual external reduction | Joint origin, axis, ratio, direction, and zero have recorded evidence |
| H2.06 | Measure the attached body mass and center of mass | The ledger includes the actual link, brackets, wiring, and attached actuator bodies |
| H2.07 | Build the partial-arm gravity model | Its component ledger matches the physical two-joint assembly |
| H2.08 | Verify gravity calculation independently | A pendulum reference and potential-energy gradient confirm sign and magnitude |
| H2.09 | Establish supported startup and shutdown | No reused cleanup path unexpectedly removes support from the loaded link |
| H2.10 | Commission low-amplitude shoulder tracking | Declared tracking, effort, and thermal criteria are measured |
| H2.11 | Add bounded gravity feedforward | Supported pose tests improve holding behavior without violating the configured envelope |
| H2.12 | Record and inspect two-joint teleoperation | Actual state, issued references, video, and build metadata remain aligned |

For gravity calculation, use the actual partial assembly.

MuJoCo exposes bias forces separately from constraint forces. Verify the chosen calculation with zero velocity and an isolated robot dynamics model so contact constraints are not mistaken for gravity compensation. Consult the [MuJoCo data definitions](https://mujoco.readthedocs.io/en/stable/APIreference/APItypes.html).

Do not use a full future-arm gravity model for the current two-joint assembly.

Conversely, if future wrist actuators or brackets are already mechanically mounted, include their mass even when they are not electrically commissioned.

### H3 covers the elbow and forearm

This stage often enables useful tool-point positioning, depending on the actual axes. Do not assume it provides arbitrary tool orientation.

| ID | Implement or measure | Pass test |
|---|---|---|
| H3.01 | Verify J3 in its commissioning fixture | Correct identity, communication, and feedback are established |
| H3.02 | Extend the physical profile to three active joints | Exactly the commissioned joint set is exposed |
| H3.03 | Measure elbow axis, zero, and external reduction | Small positive motion agrees with the model |
| H3.04 | Update forearm and attached actuator masses | Each installed component appears once on its correct body |
| H3.05 | Measure three-joint feedback age and skew | A stalled joint does not produce a falsely coherent fresh snapshot |
| H3.06 | Verify physical forward kinematics | Several measured poses agree with predicted tool or marker positions |
| H3.07 | Determine controllable Cartesian directions | Jacobian rank and physical motion support the advertised controls |
| H3.08 | Commission small Cartesian moves | Reachable commands track and unreachable requests are rejected |
| H3.09 | Update gravity and tracking validation | Shoulder and elbow pass the new load-dependent pose tests |
| H3.10 | Record a simple three-joint task | A marker-reaching or short-path task has synchronized episodes and measured outcomes |

For H3.06, use measurable reference points or a calibrated camera. Visual resemblance alone is insufficient.

Revalidate J2 after adding the forearm and elbow actuator. Its load has changed even if its software has not.

### H4 covers the first wrist joint, J4

| ID | Implement or measure | Pass test |
|---|---|---|
| H4.01 | Commission J4 identity and feedback | Correct motor and decoder behavior are verified |
| H4.02 | Update joint mapping and four-joint profile | Joint order remains explicit across controller, dashboard, and recordings |
| H4.03 | Add the actual J4 mounting body | Geometry and mass agree with the assembled mechanism |
| H4.04 | Verify its physical orientation effect | A small move rotates the tool frame around the documented axis |
| H4.05 | Revalidate shoulder and elbow load behavior | Added distal mass is reflected in gravity and effort measurements |
| H4.06 | Record four-joint teleoperation | Tool orientation changes and preceding-joint behavior are visible in playback |

### H5 covers the second wrist joint, J5

| ID | Implement or measure | Pass test |
|---|---|---|
| H5.01 | Commission J5 identity and feedback | Correct physical actuator responds |
| H5.02 | Extend the five-joint profile | Missing J6 remains absent and unavailable |
| H5.03 | Measure J5 axis and transform | Rendered and physical orientation changes agree |
| H5.04 | Determine available pose control | The advertised position and orientation controls match Jacobian rank |
| H5.05 | Revalidate combined load and bus timing | Added actuator mass and communication do not invalidate earlier assumptions |
| H5.06 | Record orientation-sensitive movements | Playback preserves joint order, camera alignment, and tool pose |

Do not advertise full six-dimensional pose control simply because five joints can move.

### H6 covers the completed six-joint arm

The workbench should already exist. This stage replaces provisional assumptions and validates full physical operation.

| ID | Implement or measure | Pass test |
|---|---|---|
| H6.01 | Commission J6 identity and feedback | Correct physical actuator responds |
| H6.02 | Freeze the full joint order and conventions | Controller, dataset, model adapter, and dashboard agree |
| H6.03 | Complete six-joint calibration | Every joint has verified direction, zero, reduction, and limits |
| H6.04 | Create the measured six-joint simulation revision | Geometry and component ledger reflect the assembled arm |
| H6.05 | Measure full-arm bus performance | Report achieved rate, latency distribution, sampling skew, and missing-joint behavior |
| H6.06 | Revalidate full-arm dynamics | Static pose tests cover representative shoulder and elbow loads |
| H6.07 | Verify complete forward kinematics | Independent pose measurements meet predeclared tolerances |
| H6.08 | Verify bounded full-pose inverse kinematics | Reachable poses work and singular or impossible requests fail clearly |
| H6.09 | Validate all control transitions physically | Teleop, hold, park, takeover, and restart follow the commissioned procedures |
| H6.10 | Complete a six-joint recording session | The dataset contains actual six-joint measurements and issued action references |

Keep `sim_6_nominal` available for reproducibility. Do not silently overwrite it with the measured model.

Earlier simulated demonstrations may remain useful for debugging or planned pretraining experiments. Their provenance must remain intact.

### H7 covers the scoop, final cameras, and workspace

Camera work should already be functional. This stage updates the actual mounted views and tool geometry.

| ID | Implement or measure | Pass test |
|---|---|---|
| H7.01 | Measure scoop mass and center of mass | Tool identity and dynamics are saved in the build revision |
| H7.02 | Calibrate the tool-center transform | Several measured poses agree with the predicted scoop reference point |
| H7.03 | Model complete tool collision geometry | Guards account for the scoop body rather than only one tip point |
| H7.04 | Mount and register the overhead policy camera | Its role, mounting, crop, orientation, and actual useful view remain stable |
| H7.05 | Calibrate metric workspace references | Known distances confirm camera or marker-based measurements |
| H7.06 | Register bins, bowl locations, and allowed regions | Stored geometry agrees with measurements and visible images |
| H7.07 | Verify empty-tool transfer motions | The scoop moves between locations without unintended contact |
| H7.08 | Freeze the food-task observation and action contract | Final cameras, task labels, timing, joint order, and tool revision are recorded |

A workspace guard must account for the full arm and tool.

For later scooping, the tool may intentionally enter a bin. Define that allowed contact region explicitly. A global table-clearance rule alone does not describe the task.

### H8 covers physical demonstrations, fine-tuning, and task evaluation

| ID | Implement or measure | Pass test |
|---|---|---|
| H8.01 | Define the first physical task and success rule | Initial conditions, timeout, acceptable outcome, and intervention counting are frozen |
| H8.02 | Collect a small physical demonstration batch | Episodes are reviewed visually and invalid recordings excluded |
| H8.03 | Inspect action and tracking distributions | Clipping, saturation, delay, and repeated failures are identified |
| H8.04 | Fine-tune using the frozen physical dataset | Training uses the exact physical contract and a recorded model configuration |
| H8.05 | Run physical-data inference without actuation | Output dimensions, timing, normalization, and proposed trajectories are checked |
| H8.06 | Run bounded physical policy trials | Each trial records task outcome, tracking, latency, faults, and intervention |
| H8.07 | Collect manual corrections | Takeover timing and human-controlled segments remain explicitly labeled |
| H8.08 | Evaluate the revised policy on held-out trials | Compare task success and interventions using the same frozen evaluation procedure |

The first learned physical task is fixed-tool reaching to a visible marker, using the one overhead camera. Object pushing follows after that gate; empty-scoop transfer and food manipulation remain later tasks.

Move to a simple scoopable material only after the empty-tool task is reliable. Food portion control then requires an actual measurement method such as a scale. Visual appearance alone does not establish portion accuracy.

Simulation success does not complete H8.

## Required cross-cutting verification

Run these checks at the milestone where they become relevant. Reuse the same tests across backends where appropriate.

| Area | Required cases |
|---|---|
| Protocol | Valid feedback, bad checksum, fragmented input, noise, wrong motor ID, missing response |
| Mapping | Both directions, nonzero zero offset, all configured external reductions, round-trip conversion |
| Numerical validation | NaN, infinity, wrong dimension, missing joint, extra joint, wrong units |
| Profiles | Wrong build, stale calibration, changed reduction, changed tool, absent joint |
| Controller | Position, velocity, acceleration, command lead, combined effort, stale feedback |
| Faults | Persistent fault, attempted implicit reset, communication loss, temperature threshold |
| Ownership | Two motion clients, expired owner, human takeover, delayed automated reply |
| Timing | Slow bus response, camera stall, encoder backlog, dashboard load, cloud timeout |
| Recording | Interrupted episode, missing frame, mismatched timestamps, wrong action labels |
| Dataset | Episode leakage, incorrect normalization, incompatible joint order, wrong camera keys |
| Policy | Processor mismatch, incompatible checkpoint, malformed chunk, old observation, wrong timing |
| Training | One batch, optimizer step, save and reload, exact resume, tiny overfit, held-out evaluation |
| Simulation | Wrong joint axis, incorrect mass ownership, reachable and unreachable targets, collisions |
| Physical model | Measured pose agreement, gravity sign, distal load changes, backlash, tool transform |
| User interface | Stale-state indication, backend rejection, actual takeover state, readable phone view |

Use fake clocks and deterministic fixtures for deadline and ordering tests. Avoid long sleeps in the test suite.

For visual verification, inspect at least

- A dashboard showing the actual physical build and inactive joints.
- A dashboard showing the six-joint simulation.
- Joint target and measured-state plots during tracking.
- A stale-feedback and a latched-fault screen.
- An episode with synchronized video and joint plots.
- A simulation view showing each joint axis in isolation.
- A Cartesian target preview.
- A policy rollout that succeeds.
- A policy rollout that fails.
- A takeover that interrupts automated motion.
- The phone layout.

For each image, write a short observation of what it demonstrates and any visible discrepancy.

Do not approve a trajectory solely from its endpoint image. Inspect the path or video.

## Physical commissioning record

For each newly added joint, collect

- Actuator identity and motor ID.
- Wiring and bus topology.
- Mounting orientation and photographs.
- External reduction and direction.
- Zero procedure and result.
- Allowed position interval.
- Measured tracking error.
- Communication latency and feedback age.
- Relevant effort and temperature traces.
- Attached masses and mounting locations.
- Startup, supported park, and shutdown results.

Revalidate preceding joints whenever a new link, actuator, camera, or tool changes their load.

## Dataset compatibility

Keep commissioning logs, task demonstrations, policy evaluations, and manual corrections distinguishable.

Every dataset revision must identify

- Physical or simulated source.
- Build revision.
- Joint order and action dimensions.
- Tool configuration.
- Camera configuration.
- Action representation.
- Sampling rate.
- Label selection from requested, accepted, or dispatched actions.
- Episode selection.
- Normalization statistics.
- Operator or generated-data source.

Do not merge data from incompatible build stages automatically.

If cross-configuration training is later desired, implement an explicit embodiment adapter and a separate experiment. Verify that the selected model actually supports the intended masking or conditioning.

## Parallel work and pacing

Use subagents for bounded independent work such as

- Protocol edge-case tests.
- Build-profile validation.
- Camera capture.
- Episode inspection.
- Simulation geometry.
- Policy processor integration.
- Training configuration.
- Agent tool adaptation.

Assign clear file ownership. Avoid concurrent edits to the controller core, model adapter, or shared schemas.

For each milestone

1. Inspect the relevant current implementation.
2. Define the behavior and pass test.
3. Implement the smallest coherent change.
4. Run the relevant check.
5. Inspect visual evidence when applicable.
6. Have the parent review the result.
7. Save the result in the ledger.
8. Proceed to the next milestone.

If a milestone grows into several independent behaviors, split it before continuing.

Run a broader regression check when crossing a build gate or changing a shared contract. Do not rerun unrelated expensive training after every dashboard edit.

## Suggested execution order today

Start with A01–A10.

Then implement B01–B14, C01–C16, and D01–D06.

Build E01–E10 and complete whichever H1 tests the current physical setup permits.

Add F01–F08 and G01–G12 so J1 becomes useful for real teleoperation and synchronized data collection.

Continue S01–S12, P01–P16, and T01–T12 while the shoulder is being assembled.

Keep RC00's sample evidence separate. Advance RC01-RC04 as final rolling geometry becomes available; apply their additional prerequisites to final-profile demonstrations and uploads without blocking the prototype route.

Add R01–R08 after the conventional workflow works.

As each physical joint becomes ready, execute its H-stage acceptance tests, update the measured model, and continue collecting compatible physical data.

Do not wait for the completed arm to build training and cloud inference.

## Completion criteria for the current one-joint stage

The current stage is complete when

- J1 has a verified profile and calibration appropriate to its mounting.
- The dashboard displays fresh measured state and real controller status.
- Teleoperation follows the configured motion limits.
- Recording and playback preserve synchronized state, actions, and video.
- The operator can distinguish hold, park, and torque disable.
- The selected custom-arm simulator uses the same controller and workbench interfaces.
- Dataset export and model preprocessing are verified.
- Cloud transport and failure tests are verified.
- Real inference and training have passed where suitable compute and access exist.
- Pending physical and cloud tests are explicitly identified.

Report learned task performance separately from workbench completion.

## Final handoff

Provide

- The working code and reproducible launch instructions.
- The pinned dependency and repository revisions.
- The milestone ledger.
- Completed and pending physical acceptance tests.
- Actual test output.
- Reviewed screenshots, recordings, and plots.
- Build profiles and calibration procedures.
- Dataset revisions and compatibility rules.
- Model and cloud configurations.
- Measured communication and inference timing.
- A short operator walkthrough.
- The exact measurements needed for the next physical build stage.

Do not conclude with a vague list of features. Demonstrate the workflow that actually runs.

## Primary reference material

Consult the actual checked-out code and pinned documentation before implementing an integration. Links below preserve the reference material used to prepare this plan and are not evidence that a particular hardware test has passed.

- [Custom LeRobot hardware integration](https://huggingface.co/docs/lerobot/main/en/integrate_hardware)
- [LeRobot π₀.₅ policy documentation](https://huggingface.co/docs/lerobot/main/en/pi05)
- [MakerMods Lab implementation guidance](https://github.com/makermods-robotics/makermodslab/blob/main/CLAUDE.md)
- [MakerMods Lab remote inference implementation](https://github.com/makermods-robotics/makermodslab/blob/main/makermodslab/remote_inference.py)
- [MakerMods agent harness](https://github.com/makermods-robotics/metal-arm-harness)
- [TRLC DK1](https://github.com/robot-learning-co/trlc-dk1)
- [OpenPI](https://github.com/Physical-Intelligence/openpi)
- [MuJoCo data definitions](https://mujoco.readthedocs.io/en/stable/APIreference/APItypes.html)
