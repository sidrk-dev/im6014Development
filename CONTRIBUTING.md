# Develop the workbench

Start from the repository root. Install bench dependencies in `.venv` as described in the README. This application uses a small checkout-runnable package; an editable install is unnecessary.

## Run the offline baseline

```bash
source .venv/bin/activate
scripts/check_offline.sh
```

This checks the portable controller regressions and effort checks, packet encoding, fake serial behavior, Qt lifecycle/UI behavior, CLI help, the frozen single-arm contract and its mutation regressions, roadmap consistency, and active documentation links. It performs no physical motor communication. Screenshots and the UI summary go to `results/bench/`.

Use a separate environment for the [rolling sample](examples/rolling_contact_arm/README.md). Its checks exercise MuJoCo, the actual Dream SDK with mocked transport, and local dataset upload validation. They make no hosted calls and require no API key.

Validate the frozen contract with the pinned client:

```bash
MUJOCO_GL=egl .venv-rolling-sample/bin/python scripts/check_sa02_client.py
MUJOCO_GL=egl .venv-rolling-sample/bin/python examples/rolling_contact_arm/verify_dream.py
```

This checks the three-wide joint-action schema, one overhead camera, and no-EEF registration. Transport is mocked; no cloud or motor call occurs.

To check the external prototype import for SA03, supply the frozen local assets:

```bash
.venv-rolling-sample/bin/python -m unittest tests.test_single_arm_contract tests.test_sa03_prototype
.venv-rolling-sample/bin/python scripts/check_sa03_prototype.py \
  --prototype-dir .worktrees/controller-c07-c10/arm_assets \
  --output-dir results/single-arm/sa03-prototype
```

The checker verifies asset hashes, preserves fixed links and visuals, and compares MuJoCo link poses with independent URDF forward kinematics. Generated imports and reports stay under ignored `results/`. A pass covers provenance and kinematics only. For that earlier CAD profile, placeholder inertias, missing collisions, and unfinished tool, camera, and marker definitions remain unverified. The selected assumed simulator has separate SA03 acceptance below. The offline baseline skips the simulator-dependent unit cases when that separate environment is absent.

## Run the selected hardware-free reaching scene

Use the same pinned simulation requirements in a separate environment:

```bash
python3.14 -m venv .venv-rolling
.venv-rolling/bin/python -m pip install -r examples/rolling_contact_arm/requirements.txt
MUJOCO_GL=egl .venv-rolling/bin/python examples/rolling_reach_arm/verify.py --render
MUJOCO_GL=egl .venv-rolling/bin/python examples/rolling_reach_arm/verify_dream.py
.venv-rolling/bin/python -m unittest tests.test_rolling_drive_semantics tests.test_rolling_native_drive tests.test_rolling_station
MUJOCO_GL=egl .venv-rolling/bin/python examples/rolling_reach_arm/reach.py --episodes 4 --seed 0
```

Use `MUJOCO_GL=osmesa` with libOSMesa on machines without EGL. These checks need no hardware, private CAD, API key, or hosted call. They verify a hand-coded RGB reference baseline and the installed SDK with mocked transport. The [example guide](examples/rolling_reach_arm/README.md) documents assumptions and the generated manifest. Earlier prototype checks above retain their own evidence; they do not block this assumed simulator.

## Portable controller checks

```bash
python -m unittest discover -s tests/control -t .
```

This standard-library suite imports `im6014_control` directly. It preserves reviewed C12 behavior and checks scalar decoded-command effort, measured faults, and terminal release vetoes. It does not connect the simulator station, accept remaining timing/station gates, or open a motor port. See the [port record](reports/controller-core/README.md).

## Place changes with their owner

`im6014_bench/motor.py` owns packet units, encoding, feedback checks, and the bounded bench CLI. `session.py` owns serial access and lifecycle transitions. `dashboard.py` owns Qt presentation. Preserve one serial owner and disconnected/disarmed startup.

The rolling example is an experiment, not the shared arm controller. New production integration must reuse the reviewed contracts described in [architecture](docs/architecture.md). Keep camera, coordinates, model identity, timing, and target semantics consistent across demonstrations and inference.

## Update the roadmap once

Edit `docs/milestones.tsv`, then run:

```bash
python scripts/check_repository.py --write-roadmap
python scripts/check_repository.py
```

Keep stable IDs. A gate needs parent-run evidence for the declared build/profile before its status changes. Hardware-matched final rolling profiles additionally obey `final_profile_requires`; the selected assumed simulator does not wait for final CAD. GitHub issues are execution work packages linked to those IDs, not a second acceptance ledger.

## Submit a reviewable change

Use a `codex/` branch for agent work. State the problem, actual commands/results, and unresolved gates. Use the supplied templates. Do not include credentials, private CAD, hardware logs, raw datasets, vendor archives, or environments.

CI runs offline checks only. Passing it does not establish motor readiness, hosted model acceptance, or physical task success.
