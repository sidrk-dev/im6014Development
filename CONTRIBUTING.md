# Develop the workbench

Start from the repository root. Install bench dependencies in `.venv` as described in the README. This application uses a small checkout-runnable package; an editable install is unnecessary.

## Run the offline baseline

```bash
source .venv/bin/activate
scripts/check_offline.sh
```

This checks packet encoding, fake serial behavior, Qt lifecycle/UI behavior, CLI help, roadmap consistency, and active documentation links. It performs no physical motor communication. Screenshots and the UI summary go to `results/bench/`.

Use a separate environment for the [rolling sample](examples/rolling_contact_arm/README.md). Its checks exercise MuJoCo, the actual Dream SDK with mocked transport, and local dataset upload validation. They make no hosted calls and require no API key.

## Place changes with their owner

`im6014_bench/motor.py` owns packet units, encoding, feedback checks, and the bounded bench CLI. `session.py` owns serial access and lifecycle transitions. `dashboard.py` owns Qt presentation. Preserve one serial owner and disconnected/disarmed startup.

The rolling example is an experiment, not the shared arm controller. New production integration must reuse the reviewed contracts described in [architecture](docs/architecture.md). Keep camera, coordinates, model identity, timing, and target semantics consistent across demonstrations and inference.

## Update the roadmap once

Edit `docs/milestones.tsv`, then run:

```bash
python scripts/check_repository.py --write-roadmap
python scripts/check_repository.py
```

Keep stable IDs. A gate needs parent-run evidence for the declared build/profile before its status changes. Final rolling profiles additionally obey `final_profile_requires`; prototype work does not wait for final CAD. GitHub issues are execution work packages linked to those IDs, not a second acceptance ledger.

## Submit a reviewable change

Use a `codex/` branch for agent work. State the problem, actual commands/results, and unresolved gates. Use the supplied templates. Do not include credentials, private CAD, hardware logs, raw datasets, vendor archives, or environments.

CI runs offline checks only. Passing it does not establish motor readiness, hosted model acceptance, or physical task success.
