# Workbench development

Read `docs/architecture.md`, `docs/roadmap.md`, and `CONTRIBUTING.md` before changing the project. `docs/milestones.tsv` owns active statuses and dependencies. Regenerate `docs/roadmap.md` with `python scripts/check_repository.py --write-roadmap`; do not create another editable milestone list.

Use the operator's current Pstack model routes. Do not copy routing choices into a second maintained configuration. Parent sessions own Git operations, installation, publication, and acceptance.

Keep `./start_dashboard.sh` working for installed desktop launchers. Bench implementation lives in `im6014_bench`; shared-controller code in local independent checkouts must be reviewed before migration. Do not replace it with a simulator-specific controller.

Default to simulation, fake transport, and disconnected startup. Software work does not authorize opening a real motor port. Distinguish source inspection, simulated mechanics, local SDK fixtures, hosted responses, learned behavior, and physical outcomes.

Run `scripts/check_offline.sh` for bench/repository checks. Run the example's simulator/SDK/dataset checks when changing that boundary. Write generated output to ignored results directories. Publish only selected reviewed reports. Keep credentials, vendor archives, raw sessions, environments, and local worktrees out of commits.

The archived 195-item plan is historical. Preserve its acceptance requirements through current gate checklists; do not restart old setup or implement deferred UI, agent, local-training, or food-physics work without a new task.
