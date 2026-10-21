# Repository cleanup, 2026-10-20

The active project had several overlapping plans, a 195-task handoff, machine-specific evidence links, and runtime/test scripts mixed with generated bench records. The public Git repository contained only the older bench application, so publishing its edited README alone would have left most new links broken.

## Decisions

- Keep one [21-gate ledger](../../docs/milestones.tsv), with a generated [roadmap](../../docs/roadmap.md). Two gates have scoped source/local evidence; 19 remain unfinished. Preserve required controller, teleoperation, failure-injection, data, and measured physical acceptance checks.
- Remove the 195-task handoff from the execution queue. Preserve its exact snapshot and [complete mapping](../../docs/archive/legacy-milestone-map.tsv). Local VLA training, food-task expansion, optional agent features, and extra UI work are deferred. Repeated implementation steps are owned by the surviving gates.
- Keep the bench in a checkout-runnable `im6014_bench` package, tests in `tests/bench`, and the stable root launcher. The small application does not currently need a distribution build. PyPA documents that a flat package can run from the checkout while a `src` layout normally adds an installation step; this is the tradeoff chosen here. [PyPA layout reference](https://packaging.python.org/en/latest/discussions/src-layout-vs-flat-layout/)
- Separate current docs, dated reports, curated assets, and ignored generated outputs. Preserve vendor archives, bench records, and independent controller checkouts locally. The controller code in those other checkouts still needs reconciliation at SA04; this cleanup does not import or accept it.
- Use GitHub issues for a few execution work packages and milestones for broad phases. Acceptance stays in the repository ledger. GitHub describes milestones as groups of issues/PRs; a second 195-item issue queue would recreate the overlap. [GitHub milestone reference](https://docs.github.com/en/issues/using-labels-and-milestones-to-track-work/about-milestones)

## Verification and limits

The [verification summary](verification.json), [before](bench-before.log)/[after](bench-after.log) bench results, and [independent review](independent-review.md) record actual checks. The packet/session functions are unchanged; the dashboard's only function-level logic change is its manual path. All 18 original UI/fake-I/O groups pass. A source-only checkout passes the bench, rendered MuJoCo checks, real SDK handling with mocked transport, and local dataset preflight. The root launcher also rendered from `/tmp` without opening serial.

The independent reviewer required three planning corrections: concrete loss-of-intent behavior, explicit failure injections, and measured physical acceptance before harder trials. All were restored and read back. The configured external reviewer was unavailable; its failure was not counted as approval.

This work performs no physical motor I/O or hosted Dream calls and advances no pending controller or learned-task gate. Local rendering used EGL. The initial hosted bench job exposed a missing EGL library before UI execution; the CI fix installs Qt's EGL/OpenGL runtime libraries. Both bench and rolling-sample jobs then passed after clean dependency installations on [head c963f23](https://github.com/sidrk-dev/im6014Development/actions/runs/39552698470), including OSMesa rendering, SDK fixtures, and dataset checks.

[PR #1](https://github.com/sidrk-dev/im6014Development/pull/1) contains the reviewable cleanup. [Issues #2–#5](https://github.com/sidrk-dev/im6014Development/issues) organize contract/client publication, prototype physics, shared-controller reconciliation, and final rolling geometry under two broad milestones. Newer local milestone candidates are preserved separately and are not accepted or published by the cleanup.

The append-only [decision trail](decisions.tsv) retains historical evidence names. Its final curation row points to the [31-file local-history catalog](../research/local-history-catalog.json); those detailed artifacts remain local. The public archive and selected sample reports are sufficient for roadmap provenance and source-hash checks.
