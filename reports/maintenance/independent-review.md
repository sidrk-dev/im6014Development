# Independent cleanup acceptance review

What this change does: It moves the existing motor bench into a checkout-runnable package, preserves the root launcher, separates generated outputs, and consolidates overlapping plans into one 21-gate ledger with a 195-ID legacy crosswalk. Expected load is one local operator and one serial-owning process.

## Final findings

No unresolved runtime or acceptance-gate findings in the reviewed result. Three substantive omissions were found during review and fixed by the parent; the fixes were read back independently:

1. **Teleoperation loss of intent — resolved** (`docs/milestones.tsv:7`; `docs/roadmap.md:50`; `docs/dream-integration.md:32`). SA05 now explicitly requires key/deadman release, focus-loss and gamepad unplug/reconnect checks, controlled-timebase behavior, supported hold/stop, no automatic resume, and interrupted-recording recovery. This restores concrete G02-G04 obligations that “bounded teleoperation” alone had omitted.
2. **Failure injection and supported stopping — resolved** (`docs/milestones.tsv:12`; `docs/roadmap.md:55`; `docs/dream-integration.md:30`). SA10 no longer refers to removed “failure scenarios below.” It explicitly lists stale frames, wrong shape/units/build, NaN/Inf, late/reordered replies, unavailable joints, network failure, expiry, faults, ownership changes, takeover, and build-specific supported stopping. The current guide retains continued local control and invalidation of outstanding chunks.
3. **Measured physical acceptance — resolved** (`docs/milestones.tsv:16`; `docs/dream-integration.md:42`). SA14 now freezes a separate physical held-out set, tolerance, timeout, and stopping criteria, reports every attempt/intervention, and explicitly requires passing the frozen measured rule before acceptance or increasing physical task difficulty/duration at SA15.

Verdict: Ship the reviewed cleanup. This approves repository organization and preservation of acceptance requirements; it does not accept pending controller, learned-task, hosted, or physical gates.

## Checks actually performed

- Ran `PYTHONDONTWRITEBYTECODE=1 python -m tests.bench.test_motor`: passed packet encoding/CRC, parsing, noisy/fragmented replies, partial-write/deadline rejection, and safety rejection; fake I/O only.
- Ran the final `PYTHONDONTWRITEBYTECODE=1 python scripts/check_repository.py`: passed 21 gates, both dependency graphs, 195 legacy mappings, archive/sample hashes, and 44 current portable links.
- Ran launcher help from the checkout and `/tmp`; both passed. Ran package motor CLI help; passed. No serial port opens in these commands.
- Compiled the 14 package/test/script/sample Python sources in memory without bytecode writes; passed.
- Exercised dependency validation in memory: valid graphs passed; unknown and cyclic normal/final-profile dependencies were rejected.
- Compared runtime/test function ASTs against `/tmp/im6014-cleanup-before-20261020`: protocol/session functions and test helper functions are unchanged; the sole runtime function change is the dashboard manual-resource path. Inspected changed top-level import/patch/output paths and callers.
- Compared sample source/model/lock files to the before snapshot: identical except the intentionally rewritten README. Bench requirements and MCP example bytes are unchanged.
- Verified public `workbench-plan-2026-10-20.md` and `milestones-before-cleanup.tsv` exactly match the original 195-item plan and prior 21-gate ledger.
- Verified all 26 original planning/research artifacts in the before snapshot remain byte-identical in either the public archive/reports or ignored local history. Every one of the 31 digests in `reports/research/local-history-catalog.json` matches preserved local bytes. Curated publication therefore does not lose original material locally; a fresh public clone intentionally lacks those cataloged local-only copies.
- Rechecked the independent controller ledger SHA-256 against its recorded pre-cleanup digest: unchanged. Every active ledger evidence file belongs to the candidate Git source set.
- Inspected CI ordering, package commands, separate bench/sample requirements, OSMesa environment/system dependency, and artifact directories. Inspected manuals and build/profile boundaries; no claim was found that local tests establish hosted or physical acceptance.

## Publication provenance note

Earlier rows in `reports/maintenance/decisions.tsv` use historical relative names for audits, provider receipts, fresh-source logs, and launcher evidence subsequently moved into ignored local history. Those records are preserved and cataloged, but those historical names are not current public file links. The parent was notified to annotate that curation in the trail or point those rows to the public local-history catalog. Selected before/after bench logs and the concise verification summary remain public.

Not checked: Full Qt/simulation/SDK/dataset reruns (they write artifacts, disallowed by this read-only assignment), a pristine dependency installation, hosted GitHub Actions execution, remote GitHub account/issues/PR state, external documentation freshness, physical hardware, hosted Dream calls, or controller implementation acceptance in independent checkouts. Parent-reported UI/simulator/fresh-source results were not counted as independently rerun evidence. CI's OSMesa execution remains pending hosted validation; the parent separately reports local EGL success.
