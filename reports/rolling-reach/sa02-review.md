# Selected-profile SA02 acceptance, 2026-10-21

The parent ran the actual pinned Dream client against rendered overhead JPEGs
and measured logical joints from `sim_rolling_reach_v1`. Transport and credential
loading were mocked with fake loopback configuration. The selected verifier
passed 44 layered negative checks; all 37 contract/import/SDK regressions passed.
The earlier CAD-profile SDK command also passed. The full bench/repository
baseline passed its 18 grouped checks without motor I/O.

The parent reproduced eight failing regression subcases before fixing numeric
string/boolean coercion, camera/limit metadata drift, and results-directory
symlink redirection. The passing-after run includes those cases. Output paths
stay inside the example's ignored results directory, including when that
directory or an output file is a symlink. Nested report directories work.

The configured Gemini source reviewer completed read-only review. Its runner
receipt records `gemini-3.8-flash-high` with pinned-argv evidence, not independent
model verification. It did not execute tests. The parent accepted its import
cleanup: the selected verifier now uses canonical package imports. One narration
comment was removed; no comments were restored or constraint encodings offered.
Remaining E402 suppressions are style-only import-order allowances.

Three reviewer discrepancies did not require changes. The older checker's
explicit `--report` path is a pre-existing user-selected file interface outside
this confinement change. Repeated timestamps in negative fixtures are accepted
by the pinned SDK; future production freshness is SA04, not evidence of a
current failure. `--output` requires a filename, so rejecting a directory is
intentional. No additional blocking selected-boundary defect remained after
parent triage. Full local receipts remain in the ignored audit directory.

SA02 is accepted only for local selected-profile/client/schema/serialization.
Policy rows and camera capture are 30 Hz; local control is 100 Hz. Numeric
vectors cannot identify their physical units or semantic order by themselves;
local profile metadata binds this fixture. A fake checkpoint label proves no
hosted training identity or learned behavior. SA04 must supply controller,
freshness, asynchronous timing, ownership, effort and final-release integration.
