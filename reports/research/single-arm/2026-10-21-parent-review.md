# Parent review of SA01 and SA02

Reviewed on 2026-10-21. The current ledger contains the single-arm milestones, separate from historical C-controller work in independent checkouts. DeepSeek supplied SA01 contract and SA02 offline client work as uncommitted changes.

## Corrections

The SA01 checker previously accepted changed joint signs and zeros, reversed limits, and deleted asset or exclusion records. It now checks these conventions and required record sets explicitly. Sixteen regression tests exercise the accepted contract and rejected mutations.

SA02 now validates the SA01 contract before SDK checks and uses a one-row, three-wide positive fixture. The earlier two-row fixture verified SDK remapping but did not match the frozen chunk length. This does not establish runtime chunk enforcement, which belongs to SA04. Historical October 20 reports remain unchanged.

## Parent verification

- The full offline baseline passed, including bench protocol and Qt lifecycle checks, the contract checker, 16 regression tests, and repository consistency. No real motor communication occurred.
- The frozen local prototype URDF and model hashes matched the contract. See [contract verification](2026-10-21-contract-check.json).
- The pinned installed Dream client passed the corrected one-row fixture and both malformed-width checks. See [client verification](2026-10-21-sa02-client-verification.json). Transport was mocked.
- The rolling sample geometry and dynamics checks, installed-SDK boundary check, and local dataset preflight passed. These checks still concern the two-coordinate illustrative sample.
- A separate native read-only review found no material issues in the corrected checkers.

## Scope

SA01 remains contract frozen and SA02 remains client verified. Neither status establishes prototype physics, hosted behavior, learned reaching, or physical readiness. No cloud training, dataset upload, motor port, or physical trial was used.

The configured Muse review returned HTTP 400. Automatic approval review blocked source sharing with Gemini. Neither route provided an independent external verdict. Local code review and parent-run checks are the verification recorded here.

## Review choices

Prove It Works required parent-run checks on the actual assets and installed client before recording acceptance. Model the Domain kept contract, client, kinematic, physics, hosted, and physical evidence distinct in the ledger and report. The implementation reused a local URDF import rather than adding another maintained robot description.

A later authorized Gemini source review completed with no blocking findings. The [continuation review](2026-10-21-continuation-review.md) records that result and fresh parent-run checks separately from the earlier unavailable attempts.
