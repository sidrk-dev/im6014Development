# Local process fixture verification

The foreground `ProcessStation` holds one rolling station, model, controller and profile lock. A disposable subprocess receives metadata-only capture notices and returns proposed vectors over a bounded Unix socketpair. Lifecycle check/arm remains owner-only. Original captures stay with the owner; fresh challenge receipts expire after 300 ms. Each session owns its wire buffers, nonce, epoch and capture correlations.

Stepping polls child retirement without waiting. Startup, explicit replacement and close may wait within bounded handshake/termination limits. This fixture does not promise a hard 100 Hz wall cadence.

## Parent verification

The pinned environment passed all 116 simulation, SDK and process tests, including 19 actual-process cases. Offline repository, controller and bench checks passed. Tests cover killed/stopped/stalled children, stale heartbeat and capture replay, prohibited lifecycle frames, malformed types, partial sends, replacement, exclusive profile locking across processes, abrupt owner death and cleanup. CI includes the process tests explicitly.

Three independent parent probes initially exposed defects despite the original 12 passing tests: unhashable message types escaped the owner loop; a partial frame contaminated a replacement connection; child termination blocked physics during grace. The corrected candidate disconnects malformed input without an escaping exception, clears pending bytes from 120 to zero before successful replacement, and completed 26 advances while a terminating child remained alive. These local observations are regression evidence, not throughput guarantees.

## Independent source review

Configured Gemini and GLM reviews completed with no material findings. DeepSeek returned malformed output and does not count as a completed review. Reviewers inspected inline source; parent executed tests and probes. Provider identity is pinned by launcher arguments rather than independently verified backend identity.

Parent triage: GLM noted passive diagnostic state between service calls, a hypothetical oversized startup hello, and an exceptional unreapable-child cleanup path. The fixed three-coordinate hello fits the bounded frame; runtime status is a snapshot, repaired at service/refresh boundaries. These observations do not establish a new defect in this scoped single-threaded fixture. Review prose claiming vectors cannot be spoofed is interpreted narrowly: original capture provenance stays owner-side, while proposed vectors remain untrusted and station-validated.

## Remaining gates

This is local fake/simulation evidence. Actual Qt dashboard integration, hosted inference transaction timing, learned behavior and physical motor-port exclusion remain unverified. SA04 remains in progress and SA05 is not advanced.
