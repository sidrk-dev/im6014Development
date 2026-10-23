# Software fault-latch acceptance

The existing shared core already retains firmware temperature-related error bits and measured-effort faults. This unit adds permanent acceptance coverage, without changing controller logic or adding temperature fields, thresholds or a thermal model.

One new test exercises 14 subcases: five thermal error bits (4, 5, 6, 8, 9), repeated measured over-effort, and combined faults, independently affecting either of two joints. The synthetic fixture uses a 1 Nm limit; that number is not a physical actuator rating.

Each case queues a target, injects the fault and verifies pending-target eviction, grant revocation and rejection of fresh target instances without changing the fault snapshot. Healthy packets, Stop and reconnect cannot clear the latch. A persistently unhealthy acknowledged check fails. Recovery requires an acknowledged healthy check, then an explicit fresh arm; the old grant remains invalid and only a fresh valid target reaches the fake byte sink.

The parent ran all 160 controller tests and the offline suite's 18 bench groups. Four in-memory mutations independently tested silent latch clearing, pending-target retention, grant retention and automatic rearm. Every mutant caused 14 assertion failures with no test errors. Original methods were restored after each experiment; no production source was modified. Gemini's independent source review completed with no findings and ran no runtime tests.

Run the permanent regression with:

```bash
python -B -m unittest tests.control.test_controller_fault_latch
```

[Verification and source hashes](verification.json) record the selected evidence. This accepts the software C14 retention rule only. Actual motor temperature, thermal ratings, firmware clearing behavior, physical ports and hardware protection remain unverified. SA04 remains in progress for process/dashboard integration and actual hosted-worker timing.
