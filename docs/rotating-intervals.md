# Atomic rotating intervals

Sampled-event and tail-contact totals are checked inside the interval transaction and returned in its work report. A reporting overflow rolls back physical and sleep state before returning an error, so a consumer does not need a fallible aggregation after commit. Failed-interval work retains counts for completed discarded solver commands. Event totals inherit diagnostic step statistics and are zero with `performance-counters` disabled; physical state and rollback do not depend on those counters.

`RotatingWorld3d::advance_interval` partitions one rational physical interval into a
positive number of equal substeps using the existing integer rotating solver. This
is a compatibility API; new CPU physics state continues to use `numeric::Scalar`.
It is the engine prerequisite for the persistent rotating adapter in
[ecs-lab#133](https://github.com/moritzbrantner/ecs-lab/issues/133).

The command validates the numerator, denominator, partition multiplication and
angular damping before stepping. It currently accepts rigid-box worlds only and
rejects any ballistic membership before mutation. Ballistic motion and retirement
are intentionally outside this transaction. A zero interval skips integration and sleep-history updates, matching ordinary
`step(0, denominator)`, while still applying explicitly requested angular damping
with the existing ECS consumer's once-per-request rule.

On success, the caller's reusable report vector contains every completed substep
in order. Angular damping is applied once, after the complete interval, using the
ECS adapter's existing integer milli scale and rounding to nearest (ties away from zero). The last report includes
any damping changes in its sorted `changed_body_ids`. Caller-owned angular velocity on externally driven bodies is preserved. Damping
visits the retained active dynamic partition, counting externally driven entries
that it skips; it does not visit fixed geometry or parked dynamics.

On any returned solver error, the vector is cleared and all interval changes to
body motion, sleep deadlines/candidate membership, boundary obligations and parked
membership are restored. The existing `step` command keeps its documented behavior;
this complete-interval guarantee is opt-in. Panic recovery is not part of either
command contract.

The transaction records a first before-image only for a touched body. Motion
images contain kind, position, linear velocity and angular state, rather than a
complete rigid body. Sleep images contain the body's sleep/candidate flags,
optional stability deadline and pending-boundary flag. Parking images exist only
for actual park/unpark transitions and retain the original dynamic representation
when necessary. Temporary sleeping proxies are restored independently and are not
mistaken for persistent park transitions. No complete world, scene, body map or
sleep map is copied for rollback.

On error, retained broad-phase geometry and current-contact geometry are
invalidated instead of copied. Their work counters remain cumulative. Contact
response scratch has no authoritative contact history and is prepared by the
existing solver on reuse. Only touched parked-index entries are restored.

`RotatingIntervalWork3d` counts completed substeps, journal entries, damping visits
and changes. Four diagnostic contact counters include discarded substeps and wake
probes. These counters are a subset of total work: they exclude current-contact
cache geometry tests, solver scratch allocation, fixed-geometry preparation and
allocator/tree overhead. Journal-entry counts are not RSS or allocated-byte
measurements. Complete-call timing is required for performance claims.

The public acceptance fixtures cover a second-substep overflow after a successful
first step, restoring a newly parked body and its deadline, restoring an admitted
wake and subsequent impacts, successful partition equivalence, once-per-interval
damping, invalid/zero intervals, ballistic rejection, and a fully parked 128-body
zero-journal/zero-contact-work interval, and fixed-preparation parity after rollback,
support removal, ID reuse, external motion authority, and positive/negative damping
ties with a report whose only delta comes from damping, including a zero interval. The separate Pages acceptance driver runs
the same fixtures three times on WASM and is not shipped in the demo module.

This slice does not implement the persistent ECS adapter, ECS lifecycle deltas,
its playground/tower routing, consumer parity or consumer performance acceptance.
Those remain requirements of ecs-lab#133.

## Complete engine-call comparison

[Recorded native paired trials](rotating-interval-work-2026-09-30.json) use clean
producer `4442c768473feae041bc569eb49881b854011b42`, Rust 1.98 and resolved
conventions `e6acb5310afaf15c0cba24f87108f5f4ad1bedc3`. Three alternating trials
per scene warm 64 intervals and time 256 intervals, each containing four solver
substeps. The same-build control collects the same reports from four ordinary
`step(1, 240)` commands. Body state, sleep state and final full reports match in
all 15 pairs, checked outside timing. Construction and ECS conversion/writeback
are outside this engine-call boundary.

| Scene | Ordinary commands, median µs/interval | Atomic command, median µs/interval | Change |
| --- | ---: | ---: | ---: |
| quiet-128 | 0.554 | 0.523 | -5.5% |
| active-4 | 23.182 | 24.563 | +6.0% |
| sparse-512 | 1526.588 | 1521.151 | -0.4% |
| active-512 | 5475.477 | 5567.441 | +1.7% |
| contacts-32 | 2250.362 | 2256.110 | +0.3% |

The parked case records no journal entries or contact queries. Sparse 512-body
work records four motion and four sleep images; active 512-body work records 512
of each. The 32 independent elastic contact corridors record 32 of each.
These three-trial results are advisory. Sub-microsecond parked timings and small
percentage changes do not establish an application speedup. The existing legacy
solver still owns its ordinary scene/working buffers; this transaction introduces
no full-world rollback snapshot. Consumer construction, metadata, mutation,
conversion, retained memory and writeback acceptance remain in ecs-lab#133.
