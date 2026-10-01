# Translational fixed-bound retention

Issue #228 retains derived fixed bounds in the compatibility `World`, following
the [fixed-heavy measurement prerequisite](fixed-bound-preparation.md). Fixed
bodies have zero velocity and do not move during contact response, so their
bounds are independent of the remaining sweep horizon. Dynamic bounds continue
to be prepared from current staged motion for every query.

## Authority and validity

The body map remains the physical authority. A private sorted vector stores only
fixed BodyIds and derived min/max bounds. Every query supplies the current staged
body index; cached entries never retain indices across membership changes.
Canonical body and simultaneous-event ordering are unchanged.

Successful fixed pose or half-extent changes, replacement with a dynamic body,
and removal discard the affected cached entry. Admission and same-ID reuse prepare
new geometry on demand. Material-only changes and unchanged commands preserve
valid geometry. The API exposes no mutable body reference, and fixed bodies do
not change pose inside a step. Cache capacity is reserved from the authoritative
fixed population; staging and broad-phase sorting still visit all bodies.

`fixed_sweep_bound_preparations` counts actual calculations,
`fixed_sweep_bound_reuses` counts cached reads, and dynamic preparation remains
horizon-dependent. Fixed preparations plus reuse equal fixed population × actual
queries; fixed plus dynamic preparations equal the existing preparation total.
`fixed_bound_invalidations` counts cached entries removed by successful mutations
since the previous positive-tick attempt. A rejected mutation leaves it unchanged.
It is consumed when such an attempt starts; the API still returns only the
existing error on failure, without a partial success report.

Scratch release frees the derived cache and all execution-vector capacity while
preserving already pending mutation invalidations. Release itself is not counted
as a dependency mutation. The next active query prepares the released fixed
bounds again. Cache population after an error remains disposable valid geometry;
body changes and events retain their existing atomic publication contract.

## Controls

The independent exhaustive world oracle covers moving fixed walls, changed
half extents, removal/re-admission, lower-ID insertion, kind changes, repeated
impacts and recontacts over changing horizons. Existing error, canonical event,
thin-wall sweep, scratch and stationary controls remain active.

Shared native/release-WASM lifecycle checks additionally verify actual work for
unchanged pose/material commands, changed pose/shape, kind transitions, removal,
same-ID reuse, failed mutations, cloning and mutation → release → step. Rebuilt
production worlds are supplementary complete-body/event controls, rather than
the independent pair oracle. Report normalization compares equivalent query work
while preparation/reuse, invalidation and payload differences have separate
assertions.

The 24-call sparse fixture performs 261 fixed preparations and 18,531 fixed
reuses, alongside the unchanged 576 dynamic preparations and 72 queries. Including
all rebuilding and quiet probes gives 6,786 fixed preparations, 31,059 reuses,
1,160 dynamic preparations, 145 queries and 13,181 staged bodies. Two native and
three WASM repeats agree within each build/target. Logical work agrees between
these targets; retained total vector payload is 89,872 bytes native and 89,680
bytes WASM. The fixed-cache component is 29,232 bytes on both recorded targets.

## Whole-call evidence

[Raw evidence](fixed-bound-retention-work-2026-10-01.json) compares baseline
`b7f37b6344bfe9a7c42a65016173751f4748faca` with clean code producer
`bee7b355d2a46ee1e25f7a275ec0a2cf5a326b84`. Inputs are the unchanged numeric
consumer fixture: 261 fixed bodies and 8/64/128 controlled dynamics. Each
case/version has nine trials of 120 complete `step(1)` calls, in three alternating
order blocks. All 45 paired body/event traces were compared byte-for-byte,
including every completed tick, before deleting them; hashes and byte counts
remain in the record. Existing seven workloads retain matching semantic/work
counts and diagnostic checksums across three trials per version, recorded at the
initial producer `b2d1c7b`.

| Workload | Fixed preparations baseline → retained | Fixed reuses | Median total ms baseline → retained |
| --- | ---: | ---: | ---: |
| Quiet 8 | 261 → 261 | 0 | 0.039 → 0.043 |
| Supported 8 | 62,640 → 261 | 62,379 | 6.115 → 5.778 |
| Supported 64 | 62,640 → 261 | 62,379 | 7.781 → 7.721 |
| Supported 128 | 62,640 → 261 | 62,379 | 10.362 → 10.195 |
| Sparse 64 | 93,960 → 261 | 93,699 | 11.295 → 10.940 |

Dynamic preparations, query/traversal counts, contact response counts and events
are unchanged. Supported traces still perform 240 queries and sparse performs
360. Preparation work falls substantially; whole-call timing remains advisory.
The earlier complete comparison at `b2d1c7b` showed less than 1% active median
changes. Its full 90 timing rows and 45 trace comparisons are retained too. The
final active medians improve by roughly 0.8%–5.5%, with overlapping ranges and
outliers. Quiet bootstrap is slightly slower. These local observations do not
establish a portable speedup or consumer frame-time improvement.

The cache adds 29,232 bytes of retained vector payload in every fixed-heavy case.
Native retained totals change from 60,640 → 89,872 bytes for N=8,
74,080 → 103,312 for N=64 and 89,440 → 118,672 for N=128. Active peak-vector
totals also include the cache. Existing staging/candidate capacities are unchanged.
These are vector capacities, excluding authoritative map storage, allocator
overhead and RSS. Quiet cached calls report zero active work while keeping
retained capacity until explicit release.

## Reproduction and remaining scope

Toolchain: rustc 1.98.0, native `x86_64-unknown-linux-gnu`, release
`wasm32-unknown-unknown`, Node 24.16.0. Resolved convention `sourceRevision`:
`e6acb5310afaf15c0cba24f87108f5f4ad1bedc3`. Fixture and binary SHA256 values,
raw repetitions, initial observations and all supplementary workload records are
in the evidence.

```sh
cargo test --locked --release --test fixed_bound_measurement -- --nocapture
cargo test --locked --release --lib world::maintenance_tests -- --nocapture
cargo test --locked --release --test translational_maintenance_benchmark fixed_heavy_consumer_workload_measurement -- --ignored --nocapture
cargo test --locked --release --test translational_maintenance_benchmark translational_consumer_workload_matrix -- --ignored --nocapture
cargo build --locked --release --manifest-path demo-wasm/Cargo.toml --target wasm32-unknown-unknown --example translational-maintenance-contract
node scripts/test-translational-maintenance-wasm.mjs demo-wasm/target/wasm32-unknown-unknown/release/examples/translational_maintenance_contract.wasm
```

For baseline timing, copy only the new diagnostic benchmark to the detached
baseline. Keep its actual fixed/dynamic preparation counters, return null for
unavailable reuse/invalidation counters, and zero for fixed-cache payload because
that baseline has no cache. Keep baseline production and the frozen fixture
unchanged. Prebuild both binaries. For each invocation set
`FIXED_BOUND_TRACE_DIR` to an existing separate directory and compare corresponding
120-row JSONL files in full. Use baseline/candidate, candidate/baseline,
baseline/candidate block order and retain every printed row. Only `step(1)` is
timed; commands, exports and validation are outside the clock. Trace writing
between calls may still affect timings.

All 16 fast checks passed at both the initial producer and `b82e3c9`. The latter
loads shared report normalization once per diagnostic driver after hosted demo
lint caught duplicate module loading. Production calculations and timing loops
are unchanged. Final focused native controls, release-WASM repeats and locked
root/demo all-target clippy pass after the review fixes; hosted PR gates cover
the complete final change. Solver limits, thresholds,
performance epoch and diagnostic-only exact-reference policy are unchanged.
All-N staging/sorting, broad-phase traversal and consumer scans remain. #190
retains consumer adoption and wider locality evidence.
