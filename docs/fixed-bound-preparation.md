# Fixed-bound preparation measurement

Issue #247 measures the prerequisite for #228 on the compatibility translational
`World`. It adds counters, frozen numeric inputs and diagnostic controls. Fixed
bounds are still prepared on every actual query. Staging, sorting, traversal,
response and commit retain their existing behavior.

## Inputs and provenance

The [numeric fixture](../tests/fixtures/translational-fixed-heavy.json) comes from
committed MMORPG `20d34fbb2d9d7014e4bfb089ed1fb85fd1e9eddc`, Greyhaven Vale content
revision 3. An external temporary extractor called `greyhaven_vale::content()`
from a `git archive` of that revision, reading definition colliders, gravity,
the first 128 validated spawn-grid positions and NPC positions. Its exact source
and the fixture SHA256 are in the [raw evidence](fixed-bound-preparation-work-2026-10-01.json).
The engine has no MMORPG dependency.

There are 246 content boxes, six range slabs and nine fixed NPC boxes: 261 fixed
bodies. The additional geometry follows the consumer's
[zone construction](https://github.com/moritzbrantner/mmorpg/blob/20d34fbb2d9d7014e4bfb089ed1fb85fd1e9eddc/crates/mmorpg-core/src/zone.rs):
range slabs are ordered by axis then negative/positive side, with axial centers
±32,500, axial half extent 500 and other half extents 33,000. Their IDs start at
`1_000_000 + 2^32`. NPC IDs are `1_000_000 + 3*2^32 + npc_id`; their centers are
`[npc_x, 90, npc_z]` and half extents `[30, 90, 30]`. Dynamic IDs start at
1,000,000, using the first N spawn positions with center y=90 and those same half
extents. Source gravity is `[0, -1, 0]`. All initial velocities are zero.

The five workloads use N=8, 64 or 128: quiet N=8 with zero gravity; supported
N=8/64/128 with source gravity; and sparse N=64 with source gravity and the first
dynamic body's x velocity commanded alternately +1/−1 before each call. Creatures,
gameplay, interest management and consumer scans are excluded. These integer
compatibility-world ticks do not establish a floating-world sleep result or
complete zone/frame throughput.

## Actual work and controls

`fixed_sweep_bound_preparations` and `dynamic_sweep_bound_preparations` increment
immediately before the actual `swept_bounds` calculation. Their sum equals the
existing total, including stabilization and remaining-interval queries. The
baseline predates these fields; its split values are null.

The 24-call sparse semantic control records 18,792 fixed and 576 dynamic
preparations across 72 queries, 6,456 staged bodies, 192 contact resolutions and
zero collision events. Every step compares complete bodies and ordered events
with a world rebuilt from the current physical inputs. This supplementary
rebuilding control uses the production query; the existing private exhaustive
broad-phase oracle remains the independent pair reference. Including rebuilding
and both quiet controls gives 37,845 fixed preparations, 1,160 dynamic
preparations, 145 queries and 13,181 staged bodies. Discarded control work is
included rather than presented as production savings.

Two native repeats and three release-WASM repeats agree within their respective
build/target. Logical work counts also agree across these targets. Retained vector
payload differs with layout: native 60,640 bytes, WASM 60,448 bytes. This excludes
maps, allocator overhead and RSS. Existing failure/recovery, mutation, same-ID
reuse, full-interval CCD and scratch-release controls remain in the WASM contract.
A quiet bootstrap performs one query; subsequent unchanged quiet calls prepare
zero bounds through the existing whole-world no-op path.

## Whole-call observation

Baseline `f26ca501d4e999d1ff8d51c858bc302d7cb02e5c` and instrumented code producer
`05db847afbb74632fee125bb8fc15953534dd612` were prebuilt, then run in three alternating
order blocks. Each case/version has nine trials of 120 complete `step(1)` calls.
Only the call is timed; commands, assertions and complete body/event trace writing
are outside the timer. Trace writing between calls can still affect local timing.
All 45 paired traces were compared byte-for-byte, including every tick and event,
before deletion. Their SHA256 values and byte counts are retained in the evidence.

| Workload | Fixed / dynamic preparations over 120 calls | Baseline median total ms | Instrumented median total ms |
| --- | ---: | ---: | ---: |
| Quiet 8 | 261 / 8 | 0.067 | 0.051 |
| Supported 8 | 62,640 / 1,920 | 5.947 | 7.181 |
| Supported 64 | 62,640 / 15,360 | 7.811 | 9.833 |
| Supported 128 | 62,640 / 30,720 | 9.966 | 12.057 |
| Sparse 64 | 93,960 / 23,040 | 10.975 | 12.616 |

These are advisory local timings, with outliers retained. The active-case medians
are slower with instrumentation; this change claims no speedup. Work and complete
physical/event traces match the baseline. Supported workloads perform 240 queries,
sparse performs 360. Fixed preparation is 67%–97% of preparation count in these
active cases; that fraction is not a fraction of whole-call time.

## Isolated preparation stage

The private diagnostic calls the actual `swept_bounds` function on separately
partitioned fixed and dynamic staged states. Partitioning is outside the clock.
Each row is the median of nine alternating-order trials, each with 120 complete
passes through the indicated body population. Zero horizon and one tick
(`4_294_967_296` internal subticks) are measured separately. Raw trials are retained.

| Dynamic count | Horizon ticks | Fixed batch ms | Dynamic batch ms |
| ---: | ---: | ---: | ---: |
| 8 | 0 | 0.275438 | 0.008085 |
| 8 | 1 | 0.277502 | 0.008116 |
| 64 | 0 | 0.275980 | 0.065133 |
| 64 | 1 | 0.274356 | 0.064682 |
| 128 | 0 | 0.277883 | 0.130025 |
| 128 | 1 | 0.280728 | 0.131427 |

These batches exclude staging, sorting, response and commit. They establish
repeated fixed preparation work and its isolated cost, without predicting cache
lookup/invalidation cost or assigning a whole-call percentage. #228 remains open
for dependency-valid retention, its mutation parity and a fresh whole-call
comparison. #190 retains the separate consumer-adoption decision.

## Reproduction and verification

The timing/stage producer above contains all fixture and diagnostic sources.
`0dcd0632632f0de6a9d9f758c66acff24634d1ee` subsequently extracts duplicated
supplementary-control accumulation; production calculations and timed loops are
unchanged. The final native/WASM contract observations use that latter producer.
Toolchain: rustc 1.98.0, native `x86_64-unknown-linux-gnu`, release WASM
`wasm32-unknown-unknown`, Node 24.16.0. Resolved convention `sourceRevision`:
`e6acb5310afaf15c0cba24f87108f5f4ad1bedc3`.

```sh
cargo test --release --test fixed_bound_measurement -- --nocapture
cargo test --release --test translational_maintenance_benchmark fixed_heavy_consumer_workload_measurement -- --ignored --nocapture
cargo test --release --lib fixed_heavy_bound_preparation_stage -- --ignored --nocapture
cargo build --release --manifest-path demo-wasm/Cargo.toml --target wasm32-unknown-unknown --example translational-maintenance-contract
node scripts/test-translational-maintenance-wasm.mjs demo-wasm/target/wasm32-unknown-unknown/release/examples/translational_maintenance_contract.wasm
```

For baseline reproduction, copy the numeric fixture, `Input` parsing/construction
portion of the support module, and new ignored workload into a detached baseline
checkout. Omit the counter-specific `run()` precheck and return `[None, None]`
from `preparation_split`; leave production source untouched. Prebuild both
release test binaries before measurement. Set `FIXED_BOUND_TRACE_DIR` to an
existing directory for each invocation and compare each corresponding JSONL file
in full, with 120 rows. Run three blocks in baseline/candidate,
candidate/baseline, baseline/candidate order, retaining every printed JSON row.

All 16 fast checks passed for the timing producer, including instrumentation
neutrality. The final helper refactor passes native/release-WASM controls and
all-target clippy. Hosted PR checks cover the complete final change. Existing
seven baseline workloads, quality thresholds, solver ceilings and performance
epoch are unchanged.
