# Contact-adjacency evaluation (#443, 2026-11-02)

Decision: **do not promote the experimental `contact-adjacency` feature to the default** on
the current evidence. The scanning implementation remains the release path. The source of
manifolds and warm-start history is always `World::cache`; the experiment adds only sorted
incident body orders and rebuilds them after checkpoint restore or transaction rollback.

## Actual consumer workload

The repository's existing `mmorpg-tick` oracle models 480 60 Hz ticks over approximately
1,150 static boxes plus characters. Its observed 16 door removals/reinsertions occurred in
both storage modes. `arpg-tick` models 240 game ticks with moving characters and 6 observed
door removals, again in both storage modes. Both engine adapters move kinematic
characters through `World::move_characters`, without `World::step`: the two workloads therefore
have **zero persistent cached contact pairs**, **zero `has_support` calls** and **zero
`set_static_position` calls**. With `physics-oracle/work-counters` enabled, every game tick
asserts `World::cached_pair_count() == 0` on both storage modes. These per-tick assertions and
`PHYSICS_CONTACT_WORKLOAD_STATS=1` produced the counts above. Door removal calls still occur,
but scan an empty cache. These are accurate current adapter workloads, not a claim about future
rigid-body consumers.

## Prototype and controls

- The opt-in `contact-adjacency` feature uses `BTreeMap<Order, Vec<Order>>`, populated only
  for bodies incident to an authoritative cached pair; neighbor lists are sorted. Every pair
  appears twice as an order reference, with no manifold duplication.
- Cache insertion/retirement, `remove_body`, `set_static_position`, and `has_support` use
  neighbor-local access when the feature is enabled. After rollback or checkpoint restoration,
  the derived view is rebuilt from the authoritative cache.
- Existing deterministic contact/wake/work tests, new lifecycle, high-degree, body reuse,
  checkpoint and rollback tests, and the Rapier oracle cover observable semantics.
- The existing `.performance/contract.json` still measures the **default scanning path**.
  No production complexity class is changed by this unmerged experiment.

`examples/contact_index_probe.rs` runs exactly the same fixtures with and without the feature,
in each body-storage mode: 0/64/256/1,024 independent persistent pairs; a 256-pair shared
platform (degree 256); 1,150 inert static bodies; and 60 Hz ticks with support queries,
unrelated static movement, door-like removal/reinsertion and ordinary steps. The 60 Hz and
single-operation timings each use three fresh cloned worlds and report the median, with setup
outside timing. This is a deterministic local workload, but **not a real game trace or a
portable wall-clock gate**.

## Deterministic work and index maintenance

| Operation with P=1,024 persistent pairs | Scan | Index |
| --- | ---: | ---: |
| Unsupported `has_support`, degree 0 | 1,024 pair visits | 0 |
| Unrelated static movement, degree 0 | 1,024 visits | 0 |
| Unrelated removal, degree 0 | 2,048 visits | 0 |
| Removal of one degree-1 support | 2,048 visits | 2 |

Constructing a new set of P persistent pairs performs **2P index entry updates**. Once these
pairs stabilize, the tested 60-tick workload has **zero further entry updates**. The hub
control intentionally requires degree-sized removal work rather than a constant-time promise.
Index maintenance also performs sorted neighbor insertion/removal and tree lookups: entry-update
counts do not measure those CPU costs.

## Memory

Retained **requested heap bytes**, using the existing scoped counting allocator around the
same warm fixture (`cargo test --release --features work-counters,checkpoints[,contact-adjacency]
--test work_complexity contact_index_heap_evidence -- --ignored --nocapture`):

| Independent pairs | Ordered scan | Ordered indexed | Extra requested heap |
| ---: | ---: | ---: | ---: |
| 0 | 1,461,337 B | 1,461,337 B | 0 B |
| 64 | 1,621,001 B | 1,639,225 B | +18,224 B |
| 256 | 2,105,065 B | 2,177,865 B | +72,800 B |
| 1,024 | 5,153,329 B | 5,446,769 B | +293,440 B |

The Indexed storage mode has the same **+293,440 B** delta at P=1,024. This counts heap
allocation request sizes still live with the world; it excludes allocator metadata, RSS and
the additional 24 B inline `World` field. The index's lower-bound vector/map payload at
P=1,024 is 229,376 B; the full measured delta is larger due to tree node allocations.

## Native exploratory timings

Native release, work counters **disabled**, nanoseconds per complete tick in the isolated
60 Hz rigid-body probe (one local A/B sample, 3 medians per case):

| Storage | Persistent contacts | Scanning | Indexed |
| --- | ---: | ---: | ---: |
| Ordered | 64 independent | 147,539 ns | 194,458 ns |
| Ordered | 256 independent | 729,242 ns | 1,119,796 ns |
| Ordered | 256 shared platform | 495,212 ns | 917,785 ns |
| Ordered | 1,024 independent | 2,812,167 ns | 5,563,550 ns |
| Indexed | 64 independent | 111,532 ns | 159,116 ns |
| Indexed | 256 independent | 707,683 ns | 647,442 ns |
| Indexed | 256 shared platform | 596,920 ns | 541,326 ns |
| Indexed | 1,024 independent | 3,136,010 ns | 3,173,677 ns |

The single-operation microbenchmarks do show useful neighbor-local speedups; for example,
at P=1,024 in Ordered mode, the unrelated support lookup fell from ~3.16 µs to ~0.07 µs and
unrelated removal from ~8.71 µs to ~0.63 µs. Removing the dense 256-contact platform was
slower with the index (~96 µs vs ~55 µs). The full-tick outcomes are mixed or worse, and the
shared runner has notable timing variation. Do not turn these samples into a CI threshold or
claim a regression percentage independent of this environment.

Fingerprint: Ubuntu 24.04.3, Intel Xeon Platinum 8573C, x86_64-unknown-linux-gnu,
Rust/Cargo 1.99.1 (stable, 2026-10), release profile, locked dependencies (`Cargo.lock`
SHA-256 `0e63d46487c159cb19159bbd5da47e31f53f628be0e1d100497c3496a3851449`).
The scanning implementation is based on `main` at `7bc638cd5a323406a67bb19b215a7ae4da98632a`.

## Correctness and Rapier checks

- `cargo test --workspace --locked --all-features` passed after updating the existing
  bookkeeping test fixture, which deliberately writes `World::cache` directly, to update
  the derived index at the same time. Production mutation paths are already incremental.
- `cargo clippy --workspace --all-targets --all-features --locked -- -D warnings` passed
  during the experiment; rerun after any changes.
- The native `cargo run --release --locked -p physics-oracle --features
  physics-engine/contact-adjacency` checked the full scenario matrix. Its only reported
  failure was the performance threshold for `debris-town-tumble` (indexed 1.13× Rapier).
  The same scenario **also failed without indexing** in a subsequent baseline-only run
  (1.64× Rapier). This is not proven index-specific, but the native full gate is red.
- `PHYSICS_CONTACT_ADJACENCY=1 node scripts/oracle-wasm.mjs` passed the entire WASM
  scenario matrix: invariants and both storage modes' 1.10× Rapier timing gates.

The tools run in a shared virtualized environment. Its wall-clock measurements can vary
between runs; especially the native `debris-town-tumble` results above are not an exact
head-to-head timing comparison. Keep this uncertainty explicit.

## Merge criterion

There is no net consumer improvement to demonstrate while those adapters have P=0, and the
positive microbenchmark result is not a substitute for a complete 60 Hz workload benefit.
The index consumes additional heap and its full-tick cost is not consistently favorable.
**Leave it experimental/unmerged.** Reconsider a narrow hybrid or production adjacency only
when a real consumer uses rigid-body contacts with measured support/movement/removal frequency,
and an exact-workload A/B provides a clear net or scaling win while all Rapier gates are green.
