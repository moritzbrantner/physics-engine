# Fixed-step bookkeeping and retained scratch

Issue #155, following prepared response (#159). Baseline source is merge
`23c2ac9655f9f6ef452cea9ae7dd4cc34ee89ee7`.

This changes derived bookkeeping, not the solver. Four substeps, eight velocity iterations,
f64 arithmetic order, material response, penetration tolerances, CCD admission, contact order,
external impulses and quiet-time thresholds are unchanged. Rust body state and contact keys
are still authoritative.

## Reuse and invalidation

- The active count and sorted body-index view change on add/remove, actual wake and actual sleep.
  The count makes quiescence O(1). Integration and sleep roots use the refreshed active view.
  External bodies with positive mass remain active, even though their inverse response is zero.
- A compact adjacency index is derived from canonical contact pairs. Each body traversal visits
  only its incident edges, rather than scanning every cached pair for each visited body. Fixed
  and externally driven bodies remain traversal boundaries. Their neighbors are still available
  for explicit support-removal invalidation.
- Contact-key insertion/removal and body layout changes invalidate adjacency. Updated impulses,
  anchors and normals do not change connectivity and do not rebuild it. Same-ID replacement and
  index shifts on insertion/removal always invalidate the derived indices.
- Contact-point buffers are cleared and refilled in place for surviving active pairs. Sleeping
  pairs retain their cached points. An absent active contact is removed after refresh; no stale
  warm-start impulse survives a real separation.
- Bounds, output manifolds, constraints, wake roots, support edges, support flags, visited marks,
  traversal queues, warm-start matching and earliest-projectile times retain Vec capacity.
  No persistent manifold/geometry shortcut is introduced; that is the separate #156 work.
- Sleeping/static padded bounds remain valid. Moving rows and rows just transitioning to sleep
  are refreshed. A lifecycle change rebuilds the indexed rows. The current timestep is used for
  every active row, so changing dt never reuses an active sweep for another horizon.
- Sorted sweep order is reused when still valid. A linear total-order check precedes sorting;
  sorting runs only after a crossing/reordering. Both order keys and emitted pairs keep BodyId
  tie breaking. The unstable in-place sort has the same unique total key as the former stable sort.
- Traversal epochs avoid clearing an entire visited array on every root. Epoch overflow clears
  the stamps explicitly. Clone copies derived data consistently; later mutations invalidate the
  clone's own storage independently.

The implementation still walks broad-phase rows, tests sweep-and-prune candidate pairs, refreshes
response scratch, and inspects contact-cache entries. It does not claim fully O(active bodies)
stepping. Geometry construction may still allocate small vectors. Retained scratch uses memory
proportional to peak body/contact counts; there is no unbounded per-tick history.

## Evidence counters

`Report::bookkeeping` contains source-level work, not measured CPU instructions. The existing
WASM `approximate_stat` indices 0 through 13 retain their meanings; indices 14 through 24 report:

| Index | Value |
|---|---|
| 14 | Active-view rebuilds |
| 15 | Bodies examined while rebuilding that view |
| 16 | Adjacency rebuilds |
| 17 | Contact edges indexed during rebuilds |
| 18 | Dynamic bodies expanded by island traversals |
| 19 | Incident edge entries visited by those traversals |
| 20 | Padded sweep rows updated |
| 21 | Rows included in full sorts |
| 22 | Capacity growths of the tracked reusable Vec buffers |
| 23 | Retained element-capacity bytes of those buffers (gauge, not a sum) |
| 24 | Adjacent-row order comparisons, including when a sort is avoided |

The retained-byte gauge excludes body storage, response scratch from #154, contact-cache and
allocator overhead, and heap allocations inside geometry routines. Capacity-growth counts are
not a global allocation profiler. API-triggered wake/graph work is included in the next positive
successful step; zero-duration steps do not consume that pending work. The experimental solver's
existing nontransactional error contract is unchanged.

The deterministic warmed-neighborhood test uses 32/128/512 sleeping bodies and one isolated
moving body. With unchanged topology it requires zero adjacency/active-view rebuilds, four body
visits, zero incident edge visits, four bounds updates, no full sorts and zero tracked capacity
growths for one four-substep tick. It does not hide the remaining broad-phase row order checks.
The real two-tower regression uses two 32-crate fixtures on one shared floor and verifies misses,
a genuine impact, and local support removal leave the other sleeping tower's full state unchanged.

## Reproduction

```sh
cargo test --locked --lib approximate::bookkeeping::
cargo test --locked --test fixed_step_approximation
cargo test --release --locked --lib sleeping_neighborhood_bookkeeping_scaling_benchmark -- --ignored --nocapture
cargo build --manifest-path demo-wasm/Cargo.toml --target wasm32-unknown-unknown --release --locked
node scripts/benchmark-prepared-response.mjs base.wasm candidate.wasm paired.json --bookkeeping
```

The paired runner compares the same solver with baseline and candidate modules. All twelve
free/upright sphere/arrow/rigid direct-hit/near-miss combinations run twice in alternating order
after module warmup. It hashes every observable pose/quaternion, linear velocity, sleep flag,
and pre-existing report counter (0..13); matches do not imply universal cross-platform identity.
Separate native tests compare indexed traversal with the original full-edge-scan oracle and
cover lifecycle, wake/retirement, support removal, changing timesteps, reused IDs, no-op calls,
point storage and epoch overflow.

Timings include only physics calls. Settling, non-quiescent post-shot work, and quiescent periods
are separate; raw repetitions are retained. Small near-miss traces contain few active ticks,
and sub-microsecond sleeping measurements are noisy. No timing threshold, solver limit, or
physical quality assertion is relaxed. Run the existing fixed-step hit/miss acceptance and
legacy structural performance contract as separate checks; paired hashes alone do not replace them.
