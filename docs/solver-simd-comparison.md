# SIMD contact solver comparison (#545)

**Recommendation: stop. Do not adopt any of the three variants.** On the workloads that run the
contact solver, every variant is slower than the scalar solver on native and on WASM in nearly every
case, and the two that change solve order fail oracle invariants. Even with row preparation at zero
cost, the colored kernel would save only about 1–7% of a tick.

The prototype is on branch `agent/545-simd-solver-prototype` (`91627cc`), built on the #544 work. It
is not for merge. The full tables are in `docs/solver-simd-report.md` on that branch.

## Variants

All three copy the selected rows and the velocities they touch into compact arrays for one solve and
write them back afterwards. That cost counts against them.

| Variant | What it does | Trajectories |
| --- | --- | --- |
| `hot` | Compact rows and velocities, same order and arithmetic, scalar code | Bit-identical on 50/50 engine traces |
| `manifold` | Up to 4 points of one pair solved together (relaxed Jacobi), `f32x4` | Changed |
| `colored` | Greedy deterministic coloring into 4-wide body-disjoint batches, `f32x4` | Changed |

`wide` is the only SIMD dependency, named in one module, and needs no `unsafe`.

## Oracle invariants

No bound was loosened. The results are the same before and after the prototype's overhead fixes.

| Variant | Native | WASM (with and without `simd128`) |
| --- | --- | --- |
| scalar, `hot` | all pass | all pass |
| `manifold` | 13 scenarios fail | 14 fail (the same 13 plus `rubble-wake`) |
| `colored` | 2 fail: `debris-lane`, `debris-town` | 1 fails: `debris-town` |

On native, every scenario, including those with a failing invariant, had `BodyStorage::Indexed`
equal to `Ordered`: `verify` tests that before the invariants. The solver is
single-threaded, so worker count does not apply. Coloring is a function of stable row order only.

## Time

Each value is the median paired ratio to the scalar solver for the same round; below 1.00 is faster.
Native is against the scalar build, WASM against the current scalar build without `simd128`.

The machine was busy, so runs were interleaved: 15 native and 10 WASM rounds, each a fresh process
with the variant order shuffled, on CPUs 4–7, taking the best of 5 samples per round (1 sample for
`pile-1024` and `pile-4096`). Run-to-run spread (p90/p10 of a config's rounds) was often 1.3–1.6 and
reached 2.0, so treat differences below about 3% as noise. An earlier full run of the same matrix,
before the overhead fixes, gave medians within about 0.04 for most entries.

`mmorpg-tick` and `crowd` are omitted: they only move characters and never call `step`, so no
variant affects them (ratios 0.97–1.01).

| Workload | Native `hot` | Native `manifold` | Native `colored` | WASM+simd scalar | WASM+simd `hot` | WASM+simd `manifold` | WASM+simd `colored` | WASM no-simd `colored` |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `rubble-wake` | 1.11 | 1.33 | 1.10 | 0.99 | 1.10 | 1.43 | 1.23 | 1.49 |
| `debris` | 1.06 | 1.20 | 1.17 | 1.00 | 1.06 | 1.20 | 1.17 | 1.31 |
| `debris-town-tumble` | 1.10 | 4.14 | 1.23 | 0.97 | 1.08 | 3.79 | 1.22 | 1.33 |
| `pile-16` | 1.19 | 0.99 | 1.10 | 0.99 | 1.23 | 1.23 | 1.34 | 1.71 |
| `pile-128` | 1.17 | 1.02 | 1.06 | 0.99 | 1.18 | 1.23 | 1.23 | 1.57 |
| `pile-1024` | 1.09 | 1.13 | 1.08 | 0.98 | 1.07 | 1.17 | 1.10 | 1.31 |
| `pile-4096` | 1.07 | 1.05 | 1.05 | 0.99 | 1.03 | 1.07 | 1.07 | 1.21 |
| `sparse-4` (few rows, 2,000 bodies) | 1.15 | 0.96 | 1.14 | 1.00 | 1.12 | 1.08 | 1.18 | 1.28 |

The only wins are `manifold` on native small worlds (0.96–0.99), and that variant fails 13 scenarios.
Its 4× on `debris-town-tumble` is not solver speed: row building also tripled, because the changed
dynamics keep bodies moving. Enabling `simd128` for the scalar solver gains about 1%, which is within
noise, on these workloads.

### Per phase

Microseconds per tick from the `phase-timing` builds, whose timers add a little to `prepare`.
*Solve* covers warm start and the velocity passes, *prepare* the copy-in, grouping or coloring, and
copy-out. *Kernel* is solve minus prepare.

| Workload (native) | scalar solve | `colored` prepare | `colored` kernel | `hot` prepare | `hot` kernel |
| --- | ---: | ---: | ---: | ---: | ---: |
| `rubble-wake` | 9.50 | 4.87 | 8.00 | 1.26 | 11.91 |
| `debris` | 8.11 | 6.01 | 9.07 | 1.61 | 9.58 |
| `pile-1024` | 1256 | 631 | 962 | 166 | 1523 |
| `pile-4096` | 5720 | 3098 | 4379 | 847 | 6780 |

- The colored kernel is 16–23% faster than scalar on `rubble-wake` and the large piles, but prepare
  costs more than that saves. On `debris` the kernel is slower.
- Solve is 17–31% of a native tick on these workloads, so free preparation would give about 5%
  native on `rubble-wake` and `pile-4096`, 6.5% on `pile-1024`. On WASM+`simd128`, `rubble-wake`
  would gain about 1% and `pile-4096` about 5.5%.
- `hot` shows that **layout alone gives no gain**. Its kernel is 18–25% slower than scalar on those
  workloads despite identical arithmetic. The cause is not established: restoring the scalar path's
  mass-zero short-circuit did not change it, and `perf` is not permitted on this machine.

## WASM size

Engine-only release build, the same configuration `size-budget.sh` measures, in bytes. The prototype's
timing harness is included in all of them.

| Variant | no `simd128` | with `simd128` |
| --- | ---: | ---: |
| scalar | 1,743,457 | 1,720,858 |
| `hot` | 1,741,001 | 1,714,855 |
| `manifold` | 1,755,030 | 1,727,996 |
| `colored` | 1,787,581 | 1,750,108 |

`colored` adds 44,124 bytes without `simd128` and 29,250 with it, both over the 16,384 byte
`maxIncreaseBytes` advisory. These sizes also exceed `maxBytes` (1 MiB) for the scalar baseline.

## Friction

A wide kernel cannot keep `solve_row`'s f64 squares cheaply. The prototype uses f32 squares and one
wide `sqrt`. Impulses above about 1.8e19 then overflow to an infinite length and clamp to zero, which
is unacceptable under the numerical policy unless guarded. An adoption would need a decided, tested
replacement.

## Not measured

- `target-cpu` or AVX builds; the default x86-64 target was used, matching CI.
- Colorings cached across substeps, or a row-count threshold below which the solver stays scalar.
  Either could reduce prepare, but the ceiling above still applies.
- Rapier's arrangement of lanes per manifold with sequential points.
- Fixing `colored`'s failing invariants; the performance result did not justify that work.
- `scripts/size-budget.sh` itself; the sizes above were read directly from equivalent builds.

## If this is revisited

Only with a consumer scenario where solve dominates the tick. It would need a colored variant with
far cheaper preparation that passes every invariant. Start from the prototype branch.
