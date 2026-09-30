# Checked primitive translation searches

The floating world's capsule/wedge sweep adapter uses the shared
`geometry-kernels::primitive3::try_swept_time` query. Its existing limit remains
128 conservative advances or swept-SAT axes. A separating-plane miss returns
`Ok(None)`; exhausted search or non-finite computation returns an error. The
engine never converts a shared-kernel search error into a collision miss, retries
with a larger budget, admits a contact, or discards the requested interval.

`World::step` returns `Error::CollisionSearchFailed { bodies, reason }` with the
pair in stable BodyId order. `SweepFailure` distinguishes invalid geometry input,
non-finite computation and iteration exhaustion. Integrated force/impulse values
can become invalid even when a public command's original inputs are finite. Such
a checked-sweep failure returns through the existing atomic step boundary: poses,
velocities, sleep state, pending inputs, contact history, elapsed time and the
previous successful report are restored. Repeated attempts preserve those inputs.
The failed-work report includes broad/narrow tests, actual search iterations and
`GeometryStats::primitive_sweep_failures`; it publishes no successful events.

This repair covers the shared capsule/wedge translation searches. The optimized
engine sphere/sphere, sphere/box and box/box analytic lanes retain their existing
behavior; this does not establish complete extreme-scale numerical coverage for
those lanes. Fixed-orientation substeps still defer secondary ricochets, and
angular motion is not continuously swept. See [the floating-world limitations](fixed-step-approximation.md).

## Evidence and scope

The upstream regression uniformly scales a capsule/sphere analytic hit by
`1e160`: normalized contact time is 0.25, but the current squared-distance kernel
overflows. Its former Option result was a false miss; the checked result is an
explicit numerical failure. These dimensions exceed the engine's supported body
ranges and are not a claim that extreme-scale geometry now works. Six upstream
tests cover budget exhaustion, numerical failure, stationary/touching cases,
separating misses, swept-SAT budgets and all ordered primitive pairs.

Two engine regressions exercise the actual public command/step seam. A finite
force produces a non-finite capsule velocity, which previously panicked in debug
builds; a second case produces finite opposing velocities whose relative motion
overflows. Both now return ordered pair/reason errors, retain checkpoint bytes
across three failed attempts and count discarded work. After cancelling the
unsupported force, continuation matches a control world, including an unrelated
pending impulse that applies once. The errors do not imply that these physical
inputs are supported.

Four million diagnostic capsule hit searches did not reproduce exhaustion of the
default 128-iteration budget. Upstream small-budget tests verify that an
insufficient search returns an error rather than a miss. A separate seeded
100,000-sweep old/new comparison, repeated three times over all 16 ordered pairs
and rotated poses, retained identical result/work digest `9ab30cc7707bb5f7`,
121,814 search iterations and 35,957 SAT axes. Advisory aggregate kernel medians
were 37.382 ms before and 38.828 ms after (about 3.9% validation overhead).

The fast tier, all-feature Clippy, default/all-feature failure and checkpoint tests,
native/WASM continuation driver and full Pages matrix pass. All 31 existing
performance fences retain their work and correctness values; no history epoch,
fixture, quality threshold or budget was changed.

This is one bounded part of [issue #3](https://github.com/moritzbrantner/physics-engine/issues/3).
Motion-class acceptance, chronological same-substep ricochets and conservative
angular guarantees remain separate work. It does not close the CCD roadmap.
Shared convention source: `e6acb5310afaf15c0cba24f87108f5f4ad1bedc3`.
