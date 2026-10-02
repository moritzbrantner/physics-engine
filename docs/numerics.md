# Floating-point numerical policy

Decision: 2026-09-22. There is no "no floating point anywhere" rule.

## Default and reference arithmetic

`physics_engine::numeric::Scalar` is `f64`. Use it for new CPU simulation state, time,
velocities, contact constraints, impulses and solver math. `f32` is permitted where its
precision is appropriate, notably GPU and compact storage boundaries. Do not add a runtime
precision toggle: a simulation build must have one documented numerical contract.

The normal crate and Pages build use `f64` for repeated-time composition and shared
multiply/divide helpers used by angular integration, normal/friction impulses and ballistics.
The scale is one 8-byte float, not two 40-limb integers. There is no hidden multi-limb fallback.

The optional Cargo feature `exact-reference` selects the historical arithmetic for diagnostic
replays. It is not enabled by default, not required for deterministic ordering, and not a
recommended production mode. Pages explicitly rejects an artifact using that feature.
`sandbox_numeric_backend()` returns 64 for the normal build and 0 for the reference build.
Build provenance and sandbox benchmark evidence name the backend.

## What has and has not migrated

This first migration changes time composition and shared scaled arithmetic. Existing `Vec3i`,
material coefficients, quantized orientation and integer SAT contact APIs remain compatibility
surfaces. Some fixed-width `i128` geometric calculations therefore still exist. They are not
an instruction to implement new state or solver code with integers.

`RigidBoxFreeFlightConfig3d::try_from_seconds` accepts a checked floating-point duration directly.
The numerator/denominator constructor remains supported, but the production implementation
converts that input to f64 instead of preserving a growing fraction.

A numerical-backend change can change a quantized pose, retirement tick or replay hash. We do
not require bit-identical trajectories between the exact-reference and floating-point builds.
Within one supported build and execution target, fixed inputs and stable ordering must remain
repeatable. Cross-target or cross-compiler bit identity is a separate acceptance claim requiring
explicit native/WASM and architecture coverage; merely using either floats or integers does not
establish that claim.

The arithmetic migration did **not** replace the box-contact solver or cure the legacy rotating-event
tower failure. Later work moved the canonical Tower Stability runtime to `approximate::World` with
bounded fixed-contact position correction; its recorded volley matrix passes. This remains a specific
acceptance result. [Admitted-contact position correction](dense-contact-correction.md) now repairs one reproduced dense impact class under an explicit shared pass budget; the broader contact-quality suite remains incomplete. The separate legacy frame-38 event-limit
regression is now repaired by [contact continuation](legacy-contact-continuation.md), with its original
240-frame finite-floor acceptance and 64-event cap. See [the tower runtime](tower-stability.md).

## f32 floating-physics build

Decision: #258 / #260. For game-scale consumers, f32 is the production floating-physics target;
f64 stays the default build and the reference/diagnostic contract.

The Cargo feature `f32-physics` is a **whole-build choice** (non-additive by design): it switches
`physics_engine::approximate::Real` from `f64` to `f32` (`approximate::REAL_BITS` is 64 or 32).
Every physical field of the floating solver (`approximate::World`) uses `Real`: vectors,
quaternions, body mass/material and shape dimensions, `Config`, `Report` and query results.
There is no runtime toggle and no generic `World<T>`. Combining it with `exact-reference` is a
`compile_error!`. `demo-wasm` and Pages stay on the default f64 build.

What stays f64 in both builds:

- `numeric::Scalar`, the legacy translational/rotating worlds and `float_math` time composition
  (bit-identical with and without the feature);
- the `step(dt)` argument, accumulated elapsed seconds and the checkpointed prior-substep
  duration. Each substep narrows its `dt` to `Real` once;
- the `geometry-kernels::primitive3` kernels.

**Kernel boundary.** `src/approximate/primitive.rs` widens `Real` inputs exactly to f64
(`widen`) and narrows results conservatively: time of impact and sweep fractions toward zero, so
an impact never moves later (`toward_zero`); bounds/extents outward (`round_up`); separation toward
smaller (`round_down`); directions and witnesses to nearest (`nearest`). The f32 build renormalizes
the widened quaternion in f64 before building the kernel frame. A swept impact is re-checked after
narrowing with an allowance of `NARROWING_ULPS` (16) `Real::EPSILON`, scaled by the pose magnitude,
which is zero in the f64 build. In the default build every conversion is the identity.

**Precision envelope.** Supported f32 inputs (documented, not enforced by new rejections):

| Quantity | f32 envelope |
| --- | --- |
| position component | \|x\| ≤ 16384 units (ulp ≤ 0.002 ≈ default `contact_slop` 0.02 / 10) |
| linear speed | ≤ 4000 units/s with default substeps |
| gravity | default `(0, -3600, 0)` |
| shapes | dimensions ≥ 0.25 units |

Inputs outside the envelope remain accepted under the existing finite/range validation but are
unsupported for f32. `tests/f32_precision_envelope.rs` holds the adverse controls at the edge
(resting crate at x = z = ±16384 and at the origin; a CCD sphere of radius 0.25 at 4000 units/s
against a 0.5-unit fixed wall) with the same physical thresholds as the f64 build.

**Checkpoints.** Same format 2 / algorithm 7 layout, with `Real` values at native width under a
distinct magic; the other width is rejected with `CheckpointError::ScalarWidthMismatch`. See
[floating checkpoints](floating-checkpoints.md#scalar-width-f32-physics).

**Sizes** (x86_64 Linux, measured on the #260 branch):

| | f64 (default) | f32-physics |
| --- | --- | --- |
| `size_of::<approximate::Body>()` | 368 B | 200 B |
| empty-world checkpoint | 236 B | 200 B |
| settled 32-crate tower + floor checkpoint (240 steps) | 55,264 B | 33,716 B |

**Tests.** The f64 build is unchanged. Under `f32-physics`, only tests asserting f64 bit patterns
are excluded (the f64 empty-world wire digest; the f32 build has its own digest test). Tolerances
that only express rounding are scale-aware via `Real::EPSILON` and reduce to the previous f64
values in the default build. Physical thresholds (penetration, settling, jitter, sleep, CCD
no-tunneling) are identical in both builds. CI runs `cargo test --locked --features f32-physics`.

## Safety and tolerances

Reject NaN, infinity and negative durations at input boundaries. Reject a computed non-finite
value before converting it to legacy integer state. Float-to-integer casts must check the
exclusive positive integer endpoint; Rust's saturating cast must not hide an overflow.

An explicit zero duration is valid. Positive time composition underflowing to zero is a range
failure, not an implicit successful zero-duration step. Swept bounds round outward so rounding
cannot narrow the volume admitted to collision detection. The floating world uses
[separate numerical candidate guards](floating-conservative-bounds.md) for cached
pose extents and swept endpoint addition, independent of physical contact slop. This conservative padding is not
allowed to become a position correction or an invented physical contact.

Use a documented absolute/relative tolerance to compare mathematically equivalent numerical
expressions. Keep exact comparisons for IDs, masks, counters, no-op state and repeated execution
of the same input sequence. Do not weaken a collision, tunneling, support or failure-propagation
test merely to accommodate the arithmetic migration.

## Box solver direction

The replacement box solver should retain floating-point state across the step, use persistent
multi-point contact manifolds and accumulated normal/friction impulses, and warm-start a bounded
constraint solve. Quantize only at an explicit compatibility or serialization boundary, not on
every solver iteration. Stable BodyId ordering and Rust/WASM ownership remain requirements.

Resting support should not repeatedly consume a chronological impact-event budget. Use targeted
swept/continuous collision handling for fast spheres, arrows and other projectiles; an arithmetic
change is not permission to replace swept collision checks with frame-end overlap checks.

## Verification

```sh
cargo test --locked
cargo clippy --all-targets -- -D warnings
cargo test --locked --features exact-reference
cargo test --locked --features f32-physics
cargo test --manifest-path demo-wasm/Cargo.toml --locked
cargo build --manifest-path demo-wasm/Cargo.toml --target wasm32-unknown-unknown --release --locked
node scripts/benchmark-tower.mjs demo-wasm/target/wasm32-unknown-unknown/release/physics_engine_demo.wasm tower.json
```

The tower matrix uses the real Pages reset and projectile exports for physical/linear character
modes, free/upright crates, and sphere/arrow/rigid projectiles. Each case runs twice by default,
checks the observable replay, verifies that the projectile changes the crates, and records failed
steps and raw timings. A failure sets a nonzero exit status. `--report-only` is an explicit
investigation mode; it writes the same failures and does not relabel them as passes. This legacy event-solver matrix
remains a separate failing diagnostic. Canonical floating-state acceptance runs through
`node scripts/test-tower-runtime.mjs engine.wasm tower-runtime.json`, as required by the Pages build.
Neither matrix substitutes for the other.

Wall-clock measurements are advisory, recorded separately from behavioral assertions. Do not
claim a speedup for an incomplete or failed simulation compared with a completed one.

### Ballistic sphere response mass

`BallisticSphere3d` keeps its integer constructor and `mass_units()` getter for compatibility.
`response_mass()` supplies the inertial mass used by contact response; `with_response_mass()`
accepts a finite `Scalar` in `[1e-6, 1e12)`, in the same relative units as rigid bodies.
Whole-unit defaults preserve the existing response arithmetic. Fractional response masses
use floating-point inverse mass and checked rounding only at the remaining integer impulse
and velocity boundaries. Invalid masses and arithmetic overflow return errors.

The legacy sandbox chooses sphere mass from equal nominal density: a radius-three sphere
relative to a mass-two, side-36 crate has mass `2 * PI / 1296`. This demo choice does not
change the canonical floating-point tower's independently configured projectile masses.
