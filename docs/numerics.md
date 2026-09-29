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
acceptance result, not a general dense-contact repair. See [the tower runtime](tower-stability.md).

## Safety and tolerances

Reject NaN, infinity and negative durations at input boundaries. Reject a computed non-finite
value before converting it to legacy integer state. Float-to-integer casts must check the
exclusive positive integer endpoint; Rust's saturating cast must not hide an overflow.

An explicit zero duration is valid. Positive time composition underflowing to zero is a range
failure, not an implicit successful zero-duration step. Swept bounds round outward so rounding
cannot narrow the volume admitted to collision detection. This conservative padding is not
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
