# Projectile miss admission

The sleeping representation is not invalidated merely because a projectile is created or its
broad bounding envelope is near a tower. Rust owns a read-only admission test before wake-up:

1. Query the retained parked-body bounds index with the projectile's swept path.
2. Refine candidate envelopes with a conservative floating-point swept-separation test.
3. Preserve sleeping poses and avoid tower response/stabilization when every candidate is a
   proven miss. Possible hits continue through the authoritative CCD and response path.

`TranslationSweep` uses `numeric::Scalar` (`f64`), current geometry, velocity and acceleration.
It expands the start/end chord for acceleration curvature, integer compatibility rounding and
floating-point projection error. This covers between-endpoint hits and acceleration reversals;
it is not an endpoint overlap test. A sphere's circumscribed volume additionally covers the
analytic lane's unrounded target faces. This padding is admission evidence only: it never
creates a physical contact or changes a pose.

## Scope and conservative fallback

This slice proves misses for an isolated sphere with no awake rigid bodies, or a single
non-spinning/rotation-locked rigid body with no analytic projectiles. Ballistic admission
includes the current interval and one following interval, so an impending impact can
reactivate support constraints before contact. It does not wait until penetration occurs.

Coupled moving worlds, multiple projectiles, spinning bodies, ambiguous impacts and potential
ricochets retain conservative wake behavior. In particular, a predicted sphere hit still
uses the existing conservative restoration of the parked world. This is not yet a general
local contact-island activation system. Direct hits into the free-rotating 32-crate tower
still expose the existing error-611 convergence problem; the miss optimization is not its fix.

## Projectile lifetime is part of the proof

An impact-and-retire projectile cannot ricochet off a platform and hit a sleeping crate later
in the same step. Rigid arrows previously removed only at frame-end did not satisfy that
contract. `RigidBox3d::with_impact_retirement(true)` now retires an explicitly opted-in dynamic
body after its first admitted impact response and before the remaining interval is advanced.
Normal unflagged rigid bodies keep their ordinary lifetime and response.

Retirement is reported in the changed-body delta and `rigid_bodies_retired_on_impact` counter.
Body indices, collision candidates, sleep bookkeeping and interaction categories are updated
at that lifecycle boundary. Tail retry journals keep stable vector slots until the attempt
commits; inert retired slots are removed before publishing a result. A retry restores the
original body, kind, masks and motion, not a half-retired state.

Removing a body proven disconnected from all parked bounds does not wake the tower. Removing
an actual or ambiguous support retains conservative dependent wake-up. Creation, deletion,
zero-time steps and rejected inputs are tested independently from physical impact response.

## Evidence

```sh
cargo test --release --locked
cargo test --release --locked --features exact-reference
cargo test --manifest-path demo-wasm/Cargo.toml --release --locked
cargo build --manifest-path demo-wasm/Cargo.toml --target wasm32-unknown-unknown --release --locked
node scripts/benchmark-projectile-misses.mjs demo-wasm/target/wasm32-unknown-unknown/release/physics_engine_demo.wasm misses.json
node scripts/benchmark-tower.mjs demo-wasm/target/wasm32-unknown-unknown/release/physics_engine_demo.wasm impacts.json --report-only
```

The native miss tests compare every crate's complete body state, orientation, velocity and
sleep status over 120 ticks, including projectile retirement. The Pages-export matrix covers
left/right sphere, arrow and rigid near misses in both character modes, with two repetitions
per case. Historical binaries without sleep telemetry report that field as unknown rather
than inventing a passing sleep assertion. Native positive-impact tests verify wake-up and
momentum transfer; the separate full-tower impact matrix remains a failing diagnostic.

Wall-clock timings are advisory. Zero tower stabilization, unchanged crate state, continued
projectile motion/retirement, and unchanged real-hit behavior are the structural acceptance
criteria. No event budget, numerical-backend policy or existing performance threshold changes.
