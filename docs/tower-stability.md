# Canonical Tower Stability runtime

`scenarios/tower/` now runs the bounded floating-state solver. The legacy event solver
is available explicitly on `scenarios/fixed-step/?solver=event`; it is not an automatic
fallback. General Sandbox is unchanged. Missing tower exports cause a visible build
compatibility error rather than silently loading an old solver.

## Actual scene and controls

`approximate_reset_tower(rules)` constructs the existing 32-crate/player/11-fixed-body
fixture in Rust, imports it once, and selects four substeps and eight maximum velocity
iterations. No legacy event step or integer pose write-back is used afterward. The
browser receives f64 snapshots and unit quaternions and reuses the first-person renderer.
The constructor's legacy world is not used for runtime telemetry.

WASD, mouse/arrow aiming, jump, F/click firing, 1/2/3 projectile selection, pause, step,
reset, fullscreen and local performance recording remain available. Changing projectile
type affects only future shots, including while earlier projectiles are in flight. Each
live projectile retains its own render kind. The existing cap remains 48. Retirement,
departure, eviction and reset all remove the matching adapter metadata.

Character response, crate rotation, impact-retire/inelastic/bounce, and collision masks
remain Rust-owned rules. Unsupported event-solver baking and per-pair stabilization
budgets are hidden/disabled and are not represented as active fixed-step settings.
Disabling a collision pair deliberately removes that interaction, including floor support
when World/Crate is disabled. Old `solver=event` parameters do not switch this canonical
page to the failed event path; use the explicit comparison link instead.

## Residual fixed-contact overlap

Single settled-shot tests were insufficient for a first-person interactive tower. Rapid
mixed volleys exposed residual fixed-world penetration. Merely increasing substeps or
velocity iterations was more expensive and did not consistently resolve it.

The canonical tower explicitly selects **two maximum fixed-position passes per substep**.
This optional `Config::fixed_position_iterations` remains **zero** for core/comparison
callers, preserving their established default. After normal pose integration, each pass
checks awake dynamic bodies against collision-enabled, non-sensor mass-zero colliders.
AABB rejection precedes a fresh geometric manifold. If its minimum separation is below
negative contact slop, the dynamic body's position receives:

```
position += fixed_to_dynamic_normal * max(penetration - contact_slop, 0)
```

The fixed body's pose and both velocities remain unchanged. This is a position projection,
not a second time integration, teleport-to-ground rule, new CCD algorithm, extra damping,
or an increase of the old event limit. Rotated fixed colliders use their own geometric
normal. Bounds are refreshed immediately; pose-dependent geometry is revalidated at the
next discovery pass. Material corrections reset the corrected body's quiet timer before
island sleeping. No awake body is manufactured for a near miss.

The two passes are a bounded approximation, not proof of nonpenetration in arbitrary
constraint configurations. They can change trajectories and potential energy. Swept
translation remains necessary and unmodified; rapidly rotating CCD and secondary
substep-ricochet limitations of the impulse solver still apply. The optional soft-contact
experiment in PR #163 is not required or enabled for this runtime.

## Acceptance and measurement

```
cargo test --locked --lib approximate::position::
cargo test --manifest-path demo-wasm/Cargo.toml --locked --lib approximate::
node --test site/tower-runtime.test.mjs site/performance-log.test.mjs
node scripts/test-tower-runtime.mjs engine.wasm tower-runtime.json
python scripts/test-tower-browser.py --url http://127.0.0.1:8765 --output browser-evidence
```

The runtime matrix imports the exact browser adapter and canonical reset, not a separate
lab reset. It covers both character modes, both rotation modes, three impact policies,
20-shot mixed repeated hits, first-flight near misses, an immediate arrow after reset,
and a 12-shot fast mixed volley. Repetitions compare every observed pose and sleep flag.
All 32 crates and the player/fixed bodies must remain present; finite normalized poses,
time advancement, actual hit response, rotation locks, bounded work and the existing
0.5-unit floor bound remain enforced. Retiring near misses must leave the sleeping tower
unchanged. For bouncing policies, later wall ricochets may genuinely hit it; first-flight
misses still must not activate it.

The Pages build runs the real WASM matrix. The browser script separately exercises the
actual canonical page, live type switching without reset, hits, misses, movement, jump,
pause/step/reset, disabled unsupported settings, performance export and mobile layout.
Its virtual clock controls callbacks, not physics state; browser acceptance timings are
not performance measurements. Node reports keep raw physics-only host call times including
post-sleep intervals. Different projection trajectories prevent interpreting timing ratios
as equal-work kernel throughput. No wall-clock CI gate or reduced physical threshold is used.
