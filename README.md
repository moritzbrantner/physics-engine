# physics-engine

A physics engine for the owner's games, built to match and beat [Rapier](https://rapier.rs) on the
capabilities those games actually use.

The owner has selected Physics Engine as MMORPG's default for supported zones, while Rapier
remains the fallback for missing capabilities and the behavioural oracle. ARPG continues to use
Rapier until separately migrated. This repository implements physics capabilities on demand,
using the games' actual requirements and comparisons against Rapier (scenario invariants and
performance gates). See [technical decisions](docs/technical-decisions.md); the MMORPG switch is
tracked in its own repository rather than inferred from this policy.

## Shape of the engine

- **One world:** `approximate::World` with `f32` physical state. The legacy integer `World` and
  `RotatingWorld3d` were removed; their consumers stay on their old git pin until they move to
  Rapier.
- **Opt-in checkpoints:** versioned physical checkpoints for debugging and logging behind the
  `checkpoints` Cargo feature.
- **Own API:** primitives shaped by consumer usage rather than a Rapier clone — batched
  multi-character moves, stable never-reused ids, depenetration reported separately,
  length-unit-relative tolerances, stable-id tie-breaking, plane constraints and grounded state.
- **Rapier as oracle:** `oracle/` runs each parity scenario on both engines; CI fails when an
  invariant breaks or Physics Engine is slower than Rapier — the consumer's call pattern or bare
  Rapier — by more than 5% natively or 10% in WASM. Each engine's oracle WASM has a byte budget in
  `.performance/size.json`, judged by `coding-tooling performance size`.
- **Complexity contracts:** `.performance/contract.json` declares each public operation's
  Big-O class, measured with deterministic test-only work counters; CI fails on a violation.
- **Pages comparison lab:** [open the scenarios](https://moritzbrantner.github.io/physics-engine/)
  to select a consumer capability and replay its Rust-authored oracle observations side by side
  with Rapier, including invariant status and optional exploratory WASM timing. The separate
  [tower lab](https://moritzbrantner.github.io/physics-engine/tower.html) retains free-play
  shooting. The comparison plots observations, not collider meshes, and the authoritative native
  and WASM performance/correctness gates remain in CI.

Consumer API, units and query conventions: [world API contract](docs/world-api-contract.md).
Precision rules: [numerics](docs/numerics.md).

## Example

```rust
use physics_engine::approximate::{Body, Config, Shape, Vector, World};

let mut world = World::new(Config::default())?;
// The game's entity id 7 is the body's key; the engine returns a generational handle.
let mut body = Body::keyed(7, Shape::Box(Vector(1.0, 1.0, 1.0)), Vector::ZERO, 1.0);
body.velocity = Vector(60.0, 0.0, 0.0);
let id = world.add_body(body)?;
world.step(1.0 / 60.0)?;
assert_eq!(world.body_id(7), Some(id));
# Ok::<(), physics_engine::approximate::Error>(())
```

## Ownership boundary

Physics Engine owns body/world state, collision detection, CCD, contact response, sleeping and
spatial/character queries. Domain-neutral geometry kernels come from
[`rust-kernels`](https://github.com/moritzbrantner/rust-kernels). ECS storage, rendering, input,
game loops, scenes and gameplay rules stay in consumers; the engine never depends on them.

## Development

```sh
cargo fmt --all -- --check
cargo clippy --all-targets --features checkpoints --locked -- -D warnings
cargo test --locked --features checkpoints
bash scripts/build-pages.sh
```

[AGENTS.md](AGENTS.md) lists the full command set and repository rules.
