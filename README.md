# physics-engine

A physics engine for the owner's games, built to match and beat [Rapier](https://rapier.rs) on the
capabilities those games actually use.

The games (ARPG, MMORPG, game-server) run on Rapier today. This repository records which Rapier
functions they rely on, provides them through its own performance-oriented API, and checks every
capability against Rapier: same scenario, behaviour invariants, and a CI gate that fails when
Physics Engine is slower than Rapier. See [technical decisions](docs/technical-decisions.md).

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
  invariant breaks or Physics Engine is more than 5% slower than Rapier natively, and reports
  WASM timings and sizes.
- **Pages sandbox:** `site/` + `demo-wasm/` show the parity scenarios side by side with Rapier.

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
