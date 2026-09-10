# physics-engine

Reusable deterministic physics simulation kernel extracted from the physics work that had accumulated inside `ecs-lab`.

The repository is intentionally a **physics engine, not a game engine**. It owns simulation semantics; ECS storage, rendering, input, audio, scenes, game loops and editor concerns remain consumers.

## Current foundation

The engine now has a deliberately narrow but real reusable core:

- 3D translational axis-aligned rigid bodies;
- fixed and dynamic body kinds with stable engine-local IDs;
- integer public state with a private Q32.32 continuous timeline;
- gravity and deterministic body ordering;
- swept AABB continuous collision detection and time of impact;
- globally earliest collision-event stepping with remaining-time continuation;
- deterministic overlap stabilization and mass-aware separation;
- restitution for dynamic/fixed and dynamic/dynamic impacts;
- bounded collision events per step;
- deterministic swept sweep-and-prune broad-phase candidate generation;
- snapshot overlap, swept-AABB and ray queries ordered by TOI and body ID;
- physics-native AABB and sphere collider geometry with exact integer contact evidence;
- deterministic fixed-point quaternion orientation and angular-velocity integration;
- exact box principal inertia ratios and off-center angular impulse evidence;
- integration tests specifically proving fast bodies do not tunnel through thin walls and broad-phase pruning does not change collision truth.

This is the clean ownership replacement for putting physics semantics directly inside `ecs-lab`. The existing ECS experiments remain valuable evidence and a migration source, but ECS entity snapshots are no longer part of the engine contract.

The current `World` simulation deliberately remains AABB-only and translational. Sphere support is present first as an independent collider/contact contract, and angular state is present as an independent deterministic rotational foundation. Neither implies mixed-shape or rotational collision response until those semantics are migrated and validated explicitly.

## Example

```rust
use physics_engine::{BodyId, RigidBody, Vec3i, World, WorldConfig};

let mut world = World::new(WorldConfig {
    gravity: Vec3i::new(0, -10, 0),
    ..WorldConfig::default()
});

world.add_body(RigidBody::dynamic(
    BodyId(1),
    Vec3i::new(0, 20, 0),
    Vec3i::new(0, -30, 0),
    Vec3i::new(1, 1, 1),
))?;
world.add_body(RigidBody::fixed(
    BodyId(2),
    Vec3i::ZERO,
    Vec3i::new(100, 1, 100),
))?;

let report = world.step(1)?;
# Ok::<(), physics_engine::PhysicsError>(())
```

## Ownership boundary

```text
ECS / game / simulation consumer
          |
          v
    physics-engine
          |
          +-- body/world state
          +-- collider geometry
          +-- broad phase
          +-- collision detection
          +-- CCD / time of impact
          +-- contact response
          +-- rotational state / inertia
          +-- spatial queries
          +-- constraints and solvers (next)
```

`physics-engine` must never depend on `ecs-lab`, a renderer, Three.js/WebGPU, or a game runtime.

## Next extraction slices

The existing `ecs-lab` experiments already contain useful evidence for more advanced physics. They should move here incrementally rather than being copied wholesale with ECS ownership attached:

1. friction and persistent/resting contact constraints;
2. OBB geometry and exact contact seeds;
3. rotational contact search, response and repeated-event stepping while preserving the explicit sampled-versus-analytic CCD boundary;
4. physics-native collider attachment plus sphere/sphere and sphere/OBB response and mixed-shape continuous collision detection;
5. joints/constraints and sleeping/islands;
6. a thin ECS adapter that maps entity IDs/components to engine bodies;
7. WASM bindings so browser demos execute the Rust engine directly.

The advanced slices should preserve the same rule as the current CCD path: calculate motion over the interval and resolve the first genuine event rather than relying on frame-end overlap. Sampled rotational search must remain explicitly described as sampled until analytic rotational CCD is actually implemented.

## Validation

`Validate` runs the repository's coding-tooling fast tier. GitHub Pages hosts an interactive CCD explainer; it is a visualization of the contract, not an independent authoritative physics implementation.
