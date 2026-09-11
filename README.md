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
- globally earliest translational collision-event stepping with remaining-time continuation;
- deterministic overlap stabilization and mass-aware separation;
- restitution for dynamic/fixed and dynamic/dynamic impacts;
- bounded translational collision events per step;
- deterministic swept sweep-and-prune broad-phase candidate generation;
- snapshot overlap, swept-AABB and ray queries ordered by TOI and body ID;
- physics-native AABB and sphere collider geometry with exact integer contact evidence;
- deterministic fixed-point quaternion orientation and angular-velocity integration;
- exact box principal inertia ratios and off-center angular impulse evidence;
- quantized oriented-box vertices and exact SAT contact seeds with stable support masks;
- conservative rotational sweep bounds for endpoint-bounded translational intervals;
- engine-native rotating cuboid state over the existing translational `RigidBody` contract;
- direct deterministic rotating-box free-flight sampling with acceleration-aware conservative sweep bounds;
- deterministic rotational broad-phase candidate pairs;
- sampled rotating OBB first-contact search plus strictly-positive clear/re-contact search;
- shared first-contact and strictly-positive re-contact frontier reconstruction;
- deterministic OBB/frontier response with coupled simultaneous-contact handling and precision-preserving angular inertia math;
- bounded repeated sampled rotating-event advancement with exact rational remaining-time reduction;
- integration tests specifically proving fast bodies do not tunnel through thin walls and broad-phase pruning does not change collision truth.

This is the clean ownership replacement for putting physics semantics directly inside `ecs-lab`. The existing ECS experiments remain valuable evidence and a migration source, but ECS entity snapshots are no longer part of the engine contract.

The current `World` simulation deliberately remains AABB-only and translational. Rotational state, OBB contact geometry, rotational broad phase/search/frontiers, frontier response, and repeated sampled-event advancement are separate engine-owned foundations that do not change the established translational contract.

The rotational contact path is deliberately sampled. First-contact search can detect and refine contact observed by its coarse grid. Re-contact search adds the state transition needed by repeated events: a pair already touching at the interval start is ignored until a coarse sample proves it clear and a later sample observes contact again. Persistent time-zero contact therefore cannot hide every later event merely by being rediscovered first. This still does not become analytic rotational CCD: a clear interval or contact island that exists wholly between adjacent coarse samples can be missed, and refinement only sharpens an already observed clear/contact bracket.

First-contact and re-contact hits use one shared frontier reconstruction authority. A persistent pair may be ineligible to select the next positive event, but if it is still touching when another pair selects that event, it is included again in the shared frontier and remains visible to simultaneous response.

`advance_repeated_rotating_events` now chains those foundations vertically. It resolves the first admitted frontier, reduces the remaining timestep exactly, searches the new segment for a strictly-positive re-contact frontier, resolves it, and repeats under an explicit event bound. Recorded event times are segment-relative. If no later sampled event is found, the function returns the exact unconsumed rational tail instead of free-flying to frame end.

That explicit tail is intentional. The engine does not yet own persistent/resting-contact stabilization across an arbitrary remaining segment, so “no more sampled event found” is not treated as proof that free flight is safe. The next solver layer is responsible for consuming that tail while maintaining resting-contact constraints.

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
          +-- OBB contact geometry
          +-- rotational/free-flight sweep bounds
          +-- direct rotating-box free flight
          +-- sampled first-contact / re-contact search
          +-- shared first-contact / re-contact frontier
          +-- coupled rotating frontier response
          +-- repeated sampled event advancement
          +-- exact unconsumed tail
          +-- spatial queries
          +-- persistent-tail solver (next)
```

`physics-engine` must never depend on `ecs-lab`, a renderer, Three.js/WebGPU, or a game runtime.

## Next extraction slices

The existing `ecs-lab` experiments already contain useful evidence for more advanced physics. They should move here incrementally rather than being copied wholesale with ECS ownership attached:

1. persistent/resting contact stabilization that safely consumes the exact remaining tail;
2. friction and fuller contact-manifold constraints;
3. physics-native collider attachment plus sphere/sphere and sphere/OBB response and mixed-shape continuous collision detection;
4. joints/constraints and sleeping/islands;
5. a thin ECS adapter that maps entity IDs/components to engine bodies;
6. WASM bindings so browser demos execute the Rust engine directly.

The advanced slices should preserve the same rule as the current CCD path: calculate motion over the interval and resolve the first genuine event rather than relying on frame-end overlap. Sampled rotational search must remain explicitly described as sampled until analytic rotational CCD is actually implemented.

## Validation

`Validate` runs the repository's coding-tooling fast tier. GitHub Pages hosts an interactive CCD explainer; it is a visualization of the contract, not an independent authoritative physics implementation.
