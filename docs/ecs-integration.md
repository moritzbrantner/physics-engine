# ECS-first physics integration

The default public rotating-world integration is `RotatingWorld3d`, which is an ECS-backed world.

## Ownership

- **Entity/component storage owns application-facing physics state.** `BodyId` is the stable entity identity and each live physics entity carries a `RigidBox3d` component.
- **The physics solver is a system resource.** Collision discovery, continuous stepping, contact response, inertia, friction, stabilization, sleeping and broad-phase caches remain authoritative in the existing stabilized rotating solver.
- **Successful physics steps write results back into components.** Rendering and gameplay code read entity/component state rather than taking ownership of solver internals.
- **The raw solver is opt-in.** `PhysicsWorld3dKernel` exists for tests, adapters and deliberately low-level integrations. New game/demo code should use `RotatingWorld3d` unless it has a concrete reason not to use ECS ownership.

This boundary keeps the physics engine deterministic and reusable while making ECS the normal integration model for games with many independently moving, colliding and sleeping objects.

`ecs-lab` remains the experiment and benchmark harness for ECS storage strategies. The production physics crate does not take a runtime dependency on that laboratory; storage experiments can evolve independently while the entity/component/system boundary stays stable.
