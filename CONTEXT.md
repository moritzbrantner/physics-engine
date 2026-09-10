# Repository context

`physics-engine` is the reusable simulation authority extracted from physics experiments that were previously hosted in `ecs-lab`.

The initial implementation intentionally focuses on deterministic translational AABB physics and continuous collision detection. Public positions and velocities are integer vectors. Within each requested step, positions are represented on a private Q32.32 timeline; the solver finds the globally earliest swept-AABB event, advances to it, resolves contact, then continues through the remaining interval.

The repository is not a game engine. It must remain usable by ECS experiments, games, browser/WASM applications, simulations and tooling without importing their ownership models.

Near-term migration should pull in the already-proven angular/OBB/rotational-CCD work from `ecs-lab`, but only after expressing it in physics-native types. `ecs-lab` should ultimately depend on this repository through a thin adapter rather than remaining a second physics authority.
