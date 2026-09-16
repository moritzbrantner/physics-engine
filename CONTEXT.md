# Repository context

`physics-engine` is the reusable simulation authority extracted from physics experiments that were previously hosted in `ecs-lab`.

The initial implementation intentionally focuses on deterministic translational AABB physics and continuous collision detection. Public positions and velocities are integer vectors. Within each requested step, positions are represented on a private Q32.32 timeline; the solver finds the globally earliest swept-AABB event, advances to it, resolves contact, then continues through the remaining interval.

The repository is not a game engine. It must remain usable by ECS experiments, games, browser/WASM applications, simulations and tooling without importing their ownership models.

The runtime architecture has one authoritative mutable world. Simulation changes are expressed and committed as explicit deltas; search/frontier/solver structures must not own complete world copies. Snapshots are explicit caller-requested products for persistence, replay/debug checkpoints, fixtures or export and must not be used as an ordinary stepping/rollback mechanism. Derived collision state should persist and be invalidated from precise changed-body IDs. See `docs/world-state-architecture.md`.

Near-term migration should pull in the already-proven angular/OBB/rotational-CCD work from `ecs-lab`, but only after expressing it in physics-native types. `ecs-lab` should ultimately depend on this repository through a thin adapter rather than remaining a second physics authority.
