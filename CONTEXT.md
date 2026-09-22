# Repository context

`physics-engine` is the reusable simulation authority extracted from physics experiments that were previously hosted in `ecs-lab`.

CPU simulation arithmetic defaults to `numeric::Scalar` (`f64`). There is no prohibition on floating point.
Time composition and shared scaled arithmetic use floats in the normal and Pages builds. The historical
multi-limb backend is opt-in under `exact-reference` for diagnostic replay only. See `docs/numerics.md`.

The original translational AABB path and current public body/geometry APIs still expose quantized integer
state as compatibility formats; these are not the design for new solver state. Fixed IDs, counters and
flags remain integers. Stable ordering and repeatable input traces remain important independently of the
number representation.

The repository is not a game engine. It must remain usable by ECS experiments, games, browser/WASM applications, simulations and tooling without importing their ownership models.

The runtime architecture has one authoritative mutable world. Simulation changes are expressed and committed as explicit deltas; search/frontier/solver structures must not own complete world copies. Snapshots are explicit caller-requested products for persistence, replay/debug checkpoints, fixtures or export and must not be used as an ordinary stepping/rollback mechanism. Derived collision state should persist and be invalidated from precise changed-body IDs. See `docs/world-state-architecture.md`.

Near-term work should retain floating-point state in a persistent-manifold, warm-started box-contact
solver with bounded constraint iterations and targeted projectile CCD. Free-rotating tower impacts are
still a failing acceptance case after the arithmetic migration; do not replace that gate with a passing
upright-only fixture or treat a number-type change as a contact-convergence repair.
