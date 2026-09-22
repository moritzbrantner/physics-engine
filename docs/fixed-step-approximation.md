# Fixed-step impulse approximation

This is an explicit alternative, not a silent replacement of `RotatingWorld3d`.
`physics_engine::approximate::World` retains f64 position, velocity, quaternion and angular
velocity across ticks. The existing sampled-event solver remains available for comparison;
it is more elaborate, not an exact ground-truth solution of rigid-body dynamics.

## The requested update rule

For an isolated body with mass m and tick duration dt:

```
v_next = v + dt * (gravity + sum(forces) / m) + sum(impulses) / m
x_next = x + dt * v_next
```

Forces last for one requested tick and impulses apply once. The resulting velocity persists
into the next tick. Off-centre impulses additionally apply inverse world inertia times
`(point - centre) cross impulse`; angular velocity advances a normalized quaternion.
There is no integer pose write-back during this path. IDs, masks and work counters stay integers.

A stack cannot calculate every contact impulse independently: changing one body's velocity
changes the velocity seen at its other contacts. The approximation therefore uses a **bounded**
sequential-impulse constraint solve. The default is four substeps and eight velocity iterations
per substep. Each iteration updates velocities using incremental impulses; accumulated normal
impulses clamp at zero, and accumulated tangential impulses clamp to the Coulomb friction disk.
Prepared contact points and effective masses are reused for those iterations. Local contact
anchors and impulses survive for warm starting on the next substep.

## One substep

1. Apply force/gravity to awake velocities. External impulses were accumulated once at tick start.
2. Query swept AABBs in a stable sweep-and-prune order. Reuse sleeping/static geometric bounds.
3. Prepare OBB SAT contacts with clipped face manifolds (at most four points); use one-point
   edge and sphere contacts. Only real contact or an admitted translation sweep wakes an island.
4. Retain the first/equal-time swept contacts of each CCD projectile, so a nearer wall shields
   a target even when the target's BodyId sorts first. Re-evaluate new motion next substep.
5. Apply cached impulses, run eight contact iterations, then integrate updated velocity and
   orientation for the entire substep. No chronological event restart or rational tail exists.
6. Cache contact anchors/impulses. Quiet connected dynamic islands sleep together. Fixed floors
   are boundaries, not bridges connecting unrelated sleeping islands. Retire impacted projectiles
   locally after applying their impulse. Unrelated near misses preserve sleeping poses.

Penetration uses a small Baumgarte velocity bias (0.2), default contact slop 0.02 scene units,
and a 60-unit/second correction-speed cap. These parameters are approximations, not proofs of
zero interpenetration or energy conservation. The reference fixture's crate width is 36 units.

## Explicit limitations

- Sweeps hold orientation fixed within each substep. This is continuous **translation** collision
  detection, not analytic rotational CCD. Fast spin and secondary ricochets within one substep
  are not fully resolved chronologically; choose a smaller step where needed.
- The API accepts finite durations from zero through 0.1 s; the demo uses 1/60 s. Positive durations
  too small to represent a substep are rejected. It does not adapt iteration counts to achieve
  an arbitrarily exact solution.
- This experimental API supports boxes and spheres, collision masks, fixed/external bodies,
  rotation locks, basic linear-support response and sleep. It is not a drop-in implementation of
  every existing interaction-policy setting, query, ECS binding or network serialization contract.
- Gyroscopic angular terms are omitted. Arrow orientation is fixed at launch in the comparison
  adapter; there is no aerodynamic or skeletal animation model.
- Floating state is repeatable in tested runs on one build/target. Cross-compiler and cross-target
  bit identity is not asserted. Hash differences between solver modes are expected.
- Finite input/range validation does not make arbitrary extreme-scale scenes supported. Mid-step
  numerical errors are reported; the experimental World does not promise transactional rollback.
- Passing a four-second impact trace is not proof of long-run settling. Free-rotation sphere and
  rigid-box cases can remain active at that boundary. Inspect that result separately from frame
  completion and bounded work.

## Existing solver vs approximation

The legacy path does already combine linear/angular corrections, but repeats them inside
`first contact -> simultaneous response -> stabilization -> recontact -> remaining-time tail`.
It quantizes state, can repeatedly rebuild/revisit contact geometry and stops when the event
budget is exhausted. Its arithmetic is now f64 by default for time/scaled helpers; integer geometry
and quantized body state remain in that path. `exact-reference` is a diagnostic numerical feature,
not an alternative promise that the event solver is physically exact.

## Reproduce and compare

```
cargo test --locked --test fixed_step_approximation
cargo test --manifest-path demo-wasm/Cargo.toml --locked --lib approximate::
cargo build --manifest-path demo-wasm/Cargo.toml --locked --release --target wasm32-unknown-unknown
node scripts/benchmark-fixed-step.mjs \
  demo-wasm/target/wasm32-unknown-unknown/release/physics_engine_demo.wasm \
  fixed-step-results.json [older-event-engine.wasm]
```

Both modes use the real Pages tower-reset exports. The approximation imports the same Rust fixture
once, then owns its own floating state. The matrix runs sphere/arrow/rigid hits and near misses,
free/upright rotation, and two repeats per case. It records completion, actual crate response,
rotation locks, finite poses, floor penetration (maximum 0.5 scene units), unchanged sleeping
near-miss poses, bounded iterations, hashes and raw timings. Reference failures remain failures
in the report and cannot produce a speedup ratio. Candidate failures exit nonzero.

Timings exclude WASM instantiation, setup/settling and observation/assertion code, and include
post-impact idle ticks. Averages across that entire window must not be presented as exclusively
active-solver time. Native tests separately verify momentum, applied forces, off-centre impulses,
thin-wall CCD, rounded-corner misses, wall shielding and support removal.

The dedicated Pages `scenarios/fixed-step/` comparison lets users select the experimental path
or the existing event solver without changing the ordinary sandbox's default behavior.
