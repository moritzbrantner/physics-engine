# Agent guidance

## Authority

This repository owns reusable physics semantics. Keep ECS, rendering, game-loop, scene, editor and UI concepts outside the engine.

## Numerical policy

- Floating point is the default, not an exception. Use `numeric::Scalar` (`f64`) for new CPU physics math/state; use `f32` when a documented precision budget permits it.
- Do not impose a "no floating point anywhere" constraint. Do not grow exact multi-limb fractions to avoid ordinary floating-point error.
- The `exact-reference` feature is diagnostic only. Never enable it implicitly for a production or Pages build.
- Retain integers for IDs, flags, counters and explicit legacy serialization boundaries, not as a mandatory representation for continuous physical quantities.
- Validate finite values and ranges, use explicit scale-aware tolerances, and keep swept bounds conservative. Do not hide numerical failures with saturating casts.
- Follow `docs/numerics.md`. The remaining integer body/geometry APIs are compatibility surfaces awaiting the floating-state/contact-solver migration.

## Determinism

- Preserve stable `BodyId` ordering for pair traversal and simultaneous events. Define replay guarantees by build/target; do not equate exact arithmetic with physically accurate or automatically cross-platform deterministic simulation.
- Keep authoritative state and collision decisions independent of JavaScript/rendering.
- Treat Q32.32 subticks as an implementation detail unless a public contract explicitly requires exposure.
- A collision repair must address tunneling/contact semantics rather than hiding a failing test or increasing an event limit without evidence.

## Continuous collision detection

Fast-moving bodies must be tested across the requested interval. Do not replace swept time-of-impact checks with frame-end overlap checks.

## Migration from ecs-lab

Move proven physics concepts in coherent slices and remove ECS-specific ownership while doing so. Do not bulk-copy experimental APIs merely to increase feature count.

## Parked-body activation

- Projectile creation, broad-phase proximity, and unrelated projectile removal are not wake evidence.
- Admit swept/analytic contacts before restoring parked dynamics, and apply impulses only after real dynamic mass/inertia is active.
- Cover same-step ricochets and support removal. Fixed floors must not connect otherwise independent dynamic islands.
- Include discarded wake-probe work in performance evidence; follow `docs/contact-wake.md`.
