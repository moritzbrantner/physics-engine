# Agent guidance

## Authority

This repository owns reusable physics semantics. Keep ECS, rendering, game-loop, scene, editor and UI concepts outside the engine.

## Determinism

- Preserve stable `BodyId` ordering for pair traversal and simultaneous events.
- Keep authoritative state and collision decisions independent of JavaScript/rendering.
- Treat Q32.32 subticks as an implementation detail unless a public contract explicitly requires exposure.
- A collision repair must address tunneling/contact semantics rather than hiding a failing test or increasing an event limit without evidence.

## Continuous collision detection

Fast-moving bodies must be tested across the requested interval. Do not replace swept time-of-impact checks with frame-end overlap checks.

## Migration from ecs-lab

Move proven physics concepts in coherent slices and remove ECS-specific ownership while doing so. Do not bulk-copy experimental APIs merely to increase feature count.
