# Behavior policies

The physics engine distinguishes **what a body is allowed to do** from **which other bodies it may interact with** and **how expensive an interaction should be**.

## Authority boundaries

- `CollisionLayers3d` owns pair eligibility.
- `Material` owns restitution and friction.
- `SolverParticipation3d` owns whether a body enters rigid contact solving.
- `MotionAuthority3d` owns whether physics or the caller may mutate dynamic motion.
- `SleepMode3d` owns per-body settling aggressiveness.
- `InteractionPolicy3d` owns pair-specific work budgets such as fixed-boundary stabilization and wake propagation.
- Shape-specific paths such as ballistic spheres remain separate when they have genuinely different collision mathematics.

These dimensions compose; they are not alternative body hierarchies.

## Reusable behavior classes

### Sensors and triggers

Use `with_overlap_only()`.

The body remains available through `body_overlaps`, with collision layers still deciding which bodies count as eligible overlaps. It is omitted before sampled time-of-impact search and rigid-contact solving, so it cannot create impulses, penetration correction, stabilization, or repeated zero-time contact work.

### Kinematic / externally driven bodies

Use `with_external_motion()` on a dynamic body.

The caller-provided linear and angular velocity participates in relative contact velocity and conservative sweep bounds, but world gravity is not applied. The body contributes zero solver inverse mass and receives no normal impulse, friction impulse, or penetration projection. Dynamic bodies may therefore react to it without making the solver authoritative over its motion.

External motion implies `SleepMode3d::Never`.

### Debris and low-value clutter

Use `with_aggressive_sleep()` and, where appropriate, a pair policy with a low fixed-boundary stabilization pass limit and/or `WakePropagation3d::None` for interactions where a parked target should remain passive.

Aggressive sleep changes only the deterministic stable-duration requirement. It does not increase the low-motion velocity threshold and therefore does not freeze fast-moving debris.

## Policy-resolution rules

Interaction categories remain consumer-defined. Directional pair overrides take precedence over symmetric pair overrides, which take precedence over the default policy. Wake propagation is directional: suppressing `debris -> debris` does not automatically suppress `character -> debris`.

A no-wake target remains represented by its parked/fixed collision proxy. The policy avoids waking work; it does not remove collision geometry.

## Future dimensions

Additional pair knobs should be added only when an existing engine stage consumes them. Good future candidates include angular-response limits, CCD strategy, contact lifetime, collision-detail proxies, and one-sided solver response. Avoid adding configuration fields that do not yet alter authoritative work.
