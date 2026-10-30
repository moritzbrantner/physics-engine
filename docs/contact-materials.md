# Floating contact material and mass response

This is the existing `approximate::World` policy, verified through its public body,
command and step APIs for #221 (parent #193). It does not change solver formulas.
The legacy translational and rotating worlds have their own material contracts.

`Body::friction` defaults to 0.5 (Rapier's collider default) and accepts finite values in `[0, 10]`.
`Body::restitution` defaults to zero and accepts finite values in `[0, 1]`.
Invalid material or mass input fails admission without modifying physical state or
consuming pending forces. Zero mass denotes immovable geometry; positive dynamic
mass is at least `1e-6` and below `1e12`. These are independent coefficients;
shape volume does not automatically choose density, mass or material.

## Combine rules

A contact combines the two bodies' coefficients by their `friction_combine` and
`restitution_combine` rules (`CombineRule`, default `Average`), as Rapier's
`CoefficientCombineRule` does: `Average` is `(a + b) / 2`, `Min`, `Multiply` and `Max` as
named. When the two bodies' rules differ the later one wins (Average < Min < Multiply < Max),
Rapier's precedence. Rapier's `ClampedSum` and `GeometricMean` are not offered: no consumer
uses them. Engine-built bodies take a `Material` (`HeightfieldDesc::material`).

## Normal impacts

The pair restitution coefficient is the combined coefficient.
Restitution uses contact-point relative normal velocity, including permitted spin.
Its target separating speed is `e * closing_speed` only when closing speed is
strictly greater than **1 m/s** (`Config::length_unit` scene units per second), witness
separation is at most `Config::contact_slop`, and the pair has no retained contact-history entry.
This speed threshold does not scale with mass, dimensions or timestep.

The history gate applies even with `warm_start: false`; that option disables
reapplication of old impulses, rather than contact history. A clear interval which
removes the pair's history permits a new bounce. Removal/same-ID replacement also
invalidates that history. Persistent contact does not receive repeated restitution
targets just because another force or command drives its endpoints together.

Separated speculative constraints always use `-separation / substep_seconds` as
their normal target, allowing arrival at the surface without a premature bounce,
even inside slop. For penetration beyond slop, the existing Baumgarte target is
`min(0.2 * excess_penetration / substep_seconds, 60 scene units/s)`; a restitution
target can raise that target. Optional diagnostic softened correction remains a
separate experiment in #158.

For an isolated, central, touching two-body impact with no friction, forces or
penetration bias, the analytic impulse is
`J = (1 + e) * closing_speed / (1 / mass_a + 1 / mass_b)`.
Linear momentum is conserved and lost kinetic energy is
`0.5 * reduced_mass * (1 - e^2) * closing_speed^2`.
The fixed-body case uses zero inverse mass, so momentum/energy of only the dynamic
body need not be conserved: the support exchanges impulse with the environment.
Multiplying both dynamic masses by a common factor preserves the velocities in
these controls and scales impulses/energy by that factor.

Penetration bias can inject kinetic energy even when restitution is zero. The
unforced initial-overlap control measures that separately while preserving linear
momentum. Restitution is consequently not a general upper bound on energy for
penetrating contacts or a dense bounded-iteration constraint system. Position
correction is a distinct optional stage.

## Tangential response

The pair friction coefficient is the combined coefficient; a frictionless surface makes the
pair frictionless only under `Min` or `Multiply`. The two accumulated
tangent components are projected onto the disk
`length(tangent_impulse) <= mu * accumulated_normal_impulse`.
This is a vector Coulomb bound, including diagonal slip, rather than separate
component clamps. There is one coefficient; static versus sliding behavior follows
from whether the bounded impulse can cancel contact-point slip. There is no
separate static/kinetic coefficient or unrelated damping.

Friction includes allowed angular response and can convert linear motion to spin.
The freely rotating solid-sphere control uses `I = 2/5 * m * r^2`; when friction
stops initial contact slip it retains `5/7` of its initial translational speed and
total kinetic energy. Rotation locking removes angular response but preserves
friction. Any pair with `linear_support` uses the existing linear response policy,
which disables friction and contact-spin response; its directional one-way rules
also govern which participant can receive normal impulse.

Under gravity a sliding rotation-locked sphere on a flat fixed support receives
normal impulse `m * g * h` per substep, with tangential speed loss bounded by
`mu * g * h`. The supported test checks the disk bound and non-increasing kinetic
energy at three masses and both BodyId orders. Gravity/support work and externally
driven surfaces must be included when interpreting other energy controls.

## Executable evidence and limits

[The shared controls](../tests/support/contact_materials.rs) verify analytic normal
impact velocities, combined restitution under every rule and precedence, threshold equality and either side of it,
unequal masses, fixed supports, speculative arrival, persistent history with/without
warm starting, clear/recontact, same-ID replacement, combined friction under every rule, diagonal
Coulomb limiting, sticking, spin, rotation lock, linear support and invalid admission.
The mass scales are `1e-6`, `1` and `1e6`; formulas use explicit absolute/relative
roundoff tolerances. Momentum, impulse and energy checks normalize by their physical
reference scales, retaining the same strict relative accuracy at small masses.
Unlocked dynamic endpoint controls distinguish both linear-support directions and
spin suppression from rotation locking, including the reciprocal friction control.
A 17-boundary penetration trace also compares complete physical
checkpoints exactly across repeated runs on each build/target.

Each contact control uses one substep, at most eight velocity passes, no convergence
shortcut, no position correction or sleeping, and complete requested time. These
choices isolate material behavior; production defaults and work limits are unchanged.

```sh
cargo test --locked --test contact_materials
```

These isolated controls do not prove dense-rubble quality, all primitive manifold
accuracy, extreme independent mass ratios, angular momentum conservation for
positional correction, moving-support quality or game adoption. #222/#223/#224
track the separate fixture work; #193 remains open. No wall-clock speedup or
cross-target/compiler bit-identity claim follows from these tests.

Convention sourceRevision: `e6acb5310afaf15c0cba24f87108f5f4ad1bedc3`.
