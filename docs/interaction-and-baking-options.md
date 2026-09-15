# Independent physics comparison options

## Interactive controls

The page has two independent options. Changing either resets the scene and stores the choice in URL query parameters, so disturbed piles do not contaminate the next comparison.

| Character | Crates | Behavior |
| --- | --- | --- |
| `linear` | `upright` | Default: linear character pushes, passive landings, and crates constrained against rotation. |
| `linear` | `free` | Passive character landings without direct character torque; other contacts, including projectiles and floor friction, can still rotate crates. |
| `physical` | `free` | Original rigid-body behavior for comparison. |
| `physical` | `upright` | Original character loading/pushing, with a separate crate rotation constraint. |

For example, `?character=linear&crates=free` keeps free crate rotation while testing the repaired landings. These modes intentionally differ in physics; they are not equivalent performance optimizations. The original physical comparison can still exhibit the reported landing disturbance. Baking is a separate future option, not a name for either of these controls.

The legacy `sandbox_reset` export and `sandbox-projectiles-v1` benchmark retain physical/free behavior. `sandbox_reset_with_character_mode(0|1)` selects character response with free crates; `sandbox_reset_with_options(character_mode, upright_crates)` independently selects both. Invalid option values leave the existing world untouched.

## Engine-owned contact policy

`RigidBox3d::with_linear_push(support_direction)` opts an externally controlled body into inelastic, normal-only response. Non-supporting contacts transfer linear motion without adding angular response or tangential friction. Supporting contacts resolve the actuator against the unchanged other body, retaining that support's existing linear velocity. Response-local inverse mass/inertia participation is restored before returning: actual body kinds, angular state, and rotation-lock settings remain intact. Ordinary crate/projectile contacts still use the physical solver.

This deliberately does not model character weight or transfer landing momentum into supports. It is not a complete kinematic character controller; stepping, slope limits, and rotating-platform transport are separate capabilities. Reported normal constraint impulse is response-policy evidence, not a promise of equal momentum transfer into a passive support.

For cardinal support directions, a contact from strictly above the support centre remains passive at shallow edges and SAT-axis ties. A conservative actuator sweep wholly inside this same half-space does not wake a sleeping support. The support remains collision-tested. Centre-level side contacts remain pushable; side pushes, other dynamic impacts, and topology changes retain their wake behavior. General directions use conservative waking. Grounding still uses actual engine contact normals.

## Upright crates are not fixed bodies

The upright option uses the existing engine rotation lock, not pose overwrites, increased damping, or conversion into static scene geometry. Crates still translate when pushed or shot. Floor friction can create torque after an initially torque-free character push, so this option explicitly prevents **all** crate rotation, including projectile-induced spin. Disable it to compare full dynamics.

The required 360-tick stack-edge landing regression runs with **free** crate rotation. It checks exact crate positions and angular state throughout, verifies that the character stays on top, and requires the scene to return to sleep. A rotation constraint therefore cannot conceal a failing landing repair. Separate tests prove direct no-torque pushing, rough-floor upright pushes, moving-support preservation, projectile spin in free mode, projectile translation in upright mode, and option validation.

## Repeatable comparisons

`node scripts/benchmark-character-options.mjs <head.wasm> <results.json>` runs the versioned `character-options-v1` walking and stack-edge landing workloads, twice per option combination. It records raw WASM tick times and per-tick observable replay fingerprints; repeated executions of the same mode must agree. Different modes are not required to share a fingerprint. Timing is advisory and excludes rendering; it is not browser FPS or a wall-clock CI threshold. The existing Performance Evidence workflow retains these results alongside the unchanged physical base/head workload comparison.

## Optional baking follow-on

Tracked in issue #73. Keep runtime preparation as the reference and add optional prepare-at-load static geometry reuse, independently of gameplay options. Start in memory; serialized bake artifacts come only after demonstrated value. No baking toggle is implemented in this slice.

Acceptance: preserve contacts, checked errors, ordering, and replay hashes with baking on/off; explicitly invalidate geometry, placement, membership, and representation/version changes; never bake sleeping dynamic crates as permanent static geometry. Measure startup cost, retained memory, preparation counts, and steady-state performance on the stable projectile workload. Do not reduce dynamic CCD samples, event limits, or solver correctness to make a baked path look faster.
