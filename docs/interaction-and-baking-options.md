# Independent physics comparison options

## Interactive controls

The page has three independent options. Changing any option resets the scene and stores the choice in URL query parameters, so disturbed piles do not contaminate the next comparison.

| Character | Crates | Fixed geometry | Behavior |
| --- | --- | --- | --- |
| `linear` | `upright` | `runtime` | Default: linear character pushes, passive landings, upright crates, and the reference runtime OBB preparation path. |
| `linear` | `free` | either | Passive character landings without direct character torque; other contacts can rotate crates. |
| `physical` | `free` | either | Original rigid-body gameplay response, with fixed preparation independently selectable. |
| either | either | `load` | Same physics semantics, but genuine fixed scene OBB geometry is prepared once and retained for exact SAT reuse. |

For example, `?character=linear&crates=free&bake=load` keeps free crate rotation while testing repaired landings and prepare-at-load fixed geometry. Character/crate modes intentionally differ in physics; the `bake` axis must not. `bake=runtime` remains the reference path and is the default when the query parameter is absent.

The legacy `sandbox_reset` export and `sandbox-projectiles-v1` benchmark retain physical/free/runtime behavior. `sandbox_reset_with_character_mode(0|1)` selects character response with free crates; `sandbox_reset_with_options(character_mode, upright_crates)` independently selects gameplay response; `sandbox_reset_with_baking_options(character_mode, upright_crates, fixed_geometry_mode)` adds the independent runtime/prepare-at-load axis. Invalid option values leave the existing world untouched.

## Engine-owned contact policy

`RigidBox3d::with_linear_push(support_direction)` opts an externally controlled body into inelastic, normal-only response. Non-supporting contacts transfer linear motion without adding angular response or tangential friction. Supporting contacts resolve the actuator against the unchanged other body, retaining that support's existing linear velocity. Response-local inverse mass/inertia participation is restored before returning: actual body kinds, angular state, and rotation-lock settings remain intact. Ordinary crate/projectile contacts still use the physical solver.

This deliberately does not model character weight or transfer landing momentum into supports. It is not a complete kinematic character controller; stepping, slope limits, and rotating-platform transport are separate capabilities. Reported normal constraint impulse is response-policy evidence, not a promise of equal momentum transfer into a passive support.

For cardinal support directions, a contact from strictly above the support centre remains passive at shallow edges and SAT-axis ties. A conservative actuator sweep wholly inside this same half-space does not wake a sleeping support. The support remains collision-tested. Centre-level side contacts remain pushable; side pushes, other dynamic impacts, and topology changes retain their wake behavior. General directions use conservative waking. Grounding still uses actual engine contact normals.

## Upright crates are not fixed bodies

The upright option uses the existing engine rotation lock, not pose overwrites, increased damping, or conversion into static scene geometry. Crates still translate when pushed or shot. Floor friction can create torque after an initially torque-free character push, so this option explicitly prevents **all** crate rotation, including projectile-induced spin. Disable it to compare full dynamics.

The required 360-tick stack-edge landing regression runs with **free** crate rotation. It checks exact crate positions and angular state throughout, verifies that the character stays on top, and requires the scene to return to sleep. A rotation constraint therefore cannot conceal a failing landing repair. Separate tests prove direct no-torque pushing, rough-floor upright pushes, moving-support preservation, projectile spin in free mode, projectile translation in upright mode, and option validation.

## Optional fixed-geometry preparation

Prepare-at-load retains the exact `PreparedObb3d` representation already used by SAT: quantized vertices, edges and face axes plus the original checked geometry results. The world owns this cache at the public ECS scene boundary. Only genuine `BodyKind::Fixed` bodies inserted by the consumer are registered. A dynamic body that later sleeps may be presented internally as a temporary fixed proxy, but it never crosses this registration boundary and cannot become baked static geometry.

Preparation is currently in memory only. Fixed additions prepare once while load mode is active; removal invalidates the entry; changing placement by removal/re-addition produces a fresh preparation; switching modes clears/rebuilds the retained set; representation version `1` is exposed with the evidence. Persistent serialized bake artifacts are intentionally deferred until the in-memory comparison demonstrates enough value to justify an artifact format and migration policy.

The world scopes prepared geometry to its own query/step. Retained maps use shared immutable storage, so activating preparation for a step does not deep-clone the geometry. Any shape not exactly matching retained fixed geometry still follows ordinary runtime preparation. Sampling, refinement, CCD event bounds, solver passes, impulse response, BodyId ordering and checked error behavior are unchanged.

## Repeatable comparisons

`node scripts/benchmark-character-options.mjs <head.wasm> <results.json>` runs the versioned `character-options-v1` walking and stack-edge landing workloads twice per gameplay combination. It records raw WASM tick times and per-tick observable replay fingerprints; repeated executions of the same mode must agree. Different gameplay modes are not required to share a fingerprint.

`node scripts/benchmark-baking.mjs <head.wasm> <results.json>` runs the stable projectile workload twice in both `runtime` and `prepare-at-load` fixed-geometry modes. Every corresponding replay hash, event count and body count must be identical. It also records preparation count, representation version, retained bytes and reset/startup time. Node/V8 WASM timings are advisory and exclude rendering; they are not browser FPS or a wall-clock CI threshold. The Performance Evidence workflow retains these results alongside the unchanged physical base/head workload comparison.


## Scenario-defined simulation rules

The sandbox scenario now owns an explicit interaction matrix for four roles: world, character, crate,
and projectile. Each pair can be enabled or disabled independently. The engine represents those choices as
symmetric collision-layer memberships/masks and rejects disabled pairs in broad-phase discovery, current
contact queries, and persistent-tail contact handling. The browser only edits/serializes the scenario; it
does not filter contacts after the fact.

Response policy is a separate axis from collision eligibility. The puzzle-friendly default uses the
engine's constrained linear-push actuator policy for the character, free rigid-body rotation for crates,
and ordinary physical response for projectiles. Walking into a crate therefore transfers predictable
linear motion without inducing torque, while an off-center projectile can still rotate the same crate.
The existing character-options benchmark remains the stable comparison workload for physical versus
linear character response. Fixed-geometry prepare-at-load remains independent of all gameplay rules.

Legacy Wasm reset values `0` and `1` still mean physical/linear character response with every collision
pair enabled. New scenario-aware callers set the explicit-rules marker bit and encode response plus pair
rules in the same integer; the legacy crate argument remains accepted only for compatibility. Rule changes
reset the acceptance world and are stored in URL query state for reproducible comparisons.
