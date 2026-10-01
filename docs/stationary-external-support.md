# Natural parking on stationary external supports

Issue [#251](https://github.com/moritzbrantner/physics-engine/issues/251) lets
quiet dynamics naturally park on stationary externally imposed supports, then
restart or fall when genuine contact/dependency evidence requires it. It follows
the separately merged [admission](driven-parked-contact.md),
[first-force](contact-wake-forces.md) and [tangential-wake](tangential-contact-wake.md)
prerequisites.

Stationary external bodies with exactly zero prescribed linear and angular
velocity now seed the existing quiet-support propagation. Quiet timers, solver
budgets, warm-start defaults and island traversal are unchanged. Moving external
bodies do not provide quiet roots. Fixed/external bodies remain boundaries between
independent dynamic islands.

The [causal evidence](https://github.com/moritzbrantner/physics-engine/issues/251#issuecomment-5925095143)
retains repeated baseline failures and the seed-only adverse result. Baseline
`1d62f70e3588b80e51746ed7d05bb8bbd0bfbfdb` gives identical resting pose/velocity
and support truth on fixed and external floors, but only the fixed rider sleeps.
Adding the stationary root fixes parking. However, a downward-moving support then
leaves its rider asleep without gravity. The departure probe confirms admitted
current geometry and a retained positive normal impulse before wake; this is a
load-bearing dependency loss, not a new unrelated separating contact.

The departure rule therefore requires admitted geometry, separating motion,
gravity-support orientation and a retained positive impulse with matching normal.
Existing response preparation and first-substep forces run before regeneration.
A motion command alone does not wake the rider. Existing filtered, stationary,
tolerance-only and unrelated separating controls remain negative. Actual support
removal continues to use the existing dependency traversal.

## Public physical controls

The shared [fixture](../tests/support/stationary_external_support.rs) uses a
mass-2 radius-1 sphere at `(0,1.5,0)` on a mass-1 external box with half-extents
`(4,.5,4)`, centered at zero. Gravity is `(0,-10,0)`, friction .6, free rotation,
default warm starting/eight velocity passes and zero position passes, with no
convergence shortcut. For 32 calls of `1/60`, compare complete rider state after
every call with an equivalent mass-0 fixed-support world. Both must naturally
sleep using unchanged quiet timers. First observed parking is bounded by the
configured .5 second quiet interval plus one observation call.

An independent parked sphere on a fixed floor at `(20,-.5,20)` remains unchanged
and stays outside the translating support's path. The shared-support control
instead places that sphere at `(10,1.5,0)` on the same stationary external floor,
widened to half-width12. A side contact must wake only its target.

All nine controls run both `[support,rider]` ID orders `[1,10]` and `[10,1]`,
with one/four substeps per call: 36 rows. Physical substep is `h=1/240`. Motion
commands and unrelated creation/removal must leave the rider parked before stepping.

| Control | Independent response after n physical substeps |
| --- | --- |
| Stationary | Remain parked at `y=1.5`, zero linear/angular velocity |
| Normal | Support `(0,3,0)`; `vy=3`, `y=1.5+3*n*h` |
| Tangent | Support `(3,0,0)`; `vx=6*n*h`, `omega_z=15*n*h`, `x=3*h*h*n*(n+1)`, `y=1.5` |
| DownwardDeparture | Support `(0,-3,0)`; `vy=-10*n*h`, `y=1.5-5*h*h*n*(n+1)` |
| HorizontalDeparture | Support `(2000,0,0)` leaves the sphere geometrically disjoint after the first h; thereafter `vx=6*h`, `omega_z=15*h`, gravity acts for n-1 substeps |
| Removal | Actual support removal wakes before stepping; ordinary gravity acts for all n substeps, with zero spin |
| NearMiss | A new external floor .015 below the rider surface translates along x; full-interval misses must not wake |
| UnrelatedRemoval | Create/remove a remote external sphere at `(100,100,100)`; no wake |
| SharedSideContact | A wall at `(-1.5,1.5,0)` moves `(3,0,0)` into one rider; `vx=3`, `omega_z=-15*n*h`, neighbor stays parked |

Tangent first-step load is `m*g*h=1/12`, so friction impulse is .05, giving
`vx=.025` and `omega_z=.0625` with sphere inertia .8. Five calls keep the
four-substep tangent control in the independently known sliding regime. Side
contact isolates floor friction with rider coefficient0, floor .6 and wall0 under
maximum mixing; the other controls retain default materials.

The horizontal departure control tests discrete substep contact discovery and
subsequent gravity after geometric disjointness. It does not establish exact
within-substep exit timing. The downward/removal controls independently check the
semi-implicit gravitational trajectory. Every case checks position and linear/
angular velocity through five calls within `1e-10`, finite state, preserved mass,
prescribed external motion, no retirement and full elapsed time.

Kinetic energy is `v·v + .4*omega·omega`. Tangent work is `3*2*vx`, horizontal
departure work is `2000*2*vx`, normal work is `3*2*(vy+10*dt)`, and side-contact
work is `3*(2*vx-.8*omega_z)`. Stationary, missed and downward/removal controls
receive no external work. Energy plus potential change `20*(y-1.5)` must not
exceed work plus `1e-10`. The first tangent substep records energy .0021875 and
work .15; side-contact records energy9.0015625 and work18.15.

The public support query retains ordinary within-slop contact history temporarily
during departure. The tests check actual platform/rider geometry independently
and require the retained flag to clear after continuation. This change does not
redefine that query. Rotational bounds remain conservative, but sweeps still hold
orientation while translating; arbitrary rotating-platform CCD/restart is not
established by these controls.

## Evidence and continuation

[Raw measurements](stationary-external-support-2026-10-01.json) identify the code
producer and resolved convention sourceRevision. All 36 rows repeat identically
twice in native release and three times in release WebAssembly within each target.
At the parked/commanded boundary, explicitly restore a checkpoint; compare complete
checkpoint bytes with the observed world and a fresh repeat after each of five
calls. Placeholder context is private same-process evidence, not portable save
identity. Algorithm advances from5 to6; format2 retains its layout and rejects
prior algorithms. Cross-target bit replay is not promised.

Each row's counters cover the first observed call, including discarded admission
probes. Auxiliary repeat, restored and continuation work is excluded; motion error
covers all five calls. Velocity rounds remain bounded by eight per substep and
visits by eight per admitted point. There is no timing or total-driver-work claim.
The dedicated WebAssembly example also retains prior physical controls and stays
outside the production Pages module.

```sh
cargo test --release --locked --test stationary_external_support -- --nocapture
cargo build --manifest-path demo-wasm/Cargo.toml --release --locked \
  --target wasm32-unknown-unknown --example dense-contact-contract
node scripts/test-dense-contact-wasm.mjs \
  demo-wasm/target/wasm32-unknown-unknown/release/examples/dense_contact_contract.wasm
```
