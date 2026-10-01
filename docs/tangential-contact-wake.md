# External tangential contact wake

Issue [#254](https://github.com/moritzbrantner/physics-engine/issues/254) repairs
wake eligibility after genuine geometry admission. The old wake test considered
swept contacts and center normal approach, so a touching external floor could
translate tangentially beneath a naturally parked body without waking it.

After the strict current/swept geometry query, a parked/external pair now checks
relative velocity at admitted contact points. Nonzero, nonseparating point motion
is wake evidence. This uses the response solver's existing angular/linear-support
policy. Existing normal and swept criteria remain intact. Sensors and layers filter
before geometry; tolerance-only and full-interval misses never reach wake. Real
mass/inertia and first-substep forces are restored before response by
[#253](contact-wake-forces.md). Fixed and external bodies remain boundaries
between independent dynamic islands. No support-root, solver budget or production
warm-start setting changes.

The [causal probes and predeclared controls](https://github.com/moritzbrantner/physics-engine/issues/254#issuecomment-5924820629)
retain the minimized baseline failure in both ID orders, repeated twice, and each
intermediate result. Baseline `376266ac6c4444842c0a89f23f795646f7d4cc62` makes the
strict current query but records zero wakes and zero response preparations. The
contact-point probe gives one wake and one response preparation in both orders.

## Physical controls

The shared [public fixture](../tests/support/tangential_contact_wake.rs) naturally
parks a mass-2 radius-1 sphere at `(0,1,0)` on fixed floor ID100, with half-extents
`(12,.5,4)` and center `(0,-.5,0)`. Gravity is `(0,-10,0)`, default friction .6,
eight velocity passes, zero position passes and no convergence shortcut. Parking
uses 32 calls of `1/60`; no sleep state is forced. A second sphere, ID30 at x10,
parks on the same floor and must remain unchanged through every control.

Create an external mass-1 box of half-extents `(4,.5,4)` with the same top surface
and command its motion. Neither creation nor the command may wake the rider.
All controls run both `[external,rider]` ID orders `[1,10]` and `[10,1]` at physical
substep `h=1/240`. LoadedFour uses four physical substeps per call; others use one.

| Control | Motion / admission and independent prediction |
| --- | --- |
| Tangent | Default warm starting; external `(3,0,0)`; one wake, unchanged resting rider |
| Loaded | Same translation, declared warm-start-disabled reference; `vx=6*n*h`, `omega_z=15*n*h` |
| LoadedFour | Same loaded reference, four substeps per call |
| Slow | Default warm starting, tangent speed .001; wake despite speed below sleep threshold |
| Normal | External `(0,3,0)`; unchanged normal behavior, `vy=3`, `y=1+3*n*h` |
| Stationary | No motion; no wake |
| Separating | External `(3,-1,0)`; separating normal motion remains negative despite tangent motion |
| SkinMiss | Top surface .015 below actual contact, translation `(3,0,0)`; no crossing or wake |
| BroadMiss | Origin `(4.9,-.5,4.9)`, translation `(3,0,0)`; bounding candidate misses sphere/box corner |
| Sensor / Layers | Touching translating body filtered before admission; no wake |
| SpinCenter | External angular velocity `(0,3,0)` with the rider at its center; admitted contact point has zero velocity, no wake |

There are 24 rows per repeat. The default Tangent and Slow controls retain an
important adverse fixture result: with coincident floors and warm starting, the
fixed floor retains the entire normal load. Moving external geometry then carries
no normal impulse and has no available friction impulse. Changing only floor ID
order does not redistribute that retained load. These controls require genuine
wake and preparation; they do not claim loaded friction from a redundant support.

The separately declared Loaded references disable warm starting and place the
external pair first, so it receives the normal load before the redundant fixed
floor. This explicit reference configuration does not change the engine default.
Gravity gives load `m*g*h=1/12`; friction .6 yields impulse `.05` along x. Mass2
therefore gives `vx=.025`, and sphere inertia .8 gives `omega_z=.0625` after the
first physical substep. After n steps the semi-implicit position is
`x=3*h*h*n*(n+1)`, with `y=1`. Five calls keep LoadedFour within the known sliding
regime through n20, before rolling surface speed reaches the imposed speed3.
Default-policy parking/restart on a single external support remains
[#251](https://github.com/moritzbrantner/physics-engine/issues/251).

Each control independently checks linear/angular velocity and position through
five calls within `1e-10`, with finite state, unchanged mass, prescribed driver
motion, no retirement and full elapsed time. Kinetic energy is
`v·v + .4*omega·omega`; tangent work is `3*2*vx`. Loaded first-step energy is
.0021875 against imposed work .15. Normal work is `3*2*(vy+10*dt)`, including the
gravity impulse. Energy plus potential change `20*(y-1)` cannot exceed imposed
work plus `1e-10`. Negative controls require zero wake, preparations, response
points and constraint visits, and unchanged rider state.

SpinCenter is a current-contact negative control, not rotating-platform CCD
evidence. The engine retains conservative rotational bounds but sweeps translation
at held orientations. Arbitrary rotating contact discovery/restart remains outside
this delivery.

## Continuation and evidence

The [raw report](tangential-contact-wake-2026-10-01.json) identifies the exact code
producer and convention sourceRevision. Native release repeats all 24 rows twice;
release WebAssembly repeats them three times, identically within each target.
Every fixture compares complete checkpoint bytes with a fresh repeat and an
explicit restore after each of five calls. Placeholder context is private
same-process evidence, not portable save identity. Algorithm advances from4 to5
because continuation now includes tangential wake; format2 keeps its layout and
rejects prior algorithms. No cross-target bit replay is promised.

Each row's counters cover its first observed call, including discarded pre-wake
geometry. Auxiliary repeat, restore and continuation calls are excluded; maximum
motion error covers all five calls. Preparation equals the physical substep count
for admitted controls; velocity rounds remain bounded by eight per substep and
visits by eight per admitted point. There is no timing or total-driver-work claim.
The diagnostic WebAssembly example also retains prior physical controls and stays
outside the production Pages module.

```sh
cargo test --release --locked --test tangential_contact_wake -- --nocapture
cargo build --manifest-path demo-wasm/Cargo.toml --release --locked \
  --target wasm32-unknown-unknown --example dense-contact-contract
node scripts/test-dense-contact-wasm.mjs \
  demo-wasm/target/wasm32-unknown-unknown/release/examples/dense_contact_contract.wasm
```
