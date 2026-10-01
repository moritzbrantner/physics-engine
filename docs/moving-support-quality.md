# Translating support quality

Issue #242 adds bounded rigid-body controls for externally driven support carry,
departure and removal, in both BodyId orders. It uses the real floating API and
keeps externally prescribed platform motion independent of solver impulses.
There is no character-controller policy or new moving-platform API. Rotation is
locked in this reference slice; arbitrary rotating platforms, parked relative-frame
sleep and stronger CCD guarantees remain outside the acceptance claim.

## Predeclared physical inputs

The [inputs and bounds were declared before recording outputs](https://github.com/moritzbrantner/physics-engine/issues/242#issuecomment-5922229959).
The platform is an external mass-1 box with half-extents `(4,.5,4)`, origin zero,
velocity `(2,0,0)` and sleep disabled. The rider is a mass-2 box with half-extents
`(1,1,1)`, origin `(0,1.5,0)` and zero initial velocity. Both orientations are
identity and rotation locked, friction .6, restitution zero and CCD disabled.
The rider retains the natural sleep policy. Gravity is `(0,-10,0)`, with unchanged
slop .02, four substeps, eight fixed velocity passes and zero position passes.

Every trace runs 240 ticks at `1/60` second, consuming four seconds. Carry has no
commands. At tick 120, Departure sets rider velocity `(2,3,0)`; Removal deletes
the actual platform through `World::remove_body`. The departure command's
instantaneous speed/energy is observed before the next physical step, so tick
sampling cannot conceal that imposed peak.

## Independent motion, contact and energy

Let `h=1/240`, `n` be the physical step, and `a=min(n,80)`. Coulomb friction supplies
horizontal acceleration 6 until the rider reaches platform speed 2:

- `vx=min(6*h*n,2)`;
- `x=3*h²*a*(a+1)+2*h*(n-a)`.

The platform must remain at `(2*h*n,0,0)` with prescribed velocity `(2,0,0)`,
identity orientation and zero spin. Input shapes and masses remain unchanged.
For flight step `j=n-480`, gravity's semi-implicit reference is
`y=1.5+vy_initial*j*h-5*h²*j*(j+1)` and `vy=vy_initial-10*h*j`. Departure uses
initial vertical velocity 3, then lands at the top face and returns to `y=1.5`,
`vy=0`; Removal uses initial vertical velocity zero and continues falling without
support. Unit-scene position/velocity errors are bounded by `1e-9` over the
four-second interval. State must remain finite, with identity rider orientation
and zero angular velocity throughout.

The observed aligned box faces independently provide the support gap. Horizontal
face overlap plus absolute vertical gap at most `1e-9` records geometric support;
engine contact counts are not the physical oracle. Carry requires continuous
geometric support and the public support flag at every observation. Departure
requires a gap above .3 and later recontact. Removal immediately clears cached
support, then has 480 unsupported physical steps following the 480 supported ones.
Penetration is bounded by unchanged slop .02 plus `1e-9` numerical tolerance.

The platform is an external energy source. With rider mass 2 and platform speed 2,
imposed horizontal work is independently the sum of `4*delta_vx`, which totals 8.
Rider kinetic energy is independently `v·v`; gravitational potential is `20*y`.
At every observation their sum must not exceed initial potential 30 plus imposed
work and, after the Departure command, explicit kinetic input 9, within `1e-9`.
This accounts for intentional energy exchange rather than rejecting legitimate
external or gravity-driven motion. No production energy helper supplies the oracle.

The rider reaches absolute speed above the existing sleep threshold before a
natural .5-second quiet window can complete, and then remains moving. Thus these
six traces observe zero sleep/wake transitions and zero contact wake events; they
do not claim parked-body activation evidence. The diagnostic driver retains the
existing naturally parked narrow-support removal control and its real wake check.
Fixed/external island boundaries, tower and CCD/wake acceptance remain required.

## Cadence, replay and actual work

A one-substep world and independent replay observe every physical step; a separate
four-substep world observes each tick. Full Body state matches at each tick, and
checkpoint bytes match at each physical step. The two physical histories and two
native complete matrix runs must have identical measured results. Each call
checks full time, at most four contact points and eight velocity passes per
substep, at most eight visits per admitted point, zero position passes and no
retirement. Per-cadence work counts identify their own world. Total solver visits
include the observed world, independent replay and discarded cadence probe.
Counters remain integers until bounded diagnostic f64 transport.

Native and release-WASM measurements repeat within their own build/target; no
cross-target bit identity is promised. The private all-zero checkpoint build tag
is a same-process fixture context. The existing `dense-contact-contract` example
runs these controls alongside dense contacts, materials, primitive pairs and
narrow supports. Its read-only 19-field getter returns NaN for uninitialized or
invalid selection and is absent from the production Pages module.

[Recorded evidence](moving-support-quality-2026-10-01.json) identifies clean producer
`f88e1b8`, with two native matrix runs and three release-WASM repeats. All 16
fast-tier checks passed. Every trace consumes four seconds; maximum observed
motion/ballistic error is `3.730349362740526e-13`, penetration is zero and imposed
horizontal work is 8. Both BodyId orders have matching measurements.

Carry has 960 physical support observations. Departure has 142 unsupported and
818 supported physical observations, reaches gap `0.44375000000000053`, and lands
at `y=1.5`, `vy=0`. Its peak kinetic energy is 13, including the instantaneous
command. Removal has 480 supported and 480 unsupported observations, ending at
`y=-18.54166666666663`, `vy=-20.00000000000002`. Total solver visits across all three
worlds are 92,160/78,720/46,080 for Carry/Departure/Removal per BodyId order.
These are measured results within declared bounds, not future exact-work ratchets
or stronger cross-platform replay promises.

```sh
cargo test --release --locked --test moving_support_quality
cargo build --manifest-path demo-wasm/Cargo.toml --release --locked \
  --target wasm32-unknown-unknown --example dense-contact-contract
node scripts/test-dense-contact-wasm.mjs \
  demo-wasm/target/wasm32-unknown-unknown/release/examples/dense_contact_contract.wasm
```

No production physics, ownership, material policy, damping, sleep threshold or
solver budget changes. The blocked momentum repair #237/#238 and adverse
frictional stack controls #232/#236 remain unresolved.

Resolved convention sourceRevision: `e6acb5310afaf15c0cba24f87108f5f4ad1bedc3`.
