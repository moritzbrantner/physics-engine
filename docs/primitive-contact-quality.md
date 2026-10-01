# Axial primitive contact quality

Issue #241 adds independent physical controls for all ten existing unordered
primitive pairs, rather than treating a specialized dispatch counter as physical
acceptance. This bounded reference slice locks rotation and uses identity poses,
zero friction/gravity/forces/torques and no CCD. It does not complete general
rotation, frictional stacks, moving-support acceptance or chronological impacts.
The global capability ledger remains partial; dynamic wedges retain their
required rotation lock and bounding-box-origin solver contract.

## Predeclared inputs and independent response

The [inputs and bounds were declared before collecting outputs](https://github.com/moritzbrantner/physics-engine/issues/241#issuecomment-5921980308).
Shapes are sphere radius 1, box half-extents `(1,1,1)`, capsule half-segment .5 and
radius 1, and wedge half-extents `(1,1,1)`. There are 16 ordered pair combinations,
both BodyId assignments, upper mass 0 or 3 with lower mass 1, restitution 0/.5/1
on both bodies, and initial axial penetration 0/.01: 384 cases.

Every shape has z half-width 1. Wedge origins are `(.5,.5,z)` and other origins
`(0,0,z)`, giving a common interior x/y cross-section for the z-facing contacts.
Lower z is 0 and upper z is `2-penetration`. Lower velocity is `(0,0,3)` and upper
velocity is zero. This construction permits a fixture-specific analytic axial
face-gap observation; it introduces no reusable geometry kernel or production
shape authority. Locked orientation and independently checked analytic x/y motion
keep the construction valid throughout the trace.

With `u` the upper inverse mass (zero for the fixed target), the independent
normal impulse is `J=(1+e)*3/(1+u)`. Expected lower/upper velocities are `(0,0,3-J)`
and `(0,0,J*u)`. Each physical observation checks the velocities and integrated
positions against that formula, finite state, unchanged input shape/mass,
identity orientation and zero spin. Dynamic-pair linear momentum is conserved;
a fixed target intentionally exchanges momentum while its pose/velocity remain
unchanged. Energy is independently `sum(m*v·v)/2`, with initial energy 4.5, rather
than reading the production energy helper.

Relative vector error is `length(actual-expected)/(1+length(expected))`, bounded
by `1e-10` at these unit-sized fixtures. Axial overlap is bounded by the unchanged
slop .02 plus `1e-10` numerical tolerance. This small tolerance cannot hide a
material penetration. Energy ratio is at most `1+1e-10`. Restitution-zero pairs
must maintain admitted current contact and unchanged analytic velocities at every
physical substep; bouncing pairs must separate by the end. A slop-sized residual
in an unforced zero-restitution contact is intentional, with position correction
disabled; no claim of zero residual penetration is made.

## Time, work and replay

Each case runs 16 ticks at `1/60` second with four substeps, eight fixed velocity
passes and zero position passes, while a one-substep world and its independent
replay run all 64 `1/240`-second physical steps. Full Body state matches the
four-substep world at every tick; checkpoint bytes match the independent
one-substep replay at every physical step. All three worlds consume the full
requested interval. Every call checks the pass ceiling, at most four contact
points per substep, visits no greater than eight times admitted points, no position
passes, no swept events, no wakes and no retirement.

Counters remain integers until bounded diagnostic f64 transport. Solver visits
in the measurement include all three worlds, including the discarded replay and
cadence probe; continuity samples count only the observed one-substep world.
Native and WASM same-target repeatability is required. Their equality does not
promise bit identity across builds or platforms. The all-zero private checkpoint
build tag is a same-process fixture context, not a consumer compatibility tag.

The fixture is shared by native integration tests and the existing diagnostic
`dense-contact-contract` example. Its ten-field read-only measurement getter is
absent from the production Pages module. The WASM runner requires three identical
repeats and validates uninitialized/out-of-range NaN results. Existing dense,
material and narrow-support controls still execute in that driver.

[Recorded evidence](primitive-contact-quality-2026-10-01.json) identifies clean
producer `9a01302`. Both native repeats and all three release-WASM repeats report
384 cases, 8,192 continuous-contact observations, zero normalized velocity and
momentum-response errors, maximum energy ratio 1, and peak/residual overlap
`0.010000000000010445`. Total solver visits across all three worlds are 259,008.
Every observed interval is `0.26666666666666694` seconds, within the declared
time tolerance. Matching measurements across these two builds are observations,
not a stronger cross-target replay contract.
All 16 fast-tier checks passed, including the existing instrumentation-neutrality
and native replay controls.

```sh
cargo test --release --locked --test primitive_contact_quality
cargo build --manifest-path demo-wasm/Cargo.toml --release --locked \
  --target wasm32-unknown-unknown --example dense-contact-contract
node scripts/test-dense-contact-wasm.mjs \
  demo-wasm/target/wasm32-unknown-unknown/release/examples/dense_contact_contract.wasm
```

No production collision/response, material policy, solver budget, damping, sleep
threshold or geometry ownership changes. The pending momentum repair #237/#238
is separate and remains blocked by #243; the new fixture runs on main's existing
semantics and does not conceal the adverse stacks in #232/#236.

Resolved convention sourceRevision: `e6acb5310afaf15c0cba24f87108f5f4ad1bedc3`.
