# External contact admission for parked dynamics

Issue [#250](https://github.com/moritzbrantner/physics-engine/issues/250) repairs
contact admission in `approximate::World`. Both a parked dynamic body and an
externally driven body have zero effective inverse mass. The ordinary zero-response
pair filter previously skipped their geometry entirely. Allowing the pair alone
also wakes a positive-gap interval miss through contact slop, and leaves genuine
slow inward motion rejected by the sleep-speed cutoff.

Parked/external pairs now query the actual current surface with zero margin.
After a miss, moving external/dynamic pairs sweep the complete requested translation
interval regardless of the automatic travel threshold, including the post-wake pass. Sensors and collision layers still filter first.
Only admitted geometry reaches wake traversal. Inward external normal motion does
not need to exceed the sleep-speed threshold. Existing wake traversal restores the
real dynamic response and prepares mass/inertia before contact impulses, then
regenerates response contacts with ordinary solver slop. Fixed and external bodies
remain nonresponders and do not bridge separate dynamic islands.

The [causal probes](https://github.com/moritzbrantner/physics-engine/issues/250#issuecomment-5923959156)
retain each intermediate adverse result. The initial18-case matrix missed an outside-slop crossing: the first sweep woke
the rider but the post-wake pass dropped the contact. The additional
[red/green regression](https://github.com/moritzbrantner/physics-engine/issues/250#issuecomment-5924057268)
and both ID orders now require preserved swept response. Prior incomplete
measurements remain linked in the raw report. No geometry authority, extra solver
passes, frame-end overlap substitute or forced sleep/wake was introduced.

## Inputs and independent bounds

The [issue declares the original reproducer and bounds](https://github.com/moritzbrantner/physics-engine/issues/250).
A mass-2 sphere of radius 1 at `(0,1,0)` naturally parks on a fixed box after
32 calls of `1/60` second. Gravity is `(0,-10,0)`, with one substep, default eight
velocity passes, zero position passes, unchanged sleep/convergence and slop .02.
The promoted fixture widens the floor to half-extents `(12,.5,4)`, at `(0,-.5,0)`,
to support a second naturally parked sphere at `(10,1,0)`. This second body proves
that a shared fixed floor does not propagate the wake.

After parking, create an external mass-1 box of half-extents `(4,.5,4)`, then
command its velocity. Neither operation may wake the spheres. Run both pair ID
orders `[external,rider]=[1,10]` and `[10,1]`, with neighbor ID30 and floor ID0.
The rider has free rotation and retains default friction/material properties.

| Case | External origin / vertical speed | Requested interval | Required admission |
| --- | --- | --- | --- |
| Current | `(0,-.5,0)` / 3 | `1/60` | Wake; `y=1.05`, `vy=3` |
| SlowCurrent | `(0,-.5,0)` / .25 | `1/240` | Wake; `y=1+.25/240`, `vy=.25` |
| Swept | `(0,-.505,0)` / 3 | `1/240` | Actual crossing; wake and normal response |
| SweptOutsideSlop | `(0,-.525,0)` / 3 | `1/60` | Gap .025 exceeds slop; retain the sweep through post-wake response |
| SkinMiss | `(0,-.515,0)` / 3 | `1/240` | Gap .015 exceeds travel .0125; stay parked |
| BroadMiss | `(4.9,-.5,4.9)` / 3 | `1/240` | Bounding-box candidate misses sphere/box corner; stay parked |
| Stationary | `(0,-.5,0)` / 0 | `1/240` | Stay parked |
| Separating | `(0,-.5,0)` / -3 | `1/240` | Stay parked |
| Sensor | Current pose/speed, sensor enabled | `1/240` | Stay parked |
| Layers | Current pose/speed, collision mask 0 | `1/240` | Stay parked |

Every call consumes the full requested time, has finite state, uses at most eight
velocity rounds and eight visits per admitted point, performs no position passes
and retires no bodies. Positive controls require one wake, one response and inertia
preparation, and nonzero inertia applications before normal response. Input mass
remains 2. Negative controls require zero wakes, preparation, admitted points and
constraint visits, with exactly unchanged rider pose and velocity. The independent
neighbor stays parked in every case.

Analytic surface height bounds penetration by `1e-10` in this unit-scale fixture.
Current normal response independently requires the platform's prescribed speed
and displacement within `1e-10`. Sphere kinetic energy is
`v·v + .4*omega·omega`; prescribed normal work is `2*u*delta_vy`. Kinetic energy
plus gravitational potential increase `20*(y-1)` must not exceed that work plus
`1e-10`. The recorded algorithm-3 producer below precedes #253's force-on-wake repair; its current case records energy9, imposed work18 and potential increase1.
The same historical slow case records energy.0625 and work.125. The current
fixture includes the additional substep gravity impulse in prescribed normal work
when waking, as documented in [First-substep forces on contact wake](contact-wake-forces.md). External position and velocity
must retain their prescribed motion; impulses cannot change the driver.

Swept admission proves contact discovery, wake, restored response and bounded
penetration/energy. Its existing speculative response records `vy≈1.8`, `y=1.0075` for Swept and `vy≈1.5`, `y=1.025`
for SweptOutsideSlop; this is not an elastic impact or remaining-time accuracy reference.
Issue [#246](https://github.com/moritzbrantner/physics-engine/issues/246) retains
that separate limitation. Natural parking on external supports and complete
frictional/rotating restart remain [#251](https://github.com/moritzbrantner/physics-engine/issues/251).
Angular motion is not analytically swept by this solver; these translating controls
do not establish arbitrary rotating-platform admission.

## Replay, work and evidence

The shared [fixture](../tests/support/driven_parked_contact.rs) uses the public
floating API. At the parked/commanded boundary, decode and restore an explicit
checkpoint. The observed world, fresh repeat and restored world must have equal
complete checkpoint bytes after the first call and each of eight continuation
calls. The fixture's placeholder context is private same-process evidence, not a
portable save identity. Checkpoint algorithm advances from2 to3 because wake
continuation changes; format2 retains its layout and prior algorithms are rejected.

[Raw measurements](driven-parked-contact-2026-10-01.json) identify producer
`c1d64fa33e4a1e044b490d81c158f44cff856044` and resolved convention revision
`e6acb5310afaf15c0cba24f87108f5f4ad1bedc3`. All20 cases repeat identically twice
in native release and three times in release WASM within each build/target.
Cross-target bit replay is not promised. The dedicated diagnostic example is
absent from the production Pages module; Pages already executes its acceptance
script as a required build check.

Each row's counters belong to the first observed call, including the discarded
pre-wake geometry pass. Auxiliary repeat, restored and continuation calls are
additional diagnostic work, excluded from these per-call counters. There is no
wall-time or total-driver-work claim. Both near-miss controls record one current
query and one swept query, with zero response work. Current controls record three
current queries, two admitted points and four constraint visits; Swept records
four current queries plus one sweep, two admitted points and four visits.
SweptOutsideSlop records five current queries plus two sweeps: both admission and
post-wake passes retain full-interval discovery, with one swept response contact.
`swept_contacts` counts the regenerated response rows, not discarded admission
probes: the within-slop Swept case records zero in that field after regeneration uses ordinary slop.
`geometry.sweep_queries` retains the actual first-pass swept work.

```sh
cargo test --release --locked --test driven_parked_contact -- --nocapture
cargo build --manifest-path demo-wasm/Cargo.toml --release --locked \
  --target wasm32-unknown-unknown --example dense-contact-contract
node scripts/test-dense-contact-wasm.mjs \
  demo-wasm/target/wasm32-unknown-unknown/release/examples/dense_contact_contract.wasm
```
