# Narrow fixed-support quality

The #231 controls exercise balance, real tipping and support removal through
`approximate::World`. They add acceptance fixtures without changing the solver.
Tilted/mixed-mass stacks remain #232; this does not complete #223 or #193.

## Reproducible inputs

All lengths are scene units. A fixed box with half-extents `(4,16,30)` and center
`(0,-16,0)` has its top at `y=0`. A separate fixed floor has half-extents
`(500,16,500)` and center `(0,-116,0)`, top `y=-100`. Their IDs are 2 and 1.
The dynamic body, ID 10, has half-extents `(18,18,18)`, mass 2, identity orientation
and zero initial linear/angular velocity. All three bodies have friction 0.6 and
restitution 0. The dynamic box has CCD enabled, free rotation and ordinary sleep
eligibility. No force, torque, impulse or velocity command is applied.

The selected existing tower budget is four substeps, up to eight velocity
iterations and two shared position passes, with explicit
`PositionCorrection::AdmittedContacts`. Other configuration uses `Config::default`
at the recorded producer: gravity `(0,-3600,0)`, slop 0.02, sleep speed 1 and sleep
duration 0.5 seconds, warm starting and ordinary island convergence. No optional
soft-contact policy is selected.

Each case requests 600 ticks of `1/60` second:

- **Balanced:** center `(0,18,0)` stays within 0.5 of its initial center at every
  observed boundary and naturally sleeps on the narrow support.
- **OffCenter:** center `(10,18,0)` puts its center of mass outside the support.
  It must develop angular speed above 0.1 rad/s, depart below `y=-36`, and settle
  on the lower floor with final center below `y=-60`.
- **Removal:** start balanced; immediately before tick 120, after two seconds,
  verify natural sleep and remove support ID 2. Removal must immediately wake the
  box. It falls to the lower floor, records one wake and at least two sleep
  transitions, and settles below `y=-60`.

## Observations and bounds

The shared `tests/support/box_fixture_oracle.rs` acceptance observer traverses
face and non-degenerate edge-cross SAT axes rather than querying engine response
manifolds. It accepts only finite box bodies from these fixtures and is compiled
through integration tests and diagnostic examples. Reusable production geometry
continues to belong in `rust-kernels`; this observer measures physics acceptance
independently of that production geometry path. It measures
post-integration overlap against each remaining fixed collider. The existing
fixed-support penetration limit remains 0.5. Position, orientation, velocities
and kinetic energy must stay finite; no body may retire.

Every case also advances a separately constructed one-substep observer four times
at `1/240` second per tick. Complete physical bodies must exactly match the real
four-substep world at every tick. Penetration, linear/angular peaks and kinetic
energy are observed at every physical substep, so tick snapshots cannot hide an
excursion. Over the last two seconds, both observation cadences require linear
speed at most 1 and angular speed times 54 at most 1. The factor 54 is a conservative
fixture angular-motion scale (sum of half-extents), larger than the box
circumscribed radius used by the existing sleep policy. All cases finish
naturally sleeping. Gravity and fixed/frictional response exchange energy and
momentum; recorded kinetic-energy peaks are observations, not a conservation claim.

Every call verifies its complete time increment, and each world finishes ten
seconds. Reported work stays within its configured velocity/position budgets and
at most eight contact points per active substep (one dynamic box against two fixed
boxes, up to four points each). Actual contact points, primary constraint visits,
position contact tests and active substeps are recorded separately. A quiescent
call still consumes time while reporting zero active substeps; this is not an
incomplete run. Observer and full-call active-substep counts can differ when sleep
begins inside a four-substep call.

Two independent worlds additionally compare exact continuation checkpoint bytes
at every tick, including after support removal. Replay is promised within the
recorded build/target; native and WASM need not have identical bits. The all-zero
checkpoint build tag is a private same-process diagnostic context, not a portable
consumer checkpoint identity.

## Recorded result and reproduction

[Native and release-WASM measurements](narrow-support-quality-2026-09-30.json)
retain both cadences. All three WASM invocations repeat exactly, as do the internal
same-target continuation checks. Peak substep penetration is approximately
0.0000611 (balanced), 0.0200 (off-center), and 0.002354 (removal), below the unchanged
0.5 limit. Last sleep is observed at approximately 0.504, 1.0375 and 2.7417 seconds
at physical substeps; the full-tick observations are later by at most one tick.
Both cadences have zero linear/angular motion throughout the final two seconds.

Substep observations matter: off-center peak speed is 786.86 versus 757.14 at
ticks, angular speed 10.47 versus 9.87, and kinetic energy 640,195 versus 594,317.
Removal has a 0.025 rad/s angular peak between ticks versus approximately 0.001
at ticks. These values are retained rather than replaced with the favorable cadence.
No performance improvement is claimed by this fixture work.

Run the native integration test with
`cargo test --release --locked --test narrow_support -- --nocapture`.
The existing `dense-contact-contract` example and Pages build now run these
controls alongside the existing dense-impact and material checks. Build that
example for `wasm32-unknown-unknown`, then run
`scripts/test-dense-contact-wasm.mjs` with the resulting WASM path, as recorded
in the evidence. The measurement getter exists only in this diagnostic example;
the production Pages module gains no export or measurement state.
