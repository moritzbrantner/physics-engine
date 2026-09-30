# Reciprocal current-contact momentum

Issue #237 repairs an internal friction couple in `approximate::World`. Contact
geometry provides two surface witnesses. For penetrating contacts these are
separated along the normal. Opposite tangential impulses at those distinct world
points preserve linear momentum but change total angular momentum.

The independent two-box reproducer uses half-extents `(18,18,18)`, masses 2, centers
`(0,0,0)` and `(0,35.99,0)`, upper velocity `(10,-1,0)` and lower velocity zero.
Both rotate freely, with friction 0.6 and restitution zero. Gravity and position
correction are disabled. One `1/240`-second step uses one substep and at most eight
velocity passes. Penetration 0.01 is within slop 0.02, so no penetration-bias energy
is requested. Uniform cube inertia is independently `m*(36²+36²)/12 = 432`.
On baseline `b81a410`, total linear momentum remains `(20,-2,0)`, while angular
momentum changes from `(0,0,-719.8)` to approximately `(0,0,-719.806)`.

## Response boundary

Reciprocal solver-owned dynamic current contacts now apply normal/friction
impulses at the midpoint of their witnesses. Relative lever arms are adjusted
together before effective-mass preparation, velocity solving and local warm-start
anchor capture. Separation still supplies the original normal target. This is a
response decision; collision geometry, admission and shared kernel ownership do
not change. The midpoint uses relative coordinates rather than adding large
absolute world points.

Fixed, external, sleeping-anchor, linear-support and nonreciprocal response keep
their existing authority. Swept contacts keep their existing temporal witness semantics. This
repair does not promise angular-momentum conservation for an externally constrained
system or stronger rotational CCD. No solver pass, simulation time, damping,
material rule or sleep threshold is added or changed. The extra work is bounded
vector arithmetic per applicable current contact; trajectory-dependent later work
can change. No performance improvement is claimed.

A slow-current-contact regression compares the awake body's full state against
the same contact with a fixed anchor, in both `BodyId` orders. The anchor remains
asleep with zero wake events; its inactive mass/inertia cannot opt into reciprocal
response just because it has positive nominal mass.

Continuation semantics change, so checkpoints use algorithm 3 with unchanged wire
format 2 and reject algorithm 2. The empty-world wire digest changes with the
algorithm header/checksum, while the empty physical trajectory remains unchanged.

## Independent acceptance

The existing material fixture adds 768 unforced one-step controls: mass scales
`1e-6`, 1, 2 and `1e6`; upper/lower mass ratios 1 and 3; box/box, sphere/sphere and
both sphere/box orders; friction 0, 0.15, 0.6 and 1; penetration 0, 0.01 and 0.015625;
and both BodyId orders. Cubes have half-extents 18 and spheres radius 18. Upper
center is `(0,36-penetration,0)` with velocity `(10,-1,0)`; lower center/velocity
are zero. Both have zero initial spin, identity orientation, free rotation,
restitution zero, no CCD/force/torque and disabled sleep. Configuration is one
substep, eight fixed velocity passes, warm start enabled, gravity zero and no
position correction. Every case consumes exactly `1/240` second.

Total angular momentum is independently the sum of `position × mass*velocity`
and isotropic `I*angular_velocity`. Cube inertia follows the formula above; sphere
inertia is `2*m*r²/5`. Linear and angular errors are normalized by their own mass,
length and initial-momentum scales, then bounded by `1e-10`. A fixed absolute
threshold cannot conceal the small-mass defect. State and kinetic energy remain
finite, total kinetic energy cannot increase beyond relative `1e-10`, and nonzero
friction must actually reduce upper tangential velocity. Each case checks full
checkpoint-byte equality against an independently constructed same-target replay,
real contacts, no retirement and unchanged pass/time budgets.

Native integration tests repeat the measured matrix. The existing diagnostic
`dense-contact-contract` example executes it natively and in release WASM; the
runner reads four bounded measurement fields and checks three exact repeats.
The getter and its stored measurements are absent from the production Pages module.
Native and WASM repeat within their own build/target; no cross-target bit identity
is implied. The private all-zero checkpoint build tag is a same-process test
context, not a consumer compatibility identity.

[Recorded evidence](contact-momentum-quality-2026-10-01.json) identifies producer
`c59d940` and retains two native and three release-WASM measurements. Maximum
normalized linear error is `5.057012078047297e-16`, angular error is
`4.731782613072224e-16`, and kinetic-energy ratio is `0.9975247524752475`.
All 16 fast-tier checks passed. The 24 original/20-second dense traces also passed
their unchanged bounds and replay assertions at `3ba6305`, before the sleeping-anchor
guard refinement; those fixtures disable sleep, so the refinement cannot change
their response. The evidence preserves that distinct producer rather than
attributing the trace to the later commit.

```sh
cargo test --release --locked --test contact_materials --test floating_checkpoints
cargo build --manifest-path demo-wasm/Cargo.toml --release --locked \
  --target wasm32-unknown-unknown --example dense-contact-contract
node scripts/test-dense-contact-wasm.mjs \
  demo-wasm/target/wasm32-unknown-unknown/release/examples/dense_contact_contract.wasm
```

## Unresolved stack controls

This does not repair #236 or complete #232/#193. The predeclared equal/mixed-mass
20-second stack controls pass their penetration limits but remain moving. On the
original baseline, physical-substep tail speeds are approximately 50.0 and 113.6,
with angular speeds 0.997 and 3.074 rad/s. The low-friction control naturally settles.
At `c59d940`, equal/mixed physical-substep tail speeds are `60.1323` and `94.2993`;
angular speeds are `0.9121` and `3.0916`. Neither stack sleeps. Equal-mass tail speed
increases, while mixed-mass speed decreases; neither meets acceptance. Peak pair
penetrations `0.5364` and `1.2157` remain within 1.8, and the low-friction control
still settles. Full raw tick/substep histories' measured extrema and work are in
the evidence JSON, with the failing diagnostic source published in
[issue #232](https://github.com/moritzbrantner/physics-engine/issues/232#issuecomment-5921786087).
Zero penetration velocity bias, coupled tangent solving,
anisotropic friction projection and separated normal/friction sweeps were rejected
because they failed the original controls. None is promoted by this repair.

Resolved convention sourceRevision: `e6acb5310afaf15c0cba24f87108f5f4ad1bedc3`.
