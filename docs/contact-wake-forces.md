# First-substep forces on contact wake

Issue [#253](https://github.com/moritzbrantner/physics-engine/issues/253) repairs
the first physical substep of a contact-awakened floating body. The ordinary force
phase skips parked bodies. Previously, later contact wake restored their mass and
inertia but omitted that substep's gravity and queued force/torque, leaving the
first supported friction impulse unloaded.

Wake traversal now retains only newly awakened indices in its existing scratch.
Each receives its real prepared response and the ordinary substep force operation
before contacts regenerate. An already awake body receives forces only in the
ordinary phase; repeated roots have no newly awakened indices. The existing
journal covers wake state, quiet time, velocities and queued loads. Discarded
geometry work remains counted. Solver budgets and wake admission are unchanged.
Stationary external-support parking and tangential wake remain separate
[#251](https://github.com/moritzbrantner/physics-engine/issues/251) and
[#254](https://github.com/moritzbrantner/physics-engine/issues/254) deliveries.

## Inputs and independent bounds

The [corrected causal probe](https://github.com/moritzbrantner/physics-engine/issues/253#issuecomment-5924424941)
retains two baseline failures, the passing candidate and an earlier adverse
material experiment. Friction mixing uses the maximum coefficient: assigning
rider friction .6 also enabled friction at the supposedly frictionless wall.
That two-patch experiment was unsuitable for the floor-only analytic oracle.
The corrected rider coefficient is zero, floor .6 and wall zero. Baseline
`2d31600df4b809b17c34befd241aba08daffe9e4` then reproducibly wakes the rider with
`vx=3`, but gives zero spin instead of the loaded `omega_z=-.0625`.

The shared [public fixture](../tests/support/contact_wake_forces.rs) naturally
parks a mass-2, radius-1 sphere at `(0,1,0)` on a fixed box with half-extents
`(4,.5,4)` and center `(0,-.5,0)`, using 32 calls of `1/60` second. Gravity is
`(0,-10,0)`, convergence is disabled, with the existing eight velocity passes and
zero position passes. Rotation is free. An independent naturally parked sphere
on a separate fixed floor remains unchanged throughout.

Create a mass-1 external box with half-extents `(.5,4,4)` at `(-1.5,1,0)`, and
command `(3,0,0)`. Creation and the command must leave the target parked. The
first physical substep `h=1/240` supplies floor load `m*g*h=1/12`. Friction .6
therefore gives impulse `-.05` along x. Sphere inertia is `.8`, so spin is
`-.0625` along z. These predictions precede the candidate implementation.

| Control | Independent trajectory after n physical substeps |
| --- | --- |
| ContactWake | `vx=3`, `vy=0`, `y=1`, `omega_z=-15*n*h` |
| RepeatedRoots | Two orthogonal driven walls; `vx=vz=3`, `vy=0`, spin components `(15*n*h/sqrt(2),0,-15*n*h/sqrt(2))`; one wake |
| AwakeReference | Same contact/load trajectory as ContactWake; declared awake reference receives no contact wake |
| RemovedSupport | Actual floor removal wakes before stepping; `vx=3`, `vy=-10*n*h`, `y=1-5*h*h*n*(n+1)`, zero spin |

All four controls run both `[wall,rider]` ID orders `[1,10]` and `[10,1]`, with
one and four substeps per call: 16 rows per repeat. Position is independently
`x=3*n*h` and, for RepeatedRoots, `z=3*n*h`. Each row checks nine calls, including
the first physical substep, within `1e-10` motion error. External velocities and
poses retain prescribed motion. The neighbor stays parked, all state remains
finite, no body retires, and the full requested interval elapses.

Kinetic energy is `v·v + .4*omega·omega`. Floor impulse is inferred independently
from angular momentum, `Jx=.8*omega_z` and `Jz=-.8*omega_x`. Prescribed wall work
is `3*(2*vx-Jx)`, plus `3*(2*vz-Jz)` for the second wall. Kinetic energy plus
potential change `20*(y-1)` must not exceed that work plus `1e-10`. The single-wall
first substep records spin .0625, energy9.0015625 and imposed work18.15.

A private response test additionally queues downward force `(0,-20,0)` and yaw
torque `(0,.8,0)` on the parked body. It compares prepared, unprepared and uncached
paths through two calls against the doubled floor load and yaw increment `h`.
This uses an internal history seam because public force/torque mutations already
wake their body. Existing rollback and negative admission tests remain active.

## Replay and evidence

[Raw results](contact-wake-forces-2026-10-01.json) record code producer
`f64dc6ab965a1b255b4e8380ac0fbb7aad6a3f98` and shared convention sourceRevision
`e6acb5310afaf15c0cba24f87108f5f4ad1bedc3`. The 16 rows repeat identically twice
in native release and three times in release WebAssembly within each target.
Each fixture compares complete checkpoint bytes with a fresh repeat and an
explicit restore after each of nine calls. Its placeholder context is private
same-process evidence, not a portable save identity. Algorithm advances from3 to4;
format2 keeps its layout and rejects prior continuation algorithms. Cross-target
bit replay is not promised.

Counters belong to the first observed call and include discarded wake-probe
geometry. Auxiliary replay, restored and continuation work is excluded from
those counters. The motion error covers all nine calls. Response and inertia
preparations equal the configured substep count, with at most eight velocity
rounds per substep and eight visits per admitted point. There is no timing or
total-driver-work claim. The dedicated WebAssembly example also runs the existing
admission, primitive, moving-support and narrow-support controls; it does not
enter the production Pages module. Historical algorithm-3 admission measurements
remain unchanged; their current work oracle includes the newly restored gravity
impulse under algorithm4.

```sh
cargo test --release --locked --test contact_wake_forces -- --nocapture
cargo build --manifest-path demo-wasm/Cargo.toml --release --locked \
  --target wasm32-unknown-unknown --example dense-contact-contract
node scripts/test-dense-contact-wasm.mjs \
  demo-wasm/target/wasm32-unknown-unknown/release/examples/dense_contact_contract.wasm
```
