# Soft-contact and relaxation experiment (#158)

Status: opt-in research comparison. `Config::soft_contact` defaults to `None`, retaining the
merged Baumgarte solver and its convergence checks. Neither the existing Pages scenarios nor
the default approximation reset silently select this experiment. Passing selected fixtures is not
a default-promotion decision: the relaxed method trades cheaper settling in some impacts against
more work and slower settling in others. A faster or quieter completed trace is not sufficient.

## Numerical method

`Some(SoftContact)` prepares mass-independent spring coefficients once per substep. The
experiment uses frequency 60 Hz and dimensionless damping ratio 1. With substep duration h,
frequency is capped at `0.25 / h`. The cap is a conservative experiment policy, not a proof of
stability at arbitrary scale. Default 60 Hz ticks still have four substeps and an eight-pass
primary ceiling. The original 0.02-unit slop and 60-unit/s correction-speed cap remain intact.

```
omega = 2*pi*frequency
A = 2*damping_ratio + h*omega
B = h*omega*A
bias_rate = omega/A
mass_scale = B/(1+B)
impulse_scale = 1/(1+B)

bias = min(60, bias_rate * max(0, -separation - slop))
delta = mass_scale * effective_mass * (bias - normal_velocity)
        - impulse_scale * accumulated_normal_impulse
next = max(0, accumulated_normal_impulse + delta)
apply(next - accumulated_normal_impulse)
```

This follows the soft-constraint parameterization described by Erin Catto in
<https://box2d.org/posts/2024/02/solver2d/>. It is a compliant contact constraint, not a change
to global linear/angular damping. The same pair impulse application and friction kernel are
used. Fixed supports (mass zero) keep a hard normal constraint while using the selected correction
bias rate. Compliance applies to overlapping dynamic pairs. Speculative separated contacts and
fresh restitutive impacts also retain the hard normal response and their existing speed targets. Collision admission, nearest/equal-hit shielding, rotation locks and numerical
ownership remain in the same Rust world.

Early stopping uses the **soft** complementarity residual, including the compliance times
accumulated impulse. It cannot reuse the rigid zero-relative-velocity condition. Friction
stationarity and final-whole-pass checks remain required; large impulses absorbing a small
increment cannot establish convergence by themselves. The default hard kernel is a separate
compile-time specialization with the original floating-point evaluation order.

## Optional relaxation

`relaxation_iterations = 0` tests softness alone. A value of 2 adds at most two bias-free passes
per nonempty substep. Thus the maximum total is **40 passes per tick**, not 32 relabeled as 32.
Primary and relaxation counts, constraint visits, residual visits and skipped passes are
reported separately. Empty contact sets do not pay for a relaxation pass.

Before relaxation, save the velocities used for movement. Relax the velocities and accumulated
impulses using the same frozen contact geometry and inertia; then advance each pose **once**
using the saved movement. The relaxed velocity and impulse cache seed the next substep. This
ordering is equivalent to integrating the saved motion first and then applying a solve with
frozen pre-integration geometry. It deliberately avoids applying an old effective mass with a
new orientation. It is NOT a fresh post-integration contact solve or rotational CCD.

Relaxation removes penetration bias and softness, but preserves the permitted closing speed of
speculative contacts and the intended restitution speed of genuine bounces. Retirement is
recorded from the primary admitted impact and cannot be erased by a later impulse decrement.
Sleep uses both physical and saved movement speeds; bodies cannot sleep merely because a bias
correction was removed from the stored velocity while still moving their poses appreciably.
There is no second gravity application, global drag, forced sleep, or dropped requested time.

The default path allocates no correction scratch. Opt-in relaxation retains per-body motion and
per-row bias buffers. Their payload bytes are separately counted; allocator overhead is not
included. The normal hard/soft flag fits the existing constraint storage; the default replay
comparison still checks every pre-existing work/memory stat, including stat 23.

## Observation and acceptance

The dedicated WASM export `approximate_reset_soft_from_sandbox(substeps, iterations, frequency,
damping_ratio, relaxation_iterations)` imports the same Rust fixture as the default reset. It
validates inputs before replacing the experiment. Existing reset exports remain unchanged.
Read-only angular-velocity and kinetic-energy exports support measurement outside timed steps.
`Body::kinetic_energy` includes local-frame rotational inertia. Units are mass-units times
scene-units squared per second squared, NOT SI joules without a conversion convention.

Generic stats 0..51 are retained. New stats are:

| Index | Meaning |
|---|---|
| 52 | Softened primary contact points |
| 53 | Actual relaxation passes, also included in total stat 5 |
| 54 | Relaxation constraint visits |
| 55 | Relaxation residual constraint visits |
| 56 | Skipped relaxation passes |
| 57 | Retained relaxation motion/bias payload bytes |
| 58 | Overlapping non-bouncing fixed-support points kept hard |

The tower matrix compares the real merged module, the same-binary Baumgarte policy, softness,
and soft-plus-relaxation. It enforces unchanged default physics/work histories; same-policy
repetition; actual crate response on hits; unchanged sleeping poses on misses; rotation locks;
finite state; bounded work; full elapsed time; and the existing 0.5-unit floor-penetration bound.
Cross-policy hashes may differ. Record settling, kinetic energy, jitter, sleep transitions and
active/sleeping timing separately. A lower mean after contacts/trajectories change does not
prove equal-work throughput, and post-sleep savings are not active-solver speedups.

## Mixed-mass counterexample and general fixed-support rule

`examples/contact_correction_quality.rs` retains a distinct tilted-stack/mass/friction matrix.
Four 36-unit crates start at heights `18 + 38*level`, the top is tilted by 0.05 radians, and it
receives a 50-unit off-center impulse after 240 ticks. All policies run 1,200 ticks total.
The mixed masses are `[0.5, 4, 1, 2]`. The initial experiment softened fixed as well as dynamic
contacts: Baumgarte reached ~0.766 floor penetration, soft-only ~1.233, and relaxed ~0.771.
All exceed the unchanged 0.5-unit bound. Settling to zero energy did not cancel those failures.
A preliminary 30 Hz tower probe also exceeded that bound; varying constraint damping from
0.5 through 4 did not repair the mixed-mass failure. Those investigations are rejected evidence,
not accepted policies or altered thresholds.

The next candidate keeps mass-zero support normals hard, without a scene-specific switch or
changing the floor thickness. In the local release comparison:

| Tilted fixture | Baumgarte peak floor penetration | Soft, hard supports | Relaxed, hard supports |
|---|---:|---:|---:|
| Equal mass | 0.14412 | 0.06395 | 0.03213 |
| Mixed mass | 0.76567 (fails) | 1.31755 (fails) | 0.28012 |
| Low friction | 0.10225 | 0.03967 | 0.03458 |

The selected **relaxed** candidate passes the same bound in all three fixtures. Baseline and
soft-only control failures remain explicit (`all_policies_passed: false`); they are not marked
as candidate successes. The command gates the chosen relaxed policy, not the historical control,
and exits nonzero whenever that candidate fails. Every record uses the same 0.5-unit criterion.
`--report-only` collects identical failed records for investigation without relabeling them.
This additional gate does not replace or relax any existing structural/physical acceptance.

The two-repeat 20-second post-shot tower comparison passes all 12 configurations with default
physical and old-work hashes matching the separately built merged binary. Relaxed free-sphere
and rigid hits settle at about 11.28 and 2.83 seconds versus baseline still active at 20 seconds
and 19.12 seconds respectively. Free-arrow settling is worse: about 12.02 seconds versus 1.20.
Relaxation reduces the observed penetration in all six tower-hit configurations. These are
fixture observations, not universal stability or speedup guarantees. Local long-run timings
include unrelated host load and are diagnostic; use an isolated repeated run for timing claims.
**Do not switch the default based only on sphere/rigid settling or zero final energy.**

## Reproduction

```
cargo test --locked --lib approximate::correction
cargo test --locked --lib approximate::convergence
cargo build --manifest-path demo-wasm/Cargo.toml --release --target wasm32-unknown-unknown --locked
TRIALS=4 TICKS=1200 node scripts/benchmark-correction.mjs candidate.wasm result.json merged.wasm
cargo run --release --locked --example contact_correction_quality -- heldout.json
```

The last command gates the relaxed candidate and currently exits 0 while retaining failed mixed-mass
control rows. The standard performance job runs it without report-only, after the existing structural
and canonical checks, and retains the complete report in the artifact.
The four-policy benchmark rotates run order over four trials, warms the actual modules, and
keeps raw physics-call timings separate from setup, assertions and observations. It reports
missing old-module energy telemetry as null, not a guessed value. Phase durations differ
between correction policies because their physical trajectories differ.

Known general approximation limits (frozen rotation within sweeps, non-chronological secondary
substep ricochets, omitted gyroscopic terms, no universal cross-target bit identity, and no
mid-step transactional rollback) are unchanged. See `fixed-step-approximation.md`.
