# Numerical policy

Physical state is `f32`: `approximate::Real` is `f32` in the only build. Rapier is the independent
reference ([technical decisions](technical-decisions.md) D1/D3); there is no f64 reference build.

What stays `f64`:

- `numeric::Scalar`: the `step(dt)` argument, accumulated elapsed seconds and the checkpointed
  prior-substep duration. Each substep narrows its `dt` to `Real` once;
- the `geometry-kernels` primitive and checked continuous-collision kernels.

**Kernel boundary.** `src/approximate/primitive.rs` widens `Real` inputs exactly to f64 (`widen`)
and narrows results conservatively: time of impact and sweep fractions toward zero, so an impact
never moves later (`toward_zero`); bounds/extents outward (`round_up`); separation toward smaller
(`round_down`); directions and witnesses to nearest (`nearest`). Quaternions are renormalized in
f64 before building the kernel frame. Moving sphere pairs call the checked analytic fraction
kernel directly with the same widened arithmetic; stationary pairs retain the primitive contact
query and its validation. A swept impact is re-checked after narrowing with an
allowance of `NARROWING_ULPS` (16) `Real::EPSILON`, scaled by the pose magnitude.

**Precision envelope** (documented, not enforced by extra rejections):

| Quantity | f32 envelope |
| --- | --- |
| position component | \|x\| ≤ 16384 units (ulp ≤ 0.002 ≈ legacy `contact_slop` 0.02 / 10) |
| linear speed | ≤ 4000 units/s with default substeps |
| gravity | legacy scene-unit `(0, -3600, 0)`; the default is `(0, -9.81, 0)` m/s² × `length_unit` |
| shapes | dimensions ≥ 0.25 units |

`tests/f32_precision_envelope.rs` holds the adverse controls at the edge of the envelope.
Length-unit-relative tolerances replace these absolute numbers under #291.

## Portable float math (#576)

std's transcendental float functions (`sin`, `cos`, `atan2`, `hypot`, `exp`, `powf`, …) call
the platform's C library (glibc, the MSVC CRT, Apple's libm; compiler-builtins' musl port on
wasm32), which differ in the last bit; the solver amplifies that within a few steps, so a native
host and a browser client predicting against it drifted apart (moenarch-games/shooter#120). The
engine calls the `libm` crate instead, the same Rust code on every target: the revolute joint
angle and `Quaternion::rotation_to` (`atan2f`), the character's `max_slope` (`cosf`) and the
friction disk (`hypotf`). `clippy.toml` bans std's versions in `src/` (tests and the oracle may
still use them for setup). IEEE `sqrt`, `mul_add` and the basic operations are correctly rounded
everywhere, and Rust never contracts `a * b + c` into a fused multiply-add on its own.

`tests/cross_target_hash.rs` pins the state hash of a fixture covering those calls (a robot on
motored wheels with a turret servo, a pose-driven turntable, a character on a ramp, boxes
sliding with friction); `scripts/cross-target-wasm.sh` (run by `scripts/build-pages.sh`) checks
the same hash on wasm32. With std's functions the native hash differed. Not covered:
`geometry-kernels` still calls std's functions in submerged-volume (media) and heightfield
normal preparation, so worlds with media or terrain are not yet portable.

## Safety and tolerances

- Reject NaN, infinity and negative durations at input boundaries; never hide numerical failures
  with saturating casts.
- Swept bounds round outward so rounding cannot narrow the volume admitted to collision detection.
  [Numerical candidate guards](floating-conservative-bounds.md) are independent of physical
  contact slop and never become position correction.
- Compare mathematically equivalent expressions with documented, scale-aware tolerances
  (`Real::EPSILON` times the magnitude under test). Keep exact comparisons for ids, masks,
  counters and repeated execution of the same input sequence.
- Do not weaken a collision, tunneling, support or failure-propagation test to make it pass.
