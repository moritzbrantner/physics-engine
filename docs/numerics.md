# Numerical policy

Physical state is `f32`: `approximate::Real` is `f32` in the only build. Rapier is the independent
reference ([technical decisions](technical-decisions.md) D1/D3); there is no f64 reference build.

What stays `f64`:

- `numeric::Scalar`: the `step(dt)` argument, accumulated elapsed seconds and the checkpointed
  prior-substep duration. Each substep narrows its `dt` to `Real` once;
- the `geometry-kernels::primitive3` kernels.

**Kernel boundary.** `src/approximate/primitive.rs` widens `Real` inputs exactly to f64 (`widen`)
and narrows results conservatively: time of impact and sweep fractions toward zero, so an impact
never moves later (`toward_zero`); bounds/extents outward (`round_up`); separation toward smaller
(`round_down`); directions and witnesses to nearest (`nearest`). Quaternions are renormalized in
f64 before building the kernel frame. A swept impact is re-checked after narrowing with an
allowance of `NARROWING_ULPS` (16) `Real::EPSILON`, scaled by the pose magnitude.

**Precision envelope** (documented, not enforced by extra rejections):

| Quantity | f32 envelope |
| --- | --- |
| position component | \|x\| ≤ 16384 units (ulp ≤ 0.002 ≈ default `contact_slop` 0.02 / 10) |
| linear speed | ≤ 4000 units/s with default substeps |
| gravity | default `(0, -3600, 0)` |
| shapes | dimensions ≥ 0.25 units |

`tests/f32_precision_envelope.rs` holds the adverse controls at the edge of the envelope.
Length-unit-relative tolerances replace these absolute numbers under #291.

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
