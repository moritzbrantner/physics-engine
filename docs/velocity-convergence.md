# Bounded velocity convergence (#157)

The fixed-step solver retains four substeps and an eight-iteration ceiling. Its default
`Config::convergence = Some(Convergence::default())` may finish a nonempty contact set early.
`None` retains the fixed-pass reference, including the old work accounting on empty sets.
The event solver is unchanged. This is an approximation to eight iterations, not a promise
of bit-identical trajectories between policies.

## Decision and scope

Probe after complete passes 2, 4, 8, ... strictly below the configured ceiling. One-pass
and two-pass configurations therefore still execute their requested nonempty solves.
A contact-free substep needs no velocity iterations; force/impulse application, integration,
CCD discovery and elapsed time continue normally. Quiet-world short circuits are unchanged.

The entire prepared contact set must pass; this slice does not schedule independent islands
separately. Within a probe, the first large correction ends observation for that pass; the
remaining constraints run the same uninstrumented row kernel. Hard stacks receive their full
iteration ceiling without paying for a residual scan every pass. No wall-clock measurement
or previous frame's convergence flag influences the decision.

Default tolerances:

- Absolute impulse: `1e-7` mass-units * scene-units/second.
- Absolute velocity: `1e-5` scene-units/second.
- Relative: `1e-6` applied to the contact's impulse scale and each row's velocity scale.

All are finite, nonnegative; relative tolerance is limited to 0.01. Zero tolerances request
exact checks. These defaults are tested for the documented 36-unit crates, not arbitrary
scene scaling. Excessively large user-selected tolerances can reduce quality.

## Both correction and residual are required

During a complete probe pass, every normal/tangent impulse change must meet both the impulse
limit and its effective-mass-scaled velocity limit. Per-row velocity scales are used: a large
sideways speed cannot loosen the normal error criterion. This also prevents a tiny mass from
hiding a large velocity error behind a small impulse.

If all changes are small, recompute the residual against the **final velocities of the whole
pass**. A later contact may have disturbed an earlier one. The normal residual honors the
unilateral constraint: separation is permitted when its accumulated impulse is zero.
Tangential residuals use projection to the Coulomb disk, so sliding at the friction limit is
not mistaken for an unsatisfied sticking constraint.

Also check normal complementarity and the friction disk's first-order stationarity directly.
Otherwise adding a small correction to a huge accumulated impulse could round back to the
same f64 value and falsely report a zero residual. Only an eight-ULP boundary allowance is
used to classify disk-edge rounding, not an extra physical contact margin. Non-finite
residuals/limits never admit an early exit.

Warm-start impulses are still carried forward. Contact generation, CCD, material parameters,
penetration correction, sleep thresholds and body ordering are unchanged. A tolerance-based
exit may nonetheless produce slightly different trajectories or sleep timing; report those
observations, do not label cross-policy equality as guaranteed.

## Evidence and instrumentation

`Report::convergence` and WASM `approximate_stat(40..=51)` expose, in order:
constraint visits; residual scans; residual row visits; converged substeps; capped substeps;
empty substeps; skipped iterations; maximum exit-pass impulse delta; maximum accepted exit
velocity residual; fixed-reference substeps; probe passes; observed correction rows.

A capped substep is not certified converged. Zero residual with zero scans means unmeasured.
The maximum exit values describe accepted exits only, not an unmeasured global solver error.
For successful active ticks, actual + skipped iterations equals substeps * configured ceiling.
Zero/quiescent ticks have zero convergence work. Instrumentation counts source work, not CPU
instructions or a global allocation profile.

The adapter adds `approximate_reset_fixed_iterations_from_sandbox(4, 8)` for the reference.
The ordinary reset selects the default convergence policy. Both import the same Rust fixture;
there is no JavaScript physics or per-frame state round-trip.

```sh
cargo test --locked --lib approximate::convergence::
cargo test --locked --test fixed_step_approximation
cargo build --manifest-path demo-wasm/Cargo.toml --locked --release --target wasm32-unknown-unknown
TRIALS=6 node scripts/benchmark-convergence.mjs candidate.wasm convergence.json baseline.wasm
```

The optional older binary checks exact fixed-reference physical and legacy-work parity and
is measured repeatedly alongside both candidate policies. All twelve free/upright sphere/arrow/rigid
hit/miss cases must complete, preserve rotation locks and sleeping near misses, satisfy the
existing 0.5-unit floor-penetration limit, and repeat their same-policy histories and decisions.
The report records cross-policy position/velocity/quaternion deltas and sleep differences.

Settling, non-quiescent post-shot calls and quiescent calls are timed separately. Setup,
observations and assertions are excluded. Residual checking costs are included. Keep raw
samples, source and module identity. Do not claim a speedup solely from fewer passes, average
sleeping ticks into active throughput, or relax physical gates to make early stopping pass.

When a baseline module is supplied, schema v2 measures all three variants repeatedly in
balanced order: the merged module, candidate fixed-pass reference, and candidate convergence.
It checks fixed-reference parity on every repeat and retains both time comparisons; same-binary
policy gains must not conceal an extraction/code-generation regression against the merged module.
Normal PR CI repeats the two candidate policies twice; detailed investigations supply the older
module and use six balanced three-way trials. Future intentionally changed contact semantics require
an explicit new reference-parity contract, not relabeling reference mismatches.
