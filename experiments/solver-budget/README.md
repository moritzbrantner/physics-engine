> Reconciled with the island-aware engine. Runs use this checkout's default convergence scope,
> recorded in `solver_policy`, not a duplicate implementation. The earlier 815-replay report
> measured revision `3074f49c` before contact-island convergence. Its timing and quality values
> are historical observations, not measurements of a new checkout. Regenerate curves to compare.
> Full sweeps remain manual; CI checks the harness and a small repeatability/quality smoke matrix.

# Solver budget vs contact capacity

A parameter experiment using the **actual `physics_engine::approximate::World` f64
implementation**, compiled to release WebAssembly. The production library and
Tower Stability defaults are not modified. This is a separate diagnostic crate,
not another solver or a JavaScript simulation.

## Questions and controls

Sweep the maximum velocity/contact passes, fixed-position correction passes and
substeps independently, plus combinations. The reference is **4 substeps × up to
8 velocity passes + up to 2 position passes per substep**, as selected by the
canonical tower on `3074f49ca2219b89cd093af6b6f8099bb387b887`.

An iteration is not a physical bounce. A contact pass revisits all eligible
constraints. This experiment never caps actual impacts, disables swept collision
detection, stops after a time budget, changes materials or drops elapsed time.
The existing early-convergence checks remain enabled, so actual work, not just
configured maxima, is reported. Zero position passes is a deliberate ablation;
zero velocity passes and zero substeps are invalid.

## Run

Use the repository Rust toolchain and a Node version supporting `toSorted`
(Node 22 was used for the retained measurements). No package download is needed
for the Rust crate: its only dependency is this checkout's physics engine.

```sh
rustup target add wasm32-unknown-unknown
cargo test --locked --manifest-path experiments/solver-budget/Cargo.toml
cargo clippy --locked --manifest-path experiments/solver-budget/Cargo.toml --all-targets -- -D warnings
node --test experiments/solver-budget/report.test.mjs
cargo build --release --locked --manifest-path experiments/solver-budget/Cargo.toml --target wasm32-unknown-unknown

COUNTS=16,32,64,128,256 TRIALS=3 TICKS=300 \
  node experiments/solver-budget/run.mjs \
  experiments/solver-budget/target/wasm32-unknown-unknown/release/physics_solver_budget.wasm \
  experiments/solver-budget/results/sweep.json
```

`PROFILES=4s-8v-2p,4s-4v-2p` selects profiles; see `report.mjs` for the full list.
`SCENES=0,1,3` runs the capacity fixtures. `SCENES=2` is a sleeping-enabled
control and **must not be used as active capacity evidence**. `TICKS=1200`
requests a longer 20-second trace. Counts are ascending powers of two in 8..2048;
ticks are 240..3600 and repetitions 1..9. A single repetition is exploratory,
not qualified capacity because repeatability has not been established.

Run timings serially on an otherwise idle host. Do not run builds, native tests
or a second benchmark simultaneously. Raw outliers remain in the output.
A partial `.ndjson` file survives interruption; only a completed `.json` report
has repeatability and capacity classification. Interrupted output is not a pass.

## Fixtures (all freely rotating, 36-unit boxes, mass 2, friction 1)

- **Sustained stack:** four levels of edge-to-edge boxes, expanding a rectangular
  footprint as population increases; no external disturbance. Sleeping is
  explicitly disabled to measure continuing contact work. The 32-box dimensions
  are 4 × 2 × 4, not a thinner stack for cheaper settings.
- **Mixed impacts:** the same awake stack with sphere, arrow and rigid-projectile
  waves at ticks 60, 120 and 180. One shot per four front-face columns; identical
  scheduling for all budgets. CCD and impact-retire are enabled. Bodies that
  depart beyond 4500 scene units are removed as declared fixture lifecycle work.
- **Shallow contact:** one awake layer at every population, to distinguish a
  shallow contact graph from coupled four-level support. It is intentionally
  easier than the stack and not a substitute acceptance case for the tower.
- **Sleeping control:** the unforced four-level stack with ordinary sleeping.
  This shows why counting sleeping bodies as collision capacity is misleading.

Counts mean dynamic boxes, **plus one fixed floor and any live projectiles**.
The floor is large enough for the tested footprints. This does not include the
Pages room's other ten fixed bodies, its character controller, or rendering.
It reuses the engine, crate dimensions/mass/friction and tower budgets—not the
complete first-person demo adapter. Results are not general shape capacity.

## Measurement and quality

One 1/60-second `budget_step()` call is timed. Its real world step and declared
projectile insertion/removal are included. Snapshot extraction, hashing,
independent geometry checks, JS orchestration and rendering are excluded.
The first 60 ticks are retained separately as startup. Remaining ticks are
classified active vs sleeping. All capacity fixtures keep every box awake.
For 300 ticks, this is five simulated seconds: one startup, four measured active.

Every tick also records contact-point work (summed across substeps), solver
constraint visits, actual passes, narrow/broad-phase work, integrated bodies,
and projectile sweeps. A diagnostic scratch-payload accessor is available but
the current sweep does not record it; no memory-capacity conclusion is made. Independently observed touching
box pairs and touching body counts are reported separately: these geometric
pairs within 0.02 units are not the same as solver-admitted constraint rows.

Quality checks are outside the timed call and never modify state:

1. Every body state is finite, orientations normalized, all boxes/floor retained,
   and complete requested time advanced. Real impact response/CCD and configured
   work ceilings are checked. Repeated cases must have identical full observed
   state and work hashes within the same build/target.
2. Floor penetration ≤ **0.5 scene units** is the existing canonical tower limit.
3. Independent 15-axis oriented-box SAT measures inter-box penetration ≤
   **1.8 units** (5% of box width). This is a **new experimental screening limit**,
   not a claim about an existing engine acceptance threshold.
4. An unforced stack/carpet must stay within **1.8 units** of its initial box
   center positions. This is another declared experimental screen. It prevents
   capacity credit for a cheap solver whose stack has spread out/collapsed.

All lower-budget failures remain in the curves and report. There is no early
stop on quality failure: continue simulating the full requested interval unless
the engine returns an error. Faster execution after objects fall apart is **not**
a performance win for an equivalent contact workload. This is why the observed
contact counts, first-failure time and quality values accompany timing.

These are tick-boundary checks, not proofs of absence of internal-substep
penetration, rotational CCD completeness or long-term fidelity. Five-second
curves are short-window evidence; run longer validation before promoting a
budget to the real demo. Floor-only position correction cannot guarantee
separation between dynamic boxes.

## Summarize

```sh
node experiments/solver-budget/summarize.mjs experiments/solver-budget/results/sweep.json experiments/solver-budget/results/summary.csv
```

The CSV preserves actual pass counts, geometric contact load, timing and every
failure category. It refuses an incomplete matrix. Do not pool different hosts
or independent sessions as though they were one controlled comparison.

## Capacity interpretation

For physics-only budgets of 4, 8 and 16.667 ms per tick, the report separately
shows (a) the largest tested population under the worst-repetition p95 timing
and (b) the largest **contiguous tested** population also passing every quality
and repeatability gate. It never skips a failed smaller size to advertise a
higher accidental pass. Reaching the end of the tested grid is a lower bound,
not a measured maximum. No linear extrapolation or FPS claim is made. A p95
budget is not a guarantee every frame fits; maxima and every raw sample remain.

No wall-clock pass/fail gate is added to normal CI. The optional manual workflow
can retain this evidence without making the ordinary PR path expensive.

## Calibrate against the real Tower Stability page

The scaling fixtures intentionally omit the room/controller. A second runner
uses the **existing browser adapter and exact 32-crate/player/11-fixed-body room**.
An explicit diagnostic feature exposes a validated budget reset; it is absent
from normal builds, and `approximate_reset_tower` still selects 4/8/2.

```sh
cargo test --manifest-path demo-wasm/Cargo.toml --features solver-budget-experiment \
  diagnostic_budget_rejects_invalid_limits_without_resetting_and_default_stays_fixed
cargo build --release --locked --manifest-path demo-wasm/Cargo.toml \
  --target wasm32-unknown-unknown --features solver-budget-experiment
node experiments/solver-budget/canonical.mjs \
  demo-wasm/target/wasm32-unknown-unknown/release/physics_engine_demo.wasm \
  experiments/solver-budget/results/canonical.json
```

Calibration covers free rotation/physical character, impact-retire and physical
bounce, repeated mixed hits, near misses, and an immediate arrow. Defaults are
two repeats, ten post-start seconds, seven budgets. Ordinary sleeping stays on;
active and quiescent timing is separated. It checks the existing finite pose,
normalized orientation, room inventory, elapsed time, hit response, retiring
miss isolation and floor-penetration rules, plus the declared new 1.8-unit
unforced pre-shot drift screen. It does **not** run the scaling fixture's
independent pair-overlap oracle. Its quality label is therefore not interchangeable
with scaling quality. Failed budgets are retained, not silently accepted for
production. No frontend controls or defaults are changed by this feature.

## Plot curves

With Python and Matplotlib available:

```sh
python experiments/solver-budget/plot.py experiments/solver-budget/results/sweep.json experiments/solver-budget/results/curves
```

Every chart uses measured counts and the worst repetition's p95; crosses mark
quality failures. There is no capacity interpolation and no graph of sleeping
bodies labeled as active load. `--profiles` selects other IDs from the report.
Generate plots after timing, not while a benchmark is using the host.
