# Physics improvement integration ledger

## Production baseline and retained guarantees

The reconciliation starts at merged #167, `92312a5d36ec1ed9cbafad92e45b2012affa27de`
(tree `6e08b8520410756e90b5d5ad89e29f75df21cf41`). This already includes prepared response
coefficients (#159), active membership/contact adjacency/scratch (#160), contact geometry preparation
(#161), bounded convergence (#162), the canonical tower runtime (#164), fixed-position preparation
(#165), and current-response contact islands (#167 / #166). Do not reapply those changes or substitute
an earlier experimental engine snapshot for this baseline.

CPU simulation defaults to f64. `exact-reference` stays an explicit diagnostic backend, not an
implicit fallback. The tower keeps 4 substeps, at most 8 primary velocity iterations and 2 fixed-
position passes. Current island convergence, swept contact admission, support removal, local wake,
projectile lifetime, shared settings, rendering and complete time advancement are retained.

## What is integrated, and what is not promoted

| Work | Integration policy |
|---|---|
| Established runtime/performance changes | Already on main; retained and regression-tested |
| #163 soft-contact and relaxation study | Reconciled with current islands/positions; compile-time `experimental-soft-contact`, disabled by default |
| Solver-budget tooling formerly supplied as a local patch | Repository diagnostic crate plus explicit demo `solver-budget-experiment` feature; no settings change |
| Old timing tables and 815-replay budget report | Historical evidence at their recorded SHAs, not new measurements of this reconciliation |
| Dense 128-body overlap finding | Still open; neither merging diagnostics nor passing the tower matrix repairs this separate quality limit |

A normal Cargo/Pages build excludes experimental soft-contact config, row coefficients, world scratch,
report fields and the soft-reset export. Even a feature-enabled build defaults to `soft_contact: None`.
The budget reset export is separately gated. `scripts/check-diagnostic-exports.mjs` tests each compiled
surface and the Pages build rejects either diagnostic reset in a production artifact. The two features
can be compiled together; neither silently enables the other. Diagnostic artifacts are stored under
separate filenames and never overwrite the production comparison artifact.

The root default API remains unchanged except for an additive, read-only `Body::kinetic_energy`
observation. Optional Config fields are only available when the corresponding feature is enabled.
The soft adapter imports a fixture once; it does not write f64 poses back into the legacy event world.

## Semantic reconciliation

Soft constraints use the current row/residual implementation and current island/global/fixed scheduling.
A separate legacy soft loop would discard local convergence and was not retained. Extra relaxation uses
frozen geometry and coefficients, carries the final impulses/velocities forward, and does not integrate
time twice. Its work counters cannot overwrite or certify the primary solve. Fixed-position correction
still executes after integration and before sleep. Existing failed experimental controls remain failures.

The budget experiment records its actual convergence scope. Reports record the runner's current git
revision/tree/dirty state (or null without metadata) and the supplied WASM hash, rather than hardcoding
an obsolete source revision. `runner_source` alone does not prove an arbitrary input binary was built
from that checkout: retained build provenance/binary hashes provide that association in CI. Full sweeps
remain manual; normal performance CI runs reporting/unit tests and a small deterministic smoke matrix.
Quality-failing low budgets are retained and excluded from qualified capacity, not relabelled passes.

## Validation entry points

```sh
cargo test --locked
cargo test --locked --features exact-reference
cargo test --locked --features experimental-soft-contact
cargo test --locked --features exact-reference,experimental-soft-contact
cargo test --locked --manifest-path demo-wasm/Cargo.toml --all-features
cargo test --locked --manifest-path experiments/solver-budget/Cargo.toml
node --test site/*.test.mjs scripts/*.test.mjs experiments/solver-budget/*.test.mjs
```

Build production, soft-only, budget-only and combined WASM with the respective feature sets and run
`node scripts/check-diagnostic-exports.mjs <artifact> production|soft|budget|combined`.
Use `scripts/benchmark-tower-position.mjs` for strict feature-OFF/baseline tower parity, including all
prior memory/work counters. The feature-enabled correction-policy comparison intentionally reports
constraint payload bytes separately (stat23) because experimental per-row fields exist there; physical
and other prior work observations remain checked. No quality threshold or structural budget is relaxed.

Use `scripts/benchmark-correction.mjs` and the selected-candidate quality example for the optional study;
use `experiments/contact-islands/run.mjs` for global/island controls. Defaults and reference trajectories
are checked separately from intentionally different correction policies. Same-build replay evidence is
not a promise of cross-platform identity, universal nonpenetration, or arbitrary-scene performance.
