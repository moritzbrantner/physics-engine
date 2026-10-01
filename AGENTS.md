# Agent guidance

## Authority

This repository owns reusable physics semantics. Domain-neutral geometry and algorithm kernels belong in `rust-kernels`; Physics Engine should consume those kernels rather than duplicate them, while retaining authority over body/world state, contact persistence, CCD admission, wake/sleep policy, and response. Keep ECS, rendering, game-loop, scene, editor and UI concepts outside the engine.

## Numerical policy

- Floating point is the default, not an exception. Use `numeric::Scalar` (`f64`) for new CPU physics math/state; use `f32` when a documented precision budget permits it.
- Do not impose a "no floating point anywhere" constraint. Do not grow exact multi-limb fractions to avoid ordinary floating-point error.
- The `exact-reference` feature is diagnostic only. Never enable it implicitly for a production or Pages build.
- Retain integers for IDs, flags, counters and explicit legacy serialization boundaries, not as a mandatory representation for continuous physical quantities.
- Validate finite values and ranges, use explicit scale-aware tolerances, and keep swept bounds conservative. Do not hide numerical failures with saturating casts.
- Follow `docs/numerics.md`. The remaining integer body/geometry APIs are compatibility surfaces awaiting the floating-state/contact-solver migration.

## Determinism

- Preserve stable `BodyId` ordering for pair traversal and simultaneous events. Define replay guarantees by build/target; do not equate exact arithmetic with physically accurate or automatically cross-platform deterministic simulation.
- Keep authoritative state and collision decisions independent of JavaScript/rendering.
- Treat Q32.32 subticks as an implementation detail unless a public contract explicitly requires exposure.
- A collision repair must address tunneling/contact semantics rather than hiding a failing test or increasing an event limit without evidence.

## Continuous collision detection

Fast-moving bodies must be tested across the requested interval. Do not replace swept time-of-impact checks with frame-end overlap checks.

## Migration from ecs-lab

Move proven physics concepts in coherent slices and remove ECS-specific ownership while doing so. Do not bulk-copy experimental APIs merely to increase feature count.

## Parked-body activation

- Projectile creation, broad-phase proximity, and unrelated projectile removal are not wake evidence.
- Admit swept/analytic contacts before restoring parked dynamics, and apply impulses only after real dynamic mass/inertia is active.
- Cover same-step ricochets and support removal. Fixed floors must not connect otherwise independent dynamic islands.
- Include discarded wake-probe work in performance evidence; follow `docs/contact-wake.md`.

## Layout and commands

| Path | Role |
| --- | --- |
| `src/` | The `physics-engine` crate: translational `World`, `RotatingWorld3d`, `approximate::World` (floating-state solver), queries, checkpoints |
| `demo-wasm/` | Narrow WASM adapter for the Pages sandbox and WASM contract examples; owns no physics rules |
| `site/` | Pages scenario UI over `demo-wasm`; never reconstructs collision truth in JavaScript |
| `experiments/` | Opt-in diagnostic crates (contact islands, solver budget); never production defaults |
| `.performance/` | Performance contract and the append-only ratchet history |

```sh
cargo fmt --all -- --check
cargo clippy --all-targets --locked -- -D warnings
(cd demo-wasm && cargo clippy --all-targets --locked -- -D warnings)
cargo build --locked
node scripts/check-instrumentation-neutrality.mjs --full   # full cargo test + counters on/off parity
node --test scripts/performance-ratchet.test.mjs && node scripts/performance-ratchet.mjs check
bash scripts/build-pages.sh                                # Pages build + WASM contract tests
bash scripts/run-sandbox-performance.sh                    # performance tier (advisory timings)
python3 scripts/test-tower-browser.py --url http://127.0.0.1:8765   # Playwright, after build-pages.sh; not in CI
```

The first five lines are the `fast` tier in `.coding-tooling.json`; `coding-tooling` runs it in CI. `nix develop` provides the CLI environment CI uses.

## Execution scope

These rules govern how work is sliced and when expensive checks run. They never relax the authority, numerical, determinism or CCD rules above.

- **One task = one branch = one PR.** A task is one `agent-task` issue, usually a bounded slice of a roadmap issue (#191 and its children). Deliver its complete declared scope on one branch, including tests, ratchet/evidence entries and docs it requires, in small commits. Do not split a task into new issues or follow-up PRs on your own; if it cannot land as one PR, stop and propose the split on the issue.
- **Stay inside the task.** Do not start tooling, CI, pin-refresh or unrelated cleanup unless the task cannot be completed without it. Note unrelated findings in one line of the PR description; do not open issues for them.
- **No new ratchets unless the task asks for one.** Do not add work ceilings, evidence collectors or gates on your own initiative. Existing ratchets stay; when a task legitimately improves work counts, record the entry in `.performance/ratchet/history/` per `docs/performance-ratchet.md` in the same PR. Never relax a ceiling or a physical threshold to make a check green.
- **One version bump per task.** Settle checkpoint format/algorithm changes (`docs/floating-checkpoints.md`) and public API changes (`docs/world-api-contract.md`) before implementing; a task bumps each checkpoint version at most once.
- **Validate in tiers.** While iterating, run the focused tests for the touched module. GitHub Actions is the full gate: `validate.yml` runs the fast tier, the long tower and Pages-adapter replays, the performance ratchet and the Pages build with its WASM contract tests; `performance-evidence.yml` packages advisory evidence for path-matched PRs. Before pushing, run locally only what CI does not cover: the Playwright scripts (`scripts/test-tower-browser.py`, `scripts/test-scenario-log-*.py`) for `site/` or browser-visible `demo-wasm` changes. A red CI check blocks merge; fix it rather than re-proving it locally.
- **Codex reviews the PR.** Codex reviews automatically when a PR is opened or marked ready, so open it only once the branch is complete. Address or explicitly answer every Codex finding before merge; after substantial fixes, comment `@codex review`.
- **Decide and continue.** When a task leaves a design choice open, pick the simplest option consistent with this file, record it in the PR description (or a doc under `docs/` when consequential) and keep going.
- **Short PR descriptions.** At most about 15 lines: what changed, checkpoint/API compatibility changes, one line naming the checks that ran, and anything not verified.

Tasks arrive as GitHub issues in the format, labels and pickup rules of `docs/AGENT_TASKS.md`; implement only `spec:ready` issues labeled for you. Claude Opus runs the loop with the `/orchestrate` skill (`.claude/skills/orchestrate/`); one `/orchestrate` works through every open issue until only Sol tasks or reported blockers remain.

## Done means

- The fast tier passes: format, Clippy (root and `demo-wasm`), build, and `scripts/check-instrumentation-neutrality.mjs --full`.
- The performance ratchet and the Pages build (WASM contract tests) pass; improvements append a ratchet history entry.
- Changes to checkpoint wire layout or continuation semantics bump the format or algorithm version and update `docs/floating-checkpoints.md`.
- Public API or capability changes update `docs/world-api-contract.md`, `docs/primitive-capabilities.*` and the README where they describe it.
- Browser-visible `site/` changes pass the relevant Playwright script locally.
