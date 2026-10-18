# Agent guidance

## Authority

The current direction is in [technical decisions](docs/technical-decisions.md): games use Rapier, Rapier is the behavioural oracle and performance baseline, and Physics Engine provides the Rapier capabilities its consumers actually use through its own performance-oriented API on one f32 `approximate::World`. Only Rapier-parity work for a capability listed in [consumer capabilities](docs/consumer-capabilities.md) is eligible; other issues stay open but parked.

Domain-neutral geometry and algorithm kernels belong in `rust-kernels`; Physics Engine consumes them and owns body/world state, contact persistence, CCD admission, wake/sleep policy, response and character movement. Keep ECS, rendering, game-loop, scene, editor and UI concepts outside the engine.

## Numerical policy

- Physical state is `f32` (`approximate::Real`); time is `f64` (`numeric::Scalar`). There is one build and no f64 reference build; Rapier is the reference.
- Keep `geometry-kernels` calls behind the conservative widen/narrow boundary in `src/approximate/primitive.rs`; see `docs/numerics.md`.
- Integers are for ids, flags and counters, not continuous physical quantities.
- Validate finite values and ranges, use scale-aware tolerances, and keep swept bounds conservative. Never hide numerical failures with saturating casts.

## Determinism

- Order pairs, events and query ties by stable body id. Results must be identical across worker counts on the same build and target; no cross-target bit-identity claim.
- Keep authoritative state and collision decisions independent of JavaScript and rendering.
- A collision repair must fix tunneling/contact semantics, not hide a failing test.

## Continuous collision detection

Fast-moving bodies must be tested across the requested interval. Do not replace swept time-of-impact checks with frame-end overlap checks.

## Layout and commands

| Path | Role |
| --- | --- |
| `src/` | The `physics-engine` crate: `approximate::World`, queries, opt-in checkpoints |
| `demo-wasm/` | Narrow WASM adapter for the Pages sandbox; owns no physics rules |
| `site/` | Pages UI over `demo-wasm`; never reconstructs collision truth in JavaScript |
| `oracle/` | Rapier oracle: every parity scenario on Rapier (`rapier3d =0.36.1`) and Physics Engine |

```sh
cargo fmt --all -- --check
cargo clippy --all-targets --features checkpoints --locked -- -D warnings
cargo clippy --all-targets --locked -- -D warnings
(cd demo-wasm && cargo clippy --all-targets --locked -- -D warnings)
cargo build --locked
cargo test --locked --features checkpoints
bash scripts/build-pages.sh                  # Pages build + WASM smoke test
cargo run --release --locked --manifest-path oracle/Cargo.toml   # Rapier oracle: invariants + native perf gate
node scripts/oracle-wasm.mjs                 # Rapier oracle on WASM: timings and sizes (not gated)
```

CI (`.github/workflows/validate.yml`) runs these on plain Ubuntu with the `rust-toolchain.toml` toolchain; the `fast` tier is declared in `.coding-tooling.json`.

## Execution scope

- **One task = one branch = one PR**, targeting `main`. A task is one eligible issue. Deliver its scope, its oracle scenario and the docs it changes on that branch.
- **Stay inside the task.** Note unrelated findings in one line of the PR description.
- **Rapier oracle.** Every parity PR adds its scenario to the shared Rapier harness; CI checks behaviour invariants and fails when Physics Engine is slower than Rapier natively. Do not add other ratchets, evidence packaging or attestation.
- **Experimental API.** Pick the cleanest API for the consumers; no deprecation windows, shims or back-compat layers. Update `docs/world-api-contract.md` and `docs/consumer-capabilities.md` when the API or a capability's status changes.
- **Checkpoints** are an opt-in debugging/logging feature. When a change alters their wire layout or continuation semantics, bump the format or algorithm version once and update `docs/floating-checkpoints.md`; no new checkpoint features.
- **Validate in tiers.** Run focused tests while iterating; CI is the full gate. A red check blocks merge; fix it.
- **Short PR descriptions.** What changed, API changes, the checks that ran, anything not verified.
