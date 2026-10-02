# Floating physical checkpoints

`approximate::World` provides an explicit physical checkpoint product. On the same supported build/target, destroying the original world and restoring the checkpoint preserves exact future physical state and semantic contact/wake/retirement reports for the same commands and timesteps. This does not promise universal compiler/target bit identity, another solver's event stream, or stronger collision quality/CCD than the selected floating solver.

## Interface and compatibility

```rust
use physics_engine::approximate::{Checkpoint, CheckpointContext, CheckpointLimits};
let saved = world.checkpoint(context)?;
let bytes = saved.to_bytes();
drop(saved);
drop(world);
let decoded = Checkpoint::from_bytes(&bytes, context, CheckpointLimits::default())?;
let mut restored = decoded.restore();
restored.step(0.01)?;
```

The caller supplies two 32-byte `CheckpointContext` identities. `build` must identify the exact engine/dependency revisions, compiler, relevant flags/features and ABI policy. `content` identifies static content and the consumer's BodyId/gameplay mapping. These are compatibility tags, not credentials. The caller must derive/check them from its actual supported save contract; a repeated placeholder does not establish compatibility. Architecture and OS are also embedded and checked independently. Format and solver-algorithm versions are separate; incompatible versions fail rather than coerce state. Algorithm version changes when continuation semantics change even if the wire layout remains identical.

The dense-contact repair introduces format 2 and algorithm 2. Format 1/algorithm 1 checkpoints are explicitly rejected. The new position-policy bit changes the wire interpretation, and admitted-contact correction changes continuation semantics. The driven/parked contact-admission repair advances algorithm to 3 while retaining format 2: prior algorithm 2 checkpoints are explicitly rejected because a moving external support now wakes an admitted parked dynamic contact in the same call. The first-substep force-on-contact-wake repair advances algorithm to 4: newly awakened bodies now receive gravity/queued force/torque before their same-substep response, and algorithm 3 checkpoints are rejected. The admitted external tangential-contact wake repair advances algorithm to 5: actual contact-point motion now wakes a parked dynamic body before frictional response, and algorithm 4 checkpoints are rejected. Stationary external-support parking and admitted load-bearing support departure advance algorithm to 6; algorithm 5 checkpoints are rejected. The empty-world digest changes for the version headers and checksum, without changing any empty-world trajectory. This is an intentional compatibility boundary, not automatic save migration.

The reciprocal current-contact momentum repair and its fixed-floor interval prerequisite advance the algorithm to 7 while
retaining wire format 2. Algorithms 1 through 6 are rejected: contact impulses and
their local warm-start anchors now use a shared point for reciprocal dynamic current
contacts, changing continuation semantics. Fixed, external, one-way and swept
response authority remains explicit. The empty-world digest changes only because
the algorithm header and its checksum change; its physical trajectory is unchanged.

### Scalar width (`f32-physics`)

The `f32-physics` build (see [numerics](numerics.md#f32-floating-physics-build)) keeps the same format/algorithm versions and record layout, but writes every `Real` value at its native 4-byte width under the magic `PEFLT32` plus a zero byte. Elapsed and prior-substep seconds stay f64 in both builds. The default f64 build keeps `PEFLOAT`, its wire bytes and its empty-world digest unchanged. Each build checks the magic first and rejects the other width with `CheckpointError::ScalarWidthMismatch { expected_bits, found_bits }` before reading versions, payload or checksum; an unknown magic remains `InvalidData`. There is no cross-width conversion: a save is valid only for the scalar width that wrote it, which the caller's `build` tag should also identify.

CCD remaining-time response (#246/#261) advances the algorithm to 8 while retaining wire format 2. Algorithms 1 through 7 are rejected: a CCD body now responds at its swept time of impact and travels the rest of the substep along the response velocity, which changes continuation semantics. The empty-world digests change only because the algorithm header and its checksum change.

All current shapes, materials and fixed geometry are inline. No external geometry reference or platform pointer is serialized. Consumer-owned gameplay/assets/ID generators and save schemas remain outside physics. Physics has no internal ID generator to persist. When a consumer saves gameplay and physics together, it owns their shared safe boundary and content compatibility. Persistence I/O, transport, authentication and distributed recovery remain outside the engine.

## Continuation inventory

| Data | Preserved |
| --- | --- |
| Configuration | Gravity, substeps, iteration limits, contact slop, sleep thresholds, warm-start flag, convergence tolerances/scope and position-correction scope and explicit optional soft-contact policy |
| Physical bodies | Canonical BodyId order, shape/dimensions, mass/material, position, velocity, quaternion, angular velocity, layers, motion/sensor/rotation/sleep/CCD/retirement policy and linear-support direction |
| Pending input | Force, torque, linear and angular impulse, including finite inputs that will cause a later returned step error |
| Sleep/history | Sleeping flag, each body's exact quiet timer, canonical live contact-pair keys and ordered local anchors, normal, accumulated normal/tangent impulses |
| Sequencing | Elapsed simulation seconds and prior substep duration used for warm-start scaling |

Reports/events and failed-work diagnostics are observations of completed/discarded attempts, not continuation authority. Restored `last_report` starts empty, so a caller cannot accidentally re-emit a prior retirement event. Subsequent physical/semantic reports match; cold-cache bookkeeping, allocations and timing may differ. Renderer/JavaScript state, wall clocks, scratch capacities, prepared response, geometry/position indices, solver constraints and traversal stamps are excluded.

Capture validates before copying physical bodies/contact history. Restore consumes that immutable product into one new mutable world; it moves storage and recomputes bounds/activity rather than calling `add_body` or normalizing quaternions a second time. Rebuildable search/response/graph state starts invalid. `checkpoint()` is never called by ordinary `step()` or transaction rollback. Capture copies all bodies/contact history; endpoint validation binary-searches the canonical body IDs. Encoding writes all saved records and restore rebuilds every body bound. These are explicit caller products, not a sparse-step optimization.

## Wire validation

Version 2 uses `PEFLOAT` plus a zero byte (`PEFLT32` in the f32 build), little-endian format/algorithm u32 versions, build/content tags, length-prefixed architecture/OS, configuration, elapsed/prior-substep f64 values, u64 body count and canonical bodies, then u64 pair count and canonical contact history. Floating values preserve their exact IEEE-754 bits. Shape tags are box=0, sphere=1, capsule=2, wedge=3. Body flags encode rotation-lock, external, sensor, sleep-allowed, CCD, retire-on-impact and sleeping in bits 0 through 6; other bits are rejected. The configuration policy byte uses bit 0 for optional soft contact and bit 1 for admitted-contact position correction; other bits are rejected. Boolean/optional/scope tags are checked. The trailing SHA-256 covers every payload byte. Hashing uses the existing [RustCrypto SHA-2 implementation](https://github.com/RustCrypto/hashes/blob/sha2-v0.10.9/sha2/Cargo.toml), rather than introducing a physics-owned hash algorithm.

The decoder checks total-byte and body/pair/point budgets, minimum record lengths before vector allocation, exact payload completion, checksum, known versions/policies, compatible target/context, finite physical values, the existing body/config ranges, nonnegative quiet time/normal impulse, strictly increasing unique IDs/pair keys, and live distinct pair endpoints. Orientation component magnitudes are bounded by 2 and stored normal components by a conservative 64. These envelopes cover the existing normalization/rotated-normal results and protect derived arithmetic; they never clamp or change solver state. Extreme finite quaternion normalization can lose unit length or yield signed zero in the existing API. The checkpoint preserves those accepted values and their continuation/failure behavior; it does not silently normalize them again. Pending inputs retain the API's finite-value policy and are not clamped. Optional experimental soft-contact data is rejected by an ordinary build. Unknown, damaged, truncated, oversized or semantically invalid data cannot mutate a live world. Panics/allocation aborts remain outside returned-error guarantees; checksum integrity does not authenticate a save.

Default decode budgets are 64 MiB, 100,000 bodies, 200,000 pairs and 800,000 points. Callers can choose stricter or larger explicit budgets. These are admission limits on a checkpoint, not simulation body limits or an RSS guarantee. All four lockfiles add the same SHA-2 dependency graph; no existing dependency version/source changes.

## Acceptance and reproduction

```sh
cargo test --locked --test floating_checkpoints
cargo test --features exact-reference,experimental-soft-contact --locked --test floating_checkpoints
cargo test --features f32-physics --locked --test floating_checkpoints
cargo test --manifest-path demo-wasm/Cargo.toml --locked --example checkpoint-continuation
cargo build --manifest-path demo-wasm/Cargo.toml --locked --release \
  --target wasm32-unknown-unknown --example checkpoint-continuation
node scripts/test-checkpoint-wasm.mjs \
  demo-wasm/target/wasm32-unknown-unknown/release/examples/checkpoint_continuation.wasm
cargo test --release --locked --test floating_checkpoint_benchmark -- --ignored --nocapture
```

Native tests destroy/restore worlds with current contacts, all four supported shapes, nontrivial orientations, material/layer/external/sensor/linear-support policies, pending force/off-center impulse, partial quiet timers, sleepers and support removal. Future timesteps include changed dt and zero-duration calls. Mutations include add/remove/same-ID reuse; failures include retirement before a later numerical error, repeated failed steps/repeated restores and huge finite queued force, subnormal-scale orientation and signed-zero orientation followed by its existing numerical failure. Tests compare subsequent exact checkpoint bytes, rebuilt body bounds and semantic contact/event ordering, rather than accepting serialization round-trip alone. Experimental relaxation has a separate all-feature continuation control.

Corruption tests cover every truncation and single-byte bit flip in a representative checkpoint, unknown format/algorithm, build/content/target mismatch, trailing data, unknown flags/shapes/booleans, duplicate IDs, nonfinite/out-of-range state, missing contact endpoints, invalid points and exhausted budgets. A fixed native x86_64/Linux empty-world wire digest guards version-2 layout. Legacy body construction can admit a nonfinite linear-support direction; checkpoint capture explicitly rejects that state without serializing it or modifying the world.

The separate native/WASM example runs the same 100-tick fixture through exact same-build continuation, partial sleep, pending input, support removal, capsule same-ID reuse, swept impact retirement and repeated failed ticks. Pages validation executes three WASM replays. This driver is a separate example module and adds no test exports or checkpoint behavior to the shipped Pages demo. Native and WASM checkpoints are tested independently, not exchanged as a portability claim.

## Explicit product cost

The ignored matrix uses free, quiet and local worlds with 128/512/2048 dynamic bodies; supported cases include one fixed floor and four warm-start points per body. Three trials each perform 16 capture/encode/decode/restore boundaries and 16 completed ticks. Setup, commands, independent uninterrupted control, assertions and extra checkpoint exports for comparison are outside timers. Capture includes physical validation/copying, encoding includes hashing, decode includes checksum/validation/allocation, and restore includes bound/activity rebuilding. The following advisory medians are milliseconds per requested product boundary; bytes are gauges for one product.

| Workload | Dynamic bodies | Capture ms | Encode ms | Decode ms | Restore ms | Encoded bytes | Physical vector bytes |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| free | 128 | 0.004 | 0.020 | 0.026 | 0.004 | 35564 | 47104 |
| free | 512 | 0.020 | 0.090 | 0.105 | 0.014 | 141548 | 188416 |
| free | 2048 | 0.092 | 0.462 | 0.419 | 0.055 | 565484 | 753664 |
| quiet | 128 | 0.013 | 0.052 | 0.067 | 0.004 | 92160 | 100720 |
| quiet | 512 | 0.056 | 0.201 | 0.273 | 0.014 | 367104 | 401776 |
| quiet | 2048 | 0.270 | 0.843 | 1.116 | 0.057 | 1466880 | 1606000 |
| local | 128 | 0.014 | 0.052 | 0.068 | 0.004 | 92160 | 100720 |
| local | 512 | 0.057 | 0.203 | 0.276 | 0.014 | 367104 | 401776 |
| local | 2048 | 0.249 | 0.846 | 1.116 | 0.057 | 1466880 | 1606000 |

All 27 runs reproduce exact subsequent physical state/history and semantic reports. Per 16 captures, free worlds copy 16N bodies and no contact points; supported worlds copy 16(N+1) bodies and 64N points. Explicit capture costs scale with saved state even when a quiet step is O(1). Restored quiet steps retain that no-op behavior. The 512-body supported product holds 401,776 vector bytes and encodes 367,104 bytes; the 2,048-body product holds 1,606,000 vector bytes and encodes 1,466,880 bytes. These figures exclude tree nodes, allocator overhead, input/output buffers held concurrently, inline state and the caller's original world; they are not whole-process memory peaks.

Fingerprint: Rust 1.98.0, release, x86_64-unknown-linux-gnu, empty RUSTFLAGS, Linux, AMD Ryzen 7 5700X. The completed executable was rerun alone after compilation/other checks. Ordinary solver correctness/work remains covered by the existing 31 ratchet controls; checkpoint timings are separate advisory evidence, not a portable throughput gate. Resolved convention sourceRevision: `e6acb5310afaf15c0cba24f87108f5f4ad1bedc3`.

## Remaining consumer work

This implements the selected floating world's engine checkpoint slice under #198. MMORPG still owns its gameplay/canonical save contract and uses the translational World adopted in PR #47. Its floating-solver migration remains gated by query/character support, CCD/contact quality and real native/browser canonical/recovery acceptance (#194, #3 and the linked consumer issue). This checkpoint product alone does not satisfy that adoption or close #198.
