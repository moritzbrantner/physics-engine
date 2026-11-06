# Floating physical checkpoints

Opt-in: enable the `checkpoints` Cargo feature. Checkpoints exist for debugging and logging; the
default build and the Rapier performance gate do not include them.

`approximate::World` provides an explicit physical checkpoint product. On the same build and
target, destroying the original world and restoring the checkpoint preserves exact future physical
state and contact/wake/retirement reports for the same commands and timesteps. Checkpoints are
build- and target-specific; moving between targets or providers reconstructs physics from
consumer-owned game state. `Checkpoint::restore()` never accepts Rapier state.

Algorithm 12 corrects tangential heightfield sweep admission for character moves. A nearly
tangential earliest hit no longer hides a later blocking triangle. This changes continuation
at terrain seams; format 9 is unchanged. Algorithm 13 fixes character reconstruction; current algorithm 14 adds box/hull response
(see below). Older algorithm checkpoints are rejected.

Format 9 (#180) adds static convex hull shape tag 7 and embedded canonical hull vertices
after the format-8 mesh slots. A u64 hull-slot count precedes one u64 vertex count per
slot, then three physical scalars per vertex; zero vertices marks a removed slot. Restore
rebuilds prepared geometry and validates one live hull body per populated slot. The algorithm
version remains 11. The format-9 empty-world fixture changes intentionally; mesh tag 6 and
its slot layout remain intact.

Format 8 (#182) adds immutable static triangle meshes: shape tag 6 contains the geometry slot
and conservative half extents. After the heightfield entries, a u64 mesh-slot count is followed
by one presence byte per slot; live slots contain a u64 vertex count and f64 triples, then a
u64 triangle count and u32 index triples. Geometry is in its prepared body-local frame. Removed
slots remain absent, so future geometry IDs continue identically. Restore rebuilds and validates
the mesh-local BVH; caller byte budgets cover mesh data. Body poses/materials and character
registrations retain their existing meaning. The algorithm version stays 11: existing worlds'
continuation rules are unchanged. The format-8 empty-world golden changes intentionally.

Format 7 (#452) checkpoints worlds with halfspaces, heightfield terrain and characters: see
[terrain and character state](#terrain-and-character-state-452). Format 6 adds each body's linear and angular damping after its restitution. Format 5 adds the configuration's body storage mode (`Config::body_storage`); a checkpoint
holds the live bodies in body order whatever the storage held between steps. Format 4 adds the
configuration's length unit. Format 3 stores each body's handle, key (or creation serial), the slot generations and the
next creation serial, so restored ids stay valid and stale ids stay rejected.

Joints (#453) are not part of the format: `checkpoint` of a world with any joint returns
`CheckpointError::UnsupportedJoints` without capturing anything, so a checkpoint never drops a
joint silently. A world without joints builds no joint rows, so its continuation is unaffected.

## Terrain and character state (#452)

A game recovering a zone owns its save. The split between what it reconstructs from its own
state and what the physics checkpoint continues:

| Data | Owner | In the checkpoint |
| --- | --- | --- |
| Heightfield heights, grid, cell size, origin, removed cells | Game content | A SHA-256 fingerprint per live heightfield (its terrain index and the exact bits of the description's geometry). `restore` takes the descriptions again and rejects any other with `TerrainMismatch { index }` |
| Heightfield body (id, key, material, layers) and its contacts | Physics | Inline body record (shape tag 4: terrain index, half extents); restore also checks the description yields the recorded centre and half extents |
| Halfspaces | Physics | Inline body records (shape tag 5) and the halfspace test order (their creation order, which orders plane contacts in the solver) |
| Convex rock hulls | Game content originally, physics continuation after admission | Canonical local hull vertices for each live slot; shape tag 6 stores slot id and conservative half extents. Removed hull slots have empty payloads so later ids remain stable. No dynamic hull solver state exists. |
| Character settings (`CharacterSettings`) | Game configuration, but tiny | Inline, so a restored world needs no `add_character` |
| Character's latest `exclude` | Physics continuation | Inline: a character pushed between its own moves still excludes that body. A game that registered its characters again after restore loses it, and the zone's push into an idler standing in the floor-excluded basin comes out differently (`re_registering_characters_after_restore_diverges`) |
| Character known free of overlap | Physics continuation | One flag, restored at the new layout generation: the next move skips separation/depenetration exactly when the uninterrupted world would. No zone divergence was observed without it; it is kept so the next move's work path is the same |
| Grounded, ground contact, support | Neither: recomputed by every move | Not stored. `World` keeps no grounded state; a game's choices from the last `CharacterMoveResult` (vertical speed, autostep after `ground_contact`) are game state it saves itself |
| Per-batch push totals, batch scratch, query tree, geometry cache, prepared terrain | Rebuilt | Not stored; `restore` rebuilds prepared terrain from the given descriptions |

`Checkpoint::heightfield_bodies()` lists the heightfield bodies in the order `restore` takes
their descriptions (the order the world added them). A removed heightfield's terrain index stays
reserved, so restored and later terrain ids match the uninterrupted world. The material of a
description given to `restore` is ignored: the body's material is inline. Terrain fingerprints are
computed once by `add_heightfield` in builds with the `checkpoints` feature, so capture stays
proportional to bodies and contacts, not heights.

Format 7 changes only the wire layout; continuation semantics of worlds without terrain or
characters are unchanged, so the algorithm stays 11. Format 6 checkpoints are rejected.

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

### Scalar width

The f32 build keeps the same format/algorithm versions and record layout, but writes every `Real` value at its native 4-byte width under the magic `PEFLT32` plus a zero byte. Elapsed and prior-substep seconds stay f64 in both builds. The default f64 build keeps `PEFLOAT`, its wire bytes and its empty-world digest unchanged. Each build checks the magic first and rejects the other width with `CheckpointError::ScalarWidthMismatch { expected_bits, found_bits }` before reading versions, payload or checksum; an unknown magic remains `InvalidData`. There is no cross-width conversion: a save is valid only for the scalar width that wrote it, which the caller's `build` tag should also identify.

CCD remaining-time response (#246/#261) advances the algorithm to 8 while retaining wire format 2. Algorithms 1 through 7 are rejected: a CCD body now responds at its swept time of impact and travels the rest of the substep along the response velocity, which changes continuation semantics. The empty-world digests change only because the algorithm header and its checksum change.

Contacts generated once per step and reused across substeps (#378) advance the algorithm to 9 while retaining the wire format; algorithm 8 checkpoints are rejected because multi-substep worlds now continue differently. The empty-world digest changes only for the algorithm header and its checksum.

Dynamics settings in metres scaled by `length_unit` (#318) advance the algorithm to 10 while retaining the wire format. The checkpoint stores the configuration as given; `gravity`, `contact_slop`, `sleep_speed` and the 1 m/s restitution threshold are scaled on restore, so algorithm 9 checkpoints with a non-unit `length_unit` would continue differently and are rejected. The empty-world digest changes for the algorithm header and the new default slop and sleep speed.

Per-body combine rules (#391) advance the algorithm to 11 while retaining format 6: body flag bits 7–8 hold `friction_combine` and bits 9–10 `restitution_combine` (Average=0, Min=1, Multiply=2, Max=3), and contacts now average coefficients by default where algorithm 10 took the larger friction and the smaller restitution, so algorithm 10 checkpoints are rejected. The default friction is 0.5. The empty-world digest changes for the algorithm header.

All primitive shapes, halfspaces, materials and fixed geometry are inline; heightfield geometry is consumer content, identified by its fingerprint and supplied again to `restore`. No external geometry reference or platform pointer is serialized. Consumer-owned gameplay/assets/ID generators and save schemas remain outside physics. Physics has no internal ID generator to persist. When a consumer saves gameplay and physics together, it owns their shared safe boundary and content compatibility. Persistence I/O, transport, authentication and distributed recovery remain outside the engine.

## Continuation inventory

| Data | Preserved |
| --- | --- |
| Configuration | Gravity, substeps, iteration limits, contact slop, sleep thresholds, warm-start flag, convergence tolerances/scope and position-correction scope and explicit optional soft-contact policy |
| Physical bodies | Canonical BodyId order, shape/dimensions, mass/material, position, velocity, quaternion, angular velocity, layers, motion/sensor/rotation/sleep/CCD/retirement policy and linear-support direction |
| Pending input | Force, torque, linear and angular impulse, including finite inputs that will cause a later returned step error |
| Sleep/history | Sleeping flag, each body's exact quiet timer, canonical live contact-pair keys and ordered local anchors, normal, accumulated normal/tangent impulses |
| Sequencing | Elapsed simulation seconds and prior substep duration used for warm-start scaling |
| Terrain | Heightfield fingerprints and terrain ids, halfspace test order |
| Characters | Registrations: settings, latest excluded body, known-clear flag |

Reports/events and failed-work diagnostics are observations of completed/discarded attempts, not continuation authority. Restored `last_report` starts empty, so a caller cannot accidentally re-emit a prior retirement or contact event; contact transitions pending for the next step (removals, held impacts) are not checkpointed. Subsequent physical/semantic reports match; cold-cache bookkeeping, allocations and timing may differ. Renderer/JavaScript state, wall clocks, scratch capacities, prepared response, geometry/position indices, solver constraints and traversal stamps are excluded.

Capture validates before copying physical bodies/contact history. Restore consumes that immutable product into one new mutable world; it moves storage and recomputes bounds/activity rather than calling `add_body` or normalizing quaternions a second time. Rebuildable search/response/graph state starts invalid. `checkpoint()` is never called by ordinary `step()` or transaction rollback. Capture copies all bodies/contact history; endpoint validation binary-searches the canonical body IDs. Encoding writes all saved records and restore rebuilds every body bound. These are explicit caller products, not a sparse-step optimization.

## Wire validation

Version 2 uses `PEFLOAT` plus a zero byte (`PEFLT32` in the f32 build), little-endian format/algorithm u32 versions, build/content tags, length-prefixed architecture/OS, configuration, elapsed/prior-substep f64 values, u64 body count and canonical bodies, then u64 pair count and canonical contact history, the next creation serial and slot generations, then (format 7) the u32 terrain count, u64 heightfield count with each live heightfield's u32 terrain index and 32-byte fingerprint in increasing index order, u64 halfspace count with their body ids in test order, and u64 character count with each character's body id, offset, max slope, optional snap, slide count, optional plane, optional push, embedded policy (0 push out, 1 stay), optional excluded body id and clear flag, in slot order. Floating values preserve their exact IEEE-754 bits. Shape tags are box=0, sphere=1, capsule=2, wedge=3, heightfield=4 (u32 terrain index, half extents), halfspace=5 (no dimensions), triangle mesh=6 (u32 mesh slot, half extents). Body flags encode rotation-lock, external, sensor, sleep-allowed, CCD, retire-on-impact and sleeping in bits 0 through 6, and the friction and restitution combine rules in bits 7–8 and 9–10; other bits are rejected. The configuration policy byte uses bit 0 for optional soft contact and bit 1 for admitted-contact position correction; other bits are rejected. Boolean/optional/scope tags are checked. The trailing SHA-256 covers every payload byte. Hashing uses the existing [RustCrypto SHA-2 implementation](https://github.com/RustCrypto/hashes/blob/sha2-v0.11.0/sha2/Cargo.toml), rather than introducing a physics-owned hash algorithm.

The decoder checks total-byte and body/pair/point budgets, minimum record lengths before vector allocation, exact payload completion, checksum, known versions/policies, compatible target/context, finite physical values, the existing body/config ranges, nonnegative quiet time/normal impulse, strictly increasing unique IDs/pair keys, and live distinct pair endpoints; each live heightfield listed exactly once below the terrain count (itself at most the creation serial and the body budget), each halfspace body listed once, and each character a live external capsule with valid settings. Orientation component magnitudes are bounded by 2 and stored normal components by a conservative 64. These envelopes cover the existing normalization/rotated-normal results and protect derived arithmetic; they never clamp or change solver state. Extreme finite quaternion normalization can lose unit length or yield signed zero in the existing API. The checkpoint preserves those accepted values and their continuation/failure behavior; it does not silently normalize them again. Pending inputs retain the API's finite-value policy and are not clamped. Optional experimental soft-contact data is rejected by an ordinary build. Unknown, damaged, truncated, oversized or semantically invalid data cannot mutate a live world. Panics/allocation aborts remain outside returned-error guarantees; checksum integrity does not authenticate a save.

Default decode budgets are 64 MiB, 100,000 bodies, 200,000 pairs and 800,000 points. Callers can choose stricter or larger explicit budgets. These are admission limits on a checkpoint, not simulation body limits or an RSS guarantee. The workspace lockfile resolves one SHA-2 0.11 dependency graph.

## Acceptance and reproduction

```sh
cargo test --locked --features checkpoints --test floating_checkpoints --test checkpoint_zone
cargo test --locked --features checkpoints,indexed-storage --test checkpoint_zone
bash scripts/checkpoint-wasm.sh   # WASM; also run by scripts/build-pages.sh
```

The MMORPG-style zone (`tests/support/checkpoint_zone.rs`, #452): a floor halfspace, a heightfield
hill and basin with a removed cell, a static platform, a crate resting against it that the walking
character touches and that takes an off-centre impulse, a crate on the hill removed and dropped
again under the same key, a walker that climbs the hill and crosses the basin with the floor
excluded, an idler standing in the basin that the walker pushes between its own moves, and a far
body whose next two ticks fail before the game removes it. Before format 7, `checkpoint` of this
world returned `InvalidData` (terrain and halfspaces refused), and character registrations were
not captured at all. The uninterrupted run and one that saves the game state plus checkpoint bytes,
drops the world and restores at 13 tick boundaries (including both failed ticks and between the
idler's moves) produce identical per-tick character results, step reports and contact events,
checkpoint bytes and body records with rebuilt bounds, on ordered and indexed storage. Every
truncation and bit 0/7 flip of a mid-climb zone checkpoint is rejected, as are tampered character and
halfspace records (rehashed), wrong/missing/extra terrain descriptions and a tight body budget.
Native and WASM checkpoints are tested independently, not exchanged as a portability claim.

Native tests destroy/restore worlds with current contacts, all four supported shapes, nontrivial orientations, material/layer/external/sensor/linear-support policies, pending force/off-center impulse, partial quiet timers, sleepers and support removal. Future timesteps include changed dt and zero-duration calls. Mutations include add/remove/same-ID reuse; failures include retirement before a later numerical error, repeated failed steps/repeated restores and huge finite queued force, subnormal-scale orientation and signed-zero orientation followed by its existing numerical failure. Tests compare subsequent exact checkpoint bytes, rebuilt body bounds and semantic contact/event ordering, rather than accepting serialization round-trip alone.

Corruption tests cover every truncation and single-byte bit flip in a representative checkpoint, unknown format/algorithm, build/content/target mismatch, trailing data, unknown flags/shapes/booleans, duplicate IDs, nonfinite/out-of-range state, missing contact endpoints, invalid points and exhausted budgets. A fixed native x86_64/Linux empty-world wire digest guards version-2 layout. Legacy body construction can admit a nonfinite linear-support direction; checkpoint capture explicitly rejects that state without serializing it or modifying the world.

The WASM acceptance is `examples/checkpoint_zone.rs`, the same zone and comparison built for `wasm32-unknown-unknown` and run in Node by `scripts/checkpoint-wasm.sh` (traps on any difference). It is a separate example and adds no exports or checkpoint behavior to the shipped Pages demo.

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

### Zone product cost (#452)

`cargo test --release --features checkpoints --test checkpoint_zone zone_checkpoint_cost -- --ignored --nocapture`
captures the zone mid-climb (7 bodies, 3 pairs, 15 points, 2 characters, one 9×9 heightfield):
median capture 1.8 µs, encode 3.5 µs, decode 3.9 µs, restore 22.8 µs (World construction plus
rebuilding and fingerprinting the terrain from its description), against 45 µs for one ordinary
zone tick (character batch plus step); 2,433 encoded bytes, 3,236 physical vector bytes. Advisory
only (Intel Xeon 2.1 GHz, 4 cores, Rust 1.99.1, release, x86_64 Linux). The WASM run is a
correctness gate; its cost is not measured. Restore cost grows with terrain size (one pass over
the heights to prepare and one to fingerprint); capture does not.

## Remaining consumer work

#452 adds terrain, halfspace and character continuation for the engine-side zone fixture; no consumer has adopted it, and MMORPG's own recovery acceptance (its save format, content tags and browser recovery) remains open, as do joints in checkpoints. This implements the selected floating world's engine checkpoint slice under #198. MMORPG still owns its gameplay/canonical save contract and uses the translational World adopted in PR #47. Its floating-solver migration remains gated by query/character support, CCD/contact quality and real native/browser canonical/recovery acceptance (#194, #3 and the linked consumer issue). This checkpoint product alone does not satisfy that adoption or close #198.

## Character reconstruction (algorithm 13, #500)

Static push-out now derives clearance from the authoritative physical pose instead
of trusting an earlier move's clear hint. An overlap-verified exact pose can reuse clearance
until stepping, placement or an exclusion change; this cache is disposable. This changes continuation on a shallow embedded
terrain pose, so the algorithm becomes 13 while the wire format stays 9. Version 12 contains the merged terrain-seam correction (#501); versions 11 and 12 are rejected.
The known-clear field remains in the wire layout as a character-separation hint, not authority
for static overlap. The empty-world digest changes with the algorithm header/checksum.
The game still owns canonical pose/settings reconstruction and its integration acceptance.

## Box/hull contact continuation (algorithm 14)

Non-CCD boxes now generate rigid manifolds against static convex hulls. This changes step
continuation from an atomic unsupported-pair error to physical response, so algorithm 14
rejects earlier checkpoints, including algorithm 13 character reconstruction while retaining format 9. The empty-world wire
digest changes only with the algorithm header/checksum. The focused box/rock checkpoint
test restores mid-contact and continues bit-identically.
