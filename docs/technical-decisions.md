# Technical decisions

Record consequential engineering choices here only when existing domain-specific documents
do not already explain them. [AGENTS.md](../AGENTS.md) owns general implementation boundaries;
[numerics.md](numerics.md) owns precision rules; [world-api-contract.md](world-api-contract.md)
owns which world APIs and capabilities consumers can rely on. Source code and tests,
not this register, prove what is implemented. GitHub issues track remaining work.

## Independent physics reference

### TECH-001 — Use Rapier as a reference, not an automatic solver replacement

- **Basis / status:** Existing selected technical direction in
  [physics-engine #279](https://github.com/moritzbrantner/physics-engine/issues/279),
  following [reuse-first ADR 0001](https://github.com/moritzbrantner/coding-agent-conventions/blob/main/docs/adr/0001-capability-internalization.md)
  and the [consumer-driven roadmap #191](https://github.com/moritzbrantner/physics-engine/issues/191).
  This record does **not** approve a game default switch.
- **Selected approach:** Keep **physics-engine** authoritative for its own world, contact,
  CCD, response, wake/sleep and recovery semantics, using domain-neutral
  **rust-kernels** geometry. Compare the evolving persistent **approximate::World**
  against pinned, independent Rapier scenarios. Preserve the supported **World**
  and **RotatingWorld3d** surfaces while their consumers rely on them.
  Rapier may remain authoritative in a consumer with an explicit local decision
  (for example [worldgen](https://github.com/moritzbrantner/worldgen/blob/main/AGENTS.md));
  that does not change this repository's simulation authority.
- **Why:** A maintained external engine supplies independent physical reference
  behavior and a complete-call performance baseline. It does not define this engine's
  numerical tolerances, checkpoint format, event ordering or game-specific rules.
  Share fixture inputs and observation categories; do not attempt byte-identical
  trajectories or hide meaningful divergent physical outcomes.
- **Alternatives:** Promoting Rapier directly in all consumers would bypass their
  migration/recovery acceptance. Internal-only comparison would lack a mature
  independent solver. A universal provider interface or shadow physics in game
  production paths is unjustified until a real consumer needs it. Prefer scoped,
  diagnostic adapters to the existing public world APIs.
- **Evidence today:** [Collision Lab](https://github.com/moritzbrantner/collision-lab/blob/main/Cargo.toml)
  pins Rapier 0.36.0 and already has
  [Rapier benchmark fixtures](https://github.com/moritzbrantner/collision-lab/blob/main/benches/rapier_scenarios.rs).
  Those fixtures use **sensor collider intersections/pair-set parity**, not
  Rapier's persistent rigid-body contact/impulse solver; they do not prove
  dynamic stack, friction, CCD, sleep or checkpoint parity.
  Our own [f32 precision tests](../tests/f32_precision_envelope.rs) exercise the
  documented envelope, not a Rapier comparison. No comparative full-solver
  result or consumer cutover is claimed.
- **Verification before promotion:** Extend existing reference fixtures to
  complete, equal-input persistent stepping; check independent invariants for
  static/dynamic contact, capsule grounding, supported friction, fast CCD,
  finite state, sleep/wake, failure/recovery and representative crowded workloads.
  Report exact library/repository revisions, build scalar (the f32-physics game
  target and f64 reference where useful), timestep, tolerated differences and
  unsupported capabilities. Compare whole-step latency, worst stalls, work,
  memory and lifecycle costs without reusing a sensor-only time as solver time.
  Keep existing native/WASM replay and physical-quality checks. Run real consumer
  acceptance under [MMORPG #165](https://github.com/moritzbrantner/mmorpg/issues/165)
  and [ARPG #162](https://github.com/moritzbrantner/arpg/issues/162); actual
  migration remains with their owning issues and separate owner approval.
- **Revisit when:** Comparable persistent-solver evidence changes the trade-off,
  a named consumer proves a different provider meets its exact gameplay and
  recovery contract, or the reference dependency's supported behavior changes.
  Do not silently change game defaults, checkpoints or physics ownership.

## External-first adoption for new consumers

### TECH-002 — Prefer an established external physics provider when a new consumer needs unsupported first-party capabilities

- **Approval / basis:** Owner-selected on 2026-10-15 in physics-engine /tech-steer Question 1, option B (external-first). This extends [TECH-001](#tech-001--use-rapier-as-a-reference-not-an-automatic-solver-replacement) and follows [reuse-first ADR 0001](https://github.com/moritzbrantner/coding-agent-conventions/blob/main/docs/adr/0001-capability-internalization.md).
- **Context:** A new game may require dependable character collision, joints, or another feature before the first-party engine provides it within that consumer's correctness, recovery and runtime requirements.
- **Selected approach:** Use a suitable maintained external implementation such as Rapier for the missing capability in a **new consumer**, rather than delay its user-visible functionality for first-party parity. Make the chosen provider and its physical authority explicit in the consuming repository. Keep developing and independently validating the first-party physics engine as a replacement candidate; consumer adoption happens only after its actual needed features and recovery/performance contract have been proven and the owning consumer approves the switch.
- **Authority boundaries:** This policy does not migrate or change existing MMORPG, ARPG, Battle Royale or other current production backends. Do not silently run two authoritative solvers for the same interacting physics state, copy consumer gameplay logic into the engine, or invent a universal provider interface before a real need. Provider selection and save/recovery compatibility are consumer-owned contracts; physics-engine owns only its own world, collision and checkpoint semantics.
- **Alternatives considered:** First-party-only would block game delivery on an incomplete foundation. Dual production providers in every new game would impose integration and maintenance overhead without evidence of a need. External-first preserves game delivery while leaving future first-party adoption an evidence-based decision.
- **Evidence and handoff:** The current [world API contract](world-api-contract.md) identifies unsupported capabilities. Existing [Rapier reference work #279](https://github.com/moritzbrantner/physics-engine/issues/279) can compare behavior but does not establish universal interchangeability. [Worldgen](https://github.com/moritzbrantner/worldgen/blob/main/AGENTS.md) already keeps Rapier authoritative and the first-party solver non-authoritative. Existing-provider comparisons and migration gates remain owned by [MMORPG #165](https://github.com/moritzbrantner/mmorpg/issues/165), [MMORPG #46](https://github.com/moritzbrantner/mmorpg/issues/46), [ARPG #162](https://github.com/moritzbrantner/arpg/issues/162) and [ARPG #48](https://github.com/moritzbrantner/arpg/issues/48). Before an external selection, verify supported APIs, license, pinned version, platform behavior, performance and persistence needs in that consumer.
- **Revisit when:** A named consumer's first-party path proves its required behavior and recovery, or changing external-provider costs/requirements justify a new choice. Do not call the candidate interchangeable or migrate stored physics state without evidence.

## Cross-provider persistent-world migration

### TECH-003 — Versioned physics-provider conversion at a controlled restart

- **Approval:** Owner-selected 2026-10-15, physics-engine /tech-steer Question 2, option B: versioned migration at a controlled save/restart boundary. Extends [TECH-002](#tech-002--prefer-an-established-external-physics-provider-when-a-new-consumer-needs-unsupported-first-party-capabilities).
- **Context:** A game world saved under a mature external physics solver such as Rapier may later move to the first-party engine. A physical checkpoint preserves continuation only within its own compatible solver/build/target contract; it is not interchangeable with Rapier's internal contacts, caches, solver history, or event stream.
- **Selected approach:** At a deterministic completed-tick save/restart boundary, stop the old authoritative solver and convert **consumer-owned, versioned game/domain state** into a new first-party world. Validate persistent identity, gameplay state, authoritative positions/orientations, velocities, collider/material/unit mapping, sequencing and required inputs; then initialize fresh first-party physical contacts, warm-start, sleep/wake and derived acceleration under the replacement's documented behavior. Never deserialize Rapier checkpoint bytes into `physics-engine::approximate::Checkpoint`. Gameplay must explicitly accept potential differences in post-migration trajectories rather than claiming bit-identical replay across solvers.
- **State and authority:** The consuming application owns conversion, persistence/journal schemas, provider selection, publication and version compatibility. Physics-engine owns first-party physical validation, steps and its checkpoint API. Keep the original save and provider identity recoverable through the uncommitted conversion; publish a new provider/schema checkpoint and migration marker together only after validation. A failed conversion must not partially publish gameplay state, consume commands twice or duplicate durable rewards. Do not hot-switch solvers mid-step, run two interacting production authorities, or introduce a general provider framework without a real need.
- **Alternatives:** Permanently retain Rapier for already-created worlds (Question 2 option A) avoids conversion but prolongs maintenance and compatibility support. Translating raw provider-internal checkpoint bytes would falsely promise incompatible solver-history semantics.
- **Verification:** Each proposed consumer migration must prove a real restart with versioned fixtures, active/resting contacts, moving supports, lifecycle/IDs, valid and corrupted saves, accepted physical divergence, full gameplay/reconnect/protocol semantics, rollback on a failed conversion, idempotent journals and durable economic grants. Compare complete workloads and memory, not only a new world instantiation. Same-solver continuation remains separately exact within its documented build/target. [Physics #198](https://github.com/moritzbrantner/physics-engine/issues/198) owns first-party checkpoints; [MMORPG #46](https://github.com/moritzbrantner/mmorpg/issues/46) and [MMORPG #40](https://github.com/moritzbrantner/mmorpg/issues/40) own multiplayer migration, persistence, durability and recovery. This record does not itself approve or execute a production migration.
- **Revisit when:** A real consumer requires a materially different migration window, or tested cross-solver interchange becomes feasible. Post-cutover rollback/roll-forward remains an explicit consumer deployment choice, not an assumed engine guarantee.

## Checkpoint portability between native and browser

### TECH-004 — Keep physical checkpoints build- and target-specific

- **Approval:** Owner-selected 2026-10-15, physics-engine /tech-steer Question 3, option A. Extends [TECH-003](#tech-003--versioned-physics-provider-conversion-at-a-controlled-restart), without changing the existing physical checkpoint format or implementation.
- **Context:** The first-party [physical checkpoint contract](floating-checkpoints.md) guarantees exact continuation only for a compatible solver build and target. Native Rust and browser WASM can differ in floating-point execution, feature flags, ABI and runtime behavior; a shared byte representation alone would not prove replay portability.
- **Selected approach:** Preserve **build/target-specific** physical checkpoints. A checkpoint is restored only on its declared compatible solver/build/target, scalar width, schema, algorithm and content context, using the existing fail-closed validation. Native and WASM checkpoint continuation are independently tested; no native-to-WASM or WASM-to-native interchange is promised. When a consumer moves its authoritative simulation to another platform, it must use an explicit, versioned **consumer-owned game/domain state** conversion at a completed-tick restart boundary to reconstruct a new world, as under TECH-003, followed by a fresh compatible physical checkpoint.
- **Consequences:** A reconstructed world may have different contact/warm-start/sleep state and future trajectories. The owning game must validate its persistence, input ordering, recovery and gameplay invariants with documented tolerances. It must not silently claim cross-target bit-identical replay, accept unsupported checkpoint bytes, or reinterpret a foreign checkpoint as an in-place restore. No new portable physics wire protocol, automatic cross-target conversion layer or universal runner identity requirement is authorized.
- **Alternatives considered:** Portable native/WASM physical checkpoints (Question 3, option B) would demand a stable cross-target physical-state ABI and substantial numerical/replay and compatibility verification; that cost is not justified by a demonstrated consumer requirement. Versioned game-state restart conversion already supplies the supported platform-migration path.
- **Evidence and handoff:** Retain the existing [native and WASM same-target checkpoint checks](floating-checkpoints.md#acceptance-and-reproduction) and incompatible context/scalar-width rejection. Before a specific consumer changes its execution target, it must prove save -> stop -> convert -> validate -> restart (plus failed-conversion rollback) on real representative state. [Physics-engine #198](https://github.com/moritzbrantner/physics-engine/issues/198) owns physical continuation; consumers such as [MMORPG #46](https://github.com/moritzbrantner/mmorpg/issues/46) and [MMORPG #40](https://github.com/moritzbrantner/mmorpg/issues/40) own game persistence and migration. No portable cross-target test result or consumer migration is claimed by this decision.
- **Revisit when:** A named consumer demonstrates a requirement that cannot be met with validated game-state reconstruction and supplies evidence justifying a cross-target checkpoint contract.
