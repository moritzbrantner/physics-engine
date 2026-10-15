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
