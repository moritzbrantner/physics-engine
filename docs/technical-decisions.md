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
