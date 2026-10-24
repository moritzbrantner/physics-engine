# Technical decisions

Current direction, owner-approved 2026-10-18. This page replaces the former TECH-001…TECH-013
register; superseded entries are intentionally not kept. Source code and tests prove what is
implemented; GitHub issues track remaining work.

## D1 — Games use Rapier; Rapier is the oracle and the performance baseline

The owner's games (ARPG, MMORPG, game-server) use [Rapier](https://rapier.rs) as their physics
(ARPG: `arpg/docs/decisions/atlas.md` 2.3.1/2.3.2). Physics Engine's job is to learn which Rapier
functions and capabilities those projects actually use and to provide them at least as fast as
Rapier, natively and in WASM. Rapier is the behavioural reference and the speed to beat; there is
no other reference build.

## D2 — Our own API, shaped by consumer usage

Physics Engine does not clone Rapier's API. It exposes performance-oriented primitives designed
around how the consumers use Rapier, for example:

- batched multi-character kinematic moves in one call (#294);
- stable, never-reused body ids (#296);
- depenetration reported separately from requested motion (#295);
- tolerances relative to a declared length unit (#291);
- ties broken by stable id (#292);
- plane constraints and an explicit grounded state (#293).

A consumer switching from Rapier needs a thin adapter; switching back must stay equally cheap.

## D3 — One world, one f32 build

`approximate::World` with `f32` physical state is the only world. The translational integer/Q32.32
`World` and `RotatingWorld3d` are removed together with their docs, tests, examples, experiments and
performance evidence; there are no shims or deprecation windows. Existing consumers stay on their
current git pin until they move to Rapier. The former f64 reference build and its feature split are
dropped; Rapier is the independent reference (D1).

## D3a — Two body storage modes, identical results

The owner's answer to Q31 asks for two modes. `Config::body_storage` selects one when the world
is created:

- `Ordered` (the default) keeps tombstones and compacts them at the next step, and has an
  append fast path.
- `Indexed` keeps an unordered arena plus an order index.

Results are identical between the modes. Both run the step on contiguous storage in body order:
`Indexed` sorts its arena once before a step instead of iterating through the index. This was a
performance tie-break. It keeps iteration cost equal and moves the difference into the cost of
adding and removing bodies. Complexity contracts are declared per mode, and the Rapier oracle
times both. See [world API contract](world-api-contract.md#body-storage).

## D4 — Rapier oracle in CI

A shared scenario harness runs each scenario on Rapier (a dev-dependency) and on Physics Engine.
CI checks behaviour invariants — no tunneling, grounding/support, normals within tolerance, finite
state and the scenario's own expectations — and **fails when Physics Engine is slower than Rapier**
on that scenario natively (more than 5% slower than Rapier with the consumer's call pattern or
than bare Rapier), and in WASM when more than 10% slower than either; both figures are always
printed. Every parity change adds its
scenario to the harness. The harness stays lean: no attestation, fingerprints or evidence
packaging.

## D4a — Complexity contracts in CI

`.performance/contract.json` (coding-tooling schemaVersion 2 `operations`) declares the Big-O
class of each public operation: queries, body churn, character batches, terrain queries and
heap allocations per warm tick. `tests/work_complexity.rs` measures deterministic work
counters (the test-only `work-counters` feature; off in every release build) on world
families that scale one dimension at doubling points, and CI fails when a declared bound
breaks. Bounds use coding-tooling's classes: exact `constant` and `linear`; tolerant
`logarithmic`, polynomial and product classes such as O(m · log n). Tree descents are
declared `logarithmic` in bodies, not held by budgets. Changing a declared class happens in the same PR, with the reason in the contract.

## D5 — Which work is eligible

Only issues that reach Rapier parity for a capability one of the owner's projects actually uses are
worked on. Consumer priority is MMORPG, then ARPG. The ARPG lessons #289–#297 are eligible; #297
(ARPG's Rapier cost: about 18 µs native / 27 µs WASM per tick, 2.68 MB WASM) is the target. Other
issues stay open but parked. The capability inventory lives in
[consumer capabilities](consumer-capabilities.md).

## D6 — Pages sandbox shows parity

The Pages sandbox (`site/` + `demo-wasm/`) runs on `approximate::World` and shows the parity
scenarios side by side with Rapier.

## D7 — Physical interactions: reuse the world, consumer-gated (Q32–Q38)

Owner-approved 2026-10-22–23 (#384). A boundary for future work, not an implementation plan:
each capability below starts only with a concrete consumer scenario, the exact API gap, Rapier
behaviour and the D4 gates. Game rules stay in the games; no new projectile, fluid or filter
subsystem.

| Q | Engine owns | Game owns |
| --- | --- | --- |
| Q32 melee | continuous swept collision queries against physical objects | attack motion, active windows, hit validity, damage, combos |
| Q33 contact enter/exit | deterministic, stably ordered enter/exit for genuinely physical contact | interpreting transitions; purely logical areas and zones |
| Q34 media | generic friction, damping/drag; buoyancy only when a consumer needs it | swimming, breath, spells, damage |
| Q35 filtering | layers/masks for bodies, solver and queries; per-query BodyId/group exclusions and body-pair exceptions only when needed | which bodies share a layer; no bit per player, no faction semantics |
| Q36 selection | reusable generic filters and ids | the semantic language (owner, team, faction, summons, pets, tags, abilities); no engine entity database, DSL or gameplay callbacks |
| Q37 multi-hit | bounded multiple candidates by time of impact, ties by stable body id; first-hit fast paths kept | shields, friendly fire, piercing, effects |
| Q38 projectiles | travelling projectiles are dynamic bodies with CCD (spheres, rotation-locked arrows, rotating rigid boxes, bouncing props), optional `retire_on_impact`; beams and melee sweeps use ray/shape queries — one world, no separate projectile engine | which policy an attack uses |

Already present and reused: body `ccd`, `retire_on_impact`, `sensor`, `layers`, friction and
damping; ordered ray/shape casts with layers, sensors and one BodyId exclusion
([shape queries](floating-shape-queries.md)); sandbox sphere/arrow/box projectiles. Not implied:
rotational CCD (CCD is translational), rotating melee volumes (casts keep a fixed orientation),
group/pair exclusions, a bounded-result query API. Consumer issues showing future demand:
mmorpg#300 (trap/trigger entry, Q33), mmorpg#290 (thrown and bouncing weapons, Q38), mmorpg#294
(movement and surface spells, Q34), mmorpg#379 (action timing, Q32 sweep sampling). #3 (CCD
roadmap) stays closed. Recording these decisions establishes neither parity nor adoption.

## Standing rules

These earlier rules remain compatible with the direction above:

- **Determinism:** identical results across worker counts and schedules on the same build and
  target; stable-id ordering for pairs, events and ties. No cross-target bit-identity claim.
- **Embedding:** physics is embedded in consumer-owned Rust/WASM; JavaScript crosses only through
  coarse commands and queries, never per-body bridge churn.
- **Checkpoints:** physical checkpoints restore only on their own build and target; moving between
  providers or targets reconstructs physics from consumer-owned game state.
- **Characters:** kinematic character movement is engine-owned; pushing dynamic bodies uses bounded
  engine-resolved impulses under game-defined ceilings.
- **Scope:** ECS, rendering, game loops, scenes and gameplay rules stay in consumers; domain-neutral
  geometry kernels come from `rust-kernels`.
