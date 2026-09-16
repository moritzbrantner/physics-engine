# Authoritative mutable world architecture

## Decision

`physics-engine` has one authoritative mutable world state during simulation. Runtime systems do not pass owned copies of that world through the physics pipeline.

All simulation changes are represented as explicit deltas and committed to the authoritative world. A normal physics step must not create a world snapshot as an implementation detail.

Snapshots are explicit products. They are created only when a caller actively requests one for a purpose such as saving, replay checkpoints, debugging capture, deterministic fixtures, or export.

## Runtime invariants

1. **One mutable authority**
   - The world owns canonical body state.
   - Search, broad phase, narrow phase, contact frontiers, solver work and reports refer to bodies by stable identity and borrow authoritative state.
   - Those structures must not own `Vec<RigidBox3d>` or another complete copy of the world.

2. **Delta-only mutation**
   - Physics algorithms compute explicit body deltas.
   - A delta contains only fields that actually change: position, linear velocity, angular velocity, sleep state, membership, or another narrowly scoped mutation.
   - Deltas are committed through the world mutation boundary. Algorithms do not replace complete bodies merely because a field changed.
   - Simultaneous contact response stages a delta batch and commits that batch together.

3. **Snapshots are explicit**
   - `snapshot()`/serialization/checkpoint operations are caller-visible and intentional.
   - A snapshot is not a rollback mechanism for ordinary stepping.
   - Speculative work either remains uncommitted or records a bounded mutation journal for touched state.

4. **Derived state follows invalidation**
   - Broad-phase bounds, contact evidence, event predictions and other derived state persist across work when their inputs are unchanged.
   - Applying a delta yields the precise changed body IDs needed to invalidate or incrementally update derived state.
   - Unchanged bodies and pair evidence are not recomputed simply because another event occurred.

5. **Necessary-work scaling**
   - Cost inside event, contact and stabilization loops should scale with touched bodies, affected pairs and admitted contacts rather than total world size.
   - No whole-world clone, materialization or map rebuild is permitted inside those loops.

## Why

Browser evidence from revision `4ef415a4aceb3ed803cd42423f1d5678361099e6` showed rendering at sub-millisecond cost while physics reached long tail steps above one frame budget. The retained broad phase was generally reused, but the event/stabilization pipeline still multiplied logical work and copied world state through frontier and response ownership boundaries.

The problem is therefore architectural: ownership of complete state was being used to express sequencing, simultaneous response and rollback. That makes otherwise small contact islands pay world-sized costs repeatedly.

## Migration plan

### Slice 1 — establish the mutation boundary

- Introduce explicit body/world delta types and a central delta commit path.
- Route externally requested world mutations through that path.
- Keep changed-body reporting sourced from committed deltas.
- Add tests for deterministic delta ordering and exact application semantics.

### Slice 2 — remove world ownership from contact frontiers

- `RotatingContactFrontier3d` becomes contact/time evidence only.
- Frontier construction may inspect sampled state, but does not retain or transfer ownership of complete world state.
- Response reads authoritative state plus frontier evidence and produces a delta batch.

### Slice 3 — eliminate solver world snapshots

- Stage only bodies participating in the active contact island when simultaneous read-before-commit semantics require it.
- Do not clone all bodies per solver pass.
- Pair response consumes references/prepared state and emits deltas rather than cloned replacement bodies.

### Slice 4 — persistent event/contact work

- Keep event predictions and current-contact evidence across events.
- Applying a delta invalidates only entries touching changed bodies.
- Preserve deterministic ordering and fail-closed collision semantics.

### Slice 5 — explicit snapshots

- Expose intentional world snapshot/checkpoint APIs for save/replay/debug use cases.
- Keep snapshot construction out of ordinary `step()`.

## Architectural performance evidence

Track work counters that expose architecture rather than only wall-clock timing:

- full-world materializations
- bodies staged for simultaneous response
- bodies changed by committed deltas
- full sweep-bound recomputations
- incremental derived-state updates
- contact edges revalidated
- event predictions invalidated

The target runtime invariant is stronger than a timing budget: ordinary stepping performs zero world snapshots and no whole-world copy inside an event, response, or stabilization loop.
