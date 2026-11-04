# World API contract

Physics Engine has one world: `physics_engine::approximate::World`, with `f32` physical state
([technical decisions](technical-decisions.md) D3). The translational integer `World` and
`RotatingWorld3d` were removed without shims; consumers that still use them stay on their old git
pin until they move to Rapier.

The API is designed around how the consuming games use Rapier (D2). It is not a Rapier clone: a
consumer uses a thin adapter, and every capability listed as at parity is checked against Rapier by
the oracle harness (D4). The per-capability parity status is tracked in
[consumer capabilities](consumer-capabilities.md).

## Units and meanings

- `Config::length_unit` declares scene units per metre (1 by default, 100 for a centimetre
  game). Built-in length defaults are defined in metres and scaled by it —
  `World::character_settings()` returns the character defaults (2.5 mm offset, 5 cm snap) in
  scene units; values the game passes explicitly stay in scene units. Query and controller
  tolerances are relative (to magnitudes or to the character offset), so a scene scaled by the
  unit moves identically. The rigid-body dynamics settings are in metres and scaled by it once
  in `World::new`, with Rapier's defaults: `gravity` (0, −9.81, 0) m/s², `contact_slop`
  0.005 m, `sleep_speed` 0.05 m/s for `sleep_seconds` 0.5 s; the restitution threshold is
  1 m/s. `World::config()` returns the configuration as given, in metres.

- Positions and collider dimensions are scene distance units; box dimensions are **half extents**.
- `step(dt)` takes seconds (finite, `0..=0.1`). Elapsed time is kept in `f64`.
- `gravity` is acceleration in distance units per second squared.
- `Body::mass` scales response and force/impulse integration.
- `Body::linear_damping` and `Body::angular_damping` (finite, nonnegative, default 0) are per
  second: after forces, each substep scales the velocity by `1 / (1 + h · damping)`, as Rapier
  does.
- `Body::friction` (default 0.5) and `Body::restitution` (default 0) combine per contact by
  `Body::friction_combine` and `Body::restitution_combine` (`CombineRule::{Average, Min,
  Multiply, Max}`, default `Average`; when the rules differ the later wins), as Rapier's
  defaults and `CoefficientCombineRule` do. `Body::set_material(Material)` sets all four. See
  [contact materials](contact-materials.md).
- `add_force(id, F)` accumulates force for the next step (`F / mass` per substep) and is cleared
  after a successful step.
- `apply_impulse(id, J, point)` changes linear velocity by `J / mass` before the first substep;
  the off-centre component adds angular impulse through the lever arm.

## Body ids and keys

- `add_body(body)` returns the body's `BodyId`: a 32-bit slot and a 32-bit generation packed in a
  `u64`. Slots are pooled (the lowest free slot is reused) and every reuse bumps the generation,
  so the id of a removed body is rejected (`MissingBody`) and never aliases a newer body. Lookup
  is a direct slot index. `BodyId::{slot, generation, to_bits, from_bits}` serve adapters.
- `Body::keyed(key, ..)` (or `body.key = Some(key)`) attaches the game's stable entity id.
  Keys are unique among live bodies (`DuplicateKey`); `World::body_id(key)` finds the live body.
- Ids are handles, not an order (`BodyId` has no `Ord`). Engine ids are never meant to be saved:
  games rebuild physics from game state on load.

## Ordering and determinism

- Every traversal, pair orientation and tie (equal time of impact or distance) follows body
  order: keyed bodies first, by key, then keyless bodies by creation serial within the run.
  Results therefore depend on keys and the current contents, not on slot reuse or the insertion
  order of keyed bodies; a world rebuilt from game state answers queries and character moves
  exactly as the uninterrupted one. Give a body a key when its ties must survive save/load;
  keyless bodies (projectiles and other transient objects) are deterministic within a run only.
  Persistent contact warm-start state is not part of a rebuild; use checkpoints to continue
  dynamics bit-exactly.
- Results are identical across worker counts on the same build and target. There is no
  cross-target bit-identity claim.
- `World::bodies()` yields the live bodies in body order; `body_count()` counts them.

## Body storage

`Config::body_storage` picks how a world stores its bodies. It is fixed when the world is
created. Both modes give identical results for the same calls: bodies, ids, step reports, query
results and character moves. CI runs every test suite in both modes, and the Rapier oracle
checks that both produce the same traces. Only the cost of adding and removing bodies differs.

| | `Ordered` (default) | `Indexed` |
| --- | --- | --- |
| Layout | one array in body order | arena in insertion order, plus an order index |
| Add after every body | amortized O(1) | O(log n) |
| Add in the middle of the order | O(n) (later records move) | O(log n) |
| Remove any body | amortized O(1) | O(log n) |
| Next step after adds and removes | O(n) compaction if bodies were removed | O(n + k log k) sort of the k out-of-order adds |
| Steps, queries, characters | contiguous, in body order | the same (the step sorts the arena first) |

A removal leaves a tombstone. The next step compacts the tombstones away, or a removal does it
once tombstones are half the records. Removing a body that touches others also costs
O(contact pairs), because its contacts wake their islands. `set_velocity`, `apply_impulse` and
`add_force` sort `Indexed` storage first when bodies were added out of order, because waking an
island searches storage by body order.

When to pick which:

- **`Ordered`** suits games whose keys mostly grow, such as entity ids handed out in sequence,
  and games that add and remove only a few bodies per tick. Appends and removals are O(1), and
  no index is kept.
- **`Indexed`** suits games that insert many bodies in the middle of the key order between
  steps, such as streaming level chunks with pre-assigned keys or spawning many keyed bodies
  while keyless ones exist (keyless bodies sort after every keyed one). Each add costs O(log n)
  instead of moving the later records.

## Queries

- `overlap_shape` and `cast_shape` query current authoritative poses through the shared
  `rust-kernels` primitives and return stable ids, ordered contact witnesses, outward target
  normals and support velocity; see [floating shape queries](floating-shape-queries.md).
  Translation-only casts hold orientations and target positions fixed.
- `cast_ray` queries a finite segment against current poses with local feature identity; see
  [floating ray queries](floating-ray-queries.md). `cast_ray_first` returns only the earliest
  hit (ties by lower BodyId), the same hit as `cast_ray(..)[0]`, without collecting the rest.
- Queries traverse AABB trees (`bvh-kernels`) kept current by `add_body`, `remove_body` and
  `step`: an incremental tree of moving bodies, and for fixed bodies an exact incremental tree
  plus a four-wide `WideBvh` that the first ray or shape query builds (#415). The trees change
  which bodies are tested, never a result.
- `has_support` is a solver-facing support predicate.

## Terrain

`add_heightfield(HeightfieldDesc { origin, cell_size, columns, rows, heights, removed_cells, material }, key)`
adds static heightfield terrain (heights row-major, Z rows then X columns, added to `origin.1`;
`removed_cells`, row-major per cell, cuts holes that rays, queries and characters pass through,
like parry's `CELL_REMOVED`) and returns its body, whose shape is `Shape::Heightfield`. Each cell is split along the same
diagonal as parry/Rapier heightfields. Rays, sphere and capsule overlaps and casts, and
character moves work against it. Box and wedge overlaps (door probes) are tested against the
triangles by the separating-axis test: whether they overlap is exact, and the reported depth and
normal come from the deepest triangle alone. The terrain is a surface, so a box wholly beneath
it does not touch it. Box and wedge casts against terrain return an error. Dynamic boxes take
rigid contacts with terrain automatically, one manifold per triangle under the box as in
Rapier: the triangle's upward normal is the contact normal, the box face most opposed to it is
clipped to the triangle's prism (so ridges and peaks under a face give points), and up to four
points within the contact slop plus the step's approach are kept, ranked by their separation
after the body's current motion. Other dynamic shapes take no terrain contacts. Worlds with
terrain are not checkpointed. `add_body` rejects
heightfield shapes.

## Halfspaces

`Shape::HalfSpace` is the solid side of the plane through the body position; the outward
normal is the body's local +Y, so the identity orientation gives an infinite floor. It is
static only (mass 0, not external). Every ray and shape query tests it with a closed form,
wherever the query is; a box or wedge meets it at its deepest point. Characters stand,
walk and snap on it. Dynamic boxes, wedges, spheres and capsules rest and slide on it: each
substep tests every halfspace against every awake dynamic body (O(halfspaces · bodies)), keeps
up to four support points within the contact slop plus the substep's approach (the deepest
after the body's current motion, so a tumbling corner about to swing in is kept),
and lets the solver close speculative points exactly to the plane, so fast bodies do not pass
through it. Worlds with a halfspace are not checkpointed.

## Collision layers

`Body::layers` (`CollisionLayers3d`: memberships and mask) is the one generic filter for solver
contacts, CCD, contact events, position correction and queries: two bodies, or a query's
`QueryFilter::layers` and a body, interact when each one's memberships intersect the other's mask.
That is Rapier's `InteractionGroups::test` with `InteractionTestMode::And`, so a consumer maps its
groups bit for bit. Which bodies share a layer is the game's choice (Q35/Q36); the engine has no
owner, team or faction rule, no pair-exclusion list and no filter callback. Differences from
Rapier, all deliberate: the default is every membership and mask (`CollisionLayers3d::ALL`; Rapier:
membership `GROUP_1`), queries always test their layers (`ALL` by default; Rapier skips the test
without `groups`) and exclude sensors unless `include_sensors` (Rapier includes them unless
`exclude_sensors`), `QueryFilter::exclude` is Rapier's `exclude_rigid_body`, and there are no
separate solver groups. Broad phases are spatial only; layers are read from the body at every
narrow-phase, CCD and query test, so no cache holds a filter decision.

`set_layers(id, layers)` changes them at runtime, as Rapier's `set_collision_groups`: the next
step and query use the new layers. Cached contacts the new layers exclude stop in the next step
(`ContactEvent::Stopped`; the pair's sleeping side wakes, so a crate on an opened trapdoor falls),
sleeping bodies the body now touches under the new layers wake, and characters near it check for
overlap again. Cost: one contact-cache scan (O(cached pairs)) and one query-tree descent. A thrown
weapon that must not hit its thrower leaves on a mask without the characters' layer and gains it
once clear of the thrower (`collision-filters` oracle scenario), without per-player bits; queries
exclude one body with `QueryFilter::exclude`. Character moves filter with `QueryFilter::default()`
and `CharacterMove::exclude`, as the consumers' `move_shape` filters do today.

## Characters

`add_character(id, CharacterSettings)` makes an external (kinematic) upright capsule body a
character. `move_characters(&[CharacterMove], &mut Vec<CharacterMoveResult>)` is the only move
primitive: collide-and-slide that keeps `offset` to every surface it touches, then a grounded
test against walkable static support: like Rapier's ground prediction, support within `offset`
plus 5 cm (in `length_unit`, measured along the ground's normal as Rapier's contact distance)
makes the character grounded without moving it, and a character
that started the move in ground contact (below) and does not move up snaps down to `offset` from
any gap beyond it within `offset + snap_to_ground`, as Rapier's snap. `CharacterMoveResult::ground_contact`
is Rapier's `grounded`: some static contact within that prediction (true distance, in any
direction) faces up at all, a step's rounded edge beside the character included; it is implied
by `grounded` and tested (one overlap query) only off walkable support. This depends on geometry alone, so a world rebuilt from game state
behaves the same. On a walkable slope a move never slips downhill unless its horizontal intent
leads downhill, as Rapier's non-slip slopes: a slide that would point down the slope keeps only
its part along the slope's contour. Zero desired motion never moves a character;
penetration is resolved first and reported as `depenetration`, apart from `translation`.
`CharacterMove::exclude: Option<BodyId>` names one body the move ignores entirely, as
`QueryFilter::exclude` does for a query (one id comparison per candidate, no group storage):
the slide, depenetration, stepping, snapping and the grounded test all pass through it, as
Rapier's `move_shape` and support cast do through a filter excluding one static collider
(MMORPG opens its floor halfspace over a lake basin). Pushes the character receives exclude it
too, until its next move; a changed exclusion makes the character check for overlap again.
`CharacterSettings::embedded` chooses how a move that starts inside geometry is handled:
`Embedded::PushOut` (the default) pushes the character out as above; `Embedded::Stay` leaves it
in place, as Rapier's `move_shape` does for a moving character: only the requested motion is
resolved, a surface the character is inside blocks only motion into it, and each such contact
steps it 0.1 mm (in `length_unit`) off that surface, as Rapier's normal nudge does. Rapier
0.36.1 depenetrates a character asked for zero motion; `Stay` never moves it then (MMORPG skips
zero-motion moves). Other characters are always separated by the batch (below).
`CharacterSettings::plane: Some(height)` makes a planar character: it moves only horizontally
at its height, needs no floor collider and is always grounded with `support` `None`.
Depenetration leaves static and kinematic bodies, not other characters.
Moves run once each in body order (keyed by key, then keyless by creation), independent of
the order of the slice; each sees the earlier movers at their new positions, and `out[i]` is
the result of `moves[i]`. Characters of a batch that overlap each other are first separated
horizontally, half each (a character overlapping one outside the batch moves the whole way);
that separation is part of `depenetration`. A character blocked head-on by another character
sidesteps to its right instead of locking. `CharacterSettings::max_push: Some(ceiling)` enables
bounded pushing: a mover pushes a blocking character horizontally by at most the smaller of their
two ceilings per move (scene units; `None` on either side: no push) and follows it; the pushed
character keeps its gap to everything else, so walls and other characters stop it, and the part
of the motion it does not yield is handled as above. What a character received from pushes during
the batch is reported as `pushed`, apart from `translation` and `depenetration`. On walkable
surfaces downward intent stops instead of sliding down the slope; horizontal intent follows the
surface. Surfaces steeper than `max_slope` are walls: unless the desired motion rises, the slide
keeps no motion up along them, so a character walking into a too-steep ramp stops at its foot
(Rapier's `max_slope_climb_angle`). Motion stops `offset` short of whichever touched surface it
reaches first along its normal, not only the first one touched. `CharacterMove::autostep:
Option<Autostep>` is chosen per move, as a consumer chooses Rapier's controller for each call
(ARPG: a walk, when the previous result had `ground_contact` and the player does not take off;
jumps and falls never step). A move with it that runs into a non-walkable static surface steps up as Rapier's
autostep does: room above for `max_height`, room on top for `min_width`, no too-steep slope there;
it is lifted by the step's height and nudged forward by up to `min_width`. A capsule meeting a
step's rounded edge may climb it over several moves, as on Rapier. Snapping to the ground applies
only to moves whose translation did not rise (as Rapier), so a step or a slide up an edge keeps its
height.
`set_kinematic_position` places an external body without resolving contact; sleeping bodies
within twice the contact slop of its old pose, or touching its new one, wake.
`set_static_position` moves a static body (mass 0, not external; not a halfspace or heightfield)
in place, as Rapier moves a fixed collider (moving platforms): the layout stays valid, sleeping
bodies it was in contact with or now touches wake, and characters it now reaches check for overlap
again. Characters are not carried; the consumer adds a platform's displacement to its riders' moves
(MMORPG's adapter, the `platforms` oracle scenario).

## Persistent projectile CCD

`Body::ccd = true` advances a non-retiring movable body through each admitted swept impact
*within the requested substep*: it resolves an impulse at the earliest time of impact and
re-sweeps the remaining time with the response velocity. Repeated impacts use the existing
`Config::velocity_iterations` budget. On exhaustion, the body remains at its last impact for
the rest of that substep rather than tunnelling (`Report::geometry.ccd_budget_fallbacks`).
`retire_on_impact` bodies instead stop at their impact and leave the world; the game owns their
damage and other attack rules. `Report::swept_contacts` counts admitted remaining-time impulses.
This is **translational** CCD: orientations are held fixed during each sweep, with rotation
integrated between substeps. Sampled rotation is not exact rotational swept collision.

`ccd-ricochet`, `ccd-glancing`, `ccd-moving-wall` and `ccd-synchronized` (C20, MMORPG #290)
exercise two elastic impacts in one 60 Hz tick, a moving kinematic obstacle, a glancing hit and
eight independent simultaneous impacts. Multiple CCD bodies share a global time-of-impact clock;
unchanged future contacts keep their originally computed TOIs and manifolds, avoiding both repeated
whole-world scans and the loss of equal-time hits after f32 clock rounding. Only changed bodies,
affected cached partners and newly eligible pairs must be re-swept. This does **not** introduce a
general impact-event API: contact transitions remain pair-sorted, not chronological hit metadata.
Rapier 0.36.1's single 60 Hz CCD step stops at its first
impact; the behavioral reference advances the same interval in 64 smaller Rapier steps, while
the ordinary single-step Rapier cost remains a separate blocking performance baseline. Analytic
and independent tests cover full-time continuation, tie order, no tunnelling, budget exhaustion
and checkpoint replay. This proves the engine-side fixture, not adoption in the MMORPG adapter.

The exact narrow-phase already specializes sphere–sphere quadratic TOI, rounded sphere–box
face/edge/corner TOI and box–box swept separating axes. Capsule/wedge cases use the existing
conservative geometry-kernel fallback. A circumsphere proxy for a visual polyhedron is instead
an intentional geometry approximation ([#465](https://github.com/moritzbrantner/physics-engine/issues/465)); it is not equivalent to optimizing an exact shape solver; the `ccd-platonic-proxy` scenario and its [experiment record](consumer-capabilities.md#experiment-circumsphere-proxies-for-dodecahedron-and-icosahedron-visuals-465) use the existing `Shape::Sphere` CCD unchanged.

The independent-sphere 1/8/16/24/32-workload is also a deterministic complexity contract:
all sources hit one wall once or bounce between two walls **twice** within one 60 Hz tick.
With n mutually distant CCD spheres, k=1 or 2 impacts per sphere and b unrelated static boxes,
`pair_tests` and exact shape sweeps are O(k n), independent of b ([#469](https://github.com/moritzbrantner/physics-engine/issues/469);
O(k n²) before). Fixed bodies are pruned through the existing query tree. Moving, kinematic and
sleeping targets go in a transient `bvh-kernels` tree holding each one's conservative padded swept
bound from the last time its trajectory was set to the end of the substep. An impact refreshes
only its participants' leaves, starting from the impact time and never from the pre-impact pose.
It then re-searches only the sources whose leaves overlap a changed body's old or new sweep.
Unbounded targets (half-spaces, non-finite bounds) stay in the unpruned list. A wake rebuilds
the tree, and a changed body outside the tree re-examines every source. Cached events wait in a
versioned min-heap ordered by absolute TOI (`total_cmp`), then by the stable body-order pair key,
then by source. Pair exclusions expire through per-body trajectory epochs instead of a rescan.
Results are bit-identical to the linear scheduler; only candidate counts drop. The moving-body
list still takes one O(n+b) preparation traversal per CCD step, and building the tree costs
O(n log n). There is no absolute wall-clock ratchet.
An explicitly ignored first-tick benchmark repeats 101 constructed worlds per point (setup
excluded) and verifies all admitted hits before reporting the median. With 128 distant statics,
in release builds on Rust 1.99.1 x86_64:

| Simultaneous spheres | Non-CCD, one wall | CCD, one impact (#468 → #469) | CCD, two impacts (#468 → #469) |
| ---: | ---: | ---: | ---: |
| 32 | 60 µs | 311 → 133 µs | 499 → 167 µs |
| 64 | 168 µs | 1,116 → 292 µs | 2,016 → 376 µs |
| 128 | 260 µs | 4,338 → 655 µs | 8,472 → 872 µs |

(Local 4-vCPU Intel Xeon @ 2.10 GHz. #468 measured 6,094/11,251 µs at 128 on a shared Xeon Platinum
8573C.) These cases are deliberately synthetic, not a legal MMORPG bomb rate, and these noisy
wall-clock observations are **advisory**, not CI thresholds. Exact geometric fast paths already
handle the common sphere/box pairs, so adding another sphere solver will not speed up the
scheduler. Dense clusters, where every source really overlaps every other's sweep, are still
quadratic in the true candidate pairs.
To measure full first-step cost without including world setup (advisory on shared hardware):

```sh
cargo test --release --locked --test ccd_scheduler_scaling -- --nocapture
cargo test --release --locked --test ccd_scheduler_scaling -- --ignored --nocapture
cargo run --release --locked -p physics-oracle -- --only ccd-synchronized
bash scripts/collect-work-evidence.sh > /tmp/ccd-work-evidence.json
```

## Contact events

`Report::contact_events` lists the step's contact enter/exit transitions (Q33,
[D7](technical-decisions.md#d7--physical-interactions-reuse-the-world-consumer-gated-q32q38)) as
`ContactEvent::Started(a, b)` and `ContactEvent::Stopped(a, b)`, `a` before `b` in body order.
They are derived from the persistent contact cache, not tracked separately: a pair is in contact
while the cache holds it (the solver found a point within twice the contact slop), and an impulse
on a pair the cache does not hold (a swept or CCD impact, a projectile retired on impact) is a
contact as well.

- Only physical pairs take part: a dynamic body against another dynamic, static, kinematic,
  halfspace or terrain body, when their layers collide. Sensors, query-only shapes and
  `move_characters` produce no events (Rapier's `move_shape` produces none either); sensor
  areas and other logical zones are game-owned and answered by `overlap_shape` with
  `QueryFilter::include_sensors`.
- Per step and pair: at most one `Started` and one `Stopped`, `Started` first. Sustained and
  sleeping contacts report nothing. A pair that touched and separated within the step reports
  both; a contact that broke and re-formed within it reports neither. An uncached impact's
  `Stopped` is held to the next step and dropped if the pair is in contact by then, so a body
  that lands through a swept impulse and rests starts once; a bounce stops one step later.
- Ordered by pair (`a`, then `b`, in body order), so identical across worker counts and storage
  modes, and, for keyed bodies, independent of insertion order.
- `remove_body` stops the removed body's contacts; the next successful step reports them, with
  the removed body's id (as Rapier's `Stopped` with `REMOVED`). A body retired on impact reports
  its `Stopped` in the step that retires it. A keyed body re-added with the same key is a new
  pair.
- A failed step reports nothing and keeps the removals and held impacts for the next successful
  step. `step(0.0)` and a quiescent step report what is pending.
- Cost: O(c log c) per step in the c cache insertions and removals; nothing when no contact
  changed. Checked against Rapier's `CollisionEvent`s by the `contact-events` oracle scenario.

`set_kinematic_position`, `set_static_position` and `remove_body` wake the sleeping bodies a
body touched or now touches, so a crate left behind falls and its contact stops in the next step
(#456; placement: one query-tree descent at each pose, no contact-cache scan). Not covered: a
character move does not wake a sleeping body the character leaves, so their contact stays until
that body wakes. Rapier's sensor intersection events,
contact forces and per-collider opt-in (`ActiveEvents`) are not provided. Pending transitions are
not checkpointed: a restored world's events start from its restored cache.

## Joints

`add_joint(DistanceJoint { a, b, anchor_a, anchor_b, max_length })` ties an anchor on body `a`
to one on body `b` (each in its body's local frame, scene units) by a rope: the anchors are at
most `max_length` apart and may come closer freely, as Rapier 0.36.1's `RopeJoint`
(`DistanceJoint::rope(a, b, max_length)` ties the centres). Rapier's coupled linear limit applies
only its maximum (`FIXME: handle min limit too`), so there is no minimum length here either; a
fixed attachment, hinges, motors and rope discretization are later slices (#453). Contacts
between the joined bodies stay enabled, as Rapier's default.

- **Ids.** `add_joint` returns a `JointId`, a creation serial never reused within the world
  (`JointId::{to_bits, from_bits}` serve adapters). `joint(id)`, `joint_impulse(id)`,
  `joints()` (canonical order), `joint_count()` and `remove_joint(id)` (`None` for an unknown or
  removed id) read and remove them.
- **Validation.** Both bodies must be live (`MissingBody`) and distinct, at least one dynamic and
  neither `retire_on_impact`; anchors finite and below 1e12, `max_length` finite and positive
  (`InvalidInput` otherwise). Body properties are fixed after `add_body`, so this holds for the
  joint's life.
- **Lifecycle.** `remove_body` removes the body's joints with it, as Rapier's
  `RigidBodySet::remove` does; their ids stop resolving, and a new body in the same slot (a new
  `BodyId` generation) is joined only by joints added for it. Adding or removing a joint, or a
  joint going with its body, wakes the joined dynamic bodies.
- **Order.** Rows run in canonical order: by the pair's body orders (lower first), then by id,
  so keyed worlds solve identically whatever the insertion order, in both storage modes and
  for any worker count.
- **Solver.** Each joint with an awake dynamic side is one frictionless row of the existing
  velocity solve, after the step's contact rows and in the same `velocity_iterations` (and
  convergence or island partitioning): along the rope, with the slack `max_length − distance`
  as its separation. It is rebuilt every substep from the current poses and warm-started from
  its previous impulse like contact history. A slack rope admits closing up to its length
  within the substep (Rapier's `rhs_wo_bias`); a stretched one corrects the excess within the
  substep, at most 3 m/s (in `length_unit`), Rapier's rigid joint softness and
  `max_corrective_velocity`. `joint_impulse` is the last substep's accumulated impulse along
  the rope (tension × substep length); `Report::joint_rows` counts the rows.
- **Islands.** Joined dynamic bodies are one island: they sleep together and a wake reaches
  across the joint. A taut rope holding a body from above supports it for sleep, as a contact
  under it does, so a still hanging load sleeps. A sleeping body joined to a moving external
  (kinematic) body wakes in the next step's contact phase.
- **Driven anchors.** An external body moved by `set_velocity` drags its rope with its
  velocity, as Rapier's velocity-based kinematic body. A body moved by `set_kinematic_position`
  or `move_characters` carries no velocity, so a rope follows it only through the 3 m/s
  position correction; a consumer dragging with a character sets the anchor body's velocity.
- **Cost.** O(joints) per substep (rows and the wake test visit every joint, asleep or not);
  nothing without joints. Lookups and `remove_joint` are O(joints); `remove_body` adds
  O(joints) while the world has joints. `work::Counter::JointRows` counts the visits
  (`step-joints` complexity contract).
- **Failure and checkpoints.** A failed step restores every joint impulse with the rest of the
  transaction. Worlds with joints are not checkpointed: `checkpoint` returns
  `CheckpointError::UnsupportedJoints`.

The `rope-joints` oracle scenario (C21) checks dragged and hanging fixtures against Rapier; see
[consumer capabilities](consumer-capabilities.md).

## Failure behaviour

A returned step error rolls back touched motion, history, sleep and retirement state; time and the
prior successful report are retained ([step transactions](floating-step-transactions.md)). Panics
are excluded. Non-finite inputs are rejected, never saturated.

## Checkpoints

Checkpoints are opt-in (Cargo feature `checkpoints`) for debugging and logging. Versioned physical
checkpoints restore only on their own build and target
([floating checkpoints](floating-checkpoints.md)). Moving between providers or targets
reconstructs physics from consumer-owned game state; Rapier state is never deserialized into a
Physics Engine checkpoint. Worlds with joints, terrain or halfspaces are not checkpointed.

## Example

`examples/floating_world.rs` is the minimal compiling example; `cargo build --examples --locked`
builds it.
