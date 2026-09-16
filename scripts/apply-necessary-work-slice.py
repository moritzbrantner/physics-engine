from __future__ import annotations

from pathlib import Path
import re


def read(path: str) -> str:
    return Path(path).read_text()


def write(path: str, text: str) -> None:
    Path(path).write_text(text)


def replace_once(path: str, old: str, new: str) -> None:
    text = read(path)
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{path}: expected one occurrence, found {count}: {old[:120]!r}")
    write(path, text.replace(old, new, 1))


def sub_once(path: str, pattern: str, replacement: str) -> None:
    text = read(path)
    updated, count = re.subn(pattern, replacement, text, count=1, flags=re.S)
    if count != 1:
        raise RuntimeError(f"{path}: expected one regex occurrence, found {count}: {pattern[:120]!r}")
    write(path, updated)


# 1. Indexed BVH: query only the neighborhood of bodies that actually changed.
replace_once(
    "src/rotating_broad_phase_tree.rs",
    "    fn node(&self, index: NodeIndex) -> &ArenaNode3d {\n",
    '''    pub(super) fn for_each_candidate_pair_for_body(
        &self,
        id: BodyId,
        mut visit: impl FnMut(BodyId, BodyId),
    ) {
        let Some(&leaf) = self.leaf_by_id.get(&id) else {
            return;
        };
        let ArenaNodeKind3d::Leaf(body) = self.node(leaf).kind else {
            return;
        };
        let Some(root) = self.root else {
            return;
        };
        self.visit_body_against(body, root, &mut visit);
    }

    fn visit_body_against(
        &self,
        body: BoundedBody3d,
        index: NodeIndex,
        visit: &mut impl FnMut(BodyId, BodyId),
    ) {
        let node = *self.node(index);
        if !bounds_overlap(body.bounds, node.bounds) {
            return;
        }
        match node.kind {
            ArenaNodeKind3d::Leaf(other) => {
                if body.id == other.id
                    || (body.kind == BodyKind::Fixed && other.kind == BodyKind::Fixed)
                {
                    return;
                }
                let (left, right) = if body.id < other.id {
                    (body.id, other.id)
                } else {
                    (other.id, body.id)
                };
                visit(left, right);
            }
            ArenaNodeKind3d::Branch { left, right } => {
                self.visit_body_against(body, left, visit);
                self.visit_body_against(body, right, visit);
            }
        }
    }

    fn node(&self, index: NodeIndex) -> &ArenaNode3d {
''',
)

# 2. Broad phase: explicit partial-current query API. It consumes only changed bodies,
# updates only their exact bounds/leaves, and traverses only their BVH neighborhoods.
replace_once(
    "src/rotating_broad_phase.rs",
    "    pub rotations: u64,\n",
    "    pub rotations: u64,\n    pub partial_queries: u64,\n    pub partial_body_updates: u64,\n",
)
replace_once(
    "src/rotating_broad_phase.rs",
    "    DuplicateBodyId(BodyId),\n    FreeFlight(RigidBoxFreeFlightError3d),\n",
    "    DuplicateBodyId(BodyId),\n    IncrementalQueryUnsynchronized(BodyId),\n    FreeFlight(RigidBoxFreeFlightError3d),\n",
)
replace_once(
    "src/rotating_broad_phase.rs",
    '''            Self::DuplicateBodyId(id) => write!(
                formatter,
                "rotational broad phase received duplicate body id {}",
                id.0
            ),
            Self::FreeFlight(error) => {
''',
    '''            Self::DuplicateBodyId(id) => write!(
                formatter,
                "rotational broad phase received duplicate body id {}",
                id.0
            ),
            Self::IncrementalQueryUnsynchronized(id) => write!(
                formatter,
                "incremental rotational broad-phase query is not synchronized for body {}",
                id.0
            ),
            Self::FreeFlight(error) => {
''',
)
replace_once(
    "src/rotating_broad_phase.rs",
    "    #[must_use]\n    pub const fn stats(&self) -> RotatingBroadPhaseStats3d {\n",
    '''    /// Updates current-position bounds only for the supplied bodies and returns only candidate
    /// pairs touching one of those bodies. The full broad phase must already be synchronized to the
    /// same current world state; this is the precise API for contact-island propagation after a local
    /// solver response, where unchanged bodies cannot create a new current overlap by themselves.
    pub(crate) fn candidate_pairs_for_changed_current_bodies<'a>(
        &mut self,
        changed_boxes: impl IntoIterator<Item = &'a RigidBox3d>,
    ) -> Result<Vec<RotationalSweepPair3d>, RotatingBroadPhaseError3d> {
        self.stats.queries = self.stats.queries.saturating_add(1);
        self.stats.partial_queries = self.stats.partial_queries.saturating_add(1);
        let current = RigidBoxFreeFlightConfig3d::new(crate::Vec3i::ZERO, 0, 1);
        let mut changed_ids = BTreeSet::new();
        let mut escaped = Vec::new();

        for rigid_box in changed_boxes {
            let body = bounded_body(rigid_box, current)?;
            if !changed_ids.insert(body.id) {
                return Err(RotatingBroadPhaseError3d::DuplicateBodyId(body.id));
            }
            let Some(previous) = self.exact.get(&body.id) else {
                return Err(RotatingBroadPhaseError3d::IncrementalQueryUnsynchronized(body.id));
            };
            if previous.kind != body.kind || previous.collision_layers != body.collision_layers {
                return Err(RotatingBroadPhaseError3d::IncrementalQueryUnsynchronized(body.id));
            }
            self.exact.insert(body.id, body);
            let Some(fat) = self.tree.leaf_bounds(body.id) else {
                return Err(RotatingBroadPhaseError3d::IncrementalQueryUnsynchronized(body.id));
            };
            if !contains_bounds(fat, body.bounds) {
                let mut fat_body = body;
                fat_body.bounds = fatten_bounds(body.bounds);
                escaped.push(fat_body);
            }
        }

        self.stats.partial_body_updates = self
            .stats
            .partial_body_updates
            .saturating_add(u64::try_from(changed_ids.len()).unwrap_or(u64::MAX));

        if escaped.is_empty() {
            self.stats.reuses = self.stats.reuses.saturating_add(1);
        } else {
            self.stats.incremental_updates = self.stats.incremental_updates.saturating_add(1);
            let mut rotations = 0_u64;
            let mut incremental_ok = true;
            for body in escaped {
                if self.tree.reinsert(body, &mut rotations) {
                    self.stats.reinserts = self.stats.reinserts.saturating_add(1);
                } else {
                    incremental_ok = false;
                    break;
                }
            }
            self.stats.rotations = self.stats.rotations.saturating_add(rotations);
            if !incremental_ok {
                self.stats.rebuilds = self.stats.rebuilds.saturating_add(1);
                let exact = self.exact.values().copied().collect();
                self.rebuild(exact);
            }
        }

        let exact = &self.exact;
        let mut pairs = BTreeSet::new();
        for id in changed_ids {
            self.tree.for_each_candidate_pair_for_body(id, |left, right| {
                let left_body = exact
                    .get(&left)
                    .expect("incremental broad phase keeps every leaf exact bound");
                let right_body = exact
                    .get(&right)
                    .expect("incremental broad phase keeps every leaf exact bound");
                if bounds_overlap(left_body.bounds, right_body.bounds)
                    && left_body
                        .collision_layers
                        .collides_with(right_body.collision_layers)
                {
                    pairs.insert(RotationalSweepPair3d { left, right });
                }
            });
        }
        Ok(pairs.into_iter().collect())
    }

    #[must_use]
    pub const fn stats(&self) -> RotatingBroadPhaseStats3d {
''',
)
sub_once(
    "src/rotating_broad_phase.rs",
    r"fn bounded_bodies\(\n    boxes: &\[RigidBox3d\],\n    config: RigidBoxFreeFlightConfig3d,\n\) -> Result<Vec<BoundedBody3d>, RotatingBroadPhaseError3d> \{.*?\n\}\n\nfn fatten_bounds",
    '''fn bounded_bodies(
    boxes: &[RigidBox3d],
    config: RigidBoxFreeFlightConfig3d,
) -> Result<Vec<BoundedBody3d>, RotatingBroadPhaseError3d> {
    let mut ids = BTreeSet::new();
    let mut bounded = Vec::with_capacity(boxes.len());
    for rigid_box in boxes {
        let body = bounded_body(rigid_box, config)?;
        if !ids.insert(body.id) {
            return Err(RotatingBroadPhaseError3d::DuplicateBodyId(body.id));
        }
        bounded.push(body);
    }
    Ok(bounded)
}

fn bounded_body(
    rigid_box: &RigidBox3d,
    config: RigidBoxFreeFlightConfig3d,
) -> Result<BoundedBody3d, RotatingBroadPhaseError3d> {
    Ok(BoundedBody3d {
        id: rigid_box.body().id(),
        kind: rigid_box.body().kind(),
        collision_layers: rigid_box.collision_layers(),
        bounds: rigid_box_free_flight_sweep_bounds(rigid_box, config)?,
    })
}

fn fatten_bounds''',
)

# Existing exhaustive error adapters stay explicit and fail closed if an incremental precondition is violated.
for path, marker in [
    ("src/rotating_world.rs", "fn map_tail_broad_phase_error"),
    ("src/current_contact_query.rs", "fn map_broad_phase_error"),
    ("src/stabilized_rotating_world.rs", "fn map_fixed_boundary_broad_phase_error"),
]:
    text = read(path)
    start = text.index(marker)
    tail = text[start:]
    old = "        RotatingBroadPhaseError3d::DuplicateBodyId(id) => RotatingWorldError3d::DuplicateBody(id),\n        RotatingBroadPhaseError3d::FreeFlight(error) => RotatingWorldError3d::FreeFlight(error),\n"
    if old not in tail:
        raise RuntimeError(f"{path}: broad-phase adapter body not found")
    new = "        RotatingBroadPhaseError3d::DuplicateBodyId(id) => RotatingWorldError3d::DuplicateBody(id),\n        RotatingBroadPhaseError3d::IncrementalQueryUnsynchronized(id) => {\n            RotatingWorldError3d::MissingBody(id)\n        }\n        RotatingBroadPhaseError3d::FreeFlight(error) => RotatingWorldError3d::FreeFlight(error),\n"
    text = text[:start] + tail.replace(old, new, 1)
    write(path, text)

# 3. Contact response scratch: callers can deliberately reuse indexing and solver buffers.
replace_once(
    "src/rotating_contact_response.rs",
    "use std::{collections::BTreeMap, error::Error, fmt};\n",
    "use std::{collections::{BTreeMap, BTreeSet}, error::Error, fmt};\n",
)
replace_once(
    "src/rotating_contact_response.rs",
    "/// Resolves every contact in one shared sampled frontier with bounded simultaneous passes.\n",
    '''/// Reusable allocation/index storage for repeated frontier responses.
///
/// The default response function remains convenient, while hot loops can opt into this precise API
/// instead of rebuilding maps and vectors for every event/pass.
#[derive(Clone, Debug, Default)]
pub struct RotatingContactResponseScratch3d {
    indices: BTreeMap<BodyId, usize>,
    resolved_indices: Vec<Option<(usize, usize)>>,
    snapshot: Vec<RigidBox3d>,
    deltas: Vec<BodyDeltaAccumulator3d>,
    combined: Vec<BodyDelta3d>,
    modified_body_ids: BTreeSet<BodyId>,
}

/// Resolves every contact in one shared sampled frontier with bounded simultaneous passes.
''',
)
sub_once(
    "src/rotating_contact_response.rs",
    r"pub fn resolve_rotating_contact_frontier\(\n    frontier: RotatingContactFrontier3d,\n    solver_passes: u8,\n\) -> Result<RotatingContactResponse3d, RotatingContactResponseError3d> \{.*?\n\}\n\nfn primitive_axis",
    '''pub fn resolve_rotating_contact_frontier(
    frontier: RotatingContactFrontier3d,
    solver_passes: u8,
) -> Result<RotatingContactResponse3d, RotatingContactResponseError3d> {
    let mut scratch = RotatingContactResponseScratch3d::default();
    resolve_rotating_contact_frontier_with_scratch(frontier, solver_passes, &mut scratch)
}

pub fn resolve_rotating_contact_frontier_with_scratch(
    frontier: RotatingContactFrontier3d,
    solver_passes: u8,
    scratch: &mut RotatingContactResponseScratch3d,
) -> Result<RotatingContactResponse3d, RotatingContactResponseError3d> {
    resolve_rotating_contact_frontier_with_activity_and_scratch(frontier, solver_passes, scratch)
        .map(|(response, _)| response)
}

pub(crate) fn resolve_rotating_contact_frontier_with_activity_and_scratch(
    frontier: RotatingContactFrontier3d,
    solver_passes: u8,
    scratch: &mut RotatingContactResponseScratch3d,
) -> Result<(RotatingContactResponse3d, Vec<BodyId>), RotatingContactResponseError3d> {
    if solver_passes == 0 {
        return Err(RotatingContactResponseError3d::ZeroSolverPasses);
    }

    let RotatingContactResponseScratch3d {
        indices,
        resolved_indices,
        snapshot,
        deltas,
        combined,
        modified_body_ids,
    } = scratch;

    indices.clear();
    indices.extend(
        frontier
            .boxes
            .iter()
            .enumerate()
            .map(|(index, rigid_box)| (rigid_box.body.id, index)),
    );
    let mut boxes = frontier.boxes;
    let mut passes_used = 0_u8;
    resolved_indices.clear();
    resolved_indices.resize(frontier.contacts.len(), None);
    snapshot.clear();
    snapshot.extend(boxes.iter().cloned());
    deltas.resize_with(boxes.len(), BodyDeltaAccumulator3d::default);
    deltas.truncate(boxes.len());
    combined.clear();
    combined.reserve(boxes.len().saturating_sub(combined.capacity()));
    modified_body_ids.clear();

    for pass in 0..solver_passes {
        if pass > 0 {
            snapshot.clone_from(&boxes);
        }
        for accumulator in deltas.iter_mut() {
            accumulator.clear();
        }

        for (contact_index, contact) in frontier.contacts.iter().enumerate() {
            let (left_index, right_index) = match resolved_indices[contact_index] {
                Some(indices) => indices,
                None => {
                    let left_index = *indices.get(&contact.pair.left).ok_or(
                        RotatingContactResponseError3d::MissingBody(contact.pair.left),
                    )?;
                    let right_index = *indices.get(&contact.pair.right).ok_or(
                        RotatingContactResponseError3d::MissingBody(contact.pair.right),
                    )?;
                    let resolved = (left_index, right_index);
                    resolved_indices[contact_index] = Some(resolved);
                    resolved
                }
            };
            let response = resolve_obb_contact(
                snapshot[left_index].clone(),
                snapshot[right_index].clone(),
                pass == 0,
            )?;
            let Some(resolved_contact) = response.contact else {
                continue;
            };
            deltas[left_index].accumulate(
                negate_axis(resolved_contact.axis, contact.pair.left)?,
                &snapshot[left_index],
                &response.left,
            )?;
            deltas[right_index].accumulate(
                resolved_contact.axis,
                &snapshot[right_index],
                &response.right,
            )?;
        }

        combined.clear();
        for (rigid_box, accumulator) in snapshot.iter().zip(deltas.iter()) {
            combined.push(accumulator.combined(rigid_box.body.id)?);
        }
        if combined.iter().copied().all(BodyDelta3d::is_zero) {
            break;
        }
        for (rigid_box, delta) in boxes.iter_mut().zip(combined.iter().copied()) {
            if !delta.is_zero() {
                modified_body_ids.insert(rigid_box.body.id);
                apply_delta(rigid_box, delta)?;
            }
        }
        passes_used = passes_used.checked_add(1).ok_or(
            RotatingContactResponseError3d::ArithmeticOverflow(
                boxes
                    .first()
                    .map_or(BodyId(0), |rigid_box| rigid_box.body.id),
            ),
        )?;
    }

    let modified = modified_body_ids.iter().copied().collect();
    Ok((
        RotatingContactResponse3d {
            boxes,
            time: frontier.time,
            contacts: frontier.contacts,
            remaining_numerator: frontier.remaining_numerator,
            passes_used,
        },
        modified,
    ))
}

fn primitive_axis''',
)

# 4. Repeated-event hot path: reuse response scratch and propagate only changed contact islands.
replace_once(
    "src/repeated_rotating_events.rs",
    "    RotatingContactSearchHit3d, SampledContactTime3d, obb_contact_seed,\n    resolve_rotating_contact_frontier,\n};\n",
    "    RotatingContactSearchHit3d, SampledContactTime3d, obb_contact_seed,\n};\n",
)
replace_once(
    "src/repeated_rotating_events.rs",
    '''    rotating_broad_phase::RotatingBroadPhase3d,
    rotating_contact_frontier::{
''',
    '''    rotating_broad_phase::RotatingBroadPhase3d,
    rotating_contact_response::{
        RotatingContactResponseScratch3d,
        resolve_rotating_contact_frontier_with_activity_and_scratch,
    },
    rotating_contact_frontier::{
''',
)
replace_once(
    "src/repeated_rotating_events.rs",
    "/// Result of consuming every sampled rotating event admitted before the first unresolved tail.\n",
    '''#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct RepeatedRotatingEventWorkStats3d {
    pub event_response_passes: u64,
    pub stabilization_passes: u64,
    pub stabilizations_hitting_limit: u64,
    pub stabilization_candidate_pairs: u64,
    pub stabilization_exact_contacts: u64,
    pub stabilization_active_bodies: u64,
}

/// Result of consuming every sampled rotating event admitted before the first unresolved tail.
''',
)
replace_once(
    "src/repeated_rotating_events.rs",
    "    pub remaining: RigidBoxFreeFlightConfig3d,\n}\n",
    "    pub remaining: RigidBoxFreeFlightConfig3d,\n    pub work: RepeatedRotatingEventWorkStats3d,\n}\n",
)
replace_once(
    "src/repeated_rotating_events.rs",
    "    validate_config(config)?;\n\n    let mut remaining = config.search.free_flight;\n",
    "    validate_config(config)?;\n\n    let mut work = RepeatedRotatingEventWorkStats3d::default();\n    let mut response_scratch = RotatingContactResponseScratch3d::default();\n    let mut remaining = config.search.free_flight;\n",
)
replace_once(
    "src/repeated_rotating_events.rs",
    '''        return Ok(RepeatedRotatingEventAdvance3d {
            boxes: boxes.to_vec(),
            events: Vec::new(),
            remaining,
        });
''',
    '''        return Ok(RepeatedRotatingEventAdvance3d {
            boxes: boxes.to_vec(),
            events: Vec::new(),
            remaining,
            work,
        });
''',
)
replace_once(
    "src/repeated_rotating_events.rs",
    '''        let response = resolve_rotating_contact_frontier(frontier, config.solver_passes)?;
        remaining = scale_remaining_time(remaining, response.remaining_numerator, response.time)?;
        let response_time = response.time;
        let response_contacts = response.contacts;
        let response_passes = response.passes_used;
        state = stabilize_current_contacts(response.boxes, config.solver_passes, broad_phase)?;
''',
    '''        let (response, modified_body_ids) =
            resolve_rotating_contact_frontier_with_activity_and_scratch(
                frontier,
                config.solver_passes,
                &mut response_scratch,
            )?;
        remaining = scale_remaining_time(remaining, response.remaining_numerator, response.time)?;
        let response_time = response.time;
        let response_contacts = response.contacts;
        let response_passes = response.passes_used;
        work.event_response_passes = work
            .event_response_passes
            .saturating_add(u64::from(response_passes));
        state = stabilize_current_contacts(
            response.boxes,
            config.solver_passes,
            broad_phase,
            &modified_body_ids,
            &mut response_scratch,
            &mut work,
        )?;
''',
)
replace_once(
    "src/repeated_rotating_events.rs",
    '''    Ok(RepeatedRotatingEventAdvance3d {
        boxes: state,
        events,
        remaining,
    })
''',
    '''    Ok(RepeatedRotatingEventAdvance3d {
        boxes: state,
        events,
        remaining,
        work,
    })
''',
)
sub_once(
    "src/repeated_rotating_events.rs",
    r"fn stabilize_current_contacts\(.*?\n\}\n\n#\[cfg\(test\)\]\nfn current_contact_frontier\(",
    '''fn stabilize_current_contacts(
    mut boxes: Vec<RigidBox3d>,
    solver_passes: u8,
    broad_phase: &mut RotatingBroadPhase3d,
    initial_active: &[crate::BodyId],
    response_scratch: &mut RotatingContactResponseScratch3d,
    work: &mut RepeatedRotatingEventWorkStats3d,
) -> Result<Vec<RigidBox3d>, RepeatedRotatingEventError3d> {
    let mut active = initial_active.to_vec();
    active.sort_unstable();
    active.dedup();
    let mut exhausted_with_changes = false;

    for pass in 0..solver_passes {
        if active.is_empty() {
            break;
        }
        work.stabilization_active_bodies = work
            .stabilization_active_bodies
            .saturating_add(u64::try_from(active.len()).unwrap_or(u64::MAX));
        let current = current_contact_frontier_for_bodies_with_broad_phase(
            &boxes,
            &active,
            broad_phase,
        )?;
        work.stabilization_candidate_pairs = work
            .stabilization_candidate_pairs
            .saturating_add(u64::try_from(current.candidate_pairs).unwrap_or(u64::MAX));
        work.stabilization_exact_contacts = work
            .stabilization_exact_contacts
            .saturating_add(u64::try_from(current.exact_contacts).unwrap_or(u64::MAX));
        let Some(frontier) = current.frontier else {
            active.clear();
            break;
        };
        work.stabilization_passes = work.stabilization_passes.saturating_add(1);
        let (response, modified_body_ids) =
            resolve_rotating_contact_frontier_with_activity_and_scratch(
                frontier,
                1,
                response_scratch,
            )?;
        boxes = response.boxes;
        active = modified_body_ids;
        if active.is_empty() {
            break;
        }
        if pass.saturating_add(1) == solver_passes {
            exhausted_with_changes = true;
        }
    }

    if exhausted_with_changes {
        work.stabilizations_hitting_limit = work.stabilizations_hitting_limit.saturating_add(1);
    }
    Ok(boxes)
}

#[derive(Clone, Debug)]
struct CurrentContactFrontierResult3d {
    frontier: Option<RotatingContactFrontier3d>,
    candidate_pairs: usize,
    exact_contacts: usize,
}

fn current_contact_frontier_for_bodies_with_broad_phase(
    boxes: &[RigidBox3d],
    active: &[crate::BodyId],
    broad_phase: &mut RotatingBroadPhase3d,
) -> Result<CurrentContactFrontierResult3d, RotatingContactFrontierError3d> {
    let mut active_boxes = Vec::with_capacity(active.len());
    for id in active {
        let rigid_box = boxes
            .iter()
            .find(|rigid_box| rigid_box.body().id() == *id)
            .ok_or(RotatingContactFrontierError3d::MissingBody(*id))?;
        active_boxes.push(rigid_box);
    }
    let candidates = broad_phase
        .candidate_pairs_for_changed_current_bodies(active_boxes.into_iter())?;
    let candidate_pairs = candidates.len();
    if candidates.is_empty() {
        return Ok(CurrentContactFrontierResult3d {
            frontier: None,
            candidate_pairs,
            exact_contacts: 0,
        });
    }

    let mut contacts = Vec::new();
    for pair in candidates {
        let left = boxes
            .iter()
            .find(|rigid_box| rigid_box.body().id() == pair.left)
            .ok_or(RotatingContactFrontierError3d::MissingBody(pair.left))?;
        let right = boxes
            .iter()
            .find(|rigid_box| rigid_box.body().id() == pair.right)
            .ok_or(RotatingContactFrontierError3d::MissingBody(pair.right))?;
        let Some(contact) = obb_contact_seed(left.oriented_box(), right.oriented_box())? else {
            continue;
        };
        contacts.push(RotatingContactSearchHit3d {
            time: SampledContactTime3d::ZERO,
            pair,
            contact,
        });
    }
    let exact_contacts = contacts.len();
    let frontier = if contacts.is_empty() {
        None
    } else {
        Some(RotatingContactFrontier3d {
            boxes: boxes.to_vec(),
            time: SampledContactTime3d::ZERO,
            contacts,
            remaining_numerator: 1,
        })
    };
    Ok(CurrentContactFrontierResult3d {
        frontier,
        candidate_pairs,
        exact_contacts,
    })
}

#[cfg(test)]
fn current_contact_frontier(''',
)
replace_once(
    "src/repeated_rotating_events.rs",
    '''    let mut broad_phase = RotatingBroadPhase3d::default();
    current_contact_frontier_with_broad_phase(boxes, &mut broad_phase)
}
''',
    '''    let mut broad_phase = RotatingBroadPhase3d::default();
    let zero_time = RigidBoxFreeFlightConfig3d::new(crate::Vec3i::ZERO, 0, 1);
    broad_phase.candidate_pairs(boxes, zero_time)?;
    let active = boxes.iter().map(|rigid_box| rigid_box.body().id()).collect::<Vec<_>>();
    Ok(current_contact_frontier_for_bodies_with_broad_phase(boxes, &active, &mut broad_phase)?.frontier)
}
''',
)

# 5. World-level diagnostics make the amplification and necessary-work savings first-class evidence.
replace_once(
    "src/rotating_world.rs",
    "    pub broad_phase_reuses: u64,\n}\n",
    '''    pub broad_phase_reuses: u64,
    pub broad_phase_incremental_updates: u64,
    pub broad_phase_reinserts: u64,
    pub broad_phase_rotations: u64,
    pub broad_phase_partial_queries: u64,
    pub broad_phase_partial_body_updates: u64,
    pub event_response_passes: u64,
    pub stabilization_passes: u64,
    pub stabilizations_hitting_limit: u64,
    pub stabilization_candidate_pairs: u64,
    pub stabilization_exact_contacts: u64,
    pub stabilization_active_bodies: u64,
}
''',
)
replace_once(
    "src/rotating_world.rs",
    '''                broad_phase_reuses: broad_phase_after
                    .reuses
                    .saturating_sub(broad_phase_before.reuses),
''',
    '''                broad_phase_reuses: broad_phase_after
                    .reuses
                    .saturating_sub(broad_phase_before.reuses),
                broad_phase_incremental_updates: broad_phase_after
                    .incremental_updates
                    .saturating_sub(broad_phase_before.incremental_updates),
                broad_phase_reinserts: broad_phase_after
                    .reinserts
                    .saturating_sub(broad_phase_before.reinserts),
                broad_phase_rotations: broad_phase_after
                    .rotations
                    .saturating_sub(broad_phase_before.rotations),
                broad_phase_partial_queries: broad_phase_after
                    .partial_queries
                    .saturating_sub(broad_phase_before.partial_queries),
                broad_phase_partial_body_updates: broad_phase_after
                    .partial_body_updates
                    .saturating_sub(broad_phase_before.partial_body_updates),
                event_response_passes: advance.work.event_response_passes,
                stabilization_passes: advance.work.stabilization_passes,
                stabilizations_hitting_limit: advance.work.stabilizations_hitting_limit,
                stabilization_candidate_pairs: advance.work.stabilization_candidate_pairs,
                stabilization_exact_contacts: advance.work.stabilization_exact_contacts,
                stabilization_active_bodies: advance.work.stabilization_active_bodies,
''',
)

# 6. Public API exports: hot callers can choose scratch explicitly; event work is inspectable.
replace_once(
    "src/lib.rs",
    '''    MAX_REPEATED_ROTATING_EVENTS, RepeatedRotatingEventAdvance3d, RepeatedRotatingEventConfig3d,
    RepeatedRotatingEventError3d, RotatingResolvedEvent3d, advance_repeated_rotating_events,
''',
    '''    MAX_REPEATED_ROTATING_EVENTS, RepeatedRotatingEventAdvance3d, RepeatedRotatingEventConfig3d,
    RepeatedRotatingEventError3d, RepeatedRotatingEventWorkStats3d, RotatingResolvedEvent3d,
    advance_repeated_rotating_events,
''',
)
replace_once(
    "src/lib.rs",
    '''pub use rotating_contact_response::{
    RotatingContactResponse3d, RotatingContactResponseError3d, resolve_rotating_contact_frontier,
};
''',
    '''pub use rotating_contact_response::{
    RotatingContactResponse3d, RotatingContactResponseError3d, RotatingContactResponseScratch3d,
    resolve_rotating_contact_frontier, resolve_rotating_contact_frontier_with_scratch,
};
''',
)

# 7. WASM/browser/benchmark observability: direct counters, not inferred formulas.
wasm_marker = '''#[unsafe(no_mangle)]
pub extern "C" fn sandbox_last_broad_phase_reuses() -> u32 {
    with_sandbox(|sandbox| saturating_u32(sandbox.last_step_stats.broad_phase_reuses))
}
'''
wasm_extra = wasm_marker + '''
#[unsafe(no_mangle)]
pub extern "C" fn sandbox_last_broad_phase_incremental_updates() -> u32 {
    with_sandbox(|sandbox| saturating_u32(sandbox.last_step_stats.broad_phase_incremental_updates))
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_last_broad_phase_reinserts() -> u32 {
    with_sandbox(|sandbox| saturating_u32(sandbox.last_step_stats.broad_phase_reinserts))
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_last_broad_phase_rotations() -> u32 {
    with_sandbox(|sandbox| saturating_u32(sandbox.last_step_stats.broad_phase_rotations))
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_last_broad_phase_partial_queries() -> u32 {
    with_sandbox(|sandbox| saturating_u32(sandbox.last_step_stats.broad_phase_partial_queries))
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_last_broad_phase_partial_body_updates() -> u32 {
    with_sandbox(|sandbox| saturating_u32(sandbox.last_step_stats.broad_phase_partial_body_updates))
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_last_event_response_passes() -> u32 {
    with_sandbox(|sandbox| saturating_u32(sandbox.last_step_stats.event_response_passes))
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_last_stabilization_passes() -> u32 {
    with_sandbox(|sandbox| saturating_u32(sandbox.last_step_stats.stabilization_passes))
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_last_stabilizations_hitting_limit() -> u32 {
    with_sandbox(|sandbox| saturating_u32(sandbox.last_step_stats.stabilizations_hitting_limit))
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_last_stabilization_candidate_pairs() -> u32 {
    with_sandbox(|sandbox| saturating_u32(sandbox.last_step_stats.stabilization_candidate_pairs))
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_last_stabilization_exact_contacts() -> u32 {
    with_sandbox(|sandbox| saturating_u32(sandbox.last_step_stats.stabilization_exact_contacts))
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_last_stabilization_active_bodies() -> u32 {
    with_sandbox(|sandbox| saturating_u32(sandbox.last_step_stats.stabilization_active_bodies))
}
'''
replace_once("demo-wasm/src/lib.rs", wasm_marker, wasm_extra)

js_counters = '''    broad_phase_reuses: readPhysicsCounter("sandbox_last_broad_phase_reuses"),
'''
js_extra = js_counters + '''    broad_phase_incremental_updates: readPhysicsCounter("sandbox_last_broad_phase_incremental_updates"),
    broad_phase_reinserts: readPhysicsCounter("sandbox_last_broad_phase_reinserts"),
    broad_phase_rotations: readPhysicsCounter("sandbox_last_broad_phase_rotations"),
    broad_phase_partial_queries: readPhysicsCounter("sandbox_last_broad_phase_partial_queries"),
    broad_phase_partial_body_updates: readPhysicsCounter("sandbox_last_broad_phase_partial_body_updates"),
    event_response_passes: readPhysicsCounter("sandbox_last_event_response_passes"),
    stabilization_passes: readPhysicsCounter("sandbox_last_stabilization_passes"),
    stabilizations_hitting_limit: readPhysicsCounter("sandbox_last_stabilizations_hitting_limit"),
    stabilization_candidate_pairs: readPhysicsCounter("sandbox_last_stabilization_candidate_pairs"),
    stabilization_exact_contacts: readPhysicsCounter("sandbox_last_stabilization_exact_contacts"),
    stabilization_active_bodies: readPhysicsCounter("sandbox_last_stabilization_active_bodies"),
'''
replace_once("site/app.js", js_counters, js_extra)

bench_counters = '''      broad_phase_reuses: readCounter("sandbox_last_broad_phase_reuses"),
'''
bench_extra = bench_counters + '''      broad_phase_incremental_updates: readCounter("sandbox_last_broad_phase_incremental_updates"),
      broad_phase_reinserts: readCounter("sandbox_last_broad_phase_reinserts"),
      broad_phase_rotations: readCounter("sandbox_last_broad_phase_rotations"),
      broad_phase_partial_queries: readCounter("sandbox_last_broad_phase_partial_queries"),
      broad_phase_partial_body_updates: readCounter("sandbox_last_broad_phase_partial_body_updates"),
      event_response_passes: readCounter("sandbox_last_event_response_passes"),
      stabilization_passes: readCounter("sandbox_last_stabilization_passes"),
      stabilizations_hitting_limit: readCounter("sandbox_last_stabilizations_hitting_limit"),
      stabilization_candidate_pairs: readCounter("sandbox_last_stabilization_candidate_pairs"),
      stabilization_exact_contacts: readCounter("sandbox_last_stabilization_exact_contacts"),
      stabilization_active_bodies: readCounter("sandbox_last_stabilization_active_bodies"),
'''
replace_once("scripts/benchmark-sandbox.mjs", bench_counters, bench_extra)

bench_sums = '''          broad_phase_reuses: sumKnown(work, "broad_phase_reuses"),
'''
bench_sums_extra = bench_sums + '''          broad_phase_incremental_updates: sumKnown(work, "broad_phase_incremental_updates"),
          broad_phase_reinserts: sumKnown(work, "broad_phase_reinserts"),
          broad_phase_rotations: sumKnown(work, "broad_phase_rotations"),
          broad_phase_partial_queries: sumKnown(work, "broad_phase_partial_queries"),
          broad_phase_partial_body_updates: sumKnown(work, "broad_phase_partial_body_updates"),
          event_response_passes: sumKnown(work, "event_response_passes"),
          stabilization_passes: sumKnown(work, "stabilization_passes"),
          stabilizations_hitting_limit: sumKnown(work, "stabilizations_hitting_limit"),
          stabilization_candidate_pairs: sumKnown(work, "stabilization_candidate_pairs"),
          stabilization_exact_contacts: sumKnown(work, "stabilization_exact_contacts"),
          stabilization_active_bodies: sumKnown(work, "stabilization_active_bodies"),
'''
replace_once("scripts/benchmark-sandbox.mjs", bench_sums, bench_sums_extra)

perf_norm = '''    broad_phase_reuses: optionalCounter(step.broad_phase_reuses, "broad_phase_reuses"),
'''
perf_norm_extra = perf_norm + '''    broad_phase_incremental_updates: optionalCounter(step.broad_phase_incremental_updates, "broad_phase_incremental_updates"),
    broad_phase_reinserts: optionalCounter(step.broad_phase_reinserts, "broad_phase_reinserts"),
    broad_phase_rotations: optionalCounter(step.broad_phase_rotations, "broad_phase_rotations"),
    broad_phase_partial_queries: optionalCounter(step.broad_phase_partial_queries, "broad_phase_partial_queries"),
    broad_phase_partial_body_updates: optionalCounter(step.broad_phase_partial_body_updates, "broad_phase_partial_body_updates"),
    event_response_passes: optionalCounter(step.event_response_passes, "event_response_passes"),
    stabilization_passes: optionalCounter(step.stabilization_passes, "stabilization_passes"),
    stabilizations_hitting_limit: optionalCounter(step.stabilizations_hitting_limit, "stabilizations_hitting_limit"),
    stabilization_candidate_pairs: optionalCounter(step.stabilization_candidate_pairs, "stabilization_candidate_pairs"),
    stabilization_exact_contacts: optionalCounter(step.stabilization_exact_contacts, "stabilization_exact_contacts"),
    stabilization_active_bodies: optionalCounter(step.stabilization_active_bodies, "stabilization_active_bodies"),
'''
replace_once("site/performance-log.mjs", perf_norm, perf_norm_extra)

perf_sum = '''            broad_phase_reuses: sumCounter(physicsStepStats, "broad_phase_reuses"),
'''
perf_sum_extra = perf_sum + '''            broad_phase_incremental_updates: sumCounter(physicsStepStats, "broad_phase_incremental_updates"),
            broad_phase_reinserts: sumCounter(physicsStepStats, "broad_phase_reinserts"),
            broad_phase_rotations: sumCounter(physicsStepStats, "broad_phase_rotations"),
            broad_phase_partial_queries: sumCounter(physicsStepStats, "broad_phase_partial_queries"),
            broad_phase_partial_body_updates: sumCounter(physicsStepStats, "broad_phase_partial_body_updates"),
            event_response_passes: sumCounter(physicsStepStats, "event_response_passes"),
            stabilization_passes: sumCounter(physicsStepStats, "stabilization_passes"),
            stabilizations_hitting_limit: sumCounter(physicsStepStats, "stabilizations_hitting_limit"),
            stabilization_candidate_pairs: sumCounter(physicsStepStats, "stabilization_candidate_pairs"),
            stabilization_exact_contacts: sumCounter(physicsStepStats, "stabilization_exact_contacts"),
            stabilization_active_bodies: sumCounter(physicsStepStats, "stabilization_active_bodies"),
'''
replace_once("site/performance-log.mjs", perf_sum, perf_sum_extra)

# Focused tests: scratch equivalence and precise partial broad-phase work.
with open("src/rotating_contact_response.rs", "a") as handle:
    handle.write(r'''

#[cfg(test)]
mod scratch_reuse_tests {
    use crate::{
        BodyId, RigidBody, RigidBox3d, RotatingContactFrontier3d, RotatingContactSearchHit3d,
        SampledContactTime3d, Vec3i, obb_contact_seed,
    };

    use super::{
        RotatingContactResponseScratch3d, resolve_rotating_contact_frontier,
        resolve_rotating_contact_frontier_with_scratch,
    };

    #[test]
    fn reusable_scratch_preserves_response_semantics() {
        let left = RigidBox3d::from_body(RigidBody::dynamic(
            BodyId(1),
            Vec3i::ZERO,
            Vec3i::new(10, 0, 0),
            Vec3i::new(5, 5, 5),
        ));
        let right = RigidBox3d::from_body(RigidBody::fixed(
            BodyId(2),
            Vec3i::new(9, 0, 0),
            Vec3i::new(5, 5, 5),
        ));
        let contact = obb_contact_seed(left.oriented_box(), right.oriented_box())
            .expect("valid boxes")
            .expect("overlap");
        let frontier = RotatingContactFrontier3d {
            boxes: vec![left, right],
            time: SampledContactTime3d::ZERO,
            contacts: vec![RotatingContactSearchHit3d {
                time: SampledContactTime3d::ZERO,
                pair: crate::RotationalSweepPair3d {
                    left: BodyId(1),
                    right: BodyId(2),
                },
                contact,
            }],
            remaining_numerator: 0,
        };
        let expected = resolve_rotating_contact_frontier(frontier.clone(), 4).expect("response");
        let mut scratch = RotatingContactResponseScratch3d::default();
        let first = resolve_rotating_contact_frontier_with_scratch(frontier.clone(), 4, &mut scratch)
            .expect("scratch response");
        let second = resolve_rotating_contact_frontier_with_scratch(frontier, 4, &mut scratch)
            .expect("reused scratch response");
        assert_eq!(first, expected);
        assert_eq!(second, expected);
    }
}
''')

with open("src/rotating_broad_phase.rs", "a") as handle:
    handle.write(r'''

#[cfg(test)]
mod necessary_work_tests {
    use crate::{BodyId, RigidBody, RigidBox3d, RigidBoxFreeFlightConfig3d, Vec3i};

    use super::RotatingBroadPhase3d;

    #[test]
    fn partial_current_query_updates_only_supplied_bodies_and_matches_full_truth() {
        let mut boxes = vec![
            RigidBox3d::from_body(RigidBody::dynamic(
                BodyId(1),
                Vec3i::new(0, 0, 0),
                Vec3i::ZERO,
                Vec3i::new(5, 5, 5),
            )),
            RigidBox3d::from_body(RigidBody::dynamic(
                BodyId(2),
                Vec3i::new(30, 0, 0),
                Vec3i::ZERO,
                Vec3i::new(5, 5, 5),
            )),
            RigidBox3d::from_body(RigidBody::fixed(
                BodyId(3),
                Vec3i::new(60, 0, 0),
                Vec3i::new(5, 5, 5),
            )),
        ];
        let current = RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 0, 1);
        let mut partial = RotatingBroadPhase3d::default();
        partial.candidate_pairs(&boxes, current).expect("initial sync");

        boxes[0].body.position = Vec3i::new(25, 0, 0);
        let partial_pairs = partial
            .candidate_pairs_for_changed_current_bodies([&boxes[0]])
            .expect("partial query");

        let mut full = RotatingBroadPhase3d::default();
        let full_pairs = full.candidate_pairs(&boxes, current).expect("full query");
        let expected = full_pairs
            .into_iter()
            .filter(|pair| pair.left == BodyId(1) || pair.right == BodyId(1))
            .collect::<Vec<_>>();
        assert_eq!(partial_pairs, expected);
        assert_eq!(partial.stats().partial_queries, 1);
        assert_eq!(partial.stats().partial_body_updates, 1);
    }
}
''')

print("necessary-work slice applied")
