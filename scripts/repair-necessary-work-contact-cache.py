from pathlib import Path
import re

path = Path("src/repeated_rotating_events.rs")
text = path.read_text()

old_import = "use std::{error::Error, fmt};\n"
new_import = "use std::{collections::{BTreeMap, BTreeSet}, error::Error, fmt};\n"
if text.count(old_import) != 1:
    raise RuntimeError("expected repeated-event import shape not found")
text = text.replace(old_import, new_import, 1)

old_call = '''        state = stabilize_current_contacts(
            response.boxes,
            config.solver_passes,
            broad_phase,
            &modified_body_ids,
            &mut response_scratch,
            &mut work,
        )?;
'''
new_call = '''        state = stabilize_current_contacts(
            response.boxes,
            config.solver_passes,
            broad_phase,
            &modified_body_ids,
            &response_contacts,
            &mut response_scratch,
            &mut work,
        )?;
'''
if text.count(old_call) != 1:
    raise RuntimeError("expected stabilization call not found")
text = text.replace(old_call, new_call, 1)

pattern = re.compile(
    r"fn stabilize_current_contacts\(.*?\n\}\n\nfn validate_config\(",
    flags=re.S,
)
replacement = r'''fn stabilize_current_contacts(
    mut boxes: Vec<RigidBox3d>,
    solver_passes: u8,
    broad_phase: &mut RotatingBroadPhase3d,
    initial_active: &[crate::BodyId],
    initial_contacts: &[RotatingContactSearchHit3d],
    response_scratch: &mut RotatingContactResponseScratch3d,
    work: &mut RepeatedRotatingEventWorkStats3d,
) -> Result<Vec<RigidBox3d>, RepeatedRotatingEventError3d> {
    let mut active = initial_active.to_vec();
    active.sort_unstable();
    active.dedup();
    let mut contacts = initial_contacts
        .iter()
        .cloned()
        .map(|mut contact| {
            contact.time = SampledContactTime3d::ZERO;
            (contact.pair, contact)
        })
        .collect::<BTreeMap<_, _>>();
    let mut exhausted_with_changes = false;

    for pass in 0..solver_passes {
        if active.is_empty() {
            break;
        }
        work.stabilization_active_bodies = work
            .stabilization_active_bodies
            .saturating_add(u64::try_from(active.len()).unwrap_or(u64::MAX));
        let current = refresh_current_contacts_for_changed_bodies(
            &boxes,
            &active,
            &mut contacts,
            broad_phase,
        )?;
        work.stabilization_candidate_pairs = work
            .stabilization_candidate_pairs
            .saturating_add(u64::try_from(current.candidate_pairs).unwrap_or(u64::MAX));
        work.stabilization_exact_contacts = work
            .stabilization_exact_contacts
            .saturating_add(u64::try_from(current.recomputed_contacts).unwrap_or(u64::MAX));
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
    recomputed_contacts: usize,
}

/// Refreshes only exact contact edges whose geometry can have changed.
///
/// Contacts whose two endpoints are unchanged are retained as authoritative exact evidence. Every cached
/// edge touching an active body is discarded and revalidated from current geometry, while the targeted
/// broad phase discovers any newly-created neighbors of those active bodies. The resulting frontier still
/// contains *all* current contacts, preserving the simultaneous solver semantics of a full-world refresh
/// without recomputing unchanged edges.
fn refresh_current_contacts_for_changed_bodies(
    boxes: &[RigidBox3d],
    active: &[crate::BodyId],
    contacts: &mut BTreeMap<crate::RotationalSweepPair3d, RotatingContactSearchHit3d>,
    broad_phase: &mut RotatingBroadPhase3d,
) -> Result<CurrentContactFrontierResult3d, RotatingContactFrontierError3d> {
    let active_set = active.iter().copied().collect::<BTreeSet<_>>();
    contacts.retain(|pair, _| {
        !active_set.contains(&pair.left) && !active_set.contains(&pair.right)
    });

    let mut active_boxes = Vec::with_capacity(active.len());
    for id in active {
        let rigid_box = boxes
            .iter()
            .find(|rigid_box| rigid_box.body().id() == *id)
            .ok_or(RotatingContactFrontierError3d::MissingBody(*id))?;
        active_boxes.push(rigid_box);
    }
    let candidates = broad_phase.candidate_pairs_for_changed_current_bodies(active_boxes)?;
    let candidate_pairs = candidates.len();
    let mut recomputed_contacts = 0_usize;

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
        recomputed_contacts = recomputed_contacts.saturating_add(1);
        contacts.insert(
            pair,
            RotatingContactSearchHit3d {
                time: SampledContactTime3d::ZERO,
                pair,
                contact,
            },
        );
    }

    let frontier = if contacts.is_empty() {
        None
    } else {
        Some(RotatingContactFrontier3d {
            boxes: boxes.to_vec(),
            time: SampledContactTime3d::ZERO,
            contacts: contacts.values().cloned().collect(),
            remaining_numerator: 1,
        })
    };
    Ok(CurrentContactFrontierResult3d {
        frontier,
        candidate_pairs,
        recomputed_contacts,
    })
}

#[cfg(test)]
fn current_contact_frontier(
    boxes: &[RigidBox3d],
) -> Result<Option<RotatingContactFrontier3d>, RotatingContactFrontierError3d> {
    let mut broad_phase = RotatingBroadPhase3d::default();
    let zero_time = RigidBoxFreeFlightConfig3d::new(crate::Vec3i::ZERO, 0, 1);
    broad_phase.candidate_pairs(boxes, zero_time)?;
    let active = boxes
        .iter()
        .map(|rigid_box| rigid_box.body().id())
        .collect::<Vec<_>>();
    let mut contacts = BTreeMap::new();
    Ok(refresh_current_contacts_for_changed_bodies(
        boxes,
        &active,
        &mut contacts,
        &mut broad_phase,
    )?
    .frontier)
}

fn validate_config('''

text, count = pattern.subn(replacement, text, count=1)
if count != 1:
    raise RuntimeError(f"expected one stabilization region, found {count}")
path.write_text(text)
