use std::collections::BTreeMap;

use crate::{
    BodyId, ObbContactSeed3d, RigidBox3d, RigidBoxFreeFlightConfig3d, RotatingBroadPhaseError3d,
    RotatingWorldError3d, Vec3i, obb_contact_seed, rotational_sweep_candidate_pairs,
};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct BodyCurrentContact3d {
    pub other: BodyId,
    pub contact: ObbContactSeed3d,
}

/// Returns exact current OBB contacts for one existing body after pruning through the engine's zero-time
/// rotational broad phase.
///
/// The broad phase is the same deterministic BVH used by rotating collision discovery. Its candidates are
/// only pruning evidence; every returned neighbor is exact-filtered with `obb_contact_seed`, with `body`
/// deliberately supplied as the left OBB so the contact axis consistently points from the subject toward
/// the neighbor. No second spatial index or contact semantics are introduced.
pub(crate) fn body_current_contacts(
    boxes: &[RigidBox3d],
    body: BodyId,
) -> Result<Vec<BodyCurrentContact3d>, RotatingWorldError3d> {
    let by_id = boxes
        .iter()
        .map(|rigid_box| (rigid_box.body().id(), rigid_box))
        .collect::<BTreeMap<_, _>>();
    let subject = by_id
        .get(&body)
        .copied()
        .ok_or(RotatingWorldError3d::MissingBody(body))?;
    let candidates =
        rotational_sweep_candidate_pairs(boxes, RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 0, 1))
            .map_err(map_broad_phase_error)?;

    let mut contacts = Vec::new();
    for pair in candidates {
        let other = if pair.left == body {
            pair.right
        } else if pair.right == body {
            pair.left
        } else {
            continue;
        };
        let other_box = by_id
            .get(&other)
            .copied()
            .ok_or(RotatingWorldError3d::MissingBody(other))?;
        let Some(contact) = obb_contact_seed(subject.oriented_box(), other_box.oriented_box())?
        else {
            continue;
        };
        contacts.push(BodyCurrentContact3d { other, contact });
    }
    contacts.sort_by_key(|contact| contact.other);
    Ok(contacts)
}

fn map_broad_phase_error(error: RotatingBroadPhaseError3d) -> RotatingWorldError3d {
    match error {
        RotatingBroadPhaseError3d::DuplicateBodyId(id) => RotatingWorldError3d::DuplicateBody(id),
        RotatingBroadPhaseError3d::FreeFlight(error) => RotatingWorldError3d::FreeFlight(error),
    }
}

#[cfg(test)]
mod tests {
    use crate::{
        AngularState3d, AngularVelocity3d, BodyId, Orientation3d, RigidBody, RigidBox3d, Vec3i,
        obb_contact_seed,
    };

    use super::body_current_contacts;

    fn rotating(body: RigidBody) -> RigidBox3d {
        RigidBox3d::new(
            body,
            AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
        )
        .expect("valid box")
    }

    #[test]
    fn broad_phase_body_query_matches_exact_scan_and_stable_order() {
        let subject = rotating(RigidBody::dynamic(
            BodyId(50),
            Vec3i::ZERO,
            Vec3i::ZERO,
            Vec3i::new(2, 2, 2),
        ));
        let boxes = vec![
            rotating(RigidBody::fixed(
                BodyId(2),
                Vec3i::new(0, -4, 0),
                Vec3i::new(2, 2, 2),
            )),
            rotating(RigidBody::fixed(
                BodyId(90),
                Vec3i::new(4, 0, 0),
                Vec3i::new(2, 2, 2),
            )),
            rotating(RigidBody::fixed(
                BodyId(7),
                Vec3i::new(40, 0, 0),
                Vec3i::new(2, 2, 2),
            )),
            subject.clone(),
        ];

        let mut expected = boxes
            .iter()
            .filter(|candidate| candidate.body().id() != subject.body().id())
            .filter_map(|candidate| {
                obb_contact_seed(subject.oriented_box(), candidate.oriented_box())
                    .expect("valid exact query")
                    .map(|contact| (candidate.body().id(), contact))
            })
            .collect::<Vec<_>>();
        expected.sort_by_key(|(id, _)| *id);

        let actual =
            body_current_contacts(&boxes, subject.body().id()).expect("broad-phase contacts");
        assert_eq!(
            actual
                .iter()
                .map(|entry| (entry.other, entry.contact))
                .collect::<Vec<_>>(),
            expected
        );
    }
}
