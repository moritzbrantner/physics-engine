from pathlib import Path

response = Path("src/rotating_contact_response.rs")
text = response.read_text()
marker = "\n#[cfg(test)]\nmod scratch_reuse_tests {"
if marker not in text:
    raise RuntimeError("scratch test marker missing")
text = text.split(marker, 1)[0]
text += r'''

#[cfg(test)]
mod scratch_reuse_tests {
    use crate::{
        AngularState3d, AngularVelocity3d, BodyId, Orientation3d, RigidBody, RigidBox3d,
        RotatingContactFrontier3d, RotatingContactSearchHit3d, SampledContactTime3d, Vec3i,
        obb_contact_seed,
    };

    use super::{
        RotatingContactResponseScratch3d, resolve_rotating_contact_frontier,
        resolve_rotating_contact_frontier_with_scratch,
    };

    fn box3d(body: RigidBody) -> RigidBox3d {
        RigidBox3d::new(
            body,
            AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
        )
        .expect("valid box")
    }

    #[test]
    fn reusable_scratch_preserves_response_semantics() {
        let left = box3d(RigidBody::dynamic(
            BodyId(1),
            Vec3i::ZERO,
            Vec3i::new(10, 0, 0),
            Vec3i::new(5, 5, 5),
        ));
        let right = box3d(RigidBody::fixed(
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
'''
response.write_text(text)

broad = Path("src/rotating_broad_phase.rs")
text = broad.read_text()
marker = "\n#[cfg(test)]\nmod necessary_work_tests {"
if marker not in text:
    raise RuntimeError("broad-phase test marker missing")
text = text.split(marker, 1)[0]
text += r'''

#[cfg(test)]
mod necessary_work_tests {
    use crate::{
        AngularState3d, AngularVelocity3d, BodyId, Orientation3d, RigidBody, RigidBox3d,
        RigidBoxFreeFlightConfig3d, Vec3i,
    };

    use super::RotatingBroadPhase3d;

    fn box3d(body: RigidBody) -> RigidBox3d {
        RigidBox3d::new(
            body,
            AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
        )
        .expect("valid box")
    }

    #[test]
    fn partial_current_query_updates_only_supplied_bodies_and_matches_full_truth() {
        let mut boxes = vec![
            box3d(RigidBody::dynamic(
                BodyId(1),
                Vec3i::new(0, 0, 0),
                Vec3i::ZERO,
                Vec3i::new(5, 5, 5),
            )),
            box3d(RigidBody::dynamic(
                BodyId(2),
                Vec3i::new(30, 0, 0),
                Vec3i::ZERO,
                Vec3i::new(5, 5, 5),
            )),
            box3d(RigidBody::fixed(
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
'''
broad.write_text(text)
