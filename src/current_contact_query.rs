use std::collections::{BTreeMap, BTreeSet};

use crate::{
    BodyId, BodyKind, RigidBox3d, RigidBoxFreeFlightConfig3d, RotatingBroadPhaseError3d,
    RotatingWorldError3d, SolverParticipation3d, Vec3i, obb_contact_seed,
    rigid_box_free_flight_sweep_bounds,
    rotating_broad_phase::RotatingBroadPhase3d,
};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct BodyCurrentContact3d {
    pub other: BodyId,
    /// Exact SAT contact axis oriented from the queried subject toward `other`.
    pub axis: [i128; 3],
}

#[derive(Clone, Debug, Eq, PartialEq)]
struct CurrentContactGraph3d {
    contacts: BTreeMap<BodyId, Vec<BodyCurrentContact3d>>,
    candidate_pairs: usize,
    exact_pair_tests: usize,
}

#[derive(Clone, Debug, Default)]
pub(crate) struct GenerationContactCache3d {
    generation: Option<u64>,
    graph: Option<CurrentContactGraph3d>,
    body_contacts: BTreeMap<BodyId, Vec<BodyCurrentContact3d>>,
    broad_phase: RotatingBroadPhase3d,
    pending_changed: BTreeSet<BodyId>,
    graph_builds: u64,
    subject_builds: u64,
    candidate_pairs: u64,
    exact_pair_tests: u64,
    incremental_refreshes: u64,
    incremental_candidate_pairs: u64,
    incremental_exact_tests: u64,
}

impl GenerationContactCache3d {
    fn activate_generation(&mut self, generation: u64) {
        if self.generation == Some(generation) {
            return;
        }
        self.generation = Some(generation);
        self.graph = None;
        self.body_contacts.clear();
        self.pending_changed.clear();
        self.broad_phase = RotatingBroadPhase3d::default();
    }

    pub(crate) fn note_membership_generation(&mut self, generation: u64) {
        self.generation = Some(generation);
        self.graph = None;
        self.body_contacts.clear();
        self.pending_changed.clear();
        self.broad_phase = RotatingBroadPhase3d::default();
    }

    pub(crate) fn note_changed_generation(
        &mut self,
        generation: u64,
        changed: impl IntoIterator<Item = BodyId>,
    ) {
        if self.generation.is_none() {
            self.generation = Some(generation);
            return;
        }
        self.generation = Some(generation);
        self.pending_changed.extend(changed);
        self.body_contacts.clear();
    }

    fn refresh_pending_graph(
        &mut self,
        boxes: &BTreeMap<BodyId, RigidBox3d>,
    ) -> Result<(), RotatingWorldError3d> {
        if self.pending_changed.is_empty() || self.graph.is_none() {
            return Ok(());
        }
        let changed = std::mem::take(&mut self.pending_changed);
        let graph = self.graph.as_mut().expect("checked contact graph");
        let mut touched = changed.clone();

        for id in &changed {
            if let Some(previous) = graph.contacts.remove(id) {
                for contact in previous {
                    if let Some(neighbors) = graph.contacts.get_mut(&contact.other) {
                        neighbors.retain(|neighbor| neighbor.other != *id);
                        touched.insert(contact.other);
                    }
                }
            }
            graph.contacts.entry(*id).or_default();
        }

        let mut changed_boxes = Vec::with_capacity(changed.len());
        for id in &changed {
            changed_boxes.push(
                boxes
                    .get(id)
                    .ok_or(RotatingWorldError3d::MissingBody(*id))?,
            );
        }
        let candidates = self
            .broad_phase
            .candidate_pairs_for_changed_current_bodies(changed_boxes)
            .map_err(map_broad_phase_error)?;
        self.incremental_candidate_pairs = self
            .incremental_candidate_pairs
            .saturating_add(u64::try_from(candidates.len()).unwrap_or(u64::MAX));

        for pair in candidates {
            let left = boxes
                .get(&pair.left)
                .ok_or(RotatingWorldError3d::MissingBody(pair.left))?;
            let right = boxes
                .get(&pair.right)
                .ok_or(RotatingWorldError3d::MissingBody(pair.right))?;
            self.incremental_exact_tests = self.incremental_exact_tests.saturating_add(1);
            let Some(contact) = obb_contact_seed(left.oriented_box(), right.oriented_box())? else {
                continue;
            };
            let reverse_axis = reverse_axis(contact.axis, pair.left)?;
            graph.contacts.entry(pair.left).or_default().push(BodyCurrentContact3d {
                other: pair.right,
                axis: contact.axis,
            });
            graph.contacts.entry(pair.right).or_default().push(BodyCurrentContact3d {
                other: pair.left,
                axis: reverse_axis,
            });
            touched.insert(pair.left);
            touched.insert(pair.right);
        }

        for id in touched {
            if let Some(neighbors) = graph.contacts.get_mut(&id) {
                neighbors.sort_by_key(|contact| contact.other);
                neighbors.dedup_by_key(|contact| contact.other);
            }
        }
        self.incremental_refreshes = self.incremental_refreshes.saturating_add(1);
        Ok(())
    }

    /// Returns exact current contacts for one known body.
    ///
    /// One-off callers keep the precise single-subject path. Repeated callers reuse that subject result,
    /// while a graph already built by overlap traversal becomes the shared authority for all subjects in
    /// the same geometry generation.
    pub(crate) fn body_contacts(
        &mut self,
        boxes: &BTreeMap<BodyId, RigidBox3d>,
        body: BodyId,
        generation: u64,
    ) -> Result<Vec<BodyCurrentContact3d>, RotatingWorldError3d> {
        self.activate_generation(generation);
        self.refresh_pending_graph(boxes)?;
        if let Some(graph) = &self.graph {
            return graph
                .contacts
                .get(&body)
                .cloned()
                .ok_or(RotatingWorldError3d::MissingBody(body));
        }
        if let Some(contacts) = self.body_contacts.get(&body) {
            return Ok(contacts.clone());
        }

        let contacts = body_current_contacts_for_body(boxes, body)?;
        self.subject_builds = self.subject_builds.saturating_add(1);
        self.body_contacts.insert(body, contacts.clone());
        Ok(contacts)
    }

    /// Returns the subject plus exact current rigid-contact neighbors in stable BodyId order.
    ///
    /// The first whole-world traversal for a geometry generation builds one deterministic broad-phase
    /// graph. Stable queries validate that graph with a scalar generation comparison instead of rebuilding
    /// a BodyId/kind/layer/pose fingerprint for every body.
    pub(crate) fn overlap_ids(
        &mut self,
        boxes: &BTreeMap<BodyId, RigidBox3d>,
        body: BodyId,
        generation: u64,
    ) -> Result<Vec<BodyId>, RotatingWorldError3d> {
        let subject = boxes
            .get(&body)
            .ok_or(RotatingWorldError3d::MissingBody(body))?;
        if subject.body().kind() != BodyKind::Dynamic {
            return Err(RotatingWorldError3d::MissingBody(body));
        }

        self.activate_generation(generation);
        if self.graph.is_none() {
            let snapshot = boxes.values().cloned().collect::<Vec<_>>();
            let graph = build_current_contact_graph_with_broad_phase(
                &snapshot,
                &mut self.broad_phase,
            )?;
            self.graph_builds = self.graph_builds.saturating_add(1);
            self.candidate_pairs = self
                .candidate_pairs
                .saturating_add(u64::try_from(graph.candidate_pairs).unwrap_or(u64::MAX));
            self.exact_pair_tests = self
                .exact_pair_tests
                .saturating_add(u64::try_from(graph.exact_pair_tests).unwrap_or(u64::MAX));
            self.graph = Some(graph);
            self.body_contacts.clear();
            self.pending_changed.clear();
        } else {
            self.refresh_pending_graph(boxes)?;
        }

        let contacts = self
            .graph
            .as_ref()
            .and_then(|graph| graph.contacts.get(&body))
            .cloned()
            .ok_or(RotatingWorldError3d::MissingBody(body))?;
        let mut ids = Vec::with_capacity(contacts.len().saturating_add(1));
        ids.push(body);
        ids.extend(contacts.into_iter().map(|contact| contact.other));
        ids.sort_unstable();
        ids.dedup();
        Ok(ids)
    }

    #[cfg(test)]
    pub(crate) const fn stats(&self) -> (u64, u64, u64, u64) {
        (
            self.graph_builds,
            self.subject_builds,
            self.candidate_pairs,
            self.exact_pair_tests,
        )
    }

    #[cfg(test)]
    pub(crate) const fn incremental_stats(&self) -> (u64, u64, u64) {
        (
            self.incremental_refreshes,
            self.incremental_candidate_pairs,
            self.incremental_exact_tests,
        )
    }
}

/// Computes exact current contacts for one known body without materializing the full-world contact graph.
///
/// The subject is looked up directly by `BodyId`. Every unrelated body pays only for its current conservative
/// bound; exact OBB contact work is performed only for collision-enabled bodies whose current bounds overlap
/// the subject. This is the precise single-subject counterpart to the cached full contact graph.
fn body_current_contacts_for_body(
    boxes: &BTreeMap<BodyId, RigidBox3d>,
    body: BodyId,
) -> Result<Vec<BodyCurrentContact3d>, RotatingWorldError3d> {
    let subject = boxes
        .get(&body)
        .ok_or(RotatingWorldError3d::MissingBody(body))?;
    if subject.solver_participation() == SolverParticipation3d::OverlapOnly {
        return Ok(Vec::new());
    }
    let zero_time = RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 0, 1);
    let subject_bounds = rigid_box_free_flight_sweep_bounds(subject, zero_time)?;
    let mut contacts = Vec::new();

    for (other_id, other) in boxes {
        if *other_id == body
            || other.solver_participation() == SolverParticipation3d::OverlapOnly
            || !subject
                .collision_layers()
                .collides_with(other.collision_layers())
        {
            continue;
        }
        let other_bounds = rigid_box_free_flight_sweep_bounds(other, zero_time)?;
        if !(0..3).all(|axis| {
            subject_bounds.minimum[axis] <= other_bounds.maximum[axis]
                && other_bounds.minimum[axis] <= subject_bounds.maximum[axis]
        }) {
            continue;
        }
        let Some(contact) = obb_contact_seed(subject.oriented_box(), other.oriented_box())? else {
            continue;
        };
        contacts.push(BodyCurrentContact3d {
            other: *other_id,
            axis: contact.axis,
        });
    }

    Ok(contacts)
}

fn build_current_contact_graph(
    boxes: &[RigidBox3d],
) -> Result<CurrentContactGraph3d, RotatingWorldError3d> {
    let mut broad_phase = RotatingBroadPhase3d::default();
    build_current_contact_graph_with_broad_phase(boxes, &mut broad_phase)
}

fn build_current_contact_graph_with_broad_phase(
    boxes: &[RigidBox3d],
    broad_phase: &mut RotatingBroadPhase3d,
) -> Result<CurrentContactGraph3d, RotatingWorldError3d> {
    let by_id = boxes
        .iter()
        .map(|rigid_box| (rigid_box.body().id(), rigid_box))
        .collect::<BTreeMap<_, _>>();
    let candidates = broad_phase
        .candidate_pairs(boxes, RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 0, 1))
        .map_err(map_broad_phase_error)?;
    let candidate_pairs = candidates.len();
    let mut exact_pair_tests = 0_usize;
    let mut contacts = by_id
        .keys()
        .copied()
        .map(|id| (id, Vec::new()))
        .collect::<BTreeMap<_, _>>();

    for pair in candidates {
        let left = by_id
            .get(&pair.left)
            .copied()
            .ok_or(RotatingWorldError3d::MissingBody(pair.left))?;
        let right = by_id
            .get(&pair.right)
            .copied()
            .ok_or(RotatingWorldError3d::MissingBody(pair.right))?;
        exact_pair_tests = exact_pair_tests.saturating_add(1);
        let Some(contact) = obb_contact_seed(left.oriented_box(), right.oriented_box())? else {
            continue;
        };
        let reverse_axis = reverse_axis(contact.axis, pair.left)?;
        contacts
            .get_mut(&pair.left)
            .ok_or(RotatingWorldError3d::MissingBody(pair.left))?
            .push(BodyCurrentContact3d {
                other: pair.right,
                axis: contact.axis,
            });
        contacts
            .get_mut(&pair.right)
            .ok_or(RotatingWorldError3d::MissingBody(pair.right))?
            .push(BodyCurrentContact3d {
                other: pair.left,
                axis: reverse_axis,
            });
    }

    for neighbors in contacts.values_mut() {
        neighbors.sort_by_key(|contact| contact.other);
    }

    Ok(CurrentContactGraph3d {
        contacts,
        candidate_pairs,
        exact_pair_tests,
    })
}

fn reverse_axis(axis: [i128; 3], body: BodyId) -> Result<[i128; 3], RotatingWorldError3d> {
    Ok([
        axis[0]
            .checked_neg()
            .ok_or_else(|| contact_overflow(body))?,
        axis[1]
            .checked_neg()
            .ok_or_else(|| contact_overflow(body))?,
        axis[2]
            .checked_neg()
            .ok_or_else(|| contact_overflow(body))?,
    ])
}

fn contact_overflow(body: BodyId) -> RotatingWorldError3d {
    RotatingWorldError3d::PersistentTailArithmeticOverflow(body)
}

fn map_broad_phase_error(error: RotatingBroadPhaseError3d) -> RotatingWorldError3d {
    match error {
        RotatingBroadPhaseError3d::DuplicateBodyId(id) => RotatingWorldError3d::DuplicateBody(id),
        RotatingBroadPhaseError3d::IncrementalQueryUnsynchronized(id) => {
            RotatingWorldError3d::MissingBody(id)
        }
        RotatingBroadPhaseError3d::FreeFlight(error) => RotatingWorldError3d::FreeFlight(error),
    }
}

#[cfg(test)]
mod tests {
    use std::{collections::BTreeMap, hint::black_box, time::Instant};

    use crate::{
        AngularState3d, AngularVelocity3d, BodyId, ORIENTATION_SCALE, Orientation3d, RigidBody,
        RigidBox3d, RotatingWorldConfig3d, Vec3i, obb_contact_seed,
        rotating_world::RotatingWorld3d,
    };

    use super::{GenerationContactCache3d, build_current_contact_graph};

    fn rotating(body: RigidBody) -> RigidBox3d {
        RigidBox3d::new(
            body,
            AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
        )
        .expect("valid box")
    }

    fn rotated(body: RigidBody, index: i32) -> RigidBox3d {
        let orientation = Orientation3d::new(
            ORIENTATION_SCALE / (index + 3),
            ORIENTATION_SCALE / (index + 5),
            -ORIENTATION_SCALE / (index + 7),
            ORIENTATION_SCALE,
        )
        .normalized()
        .expect("valid deterministic orientation");
        RigidBox3d::new(
            body,
            AngularState3d::new(orientation, AngularVelocity3d::default()),
        )
        .expect("valid rotated box")
    }

    fn assert_dynamic_graph_matches_exact_oracle(boxes: &[RigidBox3d]) {
        let graph = build_current_contact_graph(boxes).expect("current-contact graph");
        for subject in boxes
            .iter()
            .filter(|rigid_box| rigid_box.body().kind() == crate::BodyKind::Dynamic)
        {
            let mut expected = boxes
                .iter()
                .filter(|candidate| candidate.body().id() != subject.body().id())
                .filter_map(|candidate| {
                    obb_contact_seed(subject.oriented_box(), candidate.oriented_box())
                        .expect("valid exact query")
                        .map(|contact| (candidate.body().id(), contact.axis))
                })
                .collect::<Vec<_>>();
            expected.sort_by_key(|(id, _)| *id);
            let actual = graph
                .contacts
                .get(&subject.body().id())
                .expect("dynamic graph entry")
                .iter()
                .map(|entry| (entry.other, entry.axis))
                .collect::<Vec<_>>();
            assert_eq!(actual, expected);
        }
    }

    #[test]
    fn current_contact_graph_matches_sparse_stacked_and_rotated_oracles() {
        let sparse = (0..24_u64)
            .map(|index| {
                rotating(RigidBody::dynamic(
                    BodyId(index + 1),
                    Vec3i::new(i32::try_from(index).expect("small index") * 20, 0, 0),
                    Vec3i::ZERO,
                    Vec3i::new(2, 2, 2),
                ))
            })
            .collect::<Vec<_>>();
        assert_dynamic_graph_matches_exact_oracle(&sparse);

        let mut stacked = vec![rotating(RigidBody::fixed(
            BodyId(500),
            Vec3i::new(0, -2, 0),
            Vec3i::new(20, 2, 20),
        ))];
        stacked.extend((0..12_u64).map(|index| {
            rotating(RigidBody::dynamic(
                BodyId(index + 1),
                Vec3i::new(0, 2 + i32::try_from(index).expect("small index") * 4, 0),
                Vec3i::ZERO,
                Vec3i::new(2, 2, 2),
            ))
        }));
        assert_dynamic_graph_matches_exact_oracle(&stacked);

        let rotated = (0..18_u64)
            .map(|index| {
                rotated(
                    RigidBody::dynamic(
                        BodyId(index + 1),
                        Vec3i::new(
                            i32::try_from(index % 6).expect("small x") * 4,
                            i32::try_from(index / 6).expect("small y") * 4,
                            0,
                        ),
                        Vec3i::ZERO,
                        Vec3i::new(2, 2, 2),
                    ),
                    i32::try_from(index % 5).expect("small orientation index") + 1,
                )
            })
            .collect::<Vec<_>>();
        assert_dynamic_graph_matches_exact_oracle(&rotated);
    }

    #[test]
    fn repeated_body_queries_reuse_graph_until_geometry_changes() {
        let mut world = RotatingWorld3d::new(RotatingWorldConfig3d {
            gravity: Vec3i::ZERO,
            ..RotatingWorldConfig3d::default()
        });
        for index in 0..16_u64 {
            world
                .add_box(rotating(RigidBody::dynamic(
                    BodyId(index + 1),
                    Vec3i::new(i32::try_from(index).expect("small index") * 4, 0, 0),
                    Vec3i::ZERO,
                    Vec3i::new(2, 2, 2),
                )))
                .expect("dynamic body");
        }

        for rigid_box in world.boxes() {
            world
                .overlap_query(rigid_box.oriented_box())
                .expect("cached body overlap");
        }
        assert_eq!(
            world.current_contact_cache_stats().0,
            1,
            "one stable generation should build one graph"
        );

        world
            .set_linear_velocity(BodyId(1), Vec3i::new(17, 0, 0))
            .expect("velocity update");
        let first = world.box_by_id(BodyId(1)).expect("body one").oriented_box();
        world
            .overlap_query(first)
            .expect("velocity-only cache reuse");
        assert_eq!(
            world.current_contact_cache_stats().0,
            1,
            "velocity alone must not advance contact geometry"
        );

        let mut moved = world.remove_box(BodyId(1)).expect("body one");
        moved.body.position.x = moved.body.position.x.saturating_add(1);
        world.add_box(moved).expect("moved body");
        let moved_query = world
            .box_by_id(BodyId(1))
            .expect("moved body")
            .oriented_box();
        world
            .overlap_query(moved_query)
            .expect("moved geometry query");
        assert_eq!(
            world.current_contact_cache_stats().0,
            2,
            "pose change must refresh the graph"
        );
    }

    #[test]
    fn repeated_precise_subject_query_reuses_generation_until_pose_changes() {
        let mut world = RotatingWorld3d::new(RotatingWorldConfig3d {
            gravity: Vec3i::ZERO,
            ..RotatingWorldConfig3d::default()
        });
        world
            .add_box(rotating(RigidBody::dynamic(
                BodyId(1),
                Vec3i::ZERO,
                Vec3i::ZERO,
                Vec3i::new(2, 2, 2),
            )))
            .expect("subject");
        world
            .add_box(rotating(RigidBody::fixed(
                BodyId(2),
                Vec3i::new(4, 0, 0),
                Vec3i::new(2, 2, 2),
            )))
            .expect("neighbor");

        world.body_contacts(BodyId(1)).expect("first contact query");
        world
            .body_contacts(BodyId(1))
            .expect("cached contact query");
        assert_eq!(
            world.current_contact_cache_stats().1,
            1,
            "stable generation should compute one precise subject query"
        );

        world
            .set_linear_velocity(BodyId(1), Vec3i::new(7, 0, 0))
            .expect("velocity-only change");
        world
            .body_contacts(BodyId(1))
            .expect("velocity-only cached query");
        assert_eq!(
            world.current_contact_cache_stats().1,
            1,
            "velocity alone must preserve the contact generation"
        );

        let mut moved = world.remove_box(BodyId(1)).expect("subject");
        moved.body.position.x = moved.body.position.x.saturating_add(1);
        world.add_box(moved).expect("moved subject");
        world
            .body_contacts(BodyId(1))
            .expect("geometry-changed query");
        assert_eq!(
            world.current_contact_cache_stats().1,
            2,
            "pose change must recompute the subject contact neighborhood"
        );
    }

    #[test]
    fn body_current_contacts_keep_stable_neighbor_order() {
        let subject = rotating(RigidBody::dynamic(
            BodyId(50),
            Vec3i::ZERO,
            Vec3i::ZERO,
            Vec3i::new(2, 2, 2),
        ));
        let boxes = vec![
            rotating(RigidBody::fixed(
                BodyId(90),
                Vec3i::new(4, 0, 0),
                Vec3i::new(2, 2, 2),
            )),
            rotating(RigidBody::fixed(
                BodyId(2),
                Vec3i::new(0, -4, 0),
                Vec3i::new(2, 2, 2),
            )),
            subject.clone(),
        ];
        let by_id = boxes
            .into_iter()
            .map(|rigid_box| (rigid_box.body().id(), rigid_box))
            .collect::<BTreeMap<_, _>>();
        let mut cache = GenerationContactCache3d::default();
        let actual = cache
            .body_contacts(&by_id, subject.body().id(), 1)
            .expect("body contacts");
        assert_eq!(
            actual.iter().map(|entry| entry.other).collect::<Vec<_>>(),
            vec![BodyId(2), BodyId(90)]
        );
    }

    #[test]
    #[ignore = "release-mode deterministic performance evidence; run explicitly with --ignored --nocapture"]
    fn sleep_style_contact_graph_benchmark() {
        let mut world = RotatingWorld3d::new(RotatingWorldConfig3d {
            gravity: Vec3i::ZERO,
            ..RotatingWorldConfig3d::default()
        });
        world
            .add_box(rotating(RigidBody::fixed(
                BodyId(10_000),
                Vec3i::new(0, -2, 0),
                Vec3i::new(80, 2, 80),
            )))
            .expect("floor");
        for index in 0..128_u64 {
            let column = i32::try_from(index % 16).expect("small column");
            let row = i32::try_from(index / 16).expect("small row");
            world
                .add_box(rotating(RigidBody::dynamic(
                    BodyId(index + 1),
                    Vec3i::new(column * 8, 2 + row * 4, 0),
                    Vec3i::ZERO,
                    Vec3i::new(2, 2, 2),
                )))
                .expect("pile body");
        }

        let queries = world
            .boxes()
            .filter(|rigid_box| rigid_box.body().kind() == crate::BodyKind::Dynamic)
            .map(RigidBox3d::oriented_box)
            .collect::<Vec<_>>();
        let legacy_exact_tests = queries.len().saturating_mul(world.boxes().count());

        let started = Instant::now();
        for query in &queries {
            black_box(
                world
                    .overlap_query(black_box(*query))
                    .expect("body overlap"),
            );
        }
        let elapsed = started.elapsed();
        let (builds, subject_builds, candidate_pairs, exact_pair_tests) =
            world.current_contact_cache_stats();

        assert_eq!(
            builds, 1,
            "stable sleep-style traversal should build one graph"
        );
        assert_eq!(
            subject_builds, 0,
            "whole-world traversal should not pay precise subject scans"
        );
        assert!(
            usize::try_from(exact_pair_tests).unwrap_or(usize::MAX) * 4 < legacy_exact_tests,
            "broad-phase graph did not materially reduce exact OBB evaluations: graph={exact_pair_tests}, legacy={legacy_exact_tests}"
        );
        eprintln!(
            "sleep-style current contacts: bodies={}, queries={}, legacy_exact_tests={}, graph_builds={}, candidate_pairs={}, exact_pair_tests={}, elapsed={elapsed:?}",
            world.boxes().count(),
            queries.len(),
            legacy_exact_tests,
            builds,
            candidate_pairs,
            exact_pair_tests,
        );
    }
}
