use std::collections::{BTreeMap, BTreeSet};

use crate::{
    BodyId, BodyKind, RigidBox3d, RigidBoxFreeFlightConfig3d, RotatingBroadPhaseError3d,
    RotatingWorldError3d, SolverParticipation3d, Vec3i, obb_contact_seed,
    rotating_broad_phase::{RotatingBoundsIndex3d, RotatingBroadPhase3d},
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
    subject_bounds: Option<RotatingBoundsIndex3d>,
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
        self.subject_bounds = None;
        self.pending_changed.clear();
        self.broad_phase = RotatingBroadPhase3d::default();
    }

    pub(crate) fn note_membership_generation(&mut self, generation: u64) {
        self.generation = Some(generation);
        self.graph = None;
        self.body_contacts.clear();
        self.subject_bounds = None;
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
    }

    fn refresh_pending_subjects(
        &mut self,
        boxes: &BTreeMap<BodyId, RigidBox3d>,
    ) -> Result<(), RotatingWorldError3d> {
        let Some(index) = &mut self.subject_bounds else {
            return Ok(());
        };
        for id in &self.pending_changed {
            self.body_contacts.remove(id);
            if let Some(old_bounds) = index.bounds(*id) {
                for neighbor in index.overlapping_ids(old_bounds).body_ids {
                    self.body_contacts.remove(&neighbor);
                }
            }
            let body = boxes
                .get(id)
                .ok_or(RotatingWorldError3d::MissingBody(*id))?;
            index
                .insert_stationary(body)
                .map_err(map_broad_phase_error)?;
            if let Some(new_bounds) = index.bounds(*id) {
                for neighbor in index.overlapping_ids(new_bounds).body_ids {
                    self.body_contacts.remove(&neighbor);
                }
            }
        }
        Ok(())
    }

    fn refresh_pending(
        &mut self,
        boxes: &BTreeMap<BodyId, RigidBox3d>,
    ) -> Result<(), RotatingWorldError3d> {
        self.refresh_pending_subjects(boxes)?;
        self.refresh_pending_graph(boxes)?;
        // Both query paths must consume the changes before discarding invalidation evidence.
        // Retain pending IDs on error so a partially refreshed index cannot serve stale results.
        self.pending_changed.clear();
        Ok(())
    }

    fn refresh_pending_graph(
        &mut self,
        boxes: &BTreeMap<BodyId, RigidBox3d>,
    ) -> Result<(), RotatingWorldError3d> {
        if self.pending_changed.is_empty() || self.graph.is_none() {
            return Ok(());
        }
        let changed = self.pending_changed.clone();
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
            .candidate_pairs_for_changed_current_query_bodies(changed_boxes)
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
            graph
                .contacts
                .entry(pair.left)
                .or_default()
                .push(BodyCurrentContact3d {
                    other: pair.right,
                    axis: contact.axis,
                });
            graph
                .contacts
                .entry(pair.right)
                .or_default()
                .push(BodyCurrentContact3d {
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
    /// through a retained stationary bounds index. Geometry changes invalidate only subjects near the
    /// old or new bounds, including cached negative results. A graph already built by overlap traversal
    /// becomes the shared authority for dynamic subjects. Fixed subjects keep the complete indexed path.
    pub(crate) fn body_contacts(
        &mut self,
        boxes: &BTreeMap<BodyId, RigidBox3d>,
        body: BodyId,
        generation: u64,
    ) -> Result<Vec<BodyCurrentContact3d>, RotatingWorldError3d> {
        let subject = boxes
            .get(&body)
            .ok_or(RotatingWorldError3d::MissingBody(body))?;
        self.activate_generation(generation);
        self.refresh_pending(boxes)?;
        // The graph deliberately omits fixed/fixed pairs; parked proxies still need those contacts
        // when queried explicitly. Only dynamic subjects have a complete neighborhood in the graph.
        if let Some(graph) = &self.graph
            && subject.body().kind() == BodyKind::Dynamic
        {
            return graph
                .contacts
                .get(&body)
                .cloned()
                .ok_or(RotatingWorldError3d::MissingBody(body));
        }
        if self.subject_bounds.is_none() {
            let mut index = RotatingBoundsIndex3d::default();
            index
                .rebuild_stationary(boxes.values())
                .map_err(map_broad_phase_error)?;
            self.subject_bounds = Some(index);
            self.pending_changed.clear();
        }
        if let Some(contacts) = self.body_contacts.get(&body) {
            return Ok(contacts.clone());
        }

        let contacts = body_current_contacts_for_body(
            boxes,
            body,
            self.subject_bounds
                .as_ref()
                .expect("prepared subject index"),
        )?;
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
            let graph = build_current_contact_graph_with_broad_phase(
                boxes.values(),
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
            self.subject_bounds = None;
            self.pending_changed.clear();
        } else {
            self.refresh_pending(boxes)?;
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
    index: &RotatingBoundsIndex3d,
) -> Result<Vec<BodyCurrentContact3d>, RotatingWorldError3d> {
    let subject = boxes
        .get(&body)
        .ok_or(RotatingWorldError3d::MissingBody(body))?;
    if subject.solver_participation() == SolverParticipation3d::OverlapOnly {
        return Ok(Vec::new());
    }
    let subject_bounds = index
        .bounds(body)
        .ok_or(RotatingWorldError3d::MissingBody(body))?;
    let mut contacts = Vec::new();

    for other_id in index.overlapping_ids(subject_bounds).body_ids {
        let other = boxes
            .get(&other_id)
            .ok_or(RotatingWorldError3d::MissingBody(other_id))?;
        if other_id == body
            || other.solver_participation() == SolverParticipation3d::OverlapOnly
            || !subject
                .collision_layers()
                .collides_with(other.collision_layers())
        {
            continue;
        }
        let Some(contact) = obb_contact_seed(subject.oriented_box(), other.oriented_box())? else {
            continue;
        };
        contacts.push(BodyCurrentContact3d {
            other: other_id,
            axis: contact.axis,
        });
    }

    Ok(contacts)
}

#[cfg(test)]
fn build_current_contact_graph(
    boxes: &[RigidBox3d],
) -> Result<CurrentContactGraph3d, RotatingWorldError3d> {
    let mut broad_phase = RotatingBroadPhase3d::default();
    build_current_contact_graph_with_broad_phase(boxes.iter(), &mut broad_phase)
}

fn build_current_contact_graph_with_broad_phase<'a>(
    boxes: impl Iterator<Item = &'a RigidBox3d> + Clone,
    broad_phase: &mut RotatingBroadPhase3d,
) -> Result<CurrentContactGraph3d, RotatingWorldError3d> {
    let by_id = boxes
        .clone()
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
    fn active_step_refreshes_only_changed_contact_adjacency() {
        let mut world = RotatingWorld3d::new(RotatingWorldConfig3d {
            gravity: Vec3i::ZERO,
            ..RotatingWorldConfig3d::default()
        });
        for index in 0..64_u64 {
            world
                .add_box(rotating(RigidBody::dynamic(
                    BodyId(index + 1),
                    Vec3i::new(i32::try_from(index).expect("small index") * 20, 0, 0),
                    Vec3i::ZERO,
                    Vec3i::new(2, 2, 2),
                )))
                .expect("dynamic body");
        }

        let first_query = world.box_by_id(BodyId(1)).expect("body one").oriented_box();
        world.overlap_query(first_query).expect("initial graph");
        assert_eq!(world.current_contact_cache_stats().0, 1);

        world
            .set_linear_velocity(BodyId(1), Vec3i::new(60, 0, 0))
            .expect("move one body");
        let report = world.step(1, 60).expect("sparse movement step");
        assert_eq!(report.changed_body_ids, vec![BodyId(1)]);

        let moved_query = world
            .box_by_id(BodyId(1))
            .expect("moved body")
            .oriented_box();
        world
            .overlap_query(moved_query)
            .expect("incrementally refreshed graph");

        assert_eq!(
            world.current_contact_cache_stats().0,
            1,
            "local pose changes must not rebuild the full graph"
        );
        assert_eq!(
            world.current_contact_incremental_stats().0,
            1,
            "one changed-body generation should produce one local adjacency refresh"
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
    fn velocity_only_step_keeps_contact_geometry_cached() {
        let mut world = RotatingWorld3d::new(RotatingWorldConfig3d {
            gravity: Vec3i::new(0, -60, 0),
            ..RotatingWorldConfig3d::default()
        });
        world
            .add_box(rotating(RigidBody::dynamic(
                BodyId(1),
                Vec3i::ZERO,
                Vec3i::ZERO,
                Vec3i::new(2, 2, 2),
            )))
            .unwrap();
        let before = world.box_by_id(BodyId(1)).unwrap().oriented_box();
        let contacts = world.body_contacts(BodyId(1)).unwrap();
        let builds = world.current_contact_cache_stats().1;

        let report = world.step(1, 60).unwrap();

        assert_eq!(report.changed_body_ids, vec![BodyId(1)]);
        let body = world.box_by_id(BodyId(1)).unwrap();
        assert_eq!(body.oriented_box(), before);
        assert_ne!(body.body().velocity(), Vec3i::ZERO);
        assert_eq!(world.body_contacts(BodyId(1)).unwrap(), contacts);
        assert_eq!(
            world.current_contact_cache_stats().1,
            builds,
            "a velocity change without a quantized pose change must preserve contact evidence"
        );
    }

    #[test]
    fn unrelated_motion_preserves_stationary_subject_contacts() {
        let mut world = RotatingWorld3d::new(RotatingWorldConfig3d {
            gravity: Vec3i::ZERO,
            ..RotatingWorldConfig3d::default()
        });
        for id in 1..=64 {
            world
                .add_box(rotating(RigidBody::fixed(
                    BodyId(id),
                    Vec3i::new(id as i32 * 4, 0, 0),
                    Vec3i::new(2, 2, 2),
                )))
                .unwrap();
        }
        world
            .add_box(rotating(RigidBody::dynamic(
                BodyId(100),
                Vec3i::new(1_000, 0, 0),
                Vec3i::new(1, 0, 0),
                Vec3i::new(2, 2, 2),
            )))
            .unwrap();
        let before: Vec<_> = (1..=64)
            .map(|id| world.body_contacts(BodyId(id)).unwrap())
            .collect();
        let builds = world.current_contact_cache_stats().1;

        world.step(1, 1).unwrap();

        let after: Vec<_> = (1..=64)
            .map(|id| world.body_contacts(BodyId(id)).unwrap())
            .collect();
        assert_eq!(after, before);
        assert_eq!(
            world.current_contact_cache_stats().1,
            builds,
            "motion outside sleeping contact neighborhoods must preserve cached results"
        );
    }

    #[test]
    fn fixed_subject_contacts_do_not_depend_on_graph_query_history() {
        let mut world = RotatingWorld3d::new(RotatingWorldConfig3d::default());
        for (id, x) in [(1, 0), (2, 2)] {
            world
                .add_box(rotating(RigidBody::fixed(
                    BodyId(id),
                    Vec3i::new(x, 0, 0),
                    Vec3i::new(1, 1, 1),
                )))
                .unwrap();
        }
        world
            .add_box(rotating(RigidBody::dynamic(
                BodyId(3),
                Vec3i::new(100, 0, 0),
                Vec3i::ZERO,
                Vec3i::new(1, 1, 1),
            )))
            .unwrap();
        let before = world.body_contacts(BodyId(1)).unwrap();
        assert_eq!(before.len(), 1);
        world
            .overlap_query(world.box_by_id(BodyId(3)).unwrap().oriented_box())
            .unwrap();
        assert_eq!(
            world.body_contacts(BodyId(1)).unwrap(),
            before,
            "a response graph omits fixed/fixed pairs and cannot answer all fixed-subject contacts"
        );
    }

    #[test]
    fn local_invalidation_covers_new_and_lost_contacts() {
        for warm_graph in [false, true] {
            let mut world = RotatingWorld3d::new(RotatingWorldConfig3d::default());
            for (id, center) in [(1, 0), (3, 100)] {
                world
                    .add_box(rotating(RigidBody::fixed(
                        BodyId(id),
                        Vec3i::new(center, 0, 0),
                        Vec3i::new(1, 1, 1),
                    )))
                    .unwrap();
            }
            world
                .add_box(rotating(RigidBody::dynamic(
                    BodyId(2),
                    Vec3i::new(5, 0, 0),
                    Vec3i::ZERO,
                    Vec3i::new(1, 4, 1),
                )))
                .unwrap();
            if warm_graph {
                world
                    .overlap_query(world.box_by_id(BodyId(2)).unwrap().oriented_box())
                    .unwrap();
            }
            assert!(world.body_contacts(BodyId(1)).unwrap().is_empty());
            assert!(world.body_contacts(BodyId(3)).unwrap().is_empty());

            world
                .set_orientations(&[(BodyId(2), Orientation3d::new(0, 0, 1, 1))])
                .unwrap();
            if warm_graph {
                world
                    .overlap_query(world.box_by_id(BodyId(2)).unwrap().oriented_box())
                    .unwrap();
            }
            let contacts = world.body_contacts(BodyId(1)).unwrap();
            assert_eq!(
                contacts
                    .iter()
                    .map(|contact| contact.other)
                    .collect::<Vec<_>>(),
                vec![BodyId(2)]
            );
            assert!(world.body_contacts(BodyId(3)).unwrap().is_empty());
            assert_eq!(world.current_contact_cache_stats().1, 3);

            world
                .set_orientations(&[(BodyId(2), Orientation3d::IDENTITY)])
                .unwrap();
            assert!(world.body_contacts(BodyId(1)).unwrap().is_empty());
            assert!(world.body_contacts(BodyId(3)).unwrap().is_empty());
            assert_eq!(world.current_contact_cache_stats().1, 4);
        }
    }

    #[test]
    fn subject_index_prepares_only_changed_bounds() {
        for count in [32, 256, 2_048] {
            let mut boxes: BTreeMap<_, _> = (1..=count)
                .map(|id| {
                    (
                        BodyId(id),
                        rotating(RigidBody::fixed(
                            BodyId(id),
                            Vec3i::new(id as i32 * 16, 0, 0),
                            Vec3i::new(1, 1, 1),
                        )),
                    )
                })
                .collect();
            let mut cache = GenerationContactCache3d::default();
            for id in 1..=count {
                assert!(
                    cache
                        .body_contacts(&boxes, BodyId(id), 0)
                        .unwrap()
                        .is_empty()
                );
            }
            assert_eq!(
                cache.subject_bounds.as_ref().unwrap().bounds_preparations,
                count as usize
            );
            boxes.get_mut(&BodyId(1)).unwrap().body.position.x -= 1;
            cache.note_changed_generation(1, [BodyId(1)]);
            for id in 1..=count {
                assert!(
                    cache
                        .body_contacts(&boxes, BodyId(id), 1)
                        .unwrap()
                        .is_empty()
                );
            }
            assert_eq!(
                cache.subject_bounds.as_ref().unwrap().bounds_preparations,
                count as usize + 1
            );
            assert_eq!(cache.subject_builds, count + 1);
        }
    }

    #[test]
    #[ignore = "release-mode stationary subject cache scaling evidence"]
    fn stationary_subject_reuse_benchmark() {
        for count in [32, 256, 2_048] {
            let mut boxes: BTreeMap<_, _> = (1..=count)
                .map(|id| {
                    (
                        BodyId(id),
                        rotating(RigidBody::fixed(
                            BodyId(id),
                            Vec3i::new(id as i32 * 16, 0, 0),
                            Vec3i::new(1, 1, 1),
                        )),
                    )
                })
                .collect();
            let mut cache = GenerationContactCache3d::default();
            for id in 1..=count {
                cache.body_contacts(&boxes, BodyId(id), 0).unwrap();
            }
            let start = Instant::now();
            for generation in 1..=64 {
                boxes.get_mut(&BodyId(1)).unwrap().body.position.y = (generation % 2) as i32;
                cache.note_changed_generation(generation, [BodyId(1)]);
                for id in 1..=count {
                    black_box(cache.body_contacts(&boxes, BodyId(id), generation).unwrap());
                }
            }
            let elapsed = start.elapsed();
            let bounds_prepared =
                cache.subject_bounds.as_ref().unwrap().bounds_preparations - count as usize;
            let subject_rebuilds = cache.subject_builds - count;
            assert_eq!(bounds_prepared, 64);
            assert_eq!(subject_rebuilds, 64);
            crate::performance_ratchet::record(
                &format!("stationary-cache/{count}"),
                &[
                    ("bounds_prepared", bounds_prepared as u64),
                    ("subject_rebuilds", subject_rebuilds),
                ],
                &[("queries", 64 * count)],
                &[("elapsed_ms", elapsed.as_secs_f64() * 1_000.0)],
            );
            println!(
                "stationary subject cache: bodies={count}, frames=64, queries={}, bounds_prepared={bounds_prepared}, subject_rebuilds={subject_rebuilds}, elapsed={elapsed:?}",
                64 * count
            );
        }
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
