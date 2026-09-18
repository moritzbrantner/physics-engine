use std::{cell::RefCell, collections::BTreeMap};

use crate::{
    BodyId, BodyKind, CollisionLayers3d, OrientedBox3d, RigidBox3d, RigidBoxFreeFlightConfig3d,
    RotatingBroadPhaseError3d, RotatingWorldError3d, SolverParticipation3d, Vec3i,
    obb_contact_seed, rigid_box_free_flight_sweep_bounds, rotational_sweep_candidate_pairs,
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

#[derive(Clone, Debug, Default, Eq, PartialEq)]
struct CurrentContactCache3d {
    fingerprint: Vec<(
        BodyId,
        BodyKind,
        CollisionLayers3d,
        SolverParticipation3d,
        OrientedBox3d,
    )>,
    graph: Option<CurrentContactGraph3d>,
    builds: u64,
    candidate_pairs: u64,
    exact_pair_tests: u64,
}

std::thread_local! {
    static CURRENT_CONTACT_CACHE: RefCell<CurrentContactCache3d> =
        RefCell::new(CurrentContactCache3d::default());
}

/// Returns exact current OBB contacts for one existing body after pruning through the engine's zero-time
/// rotational broad phase.
///
/// A deterministic contact graph is cached by the complete BodyId/kind/geometry snapshot. Repeated queries
/// against the same snapshot therefore reuse one exact-filtered graph instead of repeating an all-body OBB
/// scan for every subject. Velocity changes do not invalidate the graph; pose, membership, or body-kind
/// changes do. The graph stores both subject directions with contact axes oriented from each subject toward
/// its neighbor.
#[cfg(test)]
pub(crate) fn body_current_contacts(
    boxes: &[RigidBox3d],
    body: BodyId,
) -> Result<Vec<BodyCurrentContact3d>, RotatingWorldError3d> {
    let fingerprint = snapshot_fingerprint(boxes.iter());
    if let Some(contacts) = cached_contacts(&fingerprint, body) {
        return Ok(contacts);
    }

    let graph = build_current_contact_graph(boxes)?;
    cache_graph(fingerprint, graph.clone());
    graph
        .contacts
        .get(&body)
        .cloned()
        .ok_or(RotatingWorldError3d::MissingBody(body))
}

/// Computes exact current contacts for one known body without materializing the full-world contact graph.
///
/// The subject is looked up directly by `BodyId`. Every unrelated body pays only for its current conservative
/// bound; exact OBB contact work is performed only for collision-enabled bodies whose current bounds overlap
/// the subject. This is the precise single-subject counterpart to the cached full contact graph.
pub(crate) fn body_current_contacts_for_body(
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

/// Fast path for an existing dynamic body's exact overlap query.
///
/// The map is already BodyId ordered, so a cache hit computes only the small geometry fingerprint and clones
/// the requested neighbor list. Bodies are cloned for broad-phase construction only when the geometry/kind
/// snapshot changed.
pub(crate) fn body_current_overlap_ids(
    boxes: &BTreeMap<BodyId, RigidBox3d>,
    body: BodyId,
) -> Result<Vec<BodyId>, RotatingWorldError3d> {
    let subject = boxes
        .get(&body)
        .ok_or(RotatingWorldError3d::MissingBody(body))?;
    if subject.body().kind() != BodyKind::Dynamic {
        return Err(RotatingWorldError3d::MissingBody(body));
    }

    let fingerprint = snapshot_fingerprint(boxes.values());
    let contacts = if let Some(contacts) = cached_contacts(&fingerprint, body) {
        contacts
    } else {
        let snapshot = boxes.values().cloned().collect::<Vec<_>>();
        let graph = build_current_contact_graph(&snapshot)?;
        let contacts = graph
            .contacts
            .get(&body)
            .cloned()
            .ok_or(RotatingWorldError3d::MissingBody(body))?;
        cache_graph(fingerprint, graph);
        contacts
    };

    let mut ids = Vec::with_capacity(contacts.len().saturating_add(1));
    ids.push(body);
    ids.extend(contacts.into_iter().map(|contact| contact.other));
    ids.sort_unstable();
    ids.dedup();
    Ok(ids)
}

fn snapshot_fingerprint<'a>(
    boxes: impl IntoIterator<Item = &'a RigidBox3d>,
) -> Vec<(
    BodyId,
    BodyKind,
    CollisionLayers3d,
    SolverParticipation3d,
    OrientedBox3d,
)> {
    let mut fingerprint = boxes
        .into_iter()
        .map(|rigid_box| {
            (
                rigid_box.body().id(),
                rigid_box.body().kind(),
                rigid_box.collision_layers(),
                rigid_box.solver_participation(),
                rigid_box.oriented_box(),
            )
        })
        .collect::<Vec<_>>();
    fingerprint.sort_by_key(|(id, _, _, _, _)| *id);
    fingerprint
}

fn cached_contacts(
    fingerprint: &[(
        BodyId,
        BodyKind,
        CollisionLayers3d,
        SolverParticipation3d,
        OrientedBox3d,
    )],
    body: BodyId,
) -> Option<Vec<BodyCurrentContact3d>> {
    CURRENT_CONTACT_CACHE.with(|cache| {
        let cache = cache.borrow();
        if cache.fingerprint != fingerprint {
            return None;
        }
        cache.graph.as_ref()?.contacts.get(&body).cloned()
    })
}

fn cache_graph(
    fingerprint: Vec<(
        BodyId,
        BodyKind,
        CollisionLayers3d,
        SolverParticipation3d,
        OrientedBox3d,
    )>,
    graph: CurrentContactGraph3d,
) {
    CURRENT_CONTACT_CACHE.with(|cache| {
        let mut cache = cache.borrow_mut();
        cache.fingerprint = fingerprint;
        cache.builds = cache.builds.saturating_add(1);
        cache.candidate_pairs = cache
            .candidate_pairs
            .saturating_add(u64::try_from(graph.candidate_pairs).unwrap_or(u64::MAX));
        cache.exact_pair_tests = cache
            .exact_pair_tests
            .saturating_add(u64::try_from(graph.exact_pair_tests).unwrap_or(u64::MAX));
        cache.graph = Some(graph);
    });
}

fn build_current_contact_graph(
    boxes: &[RigidBox3d],
) -> Result<CurrentContactGraph3d, RotatingWorldError3d> {
    let by_id = boxes
        .iter()
        .map(|rigid_box| (rigid_box.body().id(), rigid_box))
        .collect::<BTreeMap<_, _>>();
    let candidates =
        rotational_sweep_candidate_pairs(boxes, RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 0, 1))
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
fn reset_cache() {
    CURRENT_CONTACT_CACHE.with(|cache| *cache.borrow_mut() = CurrentContactCache3d::default());
}

#[cfg(test)]
fn cache_stats() -> (u64, u64, u64) {
    CURRENT_CONTACT_CACHE.with(|cache| {
        let cache = cache.borrow();
        (cache.builds, cache.candidate_pairs, cache.exact_pair_tests)
    })
}

#[cfg(test)]
mod tests {
    use std::{hint::black_box, time::Instant};

    use crate::{
        AngularState3d, AngularVelocity3d, BodyId, ORIENTATION_SCALE, Orientation3d, RigidBody,
        RigidBox3d, RotatingWorldConfig3d, Vec3i, obb_contact_seed,
        rotating_world::RotatingWorld3d,
    };

    use super::{body_current_contacts, build_current_contact_graph, cache_stats, reset_cache};

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
        reset_cache();
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
            cache_stats().0,
            1,
            "one stable snapshot should build one graph"
        );

        world
            .set_linear_velocity(BodyId(1), Vec3i::new(17, 0, 0))
            .expect("velocity update");
        let first = world.box_by_id(BodyId(1)).expect("body one").oriented_box();
        world
            .overlap_query(first)
            .expect("velocity-only cache reuse");
        assert_eq!(
            cache_stats().0,
            1,
            "velocity alone must not rebuild contact geometry"
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
        assert_eq!(cache_stats().0, 2, "pose change must refresh the graph");
    }

    #[test]
    fn body_current_contacts_keep_stable_neighbor_order() {
        reset_cache();
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
        let actual = body_current_contacts(&boxes, subject.body().id()).expect("body contacts");
        assert_eq!(
            actual.iter().map(|entry| entry.other).collect::<Vec<_>>(),
            vec![BodyId(2), BodyId(90)]
        );
    }

    #[test]
    #[ignore = "release-mode deterministic performance evidence; run explicitly with --ignored --nocapture"]
    fn sleep_style_contact_graph_benchmark() {
        reset_cache();
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
        let (builds, candidate_pairs, exact_pair_tests) = cache_stats();

        assert_eq!(
            builds, 1,
            "stable sleep-style traversal should build one graph"
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
