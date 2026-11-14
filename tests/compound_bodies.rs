//! Compound dynamic bodies (#567, moenarch-games/shooter#73): primitive children with mass at
//! their mounting positions, resting, tipping, colliding, hit per child and rebuilt exactly.
use physics_engine::BodyId;
use physics_engine::approximate::{
    Body, BodyStorage, CompoundChild, Config, Error, Medium, PreparedCompound, Quaternion,
    QueryFilter, QueryPose, RayCast, Shape, ShapeCast, Vector as V, World,
};

const DT: f64 = 1.0 / 60.0;
/// The issue's chassis: 40 x 40 x 80 cm (half extents in metres), 20 kg.
const CHASSIS: V = V(0.2, 0.4, 0.2);

fn world(storage: BodyStorage, warm_start: bool) -> World {
    let mut w = World::new(Config {
        body_storage: storage,
        warm_start,
        ..Config::default()
    })
    .unwrap();
    w.add_body(Body::keyed(1, Shape::HalfSpace, V::ZERO, 0.0))
        .unwrap();
    w
}

/// An upright chassis with a 30 kg block mounted `offset` metres along X at its top.
fn robot(offset: f32) -> PreparedCompound {
    PreparedCompound::new(&[
        CompoundChild::new(Shape::Box(CHASSIS), V::ZERO, 20.0),
        CompoundChild::new(Shape::Box(V(0.1, 0.1, 0.1)), V(offset, 0.5, 0.0), 30.0),
    ])
    .unwrap()
}

fn place(w: &mut World, c: &PreparedCompound, key: u64, origin: V) -> BodyId {
    let (position, orientation) = c.body_pose(origin, Quaternion::IDENTITY);
    let mut body = Body::keyed(key, Shape::Sphere(1.0), position, 1.0);
    body.orientation = orientation;
    body.sleep_allowed = false;
    w.add_compound(c, body).unwrap()
}

/// Tilt of the body's up axis from world up, in radians.
fn tilt(w: &World, c: &PreparedCompound, id: BodyId) -> f32 {
    let b = w.body(id).unwrap();
    let (_, origin) = c.origin_pose(b.position, b.orientation);
    origin.rotate(V::Y).dot(V::Y).clamp(-1.0, 1.0).acos()
}

#[test]
fn mass_at_the_mounting_position_decides_whether_it_tips() {
    let mut w = world(BodyStorage::Ordered, true);
    let centred = robot(0.0);
    let off = robot(0.6);
    let upright = place(&mut w, &centred, 10, V(0.0, CHASSIS.1, 0.0));
    let tipping = place(&mut w, &off, 11, V(5.0, CHASSIS.1, 0.0));
    assert_eq!(w.body(upright).unwrap().mass, 50.0);
    for _ in 0..180 {
        w.step(DT).unwrap();
    }
    assert!(
        tilt(&w, &centred, upright) < 0.01,
        "{}",
        tilt(&w, &centred, upright)
    );
    // Tipped over its +X edge onto its side.
    assert!(tilt(&w, &off, tipping) > 1.0, "{}", tilt(&w, &off, tipping));
    let b = w.body(tipping).unwrap();
    assert!(b.position.0 > 5.0, "{:?}", b.position);
    // Resting on the floor, not through it.
    for id in [upright, tipping] {
        for k in 0..2 {
            let (p, q) = w.compound_child_pose(id, k).unwrap();
            let lowest = [-1.0, 1.0]
                .iter()
                .flat_map(|&x| [-1.0, 1.0].map(move |y| (x, y)))
                .flat_map(|(x, y)| [-1.0, 1.0].map(move |z| V(x, y, z)))
                .map(|s| {
                    let h = w.compound(id).unwrap().children()[k].shape.half_extents();
                    (p + q.rotate(V(s.0 * h.0, s.1 * h.1, s.2 * h.2))).1
                })
                .fold(f32::INFINITY, f32::min);
            assert!(lowest > -0.01, "child {k} of {id:?} at {lowest}");
        }
    }
}

#[test]
fn rays_and_casts_report_the_child_they_hit() {
    let mut w = world(BodyStorage::Ordered, true);
    let c = PreparedCompound::new(&[
        CompoundChild::new(Shape::Box(CHASSIS), V::ZERO, 20.0),
        // An armour plate on the chassis' +Z face.
        CompoundChild::new(Shape::Box(V(0.15, 0.3, 0.02)), V(0.0, 0.0, 0.23), 5.0),
        CompoundChild::new(Shape::Sphere(0.1), V(0.0, 0.55, 0.0), 2.0),
    ])
    .unwrap();
    let id = place(&mut w, &c, 10, V(0.0, 1.0, 0.0));
    let ray = |origin: V, d: V| {
        w.cast_ray_first(RayCast::new(origin, d), QueryFilter::default())
            .unwrap()
            .unwrap()
    };
    let plate = ray(V(0.0, 1.0, 2.0), V(0.0, 0.0, -4.0));
    assert_eq!((plate.body, plate.child), (id, Some(1)));
    assert!((plate.point.2 - 0.25).abs() < 1e-4, "{plate:?}");
    let chassis = ray(V(0.0, 1.0, -2.0), V(0.0, 0.0, 4.0));
    assert_eq!(chassis.child, Some(0));
    let head = ray(V(0.0, 3.0, 0.0), V(0.0, -4.0, 0.0));
    assert_eq!(head.child, Some(2));
    // The floor plane is not a compound.
    let floor = ray(V(3.0, 1.0, 0.0), V(0.0, -2.0, 0.0));
    assert_eq!(floor.child, None);
    let mut all = Vec::new();
    w.cast_ray(
        RayCast::new(V(0.0, 1.0, 2.0), V(0.0, 0.0, -4.0)),
        QueryFilter::default(),
        &mut all,
    )
    .unwrap();
    assert_eq!(all.len(), 1);
    assert_eq!(all[0].child, Some(1));

    let mut hits = Vec::new();
    let pose = QueryPose::new(Shape::Sphere(0.05), V(0.0, 1.0, 2.0));
    w.cast_shape(
        ShapeCast::new(pose, V(0.0, 0.0, -4.0)),
        QueryFilter::default(),
        &mut hits,
    )
    .unwrap();
    let hit = hits.iter().find(|h| h.body == id).unwrap();
    assert_eq!(hit.child, Some(1));
    assert!((hit.target_point.2 - 0.25).abs() < 1e-3, "{hit:?}");
    w.overlap_shape(
        QueryPose::new(Shape::Sphere(0.05), V(0.0, 1.56, 0.0)),
        QueryFilter::default(),
        &mut hits,
    )
    .unwrap();
    assert_eq!(hits.len(), 1);
    assert_eq!((hits[0].body, hits[0].child), (id, Some(2)));
    // A compound is not a query shape.
    assert!(
        w.overlap_shape(
            QueryPose::new(w.body(id).unwrap().shape, V::ZERO),
            QueryFilter::default(),
            &mut hits,
        )
        .is_err()
    );
}

/// Two robots driven into each other and a third dropped onto a box block: every step's state.
fn scene(storage: BodyStorage, warm_start: bool) -> (World, Vec<BodyId>) {
    let mut w = world(storage, warm_start);
    w.add_body(Body::keyed(
        2,
        Shape::Box(V(1.0, 0.25, 1.0)),
        V(0.0, 0.25, 6.0),
        0.0,
    ))
    .unwrap();
    let a = robot(0.15);
    let b = robot(-0.15);
    let ids = vec![
        place(&mut w, &a, 20, V(-1.0, CHASSIS.1 + 0.001, 0.0)),
        place(&mut w, &b, 21, V(1.0, CHASSIS.1 + 0.001, 0.0)),
        place(&mut w, &a, 22, V(0.2, 2.0, 6.0)),
    ];
    w.set_velocity(ids[0], V(3.0, 0.0, 0.0)).unwrap();
    w.set_velocity(ids[1], V(-3.0, 0.0, 0.0)).unwrap();
    (w, ids)
}

fn state(w: &World, ids: &[BodyId]) -> Vec<(V, Quaternion, V, V)> {
    ids.iter()
        .map(|&id| {
            let b = w.body(id).unwrap();
            (b.position, b.orientation, b.velocity, b.angular_velocity)
        })
        .collect()
}

#[test]
fn compounds_collide_with_each_other_and_with_boxes() {
    let (mut w, ids) = scene(BodyStorage::Ordered, true);
    let mut hits = Vec::new();
    let mut deepest = 0.0f32;
    for _ in 0..120 {
        w.step(DT).unwrap();
        // No child sinks into another body (the other robot, the block or the floor) by more
        // than a few contact slops.
        for &id in &ids {
            for (k, child) in w.compound(id).unwrap().children().iter().enumerate() {
                let (position, orientation) = w.compound_child_pose(id, k).unwrap();
                let pose = QueryPose {
                    shape: child.shape,
                    position,
                    orientation,
                };
                let filter = QueryFilter {
                    exclude: Some(id),
                    ..QueryFilter::default()
                };
                w.overlap_shape(pose, filter, &mut hits).unwrap();
                for hit in &hits {
                    deepest = deepest.min(hit.separation);
                }
            }
        }
    }
    assert!(deepest > -0.02, "{deepest}");
    let [a, b, dropped] = [ids[0], ids[1], ids[2]].map(|id| w.body(id).unwrap().clone());
    // The chassis met: neither kept its 3 m/s or passed the other.
    assert!(a.position.0 < b.position.0, "{a:?} {b:?}");
    assert!(a.velocity.0.abs() < 1.0 && b.velocity.0.abs() < 1.0);
    // The dropped robot came to rest on the block (top at 0.5 m).
    assert!(
        dropped.position.1 > 0.5 && dropped.velocity.length() < 0.1,
        "{dropped:?}"
    );
}

#[test]
fn storage_modes_give_identical_results() {
    let (mut ordered, ids) = scene(BodyStorage::Ordered, true);
    let (mut indexed, same) = scene(BodyStorage::Indexed, true);
    for _ in 0..90 {
        ordered.step(DT).unwrap();
        indexed.step(DT).unwrap();
        assert_eq!(state(&ordered, &ids), state(&indexed, &same));
    }
}

#[test]
fn a_rebuilt_compound_continues_bit_identically() {
    // The exact-continuation contract (warm starting off): re-adding each compound from its
    // read-back body gives the same next steps, as a rollback or a client's prediction needs.
    let (mut w, ids) = scene(BodyStorage::Ordered, false);
    for _ in 0..40 {
        w.step(DT).unwrap();
    }
    let prepared: Vec<_> = ids
        .iter()
        .map(|&id| w.compound(id).unwrap().clone())
        .collect();
    let saved: Vec<Body> = ids.iter().map(|&id| w.body(id).unwrap().clone()).collect();
    let mut reference = w.clone();
    let mut rebuilt = world(BodyStorage::Ordered, false);
    rebuilt
        .add_body(Body::keyed(
            2,
            Shape::Box(V(1.0, 0.25, 1.0)),
            V(0.0, 0.25, 6.0),
            0.0,
        ))
        .unwrap();
    let mut again = Vec::new();
    for (c, b) in prepared.iter().zip(&saved).rev() {
        again.push(rebuilt.add_compound(c, b.clone()).unwrap());
    }
    again.reverse();
    // The same world, rolled back in place: remove and re-add.
    for (k, &id) in ids.clone().iter().enumerate() {
        w.remove_body(id).unwrap();
        let back = w.add_compound(&prepared[k], saved[k].clone()).unwrap();
        assert_eq!(w.body(back).unwrap().position, saved[k].position);
    }
    let in_place: Vec<_> = (0..3).map(|k| w.body_id(20 + k).unwrap()).collect();
    for _ in 0..60 {
        reference.step(DT).unwrap();
        rebuilt.step(DT).unwrap();
        w.step(DT).unwrap();
        let expected = state(&reference, &ids);
        assert_eq!(state(&rebuilt, &again), expected);
        assert_eq!(state(&w, &in_place), expected);
    }
}

#[test]
fn a_ccd_projectile_stops_at_the_child_it_strikes() {
    let mut w = World::new(Config {
        gravity: V::ZERO,
        ..Config::default()
    })
    .unwrap();
    let c = PreparedCompound::new(&[
        CompoundChild::new(Shape::Box(CHASSIS), V::ZERO, 20.0),
        // A thin plate, 2 cm thick, in front of the chassis.
        CompoundChild::new(Shape::Box(V(0.3, 0.3, 0.01)), V(0.0, 0.0, 0.5), 2.0),
    ])
    .unwrap();
    let robot = place(&mut w, &c, 10, V::ZERO);
    let mut bullet = Body::keyed(1, Shape::Sphere(0.02), V(0.0, 0.0, 3.0), 0.05);
    bullet.velocity = V(0.0, 0.0, -200.0);
    bullet.ccd = true;
    let bullet = w.add_body(bullet).unwrap();
    for _ in 0..3 {
        w.step(DT).unwrap();
    }
    let b = w.body(bullet).unwrap();
    assert!(
        b.position.2 > 0.5,
        "the bullet passed the plate: {:?}",
        b.position
    );
    // Momentum went into the robot, off its centre of mass: it moves and turns.
    let r = w.body(robot).unwrap();
    assert!(r.velocity.2 < 0.0, "{r:?}");
}

#[test]
fn compound_admission_rules() {
    let mut w = world(BodyStorage::Ordered, true);
    let c = robot(0.3);
    let id = place(&mut w, &c, 10, V(0.0, 1.0, 0.0));
    let body = w.body(id).unwrap().clone();
    // Only `add_compound` admits a compound shape.
    assert_eq!(w.add_body(body.clone()), Err(Error::InvalidInput));
    for change in [
        |b: &mut Body| b.ccd = true,
        |b: &mut Body| b.external = true,
    ] {
        let mut b = body.clone();
        b.key = Some(99);
        change(&mut b);
        assert_eq!(w.add_compound(&c, b), Err(Error::InvalidInput));
    }
    assert_eq!(
        w.set_medium(
            id,
            Some(Medium {
                surface_point: V::ZERO,
                surface_normal: V::Y,
                density: 1000.0,
                flow: V::ZERO,
                linear_drag: 0.0,
                angular_drag: 0.0,
            })
        ),
        Err(Error::InvalidInput)
    );
    assert!(w.compound(BodyId::from_bits(0)).is_none());
    assert!(w.compound_child_pose(id, 2).is_err());
    assert!(body.kinetic_energy().is_finite());
}

#[cfg(feature = "checkpoints")]
#[test]
fn checkpoints_reject_compounds() {
    use physics_engine::approximate::{CheckpointContext, CheckpointError};
    let mut w = world(BodyStorage::Ordered, true);
    place(&mut w, &robot(0.0), 10, V(0.0, 1.0, 0.0));
    assert_eq!(
        w.checkpoint(CheckpointContext {
            build: [0; 32],
            content: [0; 32],
        })
        .err(),
        Some(CheckpointError::UnsupportedCompounds)
    );
}

#[test]
fn compounds_rest_on_terrain_meshes_and_hulls() {
    use physics_engine::approximate::{ConvexHullDesc, HeightfieldDesc, TriangleMeshDesc};
    // A chassis with a ball foot hanging below one end: it rests on the box and the ball.
    let c = PreparedCompound::new(&[
        CompoundChild::new(Shape::Box(V(0.4, 0.1, 0.2)), V(0.0, 0.0, 0.0), 10.0),
        CompoundChild::new(Shape::Sphere(0.1), V(0.3, -0.1, 0.0), 2.0),
    ])
    .unwrap();
    let surfaces: [&dyn Fn(&mut World); 3] = [
        &|w| {
            w.add_heightfield(
                HeightfieldDesc {
                    origin: V(-5.0, 0.0, -5.0),
                    cell_size: (1.0, 1.0),
                    columns: 11,
                    rows: 11,
                    heights: vec![0.0; 121],
                    removed_cells: None,
                    material: Default::default(),
                },
                Some(1),
            )
            .unwrap();
        },
        &|w| {
            w.add_triangle_mesh(
                TriangleMeshDesc {
                    position: V::ZERO,
                    vertices: vec![
                        V(-5.0, 0.0, -5.0),
                        V(5.0, 0.0, -5.0),
                        V(5.0, 0.0, 5.0),
                        V(-5.0, 0.0, 5.0),
                    ],
                    triangles: vec![[0, 2, 1], [0, 3, 2]],
                    material: Default::default(),
                },
                Some(1),
            )
            .unwrap();
        },
        &|w| {
            let vertices = (0..8)
                .map(|k| {
                    V(
                        if k & 1 == 0 { -5.0 } else { 5.0 },
                        if k & 2 == 0 { -1.0 } else { 0.0 },
                        if k & 4 == 0 { -5.0 } else { 5.0 },
                    )
                })
                .collect();
            w.add_convex_hull(
                ConvexHullDesc {
                    vertices,
                    position: V::ZERO,
                    material: Default::default(),
                },
                Some(1),
            )
            .unwrap();
        },
    ];
    for (k, surface) in surfaces.iter().enumerate() {
        let mut w = World::new(Config::default()).unwrap();
        surface(&mut w);
        let id = place(&mut w, &c, 10, V(0.0, 0.25, 0.0));
        for _ in 0..120 {
            w.step(DT).unwrap();
        }
        // The chassis' -X end and the ball both touch: the chassis tilts by about
        // atan(0.1 / 0.7) and nothing sinks.
        let (ball, _) = w.compound_child_pose(id, 1).unwrap();
        assert!((ball.1 - 0.1).abs() < 0.01, "surface {k}: ball at {ball:?}");
        let b = w.body(id).unwrap();
        assert!(b.velocity.length() < 0.05, "surface {k}: {b:?}");
        assert!(tilt(&w, &c, id) > 0.05, "surface {k}: {}", tilt(&w, &c, id));
    }
}

#[test]
fn centimetre_robots_tip_as_metre_ones() {
    // The shooter's scale: `length_unit` 100. The same builds, every length times 100.
    let mut w = World::new(Config {
        length_unit: 100.0,
        ..Config::default()
    })
    .unwrap();
    w.add_body(Body::keyed(1, Shape::HalfSpace, V::ZERO, 0.0))
        .unwrap();
    let robot = |offset: f32| {
        PreparedCompound::new(&[
            CompoundChild::new(Shape::Box(CHASSIS * 100.0), V::ZERO, 20.0),
            CompoundChild::new(Shape::Box(V(10.0, 10.0, 10.0)), V(offset, 50.0, 0.0), 30.0),
        ])
        .unwrap()
    };
    let (centred, off) = (robot(0.0), robot(60.0));
    let upright = place(&mut w, &centred, 10, V(0.0, 40.0, 0.0));
    let tipping = place(&mut w, &off, 11, V(500.0, 40.0, 0.0));
    for _ in 0..180 {
        w.step(DT).unwrap();
    }
    assert!(tilt(&w, &centred, upright) < 0.01);
    assert!(tilt(&w, &off, tipping) > 1.0);
}
