//! Heightfield terrain (#195): rays, overlaps and characters standing and walking on hills.
use physics_engine::approximate::{
    Body, CharacterMove, Config, HeightfieldDesc, QueryFilter, RayCast, Shape, Vector as V, World,
};

const N: usize = 33;
const CELL: f32 = 2.0;

fn height(x: f32, z: f32) -> f32 {
    1.5 * (x * 0.2).sin() * (z * 0.15).cos()
}

fn hills() -> (World, physics_engine::BodyId) {
    hills_with(V::ZERO)
}

fn hills_with(gravity: V) -> (World, physics_engine::BodyId) {
    let mut w = World::new(Config {
        gravity,
        ..Config::default()
    })
    .unwrap();
    let origin = V(-32.0, 0.0, -32.0);
    let heights = (0..N * N)
        .map(|k| {
            height(
                origin.0 + CELL * (k % N) as f32,
                origin.2 + CELL * (k / N) as f32,
            )
        })
        .collect();
    let id = w
        .add_heightfield(
            HeightfieldDesc {
                origin,
                cell_size: (CELL, CELL),
                columns: N,
                rows: N,
                heights,
                removed_cells: None,
            },
            Some(1),
        )
        .unwrap();
    (w, id)
}

/// Terrain height of the fixed-diagonal triangulation at (x, z), by a downward ray.
fn ground(w: &World, x: f32, z: f32) -> f32 {
    // Terrain only (layer 1): characters live on layer 2.
    let filter = QueryFilter {
        layers: physics_engine::CollisionLayers3d::new(1, 1),
        ..QueryFilter::default()
    };
    let hit = w
        .cast_ray_first(RayCast::new(V(x, 50.0, z), V(0.0, -100.0, 0.0)), filter)
        .unwrap()
        .expect("terrain below");
    assert!(hit.normal.1 > 0.0);
    hit.point.1
}

#[test]
fn rays_hit_the_triangulated_surface() {
    let (w, terrain) = hills();
    // At grid vertices the surface equals the sampled height.
    for (i, j) in [(3, 4), (10, 20), (16, 16)] {
        let (x, z) = (-32.0 + CELL * i as f32, -32.0 + CELL * j as f32);
        assert!((ground(&w, x, z) - height(x, z)).abs() < 1e-4);
    }
    let hit = w
        .cast_ray_first(
            RayCast::new(V(0.3, 50.0, 0.7), V(0.0, -100.0, 0.0)),
            QueryFilter::default(),
        )
        .unwrap()
        .unwrap();
    assert_eq!(hit.body, terrain);
    // A horizontal ray far above misses.
    assert!(
        w.cast_ray_first(
            RayCast::new(V(-30.0, 20.0, 0.0), V(60.0, 0.0, 0.0)),
            QueryFilter::default()
        )
        .unwrap()
        .is_none()
    );
}

#[test]
fn characters_stand_and_walk_at_terrain_height() {
    let (mut w, terrain) = hills();
    let (half, radius) = (0.5, 0.3);
    let start = V(-10.0, ground(&w, -10.0, -10.0) + 3.0, -10.0);
    let mut body = Body::keyed(10, Shape::capsule(half, radius), start, 1.0);
    body.external = true;
    body.layers = physics_engine::CollisionLayers3d::new(2, u32::MAX);
    let c = w.add_body(body).unwrap();
    let settings = w.character_settings();
    w.add_character(c, settings).unwrap();
    let mut out = Vec::new();
    let mut vy = 0.0;
    let mut landed = false;
    let mut rest = None;
    for tick in 0..400 {
        vy -= 9.81 / 60.0;
        let walk = if tick > 100 { 0.03 } else { 0.0 };
        w.move_characters(
            &[CharacterMove {
                body: c,
                desired: V(walk, vy / 60.0, walk * 0.5),
            }],
            &mut out,
        )
        .unwrap();
        let r = out[0];
        if r.grounded {
            vy = 0.0;
            landed = true;
            assert_eq!(r.support, Some(terrain));
        }
        let p = w.body(c).unwrap().position;
        if (60..100).contains(&tick) {
            // Landed on the slope and idle: resting exactly, no sliding or jitter.
            assert!(r.grounded, "tick {tick}");
            match rest {
                None => rest = Some(p),
                Some(q) => assert_eq!(p, q, "tick {tick}: moved while resting"),
            }
        }
        // Never below the surface: the capsule's lowest point stays above the terrain under it.
        let foot = p.1 - half - radius;
        assert!(
            foot > ground(&w, p.0, p.2) - 2e-3,
            "tick {tick}: sank into the terrain"
        );
    }
    assert!(landed);
    let p = w.body(c).unwrap().position;
    assert!(p.0 > -2.0, "walked across the hills: {p:?}");
}

#[test]
fn box_and_wedge_overlaps_meet_the_terrain_surface() {
    use physics_engine::approximate::{Quaternion, QueryPose};
    let (w, terrain) = hills();
    let mut hits = Vec::new();
    for (x, z) in [(-10.0, -10.0), (3.0, 7.0), (12.0, -4.0)] {
        let g = ground(&w, x, z);
        // A door probe straddling the surface overlaps it; one clear above does not.
        for (shape, half_height) in [
            (Shape::Box(V(0.6, 1.0, 0.1)), 1.0),
            (Shape::wedge(V(0.6, 1.0, 0.6)), 1.0),
        ] {
            let mut pose = QueryPose::new(shape, V(x, g + half_height - 0.3, z));
            pose.orientation = Quaternion(0.0, 0.38, 0.0, 0.925).normalized();
            w.overlap_shape(pose, QueryFilter::default(), &mut hits)
                .unwrap();
            assert_eq!(hits.len(), 1, "{shape:?} at {x}, {z}");
            assert_eq!(hits[0].body, terrain);
            assert!(hits[0].separation < 0.0, "{:?}", hits[0]);
            pose.position.1 = g + half_height + 3.0;
            w.overlap_shape(pose, QueryFilter::default(), &mut hits)
                .unwrap();
            assert!(hits.is_empty());
        }
    }
}

#[test]
fn terrain_rejects_rigid_body_api_misuse() {
    let (mut w, terrain) = hills();
    assert!(w.add_body(w.body(terrain).unwrap().clone()).is_err());
    // Box casts against terrain are not supported (overlaps are).
    let r = w.cast_shape(
        physics_engine::approximate::ShapeCast::new(
            physics_engine::approximate::QueryPose::new(
                Shape::Box(V(1.0, 1.0, 1.0)),
                V(0.0, 9.0, 0.0),
            ),
            V(0.0, -20.0, 0.0),
        ),
        QueryFilter::default(),
        &mut Vec::new(),
    );
    assert!(r.is_err());
    // Stepping a world with terrain and a dynamic body clear of it.
    w.add_body(Body::new(Shape::Sphere(0.5), V(0.0, 10.0, 0.0), 1.0))
        .unwrap();
    w.step(1.0 / 60.0).unwrap();
}

/// A character started inside a floor plane that coincides with flat terrain is pushed out
/// once, not once per surface (#341).
#[test]
fn coincident_floor_and_flat_terrain_push_out_once() {
    let mut w = World::new(Config {
        gravity: V::ZERO,
        ..Config::default()
    })
    .unwrap();
    w.add_body(Body::keyed(1, Shape::HalfSpace, V::ZERO, 0.0))
        .unwrap();
    w.add_heightfield(
        HeightfieldDesc {
            origin: V(-8.0, 0.0, -8.0),
            cell_size: (2.0, 2.0),
            columns: 9,
            rows: 9,
            heights: vec![0.0; 81],
            removed_cells: None,
        },
        Some(2),
    )
    .unwrap();
    let (half, radius) = (0.5, 0.3);
    let mut body = Body::keyed(
        10,
        Shape::capsule(half, radius),
        V(0.3, half + radius - 0.05, 0.3),
        1.0,
    );
    body.external = true;
    let c = w.add_body(body).unwrap();
    let settings = w.character_settings();
    w.add_character(c, settings).unwrap();
    let mut out = Vec::new();
    w.move_characters(
        &[CharacterMove {
            body: c,
            desired: V::ZERO,
        }],
        &mut out,
    )
    .unwrap();
    let y = w.body(c).unwrap().position.1;
    assert!(
        (y - (half + radius + settings.offset)).abs() < 1e-4,
        "pushed to {y}, depenetration {:?}",
        out[0].depenetration
    );
}

/// Removed cells are holes: rays and characters pass through to what lies below (#342).
#[test]
fn removed_cells_are_holes() {
    let mut w = World::new(Config {
        gravity: V::ZERO,
        ..Config::default()
    })
    .unwrap();
    // A floor 2 m below a 1 m-high plateau whose middle cell (0..2, 0..2) is removed.
    w.add_body(Body::keyed(1, Shape::HalfSpace, V(0.0, -2.0, 0.0), 0.0))
        .unwrap();
    let mut removed = vec![false; 4 * 4];
    removed[2 * 4 + 2] = true;
    let terrain = w
        .add_heightfield(
            HeightfieldDesc {
                origin: V(-4.0, 0.0, -4.0),
                cell_size: (2.0, 2.0),
                columns: 5,
                rows: 5,
                heights: vec![1.0; 25],
                removed_cells: Some(removed),
            },
            Some(2),
        )
        .unwrap();
    let down = |w: &World, x: f32, z: f32| {
        w.cast_ray_first(
            RayCast::new(V(x, 10.0, z), V(0.0, -20.0, 0.0)),
            QueryFilter::default(),
        )
        .unwrap()
        .map(|h| (h.body, 10.0 - 20.0 * h.fraction))
    };
    let (body, y) = down(&w, -1.0, -1.0).unwrap();
    assert_eq!(body, terrain);
    assert!((y - 1.0).abs() < 1e-4, "{y}");
    let (body, y) = down(&w, 1.0, 1.0).unwrap();
    assert_ne!(body, terrain, "the ray passes through the hole");
    assert!((y + 2.0).abs() < 1e-4, "{y}");

    // A character standing over the hole falls through it to the floor.
    let (half, radius) = (0.5, 0.3);
    let mut body = Body::keyed(10, Shape::capsule(half, radius), V(1.0, 3.0, 1.0), 1.0);
    body.external = true;
    let c = w.add_body(body).unwrap();
    let settings = w.character_settings();
    w.add_character(c, settings).unwrap();
    let mut out = Vec::new();
    for _ in 0..200 {
        w.move_characters(
            &[CharacterMove {
                body: c,
                desired: V(0.0, -0.05, 0.0),
            }],
            &mut out,
        )
        .unwrap();
    }
    let y = w.body(c).unwrap().position.1;
    assert!(
        (y - (-2.0 + half + radius + settings.offset)).abs() < 1e-4,
        "rests on the floor below the hole: {y}"
    );
    // Validation: the mask must cover every cell.
    assert!(
        w.add_heightfield(
            HeightfieldDesc {
                origin: V::ZERO,
                cell_size: (1.0, 1.0),
                columns: 3,
                rows: 3,
                heights: vec![0.0; 9],
                removed_cells: Some(vec![false; 3]),
            },
            None,
        )
        .is_err()
    );
}

/// World-space corners of a box body.
fn corners(b: &Body) -> Vec<V> {
    let Shape::Box(h) = b.shape else {
        unreachable!()
    };
    (0..8)
        .map(|k| {
            let sign = |bit: usize| if k & bit == 0 { -1.0 } else { 1.0 };
            b.position
                + b.orientation
                    .rotate(V(sign(1) * h.0, sign(2) * h.1, sign(4) * h.2))
        })
        .collect()
}

/// Lowest clearance of the box's corners above the terrain: negative below the surface.
fn clearance(w: &World, b: &Body) -> f32 {
    corners(b)
        .iter()
        .map(|c| c.1 - ground(w, c.0, c.2))
        .fold(f32::INFINITY, f32::min)
}

/// Crates and tumbling planks (MMORPG debris) land on the hills and come to rest on them.
#[test]
fn dynamic_boxes_land_and_rest_on_terrain() {
    let (mut w, _) = hills_with(V(0.0, -9.0, 0.0));
    let mut ids = Vec::new();
    for k in 0..12 {
        let (x, z) = (-20.0 + 3.5 * k as f32, 6.0 - 1.3 * k as f32);
        let half = if k % 2 == 0 {
            V(0.35, 0.35, 0.35)
        } else {
            V(0.33, 0.03, 0.33)
        };
        let mut b = Body::new(Shape::Box(half), V(x, ground(&w, x, z) + 2.0, z), 10.0);
        b.orientation =
            physics_engine::approximate::Quaternion(0.1 * k as f32, 0.3, 0.2, 0.9).normalized();
        b.velocity = V(1.2, 1.5, -0.6);
        b.angular_velocity = V(3.6, -2.4, 1.5);
        b.friction = 0.8;
        b.linear_damping = 0.2;
        b.angular_damping = 0.8;
        // Off the terrain-only ray layer, still colliding with the terrain.
        b.layers = physics_engine::CollisionLayers3d::new(2, u32::MAX);
        ids.push(w.add_body(b).unwrap());
    }
    for tick in 0..300 {
        w.step(1.0 / 30.0).unwrap();
        for &id in &ids {
            let b = w.body(id).unwrap();
            assert!(
                clearance(&w, b) > -0.02,
                "{id:?} below the surface at {tick}: {b:?}"
            );
        }
    }
    for &id in &ids {
        let b = w.body(id).unwrap();
        assert!(clearance(&w, b) < 0.02, "{id:?} floats: {b:?}");
        assert!(
            b.velocity.length() < 0.05 && b.angular_velocity.length() < 0.05,
            "{id:?} still moving: {b:?}"
        );
    }
}

#[test]
fn fast_box_stops_at_the_terrain_and_falls_through_holes() {
    let mut w = World::new(Config {
        gravity: V(0.0, -9.0, 0.0),
        ..Config::default()
    })
    .unwrap();
    w.add_body(Body::new(Shape::HalfSpace, V(0.0, -2.0, 0.0), 0.0))
        .unwrap();
    let mut removed = vec![false; 4 * 4];
    removed[2 * 4 + 2] = true;
    w.add_heightfield(
        HeightfieldDesc {
            origin: V(-4.0, 0.0, -4.0),
            cell_size: (2.0, 2.0),
            columns: 5,
            rows: 5,
            heights: vec![1.0; 25],
            removed_cells: Some(removed),
        },
        Some(2),
    )
    .unwrap();
    let mut fast = Body::new(Shape::Box(V(0.3, 0.3, 0.3)), V(-2.5, 20.0, -2.5), 1.0);
    fast.velocity = V(0.0, -300.0, 0.0);
    let fast = w.add_body(fast).unwrap();
    // Over the removed cell (0..2, 0..2): through the hole onto the floor.
    let hole = w
        .add_body(Body::new(
            Shape::Box(V(0.3, 0.3, 0.3)),
            V(1.0, 3.0, 1.0),
            1.0,
        ))
        .unwrap();
    let sphere = w
        .add_body(Body::new(Shape::Sphere(0.4), V(-2.5, 3.0, 2.5), 1.0))
        .unwrap();
    let capsule = w
        .add_body(Body::new(Shape::capsule(0.5, 0.2), V(2.5, 3.0, -2.5), 1.0))
        .unwrap();
    for _ in 0..120 {
        w.step(1.0 / 30.0).unwrap();
        assert!(
            w.body(fast).unwrap().position.1 > 1.29,
            "{:?}",
            w.body(fast)
        );
    }
    let y = |id| w.body(id).unwrap().position.1;
    assert!((y(fast) - 1.3).abs() < 0.01, "{}", y(fast));
    assert!((y(hole) + 1.7).abs() < 0.01, "{}", y(hole));
    assert!((y(sphere) - 1.4).abs() < 0.01, "{}", y(sphere));
    // The capsule tips over and lies on the plateau.
    assert!(y(capsule) > 1.15 && y(capsule) < 1.75, "{}", y(capsule));
    for id in [fast, hole, sphere, capsule] {
        assert!(
            w.body(id).unwrap().velocity.length() < 0.05,
            "{:?}",
            w.body(id)
        );
    }
}
