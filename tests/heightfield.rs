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
    let mut w = World::new(Config {
        gravity: V::ZERO,
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
                material: Default::default(),
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
                exclude: None,
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
    // Stepping a world with terrain and a dynamic body does not touch the terrain.
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
            material: Default::default(),
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
            exclude: None,
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
                material: Default::default(),
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
                exclude: None,
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
                material: Default::default(),
            },
            None,
        )
        .is_err()
    );
}

/// Terrain with a 1 m cliff of `cell`-wide cells rising toward -X and toward +Z from a floor:
/// a concave corner at the origin.
fn cliff_corner(cell: f32) -> World {
    let mut w = World::new(Config {
        gravity: V::ZERO,
        ..Config::default()
    })
    .unwrap();
    let n = (16.0 / cell) as usize + 1;
    let origin = V(-8.0, 0.0, -8.0);
    let heights = (0..n * n)
        .map(|k| {
            let (x, z) = (
                origin.0 + cell * (k % n) as f32,
                origin.2 + cell * (k / n) as f32,
            );
            if x < -1e-4 || z > 1e-4 { 1.0 } else { 0.0 }
        })
        .collect();
    w.add_heightfield(
        HeightfieldDesc {
            origin,
            cell_size: (cell, cell),
            columns: n,
            rows: n,
            heights,
            removed_cells: None,
            material: Default::default(),
        },
        Some(1),
    )
    .unwrap();
    w
}

/// #396: a character walking into the foot of a steep heightfield cliff (and into the corner
/// where two cliffs meet) stays on the floor and never ends inside the terrain.
#[test]
fn walking_into_a_steep_heightfield_cliff_never_sinks_into_it() {
    let (half, radius) = (0.45, 0.35);
    for cell in [0.5, 0.35, 0.25] {
        for (dx, dz, z0) in [
            (-1.0, 0.0, -2.0),
            (-1.0, 0.0, -0.3),
            (-1.0, 0.0, -0.1),
            (-1.0, 1.0, -2.0),
            (-0.3, 1.0, -2.0),
            (0.0, 1.0, -2.0),
        ] {
            let mut w = cliff_corner(cell);
            let start = V(2.0, half + radius + 0.01, z0);
            let mut body = Body::keyed(10, Shape::capsule(half, radius), start, 1.0);
            body.external = true;
            body.layers = physics_engine::CollisionLayers3d::new(2, u32::MAX);
            let c = w.add_body(body).unwrap();
            let settings = w.character_settings();
            w.add_character(c, settings).unwrap();
            let mut out = Vec::new();
            let mut vy = 0.0;
            let speed = 1.5 / 60.0 / f32::hypot(dx, dz);
            for tick in 0..300 {
                vy -= 9.81 / 60.0;
                w.move_characters(
                    &[CharacterMove {
                        body: c,
                        desired: V(dx * speed, vy / 60.0, dz * speed),
                        exclude: None,
                    }],
                    &mut out,
                )
                .unwrap();
                if out[0].grounded {
                    vy = 0.0;
                }
                let p = w.body(c).unwrap().position;
                let foot = p.1 - half - radius;
                assert!(
                    foot > ground(&w, p.0, p.2) - 2e-3,
                    "cell {cell}, walk ({dx}, {dz}) from z {z0}, tick {tick}: sank into the \
                     terrain at {p:?} (ground {})",
                    ground(&w, p.0, p.2)
                );
            }
        }
    }
}
