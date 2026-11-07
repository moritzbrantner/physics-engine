//! Rigid spheres against heightfield terrain and fixed convex hulls (#519): resting, rolling,
//! striking authored rocks, and CCD spheres that never tunnel.
use physics_engine::approximate::{
    Body, Config, ConvexHullDesc, HeightfieldDesc, Shape, Vector as V, World,
};

#[path = "support/greyhaven_rock.rs"]
mod greyhaven_rock;

const N: usize = 21;
const CELL: f32 = 1.0;
/// Terrain falls along +X with this gradient until `FOOT`, then is flat.
const GRADE: f32 = 0.3;
const FOOT: f32 = 12.0;

fn slope_height(x: f32) -> f32 {
    GRADE * (FOOT - x.min(FOOT))
}

fn slope(world: &mut World) {
    let heights = (0..N * N)
        .map(|k| slope_height(CELL * (k % N) as f32))
        .collect();
    world
        .add_heightfield(
            HeightfieldDesc {
                origin: V(0.0, 0.0, -10.0),
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
}

fn world() -> World {
    World::new(Config {
        gravity: V(0.0, -9.81, 0.0),
        substeps: 2,
        velocity_iterations: 4,
        ..Config::default()
    })
    .unwrap()
}

fn ball(position: V, radius: f32) -> Body {
    let mass = 600.0 * 4.0 / 3.0 * std::f32::consts::PI * radius.powi(3);
    let mut body = Body::new(Shape::Sphere(radius), position, mass);
    body.friction = 0.8;
    body.restitution = 0.15;
    body.sleep_allowed = false;
    body
}

/// Height of the sphere's lowest point above the slope surface under its centre, along the
/// slope normal (exact for a planar region).
fn clearance(p: V, radius: f32) -> f32 {
    if p.0 < FOOT {
        let n = V(GRADE, 1.0, 0.0) * (1.0 + GRADE * GRADE).sqrt().recip();
        n.dot(p - V(p.0, slope_height(p.0), p.2)) - radius
    } else {
        p.1 - radius
    }
}

#[test]
fn a_sphere_rolls_down_terrain_without_sinking() {
    let mut w = world();
    slope(&mut w);
    let r = 0.5;
    let start = V(2.0, slope_height(2.0) + r * 1.05 + 0.2, 0.0);
    let id = w.add_body(ball(start, r)).unwrap();
    let mut lowest = f32::INFINITY;
    for _ in 0..120 {
        w.step(1.0 / 60.0).unwrap();
        let b = w.body(id).unwrap();
        if b.position.0 < FOOT - r {
            lowest = lowest.min(clearance(b.position, r));
        }
    }
    let b = w.body(id).unwrap();
    assert!(lowest > -0.01, "sank {lowest} m into the slope");
    assert!(
        b.position.0 > start.0 + 2.0,
        "did not roll downhill: {:?}",
        b.position
    );
    assert!(
        b.angular_velocity.2 < -0.5,
        "slides rather than rolls: {:?}",
        b.angular_velocity
    );
}

#[test]
fn a_sphere_strikes_an_authored_rock_and_does_not_pass_through() {
    let mut w = world();
    slope(&mut w);
    let rock = V(FOOT + 4.0, 1.1, 0.0);
    w.add_convex_hull(
        ConvexHullDesc {
            vertices: greyhaven_rock::vertices()
                .into_iter()
                .map(|p| V(p[0], p[1], p[2]))
                .collect(),
            position: rock,
            material: Default::default(),
        },
        Some(2),
    )
    .unwrap();
    let r = 0.5;
    let mut body = ball(V(FOOT, r + 0.01, 0.0), r);
    body.velocity = V(6.0, 0.0, 0.0);
    let id = w.add_body(body).unwrap();
    let mut furthest = f32::NEG_INFINITY;
    for _ in 0..120 {
        w.step(1.0 / 60.0).unwrap();
        furthest = furthest.max(w.body(id).unwrap().position.0);
    }
    // The proxy's -X face is at most 1.6 m from its centre: the ball stops short of it.
    assert!(
        furthest < rock.0 - 1.4 - r + 0.05,
        "entered the rock: {furthest}"
    );
    assert!(
        furthest > rock.0 - 2.0 - r,
        "never reached the rock: {furthest}"
    );
    assert!(w.body(id).unwrap().velocity.0 < 1.0);
}

#[test]
fn ccd_spheres_hit_terrain_and_hulls_instead_of_tunnelling() {
    for ccd in [false, true] {
        // Terrain: a small fast sphere dropped straight at the flat foot.
        let mut w = world();
        slope(&mut w);
        let mut body = ball(V(16.0, 3.0, 0.0), 0.1);
        body.velocity = V(0.0, -400.0, 0.0);
        body.ccd = ccd;
        let id = w.add_body(body).unwrap();
        let report = w.step(1.0 / 60.0).unwrap();
        assert_eq!(report.geometry.sweep_queries > 0, ccd);
        assert!(
            w.body(id).unwrap().velocity.1 >= 0.0,
            "ccd={ccd}: not stopped by terrain"
        );
        for _ in 0..30 {
            w.step(1.0 / 60.0).unwrap();
            let p = w.body(id).unwrap().position;
            assert!(p.1 > 0.05, "ccd={ccd}: tunnelled under the terrain: {p:?}");
        }
        // Hull: a small fast sphere fired through a cube rock.
        let mut w = World::new(Config {
            gravity: V::ZERO,
            ..Config::default()
        })
        .unwrap();
        let cube = [-1.0f32, 1.0]
            .into_iter()
            .flat_map(|x| {
                [-1.0f32, 1.0]
                    .into_iter()
                    .flat_map(move |y| [-1.0f32, 1.0].into_iter().map(move |z| V(x, y, z)))
            })
            .collect();
        w.add_convex_hull(
            ConvexHullDesc {
                vertices: cube,
                position: V::ZERO,
                material: Default::default(),
            },
            Some(500),
        )
        .unwrap();
        let mut body = ball(V(-3.0, 0.2, 0.1), 0.2);
        body.velocity = V(600.0, 0.0, 0.0);
        body.ccd = ccd;
        let id = w.add_body(body).unwrap();
        let report = w.step(1.0 / 60.0).unwrap();
        // A CCD sphere takes its swept time of impact; a non-CCD one a speculative contact.
        assert_eq!(report.geometry.sweep_queries > 0, ccd);
        let b = w.body(id).unwrap();
        assert!(
            b.velocity.0 <= 0.0,
            "ccd={ccd}: not stopped by the rock: {b:?}"
        );
        for _ in 0..10 {
            w.step(1.0 / 60.0).unwrap();
            let p = w.body(id).unwrap().position;
            assert!(
                p.0 < -1.15,
                "ccd={ccd}: passed into or through the rock: {p:?}"
            );
        }
    }
}

#[test]
fn a_sphere_resting_on_a_hull_face_and_inside_it_is_pushed_out_along_the_face() {
    let mut w = World::new(Config {
        gravity: V(0.0, -9.81, 0.0),
        ..Config::default()
    })
    .unwrap();
    let cube = [-1.0f32, 1.0]
        .into_iter()
        .flat_map(|x| {
            [-1.0f32, 1.0]
                .into_iter()
                .flat_map(move |y| [-1.0f32, 1.0].into_iter().map(move |z| V(x, y, z)))
        })
        .collect();
    w.add_convex_hull(
        ConvexHullDesc {
            vertices: cube,
            position: V::ZERO,
            material: Default::default(),
        },
        Some(500),
    )
    .unwrap();
    // Slightly embedded on the top face, off-centre.
    let id = w.add_body(ball(V(0.3, 1.45, -0.2), 0.5)).unwrap();
    for _ in 0..120 {
        w.step(1.0 / 60.0).unwrap();
    }
    let b = w.body(id).unwrap();
    assert!((b.position.1 - 1.5).abs() < 0.01, "{:?}", b.position);
    assert!((b.position.0 - 0.3).abs() < 0.01 && (b.position.2 + 0.2).abs() < 0.01);
}
