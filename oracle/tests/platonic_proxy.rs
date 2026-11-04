//! #465 experiment evidence: the `ccd-platonic-proxy` approximation report against Rapier's
//! convex hull of the real solid, and an advisory whole-tick scaling run of circumsphere proxies
//! on both engines.
//!
//! ```text
//! cargo test --release --locked -p physics-oracle --test platonic_proxy -- --nocapture
//! cargo test --release --locked -p physics-oracle --test platonic_proxy -- --ignored --nocapture
//! ```

use physics_oracle::{Rng, scenarios::platonic_proxy::hull_report};
use std::time::Instant;

#[test]
fn circumsphere_error_against_the_rapier_convex_hull_is_the_analytic_bias() {
    let report = hull_report().unwrap();
    for (lane, e) in report.iter().enumerate() {
        println!(
            "platonic-proxy lane={lane} R={:.4} r={:.4} h={:.4} early={:.1}µs \
             max |hull−proxy|={:.4} m (analytic 2(R−h)={:.4}, bound 2(R−r)={:.4}); hull residual {:.4} m",
            e.circumradius,
            e.inradius,
            e.support,
            e.early * 1e6,
            e.error,
            e.expected,
            e.bound,
            e.hull_residual
        );
        assert!((e.error - e.expected).abs() < 0.035);
    }
}

const DT: f32 = 1.0 / 60.0;
const TICKS: u32 = 60;
const DENSITY: f32 = 600.0;
const ICO_EDGE: f32 = 0.2;
const DODECA_EDGE: f32 = 0.14;

/// Alternating icosahedron/dodecahedron proxies on a cubic lattice inside six static walls.
struct Layout {
    radii: Vec<f32>,
    positions: Vec<[f32; 3]>,
    velocities: Vec<[f32; 3]>,
    /// Wall boxes (centre, half extents).
    walls: Vec<([f32; 3], [f32; 3])>,
}

fn circumradius(k: usize) -> f32 {
    if k.is_multiple_of(2) {
        ICO_EDGE / 4.0 * (10.0 + 2.0 * 5f32.sqrt()).sqrt()
    } else {
        DODECA_EDGE * 3f32.sqrt() / 4.0 * (1.0 + 5f32.sqrt())
    }
}

fn layout(count: usize, dense: bool) -> Layout {
    let spacing = if dense { 0.42 } else { 2.0 };
    let speed = if dense { 10.0 } else { 30.0 };
    let side = (count as f32).cbrt().ceil() as usize;
    let extent = side as f32 * spacing;
    let mut rng = Rng(0x465 + count as u64 + u64::from(dense));
    let mut radii = Vec::new();
    let mut positions = Vec::new();
    let mut velocities = Vec::new();
    for k in 0..count {
        let (i, j, l) = (k % side, (k / side) % side, k / (side * side));
        radii.push(circumradius(k));
        positions.push([
            (i as f32 + 0.5) * spacing - extent / 2.0,
            (j as f32 + 0.5) * spacing - extent / 2.0,
            (l as f32 + 0.5) * spacing - extent / 2.0,
        ]);
        let d = [
            rng.range(-1.0, 1.0),
            rng.range(-1.0, 1.0),
            rng.range(-1.0, 1.0),
        ];
        let n = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt().max(1e-3);
        velocities.push(d.map(|x| x / n * speed));
    }
    let h = extent / 2.0 + 0.05;
    let t = 0.05;
    let walls = (0..3)
        .flat_map(|axis| {
            [-1.0, 1.0].map(|sign| {
                let mut c = [0.0; 3];
                c[axis] = sign * h;
                let mut half = [h + t; 3];
                half[axis] = t;
                (c, half)
            })
        })
        .collect();
    Layout {
        radii,
        positions,
        velocities,
        walls,
    }
}

#[derive(Default)]
struct Work {
    pairs: u64,
    narrows: u64,
    sweeps: u64,
    impacts: u64,
    impulse_iterations: u64,
    fallbacks: u64,
    cached_pairs_peak: u64,
    retained_bytes: u64,
}

fn engine(layout: &Layout, ccd: bool) -> (f64, Work) {
    use physics_engine::approximate::{Body, Config, Shape, Vector, World};
    let mut w = World::new(Config {
        gravity: Vector::ZERO,
        substeps: 1,
        velocity_iterations: 8,
        ..Config::default()
    })
    .unwrap();
    for &(c, h) in &layout.walls {
        let mut wall = Body::new(
            Shape::Box(Vector(h[0], h[1], h[2])),
            Vector(c[0], c[1], c[2]),
            0.0,
        );
        wall.restitution = 0.8;
        w.add_body(wall).unwrap();
    }
    for k in 0..layout.radii.len() {
        let r = layout.radii[k];
        let p = layout.positions[k];
        let v = layout.velocities[k];
        let mut b = Body::new(
            Shape::Sphere(r),
            Vector(p[0], p[1], p[2]),
            4.0 / 3.0 * std::f32::consts::PI * r * r * r * DENSITY,
        );
        b.velocity = Vector(v[0], v[1], v[2]);
        b.restitution = 0.8;
        b.ccd = ccd;
        w.add_body(b).unwrap();
    }
    let mut work = Work::default();
    let start = Instant::now();
    for _ in 0..TICKS {
        let r = w.step(f64::from(DT)).unwrap();
        work.pairs += r.pair_tests;
        work.narrows += r.narrow_tests;
        work.sweeps += r.geometry.sweep_queries;
        work.impacts += r.swept_contacts;
        work.impulse_iterations += r.impulse_iterations;
        work.fallbacks += r.geometry.ccd_budget_fallbacks;
        work.cached_pairs_peak = work.cached_pairs_peak.max(r.geometry.cached_pairs_peak);
        work.retained_bytes = work.retained_bytes.max(r.geometry.retained_bytes);
    }
    let us = start.elapsed().as_secs_f64() * 1e6 / f64::from(TICKS);
    std::hint::black_box(&w);
    (us, work)
}

fn rapier(layout: &Layout, hull: bool) -> f64 {
    use rapier3d::prelude::*;
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::ZERO;
    world.integration_parameters.dt = DT;
    for &(c, h) in &layout.walls {
        world.colliders.insert(
            ColliderBuilder::cuboid(h[0], h[1], h[2])
                .translation(Vector::new(c[0], c[1], c[2]))
                .restitution(0.8)
                .build(),
        );
    }
    let ico = solid_points(true);
    let dodeca = solid_points(false);
    for k in 0..layout.radii.len() {
        let p = layout.positions[k];
        let v = layout.velocities[k];
        let id = world.bodies.insert(
            RigidBodyBuilder::dynamic()
                .translation(Vector::new(p[0], p[1], p[2]))
                .linvel(Vector::new(v[0], v[1], v[2]))
                .ccd_enabled(true)
                .build(),
        );
        let collider = if hull {
            ColliderBuilder::convex_hull(if k.is_multiple_of(2) { &ico } else { &dodeca }).unwrap()
        } else {
            ColliderBuilder::ball(layout.radii[k])
        };
        world.colliders.insert_with_parent(
            collider.density(DENSITY).restitution(0.8).build(),
            id,
            &mut world.bodies,
        );
    }
    let start = Instant::now();
    for _ in 0..TICKS {
        world.step();
    }
    let us = start.elapsed().as_secs_f64() * 1e6 / f64::from(TICKS);
    std::hint::black_box(&world);
    us
}

fn solid_points(icosahedron: bool) -> Vec<rapier3d::prelude::Vector> {
    let phi = (1.0 + 5f32.sqrt()) / 2.0;
    let mut raw: Vec<[f32; 3]> = Vec::new();
    let cyclic = |v: [f32; 3], out: &mut Vec<[f32; 3]>| {
        for p in [v, [v[2], v[0], v[1]], [v[1], v[2], v[0]]] {
            for s in 0..8 {
                let q = [
                    if s & 1 == 0 { p[0] } else { -p[0] },
                    if s & 2 == 0 { p[1] } else { -p[1] },
                    if s & 4 == 0 { p[2] } else { -p[2] },
                ];
                if !out.contains(&q) {
                    out.push(q);
                }
            }
        }
    };
    let scale = if icosahedron {
        cyclic([0.0, 1.0, phi], &mut raw);
        ICO_EDGE / 2.0
    } else {
        cyclic([1.0, 1.0, 1.0], &mut raw);
        cyclic([0.0, 1.0 / phi, phi], &mut raw);
        DODECA_EDGE * phi / 2.0
    };
    raw.iter()
        .map(|v| rapier3d::prelude::Vector::new(v[0] * scale, v[1] * scale, v[2] * scale))
        .collect()
}

fn median(mut v: Vec<f64>) -> f64 {
    v.sort_by(f64::total_cmp);
    v[v.len() / 2]
}

/// Advisory whole-tick cost (60 ticks, world construction excluded, median of 5) and engine
/// candidate work per tick for 32/256/1,024 proxies, sparse (2 m lattice, 30 m/s) and dense
/// (0.42 m lattice ≈ 2.1 R, 10 m/s), all CCD, in six static walls. Rapier runs the same
/// layout with balls of the same radii and masses (parity) and with free-rotating convex hulls
/// of the real solids (what the proxy replaces); the engine also runs once without `ccd` to
/// separate the CCD scheduler's cost. Shared-runner timings are not a CI gate.
#[test]
#[ignore = "advisory scaling timing; run explicitly with --ignored --nocapture"]
fn print_proxy_scaling_against_rapier() {
    for dense in [false, true] {
        for count in [32, 256, 1024] {
            let l = layout(count, dense);
            let samples = 5;
            let mut work = Work::default();
            let e = median(
                (0..samples)
                    .map(|_| {
                        let (us, w) = engine(&l, true);
                        work = w;
                        us
                    })
                    .collect(),
            );
            let discrete = median((0..samples).map(|_| engine(&l, false).0).collect());
            let rb = median((0..samples).map(|_| rapier(&l, false)).collect());
            let rh = median((0..samples).map(|_| rapier(&l, true)).collect());
            let t = u64::from(TICKS);
            println!(
                "platonic-proxy-scaling {} count={count} engine_sphere_us={e:.1} engine_no_ccd_us={discrete:.1} rapier_ball_us={rb:.1} \
                 rapier_hull_us={rh:.1} engine/ball={:.2} engine/hull={:.2} per-tick: pairs={} narrow={} \
                 sweeps={} impacts={} impulse_iterations={} fallbacks={} cached_pairs_peak={} retained_bytes={}",
                if dense { "dense" } else { "sparse" },
                e / rb,
                e / rh,
                work.pairs / t,
                work.narrows / t,
                work.sweeps / t,
                work.impacts / t,
                work.impulse_iterations / t,
                work.fallbacks,
                work.cached_pairs_peak,
                work.retained_bytes,
            );
        }
    }
}
