//! #465 experiment: regular icosahedron and dodecahedron visuals collided as their circumsphere
//! (`Shape::Sphere(circumradius)`), on the engine's existing sphere CCD. The fixtures here are
//! independent f64 references (`support/platonic.rs`): exact first contact of the real solid is
//! computed from the full separating-axis set of two convex polyhedra, never from frame-end
//! overlap. On identical one-step trajectories the proxy must never miss a real solid contact
//! and must touch no later than the solid; the early-contact bias is recorded and bounded.
//!
//! Run with `cargo test --test platonic_proxies -- --nocapture` to see the recorded biases.

#[path = "support/platonic.rs"]
mod platonic;

use physics_engine::approximate::{
    Body, Config, QueryFilter, QueryPose, Shape, ShapeCast, Vector, World,
};
use platonic::{Feature, Polyhedron, Quat, Rng, Solid, V3, length, scale};

const DT: f64 = 1.0 / 60.0;
const EDGES: [f64; 4] = [0.2, 0.01, 1.0, 100.0];

fn v(p: V3) -> Vector {
    Vector(p[0] as f32, p[1] as f32, p[2] as f32)
}

/// A world holding one static box (`half = None` → the `HalfSpace` floor at y = 0).
fn world(edge: f64, obstacle: Option<(V3, V3)>) -> World {
    let mut w = World::new(Config {
        gravity: Vector::ZERO,
        substeps: 1,
        // Scene units per metre: the 0.2 m-edge solid is the reference scale.
        length_unit: (edge / 0.2) as f32,
        ..Config::default()
    })
    .unwrap();
    let mut body = match obstacle {
        Some((center, half)) => Body::new(Shape::Box(v(half)), v(center), 0.0),
        None => Body::new(Shape::HalfSpace, Vector::ZERO, 0.0),
    };
    body.restitution = 1.0;
    body.friction = 0.0;
    w.add_body(body).unwrap();
    w
}

/// The engine's swept first contact of the proxy sphere (fraction of `displacement`).
fn proxy_toi(w: &World, radius: f64, start: V3, displacement: V3) -> Option<f64> {
    proxy_hit(w, radius, start, displacement).map(|(t, _)| t)
}

/// [`proxy_toi`] with the engine's outward contact normal.
fn proxy_hit(w: &World, radius: f64, start: V3, displacement: V3) -> Option<(f64, V3)> {
    let mut hits = Vec::new();
    w.cast_shape(
        ShapeCast::new(
            QueryPose::new(Shape::Sphere(radius as f32), v(start)),
            v(displacement),
        ),
        QueryFilter::default(),
        &mut hits,
    )
    .unwrap();
    hits.first().map(|h| {
        let n = [h.normal.0, h.normal.1, h.normal.2].map(f64::from);
        (f64::from(h.fraction), n)
    })
}

/// Angle in degrees between two unit normals.
fn angle(a: V3, b: V3) -> f64 {
    platonic::dot(a, b).clamp(-1.0, 1.0).acos().to_degrees()
}

/// One `World::step` of a CCD proxy body on the same trajectory; `true` when the step resolved
/// a contact with the obstacle (a swept impact or a started contact).
fn step_reports_contact(mut w: World, radius: f64, start: V3, displacement: V3, omega: V3) -> bool {
    let mut body = Body::new(Shape::Sphere(radius as f32), v(start), 1.0);
    body.velocity = v(scale(displacement, 1.0 / DT));
    body.angular_velocity = v(omega);
    body.restitution = 1.0;
    body.friction = 0.0;
    body.ccd = true;
    body.sleep_allowed = false;
    w.add_body(body).unwrap();
    let report = w.step(DT).unwrap();
    assert_eq!(report.geometry.ccd_budget_fallbacks, 0);
    report.swept_contacts > 0 || report.contact_events.iter().any(|e| e.started())
}

/// Scale-aware roundoff allowance in fractions of the displacement: f32 positions of magnitude
/// up to `|d| + R` carry about 6e-8 relative error per operation.
fn tolerance(radius: f64, displacement: V3) -> f64 {
    let d = length(displacement);
    2e-6 * (d + 4.0 * radius) / d
}

struct Record {
    solid: Solid,
    edge: f64,
    case: &'static str,
    exact: Option<f64>,
    proxy: Option<f64>,
    /// Early-contact bias along the path, in units of the circumradius.
    bias: f64,
}

fn print(records: &[Record]) {
    for r in records {
        eprintln!(
            "platonic-proxy {:?} edge={} case={} exact={:?} proxy={:?} bias/R={:.4}",
            r.solid, r.edge, r.case, r.exact, r.proxy, r.bias
        );
    }
}

/// Checks the sphere contract on one trajectory and returns the bias record.
#[allow(clippy::too_many_arguments)]
fn compare(
    poly: &Polyhedron,
    case: &'static str,
    exact: Option<f64>,
    obstacle: Option<(V3, V3)>,
    start: V3,
    displacement: V3,
    omega: V3,
    step: bool,
) -> Record {
    let radius = poly.circumradius();
    let w = world(poly.edge, obstacle);
    let proxy = proxy_toi(&w, radius, start, displacement);
    let tol = tolerance(radius, displacement);
    if let Some(exact) = exact {
        let proxy = proxy.unwrap_or_else(|| {
            panic!(
                "{:?} edge {} {case}: proxy missed a solid contact at {exact}",
                poly.solid, poly.edge
            )
        });
        assert!(
            proxy <= exact + tol,
            "{:?} edge {} {case}: proxy {proxy} later than solid {exact}",
            poly.solid,
            poly.edge
        );
        if step {
            assert!(
                step_reports_contact(w, radius, start, displacement, omega),
                "{:?} edge {} {case}: the CCD step missed the solid's contact at {exact}",
                poly.solid,
                poly.edge
            );
        }
    }
    let bias = match (exact, proxy) {
        (Some(e), Some(p)) => (e - p) * length(displacement) / radius,
        _ => f64::NAN,
    };
    Record {
        solid: poly.solid,
        edge: poly.edge,
        case,
        exact,
        proxy,
        bias,
    }
}

#[test]
fn fixtures_match_the_closed_forms() {
    for solid in Solid::ALL {
        for edge in [1.0, 0.2, 0.01, 0.37, 100.0] {
            let p = Polyhedron::new(solid, edge);
            let (vertices, faces) = match solid {
                Solid::Icosahedron => (12, 20),
                Solid::Dodecahedron => (20, 12),
            };
            assert_eq!(p.vertices.len(), vertices);
            assert_eq!(p.normals.len(), faces);
            assert_eq!(solid.faces(), faces);
            assert_eq!(p.edges.len(), 30, "{solid:?}");
            let rel = |a: f64, b: f64| (a - b).abs() <= 1e-12 * b;
            for &x in &p.vertices {
                assert!(rel(length(x), solid.circumradius(edge)), "{solid:?} {edge}");
            }
            for &d in &p.plane_offsets {
                assert!(rel(d, solid.inradius(edge)), "{solid:?} {edge}: {d}");
            }
            assert!(rel(p.circumradius(), solid.circumradius(edge)));
            assert!(rel(p.inradius(), solid.inradius(edge)));
        }
    }
    // Unit-edge closed forms, independently: icosahedron R = sin(2π/5), r = √3/12·(3 + √5);
    // dodecahedron R = √3/4·(1 + √5), r = ½·√(5/2 + 11/10·√5).
    let ico = Polyhedron::new(Solid::Icosahedron, 1.0);
    assert!((ico.circumradius() - (2.0 * std::f64::consts::PI / 5.0).sin()).abs() < 1e-14);
    assert!((ico.circumradius() - 0.951_056_516_295_153_6).abs() < 1e-14);
    assert!((ico.inradius() - 0.755_761_314_076_170_7).abs() < 1e-14);
    let dodeca = Polyhedron::new(Solid::Dodecahedron, 1.0);
    assert!((dodeca.circumradius() - 1.401_258_538_444_073_5).abs() < 1e-14);
    assert!((dodeca.inradius() - 1.113_516_364_411_606_7).abs() < 1e-14);
    // The circumsphere proxy's analytic early-contact bound (R − r) / R.
    assert!((1.0 - ico.inradius() / ico.circumradius() - 0.205_345).abs() < 1e-6);
    assert!((1.0 - dodeca.inradius() / dodeca.circumradius() - 0.205_345).abs() < 1e-6);
}

/// Head-on into a thin wall: the face, edge and vertex leading, plus 64 random orientations.
/// The solid's contact gap is its support distance `h ∈ [r, R]`, so the proxy is early by
/// exactly `R − h`: `R − r` face-first and zero vertex-first.
#[test]
fn head_on_face_edge_vertex_and_random_orientations() {
    let mut records = Vec::new();
    for solid in Solid::ALL {
        for edge in EDGES {
            let poly = Polyhedron::new(solid, edge);
            let (big_r, small_r) = (poly.circumradius(), poly.inradius());
            let wall = (
                [3.05 * big_r, 0.0, 0.0],
                [0.05 * big_r, 4.0 * big_r, 4.0 * big_r],
            );
            let d = [10.0 * big_r, 0.0, 0.0];
            let mut rng = Rng(0x465 + edge.to_bits());
            let mut orientations: Vec<(&'static str, Quat)> = vec![
                ("face", poly.leading(Feature::Face, [1.0, 0.0, 0.0])),
                ("edge", poly.leading(Feature::Edge, [1.0, 0.0, 0.0])),
                ("vertex", poly.leading(Feature::Vertex, [1.0, 0.0, 0.0])),
            ];
            orientations.extend((0..64).map(|_| ("random", rng.rotation())));
            for (case, q) in orientations {
                let exact = poly.box_toi(q, [0.0; 3], d, wall.0, wall.1);
                let h = poly.support(q, [1.0, 0.0, 0.0]);
                let expected = (3.0 * big_r - h) / length(d);
                let exact_value = exact.expect("the solid reaches the wall");
                assert!((exact_value - expected).abs() < 1e-12, "{solid:?} {case}");
                let record = compare(&poly, case, exact, Some(wall), [0.0; 3], d, [0.0; 3], true);
                // Head-on, both normals are the wall's face normal.
                let (_, solid_normal) = poly.box_contact(q, [0.0; 3], d, wall.0, wall.1).unwrap();
                let (_, proxy_normal) =
                    proxy_hit(&world(edge, Some(wall)), big_r, [0.0; 3], d).unwrap();
                assert!(
                    angle(solid_normal, [-1.0, 0.0, 0.0]) < 1e-6,
                    "{solid_normal:?}"
                );
                assert!(
                    angle(proxy_normal, [-1.0, 0.0, 0.0]) < 0.01,
                    "{proxy_normal:?}"
                );
                // Bias equals R − h, within [0, R − r].
                let tol = tolerance(big_r, d) * length(d) / big_r;
                assert!(
                    (record.bias - (big_r - h) / big_r).abs() <= tol
                        && record.bias <= (big_r - small_r) / big_r + tol,
                    "{solid:?} {case}: bias {}",
                    record.bias
                );
                if case != "random" {
                    records.push(record);
                } else if records.last().is_none_or(|r| r.case != "random-worst") {
                    records.push(Record {
                        case: "random-worst",
                        ..record
                    });
                } else if let Some(last) = records.last_mut()
                    && record.bias > last.bias
                {
                    *last = Record {
                        case: "random-worst",
                        ..record
                    };
                }
            }
        }
    }
    print(&records);
}

/// Passing over a box's top edge at heights from inside the insphere band to beyond the
/// circumsphere. Wherever the solid touches, the proxy does too and no later; the proxy's
/// phantom contacts (solid passes clear) are the approximation's cost and are counted.
#[test]
fn grazing_edges_never_miss_and_phantoms_are_bounded() {
    let mut records = Vec::new();
    for solid in Solid::ALL {
        for edge in EDGES {
            let poly = Polyhedron::new(solid, edge);
            let (big_r, small_r) = (poly.circumradius(), poly.inradius());
            let block = (
                [3.5 * big_r, -2.0 * big_r, 0.0],
                [0.5 * big_r, 2.0 * big_r, 2.0 * big_r],
            );
            let d = [10.0 * big_r, 0.0, 0.0];
            let mut rng = Rng(0x6ea2 ^ edge.to_bits());
            let (mut hits, mut phantoms, mut misses_beyond) = (0, 0, 0);
            let mut worst: Option<Record> = None;
            let (mut normal_max, mut normal_sum) = (0.0f64, 0.0f64);
            let w = world(edge, Some(block));
            for _ in 0..8 {
                let q = rng.rotation();
                for k in 0..=40 {
                    let y = small_r * 0.9 + (big_r * 1.1 - small_r * 0.9) * k as f64 / 40.0;
                    let start = [0.0, y, 0.0];
                    let exact = poly.box_toi(q, start, d, block.0, block.1);
                    // Step-check only contacts the solid makes with a clear margin; exact
                    // tangency is a measure-zero event for both shapes.
                    let clear =
                        exact.is_some() && y < -poly.support(q, [0.0, -1.0, 0.0]) - 1e-4 * big_r;
                    let record = compare(
                        &poly,
                        "grazing",
                        exact,
                        Some(block),
                        start,
                        d,
                        [0.0; 3],
                        clear,
                    );
                    if let (Some((_, solid)), Some((_, proxy))) = (
                        poly.box_contact(q, start, d, block.0, block.1),
                        proxy_hit(&w, big_r, start, d),
                    ) {
                        let a = angle(solid, proxy);
                        normal_max = normal_max.max(a);
                        normal_sum += a;
                    }
                    match (record.exact, record.proxy) {
                        (Some(_), _) => hits += 1,
                        (None, Some(_)) => phantoms += 1,
                        (None, None) => {}
                    }
                    if y > big_r * (1.0 + 1e-5) {
                        assert!(record.proxy.is_none(), "beyond the circumsphere");
                        misses_beyond += 1;
                    }
                    if record.bias.is_finite()
                        && worst.as_ref().is_none_or(|w| record.bias > w.bias)
                    {
                        worst = Some(record);
                    }
                }
            }
            assert!(hits > 0 && misses_beyond > 0);
            eprintln!(
                "platonic-proxy {solid:?} edge={edge} grazing samples=328 solid-hits={hits} phantom={phantoms} \
                 normal deviation mean={:.1}° max={:.1}°",
                normal_sum / f64::from(hits),
                normal_max
            );
            records.extend(worst);
        }
    }
    print(&records);
}

/// The visual spins fast (any orientation is inside the circumsphere at every instant) while
/// the body flies into a thin wall; the solid's first contact under rotation is found by dense
/// sampling and bisection of the exact static overlap.
#[test]
fn rotating_visual_never_reaches_a_wall_before_its_proxy() {
    let mut records = Vec::new();
    for solid in Solid::ALL {
        for edge in EDGES {
            let poly = Polyhedron::new(solid, edge);
            let (big_r, small_r) = (poly.circumradius(), poly.inradius());
            let wall = (
                [3.05 * big_r, 0.0, 0.0],
                [0.05 * big_r, 4.0 * big_r, 4.0 * big_r],
            );
            let d = [10.0 * big_r, 0.0, 0.0];
            let mut rng = Rng(0x0707 ^ edge.to_bits());
            for spin in [30.0, 300.0, 3000.0] {
                let q0 = rng.rotation();
                let omega = scale([1.0, 2.0, -3.0], spin / 14f64.sqrt());
                let exact = poly
                    .rotating_box_toi(
                        q0,
                        omega,
                        [0.0; 3],
                        scale(d, 1.0 / DT),
                        DT,
                        wall.0,
                        wall.1,
                        4096,
                    )
                    .map(|t| t / DT);
                assert!(exact.is_some());
                let record = compare(
                    &poly,
                    "rotating",
                    exact,
                    Some(wall),
                    [0.0; 3],
                    d,
                    omega,
                    true,
                );
                let tol = tolerance(big_r, d) * length(d) / big_r;
                assert!(record.bias >= -tol && record.bias <= (big_r - small_r) / big_r + tol);
                records.push(record);
            }
        }
    }
    print(&records);
}

/// High speed through a wall far thinner than a step's displacement (200 R per step, a wall
/// 0.004 R thick), and onto the `HalfSpace` floor: frame-end overlap would see nothing.
#[test]
fn high_speed_thin_wall_and_floor() {
    let mut records = Vec::new();
    for solid in Solid::ALL {
        for edge in EDGES {
            let poly = Polyhedron::new(solid, edge);
            let (big_r, small_r) = (poly.circumradius(), poly.inradius());
            let wall = (
                [3.002 * big_r, 0.0, 0.0],
                [0.002 * big_r, 4.0 * big_r, 4.0 * big_r],
            );
            let d = [200.0 * big_r, 0.0, 0.0];
            let mut rng = Rng(0xfa57 ^ edge.to_bits());
            for _ in 0..16 {
                let q = rng.rotation();
                let exact = poly.box_toi(q, [0.0; 3], d, wall.0, wall.1);
                // Neither end pose overlaps the wall: only a sweep sees this contact.
                assert!(!poly.overlaps_box(q, d, wall.0, wall.1));
                let record = compare(
                    &poly,
                    "thin-wall",
                    exact,
                    Some(wall),
                    [0.0; 3],
                    d,
                    [0.0; 3],
                    true,
                );
                let tol = tolerance(big_r, d) * length(d) / big_r;
                assert!(record.bias <= (big_r - small_r) / big_r + tol);
                records.push(record);
            }
            // Falling onto the floor plane at 200 R per step, reference: a huge box under y = 0.
            let floor = ([0.0, -1e4 * big_r, 0.0], [1e4 * big_r; 3]);
            let start = [0.0, 3.0 * big_r, 0.0];
            let down = [0.0, -200.0 * big_r, 0.0];
            for _ in 0..16 {
                let q = rng.rotation();
                let exact = poly.box_toi(q, start, down, floor.0, floor.1);
                let record = compare(&poly, "floor", exact, None, start, down, [0.0; 3], true);
                let tol = tolerance(big_r, down) * length(down) / big_r;
                assert!(
                    record.bias <= (big_r - small_r) / big_r + tol,
                    "{}",
                    record.bias
                );
                records.push(record);
            }
        }
    }
    let worst = |case: &str| {
        records
            .iter()
            .filter(|r| r.case == case)
            .map(|r| r.bias)
            .fold(0.0, f64::max)
    };
    eprintln!(
        "platonic-proxy thin-wall worst bias/R={:.4}, floor worst bias/R={:.4}",
        worst("thin-wall"),
        worst("floor")
    );
}
