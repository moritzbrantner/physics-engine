//! Published capability claims must match real public calls on native and WASM.
use physics_engine::approximate::Real;
use std::collections::BTreeSet;

use physics_engine::{
    BodyId,
    approximate::{
        Body, CheckpointContext, Config, Error, PRIMITIVE_CAPABILITIES_JSON, Quaternion,
        QueryFilter, QueryPose, RayCast, Shape, ShapeCast, Vector as V, World,
    },
};
use serde_json::Value;

fn ledger() -> Value {
    serde_json::from_str(PRIMITIVE_CAPABILITIES_JSON).unwrap()
}
fn world() -> World {
    World::new(Config {
        gravity: V::ZERO,
        substeps: 1,
        ..Config::default()
    })
    .unwrap()
}
fn shape(kind: &str) -> Shape {
    match kind {
        "sphere" => Shape::Sphere(1.0),
        "box" => Shape::Box(V(1.0, 1.0, 1.0)),
        "capsule" => Shape::capsule(0.5, 1.0),
        "wedge" => Shape::wedge(V(1.0, 1.0, 1.0)),
        _ => panic!("ledger advertises an unsupported shape: {kind}"),
    }
}
// Exhaustive on the engine-owned enum: adding a kind requires revisiting this gate.
fn kind(shape: Shape) -> &'static str {
    match shape {
        Shape::Sphere(_) => "sphere",
        Shape::Box(_) => "box",
        Shape::Capsule { .. } => "capsule",
        Shape::Wedge(_) => "wedge",
    }
}

#[cfg_attr(test, test)]
pub fn advertised_shape_queries_and_mass_cross_the_public_api() {
    let data = ledger();
    assert_eq!(data["schemaVersion"], 1);
    assert_eq!(data["world"], "approximate::World");
    assert_eq!(data["scalar"], "f64");
    assert_eq!(data["worldApiStatus"], "limited/experimental");
    let capabilities = &data["capabilities"];
    for (key, value) in [
        ("mass", "public-uniform"),
        ("centerOfMass", "public-local"),
        ("inertia", "public-full-com"),
        ("overlaps", "public-snapshot"),
        ("rays", "public-finite-with-local-features"),
        ("shapeCasts", "public-translation"),
    ] {
        assert_eq!(capabilities[key], value);
    }
    let rows = data["shapes"].as_array().unwrap();
    assert_eq!(rows.len(), 4);
    let mut names = BTreeSet::new();
    for row in rows {
        let name = row["kind"].as_str().unwrap();
        assert!(names.insert(name));
        let shape = shape(name);
        assert_eq!(kind(shape), name);
        let properties = shape.mass_properties(2.0).unwrap();
        assert!(properties.volume > 0.0 && properties.volume.is_finite());
        assert!(
            properties
                .local_inertia
                .iter()
                .flatten()
                .all(|x| x.is_finite())
        );
        assert_eq!(properties.local_center_of_mass != V::ZERO, name == "wedge");
        let mut world = world();
        let mut body = Body::new(BodyId(1), shape, V::ZERO, 1.0);
        body.rotation_locked = name == "wedge";
        body.orientation = Quaternion(0.0, 0.0, (0.3 as Real).sin(), (0.3 as Real).cos());
        world.add_body(body).unwrap();
        world.add_force(BodyId(1), V::X).unwrap();
        let context = CheckpointContext {
            build: [183; 32],
            content: [1; 32],
        };
        let before = world.checkpoint(context).unwrap().to_bytes();
        let mut hits = Vec::new();
        world
            .overlap_shape(
                QueryPose::new(Shape::Sphere(0.1), V::ZERO),
                QueryFilter::default(),
                &mut hits,
            )
            .unwrap();
        assert_eq!(hits.len(), 1);
        world
            .cast_shape(
                ShapeCast::new(
                    QueryPose::new(Shape::Sphere(0.1), V(-10.0, 0.0, 0.0)),
                    V(20.0, 0.0, 0.0),
                ),
                QueryFilter::default(),
                &mut hits,
            )
            .unwrap();
        assert_eq!(hits.len(), 1);
        assert!(hits[0].fraction > 0.0 && hits[0].fraction < 1.0);
        let mut rays = Vec::new();
        world
            .cast_ray(
                RayCast::new(V(-10.0, 0.0, 0.0), V(20.0, 0.0, 0.0)),
                QueryFilter::default(),
                &mut rays,
            )
            .unwrap();
        assert_eq!(rays.len(), 1);
        assert!((rays[0].normal.length() - 1.0).abs() <= (1e-12 as Real).max(4.0 * Real::EPSILON));
        assert_eq!(world.checkpoint(context).unwrap().to_bytes(), before);
    }
}

#[cfg_attr(test, test)]
pub fn every_advertised_pair_executes_its_dispatch_in_both_orders() {
    let data = ledger();
    let shapes = data["shapes"].as_array().unwrap();
    let cells = data["pairs"].as_array().unwrap();
    assert_eq!(cells.len(), shapes.len() * (shapes.len() + 1) / 2);
    let mut keys = BTreeSet::new();
    let mut indexes = BTreeSet::new();
    for cell in cells {
        let left = cell["left"].as_str().unwrap();
        let right = cell["right"].as_str().unwrap();
        let index = usize::try_from(cell["dispatchIndex"].as_u64().unwrap()).unwrap();
        assert!(keys.insert((left, right)));
        assert!(indexes.insert(index));
        assert_eq!(cell["genericFallback"], "none");
        assert!(
            [
                "analytic-specialized",
                "fixed-topology-sat",
                "fixed-topology-feature"
            ]
            .contains(&cell["classification"].as_str().unwrap())
        );
        for (a, b) in [(left, right), (right, left)] {
            let mut world = world();
            let mut moving = Body::new(BodyId(1), shape(a), V::ZERO, 1.0);
            moving.rotation_locked = true;
            moving.sleep_allowed = false;
            world.add_body(moving).unwrap();
            world
                .add_body(Body::new(BodyId(2), shape(b), V(0.5, 0.0, 0.0), 0.0))
                .unwrap();
            let report = world.step(1.0 / 60.0).unwrap();
            assert!(report.contact_points > 0, "no physical contact for {a}/{b}");
            assert!(report.contact_points <= cell["manifoldPointLimit"].as_u64().unwrap());
            assert!(report.geometry.specialized_pair_dispatches[index] > 0);
            assert_eq!(
                report
                    .geometry
                    .specialized_pair_dispatches
                    .iter()
                    .filter(|count| **count > 0)
                    .count(),
                1
            );
            assert_eq!(report.geometry.generic_fallback_calls, 0);
            assert!(report.geometry.manifold_candidates > 0);
        }
    }
    // Completeness of the unordered keys, rather than ten possibly duplicated entries.
    for (i, a) in shapes.iter().enumerate() {
        for b in &shapes[i..] {
            assert!(keys.contains(&(a["kind"].as_str().unwrap(), b["kind"].as_str().unwrap())));
        }
    }
}

#[cfg_attr(test, test)]
pub fn restrictions_and_validation_remain_explicit() {
    let data = ledger();
    let capabilities = &data["capabilities"];
    assert_eq!(
        capabilities["translationCcd"],
        "fixed-orientation-per-substep"
    );
    assert_eq!(capabilities["rotationCcd"], "unsupported");
    assert_eq!(capabilities["chronologicalImpactResponse"], "missing");
    assert_eq!(capabilities["shapeCastFeatures"], "missing");
    assert_eq!(data["referenceAcceptance"], "partial");
    let validation = &data["validation"];
    assert_eq!(validation["finiteValuesRequired"], true);
    let minimum = validation["positiveDimensionMinimum"].as_f64().unwrap() as Real;
    let maximum = validation["dimensionMaximumExclusive"].as_f64().unwrap() as Real;
    for name in ["sphere", "box", "capsule", "wedge"] {
        let row = data["shapes"]
            .as_array()
            .unwrap()
            .iter()
            .find(|row| row["kind"] == name)
            .unwrap();
        assert_eq!(
            row["solverDynamicRotation"],
            if name == "wedge" {
                "rotation-locked"
            } else {
                "supported"
            }
        );
        assert_eq!(
            row["solverCom"],
            if name == "wedge" {
                "bounding-box-origin"
            } else {
                "geometric-com"
            }
        );
        assert_eq!(row["collapsedSkeleton"], name == "capsule");
        let mut moving = Body::new(BodyId(1), shape(name), V::ZERO, 1.0);
        let mut world = world();
        if name == "wedge" {
            assert_eq!(world.add_body(moving.clone()), Err(Error::InvalidInput));
            moving.rotation_locked = true;
        }
        world.add_body(moving).unwrap();
        for dimension in [0.0, minimum * 0.5, -1.0, Real::NAN, Real::INFINITY, maximum] {
            let shape = match shape(name) {
                Shape::Sphere(_) => Shape::Sphere(dimension),
                Shape::Box(_) => Shape::Box(V(dimension, minimum, minimum)),
                Shape::Capsule { .. } => Shape::capsule(0.0, dimension),
                Shape::Wedge(_) => Shape::wedge(V(dimension, minimum, minimum)),
            };
            let mut invalid = Body::new(BodyId(2), shape, V::ZERO, 0.0);
            invalid.rotation_locked = true;
            assert_eq!(world.add_body(invalid), Err(Error::InvalidInput));
            assert!(shape.mass_properties(1.0).is_err());
        }
    }
    let mut world = world();
    world
        .add_body(Body::new(
            BodyId(1),
            Shape::capsule(
                validation["capsuleHalfSegmentMinimum"].as_f64().unwrap() as Real,
                minimum,
            ),
            V::ZERO,
            validation["positiveMassMinimum"].as_f64().unwrap() as Real,
        ))
        .unwrap();
    assert_eq!(validation["fixedMass"], 0.0);
    let mass_maximum = validation["massMaximumExclusive"].as_f64().unwrap() as Real;
    for mass in [-1.0, Real::NAN, Real::INFINITY, mass_maximum] {
        assert_eq!(
            world.add_body(Body::new(BodyId(3), Shape::Sphere(1.0), V::ZERO, mass)),
            Err(Error::InvalidInput)
        );
    }
    for half in [-1.0, Real::NAN, maximum] {
        assert_eq!(
            world.add_body(Body::new(
                BodyId(2),
                Shape::capsule(half, minimum),
                V::ZERO,
                1.0
            )),
            Err(Error::InvalidInput)
        );
    }
}
