//! Per-body fluid medium (#525): buoyancy at the submerged centroid and implicit drag toward a
//! flow, against closed forms.
use physics_engine::BodyId;
use physics_engine::approximate::{
    Body, Config, Error, Medium, Quaternion, Shape, Vector as V, World,
};

const G: f32 = 9.81;
const WATER: f32 = 1000.0;

fn world() -> World {
    World::new(Config::default()).unwrap()
}

/// Still water with its surface at y = 0.
fn water(linear_drag: f32, angular_drag: f32) -> Medium {
    Medium {
        surface_point: V(0.0, 0.0, 0.0),
        surface_normal: V(0.0, 1.0, 0.0),
        density: WATER,
        flow: V(0.0, 0.0, 0.0),
        linear_drag,
        angular_drag,
    }
}

fn run(world: &mut World, seconds: f32, dt: f64) {
    for _ in 0..(f64::from(seconds) / dt).round() as u32 {
        world.step(dt).unwrap();
    }
}

/// A unit cube of `density`, rotation locked unless `free`, in `medium`.
fn add_cube(world: &mut World, y: f32, density: f32, medium: Medium, free: bool) -> BodyId {
    let mut body = Body::new(Shape::Box(V(0.5, 0.5, 0.5)), V(0.0, y, 0.0), density);
    body.rotation_locked = !free;
    let id = world.add_body(body).unwrap();
    world.set_medium(id, Some(medium)).unwrap();
    id
}

#[test]
fn a_box_floats_at_the_archimedes_depth() {
    let mut world = world();
    // Density 600 in water: 0.6 of a 1 m cube submerged, so the centre sits 0.1 below the surface.
    let id = add_cube(&mut world, 0.5, 600.0, water(3.0, 3.0), false);
    run(&mut world, 12.0, 1.0 / 60.0);
    let body = world.body(id).unwrap();
    assert!((body.position.1 + 0.1).abs() < 2e-3, "{:?}", body.position);
    assert!(body.velocity.length() < 0.05);
}

#[test]
fn a_sphere_floats_at_the_archimedes_depth() {
    let mut world = world();
    let radius = 0.5;
    let volume = 4.0 / 3.0 * std::f32::consts::PI * radius * radius * radius;
    let density = 300.0;
    let mut body = Body::new(Shape::Sphere(radius), V(0.0, 1.0, 0.0), density * volume);
    body.sleep_allowed = false;
    let id = world.add_body(body).unwrap();
    world.set_medium(id, Some(water(3.0, 3.0))).unwrap();
    run(&mut world, 12.0, 1.0 / 60.0);
    // The cap of height `d` displacing the body's mass: bisection on the closed form.
    let target = density / WATER * volume;
    let cap = |d: f32| std::f32::consts::PI * d * d * (3.0 * radius - d) / 3.0;
    let (mut lo, mut hi) = (0.0_f32, 2.0 * radius);
    for _ in 0..60 {
        let mid = 0.5 * (lo + hi);
        if cap(mid) < target {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    let centre = radius - 0.5 * (lo + hi);
    let y = world.body(id).unwrap().position.1;
    assert!((y - centre).abs() < 3e-3, "{y} vs {centre}");
}

#[test]
fn a_dense_sphere_sinks_at_its_terminal_velocity() {
    let mut world = world();
    let radius = 0.25;
    let volume = 4.0 / 3.0 * std::f32::consts::PI * radius * radius * radius;
    // Twice the water's density; `k` per second of drag at full submersion.
    let k = 4.0;
    let mut body = Body::new(
        Shape::Sphere(radius),
        V(0.0, -2.0, 0.0),
        2.0 * WATER * volume,
    );
    body.sleep_allowed = false;
    let id = world.add_body(body).unwrap();
    world.set_medium(id, Some(water(k, 0.0))).unwrap();
    run(&mut world, 6.0, 1.0 / 60.0);
    let expected = -G * 0.5 / k;
    let velocity = world.body(id).unwrap().velocity.1;
    assert!(
        (velocity - expected).abs() < 2e-3 * expected.abs(),
        "{velocity}"
    );
}

#[test]
fn a_floating_body_drifts_to_the_flow() {
    let mut world = world();
    let flow = V(1.5, 0.0, -0.5);
    let id = add_cube(
        &mut world,
        -0.1,
        600.0,
        Medium {
            flow,
            ..water(2.0, 2.0)
        },
        false,
    );
    run(&mut world, 10.0, 1.0 / 60.0);
    let v = world.body(id).unwrap().velocity;
    // Drag acts on the submerged 0.6: e^(-1.2 * 10) of the slip remains.
    assert!(
        (v.0 - flow.0).abs() < 2e-3 && (v.2 - flow.2).abs() < 2e-3,
        "{v:?}"
    );
}

#[test]
fn a_box_heaves_at_the_closed_form_period() {
    let mut world = world();
    // A unit cube of density 500: m = 500, waterplane 1 m², T = 2π sqrt(m / (ρ A g)).
    let period = 2.0 * std::f32::consts::PI * (500.0 / (WATER * G)).sqrt();
    let id = add_cube(&mut world, 0.05, 500.0, water(0.0, 0.0), false);
    let dt = 1.0 / 240.0;
    let mut crossings = Vec::new();
    let mut last = 0.0_f32;
    for tick in 0..2880 {
        world.step(dt).unwrap();
        let v = world.body(id).unwrap().velocity.1;
        if last < 0.0 && v >= 0.0 {
            crossings.push(tick as f32 * dt as f32);
        }
        last = v;
    }
    let measured = (crossings.last().unwrap() - crossings[0]) / (crossings.len() - 1) as f32;
    assert!(
        (measured - period).abs() < 0.03 * period,
        "{measured} vs {period}"
    );
}

/// A free cube's orientation after settling from a 3 degree tilt about Z: how well its local Y
/// still lines up with the vertical (one when it floats flat, 1/sqrt(2) when tipped 45 degrees).
fn settled_tilt(density: f32) -> f32 {
    let mut world = world();
    let tilt = 3.0_f32.to_radians();
    let mut body = Body::new(Shape::Box(V(0.5, 0.5, 0.5)), V(0.0, 0.0, 0.0), density);
    body.orientation = Quaternion(0.0, 0.0, (tilt * 0.5).sin(), (tilt * 0.5).cos());
    let id = world.add_body(body).unwrap();
    world.set_medium(id, Some(water(3.0, 3.0))).unwrap();
    run(&mut world, 25.0, 1.0 / 60.0);
    world
        .body(id)
        .unwrap()
        .orientation
        .rotate(V(0.0, 1.0, 0.0))
        .1
        .abs()
}

#[test]
fn a_cube_floats_flat_or_tipped_by_its_density_ratio() {
    // A cube's flat pose is stable at density ratios below ~0.21 or above ~0.79, and tips over
    // to float on an edge in between.
    assert!(settled_tilt(100.0) > 0.99, "light cube");
    assert!(settled_tilt(900.0) > 0.99, "dense cube");
    let tipped = settled_tilt(500.0);
    assert!((0.6..0.9).contains(&tipped), "half-density cube {tipped}");
}

#[test]
fn a_floating_body_sleeps_and_a_medium_change_wakes_it() {
    let mut world = world();
    let id = add_cube(&mut world, 0.2, 600.0, water(3.0, 3.0), false);
    run(&mut world, 15.0, 1.0 / 60.0);
    let body = world.body(id).unwrap();
    assert!(body.is_sleeping());
    assert!((body.position.1 + 0.1).abs() < 2e-3, "{:?}", body.position);
    assert!(world.is_quiescent());

    // The same medium changes nothing; another wakes it.
    world.set_medium(id, Some(water(3.0, 3.0))).unwrap();
    assert!(world.body(id).unwrap().is_sleeping());
    world
        .set_medium(
            id,
            Some(Medium {
                density: 500.0,
                ..water(3.0, 3.0)
            }),
        )
        .unwrap();
    assert!(!world.body(id).unwrap().is_sleeping());
    assert!(!world.is_quiescent());
    run(&mut world, 12.0, 1.0 / 60.0);
    // Density ratio 1.2: it sinks onward.
    assert!(world.body(id).unwrap().position.1 < -0.5);
}

#[test]
fn a_body_sinking_at_terminal_speed_does_not_sleep() {
    let mut world = world();
    let radius = 0.25;
    let volume = 4.0 / 3.0 * std::f32::consts::PI * radius * radius * radius;
    // Drag so strong that the terminal speed is below the sleep speed: forces balance only in
    // sum, so the body keeps sinking.
    let k = 400.0;
    let mut body = Body::new(
        Shape::Sphere(radius),
        V(0.0, -2.0, 0.0),
        2.0 * WATER * volume,
    );
    body.sleep_allowed = true;
    let id = world.add_body(body).unwrap();
    world.set_medium(id, Some(water(k, 0.0))).unwrap();
    run(&mut world, 4.0, 1.0 / 60.0);
    assert!(!world.body(id).unwrap().is_sleeping());
}

#[test]
fn out_of_range_media_are_rejected() {
    let mut world = world();
    let dynamic = world
        .add_body(Body::new(Shape::Sphere(0.5), V(0.0, 1.0, 0.0), 1.0))
        .unwrap();
    let fixed = world
        .add_body(Body::new(
            Shape::Box(V(1.0, 1.0, 1.0)),
            V(5.0, 0.0, 0.0),
            0.0,
        ))
        .unwrap();
    let good = water(1.0, 1.0);
    for bad in [
        Medium {
            surface_point: V(f32::NAN, 0.0, 0.0),
            ..good
        },
        Medium {
            surface_point: V(1e13, 0.0, 0.0),
            ..good
        },
        Medium {
            surface_normal: V(0.0, 0.0, 0.0),
            ..good
        },
        Medium {
            surface_normal: V(f32::INFINITY, 0.0, 0.0),
            ..good
        },
        Medium {
            density: -1.0,
            ..good
        },
        Medium {
            density: f32::NAN,
            ..good
        },
        Medium {
            flow: V(0.0, f32::INFINITY, 0.0),
            ..good
        },
        Medium {
            linear_drag: -0.5,
            ..good
        },
        Medium {
            angular_drag: f32::NAN,
            ..good
        },
        Medium {
            angular_drag: 2e6,
            ..good
        },
    ] {
        assert_eq!(
            world.set_medium(dynamic, Some(bad)),
            Err(Error::InvalidInput)
        );
    }
    assert_eq!(
        world.set_medium(fixed, Some(good)),
        Err(Error::InvalidInput)
    );
    assert_eq!(world.set_medium(fixed, None), Ok(()));
    assert_eq!(
        world.set_medium(BodyId::NONE, Some(good)),
        Err(Error::MissingBody(BodyId::NONE))
    );
}

#[cfg(feature = "checkpoints")]
#[test]
fn a_checkpoint_continues_a_body_in_a_medium() {
    use physics_engine::approximate::{CheckpointContext, CheckpointLimits};
    let context = CheckpointContext {
        build: [17; 32],
        content: [29; 32],
    };
    let mut original = world();
    let flow = V(0.5, 0.0, 0.0);
    let id = add_cube(
        &mut original,
        0.4,
        600.0,
        Medium {
            flow,
            ..water(2.0, 2.0)
        },
        true,
    );
    run(&mut original, 0.75, 1.0 / 60.0);
    let saved = original.checkpoint(context).unwrap().to_bytes();
    let mut restored = physics_engine::approximate::Checkpoint::from_bytes(
        &saved,
        context,
        CheckpointLimits::default(),
    )
    .unwrap()
    .restore(&[])
    .unwrap();
    for _ in 0..240 {
        let a = original.step(1.0 / 60.0).unwrap();
        let b = restored.step(1.0 / 60.0).unwrap();
        assert_eq!(a.substeps, b.substeps);
        let (a, b) = (original.body(id).unwrap(), restored.body(id).unwrap());
        assert_eq!(format!("{a:?}"), format!("{b:?}"));
    }
}
