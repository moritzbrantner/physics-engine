use physics_engine::{
    BodyId, RigidBody, Vec3i, World as LegacyWorld, WorldConfig,
    approximate::{Body, Config, Convergence, Shape, Vector, World as FloatingWorld},
};

const TICKS_PER_SECOND: f64 = 60.0;
const POSITION_UNITS_PER_DISTANCE_UNIT: f64 = 1.0;
const EPSILON: f64 = 1e-9;

fn position_to_floating(value: Vec3i) -> Vector {
    Vector(
        f64::from(value.x) / POSITION_UNITS_PER_DISTANCE_UNIT,
        f64::from(value.y) / POSITION_UNITS_PER_DISTANCE_UNIT,
        f64::from(value.z) / POSITION_UNITS_PER_DISTANCE_UNIT,
    )
}

fn velocity_to_floating(value: Vec3i) -> Vector {
    position_to_floating(value) * TICKS_PER_SECOND
}

fn gravity_to_floating(value: Vec3i) -> Vector {
    position_to_floating(value) * (TICKS_PER_SECOND * TICKS_PER_SECOND)
}

fn assert_vector_close(actual: Vector, expected: Vector) {
    assert!(
        (actual.0 - expected.0).abs() <= EPSILON
            && (actual.1 - expected.1).abs() <= EPSILON
            && (actual.2 - expected.2).abs() <= EPSILON,
        "actual={actual:?}, expected={expected:?}"
    );
}

#[test]
fn one_substep_floating_integration_preserves_the_legacy_tick_conversion_boundary() {
    let id = BodyId(7);
    let gravity = Vec3i::new(0, -2, 1);
    let initial_position = Vec3i::new(10, 20, -30);
    let initial_velocity = Vec3i::new(3, 4, -2);
    let half_extents = Vec3i::new(1, 2, 3);

    let mut legacy = LegacyWorld::new(WorldConfig {
        gravity,
        ..WorldConfig::default()
    });
    legacy
        .add_body(RigidBody::dynamic(
            id,
            initial_position,
            initial_velocity,
            half_extents,
        ))
        .expect("valid legacy body");

    let mut floating = FloatingWorld::new(Config {
        gravity: gravity_to_floating(gravity),
        substeps: 1,
        velocity_iterations: 1,
        fixed_position_iterations: 0,
        warm_start: false,
        convergence: None::<Convergence>,
        ..Config::default()
    })
    .expect("valid floating config");
    let mut floating_body = Body::new(
        id,
        Shape::Box(position_to_floating(half_extents)),
        position_to_floating(initial_position),
        1.0,
    );
    floating_body.velocity = velocity_to_floating(initial_velocity);
    floating
        .add_body(floating_body)
        .expect("valid floating body");

    for _ in 0..4 {
        legacy.step(1).expect("legacy step");
        floating
            .step(1.0 / TICKS_PER_SECOND)
            .expect("floating step");

        let legacy_body = legacy.body(id).expect("legacy body remains");
        let floating_body = floating.body(id).expect("floating body remains");
        assert_vector_close(
            floating_body.position,
            position_to_floating(legacy_body.position()),
        );
        assert_vector_close(
            floating_body.velocity,
            velocity_to_floating(legacy_body.velocity()),
        );
    }
}

#[test]
fn floating_world_rejects_non_finite_timestep_instead_of_falling_back() {
    let mut world = FloatingWorld::new(Config::default()).expect("default config");
    assert!(world.step(f64::NAN).is_err());
}
