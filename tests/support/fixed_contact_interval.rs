//! Captured fixed-floor interval misses, through the public stepping/checkpoint seam.
use physics_engine::{
    BodyId, CollisionLayers3d,
    approximate::{
        Body, Checkpoint, CheckpointContext, CheckpointLimits, Config, Quaternion as Q, Shape,
        Vector as V, World,
    },
};
use physics_engine::{approximate::Real, numeric::Scalar};
const CONTEXT: CheckpointContext = CheckpointContext {
    build: [0; 32],
    content: [243; 32],
};

fn falling(id: BodyId, case: usize) -> Body {
    let mut b = if case == 0 {
        Body::new(
            id,
            Shape::Box(V(18.0, 18.0, 18.0)),
            V(-19.23665366661797, 28.924543517247148, 37.82350256733405),
            4.0,
        )
    } else {
        Body::new(
            id,
            Shape::Box(V(18.0, 18.0, 18.0)),
            V(-43.55331673101809, 19.97465206813004, 75.01477503981347),
            1.0,
        )
    };
    if case == 0 {
        b.velocity = V(-56.924555317926824, -395.868592593489, 149.54247810890737);
        b.orientation = Q(
            0.5608297605798673,
            -0.21911133721066142,
            0.19971777717735253,
            0.7730284671556552,
        );
        b.angular_velocity = V(3.4817844746309707, -0.972024764859137, -0.5680014695277545);
    } else {
        b.velocity = V(-128.67486078042342, -505.0651408577605, 186.447343866773);
        b.orientation = Q(
            0.6363749509194875,
            -0.22208562091278558,
            0.19673532402515362,
            0.7120394027767848,
        );
        b.angular_velocity = V(6.838168522511373, 1.9432236666155343, 6.42851732580866);
    }
    b
}
// Independent quaternion-to-matrix projection, rather than engine contact/bounds helpers.
fn floor_error(b: &Body) -> Real {
    let Q(x, y, z, w) = b.orientation;
    let radius = 18.0
        * ((2.0 * (x * y + z * w)).abs()
            + (1.0 - 2.0 * (x * x + z * z)).abs()
            + (2.0 * (y * z - x * w)).abs());
    (radius - b.position.1).max(0.0)
}
fn bytes(w: &World) -> Vec<u8> {
    w.checkpoint(CONTEXT).unwrap().to_bytes()
}

/// [64-case peak floor error, cases, points, sweeps, visits, finite-floor upward response].
pub fn run() -> [Real; 6] {
    let mut peak_floor: Real = 0.0;
    let mut cases = 0_u32;
    let (mut points, mut sweeps, mut visits) = (0_u64, 0_u64, 0_u64);
    for case in 0..2 {
        for ids in [[1, 2], [2, 1]] {
            for (h, physical_steps) in [(1.0 / 240.0, 4), (1.0 / 480.0, 8)] {
                for substeps in [1, 4] {
                    for mode in 0..4 {
                        let config = Config {
                            gravity: V(0.0, -3600.0, 0.0),
                            substeps,
                            ..Config::default()
                        };
                        #[cfg(feature = "experimental-soft-contact")]
                        let config = Config {
                            soft_contact: Some(physics_engine::approximate::SoftContact::default()),
                            ..config
                        };
                        let mut world = World::new(config).unwrap();
                        let mut floor = Body::new(
                            BodyId(ids[0]),
                            Shape::Box(V(500.0, 16.0, 500.0)),
                            V(0.0, -16.0, 0.0),
                            0.0,
                        );
                        let mut body = falling(BodyId(ids[1]), case);
                        if mode == 1 {
                            body.position.1 += 1000.0;
                        }
                        if mode == 2 {
                            floor.sensor = true;
                        }
                        if mode == 3 {
                            floor.layers = CollisionLayers3d::new(1, 1);
                            body.layers = CollisionLayers3d::new(2, 2);
                        }
                        let initial = body.clone();
                        assert_eq!(floor_error(&initial), 0.0);
                        world.add_body(floor.clone()).unwrap();
                        world.add_body(body).unwrap();
                        let floor = world.body(floor.id).unwrap().clone();
                        let mut replay = world.clone();
                        let mut reference_position = initial.position;
                        let mut reference_velocity = initial.velocity;
                        let mut reference_orientation = initial.orientation;
                        let dt = h * substeps as Real;
                        for _ in 0..physical_steps / substeps {
                            let saved = bytes(&world);
                            let checkpoint = Checkpoint::from_bytes(
                                &saved,
                                CONTEXT,
                                CheckpointLimits::default(),
                            )
                            .unwrap();
                            let mut restored = checkpoint.restore();
                            let report = world.step(dt as Scalar).unwrap();
                            replay.step(dt as Scalar).unwrap();
                            restored.step(dt as Scalar).unwrap();
                            assert_eq!(bytes(&world), bytes(&replay));
                            assert_eq!(bytes(&world), bytes(&restored));
                            assert_eq!(report.substeps, u32::from(substeps));
                            let passes = if cfg!(feature = "experimental-soft-contact") {
                                10
                            } else {
                                8
                            };
                            assert!(report.impulse_iterations <= passes * u64::from(substeps));
                            assert_eq!(report.woken_bodies, 0);
                            assert!(report.retired.is_empty());
                            let b = world.body(initial.id).unwrap();
                            assert!(
                                b.position.finite()
                                    && b.velocity.finite()
                                    && b.angular_velocity.finite()
                            );
                            assert!(b.kinetic_energy().is_finite());
                            if mode == 0 {
                                let error = floor_error(b);
                                assert!(
                                    error <= 0.5,
                                    "case={case} ids={ids:?} h={h} substeps={substeps} floor={error}"
                                );
                                peak_floor = peak_floor.max(error);
                                points += report.contact_points;
                                sweeps += report.geometry.sweep_queries;
                                visits += report.convergence.constraint_visits;
                            } else {
                                for _ in 0..substeps {
                                    reference_velocity += V(0.0, -3600.0, 0.0) * h;
                                    reference_position += reference_velocity * h;
                                    reference_orientation = reference_orientation
                                        .integrate(initial.angular_velocity, h);
                                }
                                assert_eq!(b.position, reference_position);
                                assert_eq!(b.velocity, reference_velocity);
                                assert_eq!(b.orientation, reference_orientation);
                                assert_eq!(b.angular_velocity, initial.angular_velocity);
                                assert_eq!(report.contact_points, 0);
                            }
                            assert_eq!(world.body(floor.id).unwrap(), &floor);
                        }
                        assert!((world.elapsed_seconds() - 1.0 / 60.0).abs() < 1e-12);
                        cases += 1;
                    }
                }
            }
        }
    }
    assert_eq!(cases, 64);
    assert!(points > 0 && sweeps > 0 && visits > 0);
    // Explicit diagnostic ABI boundary; these bounded integer counts are exactly representable.
    [
        peak_floor,
        cases as Real,
        points as Real,
        sweeps as Real,
        visits as Real,
        finite_floor_retains_actual_contacts(),
    ]
}

/// Upward response for the two ID orders of a seven-vertex finite-floor contact.
pub fn finite_floor_retains_actual_contacts() -> Real {
    let mut minimum_upward_velocity = Real::INFINITY;
    for ids in [[1, 2], [2, 1]] {
        let mut world = World::new(Config {
            gravity: V::ZERO,
            substeps: 1,
            convergence: None,
            ..Config::default()
        })
        .unwrap();
        world
            .add_body(Body::new(
                BodyId(ids[0]),
                Shape::Box(V(18.0, 9.0, 18.0)),
                V::ZERO,
                0.0,
            ))
            .unwrap();
        let mut body = Body::new(
            BodyId(ids[1]),
            Shape::Box(V(18.0, 18.0, 18.0)),
            V(-1.4180084689188672, 36.00023777180524, -5.57869237669652),
            2.0,
        );
        body.orientation = Q(
            0.4301449589172992,
            0.023109449875315845,
            -0.06802955923209429,
            0.8998962421941668,
        );
        body.angular_velocity = V(0.0, 0.0, -200.0);
        let initial_energy = body.kinetic_energy();
        world.add_body(body).unwrap();
        let report = world.step(1.0 / 240.0).unwrap();
        let body = world.body(BodyId(ids[1])).unwrap();
        assert!(
            body.velocity.1 > 0.0,
            "the approaching actual corner must receive a normal impulse: {body:?}"
        );
        minimum_upward_velocity = minimum_upward_velocity.min(body.velocity.1);
        assert!(body.kinetic_energy() <= initial_energy * (1.0 + 1e-10));
        assert!(report.contact_points <= 4 && report.impulse_iterations <= 8);
        assert_eq!(world.elapsed_seconds(), 1.0 / 240.0);
    }
    minimum_upward_velocity
}
