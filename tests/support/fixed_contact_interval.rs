//! Captured fixed-floor interval misses, through the public stepping/checkpoint seam.
use physics_engine::{
    BodyId, CollisionLayers3d,
    approximate::{
        Body, Checkpoint, CheckpointContext, CheckpointLimits, Config, Quaternion as Q, Shape,
        Vector as V, World,
    },
};
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
fn floor_error(b: &Body) -> f64 {
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

/// [peak floor error, cases, total contact points, sweep queries, constraint visits].
pub fn run() -> [f64; 5] {
    let mut peak_floor = 0.0_f64;
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
                        let dt = h * f64::from(substeps);
                        for _ in 0..physical_steps / substeps {
                            let saved = bytes(&world);
                            let checkpoint = Checkpoint::from_bytes(
                                &saved,
                                CONTEXT,
                                CheckpointLimits::default(),
                            )
                            .unwrap();
                            let mut restored = checkpoint.restore();
                            let report = world.step(dt).unwrap();
                            replay.step(dt).unwrap();
                            restored.step(dt).unwrap();
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
        f64::from(cases),
        points as f64,
        sweeps as f64,
        visits as f64,
    ]
}
