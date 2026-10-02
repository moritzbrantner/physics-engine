//! Contact admission for a translating external body and naturally parked dynamics.
use physics_engine::{
    BodyId, CollisionLayers3d,
    approximate::{
        Body, Checkpoint, CheckpointContext, CheckpointLimits, Config, Shape, Vector as V, World,
    },
};
use physics_engine::{approximate::Real, numeric::Scalar};

/// Rounding-only bound: the f64 reference, or 16 ulps of `scale` when larger (f32 state).
fn rounding(reference: Real, scale: Real) -> Real {
    reference.max(16.0 * Real::EPSILON * scale)
}

const H: Real = 1.0 / 240.0;
const CONTEXT: CheckpointContext = CheckpointContext {
    build: [0; 32],
    content: [250; 32],
};
#[derive(Clone, Copy, Debug)]
enum Case {
    Current,
    SlowCurrent,
    Swept,
    SweptOutsideSlop,
    SkinMiss,
    BroadMiss,
    Stationary,
    Separating,
    Sensor,
    Layers,
}
impl Case {
    fn admits(self) -> bool {
        matches!(
            self,
            Self::Current | Self::SlowCurrent | Self::Swept | Self::SweptOutsideSlop
        )
    }
    fn interval(self) -> Real {
        if matches!(self, Self::Current | Self::SweptOutsideSlop) {
            1.0 / 60.0
        } else {
            H
        }
    }
    fn speed(self) -> Real {
        match self {
            Self::SlowCurrent => 0.25,
            Self::Stationary => 0.0,
            Self::Separating => -3.0,
            _ => 3.0,
        }
    }
    fn position(self) -> V {
        match self {
            Self::Swept => V(0.0, -0.505, 0.0),
            Self::SweptOutsideSlop => V(0.0, -0.525, 0.0),
            Self::SkinMiss => V(0.0, -0.515, 0.0),
            Self::BroadMiss => V(4.9, -0.5, 4.9),
            _ => V(0.0, -0.5, 0.0),
        }
    }
}
fn fixture(case: Case, ids: [u64; 2]) -> World {
    let mut w = World::new(Config {
        gravity: V(0.0, -10.0, 0.0),
        substeps: 1,
        ..Config::default()
    })
    .unwrap();
    w.add_body(Body::new(
        BodyId(0),
        Shape::Box(V(12.0, 0.5, 4.0)),
        V(0.0, -0.5, 0.0),
        0.0,
    ))
    .unwrap();
    for (id, x) in [(ids[1], 0.0), (30, 10.0)] {
        w.add_body(Body::new(
            BodyId(id),
            Shape::Sphere(1.0),
            V(x, 1.0, 0.0),
            2.0,
        ))
        .unwrap();
    }
    for _ in 0..32 {
        w.step(1.0 / 60.0).unwrap();
    }
    assert!(w.body(BodyId(ids[1])).unwrap().is_sleeping());
    assert!(w.body(BodyId(30)).unwrap().is_sleeping());
    let mut driven = Body::new(
        BodyId(ids[0]),
        Shape::Box(V(4.0, 0.5, 4.0)),
        case.position(),
        1.0,
    );
    driven.external = true;
    driven.sensor = matches!(case, Case::Sensor);
    if matches!(case, Case::Layers) {
        driven.layers = CollisionLayers3d::new(1, 0);
    }
    w.add_body(driven).unwrap();
    assert!(
        w.body(BodyId(ids[1])).unwrap().is_sleeping(),
        "creation is not wake evidence"
    );
    w.set_velocity(BodyId(ids[0]), V(0.0, case.speed(), 0.0))
        .unwrap();
    assert!(
        w.body(BodyId(ids[1])).unwrap().is_sleeping(),
        "a command is not wake evidence"
    );
    w
}
fn run_case(case: Case, ids: [u64; 2]) -> [Real; 18] {
    let mut w = fixture(case, ids);
    let mut repeated = fixture(case, ids);
    let bytes = w.checkpoint(CONTEXT).unwrap().to_bytes();
    let mut restored = Checkpoint::from_bytes(&bytes, CONTEXT, CheckpointLimits::default())
        .unwrap()
        .restore();
    let h = case.interval();
    let before = w.elapsed_seconds();
    let report = w.step(h as Scalar).unwrap();
    repeated.step(h as Scalar).unwrap();
    restored.step(h as Scalar).unwrap();
    for other in [&repeated, &restored] {
        assert_eq!(
            w.checkpoint(CONTEXT).unwrap().to_bytes(),
            other.checkpoint(CONTEXT).unwrap().to_bytes()
        );
    }
    assert!(
        (w.elapsed_seconds() - before - h as Scalar).abs()
            <= 16.0 * f64::EPSILON * (before + h as Scalar)
    );
    assert_eq!(report.substeps, 1);
    assert!(report.impulse_iterations <= 8);
    assert!(report.convergence.constraint_visits <= 8 * report.contact_points);
    assert_eq!(report.position.passes, 0);
    assert!(report.retired.is_empty());
    let driven = w.body(BodyId(ids[0])).unwrap();
    assert!(driven.external && !driven.is_sleeping());
    assert_eq!(driven.velocity, V(0.0, case.speed(), 0.0));
    assert!(
        (driven.position - (case.position() + driven.velocity * h)).length()
            <= rounding(1e-12, 8.0)
    );
    let rider = w.body(BodyId(ids[1])).unwrap();
    assert_eq!(rider.mass, 2.0);
    assert!(rider.position.finite() && rider.velocity.finite() && rider.angular_velocity.finite());
    let neighbor = w.body(BodyId(30)).unwrap();
    assert!(
        neighbor.is_sleeping(),
        "a shared fixed floor must not bridge parked islands"
    );
    assert_eq!(neighbor.position, V(10.0, 1.0, 0.0));
    assert_eq!(neighbor.velocity, V::ZERO);
    let energy = rider.velocity.dot(rider.velocity)
        + 0.4 * rider.angular_velocity.dot(rider.angular_velocity);
    // Contact-awakened bodies receive this substep's gravity before response (#253).
    let external_work = if case.admits() {
        2.0 * case.speed() * (rider.velocity.1 + 10.0 * h)
    } else {
        0.0
    };
    assert!(energy + 20.0 * (rider.position.1 - 1.0) <= external_work + rounding(1e-10, 32.0));
    if case.admits() {
        assert_eq!(report.woken_bodies, 1);
        assert!(!rider.is_sleeping());
        assert_eq!(report.response_preparations, 1);
        assert_eq!(report.inertia_preparations, 1);
        assert!(report.inertia_applications > 0);
        assert!(report.contact_points > 0 && report.convergence.constraint_visits > 0);
        assert!(rider.velocity.1 > 0.0);
        let gap = rider.position.1 - 1.0 - (driven.position.1 + 0.5);
        assert!(
            gap >= -rounding(1e-10, 2.0),
            "admitted normal response leaves penetration {gap}"
        );
        if !matches!(case, Case::Swept | Case::SweptOutsideSlop) {
            assert!((rider.velocity.1 - case.speed()).abs() <= rounding(1e-10, 4.0));
            assert!((rider.position.1 - 1.0 - case.speed() * h).abs() <= rounding(1e-10, 2.0));
            assert_eq!(rider.angular_velocity, V::ZERO);
        } else {
            // Admission is the contract here; restitution/remaining-time response is #246.
            assert!(report.geometry.sweep_queries > 0);
            assert!(rider.velocity.1 <= case.speed() + rounding(1e-10, 4.0));
        }
    } else {
        assert_eq!(report.woken_bodies, 0);
        assert_eq!(report.response_preparations, 0);
        assert_eq!(report.inertia_preparations, 0);
        assert_eq!(report.contact_points, 0);
        assert_eq!(report.convergence.constraint_visits, 0);
        assert!(rider.is_sleeping());
        assert_eq!(rider.position, V(0.0, 1.0, 0.0));
        assert_eq!(rider.velocity, V::ZERO);
        if matches!(case, Case::SkinMiss | Case::BroadMiss) {
            // Discarded narrow/swept probes still count as work.
            assert!(report.narrow_tests > 0);
            assert!(report.geometry.current_queries > 0 && report.geometry.sweep_queries > 0);
        }
    }
    let values = [
        rider.position.1,
        rider.velocity.1,
        energy,
        external_work,
        report.woken_bodies as Real,
        report.response_preparations as Real,
        report.inertia_preparations as Real,
        report.inertia_applications as Real,
        report.pair_tests as Real,
        report.narrow_tests as Real,
        report.geometry.current_queries as Real,
        report.geometry.sweep_queries as Real,
        report.contact_points as Real,
        report.convergence.constraint_visits as Real,
        report.swept_contacts as Real,
        Real::from(u8::from(rider.is_sleeping())),
        Real::from(u8::from(neighbor.is_sleeping())),
        (w.elapsed_seconds() - before) as Real,
    ];
    // Compare complete continuation history, not only the observed first-call state.
    for _ in 0..8 {
        for world in [&mut w, &mut repeated, &mut restored] {
            world.step(H as Scalar).unwrap();
        }
        for other in [&repeated, &restored] {
            assert_eq!(
                w.checkpoint(CONTEXT).unwrap().to_bytes(),
                other.checkpoint(CONTEXT).unwrap().to_bytes()
            );
        }
    }
    println!("DRIVEN_PARKED {{\"case\":\"{case:?}\",\"ids\":{ids:?},\"values\":{values:?}}}");
    values
}
pub fn run() -> [[Real; 18]; 20] {
    std::array::from_fn(|index| {
        run_case(
            [
                Case::Current,
                Case::SlowCurrent,
                Case::Swept,
                Case::SweptOutsideSlop,
                Case::SkinMiss,
                Case::BroadMiss,
                Case::Stationary,
                Case::Separating,
                Case::Sensor,
                Case::Layers,
            ][index / 2],
            if index % 2 == 0 { [1, 10] } else { [10, 1] },
        )
    })
}
