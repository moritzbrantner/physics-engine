//! Contact admission for a translating external body and naturally parked dynamics.
use physics_engine::{
    BodyId, CollisionLayers3d,
    approximate::{
        Body, Checkpoint, CheckpointContext, CheckpointLimits, Config, Shape, Vector as V, World,
    },
};
const H: f64 = 1.0 / 240.0;
const CONTEXT: CheckpointContext = CheckpointContext {
    build: [0; 32],
    content: [250; 32],
};
#[derive(Clone, Copy, Debug)]
enum Case {
    Current,
    SlowCurrent,
    Swept,
    SkinMiss,
    BroadMiss,
    Stationary,
    Separating,
    Sensor,
    Layers,
}
impl Case {
    fn admits(self) -> bool {
        matches!(self, Self::Current | Self::SlowCurrent | Self::Swept)
    }
    fn speed(self) -> f64 {
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
fn run_case(case: Case, ids: [u64; 2]) -> [f64; 18] {
    let mut w = fixture(case, ids);
    let mut repeated = fixture(case, ids);
    let bytes = w.checkpoint(CONTEXT).unwrap().to_bytes();
    let mut restored = Checkpoint::from_bytes(&bytes, CONTEXT, CheckpointLimits::default())
        .unwrap()
        .restore();
    let before = w.elapsed_seconds();
    let report = w.step(H).unwrap();
    repeated.step(H).unwrap();
    restored.step(H).unwrap();
    for other in [&repeated, &restored] {
        assert_eq!(
            w.checkpoint(CONTEXT).unwrap().to_bytes(),
            other.checkpoint(CONTEXT).unwrap().to_bytes()
        );
    }
    assert!((w.elapsed_seconds() - before - H).abs() <= 16.0 * f64::EPSILON * (before + H));
    assert_eq!(report.substeps, 1);
    assert!(report.impulse_iterations <= 8);
    assert!(report.convergence.constraint_visits <= 8 * report.contact_points);
    assert_eq!(report.position.passes, 0);
    assert!(report.retired.is_empty());
    let driven = w.body(BodyId(ids[0])).unwrap();
    assert!(driven.external && !driven.is_sleeping());
    assert_eq!(driven.velocity, V(0.0, case.speed(), 0.0));
    assert!((driven.position - (case.position() + driven.velocity * H)).length() <= 1e-12);
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
    let external_work = 2.0 * case.speed() * rider.velocity.1;
    assert!(energy + 20.0 * (rider.position.1 - 1.0) <= external_work + 1e-10);
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
            gap >= -1e-10,
            "admitted normal response leaves penetration {gap}"
        );
        if !matches!(case, Case::Swept) {
            assert!((rider.velocity.1 - case.speed()).abs() <= 1e-10);
            assert!((rider.position.1 - 1.0 - case.speed() * H).abs() <= 1e-10);
            assert_eq!(rider.angular_velocity, V::ZERO);
        } else {
            // Admission is the contract here; restitution/remaining-time response is #246.
            assert!(report.geometry.sweep_queries > 0);
            assert!(rider.velocity.1 <= case.speed() + 1e-10);
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
        report.woken_bodies as f64,
        report.response_preparations as f64,
        report.inertia_preparations as f64,
        report.inertia_applications as f64,
        report.pair_tests as f64,
        report.narrow_tests as f64,
        report.geometry.current_queries as f64,
        report.geometry.sweep_queries as f64,
        report.contact_points as f64,
        report.convergence.constraint_visits as f64,
        report.swept_contacts as f64,
        f64::from(u8::from(rider.is_sleeping())),
        f64::from(u8::from(neighbor.is_sleeping())),
        w.elapsed_seconds() - before,
    ];
    // Compare complete continuation history, not only the observed first-call state.
    for _ in 0..8 {
        for world in [&mut w, &mut repeated, &mut restored] {
            world.step(H).unwrap();
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
pub fn run() -> [[f64; 18]; 18] {
    std::array::from_fn(|index| {
        run_case(
            [
                Case::Current,
                Case::SlowCurrent,
                Case::Swept,
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
