//! Natural external-support parking with default-policy restart and dependency loss.
use physics_engine::{
    BodyId,
    approximate::{
        Body, Checkpoint, CheckpointContext, CheckpointLimits, Config, Shape, Vector as V, World,
    },
};
const H: f64 = 1.0 / 240.0;
const CONTEXT: CheckpointContext = CheckpointContext {
    build: [0; 32],
    content: [251; 32],
};
#[derive(Clone, Copy, Debug)]
enum Case {
    Stationary,
    Normal,
    Tangent,
    DownwardDeparture,
    HorizontalDeparture,
    Removal,
    NearMiss,
    UnrelatedRemoval,
    SharedSideContact,
}
impl Case {
    fn sleeping(self) -> bool {
        matches!(
            self,
            Self::Stationary | Self::NearMiss | Self::UnrelatedRemoval
        )
    }
    fn command(self) -> V {
        match self {
            Self::Normal => V(0.0, 3.0, 0.0),
            Self::Tangent => V(3.0, 0.0, 0.0),
            Self::DownwardDeparture => V(0.0, -3.0, 0.0),
            Self::HorizontalDeparture => V(2000.0, 0.0, 0.0),
            _ => V::ZERO,
        }
    }
    fn removed(self) -> bool {
        matches!(self, Self::Removal)
    }
}
fn initial(case: Case, ids: [u64; 2], substeps: u8, external: bool) -> World {
    let mut w = World::new(Config {
        gravity: V(0.0, -10.0, 0.0),
        substeps,
        convergence: None,
        ..Config::default()
    })
    .unwrap();
    let shared = matches!(case, Case::SharedSideContact);
    let mut floor = Body::new(
        BodyId(ids[0]),
        Shape::Box(V(if shared { 12.0 } else { 4.0 }, 0.5, 4.0)),
        V::ZERO,
        if external { 1.0 } else { 0.0 },
    );
    floor.external = external;
    w.add_body(floor).unwrap();
    if !shared {
        w.add_body(Body::new(
            BodyId(200),
            Shape::Box(V(4.0, 0.5, 4.0)),
            V(20.0, -0.5, 20.0),
            0.0,
        ))
        .unwrap();
    }
    for (id, p) in [
        (ids[1], V(0.0, 1.5, 0.0)),
        (
            40,
            if shared {
                V(10.0, 1.5, 0.0)
            } else {
                V(20.0, 1.0, 20.0)
            },
        ),
    ] {
        let mut b = Body::new(BodyId(id), Shape::Sphere(1.0), p, 2.0);
        if shared {
            b.friction = 0.0;
        } // Floor .6 / wall0 under maximum coefficient mixing.
        w.add_body(b).unwrap();
    }
    w
}
fn fixture(case: Case, ids: [u64; 2], substeps: u8) -> (World, f64) {
    let mut w = initial(case, ids, substeps, true);
    let mut fixed = initial(case, ids, substeps, false);
    let mut parked = None;
    for _ in 0..32 {
        w.step(1.0 / 60.0).unwrap();
        fixed.step(1.0 / 60.0).unwrap();
        assert_eq!(w.body(BodyId(ids[1])), fixed.body(BodyId(ids[1])));
        if w.body(BodyId(ids[1])).unwrap().is_sleeping() && parked.is_none() {
            parked = Some(w.elapsed_seconds());
        }
    }
    let parked = parked.unwrap();
    let time_tolerance = 32.0 * f64::EPSILON * parked.max(w.config().sleep_seconds);
    assert!(parked + time_tolerance >= w.config().sleep_seconds);
    assert!(parked <= w.config().sleep_seconds + 1.0 / 60.0 + 1e-12);
    assert!(w.has_support(BodyId(ids[1])));
    assert!(w.body(BodyId(40)).unwrap().is_sleeping());
    w.set_velocity(BodyId(ids[0]), case.command()).unwrap();
    assert!(w.body(BodyId(ids[1])).unwrap().is_sleeping());
    if case.removed() {
        w.remove_body(BodyId(ids[0])).unwrap();
        assert!(!w.body(BodyId(ids[1])).unwrap().is_sleeping());
        assert!(!w.has_support(BodyId(ids[1])));
    }
    if matches!(
        case,
        Case::NearMiss | Case::SharedSideContact | Case::UnrelatedRemoval
    ) {
        let (shape, p, v) = match case {
            Case::NearMiss => (
                Shape::Box(V(4.0, 0.5, 4.0)),
                V(0.0, -0.015, 0.0),
                V(3.0, 0.0, 0.0),
            ),
            Case::SharedSideContact => (
                Shape::Box(V(0.5, 4.0, 4.0)),
                V(-1.5, 1.5, 0.0),
                V(3.0, 0.0, 0.0),
            ),
            _ => (Shape::Sphere(1.0), V(100.0, 100.0, 100.0), V::ZERO),
        };
        let mut b = Body::new(BodyId(20), shape, p, 1.0);
        b.external = true;
        b.friction = 0.0;
        w.add_body(b).unwrap();
        w.set_velocity(BodyId(20), v).unwrap();
        if matches!(case, Case::UnrelatedRemoval) {
            w.remove_body(BodyId(20)).unwrap();
        }
        assert!(w.body(BodyId(ids[1])).unwrap().is_sleeping());
    }
    assert!(w.body(BodyId(40)).unwrap().is_sleeping());
    (w, parked)
}
fn observe(w: &World, case: Case, ids: [u64; 2], n: u32) -> f64 {
    let b = w.body(BodyId(ids[1])).unwrap();
    let t = H * f64::from(n);
    let fall = if matches!(case, Case::HorizontalDeparture) {
        n - 1
    } else {
        n
    };
    let (v, p, spin) = match case {
        Case::Normal => (V(0.0, 3.0, 0.0), V(0.0, 1.5 + 3.0 * t, 0.0), V::ZERO),
        Case::Tangent => (
            V(6.0 * t, 0.0, 0.0),
            V(3.0 * H * H * f64::from(n) * f64::from(n + 1), 1.5, 0.0),
            V(0.0, 0.0, 15.0 * t),
        ),
        Case::DownwardDeparture | Case::Removal | Case::HorizontalDeparture => {
            let horizontal = matches!(case, Case::HorizontalDeparture);
            let vx = if horizontal { 6.0 * H } else { 0.0 };
            (
                V(vx, -10.0 * H * f64::from(fall), 0.0),
                V(
                    vx * t,
                    1.5 - 5.0 * H * H * f64::from(fall) * f64::from(fall + 1),
                    0.0,
                ),
                V(0.0, 0.0, if horizontal { 15.0 * H } else { 0.0 }),
            )
        }
        Case::SharedSideContact => (
            V(3.0, 0.0, 0.0),
            V(3.0 * t, 1.5, 0.0),
            V(0.0, 0.0, -15.0 * t),
        ),
        _ => (V::ZERO, V(0.0, 1.5, 0.0), V::ZERO),
    };
    let error = (b.velocity - v)
        .length()
        .max((b.position - p).length())
        .max((b.angular_velocity - spin).length());
    assert!(
        error <= 1e-10,
        "{case:?} ids={ids:?} n={n} error={error} body={b:?}"
    );
    assert_eq!(b.is_sleeping(), case.sleeping());
    assert_eq!(b.mass, 2.0);
    assert!(b.position.finite() && b.velocity.finite() && b.angular_velocity.finite());
    if let Some(floor) = w.body(BodyId(ids[0])) {
        assert!(floor.external);
        assert_eq!(floor.velocity, case.command());
        assert!((floor.position - case.command() * t).length() <= 1e-10);
    }
    let neighbor = w.body(BodyId(40)).unwrap();
    assert!(neighbor.is_sleeping());
    assert_eq!(
        neighbor.position,
        if matches!(case, Case::SharedSideContact) {
            V(10.0, 1.5, 0.0)
        } else {
            V(20.0, 1.0, 20.0)
        }
    );
    assert_eq!(neighbor.velocity, V::ZERO);
    if let Some(driver) = w.body(BodyId(20)) {
        let start = if matches!(case, Case::NearMiss) {
            V(0.0, -0.015, 0.0)
        } else {
            V(-1.5, 1.5, 0.0)
        };
        assert_eq!(driver.velocity, V(3.0, 0.0, 0.0));
        assert!((driver.position - (start + driver.velocity * t)).length() <= 1e-10);
    }
    error
}
fn run_case(case: Case, ids: [u64; 2], substeps: u8) -> [f64; 21] {
    let (mut w, parked) = fixture(case, ids, substeps);
    let (mut repeated, _) = fixture(case, ids, substeps);
    let bytes = w.checkpoint(CONTEXT).unwrap().to_bytes();
    let mut restored = Checkpoint::from_bytes(&bytes, CONTEXT, CheckpointLimits::default())
        .unwrap()
        .restore();
    let before = w.elapsed_seconds();
    let dt = H * f64::from(substeps);
    let mut first = None;
    let mut max_error: f64 = 0.0;
    for call in 1..=5 {
        let r = w.step(dt).unwrap();
        repeated.step(dt).unwrap();
        restored.step(dt).unwrap();
        for other in [&repeated, &restored] {
            assert_eq!(
                w.checkpoint(CONTEXT).unwrap().to_bytes(),
                other.checkpoint(CONTEXT).unwrap().to_bytes()
            );
        }
        max_error = max_error.max(observe(&w, case, ids, call * u32::from(substeps)));
        assert_eq!(r.substeps, u32::from(substeps));
        assert!(r.impulse_iterations <= 8 * u64::from(substeps));
        assert!(r.convergence.constraint_visits <= 8 * r.contact_points);
        assert_eq!(r.position.passes, 0);
        assert!(r.retired.is_empty());
        if call == 1 {
            assert_eq!(
                r.woken_bodies,
                u64::from(!case.sleeping() && !case.removed())
            );
            assert_eq!(
                r.response_preparations,
                if case.sleeping() {
                    0
                } else {
                    u64::from(substeps)
                }
            );
            assert_eq!(r.inertia_preparations, r.response_preparations);
            if case.sleeping() {
                assert_eq!(r.contact_points, 0);
                assert_eq!(r.convergence.constraint_visits, 0);
            }
            let b = w.body(BodyId(ids[1])).unwrap();
            let energy =
                b.velocity.dot(b.velocity) + 0.4 * b.angular_velocity.dot(b.angular_velocity);
            let work = match case {
                Case::Normal => 6.0 * (b.velocity.1 + 10.0 * dt),
                Case::Tangent => 6.0 * b.velocity.0,
                Case::HorizontalDeparture => 2000.0 * 2.0 * b.velocity.0,
                Case::SharedSideContact => 3.0 * (2.0 * b.velocity.0 - 0.8 * b.angular_velocity.2),
                _ => 0.0,
            };
            assert!(energy + 20.0 * (b.position.1 - 1.5) <= work + 1e-10);
            assert!((w.elapsed_seconds() - before - dt).abs() <= 1e-14);
            first = Some([
                b.velocity.0,
                b.velocity.1,
                b.velocity.2,
                b.angular_velocity.0,
                b.angular_velocity.1,
                b.angular_velocity.2,
                energy,
                work,
                r.woken_bodies as f64,
                r.response_preparations as f64,
                r.inertia_preparations as f64,
                r.inertia_applications as f64,
                r.geometry.current_queries as f64,
                r.geometry.sweep_queries as f64,
                r.contact_points as f64,
                r.convergence.constraint_visits as f64,
                w.elapsed_seconds() - before,
                0.0,
                f64::from(u8::from(b.is_sleeping())),
                f64::from(u8::from(w.body(BodyId(40)).unwrap().is_sleeping())),
                parked,
            ]);
        }
    }
    if matches!(
        case,
        Case::DownwardDeparture | Case::HorizontalDeparture | Case::Removal
    ) {
        assert!(!w.has_support(BodyId(ids[1])));
    }
    assert!((w.elapsed_seconds() - before - 5.0 * dt).abs() <= 1e-12);
    let mut values = first.unwrap();
    values[17] = max_error;
    println!(
        "STATIONARY_EXTERNAL_SUPPORT {{\"case\":\"{case:?}\",\"ids\":{ids:?},\"substeps\":{substeps},\"values\":{values:?}}}"
    );
    values
}
pub fn run() -> [[f64; 21]; 36] {
    std::array::from_fn(|i| {
        run_case(
            [
                Case::Stationary,
                Case::Normal,
                Case::Tangent,
                Case::DownwardDeparture,
                Case::HorizontalDeparture,
                Case::Removal,
                Case::NearMiss,
                Case::UnrelatedRemoval,
                Case::SharedSideContact,
            ][i / 4],
            if i % 4 < 2 { [1, 10] } else { [10, 1] },
            if i % 2 == 0 { 1 } else { 4 },
        )
    })
}
