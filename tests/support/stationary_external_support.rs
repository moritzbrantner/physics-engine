//! Natural external-support parking with default-policy restart and dependency loss.
use physics_engine::approximate::{
    Body, Checkpoint, CheckpointContext, CheckpointLimits, CombineRule, Config, Shape, Vector as V,
    World,
};
use physics_engine::{approximate::Real, numeric::Scalar};

/// Rounding-only bound: the f64 reference, or 16 ulps of `scale` when larger (f32 state).
fn rounding(reference: Real, scale: Real) -> Real {
    reference.max(16.0 * Real::EPSILON * scale)
}

const H: Real = 1.0 / 240.0;
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
    let mut floor = Body::keyed(
        ids[0],
        Shape::Box(V(if shared { 12.0 } else { 4.0 }, 0.5, 4.0)),
        V::ZERO,
        if external { 1.0 } else { 0.0 },
    );
    floor.external = external;
    floor.friction = 0.6;
    w.add_body(floor).unwrap();
    if !shared {
        let mut far = Body::keyed(200, Shape::Box(V(4.0, 0.5, 4.0)), V(20.0, -0.5, 20.0), 0.0);
        far.friction = 0.6;
        w.add_body(far).unwrap();
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
        let mut b = Body::keyed(id, Shape::Sphere(1.0), p, 2.0);
        b.friction = 0.6;
        if shared {
            // Floor 0.6 / wall 0 under maximum coefficient mixing.
            b.friction = 0.0;
            b.friction_combine = CombineRule::Max;
        }
        w.add_body(b).unwrap();
    }
    w
}
fn fixture(case: Case, ids: [u64; 2], substeps: u8) -> (World, Real) {
    let mut w = initial(case, ids, substeps, true);
    let mut fixed = initial(case, ids, substeps, false);
    let mut parked = None;
    for _ in 0..32 {
        w.step(1.0 / 60.0).unwrap();
        fixed.step(1.0 / 60.0).unwrap();
        assert_eq!(
            w.body_id(ids[1]).and_then(|id| w.body(id)),
            fixed.body(w.body_id(ids[1]).unwrap())
        );
        if w.body_id(ids[1])
            .and_then(|id| w.body(id))
            .unwrap()
            .is_sleeping()
            && parked.is_none()
        {
            parked = Some(w.elapsed_seconds());
        }
    }
    let parked = parked.unwrap();
    let sleep_seconds = w.config().sleep_seconds as Scalar;
    // Quiet time accumulates in solver precision; the bound is rounding-only.
    let time_tolerance = 32.0 * Real::EPSILON as Scalar * parked.max(sleep_seconds);
    assert!(parked + time_tolerance >= sleep_seconds);
    assert!(parked <= sleep_seconds + 1.0 / 60.0 + 1e-12);
    assert!(w.has_support(w.body_id(ids[1]).unwrap()));
    assert!(
        w.body_id(40)
            .and_then(|id| w.body(id))
            .unwrap()
            .is_sleeping()
    );
    w.set_velocity(w.body_id(ids[0]).unwrap(), case.command())
        .unwrap();
    assert!(
        w.body_id(ids[1])
            .and_then(|id| w.body(id))
            .unwrap()
            .is_sleeping()
    );
    if case.removed() {
        w.remove_body(w.body_id(ids[0]).unwrap()).unwrap();
        assert!(
            !w.body_id(ids[1])
                .and_then(|id| w.body(id))
                .unwrap()
                .is_sleeping()
        );
        assert!(!w.has_support(w.body_id(ids[1]).unwrap()));
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
        let mut b = Body::keyed(20, shape, p, 1.0);
        b.external = true;
        b.friction = 0.0;
        w.add_body(b).unwrap();
        w.set_velocity(w.body_id(20).unwrap(), v).unwrap();
        if matches!(case, Case::UnrelatedRemoval) {
            w.remove_body(w.body_id(20).unwrap()).unwrap();
        }
        assert!(
            w.body_id(ids[1])
                .and_then(|id| w.body(id))
                .unwrap()
                .is_sleeping()
        );
    }
    assert!(
        w.body_id(40)
            .and_then(|id| w.body(id))
            .unwrap()
            .is_sleeping()
    );
    (w, parked as Real)
}
fn observe(w: &World, case: Case, ids: [u64; 2], n: u32) -> Real {
    let b = w.body_id(ids[1]).and_then(|id| w.body(id)).unwrap();
    let t = H * n as Real;
    let fall = if matches!(case, Case::HorizontalDeparture) {
        n - 1
    } else {
        n
    };
    let (v, p, spin) = match case {
        Case::Normal => (V(0.0, 3.0, 0.0), V(0.0, 1.5 + 3.0 * t, 0.0), V::ZERO),
        Case::Tangent => (
            V(6.0 * t, 0.0, 0.0),
            V(3.0 * H * H * n as Real * (n + 1) as Real, 1.5, 0.0),
            V(0.0, 0.0, 15.0 * t),
        ),
        Case::DownwardDeparture | Case::Removal | Case::HorizontalDeparture => {
            let horizontal = matches!(case, Case::HorizontalDeparture);
            let vx = if horizontal { 6.0 * H } else { 0.0 };
            (
                V(vx, -10.0 * H * fall as Real, 0.0),
                V(
                    vx * t,
                    1.5 - 5.0 * H * H * fall as Real * (fall + 1) as Real,
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
        error <= rounding(1e-10, 16.0 * (n as Real + 1.0)),
        "{case:?} ids={ids:?} n={n} error={error} body={b:?}"
    );
    assert_eq!(b.is_sleeping(), case.sleeping());
    assert_eq!(b.mass, 2.0);
    assert!(b.position.finite() && b.velocity.finite() && b.angular_velocity.finite());
    if let Some(floor) = w.body_id(ids[0]).and_then(|id| w.body(id)) {
        assert!(floor.external);
        assert_eq!(floor.velocity, case.command());
        assert!((floor.position - case.command() * t).length() <= rounding(1e-10, 8.0));
    }
    let neighbor = w.body_id(40).and_then(|id| w.body(id)).unwrap();
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
    if let Some(driver) = w.body_id(20).and_then(|id| w.body(id)) {
        let start = if matches!(case, Case::NearMiss) {
            V(0.0, -0.015, 0.0)
        } else {
            V(-1.5, 1.5, 0.0)
        };
        assert_eq!(driver.velocity, V(3.0, 0.0, 0.0));
        assert!((driver.position - (start + driver.velocity * t)).length() <= rounding(1e-10, 8.0));
    }
    error
}
fn run_case(case: Case, ids: [u64; 2], substeps: u8) -> [Real; 21] {
    let (mut w, parked) = fixture(case, ids, substeps);
    let (mut repeated, _) = fixture(case, ids, substeps);
    let bytes = w.checkpoint(CONTEXT).unwrap().to_bytes();
    let mut restored = Checkpoint::from_bytes(&bytes, CONTEXT, CheckpointLimits::default())
        .unwrap()
        .restore();
    let before = w.elapsed_seconds();
    let dt = H * substeps as Real;
    let mut first = None;
    let mut max_error: Real = 0.0;
    for call in 1..=5 {
        let r = w.step(dt as Scalar).unwrap();
        repeated.step(dt as Scalar).unwrap();
        restored.step(dt as Scalar).unwrap();
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
            // Prepared once per step, plus once more in the substep after a wake, which
            // regenerates; later substeps reuse the step's rows and responses (#378).
            assert_eq!(
                r.response_preparations,
                if case.sleeping() {
                    0
                } else if case.removed() {
                    1
                } else {
                    u64::from(substeps.min(2))
                }
            );
            assert_eq!(r.inertia_preparations, r.response_preparations);
            if case.sleeping() {
                assert_eq!(r.contact_points, 0);
                assert_eq!(r.convergence.constraint_visits, 0);
            }
            let b = w.body_id(ids[1]).and_then(|id| w.body(id)).unwrap();
            let energy =
                b.velocity.dot(b.velocity) + 0.4 * b.angular_velocity.dot(b.angular_velocity);
            let work = match case {
                Case::Normal => 6.0 * (b.velocity.1 + 10.0 * dt),
                Case::Tangent => 6.0 * b.velocity.0,
                Case::HorizontalDeparture => 2000.0 * 2.0 * b.velocity.0,
                Case::SharedSideContact => 3.0 * (2.0 * b.velocity.0 - 0.8 * b.angular_velocity.2),
                _ => 0.0,
            };
            assert!(energy + 20.0 * (b.position.1 - 1.5) <= work + rounding(1e-10, 64.0));
            assert!((w.elapsed_seconds() - before - dt as Scalar).abs() <= 1e-14);
            first = Some([
                b.velocity.0,
                b.velocity.1,
                b.velocity.2,
                b.angular_velocity.0,
                b.angular_velocity.1,
                b.angular_velocity.2,
                energy,
                work,
                r.woken_bodies as Real,
                r.response_preparations as Real,
                r.inertia_preparations as Real,
                r.inertia_applications as Real,
                r.geometry.current_queries as Real,
                r.geometry.sweep_queries as Real,
                r.contact_points as Real,
                r.convergence.constraint_visits as Real,
                (w.elapsed_seconds() - before) as Real,
                0.0,
                Real::from(u8::from(b.is_sleeping())),
                Real::from(u8::from(
                    w.body_id(40)
                        .and_then(|id| w.body(id))
                        .unwrap()
                        .is_sleeping(),
                )),
                parked,
            ]);
        }
    }
    if matches!(
        case,
        Case::DownwardDeparture | Case::HorizontalDeparture | Case::Removal
    ) {
        assert!(!w.has_support(w.body_id(ids[1]).unwrap()));
    }
    assert!((w.elapsed_seconds() - before - 5.0 * dt as Scalar).abs() <= 1e-12);
    let mut values = first.unwrap();
    values[17] = max_error;
    println!(
        "STATIONARY_EXTERNAL_SUPPORT {{\"case\":\"{case:?}\",\"ids\":{ids:?},\"substeps\":{substeps},\"values\":{values:?}}}"
    );
    values
}
pub fn run() -> [[Real; 21]; 36] {
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
