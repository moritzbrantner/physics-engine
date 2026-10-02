//! Tangential wake authority after strict geometry; no external support-root changes.
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
    content: [254; 32],
};
#[derive(Clone, Copy, Debug)]
enum Case {
    Tangent,
    Loaded,
    LoadedFour,
    Slow,
    Normal,
    Stationary,
    Separating,
    SkinMiss,
    BroadMiss,
    Sensor,
    Layers,
    SpinCenter,
}
impl Case {
    fn loaded(self) -> bool {
        matches!(self, Self::Loaded | Self::LoadedFour)
    }
    fn admitted(self) -> bool {
        matches!(
            self,
            Self::Tangent | Self::Loaded | Self::LoadedFour | Self::Slow | Self::Normal
        )
    }
    fn substeps(self) -> u8 {
        if matches!(self, Self::LoadedFour) {
            4
        } else {
            1
        }
    }
    fn command(self) -> V {
        match self {
            Self::Normal => V(0.0, 3.0, 0.0),
            Self::Stationary | Self::SpinCenter => V::ZERO,
            Self::Separating => V(3.0, -1.0, 0.0),
            Self::Slow => V(0.001, 0.0, 0.0),
            _ => V(3.0, 0.0, 0.0),
        }
    }
    fn origin(self) -> V {
        match self {
            Self::SkinMiss => V(0.0, -0.515, 0.0),
            Self::BroadMiss => V(4.9, -0.5, 4.9),
            _ => V(0.0, -0.5, 0.0),
        }
    }
}
fn fixture(case: Case, ids: [u64; 2]) -> World {
    let mut w = World::new(Config {
        gravity: V(0.0, -10.0, 0.0),
        substeps: case.substeps(),
        warm_start: !case.loaded(),
        convergence: None,
        ..Config::default()
    })
    .unwrap();
    // In the cold-load reference the external pair precedes this redundant fixed floor.
    // Default warm-start controls retain the fixed floor's existing normal load.
    w.add_body(Body::new(
        BodyId(100),
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
    let mut platform = Body::new(
        BodyId(ids[0]),
        Shape::Box(V(4.0, 0.5, 4.0)),
        case.origin(),
        1.0,
    );
    platform.external = true;
    platform.sensor = matches!(case, Case::Sensor);
    if matches!(case, Case::Layers) {
        platform.layers = CollisionLayers3d::new(1, 0);
    }
    if matches!(case, Case::SpinCenter) {
        platform.angular_velocity = V(0.0, 3.0, 0.0);
    }
    w.add_body(platform).unwrap();
    w.set_velocity(BodyId(ids[0]), case.command()).unwrap();
    assert!(w.body(BodyId(ids[1])).unwrap().is_sleeping());
    w
}
fn observe(w: &World, case: Case, ids: [u64; 2], n: u32) -> Real {
    let b = w.body(BodyId(ids[1])).unwrap();
    let t = H * n as Real;
    let (v, p, spin) = if case.loaded() {
        (
            V(6.0 * t, 0.0, 0.0),
            V(3.0 * H * H * n as Real * (n + 1) as Real, 1.0, 0.0),
            V(0.0, 0.0, 15.0 * t),
        )
    } else if matches!(case, Case::Normal) {
        (V(0.0, 3.0, 0.0), V(0.0, 1.0 + 3.0 * t, 0.0), V::ZERO)
    } else {
        (V::ZERO, V(0.0, 1.0, 0.0), V::ZERO)
    };
    let error = (b.velocity - v)
        .length()
        .max((b.position - p).length())
        .max((b.angular_velocity - spin).length());
    assert!(
        error <= rounding(1e-10, 16.0 * (n as Real + 1.0)),
        "{case:?} ids={ids:?} n={n} error={error} body={b:?}"
    );
    assert_eq!(b.is_sleeping(), !case.admitted());
    assert_eq!(b.mass, 2.0);
    assert!(b.position.finite() && b.velocity.finite() && b.angular_velocity.finite());
    let driver = w.body(BodyId(ids[0])).unwrap();
    assert!(driver.external);
    assert_eq!(driver.velocity, case.command());
    assert!(
        (driver.position - (case.origin() + case.command() * t)).length() <= rounding(1e-10, 8.0)
    );
    let neighbor = w.body(BodyId(30)).unwrap();
    assert!(neighbor.is_sleeping());
    assert_eq!(neighbor.position, V(10.0, 1.0, 0.0));
    assert_eq!(neighbor.velocity, V::ZERO);
    error
}
fn run_case(case: Case, ids: [u64; 2]) -> [Real; 20] {
    let mut w = fixture(case, ids);
    let mut repeated = fixture(case, ids);
    let bytes = w.checkpoint(CONTEXT).unwrap().to_bytes();
    let mut restored = Checkpoint::from_bytes(&bytes, CONTEXT, CheckpointLimits::default())
        .unwrap()
        .restore();
    let before = w.elapsed_seconds();
    let dt = H * Real::from(case.substeps());
    let mut first = None;
    let mut max_error: Real = 0.0;
    // LoadedFour stays in the independently known sliding regime through n=20.
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
        max_error = max_error.max(observe(&w, case, ids, call * u32::from(case.substeps())));
        assert_eq!(r.substeps, u32::from(case.substeps()));
        assert!(r.impulse_iterations <= 8 * u64::from(case.substeps()));
        assert!(r.convergence.constraint_visits <= 8 * r.contact_points);
        assert_eq!(r.position.passes, 0);
        assert!(r.retired.is_empty());
        if call == 1 {
            assert_eq!(r.woken_bodies, u64::from(case.admitted()));
            assert_eq!(
                r.response_preparations,
                if case.admitted() {
                    u64::from(case.substeps())
                } else {
                    0
                }
            );
            assert_eq!(r.inertia_preparations, r.response_preparations);
            if !case.admitted() {
                assert_eq!(r.contact_points, 0);
                assert_eq!(r.convergence.constraint_visits, 0);
            }
            let b = w.body(BodyId(ids[1])).unwrap();
            let energy =
                b.velocity.dot(b.velocity) + 0.4 * b.angular_velocity.dot(b.angular_velocity);
            let work = if case.loaded() {
                3.0 * 2.0 * b.velocity.0
            } else if matches!(case, Case::Normal) {
                3.0 * 2.0 * (b.velocity.1 + 10.0 * dt)
            } else {
                0.0
            };
            assert!(energy + 20.0 * (b.position.1 - 1.0) <= work + rounding(1e-10, 64.0));
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
                Real::from(u8::from(w.body(BodyId(30)).unwrap().is_sleeping())),
            ]);
        }
    }
    assert!((w.elapsed_seconds() - before - 5.0 * dt as Scalar).abs() <= 1e-12);
    let mut values = first.unwrap();
    values[17] = max_error;
    println!(
        "TANGENTIAL_CONTACT_WAKE {{\"case\":\"{case:?}\",\"ids\":{ids:?},\"substeps\":{},\"values\":{values:?}}}",
        case.substeps()
    );
    values
}
pub fn run() -> [[Real; 20]; 24] {
    std::array::from_fn(|i| {
        run_case(
            [
                Case::Tangent,
                Case::Loaded,
                Case::LoadedFour,
                Case::Slow,
                Case::Normal,
                Case::Stationary,
                Case::Separating,
                Case::SkinMiss,
                Case::BroadMiss,
                Case::Sensor,
                Case::Layers,
                Case::SpinCenter,
            ][i / 2],
            if i % 2 == 0 { [1, 10] } else { [10, 1] },
        )
    })
}
