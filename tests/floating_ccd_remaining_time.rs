//! Remaining-time CCD response (#246/#261): a CCD body that hits a surface mid-substep responds
//! at its time of impact and travels the rest of the substep along the response velocity,
//! re-swept against every candidate.
use physics_engine::{
    BodyId,
    approximate::{
        Body, Checkpoint, CheckpointContext, CheckpointLimits, Config, Real, Report, Shape,
        Vector as V, World,
    },
    numeric::Scalar,
};

const H: Scalar = 1.0 / 240.0;
const CONTEXT: CheckpointContext = CheckpointContext {
    build: [3; 32],
    content: [5; 32],
};

fn fixed(id: u64, half: V, position: V, restitution: Real, friction: Real) -> Body {
    let mut b = Body::new(BodyId(id), Shape::Box(half), position, 0.0);
    b.restitution = restitution;
    b.friction = friction;
    b
}

fn elastic_sphere(id: u64, position: V, velocity: V) -> Body {
    let mut b = Body::new(BodyId(id), Shape::Sphere(0.25), position, 1.0);
    b.velocity = velocity;
    b.restitution = 1.0;
    b.friction = 0.0;
    b.ccd = true;
    b
}

fn one_substep(gravity: V) -> World {
    World::new(Config {
        gravity,
        substeps: 1,
        ..Config::default()
    })
    .unwrap()
}

fn state(w: &World) -> Vec<u8> {
    w.checkpoint(CONTEXT).unwrap().to_bytes()
}

fn close(actual: Real, expected: Real, tolerance: Real) -> bool {
    (actual - expected).abs() <= tolerance
}

/// Case 1: the minimised elastic wall from #246.
fn elastic_wall() -> (World, Report) {
    let mut w = one_substep(V::ZERO);
    w.add_body(fixed(1, V(0.25, 5.0, 5.0), V::ZERO, 1.0, 0.0))
        .unwrap();
    w.add_body(elastic_sphere(2, V(-6.0, 0.0, 0.0), V(4000.0, 0.0, 0.0)))
        .unwrap();
    let report = w.step(H).unwrap();
    (w, report)
}

#[test]
fn elastic_wall_responds_at_impact_and_travels_the_remaining_substep() {
    let (mut w, report) = elastic_wall();
    let tolerance: Real = (1e-9 as Real).max(4096.0 * Real::EPSILON);
    let ball = w.body(BodyId(2)).unwrap();
    let expected = -0.5 - 4000.0 * (H as Real - 5.5 / 4000.0);
    assert!(
        close(ball.position.0, expected, tolerance),
        "x = {}, expected {expected}",
        ball.position.0
    );
    assert!(
        close(ball.velocity.0, -4000.0, tolerance),
        "{:?}",
        ball.velocity
    );
    let energy = 0.5 * ball.velocity.dot(ball.velocity);
    assert!(close(energy, 8_000_000.0, 8_000_000.0 * tolerance));
    assert_eq!(report.substeps, 1);
    assert!(report.swept_contacts > 0);
    assert_eq!(report.geometry.ccd_budget_fallbacks, 0);
    assert_eq!(report.position.passes, 0);
    let before = ball.position.0;
    w.step(H).unwrap();
    let ball = w.body(BodyId(2)).unwrap();
    assert!(close(ball.velocity.0, -4000.0, tolerance));
    assert!(close(
        ball.position.0,
        before - 4000.0 * H as Real,
        16.0 * tolerance
    ));
}

/// Case 2: the predeclared floor/wall/parked-target fixture from #246.
fn parked_target() -> World {
    let mut w = one_substep(V(0.0, -10.0, 0.0));
    w.add_body(fixed(1, V(500.0, 0.5, 500.0), V(0.0, -0.5, 0.0), 0.0, 0.6))
        .unwrap();
    w.add_body(fixed(2, V(0.25, 5.0, 5.0), V(0.0, 5.0, 0.0), 1.0, 0.0))
        .unwrap();
    w.add_body(Body::new(
        BodyId(10),
        Shape::Box(V(1.0, 1.0, 1.0)),
        V(-10.0, 1.0, 0.0),
        2.0,
    ))
    .unwrap();
    for _ in 0..120 {
        w.step(1.0 / 60.0).unwrap();
    }
    assert!(w.body(BodyId(10)).unwrap().is_sleeping());
    w.add_body(elastic_sphere(100, V(-6.0, 1.0, 0.0), V(4000.0, 0.0, 0.0)))
        .unwrap();
    assert!(w.body(BodyId(10)).unwrap().is_sleeping());
    w
}

#[test]
fn returning_ricochet_wakes_and_pushes_the_parked_target_in_the_same_call() {
    let mut w = parked_target();
    let before = w.elapsed_seconds();
    let report = w.step(H).unwrap();
    assert!((w.elapsed_seconds() - before - H).abs() < 1e-12);
    assert_eq!(report.substeps, 1);
    assert!(report.impulse_iterations <= 8);
    assert_eq!(report.position.passes, 0);
    assert!(report.retired.is_empty());
    assert!(report.woken_bodies >= 1);
    assert!(report.swept_contacts >= 2, "wall and target impacts");
    let target = w.body(BodyId(10)).unwrap();
    assert!(
        !target.is_sleeping(),
        "returning CCD contact must wake target"
    );
    assert!(target.velocity.0 < -100.0, "{:?}", target.velocity);
    let sphere = w.body(BodyId(100)).unwrap();
    assert!(sphere.velocity.0 < 0.0);
    // No tunnelling into the target: the sphere stays on the wall side of its face.
    assert!(
        sphere.position.0 - 0.25 >= target.position.0 + 1.0 - Config::default().contact_slop,
        "sphere {:?} target {:?}",
        sphere.position,
        target.position
    );
    // The ~1333 units/s slide on mu = 0.6 under gravity 10 decelerates at about 6 units/s^2
    // (the frictionless sphere behind it may push it slightly): real friction, not forced
    // sleep. The fixture cannot come to rest within 120 ticks.
    let initial = target.velocity.0.abs();
    for _ in 0..120 {
        w.step(H).unwrap();
        for b in w.bodies() {
            assert!(b.position.finite() && b.velocity.finite());
        }
        assert!(!w.body(BodyId(10)).unwrap().is_sleeping());
    }
    let speed = w.body(BodyId(10)).unwrap().velocity.0.abs();
    assert!(speed < initial - 1.0, "{speed} vs {initial}");
}

/// Case 3: a corridor between parallel elastic walls forces more impacts per substep than the
/// velocity-pass budget.
fn corridor() -> World {
    let mut w = one_substep(V::ZERO);
    w.add_body(fixed(1, V(0.25, 5.0, 5.0), V(-1.0, 0.0, 0.0), 1.0, 0.0))
        .unwrap();
    w.add_body(fixed(2, V(0.25, 5.0, 5.0), V(1.0, 0.0, 0.0), 1.0, 0.0))
        .unwrap();
    w.add_body(elastic_sphere(3, V::ZERO, V(4000.0, 0.0, 0.0)))
        .unwrap();
    w
}

#[test]
fn corridor_exhausting_the_pass_budget_falls_back_without_tunnelling() {
    let mut w = corridor();
    let mut fallbacks = 0;
    for _ in 0..4 {
        let before = w.elapsed_seconds();
        let report = w.step(H).unwrap();
        assert!((w.elapsed_seconds() - before - H).abs() < 1e-12);
        fallbacks += report.geometry.ccd_budget_fallbacks;
        let ball = w.body(BodyId(3)).unwrap();
        let slop = Config::default().contact_slop;
        assert!(
            ball.position.0.abs() <= 0.5 + slop,
            "tunnelled: {:?}",
            ball.position
        );
        assert!(close(ball.velocity.0.abs(), 4000.0, 1e-3));
    }
    assert!(fallbacks > 0);
}

#[test]
fn remaining_time_cases_replay_and_continue_from_a_checkpoint() {
    let (a, _) = elastic_wall();
    let (b, _) = elastic_wall();
    assert_eq!(state(&a), state(&b));

    let run = |steps: usize, mut w: World| {
        for _ in 0..steps {
            w.step(H).unwrap();
        }
        w
    };
    let first = run(6, corridor());
    assert_eq!(state(&first), state(&run(6, corridor())));

    let reference = run(60, parked_target());
    assert_eq!(state(&reference), state(&run(60, parked_target())));
    let midpoint = run(30, parked_target());
    let saved = state(&midpoint);
    drop(midpoint);
    let restored = Checkpoint::from_bytes(&saved, CONTEXT, CheckpointLimits::default())
        .unwrap()
        .restore();
    assert_eq!(state(&run(30, restored)), state(&reference));
}

/// Two awake CCD bodies hit each other: the second one continues from the committed impact
/// pose instead of replaying the whole substep with its post-impact velocity.
#[test]
fn head_on_ccd_pair_both_travel_only_the_remaining_substep() {
    let mut w = one_substep(V::ZERO);
    w.add_body(elastic_sphere(1, V(-6.0, 0.0, 0.0), V(4000.0, 0.0, 0.0)))
        .unwrap();
    w.add_body(elastic_sphere(2, V(6.0, 0.0, 0.0), V(-4000.0, 0.0, 0.0)))
        .unwrap();
    let report = w.step(H).unwrap();
    assert!(report.swept_contacts > 0);
    let tolerance: Real = (1e-9 as Real).max(4096.0 * Real::EPSILON);
    let impact = 11.5 / 8000.0;
    let expected = 0.25 + 4000.0 * (H as Real - impact);
    let (a, b) = (w.body(BodyId(1)).unwrap(), w.body(BodyId(2)).unwrap());
    assert!(close(a.velocity.0, -4000.0, tolerance), "{:?}", a.velocity);
    assert!(close(b.velocity.0, 4000.0, tolerance), "{:?}", b.velocity);
    assert!(
        close(a.position.0, -expected, 16.0 * tolerance),
        "a x = {}, expected {}",
        a.position.0,
        -expected
    );
    assert!(
        close(b.position.0, expected, 16.0 * tolerance),
        "b x = {}, expected {expected}",
        b.position.0
    );
}

/// A touching contact the projectile separates from is not an impact and must not use the
/// velocity-pass budget: two passes cover the single real (wall) impact.
#[test]
fn separating_touch_does_not_consume_the_ccd_budget() {
    let mut w = World::new(Config {
        gravity: V::ZERO,
        substeps: 1,
        velocity_iterations: 2,
        ..Config::default()
    })
    .unwrap();
    w.add_body(fixed(1, V(0.25, 5.0, 5.0), V(-0.5, 0.0, 0.0), 1.0, 0.0))
        .unwrap();
    w.add_body(fixed(2, V(0.25, 5.0, 5.0), V(11.0, 0.0, 0.0), 1.0, 0.0))
        .unwrap();
    w.add_body(elastic_sphere(3, V::ZERO, V(4000.0, 0.0, 0.0)))
        .unwrap();
    let report = w.step(H).unwrap();
    assert_eq!(report.geometry.ccd_budget_fallbacks, 0);
    let ball = w.body(BodyId(3)).unwrap();
    let tolerance: Real = (1e-9 as Real).max(4096.0 * Real::EPSILON);
    let expected = 10.5 - 4000.0 * (H as Real - 10.5 / 4000.0);
    assert!(
        close(ball.velocity.0, -4000.0, tolerance),
        "{:?}",
        ball.velocity
    );
    assert!(
        close(ball.position.0, expected, 16.0 * tolerance),
        "x = {}, expected {expected}",
        ball.position.0
    );
}
