//! Per-body additional solver iterations (#521), Rapier's `additional_solver_iterations`: the
//! contact island holding a body that requests extra passes runs the world's passes plus the
//! largest request among its bodies; every other island keeps exactly the world's passes.
use physics_engine::approximate::{Body, Config, Error, Quaternion, Shape, Vector as V, World};

const DT: f64 = 1.0 / 30.0;

/// Two tilted boxes stacked on a fixed slab: a stack whose solve is far from converged after
/// one pass. `z` offsets the stack; `keys` start at `key`.
fn stack(world: &mut World, key: u64, z: f32, extra: [u8; 2]) {
    for (k, (y, extra)) in [(0.5, extra[0]), (1.45, extra[1])].into_iter().enumerate() {
        let mut b = Body::keyed(
            key + k as u64,
            Shape::Box(V(0.5, 0.5, 0.5)),
            V(0.1 * k as f32, y, z),
            1.0 + 4.0 * k as f32,
        );
        let (s, c) = (0.05f32 * (k as f32 + 1.0)).sin_cos();
        b.orientation = Quaternion(s, 0.0, 0.0, c);
        b.velocity = V(0.3, -1.0, 0.0);
        b.sleep_allowed = false;
        b.additional_solver_iterations = extra;
        world.add_body(b).unwrap();
    }
}

fn world(passes: u8, stacks: &[(f32, [u8; 2])]) -> World {
    let mut w = World::new(Config {
        substeps: 2,
        velocity_iterations: passes,
        // Fixed passes: an island's extra passes are then exactly more passes.
        convergence: None,
        ..Config::default()
    })
    .unwrap();
    w.add_body(Body::keyed(
        1,
        Shape::Box(V(20.0, 0.1, 20.0)),
        V(0.0, -0.1, 0.0),
        0.0,
    ))
    .unwrap();
    for (n, &(z, extra)) in stacks.iter().enumerate() {
        stack(&mut w, 10 + 10 * n as u64, z, extra);
    }
    w
}

/// Every body's pose and velocity bits, in body order.
fn state(w: &World, keys: std::ops::Range<u64>) -> Vec<[u32; 13]> {
    keys.filter_map(|k| w.body_id(k))
        .map(|id| {
            let b = w.body(id).unwrap();
            let (p, v, q, a) = (b.position, b.velocity, b.orientation, b.angular_velocity);
            [
                p.0, p.1, p.2, v.0, v.1, v.2, q.0, q.1, q.2, q.3, a.0, a.1, a.2,
            ]
            .map(f32::to_bits)
        })
        .collect()
}

fn run(w: &mut World, steps: usize) {
    for _ in 0..steps {
        w.step(DT).unwrap();
    }
}

#[test]
fn an_elevated_island_solves_like_a_world_with_its_passes() {
    // Both boxes ask for three more passes: whether they touch or not, each island gets them.
    let mut elevated = world(1, &[(0.0, [3, 3])]);
    let mut four = world(4, &[(0.0, [0, 0])]);
    run(&mut elevated, 20);
    run(&mut four, 20);
    assert_eq!(state(&elevated, 10..12), state(&four, 10..12));
    assert!(elevated.last_report.islands.elevated_islands > 0);
    // One requesting body changes its own island's solve.
    let mut top = world(1, &[(0.0, [0, 3])]);
    let mut one = world(1, &[(0.0, [0, 0])]);
    run(&mut top, 20);
    run(&mut one, 20);
    assert_ne!(state(&top, 10..12), state(&one, 10..12));
}

#[test]
fn other_islands_keep_the_world_passes() {
    // Two stacks far apart: only the elevated one changes. Twelve steps: through both landings,
    // before the stacks' later bounces diverge in time (a swept contact in one stack makes the
    // whole world regenerate its contact rows that substep, as it always has).
    let mut both = world(1, &[(0.0, [3, 3]), (10.0, [0, 0])]);
    let mut plain = world(1, &[(0.0, [0, 0]), (10.0, [0, 0])]);
    let mut four = world(4, &[(0.0, [0, 0]), (10.0, [0, 0])]);
    run(&mut both, 12);
    run(&mut plain, 12);
    run(&mut four, 12);
    assert_eq!(state(&both, 20..22), state(&plain, 20..22));
    assert_eq!(state(&both, 10..12), state(&four, 10..12));
    let r = both.last_report.islands;
    assert_eq!(r.elevated_islands, 2, "one island per substep");
    assert_eq!(r.extra_iterations, 2 * 3);
}

#[test]
fn zero_is_the_default_and_unrequested_worlds_never_look() {
    assert_eq!(
        Body::new(Shape::Sphere(1.0), V::ZERO, 1.0).additional_solver_iterations,
        0
    );
    let mut plain = world(1, &[(0.0, [0, 0])]);
    run(&mut plain, 5);
    let r = plain.last_report.islands;
    assert_eq!(
        (
            r.elevated_islands,
            r.extra_iterations,
            r.extra_constraint_visits
        ),
        (0, 0, 0)
    );
    assert_eq!(r.partition_builds, 0);
}

#[test]
fn a_removed_elevated_body_stops_the_extra_passes() {
    let mut w = world(1, &[(0.0, [0, 3])]);
    run(&mut w, 2);
    assert!(w.last_report.islands.elevated_islands > 0);
    let top = w.body_id(11).unwrap();
    w.remove_body(top);
    run(&mut w, 2);
    assert_eq!(w.last_report.islands.elevated_islands, 0);
    assert_eq!(w.last_report.islands.partition_builds, 0);
}

#[test]
fn requests_above_sixty_four_are_rejected() {
    let mut w = world(1, &[]);
    let mut b = Body::new(Shape::Sphere(0.5), V(0.0, 3.0, 0.0), 1.0);
    b.additional_solver_iterations = 64;
    w.add_body(b.clone()).unwrap();
    b.additional_solver_iterations = 65;
    assert_eq!(w.add_body(b), Err(Error::InvalidInput));
}

#[test]
fn immovable_and_sleeping_requests_are_unused() {
    // A fixed slab's request joins no island.
    let mut w = World::new(Config {
        substeps: 2,
        velocity_iterations: 1,
        convergence: None,
        ..Config::default()
    })
    .unwrap();
    let mut slab = Body::keyed(1, Shape::Box(V(20.0, 0.1, 20.0)), V(0.0, -0.1, 0.0), 0.0);
    slab.additional_solver_iterations = 8;
    w.add_body(slab).unwrap();
    stack(&mut w, 10, 0.0, [0, 0]);
    let mut plain = world(1, &[(0.0, [0, 0])]);
    run(&mut w, 10);
    run(&mut plain, 10);
    assert_eq!(state(&w, 10..12), state(&plain, 10..12));
    assert_eq!(w.last_report.islands.elevated_islands, 0);
}
