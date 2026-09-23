use super::*;
use crate::{
    BodyId, CollisionLayers3d,
    approximate::{Config, Shape, Vector as V, World},
};

fn body(id: u64, mass: f64) -> Body {
    Body::new(
        BodyId(id),
        Shape::Sphere(1.0),
        V(id as f64 * 4.0, 1.0, 0.0),
        mass,
    )
}
fn row(a: usize, b: usize) -> Constraint {
    Constraint {
        a,
        b,
        n: V::X,
        ra: V::ZERO,
        rb: V::ZERO,
        t1: V::Y,
        t2: V::Z,
        normal_mass: 1.0,
        tangent_mass: [1.0; 2],
        bias: 0.0,
        friction: 0.5,
        normal_impulse: 0.0,
        tangent_impulse: [0.0; 2],
        sep: 0.0,
        swept: false,
        response: [true, true],
        spin: false,
    }
}
fn execute(
    bodies: &mut [Body],
    constraints: &mut [Constraint],
    scratch: &mut Scratch,
    local: bool,
) -> Report {
    let mut r = Report::default();
    let responses: Vec<_> = bodies
        .iter()
        .map(|b| PreparedResponse::new(b, &mut r))
        .collect();
    if local {
        solve::<true>(
            bodies,
            &responses,
            constraints,
            8,
            Convergence::default(),
            scratch,
            &mut r,
        );
    } else {
        convergence::solve::<true, true>(
            bodies,
            &responses,
            constraints,
            8,
            Convergence::default(),
            &mut r,
        );
    }
    r
}
fn impulse_state(rows: &[Constraint]) -> Vec<(f64, [f64; 2])> {
    rows.iter()
        .map(|c| (c.normal_impulse, c.tangent_impulse))
        .collect()
}
fn groups(scratch: &Scratch, n: usize) -> Vec<Vec<usize>> {
    if scratch.roots.len() == 1 {
        return vec![(0..n).collect()];
    }
    scratch
        .offsets
        .windows(2)
        .map(|r| scratch.rows[r[0]..r[1]].to_vec())
        .collect()
}

#[test]
fn difficult_contacts_do_not_force_unrelated_floor_contacts_to_use_eight_passes() {
    let mut bodies = vec![
        body(0, 0.0),
        body(1, 1.0),
        body(2, 1.0),
        body(3, 1.0),
        body(4, 1.0),
    ];
    bodies[2].velocity = V(-10.0, 0.0, 0.0);
    bodies[3].velocity = V(-2.0, 0.0, 0.0);
    bodies[4].velocity = V(9.0, 3.0, 0.0); // Free-flight projectile, not a constraint participant.
    let mut rows = vec![row(0, 1), row(0, 3), row(1, 2)];
    rows[2].normal_mass = 0.5;
    let (mut reference, mut ref_rows) = (bodies.clone(), rows.clone());
    let global = execute(
        &mut reference,
        &mut ref_rows,
        &mut Scratch::default(),
        false,
    );
    let mut scratch = Scratch::default();
    let local = execute(&mut bodies, &mut rows, &mut scratch, true);
    assert_eq!(groups(&scratch, rows.len()), vec![vec![0, 2], vec![1]]);
    assert_eq!(bodies, reference);
    assert_eq!(impulse_state(&rows), impulse_state(&ref_rows));
    assert_eq!(global.convergence.constraint_visits, 24);
    assert_eq!(local.convergence.constraint_visits, 20);
    assert_eq!(local.islands.island_iterations, 12); // 8 difficult + 4 easy (includes the two shared passes).
    assert_eq!(local.islands.converged_islands, 1);
    assert_eq!(local.islands.capped_islands, 1);
    assert_eq!(local.islands.skipped_constraint_visits, 4);
    assert_eq!(local.impulse_iterations, 8);
    assert_eq!(local.convergence.capped_substeps, 1);
    assert_eq!(local.convergence.converged_substeps, 0);
    assert_eq!(local.islands.dynamic_nodes, 3);
}

#[test]
fn one_way_response_still_connects_mutable_read_dependencies() {
    let mut b = vec![body(0, 0.0), body(1, 1.0), body(2, 1.0), body(3, 1.0)];
    b[2].velocity = V(-10.0, 0.0, 0.0);
    let mut c = vec![row(0, 1), row(1, 3), row(1, 2)];
    c[1].response = [false, true]; // Reads body1, which row2 can change later in each pass.
    c[2].normal_mass = 0.5;
    let (mut ref_b, mut ref_c) = (b.clone(), c.clone());
    let old = execute(&mut ref_b, &mut ref_c, &mut Scratch::default(), false);
    let r = execute(&mut b, &mut c, &mut Scratch::default(), true);
    assert_eq!(r.islands.islands, 1);
    assert_eq!(b, ref_b);
    assert_eq!(impulse_state(&c), impulse_state(&ref_c));
    assert_eq!(r.convergence, old.convergence);
}

#[test]
fn external_and_sleeping_anchors_do_not_merge_but_new_awake_swept_contacts_do() {
    let mut b = vec![
        body(0, 0.0),
        body(1, 1.0),
        body(2, 1.0),
        body(3, 1.0),
        body(4, 1.0),
    ];
    b[3].external = true;
    b[3].velocity = V(1.0, 0.0, 0.0);
    b[4].sleeping = true;
    let mut c = vec![
        row(0, 1),
        row(0, 2),
        row(1, 3),
        row(2, 3),
        row(1, 4),
        row(2, 4),
    ];
    let mut s = Scratch::default();
    s.prepare(&b, &c, &mut IslandStats::default());
    assert_eq!(groups(&s, c.len()), vec![vec![0, 2, 4], vec![1, 3, 5]]);
    // Simulate the real post-admission wake stage: no dependency on previous sleep-cache keys.
    b[4].sleeping = false;
    c[4].swept = true;
    s.prepare(&b, &c, &mut IslandStats::default());
    assert_eq!(groups(&s, c.len()), vec![(0..6).collect::<Vec<_>>()]);
    c.retain(|c| c.a != 4 && c.b != 4);
    s.prepare(&b, &c, &mut IslandStats::default());
    assert_eq!(groups(&s, c.len()), vec![vec![0, 2], vec![1, 3]]);
}

// Independent full-edge traversal, deliberately not the implementation's union-find.
fn traversal_oracle(b: &[Body], c: &[Constraint]) -> Vec<Vec<usize>> {
    let mutable = |i: usize| b[i].mass > 0.0 && !b[i].external && !b[i].is_sleeping();
    let mut seen = vec![false; b.len()];
    let mut result = Vec::new();
    for root in 0..b.len() {
        if seen[root] || !mutable(root) || !c.iter().any(|c| c.a == root || c.b == root) {
            continue;
        }
        let mut stack = vec![root];
        seen[root] = true;
        while let Some(i) = stack.pop() {
            for c in c {
                let other = if c.a == i {
                    c.b
                } else if c.b == i {
                    c.a
                } else {
                    continue;
                };
                if mutable(other) && !seen[other] {
                    seen[other] = true;
                    stack.push(other);
                }
            }
        }
        // Find the component from the root again, separately from previously visited components.
        let mut component = vec![root];
        loop {
            let n = component.len();
            for c in c {
                if component.contains(&c.a) && mutable(c.b) && !component.contains(&c.b) {
                    component.push(c.b);
                }
                if component.contains(&c.b) && mutable(c.a) && !component.contains(&c.a) {
                    component.push(c.a);
                }
            }
            if n == component.len() {
                break;
            }
        }
        result.push(
            c.iter()
                .enumerate()
                .filter(|(_, c)| component.contains(&c.a) || component.contains(&c.b))
                .map(|(i, _)| i)
                .collect(),
        );
    }
    result
}

#[test]
fn partition_matches_independent_oracle_and_keeps_per_island_row_order() {
    let mut random = 71u64;
    let mut next = || {
        random ^= random << 13;
        random ^= random >> 7;
        random ^= random << 17;
        random as usize
    };
    let mut scratch = Scratch::default();
    for trial in 0..400 {
        let mut b: Vec<_> = (0..40)
            .map(|i| body(i * 7 + 1, if i % 9 == 0 { 0.0 } else { 1.0 }))
            .collect();
        b[7].external = true;
        b[13].sleeping = true;
        let mut c = Vec::new();
        for _ in 0..trial % 70 + 1 {
            let (a, z) = (next() % b.len(), next() % b.len());
            if a == z || (!mutable_velocity(&b[a]) && !mutable_velocity(&b[z])) {
                continue;
            }
            let mut r = row(a, z);
            r.swept = next() % 2 == 0;
            r.response = [next() % 2 == 0, true];
            c.push(r);
        }
        if c.is_empty() {
            continue;
        }
        scratch.prepare(&b, &c, &mut IslandStats::default());
        assert_eq!(
            groups(&scratch, c.len()),
            traversal_oracle(&b, &c),
            "trial{trial}"
        );
    }
}

#[test]
fn fresh_partitions_survive_index_shifts_reused_ids_epoch_wrap_and_clone() {
    let mut b = vec![body(0, 0.0), body(3, 1.0), body(7, 1.0), body(20, 1.0)];
    let mut c = vec![row(0, 1), row(1, 2), row(0, 3)];
    let mut s = Scratch::default();
    s.prepare(&b, &c, &mut IslandStats::default());
    assert_eq!(groups(&s, 3), vec![vec![0, 1], vec![2]]);
    b.remove(1); // Removal shifts every remaining index; old ID reuse does not reuse its graph.
    b.insert(0, body(1, 0.0));
    c = vec![row(1, 2), row(0, 3)];
    s.epoch = u64::MAX;
    let mut stats = IslandStats::default();
    s.prepare(&b, &c, &mut stats);
    assert_eq!(stats.epoch_resets, 1);
    assert_eq!(groups(&s, 2), vec![vec![0], vec![1]]);
    let mut clone = s.clone();
    let mut stats = IslandStats::default();
    clone.prepare(&b, &c, &mut stats);
    assert_eq!(groups(&s, 2), groups(&clone, 2));
    assert_eq!(stats.scratch_growths, 0);
}

fn mixed_world(scope: ConvergenceScope, easy: usize, sleeping: bool) -> World {
    let mut w = World::new(Config {
        convergence_scope: scope,
        fixed_position_iterations: 2,
        ..Config::default()
    })
    .unwrap();
    let mut ground = Body::new(
        BodyId(0),
        Shape::Box(V(5000.0, 10.0, 5000.0)),
        V(0.0, -10.0, 0.0),
        0.0,
    );
    ground.friction = 1.0;
    w.add_body(ground).unwrap();
    for level in 0..4 {
        for x in 0..4 {
            for z in 0..2 {
                let id = 1 + level * 8 + x * 2 + z;
                let mut b = Body::new(
                    BodyId(id),
                    Shape::Box(V(18.0, 18.0, 18.0)),
                    V(x as f64 * 36.0, 18.0 + level as f64 * 36.0, z as f64 * 36.0),
                    2.0,
                );
                b.friction = 1.0;
                b.sleep_allowed = sleeping;
                w.add_body(b).unwrap();
            }
        }
    }
    for n in 0..easy {
        let mut b = Body::new(
            BodyId(100 + n as u64),
            Shape::Box(V(18.0, 18.0, 18.0)),
            V(500.0 + (n % 16) as f64 * 80.0, 18.0, (n / 16) as f64 * 80.0),
            2.0,
        );
        b.friction = 1.0;
        b.sleep_allowed = sleeping;
        w.add_body(b).unwrap();
    }
    w
}

fn within(a: V, b: V, limit: f64) {
    assert!((a - b).length() <= limit, "{a:?} vs {b:?}");
}
#[test]
fn real_mixed_scene_reduces_work_preserves_difficult_island_and_near_miss() {
    let mut local = mixed_world(ConvergenceScope::ContactIslands, 32, false);
    let mut global = local.clone();
    global.config.convergence_scope = ConvergenceScope::WholeWorld;
    let mut isolated = mixed_world(ConvergenceScope::ContactIslands, 0, false);
    let mut easy_only = local.clone();
    for id in 1..=32 {
        easy_only.remove_body(BodyId(id));
    }
    let mut local_work = 0;
    let mut global_work = 0;
    for tick in 0..180 {
        if tick == 60 {
            let j = V(80.0, 0.0, 0.0);
            let p = V(0.0, 18.0, 0.0);
            for w in [&mut local, &mut global, &mut isolated] {
                w.apply_impulse(BodyId(1), j, p).unwrap();
            }
        }
        if tick == 90 {
            for w in [&mut local, &mut global, &mut isolated] {
                let mut p = body(1000, 0.2);
                p.position = V(-200.0, 200.0, -200.0);
                p.velocity = V(0.0, 0.0, -500.0);
                p.ccd = true;
                p.layers = CollisionLayers3d::new(2, 2); // Independent free-flight lane, still integrated.
                w.add_body(p).unwrap();
            }
        }
        let a = local.step(1.0 / 60.0).unwrap();
        let b = global.step(1.0 / 60.0).unwrap();
        let hard = isolated.step(1.0 / 60.0).unwrap();
        let easy = easy_only.step(1.0 / 60.0).unwrap();
        assert!(
            a.convergence.constraint_visits
                <= hard.convergence.constraint_visits + 2 * easy.convergence.constraint_visits,
            "mixed easy islands must stop by pass four rather than follow the hard island to eight at tick{tick}"
        );
        local_work += a.convergence.constraint_visits;
        global_work += b.convergence.constraint_visits;
        assert!(a.impulse_iterations <= 32);
        assert_eq!(
            a.impulse_iterations + a.convergence.skipped_iterations,
            u64::from(a.substeps) * 8
        );
        for id in 1..=32 {
            // Same difficult subsystem behaves identically with or without unrelated islands.
            assert_eq!(
                local.body(BodyId(id)),
                isolated.body(BodyId(id)),
                "tick{tick} body{id}"
            );
        }
        for id in 100..132 {
            let l = local.body(BodyId(id)).unwrap();
            let g = global.body(BodyId(id)).unwrap();
            // Deferred grouping can give an easy island four passes instead of its standalone
            // two. Use the same predeclared paired bound, not a false bit-identity claim.
            let alone = easy_only.body(BodyId(id)).unwrap();
            within(l.position, alone.position, 1e-4);
            within(l.velocity, alone.velocity, 1e-4);
            within(l.position, g.position, 1e-4);
            within(l.velocity, g.velocity, 1e-4);
            assert!(l.position.1 >= 17.98 - 1e-8);
        }
    }
    assert!(
        local_work < global_work,
        "local {local_work}, global {global_work}"
    );
}

#[test]
fn sleeping_isolation_support_removal_and_new_projectile_contact_remain_real() {
    let mut w = mixed_world(ConvergenceScope::ContactIslands, 4, true);
    for _ in 0..300 {
        w.step(1.0 / 60.0).unwrap();
    }
    assert!(w.is_quiescent());
    let before: Vec<_> = (100..104)
        .map(|id| w.body(BodyId(id)).unwrap().clone())
        .collect();
    let mut p = Body::new(BodyId(2000), Shape::Sphere(4.0), V(-150.0, 40.0, 0.0), 0.2);
    p.velocity = V(2000.0, 0.0, 0.0);
    p.ccd = true;
    p.retire_on_impact = true;
    w.add_body(p).unwrap();
    let mut saw_wake = false;
    let mut saw_hit = false;
    for _ in 0..120 {
        let r = w.step(1.0 / 60.0).unwrap();
        saw_wake |= r.woken_bodies > 0;
        saw_hit |= r.retired.contains(&BodyId(2000));
        for (i, b) in before.iter().enumerate() {
            assert_eq!(w.body(BodyId(100 + i as u64)).unwrap(), b);
        }
    }
    assert!(saw_wake && saw_hit);
    w.remove_body(BodyId(0));
    w.step(1.0 / 60.0).unwrap();
    assert!(w.body(BodyId(100)).unwrap().velocity.1 < 0.0);
}

#[test]
fn single_island_and_fixed_reference_keep_original_counters_and_momentum() {
    let mut local = World::new(Config {
        gravity: V::ZERO,
        substeps: 1,
        ..Config::default()
    })
    .unwrap();
    for (id, x, v) in [(1, -1.0, 10.0), (2, 1.0, -10.0)] {
        let mut b = Body::new(BodyId(id), Shape::Sphere(1.0), V(x, 0.0, 0.0), 1.0);
        b.velocity = V(v, 0.0, 0.0);
        b.restitution = 1.0;
        local.add_body(b).unwrap();
    }
    let mut global = local.clone();
    global.config.convergence_scope = ConvergenceScope::WholeWorld;
    let a = local.step(1.0 / 60.0).unwrap();
    let b = global.step(1.0 / 60.0).unwrap();
    assert_eq!(local.bodies, global.bodies);
    assert_eq!(a.convergence, b.convergence);
    assert_eq!(local.bodies[0].velocity + local.bodies[1].velocity, V::ZERO);
    for scope in [
        ConvergenceScope::WholeWorld,
        ConvergenceScope::ContactIslands,
    ] {
        let mut w = mixed_world(scope, 4, false);
        w.config.convergence = None;
        let r = w.step(1.0 / 60.0).unwrap();
        assert_eq!(r.impulse_iterations, 32);
        assert_eq!(r.islands, IslandStats::default());
    }
}

#[test]
fn deterministic_replay_insertion_order_dt_changes_and_no_work_paths() {
    let a = mixed_world(ConvergenceScope::ContactIslands, 8, true);
    let mut b = World::new(a.config()).unwrap();
    for body in a.bodies.iter().rev() {
        b.add_body(body.clone()).unwrap();
    }
    let mut a = a;
    for dt in [0.0, 1.0 / 30.0, 1.0 / 120.0, 0.1, 1.0 / 60.0]
        .into_iter()
        .cycle()
        .take(600)
    {
        let ra = a.step(dt).unwrap();
        let rb = b.step(dt).unwrap();
        assert_eq!(a.bodies, b.bodies);
        assert_eq!(ra.convergence, rb.convergence);
        assert_eq!(ra.islands, rb.islands);
        assert_eq!(a.elapsed, b.elapsed);
    }
    let r = a.step(0.0).unwrap();
    assert_eq!(r.islands.partition_builds, 0);
    let mut w = World::new(Config {
        gravity: V(0.0, -10.0, 0.0),
        ..Config::default()
    })
    .unwrap();
    w.add_body(body(1, 1.0)).unwrap();
    w.add_force(BodyId(1), V(2.0, 0.0, 0.0)).unwrap();
    let r = w.step(1.0 / 60.0).unwrap();
    assert_eq!(r.islands.partition_builds, 0);
    assert_eq!(r.impulse_iterations, 0);
    assert_eq!(r.convergence.empty_substeps, 4);
    within(
        w.body(BodyId(1)).unwrap().velocity,
        V(2.0 / 60.0, -10.0 / 60.0, 0.0),
        1e-14,
    );
    assert_eq!(w.elapsed_seconds(), 1.0 / 60.0);
}

#[test]
fn warmed_partition_storage_does_not_grow_with_repeated_queries() {
    let b: Vec<_> = (0..1100)
        .map(|i| body(i, if i == 0 { 0.0 } else { 1.0 }))
        .collect();
    let c: Vec<_> = (1..33).map(|i| row(0, i)).collect();
    let mut s = Scratch::default();
    s.prepare(&b, &c, &mut IslandStats::default());
    for _ in 0..20 {
        let mut stats = IslandStats::default();
        s.prepare(&b, &c, &mut stats);
        assert_eq!(stats.scratch_growths, 0);
        assert_eq!(stats.dynamic_nodes, 32);
        assert_eq!(stats.endpoints_checked, 64);
        assert_eq!(stats.islands, 32);
        assert_eq!(stats.union_attempts, 0);
    }
}

#[test]
fn globally_converged_or_tiny_budgets_do_not_pay_for_partitioning() {
    for limit in [1, 2, 3, 4, 8] {
        let mut w = mixed_world(ConvergenceScope::ContactIslands, 32, false);
        for id in 1..=32 {
            w.remove_body(BodyId(id));
        }
        w.config.velocity_iterations = limit;
        // Isolated spheres have an exact one-row solution; two passes certify it globally.
        for id in 100..132 {
            w.remove_body(BodyId(id));
        }
        for id in 1..33 {
            let mut b = body(id, 1.0);
            b.position = V(id as f64 * 5.0, 1.0, 0.0);
            b.sleep_allowed = false;
            w.add_body(b).unwrap();
        }
        for _ in 0..10 {
            let r = w.step(1.0 / 60.0).unwrap();
            assert_eq!(r.islands.partition_builds, 0);
            assert_eq!(r.islands.scratch_growths, 0);
            assert_eq!(r.islands.scratch_retained_bytes, 0);
            if limit > 4 {
                assert_eq!(r.islands.prefix_converged_substeps, 4);
            }
            assert!(r.impulse_iterations <= u64::from(limit) * 4);
        }
    }
}
