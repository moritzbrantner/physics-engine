use super::*;
use crate::approximate::{Config, Quaternion, Shape, World};
use std::collections::BTreeSet;

fn body(id: u64, position: Vector, mass: f64) -> Body {
    Body::new(
        BodyId(id),
        Shape::Box(Vector(1.0, 1.0, 1.0)),
        position,
        mass,
    )
}
fn link(w: &mut World, a: u64, b: u64) {
    w.cache.insert(
        (BodyId(a), BodyId(b)),
        vec![CachedPoint {
            a: Vector::ZERO,
            b: Vector::ZERO,
            normal: Vector::Y,
            impulse: 0.0,
            tangent: Vector::ZERO,
        }],
    );
    w.bookkeeping.graph.dirty = true;
}
// Independent oracle: the original repeated full-edge scan, including its stack ordering.
fn reference_component(w: &World, root: BodyId) -> Vec<usize> {
    let mut stack = vec![root];
    let mut seen = BTreeSet::new();
    let mut out = Vec::new();
    while let Some(id) = stack.pop() {
        if !seen.insert(id) {
            continue;
        }
        let Ok(i) = w.index(id) else {
            continue;
        };
        if !w.bodies[i].movable() {
            continue;
        }
        out.push(i);
        for &(a, b) in w.cache.keys() {
            if a == id {
                stack.push(b);
            } else if b == id {
                stack.push(a);
            }
        }
    }
    out
}
fn check_components(w: &mut World) {
    w.bookkeeping.ensure_graph(&w.bodies, &w.cache);
    for i in 0..w.bodies.len() {
        let want = reference_component(w, w.bodies[i].id);
        let s = &mut w.bookkeeping;
        s.traversal.begin(w.bodies.len(), &mut s.work);
        s.traversal.collect(i, &w.bodies, &s.graph, &mut s.work);
        assert_eq!(
            s.traversal.island, want,
            "canonical traversal and fixed/external boundaries"
        );
    }
}
fn graph_fixture() -> World {
    let mut w = World::new(Config {
        gravity: Vector::ZERO,
        ..Config::default()
    })
    .unwrap();
    for id in [30, 21, 1, 11, 20, 10] {
        let mut b = body(
            id,
            Vector(id as f64 * 10.0, 0.0, 0.0),
            if id == 1 { 0.0 } else { 1.0 },
        );
        b.external = id == 30;
        b.sleeping = id != 1 && id != 30;
        w.add_body(b).unwrap();
    }
    for (a, b) in [(1, 10), (1, 20), (10, 11), (20, 21), (11, 30), (21, 30)] {
        link(&mut w, a, b);
    }
    w
}
#[test]
fn adjacency_matches_full_edge_scan_and_does_not_bridge_fixed_or_external_bodies() {
    let mut w = graph_fixture();
    check_components(&mut w);
    assert_eq!(w.bookkeeping.work.adjacency_rebuilds, 1);
    assert_eq!(w.wake_island(BodyId(10)), 2);
    assert!(w.body(BodyId(20)).unwrap().sleeping);
    assert!(w.body(BodyId(21)).unwrap().sleeping);
    assert_eq!(w.bookkeeping.work.adjacency_rebuilds, 1);
    check_activity(&mut w);
}
#[test]
fn contact_removal_and_same_id_reinsertion_invalidate_indexed_adjacency() {
    let mut w = graph_fixture();
    check_components(&mut w);
    // Public removal discards incident contacts and wakes only the correct neighbors.
    w.remove_body(BodyId(10)).unwrap();
    assert!(!w.body(BodyId(11)).unwrap().sleeping);
    assert!(w.body(BodyId(20)).unwrap().sleeping);
    let mut replacement = body(10, Vector(-10.0, 5.0, 0.0), 3.0);
    replacement.shape = Shape::Sphere(0.5);
    w.add_body(replacement).unwrap();
    w.add_body(body(0, Vector(-100.0, 0.0, 0.0), 0.0)).unwrap();
    check_components(&mut w);
    assert!(reference_component(&w, BodyId(10)).len() == 1);
    check_activity(&mut w);
    // Contact topology can change without body membership changing.
    w.cache.remove(&(BodyId(20), BodyId(21)));
    link(&mut w, 11, 21);
    check_components(&mut w);
    let before = w.bookkeeping.work.adjacency_rebuilds;
    for points in w.cache.values_mut() {
        points[0].impulse += 2.0;
    }
    check_components(&mut w);
    assert_eq!(
        w.bookkeeping.work.adjacency_rebuilds, before,
        "impulse changes are not adjacency changes"
    );
}
#[test]
fn traversal_epoch_wrap_clears_old_visits() {
    let mut w = graph_fixture();
    check_components(&mut w);
    w.bookkeeping.traversal.epoch = u64::MAX;
    w.bookkeeping.traversal.seen.fill(1);
    check_components(&mut w);
}
fn check_activity(w: &mut World) {
    let s = &mut w.bookkeeping;
    s.activity.refresh(&w.bodies, &mut s.work);
    let expected = w
        .bodies
        .iter()
        .enumerate()
        .filter(|(_, b)| b.mass > 0.0 && !b.sleeping)
        .map(|(i, _)| i)
        .collect::<Vec<_>>();
    assert_eq!(s.activity.indices, expected);
    assert_eq!(s.activity.count, expected.len());
    assert_eq!(w.is_quiescent(), expected.is_empty());
}
#[test]
fn active_membership_tracks_noops_forces_external_bodies_and_lifecycle() {
    let mut w = World::new(Config {
        gravity: Vector::ZERO,
        ..Config::default()
    })
    .unwrap();
    let mut b = body(10, Vector::ZERO, 1.0);
    b.sleeping = true;
    w.add_body(b).unwrap();
    check_activity(&mut w);
    assert!(w.is_quiescent());
    w.step(0.0).unwrap();
    check_activity(&mut w);
    assert!(w.step(f64::NAN).is_err());
    check_activity(&mut w);
    w.add_force(BodyId(10), Vector::X).unwrap();
    check_activity(&mut w);
    assert!(!w.is_quiescent());
    w.step(0.01).unwrap();
    check_activity(&mut w);
    let mut b = body(2, Vector(20.0, 0.0, 0.0), 1.0);
    b.external = true;
    w.add_body(b).unwrap();
    check_activity(&mut w);
    w.set_velocity(BodyId(2), Vector::Y).unwrap();
    w.step(0.01).unwrap();
    check_activity(&mut w);
    w.remove_body(BodyId(10));
    w.remove_body(BodyId(2));
    check_activity(&mut w);
    assert!(w.is_quiescent());
}
fn check_bounds(w: &mut World, h: Scalar) {
    let mut out = Vec::new();
    w.manifolds(h, &mut super::super::Report::default(), &mut out);
    let mut expected = w
        .bodies
        .iter()
        .enumerate()
        .map(|(i, b)| {
            let (lo, hi) = b.cached_bounds;
            let delta = if b.sleeping {
                Vector::ZERO
            } else {
                b.velocity * h
            };
            let angular = b.angular_velocity.length() * b.shape.radius() * h;
            let pad =
                Vector(angular, angular, angular) + Vector(1.0, 1.0, 1.0) * w.config.contact_slop;
            (i, lo.min(lo + delta) - pad, hi.max(hi + delta) + pad)
        })
        .collect::<Vec<_>>();
    expected.sort_by(|a, b| {
        a.1.0
            .total_cmp(&b.1.0)
            .then_with(|| w.bodies[a.0].id.cmp(&w.bodies[b.0].id))
    });
    let got = w
        .bookkeeping
        .bounds
        .iter()
        .map(|r| (r.index, r.lo, r.hi))
        .collect::<Vec<_>>();
    assert_eq!(got, expected);
}
#[test]
fn retained_sweeps_refresh_on_wake_sleep_dt_change_and_same_length_layout_change() {
    let mut w = graph_fixture();
    let mut rotating = body(4, Vector(-100.0, 0.0, 0.0), 1.0);
    rotating.shape = Shape::Box(Vector(0.3, 2.0, 4.0));
    rotating.orientation = Quaternion(0.2, -0.3, 0.4, 0.8).normalized();
    rotating.angular_velocity = Vector(1.0, 2.0, 0.3);
    w.add_body(rotating).unwrap();
    check_bounds(&mut w, 0.01);
    check_bounds(&mut w, 0.02);
    w.set_velocity(BodyId(20), Vector(7.0, -3.0, 2.0)).unwrap();
    check_bounds(&mut w, 0.02);
    // A public physics step updates cached current bounds and contact graph.
    w.step(0.01).unwrap();
    check_bounds(&mut w, 0.01);
    w.remove_body(BodyId(4)).unwrap();
    w.add_body(body(4, Vector(100.0, 10.0, 20.0), 0.0)).unwrap();
    check_bounds(&mut w, 0.001);
}
fn tower_pair() -> World {
    let mut w = World::new(Config::default()).unwrap();
    w.add_body(Body::new(
        BodyId(1),
        Shape::Box(Vector(800.0, 16.0, 800.0)),
        Vector(0.0, -16.0, 0.0),
        0.0,
    ))
    .unwrap();
    for tower in 0..2 {
        for level in 0..4 {
            for row in 0..2 {
                for col in 0..4 {
                    let id = 100 + tower * 100 + level * 8 + row * 4 + col;
                    let mut b = Body::new(
                        BodyId(id),
                        Shape::Box(Vector(18.0, 18.0, 18.0)),
                        Vector(
                            tower as f64 * 400.0 - 54.0 + col as f64 * 36.0,
                            18.0 + level as f64 * 36.0,
                            82.0 + row as f64 * 36.0,
                        ),
                        2.0,
                    );
                    b.friction = 1.0;
                    w.add_body(b).unwrap();
                }
            }
        }
    }
    for _ in 0..240 {
        w.step(1.0 / 60.0).unwrap();
    }
    assert!(
        w.bodies.iter().filter(|b| b.mass > 0.0).all(|b| b.sleeping),
        "awake {:?}",
        w.bodies
            .iter()
            .filter(|b| b.mass > 0.0 && !b.sleeping)
            .map(|b| (b.id, b.velocity, b.quiet_time))
            .collect::<Vec<_>>()
    );
    w
}
#[test]
fn separate_sleeping_towers_stay_independent_through_miss_hit_and_support_removal() {
    let mut w = tower_pair();
    let mut removed_support = w.clone();
    let original = w.bodies.clone();
    let mut miss = Body::new(
        BodyId(1000),
        Shape::Sphere(3.0),
        Vector(140.0, 250.0, 100.0),
        1.0,
    );
    miss.velocity = Vector(0.0, 0.0, -5760.0);
    miss.ccd = true;
    w.add_body(miss).unwrap();
    for _ in 0..2 {
        let r = w.step(1.0 / 60.0).unwrap();
        assert_eq!(r.woken_bodies, 0);
    }
    w.remove_body(BodyId(1000)).unwrap();
    assert_eq!(w.bodies, original);
    let mut shot = Body::new(
        BodyId(1000),
        Shape::Sphere(3.0),
        Vector(-140.0, 50.0, 82.0),
        1.0,
    );
    shot.velocity = Vector(5760.0, 0.0, 0.0);
    shot.ccd = true;
    shot.retire_on_impact = true;
    w.add_body(shot).unwrap();
    let mut woke = 0;
    for _ in 0..12 {
        woke += w.step(1.0 / 60.0).unwrap().woken_bodies;
    }
    assert!(woke > 0);
    for b in original.iter().filter(|b| b.id.0 >= 200) {
        assert_eq!(w.body(b.id), Some(b));
    }
    removed_support.remove_body(BodyId(100)).unwrap();
    removed_support.step(1.0 / 60.0).unwrap();
    assert!(!removed_support.body(BodyId(108)).unwrap().sleeping);
    for b in original.iter().filter(|b| b.id.0 >= 200) {
        assert_eq!(removed_support.body(b.id), Some(b));
    }
    check_activity(&mut w);
    check_activity(&mut removed_support);
}
#[test]
fn stationary_contact_keys_and_point_storage_survive_updated_impulses() {
    let mut w = World::new(Config {
        gravity: Vector(0.0, -10.0, 0.0),
        ..Config::default()
    })
    .unwrap();
    w.add_body(body(1, Vector(0.0, -1.0, 0.0), 0.0)).unwrap();
    let mut b = body(2, Vector(0.0, 1.0, 0.0), 1.0);
    b.rotation_locked = true;
    b.sleep_allowed = false;
    w.add_body(b).unwrap();
    w.step(1.0 / 60.0).unwrap();
    let ptr = w.cache[&(BodyId(1), BodyId(2))].as_ptr();
    for _ in 0..20 {
        let r = w.step(1.0 / 60.0).unwrap();
        assert_eq!(r.bookkeeping.adjacency_rebuilds, 0);
        assert_eq!(w.cache[&(BodyId(1), BodyId(2))].as_ptr(), ptr);
    }
}
fn sleeping_neighborhood(count: u64) -> World {
    let mut w = World::new(Config {
        gravity: Vector::ZERO,
        ..Config::default()
    })
    .unwrap();
    // Seed an already sleeping contact graph, then prove per-step traversal does not touch it.
    for id in 1..=count {
        let mut b = body(id, Vector(0.0, id as f64 * 2.0, 0.0), 1.0);
        b.sleeping = true;
        w.add_body(b).unwrap();
        if id > 1 {
            link(&mut w, id - 1, id);
        }
    }
    let mut p = body(100_000, Vector(1000.0, 0.0, 0.0), 1.0);
    p.velocity = Vector::Z;
    p.sleep_allowed = false;
    w.add_body(p).unwrap();
    w
}
#[test]
fn warmed_bookkeeping_visits_only_active_neighborhood_and_reuses_buffer_capacity() {
    for count in [32, 128, 512] {
        let mut w = sleeping_neighborhood(count);
        w.step(1.0 / 60.0).unwrap();
        w.step(1.0 / 60.0).unwrap();
        let r = w.step(1.0 / 60.0).unwrap();
        assert_eq!(r.bookkeeping.active_view_rebuilds, 0);
        assert_eq!(r.bookkeeping.adjacency_rebuilds, 0);
        assert_eq!(r.bookkeeping.island_body_visits, 4);
        assert_eq!(r.bookkeeping.island_edge_visits, 0);
        assert_eq!(r.bookkeeping.bounds_updates, 4);
        assert_eq!(r.bookkeeping.bound_rows_sorted, 0);
        assert!(r.bookkeeping.bounds_order_checks > 0);
        assert_eq!(r.bookkeeping.scratch_growths, 0);
        assert!(r.bookkeeping.scratch_retained_bytes > 0);
        assert_eq!(
            w.bodies.iter().filter(|b| b.sleeping).count(),
            count as usize
        );
    }
}
#[test]
#[ignore = "advisory timing; deterministic work bounds are tested separately"]
fn sleeping_neighborhood_bookkeeping_scaling_benchmark() {
    for count in [32, 128, 512, 2048] {
        let mut w = sleeping_neighborhood(count);
        w.step(1.0 / 60.0).unwrap();
        w.step(1.0 / 60.0).unwrap();
        let begin = std::time::Instant::now();
        for _ in 0..120 {
            w.step(1.0 / 60.0).unwrap();
        }
        println!(
            "BOOKKEEPING sleepers={count} ticks=120 elapsed_ns={} last={:?}",
            begin.elapsed().as_nanos(),
            w.last_report.bookkeeping
        );
    }
}
