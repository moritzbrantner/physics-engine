//! Exact CCD must still find approaching contacts after pruning non-closing relative motion.
use physics_engine::approximate::{Body, BodyStorage, Config, Report, Shape, Vector as V, World};

const H: f64 = 1.0 / 60.0;

fn world(storage: BodyStorage, substeps: u8) -> World {
    World::new(Config {
        gravity: V::ZERO,
        substeps,
        body_storage: storage,
        ..Config::default()
    })
    .unwrap()
}

fn ball(key: u64, x: f32, vx: f32, ccd: bool) -> Body {
    let mut b = Body::keyed(key, Shape::Sphere(0.1), V(x, 0.0, 0.0), 1.0);
    b.velocity = V(vx, 0.0, 0.0);
    b.ccd = ccd;
    b.restitution = 1.0;
    b.friction = 0.0;
    b.sleep_allowed = false;
    b
}

fn pair(
    storage: BodyStorage,
    gap: f32,
    speeds: (f32, f32),
    ccd: bool,
    spinning: bool,
    substeps: u8,
) -> Report {
    let mut w = world(storage, substeps);
    let mut a = ball(10, 0.0, speeds.0, ccd);
    if spinning {
        a.angular_velocity = V(0.0, 0.0, 1.0);
    }
    w.add_body(a).unwrap();
    w.add_body(ball(11, 0.2 + gap, speeds.1, ccd)).unwrap();
    w.step(H).unwrap()
}

#[test]
fn dense_parallel_swept_bounds_skip_all_exact_queries_without_dropping_contacts() {
    for count in [8, 32, 64, 128] {
        let mut results = Vec::new();
        for storage in [BodyStorage::Ordered, BodyStorage::Indexed] {
            let mut w = world(storage, 1);
            for i in 0..128 {
                w.add_body(Body::keyed(
                    1000 + i,
                    Shape::Box(V(0.2, 0.2, 0.2)),
                    V(10_000.0 + i as f32, 0.0, 0.0),
                    0.0,
                ))
                .unwrap();
            }
            let speed = (count as f32 * 0.3 + 1.0) * 60.0;
            for i in 0..count {
                w.add_body(ball(10_000 + i as u64, i as f32 * 0.3, speed, true))
                    .unwrap();
            }
            let r = w.step(H).unwrap();
            assert_eq!(r.pair_tests, (count * (count - 1) / 2) as u64);
            assert_eq!(r.narrow_tests, 0, "count={count}");
            assert_eq!(r.geometry.sweep_queries, 0, "count={count}");
            assert_eq!(r.swept_contacts, 0);
            assert_eq!(r.geometry.ccd_budget_fallbacks, 0);
            assert!(r.contact_events.is_empty());
            let positions: Vec<_> = (0..count)
                .map(|i| {
                    let b = w.body(w.body_id(10_000 + i as u64).unwrap()).unwrap();
                    let expected = i as f32 * 0.3 + speed * H as f32;
                    assert!((b.position.0 - expected).abs() < 0.01);
                    [b.position.0.to_bits(), b.velocity.0.to_bits()]
                })
                .collect();
            results.push(positions);
        }
        assert_eq!(results[0], results[1], "storage divergence at {count}");
    }
}

#[test]
fn relative_gap_guard_keeps_closing_touches_rotation_and_nonccd_on_exact_path() {
    for storage in [BodyStorage::Ordered, BodyStorage::Indexed] {
        // Swept world AABBs overlap, but the relative distance only grows.
        let separating = pair(storage, 0.1, (60.0, 120.0), true, false, 1);
        assert_eq!(separating.pair_tests, 1);
        assert_eq!(separating.narrow_tests, 0);
        assert_eq!(separating.geometry.sweep_queries, 0);

        // A fast approach must still make a real within-tick CCD impact.
        let closing = pair(storage, 0.1, (60.0, 0.0), true, false, 1);
        assert!(closing.narrow_tests > 0);
        assert!(closing.geometry.sweep_queries > 0);
        assert!(closing.swept_contacts > 0);

        // A gap inside the contact slop, and actual overlap, must not be discarded.
        for gap in [0.001, -0.01] {
            let touching = pair(storage, gap, (60.0, 60.0), true, false, 1);
            assert!(touching.narrow_tests > 0, "gap={gap}");
        }

        // Rotating bounds, unlike linear relative motion, cannot be rejected here.
        let spinning = pair(storage, 0.1, (60.0, 60.0), true, true, 1);
        assert!(spinning.narrow_tests > 0);

        // Non-CCD geometry retains its previous current-contact admission path.
        let normal = pair(storage, 0.1, (60.0, 60.0), false, false, 1);
        assert!(normal.narrow_tests > 0);

        // Preserve contact regeneration even when the first substep proves separation.
        let multi = pair(storage, 0.1, (60.0, 60.0), true, false, 2);
        assert_eq!(multi.geometry.sweep_queries, 0);
        assert_eq!(multi.swept_contacts, 0);
    }
}
