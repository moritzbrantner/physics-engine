//! Per-contact normal impulses in the step report (#575, moenarch-games/shooter#75): opt in,
//! one entry per touching pair and compound child, summed over the step, in pair order.
use physics_engine::BodyId;
use physics_engine::approximate::{
    Body, CompoundChild, Config, ContactImpulse, PreparedCompound, Quaternion, Shape, Vector as V,
    World,
};

const DT: f64 = 1.0 / 60.0;
const G: f32 = 9.81;

fn world(report: bool) -> World {
    let mut w = World::new(Config {
        report_contact_impulses: report,
        ..Config::default()
    })
    .unwrap();
    w.add_body(Body::keyed(1, Shape::HalfSpace, V::ZERO, 0.0))
        .unwrap();
    w
}

fn crate_box(w: &mut World, y: f32, vy: f32) -> BodyId {
    let mut b = Body::keyed(2, Shape::Box(V(0.5, 0.5, 0.5)), V(0.0, y, 0.0), 10.0);
    b.velocity = V(0.0, vy, 0.0);
    b.restitution = 0.0;
    b.sleep_allowed = false;
    w.add_body(b).unwrap()
}

/// Two 0.2 m legs under a 0.1 m deck, legs at x = -0.5 (child 0) and x = 0.5 (child 1).
fn table() -> PreparedCompound {
    PreparedCompound::new(&[
        CompoundChild::new(Shape::Box(V(0.1, 0.2, 0.1)), V(-0.5, 0.0, 0.0), 5.0),
        CompoundChild::new(Shape::Box(V(0.1, 0.2, 0.1)), V(0.5, 0.0, 0.0), 5.0),
        CompoundChild::new(Shape::Box(V(0.7, 0.05, 0.2)), V(0.0, 0.25, 0.0), 2.0),
    ])
    .unwrap()
}

fn place(w: &mut World, c: &PreparedCompound, key: u64, origin: V) -> BodyId {
    let (position, orientation) = c.body_pose(origin, Quaternion::IDENTITY);
    let mut body = Body::keyed(key, Shape::Sphere(1.0), position, 1.0);
    body.orientation = orientation;
    body.sleep_allowed = false;
    w.add_compound(c, body).unwrap()
}

fn close(actual: f32, expected: f32, tolerance: f32) -> bool {
    (actual - expected).abs() <= tolerance * expected.abs()
}

#[test]
fn reporting_is_off_by_default() {
    let mut w = world(false);
    crate_box(&mut w, 0.5, 0.0);
    for _ in 0..10 {
        let report = w.step(DT).unwrap();
        assert!(report.contact_impulses.is_empty());
    }
}

#[test]
fn a_resting_box_reports_its_weight_per_step() {
    let mut w = world(true);
    let floor = w.body_id(1).unwrap();
    let id = crate_box(&mut w, 0.5, 0.0);
    for _ in 0..60 {
        w.step(DT).unwrap();
    }
    let report = w.step(DT).unwrap();
    let [c] = report.contact_impulses.as_slice() else {
        panic!("one contact: {:?}", report.contact_impulses);
    };
    assert_eq!((c.a, c.b, c.child_a, c.child_b), (floor, id, None, None));
    // The floor pushes the box up: the normal points from `a` (the floor) toward `b`.
    assert!(c.normal.dot(V::Y) > 0.999, "{c:?}");
    assert!(
        close(c.normal_impulse, 10.0 * G * DT as f32, 0.02),
        "weight share {c:?}"
    );
    assert!(c.point.1.abs() < 0.01 && c.point.0.abs() < 0.01, "{c:?}");
    assert!(c.point.2.abs() < 0.01, "{c:?}");
}

#[test]
fn impulses_balance_the_momentum_an_impact_takes() {
    let mut w = world(true);
    let id = crate_box(&mut w, 0.6, -5.0);
    let (mut total, mut largest) = (0.0, 0.0f32);
    let steps = 30;
    for _ in 0..steps {
        let report = w.step(DT).unwrap();
        for c in &report.contact_impulses {
            total += c.normal_impulse * c.normal.1;
            largest = largest.max(c.normal_impulse);
        }
    }
    let vy = w.body(id).unwrap().velocity.1;
    // Impulse = change of momentum less gravity's share.
    let expected = 10.0 * (vy + 5.0) + 10.0 * G * (steps as f32 * DT as f32);
    assert!(close(total, expected, 0.01), "{total} vs {expected}");
    // The impact step carries the body's momentum, far above a resting step's weight share.
    assert!(largest > 10.0 * 10.0 * G * DT as f32, "{largest}");
}

#[test]
fn compound_contacts_are_reported_per_child() {
    let mut w = world(true);
    let floor = w.body_id(1).unwrap();
    let c = table();
    let id = place(&mut w, &c, 5, V(0.0, 0.2, 0.0));
    for _ in 0..60 {
        w.step(DT).unwrap();
    }
    let report = w.step(DT).unwrap();
    let [left, right] = report.contact_impulses.as_slice() else {
        panic!("one contact per leg: {:?}", report.contact_impulses);
    };
    for (contact, child, x) in [(left, 0, -0.5), (right, 1, 0.5)] {
        assert_eq!(
            (contact.a, contact.b, contact.child_a, contact.child_b),
            (floor, id, None, Some(child))
        );
        assert!(contact.normal.dot(V::Y) > 0.999, "{contact:?}");
        // Within the leg's footprint.
        assert!((contact.point.0 - x).abs() < 0.1, "{contact:?}");
        // Sequential impulses split a statically indeterminate load only roughly evenly.
        assert!(
            close(contact.normal_impulse, 6.0 * G * DT as f32, 0.2),
            "{contact:?}"
        );
    }
    let weight = 12.0 * G * DT as f32;
    assert!(close(
        left.normal_impulse + right.normal_impulse,
        weight,
        0.02
    ));
}

#[test]
fn a_ccd_projectile_reports_the_child_it_strikes() {
    let mut w = World::new(Config {
        gravity: V::ZERO,
        report_contact_impulses: true,
        ..Config::default()
    })
    .unwrap();
    let c = PreparedCompound::new(&[
        CompoundChild::new(Shape::Box(V(0.2, 0.4, 0.2)), V::ZERO, 20.0),
        // A thin plate in front of the chassis.
        CompoundChild::new(Shape::Box(V(0.3, 0.3, 0.01)), V(0.0, 0.0, 0.5), 2.0),
    ])
    .unwrap();
    let robot = place(&mut w, &c, 10, V::ZERO);
    let mut bullet = Body::keyed(1, Shape::Sphere(0.02), V(0.0, 0.0, 3.0), 0.05);
    bullet.velocity = V(0.0, 0.0, -200.0);
    bullet.ccd = true;
    let bullet = w.add_body(bullet).unwrap();
    let mut hits: Vec<ContactImpulse> = Vec::new();
    for _ in 0..3 {
        hits.extend(w.step(DT).unwrap().contact_impulses);
    }
    assert!(!hits.is_empty());
    let mut total = 0.0;
    for hit in &hits {
        assert_eq!(
            (hit.a, hit.b, hit.child_a, hit.child_b),
            (bullet, robot, None, Some(1))
        );
        // The normal points from the bullet toward the robot.
        assert!(hit.normal.dot(-V::Z) > 0.99, "{hit:?}");
        assert!((hit.point.2 - 0.51).abs() < 0.05, "{hit:?}");
        total += hit.normal_impulse;
    }
    let change = 0.05 * (w.body(bullet).unwrap().velocity.2 + 200.0);
    assert!(close(total, change, 0.01), "{total} vs {change}");
}

#[test]
fn a_retired_projectile_keeps_its_id_in_the_report() {
    let mut w = world(true);
    let target = crate_box(&mut w, 0.5, 0.0);
    let mut shot = Body::keyed(3, Shape::Sphere(0.05), V(3.0, 0.5, 0.0), 0.1);
    shot.velocity = V(-100.0, 0.0, 0.0);
    shot.ccd = true;
    shot.retire_on_impact = true;
    let shot = w.add_body(shot).unwrap();
    let mut found = false;
    for _ in 0..5 {
        let report = w.step(DT).unwrap();
        if report.retired.contains(&shot) {
            let hit = report
                .contact_impulses
                .iter()
                .find(|c| c.b == shot)
                .expect("the impact is reported");
            assert_eq!(hit.a, target);
            assert!(hit.normal.dot(V::X) > 0.99, "{hit:?}");
            assert!(hit.normal_impulse > 0.0);
            found = true;
            break;
        }
    }
    assert!(found);
}

#[test]
fn reporting_does_not_change_the_simulation() {
    let run = |report: bool| {
        let mut w = world(report);
        let c = table();
        let ids = [
            crate_box(&mut w, 2.0, -3.0),
            place(&mut w, &c, 5, V(0.0, 0.6, 0.0)),
            place(&mut w, &c, 6, V(0.3, 2.8, 0.1)),
        ];
        let (mut impulses, mut both) = (0, false);
        for _ in 0..120 {
            let report = w.step(DT).unwrap();
            let key = |c: &ContactImpulse| {
                let order = |id| ids.iter().position(|&x| x == id).map_or(0, |k| k + 1);
                (order(c.a), order(c.b), c.child_a, c.child_b)
            };
            // Pair order (the floor, then by key: the ids are listed so), then the children.
            assert!(report.contact_impulses.is_sorted_by_key(key));
            both |= report
                .contact_impulses
                .iter()
                .any(|c| c.child_a.is_some() && c.child_b.is_some());
            impulses += report.contact_impulses.len();
        }
        // The upper table lands on the lower one: child against child.
        assert!(!report || both);
        let state: Vec<_> = ids
            .iter()
            .map(|&id| {
                let b = w.body(id).unwrap();
                (b.position, b.orientation, b.velocity, b.angular_velocity)
            })
            .collect();
        (format!("{state:?}"), impulses)
    };
    let (on, recorded) = run(true);
    let (off, none) = run(false);
    assert_eq!(on, off);
    assert!(recorded > 0);
    assert_eq!(none, 0);
}
