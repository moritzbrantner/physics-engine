//! Contact enter/exit events (Q33, D7): `Report::contact_events` reports when the persistent
//! contact cache starts or stops holding a pair, once per transition, in pair order.

use physics_engine::approximate::{Body, Config, ContactEvent, Error, Real, Shape, Vector, World};
use physics_engine::{BodyId, CollisionLayers3d};

const DT: f64 = 1.0 / 60.0;
const FLOOR: u64 = 1;
const PLATE: u64 = 10;
const CRATE: u64 = 100;

fn world() -> World {
    World::new(Config {
        substeps: 2,
        velocity_iterations: 4,
        ..Config::default()
    })
    .unwrap()
}

fn plate(key: u64, x: Real) -> Body {
    Body::keyed(
        key,
        Shape::Box(Vector(0.6, 0.05, 0.6)),
        Vector(x, 0.05, 0.0),
        0.0,
    )
}

fn crate_at(key: u64, x: Real, y: Real) -> Body {
    let mut body = Body::keyed(
        key,
        Shape::Box(Vector(0.25, 0.25, 0.25)),
        Vector(x, y, 0.0),
        75.0,
    );
    body.rotation_locked = true;
    body
}

/// A floor plane and one plate at x = 0, with a crate dropped onto it from `drop` metres.
fn plate_scene(drop: Real) -> World {
    let mut w = world();
    w.add_body(Body::keyed(FLOOR, Shape::HalfSpace, Vector::ZERO, 0.0))
        .unwrap();
    w.add_body(plate(PLATE, 0.0)).unwrap();
    w.add_body(crate_at(CRATE, 0.0, 0.35 + drop)).unwrap();
    w
}

fn id(w: &World, key: u64) -> BodyId {
    w.body_id(key).unwrap()
}

/// Steps `ticks` times and returns every event with the tick it was reported in.
fn run(w: &mut World, ticks: u32) -> Vec<(u32, ContactEvent)> {
    let mut out = Vec::new();
    for tick in 0..ticks {
        let report = w.step(DT).unwrap();
        assert_eq!(report.contact_events, w.last_report.contact_events);
        out.extend(report.contact_events.into_iter().map(|e| (tick, e)));
    }
    out
}

fn events(log: &[(u32, ContactEvent)]) -> Vec<ContactEvent> {
    log.iter().map(|&(_, e)| e).collect()
}

#[test]
fn landing_starts_once_and_resting_and_sleeping_stay_silent() {
    let mut w = plate_scene(0.5);
    let log = run(&mut w, 300);
    let (plate, crate_) = (id(&w, PLATE), id(&w, CRATE));
    assert_eq!(
        events(&log),
        [ContactEvent::Started(plate, crate_)],
        "{log:?}"
    );
    assert!(log[0].0 > 10, "the crate falls before it lands: {log:?}");
    assert!(
        w.body(crate_).unwrap().is_sleeping(),
        "the crate sleeps on the plate"
    );
    // Shoved off the plate, it stops touching the plate and starts touching the floor.
    w.set_velocity(crate_, Vector(4.0, 0.0, 0.0)).unwrap();
    let log = run(&mut w, 120);
    let floor = id(&w, FLOOR);
    let mut seen = events(&log);
    seen.sort_by_key(|e| e.started());
    assert_eq!(
        seen,
        [
            ContactEvent::Stopped(plate, crate_),
            ContactEvent::Started(floor, crate_)
        ],
        "{log:?}"
    );
    let stopped = log.iter().find(|(_, e)| !e.started()).unwrap().0;
    let started = log.iter().find(|(_, e)| e.started()).unwrap().0;
    assert!(stopped <= started + 1, "{log:?}");
}

#[test]
fn simultaneous_contacts_are_reported_in_pair_order() {
    let mut w = world();
    w.add_body(Body::keyed(FLOOR, Shape::HalfSpace, Vector::ZERO, 0.0))
        .unwrap();
    // Added out of key order: events still follow body order (keys).
    for (k, x) in [(3, 6.0), (1, 0.0), (2, 3.0)] {
        w.add_body(plate(PLATE + k, x)).unwrap();
    }
    for (k, x) in [(2, 3.0), (3, 6.0), (1, 0.0)] {
        w.add_body(crate_at(CRATE + k, x, 0.85)).unwrap();
    }
    let log = run(&mut w, 120);
    let expected: Vec<_> = (1..=3)
        .map(|k| ContactEvent::Started(id(&w, PLATE + k), id(&w, CRATE + k)))
        .collect();
    assert_eq!(events(&log), expected, "{log:?}");
    assert!(
        log.iter().all(|&(t, _)| t == log[0].0),
        "identical drops land together"
    );
}

#[test]
fn removing_a_body_stops_its_contacts_in_the_next_step() {
    let mut w = plate_scene(0.0);
    run(&mut w, 60);
    let (plate, crate_, floor) = (id(&w, PLATE), id(&w, CRATE), id(&w, FLOOR));
    w.remove_body(plate).unwrap();
    let report = w.step(DT).unwrap();
    assert_eq!(
        report.contact_events,
        [ContactEvent::Stopped(plate, crate_)]
    );
    // The crate falls 0.1 m onto the floor.
    assert_eq!(
        events(&run(&mut w, 60)),
        [ContactEvent::Started(floor, crate_)]
    );
    // A body re-added with the same key is a new body: its contact is new, not merged with the
    // old one's.
    let mut w = plate_scene(0.0);
    run(&mut w, 60);
    let old = id(&w, CRATE);
    w.remove_body(old).unwrap();
    w.add_body(crate_at(CRATE, 0.0, 0.36)).unwrap();
    let new = id(&w, CRATE);
    assert_ne!(old, new);
    let log = run(&mut w, 60);
    let plate = id(&w, PLATE);
    assert_eq!(
        events(&log),
        [
            ContactEvent::Stopped(plate, old),
            ContactEvent::Started(plate, new)
        ],
        "{log:?}"
    );
}

#[test]
fn retirement_on_impact_starts_and_stops_in_the_same_step() {
    let mut w = world();
    w.add_body(Body::keyed(
        PLATE,
        Shape::Box(Vector(0.5, 2.0, 2.0)),
        Vector(5.0, 0.0, 0.0),
        0.0,
    ))
    .unwrap();
    let mut shot = Body::keyed(CRATE, Shape::Sphere(0.05), Vector(0.0, 0.0, 0.0), 0.1);
    shot.velocity = Vector(60.0, 0.0, 0.0);
    shot.ccd = true;
    shot.retire_on_impact = true;
    let shot = w.add_body(shot).unwrap();
    let wall = id(&w, PLATE);
    let mut log = Vec::new();
    for tick in 0..30 {
        let report = w.step(DT).unwrap();
        if !report.retired.is_empty() {
            assert_eq!(report.retired, [shot]);
            assert_eq!(
                report.contact_events,
                [
                    ContactEvent::Started(wall, shot),
                    ContactEvent::Stopped(wall, shot)
                ]
            );
        }
        log.extend(report.contact_events.into_iter().map(|e| (tick, e)));
    }
    assert_eq!(log.len(), 2, "{log:?}");
}

#[test]
fn layers_sensors_and_kinematic_bodies() {
    let mut w = world();
    w.add_body(Body::keyed(FLOOR, Shape::HalfSpace, Vector::ZERO, 0.0))
        .unwrap();
    // A sensor plate and a plate on a layer the crate does not collide with: the crates fall
    // through both to the floor.
    let mut sensor = plate(PLATE + 1, 0.0);
    sensor.sensor = true;
    w.add_body(sensor).unwrap();
    let mut masked = plate(PLATE + 2, 3.0);
    masked.layers = CollisionLayers3d::new(0b10, 0b10);
    w.add_body(masked).unwrap();
    w.add_body(crate_at(CRATE + 1, 0.0, 0.8)).unwrap();
    let mut layered = crate_at(CRATE + 2, 3.0, 0.8);
    layered.layers = CollisionLayers3d::new(0b01, 0b01);
    w.add_body(layered).unwrap();
    // A kinematic capsule a sphere rolls into: a dynamic-kinematic contact is physical.
    let mut player = Body::keyed(
        PLATE + 3,
        Shape::capsule(0.5, 0.3),
        Vector(8.0, 0.8, 0.0),
        0.0,
    );
    player.external = true;
    w.add_body(player).unwrap();
    let mut boulder = Body::keyed(CRATE + 3, Shape::Sphere(0.3), Vector(6.0, 0.3, 0.0), 50.0);
    boulder.velocity = Vector(3.0, 0.0, 0.0);
    boulder.restitution = 0.5;
    w.add_body(boulder).unwrap();
    let log = run(&mut w, 120);
    let floor = id(&w, FLOOR);
    for (_, e) in &log {
        let (a, b) = e.bodies();
        assert!(a != id(&w, PLATE + 1) && a != id(&w, PLATE + 2), "{log:?}");
        assert!(b != id(&w, PLATE + 1) && b != id(&w, PLATE + 2), "{log:?}");
    }
    for k in [1, 2] {
        assert!(
            events(&log).contains(&ContactEvent::Started(floor, id(&w, CRATE + k))),
            "crate {k} lands on the floor: {log:?}"
        );
    }
    let (player, boulder) = (id(&w, PLATE + 3), id(&w, CRATE + 3));
    assert!(
        events(&log).contains(&ContactEvent::Started(player, boulder)),
        "the boulder hits the player: {log:?}"
    );
    // Every pair alternates Started, Stopped, Started, ...
    let mut pairs: Vec<_> = events(&log).into_iter().map(|e| e.bodies()).collect();
    pairs.sort_by_key(|&(a, b)| (a.to_bits(), b.to_bits()));
    pairs.dedup();
    for pair in pairs {
        let kinds: Vec<_> = events(&log)
            .into_iter()
            .filter(|e| e.bodies() == pair)
            .map(ContactEvent::started)
            .collect();
        assert!(
            kinds.iter().enumerate().all(|(i, &s)| s == (i % 2 == 0)),
            "{pair:?}: {kinds:?}"
        );
    }
}

#[test]
fn a_failed_step_reports_nothing_and_keeps_removals_for_the_next_step() {
    let mut w = plate_scene(0.0);
    run(&mut w, 60);
    let (plate, crate_) = (id(&w, PLATE), id(&w, CRATE));
    w.remove_body(plate).unwrap();
    // A body that leaves the representable range fails the step (floating_step_atomicity).
    let scale: Real = 16_777_216.0;
    let mut failing = Body::keyed(
        99,
        Shape::Sphere(0.1),
        Vector(1e12 - 0.03 * scale, 0.0, 0.0),
        1.0,
    );
    failing.velocity = Vector(scale, 0.0, 0.0);
    let failing = w.add_body(failing).unwrap();
    let before = w.last_report.contact_events.clone();
    for _ in 0..3 {
        assert!(matches!(w.step(0.1), Err(Error::NonFiniteState(_))));
        assert_eq!(w.last_report.contact_events, before);
    }
    w.remove_body(failing).unwrap();
    let report = w.step(DT).unwrap();
    assert_eq!(
        report.contact_events,
        [ContactEvent::Stopped(plate, crate_)]
    );
    assert!(w.step(DT).unwrap().contact_events.is_empty());
}

#[test]
fn replay_is_deterministic_and_independent_of_insertion_order() {
    let scene = |reverse: bool| {
        let mut w = world();
        let mut bodies = vec![Body::keyed(FLOOR, Shape::HalfSpace, Vector::ZERO, 0.0)];
        for k in 0..6u64 {
            let x = 1.5 * k as Real;
            bodies.push(plate(PLATE + k, x));
            let mut c = crate_at(CRATE + k, x + 0.1 * k as Real, 0.5 + 0.2 * k as Real);
            c.rotation_locked = k % 2 == 0;
            c.velocity = Vector(1.0 + k as Real, 0.0, 0.0);
            bodies.push(c);
        }
        if reverse {
            bodies.reverse();
        }
        for b in bodies {
            w.add_body(b).unwrap();
        }
        let log = run(&mut w, 240);
        // Keys identify bodies across the two worlds.
        let key = |id: BodyId| w.body(id).and_then(|b| b.key);
        log.into_iter()
            .map(|(t, e)| {
                let (a, b) = e.bodies();
                (t, e.started(), key(a), key(b))
            })
            .collect::<Vec<_>>()
    };
    let first = scene(false);
    assert!(first.len() >= 12, "{first:?}");
    assert_eq!(first, scene(false));
    assert_eq!(first, scene(true));
    // Pair order within each tick.
    for w in first.windows(2) {
        if w[0].0 == w[1].0 {
            assert!((w[0].2, w[0].3) <= (w[1].2, w[1].3), "{w:?}");
        }
    }
}

/// Impacts the contact cache never holds still count: a CCD bounce (a remaining-time impact)
/// starts and stops; a fast hit that comes to rest through a swept impulse starts once.
#[test]
fn swept_and_ccd_impacts_start_once_and_bounces_stop() {
    for speed in [5.0, 20.0, 80.0, 160.0] {
        // (Without CCD the ball comes to rest at the wall; with it, it bounces.)
        for ccd in [false, true] {
            let mut w = World::new(Config {
                gravity: Vector::ZERO,
                substeps: 2,
                velocity_iterations: 4,
                ..Config::default()
            })
            .unwrap();
            let mut wall = Body::keyed(
                PLATE,
                Shape::Box(Vector(0.5, 2.0, 2.0)),
                Vector(3.0, 0.0, 0.0),
                0.0,
            );
            wall.restitution = 0.8;
            let wall = w.add_body(wall).unwrap();
            let mut ball = Body::keyed(CRATE, Shape::Sphere(0.1), Vector::ZERO, 1.0);
            ball.velocity = Vector(speed, 0.0, 0.0);
            ball.restitution = 0.8;
            ball.ccd = ccd;
            let ball = w.add_body(ball).unwrap();
            let log = run(&mut w, 120);
            let bounced = w.body(ball).unwrap().velocity.0 < 0.0;
            let expected = if bounced {
                vec![
                    ContactEvent::Started(wall, ball),
                    ContactEvent::Stopped(wall, ball),
                ]
            } else {
                vec![ContactEvent::Started(wall, ball)]
            };
            assert_eq!(events(&log), expected, "speed {speed} ccd {ccd}: {log:?}");
            if bounced {
                assert_eq!(
                    log[1].0,
                    log[0].0 + 1,
                    "a bounce stops one step later: {log:?}"
                );
            }
        }
    }
}

/// #456: a kinematic body placed away from (or into) a sleeping body wakes it, so a crate left
/// behind falls and its contact stops instead of floating asleep.
#[test]
fn placing_a_kinematic_body_wakes_its_sleeping_partners() {
    let mut w = world();
    let mut lift = Body::keyed(
        PLATE,
        Shape::Box(Vector(0.6, 0.05, 0.6)),
        Vector(0.0, 0.05, 0.0),
        0.0,
    );
    lift.external = true;
    let lift = w.add_body(lift).unwrap();
    let floor = w
        .add_body(Body::keyed(
            FLOOR,
            Shape::HalfSpace,
            Vector(0.0, -2.0, 0.0),
            0.0,
        ))
        .unwrap();
    let c = w.add_body(crate_at(CRATE, 0.0, 0.36)).unwrap();
    assert_eq!(events(&run(&mut w, 120)), [ContactEvent::Started(lift, c)]);
    assert!(w.body(c).unwrap().is_sleeping());
    w.set_kinematic_position(lift, Vector(5.0, 0.05, 0.0))
        .unwrap();
    assert!(!w.body(c).unwrap().is_sleeping(), "the crate wakes");
    let log = run(&mut w, 120);
    assert_eq!(
        events(&log),
        [
            ContactEvent::Stopped(lift, c),
            ContactEvent::Started(floor, c)
        ],
        "{log:?}"
    );
    assert_eq!(log[0].0, 0, "it stops in the next step: {log:?}");
    assert!(
        w.body(c).unwrap().position.1 < -1.7,
        "the crate fell to the floor"
    );
    // Placed back into the now sleeping crate's reach, it wakes it again.
    assert!(w.body(c).unwrap().is_sleeping());
    let at = w.body(c).unwrap().position;
    w.set_kinematic_position(lift, Vector(at.0, at.1 + 0.25 + 0.05, at.2))
        .unwrap();
    assert!(!w.body(c).unwrap().is_sleeping(), "the crate wakes");
    // An unrelated sleeper far from both poses stays asleep.
    let mut w = plate_scene(0.0);
    let mut door = Body::keyed(
        PLATE + 1,
        Shape::Box(Vector(0.5, 1.0, 0.1)),
        Vector(9.0, 1.0, 0.0),
        0.0,
    );
    door.external = true;
    let door = w.add_body(door).unwrap();
    run(&mut w, 120);
    w.set_kinematic_position(door, Vector(9.0, 1.0, 2.0))
        .unwrap();
    assert!(w.body(id(&w, CRATE)).unwrap().is_sleeping());
}
