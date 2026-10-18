//! Generational body ids (#296) and key order (#292).
use physics_engine::approximate::{Body, Config, Error, Shape, Vector as V, World};

fn world() -> World {
    World::new(Config {
        gravity: V::ZERO,
        ..Config::default()
    })
    .unwrap()
}

#[test]
fn removed_ids_are_never_reused() {
    let mut w = world();
    let a = w
        .add_body(Body::new(Shape::Sphere(1.0), V::ZERO, 1.0))
        .unwrap();
    assert!(w.remove_body(a).is_some());
    let b = w
        .add_body(Body::new(Shape::Sphere(1.0), V::X * 5.0, 1.0))
        .unwrap();
    assert_eq!(a.slot(), b.slot(), "the slot is pooled");
    assert_ne!(a, b, "but the id is new");
    assert!(w.body(a).is_none());
    assert_eq!(w.set_velocity(a, V::X), Err(Error::MissingBody(a)));
    assert!(w.remove_body(a).is_none());
    assert_eq!(w.body(b).unwrap().position, V::X * 5.0);
}

#[test]
fn keys_are_unique_and_looked_up() {
    let mut w = world();
    let id = w
        .add_body(Body::keyed(42, Shape::Sphere(1.0), V::ZERO, 0.0))
        .unwrap();
    assert_eq!(w.body_id(42), Some(id));
    assert_eq!(w.body(id).unwrap().key, Some(42));
    assert_eq!(
        w.add_body(Body::keyed(42, Shape::Sphere(1.0), V::X * 5.0, 0.0)),
        Err(Error::DuplicateKey(42))
    );
    w.remove_body(id);
    assert_eq!(w.body_id(42), None);
    assert!(
        w.add_body(Body::keyed(42, Shape::Sphere(1.0), V::ZERO, 0.0))
            .is_ok()
    );
}

/// Equal-distance ties break by key, so slot history and insertion order of keyed bodies
/// never change a result.
#[test]
fn ties_follow_keys_not_insertion_or_slot_history() {
    let build = |order: &[u64], churn: bool| {
        let mut w = world();
        if churn {
            for _ in 0..3 {
                let t = w
                    .add_body(Body::new(Shape::Sphere(1.0), V(0.0, 50.0, 0.0), 0.0))
                    .unwrap();
                w.remove_body(t);
            }
        }
        for &key in order {
            w.add_body(Body::keyed(
                key,
                Shape::Box(V(1.0, 1.0, 1.0)),
                V(5.0, 0.0, 0.0),
                0.0,
            ))
            .unwrap();
        }
        w
    };
    for (order, churn) in [
        (&[1, 2, 3][..], false),
        (&[3, 1, 2][..], true),
        (&[2, 3, 1][..], false),
    ] {
        let w = build(order, churn);
        let cast = physics_engine::approximate::RayCast::new(V(-5.0, 0.0, 0.0), V(20.0, 0.0, 0.0));
        let hit = w.cast_ray_first(cast, Default::default()).unwrap().unwrap();
        assert_eq!(w.body(hit.body).unwrap().key, Some(1));
        let mut hits = Vec::new();
        w.cast_ray(cast, Default::default(), &mut hits).unwrap();
        let keys: Vec<_> = hits
            .iter()
            .map(|h| w.body(h.body).unwrap().key.unwrap())
            .collect();
        assert_eq!(keys, [1, 2, 3]);
    }
}
