//! Experimental derived, sparse incident-contact index (#443).
//!
//! Each ordered body has sorted incident *other orders*, not a copy of the authoritative
//! manifold. A cache pair exists exactly once in `World::cache` and twice in this lookup index.
use super::{CachedPoint, Order};
use std::collections::BTreeMap;

type Pair = (Order, Order);

#[derive(Clone, Debug, Default)]
pub(super) struct ContactAdjacency {
    neighbors: BTreeMap<Order, Vec<Order>>,
}

impl ContactAdjacency {
    pub fn incident(&self, order: Order) -> &[Order] {
        self.neighbors.get(&order).map_or(&[], Vec::as_slice)
    }

    fn add(&mut self, order: Order, other: Order) {
        let neighbors = self.neighbors.entry(order).or_default();
        match neighbors.binary_search(&other) {
            Ok(_) => debug_assert!(false, "cache pair indexed twice"),
            Err(at) => {
                neighbors.insert(at, other);
                crate::work::add(crate::work::Counter::AdjacencyUpdates, 1);
            }
        }
    }

    pub fn insert(&mut self, (a, b): Pair) {
        self.add(a, b);
        self.add(b, a);
    }

    fn drop(&mut self, order: Order, other: Order) {
        let Some(neighbors) = self.neighbors.get_mut(&order) else {
            debug_assert!(false, "missing indexed contact body");
            return;
        };
        let Ok(at) = neighbors.binary_search(&other) else {
            debug_assert!(false, "missing indexed contact pair");
            return;
        };
        neighbors.remove(at);
        crate::work::add(crate::work::Counter::AdjacencyUpdates, 1);
        if neighbors.is_empty() {
            self.neighbors.remove(&order);
        }
    }

    pub fn remove(&mut self, (a, b): Pair) {
        self.drop(a, b);
        self.drop(b, a);
    }

    pub fn rebuild(&mut self, cache: &BTreeMap<Pair, Vec<CachedPoint>>) {
        self.neighbors.clear();
        for &key in cache.keys() {
            self.insert(key);
        }
    }

    /// Lower bound on retained payload (excluding BTreeMap node and allocator overhead).
    #[cfg(feature = "work-counters")]
    pub fn payload_capacity_bytes(&self) -> usize {
        self.neighbors.len() * std::mem::size_of::<(Order, Vec<Order>)>()
            + self
                .neighbors
                .values()
                .map(|orders| orders.capacity() * std::mem::size_of::<Order>())
                .sum::<usize>()
    }

    #[cfg(test)]
    pub fn matches_cache(&self, cache: &BTreeMap<Pair, Vec<CachedPoint>>) -> bool {
        let mut reference = Self::default();
        reference.rebuild(cache);
        self.neighbors == reference.neighbors
    }
}

pub(super) fn pair(a: Order, b: Order) -> Pair {
    if a < b { (a, b) } else { (b, a) }
}

#[cfg(test)]
mod tests {
    use crate::approximate::{Body, BodyStorage, Config, Error, Shape, Vector as V, World};

    fn assert_consistent(world: &World) {
        assert!(world.adjacency.matches_cache(&world.cache));
        assert_eq!(
            world
                .adjacency
                .neighbors
                .values()
                .map(Vec::len)
                .sum::<usize>(),
            world.cache.len() * 2
        );
    }

    #[test]
    fn incremental_index_survives_persistence_wake_removal_and_key_reuse() {
        for storage in [BodyStorage::Ordered, BodyStorage::Indexed] {
            let mut world = World::new(Config {
                body_storage: storage,
                ..Config::default()
            })
            .unwrap();
            for k in 0..8 {
                let x = k as f32 * 5.0;
                world
                    .add_body(Body::keyed(
                        2 * k,
                        Shape::Box(V(1.0, 0.25, 1.0)),
                        V(x, 0.25, 0.0),
                        0.0,
                    ))
                    .unwrap();
                world
                    .add_body(Body::keyed(
                        2 * k + 1,
                        Shape::Sphere(0.3),
                        V(x, 0.8, 0.0),
                        1.0,
                    ))
                    .unwrap();
            }
            for _ in 0..120 {
                world.step(1.0 / 60.0).unwrap();
                assert_consistent(&world);
            }
            assert_eq!(world.cache.len(), 8);
            let removed = world.body_id(0).unwrap();
            world
                .set_static_position(removed, V(0.0, 0.1, 0.0))
                .unwrap();
            assert_consistent(&world);
            world.step(1.0 / 60.0).unwrap();
            assert_consistent(&world);
            world.remove_body(removed).unwrap();
            assert_consistent(&world);
            assert!(!world.has_support(world.body_id(1).unwrap()));
            let replacement = world
                .add_body(Body::keyed(
                    0,
                    Shape::Box(V(1.0, 0.25, 1.0)),
                    V(0.0, 0.25, 0.0),
                    0.0,
                ))
                .unwrap();
            assert_ne!(removed, replacement, "reused slot has a new generation");
            for _ in 0..3 {
                world.step(1.0 / 60.0).unwrap();
                assert_consistent(&world);
            }
            assert!(world.has_support(world.body_id(1).unwrap()));

            #[cfg(feature = "checkpoints")]
            {
                let context = crate::approximate::CheckpointContext {
                    build: [9; 32],
                    content: [11; 32],
                };
                let mut restored = world.checkpoint(context).unwrap().restore();
                assert_consistent(&restored);
                for _ in 0..10 {
                    world.step(1.0 / 60.0).unwrap();
                    restored.step(1.0 / 60.0).unwrap();
                    assert_consistent(&restored);
                    assert_eq!(
                        world.bodies().collect::<Vec<_>>(),
                        restored.bodies().collect::<Vec<_>>()
                    );
                }
            }
        }
    }

    #[test]
    fn failed_step_rebuilds_the_prior_index() {
        // Existing contact pairs survive an atomic failure that changed cache history before
        // integration rejected a fast body near the numeric position limit.
        let mut world = World::new(Config::default()).unwrap();
        world
            .add_body(Body::keyed(
                0,
                Shape::Box(V(1.0, 0.25, 1.0)),
                V(0.0, 0.25, 0.0),
                0.0,
            ))
            .unwrap();
        world
            .add_body(Body::keyed(1, Shape::Sphere(0.3), V(0.0, 0.8, 0.0), 1.0))
            .unwrap();
        for _ in 0..3 {
            world.step(1.0 / 60.0).unwrap();
        }
        assert_consistent(&world);
        let contact_count = world.cache.len();
        let scale = 16_777_216.0;
        let mut fast = Body::keyed(
            99,
            Shape::Sphere(0.1),
            V(1e12 - 0.03 * scale, 0.0, 0.0),
            1.0,
        );
        fast.velocity = V(scale, 0.0, 0.0);
        world.add_body(fast).unwrap();
        assert!(matches!(world.step(0.1), Err(Error::NonFiniteState(_))));
        assert_eq!(world.cache.len(), contact_count);
        assert_consistent(&world);
        assert!(world.has_support(world.body_id(1).unwrap()));
    }

    #[test]
    fn high_degree_support_removal_is_local_and_discards_all_pairs() {
        for storage in [BodyStorage::Ordered, BodyStorage::Indexed] {
            let mut world = World::new(Config {
                body_storage: storage,
                ..Config::default()
            })
            .unwrap();
            let platform = world
                .add_body(Body::keyed(
                    0,
                    Shape::Box(V(400.0, 0.5, 2.0)),
                    V(0.0, 0.5, 0.0),
                    0.0,
                ))
                .unwrap();
            for n in 0..128 {
                let mut body = Body::keyed(
                    n + 1,
                    Shape::Sphere(0.3),
                    V(n as f32 * 2.8 - 180.0, 1.3, 0.0),
                    1.0,
                );
                body.sleep_allowed = false;
                world.add_body(body).unwrap();
            }
            for _ in 0..3 {
                world.step(1.0 / 60.0).unwrap();
            }
            assert_eq!(world.cache.len(), 128);
            assert_eq!(world.adjacency.incident(world.bodies[0].order).len(), 128);
            assert_consistent(&world);
            world.remove_body(platform).unwrap();
            assert!(world.cache.is_empty());
            assert_consistent(&world);
            for n in 1..=128 {
                assert!(!world.has_support(world.body_id(n).unwrap()));
            }
        }
    }
}
