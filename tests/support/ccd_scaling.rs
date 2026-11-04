//! Shared deterministic 60 Hz workload for native replay and Big-O evidence.
//! All active spheres hit one wall simultaneously, but occupy distinct lanes.

use physics_engine::{
    approximate::{Body, BodyStorage, Config, Shape, Vector as V, World},
    work::{self, Counter},
};

pub(crate) const H: f64 = 1.0 / 60.0;

#[derive(Debug, PartialEq)]
pub(crate) struct Sample {
    pub(crate) states: Vec<([u32; 3], [u32; 3])>,
    pub(crate) pairs: u64,
    pub(crate) narrows: u64,
    pub(crate) sweeps: u64,
    pub(crate) tree_nodes: u64,
    pub(crate) impacts: u64,
    pub(crate) fallbacks: u64,
}

pub(crate) fn world(count: usize, irrelevant: usize, storage: BodyStorage) -> World {
    world_with_ccd(count, irrelevant, storage, true)
}

pub(crate) fn world_with_ccd(
    count: usize,
    irrelevant: usize,
    storage: BodyStorage,
    ccd: bool,
) -> World {
    build_world(count, irrelevant, storage, ccd, false)
}

fn build_world(
    count: usize,
    irrelevant: usize,
    storage: BodyStorage,
    ccd: bool,
    two_walls: bool,
) -> World {
    let mut w = World::new(Config {
        gravity: V::ZERO,
        substeps: 1,
        velocity_iterations: 8,
        body_storage: storage,
        ..Config::default()
    })
    .unwrap();

    let right = if two_walls { 0.25 } else { 0.45 };
    let mut wall = Body::keyed(1, Shape::Box(V(0.05, 64.0, 2.0)), V(right, 0.0, 0.0), 0.0);
    wall.restitution = 1.0;
    wall.friction = 0.0;
    w.add_body(wall).unwrap();
    if two_walls {
        let mut left = Body::keyed(
            9000,
            Shape::Box(V(0.05, 64.0, 2.0)),
            V(-0.25, 0.0, 0.0),
            0.0,
        );
        left.restitution = 1.0;
        left.friction = 0.0;
        w.add_body(left).unwrap();
    }

    // Distant statics are deliberately outside every moving sphere's sweep.
    for k in 0..irrelevant {
        w.add_body(Body::keyed(
            2 + k as u64,
            Shape::Box(V(0.2, 0.2, 0.2)),
            V(100.0 + k as f32, 0.0, 0.0),
            0.0,
        ))
        .unwrap();
    }
    for k in 0..count {
        let mut b = Body::keyed(
            10_000 + k as u64,
            Shape::Sphere(0.1),
            V(0.0, k as f32 * 0.5, 0.0),
            1.0,
        );
        b.velocity = V(if two_walls { 24.0 } else { 45.0 }, 0.0, 0.0);
        b.restitution = 1.0;
        b.friction = 0.0;
        b.ccd = ccd;
        b.sleep_allowed = false;
        w.add_body(b).unwrap();
    }
    w
}

pub(crate) fn run(count: usize, irrelevant: usize, storage: BodyStorage) -> Sample {
    sample(world(count, irrelevant, storage), count)
}

pub(crate) fn run_ricochets(count: usize, irrelevant: usize, storage: BodyStorage) -> Sample {
    sample(build_world(count, irrelevant, storage, true, true), count)
}

fn sample(mut w: World, count: usize) -> Sample {
    work::reset();
    let r = w.step(H).unwrap();
    let states = (0..count)
        .map(|k| {
            let b = w.body(w.body_id(10_000 + k as u64).unwrap()).unwrap();
            (
                [
                    b.position.0.to_bits(),
                    b.position.1.to_bits(),
                    b.position.2.to_bits(),
                ],
                [
                    b.velocity.0.to_bits(),
                    b.velocity.1.to_bits(),
                    b.velocity.2.to_bits(),
                ],
            )
        })
        .collect();
    Sample {
        states,
        pairs: r.pair_tests,
        narrows: r.narrow_tests,
        sweeps: r.geometry.sweep_queries,
        tree_nodes: work::get(Counter::TreeNodes),
        impacts: r.swept_contacts,
        fallbacks: r.geometry.ccd_budget_fallbacks,
    }
}
