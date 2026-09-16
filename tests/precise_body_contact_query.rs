use std::{hint::black_box, time::Instant};

use physics_engine::{
    AngularState3d, AngularVelocity3d, BodyId, Orientation3d, RigidBody, RigidBox3d,
    RotatingWorld3d, RotatingWorldConfig3d, Vec3i,
};

const FAR_BODY_COUNT: u64 = 256;

fn rotating(body: RigidBody) -> RigidBox3d {
    RigidBox3d::new(
        body,
        AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
    )
    .expect("valid box")
}

fn world(sample: u64) -> RotatingWorld3d {
    let mut world = RotatingWorld3d::new(RotatingWorldConfig3d {
        gravity: Vec3i::ZERO,
        sample_count: 8,
        refinement_steps: 2,
        solver_passes: 4,
        max_events: 8,
    });
    world
        .add_box(rotating(RigidBody::dynamic(
            BodyId(1),
            Vec3i::new(0, 2, 0),
            Vec3i::ZERO,
            Vec3i::new(1, 1, 1),
        )))
        .expect("subject");
    world
        .add_box(rotating(RigidBody::fixed(
            BodyId(2),
            Vec3i::ZERO,
            Vec3i::new(6, 1, 6),
        )))
        .expect("floor");

    let sample_offset = i32::try_from(sample).expect("small benchmark sample");
    for offset in 0..FAR_BODY_COUNT {
        let offset = i32::try_from(offset).expect("small far-body index");
        world
            .add_box(rotating(RigidBody::fixed(
                BodyId(100 + u64::try_from(offset).expect("non-negative far-body index")),
                Vec3i::new(10_000 + offset * 16 + sample_offset, 0, 0),
                Vec3i::new(2, 2, 2),
            )))
            .expect("unrelated fixed body");
    }
    world
}

#[test]
fn precise_body_contacts_match_existing_body_overlap_neighbors() {
    let world = world(0);
    let contacts = world.body_contacts(BodyId(1)).expect("precise contacts");
    assert_eq!(
        contacts
            .iter()
            .map(|contact| contact.other)
            .collect::<Vec<_>>(),
        vec![BodyId(2)],
        "unrelated scene bodies must not enter the subject contact answer"
    );

    let query = world
        .box_by_id(BodyId(1))
        .expect("subject remains")
        .oriented_box();
    let mut broad_neighbors = world
        .overlap_query(query)
        .expect("existing-body overlap query");
    broad_neighbors.retain(|id| *id != BodyId(1));
    assert_eq!(
        broad_neighbors,
        contacts
            .iter()
            .map(|contact| contact.other)
            .collect::<Vec<_>>(),
        "the precise API must preserve the exact neighbor semantics of the broader existing-body query"
    );
}

#[test]
#[ignore = "release-mode diagnostic evidence; run explicitly with --ignored --release --nocapture"]
fn benchmark_precise_single_body_contact_query() {
    const SAMPLES: u64 = 24;
    let worlds = (0..SAMPLES).map(world).collect::<Vec<_>>();

    let precise_started = Instant::now();
    let mut precise_contacts = 0_usize;
    for world in &worlds {
        precise_contacts = precise_contacts.saturating_add(
            black_box(world.body_contacts(BodyId(1)).expect("precise contacts")).len(),
        );
    }
    let precise_elapsed = precise_started.elapsed();

    let broad_started = Instant::now();
    let mut broad_contacts = 0_usize;
    for world in &worlds {
        let query = world
            .box_by_id(BodyId(1))
            .expect("subject remains")
            .oriented_box();
        let hits = black_box(
            world
                .overlap_query(black_box(query))
                .expect("broad overlap"),
        );
        broad_contacts =
            broad_contacts.saturating_add(hits.into_iter().filter(|id| *id != BodyId(1)).count());
    }
    let broad_elapsed = broad_started.elapsed();

    assert_eq!(
        precise_contacts,
        usize::try_from(SAMPLES).expect("small samples")
    );
    assert_eq!(broad_contacts, precise_contacts);
    eprintln!(
        "single-body current contacts: worlds={SAMPLES}, bodies_per_world={}, precise={precise_elapsed:?}, full_graph_existing_body_query={broad_elapsed:?}",
        worlds[0].boxes().count(),
    );
}
