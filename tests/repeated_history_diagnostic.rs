use physics_engine::{
    AngularState3d, AngularVelocity3d, BodyId, MATERIAL_SCALE, Material, Orientation3d,
    RepeatedRotatingEventConfig3d, RigidBody, RigidBox3d, RigidBoxFreeFlightConfig3d,
    RotatingContactSearchConfig3d, Vec3i, advance_repeated_rotating_events,
};

fn dynamic(id: u64, position: Vec3i, velocity: Vec3i, material: Material) -> RigidBox3d {
    RigidBox3d::new(
        RigidBody::dynamic(BodyId(id), position, velocity, Vec3i::new(1, 1, 1))
            .with_material(material),
        AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
    )
    .expect("valid dynamic box")
}

fn fixed(id: u64, position: Vec3i, material: Material) -> RigidBox3d {
    RigidBox3d::new(
        RigidBody::fixed(BodyId(id), position, Vec3i::new(1, 1, 1)).with_material(material),
        AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
    )
    .expect("valid fixed box")
}

#[test]
fn print_alternating_history_sequence() {
    let elastic = Material::new(MATERIAL_SCALE);
    let boxes = [
        fixed(1, Vec3i::new(-3, 0, 0), elastic),
        dynamic(2, Vec3i::ZERO, Vec3i::new(100, 0, 0), elastic),
        fixed(3, Vec3i::new(3, 0, 0), elastic),
    ];
    let config = RepeatedRotatingEventConfig3d::new(
        RotatingContactSearchConfig3d::new(
            RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 1, 10),
            2,
            4,
        ),
        8,
        8,
    );
    let advance = advance_repeated_rotating_events(&boxes, config)
        .expect("alternating impact diagnostic must advance");

    panic!("alternating history diagnostic: {:#?}", advance.events);
}
