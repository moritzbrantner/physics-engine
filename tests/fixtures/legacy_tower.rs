use physics_engine::{
    AngularState3d, AngularVelocity3d, BodyId, Material, Orientation3d, RigidBody, RigidBox3d,
    Vec3i,
};

pub fn minimized_frame_38() -> Vec<RigidBox3d> {
    [
        (
            1,
            Vec3i::new(-11_520, 21_650, 0),
            Vec3i::new(86_400, -7_800, 0),
            Vec3i::new(4_680, 4_680, 4_680),
            24,
            Material::new(120).with_friction(420),
            Orientation3d {
                x: 1,
                y: 75_365_501,
                z: 376_826_795,
                w: 1_002_618_129,
            },
            AngularVelocity3d::new(0, 215_544, 1_077_721),
        ),
        (
            12,
            Vec3i::new(0, 18_876, 0),
            Vec3i::new(0, -472, 0),
            Vec3i::new(3_600, 2_700, 4_320),
            2,
            Material::new(80).with_friction(760),
            Orientation3d::IDENTITY,
            AngularVelocity3d::default(),
        ),
        (
            16,
            Vec3i::new(0, 24_273, 8_640),
            Vec3i::new(0, -582, 0),
            Vec3i::new(3_600, 2_700, 4_320),
            2,
            Material::new(80).with_friction(760),
            Orientation3d::IDENTITY,
            AngularVelocity3d::default(),
        ),
    ]
    .into_iter()
    .map(
        |(id, position, velocity, half, mass, material, orientation, angular)| {
            RigidBox3d::new(
                RigidBody::dynamic(BodyId(id), position, velocity, half)
                    .with_mass(mass)
                    .with_material(material),
                AngularState3d::new(orientation, angular),
            )
            .unwrap()
        },
    )
    .collect()
}

pub fn elastic_corridor() -> Vec<RigidBox3d> {
    let material = Material::new(1_000);
    let half = Vec3i::new(1024, 1024, 1024);
    [
        RigidBody::dynamic(BodyId(1), Vec3i::ZERO, Vec3i::new(20_000, 0, 0), half),
        RigidBody::fixed(BodyId(2), Vec3i::new(4096, 0, 0), half),
        RigidBody::fixed(BodyId(3), Vec3i::new(-4096, 0, 0), half),
    ]
    .into_iter()
    .map(|body| {
        RigidBox3d::new(
            body.with_material(material),
            AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
        )
        .unwrap()
        .with_rotation_locked()
    })
    .collect()
}
