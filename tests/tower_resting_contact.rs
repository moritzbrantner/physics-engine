use physics_engine::{
    AngularState3d, AngularVelocity3d, BodyId, Material, Orientation3d, RigidBody, RigidBox3d,
    RotatingWorld3d, RotatingWorldConfig3d, Vec3i,
};

const SCALE: i32 = 3_600;
const BLOCK_HALF: Vec3i = Vec3i::new(SCALE, 2_700, 4_320);

fn floor() -> RigidBox3d {
    RigidBox3d::new(
        RigidBody::fixed(
            BodyId(0),
            Vec3i::new(0, -SCALE, 0),
            Vec3i::new(30 * SCALE, SCALE, 12 * SCALE),
        )
        .with_material(Material::new(50).with_friction(850)),
        AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
    )
    .expect("valid floor")
}

fn block(id: u64, x: i32, z: i32) -> RigidBox3d {
    RigidBox3d::new(
        RigidBody::dynamic(BodyId(id), Vec3i::new(x, 2_700, z), Vec3i::ZERO, BLOCK_HALF)
            .with_mass(2)
            .with_material(Material::new(80).with_friction(760)),
        AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
    )
    .expect("valid block")
}

#[test]
fn scaled_touching_tower_row_stays_on_floor_after_one_frame() {
    let mut world = RotatingWorld3d::new(RotatingWorldConfig3d {
        gravity: Vec3i::new(0, -10 * SCALE, 0),
        sample_count: 8,
        refinement_steps: 4,
        solver_passes: 10,
        max_events: 64,
    });
    world.add_box(floor()).expect("floor");

    let mut id = 2_u64;
    for x in [0, 2 * SCALE] {
        for z in [-8_640, 0, 8_640] {
            world.add_box(block(id, x, z)).expect("block");
            id += 1;
        }
    }

    world.step(1, 60).expect("resting tower row frame");

    for id in 2..8 {
        let body = world.box_by_id(BodyId(id)).expect("block remains").body();
        assert!(
            body.position().y >= BLOCK_HALF.y,
            "block {id} sank below the floor after one frame: {body:?}"
        );
    }
}
