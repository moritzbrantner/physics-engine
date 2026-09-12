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

fn block(id: u64, x: i32, y: i32, z: i32) -> RigidBox3d {
    RigidBox3d::new(
        RigidBody::dynamic(BodyId(id), Vec3i::new(x, y, z), Vec3i::ZERO, BLOCK_HALF)
            .with_mass(2)
            .with_material(Material::new(80).with_friction(760)),
        AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
    )
    .expect("valid block")
}

#[test]
fn five_row_touching_tower_finishes_first_frame_out_of_floor() {
    let mut world = RotatingWorld3d::new(RotatingWorldConfig3d {
        gravity: Vec3i::new(0, -10 * SCALE, 0),
        sample_count: 8,
        refinement_steps: 4,
        solver_passes: 16,
        max_events: 64,
    });
    world.add_box(floor()).expect("floor");

    let mut id = 2_u64;
    for layer in 0..2_i32 {
        for row in 0..5_i32 {
            for column in 0..3_i32 {
                world
                    .add_box(block(
                        id,
                        layer * 2 * SCALE,
                        2_700 + row * 5_400,
                        (column - 1) * 8_640,
                    ))
                    .expect("block");
                id += 1;
            }
        }
    }

    world.step(1, 60).expect("tower frame");

    for id in 2..32 {
        let rigid_box = world.box_by_id(BodyId(id)).expect("block remains");
        let bottom = rigid_box.body().position().y - BLOCK_HALF.y;
        assert!(
            bottom >= 0,
            "block {id} penetrated below the floor after one frame: {rigid_box:?}"
        );
    }
}
