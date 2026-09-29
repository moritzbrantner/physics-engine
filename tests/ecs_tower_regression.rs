#[path = "fixtures/legacy_tower.rs"]
mod legacy_tower;

use physics_engine::{
    AngularState3d, AngularVelocity3d, BallisticSphere3d, BodyId, BodyKind, MATERIAL_SCALE,
    Material, Orientation3d, RigidBody, RigidBox3d, RotatingWorld3d, RotatingWorldConfig3d, Vec3i,
    obb_contact_seed,
};

const SCALE: i32 = 3_600;
const ROWS: u64 = 5;
const COLUMNS: u64 = 3;
const LAYERS: u64 = 2;
const FIRST_BLOCK_ID: u64 = 2;
const DAMPING_MILLI: u16 = 996;

fn tower_boxes() -> Vec<RigidBox3d> {
    let mut boxes = Vec::new();
    boxes.push(
        RigidBox3d::new(
            RigidBody::fixed(
                BodyId(0),
                Vec3i::new(0, -SCALE, 0),
                Vec3i::new(30 * SCALE, SCALE, 12 * SCALE),
            )
            .with_material(Material::new(50).with_friction(850)),
            AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
        )
        .expect("valid tower floor"),
    );
    boxes.push(
        RigidBox3d::new(
            RigidBody::dynamic(
                BodyId(1),
                Vec3i::new(-18 * SCALE, 19_800, 0),
                Vec3i::new(24 * SCALE, 4 * SCALE, 0),
                Vec3i::new(4_680, 4_680, 4_680),
            )
            .with_mass(24)
            .with_material(Material::new(120).with_friction(420)),
            AngularState3d::new(
                Orientation3d::IDENTITY,
                AngularVelocity3d::new(0, 250_000, 1_250_000),
            ),
        )
        .expect("valid tower projectile"),
    );

    let half_extents = Vec3i::new(SCALE, 2_700, 4_320);
    let material = Material::new(80).with_friction(760);
    let mut id = FIRST_BLOCK_ID;
    for layer in 0..LAYERS {
        for row in 0..ROWS {
            for column in 0..COLUMNS {
                let x = i32::try_from(layer).expect("layer fits i32") * 2 * SCALE;
                let y = 2_700 + i32::try_from(row).expect("row fits i32") * 5_400;
                let z = (i32::try_from(column).expect("column fits i32") - 1) * 8_640;
                boxes.push(
                    RigidBox3d::new(
                        RigidBody::dynamic(
                            BodyId(id),
                            Vec3i::new(x, y, z),
                            Vec3i::ZERO,
                            half_extents,
                        )
                        .with_mass(2)
                        .with_material(material),
                        AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
                    )
                    .expect("valid tower block"),
                );
                id += 1;
            }
        }
    }
    boxes
}

fn world_config() -> RotatingWorldConfig3d {
    RotatingWorldConfig3d {
        gravity: Vec3i::new(0, -10 * SCALE, 0),
        sample_count: 8,
        refinement_steps: 4,
        solver_passes: 10,
        max_events: 64,
    }
}

fn damp_axis(value: i32) -> i32 {
    let numerator = i128::from(value) * i128::from(DAMPING_MILLI);
    let denominator = i128::from(MATERIAL_SCALE);
    let half = denominator / 2;
    let adjusted = if numerator >= 0 {
        numerator + half
    } else {
        numerator - half
    };
    i32::try_from(adjusted / denominator).expect("damped angular velocity fits i32")
}

fn damped_box(rigid_box: &RigidBox3d) -> RigidBox3d {
    let body = rigid_box.body();
    let rebuilt_body = match body.kind() {
        BodyKind::Dynamic => RigidBody::dynamic(
            body.id(),
            body.position(),
            body.velocity(),
            body.half_extents(),
        )
        .with_mass(body.mass_units())
        .with_material(body.material()),
        BodyKind::Fixed => RigidBody::fixed(body.id(), body.position(), body.half_extents())
            .with_material(body.material()),
    };
    let angular = rigid_box.angular();
    let velocity = if body.kind() == BodyKind::Dynamic {
        AngularVelocity3d::new(
            damp_axis(angular.angular_velocity.x),
            damp_axis(angular.angular_velocity.y),
            damp_axis(angular.angular_velocity.z),
        )
    } else {
        angular.angular_velocity
    };
    RigidBox3d::new(
        rebuilt_body,
        AngularState3d::new(angular.orientation, velocity),
    )
    .expect("damped tower body remains valid")
}

fn step_frame(boxes: &[RigidBox3d], frame: u32, with_ballistic: bool) -> Vec<RigidBox3d> {
    let mut world = RotatingWorld3d::new(world_config());
    for rigid_box in boxes {
        world
            .add_box(rigid_box.clone())
            .expect("tower body should enter engine world");
    }
    if with_ballistic {
        world
            .add_ballistic_sphere(
                BallisticSphere3d::new(
                    BodyId(100),
                    Vec3i::new(200 * SCALE, 200 * SCALE, 0),
                    Vec3i::ZERO,
                    360,
                    1,
                )
                .unwrap(),
                false,
            )
            .unwrap();
    }
    let report = world
        .step(1, 60)
        .unwrap_or_else(|error| panic!("ECS tower frame {frame} failed: {error:?}"));
    eprintln!(
        "ECS tower frame {frame}: sampled_events={}, tail_contacts={}, tail_slices={}, tail_replays={}",
        report.stats.sampled_events,
        report.stats.tail_contacts,
        report.stats.tail_slices,
        report.stats.tail_replays,
    );
    world.boxes().map(damped_box).collect()
}

fn assert_no_floor_penetration(boxes: &[RigidBox3d], frame: u32) {
    let floor = &boxes[0];
    for rigid_box in boxes.iter().skip(1) {
        let contact = obb_contact_seed(floor.oriented_box(), rigid_box.oriented_box())
            .expect("valid tower floor/body geometry");
        assert!(
            contact.is_none_or(|value| value.overlap_numerator == 0),
            "body {} penetrated the finite floor collider at frame {frame}",
            rigid_box.body().id().0
        );
    }
}

#[test]
fn minimized_crossing_edge_contact_completes_without_event_churn() {
    // Removing the floor and 28 unrelated blocks still reproduces the original 64-event failure.
    let boxes = legacy_tower::minimized_frame_38();
    let run = || {
        let mut world = RotatingWorld3d::new(world_config());
        for body in &boxes {
            world.add_box(body.clone()).unwrap();
        }
        let report = world.step(1, 60).unwrap();
        let result = world.boxes().cloned().collect::<Vec<_>>();
        (report, result)
    };
    let first = run();
    assert_eq!(first, run());
    assert_eq!(first.1.len(), 3);
    assert!(first.0.changed_body_ids.contains(&BodyId(12)));
    println!("COUNTER_SNAPSHOT minimized-edge-contact: {:?}", first.1);
}

#[test]
#[ignore = "long deterministic ECS tower acceptance runs explicitly in Validate"]
fn ecs_tower_completes_long_horizon_without_event_churn() {
    let mut boxes = tower_boxes();
    let mut mixed_boxes = boxes.clone();
    let mut maximum_spinning_blocks = 0_usize;
    let mut projectile_passed_front_face = false;

    assert_eq!(boxes.len(), 32);
    for frame in 1..=240 {
        boxes = step_frame(&boxes, frame, false);
        mixed_boxes = step_frame(&mixed_boxes, frame, true);
        assert_eq!(boxes.len(), 32, "body count changed at frame {frame}");
        assert_eq!(
            mixed_boxes, boxes,
            "an unrelated analytic projectile changed rigid contact results at frame {frame}"
        );
        if frame <= 180 {
            maximum_spinning_blocks = maximum_spinning_blocks.max(
                boxes
                    .iter()
                    .skip(usize::try_from(FIRST_BLOCK_ID).expect("block id fits usize"))
                    .filter(|rigid_box| !rigid_box.angular().angular_velocity.is_zero())
                    .count(),
            );
            projectile_passed_front_face |= boxes[1].body().position().x > SCALE;
        }
        assert_no_floor_penetration(&boxes, frame);
    }

    assert!(projectile_passed_front_face);
    assert!(maximum_spinning_blocks >= 4);
}

#[test]
fn admitted_persistent_pair_can_ricochet_and_recontact_in_the_same_step() {
    let mut world = RotatingWorld3d::new(RotatingWorldConfig3d {
        gravity: Vec3i::ZERO,
        sample_count: 64,
        refinement_steps: 4,
        solver_passes: 4,
        max_events: 64,
    });
    for body in legacy_tower::elastic_corridor() {
        world.add_box(body).unwrap();
    }
    let report = world.step(1, 1).unwrap();
    let moving = world.box_by_id(BodyId(1)).unwrap();
    assert!(moving.body().position().x.abs() <= 2048);
    assert_eq!(moving.body().velocity().x.abs(), 20_000);
    for id in [BodyId(2), BodyId(3)] {
        assert!(
            obb_contact_seed(
                moving.oriented_box(),
                world.box_by_id(id).unwrap().oriented_box()
            )
            .unwrap()
            .is_none_or(|c| c.overlap_numerator == 0)
        );
    }
    #[cfg(feature = "performance-counters")]
    assert!(report.stats.sampled_events >= 4);
    #[cfg(not(feature = "performance-counters"))]
    assert_eq!(
        report.stats,
        physics_engine::RotatingWorldStepStats3d::default()
    );
}
