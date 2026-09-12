use physics_engine::{
    AngularState3d, AngularVelocity3d, BodyId, Material, Orientation3d, RigidBody, RigidBox3d,
    RotatingWorld3d, RotatingWorldConfig3d, Vec3i,
};

const PLAYER_ID: BodyId = BodyId(1);
const TICKS_PER_SECOND: i32 = 60;
const STABILITY_TICKS: usize = 120;
const CRATE_RESTITUTION_MILLI: u16 = 0;
const CRATE_FRICTION_MILLI: u16 = 1_000;

fn rotating_box(body: RigidBody) -> RigidBox3d {
    RigidBox3d::new(
        body,
        AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
    )
    .expect("valid sandbox box")
}

fn sandbox_world() -> RotatingWorld3d {
    let mut world = RotatingWorld3d::new(RotatingWorldConfig3d {
        gravity: Vec3i::new(0, -3_600, 0),
        sample_count: 32,
        refinement_steps: 4,
        solver_passes: 8,
        max_events: 32,
    });

    let fixed_bodies = [
        (10, Vec3i::new(0, -16, 0), Vec3i::new(520, 16, 520)),
        (11, Vec3i::new(0, 72, -520), Vec3i::new(520, 72, 8)),
        (12, Vec3i::new(0, 72, 520), Vec3i::new(520, 72, 8)),
        (13, Vec3i::new(-520, 72, 0), Vec3i::new(8, 72, 520)),
        (14, Vec3i::new(520, 72, 0), Vec3i::new(8, 72, 520)),
        (15, Vec3i::new(0, 72, -180), Vec3i::new(120, 72, 3)),
        (20, Vec3i::new(-190, 24, 40), Vec3i::new(70, 24, 70)),
        (21, Vec3i::new(185, 8, 75), Vec3i::new(45, 8, 45)),
        (22, Vec3i::new(185, 16, 20), Vec3i::new(45, 16, 45)),
        (23, Vec3i::new(185, 24, -35), Vec3i::new(45, 24, 45)),
        (24, Vec3i::new(185, 32, -90), Vec3i::new(45, 32, 45)),
    ];
    for (id, position, half_extents) in fixed_bodies {
        world
            .add_box(rotating_box(RigidBody::fixed(
                BodyId(id),
                position,
                half_extents,
            )))
            .expect("fixed fixture");
    }

    world
        .add_box(
            rotating_box(
                RigidBody::dynamic(
                    PLAYER_ID,
                    Vec3i::new(0, 38, 320),
                    Vec3i::ZERO,
                    Vec3i::new(12, 20, 12),
                )
                .with_mass(4),
            )
            .with_rotation_locked(),
        )
        .expect("player");

    let crate_positions = [
        Vec3i::new(-75, 18, 135),
        Vec3i::new(-75, 54, 135),
        Vec3i::new(80, 18, 120),
        Vec3i::new(116, 18, 120),
        Vec3i::new(98, 54, 120),
        Vec3i::new(0, 18, -70),
    ];
    for (offset, position) in crate_positions.into_iter().enumerate() {
        world
            .add_box(rotating_box(
                RigidBody::dynamic(
                    BodyId(100 + offset as u64),
                    position,
                    Vec3i::ZERO,
                    Vec3i::new(18, 18, 18),
                )
                .with_mass(2)
                .with_material(
                    Material::new(CRATE_RESTITUTION_MILLI).with_friction(CRATE_FRICTION_MILLI),
                ),
            ))
            .expect("crate");
    }

    world
}

fn controlled_step(world: &mut RotatingWorld3d, move_x: i32, move_z: i32, tick: usize) {
    controlled_step_with_jump(world, move_x, move_z, false, tick);
}

fn controlled_step_with_jump(
    world: &mut RotatingWorld3d,
    move_x: i32,
    move_z: i32,
    jump: bool,
    tick: usize,
) {
    let current_velocity = world
        .box_by_id(PLAYER_ID)
        .expect("player")
        .body()
        .velocity();
    let next_y = if jump {
        16_i32.saturating_mul(TICKS_PER_SECOND)
    } else {
        current_velocity.y
    };
    let velocity = Vec3i::new(
        move_x.saturating_mul(TICKS_PER_SECOND),
        next_y,
        move_z.saturating_mul(TICKS_PER_SECOND),
    );
    if velocity != current_velocity {
        world
            .set_linear_velocity(PLAYER_ID, velocity)
            .expect("controlled player velocity");
    }
    world
        .step(1, TICKS_PER_SECOND)
        .unwrap_or_else(|error| panic!("sandbox failed at tick {tick}: {error:?}"));
}

fn shoot(world: &mut RotatingWorld3d, id: BodyId, velocity: Vec3i) {
    let player_position = world
        .box_by_id(PLAYER_ID)
        .expect("player")
        .body()
        .position();
    let spawn = Vec3i::new(
        player_position.x + velocity.x / 3,
        player_position.y + 12 + velocity.y / 3,
        player_position.z + velocity.z / 3,
    );
    let scaled_velocity = Vec3i::new(
        velocity.x.saturating_mul(TICKS_PER_SECOND),
        velocity.y.saturating_mul(TICKS_PER_SECOND),
        velocity.z.saturating_mul(TICKS_PER_SECOND),
    );
    world
        .add_box(rotating_box(
            RigidBody::dynamic(id, spawn, scaled_velocity, Vec3i::new(3, 3, 3))
                .with_material(Material::new(350)),
        ))
        .expect("projectile");
}

#[test]
fn idle_sandbox_survives_two_seconds() {
    let mut world = sandbox_world();
    for tick in 0..STABILITY_TICKS {
        controlled_step(&mut world, 0, 0, tick);
    }
}

#[test]
fn sustained_forward_input_survives_two_seconds() {
    let mut world = sandbox_world();
    for tick in 0..STABILITY_TICKS {
        controlled_step(&mut world, 0, -7, tick);
    }
}

#[test]
fn alternating_wasd_input_survives_two_seconds() {
    let mut world = sandbox_world();
    let inputs = [(0, -7), (-7, 0), (0, 7), (7, 0)];
    for tick in 0..STABILITY_TICKS {
        let (move_x, move_z) = inputs[(tick / 30) % inputs.len()];
        controlled_step(&mut world, move_x, move_z, tick);
    }
}

#[test]
fn mixed_movement_jump_and_projectile_survives_reported_path() {
    let mut world = sandbox_world();
    let changes = [
        (0, (-5, 4)),
        (15, (-5, 5)),
        (30, (-2, -4)),
        (45, (5, -3)),
        (60, (0, 0)),
        (75, (6, -7)),
        (90, (-1, -6)),
        (105, (-1, 7)),
        (120, (-6, -5)),
    ];
    let mut movement = (0, 0);

    for tick in 0..=128 {
        if let Some((_, next)) = changes.iter().find(|(at, _)| *at == tick) {
            movement = *next;
        }
        if tick == 125 {
            shoot(&mut world, BodyId(1_000), Vec3i::new(84, -3, -36));
        }
        controlled_step_with_jump(&mut world, movement.0, movement.1, tick == 71, tick);
    }
}

#[test]
fn rough_crate_push_and_turn_survives_long_play() {
    let mut world = sandbox_world();
    let phases = [
        (0, (0, -7)),
        (90, (5, -6)),
        (130, (-6, -3)),
        (170, (7, 2)),
        (210, (0, -7)),
        (270, (-5, 5)),
        (320, (0, 0)),
    ];
    let mut movement = (0, 0);

    for tick in 0..=360 {
        if let Some((_, next)) = phases.iter().find(|(at, _)| *at == tick) {
            movement = *next;
        }
        let jump = matches!(tick, 104 | 188 | 286);
        controlled_step_with_jump(&mut world, movement.0, movement.1, jump, tick);
    }
}
