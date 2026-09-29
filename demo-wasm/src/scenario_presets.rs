use physics_engine::{
    BodyId, Material, Ray, RigidBody, RotatingWorld3d, RotatingWorldConfig3d, RotatingWorldError3d,
    Vec3i, ray_cast_first,
};

use crate::{CRATE_FRICTION_MILLI, CRATE_RESTITUTION_MILLI, PLAYER_ID, rotating_box};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(i32)]
pub(crate) enum DemoScenario {
    CcdGauntlet = 1,
    RotatingBoxLab = 2,
    TowerStability = 3,
    SleepingWorld = 4,
    CollisionQueryLab = 5,
    OffCentreImpact = 6,
}

impl DemoScenario {
    pub(crate) const fn from_i32(value: i32) -> Option<Self> {
        match value {
            1 => Some(Self::CcdGauntlet),
            2 => Some(Self::RotatingBoxLab),
            3 => Some(Self::TowerStability),
            4 => Some(Self::SleepingWorld),
            5 => Some(Self::CollisionQueryLab),
            6 => Some(Self::OffCentreImpact),
            _ => None,
        }
    }
}

pub(crate) fn build_world(
    scenario: DemoScenario,
    linear_push: bool,
    upright_crates: bool,
) -> Result<RotatingWorld3d, RotatingWorldError3d> {
    let mut world = RotatingWorld3d::new(RotatingWorldConfig3d {
        gravity: Vec3i::new(0, -3_600, 0),
        sample_count: 32,
        refinement_steps: 4,
        solver_passes: 8,
        max_events: 32,
    });

    match scenario {
        DemoScenario::CcdGauntlet => build_ccd_gauntlet(&mut world, linear_push)?,
        DemoScenario::RotatingBoxLab => {
            build_rotating_box_lab(&mut world, linear_push, upright_crates)?;
        }
        DemoScenario::TowerStability => {
            build_tower_stability(&mut world, linear_push, upright_crates)?;
        }
        DemoScenario::SleepingWorld => {
            build_sleeping_world(&mut world, linear_push, upright_crates)?;
        }
        DemoScenario::CollisionQueryLab => build_collision_query_lab(&mut world, linear_push)?,
        DemoScenario::OffCentreImpact => {
            build_off_centre_impact(&mut world, linear_push, upright_crates)?;
        }
    }

    Ok(world)
}

fn add_arena(world: &mut RotatingWorld3d, half_size: i32) -> Result<(), RotatingWorldError3d> {
    let wall_height = 96;
    let fixed = [
        (
            10,
            Vec3i::new(0, -16, 0),
            Vec3i::new(half_size, 16, half_size),
        ),
        (
            11,
            Vec3i::new(0, wall_height, -half_size),
            Vec3i::new(half_size, wall_height, 8),
        ),
        (
            12,
            Vec3i::new(0, wall_height, half_size),
            Vec3i::new(half_size, wall_height, 8),
        ),
        (
            13,
            Vec3i::new(-half_size, wall_height, 0),
            Vec3i::new(8, wall_height, half_size),
        ),
        (
            14,
            Vec3i::new(half_size, wall_height, 0),
            Vec3i::new(8, wall_height, half_size),
        ),
    ];
    for (id, position, half_extents) in fixed {
        add_fixed(world, id, position, half_extents)?;
    }
    Ok(())
}

fn add_fixed(
    world: &mut RotatingWorld3d,
    id: u64,
    position: Vec3i,
    half_extents: Vec3i,
) -> Result<(), RotatingWorldError3d> {
    world.add_box(rotating_box(RigidBody::fixed(
        BodyId(id),
        position,
        half_extents,
    )))
}

fn add_player(
    world: &mut RotatingWorld3d,
    position: Vec3i,
    linear_push: bool,
) -> Result<(), RotatingWorldError3d> {
    let player = rotating_box(
        RigidBody::dynamic(PLAYER_ID, position, Vec3i::ZERO, Vec3i::new(12, 20, 12))
            .with_mass(crate::PLAYER_MASS_UNITS),
    )
    .with_rotation_locked();

    world.add_box(if linear_push {
        player.with_linear_push(Vec3i::new(0, -1, 0))
    } else {
        player
    })
}

fn add_crate(
    world: &mut RotatingWorld3d,
    id: u64,
    position: Vec3i,
    velocity: Vec3i,
    half_extents: Vec3i,
    mass: u32,
    upright: bool,
) -> Result<(), RotatingWorldError3d> {
    let rigid_box = rotating_box(
        RigidBody::dynamic(BodyId(id), position, velocity, half_extents)
            .with_mass(mass)
            .with_material(
                Material::new(CRATE_RESTITUTION_MILLI).with_friction(CRATE_FRICTION_MILLI),
            ),
    );
    world.add_box(if upright {
        rigid_box.with_rotation_locked()
    } else {
        rigid_box
    })
}

fn build_ccd_gauntlet(
    world: &mut RotatingWorld3d,
    linear_push: bool,
) -> Result<(), RotatingWorldError3d> {
    add_arena(world, 620)?;
    add_player(world, Vec3i::new(0, 38, 430), linear_push)?;

    for (id, x, z, half_x) in [
        (30, -180, 170, 52),
        (31, 0, 70, 52),
        (32, 180, -30, 52),
        (33, -120, -150, 42),
        (34, 70, -270, 42),
        (35, 0, -390, 82),
    ] {
        add_fixed(world, id, Vec3i::new(x, 70, z), Vec3i::new(half_x, 70, 2))?;
    }
    Ok(())
}

fn build_rotating_box_lab(
    world: &mut RotatingWorld3d,
    linear_push: bool,
    upright_crates: bool,
) -> Result<(), RotatingWorldError3d> {
    add_arena(world, 520)?;
    add_player(world, Vec3i::new(0, 38, 360), linear_push)?;

    for (id, x) in [(30, -180), (31, 0), (32, 180)] {
        add_fixed(world, id, Vec3i::new(x, 8, -30), Vec3i::new(62, 8, 62))?;
    }

    for (id, position, velocity, half_extents) in [
        (
            100,
            Vec3i::new(-205, 52, -30),
            Vec3i::new(220, 0, 40),
            Vec3i::new(26, 26, 18),
        ),
        (
            101,
            Vec3i::new(-145, 42, -5),
            Vec3i::ZERO,
            Vec3i::new(20, 20, 30),
        ),
        (
            102,
            Vec3i::new(-22, 54, -30),
            Vec3i::new(120, 0, 0),
            Vec3i::new(22, 30, 18),
        ),
        (
            103,
            Vec3i::new(25, 46, -6),
            Vec3i::ZERO,
            Vec3i::new(28, 22, 20),
        ),
        (
            104,
            Vec3i::new(180, 54, -30),
            Vec3i::new(0, 0, 160),
            Vec3i::new(30, 30, 16),
        ),
    ] {
        add_crate(
            world,
            id,
            position,
            velocity,
            half_extents,
            2,
            upright_crates,
        )?;
    }
    Ok(())
}

fn build_tower_stability(
    world: &mut RotatingWorld3d,
    linear_push: bool,
    upright_crates: bool,
) -> Result<(), RotatingWorldError3d> {
    add_arena(world, 520)?;
    add_player(world, Vec3i::new(0, 38, 360), linear_push)?;

    let mut id = 100_u64;
    for level in 0..4 {
        for column in [-1, 1] {
            add_crate(
                world,
                id,
                Vec3i::new(column * 17, 17 + level * 34, -80),
                Vec3i::ZERO,
                Vec3i::new(16, 16, 16),
                2,
                upright_crates,
            )?;
            id += 1;
        }
    }
    Ok(())
}

fn build_sleeping_world(
    world: &mut RotatingWorld3d,
    linear_push: bool,
    upright_crates: bool,
) -> Result<(), RotatingWorldError3d> {
    add_arena(world, 900)?;
    add_player(world, Vec3i::new(0, 38, 480), linear_push)?;

    let clusters = [(-300, -220), (300, -220), (-300, 220), (300, 220)];
    let mut id = 100_u64;
    for (center_x, center_z) in clusters {
        for offset_x in [-24, 24] {
            for offset_z in [-24, 24] {
                add_crate(
                    world,
                    id,
                    Vec3i::new(center_x + offset_x, 18, center_z + offset_z),
                    Vec3i::ZERO,
                    Vec3i::new(18, 18, 18),
                    2,
                    upright_crates,
                )?;
                id += 1;
            }
        }
    }
    Ok(())
}

fn build_collision_query_lab(
    world: &mut RotatingWorld3d,
    linear_push: bool,
) -> Result<(), RotatingWorldError3d> {
    add_arena(world, 620)?;
    add_player(world, Vec3i::new(0, 38, 430), linear_push)?;

    for (id, position, half_extents) in [
        (30, Vec3i::new(0, 55, 170), Vec3i::new(42, 42, 12)),
        (31, Vec3i::new(-170, 72, 45), Vec3i::new(34, 58, 12)),
        (32, Vec3i::new(165, 46, 5), Vec3i::new(48, 32, 12)),
        (33, Vec3i::new(-85, 92, -155), Vec3i::new(28, 72, 12)),
        (34, Vec3i::new(105, 62, -260), Vec3i::new(58, 44, 12)),
        (35, Vec3i::new(-15, 38, -390), Vec3i::new(78, 24, 12)),
    ] {
        add_fixed(world, id, position, half_extents)?;
    }
    Ok(())
}

fn build_off_centre_impact(
    world: &mut RotatingWorld3d,
    linear_push: bool,
    upright_crates: bool,
) -> Result<(), RotatingWorldError3d> {
    add_arena(world, 520)?;
    add_player(world, Vec3i::new(0, 38, 350), linear_push)?;

    add_fixed(world, 30, Vec3i::new(0, 8, -90), Vec3i::new(95, 8, 95))?;
    add_fixed(world, 31, Vec3i::new(0, 80, -260), Vec3i::new(170, 80, 8))?;
    add_fixed(world, 32, Vec3i::new(-120, 44, -90), Vec3i::new(8, 44, 95))?;
    add_fixed(world, 33, Vec3i::new(120, 44, -90), Vec3i::new(8, 44, 95))?;
    add_crate(
        world,
        100,
        Vec3i::new(0, 48, -90),
        Vec3i::ZERO,
        Vec3i::new(32, 32, 32),
        3,
        upright_crates,
    )?;
    Ok(())
}

pub(crate) fn aim_query_hit(
    world: &RotatingWorld3d,
    scenario: DemoScenario,
    direction: Vec3i,
) -> Option<BodyId> {
    if scenario != DemoScenario::CollisionQueryLab || direction == Vec3i::ZERO {
        return None;
    }

    let player = world.box_by_id(PLAYER_ID)?;
    let player_position = player.body().position();
    let origin = Vec3i::new(
        player_position.x,
        player_position.y.checked_add(13)?,
        player_position.z,
    );
    // This fixture contains fixed, axis-aligned targets. Borrow their current engine state;
    // no second world or browser geometry owns query truth.
    ray_cast_first(
        world
            .boxes()
            .map(|rigid_box| rigid_box.body())
            .filter(|body| (30..100).contains(&body.id().0)),
        Ray::new(origin, direction),
        12,
    )
    .ok()
    .flatten()
    .map(|hit| hit.body)
}

#[cfg(test)]
mod tests {
    use physics_engine::BodyId;

    use super::{DemoScenario, aim_query_hit, build_world};

    #[test]
    fn every_pages_scenario_keeps_the_shared_player_contract() {
        for scenario in [
            DemoScenario::CcdGauntlet,
            DemoScenario::RotatingBoxLab,
            DemoScenario::TowerStability,
            DemoScenario::SleepingWorld,
            DemoScenario::CollisionQueryLab,
            DemoScenario::OffCentreImpact,
        ] {
            let world = build_world(scenario, true, false).expect("valid scenario world");
            assert!(
                world.box_by_id(BodyId(1)).is_some(),
                "{scenario:?} must retain the shared first-person player"
            );
        }
    }

    #[test]
    fn collision_query_lab_uses_the_engine_ray_query() {
        let world =
            build_world(DemoScenario::CollisionQueryLab, true, false).expect("valid query lab");
        assert_eq!(
            aim_query_hit(
                &world,
                DemoScenario::CollisionQueryLab,
                physics_engine::Vec3i::new(0, 0, -96),
            ),
            Some(BodyId(30))
        );
    }
    #[test]
    fn focused_fixtures_complete_and_replay_in_both_character_modes() {
        for value in 1..=6 {
            let scenario = DemoScenario::from_i32(value).expect("focused fixture");
            for linear in [false, true] {
                let mut reference = None;
                for _ in 0..2 {
                    let mut sandbox =
                        crate::Sandbox::with_focused_options(scenario, linear, linear)
                            .expect("valid focused fixture");
                    let count = sandbox.world.boxes().count();
                    let mut checkpoints = Vec::new();
                    for tick in 0..600 {
                        assert_eq!(
                            sandbox.step_velocity(0, 0, false),
                            0,
                            "{scenario:?} linear={linear} tick={tick}, detail={}",
                            sandbox.error_detail
                        );
                        assert_eq!(sandbox.world.boxes().count(), count);
                        if tick % 60 == 59 {
                            checkpoints.push(
                                sandbox
                                    .world
                                    .boxes()
                                    .map(|body| {
                                        (body.clone(), sandbox.world.is_sleeping(body.body().id()))
                                    })
                                    .collect::<Vec<_>>(),
                            );
                        }
                    }
                    if scenario == DemoScenario::SleepingWorld {
                        assert!(
                            sandbox.is_quiescent(),
                            "separated idle clusters should settle"
                        );
                    }
                    if let Some(expected) = &reference {
                        assert_eq!(&checkpoints, expected);
                    } else {
                        reference = Some(checkpoints);
                    }
                }
            }
        }
    }
}
