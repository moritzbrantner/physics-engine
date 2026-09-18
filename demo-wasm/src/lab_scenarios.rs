use physics_engine::{
    BodyId, Material, RigidBody, RotatingWorld3d, RotatingWorldError3d, Vec3i,
};

use crate::rotating_box;

pub(crate) const SENSOR_ID_START: u64 = 400;
pub(crate) const KINEMATIC_ID_START: u64 = 500;
pub(crate) const DEBRIS_ID_START: u64 = 600;
pub(crate) const TOWER_ID_START: u64 = 700;

const KINEMATIC_CRUSHER_ID: BodyId = BodyId(KINEMATIC_ID_START);
const KINEMATIC_SPEED: i32 = 240;
const KINEMATIC_LIMIT: i32 = 185;

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
#[repr(i32)]
pub(crate) enum SandboxScenario {
    #[default]
    Playground = 0,
    ProjectileStorm = 1,
    Tower = 2,
    DebrisRain = 3,
    KinematicCrusher = 4,
    SensorCourse = 5,
    CcdGauntlet = 6,
}

impl SandboxScenario {
    pub(crate) const fn from_i32(value: i32) -> Option<Self> {
        match value {
            0 => Some(Self::Playground),
            1 => Some(Self::ProjectileStorm),
            2 => Some(Self::Tower),
            3 => Some(Self::DebrisRain),
            4 => Some(Self::KinematicCrusher),
            5 => Some(Self::SensorCourse),
            6 => Some(Self::CcdGauntlet),
            _ => None,
        }
    }
}

pub(crate) fn apply(
    world: &mut RotatingWorld3d,
    scenario: SandboxScenario,
) -> Result<(), RotatingWorldError3d> {
    match scenario {
        SandboxScenario::Playground => {}
        SandboxScenario::ProjectileStorm => add_projectile_targets(world)?,
        SandboxScenario::Tower => add_tower(world)?,
        SandboxScenario::DebrisRain => add_debris(world)?,
        SandboxScenario::KinematicCrusher => add_kinematic_crusher(world)?,
        SandboxScenario::SensorCourse => add_sensor_course(world)?,
        SandboxScenario::CcdGauntlet => add_ccd_gauntlet(world)?,
    }
    Ok(())
}

pub(crate) fn drive(
    world: &mut RotatingWorld3d,
    scenario: SandboxScenario,
) -> Result<(), RotatingWorldError3d> {
    if scenario != SandboxScenario::KinematicCrusher {
        return Ok(());
    }
    let Some(crusher) = world.box_by_id(KINEMATIC_CRUSHER_ID) else {
        return Ok(());
    };
    let position = crusher.body().position();
    let current = crusher.body().velocity().x;
    let next_x = if position.x >= KINEMATIC_LIMIT {
        -KINEMATIC_SPEED
    } else if position.x <= -KINEMATIC_LIMIT {
        KINEMATIC_SPEED
    } else if current == 0 {
        KINEMATIC_SPEED
    } else {
        current
    };
    world.set_linear_velocity(KINEMATIC_CRUSHER_ID, Vec3i::new(next_x, 0, 0))
}

pub(crate) const fn render_role(id: BodyId) -> Option<i32> {
    match id.0 {
        SENSOR_ID_START..=499 => Some(6),
        KINEMATIC_ID_START..=599 => Some(7),
        DEBRIS_ID_START..=699 => Some(8),
        TOWER_ID_START..=799 => Some(2),
        _ => None,
    }
}

fn add_projectile_targets(world: &mut RotatingWorld3d) -> Result<(), RotatingWorldError3d> {
    for (index, x) in [-150, -90, -30, 30, 90, 150].into_iter().enumerate() {
        world.add_box(rotating_box(
            RigidBody::dynamic(
                BodyId(TOWER_ID_START + u64::try_from(index).unwrap_or(0)),
                Vec3i::new(x, 22, -210),
                Vec3i::ZERO,
                Vec3i::new(14, 22, 14),
            )
            .with_mass_units(2)
            .with_material(Material::with_friction(0, 900)),
        ))?;
    }
    Ok(())
}

fn add_tower(world: &mut RotatingWorld3d) -> Result<(), RotatingWorldError3d> {
    for level in 0..6_u64 {
        world.add_box(rotating_box(
            RigidBody::dynamic(
                BodyId(TOWER_ID_START + level),
                Vec3i::new(-235, 16 + i32::try_from(level).unwrap_or(0) * 32, -165),
                Vec3i::ZERO,
                Vec3i::new(16, 16, 16),
            )
            .with_mass_units(2)
            .with_material(Material::with_friction(0, 1_000)),
        ))?;
    }
    Ok(())
}

fn add_debris(world: &mut RotatingWorld3d) -> Result<(), RotatingWorldError3d> {
    for index in 0..12_u64 {
        let column = i32::try_from(index % 4).unwrap_or(0);
        let row = i32::try_from(index / 4).unwrap_or(0);
        let body = rotating_box(
            RigidBody::dynamic(
                BodyId(DEBRIS_ID_START + index),
                Vec3i::new(-135 + column * 45, 88 + row * 42, -235),
                Vec3i::new((column - 1) * 35, 0, (row - 1) * 22),
                Vec3i::new(8, 8, 8),
            )
            .with_material(Material::with_friction(0, 850)),
        )
        .with_aggressive_sleep();
        world.add_box(body)?;
    }
    Ok(())
}

fn add_kinematic_crusher(world: &mut RotatingWorld3d) -> Result<(), RotatingWorldError3d> {
    let crusher = rotating_box(RigidBody::dynamic(
        KINEMATIC_CRUSHER_ID,
        Vec3i::new(-KINEMATIC_LIMIT, 30, -205),
        Vec3i::new(KINEMATIC_SPEED, 0, 0),
        Vec3i::new(30, 30, 12),
    ))
    .with_external_motion()
    .with_rotation_locked();
    world.add_box(crusher)?;

    for index in 0..5_u64 {
        world.add_box(rotating_box(
            RigidBody::dynamic(
                BodyId(TOWER_ID_START + index),
                Vec3i::new(-70 + i32::try_from(index).unwrap_or(0) * 35, 14, -205),
                Vec3i::ZERO,
                Vec3i::new(14, 14, 14),
            )
            .with_mass_units(2)
            .with_material(Material::with_friction(0, 900)),
        ))?;
    }
    Ok(())
}

fn add_sensor_course(world: &mut RotatingWorld3d) -> Result<(), RotatingWorldError3d> {
    for (index, z) in [-40, -110, -180].into_iter().enumerate() {
        let sensor = rotating_box(RigidBody::fixed(
            BodyId(SENSOR_ID_START + u64::try_from(index).unwrap_or(0)),
            Vec3i::new(0, 28, z),
            Vec3i::new(85, 28, 5),
        ))
        .with_overlap_only();
        world.add_box(sensor)?;
    }
    Ok(())
}

fn add_ccd_gauntlet(world: &mut RotatingWorld3d) -> Result<(), RotatingWorldError3d> {
    for (index, (x, z)) in [(0, -190), (-105, -270), (105, -350)]
        .into_iter()
        .enumerate()
    {
        world.add_box(rotating_box(RigidBody::fixed(
            BodyId(30 + u64::try_from(index).unwrap_or(0)),
            Vec3i::new(x, 42, z),
            Vec3i::new(72, 42, 2),
        )))?;
    }
    Ok(())
}
