use physics_engine::{
    AngularState3d, AngularVelocity3d, BodyId, Material, MotionAuthority3d, Orientation3d,
    RigidBody, RigidBox3d, RotatingWorld3d, RotatingWorldConfig3d, RotatingWorldError3d, Vec3i,
};

use crate::{
    CRATE_FRICTION_MILLI, CRATE_RESTITUTION_MILLI, PLAYER_ID, rotating_box,
};

pub(super) const BODY_COUNT: usize = 48;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum Axis {
    X,
    Y,
    Z,
}

impl Axis {
    const fn coordinate(self, value: Vec3i) -> i32 {
        match self {
            Self::X => value.x,
            Self::Y => value.y,
            Self::Z => value.z,
        }
    }

    const fn velocity(self, amount: i32) -> Vec3i {
        match self {
            Self::X => Vec3i::new(amount, 0, 0),
            Self::Y => Vec3i::new(0, amount, 0),
            Self::Z => Vec3i::new(0, 0, amount),
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct MovingObstacle {
    id: BodyId,
    axis: Axis,
    min: i32,
    max: i32,
    speed: i32,
}

impl MovingObstacle {
    const fn inward_velocity(self, position: Vec3i, current_velocity: Vec3i) -> Vec3i {
        let coordinate = self.axis.coordinate(position);
        let current = self.axis.coordinate(current_velocity);
        let direction = if coordinate <= self.min {
            1
        } else if coordinate >= self.max {
            -1
        } else if current < 0 {
            -1
        } else {
            1
        };
        self.axis.velocity(direction * self.speed)
    }
}

const MOVING_OBSTACLES: [MovingObstacle; 6] = [
    MovingObstacle {
        id: BodyId(60),
        axis: Axis::X,
        min: -170,
        max: 170,
        speed: 90,
    },
    MovingObstacle {
        id: BodyId(61),
        axis: Axis::X,
        min: -190,
        max: 190,
        speed: 120,
    },
    MovingObstacle {
        id: BodyId(62),
        axis: Axis::Y,
        min: 18,
        max: 150,
        speed: 72,
    },
    MovingObstacle {
        id: BodyId(63),
        axis: Axis::X,
        min: -230,
        max: 230,
        speed: 105,
    },
    MovingObstacle {
        id: BodyId(64),
        axis: Axis::X,
        min: -220,
        max: 220,
        speed: 150,
    },
    MovingObstacle {
        id: BodyId(65),
        axis: Axis::Z,
        min: -820,
        max: -690,
        speed: 90,
    },
];

pub(super) fn build_world(
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

    for (id, position, half_extents) in [
        (10, Vec3i::new(0, -176, 0), Vec3i::new(560, 16, 980)),
        (11, Vec3i::new(0, 24, -980), Vec3i::new(560, 200, 8)),
        (12, Vec3i::new(0, 24, 980), Vec3i::new(560, 200, 8)),
        (13, Vec3i::new(-560, 24, 0), Vec3i::new(8, 200, 980)),
        (14, Vec3i::new(560, 24, 0), Vec3i::new(8, 200, 980)),
        (20, Vec3i::new(0, -8, 770), Vec3i::new(170, 8, 120)),
        (21, Vec3i::new(-150, 12, 570), Vec3i::new(75, 12, 65)),
        (22, Vec3i::new(140, 24, 445), Vec3i::new(75, 24, 60)),
        (23, Vec3i::new(0, 34, 315), Vec3i::new(55, 34, 50)),
        (24, Vec3i::new(190, 46, 180), Vec3i::new(85, 46, 55)),
        (25, Vec3i::new(-180, 58, 45), Vec3i::new(85, 58, 55)),
        (26, Vec3i::new(0, 70, -95), Vec3i::new(70, 70, 55)),
        (27, Vec3i::new(0, 58, -250), Vec3i::new(150, 58, 60)),
        (28, Vec3i::new(-200, 34, -405), Vec3i::new(70, 34, 55)),
        (29, Vec3i::new(0, 46, -535), Vec3i::new(65, 46, 55)),
        (30, Vec3i::new(200, 58, -665), Vec3i::new(70, 58, 55)),
        (31, Vec3i::new(0, 70, -840), Vec3i::new(190, 70, 90)),
        (32, Vec3i::new(-340, 16, 610), Vec3i::new(65, 16, 70)),
        (33, Vec3i::new(330, 24, 500), Vec3i::new(65, 24, 70)),
        (34, Vec3i::new(-330, 32, 360), Vec3i::new(65, 32, 70)),
        (35, Vec3i::new(330, 40, 235), Vec3i::new(65, 40, 70)),
        (36, Vec3i::new(-330, 48, 100), Vec3i::new(65, 48, 70)),
        (37, Vec3i::new(330, 56, -70), Vec3i::new(65, 56, 70)),
        (38, Vec3i::new(-330, 64, -250), Vec3i::new(65, 64, 70)),
        (39, Vec3i::new(330, 72, -430), Vec3i::new(65, 72, 70)),
        (40, Vec3i::new(-330, 80, -610), Vec3i::new(65, 80, 70)),
        (41, Vec3i::new(330, 88, -780), Vec3i::new(65, 88, 70)),
        (42, Vec3i::new(0, 50, 620), Vec3i::new(25, 50, 25)),
        (43, Vec3i::new(0, 86, -700), Vec3i::new(28, 86, 28)),
    ] {
        add_fixed(&mut world, id, position, half_extents)?;
    }

    add_player(&mut world, Vec3i::new(0, 38, 820), linear_push)?;

    for (obstacle, position, velocity, half_extents) in [
        (
            MOVING_OBSTACLES[0],
            Vec3i::new(-170, 72, 510),
            Vec3i::new(-90, 0, 0),
            Vec3i::new(60, 8, 45),
        ),
        (
            MOVING_OBSTACLES[1],
            Vec3i::new(190, 130, 110),
            Vec3i::new(120, 0, 0),
            Vec3i::new(60, 8, 45),
        ),
        (
            MOVING_OBSTACLES[2],
            Vec3i::new(-260, 18, -190),
            Vec3i::new(0, -72, 0),
            Vec3i::new(55, 8, 50),
        ),
        (
            MOVING_OBSTACLES[3],
            Vec3i::new(-230, 150, -365),
            Vec3i::new(-105, 0, 0),
            Vec3i::new(65, 8, 45),
        ),
        (
            MOVING_OBSTACLES[4],
            Vec3i::new(220, 115, -560),
            Vec3i::new(150, 0, 0),
            Vec3i::new(18, 55, 65),
        ),
        (
            MOVING_OBSTACLES[5],
            Vec3i::new(0, 160, -820),
            Vec3i::new(0, 0, -90),
            Vec3i::new(80, 8, 55),
        ),
    ] {
        add_moving_obstacle(&mut world, obstacle.id, position, velocity, half_extents)?;
    }

    for (offset, position) in [
        Vec3i::new(-340, 50, 610),
        Vec3i::new(330, 66, 500),
        Vec3i::new(-330, 82, 360),
        Vec3i::new(330, 98, 235),
        Vec3i::new(-330, 114, 100),
        Vec3i::new(330, 130, -70),
        Vec3i::new(-330, 146, -250),
        Vec3i::new(330, 162, -430),
        Vec3i::new(-330, 178, -610),
        Vec3i::new(330, 194, -780),
        Vec3i::new(-110, 18, 790),
        Vec3i::new(110, 18, 750),
    ]
    .into_iter()
    .enumerate()
    {
        add_crate(
            &mut world,
            100 + offset as u64,
            position,
            upright_crates,
        )?;
    }

    debug_assert_eq!(world.boxes().count(), BODY_COUNT);
    Ok(world)
}

pub(super) fn update_moving_obstacles(
    world: &mut RotatingWorld3d,
) -> Result<(), RotatingWorldError3d> {
    for obstacle in MOVING_OBSTACLES {
        let rigid_box = world
            .box_by_id(obstacle.id)
            .ok_or(RotatingWorldError3d::MissingBody(obstacle.id))?;
        let next_velocity =
            obstacle.inward_velocity(rigid_box.body().position(), rigid_box.body().velocity());
        if next_velocity != rigid_box.body().velocity() {
            world.set_linear_velocity(obstacle.id, next_velocity)?;
        }
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
        RigidBody::dynamic(
            PLAYER_ID,
            position,
            Vec3i::ZERO,
            Vec3i::new(12, 20, 12),
        )
        .with_mass(4),
    )
    .with_rotation_locked();

    world.add_box(if linear_push {
        player.with_linear_push(Vec3i::new(0, -1, 0))
    } else {
        player
    })
}

fn add_moving_obstacle(
    world: &mut RotatingWorld3d,
    id: BodyId,
    position: Vec3i,
    velocity: Vec3i,
    half_extents: Vec3i,
) -> Result<(), RotatingWorldError3d> {
    world.add_box(
        RigidBox3d::new(
            RigidBody::dynamic(id, position, velocity, half_extents)
                .with_mass(8)
                .with_material(
                    Material::new(CRATE_RESTITUTION_MILLI)
                        .with_friction(CRATE_FRICTION_MILLI),
                ),
            AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
        )
        .expect("parkour moving obstacle geometry must be valid")
        .with_rotation_locked()
        .with_external_motion(),
    )
}

fn add_crate(
    world: &mut RotatingWorld3d,
    id: u64,
    position: Vec3i,
    upright: bool,
) -> Result<(), RotatingWorldError3d> {
    let rigid_box = rotating_box(
        RigidBody::dynamic(
            BodyId(id),
            position,
            Vec3i::ZERO,
            Vec3i::new(18, 18, 18),
        )
        .with_mass(2)
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

#[cfg(test)]
mod tests {
    use super::{
        BODY_COUNT, MOVING_OBSTACLES, MotionAuthority3d, PLAYER_ID, Vec3i, build_world,
        update_moving_obstacles,
    };

    #[test]
    fn parkour_fixture_is_large_and_keeps_movers_engine_visible() {
        let world = build_world(false, false).expect("valid parkour world");

        assert_eq!(world.boxes().count(), BODY_COUNT);
        assert_eq!(
            world
                .box_by_id(PLAYER_ID)
                .expect("parkour player")
                .body()
                .position(),
            Vec3i::new(0, 38, 820)
        );
        for obstacle in MOVING_OBSTACLES {
            assert_eq!(
                world
                    .box_by_id(obstacle.id)
                    .expect("parkour moving obstacle")
                    .motion_authority(),
                MotionAuthority3d::External,
                "moving obstacle {} must remain a real engine collider",
                obstacle.id.0
            );
        }
    }

    #[test]
    fn parkour_movers_reverse_inward_at_their_bounds() {
        let mut world = build_world(false, false).expect("valid parkour world");
        let before = MOVING_OBSTACLES.map(|obstacle| {
            world
                .box_by_id(obstacle.id)
                .expect("moving obstacle before schedule")
                .body()
                .velocity()
        });

        update_moving_obstacles(&mut world).expect("update mover schedule");

        for (index, obstacle) in MOVING_OBSTACLES.into_iter().enumerate() {
            let rigid_box = world
                .box_by_id(obstacle.id)
                .expect("moving obstacle after schedule");
            let position = obstacle.axis.coordinate(rigid_box.body().position());
            let velocity = obstacle.axis.coordinate(rigid_box.body().velocity());
            assert_ne!(velocity, 0);
            if position <= obstacle.min {
                assert!(velocity > 0, "lower-bound mover must travel inward");
            }
            if position >= obstacle.max {
                assert!(velocity < 0, "upper-bound mover must travel inward");
            }
            assert_ne!(
                rigid_box.body().velocity(),
                before[index],
                "fixture starts each mover at a boundary so the first schedule tick proves reversal"
            );
        }
    }
}
