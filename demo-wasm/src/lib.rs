use std::cell::RefCell;

use physics_engine::{
    AngularState3d, AngularVelocity3d, BallisticSphere3d, BallisticSphereError3d,
    BallisticTimelineError3d, BodyId, BodyKind, Material, Orientation3d,
    RepeatedRotatingEventError3d, RigidBody, RigidBox3d, RotatingWorld3d, RotatingWorldConfig3d,
    RotatingWorldError3d, RotatingWorldStepStats3d, Vec3i,
};

mod controller;
mod render_snapshot;

use controller::TICKS_PER_SECOND;
pub use controller::controlled_velocity;

const PLAYER_ID: BodyId = BodyId(1);
const PROJECTILE_ID_START: u64 = 1_000;
const MAX_PROJECTILES: usize = 48;
const LEGACY_MOVE_SPEED: i32 = 7;
const JUMP_SPEED: i32 = 16;
const PROJECTILE_SPEED_LIMIT: i32 = 120;
const ROTATING_TICKS_PER_SECOND: i32 = TICKS_PER_SECOND;
const CRATE_RESTITUTION_MILLI: u16 = 0;
const CRATE_FRICTION_MILLI: u16 = 1_000;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(i32)]
enum ProjectileType {
    Sphere = 0,
    Arrow = 1,
    Rigid = 2,
}

impl ProjectileType {
    const fn from_i32(value: i32) -> Option<Self> {
        match value {
            0 => Some(Self::Sphere),
            1 => Some(Self::Arrow),
            2 => Some(Self::Rigid),
            _ => None,
        }
    }
}

struct Sandbox {
    world: RotatingWorld3d,
    next_projectile_id: u64,
    projectile_ids: Vec<BodyId>,
    projectile_type: Option<ProjectileType>,
    last_rotating_events: usize,
    last_tail_contacts: usize,
    last_step_stats: RotatingWorldStepStats3d,
    total_collisions: u32,
    projectiles_retired_on_contact: u32,
    projectiles_retired_out_of_bounds: u32,
    projectiles_evicted_by_cap: u32,
    error_code: i32,
    error_detail: i32,
}

impl Sandbox {
    fn new() -> Result<Self, RotatingWorldError3d> {
        Self::with_character_mode(false)
    }

    fn with_character_mode(linear_push: bool) -> Result<Self, RotatingWorldError3d> {
        Self::with_options(linear_push, false)
    }

    fn with_options(linear_push: bool, upright_crates: bool) -> Result<Self, RotatingWorldError3d> {
        let crate_positions = [
            Vec3i::new(-75, 18, 135),
            Vec3i::new(-75, 54, 135),
            Vec3i::new(80, 18, 120),
            Vec3i::new(116, 18, 120),
            Vec3i::new(98, 54, 120),
            Vec3i::new(0, 18, -70),
        ];
        Self::with_crate_layout(linear_push, upright_crates, &crate_positions)
    }

    fn with_tower_options(
        linear_push: bool,
        upright_crates: bool,
    ) -> Result<Self, RotatingWorldError3d> {
        let mut crate_positions = Vec::with_capacity(32);
        for level in 0..4 {
            for depth in 0..2 {
                for column in 0..4 {
                    crate_positions.push(Vec3i::new(
                        -54 + column * 36,
                        18 + level * 36,
                        82 + depth * 36,
                    ));
                }
            }
        }
        Self::with_crate_layout(linear_push, upright_crates, &crate_positions)
    }

    fn with_crate_layout(
        linear_push: bool,
        upright_crates: bool,
        crate_positions: &[Vec3i],
    ) -> Result<Self, RotatingWorldError3d> {
        controller::scenario_rules::reset_default();
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
            // Deliberately thin target for sampled rotating CCD acceptance.
            (15, Vec3i::new(0, 72, -180), Vec3i::new(120, 72, 3)),
            (20, Vec3i::new(-190, 24, 40), Vec3i::new(70, 24, 70)),
            (21, Vec3i::new(185, 8, 75), Vec3i::new(45, 8, 45)),
            (22, Vec3i::new(185, 16, 20), Vec3i::new(45, 16, 45)),
            (23, Vec3i::new(185, 24, -35), Vec3i::new(45, 24, 45)),
            (24, Vec3i::new(185, 32, -90), Vec3i::new(45, 32, 45)),
        ];
        for (id, position, half_extents) in fixed_bodies {
            world.add_box(rotating_box(RigidBody::fixed(
                BodyId(id),
                position,
                half_extents,
            )))?;
        }

        let player = rotating_box(
            RigidBody::dynamic(
                PLAYER_ID,
                Vec3i::new(0, 38, 320),
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
        })?;

        for (offset, position) in crate_positions.iter().copied().enumerate() {
            let crate_body = rotating_box(
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
            );
            world.add_box(if upright_crates {
                crate_body.with_rotation_locked()
            } else {
                crate_body
            })?;
        }

        Ok(Self {
            world,
            next_projectile_id: PROJECTILE_ID_START,
            projectile_ids: Vec::new(),
            projectile_type: None,
            last_rotating_events: 0,
            last_tail_contacts: 0,
            last_step_stats: RotatingWorldStepStats3d::default(),
            total_collisions: 0,
            projectiles_retired_on_contact: 0,
            projectiles_retired_out_of_bounds: 0,
            projectiles_evicted_by_cap: 0,
            error_code: 0,
            error_detail: 0,
        })
    }

    fn grounded(&self) -> Result<bool, RotatingWorldError3d> {
        physics_engine::body_has_support(&self.world, PLAYER_ID, self.world.config().gravity)
    }

    fn is_quiescent(&self) -> bool {
        self.world.ballistic_sphere_count() == 0
            && self.world.boxes().all(|rigid_box| {
                rigid_box.body().kind() == BodyKind::Fixed
                    || self.world.is_sleeping(rigid_box.body().id())
            })
    }

    fn step(&mut self, move_x: i32, move_z: i32, jump: bool) -> i32 {
        let desired_x = move_x
            .clamp(-LEGACY_MOVE_SPEED, LEGACY_MOVE_SPEED)
            .saturating_mul(ROTATING_TICKS_PER_SECOND);
        let desired_z = move_z
            .clamp(-LEGACY_MOVE_SPEED, LEGACY_MOVE_SPEED)
            .saturating_mul(ROTATING_TICKS_PER_SECOND);
        self.step_velocity(desired_x, desired_z, jump)
    }

    fn step_velocity(&mut self, desired_x: i32, desired_z: i32, jump: bool) -> i32 {
        self.error_code = 0;
        self.error_detail = 0;
        if desired_x == 0 && desired_z == 0 && !jump && self.is_quiescent() {
            self.last_rotating_events = 0;
            self.last_tail_contacts = 0;
            self.last_step_stats = RotatingWorldStepStats3d::default();
            return 0;
        }

        let Some(player) = self.world.box_by_id(PLAYER_ID) else {
            self.error_code = 1;
            return self.error_code;
        };
        let current_velocity = player.body().velocity();
        let grounded = match self.grounded() {
            Ok(value) => value,
            Err(_) => {
                self.error_code = 2;
                return self.error_code;
            }
        };
        let next_y = if jump && grounded {
            JUMP_SPEED.saturating_mul(ROTATING_TICKS_PER_SECOND)
        } else {
            current_velocity.y
        };
        let velocity = controlled_velocity(current_velocity, desired_x, desired_z, next_y);
        if velocity != current_velocity
            && self.world.set_linear_velocity(PLAYER_ID, velocity).is_err()
        {
            self.error_code = 3;
            return self.error_code;
        }

        let report = match self.world.step(1, ROTATING_TICKS_PER_SECOND) {
            Ok(report) => report,
            Err(error) => {
                self.error_code = 6;
                self.error_detail = world_error_detail(error);
                return self.error_code;
            }
        };

        self.last_step_stats = report.stats;
        self.projectiles_retired_on_contact = self
            .projectiles_retired_on_contact
            .saturating_add(saturating_u32(self.last_step_stats.ballistic_retired));
        if let Err(error) = self.update_arrow_directions() {
            self.error_code = 6;
            self.error_detail = world_error_detail(error);
            return self.error_code;
        }
        self.last_rotating_events = self.last_step_stats.sampled_events;
        self.last_tail_contacts = self.last_step_stats.tail_contacts;
        let collisions = self
            .last_rotating_events
            .saturating_add(self.last_tail_contacts)
            .saturating_add(
                usize::try_from(self.last_step_stats.ballistic_impacts).unwrap_or(usize::MAX),
            );
        self.total_collisions = self
            .total_collisions
            .saturating_add(u32::try_from(collisions).unwrap_or(u32::MAX));
        self.cleanup_projectiles();
        if self.is_quiescent() {
            self.last_rotating_events = 0;
            self.last_tail_contacts = 0;
        }
        0
    }

    fn set_projectile_type(&mut self, projectile_type: i32) -> i32 {
        if !self.projectile_ids.is_empty() {
            return -1;
        }
        let Some(projectile_type) = ProjectileType::from_i32(projectile_type) else {
            return -1;
        };
        self.projectile_type = Some(projectile_type);
        0
    }

    fn render_role_for(&self, id: BodyId) -> i32 {
        if id.0 >= PROJECTILE_ID_START {
            return match self.projectile_type {
                Some(ProjectileType::Arrow) => 4,
                Some(ProjectileType::Rigid) => 5,
                None | Some(ProjectileType::Sphere) => 3,
            };
        }
        role_for(id)
    }

    fn shoot(&mut self, velocity_x: i32, velocity_y: i32, velocity_z: i32) -> i32 {
        let input_velocity = Vec3i::new(
            velocity_x.clamp(-PROJECTILE_SPEED_LIMIT, PROJECTILE_SPEED_LIMIT),
            velocity_y.clamp(-PROJECTILE_SPEED_LIMIT, PROJECTILE_SPEED_LIMIT),
            velocity_z.clamp(-PROJECTILE_SPEED_LIMIT, PROJECTILE_SPEED_LIMIT),
        );
        if input_velocity == Vec3i::ZERO {
            return -1;
        }
        let Some(player) = self.world.box_by_id(PLAYER_ID) else {
            return -1;
        };
        let player_position = player.body().position();
        let spawn = Vec3i::new(
            player_position.x + input_velocity.x / 3,
            player_position.y + 12 + input_velocity.y / 3,
            player_position.z + input_velocity.z / 3,
        );
        let velocity = Vec3i::new(
            input_velocity.x.saturating_mul(ROTATING_TICKS_PER_SECOND),
            input_velocity.y.saturating_mul(ROTATING_TICKS_PER_SECOND),
            input_velocity.z.saturating_mul(ROTATING_TICKS_PER_SECOND),
        );
        let id = BodyId(self.next_projectile_id);
        self.next_projectile_id = self.next_projectile_id.saturating_add(1);
        let impact_policy = controller::scenario_rules::projectile_impact_policy();

        let material = Material::new(impact_policy.restitution_milli());
        let layers = controller::scenario_rules::projectile_layers();
        let added = match self.projectile_type {
            None => self.world.add_box(
                rotating_box(
                    RigidBody::dynamic(id, spawn, velocity, Vec3i::new(3, 3, 3))
                        .with_material(material),
                )
                // Keep the pre-selector WASM behavior stable for historical benchmark callers.
                .with_collision_layers(layers)
                .with_transient_contacts(),
            ),
            Some(ProjectileType::Sphere) => {
                let Ok(projectile) = BallisticSphere3d::new(id, spawn, velocity, 3, 1) else {
                    self.error_code = 5;
                    return -1;
                };
                self.world.add_ballistic_sphere(
                    projectile
                        .with_material(material)
                        .with_collision_layers(layers),
                    impact_policy.retire_on_contact(),
                )
            }
            Some(ProjectileType::Arrow) => self.world.add_box(
                RigidBox3d::new(
                    RigidBody::dynamic(id, spawn, velocity, Vec3i::new(2, 2, 12))
                        .with_material(material),
                    AngularState3d::new(
                        projectile_direction_orientation(input_velocity),
                        AngularVelocity3d::default(),
                    ),
                )
                .expect("sandbox arrow orientation must be valid")
                .with_rotation_locked()
                .with_collision_layers(layers)
                .with_transient_contacts()
                .with_aggressive_sleep(),
            ),
            Some(ProjectileType::Rigid) => self.world.add_box(
                rotating_box(
                    RigidBody::dynamic(id, spawn, velocity, Vec3i::new(3, 3, 3))
                        .with_material(material),
                )
                .with_collision_layers(layers),
            ),
        };

        if added.is_err() {
            self.error_code = 5;
            return -1;
        }
        self.projectile_ids.push(id);
        if self.projectile_ids.len() > MAX_PROJECTILES {
            let oldest = self.projectile_ids.remove(0);
            if self.world.remove_box(oldest).is_some()
                || self.world.remove_ballistic_sphere(oldest).is_some()
            {
                self.projectiles_evicted_by_cap = self.projectiles_evicted_by_cap.saturating_add(1);
            }
        }
        i32::try_from(id.0).unwrap_or(i32::MAX)
    }

    fn update_arrow_directions(&mut self) -> Result<(), RotatingWorldError3d> {
        if self.projectile_type != Some(ProjectileType::Arrow) || self.projectile_ids.is_empty() {
            return Ok(());
        }
        let updates = self
            .projectile_ids
            .iter()
            .copied()
            .filter_map(|id| {
                self.world.box_by_id(id).and_then(|rigid_box| {
                    let velocity = rigid_box.body().velocity();
                    (velocity != Vec3i::ZERO)
                        .then_some((id, projectile_direction_orientation(velocity)))
                })
            })
            .collect::<Vec<_>>();
        self.world.set_orientations(&updates)
    }

    fn cleanup_projectiles(&mut self) {
        let retire_on_contact =
            controller::scenario_rules::projectile_impact_policy().retire_on_contact();
        let stale = self
            .projectile_ids
            .iter()
            .copied()
            .filter_map(|id| {
                if let Some(rigid_box) = self.world.box_by_id(id) {
                    let position = rigid_box.body().position();
                    let out_of_bounds = position.x.abs() > 1_200
                        || position.y < -300
                        || position.y > 900
                        || position.z.abs() > 1_200;
                    if out_of_bounds {
                        return Some((id, 2_u8));
                    }
                    if retire_on_contact
                        && self
                            .world
                            .body_contacts(id)
                            .is_ok_and(|contacts| !contacts.is_empty())
                    {
                        return Some((id, 1_u8));
                    }
                    return None;
                }

                if let Some(projectile) = self.world.ballistic_sphere_by_id(id) {
                    let position = projectile.position();
                    let out_of_bounds = position.x.abs() > 1_200
                        || position.y < -300
                        || position.y > 900
                        || position.z.abs() > 1_200;
                    return out_of_bounds.then_some((id, 2_u8));
                }

                // Analytic impact-retire removes the sphere inside the event timeline and records the
                // lifecycle counter in the step report. Only the demo membership list remains to prune.
                Some((id, 0_u8))
            })
            .collect::<Vec<_>>();

        for (id, reason) in stale {
            match reason {
                1 => {
                    if self.world.remove_box(id).is_some() {
                        self.projectiles_retired_on_contact =
                            self.projectiles_retired_on_contact.saturating_add(1);
                    }
                }
                2 if self.world.remove_box(id).is_some()
                    || self.world.remove_ballistic_sphere(id).is_some() =>
                {
                    self.projectiles_retired_out_of_bounds =
                        self.projectiles_retired_out_of_bounds.saturating_add(1);
                }
                _ => {}
            }
            self.projectile_ids.retain(|candidate| *candidate != id);
        }
    }

    fn body_count(&self) -> usize {
        self.world
            .boxes()
            .count()
            .saturating_add(self.world.ballistic_sphere_count())
    }

    fn active_projectile_count(&self) -> usize {
        self.projectile_ids
            .iter()
            .copied()
            .filter(|id| {
                self.world.ballistic_sphere_by_id(*id).is_some()
                    || (self.world.box_by_id(*id).is_some() && !self.world.is_sleeping(*id))
            })
            .count()
    }

    fn body_at(&self, index: u32) -> Option<&RigidBody> {
        self.world.boxes().nth(index as usize).map(RigidBox3d::body)
    }

    fn angular_at(&self, index: u32) -> AngularState3d {
        self.world.boxes().nth(index as usize).map_or(
            AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
            RigidBox3d::angular,
        )
    }
}

fn world_error_detail(error: RotatingWorldError3d) -> i32 {
    match error {
        RotatingWorldError3d::PersistentTailResolutionLimit(_) => 601,
        RotatingWorldError3d::PersistentTailMotionUnsafe(_) => 602,
        RotatingWorldError3d::PersistentTailArithmeticOverflow(_) => 603,
        RotatingWorldError3d::Repeated(RepeatedRotatingEventError3d::EventLimit(_)) => 611,
        RotatingWorldError3d::Repeated(RepeatedRotatingEventError3d::BallisticEventLimit(_)) => 616,
        RotatingWorldError3d::Repeated(RepeatedRotatingEventError3d::RatioTooLarge) => 612,
        RotatingWorldError3d::Repeated(RepeatedRotatingEventError3d::InvalidRemainder(_)) => 613,
        RotatingWorldError3d::Repeated(RepeatedRotatingEventError3d::Frontier(_)) => 614,
        RotatingWorldError3d::Repeated(RepeatedRotatingEventError3d::Response(_)) => 615,
        RotatingWorldError3d::Repeated(RepeatedRotatingEventError3d::Ballistic(
            BallisticTimelineError3d::DuplicateBody(_),
        )) => 651,
        RotatingWorldError3d::Repeated(RepeatedRotatingEventError3d::Ballistic(
            BallisticTimelineError3d::MissingProjectile(_),
        )) => 652,
        RotatingWorldError3d::Repeated(RepeatedRotatingEventError3d::Ballistic(
            BallisticTimelineError3d::MissingTarget(_),
        )) => 653,
        RotatingWorldError3d::Repeated(RepeatedRotatingEventError3d::Ballistic(
            BallisticTimelineError3d::FreeFlight(_),
        )) => 654,
        RotatingWorldError3d::Repeated(RepeatedRotatingEventError3d::Ballistic(
            BallisticTimelineError3d::Ballistic(BallisticSphereError3d::ArithmeticOverflow),
        )) => 655,
        RotatingWorldError3d::Repeated(RepeatedRotatingEventError3d::Ballistic(
            BallisticTimelineError3d::Ballistic(_),
        )) => 656,
        RotatingWorldError3d::Repeated(RepeatedRotatingEventError3d::Ballistic(
            BallisticTimelineError3d::Angular(_),
        )) => 657,
        RotatingWorldError3d::Repeated(RepeatedRotatingEventError3d::Ballistic(
            BallisticTimelineError3d::ArithmeticOverflow,
        )) => 658,
        RotatingWorldError3d::Repeated(_) => 610,
        RotatingWorldError3d::FreeFlight(_) => 620,
        RotatingWorldError3d::Contact(_) => 630,
        RotatingWorldError3d::Response(_) => 640,
        _ => 699,
    }
}

fn rotating_box(body: RigidBody) -> RigidBox3d {
    RigidBox3d::new(
        body,
        AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
    )
    .expect("sandbox rotating body must be valid")
}

fn integer_sqrt(value: u64) -> u32 {
    let mut low = 0_u64;
    let mut high = value.saturating_add(1);
    while low.saturating_add(1) < high {
        let middle = low + (high - low) / 2;
        if middle.saturating_mul(middle) <= value {
            low = middle;
        } else {
            high = middle;
        }
    }
    u32::try_from(low).unwrap_or(u32::MAX)
}

fn projectile_direction_orientation(direction: Vec3i) -> Orientation3d {
    let squared = [direction.x, direction.y, direction.z]
        .into_iter()
        .map(|component| i64::from(component).unsigned_abs().pow(2))
        .sum::<u64>();
    let length = i32::try_from(integer_sqrt(squared)).unwrap_or(i32::MAX);
    if direction.x == 0 && direction.y == 0 && direction.z > 0 {
        return Orientation3d::new(0, 1, 0, 0)
            .normalized()
            .expect("180-degree projectile orientation is valid");
    }

    Orientation3d::new(
        direction.y,
        direction.x.saturating_neg(),
        0,
        length.saturating_sub(direction.z),
    )
    .normalized()
    .unwrap_or(Orientation3d::IDENTITY)
}

fn role_for(id: BodyId) -> i32 {
    if id == PLAYER_ID {
        1
    } else if id.0 >= PROJECTILE_ID_START {
        3
    } else if id.0 >= 100 {
        2
    } else {
        0
    }
}

fn saturating_u32(value: u64) -> u32 {
    u32::try_from(value).unwrap_or(u32::MAX)
}

std::thread_local! {
    static SANDBOX: RefCell<Sandbox> = RefCell::new(
        Sandbox::new().expect("the built-in physics sandbox must be valid")
    );
}

fn with_sandbox<R>(callback: impl FnOnce(&Sandbox) -> R) -> R {
    SANDBOX.with(|sandbox| callback(&sandbox.borrow()))
}

fn with_sandbox_mut<R>(callback: impl FnOnce(&mut Sandbox) -> R) -> R {
    SANDBOX.with(|sandbox| callback(&mut sandbox.borrow_mut()))
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_reset() {
    with_sandbox_mut(|sandbox| {
        *sandbox = Sandbox::new().expect("the built-in physics sandbox must be valid");
    });
}

/// Explicit comparison mode: 0 = legacy physical interactions, 1 = linear pushing / passive support.
/// Invalid modes leave the current sandbox untouched. Legacy reset retains the original benchmark.
#[unsafe(no_mangle)]
pub extern "C" fn sandbox_reset_with_character_mode(mode: i32) -> i32 {
    if mode != 0 && mode != 1 {
        return -1;
    }
    let Ok(replacement) = Sandbox::with_character_mode(mode == 1) else {
        return -2;
    };
    with_sandbox_mut(|sandbox| *sandbox = replacement);
    0
}

/// Independent comparison axes. Invalid options do not mutate the current scene.
#[unsafe(no_mangle)]
pub extern "C" fn sandbox_reset_with_options(character_mode: i32, upright_crates: i32) -> i32 {
    if !(0..=1).contains(&character_mode) || !(0..=1).contains(&upright_crates) {
        return -1;
    }
    let Ok(replacement) = Sandbox::with_options(character_mode == 1, upright_crates == 1) else {
        return -2;
    };
    with_sandbox_mut(|sandbox| *sandbox = replacement);
    0
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_step(move_x: i32, move_z: i32, jump: i32) -> i32 {
    with_sandbox_mut(|sandbox| sandbox.step(move_x, move_z, jump != 0))
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_step_velocity(velocity_x: i32, velocity_z: i32, jump: i32) -> i32 {
    with_sandbox_mut(|sandbox| sandbox.step_velocity(velocity_x, velocity_z, jump != 0))
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_set_projectile_type(projectile_type: i32) -> i32 {
    with_sandbox_mut(|sandbox| sandbox.set_projectile_type(projectile_type))
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_projectile_type() -> i32 {
    with_sandbox(|sandbox| sandbox.projectile_type.map_or(-1, |value| value as i32))
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_shoot(velocity_x: i32, velocity_y: i32, velocity_z: i32) -> i32 {
    with_sandbox_mut(|sandbox| sandbox.shoot(velocity_x, velocity_y, velocity_z))
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_is_quiescent() -> i32 {
    with_sandbox(|sandbox| i32::from(sandbox.is_quiescent()))
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_refresh_render_snapshot() -> usize {
    with_sandbox(render_snapshot::refresh)
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_render_snapshot_len() -> usize {
    render_snapshot::len()
}

#[unsafe(no_mangle)]
pub const extern "C" fn sandbox_render_snapshot_stride() -> usize {
    render_snapshot::STRIDE
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_body_count() -> u32 {
    with_sandbox(|sandbox| u32::try_from(sandbox.body_count()).unwrap_or(u32::MAX))
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_projectile_count() -> u32 {
    with_sandbox(|sandbox| u32::try_from(sandbox.projectile_ids.len()).unwrap_or(u32::MAX))
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_active_projectile_count() -> u32 {
    with_sandbox(|sandbox| u32::try_from(sandbox.active_projectile_count()).unwrap_or(u32::MAX))
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_projectiles_retired_on_contact() -> u32 {
    with_sandbox(|sandbox| sandbox.projectiles_retired_on_contact)
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_projectiles_retired_out_of_bounds() -> u32 {
    with_sandbox(|sandbox| sandbox.projectiles_retired_out_of_bounds)
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_projectiles_evicted_by_cap() -> u32 {
    with_sandbox(|sandbox| sandbox.projectiles_evicted_by_cap)
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_body_role(index: u32) -> i32 {
    with_sandbox(|sandbox| {
        sandbox
            .body_at(index)
            .map(|body| sandbox.render_role_for(body.id()))
            .unwrap_or(-1)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_body_x(index: u32) -> i32 {
    with_sandbox(|sandbox| sandbox.body_at(index).map_or(0, |body| body.position().x))
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_body_y(index: u32) -> i32 {
    with_sandbox(|sandbox| sandbox.body_at(index).map_or(0, |body| body.position().y))
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_body_z(index: u32) -> i32 {
    with_sandbox(|sandbox| sandbox.body_at(index).map_or(0, |body| body.position().z))
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_body_half_x(index: u32) -> i32 {
    with_sandbox(|sandbox| {
        sandbox
            .body_at(index)
            .map_or(0, |body| body.half_extents().x)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_body_half_y(index: u32) -> i32 {
    with_sandbox(|sandbox| {
        sandbox
            .body_at(index)
            .map_or(0, |body| body.half_extents().y)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_body_half_z(index: u32) -> i32 {
    with_sandbox(|sandbox| {
        sandbox
            .body_at(index)
            .map_or(0, |body| body.half_extents().z)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_body_orientation_x(index: u32) -> i32 {
    with_sandbox(|sandbox| sandbox.angular_at(index).orientation.x)
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_body_orientation_y(index: u32) -> i32 {
    with_sandbox(|sandbox| sandbox.angular_at(index).orientation.y)
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_body_orientation_z(index: u32) -> i32 {
    with_sandbox(|sandbox| sandbox.angular_at(index).orientation.z)
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_body_orientation_w(index: u32) -> i32 {
    with_sandbox(|sandbox| sandbox.angular_at(index).orientation.w)
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_last_collision_events() -> u32 {
    with_sandbox(|sandbox| {
        let rigid = sandbox
            .last_rotating_events
            .saturating_add(sandbox.last_tail_contacts);
        saturating_u32(
            u64::try_from(rigid)
                .unwrap_or(u64::MAX)
                .saturating_add(sandbox.last_step_stats.ballistic_impacts),
        )
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_last_sampled_events() -> u32 {
    with_sandbox(|sandbox| {
        u32::try_from(sandbox.last_step_stats.sampled_events).unwrap_or(u32::MAX)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_last_tail_contacts() -> u32 {
    with_sandbox(|sandbox| u32::try_from(sandbox.last_step_stats.tail_contacts).unwrap_or(u32::MAX))
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_last_tail_slices() -> u32 {
    with_sandbox(|sandbox| saturating_u32(sandbox.last_step_stats.tail_slices))
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_last_tail_replays() -> u32 {
    with_sandbox(|sandbox| saturating_u32(sandbox.last_step_stats.tail_replays))
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_last_tail_candidate_pairs() -> u32 {
    with_sandbox(|sandbox| saturating_u32(sandbox.last_step_stats.tail_candidate_pairs))
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_last_tail_broad_phase_queries() -> u32 {
    with_sandbox(|sandbox| saturating_u32(sandbox.last_step_stats.tail_broad_phase_queries))
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_last_tail_broad_phase_rebuilds() -> u32 {
    with_sandbox(|sandbox| saturating_u32(sandbox.last_step_stats.tail_broad_phase_rebuilds))
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_last_tail_broad_phase_reuses() -> u32 {
    with_sandbox(|sandbox| saturating_u32(sandbox.last_step_stats.tail_broad_phase_reuses))
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_last_broad_phase_queries() -> u32 {
    with_sandbox(|sandbox| saturating_u32(sandbox.last_step_stats.broad_phase_queries))
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_last_broad_phase_rebuilds() -> u32 {
    with_sandbox(|sandbox| saturating_u32(sandbox.last_step_stats.broad_phase_rebuilds))
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_last_broad_phase_reuses() -> u32 {
    with_sandbox(|sandbox| saturating_u32(sandbox.last_step_stats.broad_phase_reuses))
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_last_broad_phase_incremental_updates() -> u32 {
    with_sandbox(|sandbox| saturating_u32(sandbox.last_step_stats.broad_phase_incremental_updates))
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_last_broad_phase_reinserts() -> u32 {
    with_sandbox(|sandbox| saturating_u32(sandbox.last_step_stats.broad_phase_reinserts))
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_last_broad_phase_rotations() -> u32 {
    with_sandbox(|sandbox| saturating_u32(sandbox.last_step_stats.broad_phase_rotations))
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_last_broad_phase_partial_queries() -> u32 {
    with_sandbox(|sandbox| saturating_u32(sandbox.last_step_stats.broad_phase_partial_queries))
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_last_broad_phase_partial_body_updates() -> u32 {
    with_sandbox(|sandbox| saturating_u32(sandbox.last_step_stats.broad_phase_partial_body_updates))
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_last_response_scratch_index_rebuilds() -> u32 {
    with_sandbox(|sandbox| saturating_u32(sandbox.last_step_stats.response_scratch_index_rebuilds))
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_last_event_response_passes() -> u32 {
    with_sandbox(|sandbox| saturating_u32(sandbox.last_step_stats.event_response_passes))
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_last_stabilization_passes() -> u32 {
    with_sandbox(|sandbox| saturating_u32(sandbox.last_step_stats.stabilization_passes))
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_last_stabilizations_hitting_limit() -> u32 {
    with_sandbox(|sandbox| saturating_u32(sandbox.last_step_stats.stabilizations_hitting_limit))
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_last_stabilization_candidate_pairs() -> u32 {
    with_sandbox(|sandbox| saturating_u32(sandbox.last_step_stats.stabilization_candidate_pairs))
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_last_stabilization_exact_contacts() -> u32 {
    with_sandbox(|sandbox| saturating_u32(sandbox.last_step_stats.stabilization_exact_contacts))
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_last_stabilization_active_bodies() -> u32 {
    with_sandbox(|sandbox| saturating_u32(sandbox.last_step_stats.stabilization_active_bodies))
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_last_ballistic_sphere_count() -> u32 {
    with_sandbox(|sandbox| {
        u32::try_from(sandbox.last_step_stats.ballistic_sphere_count).unwrap_or(u32::MAX)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_last_ballistic_query_rounds() -> u32 {
    with_sandbox(|sandbox| saturating_u32(sandbox.last_step_stats.ballistic_query_rounds))
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_last_ballistic_target_bound_checks() -> u32 {
    with_sandbox(|sandbox| saturating_u32(sandbox.last_step_stats.ballistic_target_bound_checks))
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_last_ballistic_broad_phase_candidates() -> u32 {
    with_sandbox(|sandbox| saturating_u32(sandbox.last_step_stats.ballistic_broad_phase_candidates))
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_last_ballistic_toi_tests() -> u32 {
    with_sandbox(|sandbox| saturating_u32(sandbox.last_step_stats.ballistic_toi_tests))
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_last_ballistic_feature_tests() -> u32 {
    with_sandbox(|sandbox| saturating_u32(sandbox.last_step_stats.ballistic_feature_tests))
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_last_ballistic_motion_samples() -> u32 {
    with_sandbox(|sandbox| saturating_u32(sandbox.last_step_stats.ballistic_motion_samples))
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_last_ballistic_impacts() -> u32 {
    with_sandbox(|sandbox| saturating_u32(sandbox.last_step_stats.ballistic_impacts))
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_last_ballistic_retired() -> u32 {
    with_sandbox(|sandbox| saturating_u32(sandbox.last_step_stats.ballistic_retired))
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_total_collisions() -> u32 {
    with_sandbox(|sandbox| sandbox.total_collisions)
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_grounded() -> i32 {
    with_sandbox(|sandbox| match sandbox.grounded() {
        Ok(true) => 1,
        Ok(false) | Err(_) => 0,
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_error_code() -> i32 {
    with_sandbox(|sandbox| sandbox.error_code)
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_error_detail() -> i32 {
    with_sandbox(|sandbox| sandbox.error_detail)
}

#[cfg(test)]
mod tests {
    use physics_engine::{
        BodyId, ContactPersistence3d, Orientation3d, RepeatedRotatingEventError3d, RigidBody,
        RotatingWorldError3d, Vec3i,
    };

    use super::{PLAYER_ID, ProjectileType, Sandbox, rotating_box, world_error_detail};

    fn settle_player(sandbox: &mut Sandbox) {
        for _ in 0..240 {
            assert_eq!(sandbox.step(0, 0, false), 0);
        }
    }

    #[test]
    fn world_error_detail_keeps_event_limit_actionable() {
        assert_eq!(
            world_error_detail(RotatingWorldError3d::Repeated(
                RepeatedRotatingEventError3d::EventLimit(32)
            )),
            611
        );
    }

    #[test]
    fn sandbox_contains_rotation_locked_player_in_unified_world() {
        let sandbox = Sandbox::new().expect("valid sandbox");
        let player = sandbox.world.box_by_id(PLAYER_ID).expect("player");
        assert!(player.rotation_locked());
        assert!(player.angular().angular_velocity.is_zero());
        assert!(sandbox.world.boxes().count() >= 10);
    }

    #[test]
    fn quiescent_idle_step_is_an_exact_no_op_until_input_reactivates_the_player() {
        let mut sandbox = Sandbox::new().expect("valid sandbox");
        settle_player(&mut sandbox);
        assert!(sandbox.is_quiescent());
        assert_eq!(sandbox.last_rotating_events, 0);
        assert_eq!(sandbox.last_tail_contacts, 0);
        let settled = sandbox.world.boxes().cloned().collect::<Vec<_>>();

        for _ in 0..120 {
            assert_eq!(sandbox.step(0, 0, false), 0);
        }
        assert!(sandbox.is_quiescent());
        assert_eq!(sandbox.world.boxes().cloned().collect::<Vec<_>>(), settled);
        assert_eq!(sandbox.last_rotating_events, 0);
        assert_eq!(sandbox.last_tail_contacts, 0);

        assert_eq!(sandbox.step(0, -7, false), 0);
        assert!(!sandbox.is_quiescent());
    }

    #[test]
    fn canonical_controller_keeps_sub_legacy_velocity_precision() {
        let mut sandbox = Sandbox::new().expect("valid sandbox");
        for _ in 0..4 {
            assert_eq!(sandbox.step_velocity(73, -413, false), 0);
        }
        let velocity = sandbox
            .world
            .box_by_id(PLAYER_ID)
            .expect("player")
            .body()
            .velocity();
        assert_eq!(velocity.x, 73);
        assert_eq!(velocity.z, -413);
    }

    #[test]
    fn canonical_controller_does_not_zero_external_horizontal_momentum() {
        let mut sandbox = Sandbox::new().expect("valid sandbox");
        settle_player(&mut sandbox);
        sandbox
            .world
            .set_linear_velocity(PLAYER_ID, Vec3i::new(300, 0, 0))
            .expect("inject external horizontal momentum");

        assert_eq!(sandbox.step_velocity(0, 0, false), 0);
        let velocity_x = sandbox
            .world
            .box_by_id(PLAYER_ID)
            .expect("player")
            .body()
            .velocity()
            .x;
        assert!(
            velocity_x > 0 && velocity_x < 300,
            "controller erased or failed to oppose external momentum: {velocity_x}"
        );
    }

    #[test]
    fn side_wall_contact_does_not_make_player_grounded() {
        let mut sandbox = Sandbox::new().expect("valid sandbox");
        sandbox.world.remove_box(BodyId(10)).expect("remove floor");
        sandbox
            .world
            .add_box(rotating_box(RigidBody::fixed(
                BodyId(900),
                Vec3i::new(20, 38, 320),
                Vec3i::new(8, 100, 100),
            )))
            .expect("touching side wall");

        assert!(
            !sandbox.grounded().expect("support-normal query"),
            "side-wall contact incorrectly counted as ground support"
        );
    }

    #[test]
    fn grounded_player_can_jump() {
        let mut sandbox = Sandbox::new().expect("valid sandbox");
        settle_player(&mut sandbox);
        assert!(sandbox.grounded().expect("valid support query"));
        let before = sandbox
            .world
            .box_by_id(PLAYER_ID)
            .expect("player")
            .body()
            .position()
            .y;

        assert_eq!(sandbox.step(0, 0, true), 0);

        let player = sandbox
            .world
            .box_by_id(PLAYER_ID)
            .expect("player after jump");
        assert!(
            player.body().position().y > before,
            "jump did not move upward"
        );
        assert!(
            player.body().velocity().y > 0,
            "jump did not preserve upward velocity"
        );
        assert!(!sandbox.grounded().expect("valid airborne support query"));
        assert!(player.rotation_locked());
        assert!(player.angular().angular_velocity.is_zero());
    }

    #[test]
    fn player_pushes_dynamic_box_without_tumbling() {
        let mut sandbox = Sandbox::new().expect("valid sandbox");
        settle_player(&mut sandbox);
        let crate_id = BodyId(901);
        sandbox
            .world
            .add_box(rotating_box(
                RigidBody::dynamic(
                    crate_id,
                    Vec3i::new(0, 18, 285),
                    Vec3i::ZERO,
                    Vec3i::new(18, 18, 18),
                )
                .with_mass(2),
            ))
            .expect("test crate");
        let before = sandbox
            .world
            .box_by_id(crate_id)
            .expect("test crate")
            .body()
            .position()
            .z;

        for _ in 0..8 {
            assert_eq!(sandbox.step(0, -7, false), 0);
        }

        let crate_z = sandbox
            .world
            .box_by_id(crate_id)
            .expect("test crate after push")
            .body()
            .position()
            .z;
        let player = sandbox
            .world
            .box_by_id(PLAYER_ID)
            .expect("player after push");
        assert!(crate_z < before, "player did not push the dynamic crate");
        assert!(player.rotation_locked());
        assert!(player.angular().angular_velocity.is_zero());
        assert_eq!(
            player.angular().orientation,
            physics_engine::Orientation3d::IDENTITY
        );
    }

    #[test]
    fn sphere_projectile_uses_dedicated_ballistic_state() {
        let mut sandbox = Sandbox::new().expect("valid sandbox");
        assert_eq!(
            sandbox.set_projectile_type(ProjectileType::Sphere as i32),
            0
        );
        assert_eq!(sandbox.projectile_type, Some(ProjectileType::Sphere));
        let projectile = sandbox.shoot(0, 0, -96);
        assert!(projectile >= 0);
        let id = BodyId(projectile as u64);

        assert!(
            sandbox.world.box_by_id(id).is_none(),
            "analytic sphere must not enter the rotating rigid-body collection"
        );
        let projectile = sandbox
            .world
            .ballistic_sphere_by_id(id)
            .expect("spawned analytic sphere projectile");
        assert_eq!(projectile.radius(), 3);
        assert_eq!(projectile.mass_units(), 1);
        assert_eq!(sandbox.active_projectile_count(), 1);
    }

    #[test]
    fn single_impact_sphere_does_not_leave_the_crate_stack_bouncing() {
        let mut sandbox = Sandbox::new().expect("valid sandbox");
        let all_pair_bits = (1_i32 << 11) - 2;
        let impact_retire_rules = (1_i32 << 29) | all_pair_bits | (1_i32 << 14) | (2_i32 << 12);
        let rules =
            super::controller::scenario_rules::ScenarioRules::decode(impact_retire_rules, false)
                .expect("impact-retire scenario rules");
        super::controller::scenario_rules::apply_to_sandbox(&mut sandbox, rules)
            .expect("apply impact-retire rules");
        settle_player(&mut sandbox);

        let crate_ids = [BodyId(100), BodyId(101)];
        let before = crate_ids.map(|id| sandbox.world.box_by_id(id).expect("crate").clone());
        assert_eq!(
            sandbox.set_projectile_type(ProjectileType::Sphere as i32),
            0
        );
        assert!(sandbox.shoot(-38, 0, -88) >= 0);

        let mut crate_responded = false;
        for tick in 0..180 {
            assert_eq!(
                sandbox.step_velocity(0, 0, false),
                0,
                "sphere impact/settling tick {tick}, detail {}",
                sandbox.error_detail
            );
            for (index, id) in crate_ids.into_iter().enumerate() {
                let current = sandbox.world.box_by_id(id).expect("crate after impact");
                crate_responded |= current.body().position() != before[index].body().position()
                    || current.angular() != before[index].angular();
            }
            if sandbox.projectiles_retired_on_contact == 1
                && crate_ids
                    .into_iter()
                    .all(|id| sandbox.world.is_sleeping(id))
            {
                break;
            }
        }

        assert!(
            crate_responded,
            "sphere must still transfer impact to the crate"
        );
        assert_eq!(sandbox.projectiles_retired_on_contact, 1);
        assert_eq!(sandbox.world.ballistic_sphere_count(), 0);
        assert!(
            crate_ids
                .into_iter()
                .all(|id| sandbox.world.is_sleeping(id)),
            "impacted stack must settle instead of continuing to bounce"
        );

        let settled =
            crate_ids.map(|id| sandbox.world.box_by_id(id).expect("settled crate").clone());
        for tick in 0..60 {
            assert_eq!(
                sandbox.step_velocity(0, 0, false),
                0,
                "post-settle tick {tick}"
            );
        }
        for (index, id) in crate_ids.into_iter().enumerate() {
            assert_eq!(
                sandbox.world.box_by_id(id),
                Some(&settled[index]),
                "settled crate {id:?} must retain its exact resting pose"
            );
        }
    }

    #[test]
    fn tower_sphere_impact_does_not_readmit_stabilized_contacts_as_rigid_events() {
        let mut sandbox = Sandbox::with_tower_options(true, true).expect("valid tower sandbox");
        let all_pair_bits = (1_i32 << 11) - 2;
        let impact_retire_rules = (1_i32 << 29) | all_pair_bits | (1_i32 << 14) | (2_i32 << 12);
        let rules =
            super::controller::scenario_rules::ScenarioRules::decode(impact_retire_rules, true)
                .expect("impact-retire tower rules");
        super::controller::scenario_rules::apply_to_sandbox(&mut sandbox, rules)
            .expect("apply impact-retire tower rules");
        settle_player(&mut sandbox);
        assert!(
            sandbox.is_quiescent(),
            "tower fixture must settle before the impact"
        );

        assert_eq!(
            sandbox.set_projectile_type(ProjectileType::Sphere as i32),
            0
        );
        assert!(sandbox.shoot(0, 0, -96) >= 0);

        let mut peak_sampled_events = 0_usize;
        let mut peak_stabilization_limit_hits = 0_u64;
        for tick in 0..120 {
            assert_eq!(
                sandbox.step_velocity(0, 0, false),
                0,
                "tower sphere impact tick {tick}, detail {}",
                sandbox.error_detail
            );
            peak_sampled_events = peak_sampled_events.max(sandbox.last_step_stats.sampled_events);
            peak_stabilization_limit_hits = peak_stabilization_limit_hits
                .max(sandbox.last_step_stats.stabilizations_hitting_limit);
            if sandbox.projectiles_retired_on_contact == 1 && sandbox.is_quiescent() {
                break;
            }
        }

        assert_eq!(sandbox.projectiles_retired_on_contact, 1);
        assert_eq!(sandbox.world.ballistic_sphere_count(), 0);
        assert!(
            peak_sampled_events < 8,
            "one localized sphere impact re-admitted {peak_sampled_events} sampled rigid events"
        );
        assert!(
            peak_stabilization_limit_hits < 8,
            "one localized sphere impact exhausted stabilization {peak_stabilization_limit_hits} times"
        );
    }

    #[test]
    fn analytic_sphere_stress_lane_completes_without_exhausting_event_budget() {
        let mut sandbox = Sandbox::new().expect("valid sandbox");
        settle_player(&mut sandbox);
        assert_eq!(
            sandbox.set_projectile_type(ProjectileType::Sphere as i32),
            0
        );

        for tick in 0..240 {
            if tick < 150 && tick % 5 == 0 {
                let shot_index = tick / 5;
                let projectile_x = if shot_index % 2 == 0 { 38 } else { -38 };
                let projectile_y = if shot_index % 3 == 2 { -7 } else { 0 };
                assert!(
                    sandbox.shoot(projectile_x, projectile_y, -88) >= 0,
                    "projectile creation failed at tick {tick}"
                );
            }
            let (x, z) = if tick >= 80 { (271, -321) } else { (0, -420) };
            let before = sandbox
                .projectile_ids
                .first()
                .and_then(|id| sandbox.world.ballistic_sphere_by_id(*id))
                .copied();
            let result = sandbox.step_velocity(x, z, false);
            let after = sandbox
                .projectile_ids
                .first()
                .and_then(|id| sandbox.world.ballistic_sphere_by_id(*id))
                .copied();
            let crates = (100_u64..=105)
                .filter_map(|id| {
                    sandbox.world.box_by_id(BodyId(id)).map(|rigid_box| {
                        (
                            id,
                            rigid_box.body().position(),
                            rigid_box.body().velocity(),
                            rigid_box.angular(),
                        )
                    })
                })
                .collect::<Vec<_>>();
            assert_eq!(
                result,
                0,
                "sphere stress lane failed at tick {tick} with detail {}, {} ballistic spheres live; before={before:?}; after={after:?}; crates={crates:?}",
                sandbox.error_detail,
                sandbox.world.ballistic_sphere_count(),
            );
        }
    }

    #[test]
    fn analytic_sphere_does_not_enter_rigid_sleep_bookkeeping() {
        let mut sandbox = Sandbox::new().expect("valid sandbox");
        assert_eq!(
            sandbox.set_projectile_type(ProjectileType::Sphere as i32),
            0
        );
        let projectile = sandbox.shoot(0, 0, -96);
        assert!(projectile >= 0);
        let id = BodyId(projectile as u64);

        assert!(sandbox.world.ballistic_sphere_by_id(id).is_some());
        assert!(!sandbox.world.is_sleeping(id));
        assert_eq!(sandbox.active_projectile_count(), 1);
    }

    #[test]
    fn arrow_projectile_is_slender_directional_and_transient() {
        let mut sandbox = Sandbox::new().expect("valid sandbox");
        assert_eq!(sandbox.set_projectile_type(ProjectileType::Arrow as i32), 0);
        let projectile = sandbox.shoot(96, 0, 0);
        assert!(projectile >= 0);
        let projectile = sandbox
            .world
            .box_by_id(BodyId(projectile as u64))
            .expect("spawned arrow projectile");
        assert_eq!(projectile.body().half_extents(), Vec3i::new(2, 2, 12));
        assert!(projectile.rotation_locked());
        assert_ne!(projectile.angular().orientation, Orientation3d::IDENTITY);
        assert!(projectile.angular().angular_velocity.is_zero());
        assert_eq!(
            projectile.contact_persistence(),
            ContactPersistence3d::Transient
        );
        assert_eq!(
            projectile.sleep_mode(),
            physics_engine::SleepMode3d::Aggressive
        );
    }

    #[test]
    fn arrow_direction_follows_ballistic_velocity_without_angular_velocity() {
        let mut sandbox = Sandbox::new().expect("valid sandbox");
        assert_eq!(sandbox.set_projectile_type(ProjectileType::Arrow as i32), 0);
        let projectile = sandbox.shoot(96, 0, 0);
        assert!(projectile >= 0);
        let id = BodyId(projectile as u64);
        let before = sandbox
            .world
            .box_by_id(id)
            .expect("arrow before step")
            .angular()
            .orientation;

        assert_eq!(sandbox.step(0, 0, false), 0);
        let arrow = sandbox.world.box_by_id(id).expect("arrow after step");
        assert_ne!(arrow.angular().orientation, before);
        assert!(arrow.angular().angular_velocity.is_zero());
    }

    #[test]
    fn rigid_projectile_keeps_general_persistent_contact_path() {
        let mut sandbox = Sandbox::new().expect("valid sandbox");
        assert_eq!(sandbox.set_projectile_type(ProjectileType::Rigid as i32), 0);
        let projectile = sandbox.shoot(0, 0, -96);
        assert!(projectile >= 0);
        let projectile = sandbox
            .world
            .box_by_id(BodyId(projectile as u64))
            .expect("spawned rigid projectile");
        assert!(!projectile.rotation_locked());
        assert_eq!(
            projectile.contact_persistence(),
            ContactPersistence3d::Persistent
        );
    }

    #[test]
    fn projectile_type_cannot_change_while_projectiles_are_live() {
        let mut sandbox = Sandbox::new().expect("valid sandbox");
        assert_eq!(
            sandbox.set_projectile_type(ProjectileType::Sphere as i32),
            0
        );
        assert!(sandbox.shoot(0, 0, -96) >= 0);
        assert_eq!(
            sandbox.set_projectile_type(ProjectileType::Arrow as i32),
            -1
        );
        assert_eq!(sandbox.projectile_type, Some(ProjectileType::Sphere));
    }

    #[test]
    fn analytic_sphere_does_not_tunnel_through_thin_target() {
        let mut sandbox = Sandbox::new().expect("valid sandbox");
        assert_eq!(
            sandbox.set_projectile_type(ProjectileType::Sphere as i32),
            0
        );
        let projectile = sandbox.shoot(0, 0, -96);
        assert!(projectile >= 0);
        let id = BodyId(projectile as u64);

        for _ in 0..7 {
            assert_eq!(sandbox.step(0, 0, false), 0);
        }

        let projectile = sandbox
            .world
            .ballistic_sphere_by_id(id)
            .expect("physical sphere remains after bouncing from the thin target");
        assert!(
            projectile.position().z > -190,
            "analytic sphere tunneled through the thin target: {:?}",
            projectile.position()
        );
        assert!(sandbox.last_step_stats.ballistic_query_rounds > 0);
    }

    #[test]
    fn fast_projectile_does_not_tunnel_through_thin_target() {
        let mut sandbox = Sandbox::new().expect("valid sandbox");
        assert_eq!(sandbox.projectile_type, None);
        let projectile = sandbox.shoot(0, 0, -96);
        assert!(projectile >= 0);
        assert_eq!(
            sandbox
                .world
                .box_by_id(BodyId(projectile as u64))
                .expect("spawned projectile")
                .contact_persistence(),
            ContactPersistence3d::Transient
        );

        for _ in 0..7 {
            assert_eq!(sandbox.step(0, 0, false), 0);
        }

        let projectile = sandbox
            .world
            .box_by_id(physics_engine::BodyId(projectile as u64))
            .expect("projectile remains in the bounded test world");
        assert!(
            projectile.body().position().z > -190,
            "projectile tunneled through the thin sampled-CCD target: {:?}",
            projectile.body().position()
        );
    }
}

#[cfg(test)]
#[path = "character_interaction_tests.rs"]
mod character_interaction_tests;
