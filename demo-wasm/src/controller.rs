use physics_engine::Vec3i;

#[path = "baking.rs"]
mod baking;

pub const TICKS_PER_SECOND: i32 = 60;
pub const MAX_HORIZONTAL_SPEED: i32 = 7 * TICKS_PER_SECOND;
const MAX_CONTROL_DELTA_PER_TICK: i32 = 120;

/// Applies one character-controller intent update without erasing collision-generated horizontal momentum.
///
/// The desired X/Z components are already expressed in the physics engine's canonical velocity units.
/// Control authority approaches the desired velocity by a bounded amount each 60 Hz controller tick instead
/// of replacing the solver's current velocity. Vertical velocity remains supplied by the caller so gravity,
/// jumping, and contact response keep their existing authority.
#[must_use]
pub fn controlled_velocity(
    current: Vec3i,
    desired_x: i32,
    desired_z: i32,
    vertical_velocity: i32,
) -> Vec3i {
    let desired_x = desired_x.clamp(-MAX_HORIZONTAL_SPEED, MAX_HORIZONTAL_SPEED);
    let desired_z = desired_z.clamp(-MAX_HORIZONTAL_SPEED, MAX_HORIZONTAL_SPEED);
    Vec3i::new(
        approach(current.x, desired_x, MAX_CONTROL_DELTA_PER_TICK),
        vertical_velocity,
        approach(current.z, desired_z, MAX_CONTROL_DELTA_PER_TICK),
    )
}

fn approach(current: i32, target: i32, maximum_delta: i32) -> i32 {
    debug_assert!(maximum_delta >= 0);
    if current < target {
        current.saturating_add(maximum_delta).min(target)
    } else if current > target {
        current.saturating_sub(maximum_delta).max(target)
    } else {
        current
    }
}

#[cfg(test)]
mod tests {
    use physics_engine::Vec3i;

    use super::{MAX_HORIZONTAL_SPEED, controlled_velocity};

    #[test]
    fn controller_does_not_erase_external_horizontal_momentum_in_one_tick() {
        let next = controlled_velocity(Vec3i::new(300, 17, -240), 0, 0, 17);
        assert_eq!(next, Vec3i::new(180, 17, -120));
    }

    #[test]
    fn canonical_targets_keep_sub_legacy_step_precision() {
        let mut velocity = Vec3i::ZERO;
        for _ in 0..4 {
            velocity = controlled_velocity(velocity, 73, -413, velocity.y);
        }
        assert_eq!(velocity.x, 73);
        assert_eq!(velocity.z, -413);
        assert_ne!(velocity.x % 60, 0);
        assert_ne!(velocity.z % 60, 0);
    }

    #[test]
    fn controller_clamps_target_speed_without_clamping_external_momentum() {
        let next =
            controlled_velocity(Vec3i::new(MAX_HORIZONTAL_SPEED + 300, 0, 0), i32::MAX, 0, 0);
        assert_eq!(next.x, MAX_HORIZONTAL_SPEED + 180);
    }
}
