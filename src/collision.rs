use std::cmp::Ordering;

use crate::RigidBody;

pub const SUBTICKS_PER_TICK: u64 = 1_u64 << 32;
pub(crate) const SUBTICK_SCALE: i128 = 1_i128 << 32;

/// Axis-aligned contact normal pointing from the right body toward the left body.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ContactNormal {
    pub x: i8,
    pub y: i8,
    pub z: i8,
}

impl ContactNormal {
    pub(crate) const fn for_axis(axis: usize, component: i8) -> Self {
        match axis {
            0 => Self {
                x: component,
                y: 0,
                z: 0,
            },
            1 => Self {
                x: 0,
                y: component,
                z: 0,
            },
            2 => Self {
                x: 0,
                y: 0,
                z: component,
            },
            _ => unreachable!(),
        }
    }

    pub(crate) const fn axis(self) -> usize {
        if self.x != 0 {
            0
        } else if self.y != 0 {
            1
        } else {
            2
        }
    }

    pub(crate) const fn component(self) -> i8 {
        match self.axis() {
            0 => self.x,
            1 => self.y,
            2 => self.z,
            _ => unreachable!(),
        }
    }
}

/// Q32.32-quantized time measured from the beginning of a physics step.
#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub struct TimeOfImpact {
    subticks: u64,
}

impl TimeOfImpact {
    pub(crate) const fn from_subticks(subticks: u64) -> Self {
        Self { subticks }
    }

    #[must_use]
    pub const fn subticks(self) -> u64 {
        self.subticks
    }

    #[must_use]
    pub const fn whole_ticks(self) -> u64 {
        self.subticks / SUBTICKS_PER_TICK
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SweepHit {
    pub time: TimeOfImpact,
    pub normal: ContactNormal,
}

/// Returns whether the two bodies currently overlap or touch.
#[must_use]
pub fn overlap_aabb(left: &RigidBody, right: &RigidBody) -> bool {
    (0..3).all(|axis| {
        let delta = i64::from(left.position().component(axis))
            - i64::from(right.position().component(axis));
        let extent = i64::from(left.half_extents().component(axis))
            + i64::from(right.half_extents().component(axis));
        delta.abs() <= extent
    })
}

/// Computes the first continuous AABB impact inside `ticks` without mutating either body.
///
/// Existing overlap/touching is intentionally not reported; callers can use [`overlap_aabb`] for
/// that case. The result is quantized upward to Q32.32 so a collision cannot be skipped by the
/// quantization boundary.
#[must_use]
pub fn swept_aabb(left: &RigidBody, right: &RigidBody, ticks: i32) -> Option<SweepHit> {
    if ticks <= 0 || overlap_aabb(left, right) {
        return None;
    }

    let horizon = i128::from(ticks) * SUBTICK_SCALE;
    let left_motion = MotionAabb::from_body(left);
    let right_motion = MotionAabb::from_body(right);
    let hit = sweep_motion(left_motion, right_motion, horizon)?;
    let subticks = u64::try_from(hit.time.ceil()).ok()?;
    Some(SweepHit {
        time: TimeOfImpact::from_subticks(subticks),
        normal: hit.normal,
    })
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct MotionAabb {
    pub center_scaled: [i128; 3],
    pub half_scaled: [i128; 3],
    pub velocity: [i32; 3],
}

impl MotionAabb {
    fn from_body(body: &RigidBody) -> Self {
        Self {
            center_scaled: [
                i128::from(body.position().x) * SUBTICK_SCALE,
                i128::from(body.position().y) * SUBTICK_SCALE,
                i128::from(body.position().z) * SUBTICK_SCALE,
            ],
            half_scaled: [
                i128::from(body.half_extents().x) * SUBTICK_SCALE,
                i128::from(body.half_extents().y) * SUBTICK_SCALE,
                i128::from(body.half_extents().z) * SUBTICK_SCALE,
            ],
            velocity: [body.velocity().x, body.velocity().y, body.velocity().z],
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct Ratio {
    numerator: i128,
    denominator: i128,
}

impl Ratio {
    fn new(mut numerator: i128, mut denominator: i128) -> Self {
        debug_assert_ne!(denominator, 0);
        if denominator < 0 {
            numerator = -numerator;
            denominator = -denominator;
        }
        Self {
            numerator,
            denominator,
        }
    }

    const fn from_integer(value: i128) -> Self {
        Self {
            numerator: value,
            denominator: 1,
        }
    }

    pub(crate) fn compare(self, other: Self) -> Ordering {
        (self.numerator * other.denominator).cmp(&(other.numerator * self.denominator))
    }

    pub(crate) fn ceil(self) -> i128 {
        let quotient = self.numerator / self.denominator;
        let remainder = self.numerator % self.denominator;
        if remainder != 0 && self.numerator > 0 {
            quotient + 1
        } else {
            quotient
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct MotionSweepHit {
    pub time: Ratio,
    pub normal: ContactNormal,
}

pub(crate) fn sweep_motion(
    left: MotionAabb,
    right: MotionAabb,
    horizon_subticks: i128,
) -> Option<MotionSweepHit> {
    if motion_overlap(left, right) {
        return None;
    }

    let mut global_entry: Option<Ratio> = None;
    let mut global_exit: Option<Ratio> = None;
    let mut impact_axis = 0_usize;

    for axis in 0..3 {
        let position = left.center_scaled[axis] - right.center_scaled[axis];
        let extent = left.half_scaled[axis] + right.half_scaled[axis];
        let velocity = i128::from(left.velocity[axis]) - i128::from(right.velocity[axis]);

        if velocity == 0 {
            if position.abs() > extent {
                return None;
            }
            continue;
        }

        let first = Ratio::new(-extent - position, velocity);
        let second = Ratio::new(extent - position, velocity);
        let (axis_entry, axis_exit) = if first.compare(second) == Ordering::Greater {
            (second, first)
        } else {
            (first, second)
        };

        if global_entry.is_none_or(|entry| axis_entry.compare(entry) == Ordering::Greater) {
            global_entry = Some(axis_entry);
            impact_axis = axis;
        }
        if global_exit.is_none_or(|exit| axis_exit.compare(exit) == Ordering::Less) {
            global_exit = Some(axis_exit);
        }
    }

    let entry = global_entry?;
    let exit = global_exit.unwrap_or_else(|| Ratio::from_integer(horizon_subticks));
    let zero = Ratio::from_integer(0);
    let horizon = Ratio::from_integer(horizon_subticks);

    if entry.compare(exit) == Ordering::Greater
        || exit.compare(zero) == Ordering::Less
        || entry.compare(zero) != Ordering::Greater
        || entry.compare(horizon) == Ordering::Greater
    {
        return None;
    }

    let relative_velocity =
        i64::from(left.velocity[impact_axis]) - i64::from(right.velocity[impact_axis]);
    let normal_component = if relative_velocity > 0 { -1 } else { 1 };
    Some(MotionSweepHit {
        time: entry,
        normal: ContactNormal::for_axis(impact_axis, normal_component),
    })
}

pub(crate) fn swept_bounds_overlap(
    left: MotionAabb,
    right: MotionAabb,
    horizon_subticks: i128,
) -> bool {
    (0..3).all(|axis| {
        let left_end = left.center_scaled[axis]
            + i128::from(left.velocity[axis]) * horizon_subticks;
        let right_end = right.center_scaled[axis]
            + i128::from(right.velocity[axis]) * horizon_subticks;
        let left_min = left.center_scaled[axis].min(left_end) - left.half_scaled[axis];
        let left_max = left.center_scaled[axis].max(left_end) + left.half_scaled[axis];
        let right_min = right.center_scaled[axis].min(right_end) - right.half_scaled[axis];
        let right_max = right.center_scaled[axis].max(right_end) + right.half_scaled[axis];
        left_min <= right_max && right_min <= left_max
    })
}

fn motion_overlap(left: MotionAabb, right: MotionAabb) -> bool {
    (0..3).all(|axis| {
        (left.center_scaled[axis] - right.center_scaled[axis]).abs()
            <= left.half_scaled[axis] + right.half_scaled[axis]
    })
}
