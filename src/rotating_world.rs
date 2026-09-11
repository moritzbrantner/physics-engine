use std::{collections::BTreeMap, error::Error, fmt};

use crate::{
    ANGULAR_VELOCITY_SCALE, BodyId, BodyKind, OrientedBoxError3d,
    RepeatedRotatingEventConfig3d, RepeatedRotatingEventError3d, RigidBox3d,
    RigidBoxFreeFlightConfig3d, RigidBoxFreeFlightError3d, RotatingContactFrontier3d,
    RotatingContactResponseError3d, RotatingContactSearchConfig3d, RotatingContactSearchHit3d,
    RotationalSweepPair3d, SampledContactTime3d, Vec3i, advance_repeated_rotating_events,
    obb_contact_seed, resolve_rotating_contact_frontier, sample_rigid_box_free_flight,
};

const MAX_PERSISTENT_TAIL_SLICES: u32 = 1_024;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct RotatingWorldConfig3d {
    pub gravity: Vec3i,
    pub sample_count: u16,
    pub refinement_steps: u8,
    pub solver_passes: u8,
    pub max_events: u16,
}

impl Default for RotatingWorldConfig3d {
    fn default() -> Self {
        Self {
            gravity: Vec3i::new(0, -10, 0),
            sample_count: 32,
            refinement_steps: 4,
            solver_passes: 8,
            max_events: 32,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct RotatingWorldStepStats3d {
    pub body_count: usize,
    pub sampled_events: usize,
    pub tail_contacts: usize,
}

#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct RotatingWorldStepReport3d {
    pub stats: RotatingWorldStepStats3d,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RotatingWorldError3d {
    DuplicateBody(BodyId),
    MissingBody(BodyId),
    NegativeTimestepNumerator(i32),
    NonPositiveTimestepDenominator(i32),
    PersistentTailResolutionLimit(u32),
    PersistentTailMotionUnsafe(BodyId),
    PersistentTailArithmeticOverflow(BodyId),
    Repeated(RepeatedRotatingEventError3d),
    FreeFlight(RigidBoxFreeFlightError3d),
    Contact(OrientedBoxError3d),
    Response(RotatingContactResponseError3d),
}

impl fmt::Display for RotatingWorldError3d {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::DuplicateBody(id) => write!(formatter, "duplicate rotating body {}", id.0),
            Self::MissingBody(id) => write!(formatter, "rotating body {} does not exist", id.0),
            Self::NegativeTimestepNumerator(value) => write!(
                formatter,
                "rotating world timestep numerator must be non-negative, got {value}"
            ),
            Self::NonPositiveTimestepDenominator(value) => write!(
                formatter,
                "rotating world timestep denominator must be positive, got {value}"
            ),
            Self::PersistentTailResolutionLimit(limit) => write!(
                formatter,
                "persistent rotating contact tail needs more than {limit} deterministic slices"
            ),
            Self::PersistentTailMotionUnsafe(id) => write!(
                formatter,
                "persistent rotating contact motion became unsafe for body {} at the selected tail resolution",
                id.0
            ),
            Self::PersistentTailArithmeticOverflow(id) => write!(
                formatter,
                "persistent rotating contact motion bound overflowed for body {}",
                id.0
            ),
            Self::Repeated(error) => write!(formatter, "rotating event advance failed: {error}"),
            Self::FreeFlight(error) => {
                write!(formatter, "rotating tail free flight failed: {error}")
            }
            Self::Contact(error) => {
                write!(formatter, "rotating tail contact query failed: {error}")
            }
            Self::Response(error) => write!(formatter, "rotating tail response failed: {error}"),
        }
    }
}

impl Error for RotatingWorldError3d {}

impl From<RepeatedRotatingEventError3d> for RotatingWorldError3d {
    fn from(value: RepeatedRotatingEventError3d) -> Self {
        Self::Repeated(value)
    }
}

impl From<RigidBoxFreeFlightError3d> for RotatingWorldError3d {
    fn from(value: RigidBoxFreeFlightError3d) -> Self {
        Self::FreeFlight(value)
    }
}

impl From<OrientedBoxError3d> for RotatingWorldError3d {
    fn from(value: OrientedBoxError3d) -> Self {
        Self::Contact(value)
    }
}

impl From<RotatingContactResponseError3d> for RotatingWorldError3d {
    fn from(value: RotatingContactResponseError3d) -> Self {
        Self::Response(value)
    }
}

/// Deterministic rotating-cuboid world built from the engine's sampled OBB event pipeline.
///
/// Collision discovery remains explicitly sampled rotational handling rather than analytic rotational
/// CCD. The event pipeline resolves every admitted impact first. A contact-free exact tail still uses
/// one direct free-flight sample. When the tail begins with persistent contact, the remaining rational
/// time is instead consumed through bounded deterministic slices. Each slice conservatively limits
/// center-plus-rotational motion relative to the thinnest dynamic extent and re-stabilizes OBB contacts,
/// so a time-zero contact cannot be free-flown completely through before the next constraint solve.
/// If the configured bound cannot make that progress safe, the world fails closed.
#[derive(Clone, Debug)]
pub struct RotatingWorld3d {
    config: RotatingWorldConfig3d,
    boxes: BTreeMap<BodyId, RigidBox3d>,
}

impl RotatingWorld3d {
    #[must_use]
    pub fn new(config: RotatingWorldConfig3d) -> Self {
        Self {
            config,
            boxes: BTreeMap::new(),
        }
    }

    #[must_use]
    pub const fn config(&self) -> RotatingWorldConfig3d {
        self.config
    }

    pub fn add_box(&mut self, rigid_box: RigidBox3d) -> Result<(), RotatingWorldError3d> {
        let id = rigid_box.body().id();
        if self.boxes.contains_key(&id) {
            return Err(RotatingWorldError3d::DuplicateBody(id));
        }
        self.boxes.insert(id, rigid_box);
        Ok(())
    }

    pub fn remove_box(&mut self, id: BodyId) -> Option<RigidBox3d> {
        self.boxes.remove(&id)
    }

    #[must_use]
    pub fn box_by_id(&self, id: BodyId) -> Option<&RigidBox3d> {
        self.boxes.get(&id)
    }

    pub fn boxes(&self) -> impl Iterator<Item = &RigidBox3d> {
        self.boxes.values()
    }

    pub fn step(
        &mut self,
        timestep_numerator: i32,
        timestep_denominator: i32,
    ) -> Result<RotatingWorldStepReport3d, RotatingWorldError3d> {
        if timestep_numerator < 0 {
            return Err(RotatingWorldError3d::NegativeTimestepNumerator(
                timestep_numerator,
            ));
        }
        if timestep_denominator <= 0 {
            return Err(RotatingWorldError3d::NonPositiveTimestepDenominator(
                timestep_denominator,
            ));
        }
        if timestep_numerator == 0 {
            return Ok(RotatingWorldStepReport3d {
                stats: RotatingWorldStepStats3d {
                    body_count: self.boxes.len(),
                    ..RotatingWorldStepStats3d::default()
                },
            });
        }

        let boxes = self.boxes.values().cloned().collect::<Vec<_>>();
        let free_flight = RigidBoxFreeFlightConfig3d::new(
            self.config.gravity,
            timestep_numerator,
            timestep_denominator,
        );
        let advance = advance_repeated_rotating_events(
            &boxes,
            RepeatedRotatingEventConfig3d::new(
                RotatingContactSearchConfig3d::new(
                    free_flight,
                    self.config.sample_count,
                    self.config.refinement_steps,
                ),
                self.config.solver_passes,
                self.config.max_events,
            ),
        )?;

        let sampled_events = advance.events.len();
        let (boxes, tail_contacts) = if advance.remaining.timestep_numerator == 0 {
            (advance.boxes, 0)
        } else {
            consume_tail(advance.boxes, advance.remaining, self.config.solver_passes)?
        };
        self.boxes = boxes
            .into_iter()
            .map(|rigid_box| (rigid_box.body().id(), rigid_box))
            .collect();

        Ok(RotatingWorldStepReport3d {
            stats: RotatingWorldStepStats3d {
                body_count: self.boxes.len(),
                sampled_events,
                tail_contacts,
            },
        })
    }
}

fn consume_tail(
    boxes: Vec<RigidBox3d>,
    remaining: RigidBoxFreeFlightConfig3d,
    solver_passes: u8,
) -> Result<(Vec<RigidBox3d>, usize), RotatingWorldError3d> {
    if contact_frontier(&boxes)?.is_empty() {
        return free_flight_and_stabilize(boxes, remaining, solver_passes);
    }

    let (slice_count, slice_config) = persistent_tail_slice_config(&boxes, remaining)?;
    let mut current = boxes;
    let mut contact_count = 0_usize;

    for _ in 0..slice_count {
        for rigid_box in &current {
            if !tail_motion_within_extent(rigid_box, slice_config)? {
                return Err(RotatingWorldError3d::PersistentTailMotionUnsafe(
                    rigid_box.body().id(),
                ));
            }
        }
        let (next, contacts) = free_flight_and_stabilize(current, slice_config, solver_passes)?;
        current = next;
        contact_count = contact_count.saturating_add(contacts);
    }

    Ok((current, contact_count))
}

fn free_flight_and_stabilize(
    boxes: Vec<RigidBox3d>,
    config: RigidBoxFreeFlightConfig3d,
    solver_passes: u8,
) -> Result<(Vec<RigidBox3d>, usize), RotatingWorldError3d> {
    let sampled = boxes
        .iter()
        .map(|rigid_box| sample_rigid_box_free_flight(rigid_box, config, 1, 1))
        .collect::<Result<Vec<_>, _>>()?;
    let contacts = contact_frontier(&sampled)?;
    let contact_count = contacts.len();
    if contacts.is_empty() {
        return Ok((sampled, 0));
    }

    let response = resolve_rotating_contact_frontier(
        RotatingContactFrontier3d {
            boxes: sampled,
            time: SampledContactTime3d::ZERO,
            contacts,
            remaining_numerator: 0,
        },
        solver_passes,
    )?;
    Ok((response.boxes, contact_count))
}

fn persistent_tail_slice_config(
    boxes: &[RigidBox3d],
    remaining: RigidBoxFreeFlightConfig3d,
) -> Result<(u32, RigidBoxFreeFlightConfig3d), RotatingWorldError3d> {
    for slices in 1..=MAX_PERSISTENT_TAIL_SLICES {
        let slices_i32 = i32::try_from(slices).map_err(|_| {
            RotatingWorldError3d::PersistentTailResolutionLimit(MAX_PERSISTENT_TAIL_SLICES)
        })?;
        let denominator = remaining
            .timestep_denominator
            .checked_mul(slices_i32)
            .ok_or(RotatingWorldError3d::PersistentTailResolutionLimit(
                MAX_PERSISTENT_TAIL_SLICES,
            ))?;
        let config = RigidBoxFreeFlightConfig3d::new(
            remaining.gravity,
            remaining.timestep_numerator,
            denominator,
        );
        let mut safe = true;
        for rigid_box in boxes {
            if !tail_motion_within_extent(rigid_box, config)? {
                safe = false;
                break;
            }
        }
        if safe {
            return Ok((slices, config));
        }
    }

    Err(RotatingWorldError3d::PersistentTailResolutionLimit(
        MAX_PERSISTENT_TAIL_SLICES,
    ))
}

fn tail_motion_within_extent(
    rigid_box: &RigidBox3d,
    config: RigidBoxFreeFlightConfig3d,
) -> Result<bool, RotatingWorldError3d> {
    if rigid_box.body().kind() == BodyKind::Fixed || config.timestep_numerator == 0 {
        return Ok(true);
    }

    let id = rigid_box.body().id();
    let numerator = u128::from(config.timestep_numerator.unsigned_abs());
    let denominator = u128::from(config.timestep_denominator.unsigned_abs());
    let half = rigid_box.body().half_extents();
    let minimum_half = u128::from(half.x.min(half.y).min(half.z).unsigned_abs());

    let velocity = rigid_box.body().velocity();
    let velocity_components = [velocity.x, velocity.y, velocity.z];
    let gravity_components = [config.gravity.x, config.gravity.y, config.gravity.z];
    let mut translation_bound = 0_u128;
    for axis in 0..3 {
        let gravity_delta = ceil_mul_div(
            u128::from(gravity_components[axis].unsigned_abs()),
            numerator,
            denominator,
            id,
        )?;
        let speed_bound = u128::from(velocity_components[axis].unsigned_abs())
            .checked_add(gravity_delta)
            .ok_or(RotatingWorldError3d::PersistentTailArithmeticOverflow(id))?;
        let travel = ceil_mul_div(speed_bound, numerator, denominator, id)?;
        translation_bound = translation_bound.max(travel);
    }

    let angular = rigid_box.angular().angular_velocity;
    let angular_speed_l1 = u128::from(angular.x.unsigned_abs())
        .checked_add(u128::from(angular.y.unsigned_abs()))
        .and_then(|value| value.checked_add(u128::from(angular.z.unsigned_abs())))
        .ok_or(RotatingWorldError3d::PersistentTailArithmeticOverflow(id))?;
    let radius_bound = u128::from(half.x.unsigned_abs())
        .checked_add(u128::from(half.y.unsigned_abs()))
        .and_then(|value| value.checked_add(u128::from(half.z.unsigned_abs())))
        .ok_or(RotatingWorldError3d::PersistentTailArithmeticOverflow(id))?;
    let angular_denominator = denominator
        .checked_mul(u128::from(ANGULAR_VELOCITY_SCALE.unsigned_abs()))
        .ok_or(RotatingWorldError3d::PersistentTailArithmeticOverflow(id))?;
    let angular_motion = ceil_mul_div(
        radius_bound
            .checked_mul(angular_speed_l1)
            .ok_or(RotatingWorldError3d::PersistentTailArithmeticOverflow(id))?,
        numerator,
        angular_denominator,
        id,
    )?;

    let total_motion = translation_bound
        .checked_add(angular_motion)
        .ok_or(RotatingWorldError3d::PersistentTailArithmeticOverflow(id))?;
    Ok(total_motion <= minimum_half)
}

fn ceil_mul_div(
    value: u128,
    numerator: u128,
    denominator: u128,
    id: BodyId,
) -> Result<u128, RotatingWorldError3d> {
    if denominator == 0 {
        return Err(RotatingWorldError3d::PersistentTailArithmeticOverflow(id));
    }
    let product = value
        .checked_mul(numerator)
        .ok_or(RotatingWorldError3d::PersistentTailArithmeticOverflow(id))?;
    let adjusted = product
        .checked_add(denominator - 1)
        .ok_or(RotatingWorldError3d::PersistentTailArithmeticOverflow(id))?;
    Ok(adjusted / denominator)
}

fn contact_frontier(
    boxes: &[RigidBox3d],
) -> Result<Vec<RotatingContactSearchHit3d>, OrientedBoxError3d> {
    let mut contacts = Vec::new();
    for left_index in 0..boxes.len() {
        for right_index in (left_index + 1)..boxes.len() {
            let left = &boxes[left_index];
            let right = &boxes[right_index];
            if left.body().kind() == BodyKind::Fixed && right.body().kind() == BodyKind::Fixed {
                continue;
            }
            let Some(contact) = obb_contact_seed(left.oriented_box(), right.oriented_box())? else {
                continue;
            };
            contacts.push(RotatingContactSearchHit3d {
                time: SampledContactTime3d::ZERO,
                pair: RotationalSweepPair3d {
                    left: left.body().id(),
                    right: right.body().id(),
                },
                contact,
            });
        }
    }
    Ok(contacts)
}

#[cfg(test)]
mod tests {
    use crate::{
        AngularState3d, AngularVelocity3d, BodyId, Orientation3d, RigidBody, RigidBox3d, Vec3i,
    };

    use super::{RotatingWorld3d, RotatingWorldConfig3d};

    fn dynamic(id: u64, position: Vec3i, velocity: Vec3i, half: Vec3i) -> RigidBox3d {
        RigidBox3d::new(
            RigidBody::dynamic(BodyId(id), position, velocity, half),
            AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
        )
        .expect("valid dynamic box")
    }

    fn fixed(id: u64, position: Vec3i, half: Vec3i) -> RigidBox3d {
        RigidBox3d::new(
            RigidBody::fixed(BodyId(id), position, half),
            AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
        )
        .expect("valid fixed box")
    }

    fn world(gravity: Vec3i) -> RotatingWorld3d {
        RotatingWorld3d::new(RotatingWorldConfig3d {
            gravity,
            sample_count: 32,
            refinement_steps: 4,
            solver_passes: 8,
            max_events: 16,
        })
    }

    #[test]
    fn persistent_floor_contact_cannot_tunnel_across_a_large_tail() {
        let mut world = world(Vec3i::new(0, -100, 0));
        world
            .add_box(fixed(1, Vec3i::new(0, -1, 0), Vec3i::new(20, 1, 20)))
            .expect("floor");
        world
            .add_box(dynamic(
                2,
                Vec3i::new(0, 1, 0),
                Vec3i::ZERO,
                Vec3i::new(1, 1, 1),
            ))
            .expect("box");

        world.step(1, 1).expect("persistent tail must remain constrained");

        let body = world.box_by_id(BodyId(2)).expect("box remains").body();
        assert!(
            body.position().y >= 1,
            "box tunneled through a time-zero floor contact: {body:?}"
        );
    }

    #[test]
    fn resting_floor_contact_consumes_the_full_frame() {
        let mut world = world(Vec3i::new(0, -3_600, 0));
        world
            .add_box(fixed(1, Vec3i::new(0, -1, 0), Vec3i::new(20, 1, 20)))
            .expect("floor");
        world
            .add_box(dynamic(
                2,
                Vec3i::new(0, 1, 0),
                Vec3i::ZERO,
                Vec3i::new(1, 1, 1),
            ))
            .expect("box");

        for _ in 0..8 {
            world.step(1, 60).expect("stable frame");
        }

        let body = world.box_by_id(BodyId(2)).expect("box remains").body();
        assert!(
            body.position().y >= 1,
            "box sank through the floor: {body:?}"
        );
        assert!(
            body.velocity().y >= 0,
            "resting box kept downward velocity: {body:?}"
        );
    }

    #[test]
    fn off_center_projectile_impact_generates_spin() {
        let mut world = world(Vec3i::ZERO);
        world
            .add_box(dynamic(10, Vec3i::ZERO, Vec3i::ZERO, Vec3i::new(3, 3, 3)))
            .expect("target");
        world
            .add_box(dynamic(
                20,
                Vec3i::new(-12, 2, 0),
                Vec3i::new(720, 0, 0),
                Vec3i::new(1, 1, 1),
            ))
            .expect("projectile");

        world.step(1, 60).expect("impact frame");

        let target = world.box_by_id(BodyId(10)).expect("target remains");
        assert!(
            !target.angular().angular_velocity.is_zero(),
            "off-center impact did not generate angular velocity"
        );
    }

    #[test]
    fn identical_steps_are_bit_for_bit_repeatable() {
        let mut first = world(Vec3i::new(0, -3_600, 0));
        first
            .add_box(fixed(1, Vec3i::new(0, -1, 0), Vec3i::new(20, 1, 20)))
            .expect("floor");
        first
            .add_box(dynamic(
                2,
                Vec3i::new(0, 8, 0),
                Vec3i::new(120, 0, 0),
                Vec3i::new(2, 2, 2),
            ))
            .expect("box");
        let mut second = first.clone();

        for _ in 0..12 {
            first.step(1, 60).expect("first frame");
            second.step(1, 60).expect("second frame");
        }

        assert_eq!(
            first.boxes().cloned().collect::<Vec<_>>(),
            second.boxes().cloned().collect::<Vec<_>>()
        );
    }
}
