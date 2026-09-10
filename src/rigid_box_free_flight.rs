use std::{error::Error, fmt};

use crate::{
    AngularError3d, AngularState3d, BodyId, BodyKind, RigidBox3d, RotationalSweepBounds3d,
    RotationalSweepError3d, Vec3i, integrate_orientation,
    rotational_sweep::rotational_sweep_bounds_for_center_interval,
};

/// Explicit rational timestep and acceleration for collision-free rotating-box sampling.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct RigidBoxFreeFlightConfig3d {
    pub gravity: Vec3i,
    pub timestep_numerator: i32,
    pub timestep_denominator: i32,
}

impl RigidBoxFreeFlightConfig3d {
    #[must_use]
    pub const fn new(
        gravity: Vec3i,
        timestep_numerator: i32,
        timestep_denominator: i32,
    ) -> Self {
        Self {
            gravity,
            timestep_numerator,
            timestep_denominator,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RigidBoxFreeFlightError3d {
    NegativeTimestepNumerator(i32),
    NonPositiveTimestepDenominator(i32),
    ZeroFractionDenominator,
    FractionOutOfRange { numerator: u32, denominator: u32 },
    RatioTooLarge,
    ArithmeticOverflow(BodyId),
    Angular(AngularError3d),
    Sweep(RotationalSweepError3d),
}

impl fmt::Display for RigidBoxFreeFlightError3d {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NegativeTimestepNumerator(value) => write!(
                formatter,
                "rigid-box free-flight timestep numerator must be non-negative, got {value}"
            ),
            Self::NonPositiveTimestepDenominator(value) => write!(
                formatter,
                "rigid-box free-flight timestep denominator must be positive, got {value}"
            ),
            Self::ZeroFractionDenominator => write!(
                formatter,
                "rigid-box free-flight sample fraction denominator must be positive"
            ),
            Self::FractionOutOfRange {
                numerator,
                denominator,
            } => write!(
                formatter,
                "rigid-box free-flight sample fraction must be in 0..=1, got {numerator}/{denominator}"
            ),
            Self::RatioTooLarge => write!(
                formatter,
                "rigid-box free-flight reduced timestep ratio exceeds the supported i32 contract"
            ),
            Self::ArithmeticOverflow(body) => write!(
                formatter,
                "rigid-box free-flight arithmetic overflowed for body {}",
                body.0
            ),
            Self::Angular(error) => write!(formatter, "rigid-box free-flight rotation failed: {error}"),
            Self::Sweep(error) => write!(formatter, "rigid-box free-flight sweep failed: {error}"),
        }
    }
}

impl Error for RigidBoxFreeFlightError3d {}

impl From<AngularError3d> for RigidBoxFreeFlightError3d {
    fn from(value: AngularError3d) -> Self {
        Self::Angular(value)
    }
}

impl From<RotationalSweepError3d> for RigidBoxFreeFlightError3d {
    fn from(value: RotationalSweepError3d) -> Self {
        Self::Sweep(value)
    }
}

/// Samples one validated rotating box directly from the start of a collision-free interval.
///
/// Sampling is direct rather than iterative: every fraction is reconstructed from the same start
/// state, so sample count does not silently alter motion. Linear motion uses the same semi-implicit
/// ordering as the existing translational `World` for a full integer tick: apply proportional gravity
/// to velocity, then advance position with the resulting velocity. Orientation uses the engine's
/// deterministic fixed-point quaternion integrator.
///
/// `fraction_numerator / fraction_denominator` must be within `0..=1`.
///
/// # Errors
///
/// Returns [`RigidBoxFreeFlightError3d`] for malformed time/fraction inputs, unsupported reduced ratios,
/// or checked linear/angular arithmetic overflow.
pub fn sample_rigid_box_free_flight(
    rigid_box: &RigidBox3d,
    config: RigidBoxFreeFlightConfig3d,
    fraction_numerator: u32,
    fraction_denominator: u32,
) -> Result<RigidBox3d, RigidBoxFreeFlightError3d> {
    let (step_numerator, step_denominator) = sample_step_ratio(
        config,
        fraction_numerator,
        fraction_denominator,
    )?;
    if step_numerator == 0 || rigid_box.body.kind() == BodyKind::Fixed {
        return Ok(rigid_box.clone());
    }

    let mut body = rigid_box.body.clone();
    let id = body.id();
    let mut velocity = body.velocity();
    let mut position = body.position();
    for axis in 0..3 {
        let velocity_delta = rounded_ratio(
            i128::from(config.gravity.component(axis)),
            step_numerator,
            step_denominator,
        )?;
        let next_velocity = i128::from(velocity.component(axis))
            .checked_add(velocity_delta)
            .ok_or(RigidBoxFreeFlightError3d::ArithmeticOverflow(id))?;
        let next_velocity = i32::try_from(next_velocity)
            .map_err(|_| RigidBoxFreeFlightError3d::ArithmeticOverflow(id))?;
        velocity.set_component(axis, next_velocity);

        let position_delta = rounded_ratio(
            i128::from(next_velocity),
            step_numerator,
            step_denominator,
        )?;
        let next_position = i128::from(position.component(axis))
            .checked_add(position_delta)
            .ok_or(RigidBoxFreeFlightError3d::ArithmeticOverflow(id))?;
        let next_position = i32::try_from(next_position)
            .map_err(|_| RigidBoxFreeFlightError3d::ArithmeticOverflow(id))?;
        position.set_component(axis, next_position);
    }
    body.velocity = velocity;
    body.position = position;

    let angular = AngularState3d::new(
        integrate_orientation(
            rigid_box.angular.orientation,
            rigid_box.angular.angular_velocity,
            step_numerator,
            step_denominator,
        )?,
        rigid_box.angular.angular_velocity,
    );
    Ok(RigidBox3d { body, angular })
}

/// Conservatively bounds every direct free-flight sample over the configured interval.
///
/// Per axis, the center interval uses an absolute upper bound on speed after acceleration and then on
/// displacement. This may overproduce broad-phase candidates, but it cannot lose a sampled contact when
/// velocity reverses and the center reaches an interior extremum outside the start/end interval.
/// Arbitrary orientation is enclosed by the same circumscribed-box radius as [`crate::rotational_sweep_bounds`].
///
/// # Errors
///
/// Returns [`RigidBoxFreeFlightError3d`] for malformed time configuration or checked bound arithmetic
/// overflow.
pub fn rigid_box_free_flight_sweep_bounds(
    rigid_box: &RigidBox3d,
    config: RigidBoxFreeFlightConfig3d,
) -> Result<RotationalSweepBounds3d, RigidBoxFreeFlightError3d> {
    let (step_numerator, step_denominator) = sample_step_ratio(config, 1, 1)?;
    let center = rigid_box.body.position();
    let center = [i64::from(center.x), i64::from(center.y), i64::from(center.z)];
    let mut center_minimum = center;
    let mut center_maximum = center;

    if rigid_box.body.kind() == BodyKind::Dynamic && step_numerator != 0 {
        for axis in 0..3 {
            let displacement = conservative_axis_displacement(
                rigid_box.body.velocity().component(axis),
                config.gravity.component(axis),
                step_numerator,
                step_denominator,
            )?;
            let displacement = i64::try_from(displacement).map_err(|_| {
                RigidBoxFreeFlightError3d::ArithmeticOverflow(rigid_box.body.id())
            })?;
            center_minimum[axis] = center[axis].checked_sub(displacement).ok_or(
                RigidBoxFreeFlightError3d::ArithmeticOverflow(rigid_box.body.id()),
            )?;
            center_maximum[axis] = center[axis].checked_add(displacement).ok_or(
                RigidBoxFreeFlightError3d::ArithmeticOverflow(rigid_box.body.id()),
            )?;
        }
    }

    Ok(rotational_sweep_bounds_for_center_interval(
        rigid_box.body.half_extents(),
        center_minimum,
        center_maximum,
    )?)
}

fn sample_step_ratio(
    config: RigidBoxFreeFlightConfig3d,
    fraction_numerator: u32,
    fraction_denominator: u32,
) -> Result<(i32, i32), RigidBoxFreeFlightError3d> {
    if config.timestep_numerator < 0 {
        return Err(RigidBoxFreeFlightError3d::NegativeTimestepNumerator(
            config.timestep_numerator,
        ));
    }
    if config.timestep_denominator <= 0 {
        return Err(RigidBoxFreeFlightError3d::NonPositiveTimestepDenominator(
            config.timestep_denominator,
        ));
    }
    if fraction_denominator == 0 {
        return Err(RigidBoxFreeFlightError3d::ZeroFractionDenominator);
    }
    if fraction_numerator > fraction_denominator {
        return Err(RigidBoxFreeFlightError3d::FractionOutOfRange {
            numerator: fraction_numerator,
            denominator: fraction_denominator,
        });
    }

    let numerator = u128::from(
        u32::try_from(config.timestep_numerator)
            .map_err(|_| RigidBoxFreeFlightError3d::RatioTooLarge)?,
    )
    .checked_mul(u128::from(fraction_numerator))
    .ok_or(RigidBoxFreeFlightError3d::RatioTooLarge)?;
    let denominator = u128::from(
        u32::try_from(config.timestep_denominator)
            .map_err(|_| RigidBoxFreeFlightError3d::RatioTooLarge)?,
    )
    .checked_mul(u128::from(fraction_denominator))
    .ok_or(RigidBoxFreeFlightError3d::RatioTooLarge)?;
    let divisor = greatest_common_divisor(numerator, denominator);
    let numerator = numerator / divisor;
    let denominator = denominator / divisor;
    Ok((
        i32::try_from(numerator).map_err(|_| RigidBoxFreeFlightError3d::RatioTooLarge)?,
        i32::try_from(denominator).map_err(|_| RigidBoxFreeFlightError3d::RatioTooLarge)?,
    ))
}

fn rounded_ratio(
    value: i128,
    numerator: i32,
    denominator: i32,
) -> Result<i128, RigidBoxFreeFlightError3d> {
    let numerator = value
        .checked_mul(i128::from(numerator))
        .ok_or(RigidBoxFreeFlightError3d::RatioTooLarge)?;
    let denominator = i128::from(denominator);
    let half = denominator / 2;
    let adjusted = if numerator >= 0 {
        numerator.checked_add(half)
    } else {
        numerator.checked_sub(half)
    }
    .ok_or(RigidBoxFreeFlightError3d::RatioTooLarge)?;
    Ok(adjusted / denominator)
}

fn conservative_axis_displacement(
    velocity: i32,
    acceleration: i32,
    numerator: i32,
    denominator: i32,
) -> Result<u128, RigidBoxFreeFlightError3d> {
    let numerator = u128::from(
        u32::try_from(numerator).map_err(|_| RigidBoxFreeFlightError3d::RatioTooLarge)?,
    );
    let denominator = u128::from(
        u32::try_from(denominator).map_err(|_| RigidBoxFreeFlightError3d::RatioTooLarge)?,
    );
    let acceleration_delta = ceil_ratio(
        u128::from(acceleration.unsigned_abs())
            .checked_mul(numerator)
            .ok_or(RigidBoxFreeFlightError3d::RatioTooLarge)?,
        denominator,
    )?;
    let maximum_speed = u128::from(velocity.unsigned_abs())
        .checked_add(acceleration_delta)
        .ok_or(RigidBoxFreeFlightError3d::RatioTooLarge)?;
    ceil_ratio(
        maximum_speed
            .checked_mul(numerator)
            .ok_or(RigidBoxFreeFlightError3d::RatioTooLarge)?,
        denominator,
    )
}

fn ceil_ratio(
    numerator: u128,
    denominator: u128,
) -> Result<u128, RigidBoxFreeFlightError3d> {
    if denominator == 0 {
        return Err(RigidBoxFreeFlightError3d::RatioTooLarge);
    }
    let quotient = numerator / denominator;
    if numerator % denominator == 0 {
        Ok(quotient)
    } else {
        quotient
            .checked_add(1)
            .ok_or(RigidBoxFreeFlightError3d::RatioTooLarge)
    }
}

fn greatest_common_divisor(mut left: u128, mut right: u128) -> u128 {
    while right != 0 {
        let remainder = left % right;
        left = right;
        right = remainder;
    }
    left
}

#[cfg(test)]
mod tests {
    use crate::{
        AngularState3d, AngularVelocity3d, BodyId, Orientation3d, RigidBody, RigidBox3d, Vec3i,
        World, WorldConfig, oriented_box_vertices, rotational_sweep_bounds,
    };

    use super::{
        RigidBoxFreeFlightConfig3d, RigidBoxFreeFlightError3d,
        rigid_box_free_flight_sweep_bounds, sample_rigid_box_free_flight,
    };

    fn rotating_box(body: RigidBody, angular_velocity: AngularVelocity3d) -> RigidBox3d {
        RigidBox3d::new(
            body,
            AngularState3d::new(Orientation3d::IDENTITY, angular_velocity),
        )
        .expect("valid rotating box")
    }

    #[test]
    fn full_integer_tick_matches_existing_world_translation() {
        let body = RigidBody::dynamic(
            BodyId(1),
            Vec3i::new(10, 20, -30),
            Vec3i::new(120, -45, 11),
            Vec3i::new(2, 3, 4),
        );
        let rigid_box = rotating_box(body.clone(), AngularVelocity3d::new(0, 0, 500_000));
        let gravity = Vec3i::new(0, -3, 1);
        let sampled = sample_rigid_box_free_flight(
            &rigid_box,
            RigidBoxFreeFlightConfig3d::new(gravity, 1, 1),
            1,
            1,
        )
        .expect("valid full-tick sample");

        let mut world = World::new(WorldConfig {
            gravity,
            ..WorldConfig::default()
        });
        world.add_body(body).expect("valid body");
        world.step(1).expect("collision-free world step");

        assert_eq!(sampled.body(), world.body(BodyId(1)).expect("body remains"));
        assert_ne!(sampled.angular().orientation, Orientation3d::IDENTITY);
    }

    #[test]
    fn direct_fraction_sampling_is_repeatable() {
        let rigid_box = rotating_box(
            RigidBody::dynamic(
                BodyId(2),
                Vec3i::ZERO,
                Vec3i::new(90, 12, -5),
                Vec3i::new(2, 2, 2),
            ),
            AngularVelocity3d::new(100_000, 200_000, 300_000),
        );
        let config = RigidBoxFreeFlightConfig3d::new(Vec3i::new(0, -10, 0), 1, 1);

        let first = sample_rigid_box_free_flight(&rigid_box, config, 7, 16)
            .expect("valid fractional sample");
        let second = sample_rigid_box_free_flight(&rigid_box, config, 7, 16)
            .expect("same valid fractional sample");
        assert_eq!(first, second);
    }

    #[test]
    fn accelerated_sweep_contains_reversal_samples_missed_by_endpoint_bounds() {
        let rigid_box = rotating_box(
            RigidBody::dynamic(
                BodyId(3),
                Vec3i::ZERO,
                Vec3i::new(100, 0, 0),
                Vec3i::new(1, 1, 1),
            ),
            AngularVelocity3d::default(),
        );
        let config = RigidBoxFreeFlightConfig3d::new(Vec3i::new(-200, 0, 0), 1, 1);
        let end = sample_rigid_box_free_flight(&rigid_box, config, 1, 1)
            .expect("valid endpoint sample");
        let endpoint_only = rotational_sweep_bounds(rigid_box.oriented_box(), end.oriented_box())
            .expect("valid endpoint sweep");
        let quarter = sample_rigid_box_free_flight(&rigid_box, config, 1, 4)
            .expect("valid interior sample");
        assert!(
            oriented_box_vertices(quarter.oriented_box())
                .expect("valid quarter OBB")
                .into_iter()
                .any(|vertex| !endpoint_only.contains([
                    i64::from(vertex.x),
                    i64::from(vertex.y),
                    i64::from(vertex.z),
                ]))
        );

        let conservative = rigid_box_free_flight_sweep_bounds(&rigid_box, config)
            .expect("valid accelerated sweep");
        for numerator in 0..=16 {
            let sample = sample_rigid_box_free_flight(&rigid_box, config, numerator, 16)
                .expect("valid bounded sample");
            for vertex in oriented_box_vertices(sample.oriented_box()).expect("valid sampled OBB") {
                assert!(conservative.contains([
                    i64::from(vertex.x),
                    i64::from(vertex.y),
                    i64::from(vertex.z),
                ]));
            }
        }
    }

    #[test]
    fn fixed_body_is_unchanged() {
        let rigid_box = rotating_box(
            RigidBody::fixed(BodyId(4), Vec3i::new(10, 20, 30), Vec3i::new(2, 2, 2)),
            AngularVelocity3d::default(),
        );
        let sampled = sample_rigid_box_free_flight(
            &rigid_box,
            RigidBoxFreeFlightConfig3d::new(Vec3i::new(0, -100, 0), 3, 2),
            1,
            2,
        )
        .expect("valid fixed-body sample");
        assert_eq!(sampled, rigid_box);
    }

    #[test]
    fn malformed_fraction_fails_closed() {
        let rigid_box = rotating_box(
            RigidBody::dynamic(
                BodyId(5),
                Vec3i::ZERO,
                Vec3i::ZERO,
                Vec3i::new(1, 1, 1),
            ),
            AngularVelocity3d::default(),
        );
        let config = RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 1, 1);

        assert_eq!(
            sample_rigid_box_free_flight(&rigid_box, config, 1, 0),
            Err(RigidBoxFreeFlightError3d::ZeroFractionDenominator)
        );
        assert_eq!(
            sample_rigid_box_free_flight(&rigid_box, config, 2, 1),
            Err(RigidBoxFreeFlightError3d::FractionOutOfRange {
                numerator: 2,
                denominator: 1,
            })
        );
    }
}
