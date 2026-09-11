use std::{error::Error, fmt};

use crate::{
    AngularError3d, AngularState3d, AngularVelocity3d, BodyId, BodyKind, RigidBox3d,
    RotationalSweepBounds3d, RotationalSweepError3d, Vec3i,
    angular::integrate_orientation_exact_ratio,
    rotational_sweep::rotational_sweep_bounds_for_center_interval,
    wide_ratio::{ExactRatio, WideRatioError},
};

/// Explicit exact timestep and acceleration for collision-free rotating-box sampling.
///
/// The public constructor still accepts the ordinary signed `i32` timestep contract. Valid timesteps are
/// canonicalized into deterministic fixed-capacity limb storage so repeated sampled-event composition can
/// remain exact beyond one `i128/i128` pair without allocation or floating point.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct RigidBoxFreeFlightConfig3d {
    pub gravity: Vec3i,
    timestep: ExactRatio,
    invalid_numerator: Option<i128>,
    invalid_denominator: Option<i128>,
}

impl RigidBoxFreeFlightConfig3d {
    #[must_use]
    pub fn new(gravity: Vec3i, timestep_numerator: i32, timestep_denominator: i32) -> Self {
        Self::new_wide(
            gravity,
            i128::from(timestep_numerator),
            i128::from(timestep_denominator),
        )
    }

    #[must_use]
    pub(crate) fn new_wide(
        gravity: Vec3i,
        timestep_numerator: i128,
        timestep_denominator: i128,
    ) -> Self {
        let invalid_numerator = (timestep_numerator < 0).then_some(timestep_numerator);
        let invalid_denominator = (timestep_denominator <= 0).then_some(timestep_denominator);
        let timestep = if invalid_numerator.is_none() && invalid_denominator.is_none() {
            ExactRatio::new(
                timestep_numerator.unsigned_abs(),
                timestep_denominator.unsigned_abs(),
            )
            .expect("an i128 timestep fits the deterministic exact-ratio capacity")
        } else {
            ExactRatio::new(0, 1).expect("zero exact timestep is valid")
        };
        Self {
            gravity,
            timestep,
            invalid_numerator,
            invalid_denominator,
        }
    }

    fn from_exact(gravity: Vec3i, timestep: ExactRatio) -> Self {
        Self {
            gravity,
            timestep,
            invalid_numerator: None,
            invalid_denominator: None,
        }
    }

    pub(crate) fn exact_timestep(self) -> Result<ExactRatio, RigidBoxFreeFlightError3d> {
        if let Some(value) = self.invalid_numerator {
            return Err(RigidBoxFreeFlightError3d::NegativeTimestepNumerator(value));
        }
        if let Some(value) = self.invalid_denominator {
            return Err(RigidBoxFreeFlightError3d::NonPositiveTimestepDenominator(
                value,
            ));
        }
        Ok(self.timestep)
    }

    pub(crate) fn scaled_fraction(
        self,
        fraction_numerator: u32,
        fraction_denominator: u32,
    ) -> Result<Self, RigidBoxFreeFlightError3d> {
        let timestep = self.exact_timestep()?;
        if fraction_denominator == 0 {
            return Err(RigidBoxFreeFlightError3d::ZeroFractionDenominator);
        }
        if fraction_numerator > fraction_denominator {
            return Err(RigidBoxFreeFlightError3d::FractionOutOfRange {
                numerator: fraction_numerator,
                denominator: fraction_denominator,
            });
        }
        Ok(Self::from_exact(
            self.gravity,
            timestep
                .scaled_u32(fraction_numerator, fraction_denominator)
                .map_err(map_ratio_error)?,
        ))
    }

    #[must_use]
    pub(crate) fn timestep_is_zero(self) -> bool {
        self.invalid_numerator.is_none()
            && self.invalid_denominator.is_none()
            && self.timestep.is_zero()
    }

    #[cfg(test)]
    #[must_use]
    pub(crate) fn timestep_i128(self) -> Option<(i128, i128)> {
        if self.invalid_numerator.is_some() || self.invalid_denominator.is_some() {
            return None;
        }
        self.timestep.as_i128_pair()
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RigidBoxFreeFlightError3d {
    NegativeTimestepNumerator(i128),
    NonPositiveTimestepDenominator(i128),
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
                "rigid-box free-flight rational calculation exceeded deterministic exact-ratio capacity"
            ),
            Self::ArithmeticOverflow(body) => write!(
                formatter,
                "rigid-box free-flight arithmetic overflowed for body {}",
                body.0
            ),
            Self::Angular(error) => {
                write!(formatter, "rigid-box free-flight rotation failed: {error}")
            }
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
/// deterministic fixed-point quaternion integrator unless the box has an explicit rotation lock.
///
/// Repeated-event time and the sampled search fraction remain one exact canonical ratio internally, even
/// when the reduced numerator or denominator exceeds `i128`.
///
/// `fraction_numerator / fraction_denominator` must be within `0..=1`.
///
/// # Errors
///
/// Returns [`RigidBoxFreeFlightError3d`] for malformed time/fraction inputs or checked linear/angular
/// arithmetic overflow.
pub fn sample_rigid_box_free_flight(
    rigid_box: &RigidBox3d,
    config: RigidBoxFreeFlightConfig3d,
    fraction_numerator: u32,
    fraction_denominator: u32,
) -> Result<RigidBox3d, RigidBoxFreeFlightError3d> {
    let sample_time = config
        .scaled_fraction(fraction_numerator, fraction_denominator)?
        .exact_timestep()?;
    if sample_time.is_zero() || rigid_box.body.kind() == BodyKind::Fixed {
        return Ok(rigid_box.clone());
    }

    let mut body = rigid_box.body.clone();
    let id = body.id();
    let mut velocity = body.velocity();
    let mut position = body.position();
    for axis in 0..3 {
        let velocity_delta = sample_time
            .mul_round_i128(i128::from(config.gravity.component(axis)))
            .map_err(map_ratio_error)?;
        let next_velocity = i128::from(velocity.component(axis))
            .checked_add(velocity_delta)
            .ok_or(RigidBoxFreeFlightError3d::ArithmeticOverflow(id))?;
        let next_velocity = i32::try_from(next_velocity)
            .map_err(|_| RigidBoxFreeFlightError3d::ArithmeticOverflow(id))?;
        velocity.set_component(axis, next_velocity);

        let position_delta = sample_time
            .mul_round_i128(i128::from(next_velocity))
            .map_err(map_ratio_error)?;
        let next_position = i128::from(position.component(axis))
            .checked_add(position_delta)
            .ok_or(RigidBoxFreeFlightError3d::ArithmeticOverflow(id))?;
        let next_position = i32::try_from(next_position)
            .map_err(|_| RigidBoxFreeFlightError3d::ArithmeticOverflow(id))?;
        position.set_component(axis, next_position);
    }
    body.velocity = velocity;
    body.position = position;

    let angular = if rigid_box.rotation_locked {
        AngularState3d::new(rigid_box.angular.orientation, AngularVelocity3d::default())
    } else {
        AngularState3d::new(
            integrate_orientation_exact_ratio(
                rigid_box.angular.orientation,
                rigid_box.angular.angular_velocity,
                sample_time,
            )?,
            rigid_box.angular.angular_velocity,
        )
    };
    Ok(RigidBox3d {
        body,
        angular,
        rotation_locked: rigid_box.rotation_locked,
    })
}

/// Conservatively bounds every direct free-flight sample over the configured interval.
///
/// Per axis, the center interval uses an absolute upper bound on speed after acceleration and then on
/// displacement. Exact limb arithmetic performs multiply/divide before the final upward rounding, so a
/// large repeated-event denominator cannot manufacture an overflow. This may overproduce broad-phase
/// candidates, but it cannot lose a sampled contact when velocity reverses and the center reaches an
/// interior extremum outside the start/end interval. Arbitrary orientation is enclosed by the same
/// circumscribed-box radius as [`crate::rotational_sweep_bounds`].
///
/// # Errors
///
/// Returns [`RigidBoxFreeFlightError3d`] for malformed time configuration or checked bound arithmetic
/// overflow.
pub fn rigid_box_free_flight_sweep_bounds(
    rigid_box: &RigidBox3d,
    config: RigidBoxFreeFlightConfig3d,
) -> Result<RotationalSweepBounds3d, RigidBoxFreeFlightError3d> {
    let timestep = config.exact_timestep()?;
    let center = rigid_box.body.position();
    let center = [
        i64::from(center.x),
        i64::from(center.y),
        i64::from(center.z),
    ];
    let mut center_minimum = center;
    let mut center_maximum = center;

    if rigid_box.body.kind() == BodyKind::Dynamic && !timestep.is_zero() {
        for axis in 0..3 {
            let displacement = conservative_axis_displacement(
                rigid_box.body.velocity().component(axis),
                config.gravity.component(axis),
                timestep,
            )?;
            let displacement = i64::try_from(displacement)
                .map_err(|_| RigidBoxFreeFlightError3d::ArithmeticOverflow(rigid_box.body.id()))?;
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

fn conservative_axis_displacement(
    velocity: i32,
    acceleration: i32,
    timestep: ExactRatio,
) -> Result<u128, RigidBoxFreeFlightError3d> {
    let acceleration_delta = timestep
        .mul_ceil_u128(u128::from(acceleration.unsigned_abs()))
        .map_err(map_ratio_error)?;
    let maximum_speed = u128::from(velocity.unsigned_abs())
        .checked_add(acceleration_delta)
        .ok_or(RigidBoxFreeFlightError3d::RatioTooLarge)?;
    timestep
        .mul_ceil_u128(maximum_speed)
        .map_err(map_ratio_error)
}

fn map_ratio_error(_: WideRatioError) -> RigidBoxFreeFlightError3d {
    RigidBoxFreeFlightError3d::RatioTooLarge
}

#[cfg(test)]
mod tests {
    use crate::{
        ANGULAR_VELOCITY_SCALE, AngularState3d, AngularVelocity3d, BodyId, Orientation3d,
        RigidBody, RigidBox3d, Vec3i, World, WorldConfig, oriented_box_vertices,
        rotational_sweep_bounds,
    };

    use super::{
        RigidBoxFreeFlightConfig3d, RigidBoxFreeFlightError3d, rigid_box_free_flight_sweep_bounds,
        sample_rigid_box_free_flight,
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
    fn rotation_locked_box_translates_without_changing_orientation() {
        let rigid_box = rotating_box(
            RigidBody::dynamic(
                BodyId(7),
                Vec3i::ZERO,
                Vec3i::new(60, 0, 0),
                Vec3i::new(2, 3, 4),
            ),
            AngularVelocity3d::new(0, 0, ANGULAR_VELOCITY_SCALE),
        )
        .with_rotation_locked();
        let sampled = sample_rigid_box_free_flight(
            &rigid_box,
            RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 1, 60),
            1,
            1,
        )
        .expect("valid locked free flight");

        assert!(sampled.body().position().x > rigid_box.body().position().x);
        assert_eq!(
            sampled.angular().orientation,
            rigid_box.angular().orientation
        );
        assert!(sampled.angular().angular_velocity.is_zero());
        assert!(sampled.rotation_locked());
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
    fn composed_sample_ratio_can_exceed_i32_without_failing() {
        let rigid_box = rotating_box(
            RigidBody::dynamic(
                BodyId(6),
                Vec3i::ZERO,
                Vec3i::new(1, 0, 0),
                Vec3i::new(1, 1, 1),
            ),
            AngularVelocity3d::new(0, 0, ANGULAR_VELOCITY_SCALE),
        );
        let config = RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 1_000_000_007, 1_500_000_000);

        let sampled = sample_rigid_box_free_flight(&rigid_box, config, 1, 512)
            .expect("wide composed sample ratio should remain representable internally");

        assert_ne!(sampled.angular().orientation, Orientation3d::IDENTITY);
    }

    #[test]
    fn exact_timestep_can_exceed_i128_without_flattening_sample_time() {
        let rigid_box = rotating_box(
            RigidBody::dynamic(
                BodyId(9),
                Vec3i::new(10, 20, -30),
                Vec3i::new(120, -45, 11),
                Vec3i::new(2, 3, 4),
            ),
            AngularVelocity3d::new(0, 0, ANGULAR_VELOCITY_SCALE),
        );
        let mut config = RigidBoxFreeFlightConfig3d::new(Vec3i::new(0, -60, 0), 1, 1);
        for _ in 0..20 {
            config = config
                .scaled_fraction(511, 512)
                .expect("bounded exact timestep growth");
        }
        assert!(config.timestep_i128().is_none());

        let first = sample_rigid_box_free_flight(&rigid_box, config, 1, 512)
            .expect("sampled exact timestep remains representable");
        let second =
            sample_rigid_box_free_flight(&rigid_box, config, 1, 512).expect("same exact sample");
        assert_eq!(first, second);
        assert_ne!(first.angular().orientation, Orientation3d::IDENTITY);
    }

    #[test]
    fn wide_conservative_sweep_avoids_intermediate_overflow() {
        let rigid_box = rotating_box(
            RigidBody::dynamic(
                BodyId(8),
                Vec3i::ZERO,
                Vec3i::new(1, 0, 0),
                Vec3i::new(1, 1, 1),
            ),
            AngularVelocity3d::default(),
        );
        let mut config = RigidBoxFreeFlightConfig3d::new(Vec3i::new(0, -3_600, 0), 1, 1);
        for _ in 0..20 {
            config = config
                .scaled_fraction(511, 512)
                .expect("bounded exact timestep growth");
        }
        assert!(config.timestep_i128().is_none());

        let bounds = rigid_box_free_flight_sweep_bounds(&rigid_box, config)
            .expect("wide conservative bound remains exact");
        assert!(bounds.minimum[1] < 0);
        assert!(bounds.maximum[1] > 0);
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
        let end =
            sample_rigid_box_free_flight(&rigid_box, config, 1, 1).expect("valid endpoint sample");
        let endpoint_only = rotational_sweep_bounds(rigid_box.oriented_box(), end.oriented_box())
            .expect("valid endpoint sweep");
        let quarter =
            sample_rigid_box_free_flight(&rigid_box, config, 1, 4).expect("valid interior sample");
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
            RigidBody::dynamic(BodyId(5), Vec3i::ZERO, Vec3i::ZERO, Vec3i::new(1, 1, 1)),
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
