use std::{error::Error, fmt};

use crate::{BodyId, BodyKind, RigidBody, wide_ratio::mul_div_round_i128_wide_denominator};

/// Fixed-point quaternion scale. `1 << 30` represents one unit.
pub const ORIENTATION_SCALE: i32 = 1_i32 << 30;
/// Angular velocity units per radian per canonical second.
pub const ANGULAR_VELOCITY_SCALE: i32 = 1_000_000;

/// Deterministic fixed-point quaternion orientation.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Orientation3d {
    pub x: i32,
    pub y: i32,
    pub z: i32,
    pub w: i32,
}

impl Orientation3d {
    pub const IDENTITY: Self = Self {
        x: 0,
        y: 0,
        z: 0,
        w: ORIENTATION_SCALE,
    };

    #[must_use]
    pub const fn new(x: i32, y: i32, z: i32, w: i32) -> Self {
        Self { x, y, z, w }
    }

    /// Renormalizes the quaternion with integer arithmetic.
    ///
    /// Components are rounded to nearest with half values away from zero, keeping authoritative
    /// orientation state independent of platform floating-point behavior.
    ///
    /// # Errors
    ///
    /// Returns [`AngularError3d::ZeroOrientation`] when every component is zero, or
    /// [`AngularError3d::ArithmeticOverflow`] when the normalized result cannot be represented.
    pub fn normalized(self) -> Result<Self, AngularError3d> {
        let norm_squared = orientation_norm_squared(self);
        if norm_squared == 0 {
            return Err(AngularError3d::ZeroOrientation);
        }
        let norm = integer_sqrt(norm_squared);
        let denominator = i128::try_from(norm).map_err(|_| AngularError3d::ArithmeticOverflow)?;
        let scale = i128::from(ORIENTATION_SCALE);
        Ok(Self {
            x: normalized_component(self.x, scale, denominator)?,
            y: normalized_component(self.y, scale, denominator)?,
            z: normalized_component(self.z, scale, denominator)?,
            w: normalized_component(self.w, scale, denominator)?,
        })
    }
}

impl Default for Orientation3d {
    fn default() -> Self {
        Self::IDENTITY
    }
}

/// Deterministic world-space angular velocity.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct AngularVelocity3d {
    pub x: i32,
    pub y: i32,
    pub z: i32,
}

impl AngularVelocity3d {
    #[must_use]
    pub const fn new(x: i32, y: i32, z: i32) -> Self {
        Self { x, y, z }
    }

    #[must_use]
    pub const fn is_zero(self) -> bool {
        self.x == 0 && self.y == 0 && self.z == 0
    }
}

/// Rotational pose and velocity owned by the physics engine rather than a consumer ECS.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct AngularState3d {
    pub orientation: Orientation3d,
    pub angular_velocity: AngularVelocity3d,
}

impl AngularState3d {
    #[must_use]
    pub const fn new(orientation: Orientation3d, angular_velocity: AngularVelocity3d) -> Self {
        Self {
            orientation,
            angular_velocity,
        }
    }

    /// Advances only rotational pose over an explicit rational timestep.
    ///
    /// This does not apply collision response or damping. Those operations remain separate so a
    /// later rigid-body solver can own event ordering explicitly.
    ///
    /// # Errors
    ///
    /// Returns [`AngularError3d`] for malformed timesteps, zero orientation, or arithmetic overflow.
    pub fn integrated(
        self,
        timestep_numerator: i32,
        timestep_denominator: i32,
    ) -> Result<Self, AngularError3d> {
        Ok(Self {
            orientation: integrate_orientation(
                self.orientation,
                self.angular_velocity,
                timestep_numerator,
                timestep_denominator,
            )?,
            angular_velocity: self.angular_velocity,
        })
    }
}

/// Exact principal-axis inertia ratio for a box.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct BoxInertia3d {
    /// Principal-axis inertia numerators. Divide by [`Self::denominator`] for physical inertia in
    /// engine mass × coordinate² units.
    pub principal_numerators: [u128; 3],
    pub denominator: u32,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AngularError3d {
    ZeroOrientation,
    NegativeTimestepNumerator(i32),
    NonPositiveTimestepDenominator(i32),
    InvalidHalfExtents(BodyId),
    ZeroMass(BodyId),
    ArithmeticOverflow,
}

impl fmt::Display for AngularError3d {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::ZeroOrientation => write!(formatter, "3D orientation quaternion cannot be zero"),
            Self::NegativeTimestepNumerator(value) => write!(
                formatter,
                "3D angular timestep numerator must be non-negative, got {value}"
            ),
            Self::NonPositiveTimestepDenominator(value) => write!(
                formatter,
                "3D angular timestep denominator must be positive, got {value}"
            ),
            Self::InvalidHalfExtents(body) => write!(
                formatter,
                "3D angular inertia body {} has negative half extents",
                body.0
            ),
            Self::ZeroMass(body) => write!(
                formatter,
                "dynamic 3D angular inertia body {} has zero mass",
                body.0
            ),
            Self::ArithmeticOverflow => write!(formatter, "3D angular calculation overflowed"),
        }
    }
}

impl Error for AngularError3d {}

/// Integrates a fixed-point quaternion with world-space angular velocity over an explicit rational step.
///
/// Angular velocity is expressed in [`ANGULAR_VELOCITY_SCALE`] units per radian per canonical second.
/// The integrator uses the quaternion derivative `0.5 * omega * q`, then deterministically renormalizes
/// the result. Supplying `1 / 60` therefore advances one authoritative 60 Hz rotational frame.
///
/// # Errors
///
/// Returns [`AngularError3d`] for malformed timesteps, zero orientation, or arithmetic overflow.
pub fn integrate_orientation(
    orientation: Orientation3d,
    angular_velocity: AngularVelocity3d,
    timestep_numerator: i32,
    timestep_denominator: i32,
) -> Result<Orientation3d, AngularError3d> {
    if timestep_numerator < 0 {
        return Err(AngularError3d::NegativeTimestepNumerator(
            timestep_numerator,
        ));
    }
    if timestep_denominator <= 0 {
        return Err(AngularError3d::NonPositiveTimestepDenominator(
            timestep_denominator,
        ));
    }

    integrate_orientation_ratio(
        orientation,
        angular_velocity,
        u128::from(timestep_numerator.unsigned_abs()),
        u128::from(timestep_denominator.unsigned_abs()),
    )
}

/// Internal wide-rational counterpart to [`integrate_orientation`].
///
/// Free-flight sampling can carry an exact repeated-event ratio whose denominator legitimately exceeds
/// `i32`. The quaternion derivative also divides by the fixed angular-velocity scale. Those two
/// denominator factors are kept separate and divided in 256-bit space, so multiplying them no longer
/// creates an artificial `i128` overflow while the physical ratio itself remains representable.
pub(crate) fn integrate_orientation_ratio(
    orientation: Orientation3d,
    angular_velocity: AngularVelocity3d,
    timestep_numerator: u128,
    timestep_denominator: u128,
) -> Result<Orientation3d, AngularError3d> {
    let timestep_numerator =
        i128::try_from(timestep_numerator).map_err(|_| AngularError3d::ArithmeticOverflow)?;
    let timestep_denominator =
        i128::try_from(timestep_denominator).map_err(|_| AngularError3d::ArithmeticOverflow)?;
    integrate_orientation_ratio_factors(
        orientation,
        angular_velocity,
        timestep_numerator,
        timestep_denominator,
        1,
        1,
    )
}

/// Integrates orientation over an exact timestep ratio kept as two numerator/denominator factors.
///
/// Repeated-event time can already occupy most of the internal `i128` range. Sampled rotational CCD
/// composes that interval with a bounded search fraction. Keeping the search fraction separate avoids
/// manufacturing an overflow merely by flattening two individually representable factors while still
/// evaluating the exact same rational timestep.
pub(crate) fn integrate_orientation_ratio_factors(
    orientation: Orientation3d,
    angular_velocity: AngularVelocity3d,
    timestep_numerator: i128,
    timestep_denominator: i128,
    fraction_numerator: i128,
    fraction_denominator: i128,
) -> Result<Orientation3d, AngularError3d> {
    if timestep_numerator < 0
        || fraction_numerator < 0
        || timestep_denominator <= 0
        || fraction_denominator <= 0
    {
        return Err(AngularError3d::ArithmeticOverflow);
    }

    let orientation = orientation.normalized()?;
    if timestep_numerator == 0 || fraction_numerator == 0 || angular_velocity.is_zero() {
        return Ok(orientation);
    }

    let qx = i128::from(orientation.x);
    let qy = i128::from(orientation.y);
    let qz = i128::from(orientation.z);
    let qw = i128::from(orientation.w);
    let wx = i128::from(angular_velocity.x);
    let wy = i128::from(angular_velocity.y);
    let wz = i128::from(angular_velocity.z);

    let derivative = [
        checked_sum([wx * qw, wy * qz, -(wz * qy)])?,
        checked_sum([wy * qw, wz * qx, -(wx * qz)])?,
        checked_sum([wz * qw, wx * qy, -(wy * qx)])?,
        checked_sum([-(wx * qx), -(wy * qy), -(wz * qz)])?,
    ];
    let derivative = derivative.map(|value| {
        value
            .checked_mul(fraction_numerator)
            .ok_or(AngularError3d::ArithmeticOverflow)
    });
    let derivative = [
        derivative[0]?,
        derivative[1]?,
        derivative[2]?,
        derivative[3]?,
    ];
    let unit_denominator = i128::from(2_i32)
        .checked_mul(i128::from(ANGULAR_VELOCITY_SCALE))
        .and_then(|value| value.checked_mul(fraction_denominator))
        .ok_or(AngularError3d::ArithmeticOverflow)?;

    let next = Orientation3d::new(
        integrated_component_wide(
            qx,
            derivative[0],
            timestep_numerator,
            unit_denominator,
            timestep_denominator,
        )?,
        integrated_component_wide(
            qy,
            derivative[1],
            timestep_numerator,
            unit_denominator,
            timestep_denominator,
        )?,
        integrated_component_wide(
            qz,
            derivative[2],
            timestep_numerator,
            unit_denominator,
            timestep_denominator,
        )?,
        integrated_component_wide(
            qw,
            derivative[3],
            timestep_numerator,
            unit_denominator,
            timestep_denominator,
        )?,
    );
    next.normalized()
}

/// Computes the exact principal-axis inertia ratio for an axis-aligned box in its local frame.
///
/// For half extents `h`, the full-width cuboid formula is `I_x = m(h_y² + h_z²) / 3`, with
/// equivalent permutations for Y and Z. Fixed bodies report zero inertia because their inverse
/// inertia is zero for response purposes.
///
/// # Errors
///
/// Returns [`AngularError3d`] for invalid dimensions, zero dynamic mass, or arithmetic overflow.
pub fn box_inertia(body: &RigidBody) -> Result<BoxInertia3d, AngularError3d> {
    let half_extents = body.half_extents();
    if half_extents.x < 0 || half_extents.y < 0 || half_extents.z < 0 {
        return Err(AngularError3d::InvalidHalfExtents(body.id()));
    }
    if body.kind() == BodyKind::Fixed {
        return Ok(BoxInertia3d {
            principal_numerators: [0; 3],
            denominator: 1,
        });
    }
    if body.mass_units() == 0 {
        return Err(AngularError3d::ZeroMass(body.id()));
    }

    let half = [
        u128::from(half_extents.x.unsigned_abs()),
        u128::from(half_extents.y.unsigned_abs()),
        u128::from(half_extents.z.unsigned_abs()),
    ];
    let squared = half.map(|value| value * value);
    let mass = u128::from(body.mass_units());
    let x = mass
        .checked_mul(
            squared[1]
                .checked_add(squared[2])
                .ok_or(AngularError3d::ArithmeticOverflow)?,
        )
        .ok_or(AngularError3d::ArithmeticOverflow)?;
    let y = mass
        .checked_mul(
            squared[0]
                .checked_add(squared[2])
                .ok_or(AngularError3d::ArithmeticOverflow)?,
        )
        .ok_or(AngularError3d::ArithmeticOverflow)?;
    let z = mass
        .checked_mul(
            squared[0]
                .checked_add(squared[1])
                .ok_or(AngularError3d::ArithmeticOverflow)?,
        )
        .ok_or(AngularError3d::ArithmeticOverflow)?;

    Ok(BoxInertia3d {
        principal_numerators: [x, y, z],
        denominator: 3,
    })
}

/// Computes the angular impulse `r × J` generated by a linear impulse applied away from the center of mass.
///
/// # Errors
///
/// Returns [`AngularError3d::ArithmeticOverflow`] when the exact integer cross product cannot be represented.
pub fn contact_angular_impulse(
    center_to_contact: [i64; 3],
    linear_impulse: [i64; 3],
) -> Result<[i128; 3], AngularError3d> {
    let r = center_to_contact.map(i128::from);
    let impulse = linear_impulse.map(i128::from);
    Ok([
        checked_cross_component(r[1], impulse[2], r[2], impulse[1])?,
        checked_cross_component(r[2], impulse[0], r[0], impulse[2])?,
        checked_cross_component(r[0], impulse[1], r[1], impulse[0])?,
    ])
}

fn integrated_component_wide(
    component: i128,
    derivative: i128,
    numerator: i128,
    unit_denominator: i128,
    timestep_denominator: i128,
) -> Result<i32, AngularError3d> {
    let delta = mul_div_round_i128_wide_denominator(
        derivative,
        numerator,
        unit_denominator,
        timestep_denominator,
    )
    .map_err(|_| AngularError3d::ArithmeticOverflow)?;
    let next = component
        .checked_add(delta)
        .ok_or(AngularError3d::ArithmeticOverflow)?;
    i32::try_from(next).map_err(|_| AngularError3d::ArithmeticOverflow)
}

fn normalized_component(component: i32, scale: i128, norm: i128) -> Result<i32, AngularError3d> {
    let numerator = i128::from(component)
        .checked_mul(scale)
        .ok_or(AngularError3d::ArithmeticOverflow)?;
    let normalized = div_round_nearest(numerator, norm)?;
    i32::try_from(normalized).map_err(|_| AngularError3d::ArithmeticOverflow)
}

fn div_round_nearest(numerator: i128, denominator: i128) -> Result<i128, AngularError3d> {
    if denominator <= 0 {
        return Err(AngularError3d::ArithmeticOverflow);
    }
    let half = denominator / 2;
    let adjusted = if numerator >= 0 {
        numerator.checked_add(half)
    } else {
        numerator.checked_sub(half)
    }
    .ok_or(AngularError3d::ArithmeticOverflow)?;
    Ok(adjusted / denominator)
}

fn checked_sum(values: [i128; 3]) -> Result<i128, AngularError3d> {
    values.into_iter().try_fold(0_i128, |sum, value| {
        sum.checked_add(value)
            .ok_or(AngularError3d::ArithmeticOverflow)
    })
}

fn checked_cross_component(
    left_a: i128,
    right_a: i128,
    left_b: i128,
    right_b: i128,
) -> Result<i128, AngularError3d> {
    left_a
        .checked_mul(right_a)
        .and_then(|value| {
            left_b
                .checked_mul(right_b)
                .and_then(|other| value.checked_sub(other))
        })
        .ok_or(AngularError3d::ArithmeticOverflow)
}

fn orientation_norm_squared(orientation: Orientation3d) -> u128 {
    [orientation.x, orientation.y, orientation.z, orientation.w]
        .into_iter()
        .map(|component| {
            let component = u128::from(component.unsigned_abs());
            component * component
        })
        .sum()
}

fn integer_sqrt(value: u128) -> u128 {
    if value < 2 {
        return value;
    }
    let bit_length = u128::BITS - value.leading_zeros();
    let mut estimate = 1_u128 << bit_length.div_ceil(2);
    loop {
        let next = u128::midpoint(estimate, value / estimate);
        if next >= estimate {
            return estimate;
        }
        estimate = next;
    }
}

#[cfg(test)]
mod tests {
    use super::{
        ANGULAR_VELOCITY_SCALE, AngularError3d, AngularState3d, AngularVelocity3d,
        ORIENTATION_SCALE, Orientation3d, box_inertia, contact_angular_impulse,
        integrate_orientation, integrate_orientation_ratio, integrate_orientation_ratio_factors,
        orientation_norm_squared,
    };
    use crate::{BodyId, RigidBody, Vec3i};

    #[test]
    fn zero_spin_is_exactly_idempotent() {
        let state = AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default());
        assert_eq!(state.integrated(1, 60).expect("valid step"), state);
    }

    #[test]
    fn sixty_hz_spin_changes_orientation_and_stays_normalized() {
        let next = integrate_orientation(
            Orientation3d::IDENTITY,
            AngularVelocity3d::new(0, 0, ANGULAR_VELOCITY_SCALE),
            1,
            60,
        )
        .expect("valid angular integration");
        assert!(next.z > 0);
        assert!(next.w > 0);

        let expected = u128::from(u32::try_from(ORIENTATION_SCALE).expect("positive scale"));
        let expected_squared = expected * expected;
        let error = orientation_norm_squared(next).abs_diff(expected_squared);
        assert!(error <= expected * 4, "normalization drift was {error}");
    }

    #[test]
    fn wide_ratio_matches_public_integrator_when_both_are_representable() {
        let angular_velocity = AngularVelocity3d::new(0, 0, ANGULAR_VELOCITY_SCALE);
        assert_eq!(
            integrate_orientation_ratio(Orientation3d::IDENTITY, angular_velocity, 1, 60)
                .expect("valid wide ratio"),
            integrate_orientation(Orientation3d::IDENTITY, angular_velocity, 1, 60)
                .expect("valid public ratio")
        );
    }

    #[test]
    fn exact_large_denominator_ratio_matches_equivalent_public_step() {
        let angular_velocity = AngularVelocity3d::new(0, 0, ANGULAR_VELOCITY_SCALE);
        let numerator = 100_000_000_000_000_000_000_000_000_000_000_u128;
        let denominator = numerator.checked_mul(60).expect("test denominator");
        assert_eq!(
            integrate_orientation_ratio(
                Orientation3d::IDENTITY,
                angular_velocity,
                numerator,
                denominator,
            )
            .expect("wide denominator product remains representable"),
            integrate_orientation(Orientation3d::IDENTITY, angular_velocity, 1, 60)
                .expect("equivalent public ratio")
        );
    }

    #[test]
    fn factored_ratio_matches_equivalent_public_step_without_flattening() {
        let angular_velocity = AngularVelocity3d::new(0, 0, ANGULAR_VELOCITY_SCALE);
        let factor = i128::MAX / 4;
        assert_eq!(
            integrate_orientation_ratio_factors(
                Orientation3d::IDENTITY,
                angular_velocity,
                factor,
                factor.checked_mul(3).expect("test denominator"),
                1,
                2,
            )
            .expect("factored exact ratio"),
            integrate_orientation(Orientation3d::IDENTITY, angular_velocity, 1, 6)
                .expect("equivalent public ratio")
        );
    }

    #[test]
    fn malformed_timesteps_fail_closed() {
        assert_eq!(
            integrate_orientation(
                Orientation3d::IDENTITY,
                AngularVelocity3d::default(),
                -1,
                60,
            ),
            Err(AngularError3d::NegativeTimestepNumerator(-1))
        );
        assert_eq!(
            integrate_orientation(Orientation3d::IDENTITY, AngularVelocity3d::default(), 1, 0,),
            Err(AngularError3d::NonPositiveTimestepDenominator(0))
        );
    }

    #[test]
    fn box_inertia_uses_full_cuboid_formula() {
        let body = RigidBody::dynamic(BodyId(7), Vec3i::ZERO, Vec3i::ZERO, Vec3i::new(2, 3, 4))
            .with_mass(6);

        assert_eq!(
            box_inertia(&body)
                .expect("valid box inertia")
                .principal_numerators,
            [150, 120, 78]
        );
        assert_eq!(
            box_inertia(&body).expect("valid box inertia").denominator,
            3
        );
    }

    #[test]
    fn fixed_body_has_zero_inverse_inertia_semantics() {
        let body = RigidBody::fixed(BodyId(8), Vec3i::ZERO, Vec3i::new(2, 3, 4));
        let inertia = box_inertia(&body).expect("fixed inertia is valid");

        assert_eq!(inertia.principal_numerators, [0; 3]);
        assert_eq!(inertia.denominator, 1);
    }

    #[test]
    fn dynamic_zero_mass_fails_closed() {
        let body = RigidBody::dynamic(BodyId(9), Vec3i::ZERO, Vec3i::ZERO, Vec3i::new(1, 1, 1))
            .with_mass(0);

        assert_eq!(box_inertia(&body), Err(AngularError3d::ZeroMass(BodyId(9))));
    }

    #[test]
    fn off_center_linear_impulse_produces_exact_torque_impulse() {
        assert_eq!(
            contact_angular_impulse([0, 2, 0], [3, 0, 0]).expect("representable cross product"),
            [0, 0, -6]
        );
    }

    #[test]
    fn zero_orientation_is_rejected() {
        assert_eq!(
            Orientation3d::new(0, 0, 0, 0).normalized(),
            Err(AngularError3d::ZeroOrientation)
        );
    }
}
