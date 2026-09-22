use std::{collections::BTreeSet, error::Error, fmt};

use crate::{
    ANGULAR_VELOCITY_SCALE, AngularError3d, AngularVelocity3d, BallisticSphere3d,
    BallisticSphereError3d, BallisticSphereQueryStats3d, BallisticSphereScene3d,
    BallisticSphereSweepHit3d, BodyId, BodyKind, MATERIAL_SCALE, ORIENTATION_SCALE, Orientation3d,
    RigidBox3d, RigidBoxFreeFlightConfig3d, RigidBoxFreeFlightError3d, SampledContactTime3d, Vec3i,
    box_inertia, sample_rigid_box_free_flight,
    wide_ratio::{WideRatioError, mul_div_round_i128, mul_div_round_u128},
};

const BALLISTIC_TIME_SCALE: u64 = 1_u64 << 32;
const TIMELINE_TIME_SCALE: u32 = 1_u32 << 31;
const RESPONSE_SCALE: i128 = 1_i128 << 50;

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub(crate) struct BallisticStepWork3d {
    pub query_rounds: u64,
    pub target_bound_checks: u64,
    pub broad_phase_candidates: u64,
    pub toi_tests: u64,
    pub feature_tests: u64,
    pub motion_samples: u64,
    pub impacts: u64,
    pub retired: u64,
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct BallisticCandidate3d {
    pub projectile: BodyId,
    pub hit: BallisticSphereSweepHit3d,
}

#[derive(Clone, Debug)]
pub(crate) struct BallisticFrontier3d {
    pub time: SampledContactTime3d,
    pub hits: Vec<BallisticCandidate3d>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BallisticTimelineError3d {
    DuplicateBody(BodyId),
    MissingProjectile(BodyId),
    MissingTarget(BodyId),
    FreeFlight(RigidBoxFreeFlightError3d),
    Ballistic(BallisticSphereError3d),
    Angular(AngularError3d),
    ArithmeticOverflow,
}

impl fmt::Display for BallisticTimelineError3d {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::DuplicateBody(id) => {
                write!(
                    formatter,
                    "ballistic timeline contains duplicate body id {}",
                    id.0
                )
            }
            Self::MissingProjectile(id) => {
                write!(formatter, "ballistic projectile {} is missing", id.0)
            }
            Self::MissingTarget(id) => {
                write!(formatter, "ballistic impact target {} is missing", id.0)
            }
            Self::FreeFlight(error) => write!(formatter, "ballistic free flight failed: {error}"),
            Self::Ballistic(error) => write!(formatter, "ballistic sphere query failed: {error}"),
            Self::Angular(error) => write!(formatter, "ballistic angular response failed: {error}"),
            Self::ArithmeticOverflow => {
                write!(formatter, "ballistic timeline arithmetic overflowed")
            }
        }
    }
}

impl Error for BallisticTimelineError3d {}

impl From<RigidBoxFreeFlightError3d> for BallisticTimelineError3d {
    fn from(value: RigidBoxFreeFlightError3d) -> Self {
        Self::FreeFlight(value)
    }
}

impl From<BallisticSphereError3d> for BallisticTimelineError3d {
    fn from(value: BallisticSphereError3d) -> Self {
        Self::Ballistic(value)
    }
}

impl From<AngularError3d> for BallisticTimelineError3d {
    fn from(value: AngularError3d) -> Self {
        Self::Angular(value)
    }
}

impl From<WideRatioError> for BallisticTimelineError3d {
    fn from(_: WideRatioError) -> Self {
        Self::ArithmeticOverflow
    }
}

pub(crate) fn validate_unique_ballistic_ids(
    boxes: &[RigidBox3d],
    projectiles: &[BallisticSphere3d],
) -> Result<(), BallisticTimelineError3d> {
    let mut ids = BTreeSet::new();
    for id in boxes
        .iter()
        .map(|rigid_box| rigid_box.body().id())
        .chain(projectiles.iter().copied().map(BallisticSphere3d::id))
    {
        if !ids.insert(id) {
            return Err(BallisticTimelineError3d::DuplicateBody(id));
        }
    }
    Ok(())
}

pub(crate) fn earliest_ballistic_frontier(
    boxes: &[RigidBox3d],
    projectiles: &[BallisticSphere3d],
    remaining: RigidBoxFreeFlightConfig3d,
    resolved_pairs: &BTreeSet<(BodyId, BodyId)>,
    work: &mut BallisticStepWork3d,
) -> Result<Option<BallisticFrontier3d>, BallisticTimelineError3d> {
    if projectiles.is_empty() || remaining.timestep_is_zero() {
        return Ok(None);
    }
    work.query_rounds = work.query_rounds.saturating_add(1);

    let prepared_targets = projected_targets(boxes, remaining)?;
    let scene = BallisticSphereScene3d::prepare(prepared_targets.iter())?;
    let step = scene.prepare_step(1, 1)?;
    work.target_bound_checks = work.target_bound_checks.saturating_add(
        u64::try_from(scene.target_count().saturating_mul(projectiles.len())).unwrap_or(u64::MAX),
    );

    let mut earliest_time = None;
    let mut candidates = Vec::new();
    for projectile in projectiles.iter().copied() {
        let query = projected_projectile(projectile, remaining)?;
        let mut stats = BallisticSphereQueryStats3d::default();
        let hit = step.earliest_hit_excluding_pairs(query, resolved_pairs, &mut stats)?;
        accumulate_query_stats(work, stats);
        let Some(hit) = hit else {
            continue;
        };
        let time = ballistic_time_to_sampled(hit.time.fraction_subticks())?;
        match earliest_time {
            None => {
                earliest_time = Some(time);
                candidates.clear();
                candidates.push(BallisticCandidate3d {
                    projectile: projectile.id(),
                    hit,
                });
            }
            Some(current) if compare_time(time, current).is_lt() => {
                earliest_time = Some(time);
                candidates.clear();
                candidates.push(BallisticCandidate3d {
                    projectile: projectile.id(),
                    hit,
                });
            }
            Some(current) if compare_time(time, current).is_eq() => {
                candidates.push(BallisticCandidate3d {
                    projectile: projectile.id(),
                    hit,
                });
            }
            Some(_) => {}
        }
    }

    let Some(time) = earliest_time else {
        return Ok(None);
    };
    candidates.sort_by_key(|candidate| (candidate.projectile, candidate.hit.body));
    Ok(Some(BallisticFrontier3d {
        time,
        hits: candidates,
    }))
}

pub(crate) fn advance_ballistic_spheres_to_time(
    projectiles: &mut [BallisticSphere3d],
    remaining: RigidBoxFreeFlightConfig3d,
    time: SampledContactTime3d,
    work: &mut BallisticStepWork3d,
) -> Result<(), BallisticTimelineError3d> {
    if time == SampledContactTime3d::ZERO || projectiles.is_empty() {
        return Ok(());
    }
    let segment = remaining.scaled_fraction(time.numerator, time.denominator)?;
    for projectile in projectiles {
        work.motion_samples = work.motion_samples.saturating_add(1);
        advance_projectile_exact(projectile, segment)?;
    }
    Ok(())
}

pub(crate) fn advance_ballistic_spheres_full(
    projectiles: &mut [BallisticSphere3d],
    remaining: RigidBoxFreeFlightConfig3d,
    work: &mut BallisticStepWork3d,
) -> Result<(), BallisticTimelineError3d> {
    advance_ballistic_spheres_to_time(
        projectiles,
        remaining,
        SampledContactTime3d {
            numerator: 1,
            denominator: 1,
        },
        work,
    )
}

pub(crate) fn resolve_ballistic_frontier(
    boxes: &mut [RigidBox3d],
    projectiles: &mut Vec<BallisticSphere3d>,
    retire_on_contact: &BTreeSet<BodyId>,
    frontier: &BallisticFrontier3d,
    work: &mut BallisticStepWork3d,
) -> Result<Vec<BodyId>, BallisticTimelineError3d> {
    let mut modified_targets = BTreeSet::new();
    let mut retired = BTreeSet::new();

    for candidate in &frontier.hits {
        let projectile_index = projectiles
            .iter()
            .position(|projectile| projectile.id() == candidate.projectile)
            .ok_or(BallisticTimelineError3d::MissingProjectile(
                candidate.projectile,
            ))?;
        let target_index = boxes
            .iter()
            .position(|rigid_box| rigid_box.body().id() == candidate.hit.body)
            .ok_or(BallisticTimelineError3d::MissingTarget(candidate.hit.body))?;

        let projectile = projectiles[projectile_index];
        let impulse =
            apply_ballistic_target_impulse(projectile, &mut boxes[target_index], candidate.hit)?;
        if impulse != [0; 3] {
            modified_targets.insert(candidate.hit.body);
            if !retire_on_contact.contains(&candidate.projectile) {
                apply_projectile_impulse(&mut projectiles[projectile_index], impulse)?;
            }
        }
        work.impacts = work.impacts.saturating_add(1);
        if retire_on_contact.contains(&candidate.projectile) {
            retired.insert(candidate.projectile);
        }
    }

    if !retired.is_empty() {
        projectiles.retain(|projectile| !retired.contains(&projectile.id()));
        work.retired = work
            .retired
            .saturating_add(u64::try_from(retired.len()).unwrap_or(u64::MAX));
    }

    Ok(modified_targets.into_iter().collect())
}

pub(crate) fn remaining_after(
    remaining: RigidBoxFreeFlightConfig3d,
    time: SampledContactTime3d,
) -> Result<RigidBoxFreeFlightConfig3d, BallisticTimelineError3d> {
    if time.denominator == 0 || time.numerator > time.denominator {
        return Err(BallisticTimelineError3d::ArithmeticOverflow);
    }
    Ok(remaining.scaled_fraction(time.denominator - time.numerator, time.denominator)?)
}

pub(crate) fn compare_time(
    left: SampledContactTime3d,
    right: SampledContactTime3d,
) -> std::cmp::Ordering {
    (u128::from(left.numerator) * u128::from(right.denominator))
        .cmp(&(u128::from(right.numerator) * u128::from(left.denominator)))
}

fn projected_targets(
    boxes: &[RigidBox3d],
    remaining: RigidBoxFreeFlightConfig3d,
) -> Result<Vec<RigidBox3d>, BallisticTimelineError3d> {
    boxes
        .iter()
        .map(|rigid_box| {
            let end = sample_rigid_box_free_flight(rigid_box, remaining, 1, 1)?;
            let mut projected = rigid_box.clone();
            projected.body.velocity =
                displacement_velocity(rigid_box.body.position, end.body.position)?;
            Ok(projected)
        })
        .collect()
}

fn projected_projectile(
    projectile: BallisticSphere3d,
    remaining: RigidBoxFreeFlightConfig3d,
) -> Result<BallisticSphere3d, BallisticTimelineError3d> {
    let mut end = projectile;
    advance_projectile_exact(&mut end, remaining)?;
    let mut projected = projectile;
    projected.set_velocity(displacement_velocity(
        projectile.position(),
        end.position(),
    )?);
    Ok(projected)
}

fn displacement_velocity(start: Vec3i, end: Vec3i) -> Result<Vec3i, BallisticTimelineError3d> {
    Ok(Vec3i::new(
        difference_i32(end.x, start.x)?,
        difference_i32(end.y, start.y)?,
        difference_i32(end.z, start.z)?,
    ))
}

fn difference_i32(left: i32, right: i32) -> Result<i32, BallisticTimelineError3d> {
    i32::try_from(i64::from(left) - i64::from(right))
        .map_err(|_| BallisticTimelineError3d::ArithmeticOverflow)
}

fn advance_projectile_exact(
    projectile: &mut BallisticSphere3d,
    free_flight: RigidBoxFreeFlightConfig3d,
) -> Result<(), BallisticTimelineError3d> {
    let timestep = free_flight.exact_timestep()?;
    if timestep.is_zero() {
        return Ok(());
    }
    let mut velocity = projectile.velocity();
    let mut position = projectile.position();
    for axis in 0..3 {
        let next_velocity = i128::from(component(velocity, axis))
            .checked_add(timestep.mul_round_i128(i128::from(component(free_flight.gravity, axis)))?)
            .ok_or(BallisticTimelineError3d::ArithmeticOverflow)?;
        let next_velocity = i32::try_from(next_velocity)
            .map_err(|_| BallisticTimelineError3d::ArithmeticOverflow)?;
        set_component(&mut velocity, axis, next_velocity);
        let next_position = i128::from(component(position, axis))
            .checked_add(timestep.mul_round_i128(i128::from(next_velocity))?)
            .ok_or(BallisticTimelineError3d::ArithmeticOverflow)?;
        set_component(
            &mut position,
            axis,
            i32::try_from(next_position)
                .map_err(|_| BallisticTimelineError3d::ArithmeticOverflow)?,
        );
    }
    projectile.set_velocity(velocity);
    projectile.set_position(position);
    Ok(())
}

fn apply_ballistic_target_impulse(
    projectile: BallisticSphere3d,
    target: &mut RigidBox3d,
    hit: BallisticSphereSweepHit3d,
) -> Result<[i128; 3], BallisticTimelineError3d> {
    let axis = primitive_axis(hit.normal)?;
    let axis_length_squared = axis_length_squared(axis)?;
    let point = projectile_contact_point(projectile, axis)?;
    let target_offset = vector_delta(target.body.position, point)?;
    let target_velocity = contact_velocity(target, target_offset)?;
    let relative_velocity = [
        i128::from(projectile.velocity().x) - i128::from(target_velocity[0]),
        i128::from(projectile.velocity().y) - i128::from(target_velocity[1]),
        i128::from(projectile.velocity().z) - i128::from(target_velocity[2]),
    ];
    let normal_velocity = checked_dot(relative_velocity, axis)?;
    if normal_velocity >= 0 {
        return Ok([0; 3]);
    }

    let response_scale_milli = (RESPONSE_SCALE as u128)
        .checked_mul(u128::from(MATERIAL_SCALE))
        .ok_or(BallisticTimelineError3d::ArithmeticOverflow)?;
    let sphere_inverse_mass = i128::try_from(mul_div_round_u128(
        axis_length_squared,
        response_scale_milli,
        u128::from(projectile.response_mass_milli_units()),
    )?)
    .map_err(|_| BallisticTimelineError3d::ArithmeticOverflow)?;
    let target_inverse_mass =
        body_effective_inverse_mass_scaled(target, target_offset, axis, axis_length_squared)?;
    let effective_inverse_mass = sphere_inverse_mass
        .checked_add(target_inverse_mass)
        .ok_or(BallisticTimelineError3d::ArithmeticOverflow)?;
    if effective_inverse_mass <= 0 {
        return Ok([0; 3]);
    }

    let restitution = projectile
        .material()
        .restitution_milli()
        .min(target.body.material.restitution_milli());
    let closing_speed = normal_velocity
        .checked_neg()
        .ok_or(BallisticTimelineError3d::ArithmeticOverflow)?;
    let mut impulse = [0_i128; 3];
    for (output, axis_component) in impulse.iter_mut().zip(axis) {
        *output = response_impulse_component(
            closing_speed,
            restitution,
            axis_component,
            effective_inverse_mass,
        )?;
    }

    if target.body.kind != BodyKind::Fixed {
        apply_body_impulse(target, target_offset, negate_axis(impulse)?)?;
    }
    Ok(impulse)
}

fn response_impulse_component(
    closing_speed: i128,
    restitution_milli: u16,
    axis_component: i128,
    effective_inverse_mass: i128,
) -> Result<i128, BallisticTimelineError3d> {
    if axis_component == 0 {
        return Ok(0);
    }

    let material_scale = i128::from(MATERIAL_SCALE);
    let response_axis = checked_mul(axis_component, RESPONSE_SCALE)?;
    let divisor = gcd_u128(u128::from(MATERIAL_SCALE), response_axis.unsigned_abs()).max(1);
    let divisor =
        i128::try_from(divisor).map_err(|_| BallisticTimelineError3d::ArithmeticOverflow)?;
    let reduced_material_scale = material_scale / divisor;
    let reduced_response_axis = response_axis / divisor;
    let denominator = checked_mul(reduced_material_scale, effective_inverse_mass)?;
    let numerator = checked_mul(
        closing_speed,
        material_scale
            .checked_add(i128::from(restitution_milli))
            .ok_or(BallisticTimelineError3d::ArithmeticOverflow)?,
    )?;

    Ok(mul_div_round_i128(
        numerator,
        reduced_response_axis,
        denominator,
    )?)
}

fn apply_projectile_impulse(
    projectile: &mut BallisticSphere3d,
    impulse: [i128; 3],
) -> Result<(), BallisticTimelineError3d> {
    let mass_milli = i128::from(projectile.response_mass_milli_units());
    let velocity = projectile.velocity();
    projectile.set_velocity(Vec3i::new(
        add_impulse_axis_milli(velocity.x, impulse[0], mass_milli)?,
        add_impulse_axis_milli(velocity.y, impulse[1], mass_milli)?,
        add_impulse_axis_milli(velocity.z, impulse[2], mass_milli)?,
    ));
    Ok(())
}

fn body_effective_inverse_mass_scaled(
    rigid_box: &RigidBox3d,
    contact_offset: [i64; 3],
    axis: [i128; 3],
    axis_length_squared: u128,
) -> Result<i128, BallisticTimelineError3d> {
    if rigid_box.body.kind == BodyKind::Fixed {
        return Ok(0);
    }
    let translational = i128::try_from(mul_div_round_u128(
        axis_length_squared,
        RESPONSE_SCALE as u128,
        u128::from(rigid_box.body.mass_units),
    )?)
    .map_err(|_| BallisticTimelineError3d::ArithmeticOverflow)?;
    if rigid_box.rotation_locked {
        return Ok(translational);
    }
    let angular_impulse = cross_i64_i128(contact_offset, axis)?;
    let local = rotate_inverse(rigid_box.angular.orientation, angular_impulse)?;
    let inertia = box_inertia(&rigid_box.body)?;
    let scale = (RESPONSE_SCALE as u128)
        .checked_mul(u128::from(inertia.denominator))
        .ok_or(BallisticTimelineError3d::ArithmeticOverflow)?;
    let mut rotational = 0_i128;
    for (index, value) in local.into_iter().enumerate() {
        let squared = checked_mul(value, value)?;
        let term = i128::try_from(mul_div_round_u128(
            squared as u128,
            scale,
            inertia.principal_numerators[index],
        )?)
        .map_err(|_| BallisticTimelineError3d::ArithmeticOverflow)?;
        rotational = rotational
            .checked_add(term)
            .ok_or(BallisticTimelineError3d::ArithmeticOverflow)?;
    }
    translational
        .checked_add(rotational)
        .ok_or(BallisticTimelineError3d::ArithmeticOverflow)
}

fn apply_body_impulse(
    rigid_box: &mut RigidBox3d,
    contact_offset: [i64; 3],
    impulse: [i128; 3],
) -> Result<(), BallisticTimelineError3d> {
    if rigid_box.body.kind == BodyKind::Fixed {
        return Ok(());
    }
    let mass = i128::from(rigid_box.body.mass_units);
    rigid_box.body.velocity = Vec3i::new(
        add_impulse_axis(rigid_box.body.velocity.x, impulse[0], mass)?,
        add_impulse_axis(rigid_box.body.velocity.y, impulse[1], mass)?,
        add_impulse_axis(rigid_box.body.velocity.z, impulse[2], mass)?,
    );
    if rigid_box.rotation_locked {
        return Ok(());
    }
    let angular_impulse = cross_i64_i128(contact_offset, impulse)?;
    let local_impulse = rotate_inverse(rigid_box.angular.orientation, angular_impulse)?;
    let inertia = box_inertia(&rigid_box.body)?;
    let angular_scale = i128::from(inertia.denominator)
        .checked_mul(i128::from(ANGULAR_VELOCITY_SCALE))
        .ok_or(BallisticTimelineError3d::ArithmeticOverflow)?;
    let mut local_delta = [0_i128; 3];
    for (index, (output, value)) in local_delta.iter_mut().zip(local_impulse).enumerate() {
        let principal = i128::try_from(inertia.principal_numerators[index])
            .map_err(|_| BallisticTimelineError3d::ArithmeticOverflow)?;
        *output = mul_div_round_i128(value, angular_scale, principal)?;
    }
    let world_delta = rotate_forward(rigid_box.angular.orientation, local_delta)?;
    rigid_box.angular.angular_velocity = AngularVelocity3d::new(
        add_angular_axis(rigid_box.angular.angular_velocity.x, world_delta[0])?,
        add_angular_axis(rigid_box.angular.angular_velocity.y, world_delta[1])?,
        add_angular_axis(rigid_box.angular.angular_velocity.z, world_delta[2])?,
    );
    Ok(())
}

fn projectile_contact_point(
    projectile: BallisticSphere3d,
    normal: [i128; 3],
) -> Result<Vec3i, BallisticTimelineError3d> {
    let length_squared = axis_length_squared(normal)?;
    let length = integer_sqrt(length_squared).max(1);
    let length =
        i128::try_from(length).map_err(|_| BallisticTimelineError3d::ArithmeticOverflow)?;
    let radius = i128::from(projectile.radius());
    let center = projectile.position();
    let mut point = [
        i128::from(center.x),
        i128::from(center.y),
        i128::from(center.z),
    ];
    for axis in 0..3 {
        let offset = div_round_nearest(checked_mul(radius, normal[axis])?, length)?;
        point[axis] = point[axis]
            .checked_sub(offset)
            .ok_or(BallisticTimelineError3d::ArithmeticOverflow)?;
    }
    Ok(Vec3i::new(
        to_i32(point[0])?,
        to_i32(point[1])?,
        to_i32(point[2])?,
    ))
}

fn contact_velocity(
    rigid_box: &RigidBox3d,
    offset: [i64; 3],
) -> Result<[i64; 3], BallisticTimelineError3d> {
    let omega = if rigid_box.rotation_locked {
        AngularVelocity3d::default()
    } else {
        rigid_box.angular.angular_velocity
    };
    let rotation = [
        checked_sub(
            checked_mul(i128::from(omega.y), i128::from(offset[2]))?,
            checked_mul(i128::from(omega.z), i128::from(offset[1]))?,
        )?,
        checked_sub(
            checked_mul(i128::from(omega.z), i128::from(offset[0]))?,
            checked_mul(i128::from(omega.x), i128::from(offset[2]))?,
        )?,
        checked_sub(
            checked_mul(i128::from(omega.x), i128::from(offset[1]))?,
            checked_mul(i128::from(omega.y), i128::from(offset[0]))?,
        )?,
    ];
    let linear = rigid_box.body.velocity;
    Ok([
        add_linear_rotation(linear.x, rotation[0])?,
        add_linear_rotation(linear.y, rotation[1])?,
        add_linear_rotation(linear.z, rotation[2])?,
    ])
}

fn add_linear_rotation(
    linear: i32,
    rotational_numerator: i128,
) -> Result<i64, BallisticTimelineError3d> {
    let rotational = i64::try_from(div_round_nearest(
        rotational_numerator,
        i128::from(ANGULAR_VELOCITY_SCALE),
    )?)
    .map_err(|_| BallisticTimelineError3d::ArithmeticOverflow)?;
    i64::from(linear)
        .checked_add(rotational)
        .ok_or(BallisticTimelineError3d::ArithmeticOverflow)
}

fn vector_delta(center: Vec3i, point: Vec3i) -> Result<[i64; 3], BallisticTimelineError3d> {
    Ok([
        i64::from(point.x) - i64::from(center.x),
        i64::from(point.y) - i64::from(center.y),
        i64::from(point.z) - i64::from(center.z),
    ])
}

fn rotate_inverse(
    orientation: Orientation3d,
    vector: [i128; 3],
) -> Result<[i128; 3], BallisticTimelineError3d> {
    let matrix = rotation_matrix(orientation.normalized()?)?;
    rotate_with_matrix(
        [
            [matrix[0][0], matrix[1][0], matrix[2][0]],
            [matrix[0][1], matrix[1][1], matrix[2][1]],
            [matrix[0][2], matrix[1][2], matrix[2][2]],
        ],
        vector,
    )
}

fn rotate_forward(
    orientation: Orientation3d,
    vector: [i128; 3],
) -> Result<[i128; 3], BallisticTimelineError3d> {
    rotate_with_matrix(rotation_matrix(orientation.normalized()?)?, vector)
}

fn rotation_matrix(orientation: Orientation3d) -> Result<[[i128; 3]; 3], BallisticTimelineError3d> {
    let x = i128::from(orientation.x);
    let y = i128::from(orientation.y);
    let z = i128::from(orientation.z);
    let w = i128::from(orientation.w);
    let scale = i128::from(ORIENTATION_SCALE);
    let xx = checked_mul(x, x)?;
    let yy = checked_mul(y, y)?;
    let zz = checked_mul(z, z)?;
    let xy = checked_mul(x, y)?;
    let xz = checked_mul(x, z)?;
    let yz = checked_mul(y, z)?;
    let xw = checked_mul(x, w)?;
    let yw = checked_mul(y, w)?;
    let zw = checked_mul(z, w)?;
    Ok([
        [
            checked_sub(scale, scaled_twice(checked_add(yy, zz)?, scale)?)?,
            scaled_twice(checked_sub(xy, zw)?, scale)?,
            scaled_twice(checked_add(xz, yw)?, scale)?,
        ],
        [
            scaled_twice(checked_add(xy, zw)?, scale)?,
            checked_sub(scale, scaled_twice(checked_add(xx, zz)?, scale)?)?,
            scaled_twice(checked_sub(yz, xw)?, scale)?,
        ],
        [
            scaled_twice(checked_sub(xz, yw)?, scale)?,
            scaled_twice(checked_add(yz, xw)?, scale)?,
            checked_sub(scale, scaled_twice(checked_add(xx, yy)?, scale)?)?,
        ],
    ])
}

fn rotate_with_matrix(
    matrix: [[i128; 3]; 3],
    vector: [i128; 3],
) -> Result<[i128; 3], BallisticTimelineError3d> {
    let scale = i128::from(ORIENTATION_SCALE);
    let mut output = [0_i128; 3];
    for (target, row) in output.iter_mut().zip(matrix) {
        *target = div_round_nearest(
            checked_add(
                checked_add(
                    checked_mul(row[0], vector[0])?,
                    checked_mul(row[1], vector[1])?,
                )?,
                checked_mul(row[2], vector[2])?,
            )?,
            scale,
        )?;
    }
    Ok(output)
}

fn scaled_twice(value: i128, scale: i128) -> Result<i128, BallisticTimelineError3d> {
    div_round_nearest(checked_mul(value, 2)?, scale)
}

fn cross_i64_i128(left: [i64; 3], right: [i128; 3]) -> Result<[i128; 3], BallisticTimelineError3d> {
    let left = left.map(i128::from);
    Ok([
        checked_sub(
            checked_mul(left[1], right[2])?,
            checked_mul(left[2], right[1])?,
        )?,
        checked_sub(
            checked_mul(left[2], right[0])?,
            checked_mul(left[0], right[2])?,
        )?,
        checked_sub(
            checked_mul(left[0], right[1])?,
            checked_mul(left[1], right[0])?,
        )?,
    ])
}

fn primitive_axis(axis: [i128; 3]) -> Result<[i128; 3], BallisticTimelineError3d> {
    let divisor = axis
        .iter()
        .map(|value| value.unsigned_abs())
        .fold(0_u128, gcd_u128);
    if divisor == 0 {
        return Err(BallisticTimelineError3d::ArithmeticOverflow);
    }
    let divisor =
        i128::try_from(divisor).map_err(|_| BallisticTimelineError3d::ArithmeticOverflow)?;
    Ok([axis[0] / divisor, axis[1] / divisor, axis[2] / divisor])
}

fn axis_length_squared(axis: [i128; 3]) -> Result<u128, BallisticTimelineError3d> {
    axis.into_iter().try_fold(0_u128, |sum, value| {
        let magnitude = value.unsigned_abs();
        sum.checked_add(
            magnitude
                .checked_mul(magnitude)
                .ok_or(BallisticTimelineError3d::ArithmeticOverflow)?,
        )
        .ok_or(BallisticTimelineError3d::ArithmeticOverflow)
    })
}

fn ballistic_time_to_sampled(
    fraction_subticks: u64,
) -> Result<SampledContactTime3d, BallisticTimelineError3d> {
    if fraction_subticks > BALLISTIC_TIME_SCALE {
        return Err(BallisticTimelineError3d::ArithmeticOverflow);
    }
    // The Q32.32 narrow phase can conservatively quantize an impact immediately after the
    // interval start to zero. The event timeline is Q1.31, so admit that evidence at its
    // first strictly-positive representable instant instead of treating it as arithmetic
    // failure. Initial overlap is already rejected by the ballistic narrow phase.
    let numerator = fraction_subticks.div_ceil(2).max(1);
    let numerator =
        u32::try_from(numerator).map_err(|_| BallisticTimelineError3d::ArithmeticOverflow)?;
    let divisor = gcd_u32(numerator, TIMELINE_TIME_SCALE);
    Ok(SampledContactTime3d {
        numerator: numerator / divisor,
        denominator: TIMELINE_TIME_SCALE / divisor,
    })
}

fn accumulate_query_stats(work: &mut BallisticStepWork3d, stats: BallisticSphereQueryStats3d) {
    work.broad_phase_candidates = work
        .broad_phase_candidates
        .saturating_add(stats.broad_phase_candidates);
    work.toi_tests = work.toi_tests.saturating_add(stats.toi_tests);
    work.feature_tests = work.feature_tests.saturating_add(stats.feature_tests);
}

fn add_impulse_axis(
    current: i32,
    impulse: i128,
    mass: i128,
) -> Result<i32, BallisticTimelineError3d> {
    to_i32(
        i128::from(current)
            .checked_add(div_round_nearest(impulse, mass)?)
            .ok_or(BallisticTimelineError3d::ArithmeticOverflow)?,
    )
}

fn add_impulse_axis_milli(
    current: i32,
    impulse: i128,
    mass_milli: i128,
) -> Result<i32, BallisticTimelineError3d> {
    let delta = mul_div_round_i128(impulse, i128::from(MATERIAL_SCALE), mass_milli)?;
    to_i32(
        i128::from(current)
            .checked_add(delta)
            .ok_or(BallisticTimelineError3d::ArithmeticOverflow)?,
    )
}

fn add_angular_axis(current: i32, delta: i128) -> Result<i32, BallisticTimelineError3d> {
    to_i32(
        i128::from(current)
            .checked_add(delta)
            .ok_or(BallisticTimelineError3d::ArithmeticOverflow)?,
    )
}

fn negate_axis(vector: [i128; 3]) -> Result<[i128; 3], BallisticTimelineError3d> {
    Ok([
        vector[0]
            .checked_neg()
            .ok_or(BallisticTimelineError3d::ArithmeticOverflow)?,
        vector[1]
            .checked_neg()
            .ok_or(BallisticTimelineError3d::ArithmeticOverflow)?,
        vector[2]
            .checked_neg()
            .ok_or(BallisticTimelineError3d::ArithmeticOverflow)?,
    ])
}

fn checked_add(left: i128, right: i128) -> Result<i128, BallisticTimelineError3d> {
    left.checked_add(right)
        .ok_or(BallisticTimelineError3d::ArithmeticOverflow)
}

fn checked_sub(left: i128, right: i128) -> Result<i128, BallisticTimelineError3d> {
    left.checked_sub(right)
        .ok_or(BallisticTimelineError3d::ArithmeticOverflow)
}

fn checked_mul(left: i128, right: i128) -> Result<i128, BallisticTimelineError3d> {
    left.checked_mul(right)
        .ok_or(BallisticTimelineError3d::ArithmeticOverflow)
}

fn checked_dot(left: [i128; 3], right: [i128; 3]) -> Result<i128, BallisticTimelineError3d> {
    checked_add(
        checked_add(
            checked_mul(left[0], right[0])?,
            checked_mul(left[1], right[1])?,
        )?,
        checked_mul(left[2], right[2])?,
    )
}

fn div_round_nearest(numerator: i128, denominator: i128) -> Result<i128, BallisticTimelineError3d> {
    if denominator <= 0 {
        return Err(BallisticTimelineError3d::ArithmeticOverflow);
    }
    let half = denominator / 2;
    let adjusted = if numerator >= 0 {
        numerator.checked_add(half)
    } else {
        numerator.checked_sub(half)
    }
    .ok_or(BallisticTimelineError3d::ArithmeticOverflow)?;
    Ok(adjusted / denominator)
}

fn to_i32(value: i128) -> Result<i32, BallisticTimelineError3d> {
    i32::try_from(value).map_err(|_| BallisticTimelineError3d::ArithmeticOverflow)
}

fn component(vector: Vec3i, axis: usize) -> i32 {
    match axis {
        0 => vector.x,
        1 => vector.y,
        _ => vector.z,
    }
}

fn set_component(vector: &mut Vec3i, axis: usize, value: i32) {
    match axis {
        0 => vector.x = value,
        1 => vector.y = value,
        _ => vector.z = value,
    }
}

fn gcd_u32(mut left: u32, mut right: u32) -> u32 {
    while right != 0 {
        let remainder = left % right;
        left = right;
        right = remainder;
    }
    left.max(1)
}

fn gcd_u128(mut left: u128, mut right: u128) -> u128 {
    while right != 0 {
        let remainder = left % right;
        left = right;
        right = remainder;
    }
    left
}

fn integer_sqrt(value: u128) -> u128 {
    if value < 2 {
        return value;
    }
    let mut low = 1_u128;
    let mut high = value.min(u128::from(u64::MAX));
    while low <= high {
        let middle = low + (high - low) / 2;
        match middle.checked_mul(middle).map(|square| square.cmp(&value)) {
            Some(std::cmp::Ordering::Equal) => return middle,
            Some(std::cmp::Ordering::Less) => low = middle.saturating_add(1),
            Some(std::cmp::Ordering::Greater) | None => high = middle.saturating_sub(1),
        }
    }
    high
}

#[cfg(test)]
mod timeline_time_tests {
    use super::{
        MATERIAL_SCALE, RESPONSE_SCALE, TIMELINE_TIME_SCALE, ballistic_time_to_sampled,
        checked_add, checked_mul, response_impulse_component,
    };

    #[test]
    fn zero_q32_hit_rounds_up_to_first_positive_timeline_tick() {
        let time = ballistic_time_to_sampled(0).expect("quantized immediate hit");
        assert_eq!(time.numerator, 1);
        assert_eq!(time.denominator, TIMELINE_TIME_SCALE);
    }

    #[test]
    fn response_impulse_reduces_q32_normal_scale_before_forming_denominator() {
        let axis_x = 12_884_901_887_i128;
        let axis_y = 12_884_901_886_i128;
        let axis_length_squared = checked_add(
            checked_mul(axis_x, axis_x).expect("x normal square"),
            checked_mul(axis_y, axis_y).expect("y normal square"),
        )
        .expect("bounded Q32 normal square");
        let effective_inverse_mass =
            checked_mul(axis_length_squared, RESPONSE_SCALE).expect("scaled inverse mass");

        assert!(
            i128::from(MATERIAL_SCALE)
                .checked_mul(effective_inverse_mass)
                .is_none(),
            "the unreduced response denominator must reproduce the former overflow"
        );

        let closing_speed = checked_mul(5_280, axis_x).expect("closing speed");
        assert_eq!(
            response_impulse_component(closing_speed, 0, axis_x, effective_inverse_mass)
                .expect("inelastic response"),
            2_640
        );
        assert_eq!(
            response_impulse_component(
                closing_speed,
                MATERIAL_SCALE,
                axis_x,
                effective_inverse_mass,
            )
            .expect("elastic response"),
            5_280
        );
    }
}
