use std::{error::Error, fmt};

use crate::{
    AngularError3d, BodyId, CollisionLayers3d, Material, ORIENTATION_SCALE, Orientation3d,
    OrientedBoxError3d, RigidBox3d, Vec3i, oriented_box_vertices,
    wide_ratio::{WideRatioError, mul_div_round_i128},
};

const BALLISTIC_TIME_SCALE: u64 = 1_u64 << 32;
const BALLISTIC_TIME_SCALE_I128: i128 = 1_i128 << 32;

/// Lightweight projectile state for a rotation-invariant spherical body.
///
/// A ballistic sphere deliberately carries no orientation, angular velocity, box inertia, or persistent
/// contact state. Rotation cannot change its collision geometry, so it can use a dedicated continuous
/// sweep instead of sampled rotating-box collision detection.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct BallisticSphere3d {
    id: BodyId,
    position: Vec3i,
    velocity: Vec3i,
    radius: i32,
    mass_units: u32,
    material: Material,
    collision_layers: CollisionLayers3d,
}

impl BallisticSphere3d {
    pub fn new(
        id: BodyId,
        position: Vec3i,
        velocity: Vec3i,
        radius: i32,
        mass_units: u32,
    ) -> Result<Self, BallisticSphereError3d> {
        if radius <= 0 {
            return Err(BallisticSphereError3d::NonPositiveRadius(id, radius));
        }
        if mass_units == 0 {
            return Err(BallisticSphereError3d::ZeroMass(id));
        }
        Ok(Self {
            id,
            position,
            velocity,
            radius,
            mass_units,
            material: Material::default(),
            collision_layers: CollisionLayers3d::default(),
        })
    }

    #[must_use]
    pub const fn id(self) -> BodyId {
        self.id
    }

    #[must_use]
    pub const fn position(self) -> Vec3i {
        self.position
    }

    #[must_use]
    pub const fn velocity(self) -> Vec3i {
        self.velocity
    }

    #[must_use]
    pub const fn radius(self) -> i32 {
        self.radius
    }

    #[must_use]
    pub const fn mass_units(self) -> u32 {
        self.mass_units
    }

    #[must_use]
    pub const fn material(self) -> Material {
        self.material
    }

    #[must_use]
    pub const fn collision_layers(self) -> CollisionLayers3d {
        self.collision_layers
    }

    #[must_use]
    pub const fn with_material(mut self, material: Material) -> Self {
        self.material = material;
        self
    }

    #[must_use]
    pub const fn with_collision_layers(mut self, collision_layers: CollisionLayers3d) -> Self {
        self.collision_layers = collision_layers;
        self
    }

    pub fn set_position(&mut self, position: Vec3i) {
        self.position = position;
    }

    pub fn set_velocity(&mut self, velocity: Vec3i) {
        self.velocity = velocity;
    }

    /// Applies acceleration over an exact rational timestep and rounds each resulting velocity component
    /// to the engine's integer velocity units.
    pub fn apply_acceleration(
        &mut self,
        acceleration: Vec3i,
        timestep_numerator: i32,
        timestep_denominator: i32,
    ) -> Result<(), BallisticSphereError3d> {
        validate_timestep(timestep_numerator, timestep_denominator)?;
        let numerator = i128::from(timestep_numerator);
        let denominator = i128::from(timestep_denominator);
        self.velocity = Vec3i::new(
            add_scaled(self.velocity.x, acceleration.x, numerator, denominator)?,
            add_scaled(self.velocity.y, acceleration.y, numerator, denominator)?,
            add_scaled(self.velocity.z, acceleration.z, numerator, denominator)?,
        );
        Ok(())
    }

    /// Advances linearly over an exact rational timestep. Collision response can split a timestep at the
    /// returned query fraction and use this operation for each contact-free segment.
    pub fn advance_linear(
        &mut self,
        timestep_numerator: i32,
        timestep_denominator: i32,
    ) -> Result<(), BallisticSphereError3d> {
        validate_timestep(timestep_numerator, timestep_denominator)?;
        let numerator = i128::from(timestep_numerator);
        let denominator = i128::from(timestep_denominator);
        self.position = Vec3i::new(
            advance_axis(self.position.x, self.velocity.x, numerator, denominator)?,
            advance_axis(self.position.y, self.velocity.y, numerator, denominator)?,
            advance_axis(self.position.z, self.velocity.z, numerator, denominator)?,
        );
        Ok(())
    }
}

/// Q32.32 fraction of the requested ballistic query interval.
#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub struct BallisticTime3d {
    fraction_subticks: u64,
}

impl BallisticTime3d {
    #[must_use]
    pub const fn fraction_subticks(self) -> u64 {
        self.fraction_subticks
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct BallisticSphereSweepHit3d {
    pub body: BodyId,
    pub time: BallisticTime3d,
    /// Primitive integer world-space direction pointing from the rigid target toward the sphere.
    /// Only direction is significant; the vector is deliberately not normalized with floating point.
    pub normal: [i128; 3],
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct BallisticSphereQueryStats3d {
    /// Targets whose conservative swept world-space bounds overlapped the sphere sweep.
    pub broad_phase_candidates: u64,
    /// Rounded-box time-of-impact queries executed after broad-phase rejection.
    pub toi_tests: u64,
    /// Face, edge, and corner feature tests used by sphere-versus-frozen-OBB CCD.
    pub feature_tests: u64,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BallisticSphereError3d {
    NonPositiveRadius(BodyId, i32),
    ZeroMass(BodyId),
    NegativeTimestepNumerator(i32),
    NonPositiveTimestepDenominator(i32),
    Geometry(OrientedBoxError3d),
    Angular(AngularError3d),
    ArithmeticOverflow,
}

impl fmt::Display for BallisticSphereError3d {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NonPositiveRadius(id, radius) => write!(
                formatter,
                "ballistic sphere {} requires a positive radius, got {radius}",
                id.0
            ),
            Self::ZeroMass(id) => {
                write!(
                    formatter,
                    "ballistic sphere {} requires non-zero mass",
                    id.0
                )
            }
            Self::NegativeTimestepNumerator(value) => write!(
                formatter,
                "ballistic sphere timestep numerator must be non-negative, got {value}"
            ),
            Self::NonPositiveTimestepDenominator(value) => write!(
                formatter,
                "ballistic sphere timestep denominator must be positive, got {value}"
            ),
            Self::Geometry(error) => write!(
                formatter,
                "ballistic sphere target geometry failed: {error}"
            ),
            Self::Angular(error) => write!(
                formatter,
                "ballistic sphere target orientation failed: {error}"
            ),
            Self::ArithmeticOverflow => write!(formatter, "ballistic sphere arithmetic overflowed"),
        }
    }
}

impl Error for BallisticSphereError3d {}

impl From<OrientedBoxError3d> for BallisticSphereError3d {
    fn from(value: OrientedBoxError3d) -> Self {
        Self::Geometry(value)
    }
}

impl From<AngularError3d> for BallisticSphereError3d {
    fn from(value: AngularError3d) -> Self {
        Self::Angular(value)
    }
}

impl From<WideRatioError> for BallisticSphereError3d {
    fn from(_: WideRatioError) -> Self {
        Self::ArithmeticOverflow
    }
}

#[derive(Clone, Debug)]
struct PreparedBallisticTarget3d {
    id: BodyId,
    center: Vec3i,
    velocity: Vec3i,
    half_extents: Vec3i,
    rotation: [[i128; 3]; 3],
    world_minimum: [i64; 3],
    world_maximum: [i64; 3],
    collision_layers: CollisionLayers3d,
}

#[derive(Clone, Copy, Debug)]
struct PreparedBallisticStepTarget3d {
    displacement: [i64; 3],
    swept_bounds: ([i64; 3], [i64; 3]),
}

/// Immutable rigid-scene preparation shared by ballistic sphere queries across frames.
///
/// Target OBB geometry is prepared once from the rigid world. A [`BallisticSphereStep3d`] then prepares
/// only per-step target displacement and swept bounds once for all projectiles in that simulation step.
#[derive(Clone, Debug, Default)]
pub struct BallisticSphereScene3d {
    targets: Vec<PreparedBallisticTarget3d>,
}

impl BallisticSphereScene3d {
    pub fn prepare<'a>(
        boxes: impl IntoIterator<Item = &'a RigidBox3d>,
    ) -> Result<Self, BallisticSphereError3d> {
        let mut targets = boxes
            .into_iter()
            .map(prepare_target)
            .collect::<Result<Vec<_>, _>>()?;
        targets.sort_by_key(|target| target.id);
        Ok(Self { targets })
    }

    #[must_use]
    pub fn target_count(&self) -> usize {
        self.targets.len()
    }

    pub fn prepare_step(
        &self,
        timestep_numerator: i32,
        timestep_denominator: i32,
    ) -> Result<BallisticSphereStep3d<'_>, BallisticSphereError3d> {
        validate_timestep(timestep_numerator, timestep_denominator)?;
        let mut prepared_targets = Vec::with_capacity(self.targets.len());
        for target in &self.targets {
            let target_displacement = displacement(
                target.velocity,
                timestep_numerator,
                timestep_denominator,
            )?;
            prepared_targets.push(PreparedBallisticStepTarget3d {
                displacement: target_displacement,
                swept_bounds: swept_target_bounds(target, target_displacement)?,
            });
        }
        Ok(BallisticSphereStep3d {
            scene: self,
            timestep_numerator,
            timestep_denominator,
            prepared_targets,
        })
    }

    pub fn earliest_hit(
        &self,
        sphere: BallisticSphere3d,
        timestep_numerator: i32,
        timestep_denominator: i32,
        stats: &mut BallisticSphereQueryStats3d,
    ) -> Result<Option<BallisticSphereSweepHit3d>, BallisticSphereError3d> {
        self.prepare_step(timestep_numerator, timestep_denominator)?
            .earliest_hit(sphere, stats)
    }
}

/// Per-step ballistic query state. Target orientation is frozen for this substep while target linear
/// displacement is included exactly in the quantized engine units. The narrow phase evaluates the rounded
/// OBB Minkowski boundary as six face prisms, twelve finite edge cylinders, and eight corner spheres.
/// Rapidly rotating targets should stay on the general rotating rigid-body path.
#[derive(Debug)]
pub struct BallisticSphereStep3d<'a> {
    scene: &'a BallisticSphereScene3d,
    timestep_numerator: i32,
    timestep_denominator: i32,
    prepared_targets: Vec<PreparedBallisticStepTarget3d>,
}

impl BallisticSphereStep3d<'_> {
    pub fn earliest_hit(
        &self,
        sphere: BallisticSphere3d,
        stats: &mut BallisticSphereQueryStats3d,
    ) -> Result<Option<BallisticSphereSweepHit3d>, BallisticSphereError3d> {
        if self.timestep_numerator == 0 {
            return Ok(None);
        }
        let sphere_displacement = displacement(
            sphere.velocity,
            self.timestep_numerator,
            self.timestep_denominator,
        )?;
        let sphere_bounds = swept_sphere_bounds(sphere, sphere_displacement)?;
        let mut earliest = None;
        for (target, step_target) in self
            .scene
            .targets
            .iter()
            .zip(self.prepared_targets.iter().copied())
        {
            if target.id == sphere.id
                || !sphere
                    .collision_layers
                    .collides_with(target.collision_layers)
                || !bounds_overlap(sphere_bounds, step_target.swept_bounds)
            {
                continue;
            }
            stats.broad_phase_candidates = stats.broad_phase_candidates.saturating_add(1);
            stats.toi_tests = stats.toi_tests.saturating_add(1);
            let Some(hit) = swept_sphere_target(
                sphere,
                sphere_displacement,
                target,
                step_target.displacement,
                stats,
            )?
            else {
                continue;
            };
            if earliest.is_none_or(|current: BallisticSphereSweepHit3d| {
                hit.time < current.time || (hit.time == current.time && hit.body < current.body)
            }) {
                earliest = Some(hit);
            }
        }
        Ok(earliest)
    }
}

fn prepare_target(
    rigid_box: &RigidBox3d,
) -> Result<PreparedBallisticTarget3d, BallisticSphereError3d> {
    let shape = rigid_box.oriented_box();
    let vertices = oriented_box_vertices(shape)?;
    let mut world_minimum = [i64::MAX; 3];
    let mut world_maximum = [i64::MIN; 3];
    for vertex in vertices {
        let axes = [
            i64::from(vertex.x),
            i64::from(vertex.y),
            i64::from(vertex.z),
        ];
        for axis in 0..3 {
            world_minimum[axis] = world_minimum[axis].min(axes[axis]);
            world_maximum[axis] = world_maximum[axis].max(axes[axis]);
        }
    }
    let orientation = shape.orientation.normalized()?;
    Ok(PreparedBallisticTarget3d {
        id: rigid_box.body().id(),
        center: shape.center,
        velocity: rigid_box.body().velocity(),
        half_extents: shape.half_extents,
        rotation: rotation_matrix(orientation)?,
        world_minimum,
        world_maximum,
        collision_layers: rigid_box.collision_layers(),
    })
}

fn swept_sphere_target(
    sphere: BallisticSphere3d,
    sphere_displacement: [i64; 3],
    target: &PreparedBallisticTarget3d,
    target_displacement: [i64; 3],
    stats: &mut BallisticSphereQueryStats3d,
) -> Result<Option<BallisticSphereSweepHit3d>, BallisticSphereError3d> {
    let inverse = transpose(target.rotation);
    let relative_position = rotate_vector(
        inverse,
        [
            i64::from(sphere.position.x) - i64::from(target.center.x),
            i64::from(sphere.position.y) - i64::from(target.center.y),
            i64::from(sphere.position.z) - i64::from(target.center.z),
        ],
    )?;
    let relative_displacement = rotate_vector(
        inverse,
        [
            sphere_displacement[0] - target_displacement[0],
            sphere_displacement[1] - target_displacement[1],
            sphere_displacement[2] - target_displacement[2],
        ],
    )?;
    let extents = [
        i64::from(target.half_extents.x),
        i64::from(target.half_extents.y),
        i64::from(target.half_extents.z),
    ];
    let radius = i64::from(sphere.radius);
    if point_aabb_distance_squared(relative_position, extents)?
        <= i128::from(radius) * i128::from(radius)
    {
        return Ok(None);
    }

    let mut best: Option<(u64, [i128; 3])> = None;
    for axis in 0..3 {
        for sign in [-1_i64, 1] {
            stats.feature_tests = stats.feature_tests.saturating_add(1);
            if let Some(time) = face_hit_time(
                relative_position,
                relative_displacement,
                extents,
                radius,
                axis,
                sign,
            )? {
                update_best_feature(&mut best, time, local_axis(axis, sign));
            }
        }
    }

    for free_axis in 0..3 {
        let side_axes = other_axes(free_axis);
        for first_sign in [-1_i64, 1] {
            for second_sign in [-1_i64, 1] {
                stats.feature_tests = stats.feature_tests.saturating_add(1);
                if let Some(time) = edge_hit_time(
                    relative_position,
                    relative_displacement,
                    extents,
                    radius,
                    free_axis,
                    side_axes,
                    [first_sign, second_sign],
                )? {
                    let local = edge_normal_at(
                        relative_position,
                        relative_displacement,
                        extents,
                        time,
                        side_axes,
                        [first_sign, second_sign],
                    )?;
                    update_best_feature(&mut best, time, local);
                }
            }
        }
    }

    for x_sign in [-1_i64, 1] {
        for y_sign in [-1_i64, 1] {
            for z_sign in [-1_i64, 1] {
                stats.feature_tests = stats.feature_tests.saturating_add(1);
                let signs = [x_sign, y_sign, z_sign];
                if let Some(time) = corner_hit_time(
                    relative_position,
                    relative_displacement,
                    extents,
                    radius,
                    signs,
                )? {
                    let local = corner_normal_at(
                        relative_position,
                        relative_displacement,
                        extents,
                        time,
                        signs,
                    )?;
                    update_best_feature(&mut best, time, local);
                }
            }
        }
    }

    let Some((fraction_subticks, local_normal)) = best else {
        return Ok(None);
    };
    let normal = rotate_fixed_vector(target.rotation, local_normal)?;
    Ok(Some(BallisticSphereSweepHit3d {
        body: target.id,
        time: BallisticTime3d { fraction_subticks },
        normal,
    }))
}

fn face_hit_time(
    position: [i64; 3],
    movement: [i64; 3],
    extents: [i64; 3],
    radius: i64,
    axis: usize,
    sign: i64,
) -> Result<Option<u64>, BallisticSphereError3d> {
    if movement[axis] == 0 || movement[axis].signum() != -sign {
        return Ok(None);
    }
    let plane = sign
        .checked_mul(
            extents[axis]
                .checked_add(radius)
                .ok_or(BallisticSphereError3d::ArithmeticOverflow)?,
        )
        .ok_or(BallisticSphereError3d::ArithmeticOverflow)?;
    let numerator = i128::from(plane) - i128::from(position[axis]);
    let denominator = i128::from(movement[axis]);
    let Some(time) = positive_fraction_subticks(numerator, denominator)? else {
        return Ok(None);
    };
    for tangent in other_axes(axis) {
        let coordinate = scaled_coordinate(position[tangent], movement[tangent], time)?;
        let limit = checked_mul(
            i128::from(extents[tangent]),
            BALLISTIC_TIME_SCALE_I128,
        )?;
        if coordinate.abs() > limit {
            return Ok(None);
        }
    }
    Ok(Some(time))
}

fn edge_hit_time(
    position: [i64; 3],
    movement: [i64; 3],
    extents: [i64; 3],
    radius: i64,
    free_axis: usize,
    side_axes: [usize; 2],
    signs: [i64; 2],
) -> Result<Option<u64>, BallisticSphereError3d> {
    let Some((minimum, maximum)) =
        axis_inside_interval(position[free_axis], movement[free_axis], extents[free_axis])?
    else {
        return Ok(None);
    };
    let offsets = [
        i128::from(position[side_axes[0]])
            - i128::from(signs[0]) * i128::from(extents[side_axes[0]]),
        i128::from(position[side_axes[1]])
            - i128::from(signs[1]) * i128::from(extents[side_axes[1]]),
    ];
    let deltas = [
        i128::from(movement[side_axes[0]]),
        i128::from(movement[side_axes[1]]),
    ];
    earliest_quadratic_contact(offsets, deltas, radius, minimum.max(1), maximum)
}

fn corner_hit_time(
    position: [i64; 3],
    movement: [i64; 3],
    extents: [i64; 3],
    radius: i64,
    signs: [i64; 3],
) -> Result<Option<u64>, BallisticSphereError3d> {
    let offsets = [
        i128::from(position[0]) - i128::from(signs[0]) * i128::from(extents[0]),
        i128::from(position[1]) - i128::from(signs[1]) * i128::from(extents[1]),
        i128::from(position[2]) - i128::from(signs[2]) * i128::from(extents[2]),
    ];
    let deltas = [
        i128::from(movement[0]),
        i128::from(movement[1]),
        i128::from(movement[2]),
    ];
    earliest_quadratic_contact(offsets, deltas, radius, 1, BALLISTIC_TIME_SCALE)
}

fn earliest_quadratic_contact<const N: usize>(
    offsets: [i128; N],
    deltas: [i128; N],
    radius: i64,
    minimum: u64,
    maximum: u64,
) -> Result<Option<u64>, BallisticSphereError3d> {
    if minimum > maximum || maximum == 0 {
        return Ok(None);
    }
    let a = deltas.iter().try_fold(0_i128, |sum, value| {
        checked_add(sum, checked_mul(*value, *value)?)
    })?;
    let scaled_radius = checked_mul(i128::from(radius), BALLISTIC_TIME_SCALE_I128)?;
    let threshold = checked_mul(scaled_radius, scaled_radius)?;
    if a == 0 {
        return Ok(
            (feature_distance_squared(offsets, deltas, minimum)? <= threshold).then_some(minimum),
        );
    }

    let dot = offsets
        .iter()
        .zip(deltas)
        .try_fold(0_i128, |sum, (offset, delta)| {
            checked_add(sum, checked_mul(*offset, delta)?)
        })?;
    let vertex_numerator = checked_mul(
        dot.checked_neg()
            .ok_or(BallisticSphereError3d::ArithmeticOverflow)?,
        BALLISTIC_TIME_SCALE_I128,
    )?;
    let closest = if vertex_numerator <= 0 {
        minimum
    } else {
        let rounded = div_round_nearest(vertex_numerator, a)?;
        u64::try_from(rounded)
            .map_err(|_| BallisticSphereError3d::ArithmeticOverflow)?
            .clamp(minimum, maximum)
    };
    if feature_distance_squared(offsets, deltas, closest)? > threshold {
        return Ok(None);
    }
    if feature_distance_squared(offsets, deltas, minimum)? <= threshold {
        return Ok(Some(minimum));
    }

    let mut low = minimum;
    let mut high = closest;
    while low < high {
        let middle = low + (high - low) / 2;
        if feature_distance_squared(offsets, deltas, middle)? <= threshold {
            high = middle;
        } else {
            low = middle.saturating_add(1);
        }
    }
    Ok(Some(low))
}

fn feature_distance_squared<const N: usize>(
    offsets: [i128; N],
    deltas: [i128; N],
    time: u64,
) -> Result<i128, BallisticSphereError3d> {
    offsets
        .into_iter()
        .zip(deltas)
        .try_fold(0_i128, |sum, (offset, delta)| {
            let coordinate = checked_add(
                checked_mul(offset, BALLISTIC_TIME_SCALE_I128)?,
                checked_mul(delta, i128::from(time))?,
            )?;
            checked_add(sum, checked_mul(coordinate, coordinate)?)
        })
}

fn axis_inside_interval(
    position: i64,
    movement: i64,
    extent: i64,
) -> Result<Option<(u64, u64)>, BallisticSphereError3d> {
    if movement == 0 {
        return Ok((position.abs() <= extent).then_some((0, BALLISTIC_TIME_SCALE)));
    }
    let mut first_numerator = i128::from(-extent) - i128::from(position);
    let mut second_numerator = i128::from(extent) - i128::from(position);
    let mut denominator = i128::from(movement);
    if denominator < 0 {
        denominator = denominator
            .checked_neg()
            .ok_or(BallisticSphereError3d::ArithmeticOverflow)?;
        first_numerator = first_numerator
            .checked_neg()
            .ok_or(BallisticSphereError3d::ArithmeticOverflow)?;
        second_numerator = second_numerator
            .checked_neg()
            .ok_or(BallisticSphereError3d::ArithmeticOverflow)?;
    }
    let lower_numerator = first_numerator.min(second_numerator);
    let upper_numerator = first_numerator.max(second_numerator);
    let lower = div_ceil_signed(
        checked_mul(lower_numerator, BALLISTIC_TIME_SCALE_I128)?,
        denominator,
    )?;
    let upper = div_floor_signed(
        checked_mul(upper_numerator, BALLISTIC_TIME_SCALE_I128)?,
        denominator,
    )?;
    let lower = lower.max(0);
    let upper = upper.min(BALLISTIC_TIME_SCALE_I128);
    if lower > upper {
        return Ok(None);
    }
    Ok(Some((
        u64::try_from(lower).map_err(|_| BallisticSphereError3d::ArithmeticOverflow)?,
        u64::try_from(upper).map_err(|_| BallisticSphereError3d::ArithmeticOverflow)?,
    )))
}

fn positive_fraction_subticks(
    mut numerator: i128,
    mut denominator: i128,
) -> Result<Option<u64>, BallisticSphereError3d> {
    if denominator == 0 {
        return Ok(None);
    }
    if denominator < 0 {
        numerator = numerator
            .checked_neg()
            .ok_or(BallisticSphereError3d::ArithmeticOverflow)?;
        denominator = denominator
            .checked_neg()
            .ok_or(BallisticSphereError3d::ArithmeticOverflow)?;
    }
    if numerator <= 0 {
        return Ok(None);
    }
    let scaled = checked_mul(numerator, BALLISTIC_TIME_SCALE_I128)?;
    let value = div_ceil_signed(scaled, denominator)?;
    if value <= 0 || value > BALLISTIC_TIME_SCALE_I128 {
        return Ok(None);
    }
    Ok(Some(
        u64::try_from(value).map_err(|_| BallisticSphereError3d::ArithmeticOverflow)?,
    ))
}

fn edge_normal_at(
    position: [i64; 3],
    movement: [i64; 3],
    extents: [i64; 3],
    time: u64,
    side_axes: [usize; 2],
    signs: [i64; 2],
) -> Result<[i128; 3], BallisticSphereError3d> {
    let mut normal = [0_i128; 3];
    for index in 0..2 {
        let axis = side_axes[index];
        normal[axis] = checked_sub(
            scaled_coordinate(position[axis], movement[axis], time)?,
            checked_mul(
                i128::from(signs[index]) * i128::from(extents[axis]),
                BALLISTIC_TIME_SCALE_I128,
            )?,
        )?;
    }
    primitive_direction(normal)
}

fn corner_normal_at(
    position: [i64; 3],
    movement: [i64; 3],
    extents: [i64; 3],
    time: u64,
    signs: [i64; 3],
) -> Result<[i128; 3], BallisticSphereError3d> {
    let mut normal = [0_i128; 3];
    for axis in 0..3 {
        normal[axis] = checked_sub(
            scaled_coordinate(position[axis], movement[axis], time)?,
            checked_mul(
                i128::from(signs[axis]) * i128::from(extents[axis]),
                BALLISTIC_TIME_SCALE_I128,
            )?,
        )?;
    }
    primitive_direction(normal)
}

fn primitive_direction(mut vector: [i128; 3]) -> Result<[i128; 3], BallisticSphereError3d> {
    let divisor = vector
        .iter()
        .map(|component| component.unsigned_abs())
        .filter(|component| *component != 0)
        .reduce(gcd)
        .ok_or(BallisticSphereError3d::ArithmeticOverflow)?;
    let divisor =
        i128::try_from(divisor).map_err(|_| BallisticSphereError3d::ArithmeticOverflow)?;
    for component in &mut vector {
        *component /= divisor;
    }
    Ok(vector)
}

fn local_axis(axis: usize, sign: i64) -> [i128; 3] {
    let mut normal = [0_i128; 3];
    normal[axis] = i128::from(sign);
    normal
}

fn update_best_feature(best: &mut Option<(u64, [i128; 3])>, time: u64, normal: [i128; 3]) {
    if best.is_none_or(|current| time < current.0) {
        *best = Some((time, normal));
    }
}

fn other_axes(axis: usize) -> [usize; 2] {
    match axis {
        0 => [1, 2],
        1 => [0, 2],
        _ => [0, 1],
    }
}

fn point_aabb_distance_squared(
    position: [i64; 3],
    extents: [i64; 3],
) -> Result<i128, BallisticSphereError3d> {
    (0..3).try_fold(0_i128, |sum, axis| {
        let distance = i128::from(position[axis].abs().saturating_sub(extents[axis]));
        checked_add(sum, checked_mul(distance, distance)?)
    })
}

fn displacement(
    velocity: Vec3i,
    timestep_numerator: i32,
    timestep_denominator: i32,
) -> Result<[i64; 3], BallisticSphereError3d> {
    Ok([
        displacement_axis(velocity.x, timestep_numerator, timestep_denominator)?,
        displacement_axis(velocity.y, timestep_numerator, timestep_denominator)?,
        displacement_axis(velocity.z, timestep_numerator, timestep_denominator)?,
    ])
}

fn displacement_axis(
    velocity: i32,
    timestep_numerator: i32,
    timestep_denominator: i32,
) -> Result<i64, BallisticSphereError3d> {
    let moved = mul_div_round_i128(
        i128::from(velocity),
        i128::from(timestep_numerator),
        i128::from(timestep_denominator),
    )?;
    i64::try_from(moved).map_err(|_| BallisticSphereError3d::ArithmeticOverflow)
}

fn swept_sphere_bounds(
    sphere: BallisticSphere3d,
    movement: [i64; 3],
) -> Result<([i64; 3], [i64; 3]), BallisticSphereError3d> {
    let start = [
        i64::from(sphere.position.x),
        i64::from(sphere.position.y),
        i64::from(sphere.position.z),
    ];
    let radius = i64::from(sphere.radius);
    let mut minimum = [0_i64; 3];
    let mut maximum = [0_i64; 3];
    for axis in 0..3 {
        let end = start[axis]
            .checked_add(movement[axis])
            .ok_or(BallisticSphereError3d::ArithmeticOverflow)?;
        minimum[axis] = start[axis]
            .min(end)
            .checked_sub(radius)
            .ok_or(BallisticSphereError3d::ArithmeticOverflow)?;
        maximum[axis] = start[axis]
            .max(end)
            .checked_add(radius)
            .ok_or(BallisticSphereError3d::ArithmeticOverflow)?;
    }
    Ok((minimum, maximum))
}

fn swept_target_bounds(
    target: &PreparedBallisticTarget3d,
    movement: [i64; 3],
) -> Result<([i64; 3], [i64; 3]), BallisticSphereError3d> {
    let mut minimum = [0_i64; 3];
    let mut maximum = [0_i64; 3];
    for axis in 0..3 {
        let moved_minimum = target.world_minimum[axis]
            .checked_add(movement[axis])
            .ok_or(BallisticSphereError3d::ArithmeticOverflow)?;
        let moved_maximum = target.world_maximum[axis]
            .checked_add(movement[axis])
            .ok_or(BallisticSphereError3d::ArithmeticOverflow)?;
        minimum[axis] = target.world_minimum[axis].min(moved_minimum);
        maximum[axis] = target.world_maximum[axis].max(moved_maximum);
    }
    Ok((minimum, maximum))
}

fn bounds_overlap(left: ([i64; 3], [i64; 3]), right: ([i64; 3], [i64; 3])) -> bool {
    (0..3).all(|axis| left.0[axis] <= right.1[axis] && right.0[axis] <= left.1[axis])
}

fn scaled_coordinate(
    position: i64,
    movement: i64,
    time: u64,
) -> Result<i128, BallisticSphereError3d> {
    checked_add(
        checked_mul(i128::from(position), BALLISTIC_TIME_SCALE_I128)?,
        checked_mul(i128::from(movement), i128::from(time))?,
    )
}

fn rotate_fixed_vector(
    matrix: [[i128; 3]; 3],
    vector: [i128; 3],
) -> Result<[i128; 3], BallisticSphereError3d> {
    let mut output = [0_i128; 3];
    for (target, row) in output.iter_mut().zip(matrix) {
        *target = checked_add(
            checked_add(
                checked_mul(row[0], vector[0])?,
                checked_mul(row[1], vector[1])?,
            )?,
            checked_mul(row[2], vector[2])?,
        )?;
    }
    primitive_direction(output)
}

fn rotate_vector(
    matrix: [[i128; 3]; 3],
    vector: [i64; 3],
) -> Result<[i64; 3], BallisticSphereError3d> {
    let scale = i128::from(ORIENTATION_SCALE);
    let mut output = [0_i64; 3];
    for (target, row) in output.iter_mut().zip(matrix) {
        let sum = checked_add(
            checked_add(
                checked_mul(row[0], i128::from(vector[0]))?,
                checked_mul(row[1], i128::from(vector[1]))?,
            )?,
            checked_mul(row[2], i128::from(vector[2]))?,
        )?;
        *target = i64::try_from(div_round_nearest(sum, scale)?)
            .map_err(|_| BallisticSphereError3d::ArithmeticOverflow)?;
    }
    Ok(output)
}

fn rotation_matrix(orientation: Orientation3d) -> Result<[[i128; 3]; 3], BallisticSphereError3d> {
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

fn transpose(matrix: [[i128; 3]; 3]) -> [[i128; 3]; 3] {
    [
        [matrix[0][0], matrix[1][0], matrix[2][0]],
        [matrix[0][1], matrix[1][1], matrix[2][1]],
        [matrix[0][2], matrix[1][2], matrix[2][2]],
    ]
}

fn scaled_twice(value: i128, scale: i128) -> Result<i128, BallisticSphereError3d> {
    div_round_nearest(checked_mul(value, 2)?, scale)
}

fn validate_timestep(
    timestep_numerator: i32,
    timestep_denominator: i32,
) -> Result<(), BallisticSphereError3d> {
    if timestep_numerator < 0 {
        return Err(BallisticSphereError3d::NegativeTimestepNumerator(
            timestep_numerator,
        ));
    }
    if timestep_denominator <= 0 {
        return Err(BallisticSphereError3d::NonPositiveTimestepDenominator(
            timestep_denominator,
        ));
    }
    Ok(())
}

fn add_scaled(
    current: i32,
    acceleration: i32,
    numerator: i128,
    denominator: i128,
) -> Result<i32, BallisticSphereError3d> {
    let delta = mul_div_round_i128(i128::from(acceleration), numerator, denominator)?;
    let next = checked_add(i128::from(current), delta)?;
    i32::try_from(next).map_err(|_| BallisticSphereError3d::ArithmeticOverflow)
}

fn advance_axis(
    position: i32,
    velocity: i32,
    numerator: i128,
    denominator: i128,
) -> Result<i32, BallisticSphereError3d> {
    let delta = mul_div_round_i128(i128::from(velocity), numerator, denominator)?;
    let next = checked_add(i128::from(position), delta)?;
    i32::try_from(next).map_err(|_| BallisticSphereError3d::ArithmeticOverflow)
}

fn div_round_nearest(numerator: i128, denominator: i128) -> Result<i128, BallisticSphereError3d> {
    if denominator <= 0 {
        return Err(BallisticSphereError3d::ArithmeticOverflow);
    }
    let half = denominator / 2;
    let adjusted = if numerator >= 0 {
        numerator.checked_add(half)
    } else {
        numerator.checked_sub(half)
    }
    .ok_or(BallisticSphereError3d::ArithmeticOverflow)?;
    Ok(adjusted / denominator)
}

fn div_floor_signed(numerator: i128, denominator: i128) -> Result<i128, BallisticSphereError3d> {
    if denominator <= 0 {
        return Err(BallisticSphereError3d::ArithmeticOverflow);
    }
    let quotient = numerator / denominator;
    let remainder = numerator % denominator;
    Ok(if remainder != 0 && numerator < 0 {
        quotient - 1
    } else {
        quotient
    })
}

fn div_ceil_signed(numerator: i128, denominator: i128) -> Result<i128, BallisticSphereError3d> {
    if denominator <= 0 {
        return Err(BallisticSphereError3d::ArithmeticOverflow);
    }
    let quotient = numerator / denominator;
    let remainder = numerator % denominator;
    Ok(if remainder != 0 && numerator > 0 {
        quotient + 1
    } else {
        quotient
    })
}

fn checked_mul(left: i128, right: i128) -> Result<i128, BallisticSphereError3d> {
    left.checked_mul(right)
        .ok_or(BallisticSphereError3d::ArithmeticOverflow)
}

fn checked_add(left: i128, right: i128) -> Result<i128, BallisticSphereError3d> {
    left.checked_add(right)
        .ok_or(BallisticSphereError3d::ArithmeticOverflow)
}

fn checked_sub(left: i128, right: i128) -> Result<i128, BallisticSphereError3d> {
    left.checked_sub(right)
        .ok_or(BallisticSphereError3d::ArithmeticOverflow)
}

fn gcd(mut left: u128, mut right: u128) -> u128 {
    while right != 0 {
        let remainder = left % right;
        left = right;
        right = remainder;
    }
    left.max(1)
}

#[cfg(test)]
mod tests {
    use crate::{
        AngularState3d, AngularVelocity3d, BallisticSphere3d, BallisticSphereQueryStats3d,
        BallisticSphereScene3d, BodyId, Orientation3d, RigidBody, RigidBox3d, Vec3i,
    };

    fn fixed_box(id: u64, position: Vec3i, half_extents: Vec3i) -> RigidBox3d {
        RigidBox3d::new(
            RigidBody::fixed(BodyId(id), position, half_extents),
            AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
        )
        .expect("valid fixed target")
    }

    #[test]
    fn fast_sphere_hits_thin_target_without_temporal_sampling() {
        let target = fixed_box(7, Vec3i::ZERO, Vec3i::new(40, 40, 1));
        let scene = BallisticSphereScene3d::prepare([&target]).expect("prepared target");
        let sphere = BallisticSphere3d::new(
            BodyId(1000),
            Vec3i::new(0, 0, 100),
            Vec3i::new(0, 0, -12_000),
            3,
            1,
        )
        .expect("valid sphere");
        let step = scene.prepare_step(1, 60).expect("prepared step");
        let mut stats = BallisticSphereQueryStats3d::default();
        let hit = step
            .earliest_hit(sphere, &mut stats)
            .expect("sphere sweep")
            .expect("thin target hit");

        assert_eq!(hit.body, BodyId(7));
        assert_eq!(hit.normal, [0, 0, 1]);
        assert!(hit.time.fraction_subticks() > 0);
        assert!(hit.time.fraction_subticks() < super::BALLISTIC_TIME_SCALE);
        assert_eq!(stats.broad_phase_candidates, 1);
        assert_eq!(stats.toi_tests, 1);
        assert_eq!(stats.feature_tests, 26);
    }

    #[test]
    fn corner_graze_does_not_use_expanded_box_false_positive() {
        let target = fixed_box(7, Vec3i::ZERO, Vec3i::new(10, 10, 10));
        let scene = BallisticSphereScene3d::prepare([&target]).expect("prepared target");
        let sphere = BallisticSphere3d::new(
            BodyId(1000),
            Vec3i::new(14, 14, 100),
            Vec3i::new(0, 0, -12_000),
            3,
            1,
        )
        .expect("valid sphere");
        let mut stats = BallisticSphereQueryStats3d::default();

        assert!(
            scene
                .earliest_hit(sphere, 1, 60, &mut stats)
                .expect("sphere sweep")
                .is_none()
        );
        assert_eq!(stats.broad_phase_candidates, 1);
        assert_eq!(stats.toi_tests, 1);
    }

    #[test]
    fn relative_target_motion_is_part_of_time_of_impact() {
        let moving = RigidBox3d::new(
            RigidBody::dynamic(
                BodyId(8),
                Vec3i::ZERO,
                Vec3i::new(0, 0, 600),
                Vec3i::new(20, 20, 2),
            ),
            AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
        )
        .expect("moving target");
        let stationary = fixed_box(9, Vec3i::ZERO, Vec3i::new(20, 20, 2));
        let sphere = BallisticSphere3d::new(
            BodyId(1000),
            Vec3i::new(0, 0, 100),
            Vec3i::new(0, 0, -6_000),
            3,
            1,
        )
        .expect("valid sphere");
        let moving_scene = BallisticSphereScene3d::prepare([&moving]).expect("moving scene");
        let stationary_scene = BallisticSphereScene3d::prepare([&stationary]).expect("fixed scene");
        let mut moving_stats = BallisticSphereQueryStats3d::default();
        let mut stationary_stats = BallisticSphereQueryStats3d::default();
        let moving_hit = moving_scene
            .earliest_hit(sphere, 1, 60, &mut moving_stats)
            .expect("moving sweep")
            .expect("moving hit");
        let stationary_hit = stationary_scene
            .earliest_hit(sphere, 1, 60, &mut stationary_stats)
            .expect("fixed sweep")
            .expect("fixed hit");

        assert!(moving_hit.time < stationary_hit.time);
    }

    #[test]
    fn broad_phase_rejects_distant_targets_before_toi_math() {
        let near = fixed_box(1, Vec3i::ZERO, Vec3i::new(10, 10, 2));
        let far = fixed_box(2, Vec3i::new(10_000, 0, 0), Vec3i::new(10, 10, 2));
        let scene = BallisticSphereScene3d::prepare([&near, &far]).expect("prepared scene");
        let sphere = BallisticSphere3d::new(
            BodyId(1000),
            Vec3i::new(0, 0, 100),
            Vec3i::new(0, 0, -12_000),
            2,
            1,
        )
        .expect("valid sphere");
        let mut stats = BallisticSphereQueryStats3d::default();
        let hit = scene
            .earliest_hit(sphere, 1, 60, &mut stats)
            .expect("sphere sweep")
            .expect("near hit");

        assert_eq!(hit.body, BodyId(1));
        assert_eq!(stats.broad_phase_candidates, 1);
        assert_eq!(stats.toi_tests, 1);
    }

    #[test]
    fn equal_time_hits_use_stable_body_id_order() {
        let high = fixed_box(20, Vec3i::ZERO, Vec3i::new(10, 10, 2));
        let low = fixed_box(10, Vec3i::ZERO, Vec3i::new(10, 10, 2));
        let scene = BallisticSphereScene3d::prepare([&high, &low]).expect("prepared scene");
        let sphere = BallisticSphere3d::new(
            BodyId(1000),
            Vec3i::new(0, 0, 100),
            Vec3i::new(0, 0, -12_000),
            2,
            1,
        )
        .expect("valid sphere");
        let mut stats = BallisticSphereQueryStats3d::default();
        let hit = scene
            .earliest_hit(sphere, 1, 60, &mut stats)
            .expect("sphere sweep")
            .expect("hit");

        assert_eq!(hit.body, BodyId(10));
    }
}
