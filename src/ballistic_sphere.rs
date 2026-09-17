use std::{cmp::Ordering, error::Error, fmt};

use crate::{
    AngularError3d, BodyId, CollisionLayers3d, Material, ORIENTATION_SCALE, Orientation3d,
    OrientedBoxError3d, RigidBox3d, Vec3i, oriented_box_vertices,
    wide_ratio::{WideRatioError, mul_div_round_i128},
};

/// Lightweight projectile state for bodies whose collision shape is rotation-invariant.
///
/// A ballistic sphere deliberately carries no orientation, angular velocity, box inertia, or persistent
/// contact state. Rotation cannot change its geometry, so continuous collision discovery can use an
/// analytic point-against-expanded-OBB sweep instead of sampled rotating-box CCD.
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

    /// Advances linearly over an exact rational timestep. Collision response should split the timestep at
    /// impact and call this method for each contact-free segment.
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

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct BallisticTime3d {
    pub numerator: u128,
    pub denominator: u128,
}

impl BallisticTime3d {
    #[must_use]
    pub fn compare(self, other: Self) -> Ordering {
        compare_unsigned_ratios(self, other).unwrap_or_else(|_| {
            // Public instances originate from checked engine arithmetic. The fallback keeps comparison
            // total for manually constructed values without allowing overflow to affect engine queries.
            self.numerator
                .saturating_mul(other.denominator)
                .cmp(&other.numerator.saturating_mul(self.denominator))
        })
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct BallisticSphereSweepHit3d {
    pub body: BodyId,
    pub time: BallisticTime3d,
    /// Fixed-point world-space normal pointing from the rigid target toward the incoming sphere.
    /// Components use [`ORIENTATION_SCALE`] units so rotated normals retain deterministic precision.
    pub normal: [i128; 3],
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct BallisticSphereQueryStats3d {
    /// Targets whose conservative swept world-space bounds overlapped the sphere sweep.
    pub broad_phase_candidates: u64,
    /// Analytic expanded-OBB slab tests actually executed.
    pub toi_tests: u64,
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
            Self::ZeroMass(id) => write!(
                formatter,
                "ballistic sphere {} requires non-zero mass",
                id.0
            ),
            Self::NegativeTimestepNumerator(value) => write!(
                formatter,
                "ballistic sphere timestep numerator must be non-negative, got {value}"
            ),
            Self::NonPositiveTimestepDenominator(value) => write!(
                formatter,
                "ballistic sphere timestep denominator must be positive, got {value}"
            ),
            Self::Geometry(error) => write!(formatter, "ballistic sphere target geometry failed: {error}"),
            Self::Angular(error) => write!(formatter, "ballistic sphere target orientation failed: {error}"),
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

/// One immutable rigid-scene preparation shared by every ballistic sphere query in a frame.
///
/// Target OBB orientation is intentionally frozen for the queried projectile substep. Relative linear
/// target motion is still included exactly. This makes the fast path suitable for fixed geometry, sleepers,
/// upright/rotation-locked bodies, and ordinary small-angle frame motion without pretending to solve
/// analytic rotating-OBB CCD. Callers that require exact interaction with rapidly rotating targets should
/// retain the general rotating rigid-body path for those bodies.
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

    pub fn earliest_hit(
        &self,
        sphere: BallisticSphere3d,
        timestep_numerator: i32,
        timestep_denominator: i32,
        stats: &mut BallisticSphereQueryStats3d,
    ) -> Result<Option<BallisticSphereSweepHit3d>, BallisticSphereError3d> {
        validate_timestep(timestep_numerator, timestep_denominator)?;
        if timestep_numerator == 0 {
            return Ok(None);
        }

        let sphere_bounds = swept_sphere_bounds(
            sphere,
            timestep_numerator,
            timestep_denominator,
        )?;
        let mut earliest = None;
        for target in &self.targets {
            if target.id == sphere.id
                || !sphere.collision_layers.collides_with(target.collision_layers)
                || !bounds_overlap(
                    sphere_bounds,
                    swept_target_bounds(target, timestep_numerator, timestep_denominator)?,
                )
            {
                continue;
            }
            stats.broad_phase_candidates = stats.broad_phase_candidates.saturating_add(1);
            stats.toi_tests = stats.toi_tests.saturating_add(1);
            let Some(hit) = swept_sphere_target(
                sphere,
                target,
                timestep_numerator,
                timestep_denominator,
            )? else {
                continue;
            };
            let replace = earliest.is_none_or(|current: BallisticSphereSweepHit3d| {
                hit.time.compare(current.time) == Ordering::Less
                    || (hit.time == current.time && hit.body < current.body)
            });
            if replace {
                earliest = Some(hit);
            }
        }
        Ok(earliest)
    }
}

fn prepare_target(rigid_box: &RigidBox3d) -> Result<PreparedBallisticTarget3d, BallisticSphereError3d> {
    let shape = rigid_box.oriented_box();
    let vertices = oriented_box_vertices(shape)?;
    let mut world_minimum = [i64::MAX; 3];
    let mut world_maximum = [i64::MIN; 3];
    for vertex in vertices {
        let axes = [i64::from(vertex.x), i64::from(vertex.y), i64::from(vertex.z)];
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
    target: &PreparedBallisticTarget3d,
    timestep_numerator: i32,
    timestep_denominator: i32,
) -> Result<Option<BallisticSphereSweepHit3d>, BallisticSphereError3d> {
    let relative_position = [
        i64::from(sphere.position.x) - i64::from(target.center.x),
        i64::from(sphere.position.y) - i64::from(target.center.y),
        i64::from(sphere.position.z) - i64::from(target.center.z),
    ];
    let relative_velocity = [
        i64::from(sphere.velocity.x) - i64::from(target.velocity.x),
        i64::from(sphere.velocity.y) - i64::from(target.velocity.y),
        i64::from(sphere.velocity.z) - i64::from(target.velocity.z),
    ];
    let inverse = transpose(target.rotation);
    let position = rotate_vector(inverse, relative_position)?;
    let velocity = rotate_vector(inverse, relative_velocity)?;
    let extent = [
        i64::from(target.half_extents.x)
            .checked_add(i64::from(sphere.radius))
            .ok_or(BallisticSphereError3d::ArithmeticOverflow)?,
        i64::from(target.half_extents.y)
            .checked_add(i64::from(sphere.radius))
            .ok_or(BallisticSphereError3d::ArithmeticOverflow)?,
        i64::from(target.half_extents.z)
            .checked_add(i64::from(sphere.radius))
            .ok_or(BallisticSphereError3d::ArithmeticOverflow)?,
    ];

    if (0..3).all(|axis| position[axis].abs() <= extent[axis]) {
        return Ok(None);
    }

    let mut global_entry: Option<SignedRatio> = None;
    let mut global_exit: Option<SignedRatio> = None;
    let mut impact_axis = 0_usize;
    for axis in 0..3 {
        if velocity[axis] == 0 {
            if position[axis].abs() > extent[axis] {
                return Ok(None);
            }
            continue;
        }

        let first = SignedRatio::new(
            -i128::from(extent[axis]) - i128::from(position[axis]),
            i128::from(velocity[axis]),
        )?;
        let second = SignedRatio::new(
            i128::from(extent[axis]) - i128::from(position[axis]),
            i128::from(velocity[axis]),
        )?;
        let (axis_entry, axis_exit) = if first.compare(second)? == Ordering::Greater {
            (second, first)
        } else {
            (first, second)
        };
        if global_entry.is_none_or(|entry| axis_entry.compare(entry).is_ok_and(|order| order == Ordering::Greater)) {
            global_entry = Some(axis_entry);
            impact_axis = axis;
        }
        if global_exit.is_none_or(|exit| axis_exit.compare(exit).is_ok_and(|order| order == Ordering::Less)) {
            global_exit = Some(axis_exit);
        }
    }

    let Some(entry) = global_entry else {
        return Ok(None);
    };
    let exit = global_exit.unwrap_or(SignedRatio::new(
        i128::from(timestep_numerator),
        i128::from(timestep_denominator),
    )?);
    let zero = SignedRatio::new(0, 1)?;
    let horizon = SignedRatio::new(
        i128::from(timestep_numerator),
        i128::from(timestep_denominator),
    )?;
    if entry.compare(exit)? == Ordering::Greater
        || exit.compare(zero)? == Ordering::Less
        || entry.compare(zero)? != Ordering::Greater
        || entry.compare(horizon)? == Ordering::Greater
    {
        return Ok(None);
    }

    let normal_sign = if velocity[impact_axis] > 0 { -1_i128 } else { 1_i128 };
    let normal = [
        target.rotation[0][impact_axis]
            .checked_mul(normal_sign)
            .ok_or(BallisticSphereError3d::ArithmeticOverflow)?,
        target.rotation[1][impact_axis]
            .checked_mul(normal_sign)
            .ok_or(BallisticSphereError3d::ArithmeticOverflow)?,
        target.rotation[2][impact_axis]
            .checked_mul(normal_sign)
            .ok_or(BallisticSphereError3d::ArithmeticOverflow)?,
    ];
    Ok(Some(BallisticSphereSweepHit3d {
        body: target.id,
        time: entry.to_public()?,
        normal,
    }))
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct SignedRatio {
    numerator: i128,
    denominator: i128,
}

impl SignedRatio {
    fn new(mut numerator: i128, mut denominator: i128) -> Result<Self, BallisticSphereError3d> {
        if denominator == 0 {
            return Err(BallisticSphereError3d::ArithmeticOverflow);
        }
        if denominator < 0 {
            numerator = numerator
                .checked_neg()
                .ok_or(BallisticSphereError3d::ArithmeticOverflow)?;
            denominator = denominator
                .checked_neg()
                .ok_or(BallisticSphereError3d::ArithmeticOverflow)?;
        }
        Ok(Self {
            numerator,
            denominator,
        })
    }

    fn compare(self, other: Self) -> Result<Ordering, BallisticSphereError3d> {
        let left = self
            .numerator
            .checked_mul(other.denominator)
            .ok_or(BallisticSphereError3d::ArithmeticOverflow)?;
        let right = other
            .numerator
            .checked_mul(self.denominator)
            .ok_or(BallisticSphereError3d::ArithmeticOverflow)?;
        Ok(left.cmp(&right))
    }

    fn to_public(self) -> Result<BallisticTime3d, BallisticSphereError3d> {
        if self.numerator <= 0 || self.denominator <= 0 {
            return Err(BallisticSphereError3d::ArithmeticOverflow);
        }
        let mut numerator = u128::try_from(self.numerator)
            .map_err(|_| BallisticSphereError3d::ArithmeticOverflow)?;
        let mut denominator = u128::try_from(self.denominator)
            .map_err(|_| BallisticSphereError3d::ArithmeticOverflow)?;
        let divisor = gcd(numerator, denominator);
        numerator /= divisor;
        denominator /= divisor;
        Ok(BallisticTime3d {
            numerator,
            denominator,
        })
    }
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

fn swept_sphere_bounds(
    sphere: BallisticSphere3d,
    timestep_numerator: i32,
    timestep_denominator: i32,
) -> Result<([i64; 3], [i64; 3]), BallisticSphereError3d> {
    let start = [
        i64::from(sphere.position.x),
        i64::from(sphere.position.y),
        i64::from(sphere.position.z),
    ];
    let end = advanced_axes(
        start,
        [
            i64::from(sphere.velocity.x),
            i64::from(sphere.velocity.y),
            i64::from(sphere.velocity.z),
        ],
        timestep_numerator,
        timestep_denominator,
    )?;
    let radius = i64::from(sphere.radius);
    let mut minimum = [0_i64; 3];
    let mut maximum = [0_i64; 3];
    for axis in 0..3 {
        minimum[axis] = start[axis]
            .min(end[axis])
            .checked_sub(radius)
            .ok_or(BallisticSphereError3d::ArithmeticOverflow)?;
        maximum[axis] = start[axis]
            .max(end[axis])
            .checked_add(radius)
            .ok_or(BallisticSphereError3d::ArithmeticOverflow)?;
    }
    Ok((minimum, maximum))
}

fn swept_target_bounds(
    target: &PreparedBallisticTarget3d,
    timestep_numerator: i32,
    timestep_denominator: i32,
) -> Result<([i64; 3], [i64; 3]), BallisticSphereError3d> {
    let delta = advanced_axes(
        [0; 3],
        [
            i64::from(target.velocity.x),
            i64::from(target.velocity.y),
            i64::from(target.velocity.z),
        ],
        timestep_numerator,
        timestep_denominator,
    )?;
    let mut minimum = [0_i64; 3];
    let mut maximum = [0_i64; 3];
    for axis in 0..3 {
        minimum[axis] = target.world_minimum[axis].min(
            target.world_minimum[axis]
                .checked_add(delta[axis])
                .ok_or(BallisticSphereError3d::ArithmeticOverflow)?,
        );
        maximum[axis] = target.world_maximum[axis].max(
            target.world_maximum[axis]
                .checked_add(delta[axis])
                .ok_or(BallisticSphereError3d::ArithmeticOverflow)?,
        );
    }
    Ok((minimum, maximum))
}

fn bounds_overlap(left: ([i64; 3], [i64; 3]), right: ([i64; 3], [i64; 3])) -> bool {
    (0..3).all(|axis| left.0[axis] <= right.1[axis] && right.0[axis] <= left.1[axis])
}

fn advanced_axes(
    start: [i64; 3],
    velocity: [i64; 3],
    timestep_numerator: i32,
    timestep_denominator: i32,
) -> Result<[i64; 3], BallisticSphereError3d> {
    let mut result = start;
    for axis in 0..3 {
        let delta = mul_div_round_i128(
            i128::from(velocity[axis]),
            i128::from(timestep_numerator),
            i128::from(timestep_denominator),
        )?;
        result[axis] = result[axis]
            .checked_add(i64::try_from(delta).map_err(|_| BallisticSphereError3d::ArithmeticOverflow)?)
            .ok_or(BallisticSphereError3d::ArithmeticOverflow)?;
    }
    Ok(result)
}

fn add_scaled(
    current: i32,
    acceleration: i32,
    numerator: i128,
    denominator: i128,
) -> Result<i32, BallisticSphereError3d> {
    let delta = mul_div_round_i128(i128::from(acceleration), numerator, denominator)?;
    let next = i128::from(current)
        .checked_add(delta)
        .ok_or(BallisticSphereError3d::ArithmeticOverflow)?;
    i32::try_from(next).map_err(|_| BallisticSphereError3d::ArithmeticOverflow)
}

fn advance_axis(
    position: i32,
    velocity: i32,
    numerator: i128,
    denominator: i128,
) -> Result<i32, BallisticSphereError3d> {
    let delta = mul_div_round_i128(i128::from(velocity), numerator, denominator)?;
    let next = i128::from(position)
        .checked_add(delta)
        .ok_or(BallisticSphereError3d::ArithmeticOverflow)?;
    i32::try_from(next).map_err(|_| BallisticSphereError3d::ArithmeticOverflow)
}

fn rotation_matrix(
    orientation: Orientation3d,
) -> Result<[[i128; 3]; 3], BallisticSphereError3d> {
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

fn rotate_vector(
    matrix: [[i128; 3]; 3],
    vector: [i64; 3],
) -> Result<[i64; 3], BallisticSphereError3d> {
    let scale = i128::from(ORIENTATION_SCALE);
    let mut result = [0_i64; 3];
    for (target, row) in result.iter_mut().zip(matrix) {
        let sum = checked_add(
            checked_add(
                checked_mul(row[0], i128::from(vector[0]))?,
                checked_mul(row[1], i128::from(vector[1]))?,
            )?,
            checked_mul(row[2], i128::from(vector[2]))?,
        )?;
        let value = div_round_nearest(sum, scale)?;
        *target = i64::try_from(value).map_err(|_| BallisticSphereError3d::ArithmeticOverflow)?;
    }
    Ok(result)
}

fn scaled_twice(value: i128, scale: i128) -> Result<i128, BallisticSphereError3d> {
    div_round_nearest(checked_mul(value, 2)?, scale)
}

fn div_round_nearest(
    numerator: i128,
    denominator: i128,
) -> Result<i128, BallisticSphereError3d> {
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

fn compare_unsigned_ratios(
    left: BallisticTime3d,
    right: BallisticTime3d,
) -> Result<Ordering, BallisticSphereError3d> {
    if left.denominator == 0 || right.denominator == 0 {
        return Err(BallisticSphereError3d::ArithmeticOverflow);
    }
    let left_cross = left
        .numerator
        .checked_mul(right.denominator)
        .ok_or(BallisticSphereError3d::ArithmeticOverflow)?;
    let right_cross = right
        .numerator
        .checked_mul(left.denominator)
        .ok_or(BallisticSphereError3d::ArithmeticOverflow)?;
    Ok(left_cross.cmp(&right_cross))
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
        let target = fixed_box(7, Vec3i::new(0, 0, 0), Vec3i::new(40, 40, 1));
        let scene = BallisticSphereScene3d::prepare([&target]).expect("prepared target");
        let sphere = BallisticSphere3d::new(
            BodyId(1000),
            Vec3i::new(0, 0, 100),
            Vec3i::new(0, 0, -12_000),
            3,
            1,
        )
        .expect("valid sphere");
        let mut stats = BallisticSphereQueryStats3d::default();
        let hit = scene
            .earliest_hit(sphere, 1, 60, &mut stats)
            .expect("analytic sweep")
            .expect("thin target hit");

        assert_eq!(hit.body, BodyId(7));
        assert_eq!(hit.time.numerator, 2);
        assert_eq!(hit.time.denominator, 250);
        assert_eq!(hit.normal, [0, 0, i128::from(ORIENTATION_SCALE)]);
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

        assert_eq!(moving_hit.time.compare(stationary_hit.time), Ordering::Less);
    }

    #[test]
    fn broad_phase_rejects_distant_targets_before_toi_math() {
        let near = fixed_box(1, Vec3i::new(0, 0, 0), Vec3i::new(10, 10, 2));
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
            .expect("analytic sweep")
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
            .expect("analytic sweep")
            .expect("hit");

        assert_eq!(hit.body, BodyId(10));
    }
}
