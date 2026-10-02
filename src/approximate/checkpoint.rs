//! Explicit, validated physical products. Nothing here runs during ordinary stepping.
use std::collections::BTreeMap;

use sha2::{Digest, Sha256};

use super::{
    Body, BodyId, CachedPoint, CollisionLayers3d, Config, Convergence, ConvergenceScope,
    PositionCorrection, Quaternion, REAL_BITS, Real, Scalar, Shape, Vector, World, contact,
    narrow_time, substep_length, widen_time,
};

// Each scalar width has its own magic; the layout is otherwise shared. Physical
// values use `Real` at native width; elapsed/prior-substep time is always f64.
const MAGIC_F64: &[u8; 8] = b"PEFLOAT\0";
const MAGIC_F32: &[u8; 8] = b"PEFLT32\0";
const MAGIC: &[u8; 8] = if REAL_BITS == 32 {
    MAGIC_F32
} else {
    MAGIC_F64
};
const FORMAT: u32 = 2;
// Bump when continuation semantics change, even if the byte layout does not.
const ALGORITHM: u32 = 8;
const DIGEST_BYTES: usize = 32;
const REAL_BYTES: usize = size_of::<Real>();
// Fixed fields: id 8, shape tag 1, flags 2, layers 8, support tag 1; 30 physical scalars.
const MIN_BODY_BYTES: usize = 20 + 30 * REAL_BYTES;
const POINT_BYTES: usize = 13 * REAL_BYTES;
// Conservative envelopes for stored normalization/rotated-normal results. The
// existing normalization can lose unit length for extreme finite input scales;
// a checkpoint preserves that accepted state rather than silently repairing it.
const ORIENTATION_COMPONENT_LIMIT: Real = 2.0;
const NORMAL_COMPONENT_LIMIT: Real = 64.0;

/// Consumer-owned compatibility identities. The build tag must identify the exact
/// engine/dependency/compiler/flags/ABI policy; content identifies static content
/// and the consumer's BodyId mapping. Equal tags do not establish portability.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CheckpointContext {
    pub build: [u8; 32],
    pub content: [u8; 32],
}

/// Explicit resource budgets for decoding untrusted or damaged input. These are
/// checkpoint admission limits, not limits on the number of simulated bodies.
#[derive(Clone, Copy, Debug)]
pub struct CheckpointLimits {
    pub bytes: usize,
    pub bodies: usize,
    pub pairs: usize,
    pub contact_points: usize,
}
impl Default for CheckpointLimits {
    fn default() -> Self {
        Self {
            bytes: 64 * 1024 * 1024,
            bodies: 100_000,
            pairs: 200_000,
            contact_points: 800_000,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CheckpointError {
    InvalidData,
    IntegrityMismatch,
    UnsupportedFormat(u32),
    UnsupportedAlgorithm(u32),
    IncompatibleContext,
    IncompatibleTarget,
    UnsupportedPolicy,
    ResourceLimit,
    /// The bytes come from a build whose floating-solver scalar width differs
    /// (`f32-physics` versus the default f64 build). Rejected before the version fields.
    ScalarWidthMismatch {
        expected_bits: u32,
        found_bits: u32,
    },
}
impl std::fmt::Display for CheckpointError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "physical checkpoint rejected: {self:?}")
    }
}
impl std::error::Error for CheckpointError {}

/// Physical vector payload only, excluding tree nodes, allocator overhead,
/// inline configuration and disposable acceleration. Encoding allocates separately.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CheckpointStats {
    pub bodies: usize,
    pub pairs: usize,
    pub contact_points: usize,
    pub vector_capacity_bytes: usize,
}

/// Immutable, continuation-complete physical product at an explicit call boundary.
/// Reports/events, scratch and consumer gameplay state are deliberately excluded.
#[derive(Clone, Debug)]
pub struct Checkpoint {
    context: CheckpointContext,
    config: Config,
    elapsed: Scalar,
    last_h: Scalar,
    bodies: Vec<Body>,
    cache: BTreeMap<(BodyId, BodyId), Vec<CachedPoint>>,
}

impl World {
    /// Capture exact floating state/history, including queued input. This explicit
    /// product copies all bodies and contact history; ordinary step never calls it.
    pub fn checkpoint(&self, context: CheckpointContext) -> Result<Checkpoint, CheckpointError> {
        validate(
            self.config,
            self.elapsed,
            self.last_h,
            &self.bodies,
            &self.cache,
        )?;
        Ok(Checkpoint {
            context,
            config: self.config,
            elapsed: self.elapsed,
            last_h: self.last_h,
            bodies: self.bodies.clone(),
            cache: self.cache.clone(),
        })
    }
}

impl Checkpoint {
    pub fn stats(&self) -> CheckpointStats {
        CheckpointStats {
            bodies: self.bodies.len(),
            pairs: self.cache.len(),
            contact_points: self.cache.values().map(Vec::len).sum(),
            vector_capacity_bytes: self.bodies.capacity() * size_of::<Body>()
                + self
                    .cache
                    .values()
                    .map(|p| p.capacity() * size_of::<CachedPoint>())
                    .sum::<usize>(),
        }
    }

    /// Consume the validated product into one new mutable authority. Orientation
    /// is not normalized again. Derived bounds/views rebuild; the report starts empty
    /// so restoring cannot re-emit already completed events. Cold-cache work can differ.
    pub fn restore(self) -> World {
        let mut world = World::new(self.config).expect("checkpoint configuration was validated");
        world.elapsed = self.elapsed;
        world.last_h = self.last_h;
        world.bodies = self.bodies;
        world.cache = self.cache;
        for body in &mut world.bodies {
            body.cached_bounds = contact::bounds(body);
        }
        world.bookkeeping.activity.count = world
            .bodies
            .iter()
            .filter(|b| b.mass > 0.0 && !b.sleeping)
            .count();
        world.bookkeeping.layout_changed();
        world
    }

    /// Canonical little-endian encoding of exact `Real` bits (f64 time). SHA-256 detects damaged
    /// bytes; authentication/persistence and consumer snapshot atomicity stay outside physics.
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut out = Vec::new();
        out.extend_from_slice(MAGIC);
        out.extend_from_slice(&FORMAT.to_le_bytes());
        out.extend_from_slice(&ALGORITHM.to_le_bytes());
        out.extend_from_slice(&self.context.build);
        out.extend_from_slice(&self.context.content);
        for target in [std::env::consts::ARCH, std::env::consts::OS] {
            // Rust's fixed target names fit in the format's one-byte length.
            out.push(u8::try_from(target.len()).expect("Rust target name fits in u8"));
            out.extend_from_slice(target.as_bytes());
        }
        encode_config(&mut out, self.config);
        time(&mut out, self.elapsed);
        time(&mut out, self.last_h);
        count(&mut out, self.bodies.len());
        for body in &self.bodies {
            encode_body(&mut out, body);
        }
        count(&mut out, self.cache.len());
        for (&(a, b), points) in &self.cache {
            out.extend_from_slice(&a.0.to_le_bytes());
            out.extend_from_slice(&b.0.to_le_bytes());
            count(&mut out, points.len());
            for point in points {
                vector(&mut out, point.a);
                vector(&mut out, point.b);
                vector(&mut out, point.normal);
                scalar(&mut out, point.impulse);
                vector(&mut out, point.tangent);
            }
        }
        let digest = Sha256::digest(&out);
        out.extend_from_slice(&digest);
        out
    }

    /// Fully validate before creating a restorable product. Unknown layouts,
    /// target/build/content mismatches, damaged/truncated bytes and exhausted
    /// caller budgets fail without touching any live world. Allocation aborts are excluded.
    pub fn from_bytes(
        bytes: &[u8],
        expected: CheckpointContext,
        limits: CheckpointLimits,
    ) -> Result<Self, CheckpointError> {
        if bytes.len() > limits.bytes {
            return Err(CheckpointError::ResourceLimit);
        }
        let payload_len = bytes
            .len()
            .checked_sub(DIGEST_BYTES)
            .ok_or(CheckpointError::InvalidData)?;
        let (payload, digest) = bytes.split_at(payload_len);
        let mut reader = Reader { bytes: payload };
        match &reader.array::<8>()? {
            magic if magic == MAGIC => {}
            magic if magic == MAGIC_F64 || magic == MAGIC_F32 => {
                return Err(CheckpointError::ScalarWidthMismatch {
                    expected_bits: REAL_BITS,
                    found_bits: if magic == MAGIC_F32 { 32 } else { 64 },
                });
            }
            _ => return Err(CheckpointError::InvalidData),
        }
        let format = reader.u32()?;
        if format != FORMAT {
            return Err(CheckpointError::UnsupportedFormat(format));
        }
        let algorithm = reader.u32()?;
        if algorithm != ALGORITHM {
            return Err(CheckpointError::UnsupportedAlgorithm(algorithm));
        }
        if Sha256::digest(payload).as_slice() != digest {
            return Err(CheckpointError::IntegrityMismatch);
        }
        let context = CheckpointContext {
            build: reader.array()?,
            content: reader.array()?,
        };
        if context != expected {
            return Err(CheckpointError::IncompatibleContext);
        }
        for target in [std::env::consts::ARCH, std::env::consts::OS] {
            let length = usize::from(reader.u8()?);
            if reader.take(length)? != target.as_bytes() {
                return Err(CheckpointError::IncompatibleTarget);
            }
        }
        let config = reader.config()?;
        let elapsed = reader.time()?;
        let last_h = reader.time()?;
        let body_count = reader.count(limits.bodies, MIN_BODY_BYTES)?;
        let mut bodies = Vec::with_capacity(body_count);
        for _ in 0..body_count {
            let body = reader.body()?;
            if bodies.last().is_some_and(|b: &Body| b.id >= body.id) {
                return Err(CheckpointError::InvalidData);
            }
            bodies.push(body);
        }
        let pair_count = reader.count(limits.pairs, 24 + POINT_BYTES)?;
        let mut cache = BTreeMap::new();
        let mut previous = None;
        let mut total_points = 0usize;
        for _ in 0..pair_count {
            let key = (BodyId(reader.u64()?), BodyId(reader.u64()?));
            if key.0 >= key.1 || previous.is_some_and(|p| p >= key) {
                return Err(CheckpointError::InvalidData);
            }
            previous = Some(key);
            let points_count = reader.count(limits.contact_points - total_points, POINT_BYTES)?;
            if points_count == 0 {
                return Err(CheckpointError::InvalidData);
            }
            total_points += points_count;
            let mut points = Vec::with_capacity(points_count);
            for _ in 0..points_count {
                points.push(CachedPoint {
                    a: reader.vector()?,
                    b: reader.vector()?,
                    normal: reader.vector()?,
                    impulse: reader.scalar()?,
                    tangent: reader.vector()?,
                });
            }
            cache.insert(key, points);
        }
        if !reader.bytes.is_empty() {
            return Err(CheckpointError::InvalidData);
        }
        validate(config, elapsed, last_h, &bodies, &cache)?;
        Ok(Self {
            context,
            config,
            elapsed,
            last_h,
            bodies,
            cache,
        })
    }
}

fn validate(
    config: Config,
    elapsed: Scalar,
    last_h: Scalar,
    bodies: &[Body],
    cache: &BTreeMap<(BodyId, BodyId), Vec<CachedPoint>>,
) -> Result<(), CheckpointError> {
    World::new(config).map_err(|_| CheckpointError::InvalidData)?;
    if !elapsed.is_finite()
        || elapsed < 0.0
        || !last_h.is_finite()
        // The largest substep, as stored: narrowed to `Real` and widened back to f64.
        || !(0.0..=widen_time(substep_length(0.1, config.substeps))).contains(&last_h)
        // `last_h` is the narrowed substep; rounding is monotonic, so compare at that width.
        || (last_h > 0.0 && narrow_time(elapsed) < narrow_time(last_h))
        || bodies.windows(2).any(|p| p[0].id >= p[1].id)
    {
        return Err(CheckpointError::InvalidData);
    }
    for body in bodies {
        if !body.valid()
            || !valid_orientation(body.orientation)
            || !body.quiet_time.is_finite()
            || body.quiet_time < 0.0
            || [body.force, body.torque, body.impulse, body.angular_impulse]
                .into_iter()
                .any(|v| !v.finite())
            || body.linear_support.is_some_and(|v| !v.finite())
        {
            return Err(CheckpointError::InvalidData);
        }
    }
    for (&(a, b), points) in cache {
        if a >= b
            || points.is_empty()
            || [a, b]
                .into_iter()
                .any(|id| bodies.binary_search_by_key(&id, |b| b.id).is_err())
            || points.iter().any(|p| {
                !p.a.finite()
                    || !p.b.finite()
                    || !p.normal.finite()
                    || p.normal.abs().max_component() > NORMAL_COMPONENT_LIMIT
                    || !p.impulse.is_finite()
                    || p.impulse < 0.0
                    || !p.tangent.finite()
            })
        {
            return Err(CheckpointError::InvalidData);
        }
    }
    Ok(())
}

fn valid_orientation(q: Quaternion) -> bool {
    [q.0, q.1, q.2, q.3]
        .into_iter()
        .all(|x| x.abs() <= ORIENTATION_COMPONENT_LIMIT)
}

fn count(out: &mut Vec<u8>, value: usize) {
    out.extend_from_slice(&(value as u64).to_le_bytes());
}
fn scalar(out: &mut Vec<u8>, value: Real) {
    out.extend_from_slice(&value.to_bits().to_le_bytes());
}
fn time(out: &mut Vec<u8>, value: Scalar) {
    out.extend_from_slice(&value.to_bits().to_le_bytes());
}
fn vector(out: &mut Vec<u8>, v: Vector) {
    for x in [v.0, v.1, v.2] {
        scalar(out, x);
    }
}
fn optional_vector(out: &mut Vec<u8>, v: Option<Vector>) {
    out.push(u8::from(v.is_some()));
    if let Some(v) = v {
        vector(out, v);
    }
}
fn encode_config(out: &mut Vec<u8>, config: Config) {
    vector(out, config.gravity);
    out.extend_from_slice(&[
        config.substeps,
        config.velocity_iterations,
        config.fixed_position_iterations,
    ]);
    for x in [
        config.contact_slop,
        config.sleep_speed,
        config.sleep_seconds,
    ] {
        scalar(out, x);
    }
    out.push(u8::from(config.warm_start));
    out.push(u8::from(config.convergence.is_some()));
    if let Some(c) = config.convergence {
        for x in [c.absolute_velocity, c.absolute_impulse, c.relative] {
            scalar(out, x);
        }
    }
    out.push(match config.convergence_scope {
        ConvergenceScope::WholeWorld => 0,
        ConvergenceScope::ContactIslands => 1,
    });
    let position =
        u8::from(config.position_correction == PositionCorrection::AdmittedContacts) << 1;
    #[cfg(not(feature = "experimental-soft-contact"))]
    out.push(position);
    #[cfg(feature = "experimental-soft-contact")]
    {
        out.push(position | u8::from(config.soft_contact.is_some()));
        if let Some(c) = config.soft_contact {
            scalar(out, c.frequency_hz);
            scalar(out, c.damping_ratio);
            out.push(c.relaxation_iterations);
        }
    }
}
fn encode_body(out: &mut Vec<u8>, b: &Body) {
    out.extend_from_slice(&b.id.0.to_le_bytes());
    vector(out, b.position);
    vector(out, b.velocity);
    for x in [
        b.orientation.0,
        b.orientation.1,
        b.orientation.2,
        b.orientation.3,
    ] {
        scalar(out, x);
    }
    vector(out, b.angular_velocity);
    match b.shape {
        Shape::Box(h) => {
            out.push(0);
            vector(out, h);
        }
        Shape::Sphere(r) => {
            out.push(1);
            scalar(out, r);
        }
        Shape::Capsule {
            half_segment,
            radius,
        } => {
            out.push(2);
            scalar(out, half_segment);
            scalar(out, radius);
        }
        Shape::Wedge(h) => {
            out.push(3);
            vector(out, h);
        }
    }
    for x in [b.mass, b.friction, b.restitution] {
        scalar(out, x);
    }
    let flags = [
        b.rotation_locked,
        b.external,
        b.sensor,
        b.sleep_allowed,
        b.ccd,
        b.retire_on_impact,
        b.sleeping,
    ]
    .into_iter()
    .enumerate()
    .fold(0u16, |bits, (i, yes)| bits | (u16::from(yes) << i));
    out.extend_from_slice(&flags.to_le_bytes());
    out.extend_from_slice(&b.layers.memberships().to_le_bytes());
    out.extend_from_slice(&b.layers.mask().to_le_bytes());
    optional_vector(out, b.linear_support);
    scalar(out, b.quiet_time);
    for v in [b.force, b.torque, b.impulse, b.angular_impulse] {
        vector(out, v);
    }
}

struct Reader<'a> {
    bytes: &'a [u8],
}
impl<'a> Reader<'a> {
    fn take(&mut self, length: usize) -> Result<&'a [u8], CheckpointError> {
        let value = self
            .bytes
            .get(..length)
            .ok_or(CheckpointError::InvalidData)?;
        self.bytes = &self.bytes[length..];
        Ok(value)
    }
    fn array<const N: usize>(&mut self) -> Result<[u8; N], CheckpointError> {
        self.take(N)?
            .try_into()
            .map_err(|_| CheckpointError::InvalidData)
    }
    fn u8(&mut self) -> Result<u8, CheckpointError> {
        Ok(self.array::<1>()?[0])
    }
    fn u16(&mut self) -> Result<u16, CheckpointError> {
        Ok(u16::from_le_bytes(self.array()?))
    }
    fn u32(&mut self) -> Result<u32, CheckpointError> {
        Ok(u32::from_le_bytes(self.array()?))
    }
    fn u64(&mut self) -> Result<u64, CheckpointError> {
        Ok(u64::from_le_bytes(self.array()?))
    }
    fn scalar(&mut self) -> Result<Real, CheckpointError> {
        Ok(Real::from_le_bytes(self.array()?))
    }
    fn time(&mut self) -> Result<Scalar, CheckpointError> {
        Ok(Scalar::from_bits(self.u64()?))
    }
    fn vector(&mut self) -> Result<Vector, CheckpointError> {
        Ok(Vector(self.scalar()?, self.scalar()?, self.scalar()?))
    }
    fn boolean(&mut self) -> Result<bool, CheckpointError> {
        match self.u8()? {
            0 => Ok(false),
            1 => Ok(true),
            _ => Err(CheckpointError::InvalidData),
        }
    }
    fn optional_vector(&mut self) -> Result<Option<Vector>, CheckpointError> {
        if self.boolean()? {
            Ok(Some(self.vector()?))
        } else {
            Ok(None)
        }
    }
    fn count(&mut self, limit: usize, minimum: usize) -> Result<usize, CheckpointError> {
        let count = usize::try_from(self.u64()?).map_err(|_| CheckpointError::ResourceLimit)?;
        if count > limit {
            return Err(CheckpointError::ResourceLimit);
        }
        if count > self.bytes.len() / minimum {
            return Err(CheckpointError::InvalidData);
        }
        Ok(count)
    }
    fn config(&mut self) -> Result<Config, CheckpointError> {
        let mut config = Config {
            gravity: self.vector()?,
            substeps: self.u8()?,
            velocity_iterations: self.u8()?,
            fixed_position_iterations: self.u8()?,
            position_correction: PositionCorrection::FixedColliders,
            contact_slop: self.scalar()?,
            sleep_speed: self.scalar()?,
            sleep_seconds: self.scalar()?,
            warm_start: self.boolean()?,
            convergence: if self.boolean()? {
                Some(Convergence {
                    absolute_velocity: self.scalar()?,
                    absolute_impulse: self.scalar()?,
                    relative: self.scalar()?,
                })
            } else {
                None
            },
            convergence_scope: match self.u8()? {
                0 => ConvergenceScope::WholeWorld,
                1 => ConvergenceScope::ContactIslands,
                _ => return Err(CheckpointError::InvalidData),
            },
            #[cfg(feature = "experimental-soft-contact")]
            soft_contact: None,
        };
        let policies = self.u8()?;
        if policies & !3 != 0 {
            return Err(CheckpointError::InvalidData);
        }
        if policies & 2 != 0 {
            config.position_correction = PositionCorrection::AdmittedContacts;
        }
        let soft = policies & 1 != 0;
        #[cfg(not(feature = "experimental-soft-contact"))]
        if soft {
            return Err(CheckpointError::UnsupportedPolicy);
        }
        #[cfg(feature = "experimental-soft-contact")]
        if soft {
            config.soft_contact = Some(super::SoftContact {
                frequency_hz: self.scalar()?,
                damping_ratio: self.scalar()?,
                relaxation_iterations: self.u8()?,
            });
        }
        Ok(config)
    }
    fn body(&mut self) -> Result<Body, CheckpointError> {
        let id = BodyId(self.u64()?);
        let position = self.vector()?;
        let velocity = self.vector()?;
        let orientation = Quaternion(
            self.scalar()?,
            self.scalar()?,
            self.scalar()?,
            self.scalar()?,
        );
        let angular_velocity = self.vector()?;
        let shape = match self.u8()? {
            0 => Shape::Box(self.vector()?),
            1 => Shape::Sphere(self.scalar()?),
            2 => Shape::capsule(self.scalar()?, self.scalar()?),
            3 => Shape::Wedge(self.vector()?),
            _ => return Err(CheckpointError::InvalidData),
        };
        let mut body = Body::new(id, shape, position, self.scalar()?);
        body.velocity = velocity;
        body.orientation = orientation;
        body.angular_velocity = angular_velocity;
        body.friction = self.scalar()?;
        body.restitution = self.scalar()?;
        let flags = self.u16()?;
        if flags & !0x7f != 0 {
            return Err(CheckpointError::InvalidData);
        }
        body.rotation_locked = flags & 1 != 0;
        body.external = flags & 2 != 0;
        body.sensor = flags & 4 != 0;
        body.sleep_allowed = flags & 8 != 0;
        body.ccd = flags & 16 != 0;
        body.retire_on_impact = flags & 32 != 0;
        body.sleeping = flags & 64 != 0;
        body.layers = CollisionLayers3d::new(self.u32()?, self.u32()?);
        body.linear_support = self.optional_vector()?;
        body.quiet_time = self.scalar()?;
        body.force = self.vector()?;
        body.torque = self.vector()?;
        body.impulse = self.vector()?;
        body.angular_impulse = self.vector()?;
        Ok(body)
    }
}
