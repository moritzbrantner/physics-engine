//! Explicit f64 fixed-step approximation, alongside (not silently replacing) the event solver.
//!
//! External impulses change velocity once; forces act for the requested duration. Each bounded
//! substep prepares contacts, accumulates clamped sequential impulses, then integrates the new
//! linear/angular velocity. Contact impulses persist for warm starting. No integer quantization,
//! rational remainder tree, event-restart loop, or whole-world sleep proxy conversion is used.
//!
//! CCD sweeps translation over each substep for fast shapes. Orientations are held fixed during
//! those sweeps and integrated between substeps, so this is NOT analytic rotational CCD.
mod bookkeeping;
mod contact;
mod position;
mod primitive;
pub use position::PositionReport;
mod convergence;
mod islands;
pub use convergence::{Convergence, ConvergenceStats};
pub use islands::{ConvergenceScope, IslandStats};
mod geometry;
pub use bookkeeping::BookkeepingStats;
use bookkeeping::{Scratch, SweepRow, push, reserve};
use geometry::GeometryCache;
pub use geometry::GeometryStats;
mod math;
mod response;
use crate::{
    BodyId, BodyKind, CollisionLayers3d, ContactMode3d, MotionAuthority3d, RigidBox3d, SleepMode3d,
    SolverParticipation3d, numeric,
};
pub use math::{Quaternion, Vector};
use numeric::Scalar;
use response::PreparedResponse;
use std::collections::BTreeMap;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Shape {
    Box(Vector),
    Sphere(Scalar),
    /// Local-Y segment expanded by a spherical radius.
    Capsule {
        half_segment: Scalar,
        radius: Scalar,
    },
    /// Right triangular prism inside the local bounding box. The ramp rises toward -X.
    Wedge(Vector),
}
impl Shape {
    pub const fn capsule(half_segment: Scalar, radius: Scalar) -> Self {
        Self::Capsule {
            half_segment,
            radius,
        }
    }
    pub const fn wedge(half_extents: Vector) -> Self {
        Self::Wedge(half_extents)
    }
    pub fn radius(self) -> Scalar {
        match self {
            Self::Box(h) | Self::Wedge(h) => h.length(),
            Self::Sphere(r) => r,
            Self::Capsule {
                half_segment,
                radius,
            } => half_segment + radius,
        }
    }
    pub fn half_extents(self) -> Vector {
        match self {
            Self::Box(h) | Self::Wedge(h) => h,
            Self::Sphere(r) => Vector(r, r, r),
            Self::Capsule {
                half_segment,
                radius,
            } => Vector(radius, half_segment + radius, radius),
        }
    }
    fn valid_dimensions(self) -> bool {
        match self {
            Self::Box(h) | Self::Wedge(h) => {
                h.finite() && h.min_component() >= 1e-6 && h.max_component() < 1e12
            }
            Self::Sphere(radius) => radius.is_finite() && (1e-6..1e12).contains(&radius),
            Self::Capsule {
                half_segment,
                radius,
            } => {
                half_segment.is_finite()
                    && half_segment >= 0.0
                    && half_segment < 1e12
                    && radius.is_finite()
                    && (1e-6..1e12).contains(&radius)
            }
        }
    }
    fn local_inverse_inertia(self, mass: Scalar) -> Option<Vector> {
        match self {
            Self::Sphere(r) => {
                let inverse = 2.5 / (mass * r * r);
                Some(Vector(inverse, inverse, inverse))
            }
            Self::Box(h) => Some(Vector(
                3.0 / (mass * (h.1 * h.1 + h.2 * h.2)),
                3.0 / (mass * (h.0 * h.0 + h.2 * h.2)),
                3.0 / (mass * (h.0 * h.0 + h.1 * h.1)),
            )),
            Self::Capsule {
                half_segment: h,
                radius: r,
            } => {
                let cylinder_volume = r * r * (2.0 * h);
                let sphere_volume = (4.0 / 3.0) * r * r * r;
                let total_volume = cylinder_volume + sphere_volume;
                let cylinder_mass = mass * cylinder_volume / total_volume;
                let sphere_mass = mass * sphere_volume / total_volume;
                let axial = cylinder_mass * r * r * 0.5
                    + sphere_mass * r * r * (2.0 / 5.0);
                let radial = cylinder_mass * (3.0 * r * r + 4.0 * h * h) / 12.0
                    + sphere_mass
                        * ((2.0 / 5.0) * r * r + h * h + (3.0 / 4.0) * h * r);
                Some(Vector(1.0 / radial, 1.0 / axial, 1.0 / radial))
            }
            // Solver-owned dynamic wedges are required to be rotation locked until a COM-centered
            // wedge inertia/pose contract is introduced.
            Self::Wedge(_) => None,
        }
    }
}
#[derive(Clone, Debug, PartialEq)]
pub struct Body {
    pub id: BodyId,
    pub position: Vector,
    pub velocity: Vector,
    pub orientation: Quaternion,
    /// Radians per second in world coordinates.
    pub angular_velocity: Vector,
    pub shape: Shape,
    /// Zero is immovable geometry; positive mass is dynamic unless `external` is set.
    pub mass: Scalar,
    pub friction: Scalar,
    pub restitution: Scalar,
    pub rotation_locked: bool,
    pub layers: CollisionLayers3d,
    pub external: bool,
    pub sensor: bool,
    pub sleep_allowed: bool,
    pub ccd: bool,
    pub retire_on_impact: bool,
    pub linear_support: Option<Vector>,
    sleeping: bool,
    quiet_time: Scalar,
    force: Vector,
    torque: Vector,
    impulse: Vector,
    angular_impulse: Vector,
    cached_bounds: (Vector, Vector),
}
impl Body {
    pub fn new(id: BodyId, shape: Shape, position: Vector, mass: Scalar) -> Self {
        Self {
            id,
            shape,
            position,
            mass,
            velocity: Vector::ZERO,
            orientation: Quaternion::IDENTITY,
            angular_velocity: Vector::ZERO,
            friction: 0.6,
            restitution: 0.0,
            rotation_locked: false,
            layers: CollisionLayers3d::ALL,
            external: false,
            sensor: false,
            sleep_allowed: true,
            ccd: false,
            retire_on_impact: false,
            linear_support: None,
            sleeping: false,
            quiet_time: 0.0,
            force: Vector::ZERO,
            torque: Vector::ZERO,
            impulse: Vector::ZERO,
            angular_impulse: Vector::ZERO,
            cached_bounds: (Vector::ZERO, Vector::ZERO),
        }
    }
    /// One-way compatibility import at scene construction. Never performed during integration.
    pub fn from_legacy(body: &RigidBox3d) -> Self {
        let b = body.body();
        let a = body.angular();
        let q = a.orientation;
        let qs = crate::ORIENTATION_SCALE as Scalar;
        let mut out = Self::new(
            b.id(),
            Shape::Box(b.half_extents().into()),
            b.position().into(),
            if b.kind() == BodyKind::Fixed {
                0.0
            } else {
                b.mass_units() as Scalar
            },
        );
        out.velocity = b.velocity().into();
        out.orientation = Quaternion(
            q.x as Scalar / qs,
            q.y as Scalar / qs,
            q.z as Scalar / qs,
            q.w as Scalar / qs,
        )
        .normalized();
        let w = a.angular_velocity;
        let ws = crate::ANGULAR_VELOCITY_SCALE as Scalar;
        out.angular_velocity = Vector(w.x as Scalar / ws, w.y as Scalar / ws, w.z as Scalar / ws);
        out.friction = b.material().friction_milli() as Scalar / 1000.0;
        out.restitution = b.material().restitution_milli() as Scalar / 1000.0;
        out.rotation_locked = body.rotation_locked();
        out.layers = body.collision_layers();
        out.external = body.motion_authority() == MotionAuthority3d::External;
        out.sensor = body.solver_participation() == SolverParticipation3d::OverlapOnly;
        out.sleep_allowed = body.sleep_mode() != SleepMode3d::Never;
        if let ContactMode3d::LinearPush { support_direction } = body.contact_mode() {
            out.linear_support = Some(support_direction.into());
        }
        out
    }
    pub fn is_sleeping(&self) -> bool {
        self.sleeping
    }
    fn movable(&self) -> bool {
        self.mass > 0.0 && !self.external
    }
    fn inverse_mass(&self) -> Scalar {
        if self.movable() && !self.sleeping {
            1.0 / self.mass
        } else {
            0.0
        }
    }
    #[cfg(test)]
    fn inverse_inertia(&self, v: Vector) -> Vector {
        if self.inverse_mass() == 0.0 || self.rotation_locked {
            return Vector::ZERO;
        }
        let inverse = self
            .shape
            .local_inverse_inertia(self.mass)
            .unwrap_or(Vector::ZERO);
        self.orientation
            .rotate(self.orientation.inverse_rotate(v).component_mul(inverse))
    }
    fn valid(&self) -> bool {
        self.position.finite()
            && self.velocity.finite()
            && self.angular_velocity.finite()
            && self.angular_velocity.abs().max_component() < 1e9
            && self.orientation.finite()
            && self.shape.valid_dimensions()
            && (!matches!(self.shape, Shape::Wedge(_))
                || self.rotation_locked
                || !self.movable())
            && self.mass.is_finite()
            && (self.mass == 0.0 || self.mass >= 1e-6)
            && self.mass < 1e12
            && self.friction.is_finite()
            && (0.0..=10.0).contains(&self.friction)
            && self.restitution.is_finite()
            && (0.0..=1.0).contains(&self.restitution)
            && (!self.rotation_locked || self.angular_velocity == Vector::ZERO)
            && (self.mass > 0.0
                || (self.velocity == Vector::ZERO && self.angular_velocity == Vector::ZERO))
            && self.position.abs().max_component() < 1e12
            && self.velocity.abs().max_component() < 1e12
    }
}
#[derive(Clone, Copy, Debug)]
pub struct Config {
    pub gravity: Vector,
    pub substeps: u8,
    pub velocity_iterations: u8,
    /// Optional bounded position-only correction against fixed colliders, after integration.
    /// Zero preserves the comparison kernel; the interactive tower explicitly selects two.
    pub fixed_position_iterations: u8,
    /// Scene-space length, default 0.02. Chosen explicitly for the legacy 36-unit crates.
    pub contact_slop: Scalar,
    pub sleep_speed: Scalar,
    pub sleep_seconds: Scalar,
    pub warm_start: bool,
    /// None retains the fixed-pass reference. Some permits a checked early exit.
    pub convergence: Option<Convergence>,
    /// Contact-island-local stopping, or the original whole-world convergence reference.
    /// With `convergence: None`, both scopes use the unchanged fixed-pass solver.
    pub convergence_scope: ConvergenceScope,
}
impl Default for Config {
    fn default() -> Self {
        Self {
            gravity: Vector(0.0, -3600.0, 0.0),
            substeps: 4,
            velocity_iterations: 8,
            fixed_position_iterations: 0,
            contact_slop: 0.02,
            sleep_speed: 1.0,
            sleep_seconds: 0.5,
            warm_start: true,
            convergence: Some(Convergence::default()),
            convergence_scope: ConvergenceScope::ContactIslands,
        }
    }
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    InvalidInput,
    DuplicateBody(BodyId),
    MissingBody(BodyId),
    NonFiniteState(BodyId),
}
impl std::fmt::Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "fixed-step approximation: {self:?}")
    }
}
impl std::error::Error for Error {}
#[derive(Clone, Debug, Default)]
pub struct Report {
    pub position: PositionReport,
    pub substeps: u32,
    pub pair_tests: u64,
    pub narrow_tests: u64,
    pub contact_points: u64,
    /// Equivalent solver rounds per substep (maximum over islands), not the sum over islands.
    /// Use `convergence.constraint_visits` for actual row work and `islands` for grouping costs.
    pub impulse_iterations: u64,
    pub integrated_bodies: u64,
    pub woken_bodies: u64,
    pub swept_contacts: u64,
    pub retired: Vec<BodyId>,
    pub max_penetration: Scalar,
    pub bookkeeping: BookkeepingStats,
    pub geometry: GeometryStats,
    pub convergence: ConvergenceStats,
    pub islands: IslandStats,
    /// Active response bodies prepared, including newly awakened bodies in the same substep.
    pub response_preparations: u64,
    /// Shape/mass inertia coefficients calculated; independent of contact iteration count.
    pub inertia_preparations: u64,
    /// Inverse-inertia vector evaluations (preparation is reused for these).
    pub inertia_applications: u64,
}
#[derive(Clone, Debug)]
struct CachedPoint {
    a: Vector,
    b: Vector,
    normal: Vector,
    impulse: Scalar,
    tangent: Vector,
}
#[derive(Clone, Debug)]
struct Constraint {
    a: usize,
    b: usize,
    n: Vector,
    ra: Vector,
    rb: Vector,
    t1: Vector,
    t2: Vector,
    normal_mass: Scalar,
    tangent_mass: [Scalar; 2],
    bias: Scalar,
    friction: Scalar,
    normal_impulse: Scalar,
    tangent_impulse: [Scalar; 2],
    sep: Scalar,
    swept: bool,
    response: [bool; 2],
    spin: bool,
}
#[derive(Clone, Debug)]
pub struct World {
    config: Config,
    bodies: Vec<Body>,
    cache: BTreeMap<(BodyId, BodyId), Vec<CachedPoint>>,
    last_h: Scalar,
    // Derived substep scratch, indexed like bodies. Refresh contents; reuse allocated capacity.
    responses: Vec<PreparedResponse>,
    bookkeeping: Scratch,
    geometry: GeometryCache,
    island_scratch: islands::Scratch,
    constraints: Vec<Constraint>,
    manifold_scratch: Vec<(usize, usize, contact::Manifold)>,
    pub last_report: Report,
    elapsed: Scalar,
}
impl World {
    pub fn new(config: Config) -> Result<Self, Error> {
        if !config.gravity.finite()
            || config.substeps == 0
            || config.substeps > 32
            || config.velocity_iterations == 0
            || config.velocity_iterations > 64
            || config.fixed_position_iterations > 8
            || config.convergence.is_some_and(|c| !c.valid())
            || !config.contact_slop.is_finite()
            || config.contact_slop <= 0.0
            || !config.sleep_speed.is_finite()
            || config.sleep_speed <= 0.0
            || !config.sleep_seconds.is_finite()
            || config.sleep_seconds <= 0.0
        {
            return Err(Error::InvalidInput);
        }
        Ok(Self {
            config,
            bodies: Vec::new(),
            cache: BTreeMap::new(),
            last_h: 0.0,
            responses: Vec::new(),
            bookkeeping: Scratch::default(),
            geometry: GeometryCache::default(),
            island_scratch: islands::Scratch::default(),
            constraints: Vec::new(),
            manifold_scratch: Vec::new(),
            last_report: Report::default(),
            elapsed: 0.0,
        })
    }
    pub fn bodies(&self) -> impl Iterator<Item = &Body> {
        self.bodies.iter()
    }
    pub fn body(&self, id: BodyId) -> Option<&Body> {
        self.index(id).ok().map(|i| &self.bodies[i])
    }
    pub fn elapsed_seconds(&self) -> Scalar {
        self.elapsed
    }
    pub fn is_quiescent(&self) -> bool {
        self.bookkeeping.activity.count == 0
    }
    pub fn config(&self) -> Config {
        self.config
    }
    fn index(&self, id: BodyId) -> Result<usize, Error> {
        self.bodies
            .binary_search_by_key(&id, |b| b.id)
            .map_err(|_| Error::MissingBody(id))
    }
    pub fn add_body(&mut self, mut body: Body) -> Result<(), Error> {
        body.orientation = body.orientation.normalized();
        if !body.valid() {
            return Err(Error::InvalidInput);
        }
        let i = match self.bodies.binary_search_by_key(&body.id, |b| b.id) {
            Ok(_) => return Err(Error::DuplicateBody(body.id)),
            Err(i) => i,
        };
        // Local topology change: only a genuine new fixed overlap can disturb a sleeper.
        let wake = if body.mass == 0.0 && !body.sensor {
            self.bodies
                .iter()
                .filter(|b| {
                    b.sleeping
                        && b.layers.collides_with(body.layers)
                        && contact::current(b, &body, 0.0).is_some()
                })
                .map(|b| b.id)
                .collect::<Vec<_>>()
        } else {
            Vec::new()
        };
        for id in wake {
            self.wake_island(id);
        }
        body.cached_bounds = contact::bounds(&body);
        self.bookkeeping.activity.count += usize::from(body.mass > 0.0 && !body.sleeping);
        self.bodies.insert(i, body);
        self.bookkeeping.layout_changed();
        Ok(())
    }
    pub fn remove_body(&mut self, id: BodyId) -> Option<Body> {
        let i = self.index(id).ok()?;
        self.bookkeeping.ensure_graph(&self.bodies, &self.cache);
        let neighbors = self
            .bookkeeping
            .graph
            .neighbors(i)
            .iter()
            .map(|&j| self.bodies[j].id)
            .collect::<Vec<_>>();
        for n in neighbors {
            self.wake_island(n);
        }
        self.geometry.remove(id);
        self.cache.retain(|&(a, b), _| a != id && b != id);
        let removed = self.bodies.remove(i);
        self.bookkeeping.activity.count -= usize::from(removed.mass > 0.0 && !removed.sleeping);
        self.bookkeeping.layout_changed();
        Some(removed)
    }
    pub fn set_velocity(&mut self, id: BodyId, v: Vector) -> Result<(), Error> {
        if !v.finite() || v.abs().max_component() >= 1e12 {
            return Err(Error::InvalidInput);
        }
        let i = self.index(id)?;
        if self.bodies[i].mass == 0.0 {
            return if v == Vector::ZERO {
                Ok(())
            } else {
                Err(Error::InvalidInput)
            };
        }
        if self.bodies[i].velocity != v {
            self.wake_island(id);
            self.bodies[i].velocity = v;
        }
        Ok(())
    }
    pub fn apply_impulse(
        &mut self,
        id: BodyId,
        j: Vector,
        world_point: Vector,
    ) -> Result<(), Error> {
        if !j.finite() || !world_point.finite() {
            return Err(Error::InvalidInput);
        }
        let i = self.index(id)?;
        if !self.bodies[i].movable() {
            return Err(Error::InvalidInput);
        }
        let angular = (world_point - self.bodies[i].position).cross(j);
        let pending = self.bodies[i].impulse + j;
        let spin = self.bodies[i].angular_impulse + angular;
        if !pending.finite() || !spin.finite() {
            return Err(Error::InvalidInput);
        }
        if j != Vector::ZERO {
            self.wake_island(id);
        }
        self.bodies[i].impulse = pending;
        self.bodies[i].angular_impulse = spin;
        Ok(())
    }
    pub fn add_force(&mut self, id: BodyId, force: Vector) -> Result<(), Error> {
        if !force.finite() {
            return Err(Error::InvalidInput);
        }
        let i = self.index(id)?;
        if !self.bodies[i].movable() {
            return Err(Error::InvalidInput);
        }
        let total = self.bodies[i].force + force;
        if !total.finite() {
            return Err(Error::InvalidInput);
        }
        if force != Vector::ZERO {
            self.wake_island(id);
        }
        self.bodies[i].force = total;
        Ok(())
    }
    fn wake_island(&mut self, root: BodyId) -> u64 {
        let Ok(root) = self.index(root) else {
            return 0;
        };
        let s = &mut self.bookkeeping;
        s.ensure_graph(&self.bodies, &self.cache);
        s.traversal.begin(self.bodies.len(), &mut s.work);
        s.traversal
            .collect(root, &self.bodies, &s.graph, &mut s.work);
        let mut count = 0;
        for &i in &s.traversal.island {
            let b = &mut self.bodies[i];
            if b.sleeping {
                count += 1;
                b.sleeping = false;
                b.quiet_time = 0.0;
            }
        }
        s.activity.count += count as usize;
        s.activity.dirty |= count > 0;
        count
    }
    pub fn has_support(&self, id: BodyId) -> bool {
        let up = (-self.config.gravity).unit();
        self.cache.iter().any(|(&(a, b), p)| {
            p.iter().any(|p| {
                (b == id && p.normal.dot(up) > 0.5) || (a == id && p.normal.dot(up) < -0.5)
            })
        })
    }
    /// Advance up to 0.1 seconds in a bounded number of substeps. No dropped remainder.
    pub fn step(&mut self, dt: Scalar) -> Result<Report, Error> {
        self.step_with_preparation::<true>(dt)
    }

    // The false specialization is only instantiated by tests, as an unprepared replay oracle.
    fn step_with_preparation<const PREPARED: bool>(&mut self, dt: Scalar) -> Result<Report, Error> {
        self.step_with_geometry::<PREPARED, true>(dt)
    }
    fn step_with_geometry<const PREPARED: bool, const CACHED: bool>(
        &mut self,
        dt: Scalar,
    ) -> Result<Report, Error> {
        if !dt.is_finite() || !(0.0..=0.1).contains(&dt) {
            return Err(Error::InvalidInput);
        }
        if dt == 0.0 {
            self.last_report = Report::default();
            self.last_report.islands.scratch_retained_bytes = self.island_scratch.retained_bytes();
            self.last_report.geometry.retained_bytes = self.geometry.retained_bytes() as u64;
            self.last_report.position.scratch_retained_bytes =
                self.bookkeeping.position.retained_bytes();
            return Ok(self.last_report.clone());
        }
        if dt / self.config.substeps as Scalar == 0.0 {
            return Err(Error::InvalidInput);
        }
        let mut report = Report::default();
        if self.is_quiescent() {
            self.elapsed += dt;
            self.finish_bookkeeping(&mut report);
            self.last_report = report.clone();
            return Ok(report);
        }
        let h = dt / self.config.substeps as Scalar;
        for substep in 0..self.config.substeps {
            report.substeps += 1;
            self.responses
                .resize(self.bodies.len(), PreparedResponse::default());
            for (b, prepared) in self.bodies.iter().zip(&mut self.responses) {
                *prepared = if PREPARED {
                    PreparedResponse::new(b, &mut report)
                } else {
                    PreparedResponse::default()
                };
            }
            // External impulses still apply exactly once, before the first force integration.
            if substep == 0 {
                for (b, cached) in self.bodies.iter_mut().zip(&self.responses) {
                    if b.movable() {
                        let prepared = cached.select::<PREPARED>(b, &mut report);
                        b.velocity += b.impulse * prepared.inverse_mass;
                        b.angular_velocity += prepared.inertia(b, b.angular_impulse, &mut report);
                    }
                    b.impulse = Vector::ZERO;
                    b.angular_impulse = Vector::ZERO;
                }
            }
            for (b, cached) in self.bodies.iter_mut().zip(&self.responses) {
                if b.movable() && !b.sleeping {
                    let prepared = cached.select::<PREPARED>(b, &mut report);
                    b.velocity += (self.config.gravity + b.force / b.mass) * h;
                    b.angular_velocity += prepared.inertia(b, b.torque, &mut report) * h;
                }
            }
            let mut pairs = std::mem::take(&mut self.manifold_scratch);
            self.manifolds::<CACHED>(h, &mut report, &mut pairs);
            let mut roots = std::mem::take(&mut self.bookkeeping.roots);
            roots.clear();
            reserve(&mut roots, pairs.len() * 2, &mut self.bookkeeping.work);
            roots.extend(
                pairs
                    .iter()
                    .flat_map(|(i, j, m)| {
                        let a = &self.bodies[*i];
                        let b = &self.bodies[*j];
                        let approach = -(b.velocity - a.velocity).dot(m.normal);
                        let disruptive = m.swept || approach > self.config.sleep_speed;
                        [
                            if disruptive && a.sleeping && b.mass > 0.0 {
                                Some(a.id)
                            } else {
                                None
                            },
                            if disruptive && b.sleeping && a.mass > 0.0 {
                                Some(b.id)
                            } else {
                                None
                            },
                        ]
                    })
                    .flatten(),
            );
            let mut woke = 0;
            for root in roots.drain(..) {
                woke += self.wake_island(root);
            }
            self.bookkeeping.roots = roots;
            report.woken_bodies += woke;
            if woke > 0 {
                // Wake admission changes inverse response before the contact's impulse, not next tick.
                // Pose/shape/mass cannot otherwise change during these velocity iterations.
                if PREPARED {
                    for (b, prepared) in self.bodies.iter().zip(&mut self.responses) {
                        if prepared.inverse_mass == 0.0 && b.movable() && !b.sleeping {
                            *prepared = PreparedResponse::new(b, &mut report);
                        }
                    }
                }
                self.manifolds::<CACHED>(h, &mut report, &mut pairs);
            }
            let mut constraints = std::mem::take(&mut self.constraints);
            constraints.clear();
            reserve(
                &mut constraints,
                pairs.iter().map(|(_, _, m)| m.points.len()).sum(),
                &mut self.bookkeeping.work,
            );
            for (i, j, m) in pairs.drain(..) {
                let a = &self.bodies[i];
                let b = &self.bodies[j];
                let n = m.normal;
                let linear = a.linear_support.is_some() || b.linear_support.is_some();
                let mut response = [true, true];
                if let Some(s) = a.linear_support
                    && n.dot(s.unit()) > 0.5
                {
                    response[1] = false;
                }
                if let Some(s) = b.linear_support
                    && (-n).dot(s.unit()) > 0.5
                {
                    response[0] = false;
                }
                let t1 = if n.0.abs() < 0.7 {
                    n.cross(Vector::X).unit()
                } else {
                    n.cross(Vector::Y).unit()
                };
                let t2 = n.cross(t1);
                let used = &mut self.bookkeeping.used;
                used.clear();
                reserve(used, m.points.len(), &mut self.bookkeeping.work);
                for point in &m.points {
                    let response_bodies = [(a, &self.responses[i]), (b, &self.responses[j])];
                    let arms = [point.ra, point.rb];
                    let k = effective_mass::<PREPARED>(
                        response_bodies,
                        arms,
                        n,
                        response,
                        !linear,
                        &mut report,
                    );
                    if k <= 1e-14 {
                        continue;
                    }
                    let rel = contact_velocity(b, point.rb, !linear)
                        - contact_velocity(a, point.ra, !linear);
                    let restitution = if rel.dot(n) < -1.0
                        && point.separation <= self.config.contact_slop
                        && !self.cache.contains_key(&(a.id, b.id))
                    {
                        -a.restitution.min(b.restitution) * rel.dot(n)
                    } else {
                        0.0
                    };
                    let bias = if point.separation > 0.0 {
                        -point.separation / h
                    } else {
                        (0.2 * (-point.separation - self.config.contact_slop).max(0.0) / h)
                            .min(60.0)
                    }
                    .max(restitution);
                    // A separated speculative constraint must still permit closing up to its surface.
                    let bias = if point.separation > 0.0 {
                        -point.separation / h
                    } else {
                        bias
                    };
                    let mut c = Constraint {
                        a: i,
                        b: j,
                        n,
                        ra: point.ra,
                        rb: point.rb,
                        t1,
                        t2,
                        normal_mass: 1.0 / k,
                        tangent_mass: [
                            effective_mass::<PREPARED>(
                                response_bodies,
                                arms,
                                t1,
                                response,
                                !linear,
                                &mut report,
                            )
                            .recip(),
                            effective_mass::<PREPARED>(
                                response_bodies,
                                arms,
                                t2,
                                response,
                                !linear,
                                &mut report,
                            )
                            .recip(),
                        ],
                        bias,
                        friction: if linear {
                            0.0
                        } else {
                            a.friction.max(b.friction)
                        },
                        normal_impulse: 0.0,
                        tangent_impulse: [0.0; 2],
                        sep: point.separation,
                        swept: m.swept,
                        response,
                        spin: !linear,
                    };
                    if self.config.warm_start
                        && point.separation <= self.config.contact_slop
                        && self.last_h > 0.0
                        && let Some(old) = self.cache.get(&(a.id, b.id))
                    {
                        let la = a.orientation.inverse_rotate(c.ra);
                        let lb = b.orientation.inverse_rotate(c.rb);
                        if let Some((idx, p)) = old
                            .iter()
                            .enumerate()
                            .filter(|(idx, p)| !used.contains(idx) && p.normal.dot(n) > 0.99)
                            .min_by(|(_, x), (_, y)| {
                                ((x.a - la).dot(x.a - la) + (x.b - lb).dot(x.b - lb)).total_cmp(
                                    &((y.a - la).dot(y.a - la) + (y.b - lb).dot(y.b - lb)),
                                )
                            })
                            && (p.a - la).length() + (p.b - lb).length()
                                < 0.1 * a.shape.radius().min(b.shape.radius())
                        {
                            used.push(idx);
                            let scale = (h / self.last_h).clamp(0.0, 2.0);
                            c.normal_impulse = p.impulse * scale;
                            c.tangent_impulse =
                                [p.tangent.dot(t1) * scale, p.tangent.dot(t2) * scale];
                        }
                    }
                    report.max_penetration =
                        report.max_penetration.max((-point.separation).max(0.0));
                    constraints.push(c);
                }
            }
            self.manifold_scratch = pairs;
            report.contact_points += constraints.len() as u64;
            for c in &constraints {
                apply::<PREPARED>(
                    &mut self.bodies,
                    &self.responses,
                    c,
                    c.n * c.normal_impulse
                        + c.t1 * c.tangent_impulse[0]
                        + c.t2 * c.tangent_impulse[1],
                    &mut report,
                );
            }
            match self.config.convergence {
                Some(tolerances)
                    if self.config.convergence_scope == ConvergenceScope::ContactIslands =>
                {
                    islands::solve::<PREPARED>(
                        &mut self.bodies,
                        &self.responses,
                        &mut constraints,
                        self.config.velocity_iterations,
                        tolerances,
                        &mut self.island_scratch,
                        &mut report,
                    )
                }
                Some(tolerances) => convergence::solve::<PREPARED, true>(
                    &mut self.bodies,
                    &self.responses,
                    &mut constraints,
                    self.config.velocity_iterations,
                    tolerances,
                    &mut report,
                ),
                None => convergence::solve::<PREPARED, false>(
                    &mut self.bodies,
                    &self.responses,
                    &mut constraints,
                    self.config.velocity_iterations,
                    Convergence::default(),
                    &mut report,
                ),
            }
            let scratch = &mut self.bookkeeping;
            scratch.retired.clear();
            scratch.support_edges.clear();
            reserve(&mut scratch.supported, self.bodies.len(), &mut scratch.work);
            scratch.supported.clear();
            scratch
                .supported
                .extend(self.bodies.iter().map(|b| b.mass == 0.0 || b.sleeping));
            // Reuse point allocations when the pair survives. Warm starting has already read the
            // old impulses; only adjacency-key changes invalidate the graph, not updated impulses.
            for (&(a, b), points) in &mut self.cache {
                let a = self
                    .bodies
                    .binary_search_by_key(&a, |b| b.id)
                    .expect("live contact");
                let b = self
                    .bodies
                    .binary_search_by_key(&b, |b| b.id)
                    .expect("live contact");
                if !scratch.supported[a] || !scratch.supported[b] {
                    points.clear();
                }
            }
            for c in &constraints {
                let a = &self.bodies[c.a];
                let b = &self.bodies[c.b];
                if c.normal_impulse > 0.0 {
                    if a.retire_on_impact {
                        push(&mut scratch.retired, a.id, &mut scratch.work);
                    }
                    if b.retire_on_impact {
                        push(&mut scratch.retired, b.id, &mut scratch.work);
                    }
                    if c.swept {
                        report.swept_contacts += 1;
                    }
                }
                if c.sep <= self.config.contact_slop * 2.0 {
                    let points = match self.cache.entry((a.id, b.id)) {
                        std::collections::btree_map::Entry::Occupied(entry) => entry.into_mut(),
                        std::collections::btree_map::Entry::Vacant(entry) => {
                            scratch.graph.dirty = true;
                            entry.insert(Vec::new())
                        }
                    };
                    points.push(CachedPoint {
                        a: a.orientation.inverse_rotate(c.ra),
                        b: b.orientation.inverse_rotate(c.rb),
                        normal: c.n,
                        impulse: c.normal_impulse,
                        tangent: c.t1 * c.tangent_impulse[0] + c.t2 * c.tangent_impulse[1],
                    });
                    if c.n.dot((-self.config.gravity).unit()) > 0.5 {
                        push(&mut scratch.support_edges, (c.a, c.b), &mut scratch.work);
                    }
                    if c.n.dot((-self.config.gravity).unit()) < -0.5 {
                        push(&mut scratch.support_edges, (c.b, c.a), &mut scratch.work);
                    }
                }
            }
            self.cache.retain(|_, points| {
                if points.is_empty() {
                    scratch.graph.dirty = true;
                    false
                } else {
                    true
                }
            });
            scratch.support.propagate(
                &mut scratch.supported,
                &scratch.support_edges,
                &mut scratch.work,
            );
            scratch.activity.refresh(&self.bodies, &mut scratch.work);
            for &i in &scratch.activity.indices {
                let b = &mut self.bodies[i];
                report.integrated_bodies += 1;
                b.position += b.velocity * h;
                if !b.rotation_locked {
                    b.orientation = b.orientation.integrate(b.angular_velocity, h);
                }
                if !b.valid() {
                    return Err(Error::NonFiniteState(b.id));
                }
                if b.movable()
                    && b.sleep_allowed
                    && scratch.supported[i]
                    && b.velocity.length() + b.angular_velocity.length() * b.shape.radius()
                        < self.config.sleep_speed
                {
                    b.quiet_time += h;
                } else {
                    b.quiet_time = 0.0;
                }
                b.cached_bounds = contact::bounds(b);
            }
            self.constraints = constraints;
            self.correct_fixed_positions(h, &mut report.position)?;
            self.sleep_quiet_islands();
            // Retirement is local; remove all affected cached edges before indexed graph reuse.
            self.bookkeeping.retired.sort_unstable();
            self.bookkeeping.retired.dedup();
            for n in 0..self.bookkeeping.retired.len() {
                let id = self.bookkeeping.retired[n];
                if let Ok(i) = self.index(id) {
                    let b = self.bodies.remove(i);
                    self.bookkeeping.activity.count -= usize::from(b.mass > 0.0 && !b.sleeping);
                    self.geometry.remove(id);
                    self.cache.retain(|&(a, b), _| a != id && b != id);
                    self.bookkeeping.layout_changed();
                    report.retired.push(id);
                }
            }
            self.last_h = h;
        }
        for b in &mut self.bodies {
            b.force = Vector::ZERO;
            b.torque = Vector::ZERO;
        }
        self.elapsed += dt;
        self.finish_bookkeeping(&mut report);
        self.last_report = report.clone();
        Ok(report)
    }
    fn finish_bookkeeping(&mut self, report: &mut Report) {
        report.islands.scratch_retained_bytes = self.island_scratch.retained_bytes();
        report.position.scratch_retained_bytes = self.bookkeeping.position.retained_bytes();
        report.geometry.retained_bytes = self.geometry.retained_bytes() as u64;
        self.bookkeeping.work.scratch_retained_bytes = (self.bookkeeping.retained_bytes()
            + bookkeeping::bytes(&self.constraints)
            + bookkeeping::bytes(&self.manifold_scratch))
            as u64;
        report.bookkeeping = std::mem::take(&mut self.bookkeeping.work);
    }
    fn sleep_quiet_islands(&mut self) {
        let s = &mut self.bookkeeping;
        s.activity.refresh(&self.bodies, &mut s.work);
        s.ensure_graph(&self.bodies, &self.cache);
        s.traversal.begin(self.bodies.len(), &mut s.work);
        let mut slept = 0;
        // Fully sleeping components need no rediscovery. An awake root still reaches every
        // dynamic neighbor, including sleeping members, so support and wake semantics are unchanged.
        for &root in &s.activity.indices {
            s.traversal
                .collect(root, &self.bodies, &s.graph, &mut s.work);
            if s.traversal.island.iter().all(|&i| {
                let b = &self.bodies[i];
                b.sleeping || (b.sleep_allowed && b.quiet_time >= self.config.sleep_seconds)
            }) {
                for &i in &s.traversal.island {
                    let b = &mut self.bodies[i];
                    slept += usize::from(!b.sleeping);
                    b.sleeping = true;
                    b.velocity = Vector::ZERO;
                    b.angular_velocity = Vector::ZERO;
                }
            }
        }
        s.activity.count -= slept;
        s.activity.dirty |= slept > 0;
    }

    fn manifolds<const CACHED: bool>(
        &mut self,
        h: Scalar,
        report: &mut Report,
        out: &mut Vec<(usize, usize, contact::Manifold)>,
    ) {
        let s = &mut self.bookkeeping;
        if CACHED {
            self.geometry.begin(self.bodies.len());
        }
        out.clear();
        if s.bounds.len() != self.bodies.len() {
            reserve(&mut s.bounds, self.bodies.len(), &mut s.work);
            s.bounds.clear();
            s.bounds
                .extend((0..self.bodies.len()).map(|index| SweepRow {
                    index,
                    lo: Vector::ZERO,
                    hi: Vector::ZERO,
                    active: true,
                }));
        }
        for row in &mut s.bounds {
            let b = &self.bodies[row.index];
            let active = b.mass > 0.0 && !b.sleeping;
            if !active && !row.active {
                continue;
            }
            let (lo, hi) = b.cached_bounds;
            let delta = if b.sleeping {
                Vector::ZERO
            } else {
                b.velocity * h
            };
            let angular = b.angular_velocity.length() * b.shape.radius() * h;
            let pad = Vector(angular, angular, angular)
                + Vector(1.0, 1.0, 1.0) * self.config.contact_slop;
            row.lo = lo.min(lo + delta) - pad;
            row.hi = hi.max(hi + delta) + pad;
            row.active = active;
            s.work.bounds_updates += 1;
        }
        // The key is total and unique (BodyId), so unstable sort preserves canonical pair order
        // while avoiding stable-sort allocation. Stationary rows retain their valid padded bounds.
        let compare = |a: &SweepRow, b: &SweepRow| {
            a.lo.0
                .total_cmp(&b.lo.0)
                .then_with(|| self.bodies[a.index].id.cmp(&self.bodies[b.index].id))
        };
        let ordered = s.bounds.windows(2).all(|rows| {
            s.work.bounds_order_checks += 1;
            !compare(&rows[0], &rows[1]).is_gt()
        });
        if !ordered {
            s.bounds.sort_unstable_by(compare);
            s.work.bound_rows_sorted += s.bounds.len() as u64;
        }
        let bounds = &s.bounds;
        for p in 0..bounds.len() {
            let SweepRow {
                index: i, lo, hi, ..
            } = bounds[p];
            for &SweepRow {
                index: j,
                lo: bl,
                hi: bh,
                ..
            } in bounds.iter().skip(p + 1)
            {
                if bl.0 > hi.0 {
                    break;
                }
                report.pair_tests += 1;
                if bl.1 > hi.1 || bh.1 < lo.1 || bl.2 > hi.2 || bh.2 < lo.2 {
                    continue;
                }
                let (i, j) = if self.bodies[i].id < self.bodies[j].id {
                    (i, j)
                } else {
                    (j, i)
                };
                let a = &self.bodies[i];
                let b = &self.bodies[j];
                if a.sensor
                    || b.sensor
                    || !a.layers.collides_with(b.layers)
                    || (a.inverse_mass() == 0.0 && b.inverse_mass() == 0.0)
                {
                    continue;
                }
                report.narrow_tests += 1;
                // GeometryCache shares read-only frames even for rotating pairs; it retains
                // complete pair results only when both orientation dependencies are stable.
                let current = if CACHED {
                    self.geometry.query(
                        [i, j],
                        [a, b],
                        self.config.contact_slop,
                        &mut report.geometry,
                    )
                } else {
                    contact::current_counted(a, b, self.config.contact_slop, &mut report.geometry)
                };
                let m = current.or_else(|| {
                    let travel = (b.velocity - a.velocity).length() * h;
                    if a.ccd
                        || b.ccd
                        || travel
                            > 0.5
                                * a.shape
                                    .half_extents()
                                    .min_component()
                                    .min(b.shape.half_extents().min_component())
                    {
                        if CACHED {
                            self.geometry.swept(
                                [i, j],
                                [a, b],
                                h,
                                self.config.contact_slop,
                                &mut report.geometry,
                            )
                        } else {
                            contact::swept(a, b, h, self.config.contact_slop, &mut report.geometry)
                        }
                    } else {
                        None
                    }
                });
                if let Some(m) = m {
                    push(out, (i, j, m), &mut s.work);
                }
            }
        }
        if CACHED {
            self.geometry.finish(&mut report.geometry);
        }
        // A fast projectile's original trajectory must not activate bodies behind its first hit.
        // Keep all equal-time contacts, but defer later speculative contacts to the next substep,
        // where the already-updated velocity becomes authoritative. No event-restart loop is added.
        reserve(&mut s.earliest, self.bodies.len(), &mut s.work);
        s.earliest.clear();
        s.earliest.resize(self.bodies.len(), Scalar::INFINITY);
        for (i, j, m) in out.iter() {
            for index in [*i, *j] {
                if self.bodies[index].ccd {
                    s.earliest[index] = s.earliest[index].min(m.time);
                }
            }
        }
        out.retain(|(i, j, m)| {
            [i, j]
                .into_iter()
                .all(|index| m.time <= s.earliest[*index] + 1e-7)
        });
        out.sort_unstable_by_key(|(i, j, _)| (self.bodies[*i].id, self.bodies[*j].id));
    }
}
fn contact_velocity(b: &Body, r: Vector, spin: bool) -> Vector {
    b.velocity
        + if spin {
            b.angular_velocity.cross(r)
        } else {
            Vector::ZERO
        }
}
fn effective_mass<const PREPARED: bool>(
    bodies: [(&Body, &PreparedResponse); 2],
    arms: [Vector; 2],
    n: Vector,
    response: [bool; 2],
    spin: bool,
    report: &mut Report,
) -> Scalar {
    let mut k = 0.0;
    for (((body, cached), r), yes) in bodies.into_iter().zip(arms).zip(response) {
        if yes {
            let prepared = cached.select::<PREPARED>(body, report);
            k += prepared.inverse_mass;
            if spin {
                k += prepared.inertia(body, r.cross(n), report).cross(r).dot(n);
            }
        }
    }
    k
}
fn apply<const PREPARED: bool>(
    bodies: &mut [Body],
    responses: &[PreparedResponse],
    c: &Constraint,
    j: Vector,
    report: &mut Report,
) {
    for (i, r, j, yes) in [
        (c.a, c.ra, -j, c.response[0]),
        (c.b, c.rb, j, c.response[1]),
    ] {
        if yes {
            let b = &mut bodies[i];
            let prepared = responses[i].select::<PREPARED>(b, report);
            b.velocity += j * prepared.inverse_mass;
            if c.spin {
                b.angular_velocity += prepared.inertia(b, r.cross(j), report);
            }
        }
    }
}
