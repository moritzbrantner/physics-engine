//! Explicit f64 fixed-step approximation, alongside (not silently replacing) the event solver.
//!
//! External impulses change velocity once; forces act for the requested duration. Each bounded
//! substep prepares contacts, accumulates clamped sequential impulses, then integrates the new
//! linear/angular velocity. Contact impulses persist for warm starting. No integer quantization,
//! rational remainder tree, event-restart loop, or whole-world sleep proxy conversion is used.
//!
//! CCD sweeps translation over each substep for fast shapes. Orientations are held fixed during
//! those sweeps and integrated between substeps, so this is NOT analytic rotational CCD.
mod contact;
mod math;
use crate::{
    BodyId, BodyKind, CollisionLayers3d, ContactMode3d, MotionAuthority3d, RigidBox3d, SleepMode3d,
    SolverParticipation3d, numeric,
};
pub use math::{Quaternion, Vector};
use numeric::Scalar;
use std::collections::{BTreeMap, BTreeSet};

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Shape {
    Box(Vector),
    Sphere(Scalar),
}
impl Shape {
    pub fn radius(self) -> Scalar {
        match self {
            Self::Box(h) => h.length(),
            Self::Sphere(r) => r,
        }
    }
    pub fn half_extents(self) -> Vector {
        match self {
            Self::Box(h) => h,
            Self::Sphere(r) => Vector(r, r, r),
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
    fn inverse_inertia(&self, v: Vector) -> Vector {
        if self.inverse_mass() == 0.0 || self.rotation_locked {
            return Vector::ZERO;
        }
        let inverse = match self.shape {
            Shape::Sphere(r) => {
                let k = 2.5 / (self.mass * r * r);
                Vector(k, k, k)
            }
            Shape::Box(h) => Vector(
                3.0 / (self.mass * (h.1 * h.1 + h.2 * h.2)),
                3.0 / (self.mass * (h.0 * h.0 + h.2 * h.2)),
                3.0 / (self.mass * (h.0 * h.0 + h.1 * h.1)),
            ),
        };
        self.orientation
            .rotate(self.orientation.inverse_rotate(v).component_mul(inverse))
    }
    fn valid(&self) -> bool {
        self.position.finite()
            && self.velocity.finite()
            && self.angular_velocity.finite()
            && self.angular_velocity.abs().max_component() < 1e9
            && self.orientation.finite()
            && self.shape.half_extents().finite()
            && self.shape.half_extents().min_component() >= 1e-6
            && self.shape.half_extents().max_component() < 1e12
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
    /// Scene-space length, default 0.02. Chosen explicitly for the legacy 36-unit crates.
    pub contact_slop: Scalar,
    pub sleep_speed: Scalar,
    pub sleep_seconds: Scalar,
    pub warm_start: bool,
}
impl Default for Config {
    fn default() -> Self {
        Self {
            gravity: Vector(0.0, -3600.0, 0.0),
            substeps: 4,
            velocity_iterations: 8,
            contact_slop: 0.02,
            sleep_speed: 1.0,
            sleep_seconds: 0.5,
            warm_start: true,
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
    pub substeps: u32,
    pub pair_tests: u64,
    pub narrow_tests: u64,
    pub contact_points: u64,
    pub impulse_iterations: u64,
    pub integrated_bodies: u64,
    pub woken_bodies: u64,
    pub swept_contacts: u64,
    pub retired: Vec<BodyId>,
    pub max_penetration: Scalar,
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
        self.bodies.iter().all(|b| b.mass == 0.0 || b.sleeping)
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
        self.bodies.insert(i, body);
        Ok(())
    }
    pub fn remove_body(&mut self, id: BodyId) -> Option<Body> {
        let i = self.index(id).ok()?;
        let neighbors = self
            .cache
            .keys()
            .filter_map(|&(a, b)| {
                if a == id {
                    Some(b)
                } else if b == id {
                    Some(a)
                } else {
                    None
                }
            })
            .collect::<Vec<_>>();
        for n in neighbors {
            self.wake_island(n);
        }
        self.cache.retain(|&(a, b), _| a != id && b != id);
        Some(self.bodies.remove(i))
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
        let mut frontier = vec![root];
        let mut seen = BTreeSet::new();
        let mut count = 0;
        while let Some(id) = frontier.pop() {
            if !seen.insert(id) {
                continue;
            }
            let Ok(i) = self.index(id) else {
                continue;
            };
            let b = &mut self.bodies[i];
            if b.mass == 0.0 || b.external {
                continue;
            }
            if b.sleeping {
                count += 1;
                b.sleeping = false;
                b.quiet_time = 0.0;
            }
            for &(a, b) in self.cache.keys() {
                if a == id {
                    frontier.push(b);
                } else if b == id {
                    frontier.push(a);
                }
            }
        }
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
        if !dt.is_finite() || !(0.0..=0.1).contains(&dt) {
            return Err(Error::InvalidInput);
        }
        if dt == 0.0 {
            self.last_report = Report::default();
            return Ok(self.last_report.clone());
        }
        if dt / self.config.substeps as Scalar == 0.0 {
            return Err(Error::InvalidInput);
        }
        let mut report = Report::default();
        if self.is_quiescent() {
            self.elapsed += dt;
            self.last_report = report.clone();
            return Ok(report);
        }
        for b in &mut self.bodies {
            if b.movable() {
                b.velocity += b.impulse * b.inverse_mass();
                b.angular_velocity += b.inverse_inertia(b.angular_impulse);
            }
            b.impulse = Vector::ZERO;
            b.angular_impulse = Vector::ZERO;
        }
        let h = dt / self.config.substeps as Scalar;
        for _ in 0..self.config.substeps {
            report.substeps += 1;
            for b in &mut self.bodies {
                if b.movable() && !b.sleeping {
                    b.velocity += (self.config.gravity + b.force / b.mass) * h;
                    b.angular_velocity += b.inverse_inertia(b.torque) * h;
                }
            }
            let mut pairs = self.manifolds(h, &mut report);
            let roots = pairs
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
                .flatten()
                .collect::<Vec<_>>();
            let mut woke = 0;
            for root in roots {
                woke += self.wake_island(root);
            }
            report.woken_bodies += woke;
            if woke > 0 {
                pairs = self.manifolds(h, &mut report);
            }
            let mut constraints = Vec::new();
            for (i, j, m) in pairs {
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
                let mut used = BTreeSet::new();
                for point in m.points {
                    let k = effective_mass(a, b, point.ra, point.rb, n, response, !linear);
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
                            effective_mass(a, b, point.ra, point.rb, t1, response, !linear).recip(),
                            effective_mass(a, b, point.ra, point.rb, t2, response, !linear).recip(),
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
                            used.insert(idx);
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
            report.contact_points += constraints.len() as u64;
            for c in &constraints {
                apply(
                    &mut self.bodies,
                    c,
                    c.n * c.normal_impulse
                        + c.t1 * c.tangent_impulse[0]
                        + c.t2 * c.tangent_impulse[1],
                );
            }
            for _ in 0..self.config.velocity_iterations {
                report.impulse_iterations += 1;
                for c in &mut constraints {
                    let vn = (contact_velocity(&self.bodies[c.b], c.rb, c.spin)
                        - contact_velocity(&self.bodies[c.a], c.ra, c.spin))
                    .dot(c.n);
                    let next = (c.normal_impulse + (c.bias - vn) * c.normal_mass).max(0.0);
                    let dj = next - c.normal_impulse;
                    c.normal_impulse = next;
                    apply(&mut self.bodies, c, c.n * dj);
                    let rel = contact_velocity(&self.bodies[c.b], c.rb, c.spin)
                        - contact_velocity(&self.bodies[c.a], c.ra, c.spin);
                    let mut tangent = [
                        c.tangent_impulse[0] - rel.dot(c.t1) * c.tangent_mass[0],
                        c.tangent_impulse[1] - rel.dot(c.t2) * c.tangent_mass[1],
                    ];
                    let len = tangent[0].hypot(tangent[1]);
                    let limit = c.friction * c.normal_impulse;
                    if len > limit && len > 0.0 {
                        tangent[0] *= limit / len;
                        tangent[1] *= limit / len;
                    }
                    let jt = c.t1 * (tangent[0] - c.tangent_impulse[0])
                        + c.t2 * (tangent[1] - c.tangent_impulse[1]);
                    c.tangent_impulse = tangent;
                    apply(&mut self.bodies, c, jt);
                }
            }
            let mut retired = BTreeSet::new();
            let mut support_edges = Vec::new();
            let sleeping_ids = self
                .bodies
                .iter()
                .filter(|b| b.mass == 0.0 || b.sleeping)
                .map(|b| b.id)
                .collect::<BTreeSet<_>>();
            self.cache
                .retain(|&(a, b), _| sleeping_ids.contains(&a) && sleeping_ids.contains(&b));
            let mut fresh: BTreeMap<(BodyId, BodyId), Vec<CachedPoint>> = BTreeMap::new();
            for c in &constraints {
                let a = &self.bodies[c.a];
                let b = &self.bodies[c.b];
                if c.normal_impulse > 0.0 {
                    if a.retire_on_impact {
                        retired.insert(a.id);
                    }
                    if b.retire_on_impact {
                        retired.insert(b.id);
                    }
                    if c.swept {
                        report.swept_contacts += 1;
                    }
                }
                if c.sep <= self.config.contact_slop * 2.0 {
                    fresh.entry((a.id, b.id)).or_default().push(CachedPoint {
                        a: a.orientation.inverse_rotate(c.ra),
                        b: b.orientation.inverse_rotate(c.rb),
                        normal: c.n,
                        impulse: c.normal_impulse,
                        tangent: c.t1 * c.tangent_impulse[0] + c.t2 * c.tangent_impulse[1],
                    });
                    if c.n.dot((-self.config.gravity).unit()) > 0.5 {
                        support_edges.push((a.id, b.id));
                    }
                    if c.n.dot((-self.config.gravity).unit()) < -0.5 {
                        support_edges.push((b.id, a.id));
                    }
                }
            }
            self.cache.extend(fresh);
            let mut supported = sleeping_ids;
            for _ in 0..self.bodies.len() {
                let old = supported.len();
                for &(lower, upper) in &support_edges {
                    if supported.contains(&lower) {
                        supported.insert(upper);
                    }
                }
                if old == supported.len() {
                    break;
                }
            }
            for b in &mut self.bodies {
                if b.mass == 0.0 || b.sleeping {
                    continue;
                }
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
                    && supported.contains(&b.id)
                    && b.velocity.length() + b.angular_velocity.length() * b.shape.radius()
                        < self.config.sleep_speed
                {
                    b.quiet_time += h;
                } else {
                    b.quiet_time = 0.0;
                }
                b.cached_bounds = contact::bounds(b);
            }
            self.sleep_quiet_islands();
            // Retirement is local. Projectiles never silently invalidate all sleeping islands.
            for id in retired {
                if let Ok(i) = self.index(id) {
                    self.bodies.remove(i);
                    self.cache.retain(|&(a, b), _| a != id && b != id);
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
        self.last_report = report.clone();
        Ok(report)
    }
    fn sleep_quiet_islands(&mut self) {
        let mut visited = BTreeSet::new();
        for index in 0..self.bodies.len() {
            let root = self.bodies[index].id;
            if !self.bodies[index].movable() || visited.contains(&root) {
                continue;
            }
            let mut frontier = vec![root];
            let mut island = Vec::new();
            while let Some(id) = frontier.pop() {
                let Ok(i) = self.index(id) else {
                    continue;
                };
                if !self.bodies[i].movable() || !visited.insert(id) {
                    continue;
                }
                island.push(i);
                for &(a, b) in self.cache.keys() {
                    if a == id {
                        frontier.push(b);
                    } else if b == id {
                        frontier.push(a);
                    }
                }
            }
            if island.iter().all(|&i| {
                self.bodies[i].sleeping
                    || (self.bodies[i].sleep_allowed
                        && self.bodies[i].quiet_time >= self.config.sleep_seconds)
            }) {
                for i in island {
                    let b = &mut self.bodies[i];
                    b.sleeping = true;
                    b.velocity = Vector::ZERO;
                    b.angular_velocity = Vector::ZERO;
                }
            }
        }
    }

    fn manifolds(&self, h: Scalar, report: &mut Report) -> Vec<(usize, usize, contact::Manifold)> {
        let mut bounds = self
            .bodies
            .iter()
            .enumerate()
            .map(|(i, b)| {
                let (lo, hi) = b.cached_bounds;
                let delta = if b.sleeping {
                    Vector::ZERO
                } else {
                    b.velocity * h
                };
                let angular = b.angular_velocity.length() * b.shape.radius() * h;
                let pad = Vector(angular, angular, angular)
                    + Vector(1.0, 1.0, 1.0) * self.config.contact_slop;
                (i, lo.min(lo + delta) - pad, hi.max(hi + delta) + pad)
            })
            .collect::<Vec<_>>();
        bounds.sort_by(|a, b| {
            a.1.0
                .total_cmp(&b.1.0)
                .then_with(|| self.bodies[a.0].id.cmp(&self.bodies[b.0].id))
        });
        let mut out = Vec::new();
        for p in 0..bounds.len() {
            let (i, lo, hi) = bounds[p];
            for &(j, bl, bh) in bounds.iter().skip(p + 1) {
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
                let m = contact::current(a, b, self.config.contact_slop).or_else(|| {
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
                        contact::swept(a, b, h, self.config.contact_slop)
                    } else {
                        None
                    }
                });
                if let Some(m) = m {
                    out.push((i, j, m));
                }
            }
        }
        // A fast projectile's original trajectory must not activate bodies behind its first hit.
        // Keep all equal-time contacts, but defer later speculative contacts to the next substep,
        // where the already-updated velocity becomes authoritative. No event-restart loop is added.
        let mut earliest: BTreeMap<usize, Scalar> = BTreeMap::new();
        for (i, j, m) in &out {
            for index in [*i, *j] {
                if self.bodies[index].ccd {
                    earliest
                        .entry(index)
                        .and_modify(|t| *t = t.min(m.time))
                        .or_insert(m.time);
                }
            }
        }
        out.retain(|(i, j, m)| {
            [i, j]
                .into_iter()
                .all(|index| earliest.get(index).is_none_or(|t| m.time <= *t + 1e-7))
        });
        out.sort_by_key(|(i, j, _)| (self.bodies[*i].id, self.bodies[*j].id));
        out
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
fn effective_mass(
    a: &Body,
    b: &Body,
    ra: Vector,
    rb: Vector,
    n: Vector,
    response: [bool; 2],
    spin: bool,
) -> Scalar {
    let mut k = 0.0;
    for (body, r, yes) in [(a, ra, response[0]), (b, rb, response[1])] {
        if yes {
            k += body.inverse_mass();
            if spin {
                k += body.inverse_inertia(r.cross(n)).cross(r).dot(n);
            }
        }
    }
    k
}
fn apply(bodies: &mut [Body], c: &Constraint, j: Vector) {
    for (i, r, j, yes) in [
        (c.a, c.ra, -j, c.response[0]),
        (c.b, c.rb, j, c.response[1]),
    ] {
        if yes {
            let b = &mut bodies[i];
            b.velocity += j * b.inverse_mass();
            if c.spin {
                b.angular_velocity += b.inverse_inertia(r.cross(j));
            }
        }
    }
}
