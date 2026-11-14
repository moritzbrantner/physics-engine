//! Compound dynamic bodies (#567): one rigid body built from an ordered list of primitive
//! children, each with its own local pose and mass.
//!
//! [`PreparedCompound::new`] computes the total mass, the centre of mass and the full inertia
//! tensor from the children (each child's own tensor, rotated into the authoring frame and moved
//! to the common centre of mass by the parallel-axis theorem), then diagonalizes it. The body's
//! frame is that principal frame at the centre of mass: [`Body::position`] is the centre of mass
//! and [`Body::orientation`] turns the principal axes, so the solver keeps its diagonal local
//! inverse inertia about the body position. [`PreparedCompound::body_pose`] and
//! [`PreparedCompound::origin_pose`] convert between that pose and the authoring origin's.
//!
//! Contacts, sweeps and queries resolve against each child at its world pose and report the
//! child's stable index (its position in the authored list).

use std::sync::Arc;

use super::{Body, Error, Quaternion, Real, Shape, Vector, World, contact};
use crate::BodyId;

/// The most children one compound holds.
pub const MAX_COMPOUND_CHILDREN: usize = 256;

/// One primitive part of a compound body: a box, sphere or capsule at a local pose, with its
/// own mass (uniform over the part).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CompoundChild {
    pub shape: Shape,
    /// The part's centre in the compound's frame.
    pub position: Vector,
    pub orientation: Quaternion,
    /// Positive, in `[1e-6, 1e12)`.
    pub mass: Real,
}
impl CompoundChild {
    pub const fn new(shape: Shape, position: Vector, mass: Real) -> Self {
        Self {
            shape,
            position,
            orientation: Quaternion::IDENTITY,
            mass,
        }
    }
}

/// The derived data a compound body's [`Shape::Compound`] carries: what the solver and the
/// bounds need without the children.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CompoundShape {
    /// Bounding radius about the centre of mass.
    radius: Real,
    /// Principal moments of inertia per unit mass, along the body axes.
    inertia: Vector,
}
impl CompoundShape {
    /// Bounding radius about the centre of mass (the body position).
    pub fn radius(self) -> Real {
        self.radius
    }
    pub(super) fn valid(self) -> bool {
        self.radius.is_finite()
            && (1e-6..1e12).contains(&self.radius)
            && self.inertia.finite()
            && self.inertia.min_component() > 0.0
    }
    /// Evaluated in f64, as the primitive shapes' inverse inertia.
    #[allow(clippy::unnecessary_cast)]
    pub(super) fn local_inverse_inertia(self, mass: Real) -> Option<Vector> {
        let mass = mass as f64;
        let k = self.inertia;
        let inverse = |k: Real| (1.0 / (mass * k as f64)) as Real;
        Some(Vector(inverse(k.0), inverse(k.1), inverse(k.2)))
    }
}

/// A compound prepared once from its children ([`Self::new`]) and admitted into any number of
/// worlds with [`World::add_compound`].
#[derive(Clone, Debug, PartialEq)]
pub struct PreparedCompound {
    /// As authored, in the authoring frame.
    authored: Vec<CompoundChild>,
    /// In the body frame: the principal frame at the centre of mass.
    children: Vec<CompoundChild>,
    mass: Real,
    center_of_mass: Vector,
    principal_orientation: Quaternion,
    principal_inertia: Vector,
    shape: CompoundShape,
}

type Matrix = [[f64; 3]; 3];

fn wide(v: Vector) -> [f64; 3] {
    [f64::from(v.0), f64::from(v.1), f64::from(v.2)]
}
#[allow(clippy::cast_possible_truncation)]
fn narrow(v: [f64; 3]) -> Vector {
    Vector(v[0] as Real, v[1] as Real, v[2] as Real)
}
/// The rotation matrix of a unit quaternion, in f64.
fn rotation(q: Quaternion) -> Matrix {
    let [x, y, z, w] = [q.0, q.1, q.2, q.3].map(f64::from);
    [
        [
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - z * w),
            2.0 * (x * z + y * w),
        ],
        [
            2.0 * (x * y + z * w),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - x * w),
        ],
        [
            2.0 * (x * z - y * w),
            2.0 * (y * z + x * w),
            1.0 - 2.0 * (x * x + y * y),
        ],
    ]
}
fn multiply(a: &Matrix, b: &Matrix) -> Matrix {
    std::array::from_fn(|i| std::array::from_fn(|j| (0..3).map(|k| a[i][k] * b[k][j]).sum()))
}
fn transpose(a: &Matrix) -> Matrix {
    std::array::from_fn(|i| std::array::from_fn(|j| a[j][i]))
}
fn apply(a: &Matrix, v: [f64; 3]) -> [f64; 3] {
    std::array::from_fn(|i| (0..3).map(|k| a[i][k] * v[k]).sum())
}
/// `R A Rᵀ`.
fn rotated(r: &Matrix, a: &Matrix) -> Matrix {
    multiply(&multiply(r, a), &transpose(r))
}

/// Cyclic Jacobi eigen-decomposition of a symmetric matrix: eigenvalues and the eigenvectors as
/// the columns of a rotation matrix. Off-diagonal terms below a relative 1e-12 of the diagonal
/// are already zero, so an axis-aligned tensor keeps the identity frame.
fn principal(mut a: Matrix) -> ([f64; 3], Matrix) {
    let mut v: Matrix = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    for _ in 0..64 {
        let scale = a[0][0].abs() + a[1][1].abs() + a[2][2].abs();
        let off = a[0][1].abs() + a[0][2].abs() + a[1][2].abs();
        if off <= 1e-12 * scale {
            break;
        }
        for (p, q) in [(0, 1), (0, 2), (1, 2)] {
            if a[p][q].abs() <= 1e-15 * scale {
                continue;
            }
            let theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
            let t = theta.signum() / (theta.abs() + (theta * theta + 1.0).sqrt());
            let t = if theta == 0.0 { 1.0 } else { t };
            let c = 1.0 / (t * t + 1.0).sqrt();
            let s = t * c;
            // a = Jᵀ a J, v = v J for the rotation J in the (p, q) plane.
            let mut j: Matrix = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
            j[p][p] = c;
            j[q][q] = c;
            j[p][q] = s;
            j[q][p] = -s;
            a = multiply(&multiply(&transpose(&j), &a), &j);
            a[p][q] = 0.0;
            a[q][p] = 0.0;
            v = multiply(&v, &j);
        }
    }
    // A proper rotation: flip the last axis of a reflection.
    let det = v[0][0] * (v[1][1] * v[2][2] - v[1][2] * v[2][1])
        - v[0][1] * (v[1][0] * v[2][2] - v[1][2] * v[2][0])
        + v[0][2] * (v[1][0] * v[2][1] - v[1][1] * v[2][0]);
    if det < 0.0 {
        for row in &mut v {
            row[2] = -row[2];
        }
    }
    ([a[0][0], a[1][1], a[2][2]], v)
}
/// The unit quaternion of a rotation matrix.
#[allow(clippy::cast_possible_truncation)]
fn quaternion(m: &Matrix) -> Quaternion {
    let trace = m[0][0] + m[1][1] + m[2][2];
    let q = if trace > 0.0 {
        let s = (trace + 1.0).sqrt() * 2.0;
        [
            (m[2][1] - m[1][2]) / s,
            (m[0][2] - m[2][0]) / s,
            (m[1][0] - m[0][1]) / s,
            0.25 * s,
        ]
    } else if m[0][0] > m[1][1] && m[0][0] > m[2][2] {
        let s = (1.0 + m[0][0] - m[1][1] - m[2][2]).sqrt() * 2.0;
        [
            0.25 * s,
            (m[0][1] + m[1][0]) / s,
            (m[0][2] + m[2][0]) / s,
            (m[2][1] - m[1][2]) / s,
        ]
    } else if m[1][1] > m[2][2] {
        let s = (1.0 + m[1][1] - m[0][0] - m[2][2]).sqrt() * 2.0;
        [
            (m[0][1] + m[1][0]) / s,
            0.25 * s,
            (m[1][2] + m[2][1]) / s,
            (m[0][2] - m[2][0]) / s,
        ]
    } else {
        let s = (1.0 + m[2][2] - m[0][0] - m[1][1]).sqrt() * 2.0;
        [
            (m[0][2] + m[2][0]) / s,
            (m[1][2] + m[2][1]) / s,
            0.25 * s,
            (m[1][0] - m[0][1]) / s,
        ]
    };
    let n = q.iter().map(|x| x * x).sum::<f64>().sqrt();
    let q = q.map(|x| (x / n) as Real);
    // One sign for the same rotation.
    if q[3] < 0.0 {
        Quaternion(-q[0], -q[1], -q[2], -q[3])
    } else {
        Quaternion(q[0], q[1], q[2], q[3])
    }
}

fn unit(q: Quaternion) -> Option<Quaternion> {
    let norm = (q.0 * q.0 + q.1 * q.1 + q.2 * q.2 + q.3 * q.3).sqrt();
    if !q.finite() || !norm.is_finite() || (norm - 1.0).abs() > 1e-4 {
        return None;
    }
    Some(if q.unit_to_rounding() {
        q
    } else {
        q.normalized()
    })
}

impl PreparedCompound {
    /// Validates the children and computes the compound's mass properties. Children are boxes,
    /// spheres or capsules with valid dimensions, finite positions below 1e12, unit orientations
    /// (within 1e-4; normalized) and masses in `[1e-6, 1e12)`; there are 1 to
    /// [`MAX_COMPOUND_CHILDREN`]. The result is deterministic: the same children give the same
    /// bits on the same build and target.
    pub fn new(children: &[CompoundChild]) -> Result<Self, Error> {
        if children.is_empty() || children.len() > MAX_COMPOUND_CHILDREN {
            return Err(Error::InvalidInput);
        }
        let mut authored = Vec::with_capacity(children.len());
        for child in children {
            if !matches!(
                child.shape,
                Shape::Box(_) | Shape::Sphere(_) | Shape::Capsule { .. }
            ) || !child.shape.valid_dimensions()
                || !child.position.finite()
                || child.position.abs().max_component() >= 1e12
                || !child.mass.is_finite()
                || !(1e-6..1e12).contains(&child.mass)
            {
                return Err(Error::InvalidInput);
            }
            let orientation = unit(child.orientation).ok_or(Error::InvalidInput)?;
            authored.push(CompoundChild {
                orientation,
                ..*child
            });
        }
        // Mass and centre of mass, in f64.
        let mut mass = 0.0f64;
        let mut moment = [0.0f64; 3];
        let mut parts = Vec::with_capacity(authored.len());
        for child in &authored {
            let properties = child
                .shape
                .mass_properties(child.mass)
                .map_err(|_| Error::InvalidInput)?;
            let r = rotation(child.orientation);
            let centre = apply(&r, wide(properties.local_center_of_mass));
            let centre: [f64; 3] = std::array::from_fn(|k| centre[k] + wide(child.position)[k]);
            let local: Matrix = properties.local_inertia.map(|row| row.map(f64::from));
            let m = f64::from(child.mass);
            mass += m;
            for k in 0..3 {
                moment[k] += m * centre[k];
            }
            parts.push((m, centre, rotated(&r, &local)));
        }
        let com = moment.map(|x| x / mass);
        // Inertia about the centre of mass: each part's own tensor plus the parallel-axis term
        // m (|d|² E − d dᵀ).
        let mut inertia: Matrix = [[0.0; 3]; 3];
        for (m, centre, own) in &parts {
            let d: [f64; 3] = std::array::from_fn(|k| centre[k] - com[k]);
            let dd = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
            for i in 0..3 {
                for j in 0..3 {
                    let shift = if i == j { dd } else { 0.0 } - d[i] * d[j];
                    inertia[i][j] += own[i][j] + m * shift;
                }
            }
        }
        let (_, axes) = principal(inertia);
        let principal_orientation = quaternion(&axes);
        // The moments along the rounded frame actually used, so the solver's diagonal is
        // the tensor in that frame (off-diagonal terms are rounding only).
        let r = rotation(principal_orientation);
        let body_inertia = multiply(&multiply(&transpose(&r), &inertia), &r);
        let moments = [body_inertia[0][0], body_inertia[1][1], body_inertia[2][2]];
        if !mass.is_finite()
            || mass >= 1e12
            || com.iter().any(|x| !x.is_finite())
            || moments.iter().any(|&x| !x.is_finite() || x <= 0.0)
        {
            return Err(Error::InvalidInput);
        }
        let center_of_mass = narrow(com);
        let inverse = principal_orientation.conjugate();
        let rt = transpose(&r);
        let mut radius = 0.0f64;
        let body_children: Vec<CompoundChild> = authored
            .iter()
            .map(|child| {
                let offset: [f64; 3] = std::array::from_fn(|k| wide(child.position)[k] - com[k]);
                let position = apply(&rt, offset);
                let length = position.iter().map(|x| x * x).sum::<f64>().sqrt();
                radius = radius.max(length + f64::from(child.shape.radius()));
                let orientation = inverse * child.orientation;
                CompoundChild {
                    position: narrow(position),
                    orientation: unit(orientation).unwrap_or(orientation.normalized()),
                    ..*child
                }
            })
            .collect();
        #[allow(clippy::cast_possible_truncation)]
        let narrow_up = |x: f64| {
            let y = x as Real;
            if f64::from(y) < x { y.next_up() } else { y }
        };
        // Bounds hold every child after the body-frame positions and orientations round.
        let radius = narrow_up(radius * (1.0 + 1e-6));
        let principal_inertia = narrow(moments);
        let shape = CompoundShape {
            radius,
            inertia: narrow(moments.map(|x| x / mass)),
        };
        #[allow(clippy::cast_possible_truncation)]
        let mass = mass as Real;
        if !shape.valid() || !(1e-6..1e12).contains(&mass) {
            return Err(Error::InvalidInput);
        }
        Ok(Self {
            authored,
            children: body_children,
            mass,
            center_of_mass,
            principal_orientation,
            principal_inertia,
            shape,
        })
    }
    /// The children as authored (orientations normalized), in their stable order.
    pub fn authored_children(&self) -> &[CompoundChild] {
        &self.authored
    }
    /// The children in the body frame (the principal frame at the centre of mass).
    pub fn children(&self) -> &[CompoundChild] {
        &self.children
    }
    /// The sum of the children's masses.
    pub fn mass(&self) -> Real {
        self.mass
    }
    /// The centre of mass in the authoring frame.
    pub fn center_of_mass(&self) -> Vector {
        self.center_of_mass
    }
    /// The body frame's orientation in the authoring frame: its axes are the principal axes of
    /// inertia (the identity when the authored tensor is already diagonal).
    pub fn principal_orientation(&self) -> Quaternion {
        self.principal_orientation
    }
    /// Principal moments of inertia about the centre of mass, along the body axes.
    pub fn principal_inertia(&self) -> Vector {
        self.principal_inertia
    }
    pub fn shape(&self) -> CompoundShape {
        self.shape
    }
    /// The body pose (centre of mass, principal frame) of a compound whose authoring origin is
    /// at `origin` with `orientation`.
    pub fn body_pose(&self, origin: Vector, orientation: Quaternion) -> (Vector, Quaternion) {
        (
            origin + orientation.rotate(self.center_of_mass),
            (orientation * self.principal_orientation).normalized(),
        )
    }
    /// A point given in the authoring frame (a joint's mounting point) in the body frame, as
    /// joint anchors take it.
    pub fn body_point(&self, point: Vector) -> Vector {
        self.principal_orientation
            .inverse_rotate(point - self.center_of_mass)
    }
    /// A direction given in the authoring frame (a joint's axis) in the body frame.
    pub fn body_direction(&self, direction: Vector) -> Vector {
        self.principal_orientation.inverse_rotate(direction)
    }
    /// The authoring origin's pose for a body at `position` with `orientation`: the inverse of
    /// [`Self::body_pose`] (to rounding).
    pub fn origin_pose(&self, position: Vector, orientation: Quaternion) -> (Vector, Quaternion) {
        let origin = (orientation * self.principal_orientation.conjugate()).normalized();
        (position - origin.rotate(self.center_of_mass), origin)
    }
}

/// A child of `parent` as a body of its own at its world pose, for the narrow phase and queries:
/// the parent's id, order, material, layers and flags, the child's shape, and the velocity of the
/// parent's point at the child's centre. Never stored. Its `cached_bounds` are unset: see
/// [`bounded`].
pub(super) fn child_body(parent: &Body, child: &CompoundChild) -> Body {
    crate::work::add(crate::work::Counter::CompoundChildren, 1);
    let offset = parent.orientation.rotate(child.position);
    let mut b = Body::new(child.shape, parent.position + offset, parent.mass);
    b.id = parent.id;
    b.order = parent.order;
    b.key = parent.key;
    b.orientation = parent.orientation * child.orientation;
    b.velocity = parent.velocity + parent.angular_velocity.cross(offset);
    b.angular_velocity = parent.angular_velocity;
    b.friction = parent.friction;
    b.restitution = parent.restitution;
    b.friction_combine = parent.friction_combine;
    b.restitution_combine = parent.restitution_combine;
    b.linear_damping = parent.linear_damping;
    b.angular_damping = parent.angular_damping;
    b.rotation_locked = parent.rotation_locked;
    b.layers = parent.layers;
    b.external = parent.external;
    b.sensor = parent.sensor;
    b.sleep_allowed = parent.sleep_allowed;
    b.ccd = parent.ccd;
    b.retire_on_impact = parent.retire_on_impact;
    b.linear_support = parent.linear_support;
    b.sleeping = parent.sleeping;
    b.force = parent.force;
    b
}
/// A child body with its bounds, for the paths that cull by them.
pub(super) fn bounded(mut b: Body) -> Body {
    b.cached_bounds = contact::bounds(&b);
    b
}

/// Re-expresses a manifold's arm from a child body's centre to its parent's.
pub(super) fn parent_arm(child: &Body, parent: &Body, arm: Vector) -> Vector {
    arm + (child.position - parent.position)
}

impl World {
    /// Adds a compound dynamic body built from `prepared` and returns its id. `body` supplies
    /// the pose of the body frame (see [`PreparedCompound::body_pose`]), the velocities, the
    /// key, material, layers and flags; its `shape` and `mass` are replaced by the compound's.
    /// The body must be dynamic (not external) and without `ccd` (persistent projectile CCD
    /// is for single primitives); other bodies' CCD, the per-substep sweep of fast pairs and
    /// every query resolve against its children.
    ///
    /// A pose read back from a world keeps its bits, so removing a compound body and adding it
    /// again from its read-back [`Body`] (and the same prepared compound) restores it exactly.
    pub fn add_compound(
        &mut self,
        prepared: &PreparedCompound,
        mut body: Body,
    ) -> Result<BodyId, Error> {
        body.shape = Shape::Compound(prepared.shape);
        body.mass = prepared.mass;
        if body.external || body.ccd || body.medium.is_some() {
            return Err(Error::InvalidInput);
        }
        let id = self.insert_body(body)?;
        let slot = id.slot() as usize;
        if self.compounds.len() <= slot {
            self.compounds.resize(slot + 1, None);
        }
        self.compounds[slot] = Some(Arc::new(prepared.clone()));
        Ok(id)
    }
    /// The prepared compound of live compound body `id`.
    pub fn compound(&self, id: BodyId) -> Option<&PreparedCompound> {
        let body = self.body(id)?;
        matches!(body.shape, Shape::Compound(_))
            .then(|| self.compounds[id.slot() as usize].as_deref())
            .flatten()
    }
    /// The world pose (centre, orientation) of child `child` of compound body `id`.
    pub fn compound_child_pose(
        &self,
        id: BodyId,
        child: usize,
    ) -> Result<(Vector, Quaternion), Error> {
        let body = self.body(id).ok_or(Error::MissingBody(id))?;
        let c = self
            .compound(id)
            .and_then(|c| c.children.get(child))
            .ok_or(Error::InvalidInput)?;
        Ok((
            body.position + body.orientation.rotate(c.position),
            body.orientation * c.orientation,
        ))
    }
}

/// The body-frame children of a live compound body `b` (empty for any other body), from the
/// world's compound table.
pub(super) fn children<'a>(
    compounds: &'a [Option<Arc<PreparedCompound>>],
    b: &Body,
) -> &'a [CompoundChild] {
    if !matches!(b.shape, Shape::Compound(_)) {
        return &[];
    }
    compounds
        .get(b.id.slot() as usize)
        .and_then(Option::as_deref)
        .map_or(&[], |c| &c.children)
}

/// Whether primitive or compound bodies `a` and `b` have a current contact within `margin`:
/// any pair of their children does (a primitive is its own one child).
pub(super) fn touching(
    compounds: &[Option<Arc<PreparedCompound>>],
    a: &Body,
    b: &Body,
    margin: Real,
) -> bool {
    let (ca, cb) = (children(compounds, a), children(compounds, b));
    if ca.is_empty() && cb.is_empty() {
        return contact::current(a, b, margin).is_some();
    }
    (0..ca.len().max(1)).any(|ka| {
        let child_a = ca.get(ka).map(|c| child_body(a, c));
        let pa = child_a.as_ref().unwrap_or(a);
        (0..cb.len().max(1)).any(|kb| {
            let child_b = cb.get(kb).map(|c| child_body(b, c));
            let pb = child_b.as_ref().unwrap_or(b);
            contact::current(pa, pb, margin).is_some()
        })
    })
}

#[cfg(test)]
mod tests;
