//! Joints between two bodies (#453): the first slice is one distance limit, Rapier 0.36.1's
//! `RopeJoint`. Each joint becomes one row of the existing velocity solve (the contact rows'
//! kernel with no friction), rebuilt every substep from the current poses and warm-started
//! from its last impulse. Joints link islands for sleep and wake. See
//! docs/world-api-contract.md#joints. Revolute joints (#568, `revolute.rs`) share the ids, the
//! record list's lifecycle and the island links kept here.
use super::revolute;
use super::{Body, BodyId, Constraint, Error, Order, Real, Report, Vector, World, narrow_time};

/// A joint's stable id: a creation serial, never reused within a world, so the id of a
/// removed joint is never confused with a newer one.
#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct JointId(u64);
impl JointId {
    /// The packed bits, for adapters that store ids outside the engine.
    #[must_use]
    pub const fn to_bits(self) -> u64 {
        self.0
    }
    #[must_use]
    pub const fn from_bits(bits: u64) -> Self {
        Self(bits)
    }
}

/// A rope between an anchor on body `a` and one on body `b`: the anchors are at most
/// `max_length` apart and may come closer freely (Rapier 0.36.1's `RopeJoint`, whose coupled
/// linear limit applies the maximum only). Anchors are in each body's local frame, in scene
/// units; `max_length` is in scene units.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DistanceJoint {
    pub a: BodyId,
    pub b: BodyId,
    pub anchor_a: Vector,
    pub anchor_b: Vector,
    pub max_length: Real,
}
impl DistanceJoint {
    /// A rope of `max_length` between the two bodies' centres.
    pub fn rope(a: BodyId, b: BodyId, max_length: Real) -> Self {
        Self {
            a,
            b,
            anchor_a: Vector::ZERO,
            anchor_b: Vector::ZERO,
            max_length,
        }
    }
    fn valid(&self) -> bool {
        let bounded = |v: Vector| v.finite() && v.abs().max_component() < 1e12;
        self.a != self.b
            && bounded(self.anchor_a)
            && bounded(self.anchor_b)
            && self.max_length.is_finite()
            && self.max_length > 0.0
            && self.max_length < 1e12
    }
}

#[derive(Clone, Debug)]
pub(super) struct Record {
    id: JointId,
    joint: DistanceJoint,
    /// The bodies' orders, lower first: the canonical row order is (key, id).
    key: (Order, Order),
    /// Accumulated impulse of the last solved substep, for warm starting.
    impulse: Real,
}

/// The world's joints in canonical row order: by the pair's body orders, then by id. The
/// order depends on keys and creation, never on slots or storage.
#[derive(Clone, Debug, Default)]
pub(super) struct Joints {
    records: Vec<Record>,
    next: u64,
    /// Before-images of the impulses for a step's rollback.
    saved: Vec<Real>,
    /// The record of each joint row the current substep built.
    rows: Vec<u32>,
    /// Revolute joints (#568) in canonical order: by the pair's body orders, then by id.
    pub(super) revolute: Vec<revolute::Record>,
    /// The revolute rows the current substep built.
    pub(super) prepared: Vec<revolute::Row>,
    /// Before-images of the revolute solver states for a step's rollback.
    pub(super) saved_revolute: Vec<revolute::Solved>,
    /// The body pairs joined by a revolute joint without `contacts`, sorted and deduplicated:
    /// the broad phase drops their contacts.
    no_contact: Vec<(Order, Order)>,
}
/// The joined body pairs of both joint kinds.
pub(super) struct Pairs<'a> {
    distance: std::slice::Iter<'a, Record>,
    revolute: std::slice::Iter<'a, revolute::Record>,
}
impl Iterator for Pairs<'_> {
    type Item = (Order, Order);
    fn next(&mut self) -> Option<Self::Item> {
        self.distance
            .next()
            .map(|r| r.key)
            .or_else(|| self.revolute.next().map(|r| r.key))
    }
    fn size_hint(&self) -> (usize, Option<usize>) {
        let n = self.distance.len() + self.revolute.len();
        (n, Some(n))
    }
}
impl ExactSizeIterator for Pairs<'_> {}
impl Joints {
    pub fn is_empty(&self) -> bool {
        self.records.is_empty() && self.revolute.is_empty()
    }
    /// The joined body pairs, for the island graph.
    pub fn pairs(&self) -> Pairs<'_> {
        Pairs {
            distance: self.records.iter(),
            revolute: self.revolute.iter(),
        }
    }
    fn find(&self, id: JointId) -> Option<usize> {
        self.records.iter().position(|r| r.id == id)
    }
    /// A new joint id, never reused within the world.
    pub(super) fn issue(&mut self) -> JointId {
        let id = JointId(self.next);
        self.next += 1;
        id
    }
    /// After revolute joints were added or removed: the pairs without contacts follow.
    /// O(revolute joints · log), and marks the island graph dirty.
    pub(super) fn joined_changed(&mut self) {
        self.no_contact.clear();
        self.no_contact.extend(
            self.revolute
                .iter()
                .filter(|r| !r.joint.contacts)
                .map(|r| r.key),
        );
        self.no_contact.sort_unstable();
        self.no_contact.dedup();
    }
    /// Whether the broad phase drops contacts between the bodies with orders `a < b`.
    #[inline]
    pub(super) fn contacts_disabled(&self, a: Order, b: Order) -> bool {
        self.no_contact.binary_search(&(a, b)).is_ok()
    }
    pub(super) fn any_contacts_disabled(&self) -> bool {
        !self.no_contact.is_empty()
    }
    /// Saves the impulses a step may change (O(joints), nothing without joints).
    pub fn begin(&mut self, warm_start: bool) {
        self.saved.clear();
        self.saved.extend(self.records.iter().map(|r| r.impulse));
        self.begin_revolute(warm_start);
    }
    /// Restores the impulses saved by [`Self::begin`] after a failed step.
    pub fn rollback(&mut self) {
        for (r, &impulse) in self.records.iter_mut().zip(&self.saved) {
            r.impulse = impulse;
        }
        self.rollback_revolute();
    }
}

impl World {
    /// Adds a joint and returns its new id. Both bodies must be live and distinct, at least one
    /// of them dynamic, neither retired on impact; anchors and `max_length` (positive) finite.
    /// The joined bodies wake, as Rapier's `ImpulseJointSet::insert(.., true)`.
    pub fn add_joint(&mut self, joint: DistanceJoint) -> Result<JointId, Error> {
        if !joint.valid() {
            return Err(Error::InvalidInput);
        }
        let (a, b) = (self.index(joint.a)?, self.index(joint.b)?);
        let (a, b) = (&self.bodies[a], &self.bodies[b]);
        if !(a.movable() || b.movable()) || a.retire_on_impact || b.retire_on_impact {
            return Err(Error::InvalidInput);
        }
        let key = if a.order < b.order {
            (a.order, b.order)
        } else {
            (b.order, a.order)
        };
        let id = self.joints.issue();
        // New ids are the largest: the joint goes after every joint of its pair.
        let at = self.joints.records.partition_point(|r| r.key <= key);
        self.joints.records.insert(
            at,
            Record {
                id,
                joint,
                key,
                impulse: 0.0,
            },
        );
        self.bookkeeping.graph.dirty = true;
        self.wake_joined(key);
        Ok(id)
    }
    /// Removes a joint; the bodies it joined wake. `None` for an unknown or removed id.
    pub fn remove_joint(&mut self, id: JointId) -> Option<DistanceJoint> {
        let at = self.joints.find(id)?;
        let record = self.joints.records.remove(at);
        self.bookkeeping.graph.dirty = true;
        self.wake_joined(record.key);
        Some(record.joint)
    }
    pub fn joint(&self, id: JointId) -> Option<&DistanceJoint> {
        self.joints.find(id).map(|i| &self.joints.records[i].joint)
    }
    /// The joint's accumulated impulse in its last solved substep (mass · scene units per
    /// second, along the rope; zero while slack). `None` for an unknown id.
    pub fn joint_impulse(&self, id: JointId) -> Option<Real> {
        self.joints.find(id).map(|i| self.joints.records[i].impulse)
    }
    /// The live joints in canonical row order.
    pub fn joints(&self) -> impl Iterator<Item = (JointId, &DistanceJoint)> {
        self.joints.records.iter().map(|r| (r.id, &r.joint))
    }
    pub fn joint_count(&self) -> usize {
        self.joints.records.len()
    }
    /// Wakes the islands of a joined pair's dynamic bodies.
    pub(super) fn wake_joined(&mut self, (a, b): (Order, Order)) {
        self.sort_storage();
        for order in [a, b] {
            if self
                .index_of_order(order)
                .is_some_and(|i| self.bodies[i].movable())
            {
                self.wake_island(order);
            }
        }
    }
    /// `remove_body` of the body with `order`: its joints go too, as Rapier removes a body's
    /// joints with it, and the bodies they held wake. O(joints).
    pub(super) fn remove_body_joints(&mut self, order: Order) {
        if self.joints.is_empty() {
            return;
        }
        let mut partners = Vec::new();
        let mut keep = |key: (Order, Order)| {
            crate::work::add(crate::work::Counter::JointRows, 1);
            if key.0 == order || key.1 == order {
                partners.push(if key.0 == order { key.1 } else { key.0 });
                false
            } else {
                true
            }
        };
        self.joints.records.retain(|r| keep(r.key));
        let revolute = self.joints.revolute.len();
        self.joints.revolute.retain(|r| keep(r.key));
        if self.joints.revolute.len() != revolute {
            self.joints.joined_changed();
        }
        if partners.is_empty() {
            return;
        }
        self.bookkeeping.graph.dirty = true;
        partners.sort_unstable();
        partners.dedup();
        self.sort_storage();
        for order in partners {
            if self
                .index_of_order(order)
                .is_some_and(|i| self.bodies[i].movable())
            {
                self.wake_island(order);
            }
        }
    }
    /// [`World::remove_bodies`]: drops the joints of the bodies with `orders` (sorted) in one
    /// pass and returns their surviving partners, whose islands wake.
    pub(super) fn remove_batch_joints(&mut self, orders: &[Order]) -> Vec<Order> {
        let mut roots = Vec::new();
        if self.joints.is_empty() {
            return roots;
        }
        let before = (self.joints.records.len(), self.joints.revolute.len());
        let mut keep = |(a, b): (Order, Order)| {
            crate::work::add(crate::work::Counter::JointRows, 1);
            let (gone_a, gone_b) = (
                orders.binary_search(&a).is_ok(),
                orders.binary_search(&b).is_ok(),
            );
            if gone_a && !gone_b {
                roots.push(b);
            } else if gone_b && !gone_a {
                roots.push(a);
            }
            !(gone_a || gone_b)
        };
        self.joints.records.retain(|r| keep(r.key));
        self.joints.revolute.retain(|r| keep(r.key));
        if self.joints.revolute.len() != before.1 {
            self.joints.joined_changed();
        }
        self.bookkeeping.graph.dirty |=
            (self.joints.records.len(), self.joints.revolute.len()) != before;
        roots
    }
    /// Contacts phase: a sleeping body held by a moving kinematic body wakes (its island
    /// roots), as a moving kinematic contact wakes what it touches.
    pub(super) fn joint_roots(&self, roots: &mut Vec<Order>) {
        let distance = self.joints.records.iter().map(|r| (r.joint.a, r.joint.b));
        let revolute = self.joints.revolute.iter().map(|r| (r.joint.a, r.joint.b));
        for (a, b) in distance.chain(revolute) {
            crate::work::add(crate::work::Counter::JointRows, 1);
            let (Ok(a), Ok(b)) = (self.index(a), self.index(b)) else {
                continue;
            };
            let (a, b) = (&self.bodies[a], &self.bodies[b]);
            let driven = |x: &Body| {
                x.external && (x.velocity != Vector::ZERO || x.angular_velocity != Vector::ZERO)
            };
            if a.sleeping && a.movable() && driven(b) {
                roots.push(a.order);
            }
            if b.sleeping && b.movable() && driven(a) {
                roots.push(b.order);
            }
        }
    }
    /// Constraint phase: one row per joint with an awake dynamic side, appended after the
    /// contact rows in canonical order. The row is a frictionless contact row along the rope:
    /// `n` points from `b`'s anchor to `a`'s, its separation is the slack `max_length −
    /// distance`. A slack rope admits closing up to the limit within the substep (as Rapier's
    /// `rhs_wo_bias`); a stretched one corrects the excess within the substep, at most 3 m/s
    /// (in `length_unit`, Rapier's `max_corrective_velocity`), as Rapier's rigid joint
    /// softness does. The last impulse warm-starts the row, scaled like contact history.
    pub(super) fn joint_rows(&mut self, rows: &mut Vec<Constraint>, h: Real, report: &mut Report) {
        self.joints.rows.clear();
        if self.joints.is_empty() {
            return;
        }
        let max_corrective = 3.0 * self.config.length_unit;
        let scale = if self.scene.warm_start && self.last_h > 0.0 {
            (h / narrow_time(self.last_h)).clamp(0.0, 2.0)
        } else {
            0.0
        };
        for (k, r) in self.joints.records.iter().enumerate() {
            crate::work::add(crate::work::Counter::JointRows, 1);
            let (Ok(i), Ok(j)) = (self.index(r.joint.a), self.index(r.joint.b)) else {
                continue;
            };
            let prepared = [self.responses[i], self.responses[j]];
            let response = [
                prepared[0].inverse_mass > 0.0,
                prepared[1].inverse_mass > 0.0,
            ];
            if response == [false, false] {
                continue;
            }
            let (a, b) = (&self.bodies[i], &self.bodies[j]);
            let ra = a.orientation.rotate(r.joint.anchor_a);
            let rb = b.orientation.rotate(r.joint.anchor_b);
            let d = (b.position + rb) - (a.position + ra);
            let distance = d.length();
            // Coincident anchors have no direction: Rapier's row is zero there too.
            if distance <= Real::EPSILON * (1.0 + r.joint.max_length) {
                continue;
            }
            let n = d * (-1.0 / distance);
            let spin = prepared[0].spins()
                || prepared[1].spins()
                || a.angular_velocity != Vector::ZERO
                || b.angular_velocity != Vector::ZERO;
            let mut angular = [[Vector::ZERO; 3]; 2];
            let arms = [ra, rb];
            let mut k_n = 0.0;
            for side in 0..2 {
                if response[side] {
                    if spin && prepared[side].spins() {
                        angular[side][0] = prepared[side].inertia(arms[side].cross(n), report);
                    }
                    k_n += prepared[side].inverse_mass + arms[side].cross(n).dot(angular[side][0]);
                }
            }
            if k_n <= 1e-14 {
                continue;
            }
            let sep = r.joint.max_length - distance;
            let bias = if sep > 0.0 {
                -sep / h
            } else {
                (-sep / h).min(max_corrective)
            };
            let t1 = if n.0.abs() < 0.7 {
                n.cross(Vector::X).unit()
            } else {
                n.cross(Vector::Y).unit()
            };
            let impulse = r.impulse * scale;
            report.max_penetration = report.max_penetration.max((-sep).max(0.0));
            let mut constraint = Constraint {
                a: i,
                b: j,
                n,
                ra,
                rb,
                t1,
                t2: n.cross(t1),
                angular,
                finite_response: false,
                normal_mass: 1.0 / k_n,
                // Frictionless: the tangent impulses stay zero.
                tangent_mass: [1.0 / k_n; 2],
                bias,
                friction: 0.0,
                normal_impulse: impulse,
                tangent_impulse: [0.0; 2],
                sep,
                swept: false,
                response,
                spin,
                offset: b.position - a.position,
                restitution: 0.0,
                bounce: 0.0,
            };
            constraint.prepare_finite_response(&self.responses);
            rows.push(constraint);
            self.joints.rows.push(k as u32);
        }
    }
    /// After the solve: each joint row's impulse becomes its warm-start history, and a taut
    /// rope holding a body from above supports it for sleep, as a contact below it does.
    pub(super) fn joint_writeback(
        &mut self,
        rows: &[Constraint],
        support_edges: &mut Vec<(usize, usize)>,
    ) {
        debug_assert_eq!(rows.len(), self.joints.rows.len());
        let up = (-self.scene.gravity).unit();
        let slop = self.scene.contact_slop;
        for (c, &k) in rows.iter().zip(&self.joints.rows) {
            self.joints.records[k as usize].impulse = c.normal_impulse;
            if c.sep <= 2.0 * slop {
                if c.n.dot(up) > 0.5 {
                    support_edges.push((c.a, c.b));
                }
                if c.n.dot(up) < -0.5 {
                    support_edges.push((c.b, c.a));
                }
            }
        }
    }
}
