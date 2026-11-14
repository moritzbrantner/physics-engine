//! Revolute joints (#568): hinges, servos and torque-limited rotors between two bodies, for
//! the shooter's robots (moenarch-games/shooter#73). A joint holds an anchor on each body
//! together and one local axis on each body aligned, leaving one rotational degree of freedom,
//! with optional angle limits and an optional motor (a target rate or a target angle, each with
//! a maximum torque).
//!
//! Each substep builds one row per joint from the current poses: a 3 × 3 block for the anchor
//! point, a 2 × 2 block for the axis alignment, and scalar rows along the axis for the motor
//! (impulse bounded by `max_torque · h`) and each limit (one-sided). The rows are solved by the
//! same sequential-impulse passes as the contacts, joints last in each pass so a structure holds,
//! and warm-start from the previous substep's impulses (and from the previous step's when
//! `Config::warm_start` is on). See docs/world-api-contract.md#revolute-joints.
use super::{BodyId, Error, JointId, Order, Real, Report, Vector, World, narrow_time};
use std::f32::consts::PI;

/// Fraction of a joint's position error corrected per substep, as a velocity bias.
const ERP: Real = 0.5;
/// Largest anchor and axis correction speed, in m/s (scaled by `length_unit`) and rad/s:
/// Rapier's `max_corrective_velocity`, as the rope uses.
const MAX_CORRECTIVE: Real = 3.0;
/// A servo closes its angle error with this time constant (seconds), at most `max_rate`:
/// it slows smoothly into its target instead of stopping within one substep.
const SERVO_TIME: Real = 0.05;
/// Motors give up this fraction of the rate change their impulse would make on its own: a
/// stiff but not rigid drive, so several motors driving one body through sticking contacts (a
/// robot's wheels) share the load instead of locking in whatever split the solver found
/// first (Rapier's motors are soft too). Under a steady load the rate falls short of the
/// target by this fraction of the impulse's rate change per substep.
const MOTOR_GIVE: Real = 0.01;

/// What drives a revolute joint about its axis. Torques are N·m (mass · scene units² / s²);
/// angles radians; rates radians per second, of body `b` relative to body `a` about the axis.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub enum Motor {
    /// A free hinge (limits still apply).
    #[default]
    Off,
    /// A rotor or wheel drive: the joint turns at `rate` unless that takes more than
    /// `max_torque`, when it stalls or slows under the load.
    Velocity { rate: Real, max_torque: Real },
    /// A servo: the joint turns towards `angle` (wrapped to `[-π, π]`, along the shorter way)
    /// at up to `max_rate`, with at most `max_torque`, and holds it there.
    Position {
        angle: Real,
        max_rate: Real,
        max_torque: Real,
    },
}
impl Motor {
    fn valid(self) -> bool {
        let bounded = |x: Real| x.is_finite() && x.abs() < 1e12;
        match self {
            Self::Off => true,
            Self::Velocity { rate, max_torque } => {
                bounded(rate) && bounded(max_torque) && max_torque >= 0.0
            }
            Self::Position {
                angle,
                max_rate,
                max_torque,
            } => {
                bounded(angle)
                    && bounded(max_rate)
                    && max_rate >= 0.0
                    && bounded(max_torque)
                    && max_torque >= 0.0
            }
        }
    }
}

/// A hinge between body `a` and body `b` (Rapier 0.36.1's `RevoluteJoint`). Anchors, axes and
/// references are in each body's local frame: for a compound body, the body frame at its centre
/// of mass ([`PreparedCompound::body_point`](super::PreparedCompound::body_point) and
/// [`PreparedCompound::body_direction`](super::PreparedCompound::body_direction) convert from
/// the authoring frame). The anchors coincide and the axes align in the world; the joint angle is
/// the turn of `reference_b` from `reference_a` about the axis (right-handed), zero where they
/// coincide.
///
/// [`World::add_revolute_joint`] normalizes the axes and makes each reference a unit vector
/// perpendicular to its axis.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RevoluteJoint {
    pub a: BodyId,
    pub b: BodyId,
    pub anchor_a: Vector,
    pub anchor_b: Vector,
    pub axis_a: Vector,
    pub axis_b: Vector,
    pub reference_a: Vector,
    pub reference_b: Vector,
    /// `[min, max]` angle, radians, `-π <= min <= max <= π`; `None` turns freely.
    pub limits: Option<[Real; 2]>,
    pub motor: Motor,
    /// Whether the two joined bodies collide with each other. Default false: a wheel or turret
    /// mounted against its chassis does not touch it (Rapier's `contacts_enabled(false)`).
    pub contacts: bool,
}
impl RevoluteJoint {
    /// A free hinge about `axis` (the same local axis on both bodies) through both bodies'
    /// centres. The references are [`Self::perpendicular`]`(axis)` on both, so the angle is zero in the
    /// pose where the two bodies' frames agree, as for Rapier's `RevoluteJointBuilder::new`.
    pub fn new(a: BodyId, b: BodyId, axis: Vector) -> Self {
        let reference = perpendicular(axis);
        Self {
            a,
            b,
            anchor_a: Vector::ZERO,
            anchor_b: Vector::ZERO,
            axis_a: axis,
            axis_b: axis,
            reference_a: reference,
            reference_b: reference,
            limits: None,
            motor: Motor::Off,
            contacts: false,
        }
    }
    /// The unit vector perpendicular to `axis` (nonzero) that the shortest rotation taking +X
    /// onto `axis` takes +Y to, as Rapier completes a revolute frame; −Y for `axis` along −X.
    pub fn perpendicular(axis: Vector) -> Vector {
        perpendicular(axis)
    }
    #[must_use]
    pub fn anchors(mut self, anchor_a: Vector, anchor_b: Vector) -> Self {
        self.anchor_a = anchor_a;
        self.anchor_b = anchor_b;
        self
    }
    /// Different local axes on the two bodies; the references become [`Self::perpendicular`] of each.
    #[must_use]
    pub fn axes(mut self, axis_a: Vector, axis_b: Vector) -> Self {
        self.axis_a = axis_a;
        self.axis_b = axis_b;
        self.reference_a = perpendicular(axis_a);
        self.reference_b = perpendicular(axis_b);
        self
    }
    #[must_use]
    pub fn limits(mut self, min: Real, max: Real) -> Self {
        self.limits = Some([min, max]);
        self
    }
    #[must_use]
    pub fn motor(mut self, motor: Motor) -> Self {
        self.motor = motor;
        self
    }
    /// The joint with unit axes and unit references perpendicular to them, or `None` when a
    /// value is out of range.
    fn normalized(mut self) -> Option<Self> {
        let bounded = |v: Vector| v.finite() && v.abs().max_component() < 1e12;
        if self.a == self.b
            || !bounded(self.anchor_a)
            || !bounded(self.anchor_b)
            || !self.motor.valid()
        {
            return None;
        }
        if let Some([min, max]) = self.limits
            && !(min.is_finite() && max.is_finite() && -PI <= min && min <= max && max <= PI)
        {
            return None;
        }
        for (axis, reference) in [
            (&mut self.axis_a, &mut self.reference_a),
            (&mut self.axis_b, &mut self.reference_b),
        ] {
            let length = axis.length();
            if !bounded(*axis) || !bounded(*reference) || length <= 1e-6 {
                return None;
            }
            *axis = *axis * (1.0 / length);
            let r = *reference - *axis * axis.dot(*reference);
            let r_length = r.length();
            if r_length <= 1e-3 * reference.length() {
                return None;
            }
            *reference = r * (1.0 / r_length);
        }
        Some(self)
    }
}

/// The unit vector perpendicular to `axis` that the shortest rotation taking +X onto `axis`
/// takes +Y to (Rapier's revolute frame), for a nonzero `axis`. For `axis` along −X, −Y.
fn perpendicular(axis: Vector) -> Vector {
    let u = axis.unit();
    let w = 1.0 + u.0;
    if w.is_nan() || w <= 1e-6 {
        return Vector(0.0, -1.0, 0.0);
    }
    // q = (X × u, 1 + X · u), normalized: rotates X onto u along the shorter arc.
    let q = super::Quaternion(0.0, -u.2, u.1, w).normalized();
    q.rotate(Vector::Y)
}

/// A revolute joint's state after the last step, read back for the game's energy and stall
/// rules.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RevoluteState {
    /// Radians in `[-π, π]`: the turn of `b`'s reference from `a`'s about `a`'s axis.
    pub angle: Real,
    /// Radians per second: `b`'s angular velocity relative to `a`'s about `a`'s axis.
    pub rate: Real,
    /// The motor's torque about the axis averaged over the last step, signed like `rate`
    /// (N·m; zero without a motor or while the bodies sleep).
    pub motor_torque: Real,
    /// Whether the motor's torque was at its `max_torque` in the last substep of the last step:
    /// with `rate` far from the motor's target, a stall.
    pub saturated: bool,
}

/// The warm-start impulses of a joint's rows.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub(super) struct Impulses {
    linear: Vector,
    /// The axis-alignment impulse as a world vector (perpendicular to the axis when solved).
    angular: Vector,
    motor: Real,
    lower: Real,
    upper: Real,
}

/// A joint's solver state: what a step changes and a failed step restores.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub(super) struct Solved {
    impulses: Impulses,
    /// The motor impulses summed over the current step's substeps.
    motor_sum: Real,
    torque: Real,
    saturated: bool,
}

#[derive(Clone, Debug)]
pub(super) struct Record {
    pub id: JointId,
    pub joint: RevoluteJoint,
    /// The bodies' orders, lower first: the canonical order is (key, id).
    pub key: (Order, Order),
    pub solved: Solved,
}

/// One joint's prepared rows for a substep.
#[derive(Clone, Copy, Debug)]
pub(super) struct Row {
    pub a: usize,
    pub b: usize,
    record: u32,
    write: [bool; 2],
    inverse_mass: [Real; 2],
    ra: Vector,
    rb: Vector,
    /// Per side, the angular velocity change per unit anchor impulse along X, Y and Z
    /// (`I⁻¹ (r × e)`), prepared once per substep as the contact rows' `angular`.
    lever: [[Vector; 3]; 2],
    /// Rows of the inverse of the anchor block's effective mass.
    linear_inverse: [Vector; 3],
    linear_bias: Vector,
    tangents: [Vector; 2],
    /// Per side, `I⁻¹ t` for each tangent.
    tangent_turn: [[Vector; 2]; 2],
    /// The symmetric inverse of the alignment block: [00, 01, 11].
    angular_inverse: [Real; 3],
    angular_bias: [Real; 2],
    axis: Vector,
    /// Per side, `I⁻¹ axis`.
    axis_turn: [Vector; 2],
    axis_mass: Real,
    /// The motor row's softened mass and the rate it gives up per unit impulse (see
    /// `MOTOR_GIVE`).
    motor_mass: Real,
    motor_give: Real,
    motor: bool,
    motor_target: Real,
    motor_bound: Real,
    limited: bool,
    lower_bias: Real,
    upper_bias: Real,
    linear: Vector,
    angular: [Real; 2],
    motor_impulse: Real,
    lower: Real,
    upper: Real,
}

/// `x` wrapped to `[-π, π]`.
fn wrap(x: Real) -> Real {
    let tau = 2.0 * PI;
    let y = x - tau * (x / tau).round();
    y.clamp(-PI, PI)
}

/// The joint angle and the world axis for bodies at the given orientations.
fn angle(joint: &RevoluteJoint, a: super::Quaternion, b: super::Quaternion) -> (Real, Vector) {
    let axis = a.rotate(joint.axis_a);
    let (ra, rb) = (a.rotate(joint.reference_a), b.rotate(joint.reference_b));
    (ra.cross(rb).dot(axis).atan2(ra.dot(rb)), axis)
}

/// Velocity of a body for the joint rows: immovable geometry contributes none.
fn velocities(b: &super::Body) -> (Vector, Vector) {
    if b.mass == 0.0 {
        (Vector::ZERO, Vector::ZERO)
    } else {
        (b.velocity, b.angular_velocity)
    }
}

impl World {
    /// Adds a revolute joint and returns its id (ids are shared with [`World::add_joint`]). Both
    /// bodies must be live and distinct, at least one of them dynamic, neither retired on impact;
    /// anchors finite, axes and references nonzero and not parallel, limits within `[-π, π]` and
    /// ordered, motor values finite with nonnegative maxima. The joined bodies wake.
    pub fn add_revolute_joint(&mut self, joint: RevoluteJoint) -> Result<JointId, Error> {
        let joint = joint.normalized().ok_or(Error::InvalidInput)?;
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
        let records = &mut self.joints.revolute;
        let at = records.partition_point(|r| r.key <= key);
        records.insert(
            at,
            Record {
                id,
                joint,
                key,
                solved: Solved::default(),
            },
        );
        self.joints.joined_changed();
        self.bookkeeping.graph.dirty = true;
        self.wake_joined(key);
        Ok(id)
    }
    /// Removes a revolute joint; the bodies it joined wake. `None` for an unknown or removed id
    /// (or a distance joint's).
    pub fn remove_revolute_joint(&mut self, id: JointId) -> Option<RevoluteJoint> {
        let at = self.joints.find_revolute(id)?;
        let record = self.joints.revolute.remove(at);
        self.joints.joined_changed();
        self.bookkeeping.graph.dirty = true;
        self.wake_joined(record.key);
        Some(record.joint)
    }
    /// The revolute joint `id` as stored (unit axes and references).
    pub fn revolute_joint(&self, id: JointId) -> Option<&RevoluteJoint> {
        self.joints
            .find_revolute(id)
            .map(|i| &self.joints.revolute[i].joint)
    }
    /// The live revolute joints in canonical order (by their bodies' order, then id).
    pub fn revolute_joints(&self) -> impl Iterator<Item = (JointId, &RevoluteJoint)> {
        self.joints.revolute.iter().map(|r| (r.id, &r.joint))
    }
    /// Sets joint `id`'s motor, as a game does every tick with its commanded rate or angle. The
    /// joined bodies wake when the motor changes.
    pub fn set_revolute_motor(&mut self, id: JointId, motor: Motor) -> Result<(), Error> {
        if !motor.valid() {
            return Err(Error::InvalidInput);
        }
        let at = self.joints.find_revolute(id).ok_or(Error::InvalidInput)?;
        let record = &mut self.joints.revolute[at];
        if record.joint.motor != motor {
            record.joint.motor = motor;
            let key = record.key;
            self.wake_joined(key);
        }
        Ok(())
    }
    /// Sets or clears joint `id`'s angle limits (`-π <= min <= max <= π`). The joined bodies
    /// wake when the limits change.
    pub fn set_revolute_limits(
        &mut self,
        id: JointId,
        limits: Option<[Real; 2]>,
    ) -> Result<(), Error> {
        let at = self.joints.find_revolute(id).ok_or(Error::InvalidInput)?;
        let record = &mut self.joints.revolute[at];
        let joint = RevoluteJoint {
            limits,
            ..record.joint
        }
        .normalized()
        .ok_or(Error::InvalidInput)?;
        if record.joint.limits != limits {
            record.joint = joint;
            let key = record.key;
            self.wake_joined(key);
        }
        Ok(())
    }
    /// Joint `id`'s angle and rate at the bodies' current poses and velocities, and its motor's
    /// torque over the last step. `None` for an unknown id.
    pub fn revolute_state(&self, id: JointId) -> Option<RevoluteState> {
        let record = &self.joints.revolute[self.joints.find_revolute(id)?];
        let (a, b) = (
            &self.bodies[self.index(record.joint.a).ok()?],
            &self.bodies[self.index(record.joint.b).ok()?],
        );
        let (angle, axis) = angle(&record.joint, a.orientation, b.orientation);
        let rate = (velocities(b).1 - velocities(a).1).dot(axis);
        Some(RevoluteState {
            angle,
            rate,
            motor_torque: record.solved.torque,
            saturated: record.solved.saturated,
        })
    }

    /// Constraint phase: one row per revolute joint with an awake dynamic side, in canonical
    /// order, into `self.joints.prepared`.
    pub(super) fn revolute_rows(&mut self, h: Real, report: &mut Report) {
        let mut rows = std::mem::take(&mut self.joints.prepared);
        rows.clear();
        if self.joints.revolute.is_empty() {
            self.joints.prepared = rows;
            return;
        }
        let max_corrective = MAX_CORRECTIVE * self.config.length_unit;
        // Warm starting within the step always; across steps only with `Config::warm_start`
        // (the impulses were cleared at the step's start otherwise).
        let scale = if self.last_h > 0.0 {
            (h / narrow_time(self.last_h)).clamp(0.0, 2.0)
        } else {
            0.0
        };
        for (k, r) in self.joints.revolute.iter().enumerate() {
            crate::work::add(crate::work::Counter::JointRows, 1);
            let (Ok(i), Ok(j)) = (self.index(r.joint.a), self.index(r.joint.b)) else {
                continue;
            };
            let response = [self.responses[i], self.responses[j]];
            let write = [
                response[0].inverse_mass > 0.0,
                response[1].inverse_mass > 0.0,
            ];
            if write == [false, false] {
                continue;
            }
            let (a, b) = (&self.bodies[i], &self.bodies[j]);
            let joint = &r.joint;
            let ra = a.orientation.rotate(joint.anchor_a);
            let rb = b.orientation.rotate(joint.anchor_b);
            let inverse_mass = [response[0].inverse_mass, response[1].inverse_mass];
            let units = [Vector::X, Vector::Y, Vector::Z];
            let lever = [
                units.map(|e| response[0].turn(ra.cross(e))),
                units.map(|e| response[1].turn(rb.cross(e))),
            ];
            // Effective mass of the anchor block: K p = (1/ma + 1/mb) p + Σ (I⁻¹ (r × p)) × r.
            let column = |k: usize| {
                units[k] * (inverse_mass[0] + inverse_mass[1])
                    + lever[0][k].cross(ra)
                    + lever[1][k].cross(rb)
            };
            let Some(linear_inverse) = inverse3([column(0), column(1), column(2)]) else {
                continue;
            };
            let error = (b.position + rb) - (a.position + ra);
            let mut linear_bias = error * (-ERP / h);
            let speed = linear_bias.length();
            if speed > max_corrective {
                linear_bias = linear_bias * (max_corrective / speed);
            }
            let (theta, axis) = angle(joint, a.orientation, b.orientation);
            let axis_b = b.orientation.rotate(joint.axis_b);
            let t0 = if axis.0.abs() < 0.7 {
                axis.cross(Vector::X).unit()
            } else {
                axis.cross(Vector::Y).unit()
            };
            let tangents = [t0, axis.cross(t0)];
            let tangent_turn = [
                tangents.map(|t| response[0].turn(t)),
                tangents.map(|t| response[1].turn(t)),
            ];
            let axis_turn = [response[0].turn(axis), response[1].turn(axis)];
            let (k00, k01, k11) = (
                tangents[0].dot(tangent_turn[0][0] + tangent_turn[1][0]),
                tangents[0].dot(tangent_turn[0][1] + tangent_turn[1][1]),
                tangents[1].dot(tangent_turn[0][1] + tangent_turn[1][1]),
            );
            let det = k00 * k11 - k01 * k01;
            let angular_inverse = if det > 1e-12 * (k00 * k11).max(Real::MIN_POSITIVE) {
                [k11 / det, -k01 / det, k00 / det]
            } else {
                [0.0; 3]
            };
            let misalignment = axis.cross(axis_b);
            let correct = |e: Real| (-ERP * e / h).clamp(-MAX_CORRECTIVE, MAX_CORRECTIVE);
            let angular_bias = [
                correct(misalignment.dot(tangents[0])),
                correct(misalignment.dot(tangents[1])),
            ];
            let k_axis = axis.dot(axis_turn[0] + axis_turn[1]);
            let axis_mass = if k_axis > 1e-12 { 1.0 / k_axis } else { 0.0 };
            let (motor, motor_target, motor_bound) = match joint.motor {
                Motor::Off => (false, 0.0, 0.0),
                Motor::Velocity { rate, max_torque } => (true, rate, max_torque * h),
                Motor::Position {
                    angle,
                    max_rate,
                    max_torque,
                } => (
                    true,
                    (wrap(angle - theta) / h.max(SERVO_TIME)).clamp(-max_rate, max_rate),
                    max_torque * h,
                ),
            };
            let motor = motor && axis_mass > 0.0;
            let limit_bias = |c: Real| if c > 0.0 { -c / h } else { -ERP * c / h };
            let (limited, lower_bias, upper_bias) = match joint.limits {
                Some([min, max]) if axis_mass > 0.0 => {
                    (true, limit_bias(theta - min), limit_bias(max - theta))
                }
                _ => (false, 0.0, 0.0),
            };
            let warm = r.solved.impulses;
            rows.push(Row {
                a: i,
                b: j,
                record: k as u32,
                write,
                inverse_mass,
                ra,
                rb,
                lever,
                linear_inverse,
                linear_bias,
                tangents,
                tangent_turn,
                angular_inverse,
                angular_bias,
                axis,
                axis_turn,
                axis_mass,
                motor_mass: if k_axis > 1e-12 {
                    1.0 / (k_axis * (1.0 + MOTOR_GIVE))
                } else {
                    0.0
                },
                motor_give: MOTOR_GIVE * k_axis,
                motor,
                motor_target,
                motor_bound,
                limited,
                lower_bias,
                upper_bias,
                linear: warm.linear * scale,
                angular: if angular_inverse == [0.0; 3] {
                    [0.0; 2]
                } else {
                    [
                        warm.angular.dot(tangents[0]) * scale,
                        warm.angular.dot(tangents[1]) * scale,
                    ]
                },
                motor_impulse: if motor {
                    (warm.motor * scale).clamp(-motor_bound, motor_bound)
                } else {
                    0.0
                },
                lower: if limited { warm.lower * scale } else { 0.0 },
                upper: if limited { warm.upper * scale } else { 0.0 },
            });
        }
        report.joint_rows += rows.len() as u64;
        self.joints.prepared = rows;
    }

    /// Solve phase: applies the revolute rows' warm-start impulses, journalling their bodies.
    pub(super) fn warm_start_revolute(&mut self) {
        for r in &self.joints.prepared {
            for (side, i) in [(0, r.a), (1, r.b)] {
                if r.write[side] {
                    self.transaction.body(i, &self.bodies[i]);
                }
            }
            apply(
                &mut self.bodies,
                r,
                r.linear,
                r.angular,
                r.motor_impulse + r.lower - r.upper,
            );
        }
    }

    /// After the solve: each row's impulses become its joint's warm start; the motor's add to
    /// the step's sum. A joint holds its bodies together, so support for sleep crosses it
    /// both ways.
    pub(super) fn revolute_writeback(&mut self, support_edges: &mut Vec<(usize, usize)>) {
        for r in &self.joints.prepared {
            let solved = &mut self.joints.revolute[r.record as usize].solved;
            solved.impulses = Impulses {
                linear: r.linear,
                angular: r.tangents[0] * r.angular[0] + r.tangents[1] * r.angular[1],
                motor: r.motor_impulse,
                lower: r.lower,
                upper: r.upper,
            };
            solved.motor_sum += r.motor_impulse;
            solved.saturated = r.motor
                && r.motor_bound > 0.0
                && r.motor_impulse.abs() >= r.motor_bound * (1.0 - 1e-4);
            support_edges.push((r.a, r.b));
            support_edges.push((r.b, r.a));
        }
    }
}

/// One sequential-impulse pass over the revolute rows: per joint the axis alignment, the
/// anchor, the motor and the limits, in that order (the motor sees the structure's response
/// and the limits have the last word).
pub(super) fn solve_pass(bodies: &mut [super::Body], rows: &mut [Row]) {
    for r in rows.iter_mut() {
        let (mut va, mut wa) = velocities(&bodies[r.a]);
        let (mut vb, mut wb) = velocities(&bodies[r.b]);
        let [m00, m01, m11] = r.angular_inverse;
        if m00 != 0.0 || m11 != 0.0 {
            let w = wb - wa;
            let e = [
                r.angular_bias[0] - w.dot(r.tangents[0]),
                r.angular_bias[1] - w.dot(r.tangents[1]),
            ];
            let d = [m00 * e[0] + m01 * e[1], m01 * e[0] + m11 * e[1]];
            r.angular[0] += d[0];
            r.angular[1] += d[1];
            let [ta, tb] = r.tangent_turn;
            wa -= ta[0] * d[0] + ta[1] * d[1];
            wb += tb[0] * d[0] + tb[1] * d[1];
        }
        let relative = vb + wb.cross(r.rb) - va - wa.cross(r.ra);
        let e = r.linear_bias - relative;
        let [x, y, z] = r.linear_inverse;
        let p = Vector(x.dot(e), y.dot(e), z.dot(e));
        r.linear += p;
        let [la, lb] = r.lever;
        va -= p * r.inverse_mass[0];
        wa -= la[0] * p.0 + la[1] * p.1 + la[2] * p.2;
        vb += p * r.inverse_mass[1];
        wb += lb[0] * p.0 + lb[1] * p.1 + lb[2] * p.2;
        let [aa, ab] = r.axis_turn;
        if r.motor {
            let rate = (wb - wa).dot(r.axis);
            let old = r.motor_impulse;
            r.motor_impulse = (old + (r.motor_target - rate - old * r.motor_give) * r.motor_mass)
                .clamp(-r.motor_bound, r.motor_bound);
            let d = r.motor_impulse - old;
            wa -= aa * d;
            wb += ab * d;
        }
        if r.limited {
            let rate = (wb - wa).dot(r.axis);
            let old = r.lower;
            r.lower = (old + (r.lower_bias - rate) * r.axis_mass).max(0.0);
            let d = r.lower - old;
            wa -= aa * d;
            wb += ab * d;
            let rate = (wb - wa).dot(r.axis);
            let old = r.upper;
            r.upper = (old + (r.upper_bias + rate) * r.axis_mass).max(0.0);
            let d = old - r.upper;
            wa -= aa * d;
            wb += ab * d;
        }
        if r.write[0] {
            let body = &mut bodies[r.a];
            body.velocity = va;
            body.angular_velocity = wa;
        }
        if r.write[1] {
            let body = &mut bodies[r.b];
            body.velocity = vb;
            body.angular_velocity = wb;
        }
    }
}

/// Applies the warm-start impulses: `p` at the anchors, `t` along the two tangents and `s` along
/// the axis (to `b`; their opposites to `a`).
fn apply(bodies: &mut [super::Body], r: &Row, p: Vector, t: [Real; 2], s: Real) {
    for side in 0..2 {
        if !r.write[side] {
            continue;
        }
        let (i, sign) = if side == 0 { (r.a, -1.0) } else { (r.b, 1.0) };
        let l = r.lever[side];
        let tt = r.tangent_turn[side];
        let body = &mut bodies[i];
        body.velocity += p * (sign * r.inverse_mass[side]);
        body.angular_velocity += (l[0] * p.0
            + l[1] * p.1
            + l[2] * p.2
            + tt[0] * t[0]
            + tt[1] * t[1]
            + r.axis_turn[side] * s)
            * sign;
    }
}

/// The rows of the inverse of the 3 × 3 matrix with columns `c`, or `None` when it is singular.
fn inverse3(c: [Vector; 3]) -> Option<[Vector; 3]> {
    let r = [c[1].cross(c[2]), c[2].cross(c[0]), c[0].cross(c[1])];
    let det = c[0].dot(r[0]);
    let scale = c[0].length() * c[1].length() * c[2].length();
    if !(det.is_finite() && det > 1e-12 * scale) || scale == 0.0 {
        return None;
    }
    let inverse = 1.0 / det;
    Some([r[0] * inverse, r[1] * inverse, r[2] * inverse])
}

impl super::joint::Joints {
    pub(super) fn find_revolute(&self, id: JointId) -> Option<usize> {
        self.revolute.iter().position(|r| r.id == id)
    }
    /// Step start: the solver states a failed step restores; without warm starting across
    /// steps, the impulses start from zero. O(revolute joints).
    pub(super) fn begin_revolute(&mut self, warm_start: bool) {
        self.saved_revolute.clear();
        self.saved_revolute
            .extend(self.revolute.iter().map(|r| r.solved));
        for r in &mut self.revolute {
            if !warm_start {
                r.solved.impulses = Impulses::default();
            }
            r.solved.motor_sum = 0.0;
        }
    }
    pub(super) fn rollback_revolute(&mut self) {
        for (r, &solved) in self.revolute.iter_mut().zip(&self.saved_revolute) {
            r.solved = solved;
        }
    }
    /// Step end: the motor torque averaged over the step's `dt` seconds.
    pub(super) fn finish_revolute(&mut self, dt: Real) {
        for r in &mut self.revolute {
            r.solved.torque = r.solved.motor_sum / dt;
        }
    }
    /// A step that moved nothing (every body asleep): no motor acted.
    pub(super) fn idle_revolute(&mut self) {
        for r in &mut self.revolute {
            r.solved.torque = 0.0;
            r.solved.saturated = false;
        }
    }
}
