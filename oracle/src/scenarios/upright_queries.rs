//! The consumers' query dispatcher, copied unchanged from moritzbrantner/mmorpg
//! `crates/mmorpg-core/src/physics/queries.rs` (main at 0782d4b); ARPG's
//! `crates/arpg-core/src/physics/queries.rs` (main at d4ff0ab) is byte-identical. The
//! `mmorpg-tick` and `arpg-tick` scenarios' Rapier sides use it to run the consumers' exact call
//! pattern. Its original notes follow.
//!
//! Exact contact and shape-cast queries for the shape pairs the character controller meets:
//! an upright capsule against another upright capsule or against an axis-aligned box whose
//! height overlaps the capsule's axis, and an upright capsule above the floor's upward
//! half-space (#59).
//!
//! parry 0.31's general GJK/EPA path is not precise enough for them in `f32`:
//!
//! - Two overlapping parallel capsules get zero depth and a normal perpendicular to the true
//!   one (EPA on a degenerate Minkowski difference), so the controller slid one character
//!   into another.
//! - A capsule sliding along a long box (the dungeon's 60 m outer walls) got normals off by
//!   up to a few degrees, so it jumped up to 2 units off the wall and lost or gained a few
//!   percent of its speed from one tick to the next.
//!
//! Both pairs reduce to circles and rectangles on the plane, solved here in closed form, as
//! long as the two vertical spans overlap for the whole motion: vertical motion (a jump or a
//! fall, #59) then changes nothing on the plane. A capsule against the floor is a vertical
//! distance. parry's own half-space cast is exact too, but a capsule resting one contact
//! offset above the floor and moving horizontally sits on the cast's tolerance: `f32`
//! rounding decided whether it reported a hit at time zero (and a normal nudge upwards). Here
//! motion parallel to the floor never touches it, and a gap within [`TOUCHING`] of the
//! target distance counts as reached, so a body at rest stays exactly where it is. Every
//! other pair, pose or motion falls through to Rapier's default dispatcher.

use rapier3d::parry::query::details::{cast_shapes_ball_ball, contact_ball_ball};
use rapier3d::parry::query::{
    ClosestPoints, Contact, NonlinearRigidMotion, QueryDispatcher, ShapeCastHit, ShapeCastOptions,
    ShapeCastStatus, ShapeDistance, ShapeIntersection, Unsupported,
};
use rapier3d::prelude::*;

/// Below this time of impact (metres along a unit cast) a cast that separates or slides
/// reports nothing, as parry's own ball cast does.
const SMALL_TIME_OF_IMPACT: f32 = 1.0e-4;
/// A cast whose planar part is at most this fraction of the whole motion moves vertically.
const PLANAR_RESIDUE: f32 = 1.0e-5;
/// A floor gap (metres) within this of the cast's target distance has reached it.
const TOUCHING: f32 = 1.0e-6;

pub(super) struct UprightShapes;

/// The vertical span of a capsule's axis, if the capsule is upright at its frame's origin.
fn upright_axis(capsule: &Capsule) -> Option<(f32, f32)> {
    let (a, b) = (capsule.segment.a, capsule.segment.b);
    (a.x == 0.0 && a.z == 0.0 && b.x == 0.0 && b.z == 0.0).then(|| (a.y.min(b.y), a.y.max(b.y)))
}

/// An upright capsule seen from an axis-aligned box: the capsule centre in the box's frame
/// and its radius, when only the box's vertical faces and edges can be nearest.
struct CapsuleBesideBox {
    centre: Vector,
    radius: f32,
    half: Vector,
}

impl CapsuleBesideBox {
    /// The pair when only the box's vertical faces and edges can be nearest to the capsule
    /// while its centre moves from the frame's translation by `motion`: the capsule's axis
    /// overlaps the box's height all along, and lies within it wherever the centre is
    /// inside the footprint. The spans are checked at both ends; overlap is convex in time.
    fn new(
        capsule: &Capsule,
        cuboid: &Cuboid,
        capsule_in_box: &Pose,
        motion: Vector,
    ) -> Option<Self> {
        if capsule_in_box.rotation != Rotation::IDENTITY || !motion.is_finite() {
            return None;
        }
        let (low, high) = upright_axis(capsule)?;
        let centre = capsule_in_box.translation;
        let half = cuboid.half_extents;
        let within = |y: f32| y + low >= -half.y && y + high <= half.y;
        let overlaps = |y: f32| y + high >= -half.y && y + low <= half.y;
        let outside = centre.x.abs() > half.x || centre.z.abs() > half.z;
        let end = centre.y + motion.y;
        let valid =
            (within(centre.y) && within(end)) || (outside && overlaps(centre.y) && overlaps(end));
        valid.then_some(Self {
            centre,
            radius: capsule.radius,
            half,
        })
    }

    /// Witness points (on the box in its frame, on the capsule in its own) where a capsule
    /// centred at `at` meets the box along `normal`: at the centre's height, kept on the
    /// box's side where the axis only partly overlaps it.
    fn witnesses(&self, at: Vector, on_box: Vector, normal: Vector) -> (Vector, Vector) {
        let height = at.y.clamp(-self.half.y, self.half.y);
        (
            Vector::new(on_box.x, height, on_box.z),
            -normal * self.radius + Vector::new(0.0, height - at.y, 0.0),
        )
    }

    /// The nearest point of the box's footprint to `at` (at `at`'s height), and the unit
    /// normal from the box towards `at` with the signed distance between them.
    fn nearest(&self, at: Vector) -> (Vector, Vector, f32) {
        let clamped = Vector::new(
            at.x.clamp(-self.half.x, self.half.x),
            at.y,
            at.z.clamp(-self.half.z, self.half.z),
        );
        let outside = at - clamped;
        let distance = outside.length();
        if distance > 0.0 {
            return (clamped, outside / distance, distance);
        }
        // Inside the footprint: out through the nearest side.
        let (depth_x, depth_z) = (self.half.x - at.x.abs(), self.half.z - at.z.abs());
        if depth_x <= depth_z {
            let side = if at.x < 0.0 { -1.0 } else { 1.0 };
            (
                Vector::new(side * self.half.x, at.y, at.z),
                Vector::new(side, 0.0, 0.0),
                -depth_x,
            )
        } else {
            let side = if at.z < 0.0 { -1.0 } else { 1.0 };
            (
                Vector::new(at.x, at.y, side * self.half.z),
                Vector::new(0.0, 0.0, side),
                -depth_z,
            )
        }
    }

    /// The earliest time in `0..=max` at which the capsule centre moving by `velocity`
    /// comes within `reach` of the footprint, starting outside that reach.
    fn time_of_impact(&self, velocity: Vector, reach: f32, max: f32) -> Option<f32> {
        let (px, pz, vx, vz) = (self.centre.x, self.centre.z, velocity.x, velocity.z);
        // Entry into an axis-aligned rectangle centred on the origin.
        let slab = |half_x: f32, half_z: f32| -> Option<f32> {
            let axis = |p: f32, v: f32, half: f32| -> Option<(f32, f32)> {
                if v == 0.0 {
                    return (p.abs() <= half).then_some((f32::NEG_INFINITY, f32::INFINITY));
                }
                let (a, b) = ((-half - p) / v, (half - p) / v);
                Some((a.min(b), a.max(b)))
            };
            let (enter_x, exit_x) = axis(px, vx, half_x)?;
            let (enter_z, exit_z) = axis(pz, vz, half_z)?;
            let (enter, exit) = (enter_x.max(enter_z), exit_x.min(exit_z));
            (enter <= exit && enter >= 0.0).then_some(enter)
        };
        // Entry into a circle of `reach` around a corner.
        let corner = |cx: f32, cz: f32| -> Option<f32> {
            let (dx, dz) = (px - cx, pz - cz);
            let a = vx * vx + vz * vz;
            let b = dx * vx + dz * vz;
            let c = dx * dx + dz * dz - reach * reach;
            let discriminant = b * b - a * c;
            if a == 0.0 || discriminant < 0.0 {
                return None;
            }
            let t = (-b - discriminant.sqrt()) / a;
            (t >= 0.0).then_some(t)
        };
        let (hx, hz) = (self.half.x, self.half.z);
        [
            slab(hx + reach, hz),
            slab(hx, hz + reach),
            corner(hx, hz),
            corner(hx, -hz),
            corner(-hx, hz),
            corner(-hx, -hz),
        ]
        .into_iter()
        .flatten()
        .filter(|&t| t <= max)
        .min_by(f32::total_cmp)
    }
}

/// Two upright capsules whose axes overlap in height, as two balls at a shared height: their
/// radii, the second ball's pose relative to the first and each ball centre in its own
/// capsule's frame. With `motion` (the second capsule's, relative to the first) the axes
/// must overlap at both ends of it, and so all along.
fn capsules_as_balls(
    pos12: &Pose,
    c1: &Capsule,
    c2: &Capsule,
    motion: Vector,
) -> Option<(Ball, Ball, Pose, Vector, Vector)> {
    if pos12.rotation != Rotation::IDENTITY || !motion.is_finite() {
        return None;
    }
    let ((low1, high1), (low2, high2)) = (upright_axis(c1)?, upright_axis(c2)?);
    let offset = pos12.translation;
    let overlap = |y: f32| (low1.max(low2 + y), high1.min(high2 + y));
    let (low, high) = overlap(offset.y);
    let (end_low, end_high) = overlap(offset.y + motion.y);
    if low > high || end_low > end_high {
        return None;
    }
    let height = (low + high) / 2.0;
    Some((
        Ball::new(c1.radius),
        Ball::new(c2.radius),
        Pose::from_translation(Vector::new(offset.x, 0.0, offset.z)),
        Vector::new(0.0, height, 0.0),
        Vector::new(0.0, height - offset.y, 0.0),
    ))
}

/// The capsule and the box of a capsule-box pair with the capsule's pose in the box's
/// frame, and whether the box is the first shape. `motion` is the second shape's, relative to
/// the first.
fn capsule_and_box<'a>(
    pos12: &Pose,
    g1: &'a dyn Shape,
    g2: &'a dyn Shape,
    motion: Vector,
) -> Option<(CapsuleBesideBox, bool)> {
    if let (Some(cuboid), Some(capsule)) = (g1.as_cuboid(), g2.as_capsule()) {
        return Some((CapsuleBesideBox::new(capsule, cuboid, pos12, motion)?, true));
    }
    let (capsule, cuboid) = (g1.as_capsule()?, g2.as_cuboid()?);
    Some((
        CapsuleBesideBox::new(capsule, cuboid, &pos12.inverse(), -motion)?,
        false,
    ))
}

/// An upright capsule and an upward half-space (the floor), seen from the floor: the lowest
/// point of the capsule in the floor's frame and in its own, and whether the floor is the
/// first shape.
struct CapsuleOverFloor {
    lowest: Vector,
    on_capsule: Vector,
    floor_first: bool,
}

impl CapsuleOverFloor {
    fn new(pos12: &Pose, g1: &dyn Shape, g2: &dyn Shape) -> Option<Self> {
        if pos12.rotation != Rotation::IDENTITY {
            return None;
        }
        let upward = |shape: &dyn Shape| {
            shape
                .as_shape::<HalfSpace>()
                .is_some_and(|floor| floor.normal == Vector::Y)
        };
        let (capsule, capsule_in_floor, floor_first) = if upward(g1) {
            (g2.as_capsule()?, *pos12, true)
        } else if upward(g2) {
            (g1.as_capsule()?, pos12.inverse(), false)
        } else {
            return None;
        };
        let (low, _) = upright_axis(capsule)?;
        let on_capsule = Vector::new(0.0, low - capsule.radius, 0.0);
        Some(Self {
            lowest: capsule_in_floor.translation + on_capsule,
            on_capsule,
            floor_first,
        })
    }

    /// Witness points and normals (first shape, second shape) with the capsule's lowest
    /// point at `lowest` in the floor's frame.
    fn geometry(&self, lowest: Vector) -> (Vector, Vector, Vector, Vector) {
        let on_floor = Vector::new(lowest.x, 0.0, lowest.z);
        if self.floor_first {
            (on_floor, self.on_capsule, Vector::Y, -Vector::Y)
        } else {
            (self.on_capsule, on_floor, -Vector::Y, Vector::Y)
        }
    }
}

impl QueryDispatcher for UprightShapes {
    fn intersection_test(
        &self,
        _: &Pose,
        _: &dyn Shape,
        _: &dyn Shape,
    ) -> Result<ShapeIntersection, Unsupported> {
        Err(Unsupported)
    }

    fn distance(
        &self,
        _: &Pose,
        _: &dyn Shape,
        _: &dyn Shape,
    ) -> Result<ShapeDistance, Unsupported> {
        Err(Unsupported)
    }

    fn contact(
        &self,
        pos12: &Pose,
        g1: &dyn Shape,
        g2: &dyn Shape,
        prediction: f32,
    ) -> Result<Option<Contact>, Unsupported> {
        if let (Some(c1), Some(c2)) = (g1.as_capsule(), g2.as_capsule()) {
            let (b1, b2, balls12, centre1, centre2) =
                capsules_as_balls(pos12, c1, c2, Vector::ZERO).ok_or(Unsupported)?;
            return Ok(
                contact_ball_ball(&balls12, &b1, &b2, prediction).map(|contact| Contact {
                    point1: contact.point1 + centre1,
                    point2: contact.point2 + centre2,
                    ..contact
                }),
            );
        }
        if let Some(pair) = CapsuleOverFloor::new(pos12, g1, g2) {
            let dist = pair.lowest.y;
            if dist > prediction {
                return Ok(None);
            }
            let (point1, point2, normal1, normal2) = pair.geometry(pair.lowest);
            return Ok(Some(Contact::new(point1, point2, normal1, normal2, dist)));
        }
        let (pair, box_first) = capsule_and_box(pos12, g1, g2, Vector::ZERO).ok_or(Unsupported)?;
        let (on_box, normal, distance) = pair.nearest(pair.centre);
        let dist = distance - pair.radius;
        if dist > prediction {
            return Ok(None);
        }
        // `normal` points from the box towards the capsule.
        let (on_box, on_capsule) = pair.witnesses(pair.centre, on_box, normal);
        Ok(Some(if box_first {
            Contact::new(on_box, on_capsule, normal, -normal, dist)
        } else {
            Contact::new(on_capsule, on_box, -normal, normal, dist)
        }))
    }

    fn closest_points(
        &self,
        _: &Pose,
        _: &dyn Shape,
        _: &dyn Shape,
        _: f32,
    ) -> Result<ClosestPoints, Unsupported> {
        Err(Unsupported)
    }

    fn cast_shapes(
        &self,
        pos12: &Pose,
        local_vel12: Vector,
        g1: &dyn Shape,
        g2: &dyn Shape,
        options: ShapeCastOptions,
    ) -> Result<Option<ShapeCastHit>, Unsupported> {
        // The second shape's whole motion relative to the first; vertical motion keeps the
        // planar closed forms only while the vertical spans overlap at both of its ends.
        let motion = if local_vel12.y == 0.0 {
            Vector::ZERO
        } else {
            local_vel12 * options.max_time_of_impact
        };
        // A vertical motion whose planar part is only `f32` residue (the controller removes
        // the part into a touching body, and Rapier's kinematic friction then removes its
        // normal nudge as well) is purely vertical: the residue's sign must not decide
        // whether a body falling beside another one is stopped.
        let horizontal = Vector::new(local_vel12.x, 0.0, local_vel12.z);
        let planar = if horizontal.length() <= PLANAR_RESIDUE * local_vel12.length() {
            Vector::ZERO
        } else {
            horizontal
        };
        if let (Some(c1), Some(c2)) = (g1.as_capsule(), g2.as_capsule()) {
            let (b1, b2, balls12, centre1, centre2) =
                capsules_as_balls(pos12, c1, c2, motion).ok_or(Unsupported)?;
            return Ok(
                cast_shapes_ball_ball(&balls12, planar, &b1, &b2, options).map(|hit| {
                    ShapeCastHit {
                        witness1: hit.witness1 + centre1,
                        witness2: hit.witness2 + centre2,
                        ..hit
                    }
                }),
            );
        }
        if let Some(pair) = CapsuleOverFloor::new(pos12, g1, g2) {
            return Ok(cast_capsule_over_floor(&pair, local_vel12, options));
        }
        let (pair, box_first) = capsule_and_box(pos12, g1, g2, motion).ok_or(Unsupported)?;
        // The capsule's motion relative to the box; only its planar part can meet a side.
        let full_velocity = if box_first { local_vel12 } else { -local_vel12 };
        let velocity = if box_first { planar } else { -planar };
        let reach = pair.radius + options.target_distance;
        let (_, normal, distance) = pair.nearest(pair.centre);
        let (time_of_impact, status) = if distance <= reach {
            let status = if distance < reach {
                ShapeCastStatus::PenetratingOrWithinTargetDist
            } else {
                ShapeCastStatus::Converged
            };
            (0.0, status)
        } else {
            match pair.time_of_impact(velocity, reach, options.max_time_of_impact) {
                Some(time) => (time, ShapeCastStatus::Converged),
                None => return Ok(None),
            }
        };
        let at = pair.centre + full_velocity * time_of_impact;
        let (on_box, normal_at, _) = pair.nearest(at);
        let normal = if time_of_impact == 0.0 {
            normal
        } else {
            normal_at
        };
        // Leaving or sliding along what it touches is no impact.
        if !options.stop_at_penetration
            && time_of_impact < SMALL_TIME_OF_IMPACT
            && normal.dot(velocity) >= 0.0
        {
            return Ok(None);
        }
        let (on_box, on_capsule) = pair.witnesses(at, on_box, normal);
        Ok(Some(if box_first {
            ShapeCastHit {
                time_of_impact,
                witness1: on_box,
                witness2: on_capsule,
                normal1: normal,
                normal2: -normal,
                status,
                subshape1: 0,
                subshape2: 0,
            }
        } else {
            ShapeCastHit {
                time_of_impact,
                witness1: on_capsule,
                witness2: on_box,
                normal1: -normal,
                normal2: normal,
                status,
                subshape1: 0,
                subshape2: 0,
            }
        }))
    }

    fn cast_shapes_nonlinear(
        &self,
        _: &NonlinearRigidMotion,
        _: &dyn Shape,
        _: &NonlinearRigidMotion,
        _: &dyn Shape,
        _: f32,
        _: f32,
        _: bool,
    ) -> Result<Option<ShapeCastHit>, Unsupported> {
        Err(Unsupported)
    }
}

/// A capsule cast against the floor: only motion into the floor meets it, and a gap within
/// [`TOUCHING`] of the target distance is reached at once.
fn cast_capsule_over_floor(
    pair: &CapsuleOverFloor,
    local_vel12: Vector,
    options: ShapeCastOptions,
) -> Option<ShapeCastHit> {
    // The capsule's motion relative to the floor.
    let velocity = if pair.floor_first {
        local_vel12
    } else {
        -local_vel12
    };
    let gap = pair.lowest.y - options.target_distance;
    let (time_of_impact, status) = if gap <= TOUCHING {
        if !options.stop_at_penetration && velocity.y >= 0.0 {
            return None;
        }
        let status = if gap < -TOUCHING {
            ShapeCastStatus::PenetratingOrWithinTargetDist
        } else {
            ShapeCastStatus::Converged
        };
        (0.0, status)
    } else {
        if velocity.y >= 0.0 {
            return None;
        }
        let time = gap / -velocity.y;
        if time > options.max_time_of_impact {
            return None;
        }
        (time, ShapeCastStatus::Converged)
    };
    let (witness1, witness2, normal1, normal2) =
        pair.geometry(pair.lowest + velocity * time_of_impact);
    Some(ShapeCastHit {
        time_of_impact,
        witness1,
        witness2,
        normal1,
        normal2,
        status,
        subshape1: 0,
        subshape2: 0,
    })
}
