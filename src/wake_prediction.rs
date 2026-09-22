//! Read-only, conservative floating-point admission before changing sleeping-body state.
//!
//! A miss is a proof of separation over the whole requested free-flight interval, not an endpoint
//! overlap test. Curvature and the legacy integrator's integer rounding expand the swept volume.
//! Possible hits still go through the authoritative CCD/response solver. Rotating trajectories that
//! this predictor cannot bound tightly keep the existing conservative rotational sweep.

use crate::{
    BallisticSphere3d, BodyKind, MotionAuthority3d, RigidBox3d, RotatingWorldError3d,
    RotationalSweepBounds3d, Vec3i, numeric::Scalar, oriented_box_vertices,
};

type Vector = [Scalar; 3];

pub(crate) struct TranslationSweep {
    vertices: [Vector; 8],
    displacement: Vector,
    padding: Vector,
}

impl TranslationSweep {
    pub(crate) fn for_box(
        body: &RigidBox3d,
        gravity: Vec3i,
        seconds: Scalar,
    ) -> Result<Option<Self>, RotatingWorldError3d> {
        if !body.rotation_locked() && !body.angular().angular_velocity.is_zero() {
            return Ok(None);
        }
        let gravity = if body.motion_authority() == MotionAuthority3d::External
            || body.body().kind() == BodyKind::Fixed
        {
            Vec3i::ZERO
        } else {
            gravity
        };
        Ok(Some(Self::new(
            oriented_box_vertices(body.oriented_box())?.map(vector),
            body.body().velocity(),
            gravity,
            seconds,
        )))
    }

    pub(crate) fn for_sphere(sphere: BallisticSphere3d, gravity: Vec3i, seconds: Scalar) -> Self {
        let center = vector(sphere.position());
        let radius = Scalar::from(sphere.radius());
        // Circumscribed cube: admits some corner grazes but cannot reject a genuine sphere hit.
        let vertices = std::array::from_fn(|corner| {
            std::array::from_fn(|axis| {
                center[axis]
                    + if corner & (1 << axis) == 0 {
                        -radius
                    } else {
                        radius
                    }
            })
        });
        let mut sweep = Self::new(vertices, sphere.velocity(), gravity, seconds);
        // The analytic sphere lane uses unrounded target faces, while the shared geometry API
        // exposes rounded vertices. Expand admission only; never turn this margin into a contact.
        sweep.padding = sweep.padding.map(|value| value + 1.0);
        sweep
    }

    fn new(vertices: [Vector; 8], velocity: Vec3i, gravity: Vec3i, seconds: Scalar) -> Self {
        let velocity = vector(velocity);
        let gravity = vector(gravity);
        let displacement =
            std::array::from_fn(|axis| (velocity[axis] + gravity[axis] * seconds) * seconds);
        let padding = std::array::from_fn(|axis| {
            // Semi-implicit free flight is v*t + a*t*t. Relative to the start/end chord its
            // maximum curvature is |a|*dt²/4, including an interior acceleration reversal.
            // Each legacy sample also rounds velocity (<= .5) then displacement (<= .5).
            let quantization = if velocity[axis] == 0.0 && gravity[axis] == 0.0 {
                0.0
            } else {
                0.5 * seconds + 0.5
            };
            gravity[axis].abs() * seconds * seconds * 0.25 + quantization
        });
        Self {
            vertices,
            displacement,
            padding,
        }
    }

    pub(crate) fn bounds(&self) -> RotationalSweepBounds3d {
        let mut minimum = [0; 3];
        let mut maximum = [0; 3];
        for axis in 0..3 {
            let low = self
                .vertices
                .iter()
                .map(|v| v[axis])
                .fold(Scalar::INFINITY, Scalar::min)
                + self.displacement[axis].min(0.0)
                - self.padding[axis];
            let high = self
                .vertices
                .iter()
                .map(|v| v[axis])
                .fold(Scalar::NEG_INFINITY, Scalar::max)
                + self.displacement[axis].max(0.0)
                + self.padding[axis];
            // Clamp only this conservative integer index envelope, never physical state. An
            // extreme range becomes an unbounded candidate envelope, not a false miss.
            minimum[axis] = low
                .next_down()
                .floor()
                .clamp(i64::MIN as Scalar, i64::MAX as Scalar) as i64;
            maximum[axis] = high
                .next_up()
                .ceil()
                .clamp(i64::MIN as Scalar, i64::MAX as Scalar) as i64;
        }
        RotationalSweepBounds3d { minimum, maximum }
    }

    pub(crate) fn may_reach(&self, target: &RigidBox3d) -> Result<bool, RotatingWorldError3d> {
        let other = oriented_box_vertices(target.oriented_box())?.map(vector);
        let left_edges = edges(&self.vertices);
        let right_edges = edges(&other);
        let mut enter: Scalar = 0.0;
        let mut exit: Scalar = 1.0;
        let axes = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
            .into_iter()
            .chain(face_axes(left_edges))
            .chain(face_axes(right_edges))
            .chain(
                left_edges
                    .into_iter()
                    .flat_map(|left| right_edges.map(|right| cross(left, right))),
            );
        for axis in axes {
            let length = dot(axis, axis).sqrt();
            if length <= Scalar::EPSILON || !length.is_finite() {
                continue; // Degenerate axes provide no separation proof.
            }
            let axis = axis.map(|component| component / length);
            let (left_min, left_max) = project(&self.vertices, axis);
            let (right_min, right_max) = project(&other, axis);
            let rounding = 64.0
                * Scalar::EPSILON
                * (left_min.abs()
                    + left_max.abs()
                    + right_min.abs()
                    + right_max.abs()
                    + dot(self.displacement, axis).abs()
                    + 1.0);
            let padding = dot(self.padding, axis.map(Scalar::abs)) + rounding;
            let lower = right_min - left_max - padding;
            let upper = right_max - left_min + padding;
            let motion = dot(self.displacement, axis);
            if motion == 0.0 {
                if lower > 0.0 || upper < 0.0 {
                    return Ok(false);
                }
            } else {
                let a = lower / motion;
                let b = upper / motion;
                enter = enter.max(a.min(b).next_down());
                exit = exit.min(a.max(b).next_up());
                if enter > exit {
                    return Ok(false);
                }
            }
        }
        Ok(true)
    }
}

fn vector(value: Vec3i) -> Vector {
    [
        Scalar::from(value.x),
        Scalar::from(value.y),
        Scalar::from(value.z),
    ]
}
fn subtract(a: Vector, b: Vector) -> Vector {
    std::array::from_fn(|i| a[i] - b[i])
}
fn dot(a: Vector, b: Vector) -> Scalar {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
fn cross(a: Vector, b: Vector) -> Vector {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}
fn edges(vertices: &[Vector; 8]) -> [Vector; 3] {
    [
        subtract(vertices[1], vertices[0]),
        subtract(vertices[2], vertices[0]),
        subtract(vertices[4], vertices[0]),
    ]
}
fn face_axes(edges: [Vector; 3]) -> [Vector; 3] {
    [
        cross(edges[0], edges[1]),
        cross(edges[1], edges[2]),
        cross(edges[2], edges[0]),
    ]
}
fn project(vertices: &[Vector; 8], axis: Vector) -> (Scalar, Scalar) {
    vertices.iter().map(|v| dot(*v, axis)).fold(
        (Scalar::INFINITY, Scalar::NEG_INFINITY),
        |(low, high), value| (low.min(value), high.max(value)),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        AngularState3d, AngularVelocity3d, BodyId, Orientation3d, RigidBody,
        RigidBoxFreeFlightConfig3d, obb_contact_seed, sample_rigid_box_free_flight,
    };

    fn body(id: u64, position: Vec3i, velocity: Vec3i, orientation: Orientation3d) -> RigidBox3d {
        RigidBox3d::new(
            RigidBody::dynamic(BodyId(id), position, velocity, Vec3i::new(2, 3, 12)),
            AngularState3d::new(orientation, AngularVelocity3d::default()),
        )
        .unwrap()
        .with_rotation_locked()
    }

    #[test]
    fn a_diagonal_swept_envelope_is_not_a_collision() {
        let source = body(
            1,
            Vec3i::new(-100, 0, -100),
            Vec3i::new(200, 0, 200),
            Orientation3d::IDENTITY,
        );
        let target = body(
            2,
            Vec3i::new(-75, 0, 75),
            Vec3i::ZERO,
            Orientation3d::IDENTITY,
        );
        let sweep = TranslationSweep::for_box(&source, Vec3i::ZERO, 1.0)
            .unwrap()
            .unwrap();
        assert!(
            sweep.bounds().contains([-75, 0, 75]),
            "fixture must be a broad-phase false positive"
        );
        assert!(!sweep.may_reach(&target).unwrap());
    }

    #[test]
    fn admits_between_endpoint_impacts_and_acceleration_reversals() {
        let target = body(2, Vec3i::ZERO, Vec3i::ZERO, Orientation3d::IDENTITY);
        for (start, velocity, gravity) in [(-100, 200, 0), (-10, 80, -160)] {
            let source = body(
                1,
                Vec3i::new(start, 0, 0),
                Vec3i::new(velocity, 0, 0),
                Orientation3d::IDENTITY,
            );
            let sweep = TranslationSweep::for_box(&source, Vec3i::new(gravity, 0, 0), 1.0)
                .unwrap()
                .unwrap();
            assert!(sweep.may_reach(&target).unwrap());
        }
    }

    #[test]
    fn swept_admission_never_rejects_a_sampled_obb_hit() {
        let orientations = [
            Orientation3d::IDENTITY,
            Orientation3d::new(0, 0, 410_903_207, 992_008_094)
                .normalized()
                .unwrap(),
        ];
        let mut seed = 7_u64;
        let mut next = || {
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
            ((seed >> 32) % 161) as i32 - 80
        };
        let mut admitted_hits = 0;
        let mut proven_misses = 0;
        for case in 0..192 {
            let source = body(
                1,
                Vec3i::new(next(), next() / 4, next()),
                Vec3i::new(next(), next(), next()),
                orientations[case % 2],
            );
            let target = body(
                2,
                Vec3i::new(next() / 2, next() / 4, next() / 2),
                Vec3i::ZERO,
                orientations[(case / 2) % 2],
            );
            let gravity = Vec3i::new(next(), next(), next());
            let config = RigidBoxFreeFlightConfig3d::new(gravity, 1, 1);
            let sweep = TranslationSweep::for_box(&source, gravity, 1.0)
                .unwrap()
                .unwrap();
            let admitted = sweep.may_reach(&target).unwrap();
            proven_misses += usize::from(!admitted);
            for tick in 0..=128 {
                let sample = sample_rigid_box_free_flight(&source, config, tick, 128).unwrap();
                for vertex in oriented_box_vertices(sample.oriented_box()).unwrap() {
                    assert!(
                        sweep.bounds().contains([
                            i64::from(vertex.x),
                            i64::from(vertex.y),
                            i64::from(vertex.z)
                        ]),
                        "case {case}, sample {tick}"
                    );
                }
                if obb_contact_seed(sample.oriented_box(), target.oriented_box())
                    .unwrap()
                    .is_some()
                {
                    admitted_hits += 1;
                    assert!(admitted, "false miss: case {case}, sample {tick}");
                }
            }
        }
        assert!(
            admitted_hits > 0 && proven_misses > 0,
            "oracle must exercise hits and misses"
        );
    }

    #[test]
    fn rotating_sources_keep_the_conservative_fallback() {
        let source = RigidBox3d::new(
            RigidBody::dynamic(BodyId(1), Vec3i::ZERO, Vec3i::ZERO, Vec3i::new(2, 2, 12)),
            AngularState3d::new(
                Orientation3d::IDENTITY,
                AngularVelocity3d::new(0, 1_000_000, 0),
            ),
        )
        .unwrap();
        assert!(
            TranslationSweep::for_box(&source, Vec3i::ZERO, 1.0)
                .unwrap()
                .is_none()
        );
    }
}
