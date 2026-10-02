use super::super::{Config, Quaternion, Real, World};
use super::{Body, Shape, V, bounds};
use crate::BodyId;

fn surface_extrema(body: &Body) -> Vec<V> {
    let mut points = Vec::new();
    match body.shape {
        Shape::Sphere(radius) => {
            for direction in [V::X, V::Y, V::Z] {
                points.extend([
                    body.position + direction * radius,
                    body.position - direction * radius,
                ]);
            }
        }
        Shape::Capsule {
            half_segment,
            radius,
        } => {
            for half in [-half_segment, half_segment] {
                let center = body.position + body.orientation.rotate(V::Y * half);
                for direction in [V::X, V::Y, V::Z] {
                    points.extend([center + direction * radius, center - direction * radius]);
                }
            }
        }
        Shape::Box(half) | Shape::Wedge(half) => {
            for x in [-half.0, half.0] {
                for y in [-half.1, half.1] {
                    if matches!(body.shape, Shape::Wedge(_)) && x > 0.0 && y > 0.0 {
                        continue;
                    }
                    for z in [-half.2, half.2] {
                        points.push(body.position + body.orientation.rotate(V(x, y, z)));
                    }
                }
            }
        }
    }
    points
}
fn contains(bounds: (V, V), point: V) {
    for axis in 0..3 {
        assert!(
            bounds.0.at(axis) <= point.at(axis) && point.at(axis) <= bounds.1.at(axis),
            "axis={axis} bounds={bounds:?} point={point:?}"
        );
    }
}
fn shapes(scale: Real) -> [Shape; 4] {
    [
        Shape::Sphere(scale),
        Shape::Box(V(scale, 2.0 * scale, 3.0 * scale)),
        Shape::capsule(2.0 * scale, scale),
        Shape::wedge(V(scale, 2.0 * scale, 3.0 * scale)),
    ]
}

#[test]
fn rounded_pose_bounds_enclose_independently_rotated_surface_extrema() {
    let mut state = 183_u64;
    for _ in 0..128 {
        let mut component = || {
            state = state.wrapping_mul(6364136223846793005).wrapping_add(1);
            ((state >> 32) as Real / u32::MAX as Real) * 2.0 - 1.0
        };
        let orientation =
            Quaternion(component(), component(), component(), component()).normalized();
        for scale in [1e-6, 1.0, 1e6] {
            for shape in shapes(scale) {
                let mut body = Body::new(
                    BodyId(1),
                    shape,
                    V(-11.0 * scale, 4.0 * scale, 3.0 * scale),
                    0.0,
                );
                body.orientation = orientation;
                let extents = bounds(&body);
                for point in surface_extrema(&body) {
                    contains(extents, point);
                }
            }
        }
    }
}

#[test]
fn real_sweep_rows_cover_endpoint_rounding_for_large_displacements() {
    for velocity in [V(1e11, -1e11, 5e10), V(-1e11, 1e11, -5e10)] {
        for scale in [1e-6, 1.0, 1e6] {
            for shape in shapes(scale) {
                let mut world = World::new(Config {
                    gravity: V::ZERO,
                    substeps: 1,
                    contact_slop: 1e-20,
                    ..Config::default()
                })
                .unwrap();
                let mut moving = Body::new(
                    BodyId(1),
                    shape,
                    V(-11.0 * scale, 4.0 * scale, 3.0 * scale),
                    1.0,
                );
                moving.orientation = Quaternion(0.2, 0.3, 0.1, 0.9).normalized();
                moving.rotation_locked = true;
                moving.velocity = velocity;
                let initial = moving.clone();
                world.add_body(moving).unwrap();
                world.step(1.0 / 60.0).unwrap();
                let row = &world.bookkeeping.bounds[0];
                for body in [&initial, world.body(BodyId(1)).unwrap()] {
                    for point in surface_extrema(body) {
                        contains((row.lo, row.hi), point);
                    }
                }
            }
        }
    }
}
