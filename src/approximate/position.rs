//! Optional bounded nonlinear *position* correction against immovable geometry.
//!
//! This is not a sweep and cannot replace CCD. It removes residual overlap after pose
//! integration, without adding correction velocity/kinetic energy or integrating time again.
//! Movable pairs stay in the impulse solver; fixed bodies are never modified.
use super::{Error, World};

#[derive(Clone, Debug, Default)]
pub struct PositionReport {
    pub passes: u64,
    pub bounds_tests: u64,
    pub contact_tests: u64,
    pub corrections: u64,
    pub max_distance: f64,
}

impl World {
    pub(super) fn correct_fixed_positions(
        &mut self,
        h: f64,
        report: &mut PositionReport,
    ) -> Result<(), Error> {
        for _ in 0..self.config.fixed_position_iterations {
            report.passes += 1;
            let mut changed = false;
            for i in 0..self.bodies.len() {
                if !self.bodies[i].movable() || self.bodies[i].sleeping || self.bodies[i].sensor {
                    continue;
                }
                for j in 0..self.bodies.len() {
                    if self.bodies[j].mass != 0.0
                        || self.bodies[j].sensor
                        || !self.bodies[i].layers.collides_with(self.bodies[j].layers)
                    {
                        continue;
                    }
                    report.bounds_tests += 1;
                    let (lo, hi) = self.bodies[i].cached_bounds;
                    let (bl, bh) = self.bodies[j].cached_bounds;
                    if lo.0 > bh.0
                        || hi.0 < bl.0
                        || lo.1 > bh.1
                        || hi.1 < bl.1
                        || lo.2 > bh.2
                        || hi.2 < bl.2
                    {
                        continue;
                    }
                    report.contact_tests += 1;
                    // Fresh post-integration manifold: no old normals or swept TOI may substitute.
                    let Some(m) = super::contact::current(&self.bodies[j], &self.bodies[i], 0.0)
                    else {
                        continue;
                    };
                    let depth = (&m.points)
                        .into_iter()
                        .map(|p| -p.separation)
                        .fold(0.0, f64::max);
                    let distance = (depth - self.config.contact_slop).max(0.0);
                    if distance <= 0.0 {
                        continue;
                    }
                    let body = &mut self.bodies[i];
                    body.position += m.normal * distance;
                    if !body.valid() {
                        return Err(Error::NonFiniteState(body.id));
                    }
                    body.cached_bounds = super::contact::bounds(body);
                    // A body still receiving material correction is not ready to sleep.
                    if distance > self.config.sleep_speed * h {
                        body.quiet_time = 0.0;
                    }
                    report.corrections += 1;
                    report.max_distance = report.max_distance.max(distance);
                    changed = true;
                }
            }
            if !changed {
                break;
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        BodyId, CollisionLayers3d,
        approximate::{Body, Config, Quaternion, Shape, Vector},
    };

    fn fixture() -> World {
        let mut w = World::new(Config {
            fixed_position_iterations: 2,
            ..Config::default()
        })
        .unwrap();
        w.add_body(Body::new(
            BodyId(1),
            Shape::Box(Vector(50.0, 1.0, 50.0)),
            Vector(0.0, -1.0, 0.0),
            0.0,
        ))
        .unwrap();
        w
    }
    #[test]
    fn residual_overlap_correction_does_not_add_velocity_or_advance_time() {
        let mut w = fixture();
        let mut b = Body::new(
            BodyId(2),
            Shape::Box(Vector(2.0, 1.0, 1.0)),
            Vector(0.0, 0.6, 0.0),
            3.0,
        );
        b.velocity = Vector(1.0, -2.0, 3.0);
        b.angular_velocity = Vector(0.1, 0.2, 0.3);
        w.add_body(b.clone()).unwrap();
        let fixed = w.bodies[0].clone();
        let mut report = PositionReport::default();
        w.correct_fixed_positions(1.0 / 240.0, &mut report).unwrap();
        assert!((w.bodies[1].position.1 - 0.98).abs() < 1e-12);
        assert_eq!(w.bodies[1].velocity, b.velocity);
        assert_eq!(w.bodies[1].angular_velocity, b.angular_velocity);
        assert_eq!(w.bodies[1].orientation, b.orientation);
        assert_eq!(w.bodies[0], fixed);
        assert_eq!(w.elapsed_seconds(), 0.0);
        assert!(report.corrections > 0 && report.passes <= 2);
        assert_eq!(
            w.bodies[1].cached_bounds,
            super::super::contact::bounds(&w.bodies[1])
        );
    }
    #[test]
    fn rotated_fixed_colliders_use_their_actual_contact_normal() {
        let mut w = World::new(Config {
            fixed_position_iterations: 2,
            ..Config::default()
        })
        .unwrap();
        let q = Quaternion(0.0, 0.0, (0.25_f64).sin(), (0.25_f64).cos());
        let mut fixed = Body::new(
            BodyId(1),
            Shape::Box(Vector(30.0, 1.0, 30.0)),
            Vector::ZERO,
            0.0,
        );
        fixed.orientation = q;
        let n = q.rotate(Vector::Y);
        w.add_body(fixed).unwrap();
        let mut b = Body::new(BodyId(2), Shape::Box(Vector(1.0, 1.0, 1.0)), n * 1.5, 1.0);
        b.orientation = q;
        w.add_body(b).unwrap();
        let old = w.bodies[1].position;
        w.correct_fixed_positions(1.0 / 240.0, &mut PositionReport::default())
            .unwrap();
        let d = w.bodies[1].position - old;
        assert!(d.dot(n) > 0.45);
        assert!(d.cross(n).length() < 1e-12);
        let m = super::super::contact::current(&w.bodies[0], &w.bodies[1], 0.0).unwrap();
        assert!(
            (&m.points)
                .into_iter()
                .all(|p| p.separation >= -0.020000001)
        );
    }
    #[test]
    fn no_correction_for_sensors_disabled_layers_sleepers_or_external_authority() {
        for mode in 0..4 {
            let mut w = fixture();
            let mut b = Body::new(
                BodyId(2),
                Shape::Box(Vector(1.0, 1.0, 1.0)),
                Vector(0.0, 0.5, 0.0),
                1.0,
            );
            match mode {
                0 => b.sensor = true,
                1 => b.layers = CollisionLayers3d::new(0, 0),
                2 => {
                    b.sleeping = true;
                    b.quiet_time = 1.0;
                }
                _ => b.external = true,
            }
            w.add_body(b).unwrap();
            let before = w.bodies.clone();
            let mut report = PositionReport::default();
            w.correct_fixed_positions(1.0 / 240.0, &mut report).unwrap();
            assert_eq!(w.bodies, before);
            assert_eq!(report.corrections, 0);
        }
    }
    #[test]
    fn separation_and_default_mode_do_not_apply_positional_work() {
        let mut w = fixture();
        w.add_body(Body::new(
            BodyId(2),
            Shape::Sphere(1.0),
            Vector(0.0, 4.0, 0.0),
            1.0,
        ))
        .unwrap();
        let before = w.bodies.clone();
        let mut report = PositionReport::default();
        w.correct_fixed_positions(1.0 / 240.0, &mut report).unwrap();
        assert_eq!(w.bodies, before);
        assert_eq!(report.contact_tests, 0);
        w.config.fixed_position_iterations = 0;
        report = PositionReport::default();
        w.correct_fixed_positions(1.0 / 240.0, &mut report).unwrap();
        assert_eq!(report.passes, 0);
        assert_eq!(report.bounds_tests, 0);
        assert!(
            World::new(Config {
                fixed_position_iterations: 9,
                ..Config::default()
            })
            .is_err()
        );
    }
    #[test]
    fn material_position_correction_resets_quiet_time_before_island_sleep() {
        let mut w = fixture();
        let mut b = Body::new(BodyId(2), Shape::Sphere(1.0), Vector(0.0, 0.5, 0.0), 1.0);
        b.quiet_time = 1.0;
        w.add_body(b).unwrap();
        w.correct_fixed_positions(1.0 / 240.0, &mut PositionReport::default())
            .unwrap();
        assert_eq!(w.bodies[1].quiet_time, 0.0);
        assert!(!w.bodies[1].sleeping);
    }
}
