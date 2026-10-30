//! Single-move probe: one character move from the same pose on Rapier's
//! `KinematicCharacterController::move_shape` and on physics-engine's `move_characters`, over
//! the same static geometry. Use it to settle a behaviour question, or to check a reported
//! engine gap against Rapier before filing it; a whole-scenario trace mixes many moves.
//!
//! Grounded is each engine's own: Rapier's `movement.grounded` (any upward-facing contact
//! within its ground prediction) and the engine's `grounded` (walkable static support within
//! the prediction, the consumers' pattern).

/// Static geometry, in metres.
#[derive(Clone, Debug, Default)]
pub struct Geometry {
    /// An infinite floor at y = 0.
    pub floor: bool,
    /// Boxes: centre and half extents.
    pub boxes: Vec<([f32; 3], [f32; 3])>,
    /// Wedges: centre and half extents of their box, rising toward -X (the engine's
    /// `Shape::Wedge`; on Rapier the convex hull MMORPG builds).
    pub wedges: Vec<([f32; 3], [f32; 3])>,
}

/// The character and its controller settings.
#[derive(Clone, Copy, Debug)]
pub struct Controller {
    pub half_segment: f32,
    pub radius: f32,
    pub offset: f32,
    /// Steepest walkable slope (radians); also Rapier's slide angle.
    pub max_slope: f32,
    /// Snap distance beyond `offset` (Rapier: `offset + snap`).
    pub snap: Option<f32>,
    /// Autostep `(max_height, min_width)`.
    pub autostep: Option<(f32, f32)>,
    /// The engine leaves an embedded start in place (`Embedded::Stay`), as Rapier always does.
    pub stay: bool,
}

impl Default for Controller {
    fn default() -> Self {
        Self {
            half_segment: 0.5,
            radius: 0.3,
            offset: 0.0025,
            max_slope: core::f32::consts::FRAC_PI_4,
            snap: Some(0.05),
            autostep: None,
            stay: false,
        }
    }
}

/// What one move did on one engine.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Outcome {
    pub translation: [f32; 3],
    /// Rapier's `grounded`; the engine's `ground_contact`.
    pub grounded: bool,
}

/// Rapier's move from `at` by `desired`.
#[cfg(feature = "rapier")]
pub fn rapier(geometry: &Geometry, c: Controller, at: [f32; 3], desired: [f32; 3]) -> Outcome {
    use crate::kcc::rapier::{Characters, cuboid, halfspace, mirror_controller, v};
    use rapier3d::control::KinematicCharacterController;
    use rapier3d::prelude::*;
    let mut statics = Vec::new();
    if geometry.floor {
        statics.push(halfspace());
    }
    statics.extend(geometry.boxes.iter().copied().map(cuboid));
    for &(centre, h) in &geometry.wedges {
        let mut points = Vec::with_capacity(6);
        for sx in [-1.0, 1.0] {
            for sz in [-1.0, 1.0] {
                points.push(Vector::new(sx * h[0], -h[1], sz * h[2]));
                if sx < 0.0 {
                    points.push(Vector::new(sx * h[0], h[1], sz * h[2]));
                }
            }
        }
        let hull = SharedShape::convex_hull(&points).expect("solid wedge");
        statics.push(ColliderBuilder::new(hull).translation(v(centre)));
    }
    let controller = KinematicCharacterController {
        max_slope_climb_angle: c.max_slope,
        min_slope_slide_angle: c.max_slope,
        ..mirror_controller(c.offset, c.snap, c.autostep)
    };
    let shape = Capsule::new_y(c.half_segment, c.radius);
    let characters = Characters::new(statics, [at], shape, controller, false);
    let movement = characters.move_shape(0, v(desired));
    let t = movement.translation;
    Outcome {
        translation: [t.x, t.y, t.z],
        grounded: movement.grounded,
    }
}

/// physics-engine's move from `at` by `desired`.
#[cfg(feature = "engine")]
pub fn engine(geometry: &Geometry, c: Controller, at: [f32; 3], desired: [f32; 3]) -> Outcome {
    use crate::kcc::engine::{Characters, capsules, fixed, v, world};
    use physics_engine::approximate::{
        Autostep, BodyStorage, CharacterMove, CharacterSettings, Embedded, Shape,
    };
    let statics = geometry
        .floor
        .then_some((Shape::HalfSpace, [0.0; 3]))
        .into_iter()
        .chain(geometry.boxes.iter().map(|&(c, h)| (Shape::Box(v(h)), c)))
        .chain(
            geometry
                .wedges
                .iter()
                .map(|&(c, h)| (Shape::wedge(v(h)), c)),
        )
        .enumerate()
        .map(|(i, (shape, at))| fixed(1 + i as u64, shape, at))
        .collect::<Vec<_>>();
    let key = 1 + statics.len() as u64;
    let settings = CharacterSettings {
        offset: c.offset,
        max_slope: c.max_slope,
        snap_to_ground: c.snap,
        embedded: if c.stay {
            Embedded::Stay
        } else {
            Embedded::PushOut
        },
        ..CharacterSettings::default()
    };
    let shape = Shape::capsule(c.half_segment, c.radius);
    let mut characters = Characters::new(
        world(BodyStorage::Ordered, statics),
        capsules(key, shape, [at]),
        settings,
    );
    characters.move_each(|_, body| CharacterMove {
        body,
        desired: v(desired),
        exclude: None,
        autostep: c.autostep.map(|(max_height, min_width)| Autostep {
            max_height,
            min_width,
        }),
    });
    let r = &characters.results[0];
    let t = r.translation + r.depenetration;
    Outcome {
        translation: [t.0, t.1, t.2],
        grounded: r.ground_contact,
    }
}

#[cfg(all(test, feature = "engine", feature = "rapier"))]
mod tests {
    use super::*;

    fn close(a: Outcome, b: Outcome, tolerance: f32) -> bool {
        (0..3).all(|i| (a.translation[i] - b.translation[i]).abs() <= tolerance)
    }

    #[test]
    fn a_free_move_and_a_move_into_a_wall_agree() {
        let geometry = Geometry {
            floor: true,
            boxes: vec![([2.0, 1.0, 0.0], [0.25, 1.0, 2.0])],
            ..Geometry::default()
        };
        let c = Controller::default();
        let at = [0.0, 0.8025, 0.0];
        for desired in [[0.1, 0.0, 0.0], [2.0, -0.01, 0.3]] {
            let (r, e) = (
                rapier(&geometry, c, at, desired),
                engine(&geometry, c, at, desired),
            );
            assert!(close(r, e, 2e-3), "{desired:?}: rapier {r:?}, engine {e:?}");
            assert!(r.grounded && e.grounded);
        }
    }

    #[test]
    fn a_step_edge_move_agrees() {
        // The ARPG stairs case (physics-engine#356/#357): a 0.205 m step edge, mid-climb.
        let geometry = Geometry {
            floor: true,
            boxes: vec![([9.8, 0.1025, 0.0], [0.2, 0.1025, 0.8])],
            ..Geometry::default()
        };
        let c = Controller {
            autostep: Some((0.25, 0.15)),
            ..Controller::default()
        };
        let (at, desired) = ([10.28551, 0.8126206, 0.0], [-0.033333335, -0.002725, 0.0]);
        let (r, e) = (
            rapier(&geometry, c, at, desired),
            engine(&geometry, c, at, desired),
        );
        assert!(close(r, e, 2e-3), "rapier {r:?}, engine {e:?}");
    }

    #[test]
    fn a_shallow_landing_ends_at_the_offset() {
        // physics-engine#349: walking at 2 m/s while falling the last 1.3 mm onto the floor
        // (gap 3.8 mm, approach cosine 0.08) stops `offset` above it, as Rapier does.
        let geometry = Geometry {
            floor: true,
            ..Geometry::default()
        };
        let c = Controller::default();
        let (at, desired) = ([0.0, 0.8038, 0.0], [0.0333, -0.00267, 0.0]);
        let (r, e) = (
            rapier(&geometry, c, at, desired),
            engine(&geometry, c, at, desired),
        );
        assert!(close(r, e, 1e-5), "rapier {r:?}, engine {e:?}");
    }
}
