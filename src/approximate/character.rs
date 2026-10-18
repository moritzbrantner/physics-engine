//! Kinematic character movement (consumer capability C4/C5): collide-and-slide for upright
//! capsules, moved only through the batched [`World::move_characters`].
//!
//! A character is an ordinary capsule body marked `external` (kinematic); the World keeps its
//! settings and support state. Motion keeps a gap of `offset` to everything it touches, so a
//! character pushing into a wall, a ceiling or its floor stays exactly where it is, and zero
//! desired motion never moves it. Penetration is resolved first and reported separately from
//! the requested motion.

use super::{
    Body, BodyId, Error, QueryFilter, QueryHit, QueryPose, Real, Shape, ShapeCast, Vector as V,
    World,
};

/// Per-character movement settings, fixed when the character is added.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CharacterSettings {
    /// Gap kept to everything the character touches, in scene units.
    pub offset: Real,
    /// Steepest walkable slope, radians from up.
    pub max_slope: Real,
    /// Distance below the character within which it follows the ground down while grounded
    /// (`None`: never). Also the reach of the grounded test beyond `offset`.
    pub snap_to_ground: Option<Real>,
    /// Collide-and-slide iterations per move.
    pub max_slides: u8,
    /// Planar movement: the character stays at its height and moves only horizontally. Such a
    /// character needs no floor collider and is always grounded, with `support` `None`.
    pub plane: Option<Real>,
}

impl Default for CharacterSettings {
    fn default() -> Self {
        Self {
            offset: 0.0025,
            max_slope: core::f32::consts::FRAC_PI_4,
            snap_to_ground: Some(0.05),
            max_slides: 4,
            plane: None,
        }
    }
}

/// One character's requested translation for this move.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CharacterMove {
    pub body: BodyId,
    pub desired: V,
}

/// What one move did.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CharacterMoveResult {
    pub body: BodyId,
    /// Translation applied for the requested motion (slides and snapping included).
    pub translation: V,
    /// Translation applied to leave penetration, never part of `translation`.
    pub depenetration: V,
    /// Standing on walkable static support within reach after the move.
    pub grounded: bool,
    /// The supporting static body when grounded.
    pub support: Option<BodyId>,
    /// Number of surfaces the motion touched (at most `max_slides`).
    pub contacts: u8,
}

#[derive(Clone, Copy, Debug)]
pub(super) struct CharacterState {
    pub settings: CharacterSettings,
    pub grounded: bool,
}

fn is_static(b: &Body) -> bool {
    b.mass == 0.0 && !b.external
}

impl World {
    /// Makes an existing upright capsule body a kinematic character.
    pub fn add_character(&mut self, id: BodyId, settings: CharacterSettings) -> Result<(), Error> {
        let i = self.index(id)?;
        let valid = settings.offset.is_finite()
            && settings.offset > 0.0
            && settings.max_slope.is_finite()
            && (0.0..core::f32::consts::FRAC_PI_2).contains(&settings.max_slope)
            && settings
                .snap_to_ground
                .is_none_or(|s| s.is_finite() && s >= 0.0)
            && settings.max_slides > 0
            && settings.plane.is_none_or(Real::is_finite);
        let body = &self.bodies[i];
        if !valid || !matches!(body.shape, Shape::Capsule { .. }) || !body.external {
            return Err(Error::InvalidInput);
        }
        self.characters.insert(
            id,
            CharacterState {
                settings,
                grounded: false,
            },
        );
        Ok(())
    }

    /// Moves characters by their desired translations, in the given order: each later move
    /// sees the earlier characters at their new positions. Results replace `out`.
    pub fn move_characters(
        &mut self,
        moves: &[CharacterMove],
        out: &mut Vec<CharacterMoveResult>,
    ) -> Result<(), Error> {
        out.clear();
        for m in moves {
            if !m.desired.finite() || m.desired.abs().max_component() >= 1e12 {
                return Err(Error::InvalidInput);
            }
            let state = *self
                .characters
                .get(&m.body)
                .ok_or(Error::MissingBody(m.body))?;
            let result = self.move_character(m.body, m.desired, state)?;
            if let Some(s) = self.characters.get_mut(&m.body) {
                s.grounded = result.grounded;
            }
            out.push(result);
        }
        Ok(())
    }

    fn character_pose(&self, id: BodyId) -> Result<QueryPose, Error> {
        let b = &self.bodies[self.index(id)?];
        Ok(QueryPose {
            shape: b.shape,
            position: b.position,
            orientation: b.orientation,
        })
    }

    /// Earliest hit of `pose` swept by `displacement` that the motion runs into (touching
    /// surfaces it moves away from are ignored), excluding the character itself.
    fn blocking_hit(
        &self,
        id: BodyId,
        pose: QueryPose,
        displacement: V,
        hits: &mut Vec<QueryHit>,
        admit: impl Fn(&Body) -> bool,
    ) -> Result<Option<QueryHit>, Error> {
        let filter = QueryFilter {
            exclude: Some(id),
            ..QueryFilter::default()
        };
        self.cast_shape(ShapeCast::new(pose, displacement), filter, hits)
            .map_err(|failure| match failure.reason {
                super::QueryFailureReason::Search { body, reason } => {
                    Error::CollisionSearchFailed {
                        bodies: [id, body],
                        reason,
                    }
                }
                super::QueryFailureReason::InvalidInput => Error::InvalidInput,
            })?;
        Ok(hits.iter().copied().find(|h| {
            (h.starts_overlapping || h.normal.dot(displacement) < 0.0)
                && self.index(h.body).is_ok_and(|i| admit(&self.bodies[i]))
        }))
    }

    fn move_character(
        &mut self,
        id: BodyId,
        desired: V,
        state: CharacterState,
    ) -> Result<CharacterMoveResult, Error> {
        let s = state.settings;
        let up = V::Y;
        let min_up = s.max_slope.cos();
        let planar = s.plane.is_some();
        // Planar characters see only the horizontal part of motion and surface normals.
        let flat = |v: V| if planar { V(v.0, 0.0, v.2) } else { v };
        let mut pose = self.character_pose(id)?;
        let start = pose.position;
        let mut hits = Vec::new();
        let mut result = CharacterMoveResult {
            body: id,
            translation: V::ZERO,
            depenetration: V::ZERO,
            grounded: false,
            support: None,
            contacts: 0,
        };

        // Leave penetration first; reported apart from the requested motion. A few passes
        // resolve overlap with several bodies at once (a pillar against a wall).
        let filter = QueryFilter {
            exclude: Some(id),
            ..QueryFilter::default()
        };
        for _ in 0..4 {
            self.overlap_shape(pose, filter, &mut hits)
                .map_err(|_| Error::InvalidInput)?;
            let mut push = V::ZERO;
            // Other characters are separated by the character interaction rules, not here.
            for h in hits
                .iter()
                .filter(|h| h.separation < 0.0 && !self.characters.contains_key(&h.body))
            {
                let n = flat(h.normal);
                if n != V::ZERO {
                    push += n.unit() * (s.offset - h.separation);
                }
            }
            if push == V::ZERO {
                break;
            }
            pose.position += push;
            result.depenetration += push;
        }

        // Collide and slide, keeping `offset` to every surface.
        let mut remaining = flat(desired);
        let mut support = None;
        for _ in 0..s.max_slides {
            let length = remaining.length();
            if length <= s.offset * 1e-3 {
                break;
            }
            let direction = remaining * (1.0 / length);
            // Cast one offset further so motion never ends closer than `offset`.
            let reach = length + s.offset;
            let Some(hit) = self.blocking_hit(id, pose, direction * reach, &mut hits, |_| true)?
            else {
                pose.position += remaining;
                break;
            };
            let normal = flat(hit.normal);
            if normal == V::ZERO {
                // A planar character meeting a purely horizontal surface: nothing blocks it.
                pose.position += remaining;
                break;
            }
            let normal = normal.unit();
            let approach = -direction.dot(normal);
            // Stop `offset` short of the surface along its normal.
            let allowed = if approach > 0.0 {
                (hit.fraction * reach - s.offset / approach).max(0.0)
            } else {
                0.0
            };
            if allowed >= length {
                pose.position += remaining;
                break;
            }
            result.contacts += 1;
            // Already at the gap (within rounding): stay put rather than creep by an ulp.
            let travel = if allowed <= s.offset * 1e-2 {
                0.0
            } else {
                allowed
            };
            pose.position += direction * travel;
            let rest = remaining - direction * travel;
            remaining = flat(rest - normal * rest.dot(normal));
        }

        // Grounded test and snapping: walkable static support within reach below.
        let reach = s.offset + s.snap_to_ground.unwrap_or(0.0) + s.offset;
        if planar {
            // The plane is the support.
        } else if let Some(hit) = self.blocking_hit(id, pose, -up * reach, &mut hits, is_static)?
            && hit.normal.dot(up) >= min_up
        {
            let gap = hit.fraction * reach;
            let snap = gap - s.offset;
            let may_snap = state.grounded && desired.dot(up) <= 0.0 && s.snap_to_ground.is_some();
            if gap <= s.offset * 2.0 {
                support = Some(hit.body);
            } else if may_snap {
                pose.position += -up * snap;
                support = Some(hit.body);
            }
        }

        result.translation = pose.position - start - result.depenetration;
        result.grounded = planar || support.is_some();
        result.support = support;
        if pose.position != start {
            self.set_kinematic_position(id, pose.position)?;
        }
        Ok(result)
    }
}
