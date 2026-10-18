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

/// Per-character movement settings, fixed when the character is added. Lengths are scene
/// units; [`Default`] gives the metre values (a 2.5 mm offset, 5 cm snap) and
/// [`World::character_settings`] the same values in the world's
/// [`Config::length_unit`](super::Config::length_unit).
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
struct CharacterState {
    settings: CharacterSettings,
    grounded: bool,
}

/// Character state in dense arrays indexed by body slot (no hashing per move).
#[derive(Clone, Debug, Default)]
pub(super) struct Characters {
    /// Owning body per slot; [`BodyId::NONE`] where the slot has no character.
    ids: Vec<BodyId>,
    settings: Vec<CharacterSettings>,
    grounded: Vec<bool>,
    /// Layout generation at which the character was last known free of overlap
    /// ([`DIRTY`] when unknown). Moves keep a gap to everything, so a character stays free
    /// until bodies are added or removed, or it is placed by the game.
    clear: Vec<(u64, u64)>,
    /// Incremented whenever a character is placed rather than moved: a placed character can
    /// overlap any other, so every character checks again once.
    placements: u64,
}

const DIRTY: (u64, u64) = (u64::MAX, u64::MAX);

impl Characters {
    fn slot(&self, id: BodyId) -> Option<usize> {
        let slot = id.slot() as usize;
        (self.ids.get(slot) == Some(&id)).then_some(slot)
    }
    pub fn contains(&self, id: BodyId) -> bool {
        self.slot(id).is_some()
    }
    fn get(&self, id: BodyId) -> Option<CharacterState> {
        self.slot(id).map(|s| CharacterState {
            settings: self.settings[s],
            grounded: self.grounded[s],
        })
    }
    fn insert(&mut self, id: BodyId, settings: CharacterSettings) {
        let slot = id.slot() as usize;
        if slot >= self.ids.len() {
            self.ids.resize(slot + 1, BodyId::NONE);
            self.settings.resize(slot + 1, CharacterSettings::default());
            self.grounded.resize(slot + 1, false);
            self.clear.resize(slot + 1, DIRTY);
        }
        self.ids[slot] = id;
        self.settings[slot] = settings;
        self.grounded[slot] = false;
        self.clear[slot] = DIRTY;
    }
    /// Whether the character is known free of overlap at layout `generation`.
    fn is_clear(&self, id: BodyId, generation: u64) -> bool {
        self.slot(id)
            .is_some_and(|s| self.clear[s] == (generation, self.placements))
    }
    fn set_clear(&mut self, id: BodyId, generation: Option<u64>) {
        let state = generation.map_or(DIRTY, |g| (g, self.placements));
        if let Some(slot) = self.slot(id) {
            self.clear[slot] = state;
        }
    }
    /// A character was placed instead of moved: overlap is unknown again for all of them.
    pub fn placed(&mut self, id: BodyId) {
        if self.slot(id).is_some() {
            self.placements += 1;
        }
    }
    pub fn remove(&mut self, id: BodyId) {
        if let Some(slot) = self.slot(id) {
            self.ids[slot] = BodyId::NONE;
        }
    }
    fn set_grounded(&mut self, id: BodyId, grounded: bool) {
        if let Some(slot) = self.slot(id) {
            self.grounded[slot] = grounded;
        }
    }
}

/// Contact normals within about 5 degrees of the motion count as head-on.
const HEAD_ON_COS: Real = 0.996;

fn is_static(b: &Body) -> bool {
    b.mass == 0.0 && !b.external
}

impl World {
    /// Default character settings in this world's length unit.
    pub fn character_settings(&self) -> CharacterSettings {
        let unit = self.config.length_unit;
        let metres = CharacterSettings::default();
        CharacterSettings {
            offset: metres.offset * unit,
            snap_to_ground: metres.snap_to_ground.map(|s| s * unit),
            ..metres
        }
    }

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
        self.characters.insert(id, settings);
        Ok(())
    }

    /// Moves characters by their desired translations. Moves run once each in body order
    /// (keyed bodies by key, then keyless by creation), independent of the order of `moves`;
    /// each later mover sees the earlier ones at their new positions. Characters of the batch
    /// that overlap each other are first separated horizontally, half each. `out[i]` is the
    /// result of `moves[i]`.
    pub fn move_characters(
        &mut self,
        moves: &[CharacterMove],
        out: &mut Vec<CharacterMoveResult>,
    ) -> Result<(), Error> {
        out.clear();
        let mut batch = Vec::with_capacity(moves.len());
        for (n, m) in moves.iter().enumerate() {
            if !m.desired.finite() || m.desired.abs().max_component() >= 1e12 {
                return Err(Error::InvalidInput);
            }
            let state = self
                .characters
                .get(m.body)
                .ok_or(Error::MissingBody(m.body))?;
            batch.push((self.bodies[self.index(m.body)?].order, n, state));
        }
        batch.sort_unstable_by_key(|&(order, n, _)| (order, n));
        let mut separation = vec![V::ZERO; moves.len()];
        self.separate_characters(moves, &batch, &mut separation)?;
        out.resize(
            moves.len(),
            CharacterMoveResult {
                body: BodyId::NONE,
                translation: V::ZERO,
                depenetration: V::ZERO,
                grounded: false,
                support: None,
                contacts: 0,
            },
        );
        for &(_, n, state) in &batch {
            let m = moves[n];
            let mut result = self.move_character(m.body, m.desired, state)?;
            result.depenetration += separation[n];
            self.characters.set_grounded(m.body, result.grounded);
            out[n] = result;
        }
        Ok(())
    }

    /// Separates overlapping characters of one batch horizontally, half each, in body order.
    /// A character overlapping one outside the batch moves the whole way.
    fn separate_characters(
        &mut self,
        moves: &[CharacterMove],
        batch: &[(super::Order, usize, CharacterState)],
        separation: &mut [V],
    ) -> Result<(), Error> {
        let mut hits = Vec::new();
        let generation = self.bookkeeping.generation;
        for &(_, n, state) in batch {
            let id = moves[n].body;
            // Two characters can only overlap if one of them was placed or the layout changed.
            if self.characters.is_clear(id, generation) {
                continue;
            }
            let pose = self.character_pose(id)?;
            let filter = QueryFilter {
                exclude: Some(id),
                ..QueryFilter::default()
            };
            self.overlap_shape(pose, filter, &mut hits)
                .map_err(|_| Error::InvalidInput)?;
            for h in hits.iter().filter(|h| h.separation < 0.0) {
                if !self.characters.contains(h.body) {
                    continue;
                }
                let n_flat = V(h.normal.0, 0.0, h.normal.2);
                if n_flat == V::ZERO {
                    continue;
                }
                let push = n_flat.unit() * (state.settings.offset - h.separation);
                match batch.iter().find(|b| moves[b.1].body == h.body) {
                    Some(&(_, other, _)) => {
                        let half = push * 0.5;
                        self.shift_character(id, half)?;
                        self.shift_character(h.body, -half)?;
                        self.characters.placed(h.body);
                        separation[n] += half;
                        separation[other] += -half;
                    }
                    None => {
                        self.shift_character(id, push)?;
                        separation[n] += push;
                    }
                }
            }
        }
        Ok(())
    }

    fn shift_character(&mut self, id: BodyId, by: V) -> Result<(), Error> {
        let p = self.bodies[self.index(id)?].position + by;
        self.place_kinematic(id, p)?;
        self.characters.placed(id);
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
        let generation = self.bookkeeping.generation;
        let mut resolved = self.characters.is_clear(id, generation);
        for _ in 0..if resolved { 0 } else { 4 } {
            self.overlap_shape(pose, filter, &mut hits)
                .map_err(|_| Error::InvalidInput)?;
            let mut push = V::ZERO;
            // Other characters are separated by the character interaction rules, not here.
            for h in hits
                .iter()
                .filter(|h| h.separation < 0.0 && !self.characters.contains(h.body))
            {
                let n = flat(h.normal);
                if n != V::ZERO {
                    push += n.unit() * (s.offset - h.separation);
                }
            }
            if push == V::ZERO {
                resolved = true;
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
            // On walkable ground, downward intent stops instead of sliding down the slope;
            // horizontal intent follows the surface.
            let rest = if !planar && normal.dot(up) >= min_up && rest.dot(up) < 0.0 {
                V(rest.0, 0.0, rest.2)
            } else {
                rest
            };
            remaining = flat(rest - normal * rest.dot(normal));
            // Head-on against another character: sidestep to the right instead of locking.
            if approach > HEAD_ON_COS && self.characters.contains(hit.body) {
                let side = V(direction.0, 0.0, direction.2).cross(up);
                let blocked = V(rest.0, 0.0, rest.2).length();
                if side != V::ZERO {
                    remaining = side.unit() * blocked + V(0.0, remaining.1, 0.0);
                }
            }
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
            self.place_kinematic(id, pose.position)?;
        }
        // The slide kept its gap: free of overlap until the layout changes or it is placed.
        self.characters
            .set_clear(id, resolved.then_some(generation));
        Ok(result)
    }
}
