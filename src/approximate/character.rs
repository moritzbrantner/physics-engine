//! Kinematic character movement (consumer capability C4/C5): collide-and-slide for upright
//! capsules, moved only through the batched [`World::move_characters`].
//!
//! A character is an ordinary capsule body marked `external` (kinematic); the World keeps its
//! settings and support state. Motion keeps a gap of `offset` to everything it touches, so a
//! character pushing into a wall, a ceiling or its floor stays exactly where it is, and zero
//! desired motion never moves it. Penetration is resolved first and reported separately from
//! the requested motion, unless the character's [`Embedded`] setting leaves it in place.

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
    /// Steepest walkable slope, radians from up. Steeper surfaces are walls: sliding along one
    /// never climbs it unless the desired motion rises.
    pub max_slope: Real,
    /// Distance below the character within which it follows the ground down while grounded
    /// (`None`: never). Also the reach of the grounded test beyond `offset`.
    pub snap_to_ground: Option<Real>,
    /// Collide-and-slide iterations per move.
    pub max_slides: u8,
    /// Planar movement: the character stays at its height and moves only horizontally. Such a
    /// character needs no floor collider and is always grounded, with `support` `None`.
    pub plane: Option<Real>,
    /// Bounded pushing between characters (`None`: never pushes nor is pushed). A moving
    /// character pushes a blocking one horizontally by at most the smaller of their two
    /// ceilings per move, in scene units; the pushed character keeps its gap to everything
    /// else, so it is never pushed into walls or other characters.
    pub max_push: Option<Real>,
    /// How a move that starts embedded in geometry is handled.
    pub embedded: Embedded,
}

/// What a character's move does when it starts inside geometry (other characters aside: the
/// batch always separates overlapping characters).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Embedded {
    /// Push the character out first, reported in [`CharacterMoveResult::depenetration`].
    #[default]
    PushOut,
    /// Leave it in place, as Rapier's `move_shape` does for a moving character: only the
    /// requested motion is resolved, a surface the character is inside blocks only motion into
    /// it, and each such contact steps it Rapier's 0.1 mm normal nudge off that surface.
    Stay,
}

/// Automatic step-up for one move ([`CharacterMove::autostep`]), in scene units.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Autostep {
    /// Highest step climbed, measured from the character's lowest point.
    pub max_height: Real,
    /// Free depth needed on top of the step.
    pub min_width: Real,
}

impl Default for CharacterSettings {
    fn default() -> Self {
        Self {
            offset: 0.0025,
            max_slope: core::f32::consts::FRAC_PI_4,
            snap_to_ground: Some(0.05),
            max_slides: 4,
            plane: None,
            max_push: None,
            embedded: Embedded::PushOut,
        }
    }
}

/// One character's requested translation for this move.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CharacterMove {
    pub body: BodyId,
    pub desired: V,
    /// A body this move ignores entirely, as [`QueryFilter::exclude`] does for a query: the
    /// slide, leaving penetration, stepping, snapping and the grounded test all pass through
    /// it, as Rapier's `move_shape` and support cast do through a filter that excludes one
    /// static collider (MMORPG opens its floor halfspace over a lake basin this way). Pushes
    /// the character receives exclude it too, until its next move.
    pub exclude: Option<BodyId>,
    /// Stepping up onto ledges low enough to climb without jumping on this move (`None`:
    /// never), as Rapier's `autostep` with dynamic bodies excluded. The consumer chooses it
    /// per move, as it chooses Rapier's controller: ARPG steps only on walks, moves whose
    /// previous result had [`CharacterMoveResult::ground_contact`] and that do not take off,
    /// so jumps and falls never step.
    pub autostep: Option<Autostep>,
}

/// What one move did.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CharacterMoveResult {
    pub body: BodyId,
    /// Translation applied for the requested motion (slides and snapping included).
    pub translation: V,
    /// Translation applied to leave penetration, never part of `translation`.
    pub depenetration: V,
    /// Translation received from other characters' pushes during the batch, never part of
    /// `translation` or `depenetration`.
    pub pushed: V,
    /// Standing on walkable static support within reach after the move.
    pub grounded: bool,
    /// Rapier's `grounded` after the move: some static contact within the ground prediction
    /// (true distance, in any direction) faces up at all, a step's rounded edge beside the
    /// character included. Implied by `grounded`; a consumer walking up stairs chooses the
    /// next move's [`CharacterMove::autostep`] by it.
    pub ground_contact: bool,
    /// The supporting static body when grounded.
    pub support: Option<BodyId>,
    /// Number of surfaces the motion touched (at most `max_slides`).
    pub contacts: u8,
}

#[derive(Clone, Copy, Debug)]
struct CharacterState {
    settings: CharacterSettings,
}

/// Character state in dense arrays indexed by body slot (no hashing per move).
#[derive(Clone, Debug, Default)]
pub(super) struct Characters {
    /// Owning body per slot; [`BodyId::NONE`] where the slot has no character.
    ids: Vec<BodyId>,
    settings: Vec<CharacterSettings>,
    /// Translation received from pushes in the current batch.
    pushed: Vec<V>,
    /// Layout generation at which the character was last known free of overlap
    /// ([`DIRTY`] when unknown). Moves keep a gap to everything, so a character stays free
    /// until bodies are added or removed, or it is placed by the game.
    clear: Vec<(u64, u64)>,
    /// The body the latest move excluded ([`CharacterMove::exclude`]).
    exclude: Vec<Option<BodyId>>,
    /// Incremented whenever a character is placed rather than moved: a placed character can
    /// overlap any other, so every character checks again once.
    placements: u64,
    /// Batch scratch kept between calls so a warm batch allocates nothing.
    batch: Vec<(super::Order, usize, CharacterState)>,
    separation: Vec<V>,
    hits: Vec<QueryHit>,
    nearby: Vec<BodyId>,
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
        })
    }
    fn insert(&mut self, id: BodyId, settings: CharacterSettings) {
        let slot = id.slot() as usize;
        if slot >= self.ids.len() {
            self.ids.resize(slot + 1, BodyId::NONE);
            self.settings.resize(slot + 1, CharacterSettings::default());
            self.pushed.resize(slot + 1, V::ZERO);
            self.clear.resize(slot + 1, DIRTY);
            self.exclude.resize(slot + 1, None);
        }
        self.ids[slot] = id;
        self.settings[slot] = settings;
        self.clear[slot] = DIRTY;
        self.exclude[slot] = None;
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
    /// Overlap is unknown again for every character.
    fn placed_all(&mut self) {
        self.placements += 1;
    }
    pub fn remove(&mut self, id: BodyId) {
        if let Some(slot) = self.slot(id) {
            self.ids[slot] = BodyId::NONE;
        }
    }
    fn pushed(&mut self, id: BodyId) -> Option<&mut V> {
        self.slot(id).map(|s| &mut self.pushed[s])
    }
    /// The body the character's latest move excluded.
    fn exclude(&self, id: BodyId) -> Option<BodyId> {
        self.slot(id).and_then(|s| self.exclude[s])
    }
    /// Records a move's exclusion. A changed one can uncover overlap the last move kept clear
    /// of (a floor it moved under while excluded), so overlap is unknown again.
    fn set_exclude(&mut self, id: BodyId, exclude: Option<BodyId>) {
        if let Some(slot) = self.slot(id)
            && self.exclude[slot] != exclude
        {
            self.exclude[slot] = exclude;
            self.clear[slot] = DIRTY;
        }
    }
}

/// Rapier's ground prediction beyond `offset`, in metres.
const GROUND_PREDICTION: Real = 0.05;

/// How far, as a fraction of `offset`, a slide's next cast leans off the surface it slid along
/// (Rapier's `normal_nudge_factor`: 1e-4 against its 0.01 offset). Well above the rounding of
/// the projected motion, well below the offset.
const NORMAL_NUDGE: Real = 1e-2;

/// Rapier's `normal_nudge_factor` in metres: the step off a surface an embedded character
/// runs into.
const EMBEDDED_NUDGE: Real = 1e-4;

/// Contact normals within about 5 degrees of the motion count as head-on.
const HEAD_ON_COS: Real = 0.996;

fn is_static(b: &Body) -> bool {
    b.mass == 0.0 && !b.external
}

impl World {
    /// Default character settings in this world's length unit.
    pub fn character_settings(&self) -> CharacterSettings {
        let unit = self.scene.length_unit;
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
            && settings.plane.is_none_or(Real::is_finite)
            && settings.max_push.is_none_or(|p| p.is_finite() && p >= 0.0);
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
    /// that overlap each other are first separated horizontally, half each. A mover pushes
    /// blocking characters within [`CharacterSettings::max_push`]. `out[i]` is the result of
    /// `moves[i]`.
    pub fn move_characters(
        &mut self,
        moves: &[CharacterMove],
        out: &mut Vec<CharacterMoveResult>,
    ) -> Result<(), Error> {
        out.clear();
        let mut batch = std::mem::take(&mut self.characters.batch);
        let mut separation = std::mem::take(&mut self.characters.separation);
        let result = self.move_batch(moves, out, &mut batch, &mut separation);
        self.characters.batch = batch;
        self.characters.separation = separation;
        result
    }

    fn move_batch(
        &mut self,
        moves: &[CharacterMove],
        out: &mut Vec<CharacterMoveResult>,
        batch: &mut Vec<(super::Order, usize, CharacterState)>,
        separation: &mut Vec<V>,
    ) -> Result<(), Error> {
        batch.clear();
        for (n, m) in moves.iter().enumerate() {
            let step_valid = m.autostep.is_none_or(|a| {
                a.max_height.is_finite()
                    && a.max_height > 0.0
                    && a.min_width.is_finite()
                    && a.min_width >= 0.0
            });
            if !m.desired.finite() || m.desired.abs().max_component() >= 1e12 || !step_valid {
                return Err(Error::InvalidInput);
            }
            let state = self
                .characters
                .get(m.body)
                .ok_or(Error::MissingBody(m.body))?;
            batch.push((self.bodies[self.index(m.body)?].order, n, state));
            self.characters.set_exclude(m.body, m.exclude);
            if let Some(pushed) = self.characters.pushed(m.body) {
                *pushed = V::ZERO;
            }
        }
        batch.sort_unstable_by_key(|&(order, n, _)| (order, n));
        separation.clear();
        separation.resize(moves.len(), V::ZERO);
        self.separate_characters(moves, batch, separation)?;
        out.resize(
            moves.len(),
            CharacterMoveResult {
                body: BodyId::NONE,
                translation: V::ZERO,
                depenetration: V::ZERO,
                pushed: V::ZERO,
                grounded: false,
                ground_contact: false,
                support: None,
                contacts: 0,
            },
        );
        for &(_, n, state) in batch.iter() {
            let m = moves[n];
            let mut result = self.move_character(m.body, m.desired, m.autostep, state)?;
            result.depenetration += separation[n];
            out[n] = result;
        }
        for &(_, n, _) in batch.iter() {
            out[n].pushed = self
                .characters
                .pushed(moves[n].body)
                .map_or(V::ZERO, |p| *p);
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
        let mut hits = std::mem::take(&mut self.characters.hits);
        let result = self.separate_with(moves, batch, separation, &mut hits);
        self.characters.hits = hits;
        result
    }

    fn separate_with(
        &mut self,
        moves: &[CharacterMove],
        batch: &[(super::Order, usize, CharacterState)],
        separation: &mut [V],
        hits: &mut Vec<QueryHit>,
    ) -> Result<(), Error> {
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
            self.overlap_shape(pose, filter, hits)
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

    /// Moves character `id` by up to `by` (horizontal), stopping `offset` short of whatever
    /// it would run into; the pusher behind it is passed by. Returns the distance moved.
    fn push_character(
        &mut self,
        id: BodyId,
        pusher: BodyId,
        by: V,
        settings: CharacterSettings,
        hits: &mut Vec<QueryHit>,
    ) -> Result<Real, Error> {
        let length = by.length();
        let direction = by * (1.0 / length);
        let pose = self.character_pose(id)?;
        let reach = length + settings.offset;
        let travel =
            match self.blocking_hit(id, pose, direction * reach, hits, |b| b.id != pusher)? {
                None => length,
                Some(hit) => {
                    let approach = -direction.dot(hit.normal);
                    let allowed = if approach > 0.0 {
                        (hit.fraction * reach - settings.offset / approach).max(0.0)
                    } else {
                        0.0
                    };
                    if allowed <= settings.offset * 1e-2 {
                        0.0
                    } else {
                        allowed.min(length)
                    }
                }
            };
        if travel > 0.0 {
            self.place_kinematic(id, pose.position + direction * travel)?;
            if let Some(pushed) = self.characters.pushed(id) {
                *pushed += direction * travel;
            }
        }
        Ok(travel)
    }

    fn shift_character(&mut self, id: BodyId, by: V) -> Result<(), Error> {
        let p = self.bodies[self.index(id)?].position + by;
        self.place_kinematic(id, p)?;
        self.body_placed(id);
        Ok(())
    }

    /// Body `id` was placed instead of moved, so it can overlap anything: it (if a character)
    /// and every character whose bounds it touches check for overlap again (all of them when
    /// the query tree is stale).
    pub(super) fn body_placed(&mut self, id: BodyId) {
        let Ok(i) = self.index(id) else {
            return;
        };
        let pad = match self.characters.get(id) {
            Some(state) => state.settings.offset,
            None => self.character_settings().offset,
        };
        if !self.query_tree.current(self.bookkeeping.generation) {
            self.characters.placed_all();
            return;
        }
        let (lo, hi) = self.bodies[i].cached_bounds;
        let mut nearby = std::mem::take(&mut self.characters.nearby);
        nearby.clear();
        let nodes = self.query_tree.tree().visit_aabb(
            spatial_kernels::Aabb {
                min: [lo.0 - pad, lo.1 - pad, lo.2 - pad],
                max: [hi.0 + pad, hi.1 + pad, hi.2 + pad],
            },
            |leaf| nearby.push(self.bodies[self.slot_index(leaf.id)].id),
        );
        crate::work::add(crate::work::Counter::TreeNodes, nodes as u64);
        self.characters.set_clear(id, None);
        for &other in &nearby {
            self.characters.set_clear(other, None);
        }
        self.characters.nearby = nearby;
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
        self.cast_hits(id, pose, displacement, hits)?;
        Ok(hits.iter().copied().find(|h| {
            (h.starts_overlapping || h.normal.dot(displacement) < 0.0)
                && self.index(h.body).is_ok_and(|i| admit(&self.bodies[i]))
        }))
    }

    /// Earliest surface `displacement` approaches with the character inflated by `offset`, as
    /// Rapier's cast with that target distance: the hit's `fraction` is where the true gap
    /// closes to `offset`, and surfaces already within `offset` count only when approached.
    fn offset_hit(
        &self,
        id: BodyId,
        mut pose: QueryPose,
        displacement: V,
        offset: Real,
        hits: &mut Vec<QueryHit>,
    ) -> Result<Option<QueryHit>, Error> {
        if let Shape::Capsule { radius, .. } = &mut pose.shape {
            *radius += offset;
        }
        self.cast_hits(id, pose, displacement, hits)?;
        // Sliding along a surface at the offset approaches it only by rounding; motion that
        // closes the gap by under a thousandth of the offset does not count as approaching.
        let tolerance = offset * 1e-3;
        Ok(hits
            .iter()
            .copied()
            .find(|h| h.normal.dot(displacement) < -tolerance))
    }

    fn cast_hits(
        &self,
        id: BodyId,
        pose: QueryPose,
        displacement: V,
        hits: &mut Vec<QueryHit>,
    ) -> Result<(), Error> {
        let filter = QueryFilter {
            exclude: Some(id),
            ..QueryFilter::default()
        };
        let cast = ShapeCast::new(pose, displacement);
        let also = self.characters.exclude(id);
        self.shape_query(pose, Some(cast), filter, also, hits, true)
            .map_err(|failure| match failure.reason {
                super::QueryFailureReason::Search { body, reason } => {
                    Error::CollisionSearchFailed {
                        bodies: [id, body],
                        reason,
                    }
                }
                super::QueryFailureReason::InvalidInput => Error::InvalidInput,
            })?;
        Ok(())
    }

    /// Rapier's step-up: room above, room on top of the step for `min_width`, and no
    /// too-steep slope there. Returns the lift and the forward nudge onto the step, or `None`
    /// when the character cannot step.
    fn step_up(
        &self,
        id: BodyId,
        pose: QueryPose,
        rest: V,
        s: CharacterSettings,
        step: Autostep,
        hits: &mut Vec<QueryHit>,
    ) -> Result<Option<V>, Error> {
        let up = V::Y;
        let min_width = step.min_width + s.offset;
        let max_height = step.max_height + s.offset;
        let horizontal = rest - up * rest.dot(up);
        if horizontal == V::ZERO {
            return Ok(None);
        }
        let forward = horizontal.unit();
        let solid = |b: &Body| !b.movable();
        // Stops `offset` short along a hit's normal, as a cast with Rapier's target distance.
        let stop = |h: &QueryHit, direction: V, reach: Real| {
            let approach = -direction.dot(h.normal);
            if h.starts_overlapping || approach <= 0.0 {
                0.0
            } else {
                (h.fraction * reach - s.offset / approach).max(0.0)
            }
        };
        let reach = max_height + s.offset;
        if self
            .blocking_hit(id, pose, up * reach, hits, solid)?
            .is_some_and(|h| stop(&h, up, reach) <= max_height)
        {
            return Ok(None);
        }
        let mut lifted = pose;
        lifted.position += up * max_height;
        let reach = min_width + s.offset;
        if self
            .blocking_hit(id, lifted, forward * reach, hits, solid)?
            .is_some_and(|h| stop(&h, forward, reach) <= min_width)
        {
            return Ok(None);
        }
        let mut above = lifted;
        above.position += forward * min_width;
        let reach = max_height + s.offset;
        let fall = match self.blocking_hit(id, above, -up * reach, hits, solid)? {
            Some(h) => {
                let n = h.normal;
                if n.dot(up) < s.max_slope.cos() {
                    let along = rest - n * rest.dot(n).min(0.0);
                    if along.dot(up) >= 0.0 {
                        return Ok(None);
                    }
                }
                stop(&h, -up, reach).min(max_height)
            }
            None => max_height,
        };
        let lift = up * (max_height - fall);
        let nudge = forward * forward.dot(rest - lift).min(min_width).max(0.0);
        Ok(Some(lift + nudge))
    }

    /// The nearest static surface below `pose` within `reach` that a downward cast runs into
    /// and whose normal points up by at least `min_up`. Too-steep surfaces met first (a steep
    /// ramp's face at its foot) are passed over, as Rapier's consumers recast past them, so the
    /// floor beneath still supports the character (#395).
    fn support_hit(
        &self,
        id: BodyId,
        pose: QueryPose,
        reach: Real,
        min_up: Real,
        hits: &mut Vec<QueryHit>,
    ) -> Result<Option<QueryHit>, Error> {
        let displacement = -V::Y * reach;
        self.cast_hits(id, pose, displacement, hits)?;
        Ok(hits.iter().copied().find(|h| {
            (h.starts_overlapping || h.normal.dot(displacement) < 0.0)
                && h.normal.dot(V::Y) >= min_up
                && self.index(h.body).is_ok_and(|i| is_static(&self.bodies[i]))
        }))
    }

    /// Rapier's grounded: some static contact within the ground prediction (true distance, in
    /// any direction) faces up at all, a step's rounded edge beside the character included.
    fn ground_contact_at(
        &self,
        id: BodyId,
        pose: QueryPose,
        s: CharacterSettings,
        hits: &mut Vec<QueryHit>,
    ) -> Result<bool, Error> {
        let Shape::Capsule {
            half_segment,
            radius,
        } = pose.shape
        else {
            return Ok(false);
        };
        let mut probe = pose;
        probe.shape = Shape::Capsule {
            half_segment,
            radius: radius + self.ground_prediction(s),
        };
        self.character_overlap(id, probe, hits)?;
        Ok(hits.iter().any(|h| {
            h.normal.dot(V::Y) >= 1e-3
                && self.index(h.body).is_ok_and(|i| is_static(&self.bodies[i]))
        }))
    }

    /// Everything `pose` overlaps except character `id` and its move's exclusion.
    fn character_overlap(
        &self,
        id: BodyId,
        pose: QueryPose,
        hits: &mut Vec<QueryHit>,
    ) -> Result<(), Error> {
        let filter = QueryFilter {
            exclude: Some(id),
            ..QueryFilter::default()
        };
        let also = self.characters.exclude(id);
        self.shape_query(pose, None, filter, also, hits, true)
            .map_err(|_| Error::InvalidInput)?;
        Ok(())
    }

    /// Distance below a character within which walkable ground counts as support: Rapier's
    /// ground prediction, `offset` plus 5 cm in this world's length unit.
    fn ground_prediction(&self, s: CharacterSettings) -> Real {
        s.offset + GROUND_PREDICTION * self.scene.length_unit
    }

    fn move_character(
        &mut self,
        id: BodyId,
        desired: V,
        autostep: Option<Autostep>,
        state: CharacterState,
    ) -> Result<CharacterMoveResult, Error> {
        let mut hits = std::mem::take(&mut self.characters.hits);
        let result = self.move_character_with(id, desired, autostep, state, &mut hits);
        self.characters.hits = hits;
        result
    }

    fn move_character_with(
        &mut self,
        id: BodyId,
        desired: V,
        autostep: Option<Autostep>,
        state: CharacterState,
        hits: &mut Vec<QueryHit>,
    ) -> Result<CharacterMoveResult, Error> {
        let s = state.settings;
        let up = V::Y;
        let min_up = s.max_slope.cos();
        let planar = s.plane.is_some();
        // Planar characters see only the horizontal part of motion and surface normals.
        let flat = |v: V| if planar { V(v.0, 0.0, v.2) } else { v };
        let mut pose = self.character_pose(id)?;
        let start_pose = pose;
        let start = pose.position;
        let mut result = CharacterMoveResult {
            body: id,
            translation: V::ZERO,
            depenetration: V::ZERO,
            pushed: V::ZERO,
            grounded: false,
            ground_contact: false,
            support: None,
            contacts: 0,
        };
        let mut push_budget = s.max_push.unwrap_or(0.0);

        // Leave penetration first; reported apart from the requested motion. A few passes
        // resolve overlap with several bodies at once (a pillar against a wall).
        let generation = self.bookkeeping.generation;
        let mut resolved = self.characters.is_clear(id, generation);
        if !resolved && s.embedded == Embedded::Stay {
            // Left in geometry: clear once no other character overlaps it (the batch
            // separates those).
            self.character_overlap(id, pose, hits)?;
            resolved = !hits
                .iter()
                .any(|h| h.separation < 0.0 && self.characters.contains(h.body));
        }
        let passes = if resolved || s.embedded == Embedded::Stay {
            0
        } else {
            4
        };
        for _ in 0..passes {
            self.character_overlap(id, pose, hits)?;
            let mut push = V::ZERO;
            // Other characters are separated by the character interaction rules, not here.
            for h in hits
                .iter()
                .filter(|h| h.separation < 0.0 && !self.characters.contains(h.body))
            {
                let n = flat(h.normal);
                if n != V::ZERO {
                    // Only what the push so far leaves along this normal: coincident surfaces
                    // (a floor plane under flat terrain) push once, not once each.
                    let n = n.unit();
                    let need = s.offset - h.separation - push.dot(n);
                    if need > 0.0 {
                        push += n * need;
                    }
                }
            }
            if push == V::ZERO {
                // Still inside another character (or pushed into one): not clear, so the
                // next batch separates them, as it would after a restore.
                resolved = !hits
                    .iter()
                    .any(|h| h.separation < 0.0 && self.characters.contains(h.body));
                break;
            }
            pose.position += push;
            result.depenetration += push;
        }

        // Collide and slide, keeping `offset` to every surface.
        let mut translation = V::ZERO;
        let mut remaining = flat(desired);
        let mut support = None;
        // The surface the slide last met: the next cast leans off it by the nudge.
        let mut leaving = V::ZERO;
        for _ in 0..s.max_slides {
            let length = remaining.length();
            if length <= s.offset * 1e-3 {
                break;
            }
            let direction = remaining * (1.0 / length);
            // Cast the character inflated by `offset` (Rapier's target distance): a hit is
            // where the true gap to a surface the motion approaches closes to `offset`. The
            // cast leans off the surface just slid along (Rapier's normal nudge), so rounding
            // in the projection never makes that surface the first contact, at fraction 0, of
            // a terrain whose next triangle the slide runs into (#394). Only the cast leans;
            // the character moves by `remaining`.
            let cast = remaining + leaving * (s.offset * NORMAL_NUDGE);
            let Some(hit) = self.offset_hit(id, pose, cast, s.offset, hits)? else {
                pose.position += remaining;
                translation += remaining;
                break;
            };
            let normal = flat(hit.normal);
            if normal == V::ZERO {
                // A planar character meeting a purely horizontal surface: nothing blocks it.
                pose.position += remaining;
                translation += remaining;
                break;
            }
            let normal = normal.unit();
            let approach = -direction.dot(normal);
            let allowed = hit.fraction * length;
            if allowed >= length {
                pose.position += remaining;
                translation += remaining;
                break;
            }
            result.contacts += 1;
            leaving = normal;
            if s.embedded == Embedded::Stay && hit.separation + s.offset < 0.0 {
                // Inside the surface it runs into: Rapier's slide steps off it by its normal
                // nudge, so a character left embedded works its way out at that rate.
                let nudge = normal * (EMBEDDED_NUDGE * self.scene.length_unit);
                pose.position += nudge;
                translation += nudge;
            }
            // Already at the gap (within rounding): stay put rather than creep by an ulp.
            let travel = if allowed <= s.offset * 1e-2 {
                0.0
            } else {
                allowed
            };
            pose.position += direction * travel;
            translation += direction * travel;
            let rest = remaining - direction * travel;
            // A low ledge in the way of a stepping move is stepped onto instead of slid along,
            // as Rapier's autostep does on a wall.
            if let Some(step) = autostep
                && !planar
                && normal.dot(up) >= 0.0
                && normal.dot(up) < min_up
                && self
                    .index(hit.body)
                    .is_ok_and(|i| !self.bodies[i].movable())
                && let Some(lift) = self.step_up(id, pose, rest, s, step, hits)?
            {
                pose.position += lift;
                translation += lift;
                remaining = rest - lift;
                continue;
            }
            // On walkable ground the character never slips down the slope unless its
            // horizontal intent leads downhill (a swimmer diving down a basin's slope keeps
            // its descent): as Rapier's non-slip slopes, motion whose slide would point down
            // the slope keeps only its part along the slope's contour.
            let tangent = rest - normal * rest.dot(normal);
            let rest = if !planar
                && normal.dot(up) >= min_up
                && tangent.dot(up) < 0.0
                && V(desired.0, 0.0, desired.2).dot(normal) <= 0.0
            {
                let contour = normal.cross(up);
                if contour == V::ZERO {
                    V::ZERO
                } else {
                    let contour = contour.unit();
                    contour * tangent.dot(contour)
                }
            } else {
                rest
            };
            remaining = flat(rest - normal * rest.dot(normal));
            // A slope too steep to walk is a wall: the slide keeps no motion up along it
            // unless the intent itself rises (as Rapier's `max_slope_climb_angle`).
            if !planar && normal.dot(up) < min_up && desired.dot(up) <= 0.0 {
                let uphill = up - normal * normal.dot(up);
                if uphill != V::ZERO && remaining.dot(uphill) > 0.0 {
                    let uphill = uphill.unit();
                    remaining -= uphill * remaining.dot(uphill);
                }
            }
            if let Some(other) = self.characters.get(hit.body) {
                let mut rest = rest;
                let away = V(-normal.0, 0.0, -normal.2);
                if let Some(ceiling) = other.settings.max_push
                    && push_budget > 0.0
                    && away != V::ZERO
                {
                    let away = away.unit();
                    let wanted = V(rest.0, 0.0, rest.2)
                        .dot(away)
                        .min(push_budget)
                        .min(ceiling);
                    if wanted > s.offset * 1e-2 {
                        // Room past the pushed character, from the hits this cast already found.
                        let room = hits
                            .iter()
                            .filter(|h| h.body != hit.body)
                            .filter(|h| {
                                let n = flat(h.normal);
                                n != V::ZERO && direction.dot(n) < 0.0
                            })
                            .map(|h| h.fraction * length)
                            .fold(Real::INFINITY, Real::min)
                            - travel;
                        let pushed =
                            self.push_character(hit.body, id, away * wanted, other.settings, hits)?;
                        push_budget -= pushed;
                        // Follow the pushed character along the motion, keeping the gap to it.
                        let follow = (pushed * away.dot(-normal) / approach)
                            .min(room)
                            .min(rest.dot(direction))
                            .max(0.0);
                        pose.position += direction * follow;
                        translation += direction * follow;
                        rest -= direction * follow;
                        remaining = flat(rest - normal * rest.dot(normal));
                    }
                }
                // Head-on against another character: sidestep to the right instead of locking.
                if approach > HEAD_ON_COS {
                    let side = V(direction.0, 0.0, direction.2).cross(up);
                    let blocked = V(rest.0, 0.0, rest.2).length();
                    if side != V::ZERO {
                        remaining = side.unit() * blocked + V(0.0, remaining.1, 0.0);
                    }
                }
            }
        }

        // Grounded test and snapping: walkable static support within reach below. Ground
        // within the prediction counts as support without moving the character (as in Rapier).
        let predict = self.ground_prediction(s);
        let reach = (predict / min_up).max(s.offset + s.snap_to_ground.unwrap_or(0.0)) + s.offset;
        if planar {
            // The plane is the support.
        } else if let Some(hit) = self.support_hit(id, pose, reach, min_up, hits)? {
            let gap = hit.fraction * reach;
            let snap = gap - s.offset;
            // As Rapier: a move that started in ground contact (Rapier's grounded, a step's
            // rounded edge included) and has not risen (a step or a slide up an edge keeps its
            // height) snaps down to the offset from any gap beyond it.
            if snap > s.offset * 1e-2
                && translation.dot(up) <= 0.0
                && s.snap_to_ground.is_some_and(|reach| snap <= reach)
                && self.ground_contact_at(id, start_pose, s, hits)?
            {
                pose.position += -up * snap;
                translation += -up * snap;
                support = Some(hit.body);
            } else if gap * hit.normal.dot(up) <= predict {
                // Rapier measures the prediction along the contact normal, not vertically: on
                // a slope, ground a little beyond it straight down is still within it.
                support = Some(hit.body);
            }
        }

        // Accumulated, not a difference of positions, so a free move reports `desired` exactly.
        result.translation = translation;
        result.grounded = planar || support.is_some();
        result.support = support;
        if pose.position != start {
            self.place_kinematic(id, pose.position)?;
        }
        // Walkable support is a contact facing up; only off it does the contact test run.
        result.ground_contact = result.grounded || self.ground_contact_at(id, pose, s, hits)?;
        // The slide kept its gap: free of overlap until the layout changes or it is placed.
        self.characters
            .set_clear(id, resolved.then_some(generation));
        Ok(result)
    }
}
