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
    /// Stepping up onto ledges low enough to climb without jumping (`None`: never), as
    /// Rapier's `autostep` with dynamic bodies excluded. Only walking moves step: ones that
    /// start on the ground and do not rise, so jumps and falls never step (ARPG's use).
    pub autostep: Option<Autostep>,
}

/// Automatic step-up, in scene units.
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
            autostep: None,
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
    /// Translation received from other characters' pushes during the batch, never part of
    /// `translation` or `depenetration`.
    pub pushed: V,
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
        }
        self.ids[slot] = id;
        self.settings[slot] = settings;
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
}

/// Rapier's ground prediction beyond `offset`, in metres.
const GROUND_PREDICTION: Real = 0.05;

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
            && settings.plane.is_none_or(Real::is_finite)
            && settings.max_push.is_none_or(|p| p.is_finite() && p >= 0.0)
            && settings.autostep.is_none_or(|a| {
                a.max_height.is_finite()
                    && a.max_height > 0.0
                    && a.min_width.is_finite()
                    && a.min_width >= 0.0
            });
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
            if !m.desired.finite() || m.desired.abs().max_component() >= 1e12 {
                return Err(Error::InvalidInput);
            }
            let state = self
                .characters
                .get(m.body)
                .ok_or(Error::MissingBody(m.body))?;
            batch.push((self.bodies[self.index(m.body)?].order, n, state));
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
                support: None,
                contacts: 0,
            },
        );
        for &(_, n, state) in batch.iter() {
            let m = moves[n];
            let mut result = self.move_character(m.body, m.desired, state)?;
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

    /// Whether `pose` stands on walkable static support (within twice the offset). Snapping
    /// asks this of the move's start, as Rapier does, so it depends on geometry alone and a
    /// world rebuilt from game state snaps exactly like the original.
    fn grounded_at(
        &self,
        id: BodyId,
        pose: QueryPose,
        s: CharacterSettings,
        hits: &mut Vec<QueryHit>,
    ) -> Result<bool, Error> {
        self.on_ground_at(id, pose, s, s.max_slope.cos(), hits)
    }

    /// Whether `pose` has static ground within the ground prediction below whose normal points
    /// up by at least `min_up`.
    fn on_ground_at(
        &self,
        id: BodyId,
        pose: QueryPose,
        s: CharacterSettings,
        min_up: Real,
        hits: &mut Vec<QueryHit>,
    ) -> Result<bool, Error> {
        let predict = self.ground_prediction(s);
        let reach = predict + s.offset;
        Ok(self
            .blocking_hit(id, pose, -V::Y * reach, hits, is_static)?
            .is_some_and(|hit| hit.normal.dot(V::Y) >= min_up && hit.fraction * reach <= predict))
    }

    /// Distance below a character within which walkable ground counts as support: Rapier's
    /// ground prediction, `offset` plus 5 cm in this world's length unit.
    fn ground_prediction(&self, s: CharacterSettings) -> Real {
        s.offset + GROUND_PREDICTION * self.config.length_unit
    }

    fn move_character(
        &mut self,
        id: BodyId,
        desired: V,
        state: CharacterState,
    ) -> Result<CharacterMoveResult, Error> {
        let mut hits = std::mem::take(&mut self.characters.hits);
        let result = self.move_character_with(id, desired, state, &mut hits);
        self.characters.hits = hits;
        result
    }

    fn move_character_with(
        &mut self,
        id: BodyId,
        desired: V,
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
            support: None,
            contacts: 0,
        };
        let mut push_budget = s.max_push.unwrap_or(0.0);

        // Leave penetration first; reported apart from the requested motion. A few passes
        // resolve overlap with several bodies at once (a pillar against a wall).
        let filter = QueryFilter {
            exclude: Some(id),
            ..QueryFilter::default()
        };
        let generation = self.bookkeeping.generation;
        let mut resolved = self.characters.is_clear(id, generation);
        for _ in 0..if resolved { 0 } else { 4 } {
            self.overlap_shape(pose, filter, hits)
                .map_err(|_| Error::InvalidInput)?;
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
                resolved = true;
                break;
            }
            pose.position += push;
            result.depenetration += push;
        }

        // Collide and slide, keeping `offset` to every surface.
        let mut translation = V::ZERO;
        let mut remaining = flat(desired);
        // Whether the move starts on the ground (Rapier's grounded: any upward-facing static
        // contact within the prediction, a step's rounded edge included), found once on the
        // first step attempt.
        let mut walking = None;
        let mut support = None;
        for _ in 0..s.max_slides {
            let length = remaining.length();
            if length <= s.offset * 1e-3 {
                break;
            }
            let direction = remaining * (1.0 / length);
            // Cast one offset further so motion never ends closer than `offset`.
            let reach = length + s.offset;
            let Some(hit) = self.blocking_hit(id, pose, direction * reach, hits, |_| true)? else {
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
            // Stop `offset` short of the surface along its normal.
            let mut allowed = if approach > 0.0 {
                (hit.fraction * reach - s.offset / approach).max(0.0)
            } else {
                0.0
            };
            // A later-touched surface approached at a shallow angle (the floor under a
            // descending walk into a steep slope) may need the motion to stop sooner.
            let first = hit.body;
            let (mut hit, mut normal, mut approach) = (hit, normal, approach);
            for other in hits.iter().filter(|h| h.body != first) {
                let n = flat(other.normal);
                if n == V::ZERO || other.starts_overlapping {
                    continue;
                }
                let n = n.unit();
                let a = -direction.dot(n);
                if a > 0.0 {
                    let stop = (other.fraction * reach - s.offset / a).max(0.0);
                    if stop < allowed {
                        (allowed, hit, normal, approach) = (stop, *other, n, a);
                    }
                }
            }
            if allowed >= length {
                pose.position += remaining;
                translation += remaining;
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
            translation += direction * travel;
            let rest = remaining - direction * travel;
            // A low ledge in the way of a walking character is stepped onto instead of slid
            // along. Jumps and falls never step (as ARPG uses Rapier's autostep).
            if let Some(step) = s.autostep
                && !planar
                && normal.dot(up) >= 0.0
                && normal.dot(up) < min_up
                && desired.dot(up) <= 0.0
                && self
                    .index(hit.body)
                    .is_ok_and(|i| !self.bodies[i].movable())
                && *walking.get_or_insert(self.on_ground_at(id, start_pose, s, 1e-3, hits)?)
                && let Some(lift) = self.step_up(id, pose, rest, s, step, hits)?
            {
                pose.position += lift;
                translation += lift;
                remaining = rest - lift;
                continue;
            }
            // On walkable ground, downward intent stops instead of sliding down the slope;
            // horizontal intent follows the surface.
            let rest = if !planar && normal.dot(up) >= min_up && rest.dot(up) < 0.0 {
                V(rest.0, 0.0, rest.2)
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
                            .filter_map(|h| {
                                let n = flat(h.normal);
                                let approach = -direction.dot(n.unit());
                                (n != V::ZERO && (h.starts_overlapping || approach > 0.0)).then(
                                    || {
                                        if approach > 0.0 {
                                            (h.fraction * reach - s.offset / approach).max(0.0)
                                        } else {
                                            0.0
                                        }
                                    },
                                )
                            })
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
        let reach = predict.max(s.offset + s.snap_to_ground.unwrap_or(0.0)) + s.offset;
        if planar {
            // The plane is the support.
        } else if let Some(hit) = self.blocking_hit(id, pose, -up * reach, hits, is_static)?
            && hit.normal.dot(up) >= min_up
        {
            let gap = hit.fraction * reach;
            let snap = gap - s.offset;
            // As Rapier: only a move that has not risen snaps (a step or a slide up an edge
            // keeps its height).
            if gap > s.offset * 2.0
                && translation.dot(up) <= 0.0
                && s.snap_to_ground.is_some_and(|reach| snap <= reach)
                && self.grounded_at(id, start_pose, s, hits)?
            {
                pose.position += -up * snap;
                translation += -up * snap;
                support = Some(hit.body);
            } else if gap <= predict {
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
        // The slide kept its gap: free of overlap until the layout changes or it is placed.
        self.characters
            .set_clear(id, resolved.then_some(generation));
        Ok(result)
    }
}
