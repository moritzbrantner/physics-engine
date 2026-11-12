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
    /// What the character's [`Tether`] added to `desired` before the slide: the outward motion
    /// it took out and the pull back of a stretched rope (zero without one, or while slack).
    /// A consumer deriving its velocity from `translation` sees it there already.
    pub tether: V,
}

/// A rope from a fixed world point to a character ([`World::tether_character`]; a grapple
/// hook). Each move of the character keeps the attachment within `length` of `anchor`: the
/// part of the desired motion that would take it past the rope's length is removed (motion
/// across the rope is kept, so the character swings), a rope already stretched draws it back
/// by up to `max_pull`, and the slide then resolves collisions as for any move. Scene units.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Tether {
    /// The fixed world point the rope holds on.
    pub anchor: V,
    /// Where the rope holds on the character, relative to its centre (characters stay
    /// upright, so this is a world-frame offset: the shooter's eye is `(0, 0.75, 0)`).
    pub attachment: V,
    /// The rope's length: the most the attachment may be from `anchor` after a move.
    pub length: Real,
    /// The most one move draws a stretched rope's character back toward `anchor` (the
    /// shooter's 24 m/s at 60 Hz: 0.4 m).
    pub max_pull: Real,
}

impl Tether {
    fn valid(&self) -> bool {
        let point = |p: V| p.finite() && p.abs().max_component() < 1e12;
        point(self.anchor)
            && point(self.attachment)
            && valid_length(self.length)
            && self.max_pull.is_finite()
            && (0.0..1e12).contains(&self.max_pull)
    }

    /// `desired` from a character centred at `centre`, with the rope's change to it.
    fn constrain(&self, centre: V, desired: V, offset: Real) -> V {
        let to_anchor = self.anchor - (centre + self.attachment);
        let span = to_anchor.length();
        if span <= offset * 1e-3 {
            return V::ZERO;
        }
        let toward = to_anchor * (1.0 / span);
        // Linearized about the start: the move's motion toward the anchor must make up what
        // the attachment stands beyond the rope's length (bounded by `max_pull`); a slack rope
        // admits outward motion up to its slack.
        let wanted = (span - self.length).min(self.max_pull);
        let inward = desired.dot(toward);
        if inward < wanted {
            toward * (wanted - inward)
        } else {
            V::ZERO
        }
    }
}

fn valid_length(length: Real) -> bool {
    length.is_finite() && length > 0.0 && length < 1e12
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
    /// Layout/placement hint for avoiding redundant character/character separation.
    /// [`DIRTY`] means unknown. This is never an authoritative decision about static
    /// penetration: only an exact pose verified by an overlap query can skip that query.
    clear: Vec<(u64, u64)>,
    /// An exact pose actually observed free of overlap, valid until placement or stepping.
    verified_clear: Vec<Option<(QueryPose, f64)>>,
    /// The body the latest move excluded ([`CharacterMove::exclude`]).
    exclude: Vec<Option<BodyId>>,
    /// The latest zero-motion move's support outcome, reused by the next zero-motion move from
    /// the same verified pose while nothing it depends on changed (#526).
    still: Vec<Option<StillMove>>,
    /// The character's rope ([`World::tether_character`]).
    tether: Vec<Option<Tether>>,
    /// How many characters hold a tether (checkpoints reject a world with any).
    tethers: usize,
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

/// What a zero-motion move from a verified clear pose found below the character, and the key
/// it holds for (#526). Such a move neither slides nor depenetrates, and its support and ground
/// contact tests admit only static bodies, so it is a function of the exact pose, the move's
/// exclusion and the static geometry: the layout generation and [`World::static_revision`]
/// cover the latter. Only a move that did not snap is kept (it left the pose unchanged).
#[derive(Clone, Copy, Debug)]
struct StillMove {
    /// The pose bitwise: position, orientation, then the capsule's half segment and radius.
    pose: [u32; 9],
    exclude: Option<BodyId>,
    generation: u64,
    revision: u64,
    support: Option<BodyId>,
    ground_contact: bool,
}

impl StillMove {
    fn key(pose: QueryPose) -> [u32; 9] {
        let (half_segment, radius) = match pose.shape {
            Shape::Capsule {
                half_segment,
                radius,
            } => (half_segment, radius),
            _ => (Real::NAN, Real::NAN),
        };
        let (p, q) = (pose.position, pose.orientation);
        [p.0, p.1, p.2, q.0, q.1, q.2, q.3, half_segment, radius].map(Real::to_bits)
    }
}

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
            self.verified_clear.resize(slot + 1, None);
            self.exclude.resize(slot + 1, None);
            self.still.resize(slot + 1, None);
            self.tether.resize(slot + 1, None);
        }
        self.release(slot);
        self.ids[slot] = id;
        self.settings[slot] = settings;
        self.clear[slot] = DIRTY;
        self.verified_clear[slot] = None;
        self.exclude[slot] = None;
        self.still[slot] = None;
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
            if generation.is_none() {
                self.verified_clear[slot] = None;
            }
        }
    }
    /// Overlap is unknown again for every character.
    fn placed_all(&mut self) {
        self.placements += 1;
    }
    /// Every registration in slot order with the state a next move reads: its settings, the
    /// body its latest move excluded and whether it is known free of overlap at `generation`
    /// (checkpoint capture, #452). Push totals and batch scratch are per batch.
    #[cfg(feature = "checkpoints")]
    pub fn entries(
        &self,
        generation: u64,
    ) -> impl Iterator<Item = (BodyId, CharacterSettings, Option<BodyId>, bool)> + '_ {
        (0..self.ids.len())
            .filter(|&s| self.ids[s] != BodyId::NONE)
            .map(move |s| {
                (
                    self.ids[s],
                    self.settings[s],
                    self.exclude[s],
                    self.clear[s] == (generation, self.placements),
                )
            })
    }
    /// Registers a restored character with its latest exclusion, free of overlap at
    /// `clear` (a layout generation) when known.
    #[cfg(feature = "checkpoints")]
    pub fn restore(
        &mut self,
        id: BodyId,
        settings: CharacterSettings,
        exclude: Option<BodyId>,
        clear: Option<u64>,
    ) {
        self.insert(id, settings);
        let slot = id.slot() as usize;
        self.exclude[slot] = exclude;
        self.set_clear(id, clear);
    }
    pub fn remove(&mut self, id: BodyId) {
        if let Some(slot) = self.slot(id) {
            self.ids[slot] = BodyId::NONE;
            self.still[slot] = None;
            self.release(slot);
        }
    }
    fn release(&mut self, slot: usize) -> Option<Tether> {
        let tether = self.tether[slot].take();
        self.tethers -= usize::from(tether.is_some());
        tether
    }
    /// Whether any character holds a tether.
    #[cfg(feature = "checkpoints")]
    pub fn any_tether(&self) -> bool {
        self.tethers > 0
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
            self.verified_clear[slot] = None;
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

impl CharacterSettings {
    pub(super) fn valid(&self) -> bool {
        self.offset.is_finite()
            && self.offset > 0.0
            && self.max_slope.is_finite()
            && (0.0..core::f32::consts::FRAC_PI_2).contains(&self.max_slope)
            && self
                .snap_to_ground
                .is_none_or(|s| s.is_finite() && s >= 0.0)
            && self.max_slides > 0
            && self.plane.is_none_or(Real::is_finite)
            && self.max_push.is_none_or(|p| p.is_finite() && p >= 0.0)
    }
}

/// Characters are external (kinematic) capsules.
pub(super) fn can_be_character(b: &Body) -> bool {
    matches!(b.shape, Shape::Capsule { .. }) && b.external
}

fn is_static(b: &Body) -> bool {
    b.is_static()
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
        if !settings.valid() || !can_be_character(&self.bodies[i]) {
            return Err(Error::InvalidInput);
        }
        self.characters.insert(id, settings);
        Ok(())
    }

    /// Ties character `id` to a fixed world point by `tether` (a grapple hook), replacing any
    /// tether it held: every later move keeps its attachment within the rope's length of the
    /// anchor (see [`Tether`]). Adding the character again or removing its body releases it.
    pub fn tether_character(&mut self, id: BodyId, tether: Tether) -> Result<(), Error> {
        let slot = self.characters.slot(id).ok_or(Error::MissingBody(id))?;
        if !tether.valid() {
            return Err(Error::InvalidInput);
        }
        self.characters.release(slot);
        self.characters.tether[slot] = Some(tether);
        self.characters.tethers += 1;
        Ok(())
    }

    /// Sets the length of character `id`'s tether (reeling in or paying out).
    pub fn set_tether_length(&mut self, id: BodyId, length: Real) -> Result<(), Error> {
        let slot = self.characters.slot(id).ok_or(Error::MissingBody(id))?;
        match &mut self.characters.tether[slot] {
            Some(tether) if valid_length(length) => {
                tether.length = length;
                Ok(())
            }
            _ => Err(Error::InvalidInput),
        }
    }

    /// Releases character `id`'s tether, returning it (`None` without one).
    pub fn release_tether(&mut self, id: BodyId) -> Option<Tether> {
        let slot = self.characters.slot(id)?;
        self.characters.release(slot)
    }

    /// Character `id`'s tether.
    pub fn tether(&self, id: BodyId) -> Option<Tether> {
        self.characters
            .slot(id)
            .and_then(|slot| self.characters.tether[slot])
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
                tether: V::ZERO,
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
            // Unreachable through the public API: every mutator keeps the tree current (#414).
            debug_assert!(false, "query tree stale outside a step");
            crate::work::add(
                crate::work::Counter::FullScanBodies,
                self.characters.ids.len() as u64,
            );
            self.characters.placed_all();
            return;
        }
        let (lo, hi) = self.bodies[i].cached_bounds;
        let mut nearby = std::mem::take(&mut self.characters.nearby);
        nearby.clear();
        let nodes = self.query_tree.visit_moving_aabb(
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
        // A tangential terrain contact can win the body's earliest-hit tie by rounding,
        // hiding the next triangle. Recast leaning off that contact just as a later slide
        // does, while still judging approach against the requested displacement.
        if let Some(normal) = hits.iter().find_map(|hit| {
            (matches!(hit.feature, Some(super::RayFeature::HeightfieldTriangle(_)))
                && hit.normal.dot(displacement).abs() <= tolerance)
                .then_some(hit.normal)
        }) {
            self.cast_hits(
                id,
                pose,
                displacement + normal * (offset * NORMAL_NUDGE),
                hits,
            )?;
        }
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
            tether: V::ZERO,
        };
        let mut push_budget = s.max_push.unwrap_or(0.0);

        // Leave penetration first; reported apart from the requested motion. A few passes
        // resolve overlap with several bodies at once (a pillar against a wall).
        let generation = self.bookkeeping.generation;
        // Clearance cached by a previous move is not an authoritative penetration
        // decision. Terrain motion can leave a small overlap; a reconstructed world
        // must resolve exactly the same physical pose as an uninterrupted one (#500).
        let slot = self.characters.slot(id).ok_or(Error::MissingBody(id))?;
        let verified = self.characters.is_clear(id, generation)
            && self.characters.verified_clear[slot] == Some((pose, self.elapsed));
        let mut resolved = verified;
        if !verified && s.embedded == Embedded::Stay {
            // Left in geometry: clear once no other character overlaps it (the batch
            // separates those).
            self.character_overlap(id, pose, hits)?;
            resolved = !hits
                .iter()
                .any(|h| h.separation < 0.0 && self.characters.contains(h.body));
        }
        let passes = if verified || s.embedded == Embedded::Stay {
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
                if resolved {
                    self.characters.verified_clear[slot] = Some((pose, self.elapsed));
                }
                break;
            }
            pose.position += push;
            result.depenetration += push;
        }

        // A tether takes out what would carry the character past its rope from where it now
        // stands, then the slide resolves the rest as for any move.
        let mut desired = desired;
        if let Some(tether) = self.characters.tether[slot] {
            result.tether = tether.constrain(pose.position, desired, s.offset);
            desired += result.tether;
        }

        // A zero-motion move from a verified clear pose neither slides nor depenetrates: what
        // it finds below depends only on the pose, its exclusion and static geometry, so the
        // last such move's outcome stands while none of them changed (#526).
        let still = (!planar
            && flat(desired).length() <= s.offset * 1e-3
            && result.depenetration == V::ZERO
            && self.characters.verified_clear[slot] == Some((pose, self.elapsed)))
        .then(|| StillMove::key(pose));
        let (exclude, revision) = (self.characters.exclude[slot], self.static_revision);
        if let Some(key) = still
            && let Some(last) = self.characters.still[slot].filter(|last| {
                last.pose == key
                    && last.exclude == exclude
                    && last.generation == generation
                    && last.revision == revision
            })
        {
            result.grounded = last.support.is_some();
            result.support = last.support;
            result.ground_contact = last.ground_contact;
            self.characters
                .set_clear(id, resolved.then_some(generation));
            return Ok(result);
        }

        // Collide and slide, keeping `offset` to every surface.
        let mut translation = V::ZERO;
        let mut remaining = flat(desired);
        let mut support = None;
        // The surface the slide last met: the next cast leans off it by the nudge.
        let mut leaving = V::ZERO;
        let mut crossed_support = false;
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
                // A positive-time grazing triangle contact crosses a support edge.
                // Preserve height through that move, like Rapier's edge slide,
                // instead of turning a rounded tangent into an immediate snap.
                // This does not retain support after the character leaves the edge.
                crossed_support |= hits.iter().any(|hit| {
                    matches!(
                        hit.feature,
                        Some(
                            super::RayFeature::TriangleMeshTriangle(_)
                                | super::RayFeature::HeightfieldTriangle(_)
                        )
                    ) && hit.fraction > 0.0
                        && hit.fraction < 1.0
                        && hit.normal.dot(up) >= min_up
                        && hit.normal.dot(remaining).abs() <= s.offset * 1e-3
                });
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
            if !crossed_support
                && snap > s.offset * 1e-2
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
        // Kept only when it did not snap, so the pose it leaves is the one it is keyed by.
        self.characters.still[slot] =
            still
                .filter(|&key| StillMove::key(pose) == key)
                .map(|pose| StillMove {
                    pose,
                    exclude,
                    generation,
                    revision,
                    support,
                    ground_contact: result.ground_contact,
                });
        // Keep the inter-character separation hint. Static penetration is checked
        // independently from the physical pose on the next move.
        self.characters
            .set_clear(id, resolved.then_some(generation));
        Ok(result)
    }
}

#[cfg(test)]
mod recovery_tests {
    use super::*;
    use crate::approximate::{Config, HeightfieldDesc, Material};
    fn sample() -> (World, BodyId) {
        let mut world = World::new(Config {
            gravity: V::ZERO,
            length_unit: 100.0,
            ..Config::default()
        })
        .unwrap();
        world
            .add_heightfield(
                HeightfieldDesc {
                    origin: V(800.0, 0.0, -7600.0),
                    cell_size: (400.0, 400.0),
                    columns: 2,
                    rows: 2,
                    heights: vec![1100.0, 1100.0, 1287.0, 1150.0],
                    removed_cells: None,
                    material: Material::default(),
                },
                Some(1),
            )
            .unwrap();
        let mut body = Body::keyed(
            2,
            Shape::capsule(60.0, 30.0),
            V(1116.2186, 1247.0002, -7388.426),
            1.0,
        );
        body.external = true;
        let id = world.add_body(body).unwrap();
        world
            .add_character(
                id,
                CharacterSettings {
                    offset: 0.25,
                    ..world.character_settings()
                },
            )
            .unwrap();
        (world, id)
    }
    #[test]
    fn verified_clear_pose_matches_cold_queries_after_placement_exclusion_and_step() {
        let (mut world, id) = sample();
        world
            .set_kinematic_position(id, V(1100.0, 1400.0, -7400.0))
            .unwrap();
        let obstacle = world
            .add_body(Body::keyed(
                3,
                Shape::Box(V(10.0, 10.0, 10.0)),
                V::ZERO,
                0.0,
            ))
            .unwrap();
        let mut out = Vec::new();
        let mut movement = CharacterMove {
            body: id,
            desired: V::ZERO,
            exclude: None,
            autostep: None,
        };
        world.move_characters(&[movement], &mut out).unwrap();
        assert!(world.characters.verified_clear[id.slot() as usize].is_some());
        for change in 0..4 {
            match change {
                0 => world
                    .set_static_position(obstacle, V(1100.0, 1400.0, -7400.0))
                    .unwrap(),
                1 => movement.exclude = Some(obstacle),
                2 => movement.exclude = None,
                _ => {
                    world.step(1.0 / 60.0).unwrap();
                }
            }
            let mut cold = world.clone();
            cold.characters.verified_clear.fill(None);
            let mut expected = Vec::new();
            world.move_characters(&[movement], &mut out).unwrap();
            cold.move_characters(&[movement], &mut expected).unwrap();
            assert_eq!(out, expected, "change {change}");
            assert_eq!(
                world.body(id).unwrap().position,
                cold.body(id).unwrap().position
            );
        }
    }
    #[test]
    fn prior_clearance_hint_cannot_change_continuation_from_the_same_terrain_pose() {
        let (mut warm, id) = sample();
        warm.characters
            .set_clear(id, Some(warm.bookkeeping.generation));
        let (mut fresh, other) = sample();
        let mut left = Vec::new();
        let mut right = Vec::new();
        let desired = V(0.0, -0.2725, 3.0);
        warm.move_characters(
            &[CharacterMove {
                body: id,
                desired,
                exclude: None,
                autostep: None,
            }],
            &mut left,
        )
        .unwrap();
        fresh
            .move_characters(
                &[CharacterMove {
                    body: other,
                    desired,
                    exclude: None,
                    autostep: None,
                }],
                &mut right,
            )
            .unwrap();
        assert_eq!(left, right);
        assert_eq!(
            warm.body(id).unwrap().position,
            fresh.body(other).unwrap().position,
            "warm {left:?}, fresh {right:?}"
        );
    }
}

#[cfg(test)]
mod still_tests {
    use super::*;
    use crate::CollisionLayers3d;
    use crate::approximate::Config;

    const STAND: Real = 0.5 + 0.3 + 0.0025;

    fn capsule(at: V) -> Body {
        let mut body = Body::new(Shape::capsule(0.5, 0.3), at, 1.0);
        body.external = true;
        body
    }

    /// A character standing on a static floor box, and a kinematic box beside it.
    fn sample() -> (World, BodyId, BodyId, BodyId) {
        let mut world = World::new(Config {
            gravity: V::ZERO,
            ..Config::default()
        })
        .unwrap();
        let floor = world
            .add_body(Body::keyed(
                1,
                Shape::Box(V(5.0, 0.5, 5.0)),
                V(0.0, -0.5, 0.0),
                0.0,
            ))
            .unwrap();
        let mut platform = Body::keyed(2, Shape::Box(V(0.5, 0.1, 0.5)), V(3.0, 0.1, 0.0), 1.0);
        platform.external = true;
        let platform = world.add_body(platform).unwrap();
        let id = world.add_body(capsule(V(0.0, STAND, 0.0))).unwrap();
        world.add_character(id, world.character_settings()).unwrap();
        (world, id, floor, platform)
    }

    /// The same move on `world` and on a clone without any zero-motion outcome kept.
    fn compare(world: &mut World, movement: CharacterMove, what: &str) -> CharacterMoveResult {
        let mut cold = world.clone();
        cold.characters.still.fill(None);
        let (mut out, mut expected) = (Vec::new(), Vec::new());
        world.move_characters(&[movement], &mut out).unwrap();
        cold.move_characters(&[movement], &mut expected).unwrap();
        assert_eq!(out, expected, "{what}");
        let id = movement.body;
        assert_eq!(
            world.body(id).unwrap().position,
            cold.body(id).unwrap().position,
            "{what}"
        );
        out[0]
    }

    #[test]
    fn reused_zero_motion_moves_match_a_cold_world() {
        let (mut world, mut id, floor, platform) = sample();
        let still = |body| CharacterMove {
            body,
            desired: V::ZERO,
            exclude: None,
            autostep: None,
        };
        let first = compare(&mut world, still(id), "first");
        assert!(first.grounded && first.support == Some(floor));
        // An unchanged repeat reuses the kept outcome: a poisoned one shows through.
        let slot = id.slot() as usize;
        let kept = world.characters.still[slot].expect("kept");
        world.characters.still[slot] = Some(StillMove {
            support: None,
            ..kept
        });
        let mut out = Vec::new();
        world.move_characters(&[still(id)], &mut out).unwrap();
        assert_eq!(out[0].support, None, "reused");
        world.characters.still[slot] = Some(kept);

        let mut movement = still(id);
        for change in 0..14 {
            let revision = world.static_revision();
            let static_change = match change {
                0 => false,
                1 => {
                    world.set_static_position(floor, V(0.0, -0.8, 0.0)).unwrap();
                    true
                }
                2 => {
                    world.set_static_position(floor, V(0.0, -0.5, 0.0)).unwrap();
                    true
                }
                3 => {
                    world
                        .set_layers(floor, CollisionLayers3d::new(0, 0))
                        .unwrap();
                    true
                }
                4 => {
                    world.set_layers(floor, CollisionLayers3d::ALL).unwrap();
                    true
                }
                5 => {
                    movement.exclude = Some(floor);
                    false
                }
                6 => {
                    movement.exclude = None;
                    false
                }
                7 => {
                    // Under the character's feet, inside the floor: never support.
                    world
                        .set_kinematic_position(platform, V(0.0, -0.2, 0.0))
                        .unwrap();
                    false
                }
                8 => {
                    world
                        .set_next_kinematic_pose(platform, V(3.0, 0.1, 0.0), Default::default())
                        .unwrap();
                    world.step(1.0 / 60.0).unwrap();
                    false
                }
                9 => {
                    world.step(1.0 / 60.0).unwrap();
                    false
                }
                10 => {
                    // A static slab level with the floor's top, earlier in body order: the
                    // support changes.
                    world
                        .add_body(Body::keyed(
                            0,
                            Shape::Box(V(1.0, 0.25, 1.0)),
                            V(0.0, -0.25, 0.0),
                            0.0,
                        ))
                        .unwrap();
                    true
                }
                11 => {
                    let slab = world.body_id(0).unwrap();
                    world.remove_body(slab).unwrap();
                    true
                }
                12 => {
                    // The slot's next character, at the same pose.
                    world.remove_body(id).unwrap();
                    let next = world.add_body(capsule(V(0.0, STAND, 0.0))).unwrap();
                    assert_eq!(next.slot(), id.slot());
                    world
                        .add_character(next, world.character_settings())
                        .unwrap();
                    id = next;
                    movement.body = id;
                    false
                }
                _ => {
                    #[cfg(feature = "checkpoints")]
                    {
                        let context = crate::approximate::CheckpointContext {
                            build: [0; 32],
                            content: [0; 32],
                        };
                        let restored = world.checkpoint(context).unwrap().restore(&[]).unwrap();
                        assert!(restored.characters.still.iter().all(Option::is_none));
                        world = restored;
                    }
                    false
                }
            };
            assert_eq!(
                world.static_revision() > revision,
                static_change,
                "revision after change {change}"
            );
            compare(&mut world, movement, &format!("change {change}"));
            // And once more from the outcome that move kept.
            let result = compare(&mut world, movement, &format!("repeat {change}"));
            match change {
                1 | 3 | 5 => assert!(!result.grounded, "change {change}"),
                10 => assert!(result.grounded && result.support == world.body_id(0)),
                _ => assert!(
                    result.grounded && result.support == Some(floor),
                    "change {change}"
                ),
            }
        }
    }
}
