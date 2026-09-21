use std::{
    cell::RefCell,
    collections::{BTreeMap, BTreeSet},
    error::Error,
    fmt,
};

use crate::{
    ANGULAR_VELOCITY_SCALE, BallisticSphere3d, BodyId, BodyKind, MotionAuthority3d, Orientation3d,
    OrientedBox3d, OrientedBoxError3d, RepeatedRotatingEventConfig3d, RepeatedRotatingEventError3d,
    RepeatedRotatingEventWorkStats3d, RigidBox3d, RigidBoxFreeFlightConfig3d,
    RigidBoxFreeFlightError3d, RotatingContactFrontier3d, RotatingContactResponseError3d,
    RotatingContactSearchConfig3d, RotatingContactSearchHit3d, SampledContactTime3d,
    SolverParticipation3d, Vec3i, obb_contact_seed, oriented_box_vertices,
    sample_rigid_box_free_flight,
};
use crate::{
    ballistic_event::{
        BallisticStepWork3d, BallisticTimelineError3d, advance_ballistic_spheres_full,
        advance_ballistic_spheres_to_time, earliest_ballistic_frontier,
        remaining_after as ballistic_remaining_after, resolve_ballistic_frontier,
    },
    current_contact_query::{BodyCurrentContact3d, GenerationContactCache3d},
    repeated_rotating_events::{
        advance_repeated_rotating_events_with_ballistics,
        advance_repeated_rotating_events_with_broad_phase,
    },
    rotating_broad_phase::{RotatingBroadPhase3d, RotatingBroadPhaseError3d},
    rotating_contact_response::{
        RotatingContactResponseScratch3d,
        resolve_rotating_contact_frontier_with_activity_and_scratch,
    },
};

const MAX_PERSISTENT_TAIL_SLICES: u32 = 1_024;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct RotatingWorldConfig3d {
    pub gravity: Vec3i,
    pub sample_count: u16,
    pub refinement_steps: u8,
    pub solver_passes: u8,
    pub max_events: u16,
}

impl Default for RotatingWorldConfig3d {
    fn default() -> Self {
        Self {
            gravity: Vec3i::new(0, -10, 0),
            sample_count: 32,
            refinement_steps: 4,
            solver_passes: 8,
            max_events: 32,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct RotatingWorldStepStats3d {
    pub body_count: usize,
    /// Bodies admitted to the expensive rigid-contact event/tail pipeline.
    pub solver_body_count: usize,
    /// Bodies excluded from the rigid-contact solver for this step.
    pub solver_bypassed_body_count: usize,
    /// Solid physics-owned dynamics that can actually receive solver mutation.
    pub response_authority_body_count: usize,
    /// Geometrically eligible pairs rejected before CCD because neither participant can receive solver mutation.
    pub response_authority_pair_rejections: u64,
    pub sampled_events: usize,
    pub tail_contacts: usize,
    /// Persistent-tail slices actually executed, including discarded replay work.
    pub tail_slices: u64,
    /// Persistent-tail attempts discarded because an impulse required a finer deterministic slice.
    pub tail_replays: u64,
    /// Conservative current-overlap candidates that reached exact OBB testing in the tail solver.
    pub tail_candidate_pairs: u64,
    /// Current-overlap broad-phase queries issued by the tail solver.
    pub tail_broad_phase_queries: u64,
    /// Tail broad-phase queries that rebuilt the balanced fat-AABB topology.
    pub tail_broad_phase_rebuilds: u64,
    /// Tail broad-phase queries that reused the retained fat-AABB topology.
    pub tail_broad_phase_reuses: u64,
    /// Conservative sampled-event broad-phase queries issued during this step.
    pub broad_phase_queries: u64,
    /// Sampled-event queries that had to rebuild the balanced fat-AABB topology.
    pub broad_phase_rebuilds: u64,
    /// Sampled-event queries that reused the existing fat-AABB topology and exact-filtered its candidate leaves.
    pub broad_phase_reuses: u64,
    pub broad_phase_incremental_updates: u64,
    pub broad_phase_reinserts: u64,
    pub broad_phase_rotations: u64,
    pub broad_phase_partial_queries: u64,
    pub broad_phase_partial_body_updates: u64,
    /// Response body-index scratch rebuilds during this step. Stable solver membership should normally report zero after warm-up.
    pub response_scratch_index_rebuilds: u64,
    pub event_response_passes: u64,
    pub stabilization_passes: u64,
    pub stabilizations_hitting_limit: u64,
    pub stabilization_candidate_pairs: u64,
    pub stabilization_exact_contacts: u64,
    pub stabilization_active_bodies: u64,
    /// Active dynamic bodies examined as sources for parked-body wake discovery.
    pub parked_wake_source_body_checks: u64,
    /// Retained parked-body spatial-index queries issued before the strict solver step.
    pub parked_wake_queries: u64,
    /// BVH nodes visited by parked-body wake queries. This should scale with the local query, not world size.
    pub parked_wake_index_nodes_visited: u64,
    /// Parked bodies whose exact stationary bounds overlapped an awake body's conservative sweep.
    pub parked_wake_candidates: u64,
    /// Rotation-invariant spheres currently owned by the analytic projectile lane.
    pub ballistic_sphere_count: usize,
    pub ballistic_query_rounds: u64,
    pub ballistic_target_bound_checks: u64,
    pub ballistic_broad_phase_candidates: u64,
    pub ballistic_toi_tests: u64,
    pub ballistic_feature_tests: u64,
    pub ballistic_motion_samples: u64,
    pub ballistic_impacts: u64,
    pub ballistic_retired: u64,
}

#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct RotatingWorldStepReport3d {
    /// Stable `BodyId`-ordered bodies whose observable rigid-body or sleep state changed this step.
    ///
    /// Higher integration layers consume this precise delta instead of cloning every dynamic body.
    pub changed_body_ids: Vec<BodyId>,
    pub stats: RotatingWorldStepStats3d,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
struct TailStepStats3d {
    contacts: usize,
    slices: u64,
    replays: u64,
    candidate_pairs: u64,
}

#[derive(Clone, Debug, Default)]
struct TailSliceResult3d {
    contact_count: usize,
    reusable_contacts: Option<Vec<RotatingContactSearchHit3d>>,
}

#[derive(Debug)]
struct TailMutationJournal3d {
    originals: Vec<(usize, RigidBox3d)>,
    recorded: Vec<bool>,
}

impl TailMutationJournal3d {
    fn new(body_count: usize) -> Self {
        Self {
            originals: Vec::new(),
            recorded: vec![false; body_count],
        }
    }

    fn record(&mut self, world_index: usize, original: &RigidBox3d) {
        if self.recorded[world_index] {
            return;
        }
        self.recorded[world_index] = true;
        self.originals.push((world_index, original.clone()));
    }

    fn rollback(&mut self, boxes: &mut [RigidBox3d]) {
        for (world_index, original) in self.originals.drain(..) {
            boxes[world_index] = original;
            self.recorded[world_index] = false;
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RotatingWorldError3d {
    DuplicateBody(BodyId),
    MissingBody(BodyId),
    FixedBodyVelocity(BodyId),
    FixedBodyOrientation(BodyId),
    NegativeTimestepNumerator(i32),
    NonPositiveTimestepDenominator(i32),
    PersistentTailResolutionLimit(u32),
    PersistentTailMotionUnsafe(BodyId),
    PersistentTailArithmeticOverflow(BodyId),
    Repeated(RepeatedRotatingEventError3d),
    FreeFlight(RigidBoxFreeFlightError3d),
    Contact(OrientedBoxError3d),
    Response(RotatingContactResponseError3d),
    Ballistic(BallisticTimelineError3d),
}

impl fmt::Display for RotatingWorldError3d {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::DuplicateBody(id) => write!(formatter, "duplicate rotating body {}", id.0),
            Self::MissingBody(id) => write!(formatter, "rotating body {} does not exist", id.0),
            Self::FixedBodyVelocity(id) => write!(
                formatter,
                "fixed rotating body {} cannot receive linear velocity",
                id.0
            ),
            Self::FixedBodyOrientation(id) => write!(
                formatter,
                "fixed rotating body {} cannot receive a runtime orientation update",
                id.0
            ),
            Self::NegativeTimestepNumerator(value) => write!(
                formatter,
                "rotating world timestep numerator must be non-negative, got {value}"
            ),
            Self::NonPositiveTimestepDenominator(value) => write!(
                formatter,
                "rotating world timestep denominator must be positive, got {value}"
            ),
            Self::PersistentTailResolutionLimit(limit) => write!(
                formatter,
                "persistent rotating contact tail needs more than {limit} deterministic slices"
            ),
            Self::PersistentTailMotionUnsafe(id) => write!(
                formatter,
                "persistent rotating contact motion became unsafe for body {} at the maximum tail resolution",
                id.0
            ),
            Self::PersistentTailArithmeticOverflow(id) => write!(
                formatter,
                "persistent rotating contact motion bound overflowed for body {}",
                id.0
            ),
            Self::Repeated(error) => write!(formatter, "rotating event advance failed: {error}"),
            Self::FreeFlight(error) => {
                write!(formatter, "rotating tail free flight failed: {error}")
            }
            Self::Contact(error) => {
                write!(formatter, "rotating tail contact query failed: {error}")
            }
            Self::Response(error) => write!(formatter, "rotating tail response failed: {error}"),
            Self::Ballistic(error) => write!(formatter, "rotating ballistic event failed: {error}"),
        }
    }
}

impl Error for RotatingWorldError3d {}

impl From<RepeatedRotatingEventError3d> for RotatingWorldError3d {
    fn from(value: RepeatedRotatingEventError3d) -> Self {
        Self::Repeated(value)
    }
}

impl From<RigidBoxFreeFlightError3d> for RotatingWorldError3d {
    fn from(value: RigidBoxFreeFlightError3d) -> Self {
        Self::FreeFlight(value)
    }
}

impl From<OrientedBoxError3d> for RotatingWorldError3d {
    fn from(value: OrientedBoxError3d) -> Self {
        Self::Contact(value)
    }
}

impl From<RotatingContactResponseError3d> for RotatingWorldError3d {
    fn from(value: RotatingContactResponseError3d) -> Self {
        Self::Response(value)
    }
}

impl From<BallisticTimelineError3d> for RotatingWorldError3d {
    fn from(value: BallisticTimelineError3d) -> Self {
        Self::Ballistic(value)
    }
}

/// Deterministic rotating-cuboid world built from the engine's sampled OBB event pipeline.
///
/// Collision discovery remains explicitly sampled rotational handling rather than analytic rotational
/// CCD. The event pipeline resolves every admitted impact first. A contact-free exact tail still uses
/// one direct free-flight sample. When the tail begins with persistent contact, the remaining rational
/// time is instead consumed through bounded deterministic slices. Bodies participating in the current
/// persistent frontier are limited to less than their narrowest full thickness of conservative
/// center-plus-rotational motion before the next OBB stabilization pass, so a time-zero contact cannot
/// be free-flown completely through before the next constraint solve. Tail slicing scales the canonical
/// exact ratio directly, so repeated-event precision is preserved without forcing the tail back into a
/// narrower integer pair. Each tail slice advances the same authoritative working buffer in place: fixed
/// bodies are not sampled because free flight cannot change them, while dynamic bodies are written back
/// only when their sampled state differs. If an impulse makes the selected resolution too coarse, the
/// exact tail is replayed from its starting state with a finer deterministic resolution. Replay rollback
/// journals only bodies actually changed by the discarded attempt instead of cloning the complete world
/// at every retry; exceeding the hard bound still fails closed.
///
/// The world retains independent fat-AABB broad-phase trees for sampled sweeps and persistent-tail
/// current-overlap queries. Exact conservative bounds remain candidate truth in both paths; retained tree
/// topology only prunes exact OBB work and cannot change collision truth.
#[derive(Clone, Debug, Default)]
struct SolverPartitions3d {
    solid_body_ids: BTreeSet<BodyId>,
    dynamic_body_ids: BTreeSet<BodyId>,
    bypassed_dynamic_ids: BTreeSet<BodyId>,
    response_authority_body_ids: BTreeSet<BodyId>,
}

impl SolverPartitions3d {
    fn insert(&mut self, rigid_box: &RigidBox3d) {
        let id = rigid_box.body().id();
        if rigid_box.solver_participation() == SolverParticipation3d::Solid {
            self.solid_body_ids.insert(id);
        } else if rigid_box.body().kind() == BodyKind::Dynamic {
            self.bypassed_dynamic_ids.insert(id);
        }
        if rigid_box.body().kind() == BodyKind::Dynamic {
            self.dynamic_body_ids.insert(id);
        }
        if receives_solver_response(rigid_box) {
            self.response_authority_body_ids.insert(id);
        }
    }

    fn remove(&mut self, id: BodyId) {
        self.solid_body_ids.remove(&id);
        self.dynamic_body_ids.remove(&id);
        self.bypassed_dynamic_ids.remove(&id);
        self.response_authority_body_ids.remove(&id);
    }
}

#[derive(Clone, Debug)]
pub struct RotatingWorld3d {
    config: RotatingWorldConfig3d,
    boxes: BTreeMap<BodyId, RigidBox3d>,
    ballistic_spheres: Vec<BallisticSphere3d>,
    ballistic_retire_on_contact: BTreeSet<BodyId>,
    contact_geometry_generation: u64,
    current_contact_cache: RefCell<GenerationContactCache3d>,
    solver_partitions: SolverPartitions3d,
    broad_phase: RotatingBroadPhase3d,
    tail_broad_phase: RotatingBroadPhase3d,
    response_scratch: RotatingContactResponseScratch3d,
}

impl RotatingWorld3d {
    #[must_use]
    pub fn new(config: RotatingWorldConfig3d) -> Self {
        Self {
            config,
            boxes: BTreeMap::new(),
            ballistic_spheres: Vec::new(),
            ballistic_retire_on_contact: BTreeSet::new(),
            contact_geometry_generation: 0,
            current_contact_cache: RefCell::new(GenerationContactCache3d::default()),
            solver_partitions: SolverPartitions3d::default(),
            broad_phase: RotatingBroadPhase3d::default(),
            tail_broad_phase: RotatingBroadPhase3d::default(),
            response_scratch: RotatingContactResponseScratch3d::default(),
        }
    }

    #[must_use]
    pub const fn config(&self) -> RotatingWorldConfig3d {
        self.config
    }

    pub fn add_box(&mut self, rigid_box: RigidBox3d) -> Result<(), RotatingWorldError3d> {
        let id = rigid_box.body().id();
        if self.boxes.contains_key(&id)
            || self
                .ballistic_spheres
                .iter()
                .any(|projectile| projectile.id() == id)
        {
            return Err(RotatingWorldError3d::DuplicateBody(id));
        }
        self.solver_partitions.insert(&rigid_box);
        self.boxes.insert(id, rigid_box);
        self.mark_contact_membership_changed();
        Ok(())
    }

    pub fn remove_box(&mut self, id: BodyId) -> Option<RigidBox3d> {
        let removed = self.boxes.remove(&id);
        if removed.is_some() {
            self.solver_partitions.remove(id);
            self.mark_contact_membership_changed();
        }
        removed
    }

    #[must_use]
    pub fn box_by_id(&self, id: BodyId) -> Option<&RigidBox3d> {
        self.boxes.get(&id)
    }

    pub fn boxes(&self) -> impl Iterator<Item = &RigidBox3d> {
        self.boxes.values()
    }

    pub fn add_ballistic_sphere(
        &mut self,
        projectile: BallisticSphere3d,
        retire_on_contact: bool,
    ) -> Result<(), RotatingWorldError3d> {
        let id = projectile.id();
        if self.boxes.contains_key(&id)
            || self
                .ballistic_spheres
                .iter()
                .any(|existing| existing.id() == id)
        {
            return Err(RotatingWorldError3d::DuplicateBody(id));
        }
        self.ballistic_spheres.push(projectile);
        self.ballistic_spheres
            .sort_by_key(|projectile| projectile.id());
        if retire_on_contact {
            self.ballistic_retire_on_contact.insert(id);
        }
        Ok(())
    }

    pub fn remove_ballistic_sphere(&mut self, id: BodyId) -> Option<BallisticSphere3d> {
        let index = self
            .ballistic_spheres
            .iter()
            .position(|projectile| projectile.id() == id)?;
        self.ballistic_retire_on_contact.remove(&id);
        Some(self.ballistic_spheres.remove(index))
    }

    #[must_use]
    pub fn ballistic_sphere_by_id(&self, id: BodyId) -> Option<&BallisticSphere3d> {
        self.ballistic_spheres
            .iter()
            .find(|projectile| projectile.id() == id)
    }

    pub fn ballistic_spheres(&self) -> impl Iterator<Item = &BallisticSphere3d> {
        self.ballistic_spheres.iter()
    }

    #[must_use]
    pub fn ballistic_sphere_count(&self) -> usize {
        self.ballistic_spheres.len()
    }

    /// Returns exact current contacts for one known body without discovering that body from query geometry.
    ///
    /// A one-off query uses the precise subject path. Repeated queries within the same contact-geometry
    /// generation reuse the cached subject result, and an already-built graph is shared when available.
    pub fn body_contacts(
        &self,
        body: BodyId,
    ) -> Result<Vec<BodyCurrentContact3d>, RotatingWorldError3d> {
        self.current_contact_cache.borrow_mut().body_contacts(
            &self.boxes,
            body,
            self.contact_geometry_generation,
        )
    }

    /// Replaces one dynamic body's linear velocity while preserving its rotational state.
    ///
    /// This is intended for controlled bodies and external impulses that are already expressed in the
    /// engine's canonical velocity units. Fixed bodies reject non-zero velocity.
    pub fn set_linear_velocity(
        &mut self,
        id: BodyId,
        velocity: Vec3i,
    ) -> Result<(), RotatingWorldError3d> {
        let rigid_box = self
            .boxes
            .get_mut(&id)
            .ok_or(RotatingWorldError3d::MissingBody(id))?;
        if rigid_box.body.kind() == BodyKind::Fixed {
            if velocity != Vec3i::ZERO {
                return Err(RotatingWorldError3d::FixedBodyVelocity(id));
            }
            return Ok(());
        }
        rigid_box.body.velocity = velocity;
        Ok(())
    }

    /// Applies a batch of dynamic-body orientation deltas and invalidates contact geometry once.
    ///
    /// This is intended for constrained directional bodies such as arrows whose direction follows
    /// authoritative linear velocity without participating in full angular integration. The complete batch
    /// is validated before mutation, so a missing/fixed body or invalid quaternion leaves the world unchanged.
    pub fn set_orientations(
        &mut self,
        updates: &[(BodyId, Orientation3d)],
    ) -> Result<(), RotatingWorldError3d> {
        let mut normalized = Vec::with_capacity(updates.len());
        let mut ids = BTreeSet::new();
        for (id, orientation) in updates.iter().copied() {
            if !ids.insert(id) {
                return Err(RotatingWorldError3d::DuplicateBody(id));
            }
            let rigid_box = self
                .boxes
                .get(&id)
                .ok_or(RotatingWorldError3d::MissingBody(id))?;
            if rigid_box.body.kind() == BodyKind::Fixed {
                return Err(RotatingWorldError3d::FixedBodyOrientation(id));
            }
            let orientation = orientation.normalized().map_err(|error| {
                RotatingWorldError3d::Contact(OrientedBoxError3d::Angular(error))
            })?;
            normalized.push((id, orientation));
        }

        let mut changed = BTreeSet::new();
        for (id, orientation) in normalized {
            let rigid_box = self
                .boxes
                .get_mut(&id)
                .expect("orientation batch was validated before mutation");
            if rigid_box.angular.orientation != orientation {
                rigid_box.angular.orientation = orientation;
                changed.insert(id);
            }
        }
        if !changed.is_empty() {
            self.mark_contact_geometry_changed_for(&changed);
        }
        Ok(())
    }

    /// Returns stable `BodyId`-ordered OBB overlaps for an arbitrary oriented query box.
    ///
    /// When the query is exactly one current dynamic body's OBB, the engine reuses the deterministic
    /// zero-time current-contact graph. Arbitrary query geometry and fixed-body queries keep the existing
    /// direct exact scan, so the public query contract is unchanged.
    pub fn overlap_query(&self, query: OrientedBox3d) -> Result<Vec<BodyId>, RotatingWorldError3d> {
        oriented_box_vertices(query)?;
        if let Some(rigid_box) = self.boxes.values().find(|rigid_box| {
            rigid_box.body().kind() == BodyKind::Dynamic && rigid_box.oriented_box() == query
        }) {
            let body = rigid_box.body().id();
            if rigid_box.solver_participation() == SolverParticipation3d::Solid {
                return self.current_contact_cache.borrow_mut().overlap_ids(
                    &self.boxes,
                    body,
                    self.contact_geometry_generation,
                );
            }
            let mut hits = self.body_overlaps(body)?;
            hits.push(body);
            hits.sort_unstable();
            return Ok(hits);
        }

        let mut hits = Vec::new();
        for (id, rigid_box) in &self.boxes {
            if obb_contact_seed(query, rigid_box.oriented_box())?.is_some() {
                hits.push(*id);
            }
        }
        Ok(hits)
    }

    /// Returns stable BodyId-ordered overlaps for one known body without treating those overlaps as solver
    /// contacts. Collision layers still define interaction eligibility, making this the intended query for
    /// overlap-only sensors and triggers.
    pub fn body_overlaps(&self, body: BodyId) -> Result<Vec<BodyId>, RotatingWorldError3d> {
        let subject = self
            .boxes
            .get(&body)
            .ok_or(RotatingWorldError3d::MissingBody(body))?;
        oriented_box_vertices(subject.oriented_box())?;
        let mut hits = Vec::new();
        for (other_id, other) in &self.boxes {
            if *other_id == body
                || !subject
                    .collision_layers()
                    .collides_with(other.collision_layers())
            {
                continue;
            }
            if obb_contact_seed(subject.oriented_box(), other.oriented_box())?.is_some() {
                hits.push(*other_id);
            }
        }
        Ok(hits)
    }

    pub fn step(
        &mut self,
        timestep_numerator: i32,
        timestep_denominator: i32,
    ) -> Result<RotatingWorldStepReport3d, RotatingWorldError3d> {
        if timestep_numerator < 0 {
            return Err(RotatingWorldError3d::NegativeTimestepNumerator(
                timestep_numerator,
            ));
        }
        if timestep_denominator <= 0 {
            return Err(RotatingWorldError3d::NonPositiveTimestepDenominator(
                timestep_denominator,
            ));
        }

        let total_body_count = self
            .boxes
            .len()
            .saturating_add(self.ballistic_spheres.len());
        if timestep_numerator == 0 {
            let response_authority_body_count =
                self.solver_partitions.response_authority_body_ids.len();
            let solver_body_count = if response_authority_body_count == 0 {
                0
            } else {
                self.solver_partitions.solid_body_ids.len()
            };
            return Ok(RotatingWorldStepReport3d {
                changed_body_ids: Vec::new(),
                stats: RotatingWorldStepStats3d {
                    body_count: total_body_count,
                    solver_body_count,
                    solver_bypassed_body_count: total_body_count.saturating_sub(solver_body_count),
                    response_authority_body_count,
                    ballistic_sphere_count: self.ballistic_spheres.len(),
                    ..RotatingWorldStepStats3d::default()
                },
            });
        }

        let broad_phase_before = self.broad_phase.stats();
        let tail_broad_phase_before = self.tail_broad_phase.stats();
        let free_flight = RigidBoxFreeFlightConfig3d::new(
            self.config.gravity,
            timestep_numerator,
            timestep_denominator,
        );

        // Ballistic spheres never enter the sampled rotating-body solver. When one is live, however, every
        // solid rigid body remains available as an analytic target even if there is otherwise no mutable rigid
        // response authority in the world.
        let response_authority_body_count =
            self.solver_partitions.response_authority_body_ids.len();
        let mixed_ballistic_step = !self.ballistic_spheres.is_empty();
        let mut solver_boxes = Vec::with_capacity(self.solver_partitions.solid_body_ids.len());
        let mut bypassed_updates = Vec::new();

        if response_authority_body_count > 0 || mixed_ballistic_step {
            for id in &self.solver_partitions.solid_body_ids {
                let rigid_box = self
                    .boxes
                    .get(id)
                    .ok_or(RotatingWorldError3d::MissingBody(*id))?;
                solver_boxes.push(rigid_box.clone());
            }
            for id in &self.solver_partitions.bypassed_dynamic_ids {
                let rigid_box = self
                    .boxes
                    .get(id)
                    .ok_or(RotatingWorldError3d::MissingBody(*id))?;
                let next = sample_rigid_box_free_flight(rigid_box, free_flight, 1, 1)?;
                if next != *rigid_box {
                    bypassed_updates.push(next);
                }
            }
        } else {
            for id in &self.solver_partitions.dynamic_body_ids {
                let rigid_box = self
                    .boxes
                    .get(id)
                    .ok_or(RotatingWorldError3d::MissingBody(*id))?;
                let next = sample_rigid_box_free_flight(rigid_box, free_flight, 1, 1)?;
                if next != *rigid_box {
                    bypassed_updates.push(next);
                }
            }
        }

        let solver_body_count = solver_boxes.len();
        let solver_bypassed_body_count = total_body_count.saturating_sub(solver_body_count);
        let mut ballistic_work = BallisticStepWork3d::default();
        let mut resolved_ballistic_pairs = BTreeSet::new();

        let (solver_boxes, sampled_events, tail, work) = if mixed_ballistic_step {
            if solver_boxes.is_empty() {
                advance_ballistic_spheres_full(
                    &mut self.ballistic_spheres,
                    free_flight,
                    &mut ballistic_work,
                )?;
                (
                    solver_boxes,
                    0,
                    TailStepStats3d::default(),
                    RepeatedRotatingEventWorkStats3d::default(),
                )
            } else {
                let advance = advance_repeated_rotating_events_with_ballistics(
                    &mut solver_boxes,
                    &mut self.ballistic_spheres,
                    &self.ballistic_retire_on_contact,
                    RepeatedRotatingEventConfig3d::new(
                        RotatingContactSearchConfig3d::new(
                            free_flight,
                            self.config.sample_count,
                            self.config.refinement_steps,
                        ),
                        self.config.solver_passes,
                        self.config.max_events,
                    ),
                    &mut self.broad_phase,
                    &mut self.response_scratch,
                    &mut resolved_ballistic_pairs,
                    &mut ballistic_work,
                )?;
                let sampled_events = advance.events.len();
                let work = advance.work;
                let (solver_boxes, tail) = if advance.remaining.timestep_is_zero() {
                    (solver_boxes, TailStepStats3d::default())
                } else {
                    consume_tail_with_ballistics(
                        solver_boxes,
                        &mut self.ballistic_spheres,
                        &self.ballistic_retire_on_contact,
                        advance.remaining,
                        self.config.solver_passes,
                        &mut self.tail_broad_phase,
                        &mut self.response_scratch,
                        &mut resolved_ballistic_pairs,
                        &mut ballistic_work,
                    )?
                };
                (solver_boxes, sampled_events, tail, work)
            }
        } else if solver_boxes.is_empty() {
            (
                solver_boxes,
                0,
                TailStepStats3d::default(),
                RepeatedRotatingEventWorkStats3d::default(),
            )
        } else {
            let advance = advance_repeated_rotating_events_with_broad_phase(
                &mut solver_boxes,
                RepeatedRotatingEventConfig3d::new(
                    RotatingContactSearchConfig3d::new(
                        free_flight,
                        self.config.sample_count,
                        self.config.refinement_steps,
                    ),
                    self.config.solver_passes,
                    self.config.max_events,
                ),
                &mut self.broad_phase,
                &mut self.response_scratch,
            )?;
            let sampled_events = advance.events.len();
            let work = advance.work;
            let (solver_boxes, tail) = if advance.remaining.timestep_is_zero() {
                (solver_boxes, TailStepStats3d::default())
            } else {
                consume_tail(
                    solver_boxes,
                    advance.remaining,
                    self.config.solver_passes,
                    &mut self.tail_broad_phase,
                    &mut self.response_scratch,
                )?
            };
            (solver_boxes, sampled_events, tail, work)
        };

        let live_ballistic_ids = self
            .ballistic_spheres
            .iter()
            .map(|projectile| projectile.id())
            .collect::<BTreeSet<_>>();
        self.ballistic_retire_on_contact
            .retain(|id| live_ballistic_ids.contains(id));

        let mut changed_body_ids = BTreeSet::new();
        for rigid_box in solver_boxes.into_iter().chain(bypassed_updates) {
            let id = rigid_box.body().id();
            if self.boxes.get(&id) == Some(&rigid_box) {
                continue;
            }
            self.boxes.insert(id, rigid_box);
            changed_body_ids.insert(id);
        }

        if !changed_body_ids.is_empty() {
            self.mark_contact_geometry_changed_for(&changed_body_ids);
        }

        let broad_phase_after = self.broad_phase.stats();
        let tail_broad_phase_after = self.tail_broad_phase.stats();
        Ok(RotatingWorldStepReport3d {
            changed_body_ids: changed_body_ids.into_iter().collect(),
            stats: RotatingWorldStepStats3d {
                body_count: self
                    .boxes
                    .len()
                    .saturating_add(self.ballistic_spheres.len()),
                solver_body_count,
                solver_bypassed_body_count,
                response_authority_body_count,
                response_authority_pair_rejections: broad_phase_after
                    .response_authority_pair_rejections
                    .saturating_sub(broad_phase_before.response_authority_pair_rejections)
                    .saturating_add(
                        tail_broad_phase_after
                            .response_authority_pair_rejections
                            .saturating_sub(
                                tail_broad_phase_before.response_authority_pair_rejections,
                            ),
                    ),
                sampled_events,
                tail_contacts: tail.contacts,
                tail_slices: tail.slices,
                tail_replays: tail.replays,
                tail_candidate_pairs: tail.candidate_pairs,
                tail_broad_phase_queries: tail_broad_phase_after
                    .queries
                    .saturating_sub(tail_broad_phase_before.queries),
                tail_broad_phase_rebuilds: tail_broad_phase_after
                    .rebuilds
                    .saturating_sub(tail_broad_phase_before.rebuilds),
                tail_broad_phase_reuses: tail_broad_phase_after
                    .reuses
                    .saturating_sub(tail_broad_phase_before.reuses),
                broad_phase_queries: broad_phase_after
                    .queries
                    .saturating_sub(broad_phase_before.queries),
                broad_phase_rebuilds: broad_phase_after
                    .rebuilds
                    .saturating_sub(broad_phase_before.rebuilds),
                broad_phase_reuses: broad_phase_after
                    .reuses
                    .saturating_sub(broad_phase_before.reuses),
                broad_phase_incremental_updates: broad_phase_after
                    .incremental_updates
                    .saturating_sub(broad_phase_before.incremental_updates),
                broad_phase_reinserts: broad_phase_after
                    .reinserts
                    .saturating_sub(broad_phase_before.reinserts),
                broad_phase_rotations: broad_phase_after
                    .rotations
                    .saturating_sub(broad_phase_before.rotations),
                broad_phase_partial_queries: broad_phase_after
                    .partial_queries
                    .saturating_sub(broad_phase_before.partial_queries),
                broad_phase_partial_body_updates: broad_phase_after
                    .partial_body_updates
                    .saturating_sub(broad_phase_before.partial_body_updates),
                response_scratch_index_rebuilds: work.response_scratch_index_rebuilds,
                event_response_passes: work.event_response_passes,
                stabilization_passes: work.stabilization_passes,
                stabilizations_hitting_limit: work.stabilizations_hitting_limit,
                stabilization_candidate_pairs: work.stabilization_candidate_pairs,
                stabilization_exact_contacts: work.stabilization_exact_contacts,
                stabilization_active_bodies: work.stabilization_active_bodies,
                parked_wake_source_body_checks: 0,
                parked_wake_queries: 0,
                parked_wake_index_nodes_visited: 0,
                parked_wake_candidates: 0,
                ballistic_sphere_count: self.ballistic_spheres.len(),
                ballistic_query_rounds: ballistic_work.query_rounds,
                ballistic_target_bound_checks: ballistic_work.target_bound_checks,
                ballistic_broad_phase_candidates: ballistic_work.broad_phase_candidates,
                ballistic_toi_tests: ballistic_work.toi_tests,
                ballistic_feature_tests: ballistic_work.feature_tests,
                ballistic_motion_samples: ballistic_work.motion_samples,
                ballistic_impacts: ballistic_work.impacts,
                ballistic_retired: ballistic_work.retired,
            },
        })
    }

    fn next_contact_geometry_generation(&mut self) -> u64 {
        if let Some(next) = self.contact_geometry_generation.checked_add(1) {
            self.contact_geometry_generation = next;
            return next;
        }

        self.contact_geometry_generation = 0;
        self.current_contact_cache = RefCell::new(GenerationContactCache3d::default());
        0
    }

    fn mark_contact_membership_changed(&mut self) {
        let generation = self.next_contact_geometry_generation();
        self.current_contact_cache
            .borrow_mut()
            .note_membership_generation(generation);
    }

    fn mark_contact_geometry_changed_for(&mut self, changed: &BTreeSet<BodyId>) {
        let generation = self.next_contact_geometry_generation();
        self.current_contact_cache
            .borrow_mut()
            .note_changed_generation(generation, changed.iter().copied());
    }

    #[cfg(test)]
    pub(crate) fn current_contact_cache_stats(&self) -> (u64, u64, u64, u64) {
        self.current_contact_cache.borrow().stats()
    }

    #[cfg(test)]
    pub(crate) fn current_contact_incremental_stats(&self) -> (u64, u64, u64) {
        self.current_contact_cache.borrow().incremental_stats()
    }
}

fn receives_solver_response(rigid_box: &RigidBox3d) -> bool {
    rigid_box.solver_participation() == SolverParticipation3d::Solid
        && rigid_box.body().kind() == BodyKind::Dynamic
        && rigid_box.motion_authority() == MotionAuthority3d::Physics
}

#[allow(clippy::too_many_arguments)]
fn consume_tail_with_ballistics(
    boxes: Vec<RigidBox3d>,
    projectiles: &mut Vec<BallisticSphere3d>,
    retire_on_contact: &BTreeSet<BodyId>,
    remaining: RigidBoxFreeFlightConfig3d,
    solver_passes: u8,
    broad_phase: &mut RotatingBroadPhase3d,
    response_scratch: &mut RotatingContactResponseScratch3d,
    resolved_ballistic_pairs: &mut BTreeSet<(BodyId, BodyId)>,
    ballistic_work: &mut BallisticStepWork3d,
) -> Result<(Vec<RigidBox3d>, TailStepStats3d), RotatingWorldError3d> {
    let mut stats = TailStepStats3d::default();
    let mut current = boxes;
    let initial_contacts = contact_frontier(&current, broad_phase, &mut stats)?;
    if initial_contacts.is_empty() {
        advance_ballistic_spheres_full(projectiles, remaining, ballistic_work)?;
        let result = free_flight_and_stabilize_in_place(
            &mut current,
            remaining,
            solver_passes,
            broad_phase,
            response_scratch,
            &mut stats,
            None,
        )?;
        stats.contacts = result.contact_count;
        return Ok((current, stats));
    }

    let mut slice_count = persistent_tail_slice_count(&current, remaining, &initial_contacts)?;
    let mut journal = TailMutationJournal3d::new(current.len());
    let mut reusable_contacts = Some(initial_contacts.clone());
    loop {
        let slice_config = tail_slice_config(remaining, slice_count)?;
        let projectile_start = projectiles.clone();
        let resolved_pairs_start = resolved_ballistic_pairs.clone();
        let mut contact_count = 0_usize;
        let mut unsafe_body = None;

        for _ in 0..slice_count {
            stats.slices = stats.slices.saturating_add(1);
            let current_contacts = match reusable_contacts.take() {
                Some(contacts) => contacts,
                None => contact_frontier(&current, broad_phase, &mut stats)?,
            };
            let (result, unsafe_id) = advance_tail_slice_with_ballistics(
                &mut current,
                projectiles,
                retire_on_contact,
                slice_config,
                solver_passes,
                broad_phase,
                response_scratch,
                &mut stats,
                &mut journal,
                resolved_ballistic_pairs,
                ballistic_work,
                current_contacts,
            )?;
            contact_count = contact_count.saturating_add(result.contact_count);
            reusable_contacts = result.reusable_contacts;
            if unsafe_id.is_some() {
                unsafe_body = unsafe_id;
                break;
            }
        }

        if unsafe_body.is_none() {
            stats.contacts = contact_count;
            return Ok((current, stats));
        }

        stats.replays = stats.replays.saturating_add(1);
        let target = slice_count
            .saturating_mul(2)
            .min(MAX_PERSISTENT_TAIL_SLICES);
        if target <= slice_count {
            return Err(RotatingWorldError3d::PersistentTailMotionUnsafe(
                unsafe_body.expect("unsafe tail body was observed"),
            ));
        }
        let next_slice_count = next_representable_tail_slice_count(remaining, target)?;
        journal.rollback(&mut current);
        *projectiles = projectile_start;
        *resolved_ballistic_pairs = resolved_pairs_start;
        reusable_contacts = Some(initial_contacts.clone());
        slice_count = next_slice_count;
    }
}

#[allow(clippy::too_many_arguments)]
fn advance_tail_slice_with_ballistics(
    boxes: &mut [RigidBox3d],
    projectiles: &mut Vec<BallisticSphere3d>,
    retire_on_contact: &BTreeSet<BodyId>,
    mut remaining: RigidBoxFreeFlightConfig3d,
    solver_passes: u8,
    broad_phase: &mut RotatingBroadPhase3d,
    response_scratch: &mut RotatingContactResponseScratch3d,
    stats: &mut TailStepStats3d,
    journal: &mut TailMutationJournal3d,
    resolved_ballistic_pairs: &mut BTreeSet<(BodyId, BodyId)>,
    ballistic_work: &mut BallisticStepWork3d,
    initial_contacts: Vec<RotatingContactSearchHit3d>,
) -> Result<(TailSliceResult3d, Option<BodyId>), RotatingWorldError3d> {
    let mut contact_count = 0_usize;
    let mut reusable_contacts = Some(initial_contacts);

    while !remaining.timestep_is_zero() {
        let current_contacts = match reusable_contacts.take() {
            Some(contacts) => contacts,
            None => contact_frontier(boxes, broad_phase, stats)?,
        };
        if let Some(id) = first_unsafe_tail_body(boxes, remaining, &current_contacts)? {
            return Ok((
                TailSliceResult3d {
                    contact_count,
                    reusable_contacts: Some(current_contacts),
                },
                Some(id),
            ));
        }

        let Some(frontier) = earliest_ballistic_frontier(
            boxes,
            projectiles,
            remaining,
            resolved_ballistic_pairs,
            ballistic_work,
        )?
        else {
            advance_ballistic_spheres_full(projectiles, remaining, ballistic_work)?;
            let result = free_flight_and_stabilize_in_place(
                boxes,
                remaining,
                solver_passes,
                broad_phase,
                response_scratch,
                stats,
                Some(journal),
            )?;
            contact_count = contact_count.saturating_add(result.contact_count);
            return Ok((
                TailSliceResult3d {
                    contact_count,
                    reusable_contacts: result.reusable_contacts,
                },
                None,
            ));
        };

        let segment =
            remaining.scaled_fraction(frontier.time.numerator, frontier.time.denominator)?;
        advance_tail_free_flight_in_place(boxes, segment, Some(journal))?;
        advance_ballistic_spheres_to_time(projectiles, remaining, frontier.time, ballistic_work)?;

        // Persistent rigid contacts are authoritative at the admitted sphere time. Resolve them before
        // applying the projectile impulse, then retain their exact contact evidence only if response did
        // not move geometry.
        let pre_impact = stabilize_tail_contacts_in_place(
            boxes,
            solver_passes,
            broad_phase,
            response_scratch,
            stats,
            Some(journal),
        )?;
        contact_count = contact_count.saturating_add(pre_impact.contact_count);

        for candidate in &frontier.hits {
            if let Some(world_index) = boxes
                .iter()
                .position(|rigid_box| rigid_box.body().id() == candidate.hit.body)
                && boxes[world_index].body().kind() == BodyKind::Dynamic
            {
                journal.record(world_index, &boxes[world_index]);
            }
        }
        resolve_ballistic_frontier(
            boxes,
            projectiles,
            retire_on_contact,
            &frontier,
            ballistic_work,
        )?;
        for candidate in &frontier.hits {
            resolved_ballistic_pairs.insert((candidate.projectile, candidate.hit.body));
        }

        // Ballistic response changes velocity/angular velocity but not current geometry, so any exact
        // contact evidence retained by the pre-impact solve remains valid for the next safety check.
        reusable_contacts = pre_impact.reusable_contacts;
        remaining = ballistic_remaining_after(remaining, frontier.time)?;
    }

    Ok((
        TailSliceResult3d {
            contact_count,
            reusable_contacts,
        },
        None,
    ))
}

fn stabilize_tail_contacts_in_place(
    boxes: &mut [RigidBox3d],
    solver_passes: u8,
    broad_phase: &mut RotatingBroadPhase3d,
    response_scratch: &mut RotatingContactResponseScratch3d,
    stats: &mut TailStepStats3d,
    journal: Option<&mut TailMutationJournal3d>,
) -> Result<TailSliceResult3d, RotatingWorldError3d> {
    let contacts = contact_frontier(boxes, broad_phase, stats)?;
    let contact_count = contacts.len();
    if contacts.is_empty() {
        return Ok(TailSliceResult3d {
            contact_count: 0,
            reusable_contacts: Some(contacts),
        });
    }

    if let Some(journal) = journal {
        for contact in &contacts {
            for id in [contact.pair.left, contact.pair.right] {
                let world_index = boxes
                    .iter()
                    .position(|rigid_box| rigid_box.body().id() == id)
                    .expect("tail contact participant must exist in current world");
                if boxes[world_index].body().kind() == BodyKind::Dynamic {
                    journal.record(world_index, &boxes[world_index]);
                }
            }
        }
    }

    let frontier = RotatingContactFrontier3d {
        free_flight: RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 0, 1),
        time: SampledContactTime3d::ZERO,
        contacts,
        remaining_numerator: 0,
    };
    let (_, _, geometry_modified_body_ids) =
        resolve_rotating_contact_frontier_with_activity_and_scratch(
            boxes,
            &frontier,
            solver_passes,
            response_scratch,
        )?;
    Ok(TailSliceResult3d {
        contact_count,
        reusable_contacts: geometry_modified_body_ids
            .is_empty()
            .then_some(frontier.contacts),
    })
}

fn consume_tail(
    boxes: Vec<RigidBox3d>,
    remaining: RigidBoxFreeFlightConfig3d,
    solver_passes: u8,
    broad_phase: &mut RotatingBroadPhase3d,
    response_scratch: &mut RotatingContactResponseScratch3d,
) -> Result<(Vec<RigidBox3d>, TailStepStats3d), RotatingWorldError3d> {
    let mut stats = TailStepStats3d::default();
    let mut current = boxes;
    let initial_contacts = contact_frontier(&current, broad_phase, &mut stats)?;
    if initial_contacts.is_empty() {
        let result = free_flight_and_stabilize_in_place(
            &mut current,
            remaining,
            solver_passes,
            broad_phase,
            response_scratch,
            &mut stats,
            None,
        )?;
        stats.contacts = result.contact_count;
        return Ok((current, stats));
    }

    let mut slice_count = persistent_tail_slice_count(&current, remaining, &initial_contacts)?;
    let mut journal = TailMutationJournal3d::new(current.len());
    let mut reusable_contacts = Some(initial_contacts.clone());
    loop {
        let slice_config = tail_slice_config(remaining, slice_count)?;
        let mut contact_count = 0_usize;
        let mut unsafe_body = None;

        for _ in 0..slice_count {
            stats.slices = stats.slices.saturating_add(1);
            let current_contacts = match reusable_contacts.take() {
                Some(contacts) => contacts,
                None => contact_frontier(&current, broad_phase, &mut stats)?,
            };
            if let Some(id) = first_unsafe_tail_body(&current, slice_config, &current_contacts)? {
                unsafe_body = Some(id);
                break;
            }
            let result = free_flight_and_stabilize_in_place(
                &mut current,
                slice_config,
                solver_passes,
                broad_phase,
                response_scratch,
                &mut stats,
                Some(&mut journal),
            )?;
            contact_count = contact_count.saturating_add(result.contact_count);
            reusable_contacts = result.reusable_contacts;
        }

        if unsafe_body.is_none() {
            stats.contacts = contact_count;
            return Ok((current, stats));
        }
        stats.replays = stats.replays.saturating_add(1);
        let target = slice_count
            .saturating_mul(2)
            .min(MAX_PERSISTENT_TAIL_SLICES);
        if target <= slice_count {
            return Err(RotatingWorldError3d::PersistentTailMotionUnsafe(
                unsafe_body.expect("unsafe tail body was observed"),
            ));
        }
        let next_slice_count = next_representable_tail_slice_count(remaining, target)?;
        journal.rollback(&mut current);
        reusable_contacts = Some(initial_contacts.clone());
        slice_count = next_slice_count;
    }
}

fn advance_tail_free_flight_in_place(
    boxes: &mut [RigidBox3d],
    config: RigidBoxFreeFlightConfig3d,
    mut journal: Option<&mut TailMutationJournal3d>,
) -> Result<usize, RotatingWorldError3d> {
    let mut changed = 0_usize;
    for (world_index, rigid_box) in boxes.iter_mut().enumerate() {
        if rigid_box.body().kind() == BodyKind::Fixed {
            continue;
        }
        let next = sample_rigid_box_free_flight(rigid_box, config, 1, 1)?;
        if next == *rigid_box {
            continue;
        }
        if let Some(journal) = journal.as_deref_mut() {
            journal.record(world_index, rigid_box);
        }
        *rigid_box = next;
        changed = changed.saturating_add(1);
    }
    Ok(changed)
}

fn free_flight_and_stabilize_in_place(
    boxes: &mut [RigidBox3d],
    config: RigidBoxFreeFlightConfig3d,
    solver_passes: u8,
    broad_phase: &mut RotatingBroadPhase3d,
    response_scratch: &mut RotatingContactResponseScratch3d,
    stats: &mut TailStepStats3d,
    mut journal: Option<&mut TailMutationJournal3d>,
) -> Result<TailSliceResult3d, RotatingWorldError3d> {
    advance_tail_free_flight_in_place(boxes, config, journal.as_deref_mut())?;

    let contacts = contact_frontier(boxes, broad_phase, stats)?;
    let contact_count = contacts.len();
    if contacts.is_empty() {
        return Ok(TailSliceResult3d {
            contact_count: 0,
            reusable_contacts: Some(contacts),
        });
    }

    if let Some(journal) = journal {
        for contact in &contacts {
            for id in [contact.pair.left, contact.pair.right] {
                let world_index = boxes
                    .iter()
                    .position(|rigid_box| rigid_box.body().id() == id)
                    .expect("tail contact participant must exist in current world");
                if boxes[world_index].body().kind() == BodyKind::Dynamic {
                    journal.record(world_index, &boxes[world_index]);
                }
            }
        }
    }

    let frontier = RotatingContactFrontier3d {
        free_flight: RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 0, 1),
        time: SampledContactTime3d::ZERO,
        contacts,
        remaining_numerator: 0,
    };
    let (_, _, geometry_modified_body_ids) =
        resolve_rotating_contact_frontier_with_activity_and_scratch(
            boxes,
            &frontier,
            solver_passes,
            response_scratch,
        )?;
    Ok(TailSliceResult3d {
        contact_count,
        reusable_contacts: geometry_modified_body_ids
            .is_empty()
            .then_some(frontier.contacts),
    })
}

fn persistent_tail_slice_count(
    boxes: &[RigidBox3d],
    remaining: RigidBoxFreeFlightConfig3d,
    contacts: &[RotatingContactSearchHit3d],
) -> Result<u32, RotatingWorldError3d> {
    for slices in 1..=MAX_PERSISTENT_TAIL_SLICES {
        let Ok(config) = tail_slice_config(remaining, slices) else {
            continue;
        };
        if first_unsafe_tail_body(boxes, config, contacts)?.is_none() {
            return Ok(slices);
        }
    }

    Err(RotatingWorldError3d::PersistentTailResolutionLimit(
        MAX_PERSISTENT_TAIL_SLICES,
    ))
}

fn next_representable_tail_slice_count(
    remaining: RigidBoxFreeFlightConfig3d,
    minimum: u32,
) -> Result<u32, RotatingWorldError3d> {
    for slices in minimum..=MAX_PERSISTENT_TAIL_SLICES {
        if tail_slice_config(remaining, slices).is_ok() {
            return Ok(slices);
        }
    }
    Err(RotatingWorldError3d::PersistentTailResolutionLimit(
        MAX_PERSISTENT_TAIL_SLICES,
    ))
}

fn tail_slice_config(
    remaining: RigidBoxFreeFlightConfig3d,
    slices: u32,
) -> Result<RigidBoxFreeFlightConfig3d, RotatingWorldError3d> {
    if slices == 0 {
        return Err(RotatingWorldError3d::PersistentTailResolutionLimit(
            MAX_PERSISTENT_TAIL_SLICES,
        ));
    }
    remaining.scaled_fraction(1, slices).map_err(|_| {
        RotatingWorldError3d::PersistentTailResolutionLimit(MAX_PERSISTENT_TAIL_SLICES)
    })
}

fn first_unsafe_tail_body(
    boxes: &[RigidBox3d],
    config: RigidBoxFreeFlightConfig3d,
    contacts: &[RotatingContactSearchHit3d],
) -> Result<Option<BodyId>, RotatingWorldError3d> {
    for rigid_box in boxes {
        let id = rigid_box.body().id();
        if !contacts
            .iter()
            .any(|contact| contact.pair.left == id || contact.pair.right == id)
        {
            continue;
        }
        if !tail_motion_within_extent(rigid_box, config)? {
            return Ok(Some(id));
        }
    }
    Ok(None)
}

fn tail_motion_within_extent(
    rigid_box: &RigidBox3d,
    config: RigidBoxFreeFlightConfig3d,
) -> Result<bool, RotatingWorldError3d> {
    if rigid_box.body().kind() == BodyKind::Fixed || config.timestep_is_zero() {
        return Ok(true);
    }

    let id = rigid_box.body().id();
    let timestep = config
        .exact_timestep()
        .map_err(|_| RotatingWorldError3d::PersistentTailArithmeticOverflow(id))?;
    let half = rigid_box.body().half_extents();
    let minimum_half = u128::from(half.x.min(half.y).min(half.z).unsigned_abs());
    let motion_budget = minimum_half
        .checked_mul(2)
        .ok_or(RotatingWorldError3d::PersistentTailArithmeticOverflow(id))?;

    let velocity = rigid_box.body().velocity();
    let velocity_components = [velocity.x, velocity.y, velocity.z];
    let gravity_components = [config.gravity.x, config.gravity.y, config.gravity.z];
    let mut translation_bound = 0_u128;
    for axis in 0..3 {
        let gravity_delta = timestep
            .mul_ceil_u128(u128::from(gravity_components[axis].unsigned_abs()))
            .map_err(|_| RotatingWorldError3d::PersistentTailArithmeticOverflow(id))?;
        let speed_bound = u128::from(velocity_components[axis].unsigned_abs())
            .checked_add(gravity_delta)
            .ok_or(RotatingWorldError3d::PersistentTailArithmeticOverflow(id))?;
        let travel = timestep
            .mul_ceil_u128(speed_bound)
            .map_err(|_| RotatingWorldError3d::PersistentTailArithmeticOverflow(id))?;
        translation_bound = translation_bound.max(travel);
    }

    let angular = rigid_box.angular().angular_velocity;
    let angular_speed_l1 = if rigid_box.rotation_locked() {
        0
    } else {
        u128::from(angular.x.unsigned_abs())
            .checked_add(u128::from(angular.y.unsigned_abs()))
            .and_then(|value| value.checked_add(u128::from(angular.z.unsigned_abs())))
            .ok_or(RotatingWorldError3d::PersistentTailArithmeticOverflow(id))?
    };
    let radius_bound = u128::from(half.x.unsigned_abs())
        .checked_add(u128::from(half.y.unsigned_abs()))
        .and_then(|value| value.checked_add(u128::from(half.z.unsigned_abs())))
        .ok_or(RotatingWorldError3d::PersistentTailArithmeticOverflow(id))?;
    let angular_numerator_motion = timestep
        .mul_ceil_u128(
            radius_bound
                .checked_mul(angular_speed_l1)
                .ok_or(RotatingWorldError3d::PersistentTailArithmeticOverflow(id))?,
        )
        .map_err(|_| RotatingWorldError3d::PersistentTailArithmeticOverflow(id))?;
    let angular_motion = ceil_div(
        angular_numerator_motion,
        u128::from(ANGULAR_VELOCITY_SCALE.unsigned_abs()),
        id,
    )?;

    let total_motion = translation_bound
        .checked_add(angular_motion)
        .ok_or(RotatingWorldError3d::PersistentTailArithmeticOverflow(id))?;
    Ok(total_motion <= motion_budget)
}

fn ceil_div(value: u128, denominator: u128, id: BodyId) -> Result<u128, RotatingWorldError3d> {
    if denominator == 0 {
        return Err(RotatingWorldError3d::PersistentTailArithmeticOverflow(id));
    }
    let quotient = value / denominator;
    if value.is_multiple_of(denominator) {
        Ok(quotient)
    } else {
        quotient
            .checked_add(1)
            .ok_or(RotatingWorldError3d::PersistentTailArithmeticOverflow(id))
    }
}

fn contact_frontier(
    boxes: &[RigidBox3d],
    broad_phase: &mut RotatingBroadPhase3d,
    stats: &mut TailStepStats3d,
) -> Result<Vec<RotatingContactSearchHit3d>, RotatingWorldError3d> {
    let pairs = broad_phase
        .response_candidate_pairs(boxes, RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 0, 1))
        .map_err(map_tail_broad_phase_error)?;
    stats.candidate_pairs = stats
        .candidate_pairs
        .saturating_add(u64::try_from(pairs.len()).unwrap_or(u64::MAX));

    let mut contacts = Vec::new();
    for pair in pairs {
        let left = boxes
            .iter()
            .find(|rigid_box| rigid_box.body().id() == pair.left)
            .expect("tail broad phase candidate left body must exist");
        let right = boxes
            .iter()
            .find(|rigid_box| rigid_box.body().id() == pair.right)
            .expect("tail broad phase candidate right body must exist");
        let Some(contact) = obb_contact_seed(left.oriented_box(), right.oriented_box())? else {
            continue;
        };
        contacts.push(RotatingContactSearchHit3d {
            time: SampledContactTime3d::ZERO,
            pair,
            contact,
        });
    }
    Ok(contacts)
}

fn map_tail_broad_phase_error(error: RotatingBroadPhaseError3d) -> RotatingWorldError3d {
    match error {
        RotatingBroadPhaseError3d::DuplicateBodyId(id) => RotatingWorldError3d::DuplicateBody(id),
        RotatingBroadPhaseError3d::IncrementalQueryUnsynchronized(id) => {
            RotatingWorldError3d::MissingBody(id)
        }
        RotatingBroadPhaseError3d::FreeFlight(error) => RotatingWorldError3d::FreeFlight(error),
    }
}

#[cfg(test)]
mod tests {
    use std::{hint::black_box, time::Instant};

    use crate::{
        ANGULAR_VELOCITY_SCALE, AngularState3d, AngularVelocity3d, BodyId, Orientation3d,
        OrientedBox3d, RigidBody, RigidBox3d, RigidBoxFreeFlightConfig3d, Vec3i,
        sample_rigid_box_free_flight,
    };

    use super::{
        RotatingBroadPhase3d, RotatingContactResponseScratch3d, RotatingWorld3d,
        RotatingWorldConfig3d, RotatingWorldError3d, TailMutationJournal3d, TailStepStats3d,
        advance_tail_free_flight_in_place, consume_tail, contact_frontier,
        tail_motion_within_extent, tail_slice_config,
    };

    fn dynamic(id: u64, position: Vec3i, velocity: Vec3i, half: Vec3i) -> RigidBox3d {
        RigidBox3d::new(
            RigidBody::dynamic(BodyId(id), position, velocity, half),
            AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
        )
        .expect("valid dynamic box")
    }

    fn fixed(id: u64, position: Vec3i, half: Vec3i) -> RigidBox3d {
        RigidBox3d::new(
            RigidBody::fixed(BodyId(id), position, half),
            AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
        )
        .expect("valid fixed box")
    }

    fn world(gravity: Vec3i) -> RotatingWorld3d {
        RotatingWorld3d::new(RotatingWorldConfig3d {
            gravity,
            sample_count: 32,
            refinement_steps: 4,
            solver_passes: 8,
            max_events: 16,
        })
    }

    #[test]
    fn tail_free_flight_in_place_matches_materialized_sampling() {
        let config = RigidBoxFreeFlightConfig3d::new(Vec3i::new(0, -3, 0), 1, 4);
        let baseline = vec![
            dynamic(
                1,
                Vec3i::new(0, 10, 0),
                Vec3i::new(8, -2, 0),
                Vec3i::new(1, 1, 1),
            ),
            fixed(2, Vec3i::new(20, 0, 0), Vec3i::new(2, 2, 2)),
            dynamic(
                3,
                Vec3i::new(-10, 5, 0),
                Vec3i::new(-4, 0, 0),
                Vec3i::new(1, 2, 1),
            ),
        ];
        let expected = baseline
            .iter()
            .map(|rigid_box| sample_rigid_box_free_flight(rigid_box, config, 1, 1))
            .collect::<Result<Vec<_>, _>>()
            .expect("materialized free flight");
        let mut actual = baseline.clone();

        let changed = advance_tail_free_flight_in_place(&mut actual, config, None)
            .expect("in-place free flight");

        assert_eq!(actual, expected);
        assert_eq!(changed, 2);
        assert_eq!(actual[1], baseline[1], "fixed body must remain untouched");
    }

    #[test]
    fn tail_mutation_journal_restores_exact_attempt_state() {
        let baseline = (0..64)
            .map(|index| {
                dynamic(
                    index + 1,
                    Vec3i::new(index as i32 * 4, 0, 0),
                    Vec3i::ZERO,
                    Vec3i::new(1, 1, 1),
                )
            })
            .collect::<Vec<_>>();
        let mut current = baseline.clone();
        let mut journal = TailMutationJournal3d::new(current.len());

        for index in [3_usize, 17, 42] {
            journal.record(index, &current[index]);
            current[index].body.position.x = current[index].body.position.x.saturating_add(11);
            journal.record(index, &current[index]);
            current[index].body.velocity.y = current[index].body.velocity.y.saturating_sub(7);
        }

        assert_eq!(journal.originals.len(), 3);
        assert_ne!(current, baseline);
        journal.rollback(&mut current);
        assert_eq!(current, baseline);
        assert!(journal.originals.is_empty());
        assert!(journal.recorded.iter().all(|recorded| !recorded));
    }

    #[test]
    #[ignore = "release performance evidence"]
    fn tail_in_place_sparse_free_flight_benchmark() {
        const BODY_COUNT: usize = 4_096;
        const DYNAMIC_COUNT: usize = 8;
        const SLICES: usize = 256;

        let baseline = (0..BODY_COUNT)
            .map(|index| {
                if index < DYNAMIC_COUNT {
                    dynamic(
                        index as u64 + 1,
                        Vec3i::new(index as i32 * 8, 100, 0),
                        Vec3i::new(1, 0, 0),
                        Vec3i::new(1, 1, 1),
                    )
                } else {
                    fixed(
                        index as u64 + 1,
                        Vec3i::new(index as i32 * 8, 0, 0),
                        Vec3i::new(1, 1, 1),
                    )
                }
            })
            .collect::<Vec<_>>();
        let config = RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 1, 1);

        let mut materialized = baseline.clone();
        let materialized_started = Instant::now();
        for _ in 0..SLICES {
            materialized = materialized
                .iter()
                .map(|rigid_box| sample_rigid_box_free_flight(rigid_box, config, 1, 1))
                .collect::<Result<Vec<_>, _>>()
                .expect("materialized tail free flight");
            black_box(&materialized);
        }
        let materialized_elapsed = materialized_started.elapsed();

        let mut in_place = baseline;
        let in_place_started = Instant::now();
        let mut changed = 0_usize;
        for _ in 0..SLICES {
            changed = changed.saturating_add(
                advance_tail_free_flight_in_place(&mut in_place, config, None)
                    .expect("in-place tail free flight"),
            );
            black_box(&in_place);
        }
        let in_place_elapsed = in_place_started.elapsed();

        assert_eq!(materialized, in_place);
        assert_eq!(changed, DYNAMIC_COUNT * SLICES);
        let speedup = materialized_elapsed.as_secs_f64() / in_place_elapsed.as_secs_f64();
        println!(
            "persistent tail sparse free flight {BODY_COUNT}-body world × {SLICES}: materialized={materialized_elapsed:?}, in_place={in_place_elapsed:?}, speedup={speedup:.2}x, fixed_sample_reduction={}x, world_vec_materializations={SLICES}->0",
            BODY_COUNT / DYNAMIC_COUNT,
        );
    }

    #[test]
    #[ignore = "release performance evidence"]
    fn tail_mutation_journal_sparse_rollback_benchmark() {
        const BODY_COUNT: usize = 4_096;
        const TOUCHED: usize = 8;
        const REPLAYS: usize = 256;

        let baseline = (0..BODY_COUNT)
            .map(|index| {
                if index < TOUCHED {
                    dynamic(
                        index as u64 + 1,
                        Vec3i::new(index as i32 * 4, 0, 0),
                        Vec3i::ZERO,
                        Vec3i::new(1, 1, 1),
                    )
                } else {
                    fixed(
                        index as u64 + 1,
                        Vec3i::new(index as i32 * 4, 0, 0),
                        Vec3i::new(1, 1, 1),
                    )
                }
            })
            .collect::<Vec<_>>();

        let snapshot_started = Instant::now();
        let mut snapshot_checksum = 0_i64;
        for replay in 0..REPLAYS {
            let mut attempt = baseline.clone();
            let delta = i32::try_from(replay % 17 + 1).expect("bounded benchmark delta");
            for rigid_box in attempt.iter_mut().take(TOUCHED) {
                rigid_box.body.position.x = rigid_box.body.position.x.saturating_add(delta);
                snapshot_checksum =
                    snapshot_checksum.saturating_add(i64::from(rigid_box.body.position.x));
            }
            black_box(&attempt);
        }
        let snapshot_elapsed = snapshot_started.elapsed();

        let mut current = baseline.clone();
        let mut journal = TailMutationJournal3d::new(current.len());
        let journal_started = Instant::now();
        let mut journal_checksum = 0_i64;
        for replay in 0..REPLAYS {
            let delta = i32::try_from(replay % 17 + 1).expect("bounded benchmark delta");
            for (index, rigid_box) in current.iter_mut().enumerate().take(TOUCHED) {
                journal.record(index, rigid_box);
                rigid_box.body.position.x = rigid_box.body.position.x.saturating_add(delta);
                journal_checksum =
                    journal_checksum.saturating_add(i64::from(rigid_box.body.position.x));
            }
            black_box(&current);
            journal.rollback(&mut current);
        }
        let journal_elapsed = journal_started.elapsed();

        assert_eq!(current, baseline);
        assert_eq!(snapshot_checksum, journal_checksum);
        let speedup = snapshot_elapsed.as_secs_f64() / journal_elapsed.as_secs_f64();
        println!(
            "persistent tail sparse rollback {BODY_COUNT}-body world × {REPLAYS}: full_world_snapshots={snapshot_elapsed:?}, mutation_journal={journal_elapsed:?}, speedup={speedup:.2}x, body_clone_reduction={}x",
            BODY_COUNT / TOUCHED,
        );
    }

    #[test]
    fn tail_reuses_current_contact_evidence_between_slice_boundaries() {
        let boxes = vec![
            fixed(1, Vec3i::new(0, -1, 0), Vec3i::new(20, 1, 20)),
            dynamic(2, Vec3i::new(0, 1, 0), Vec3i::ZERO, Vec3i::new(1, 1, 1)),
        ];
        let remaining = RigidBoxFreeFlightConfig3d::new(Vec3i::new(0, -3_600, 0), 1, 60);
        let mut broad_phase = RotatingBroadPhase3d::default();
        let mut response_scratch = RotatingContactResponseScratch3d::default();

        let (_, stats) = consume_tail(boxes, remaining, 8, &mut broad_phase, &mut response_scratch)
            .expect("resting tail");

        assert!(stats.slices > 0, "fixture must exercise sliced tail work");
        assert_eq!(stats.replays, 0, "fixture should not need a replay");
        assert!(
            broad_phase.stats().queries <= stats.slices.saturating_mul(2),
            "tail re-queried current contacts at both sides of every slice: stats={stats:?}, broad_phase={:?}",
            broad_phase.stats()
        );
    }

    #[test]
    fn widened_tail_denominator_can_be_sliced_exactly() {
        let remaining =
            RigidBoxFreeFlightConfig3d::new_wide(Vec3i::ZERO, 35_582_088, 2_147_483_647);
        let sliced = tail_slice_config(remaining, 2).expect("wide tail slice");
        assert_eq!(sliced.timestep_i128(), Some((17_791_044, 2_147_483_647)));
        assert!(
            sliced
                .timestep_i128()
                .expect("ratio remains narrow in this regression")
                .1
                > i128::from(i32::MAX) - 1
        );
    }

    #[test]
    fn widened_tail_motion_bound_avoids_intermediate_overflow() {
        let rigid_box = RigidBox3d::new(
            RigidBody::dynamic(
                BodyId(9),
                Vec3i::ZERO,
                Vec3i::new(10, 0, 0),
                Vec3i::new(100, 100, 100),
            ),
            AngularState3d::new(
                Orientation3d::IDENTITY,
                AngularVelocity3d::new(0, 0, ANGULAR_VELOCITY_SCALE / 10),
            ),
        )
        .expect("valid rotating box");
        let mut config = RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 1, 1);
        for _ in 0..20 {
            config = config
                .scaled_fraction(511, 512)
                .expect("bounded exact tail growth");
        }
        assert!(config.timestep_i128().is_none());

        assert!(
            tail_motion_within_extent(&rigid_box, config)
                .expect("wide tail bound remains representable")
        );
    }

    #[test]
    fn tail_frontier_broad_phase_prunes_separated_pairs_without_changing_contact_truth() {
        let mut boxes = vec![
            dynamic(1, Vec3i::new(0, 1, 0), Vec3i::ZERO, Vec3i::new(1, 1, 1)),
            fixed(2, Vec3i::new(0, -1, 0), Vec3i::new(20, 1, 20)),
        ];
        for id in 3..35 {
            boxes.push(fixed(
                id,
                Vec3i::new((id as i32) * 100, 100, 100),
                Vec3i::new(1, 1, 1),
            ));
        }
        let all_pairs = boxes.len().saturating_mul(boxes.len().saturating_sub(1)) / 2;
        let mut broad_phase = RotatingBroadPhase3d::default();
        let mut stats = TailStepStats3d::default();

        let contacts =
            contact_frontier(&boxes, &mut broad_phase, &mut stats).expect("broad-phase frontier");

        assert_eq!(contacts.len(), 1);
        assert_eq!(contacts[0].pair.left, BodyId(1));
        assert_eq!(contacts[0].pair.right, BodyId(2));
        assert!(
            stats.candidate_pairs < u64::try_from(all_pairs).unwrap_or(u64::MAX),
            "tail broad phase did not prune any exact OBB tests"
        );
    }

    #[test]
    fn controlled_velocity_and_overlap_queries_preserve_stable_identity() {
        let empty_world = world(Vec3i::ZERO);
        assert!(
            empty_world
                .overlap_query(OrientedBox3d::new(
                    Vec3i::ZERO,
                    Vec3i::new(1, 0, 1),
                    Orientation3d::IDENTITY,
                ))
                .is_err(),
            "empty-world queries must still validate their geometry"
        );

        let mut world = world(Vec3i::ZERO);
        world
            .add_box(dynamic(1, Vec3i::ZERO, Vec3i::ZERO, Vec3i::new(2, 2, 2)))
            .expect("dynamic");
        world
            .add_box(fixed(2, Vec3i::new(5, 0, 0), Vec3i::new(2, 2, 2)))
            .expect("fixed");
        world
            .set_linear_velocity(BodyId(1), Vec3i::new(60, 0, 0))
            .expect("controlled velocity");
        assert_eq!(
            world
                .box_by_id(BodyId(1))
                .expect("dynamic")
                .body()
                .velocity(),
            Vec3i::new(60, 0, 0)
        );
        assert_eq!(
            world.set_linear_velocity(BodyId(2), Vec3i::new(1, 0, 0)),
            Err(RotatingWorldError3d::FixedBodyVelocity(BodyId(2)))
        );

        let hits = world
            .overlap_query(OrientedBox3d::new(
                Vec3i::new(2, 0, 0),
                Vec3i::new(1, 1, 1),
                Orientation3d::IDENTITY,
            ))
            .expect("valid overlap query");
        assert_eq!(hits, vec![BodyId(1), BodyId(2)]);
    }

    #[test]
    fn persistent_solver_partitions_track_lifecycle_without_frame_rediscovery() {
        let mut world = world(Vec3i::ZERO);
        world
            .add_box(dynamic(1, Vec3i::ZERO, Vec3i::ZERO, Vec3i::new(1, 1, 1)))
            .expect("physics-owned dynamic");
        world
            .add_box(
                dynamic(
                    2,
                    Vec3i::new(100, 0, 0),
                    Vec3i::new(1, 0, 0),
                    Vec3i::new(1, 1, 1),
                )
                .with_overlap_only(),
            )
            .expect("overlap-only dynamic");
        world
            .add_box(fixed(3, Vec3i::new(200, 0, 0), Vec3i::new(1, 1, 1)))
            .expect("fixed");

        assert_eq!(world.solver_partitions.dynamic_body_ids.len(), 2);
        assert_eq!(world.solver_partitions.solid_body_ids.len(), 2);
        assert_eq!(world.solver_partitions.bypassed_dynamic_ids.len(), 1);
        assert_eq!(world.solver_partitions.response_authority_body_ids.len(), 1);

        let first = world.step(1, 60).expect("partitioned step");
        let second = world.step(1, 60).expect("stable partitioned step");
        assert_eq!(first.stats.solver_body_count, 2);
        assert_eq!(second.stats.solver_body_count, 2);

        world
            .remove_box(BodyId(1))
            .expect("remove solver authority");
        assert!(
            world
                .solver_partitions
                .response_authority_body_ids
                .is_empty()
        );
        let bypassed = world.step(1, 60).expect("all-dynamic bypass step");
        assert_eq!(bypassed.stats.solver_body_count, 0);
        assert_eq!(bypassed.stats.solver_bypassed_body_count, 2);
    }

    #[test]
    fn persistent_broad_phase_reuses_topology_across_unchanged_steps() {
        let mut world = world(Vec3i::ZERO);
        world
            .add_box(dynamic(1, Vec3i::ZERO, Vec3i::ZERO, Vec3i::new(1, 1, 1)))
            .expect("dynamic");
        world
            .add_box(fixed(2, Vec3i::new(100, 0, 0), Vec3i::new(1, 1, 1)))
            .expect("fixed");

        let first = world.step(1, 60).expect("first static frame");
        let second = world.step(1, 60).expect("second static frame");

        assert_eq!(first.stats.broad_phase_queries, 1);
        assert_eq!(first.stats.broad_phase_rebuilds, 1);
        assert_eq!(first.stats.broad_phase_reuses, 0);
        assert_eq!(second.stats.broad_phase_queries, 1);
        assert_eq!(second.stats.broad_phase_rebuilds, 0);
        assert_eq!(second.stats.broad_phase_reuses, 1);
    }

    #[test]
    fn persistent_floor_contact_cannot_tunnel_across_a_large_tail() {
        let mut world = world(Vec3i::new(0, -100, 0));
        world
            .add_box(fixed(1, Vec3i::new(0, -1, 0), Vec3i::new(20, 1, 20)))
            .expect("floor");
        world
            .add_box(dynamic(
                2,
                Vec3i::new(0, 1, 0),
                Vec3i::ZERO,
                Vec3i::new(1, 1, 1),
            ))
            .expect("box");

        world
            .step(1, 1)
            .expect("persistent tail must remain constrained");

        let body = world.box_by_id(BodyId(2)).expect("box remains").body();
        assert!(
            body.position().y >= 1,
            "box tunneled through a time-zero floor contact: {body:?}"
        );
    }

    #[test]
    fn resting_floor_contact_consumes_the_full_frame() {
        let mut world = world(Vec3i::new(0, -3_600, 0));
        world
            .add_box(fixed(1, Vec3i::new(0, -1, 0), Vec3i::new(20, 1, 20)))
            .expect("floor");
        world
            .add_box(dynamic(
                2,
                Vec3i::new(0, 1, 0),
                Vec3i::ZERO,
                Vec3i::new(1, 1, 1),
            ))
            .expect("box");

        for _ in 0..8 {
            world.step(1, 60).expect("stable frame");
        }

        let body = world.box_by_id(BodyId(2)).expect("box remains").body();
        assert!(
            body.position().y >= 1,
            "box sank through the floor: {body:?}"
        );
        assert!(
            body.velocity().y >= 0,
            "resting box kept downward velocity: {body:?}"
        );
    }

    #[test]
    fn off_center_projectile_impact_generates_spin() {
        let mut world = world(Vec3i::ZERO);
        world
            .add_box(dynamic(10, Vec3i::ZERO, Vec3i::ZERO, Vec3i::new(3, 3, 3)))
            .expect("target");
        world
            .add_box(dynamic(
                20,
                Vec3i::new(-12, 2, 0),
                Vec3i::new(720, 0, 0),
                Vec3i::new(1, 1, 1),
            ))
            .expect("projectile");

        world.step(1, 60).expect("impact frame");

        let target = world.box_by_id(BodyId(10)).expect("target remains");
        assert!(
            !target.angular().angular_velocity.is_zero(),
            "off-center impact did not generate angular velocity"
        );
    }

    #[test]
    fn identical_steps_are_bit_for_bit_repeatable() {
        let mut first = world(Vec3i::new(0, -3_600, 0));
        first
            .add_box(fixed(1, Vec3i::new(0, -1, 0), Vec3i::new(20, 1, 20)))
            .expect("floor");
        first
            .add_box(dynamic(
                2,
                Vec3i::new(0, 8, 0),
                Vec3i::new(120, 0, 0),
                Vec3i::new(2, 2, 2),
            ))
            .expect("box");
        let mut second = first.clone();

        for _ in 0..12 {
            first.step(1, 60).expect("first frame");
            second.step(1, 60).expect("second frame");
        }

        assert_eq!(
            first.boxes().cloned().collect::<Vec<_>>(),
            second.boxes().cloned().collect::<Vec<_>>()
        );
    }
}
