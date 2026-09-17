use std::{
    cmp::Ordering,
    collections::{BTreeMap, BTreeSet},
    error::Error,
    fmt,
};

use crate::{
    ANGULAR_VELOCITY_SCALE, AngularError3d, AngularVelocity3d, BallisticSphere3d,
    BallisticSphereError3d, BallisticSphereQueryStats3d, BallisticSphereScene3d,
    BallisticSphereSweepHit3d, BodyId, BodyKind, MATERIAL_SCALE, ORIENTATION_SCALE, Orientation3d,
    RepeatedRotatingEventConfig3d, RepeatedRotatingEventError3d, RepeatedRotatingEventWorkStats3d,
    RigidBox3d, RigidBoxFreeFlightConfig3d, RigidBoxFreeFlightError3d,
    RotatingContactFrontierError3d, RotatingContactResponseError3d, RotatingContactSearchConfig3d,
    RotatingResolvedEvent3d, SampledContactTime3d, Vec3i, box_inertia,
    sample_rigid_box_free_flight,
    wide_ratio::{WideRatioError, mul_div_round_i128, mul_div_round_u128},
};
use crate::{
    repeated_rotating_events::advance_repeated_rotating_events_with_broad_phase,
    rotating_broad_phase::RotatingBroadPhase3d,
    rotating_contact_frontier::next_rotating_contact_frontier_with_broad_phase,
    rotating_contact_response::{
        RotatingContactResponseScratch3d,
        resolve_rotating_contact_frontier_with_activity_and_scratch,
    },
};

const BALLISTIC_TIME_SCALE: u64 = 1_u64 << 32;
const TIMELINE_TIME_SCALE: u32 = 1_u32 << 31;
const RESPONSE_SCALE: i128 = 1_i128 << 50;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct BallisticEventTimelineConfig3d {
    pub rigid: RepeatedRotatingEventConfig3d,
}

impl BallisticEventTimelineConfig3d {
    #[must_use]
    pub const fn new(rigid: RepeatedRotatingEventConfig3d) -> Self {
        Self { rigid }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct BallisticResolvedImpact3d {
    pub projectile: BodyId,
    pub target: BodyId,
    pub time: SampledContactTime3d,
    pub normal: [i128; 3],
    /// Impulse that would be applied to the projectile. The rigid target receives the opposite vector.
    pub normal_impulse: [i128; 3],
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum BallisticTimelineEvent3d {
    Rigid(RotatingResolvedEvent3d),
    Ballistic(Vec<BallisticResolvedImpact3d>),
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct BallisticEventTimelineWorkStats3d {
    pub ballistic_query_rounds: u64,
    pub ballistic_target_bound_checks: u64,
    pub ballistic_broad_phase_candidates: u64,
    pub ballistic_toi_tests: u64,
    pub ballistic_feature_tests: u64,
    pub ballistic_impacts: u64,
    pub rigid_body_motion_samples: u64,
    pub projectile_motion_samples: u64,
    pub zero_time_rigid_resolutions: u64,
    pub rigid: RepeatedRotatingEventWorkStats3d,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BallisticEventTimelineReport3d {
    pub events: Vec<BallisticTimelineEvent3d>,
    pub work: BallisticEventTimelineWorkStats3d,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BallisticEventTimelineError3d {
    ZeroEventLimit,
    EventLimit(u16),
    DuplicateBody(BodyId),
    MissingTarget(BodyId),
    Rigid(RepeatedRotatingEventError3d),
    Frontier(RotatingContactFrontierError3d),
    Response(RotatingContactResponseError3d),
    FreeFlight(RigidBoxFreeFlightError3d),
    Ballistic(BallisticSphereError3d),
    Angular(AngularError3d),
    ArithmeticOverflow,
}

impl fmt::Display for BallisticEventTimelineError3d {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::ZeroEventLimit => write!(
                formatter,
                "ballistic event timeline requires at least one event slot"
            ),
            Self::EventLimit(limit) => write!(
                formatter,
                "ballistic event timeline reached its {limit}-event limit while time remained"
            ),
            Self::DuplicateBody(id) => write!(
                formatter,
                "ballistic event timeline contains duplicate body id {}",
                id.0
            ),
            Self::MissingTarget(id) => write!(
                formatter,
                "ballistic impact target {} is missing from the rigid world",
                id.0
            ),
            Self::Rigid(error) => {
                write!(
                    formatter,
                    "ballistic event timeline rigid advance failed: {error}"
                )
            }
            Self::Frontier(error) => {
                write!(
                    formatter,
                    "ballistic event timeline rigid frontier failed: {error}"
                )
            }
            Self::Response(error) => {
                write!(
                    formatter,
                    "ballistic event timeline rigid response failed: {error}"
                )
            }
            Self::FreeFlight(error) => {
                write!(
                    formatter,
                    "ballistic event timeline free flight failed: {error}"
                )
            }
            Self::Ballistic(error) => {
                write!(
                    formatter,
                    "ballistic event timeline sphere query failed: {error}"
                )
            }
            Self::Angular(error) => {
                write!(
                    formatter,
                    "ballistic event timeline angular response failed: {error}"
                )
            }
            Self::ArithmeticOverflow => {
                write!(formatter, "ballistic event timeline arithmetic overflowed")
            }
        }
    }
}

impl Error for BallisticEventTimelineError3d {}

impl From<RepeatedRotatingEventError3d> for BallisticEventTimelineError3d {
    fn from(value: RepeatedRotatingEventError3d) -> Self {
        Self::Rigid(value)
    }
}

impl From<RotatingContactFrontierError3d> for BallisticEventTimelineError3d {
    fn from(value: RotatingContactFrontierError3d) -> Self {
        Self::Frontier(value)
    }
}

impl From<RotatingContactResponseError3d> for BallisticEventTimelineError3d {
    fn from(value: RotatingContactResponseError3d) -> Self {
        Self::Response(value)
    }
}

impl From<RigidBoxFreeFlightError3d> for BallisticEventTimelineError3d {
    fn from(value: RigidBoxFreeFlightError3d) -> Self {
        Self::FreeFlight(value)
    }
}

impl From<BallisticSphereError3d> for BallisticEventTimelineError3d {
    fn from(value: BallisticSphereError3d) -> Self {
        Self::Ballistic(value)
    }
}

impl From<AngularError3d> for BallisticEventTimelineError3d {
    fn from(value: AngularError3d) -> Self {
        Self::Angular(value)
    }
}

impl From<WideRatioError> for BallisticEventTimelineError3d {
    fn from(_: WideRatioError) -> Self {
        Self::ArithmeticOverflow
    }
}

#[derive(Clone, Copy, Debug)]
struct BallisticCandidate3d {
    projectile: BodyId,
    hit: BallisticSphereSweepHit3d,
}

#[derive(Clone, Debug)]
struct BallisticFrontier3d {
    time: SampledContactTime3d,
    hits: Vec<BallisticCandidate3d>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct MotionState3d {
    position: Vec3i,
    velocity: Vec3i,
    orientation: Orientation3d,
    angular_velocity: AngularVelocity3d,
}

impl MotionState3d {
    fn from_box(rigid_box: &RigidBox3d) -> Self {
        Self {
            position: rigid_box.body.position,
            velocity: rigid_box.body.velocity,
            orientation: rigid_box.angular.orientation,
            angular_velocity: rigid_box.angular.angular_velocity,
        }
    }

    fn apply(self, rigid_box: &mut RigidBox3d) {
        rigid_box.body.position = self.position;
        rigid_box.body.velocity = self.velocity;
        rigid_box.angular.orientation = self.orientation;
        rigid_box.angular.angular_velocity = self.angular_velocity;
    }
}

#[derive(Clone, Copy, Debug)]
struct MotionUpdate3d {
    world_index: usize,
    state: MotionState3d,
}

/// Advances one rigid world and a set of single-impact ballistic spheres on one chronological event line.
///
/// Ballistic spheres are impact-and-retire participants: they use the analytic sphere sweep for collision
/// discovery, transfer a deterministic normal impulse (including target torque), and are removed immediately
/// after their first admitted impact. Bouncy or persistent projectiles intentionally remain on the general
/// rigid-body path.
///
/// Current rigid contacts are stabilized before every positive search after a ballistic impulse. Future rigid
/// contacts use the existing sampled re-contact frontier, while ballistic hits use the prepared rounded-OBB
/// sphere sweep. The earlier event wins; exact ties preserve rigid-body authority. Ballistic Q32.32 hit times
/// are rounded upward onto a Q1.31 event fraction before state mutation, bounding temporal error below one
/// 2^-31 fraction of the current interval while keeping the existing u32 exact-fraction machinery. Hits that
/// land in the same committed Q1.31 slot share one frontier, so the conservative time rounding cannot advance
/// a later hit into overlap and then lose it on the next query.
pub fn advance_ballistic_event_timeline(
    boxes: &mut [RigidBox3d],
    projectiles: &mut Vec<BallisticSphere3d>,
    config: BallisticEventTimelineConfig3d,
) -> Result<BallisticEventTimelineReport3d, BallisticEventTimelineError3d> {
    if config.rigid.max_events == 0 {
        return Err(BallisticEventTimelineError3d::ZeroEventLimit);
    }
    validate_unique_ids(boxes, projectiles)?;

    let mut remaining = config.rigid.search.free_flight;
    let mut broad_phase = RotatingBroadPhase3d::default();
    let mut response_scratch = RotatingContactResponseScratch3d::default();
    let mut events = Vec::new();
    let mut work = BallisticEventTimelineWorkStats3d::default();
    let mut positive_events = 0_u16;

    while !remaining.timestep_is_zero() {
        if positive_events >= config.rigid.max_events {
            return Err(BallisticEventTimelineError3d::EventLimit(
                config.rigid.max_events,
            ));
        }

        resolve_current_rigid_contacts(boxes, config.rigid, &mut broad_phase, &mut work)?;

        let rigid_search = RotatingContactSearchConfig3d {
            free_flight: remaining,
            ..config.rigid.search
        };
        let rigid_frontier =
            next_rotating_contact_frontier_with_broad_phase(boxes, rigid_search, &mut broad_phase)?;
        let ballistic_frontier =
            earliest_ballistic_frontier(boxes, projectiles, remaining, &mut work)?;

        match select_event(rigid_frontier.as_ref(), ballistic_frontier.as_ref()) {
            SelectedEvent3d::None => {
                advance_boxes_to_fraction(boxes, remaining, full_time(), &mut work)?;
                advance_projectiles_to_fraction(projectiles, remaining, full_time(), &mut work)?;
                break;
            }
            SelectedEvent3d::Rigid => {
                let frontier = rigid_frontier.expect("selected rigid frontier exists");
                let staged_projectiles = staged_projectiles_after_fraction(
                    projectiles,
                    remaining,
                    frontier.time,
                    &mut work,
                )?;
                let (response, _) = resolve_rotating_contact_frontier_with_activity_and_scratch(
                    boxes,
                    &frontier,
                    config.rigid.solver_passes,
                    &mut response_scratch,
                )?;
                *projectiles = staged_projectiles;
                accumulate_response_passes(&mut work.rigid, response.passes_used);
                remaining = remaining_after(remaining, frontier.time)?;
                events.push(BallisticTimelineEvent3d::Rigid(RotatingResolvedEvent3d {
                    time: frontier.time,
                    contacts: response.contacts,
                    response_passes: response.passes_used,
                }));
                positive_events = positive_events.saturating_add(1);
            }
            SelectedEvent3d::Ballistic => {
                let frontier = ballistic_frontier.expect("selected ballistic frontier exists");
                let impacts = resolve_ballistic_frontier(
                    boxes,
                    projectiles,
                    remaining,
                    &frontier,
                    &mut work,
                )?;
                remaining = remaining_after(remaining, frontier.time)?;
                work.ballistic_impacts = work
                    .ballistic_impacts
                    .saturating_add(u64::try_from(impacts.len()).unwrap_or(u64::MAX));
                events.push(BallisticTimelineEvent3d::Ballistic(impacts));
                positive_events = positive_events.saturating_add(1);
            }
        }
    }

    Ok(BallisticEventTimelineReport3d { events, work })
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum SelectedEvent3d {
    None,
    Rigid,
    Ballistic,
}

fn select_event(
    rigid: Option<&crate::RotatingContactFrontier3d>,
    ballistic: Option<&BallisticFrontier3d>,
) -> SelectedEvent3d {
    match (rigid, ballistic) {
        (None, None) => SelectedEvent3d::None,
        (Some(_), None) => SelectedEvent3d::Rigid,
        (None, Some(_)) => SelectedEvent3d::Ballistic,
        (Some(rigid), Some(ballistic)) => {
            if compare_time(rigid.time, ballistic.time) != Ordering::Greater {
                SelectedEvent3d::Rigid
            } else {
                SelectedEvent3d::Ballistic
            }
        }
    }
}

fn resolve_current_rigid_contacts(
    boxes: &mut [RigidBox3d],
    rigid: RepeatedRotatingEventConfig3d,
    broad_phase: &mut RotatingBroadPhase3d,
    work: &mut BallisticEventTimelineWorkStats3d,
) -> Result<(), BallisticEventTimelineError3d> {
    let zero_search = RotatingContactSearchConfig3d {
        free_flight: RigidBoxFreeFlightConfig3d::new(rigid.search.free_flight.gravity, 0, 1),
        ..rigid.search
    };
    let progress = advance_repeated_rotating_events_with_broad_phase(
        boxes,
        RepeatedRotatingEventConfig3d {
            search: zero_search,
            ..rigid
        },
        broad_phase,
    )?;
    if !progress.events.is_empty() {
        work.zero_time_rigid_resolutions = work.zero_time_rigid_resolutions.saturating_add(1);
    }
    accumulate_rigid_work(&mut work.rigid, progress.work);
    Ok(())
}

fn earliest_ballistic_frontier(
    boxes: &[RigidBox3d],
    projectiles: &[BallisticSphere3d],
    remaining: RigidBoxFreeFlightConfig3d,
    work: &mut BallisticEventTimelineWorkStats3d,
) -> Result<Option<BallisticFrontier3d>, BallisticEventTimelineError3d> {
    if projectiles.is_empty() {
        return Ok(None);
    }
    work.ballistic_query_rounds = work.ballistic_query_rounds.saturating_add(1);

    let prepared_targets = projected_targets(boxes, remaining)?;
    let scene = BallisticSphereScene3d::prepare(prepared_targets.iter())?;
    let step = scene.prepare_step(1, 1)?;
    work.ballistic_target_bound_checks = work.ballistic_target_bound_checks.saturating_add(
        u64::try_from(scene.target_count().saturating_mul(projectiles.len())).unwrap_or(u64::MAX),
    );

    let mut earliest_time = None;
    let mut candidates = Vec::new();
    for projectile in projectiles.iter().copied() {
        let query = projected_projectile(projectile, remaining)?;
        let mut stats = BallisticSphereQueryStats3d::default();
        let hit = step.earliest_hit(query, &mut stats)?;
        accumulate_ballistic_query_stats(work, stats);
        let Some(hit) = hit else {
            continue;
        };
        let time = ballistic_time_to_sampled(hit.time.fraction_subticks())?;
        match earliest_time {
            None => {
                earliest_time = Some(time);
                candidates.clear();
                candidates.push(BallisticCandidate3d {
                    projectile: projectile.id(),
                    hit,
                });
            }
            Some(current) if compare_time(time, current) == Ordering::Less => {
                earliest_time = Some(time);
                candidates.clear();
                candidates.push(BallisticCandidate3d {
                    projectile: projectile.id(),
                    hit,
                });
            }
            Some(current) if compare_time(time, current) == Ordering::Equal => {
                candidates.push(BallisticCandidate3d {
                    projectile: projectile.id(),
                    hit,
                });
            }
            Some(_) => {}
        }
    }
    let Some(time) = earliest_time else {
        return Ok(None);
    };
    candidates.sort_by_key(|candidate| (candidate.projectile, candidate.hit.body));
    Ok(Some(BallisticFrontier3d {
        time,
        hits: candidates,
    }))
}

fn projected_targets(
    boxes: &[RigidBox3d],
    remaining: RigidBoxFreeFlightConfig3d,
) -> Result<Vec<RigidBox3d>, BallisticEventTimelineError3d> {
    boxes
        .iter()
        .map(|rigid_box| {
            let end = sample_rigid_box_free_flight(rigid_box, remaining, 1, 1)?;
            let mut projected = rigid_box.clone();
            projected.body.velocity =
                displacement_velocity(rigid_box.body.position, end.body.position)?;
            Ok(projected)
        })
        .collect()
}

fn projected_projectile(
    projectile: BallisticSphere3d,
    remaining: RigidBoxFreeFlightConfig3d,
) -> Result<BallisticSphere3d, BallisticEventTimelineError3d> {
    let mut end = projectile;
    advance_projectile_exact(&mut end, remaining)?;
    let mut projected = projectile;
    projected.set_velocity(displacement_velocity(
        projectile.position(),
        end.position(),
    )?);
    Ok(projected)
}

fn displacement_velocity(start: Vec3i, end: Vec3i) -> Result<Vec3i, BallisticEventTimelineError3d> {
    Ok(Vec3i::new(
        difference_i32(end.x, start.x)?,
        difference_i32(end.y, start.y)?,
        difference_i32(end.z, start.z)?,
    ))
}

fn difference_i32(left: i32, right: i32) -> Result<i32, BallisticEventTimelineError3d> {
    i32::try_from(i64::from(left) - i64::from(right))
        .map_err(|_| BallisticEventTimelineError3d::ArithmeticOverflow)
}

fn resolve_ballistic_frontier(
    boxes: &mut [RigidBox3d],
    projectiles: &mut Vec<BallisticSphere3d>,
    remaining: RigidBoxFreeFlightConfig3d,
    frontier: &BallisticFrontier3d,
    work: &mut BallisticEventTimelineWorkStats3d,
) -> Result<Vec<BallisticResolvedImpact3d>, BallisticEventTimelineError3d> {
    let updates = stage_box_motion_updates(boxes, remaining, frontier.time, work)?;
    let staged_projectiles =
        staged_projectiles_after_fraction(projectiles, remaining, frontier.time, work)?;
    let projectile_by_id = staged_projectiles
        .iter()
        .copied()
        .map(|projectile| (projectile.id(), projectile))
        .collect::<BTreeMap<_, _>>();

    let mut target_states = BTreeMap::<BodyId, RigidBox3d>::new();
    for candidate in &frontier.hits {
        if target_states.contains_key(&candidate.hit.body) {
            continue;
        }
        let world_index = boxes
            .iter()
            .position(|rigid_box| rigid_box.body().id() == candidate.hit.body)
            .ok_or(BallisticEventTimelineError3d::MissingTarget(
                candidate.hit.body,
            ))?;
        let mut target = boxes[world_index].clone();
        if let Some(update) = updates
            .iter()
            .find(|update| update.world_index == world_index)
        {
            update.state.apply(&mut target);
        }
        target_states.insert(candidate.hit.body, target);
    }

    let mut impacts = Vec::with_capacity(frontier.hits.len());
    for candidate in &frontier.hits {
        let projectile = *projectile_by_id.get(&candidate.projectile).ok_or(
            BallisticEventTimelineError3d::DuplicateBody(candidate.projectile),
        )?;
        let target = target_states.get_mut(&candidate.hit.body).ok_or(
            BallisticEventTimelineError3d::MissingTarget(candidate.hit.body),
        )?;
        let normal_impulse = apply_ballistic_target_impulse(projectile, target, candidate.hit)?;
        impacts.push(BallisticResolvedImpact3d {
            projectile: candidate.projectile,
            target: candidate.hit.body,
            time: frontier.time,
            normal: candidate.hit.normal,
            normal_impulse,
        });
    }

    for update in updates {
        update.state.apply(
            boxes
                .get_mut(update.world_index)
                .expect("staged world index remains valid for one timeline event"),
        );
    }
    for (id, target) in target_states {
        let rigid_box = boxes
            .iter_mut()
            .find(|rigid_box| rigid_box.body().id() == id)
            .ok_or(BallisticEventTimelineError3d::MissingTarget(id))?;
        *rigid_box = target;
    }

    let retired = frontier
        .hits
        .iter()
        .map(|candidate| candidate.projectile)
        .collect::<BTreeSet<_>>();
    *projectiles = staged_projectiles
        .into_iter()
        .filter(|projectile| !retired.contains(&projectile.id()))
        .collect();
    Ok(impacts)
}

fn stage_box_motion_updates(
    boxes: &[RigidBox3d],
    remaining: RigidBoxFreeFlightConfig3d,
    time: SampledContactTime3d,
    work: &mut BallisticEventTimelineWorkStats3d,
) -> Result<Vec<MotionUpdate3d>, BallisticEventTimelineError3d> {
    let mut updates = Vec::new();
    for (world_index, rigid_box) in boxes.iter().enumerate() {
        work.rigid_body_motion_samples = work.rigid_body_motion_samples.saturating_add(1);
        let sampled =
            sample_rigid_box_free_flight(rigid_box, remaining, time.numerator, time.denominator)?;
        let before = MotionState3d::from_box(rigid_box);
        let after = MotionState3d::from_box(&sampled);
        if before != after {
            updates.push(MotionUpdate3d {
                world_index,
                state: after,
            });
        }
    }
    Ok(updates)
}

fn advance_boxes_to_fraction(
    boxes: &mut [RigidBox3d],
    remaining: RigidBoxFreeFlightConfig3d,
    time: SampledContactTime3d,
    work: &mut BallisticEventTimelineWorkStats3d,
) -> Result<(), BallisticEventTimelineError3d> {
    let updates = stage_box_motion_updates(boxes, remaining, time, work)?;
    for update in updates {
        update.state.apply(
            boxes
                .get_mut(update.world_index)
                .expect("staged world index remains valid for one free-flight commit"),
        );
    }
    Ok(())
}

fn staged_projectiles_after_fraction(
    projectiles: &[BallisticSphere3d],
    remaining: RigidBoxFreeFlightConfig3d,
    time: SampledContactTime3d,
    work: &mut BallisticEventTimelineWorkStats3d,
) -> Result<Vec<BallisticSphere3d>, BallisticEventTimelineError3d> {
    let segment = remaining.scaled_fraction(time.numerator, time.denominator)?;
    let mut staged = projectiles.to_vec();
    for projectile in &mut staged {
        work.projectile_motion_samples = work.projectile_motion_samples.saturating_add(1);
        advance_projectile_exact(projectile, segment)?;
    }
    Ok(staged)
}

fn advance_projectiles_to_fraction(
    projectiles: &mut Vec<BallisticSphere3d>,
    remaining: RigidBoxFreeFlightConfig3d,
    time: SampledContactTime3d,
    work: &mut BallisticEventTimelineWorkStats3d,
) -> Result<(), BallisticEventTimelineError3d> {
    *projectiles = staged_projectiles_after_fraction(projectiles, remaining, time, work)?;
    Ok(())
}

fn advance_projectile_exact(
    projectile: &mut BallisticSphere3d,
    free_flight: RigidBoxFreeFlightConfig3d,
) -> Result<(), BallisticEventTimelineError3d> {
    let timestep = free_flight.exact_timestep()?;
    if timestep.is_zero() {
        return Ok(());
    }
    let mut velocity = projectile.velocity();
    let mut position = projectile.position();
    for axis in 0..3 {
        let next_velocity = i128::from(component(velocity, axis))
            .checked_add(timestep.mul_round_i128(i128::from(component(free_flight.gravity, axis)))?)
            .ok_or(BallisticEventTimelineError3d::ArithmeticOverflow)?;
        let next_velocity = i32::try_from(next_velocity)
            .map_err(|_| BallisticEventTimelineError3d::ArithmeticOverflow)?;
        set_component(&mut velocity, axis, next_velocity);
        let next_position = i128::from(component(position, axis))
            .checked_add(timestep.mul_round_i128(i128::from(next_velocity))?)
            .ok_or(BallisticEventTimelineError3d::ArithmeticOverflow)?;
        set_component(
            &mut position,
            axis,
            i32::try_from(next_position)
                .map_err(|_| BallisticEventTimelineError3d::ArithmeticOverflow)?,
        );
    }
    projectile.set_velocity(velocity);
    projectile.set_position(position);
    Ok(())
}

fn apply_ballistic_target_impulse(
    projectile: BallisticSphere3d,
    target: &mut RigidBox3d,
    hit: BallisticSphereSweepHit3d,
) -> Result<[i128; 3], BallisticEventTimelineError3d> {
    if target.body.kind == BodyKind::Fixed {
        return Ok([0; 3]);
    }
    let axis = primitive_axis(hit.normal)?;
    let axis_length_squared = axis_length_squared(axis)?;
    let point = projectile_contact_point(projectile, axis)?;
    let target_offset = vector_delta(target.body.position, point)?;
    let target_velocity = contact_velocity(target, target_offset)?;
    let relative_velocity = [
        i128::from(projectile.velocity().x) - i128::from(target_velocity[0]),
        i128::from(projectile.velocity().y) - i128::from(target_velocity[1]),
        i128::from(projectile.velocity().z) - i128::from(target_velocity[2]),
    ];
    let normal_velocity = checked_dot(relative_velocity, axis)?;
    if normal_velocity >= 0 {
        return Ok([0; 3]);
    }

    let sphere_inverse_mass = i128::try_from(mul_div_round_u128(
        axis_length_squared,
        RESPONSE_SCALE as u128,
        u128::from(projectile.mass_units()),
    )?)
    .map_err(|_| BallisticEventTimelineError3d::ArithmeticOverflow)?;
    let target_inverse_mass =
        body_effective_inverse_mass_scaled(target, target_offset, axis, axis_length_squared)?;
    let effective_inverse_mass = sphere_inverse_mass
        .checked_add(target_inverse_mass)
        .ok_or(BallisticEventTimelineError3d::ArithmeticOverflow)?;
    if effective_inverse_mass <= 0 {
        return Ok([0; 3]);
    }

    let restitution = projectile
        .material()
        .restitution_milli()
        .min(target.body.material.restitution_milli());
    let closing_speed = normal_velocity
        .checked_neg()
        .ok_or(BallisticEventTimelineError3d::ArithmeticOverflow)?;
    let numerator = checked_mul(
        closing_speed,
        i128::from(MATERIAL_SCALE)
            .checked_add(i128::from(restitution))
            .ok_or(BallisticEventTimelineError3d::ArithmeticOverflow)?,
    )?;
    let denominator = checked_mul(i128::from(MATERIAL_SCALE), effective_inverse_mass)?;
    let mut impulse = [0_i128; 3];
    for (output, axis_component) in impulse.iter_mut().zip(axis) {
        *output = mul_div_round_i128(
            numerator,
            checked_mul(axis_component, RESPONSE_SCALE)?,
            denominator,
        )?;
    }
    apply_body_impulse(target, target_offset, negate_axis(impulse)?)?;
    Ok(impulse)
}

fn body_effective_inverse_mass_scaled(
    rigid_box: &RigidBox3d,
    contact_offset: [i64; 3],
    axis: [i128; 3],
    axis_length_squared: u128,
) -> Result<i128, BallisticEventTimelineError3d> {
    if rigid_box.body.kind == BodyKind::Fixed {
        return Ok(0);
    }
    let translational = i128::try_from(mul_div_round_u128(
        axis_length_squared,
        RESPONSE_SCALE as u128,
        u128::from(rigid_box.body.mass_units),
    )?)
    .map_err(|_| BallisticEventTimelineError3d::ArithmeticOverflow)?;
    if rigid_box.rotation_locked {
        return Ok(translational);
    }
    let angular_impulse = cross_i64_i128(contact_offset, axis)?;
    let local = rotate_inverse(rigid_box.angular.orientation, angular_impulse)?;
    let inertia = box_inertia(&rigid_box.body)?;
    let scale = (RESPONSE_SCALE as u128)
        .checked_mul(u128::from(inertia.denominator))
        .ok_or(BallisticEventTimelineError3d::ArithmeticOverflow)?;
    let mut rotational = 0_i128;
    for (index, value) in local.into_iter().enumerate() {
        let squared = checked_mul(value, value)?;
        let term = i128::try_from(mul_div_round_u128(
            squared as u128,
            scale,
            inertia.principal_numerators[index],
        )?)
        .map_err(|_| BallisticEventTimelineError3d::ArithmeticOverflow)?;
        rotational = rotational
            .checked_add(term)
            .ok_or(BallisticEventTimelineError3d::ArithmeticOverflow)?;
    }
    translational
        .checked_add(rotational)
        .ok_or(BallisticEventTimelineError3d::ArithmeticOverflow)
}

fn apply_body_impulse(
    rigid_box: &mut RigidBox3d,
    contact_offset: [i64; 3],
    impulse: [i128; 3],
) -> Result<(), BallisticEventTimelineError3d> {
    if rigid_box.body.kind == BodyKind::Fixed {
        return Ok(());
    }
    let mass = i128::from(rigid_box.body.mass_units);
    rigid_box.body.velocity = Vec3i::new(
        add_impulse_axis(rigid_box.body.velocity.x, impulse[0], mass)?,
        add_impulse_axis(rigid_box.body.velocity.y, impulse[1], mass)?,
        add_impulse_axis(rigid_box.body.velocity.z, impulse[2], mass)?,
    );
    if rigid_box.rotation_locked {
        return Ok(());
    }
    let angular_impulse = cross_i64_i128(contact_offset, impulse)?;
    let local_impulse = rotate_inverse(rigid_box.angular.orientation, angular_impulse)?;
    let inertia = box_inertia(&rigid_box.body)?;
    let angular_scale = i128::from(inertia.denominator)
        .checked_mul(i128::from(ANGULAR_VELOCITY_SCALE))
        .ok_or(BallisticEventTimelineError3d::ArithmeticOverflow)?;
    let mut local_delta = [0_i128; 3];
    for (index, (output, value)) in local_delta.iter_mut().zip(local_impulse).enumerate() {
        let principal = i128::try_from(inertia.principal_numerators[index])
            .map_err(|_| BallisticEventTimelineError3d::ArithmeticOverflow)?;
        *output = mul_div_round_i128(value, angular_scale, principal)?;
    }
    let world_delta = rotate_forward(rigid_box.angular.orientation, local_delta)?;
    rigid_box.angular.angular_velocity = AngularVelocity3d::new(
        add_angular_axis(rigid_box.angular.angular_velocity.x, world_delta[0])?,
        add_angular_axis(rigid_box.angular.angular_velocity.y, world_delta[1])?,
        add_angular_axis(rigid_box.angular.angular_velocity.z, world_delta[2])?,
    );
    Ok(())
}

fn projectile_contact_point(
    projectile: BallisticSphere3d,
    normal: [i128; 3],
) -> Result<Vec3i, BallisticEventTimelineError3d> {
    let length_squared = axis_length_squared(normal)?;
    let length = integer_sqrt(length_squared).max(1);
    let length =
        i128::try_from(length).map_err(|_| BallisticEventTimelineError3d::ArithmeticOverflow)?;
    let radius = i128::from(projectile.radius());
    let center = projectile.position();
    let mut point = [
        i128::from(center.x),
        i128::from(center.y),
        i128::from(center.z),
    ];
    for axis in 0..3 {
        let offset = div_round_nearest(checked_mul(radius, normal[axis])?, length)?;
        point[axis] = point[axis]
            .checked_sub(offset)
            .ok_or(BallisticEventTimelineError3d::ArithmeticOverflow)?;
    }
    Ok(Vec3i::new(
        to_i32(point[0])?,
        to_i32(point[1])?,
        to_i32(point[2])?,
    ))
}

fn contact_velocity(
    rigid_box: &RigidBox3d,
    offset: [i64; 3],
) -> Result<[i64; 3], BallisticEventTimelineError3d> {
    let omega = if rigid_box.rotation_locked {
        AngularVelocity3d::default()
    } else {
        rigid_box.angular.angular_velocity
    };
    let rotation = [
        checked_sub(
            checked_mul(i128::from(omega.y), i128::from(offset[2]))?,
            checked_mul(i128::from(omega.z), i128::from(offset[1]))?,
        )?,
        checked_sub(
            checked_mul(i128::from(omega.z), i128::from(offset[0]))?,
            checked_mul(i128::from(omega.x), i128::from(offset[2]))?,
        )?,
        checked_sub(
            checked_mul(i128::from(omega.x), i128::from(offset[1]))?,
            checked_mul(i128::from(omega.y), i128::from(offset[0]))?,
        )?,
    ];
    let linear = rigid_box.body.velocity;
    Ok([
        add_linear_rotation(linear.x, rotation[0])?,
        add_linear_rotation(linear.y, rotation[1])?,
        add_linear_rotation(linear.z, rotation[2])?,
    ])
}

fn add_linear_rotation(
    linear: i32,
    rotational_numerator: i128,
) -> Result<i64, BallisticEventTimelineError3d> {
    let rotational = i64::try_from(div_round_nearest(
        rotational_numerator,
        i128::from(ANGULAR_VELOCITY_SCALE),
    )?)
    .map_err(|_| BallisticEventTimelineError3d::ArithmeticOverflow)?;
    i64::from(linear)
        .checked_add(rotational)
        .ok_or(BallisticEventTimelineError3d::ArithmeticOverflow)
}

fn vector_delta(center: Vec3i, point: Vec3i) -> Result<[i64; 3], BallisticEventTimelineError3d> {
    Ok([
        i64::from(point.x) - i64::from(center.x),
        i64::from(point.y) - i64::from(center.y),
        i64::from(point.z) - i64::from(center.z),
    ])
}

fn rotate_inverse(
    orientation: Orientation3d,
    vector: [i128; 3],
) -> Result<[i128; 3], BallisticEventTimelineError3d> {
    let matrix = rotation_matrix(orientation.normalized()?)?;
    rotate_with_matrix(
        [
            [matrix[0][0], matrix[1][0], matrix[2][0]],
            [matrix[0][1], matrix[1][1], matrix[2][1]],
            [matrix[0][2], matrix[1][2], matrix[2][2]],
        ],
        vector,
    )
}

fn rotate_forward(
    orientation: Orientation3d,
    vector: [i128; 3],
) -> Result<[i128; 3], BallisticEventTimelineError3d> {
    rotate_with_matrix(rotation_matrix(orientation.normalized()?)?, vector)
}

fn rotation_matrix(
    orientation: Orientation3d,
) -> Result<[[i128; 3]; 3], BallisticEventTimelineError3d> {
    let x = i128::from(orientation.x);
    let y = i128::from(orientation.y);
    let z = i128::from(orientation.z);
    let w = i128::from(orientation.w);
    let scale = i128::from(ORIENTATION_SCALE);
    let xx = checked_mul(x, x)?;
    let yy = checked_mul(y, y)?;
    let zz = checked_mul(z, z)?;
    let xy = checked_mul(x, y)?;
    let xz = checked_mul(x, z)?;
    let yz = checked_mul(y, z)?;
    let xw = checked_mul(x, w)?;
    let yw = checked_mul(y, w)?;
    let zw = checked_mul(z, w)?;
    Ok([
        [
            checked_sub(scale, scaled_twice(checked_add(yy, zz)?, scale)?)?,
            scaled_twice(checked_sub(xy, zw)?, scale)?,
            scaled_twice(checked_add(xz, yw)?, scale)?,
        ],
        [
            scaled_twice(checked_add(xy, zw)?, scale)?,
            checked_sub(scale, scaled_twice(checked_add(xx, zz)?, scale)?)?,
            scaled_twice(checked_sub(yz, xw)?, scale)?,
        ],
        [
            scaled_twice(checked_sub(xz, yw)?, scale)?,
            scaled_twice(checked_add(yz, xw)?, scale)?,
            checked_sub(scale, scaled_twice(checked_add(xx, yy)?, scale)?)?,
        ],
    ])
}

fn rotate_with_matrix(
    matrix: [[i128; 3]; 3],
    vector: [i128; 3],
) -> Result<[i128; 3], BallisticEventTimelineError3d> {
    let scale = i128::from(ORIENTATION_SCALE);
    let mut output = [0_i128; 3];
    for (target, row) in output.iter_mut().zip(matrix) {
        *target = div_round_nearest(
            checked_add(
                checked_add(
                    checked_mul(row[0], vector[0])?,
                    checked_mul(row[1], vector[1])?,
                )?,
                checked_mul(row[2], vector[2])?,
            )?,
            scale,
        )?;
    }
    Ok(output)
}

fn scaled_twice(value: i128, scale: i128) -> Result<i128, BallisticEventTimelineError3d> {
    div_round_nearest(checked_mul(value, 2)?, scale)
}

fn cross_i64_i128(
    left: [i64; 3],
    right: [i128; 3],
) -> Result<[i128; 3], BallisticEventTimelineError3d> {
    let left = left.map(i128::from);
    Ok([
        checked_sub(
            checked_mul(left[1], right[2])?,
            checked_mul(left[2], right[1])?,
        )?,
        checked_sub(
            checked_mul(left[2], right[0])?,
            checked_mul(left[0], right[2])?,
        )?,
        checked_sub(
            checked_mul(left[0], right[1])?,
            checked_mul(left[1], right[0])?,
        )?,
    ])
}

fn primitive_axis(axis: [i128; 3]) -> Result<[i128; 3], BallisticEventTimelineError3d> {
    let divisor = axis
        .iter()
        .map(|value| value.unsigned_abs())
        .fold(0_u128, gcd_u128);
    if divisor == 0 {
        return Err(BallisticEventTimelineError3d::ArithmeticOverflow);
    }
    let divisor =
        i128::try_from(divisor).map_err(|_| BallisticEventTimelineError3d::ArithmeticOverflow)?;
    Ok([axis[0] / divisor, axis[1] / divisor, axis[2] / divisor])
}

fn axis_length_squared(axis: [i128; 3]) -> Result<u128, BallisticEventTimelineError3d> {
    axis.into_iter().try_fold(0_u128, |sum, value| {
        let magnitude = value.unsigned_abs();
        sum.checked_add(
            magnitude
                .checked_mul(magnitude)
                .ok_or(BallisticEventTimelineError3d::ArithmeticOverflow)?,
        )
        .ok_or(BallisticEventTimelineError3d::ArithmeticOverflow)
    })
}

fn ballistic_time_to_sampled(
    fraction_subticks: u64,
) -> Result<SampledContactTime3d, BallisticEventTimelineError3d> {
    if fraction_subticks == 0 || fraction_subticks > BALLISTIC_TIME_SCALE {
        return Err(BallisticEventTimelineError3d::ArithmeticOverflow);
    }
    let numerator = fraction_subticks.div_ceil(2);
    let numerator =
        u32::try_from(numerator).map_err(|_| BallisticEventTimelineError3d::ArithmeticOverflow)?;
    let divisor = gcd_u32(numerator, TIMELINE_TIME_SCALE);
    Ok(SampledContactTime3d {
        numerator: numerator / divisor,
        denominator: TIMELINE_TIME_SCALE / divisor,
    })
}

fn full_time() -> SampledContactTime3d {
    SampledContactTime3d {
        numerator: 1,
        denominator: 1,
    }
}

fn remaining_after(
    remaining: RigidBoxFreeFlightConfig3d,
    time: SampledContactTime3d,
) -> Result<RigidBoxFreeFlightConfig3d, BallisticEventTimelineError3d> {
    if time.denominator == 0 || time.numerator > time.denominator {
        return Err(BallisticEventTimelineError3d::ArithmeticOverflow);
    }
    Ok(remaining.scaled_fraction(time.denominator - time.numerator, time.denominator)?)
}

fn compare_time(left: SampledContactTime3d, right: SampledContactTime3d) -> Ordering {
    (u128::from(left.numerator) * u128::from(right.denominator))
        .cmp(&(u128::from(right.numerator) * u128::from(left.denominator)))
}

fn validate_unique_ids(
    boxes: &[RigidBox3d],
    projectiles: &[BallisticSphere3d],
) -> Result<(), BallisticEventTimelineError3d> {
    let mut ids = BTreeSet::new();
    for id in boxes
        .iter()
        .map(|rigid_box| rigid_box.body().id())
        .chain(projectiles.iter().copied().map(BallisticSphere3d::id))
    {
        if !ids.insert(id) {
            return Err(BallisticEventTimelineError3d::DuplicateBody(id));
        }
    }
    Ok(())
}

fn accumulate_ballistic_query_stats(
    work: &mut BallisticEventTimelineWorkStats3d,
    stats: BallisticSphereQueryStats3d,
) {
    work.ballistic_broad_phase_candidates = work
        .ballistic_broad_phase_candidates
        .saturating_add(stats.broad_phase_candidates);
    work.ballistic_toi_tests = work.ballistic_toi_tests.saturating_add(stats.toi_tests);
    work.ballistic_feature_tests = work
        .ballistic_feature_tests
        .saturating_add(stats.feature_tests);
}

fn accumulate_response_passes(work: &mut RepeatedRotatingEventWorkStats3d, passes: u8) {
    work.event_response_passes = work.event_response_passes.saturating_add(u64::from(passes));
}

fn accumulate_rigid_work(
    target: &mut RepeatedRotatingEventWorkStats3d,
    source: RepeatedRotatingEventWorkStats3d,
) {
    target.event_response_passes = target
        .event_response_passes
        .saturating_add(source.event_response_passes);
    target.stabilization_passes = target
        .stabilization_passes
        .saturating_add(source.stabilization_passes);
    target.stabilizations_hitting_limit = target
        .stabilizations_hitting_limit
        .saturating_add(source.stabilizations_hitting_limit);
    target.stabilization_candidate_pairs = target
        .stabilization_candidate_pairs
        .saturating_add(source.stabilization_candidate_pairs);
    target.stabilization_exact_contacts = target
        .stabilization_exact_contacts
        .saturating_add(source.stabilization_exact_contacts);
    target.stabilization_active_bodies = target
        .stabilization_active_bodies
        .saturating_add(source.stabilization_active_bodies);
}

fn add_impulse_axis(
    current: i32,
    impulse: i128,
    mass: i128,
) -> Result<i32, BallisticEventTimelineError3d> {
    to_i32(
        i128::from(current)
            .checked_add(div_round_nearest(impulse, mass)?)
            .ok_or(BallisticEventTimelineError3d::ArithmeticOverflow)?,
    )
}

fn add_angular_axis(current: i32, delta: i128) -> Result<i32, BallisticEventTimelineError3d> {
    to_i32(
        i128::from(current)
            .checked_add(delta)
            .ok_or(BallisticEventTimelineError3d::ArithmeticOverflow)?,
    )
}

fn negate_axis(vector: [i128; 3]) -> Result<[i128; 3], BallisticEventTimelineError3d> {
    Ok([
        vector[0]
            .checked_neg()
            .ok_or(BallisticEventTimelineError3d::ArithmeticOverflow)?,
        vector[1]
            .checked_neg()
            .ok_or(BallisticEventTimelineError3d::ArithmeticOverflow)?,
        vector[2]
            .checked_neg()
            .ok_or(BallisticEventTimelineError3d::ArithmeticOverflow)?,
    ])
}

fn checked_add(left: i128, right: i128) -> Result<i128, BallisticEventTimelineError3d> {
    left.checked_add(right)
        .ok_or(BallisticEventTimelineError3d::ArithmeticOverflow)
}

fn checked_sub(left: i128, right: i128) -> Result<i128, BallisticEventTimelineError3d> {
    left.checked_sub(right)
        .ok_or(BallisticEventTimelineError3d::ArithmeticOverflow)
}

fn checked_mul(left: i128, right: i128) -> Result<i128, BallisticEventTimelineError3d> {
    left.checked_mul(right)
        .ok_or(BallisticEventTimelineError3d::ArithmeticOverflow)
}

fn checked_dot(left: [i128; 3], right: [i128; 3]) -> Result<i128, BallisticEventTimelineError3d> {
    checked_add(
        checked_add(
            checked_mul(left[0], right[0])?,
            checked_mul(left[1], right[1])?,
        )?,
        checked_mul(left[2], right[2])?,
    )
}

fn div_round_nearest(
    numerator: i128,
    denominator: i128,
) -> Result<i128, BallisticEventTimelineError3d> {
    if denominator <= 0 {
        return Err(BallisticEventTimelineError3d::ArithmeticOverflow);
    }
    let half = denominator / 2;
    let adjusted = if numerator >= 0 {
        numerator.checked_add(half)
    } else {
        numerator.checked_sub(half)
    }
    .ok_or(BallisticEventTimelineError3d::ArithmeticOverflow)?;
    Ok(adjusted / denominator)
}

fn to_i32(value: i128) -> Result<i32, BallisticEventTimelineError3d> {
    i32::try_from(value).map_err(|_| BallisticEventTimelineError3d::ArithmeticOverflow)
}

fn component(vector: Vec3i, axis: usize) -> i32 {
    match axis {
        0 => vector.x,
        1 => vector.y,
        _ => vector.z,
    }
}

fn set_component(vector: &mut Vec3i, axis: usize, value: i32) {
    match axis {
        0 => vector.x = value,
        1 => vector.y = value,
        _ => vector.z = value,
    }
}

fn gcd_u32(mut left: u32, mut right: u32) -> u32 {
    while right != 0 {
        let remainder = left % right;
        left = right;
        right = remainder;
    }
    left.max(1)
}

fn gcd_u128(mut left: u128, mut right: u128) -> u128 {
    while right != 0 {
        let remainder = left % right;
        left = right;
        right = remainder;
    }
    left
}

fn integer_sqrt(value: u128) -> u128 {
    if value < 2 {
        return value;
    }
    let mut low = 1_u128;
    let mut high = value.min(u128::from(u64::MAX));
    while low <= high {
        let middle = low + (high - low) / 2;
        match middle.checked_mul(middle).map(|square| square.cmp(&value)) {
            Some(Ordering::Equal) => return middle,
            Some(Ordering::Less) => low = middle.saturating_add(1),
            Some(Ordering::Greater) | None => high = middle.saturating_sub(1),
        }
    }
    high
}

#[cfg(test)]
mod tests {
    use crate::{
        AngularState3d, AngularVelocity3d, BallisticSphere3d, BodyId, CollisionLayers3d, Material,
        Orientation3d, RepeatedRotatingEventConfig3d, RigidBody, RigidBox3d,
        RigidBoxFreeFlightConfig3d, RotatingContactSearchConfig3d, Vec3i,
    };

    use super::{
        BallisticEventTimelineConfig3d, BallisticTimelineEvent3d,
        advance_ballistic_event_timeline, ballistic_time_to_sampled,
    };

    fn rigid_config() -> BallisticEventTimelineConfig3d {
        BallisticEventTimelineConfig3d::new(RepeatedRotatingEventConfig3d::new(
            RotatingContactSearchConfig3d::new(
                RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 1, 1),
                32,
                4,
            ),
            8,
            32,
        ))
    }

    fn fixed(
        id: u64,
        position: Vec3i,
        half_extents: Vec3i,
        layers: CollisionLayers3d,
    ) -> RigidBox3d {
        RigidBox3d::new(
            RigidBody::fixed(BodyId(id), position, half_extents),
            AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
        )
        .expect("valid fixed box")
        .with_collision_layers(layers)
    }

    fn dynamic(
        id: u64,
        position: Vec3i,
        velocity: Vec3i,
        half_extents: Vec3i,
        layers: CollisionLayers3d,
    ) -> RigidBox3d {
        RigidBox3d::new(
            RigidBody::dynamic(BodyId(id), position, velocity, half_extents)
                .with_mass(4)
                .with_material(Material::new(0)),
            AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
        )
        .expect("valid dynamic box")
        .with_collision_layers(layers)
    }

    #[test]
    fn adjacent_q32_hits_share_the_same_conservative_q31_slot() {
        let first = ballistic_time_to_sampled(1).expect("first sub-tick");
        let second = ballistic_time_to_sampled(2).expect("second sub-tick");
        assert_eq!(first, second);
    }

    #[test]
    fn off_center_single_impact_retires_projectile_and_spins_dynamic_target() {
        let layers = CollisionLayers3d::ALL;
        let mut boxes = vec![dynamic(
            1,
            Vec3i::ZERO,
            Vec3i::ZERO,
            Vec3i::new(5, 5, 5),
            layers,
        )];
        let mut projectiles = vec![
            BallisticSphere3d::new(
                BodyId(100),
                Vec3i::new(-30, 4, 0),
                Vec3i::new(60, 0, 0),
                2,
                1,
            )
            .expect("valid projectile")
            .with_collision_layers(layers),
        ];

        let report = advance_ballistic_event_timeline(&mut boxes, &mut projectiles, rigid_config())
            .expect("ballistic timeline");

        assert!(projectiles.is_empty());
        assert_eq!(report.work.ballistic_impacts, 1);
        assert!(boxes[0].body().velocity().x > 0);
        assert_ne!(boxes[0].angular().angular_velocity.z, 0);
        assert!(matches!(
            report.events[0],
            BallisticTimelineEvent3d::Ballistic(_)
        ));
    }

    #[test]
    fn earlier_rigid_contact_is_resolved_before_later_ballistic_impact() {
        let rigid_layers = CollisionLayers3d::new(0b0001, 0b0001);
        let target_layers = CollisionLayers3d::new(0b0010, 0b0100);
        let projectile_layers = CollisionLayers3d::new(0b0100, 0b0010);
        let mut boxes = vec![
            fixed(1, Vec3i::new(10, 0, 0), Vec3i::new(1, 4, 4), rigid_layers),
            dynamic(
                2,
                Vec3i::ZERO,
                Vec3i::new(100, 0, 0),
                Vec3i::new(1, 1, 1),
                rigid_layers,
            ),
            fixed(
                3,
                Vec3i::new(100, 30, 0),
                Vec3i::new(5, 5, 5),
                target_layers,
            ),
        ];
        let mut projectiles = vec![
            BallisticSphere3d::new(
                BodyId(100),
                Vec3i::new(40, 30, 0),
                Vec3i::new(100, 0, 0),
                2,
                1,
            )
            .expect("valid projectile")
            .with_collision_layers(projectile_layers),
        ];

        let report = advance_ballistic_event_timeline(&mut boxes, &mut projectiles, rigid_config())
            .expect("mixed timeline");

        assert!(report.events.len() >= 2);
        assert!(matches!(
            report.events[0],
            BallisticTimelineEvent3d::Rigid(_)
        ));
        assert!(matches!(
            report.events[1],
            BallisticTimelineEvent3d::Ballistic(_)
        ));
        assert!(projectiles.is_empty());
    }

    #[test]
    fn simultaneous_ballistic_hits_share_one_frontier_and_one_advance() {
        let layers = CollisionLayers3d::ALL;
        let mut boxes = vec![fixed(1, Vec3i::ZERO, Vec3i::new(5, 50, 5), layers)];
        let mut projectiles = (0..32)
            .map(|index| {
                BallisticSphere3d::new(
                    BodyId(100 + index),
                    Vec3i::new(-30, index as i32 - 16, 0),
                    Vec3i::new(60, 0, 0),
                    2,
                    1,
                )
                .expect("valid projectile")
                .with_collision_layers(layers)
            })
            .collect::<Vec<_>>();

        let report = advance_ballistic_event_timeline(&mut boxes, &mut projectiles, rigid_config())
            .expect("simultaneous ballistic frontier");

        assert!(projectiles.is_empty());
        assert_eq!(report.work.ballistic_impacts, 32);
        assert_eq!(report.work.ballistic_query_rounds, 1);
        let BallisticTimelineEvent3d::Ballistic(impacts) = &report.events[0] else {
            panic!("expected ballistic frontier");
        };
        assert_eq!(impacts.len(), 32);
    }
}
