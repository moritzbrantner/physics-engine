use std::{
    collections::{BTreeMap, BTreeSet},
    error::Error,
    fmt,
};

use crate::{
    ANGULAR_VELOCITY_SCALE, AngularVelocity3d, BodyId, BodyKind, ObbContactResponseError3d,
    ObbContactSeed3d, RigidBox3d, RigidBoxFreeFlightConfig3d, RigidBoxFreeFlightError3d,
    RotatingContactFrontier3d, RotatingContactFrontierError3d, RotatingContactResponseError3d,
    RotatingContactSearchConfig3d, RotatingContactSearchHit3d, RotationalSweepPair3d,
    SampledContactTime3d, Vec3i, obb_contact_seed, oriented_box_vertices,
};
use crate::{
    rotating_broad_phase::RotatingBroadPhase3d,
    rotating_contact_frontier::{
        earliest_rotating_contact_frontier_with_broad_phase,
        next_rotating_contact_frontier_with_broad_phase,
    },
    rotating_contact_response::{
        RotatingContactResponseScratch3d,
        resolve_rotating_contact_frontier_with_activity_and_scratch,
    },
    rotating_recontact_search::sampled_rotating_recontact_search_with_broad_phase,
};

pub const MAX_REPEATED_ROTATING_EVENTS: u16 = 64;

/// Bounded policy for advancing through sampled rotating collision events.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct RepeatedRotatingEventConfig3d {
    pub search: RotatingContactSearchConfig3d,
    pub solver_passes: u8,
    pub max_events: u16,
}

impl RepeatedRotatingEventConfig3d {
    #[must_use]
    pub const fn new(
        search: RotatingContactSearchConfig3d,
        solver_passes: u8,
        max_events: u16,
    ) -> Self {
        Self {
            search,
            solver_passes,
            max_events,
        }
    }
}

/// One resolved sampled event, expressed relative to the segment that began after the previous event.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RotatingResolvedEvent3d {
    pub time: SampledContactTime3d,
    pub contacts: Vec<RotatingContactSearchHit3d>,
    pub response_passes: u8,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct RepeatedRotatingEventWorkStats3d {
    pub event_response_passes: u64,
    pub stabilization_passes: u64,
    pub stabilizations_hitting_limit: u64,
    pub stabilization_candidate_pairs: u64,
    pub stabilization_exact_contacts: u64,
    pub stabilization_active_bodies: u64,
}

#[derive(Clone, Debug)]
struct CurrentContactStabilization3d {
    boxes: Vec<RigidBox3d>,
    observed_contacts: BTreeMap<RotationalSweepPair3d, ObbContactSeed3d>,
    exhausted_with_changes: bool,
}

#[derive(Clone, Debug, Default)]
struct ContactContinuationGraph3d {
    adjacency: BTreeMap<BodyId, BTreeSet<BodyId>>,
    projected_clear_pairs: BTreeSet<RotationalSweepPair3d>,
}

impl ContactContinuationGraph3d {
    fn add_edge(&mut self, pair: RotationalSweepPair3d, projected_clear: bool) {
        self.adjacency.entry(pair.left).or_default().insert(pair.right);
        self.adjacency.entry(pair.right).or_default().insert(pair.left);
        if projected_clear {
            self.projected_clear_pairs.insert(pair);
        }
    }

    fn has_projected_clear_component(&self) -> bool {
        !self.projected_clear_pairs.is_empty()
    }

    fn contains_selector(&self, pair: RotationalSweepPair3d) -> bool {
        if !self.adjacency.contains_key(&pair.left) || !self.adjacency.contains_key(&pair.right) {
            return false;
        }

        let mut visited = BTreeSet::new();
        let mut pending = vec![pair.left];
        while let Some(body) = pending.pop() {
            if !visited.insert(body) {
                continue;
            }
            if let Some(neighbors) = self.adjacency.get(&body) {
                pending.extend(neighbors.iter().copied());
            }
        }
        if visited.len() <= 2 || !visited.contains(&pair.right) {
            return false;
        }

        self.projected_clear_pairs
            .iter()
            .any(|projected| visited.contains(&projected.left) && visited.contains(&projected.right))
    }
}

/// Result of consuming every sampled rotating event admitted before the first unresolved tail.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RepeatedRotatingEventAdvance3d {
    /// State immediately after the last resolved sampled frontier, or the unchanged input if no event
    /// was admitted.
    pub boxes: Vec<RigidBox3d>,
    pub events: Vec<RotatingResolvedEvent3d>,
    /// Requested time that remains deliberately unconsumed.
    ///
    /// The ratio is preserved exactly in deterministic fixed-capacity limb storage. It is not rounded
    /// back to the narrower public constructor inputs, so a collision in the suffix remains part of the
    /// same requested interval and is still available to the world-level tail solver.
    ///
    /// The remaining segment is not automatically free-flown because it may contain persistent/resting
    /// contacts that the world-level tail solver must stabilize.
    pub remaining: RigidBoxFreeFlightConfig3d,
    pub work: RepeatedRotatingEventWorkStats3d,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RepeatedRotatingEventError3d {
    ZeroEventLimit,
    EventLimit(u16),
    NegativeTimestepNumerator(i128),
    NonPositiveTimestepDenominator(i128),
    InvalidRemainder(SampledContactTime3d),
    RatioTooLarge,
    Frontier(RotatingContactFrontierError3d),
    Response(RotatingContactResponseError3d),
}

impl fmt::Display for RepeatedRotatingEventError3d {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::ZeroEventLimit => write!(
                formatter,
                "repeated rotating event advance requires at least one event slot"
            ),
            Self::EventLimit(limit) => write!(
                formatter,
                "repeated rotating event advance reached its {limit}-event limit while another sampled event remained"
            ),
            Self::NegativeTimestepNumerator(value) => write!(
                formatter,
                "repeated rotating event timestep numerator must be non-negative, got {value}"
            ),
            Self::NonPositiveTimestepDenominator(value) => write!(
                formatter,
                "repeated rotating event timestep denominator must be positive, got {value}"
            ),
            Self::InvalidRemainder(time) => write!(
                formatter,
                "repeated rotating event response returned invalid remaining fraction after {}/{}",
                time.numerator, time.denominator
            ),
            Self::RatioTooLarge => write!(
                formatter,
                "repeated rotating event exact remaining-time arithmetic exceeded deterministic exact-ratio capacity"
            ),
            Self::Frontier(error) => write!(
                formatter,
                "repeated rotating event frontier failed: {error}"
            ),
            Self::Response(error) => write!(
                formatter,
                "repeated rotating event response failed: {error}"
            ),
        }
    }
}

impl Error for RepeatedRotatingEventError3d {}

impl From<RotatingContactFrontierError3d> for RepeatedRotatingEventError3d {
    fn from(value: RotatingContactFrontierError3d) -> Self {
        Self::Frontier(value)
    }
}

impl From<RotatingContactResponseError3d> for RepeatedRotatingEventError3d {
    fn from(value: RotatingContactResponseError3d) -> Self {
        Self::Response(value)
    }
}

/// Advances through a bounded sequence of sampled rotating collision events while preserving the exact
/// remaining requested interval.
///
/// The first event is selected with [`crate::earliest_rotating_contact_frontier`]. After response, the
/// remaining rational timestep becomes the next segment. Before that next segment is searched, the current
/// contact frontier is stabilized through the configured bounded solver-pass budget. Each pass refreshes
/// the current zero-time contact set and applies one simultaneous response pass. This keeps resting and
/// newly-created support constraints in the authoritative solver without demanding exact global
/// idempotence from quantized contact projection. Current-frontier discovery reuses the persistent
/// conservative broad phase at a zero timestep, then exact-filters every candidate with the current OBB
/// geometry; it deliberately does not re-enter the sampled temporal search.
///
/// Quantized projection can leave a contact pair microscopically clear even when the resolved contact-point
/// motion is not separating. Such a solver-created gap is not evidence that the physical constraint was
/// released. When current-contact stabilization exhausts its configured passes, this function builds the
/// non-separating multi-body contact component from the solver's own observed contacts. A pair belongs to
/// that component only when its post-solver contact-point normal velocity and relative normal acceleration
/// are both non-separating. The component is eligible for continuation only if at least one of those pairs
/// was projected clear. If the ordinary positive re-contact selector stays inside that component, the
/// solver advances exactly to the selected contact state and hands the contacted suffix to the existing
/// persistent-tail solver without minting another impact event. A genuine releasing pair, a new external
/// impact, and every two-body bounce remain on the ordinary repeated-event path.
///
/// This distinction is solver-owned and does not change the 64-event safety cap, sampled search resolution,
/// tolerances, or retry policy. The cap still fails closed when genuine impact progression exceeds it.
///
/// Every admitted frontier is resolved before the next segment is searched. Event times in
/// [`RotatingResolvedEvent3d`] are therefore **segment-relative**, not absolute fractions of the original
/// requested interval. Zero-time stabilization passes consume no requested time and are not counted as
/// sampled events. Remaining-time composition canonicalizes each sampled remainder, cross-cancels it
/// against the current exact ratio, and stores the reduced result in deterministic limb storage. No
/// positive suffix is silently discarded merely because its reduced numerator or denominator exceeds
/// `i128`.
///
/// This function is intentionally not yet a complete frame step. When no further positive sampled event
/// is found, the final tail is returned in [`RepeatedRotatingEventAdvance3d::remaining`] rather than being
/// free-flown through potentially persistent contacts. Persistent/resting-contact stabilization over that
/// exact tail belongs to the world-level solver. The search itself remains sampled rotational collision
/// handling, not analytic rotational CCD, so an event island wholly between adjacent coarse samples can
/// still be missed.
///
/// # Errors
///
/// Returns [`RepeatedRotatingEventError3d`] for invalid bounds/timestep configuration, deterministic
/// exact-ratio capacity exhaustion, frontier/response failures, or when an actual additional sampled
/// event exists beyond `max_events`.
pub fn advance_repeated_rotating_events(
    boxes: &[RigidBox3d],
    config: RepeatedRotatingEventConfig3d,
) -> Result<RepeatedRotatingEventAdvance3d, RepeatedRotatingEventError3d> {
    let mut broad_phase = RotatingBroadPhase3d::default();
    advance_repeated_rotating_events_with_broad_phase(boxes, config, &mut broad_phase)
}

pub(crate) fn advance_repeated_rotating_events_with_broad_phase(
    boxes: &[RigidBox3d],
    config: RepeatedRotatingEventConfig3d,
    broad_phase: &mut RotatingBroadPhase3d,
) -> Result<RepeatedRotatingEventAdvance3d, RepeatedRotatingEventError3d> {
    validate_config(config)?;

    let mut work = RepeatedRotatingEventWorkStats3d::default();
    let mut response_scratch = RotatingContactResponseScratch3d::default();
    let mut remaining = config.search.free_flight;
    let first_search = search_with_free_flight(config.search, remaining);
    let Some(first_frontier) =
        earliest_rotating_contact_frontier_with_broad_phase(boxes, first_search, broad_phase)?
    else {
        return Ok(RepeatedRotatingEventAdvance3d {
            boxes: boxes.to_vec(),
            events: Vec::new(),
            remaining,
            work,
        });
    };

    let mut state;
    let mut events = Vec::new();
    let mut frontier = first_frontier;

    loop {
        if events.len() >= usize::from(config.max_events) {
            return Err(RepeatedRotatingEventError3d::EventLimit(config.max_events));
        }

        let (response, modified_body_ids) =
            resolve_rotating_contact_frontier_with_activity_and_scratch(
                frontier,
                config.solver_passes,
                &mut response_scratch,
            )?;
        remaining = scale_remaining_time(remaining, response.remaining_numerator, response.time)?;
        let response_time = response.time;
        let response_contacts = response.contacts;
        let response_passes = response.passes_used;
        work.event_response_passes = work
            .event_response_passes
            .saturating_add(u64::from(response_passes));
        let stabilization = stabilize_current_contacts(
            response.boxes,
            config.solver_passes,
            broad_phase,
            (&modified_body_ids, &response_contacts),
            &mut response_scratch,
            &mut work,
        )?;
        state = stabilization.boxes;
        let continuation_graph = if stabilization.exhausted_with_changes {
            build_contact_continuation_graph(
                &state,
                &stabilization.observed_contacts,
                config.search.free_flight.gravity,
                &mut response_scratch,
            )?
        } else {
            ContactContinuationGraph3d::default()
        };
        events.push(RotatingResolvedEvent3d {
            time: response_time,
            contacts: response_contacts,
            response_passes,
        });

        if remaining.timestep_is_zero() {
            break;
        }
        let next_search = search_with_free_flight(config.search, remaining);
        let selector = if continuation_graph.has_projected_clear_component() {
            let mut selector_broad_phase = RotatingBroadPhase3d::default();
            sampled_rotating_recontact_search_with_broad_phase(
                &state,
                next_search,
                &mut selector_broad_phase,
            )
            .map_err(RotatingContactFrontierError3d::Search)?
        } else {
            None
        };
        let Some(next) =
            next_rotating_contact_frontier_with_broad_phase(&state, next_search, broad_phase)?
        else {
            break;
        };
        let continuing_constraint = selector.as_ref().is_some_and(|hit| {
            hit.time == next.time && continuation_graph.contains_selector(hit.pair)
        });
        if continuing_constraint {
            remaining = scale_remaining_time(remaining, next.remaining_numerator, next.time)?;
            state = next.boxes;
            break;
        }
        frontier = next;
    }

    Ok(RepeatedRotatingEventAdvance3d {
        boxes: state,
        events,
        remaining,
        work,
    })
}

fn build_contact_continuation_graph(
    boxes: &[RigidBox3d],
    observed_contacts: &BTreeMap<RotationalSweepPair3d, ObbContactSeed3d>,
    gravity: Vec3i,
    body_index: &mut RotatingContactResponseScratch3d,
) -> Result<ContactContinuationGraph3d, RepeatedRotatingEventError3d> {
    body_index.ensure_body_index(boxes);
    let mut graph = ContactContinuationGraph3d::default();

    for (pair, observed_seed) in observed_contacts {
        let left = body_index.indexed_box(boxes, pair.left).ok_or(
            RotatingContactResponseError3d::MissingBody(pair.left),
        )?;
        let right = body_index.indexed_box(boxes, pair.right).ok_or(
            RotatingContactResponseError3d::MissingBody(pair.right),
        )?;
        let current_seed = obb_contact_seed(left.oriented_box(), right.oriented_box())
            .map_err(RotatingContactFrontierError3d::Geometry)?;
        let projected_clear = current_seed.is_none();
        let seed = current_seed.unwrap_or(*observed_seed);
        let normal_velocity = progression_contact_normal_velocity(left, right, seed)
            .map_err(RotatingContactResponseError3d::from)?;
        let normal_acceleration = progression_relative_normal_acceleration(left, right, gravity, seed)
            .map_err(RotatingContactResponseError3d::from)?;
        if normal_velocity <= 0 && normal_acceleration <= 0 {
            graph.add_edge(*pair, projected_clear);
        }
    }

    Ok(graph)
}

fn progression_relative_normal_acceleration(
    left: &RigidBox3d,
    right: &RigidBox3d,
    gravity: Vec3i,
    seed: ObbContactSeed3d,
) -> Result<i128, ObbContactResponseError3d> {
    let left_acceleration = if left.body().kind() == BodyKind::Dynamic {
        gravity
    } else {
        Vec3i::ZERO
    };
    let right_acceleration = if right.body().kind() == BodyKind::Dynamic {
        gravity
    } else {
        Vec3i::ZERO
    };
    progression_checked_dot(
        [
            progression_checked_sub(
                i128::from(right_acceleration.x),
                i128::from(left_acceleration.x),
            )?,
            progression_checked_sub(
                i128::from(right_acceleration.y),
                i128::from(left_acceleration.y),
            )?,
            progression_checked_sub(
                i128::from(right_acceleration.z),
                i128::from(left_acceleration.z),
            )?,
        ],
        seed.axis,
    )
}

fn progression_contact_normal_velocity(
    left: &RigidBox3d,
    right: &RigidBox3d,
    seed: ObbContactSeed3d,
) -> Result<i128, ObbContactResponseError3d> {
    let left_vertices = oriented_box_vertices(left.oriented_box())?;
    let right_vertices = oriented_box_vertices(right.oriented_box())?;
    let left_support = progression_support_centroid(&left_vertices, seed.left_support_mask)?;
    let right_support = progression_support_centroid(&right_vertices, seed.right_support_mask)?;
    let point = progression_reduced_contact_point(
        &left_vertices,
        seed.left_support_mask,
        left_support,
        left,
        &right_vertices,
        seed.right_support_mask,
        right_support,
        right,
        seed,
    )?;
    let left_offset = progression_vector_delta(left.body().position(), point);
    let right_offset = progression_vector_delta(right.body().position(), point);
    let left_velocity = progression_contact_velocity(left, left_offset)?;
    let right_velocity = progression_contact_velocity(right, right_offset)?;
    progression_checked_dot(
        [
            progression_checked_sub(
                i128::from(right_velocity[0]),
                i128::from(left_velocity[0]),
            )?,
            progression_checked_sub(
                i128::from(right_velocity[1]),
                i128::from(left_velocity[1]),
            )?,
            progression_checked_sub(
                i128::from(right_velocity[2]),
                i128::from(left_velocity[2]),
            )?,
        ],
        seed.axis,
    )
}

fn progression_support_centroid(
    vertices: &[Vec3i; 8],
    mask: u8,
) -> Result<Vec3i, ObbContactResponseError3d> {
    let mut sum = [0_i128; 3];
    let mut count = 0_i128;
    for (index, vertex) in vertices.iter().enumerate() {
        if mask & (1_u8 << index) == 0 {
            continue;
        }
        count = progression_checked_add(count, 1)?;
        sum[0] = progression_checked_add(sum[0], i128::from(vertex.x))?;
        sum[1] = progression_checked_add(sum[1], i128::from(vertex.y))?;
        sum[2] = progression_checked_add(sum[2], i128::from(vertex.z))?;
    }
    if count == 0 {
        return Err(ObbContactResponseError3d::ArithmeticOverflow);
    }
    Ok(Vec3i::new(
        progression_to_i32(progression_div_round_nearest(sum[0], count)?)?,
        progression_to_i32(progression_div_round_nearest(sum[1], count)?)?,
        progression_to_i32(progression_div_round_nearest(sum[2], count)?)?,
    ))
}

#[allow(clippy::too_many_arguments)]
fn progression_reduced_contact_point(
    left_vertices: &[Vec3i; 8],
    left_mask: u8,
    left_support: Vec3i,
    left: &RigidBox3d,
    right_vertices: &[Vec3i; 8],
    right_mask: u8,
    right_support: Vec3i,
    right: &RigidBox3d,
    seed: ObbContactSeed3d,
) -> Result<Vec3i, ObbContactResponseError3d> {
    if let Some(point) = progression_axis_aligned_face_overlap_centroid(
        left_vertices,
        left_mask,
        left_support,
        right_vertices,
        right_mask,
        right_support,
        seed.axis,
    )? {
        return Ok(point);
    }

    let left_spread = progression_support_spread_squared(left_vertices, left_mask, left_support)?;
    let right_spread = progression_support_spread_squared(right_vertices, right_mask, right_support)?;
    let anchor = match left_spread.cmp(&right_spread) {
        std::cmp::Ordering::Less => left_support,
        std::cmp::Ordering::Greater => right_support,
        std::cmp::Ordering::Equal => {
            if left.body().id() < right.body().id() {
                left_support
            } else {
                right_support
            }
        }
    };
    progression_project_to_support_midplane(
        anchor,
        left_support,
        right_support,
        seed.axis,
        seed.axis_length_squared,
    )
}

fn progression_axis_aligned_face_overlap_centroid(
    left_vertices: &[Vec3i; 8],
    left_mask: u8,
    left_support: Vec3i,
    right_vertices: &[Vec3i; 8],
    right_mask: u8,
    right_support: Vec3i,
    axis: [i128; 3],
) -> Result<Option<Vec3i>, ObbContactResponseError3d> {
    if left_mask.count_ones() != 4 || right_mask.count_ones() != 4 {
        return Ok(None);
    }
    let Some(normal_axis) = progression_coordinate_axis(axis) else {
        return Ok(None);
    };
    let tangent_axes = match normal_axis {
        0 => [1, 2],
        1 => [0, 2],
        2 => [0, 1],
        _ => return Ok(None),
    };
    if !progression_support_is_axis_aligned_rectangle(left_vertices, left_mask, tangent_axes)
        || !progression_support_is_axis_aligned_rectangle(right_vertices, right_mask, tangent_axes)
    {
        return Ok(None);
    }

    let mut coordinate = [0_i32; 3];
    coordinate[normal_axis] = progression_midpoint_axis(
        progression_component(left_support, normal_axis),
        progression_component(right_support, normal_axis),
    )?;
    for tangent_axis in tangent_axes {
        let (left_minimum, left_maximum) =
            progression_support_interval(left_vertices, left_mask, tangent_axis)?;
        let (right_minimum, right_maximum) =
            progression_support_interval(right_vertices, right_mask, tangent_axis)?;
        let overlap_minimum = left_minimum.max(right_minimum);
        let overlap_maximum = left_maximum.min(right_maximum);
        if overlap_minimum > overlap_maximum {
            return Ok(None);
        }
        coordinate[tangent_axis] =
            progression_midpoint_axis(overlap_minimum, overlap_maximum)?;
    }
    Ok(Some(Vec3i::new(
        coordinate[0],
        coordinate[1],
        coordinate[2],
    )))
}

fn progression_coordinate_axis(axis: [i128; 3]) -> Option<usize> {
    let mut found = None;
    for (index, component) in axis.into_iter().enumerate() {
        if component == 0 {
            continue;
        }
        if found.is_some() {
            return None;
        }
        found = Some(index);
    }
    found
}

fn progression_support_is_axis_aligned_rectangle(
    vertices: &[Vec3i; 8],
    mask: u8,
    tangent_axes: [usize; 2],
) -> bool {
    tangent_axes.into_iter().all(|axis| {
        let mut unique = [0_i32; 4];
        let mut unique_count = 0_usize;
        for (index, vertex) in vertices.iter().enumerate() {
            if mask & (1_u8 << index) == 0 {
                continue;
            }
            let value = progression_component(*vertex, axis);
            if unique[..unique_count].contains(&value) {
                continue;
            }
            if unique_count == unique.len() {
                return false;
            }
            unique[unique_count] = value;
            unique_count += 1;
        }
        unique_count == 2
    })
}

fn progression_support_interval(
    vertices: &[Vec3i; 8],
    mask: u8,
    axis: usize,
) -> Result<(i32, i32), ObbContactResponseError3d> {
    let mut minimum = None;
    let mut maximum = None;
    for (index, vertex) in vertices.iter().enumerate() {
        if mask & (1_u8 << index) == 0 {
            continue;
        }
        let value = progression_component(*vertex, axis);
        minimum = Some(minimum.map_or(value, |current: i32| current.min(value)));
        maximum = Some(maximum.map_or(value, |current: i32| current.max(value)));
    }
    match (minimum, maximum) {
        (Some(minimum), Some(maximum)) => Ok((minimum, maximum)),
        _ => Err(ObbContactResponseError3d::ArithmeticOverflow),
    }
}

fn progression_support_spread_squared(
    vertices: &[Vec3i; 8],
    mask: u8,
    center: Vec3i,
) -> Result<u128, ObbContactResponseError3d> {
    let mut total = 0_u128;
    let mut count = 0_u8;
    for (index, vertex) in vertices.iter().enumerate() {
        if mask & (1_u8 << index) == 0 {
            continue;
        }
        count = count
            .checked_add(1)
            .ok_or(ObbContactResponseError3d::ArithmeticOverflow)?;
        for value in [
            i128::from(vertex.x) - i128::from(center.x),
            i128::from(vertex.y) - i128::from(center.y),
            i128::from(vertex.z) - i128::from(center.z),
        ] {
            let magnitude = value.unsigned_abs();
            total = total
                .checked_add(
                    magnitude
                        .checked_mul(magnitude)
                        .ok_or(ObbContactResponseError3d::ArithmeticOverflow)?,
                )
                .ok_or(ObbContactResponseError3d::ArithmeticOverflow)?;
        }
    }
    if count == 0 {
        return Err(ObbContactResponseError3d::ArithmeticOverflow);
    }
    Ok(total)
}

fn progression_project_to_support_midplane(
    anchor: Vec3i,
    left_support: Vec3i,
    right_support: Vec3i,
    axis: [i128; 3],
    axis_length_squared: u128,
) -> Result<Vec3i, ObbContactResponseError3d> {
    let length_squared = i128::try_from(axis_length_squared)
        .map_err(|_| ObbContactResponseError3d::ArithmeticOverflow)?;
    if length_squared <= 0 {
        return Err(ObbContactResponseError3d::ArithmeticOverflow);
    }
    let target_projection = progression_div_round_nearest(
        progression_checked_add(
            progression_dot_vec(left_support, axis)?,
            progression_dot_vec(right_support, axis)?,
        )?,
        2,
    )?;
    let projection_delta =
        progression_checked_sub(target_projection, progression_dot_vec(anchor, axis)?)?;
    let mut coordinate = [
        i128::from(anchor.x),
        i128::from(anchor.y),
        i128::from(anchor.z),
    ];
    for index in 0..3 {
        coordinate[index] = progression_checked_add(
            coordinate[index],
            progression_div_round_nearest(
                progression_checked_mul(projection_delta, axis[index])?,
                length_squared,
            )?,
        )?;
    }
    Ok(Vec3i::new(
        progression_to_i32(coordinate[0])?,
        progression_to_i32(coordinate[1])?,
        progression_to_i32(coordinate[2])?,
    ))
}

fn progression_midpoint_axis(
    left: i32,
    right: i32,
) -> Result<i32, ObbContactResponseError3d> {
    progression_to_i32(progression_div_round_nearest(
        progression_checked_add(i128::from(left), i128::from(right))?,
        2,
    )?)
}

fn progression_vector_delta(center: Vec3i, point: Vec3i) -> [i64; 3] {
    [
        i64::from(point.x) - i64::from(center.x),
        i64::from(point.y) - i64::from(center.y),
        i64::from(point.z) - i64::from(center.z),
    ]
}

fn progression_contact_velocity(
    rigid_box: &RigidBox3d,
    offset: [i64; 3],
) -> Result<[i64; 3], ObbContactResponseError3d> {
    let omega = if rigid_box.rotation_locked {
        AngularVelocity3d::default()
    } else {
        rigid_box.angular.angular_velocity
    };
    let rotation_x = progression_checked_sub(
        progression_checked_mul(i128::from(omega.y), i128::from(offset[2]))?,
        progression_checked_mul(i128::from(omega.z), i128::from(offset[1]))?,
    )?;
    let rotation_y = progression_checked_sub(
        progression_checked_mul(i128::from(omega.z), i128::from(offset[0]))?,
        progression_checked_mul(i128::from(omega.x), i128::from(offset[2]))?,
    )?;
    let rotation_z = progression_checked_sub(
        progression_checked_mul(i128::from(omega.x), i128::from(offset[1]))?,
        progression_checked_mul(i128::from(omega.y), i128::from(offset[0]))?,
    )?;
    let scale = i128::from(ANGULAR_VELOCITY_SCALE);
    Ok([
        progression_add_linear_rotation(rigid_box.body().velocity().x, rotation_x, scale)?,
        progression_add_linear_rotation(rigid_box.body().velocity().y, rotation_y, scale)?,
        progression_add_linear_rotation(rigid_box.body().velocity().z, rotation_z, scale)?,
    ])
}

fn progression_add_linear_rotation(
    linear: i32,
    rotational_numerator: i128,
    scale: i128,
) -> Result<i64, ObbContactResponseError3d> {
    let rotational = i64::try_from(progression_div_round_nearest(rotational_numerator, scale)?)
        .map_err(|_| ObbContactResponseError3d::ArithmeticOverflow)?;
    i64::from(linear)
        .checked_add(rotational)
        .ok_or(ObbContactResponseError3d::ArithmeticOverflow)
}

fn progression_component(vector: Vec3i, axis: usize) -> i32 {
    match axis {
        0 => vector.x,
        1 => vector.y,
        2 => vector.z,
        _ => 0,
    }
}

fn progression_dot_vec(
    vector: Vec3i,
    axis: [i128; 3],
) -> Result<i128, ObbContactResponseError3d> {
    progression_checked_dot(
        [
            i128::from(vector.x),
            i128::from(vector.y),
            i128::from(vector.z),
        ],
        axis,
    )
}

fn progression_checked_dot(
    left: [i128; 3],
    right: [i128; 3],
) -> Result<i128, ObbContactResponseError3d> {
    progression_checked_add(
        progression_checked_add(
            progression_checked_mul(left[0], right[0])?,
            progression_checked_mul(left[1], right[1])?,
        )?,
        progression_checked_mul(left[2], right[2])?,
    )
}

fn progression_checked_mul(
    left: i128,
    right: i128,
) -> Result<i128, ObbContactResponseError3d> {
    left.checked_mul(right)
        .ok_or(ObbContactResponseError3d::ArithmeticOverflow)
}

fn progression_checked_add(
    left: i128,
    right: i128,
) -> Result<i128, ObbContactResponseError3d> {
    left.checked_add(right)
        .ok_or(ObbContactResponseError3d::ArithmeticOverflow)
}

fn progression_checked_sub(
    left: i128,
    right: i128,
) -> Result<i128, ObbContactResponseError3d> {
    left.checked_sub(right)
        .ok_or(ObbContactResponseError3d::ArithmeticOverflow)
}

fn progression_div_round_nearest(
    numerator: i128,
    denominator: i128,
) -> Result<i128, ObbContactResponseError3d> {
    if denominator <= 0 {
        return Err(ObbContactResponseError3d::ArithmeticOverflow);
    }
    let half = denominator / 2;
    let adjusted = if numerator >= 0 {
        numerator.checked_add(half)
    } else {
        numerator.checked_sub(half)
    }
    .ok_or(ObbContactResponseError3d::ArithmeticOverflow)?;
    Ok(adjusted / denominator)
}

fn progression_to_i32(value: i128) -> Result<i32, ObbContactResponseError3d> {
    i32::try_from(value).map_err(|_| ObbContactResponseError3d::ArithmeticOverflow)
}

fn stabilize_current_contacts(
    mut boxes: Vec<RigidBox3d>,
    solver_passes: u8,
    broad_phase: &mut RotatingBroadPhase3d,
    stabilization_seed: (&[BodyId], &[RotatingContactSearchHit3d]),
    response_scratch: &mut RotatingContactResponseScratch3d,
    work: &mut RepeatedRotatingEventWorkStats3d,
) -> Result<CurrentContactStabilization3d, RepeatedRotatingEventError3d> {
    let (initial_active, initial_contacts) = stabilization_seed;
    let mut active = initial_active.to_vec();
    active.sort_unstable();
    active.dedup();
    let mut observed_contacts = initial_contacts
        .iter()
        .map(|contact| (contact.pair, contact.contact))
        .collect::<BTreeMap<_, _>>();
    let mut contacts = initial_contacts
        .iter()
        .cloned()
        .map(|mut contact| {
            contact.time = SampledContactTime3d::ZERO;
            (contact.pair, contact)
        })
        .collect::<BTreeMap<_, _>>();
    let mut exhausted_with_changes = false;

    for pass in 0..solver_passes {
        if active.is_empty() {
            break;
        }
        work.stabilization_active_bodies = work
            .stabilization_active_bodies
            .saturating_add(u64::try_from(active.len()).unwrap_or(u64::MAX));
        let current = refresh_current_contacts_for_changed_bodies(
            &boxes,
            &active,
            &mut contacts,
            broad_phase,
            response_scratch,
        )?;
        work.stabilization_candidate_pairs = work
            .stabilization_candidate_pairs
            .saturating_add(u64::try_from(current.candidate_pairs).unwrap_or(u64::MAX));
        work.stabilization_exact_contacts = work
            .stabilization_exact_contacts
            .saturating_add(u64::try_from(current.recomputed_contacts).unwrap_or(u64::MAX));
        let Some(frontier) = current.frontier else {
            active.clear();
            break;
        };
        observed_contacts.extend(
            frontier
                .contacts
                .iter()
                .map(|contact| (contact.pair, contact.contact)),
        );
        work.stabilization_passes = work.stabilization_passes.saturating_add(1);
        let (response, modified_body_ids) =
            resolve_rotating_contact_frontier_with_activity_and_scratch(
                frontier,
                1,
                response_scratch,
            )?;
        boxes = response.boxes;
        active = modified_body_ids;
        if active.is_empty() {
            break;
        }
        if pass.saturating_add(1) == solver_passes {
            exhausted_with_changes = true;
        }
    }

    if exhausted_with_changes {
        work.stabilizations_hitting_limit = work.stabilizations_hitting_limit.saturating_add(1);
    }
    Ok(CurrentContactStabilization3d {
        boxes,
        observed_contacts,
        exhausted_with_changes,
    })
}

#[derive(Clone, Debug)]
struct CurrentContactFrontierResult3d {
    frontier: Option<RotatingContactFrontier3d>,
    candidate_pairs: usize,
    recomputed_contacts: usize,
}

/// Refreshes only exact contact edges whose geometry can have changed.
///
/// Contacts whose two endpoints are unchanged are retained as authoritative exact evidence. Every cached
/// edge touching an active body is discarded and revalidated from current geometry, while the targeted
/// broad phase discovers any newly-created neighbors of those active bodies. The resulting frontier still
/// contains *all* current contacts, preserving the simultaneous solver semantics of a full-world refresh
/// without recomputing unchanged edges.
fn refresh_current_contacts_for_changed_bodies(
    boxes: &[RigidBox3d],
    active: &[BodyId],
    contacts: &mut BTreeMap<RotationalSweepPair3d, RotatingContactSearchHit3d>,
    broad_phase: &mut RotatingBroadPhase3d,
    body_index: &RotatingContactResponseScratch3d,
) -> Result<CurrentContactFrontierResult3d, RotatingContactFrontierError3d> {
    let active_set = active.iter().copied().collect::<BTreeSet<_>>();
    contacts
        .retain(|pair, _| !active_set.contains(&pair.left) && !active_set.contains(&pair.right));

    let mut active_boxes = Vec::with_capacity(active.len());
    for id in active {
        let rigid_box = body_index
            .indexed_box(boxes, *id)
            .ok_or(RotatingContactFrontierError3d::MissingBody(*id))?;
        active_boxes.push(rigid_box);
    }
    let candidates = broad_phase.candidate_pairs_for_changed_current_bodies(active_boxes)?;
    let candidate_pairs = candidates.len();
    let mut recomputed_contacts = 0_usize;

    for pair in candidates {
        let left = body_index
            .indexed_box(boxes, pair.left)
            .ok_or(RotatingContactFrontierError3d::MissingBody(pair.left))?;
        let right = body_index
            .indexed_box(boxes, pair.right)
            .ok_or(RotatingContactFrontierError3d::MissingBody(pair.right))?;
        let Some(contact) = obb_contact_seed(left.oriented_box(), right.oriented_box())? else {
            continue;
        };
        recomputed_contacts = recomputed_contacts.saturating_add(1);
        contacts.insert(
            pair,
            RotatingContactSearchHit3d {
                time: SampledContactTime3d::ZERO,
                pair,
                contact,
            },
        );
    }

    let frontier = if contacts.is_empty() {
        None
    } else {
        Some(RotatingContactFrontier3d {
            boxes: boxes.to_vec(),
            time: SampledContactTime3d::ZERO,
            contacts: contacts.values().cloned().collect(),
            remaining_numerator: 1,
        })
    };
    Ok(CurrentContactFrontierResult3d {
        frontier,
        candidate_pairs,
        recomputed_contacts,
    })
}

#[cfg(test)]
fn current_contact_frontier(
    boxes: &[RigidBox3d],
) -> Result<Option<RotatingContactFrontier3d>, RotatingContactFrontierError3d> {
    let mut broad_phase = RotatingBroadPhase3d::default();
    let zero_time = RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 0, 1);
    broad_phase.candidate_pairs(boxes, zero_time)?;
    let active = boxes
        .iter()
        .map(|rigid_box| rigid_box.body().id())
        .collect::<Vec<_>>();
    let mut contacts = BTreeMap::new();
    let mut body_index = RotatingContactResponseScratch3d::default();
    body_index.ensure_body_index(boxes);
    Ok(refresh_current_contacts_for_changed_bodies(
        boxes,
        &active,
        &mut contacts,
        &mut broad_phase,
        &body_index,
    )?
    .frontier)
}

fn validate_config(
    config: RepeatedRotatingEventConfig3d,
) -> Result<(), RepeatedRotatingEventError3d> {
    if config.max_events == 0 {
        return Err(RepeatedRotatingEventError3d::ZeroEventLimit);
    }
    if config.max_events > MAX_REPEATED_ROTATING_EVENTS {
        return Err(RepeatedRotatingEventError3d::EventLimit(
            MAX_REPEATED_ROTATING_EVENTS,
        ));
    }
    match config.search.free_flight.exact_timestep() {
        Ok(_) => Ok(()),
        Err(RigidBoxFreeFlightError3d::NegativeTimestepNumerator(value)) => Err(
            RepeatedRotatingEventError3d::NegativeTimestepNumerator(value),
        ),
        Err(RigidBoxFreeFlightError3d::NonPositiveTimestepDenominator(value)) => {
            Err(RepeatedRotatingEventError3d::NonPositiveTimestepDenominator(value))
        }
        Err(_) => Err(RepeatedRotatingEventError3d::RatioTooLarge),
    }
}

fn search_with_free_flight(
    search: RotatingContactSearchConfig3d,
    free_flight: RigidBoxFreeFlightConfig3d,
) -> RotatingContactSearchConfig3d {
    RotatingContactSearchConfig3d {
        free_flight,
        ..search
    }
}

fn scale_remaining_time(
    current: RigidBoxFreeFlightConfig3d,
    remaining_numerator: u32,
    event_time: SampledContactTime3d,
) -> Result<RigidBoxFreeFlightConfig3d, RepeatedRotatingEventError3d> {
    if event_time.denominator == 0 || remaining_numerator > event_time.denominator {
        return Err(RepeatedRotatingEventError3d::InvalidRemainder(event_time));
    }

    current
        .scaled_fraction(remaining_numerator, event_time.denominator)
        .map_err(|error| match error {
            RigidBoxFreeFlightError3d::NegativeTimestepNumerator(value) => {
                RepeatedRotatingEventError3d::NegativeTimestepNumerator(value)
            }
            RigidBoxFreeFlightError3d::NonPositiveTimestepDenominator(value) => {
                RepeatedRotatingEventError3d::NonPositiveTimestepDenominator(value)
            }
            _ => RepeatedRotatingEventError3d::RatioTooLarge,
        })
}

#[cfg(test)]
mod tests {
    use crate::{
        AngularState3d, AngularVelocity3d, BodyId, MATERIAL_SCALE, Material, Orientation3d,
        RigidBody, RigidBox3d, RigidBoxFreeFlightConfig3d, RotatingContactSearchConfig3d, Vec3i,
    };

    use super::{
        RepeatedRotatingEventConfig3d, RepeatedRotatingEventError3d,
        advance_repeated_rotating_events, current_contact_frontier, scale_remaining_time,
    };

    fn dynamic(id: u64, position: Vec3i, velocity: Vec3i, material: Material) -> RigidBox3d {
        RigidBox3d::new(
            RigidBody::dynamic(BodyId(id), position, velocity, Vec3i::new(1, 1, 1))
                .with_material(material),
            AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
        )
        .expect("valid dynamic box")
    }

    fn fixed(id: u64, position: Vec3i, material: Material) -> RigidBox3d {
        RigidBox3d::new(
            RigidBody::fixed(BodyId(id), position, Vec3i::new(1, 1, 1)).with_material(material),
            AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
        )
        .expect("valid fixed box")
    }

    fn config(max_events: u16) -> RepeatedRotatingEventConfig3d {
        RepeatedRotatingEventConfig3d::new(
            RotatingContactSearchConfig3d::new(
                RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 1, 1),
                32,
                4,
            ),
            8,
            max_events,
        )
    }

    #[test]
    fn no_event_leaves_the_full_tail_unconsumed() {
        let boxes = [
            dynamic(1, Vec3i::new(-100, 0, 0), Vec3i::ZERO, Material::new(0)),
            fixed(2, Vec3i::new(100, 0, 0), Material::new(0)),
        ];
        let advance =
            advance_repeated_rotating_events(&boxes, config(8)).expect("valid no-event advance");

        assert_eq!(advance.boxes, boxes);
        assert!(advance.events.is_empty());
        assert_eq!(advance.remaining, config(8).search.free_flight);
    }

    #[test]
    fn current_frontier_does_not_look_ahead_into_future_motion() {
        let boxes = [
            dynamic(1, Vec3i::ZERO, Vec3i::new(100, 0, 0), Material::new(0)),
            fixed(2, Vec3i::new(20, 0, 0), Material::new(0)),
        ];

        assert!(
            current_contact_frontier(&boxes)
                .expect("current contact query")
                .is_none()
        );
    }

    #[test]
    fn time_zero_event_is_resolved_without_consuming_the_tail() {
        let boxes = [
            dynamic(1, Vec3i::ZERO, Vec3i::new(60, 0, 0), Material::new(0)),
            fixed(2, Vec3i::new(2, 0, 0), Material::new(0)),
        ];
        let advance = advance_repeated_rotating_events(&boxes, config(8))
            .expect("valid initial-contact advance");

        assert_eq!(advance.events.len(), 1);
        assert_eq!(advance.events[0].time, SampledContactTime3d::ZERO);
        assert_eq!(advance.boxes[0].body().velocity().x, 0);
        assert_eq!(advance.remaining, config(8).search.free_flight);
    }

    #[test]
    fn multiple_wall_impacts_are_resolved_in_one_requested_interval() {
        let elastic = Material::new(MATERIAL_SCALE);
        let boxes = [
            fixed(1, Vec3i::new(-20, 0, 0), elastic),
            dynamic(2, Vec3i::ZERO, Vec3i::new(100, 0, 0), elastic),
            fixed(3, Vec3i::new(20, 0, 0), elastic),
        ];
        let advance = advance_repeated_rotating_events(&boxes, config(8))
            .expect("valid repeated impact advance");

        assert!(advance.events.len() >= 2);
        assert_eq!(advance.events[0].contacts[0].pair.left, BodyId(2));
        assert_eq!(advance.events[0].contacts[0].pair.right, BodyId(3));
        assert_eq!(advance.events[1].contacts[0].pair.left, BodyId(1));
        assert_eq!(advance.events[1].contacts[0].pair.right, BodyId(2));
    }

    #[test]
    fn event_limit_fails_only_when_a_real_additional_event_exists() {
        let elastic = Material::new(MATERIAL_SCALE);
        let boxes = [
            fixed(1, Vec3i::new(-20, 0, 0), elastic),
            dynamic(2, Vec3i::ZERO, Vec3i::new(100, 0, 0), elastic),
            fixed(3, Vec3i::new(20, 0, 0), elastic),
        ];

        assert_eq!(
            advance_repeated_rotating_events(&boxes, config(1)),
            Err(RepeatedRotatingEventError3d::EventLimit(1))
        );
    }

    #[test]
    fn exact_remaining_time_reduction_preserves_rational_value() {
        let current = RigidBoxFreeFlightConfig3d::new(Vec3i::new(0, -10, 0), 2, 3);
        assert_eq!(
            scale_remaining_time(
                current,
                5,
                SampledContactTime3d {
                    numerator: 3,
                    denominator: 8,
                },
            )
            .expect("representable scaled remainder"),
            RigidBoxFreeFlightConfig3d::new(Vec3i::new(0, -10, 0), 5, 12)
        );
        assert_eq!(
            scale_remaining_time(
                current,
                0,
                SampledContactTime3d {
                    numerator: 1,
                    denominator: 1,
                },
            )
            .expect("zero remainder"),
            RigidBoxFreeFlightConfig3d::new(Vec3i::new(0, -10, 0), 0, 1)
        );
    }

    #[test]
    fn reducible_remainder_is_normalized_before_exact_composition() {
        let current = RigidBoxFreeFlightConfig3d::new_wide(Vec3i::ZERO, i128::MAX, 1);
        let scaled = scale_remaining_time(
            current,
            256,
            SampledContactTime3d {
                numerator: 256,
                denominator: 512,
            },
        )
        .expect("reducible sampled remainder must not manufacture overflow");

        assert_eq!(scaled.timestep_i128(), Some((i128::MAX, 2)));
    }

    #[test]
    fn denominator_growth_stays_exact_beyond_i32() {
        let mut remaining = RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 1, 60);
        let event = SampledContactTime3d {
            numerator: 1,
            denominator: 512,
        };
        let mut expected_numerator = 1_u128;
        let mut expected_denominator = 60_u128;
        for _ in 0..8 {
            remaining = scale_remaining_time(remaining, 511, event)
                .expect("exact widened denominator composition");
            expected_numerator = expected_numerator.checked_mul(511).expect("test numerator");
            expected_denominator = expected_denominator
                .checked_mul(512)
                .expect("test denominator");
            let (numerator, denominator) = remaining
                .timestep_i128()
                .expect("eight sampled factors still fit i128");
            assert_eq!(numerator as u128, expected_numerator);
            assert_eq!(denominator as u128, expected_denominator);
        }
        assert!(
            remaining.timestep_i128().expect("eight factors fit i128").1 > i128::from(i32::MAX)
        );
    }

    #[test]
    fn exact_remaining_time_survives_beyond_i128() {
        let mut remaining = RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 1, 60);
        let event = SampledContactTime3d {
            numerator: 1,
            denominator: 512,
        };
        for _ in 0..20 {
            remaining = scale_remaining_time(remaining, 511, event)
                .expect("bounded repeated-event exact ratio");
        }
        assert!(remaining.timestep_i128().is_none());
        assert!(!remaining.timestep_is_zero());
    }

    #[test]
    fn repeated_event_advance_is_bit_for_bit_repeatable() {
        let elastic = Material::new(MATERIAL_SCALE);
        let boxes = [
            fixed(1, Vec3i::new(-20, 0, 0), elastic),
            dynamic(2, Vec3i::ZERO, Vec3i::new(100, 0, 0), elastic),
            fixed(3, Vec3i::new(20, 0, 0), elastic),
        ];

        let first = advance_repeated_rotating_events(&boxes, config(8)).expect("first advance");
        let second = advance_repeated_rotating_events(&boxes, config(8)).expect("second advance");
        assert_eq!(first, second);
    }

    #[test]
    fn zero_event_limit_fails_closed() {
        let boxes = [
            dynamic(1, Vec3i::ZERO, Vec3i::ZERO, Material::new(0)),
            fixed(2, Vec3i::new(2, 0, 0), Material::new(0)),
        ];
        assert_eq!(
            advance_repeated_rotating_events(&boxes, config(0)),
            Err(RepeatedRotatingEventError3d::ZeroEventLimit)
        );
    }
}
