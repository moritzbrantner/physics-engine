use std::{error::Error, fmt};

use crate::{
    RigidBox3d, RigidBoxFreeFlightConfig3d, RotatingContactFrontierError3d,
    RotatingContactResponseError3d, RotatingContactSearchConfig3d, RotatingContactSearchHit3d,
    SampledContactTime3d, earliest_rotating_contact_frontier, next_rotating_contact_frontier,
    resolve_rotating_contact_frontier,
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

/// Result of consuming every sampled rotating event admitted before the first unresolved tail.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RepeatedRotatingEventAdvance3d {
    /// State immediately after the last resolved sampled frontier, or the unchanged input if no event
    /// was admitted.
    pub boxes: Vec<RigidBox3d>,
    pub events: Vec<RotatingResolvedEvent3d>,
    /// Requested time that remains deliberately unconsumed.
    ///
    /// The value stays exact while its reduced ratio fits the public `i32` timestep contract. If repeated
    /// event composition would exceed that representation, it is deterministically rounded downward to
    /// the highest-precision representable ratio. This conservative truncation never advances beyond the
    /// requested frame; at worst it leaves an unconsumed sub-quantum tail instead of failing on a purely
    /// representational denominator growth.
    ///
    /// The remaining segment is not automatically free-flown because it may contain persistent/resting
    /// contacts that the world-level tail solver must stabilize.
    pub remaining: RigidBoxFreeFlightConfig3d,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RepeatedRotatingEventError3d {
    ZeroEventLimit,
    EventLimit(u16),
    NegativeTimestepNumerator(i32),
    NonPositiveTimestepDenominator(i32),
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
                "repeated rotating event remaining-time arithmetic exceeded the supported bounded ratio"
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

/// Advances through a bounded sequence of sampled rotating collision events while preserving the
/// remaining requested interval without overflowing its stable public timestep representation.
///
/// The first event is selected with [`earliest_rotating_contact_frontier`]. After response, the remaining
/// rational timestep becomes the next segment. Later events are selected with
/// [`next_rotating_contact_frontier`], so a persistent time-zero pair cannot monopolize event discovery;
/// if it is still touching when another pair selects a later event, shared frontier reconstruction brings
/// it back into simultaneous response.
///
/// Every admitted frontier is resolved before the next segment is searched. Event times in
/// [`RotatingResolvedEvent3d`] are therefore **segment-relative**, not absolute fractions of the original
/// requested interval. Remaining time is reduced exactly while representable. If repeated denominator
/// composition exceeds the public `i32` ratio contract, the remaining duration is rounded downward to the
/// most precise representable rational. This can only shorten the unresolved tail; it cannot advance a
/// body beyond the requested frame or turn a missed collision into forward motion.
///
/// This function is intentionally not yet a complete frame step. When no further positive sampled event
/// is found, the final tail is returned in [`RepeatedRotatingEventAdvance3d::remaining`] rather than being
/// free-flown through potentially persistent contacts. Persistent/resting-contact stabilization over that
/// tail belongs to the world-level solver. The search itself remains sampled rotational collision handling,
/// not analytic rotational CCD, so an event island wholly between adjacent coarse samples can still be
/// missed.
///
/// # Errors
///
/// Returns [`RepeatedRotatingEventError3d`] for invalid bounds/timestep configuration, bounded-ratio
/// arithmetic overflow, frontier/response failures, or when an actual additional sampled event exists
/// beyond `max_events`.
pub fn advance_repeated_rotating_events(
    boxes: &[RigidBox3d],
    config: RepeatedRotatingEventConfig3d,
) -> Result<RepeatedRotatingEventAdvance3d, RepeatedRotatingEventError3d> {
    validate_config(config)?;

    let mut remaining = config.search.free_flight;
    let first_search = search_with_free_flight(config.search, remaining);
    let Some(first_frontier) = earliest_rotating_contact_frontier(boxes, first_search)? else {
        return Ok(RepeatedRotatingEventAdvance3d {
            boxes: boxes.to_vec(),
            events: Vec::new(),
            remaining,
        });
    };

    let mut state;
    let mut events = Vec::new();
    let mut frontier = first_frontier;

    loop {
        if events.len() >= usize::from(config.max_events) {
            return Err(RepeatedRotatingEventError3d::EventLimit(config.max_events));
        }

        let response = resolve_rotating_contact_frontier(frontier, config.solver_passes)?;
        remaining = scale_remaining_time(remaining, response.remaining_numerator, response.time)?;
        events.push(RotatingResolvedEvent3d {
            time: response.time,
            contacts: response.contacts,
            response_passes: response.passes_used,
        });
        state = response.boxes;

        if remaining.timestep_numerator == 0 {
            break;
        }
        let next_search = search_with_free_flight(config.search, remaining);
        let Some(next) = next_rotating_contact_frontier(&state, next_search)? else {
            break;
        };
        frontier = next;
    }

    Ok(RepeatedRotatingEventAdvance3d {
        boxes: state,
        events,
        remaining,
    })
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
    if config.search.free_flight.timestep_numerator < 0 {
        return Err(RepeatedRotatingEventError3d::NegativeTimestepNumerator(
            config.search.free_flight.timestep_numerator,
        ));
    }
    if config.search.free_flight.timestep_denominator <= 0 {
        return Err(
            RepeatedRotatingEventError3d::NonPositiveTimestepDenominator(
                config.search.free_flight.timestep_denominator,
            ),
        );
    }
    Ok(())
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
    if current.timestep_numerator < 0 {
        return Err(RepeatedRotatingEventError3d::NegativeTimestepNumerator(
            current.timestep_numerator,
        ));
    }
    if current.timestep_denominator <= 0 {
        return Err(
            RepeatedRotatingEventError3d::NonPositiveTimestepDenominator(
                current.timestep_denominator,
            ),
        );
    }
    if remaining_numerator == 0 || current.timestep_numerator == 0 {
        return Ok(RigidBoxFreeFlightConfig3d::new(current.gravity, 0, 1));
    }

    let numerator = u128::from(current.timestep_numerator.unsigned_abs())
        .checked_mul(u128::from(remaining_numerator))
        .ok_or(RepeatedRotatingEventError3d::RatioTooLarge)?;
    let denominator = u128::from(current.timestep_denominator.unsigned_abs())
        .checked_mul(u128::from(event_time.denominator))
        .ok_or(RepeatedRotatingEventError3d::RatioTooLarge)?;
    let divisor = greatest_common_divisor(numerator, denominator);
    bounded_ratio_floor(numerator / divisor, denominator / divisor).map(
        |(numerator, denominator)| {
            RigidBoxFreeFlightConfig3d::new(current.gravity, numerator, denominator)
        },
    )
}

fn bounded_ratio_floor(
    numerator: u128,
    denominator: u128,
) -> Result<(i32, i32), RepeatedRotatingEventError3d> {
    if denominator == 0 {
        return Err(RepeatedRotatingEventError3d::RatioTooLarge);
    }
    if numerator == 0 {
        return Ok((0, 1));
    }

    let maximum = i32::MAX as u128;
    if numerator <= maximum && denominator <= maximum {
        return Ok((
            i32::try_from(numerator).map_err(|_| RepeatedRotatingEventError3d::RatioTooLarge)?,
            i32::try_from(denominator).map_err(|_| RepeatedRotatingEventError3d::RatioTooLarge)?,
        ));
    }

    let whole = numerator / denominator;
    if whole > maximum {
        return Err(RepeatedRotatingEventError3d::RatioTooLarge);
    }

    let maximum_denominator_for_numerator = maximum
        .checked_mul(denominator)
        .ok_or(RepeatedRotatingEventError3d::RatioTooLarge)?
        / numerator;
    let target_denominator = maximum_denominator_for_numerator.clamp(1, maximum);
    let target_numerator = numerator
        .checked_mul(target_denominator)
        .ok_or(RepeatedRotatingEventError3d::RatioTooLarge)?
        / denominator;
    if target_numerator == 0 {
        return Ok((0, 1));
    }
    if target_numerator > maximum {
        return Err(RepeatedRotatingEventError3d::RatioTooLarge);
    }

    let divisor = greatest_common_divisor(target_numerator, target_denominator);
    Ok((
        i32::try_from(target_numerator / divisor)
            .map_err(|_| RepeatedRotatingEventError3d::RatioTooLarge)?,
        i32::try_from(target_denominator / divisor)
            .map_err(|_| RepeatedRotatingEventError3d::RatioTooLarge)?,
    ))
}

fn greatest_common_divisor(mut left: u128, mut right: u128) -> u128 {
    while right != 0 {
        let remainder = left % right;
        left = right;
        right = remainder;
    }
    left
}

#[cfg(test)]
mod tests {
    use crate::{
        AngularState3d, AngularVelocity3d, BodyId, MATERIAL_SCALE, Material, Orientation3d,
        RigidBody, RigidBox3d, RigidBoxFreeFlightConfig3d, RotatingContactSearchConfig3d, Vec3i,
    };

    use super::{
        RepeatedRotatingEventConfig3d, RepeatedRotatingEventError3d,
        advance_repeated_rotating_events, scale_remaining_time,
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
    fn time_zero_event_is_resolved_without_consuming_the_tail() {
        let boxes = [
            dynamic(1, Vec3i::ZERO, Vec3i::new(60, 0, 0), Material::new(0)),
            fixed(2, Vec3i::new(2, 0, 0), Material::new(0)),
        ];
        let advance = advance_repeated_rotating_events(&boxes, config(8))
            .expect("valid initial-contact advance");

        assert_eq!(advance.events.len(), 1);
        assert_eq!(advance.events[0].time, crate::SampledContactTime3d::ZERO);
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
                crate::SampledContactTime3d {
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
                crate::SampledContactTime3d {
                    numerator: 1,
                    denominator: 1,
                },
            )
            .expect("zero remainder"),
            RigidBoxFreeFlightConfig3d::new(Vec3i::new(0, -10, 0), 0, 1)
        );
    }

    #[test]
    fn denominator_growth_is_conservatively_bounded_instead_of_failing() {
        let mut remaining = RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 1, 60);
        let event = crate::SampledContactTime3d {
            numerator: 1,
            denominator: 512,
        };
        for _ in 0..8 {
            let previous = remaining;
            remaining = scale_remaining_time(remaining, 511, event)
                .expect("bounded denominator composition");
            assert!(remaining.timestep_numerator > 0);
            assert!(remaining.timestep_denominator > 0);
            assert!(remaining.timestep_numerator <= i32::MAX);
            assert!(remaining.timestep_denominator <= i32::MAX);
            assert!(
                i128::from(remaining.timestep_numerator)
                    * i128::from(previous.timestep_denominator)
                    <= i128::from(previous.timestep_numerator)
                        * i128::from(remaining.timestep_denominator),
                "bounded remainder advanced beyond the previous interval"
            );
        }
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
