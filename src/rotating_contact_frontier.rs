use std::{collections::BTreeMap, error::Error, fmt};

use crate::{
    BodyId, OrientedBoxError3d, RigidBox3d, RigidBoxFreeFlightError3d, RotatingBroadPhaseError3d,
    RotatingContactSearchConfig3d, RotatingContactSearchError3d, RotatingContactSearchHit3d,
    RotationalSweepPair3d, SampledContactTime3d, obb_contact_seed, sample_rigid_box_free_flight,
};
use crate::{
    rotating_broad_phase::RotatingBroadPhase3d,
    rotating_contact_search_ordered::sampled_rotating_contact_search_with_broad_phase,
    rotating_recontact_search::sampled_rotating_recontact_search_with_broad_phase,
};

/// One shared pre-response world reconstructed at an admitted sampled rotating contact time.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RotatingContactFrontier3d {
    /// Every rotating body sampled directly from the common interval start at [`Self::time`].
    pub boxes: Vec<RigidBox3d>,
    /// Sampled contact fraction admitted by the search policy that selected this frontier.
    pub time: SampledContactTime3d,
    /// Every broad-phase candidate that is in OBB contact in the shared frontier state.
    pub contacts: Vec<RotatingContactSearchHit3d>,
    /// Remaining numerator of the requested interval over [`Self::time`]'s denominator.
    pub remaining_numerator: u32,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RotatingContactFrontierError3d {
    InvalidSearchTime(SampledContactTime3d),
    MissingBody(BodyId),
    EarliestContactMissing(RotationalSweepPair3d),
    EarliestContactChanged(RotationalSweepPair3d),
    Search(RotatingContactSearchError3d),
    BroadPhase(RotatingBroadPhaseError3d),
    FreeFlight(RigidBoxFreeFlightError3d),
    Geometry(OrientedBoxError3d),
}

impl fmt::Display for RotatingContactFrontierError3d {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidSearchTime(time) => write!(
                formatter,
                "rotating contact frontier received invalid search time {}/{}",
                time.numerator, time.denominator
            ),
            Self::MissingBody(id) => write!(
                formatter,
                "rotating contact frontier search referenced missing body {}",
                id.0
            ),
            Self::EarliestContactMissing(pair) => write!(
                formatter,
                "rotating contact frontier earliest pair {}-{} is not in contact in the shared sampled state",
                pair.left.0, pair.right.0
            ),
            Self::EarliestContactChanged(pair) => write!(
                formatter,
                "rotating contact frontier earliest pair {}-{} changed contact geometry during shared-state reconstruction",
                pair.left.0, pair.right.0
            ),
            Self::Search(error) => write!(formatter, "rotating contact frontier search failed: {error}"),
            Self::BroadPhase(error) => write!(
                formatter,
                "rotating contact frontier broad phase failed: {error}"
            ),
            Self::FreeFlight(error) => write!(
                formatter,
                "rotating contact frontier free flight failed: {error}"
            ),
            Self::Geometry(error) => write!(
                formatter,
                "rotating contact frontier geometry failed: {error}"
            ),
        }
    }
}

impl Error for RotatingContactFrontierError3d {}

impl From<RotatingContactSearchError3d> for RotatingContactFrontierError3d {
    fn from(value: RotatingContactSearchError3d) -> Self {
        Self::Search(value)
    }
}

impl From<RotatingBroadPhaseError3d> for RotatingContactFrontierError3d {
    fn from(value: RotatingBroadPhaseError3d) -> Self {
        Self::BroadPhase(value)
    }
}

impl From<RigidBoxFreeFlightError3d> for RotatingContactFrontierError3d {
    fn from(value: RigidBoxFreeFlightError3d) -> Self {
        Self::FreeFlight(value)
    }
}

impl From<OrientedBoxError3d> for RotatingContactFrontierError3d {
    fn from(value: OrientedBoxError3d) -> Self {
        Self::Geometry(value)
    }
}

/// Finds the earliest sampled contact and reconstructs every contact sharing that exact sampled time.
///
/// # Errors
///
/// Returns [`RotatingContactFrontierError3d`] for invalid search/free-flight state, broad-phase or exact
/// OBB failures, missing body identity, or if the selected earliest contact cannot be reproduced exactly in
/// the shared sampled state.
pub fn earliest_rotating_contact_frontier(
    boxes: &[RigidBox3d],
    config: RotatingContactSearchConfig3d,
) -> Result<Option<RotatingContactFrontier3d>, RotatingContactFrontierError3d> {
    let mut broad_phase = RotatingBroadPhase3d::default();
    earliest_rotating_contact_frontier_with_broad_phase(boxes, config, &mut broad_phase)
}

pub(crate) fn earliest_rotating_contact_frontier_with_broad_phase(
    boxes: &[RigidBox3d],
    config: RotatingContactSearchConfig3d,
    broad_phase: &mut RotatingBroadPhase3d,
) -> Result<Option<RotatingContactFrontier3d>, RotatingContactFrontierError3d> {
    let Some(earliest) =
        sampled_rotating_contact_search_with_broad_phase(boxes, config, broad_phase)?
    else {
        return Ok(None);
    };
    reconstruct_frontier(boxes, config, earliest, broad_phase)
}

/// Finds the earliest strictly-positive sampled re-contact and reconstructs every contact sharing that
/// exact sampled time.
///
/// # Errors
///
/// Returns [`RotatingContactFrontierError3d`] under the same fail-closed rules as
/// [`earliest_rotating_contact_frontier`].
pub fn next_rotating_contact_frontier(
    boxes: &[RigidBox3d],
    config: RotatingContactSearchConfig3d,
) -> Result<Option<RotatingContactFrontier3d>, RotatingContactFrontierError3d> {
    let mut broad_phase = RotatingBroadPhase3d::default();
    next_rotating_contact_frontier_with_broad_phase(boxes, config, &mut broad_phase)
}

pub(crate) fn next_rotating_contact_frontier_with_broad_phase(
    boxes: &[RigidBox3d],
    config: RotatingContactSearchConfig3d,
    broad_phase: &mut RotatingBroadPhase3d,
) -> Result<Option<RotatingContactFrontier3d>, RotatingContactFrontierError3d> {
    let Some(earliest) =
        sampled_rotating_recontact_search_with_broad_phase(boxes, config, broad_phase)?
    else {
        return Ok(None);
    };
    reconstruct_frontier(boxes, config, earliest, broad_phase)
}

fn reconstruct_frontier(
    boxes: &[RigidBox3d],
    config: RotatingContactSearchConfig3d,
    earliest: RotatingContactSearchHit3d,
    broad_phase: &mut RotatingBroadPhase3d,
) -> Result<Option<RotatingContactFrontier3d>, RotatingContactFrontierError3d> {
    if earliest.time.denominator == 0 || earliest.time.numerator > earliest.time.denominator {
        return Err(RotatingContactFrontierError3d::InvalidSearchTime(
            earliest.time,
        ));
    }

    let sampled_boxes = boxes
        .iter()
        .map(|rigid_box| {
            sample_rigid_box_free_flight(
                rigid_box,
                config.free_flight,
                earliest.time.numerator,
                earliest.time.denominator,
            )
        })
        .collect::<Result<Vec<_>, _>>()?;
    let by_id = sampled_boxes
        .iter()
        .enumerate()
        .map(|(index, rigid_box)| (rigid_box.body().id(), index))
        .collect::<BTreeMap<_, _>>();
    let zero_time = RigidBoxFreeFlightError3d::validated_zero_time(config.free_flight)?;
    let candidates = broad_phase.candidate_pairs(&sampled_boxes, zero_time)?;
    let mut contacts = Vec::new();
    for pair in candidates {
        let left_index = *by_id
            .get(&pair.left)
            .ok_or(RotatingContactFrontierError3d::MissingBody(pair.left))?;
        let right_index = *by_id
            .get(&pair.right)
            .ok_or(RotatingContactFrontierError3d::MissingBody(pair.right))?;
        let Some(contact) = obb_contact_seed(
            sampled_boxes[left_index].oriented_box(),
            sampled_boxes[right_index].oriented_box(),
        )?
        else {
            continue;
        };
        contacts.push(RotatingContactSearchHit3d {
            time: earliest.time,
            pair,
            contact,
        });
    }

    let selected = contacts.iter().find(|candidate| candidate.pair == earliest.pair);
    let Some(selected) = selected else {
        return Err(RotatingContactFrontierError3d::EarliestContactMissing(
            earliest.pair,
        ));
    };
    if selected.contact != earliest.contact {
        return Err(RotatingContactFrontierError3d::EarliestContactChanged(
            earliest.pair,
        ));
    }

    let remaining_numerator = earliest
        .time
        .denominator
        .checked_sub(earliest.time.numerator)
        .ok_or(RotatingContactFrontierError3d::InvalidSearchTime(
            earliest.time,
        ))?;
    Ok(Some(RotatingContactFrontier3d {
        boxes: sampled_boxes,
        time: earliest.time,
        contacts,
        remaining_numerator,
    }))
}

#[cfg(test)]
mod tests {
    use crate::{
        AngularState3d, AngularVelocity3d, BodyId, Orientation3d, RigidBody, RigidBox3d,
        RigidBoxFreeFlightConfig3d, RotatingContactSearchConfig3d, Vec3i,
    };

    use super::{earliest_rotating_contact_frontier, next_rotating_contact_frontier};

    fn dynamic(id: u64, position: Vec3i, velocity: Vec3i) -> RigidBox3d {
        RigidBox3d::new(
            RigidBody::dynamic(BodyId(id), position, velocity, Vec3i::new(1, 1, 1)),
            AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
        )
        .expect("valid dynamic box")
    }

    fn fixed(id: u64, position: Vec3i) -> RigidBox3d {
        RigidBox3d::new(
            RigidBody::fixed(BodyId(id), position, Vec3i::new(1, 1, 1)),
            AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
        )
        .expect("valid fixed box")
    }

    fn config() -> RotatingContactSearchConfig3d {
        RotatingContactSearchConfig3d::new(
            RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 1, 1),
            4,
            3,
        )
    }

    #[test]
    fn earliest_frontier_collects_shared_time_contacts() {
        let moving = dynamic(10, Vec3i::new(-10, 0, 0), Vec3i::new(20, 0, 0));
        let first = fixed(1, Vec3i::ZERO);
        let second = fixed(2, Vec3i::ZERO);
        let frontier = earliest_rotating_contact_frontier(&[moving, first, second], config())
            .expect("valid frontier")
            .expect("shared frontier");
        assert_eq!(frontier.contacts.len(), 2);
        assert_eq!(frontier.contacts[0].time, frontier.contacts[1].time);
    }

    #[test]
    fn next_frontier_skips_persistent_zero_time_contacts() {
        let persistent = dynamic(1, Vec3i::ZERO, Vec3i::ZERO);
        let persistent_fixed = fixed(2, Vec3i::new(2, 0, 0));
        let moving = dynamic(10, Vec3i::new(-10, 4, 0), Vec3i::new(20, 0, 0));
        let obstacle = fixed(11, Vec3i::new(0, 4, 0));
        let frontier = next_rotating_contact_frontier(
            &[persistent, persistent_fixed, moving, obstacle],
            config(),
        )
        .expect("valid recontact frontier")
        .expect("positive frontier");
        assert!(frontier.time.numerator > 0);
    }
}
