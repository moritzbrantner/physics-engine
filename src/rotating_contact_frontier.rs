use std::{collections::BTreeMap, error::Error, fmt};

use crate::{
    BodyId, OrientedBoxError3d, RigidBox3d, RigidBoxFreeFlightError3d, RotatingBroadPhaseError3d,
    RotatingContactSearchConfig3d, RotatingContactSearchError3d, RotatingContactSearchHit3d,
    RotationalSweepPair3d, SampledContactTime3d, obb_contact_seed,
    rotational_sweep_candidate_pairs, sample_rigid_box_free_flight,
    sampled_rotating_contact_search,
};

/// One shared pre-response world reconstructed at the globally earliest sampled rotating contact time.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RotatingContactFrontier3d {
    /// Every rotating body sampled directly from the common interval start at [`Self::time`].
    pub boxes: Vec<RigidBox3d>,
    /// Earliest sampled contact fraction admitted by the configured search policy.
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
                "rotating contact search returned invalid fraction {}/{}",
                time.numerator, time.denominator
            ),
            Self::MissingBody(id) => write!(
                formatter,
                "rotating contact frontier cannot find body {} in the sampled world",
                id.0
            ),
            Self::EarliestContactMissing(pair) => write!(
                formatter,
                "rotating contact frontier pair {}-{} is no longer in contact at the sampled time",
                pair.left.0, pair.right.0
            ),
            Self::EarliestContactChanged(pair) => write!(
                formatter,
                "rotating contact frontier pair {}-{} no longer matches the search contact evidence",
                pair.left.0, pair.right.0
            ),
            Self::Search(error) => write!(
                formatter,
                "rotating contact frontier search failed: {error}"
            ),
            Self::BroadPhase(error) => {
                write!(
                    formatter,
                    "rotating contact frontier broad phase failed: {error}"
                )
            }
            Self::FreeFlight(error) => {
                write!(
                    formatter,
                    "rotating contact frontier free flight failed: {error}"
                )
            }
            Self::Geometry(error) => {
                write!(
                    formatter,
                    "rotating contact frontier OBB query failed: {error}"
                )
            }
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

/// Reconstructs the globally earliest sampled rotating-contact frontier from one common interval start.
///
/// [`sampled_rotating_contact_search`] supplies the earliest admitted sampled contact fraction. Every body
/// is then sampled directly from the original state at exactly that rational fraction. The frontier
/// re-evaluates every conservative rotational broad-phase candidate in that shared state and retains all
/// equal-time OBB contacts, so a later response solver can consume one deterministic contact set rather
/// than resolving independently sampled pairs in discovery order.
///
/// The original earliest hit must still exist with identical contact evidence in the reconstructed state;
/// drift fails closed. This remains **sampled rotational collision handling, not analytic rotational CCD**:
/// contact or separation islands that exist wholly between coarse samples can still be missed.
///
/// # Errors
///
/// Returns [`RotatingContactFrontierError3d`] when search, conservative broad phase, free-flight sampling,
/// OBB geometry, body identity, or reconstructed contact evidence is inconsistent.
pub fn earliest_rotating_contact_frontier(
    boxes: &[RigidBox3d],
    config: RotatingContactSearchConfig3d,
) -> Result<Option<RotatingContactFrontier3d>, RotatingContactFrontierError3d> {
    let Some(earliest) = sampled_rotating_contact_search(boxes, config)? else {
        return Ok(None);
    };
    if earliest.time.denominator == 0 || earliest.time.numerator > earliest.time.denominator {
        return Err(RotatingContactFrontierError3d::InvalidSearchTime(
            earliest.time,
        ));
    }

    let sampled = boxes
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
    let indices = sampled
        .iter()
        .enumerate()
        .map(|(index, rigid_box)| (rigid_box.body().id(), index))
        .collect::<BTreeMap<_, _>>();
    let candidates = rotational_sweep_candidate_pairs(boxes, config.free_flight)?;
    let mut contacts = Vec::new();

    for pair in candidates {
        let left_index = *indices
            .get(&pair.left)
            .ok_or(RotatingContactFrontierError3d::MissingBody(pair.left))?;
        let right_index = *indices
            .get(&pair.right)
            .ok_or(RotatingContactFrontierError3d::MissingBody(pair.right))?;
        let Some(contact) = obb_contact_seed(
            sampled[left_index].oriented_box(),
            sampled[right_index].oriented_box(),
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

    let reconstructed = contacts
        .iter()
        .find(|contact| contact.pair == earliest.pair)
        .ok_or(RotatingContactFrontierError3d::EarliestContactMissing(
            earliest.pair,
        ))?;
    if reconstructed.contact != earliest.contact {
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
        boxes: sampled,
        time: earliest.time,
        contacts,
        remaining_numerator,
    }))
}

#[cfg(test)]
mod tests {
    use crate::{
        AngularState3d, AngularVelocity3d, BodyId, Orientation3d, RigidBody, RigidBox3d,
        RigidBoxFreeFlightConfig3d, RotatingContactSearchConfig3d, RotationalSweepPair3d,
        SampledContactTime3d, Vec3i,
    };

    use super::earliest_rotating_contact_frontier;

    fn dynamic(id: u64, position: Vec3i, velocity: Vec3i, half_extents: Vec3i) -> RigidBox3d {
        RigidBox3d::new(
            RigidBody::dynamic(BodyId(id), position, velocity, half_extents),
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
        RotatingContactSearchConfig3d::new(RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 1, 1), 4, 3)
    }

    #[test]
    fn initial_equal_time_contacts_form_one_shared_frontier() {
        let boxes = [
            dynamic(1, Vec3i::ZERO, Vec3i::ZERO, Vec3i::new(1, 1, 1)),
            fixed(2, Vec3i::new(-2, 0, 0)),
            fixed(3, Vec3i::new(2, 0, 0)),
        ];
        let frontier = earliest_rotating_contact_frontier(&boxes, config())
            .expect("valid frontier")
            .expect("initial contacts");

        assert_eq!(frontier.time, SampledContactTime3d::ZERO);
        assert_eq!(frontier.remaining_numerator, 1);
        assert_eq!(frontier.boxes, boxes);
        assert_eq!(frontier.contacts.len(), 2);
        assert_eq!(
            frontier.contacts[0].pair,
            RotationalSweepPair3d {
                left: BodyId(1),
                right: BodyId(2),
            }
        );
        assert_eq!(
            frontier.contacts[1].pair,
            RotationalSweepPair3d {
                left: BodyId(1),
                right: BodyId(3),
            }
        );
    }

    #[test]
    fn nonzero_search_hit_advances_the_entire_world_to_one_fraction() {
        let boxes = [
            dynamic(
                2,
                Vec3i::new(-10, 0, 0),
                Vec3i::new(20, 0, 0),
                Vec3i::new(1, 1, 1),
            ),
            fixed(8, Vec3i::ZERO),
        ];
        let frontier = earliest_rotating_contact_frontier(&boxes, config())
            .expect("valid frontier")
            .expect("sampled contact");

        assert_eq!(
            frontier.time,
            SampledContactTime3d {
                numerator: 3,
                denominator: 8,
            }
        );
        assert_eq!(frontier.remaining_numerator, 5);
        assert_eq!(frontier.contacts.len(), 1);
        assert_ne!(
            frontier.boxes[0].body().position(),
            boxes[0].body().position()
        );
    }

    #[test]
    fn reconstructed_frontier_is_bit_for_bit_repeatable() {
        let boxes = [
            dynamic(
                2,
                Vec3i::new(-10, 0, 0),
                Vec3i::new(20, 0, 0),
                Vec3i::new(1, 1, 1),
            ),
            fixed(8, Vec3i::ZERO),
        ];

        let first = earliest_rotating_contact_frontier(&boxes, config()).expect("first frontier");
        let second = earliest_rotating_contact_frontier(&boxes, config()).expect("second frontier");
        assert_eq!(first, second);
    }
}
