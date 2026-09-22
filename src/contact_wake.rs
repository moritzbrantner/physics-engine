//! Read-only admission guard for bodies represented by parked collision proxies.
//!
//! A broad-phase candidate is not a wake command. Only contacts admitted by the existing
//! rigid/ballistic narrow phase can request activation. The world retries an uncommitted
//! step after restoring the affected dynamic island; even same-step ricochets are covered.
use std::collections::BTreeMap;

use crate::{
    BodyId, BodyKind, InteractionPolicies3d, RigidBox3d, RigidBoxFreeFlightConfig3d,
    RotatingContactResponseError3d, RotatingContactSearchHit3d, Vec3i, WakePropagation3d,
    ballistic_event::BallisticFrontier3d, rigid_box_free_flight_sweep_bounds,
};

pub(crate) struct ContactWakeGuard3d<'a> {
    pub parked: &'a BTreeMap<BodyId, RigidBox3d>,
    pub policies: &'a InteractionPolicies3d,
}

impl ContactWakeGuard3d<'_> {
    pub fn rigid_contacts(
        &self,
        boxes: &[RigidBox3d],
        contacts: &[RotatingContactSearchHit3d],
    ) -> Result<(), RotatingContactResponseError3d> {
        for contact in contacts {
            for (source_id, target_id) in [
                (contact.pair.left, contact.pair.right),
                (contact.pair.right, contact.pair.left),
            ] {
                let Some(target) = self.parked.get(&target_id) else {
                    continue;
                };
                if self
                    .policies
                    .execution_plan_for_bodies(source_id, target_id)
                    .wake_propagation()
                    != WakePropagation3d::Full
                {
                    continue;
                }
                let source = boxes
                    .iter()
                    .find(|body| body.body().id() == source_id)
                    .ok_or(RotatingContactResponseError3d::MissingBody(source_id))?;
                // Actual fixed geometry (including other parked proxies) cannot deliver an impulse.
                if source.body().kind() != BodyKind::Dynamic {
                    continue;
                }
                let bounds = rigid_box_free_flight_sweep_bounds(
                    source,
                    RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 0, 1),
                )?;
                if !crate::linear_contact::sweep_is_passive_support(source, bounds, target) {
                    return Err(RotatingContactResponseError3d::ParkedBodyContact(target_id));
                }
            }
        }
        Ok(())
    }

    pub fn ballistic_contacts(
        &self,
        frontier: &BallisticFrontier3d,
    ) -> Result<(), RotatingContactResponseError3d> {
        for candidate in &frontier.hits {
            if self.parked.contains_key(&candidate.hit.body)
                && self
                    .policies
                    .execution_plan_for_bodies(candidate.projectile, candidate.hit.body)
                    .wake_propagation()
                    == WakePropagation3d::Full
            {
                return Err(RotatingContactResponseError3d::ParkedBodyContact(
                    candidate.hit.body,
                ));
            }
        }
        Ok(())
    }
}
