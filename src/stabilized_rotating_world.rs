use crate::{
    BodyId, BodyKind, OrientedBox3d, RigidBox3d, RotatingContactResponseError3d,
    RotatingWorldConfig3d, RotatingWorldError3d, RotatingWorldStepReport3d, Vec3i,
    obb_response::resolve_obb_contact,
    rotating_world::RotatingWorld3d as InnerRotatingWorld3d,
};

const MAX_POSITION_STABILIZATION_PASSES: u8 = 64;

/// Engine-owned rotating world with an exact position-only stabilization tail.
///
/// The inner rotating solver keeps its simultaneous Jacobi impulse response so equal-time constraints
/// remain order-invariant. After one requested world step, this facade performs a canonical body-id ordered
/// projection-only sweep over any residual OBB penetration. Only positions from the pair resolver are
/// committed during this phase: linear/angular velocity, restitution, and friction have already been
/// resolved by the simultaneous solver and are deliberately not applied a second time.
///
/// This closes integer residuals in coupled resting stacks without adding another physical impulse. The
/// sweep is independently bounded and fails closed if it cannot reach an idempotent position state.
#[derive(Clone, Debug)]
pub struct RotatingWorld3d {
    inner: InnerRotatingWorld3d,
}

impl RotatingWorld3d {
    #[must_use]
    pub fn new(config: RotatingWorldConfig3d) -> Self {
        Self {
            inner: InnerRotatingWorld3d::new(config),
        }
    }

    #[must_use]
    pub fn config(&self) -> RotatingWorldConfig3d {
        self.inner.config()
    }

    pub fn add_box(&mut self, rigid_box: RigidBox3d) -> Result<(), RotatingWorldError3d> {
        self.inner.add_box(rigid_box)
    }

    pub fn remove_box(&mut self, id: BodyId) -> Option<RigidBox3d> {
        self.inner.remove_box(id)
    }

    #[must_use]
    pub fn box_by_id(&self, id: BodyId) -> Option<&RigidBox3d> {
        self.inner.box_by_id(id)
    }

    pub fn boxes(&self) -> impl Iterator<Item = &RigidBox3d> {
        self.inner.boxes()
    }

    pub fn set_linear_velocity(
        &mut self,
        id: BodyId,
        velocity: Vec3i,
    ) -> Result<(), RotatingWorldError3d> {
        self.inner.set_linear_velocity(id, velocity)
    }

    pub fn overlap_query(&self, query: OrientedBox3d) -> Result<Vec<BodyId>, RotatingWorldError3d> {
        self.inner.overlap_query(query)
    }

    pub fn step(
        &mut self,
        timestep_numerator: i32,
        timestep_denominator: i32,
    ) -> Result<RotatingWorldStepReport3d, RotatingWorldError3d> {
        let report = self.inner.step(timestep_numerator, timestep_denominator)?;
        self.stabilize_positions()?;
        Ok(report)
    }

    fn stabilize_positions(&mut self) -> Result<(), RotatingWorldError3d> {
        let mut boxes = self.inner.boxes().cloned().collect::<Vec<_>>();
        if boxes.len() < 2 {
            return Ok(());
        }

        let mut converged = false;
        for _ in 0..MAX_POSITION_STABILIZATION_PASSES {
            let mut changed = false;
            for left_index in 0..boxes.len() {
                for right_index in (left_index + 1)..boxes.len() {
                    if boxes[left_index].body.kind == BodyKind::Fixed
                        && boxes[right_index].body.kind == BodyKind::Fixed
                    {
                        continue;
                    }

                    let before_left = boxes[left_index].body.position;
                    let before_right = boxes[right_index].body.position;
                    let response = resolve_obb_contact(
                        boxes[left_index].clone(),
                        boxes[right_index].clone(),
                        false,
                    )
                    .map_err(|error| {
                        RotatingWorldError3d::Response(RotatingContactResponseError3d::Pair(error))
                    })?;
                    let after_left = response.left.body.position;
                    let after_right = response.right.body.position;
                    if after_left != before_left || after_right != before_right {
                        boxes[left_index].body.position = after_left;
                        boxes[right_index].body.position = after_right;
                        changed = true;
                    }
                }
            }
            if !changed {
                converged = true;
                break;
            }
        }

        if !converged {
            return Err(RotatingWorldError3d::PersistentTailResolutionLimit(u32::from(
                MAX_POSITION_STABILIZATION_PASSES,
            )));
        }

        let ids = boxes
            .iter()
            .map(|rigid_box| rigid_box.body.id)
            .collect::<Vec<_>>();
        for id in ids {
            let removed = self
                .inner
                .remove_box(id)
                .ok_or(RotatingWorldError3d::MissingBody(id))?;
            debug_assert_eq!(removed.body.id, id);
        }
        for rigid_box in boxes {
            self.inner.add_box(rigid_box)?;
        }
        Ok(())
    }
}
