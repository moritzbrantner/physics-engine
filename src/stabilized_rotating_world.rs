use crate::{
    BodyId, BodyKind, OrientedBox3d, RigidBox3d, RotatingContactResponseError3d,
    RotatingWorldConfig3d, RotatingWorldError3d, RotatingWorldStepReport3d, Vec3i,
    obb_response::resolve_obb_contact, rotating_world::RotatingWorld3d as InnerRotatingWorld3d,
};

const MAX_FIXED_POSITION_STABILIZATION_PASSES: u8 = 16;

/// Engine-owned rotating world with a bounded fixed-boundary position stabilization tail.
///
/// The inner rotating solver remains the sole authority for impulses, dynamic/dynamic response,
/// restitution, friction, and angular response. After one requested non-zero world step, this facade only
/// removes residual integer penetration between fixed and dynamic bodies. Only the dynamic body's projected
/// position is committed; every velocity and orientation result from the simultaneous solver is preserved.
///
/// The requested frame is staged on a clone and committed only after both the authoritative step and the
/// fixed-boundary stabilization succeed. Failed frames therefore leave the public world unchanged. A zero
/// timestep keeps the inner world's exact no-op contract and deliberately skips stabilization.
///
/// Multiple fixed boundaries can still constrain one body, so the position-only pass is independently
/// bounded and fails closed if those fixed constraints cannot reach an idempotent state.
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
        if timestep_numerator == 0 {
            return self.inner.step(timestep_numerator, timestep_denominator);
        }

        let mut staged = self.clone();
        let report = staged
            .inner
            .step(timestep_numerator, timestep_denominator)?;
        staged.stabilize_fixed_boundaries()?;
        *self = staged;
        Ok(report)
    }

    fn stabilize_fixed_boundaries(&mut self) -> Result<(), RotatingWorldError3d> {
        let mut boxes = self.inner.boxes().cloned().collect::<Vec<_>>();
        if boxes.len() < 2 {
            return Ok(());
        }

        let mut converged = false;
        for _ in 0..MAX_FIXED_POSITION_STABILIZATION_PASSES {
            let mut changed = false;
            for left_index in 0..boxes.len() {
                for right_index in (left_index + 1)..boxes.len() {
                    let dynamic_index =
                        match (boxes[left_index].body.kind, boxes[right_index].body.kind) {
                            (BodyKind::Fixed, BodyKind::Dynamic) => Some(right_index),
                            (BodyKind::Dynamic, BodyKind::Fixed) => Some(left_index),
                            _ => None,
                        };
                    let Some(dynamic_index) = dynamic_index else {
                        continue;
                    };

                    let response = resolve_obb_contact(
                        boxes[left_index].clone(),
                        boxes[right_index].clone(),
                        false,
                    )
                    .map_err(|error| {
                        RotatingWorldError3d::Response(RotatingContactResponseError3d::Pair(error))
                    })?;
                    let projected = if dynamic_index == left_index {
                        response.left.body.position
                    } else {
                        response.right.body.position
                    };
                    if projected != boxes[dynamic_index].body.position {
                        boxes[dynamic_index].body.position = projected;
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
            return Err(RotatingWorldError3d::PersistentTailResolutionLimit(
                u32::from(MAX_FIXED_POSITION_STABILIZATION_PASSES),
            ));
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
