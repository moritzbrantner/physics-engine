use crate::{
    BodyId, BodyKind, ObbContactResponseError3d, OrientedBox3d, RigidBox3d,
    RotatingContactResponseError3d, RotatingWorldConfig3d, RotatingWorldError3d,
    RotatingWorldStepReport3d, Vec3i, obb_response::resolve_obb_contact,
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
/// For a dynamic/dynamic pair, the pair resolver's exact minimum-translation vector is reconstructed from
/// its two position deltas and assigned wholly to the later body in canonical `BodyId` order. This is the
/// integer analogue of a Gauss-Seidel residual sweep: it avoids round-to-nearest half-unit oscillation in
/// touching stacks while remaining independent of insertion order. Fixed/dynamic pairs retain the ordinary
/// full projection away from the fixed body.
///
/// The sweep is independently bounded and fails closed if it cannot reach an idempotent position state.
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

                    changed |= stabilize_pair_positions(&mut boxes, left_index, right_index)?;
                }
            }
            if !changed {
                converged = true;
                break;
            }
        }

        if !converged {
            return Err(RotatingWorldError3d::PersistentTailResolutionLimit(
                u32::from(MAX_POSITION_STABILIZATION_PASSES),
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

fn stabilize_pair_positions(
    boxes: &mut [RigidBox3d],
    left_index: usize,
    right_index: usize,
) -> Result<bool, RotatingWorldError3d> {
    let before_left = boxes[left_index].body.position;
    let before_right = boxes[right_index].body.position;
    let response = resolve_obb_contact(
        boxes[left_index].clone(),
        boxes[right_index].clone(),
        false,
    )
    .map_err(|error| RotatingWorldError3d::Response(RotatingContactResponseError3d::Pair(error)))?;
    let pair_changed = response.left.body.position != before_left
        || response.right.body.position != before_right;
    if !pair_changed {
        return Ok(false);
    }

    match (boxes[left_index].body.kind, boxes[right_index].body.kind) {
        (BodyKind::Fixed, BodyKind::Fixed) => Ok(false),
        (BodyKind::Fixed, BodyKind::Dynamic) => {
            boxes[right_index].body.position = response.right.body.position;
            Ok(true)
        }
        (BodyKind::Dynamic, BodyKind::Fixed) => {
            boxes[left_index].body.position = response.left.body.position;
            Ok(true)
        }
        (BodyKind::Dynamic, BodyKind::Dynamic) => {
            let correction = correction_from_pair_response(
                before_left,
                response.left.body.position,
                before_right,
                response.right.body.position,
            )?;
            boxes[right_index].body.position = offset_position(before_right, correction)?;
            Ok(true)
        }
    }
}

fn correction_from_pair_response(
    before_left: Vec3i,
    after_left: Vec3i,
    before_right: Vec3i,
    after_right: Vec3i,
) -> Result<[i64; 3], RotatingWorldError3d> {
    let left_delta = position_delta(before_left, after_left);
    let right_delta = position_delta(before_right, after_right);
    Ok([
        right_delta[0]
            .checked_sub(left_delta[0])
            .ok_or_else(position_overflow)?,
        right_delta[1]
            .checked_sub(left_delta[1])
            .ok_or_else(position_overflow)?,
        right_delta[2]
            .checked_sub(left_delta[2])
            .ok_or_else(position_overflow)?,
    ])
}

fn position_delta(before: Vec3i, after: Vec3i) -> [i64; 3] {
    [
        i64::from(after.x) - i64::from(before.x),
        i64::from(after.y) - i64::from(before.y),
        i64::from(after.z) - i64::from(before.z),
    ]
}

fn offset_position(position: Vec3i, delta: [i64; 3]) -> Result<Vec3i, RotatingWorldError3d> {
    Ok(Vec3i::new(
        checked_position_axis(position.x, delta[0])?,
        checked_position_axis(position.y, delta[1])?,
        checked_position_axis(position.z, delta[2])?,
    ))
}

fn checked_position_axis(current: i32, delta: i64) -> Result<i32, RotatingWorldError3d> {
    let value = i64::from(current)
        .checked_add(delta)
        .ok_or_else(position_overflow)?;
    i32::try_from(value).map_err(|_| position_overflow())
}

fn position_overflow() -> RotatingWorldError3d {
    RotatingWorldError3d::Response(RotatingContactResponseError3d::Pair(
        ObbContactResponseError3d::ArithmeticOverflow,
    ))
}
