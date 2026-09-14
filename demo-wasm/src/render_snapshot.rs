use std::cell::RefCell;

use super::{Sandbox, role_for};

pub(super) const STRIDE: usize = 11;

std::thread_local! {
    static SNAPSHOT: RefCell<Vec<i32>> = const { RefCell::new(Vec::new()) };
}

pub(super) fn refresh(sandbox: &Sandbox) -> usize {
    SNAPSHOT.with(|snapshot| {
        let mut snapshot = snapshot.borrow_mut();
        snapshot.clear();
        snapshot.reserve(sandbox.body_count().saturating_mul(STRIDE));
        for rigid_box in sandbox.world.boxes() {
            let body = rigid_box.body();
            let angular = rigid_box.angular();
            snapshot.extend_from_slice(&[
                role_for(body.id()),
                body.position().x,
                body.position().y,
                body.position().z,
                body.half_extents().x,
                body.half_extents().y,
                body.half_extents().z,
                angular.orientation.x,
                angular.orientation.y,
                angular.orientation.z,
                angular.orientation.w,
            ]);
        }
        snapshot.as_ptr() as usize
    })
}

pub(super) fn len() -> usize {
    SNAPSHOT.with(|snapshot| snapshot.borrow().len())
}

#[cfg(test)]
mod tests {
    use super::{STRIDE, Sandbox};
    use crate::role_for;

    #[test]
    fn packed_snapshot_matches_authoritative_body_state() {
        let sandbox = Sandbox::new().expect("valid sandbox");
        let values = sandbox
            .world
            .boxes()
            .flat_map(|rigid_box| {
                let body = rigid_box.body();
                let angular = rigid_box.angular();
                [
                    role_for(body.id()),
                    body.position().x,
                    body.position().y,
                    body.position().z,
                    body.half_extents().x,
                    body.half_extents().y,
                    body.half_extents().z,
                    angular.orientation.x,
                    angular.orientation.y,
                    angular.orientation.z,
                    angular.orientation.w,
                ]
            })
            .collect::<Vec<_>>();

        assert_eq!(values.len(), sandbox.body_count() * STRIDE);
        assert_eq!(values.chunks_exact(STRIDE).len(), sandbox.body_count());
    }
}
