use std::cell::RefCell;

use super::Sandbox;

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
                sandbox.render_role_for(body.id()),
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
        for projectile in sandbox.world.ballistic_spheres() {
            let position = projectile.position();
            let radius = projectile.radius();
            snapshot.extend_from_slice(&[
                3,
                position.x,
                position.y,
                position.z,
                radius,
                radius,
                radius,
                0,
                0,
                0,
                physics_engine::ORIENTATION_SCALE,
            ]);
        }
        snapshot.as_ptr() as usize
    })
}

pub(super) fn len() -> usize {
    SNAPSHOT.with(|snapshot| snapshot.borrow().len())
}

#[cfg(test)]
fn values() -> Vec<i32> {
    SNAPSHOT.with(|snapshot| snapshot.borrow().clone())
}

#[cfg(test)]
mod tests {
    use super::{STRIDE, Sandbox, len, refresh, values};

    #[test]
    fn packed_snapshot_matches_authoritative_body_state() {
        let sandbox = Sandbox::new().expect("valid sandbox");
        let expected = sandbox
            .world
            .boxes()
            .flat_map(|rigid_box| {
                let body = rigid_box.body();
                let angular = rigid_box.angular();
                [
                    sandbox.render_role_for(body.id()),
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

        let pointer = refresh(&sandbox);
        assert_ne!(pointer, 0);
        assert_eq!(len(), expected.len());
        assert_eq!(values(), expected);
        assert_eq!(expected.len(), sandbox.body_count() * STRIDE);
    }
}
