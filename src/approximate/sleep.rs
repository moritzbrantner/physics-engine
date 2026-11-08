//! Game-owned sleep: a game settles and wakes its own bodies (#524).

use super::{BodyId, Error, PreparedResponse, Vector, World, step};

impl World {
    /// Whether the body sleeps; `false` for an unknown body. Fixed and external bodies never do.
    pub fn is_sleeping(&self, id: BodyId) -> bool {
        self.body(id).is_some_and(|b| b.is_sleeping())
    }
    /// Puts a dynamic body to sleep or wakes it, as Rapier's `RigidBody::sleep()` and
    /// `wake_up`. Sleeping zeroes its velocities and drops its pending forces and impulses;
    /// it stays put, takes no work in a step and still wakes on contact as a body the engine
    /// put to sleep does. Waking reaches the sleeping bodies touching it, as an impulse does.
    /// Fixed and external bodies are rejected, and a body already in the requested state is
    /// left alone. A game owning sleep sets [`Body::sleep_allowed`](super::Body) to `false`
    /// so the engine's quiet timers stay out of it.
    pub fn set_sleeping(&mut self, id: BodyId, sleeping: bool) -> Result<(), Error> {
        self.sort_storage();
        let i = self.index(id)?;
        if !self.bodies[i].movable() {
            return Err(Error::InvalidInput);
        }
        if self.bodies[i].sleeping == sleeping {
            return Ok(());
        }
        if !sleeping {
            self.wake_island(self.bodies[i].order);
            return Ok(());
        }
        let b = &mut self.bodies[i];
        b.sleeping = true;
        b.quiet_time = 0.0;
        b.velocity = Vector::ZERO;
        b.angular_velocity = Vector::ZERO;
        b.force = Vector::ZERO;
        b.torque = Vector::ZERO;
        b.impulse = Vector::ZERO;
        b.angular_impulse = Vector::ZERO;
        let s = &mut self.bookkeeping;
        s.activity.count -= 1;
        if !s.activity.dirty
            && let Ok(at) = s.activity.indices.binary_search(&i)
        {
            s.activity.indices.remove(at);
        }
        // Resting values (`Scratch::per_body`), where they are current.
        if let Some(response) = self.responses.get_mut(i) {
            *response = PreparedResponse::default();
        }
        if let Some(supported) = s.supported.get_mut(i) {
            *supported = true;
        }
        step::rest_broad_phase(
            &self.bodies[i],
            s.generation,
            (&mut s.broad, &mut s.broad_epoch, &mut s.boxes_live),
            &mut self.query_tree,
            self.scene.contact_slop,
        );
        self.park_pending = true;
        Ok(())
    }
}
