//! Native/WASM acceptance driver; never linked into the shipped Pages module.
use physics_engine::{
    BodyId,
    approximate::{
        Body, Checkpoint, CheckpointContext, CheckpointLimits, Config, Error, Shape, Vector as V,
        World,
    },
};

// Fixture identities only. Applications supply their exact build/content identities.
const CONTEXT: CheckpointContext = CheckpointContext {
    build: [71; 32],
    content: [83; 32],
};
type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

fn fixture() -> Result<World> {
    let mut world = World::new(Config {
        gravity: V(0.0, -10.0, 0.0),
        substeps: 2,
        fixed_position_iterations: 2,
        sleep_speed: 0.1,
        sleep_seconds: 0.08,
        ..Config::default()
    })?;
    world.add_body(Body::new(
        BodyId(0),
        Shape::Box(V(100.0, 0.1, 100.0)),
        V(0.0, -0.1, 0.0),
        0.0,
    ))?;
    for id in 1..=8 {
        world.add_body(Body::new(
            BodyId(id),
            Shape::Box(V(0.5, 0.5, 0.5)),
            V(id as f64 * 3.0, 0.5, 0.0),
            1.0,
        ))?;
    }
    world.add_body(Body::new(
        BodyId(20),
        Shape::Box(V(0.1, 3.0, 3.0)),
        V(-10.0, 3.0, 0.0),
        0.0,
    ))?;
    Ok(world)
}
fn physical(world: &World) -> Result<Vec<u8>> {
    Ok(world.checkpoint(CONTEXT)?.to_bytes())
}
fn restore(bytes: &[u8]) -> Result<World> {
    Ok(Checkpoint::from_bytes(bytes, CONTEXT, CheckpointLimits::default())?.restore())
}
fn run() -> Result<()> {
    let mut uninterrupted = fixture()?;
    let mut captured = fixture()?;
    for world in [&mut uninterrupted, &mut captured] {
        for _ in 0..3 {
            world.step(0.01)?;
        }
    }
    // First restore occurs partway through the quiet timer, with real warm starts.
    let saved = physical(&captured)?;
    drop(captured);
    let mut continued = restore(&saved)?;
    let mut retirement_seen = false;
    for tick in 0..100 {
        if tick == 20 {
            if !continued.is_quiescent() {
                return Err("sleep continuation did not settle".into());
            }
            for world in [&mut uninterrupted, &mut continued] {
                world.add_force(BodyId(1), V(0.2, 0.0, 0.0))?;
                let at = world
                    .body(BodyId(2))
                    .ok_or("missing queued-input subject")?
                    .position;
                world.apply_impulse(BodyId(2), V(0.1, 0.0, 0.0), at + V(0.0, 0.2, 0.0))?;
            }
            let saved = physical(&continued)?;
            drop(continued);
            continued = restore(&saved)?;
        }
        if tick == 35 {
            for world in [&mut uninterrupted, &mut continued] {
                world
                    .remove_body(BodyId(0))
                    .ok_or("support removal failed")?;
            }
        }
        if tick == 40 {
            for world in [&mut uninterrupted, &mut continued] {
                world
                    .remove_body(BodyId(3))
                    .ok_or("same-ID removal failed")?;
                world.add_body(Body::new(
                    BodyId(3),
                    Shape::capsule(0.3, 0.2),
                    V(0.0, 10.0, 0.0),
                    0.7,
                ))?;
            }
        }
        if tick == 50 {
            for world in [&mut uninterrupted, &mut continued] {
                let mut shot = Body::new(BodyId(90), Shape::Sphere(0.1), V(-12.0, 3.0, 0.0), 0.4);
                shot.ccd = true;
                shot.retire_on_impact = true;
                shot.velocity = V(80.0, 0.0, 0.0);
                world.add_body(shot)?;
            }
            let saved = physical(&continued)?;
            drop(continued);
            continued = restore(&saved)?;
        }
        if tick == 70 {
            for world in [&mut uninterrupted, &mut continued] {
                let mut bad = Body::new(
                    BodyId(99),
                    Shape::Sphere(0.1),
                    V(1e12 - 0.075, 0.0, 0.0),
                    1.0,
                );
                bad.velocity = V(1.0, 0.0, 0.0);
                world.add_body(bad)?;
            }
            let saved = physical(&continued)?;
            drop(continued);
            continued = restore(&saved)?;
            for _ in 0..2 {
                for world in [&mut uninterrupted, &mut continued] {
                    if !matches!(world.step(0.1), Err(Error::NonFiniteState(BodyId(99)))) {
                        return Err("late failure contract changed".into());
                    }
                }
                if physical(&continued)? != saved || physical(&uninterrupted)? != saved {
                    return Err("failed tick changed physical checkpoint".into());
                }
                let saved = physical(&continued)?;
                drop(continued);
                continued = restore(&saved)?;
            }
            for world in [&mut uninterrupted, &mut continued] {
                world
                    .remove_body(BodyId(99))
                    .ok_or("failure recovery removal failed")?;
            }
        }
        let dt = [0.01, 0.02, 0.005][tick % 3];
        let a = continued.step(dt)?;
        let e = uninterrupted.step(dt)?;
        retirement_seen |= a.retired.contains(&BodyId(90));
        if (
            a.substeps,
            a.contact_points,
            a.woken_bodies,
            a.swept_contacts,
            a.integrated_bodies,
            &a.retired,
            a.max_penetration.to_bits(),
        ) != (
            e.substeps,
            e.contact_points,
            e.woken_bodies,
            e.swept_contacts,
            e.integrated_bodies,
            &e.retired,
            e.max_penetration.to_bits(),
        ) || physical(&continued)? != physical(&uninterrupted)?
        {
            return Err(format!("continuation differs at tick {tick}").into());
        }
    }
    if !retirement_seen {
        return Err("retirement control never hit the wall".into());
    }
    Ok(())
}

#[unsafe(no_mangle)]
pub extern "C" fn checkpoint_continuation() -> i32 {
    run().map_or(-1, |()| 0)
}

#[cfg(test)]
mod tests {
    #[test]
    fn same_build_native_continuation() {
        super::run().unwrap();
        super::run().unwrap();
    }
}
