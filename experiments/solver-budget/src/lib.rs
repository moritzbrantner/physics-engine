//! Diagnostic-only fixtures and observations for the existing f64 solver.
//! No production defaults, collision admission or engine tolerances are changed here.
use physics_engine::{
    BodyId,
    approximate::{Body, Config, Error, Shape, Vector as V, World},
};
use std::cell::RefCell;

pub const DT: f64 = 1.0 / 60.0;
pub const STRIDE: usize = 21;
const BOX_START: u64 = 100;
const SHOT_START: u64 = 100_000;

pub struct Fixture {
    pub world: World,
    count: usize,
    scene: u32,
    tick: u32,
    next_id: u64,
    starts: Vec<V>,
    snapshot: Vec<f64>,
    metrics: [f64; 12],
    shots: u32,
    retired: u32,
    error: bool,
}
impl Fixture {
    /// scene 0: sustained contact; 1: sustained contact plus three mixed volleys;
    /// scene 2: sleeping-enabled unforced settling control (not capacity evidence);
    /// scene 3: one-layer, awake touching carpet (lower contact coupling).
    pub fn new(
        count: usize,
        scene: u32,
        substeps: u8,
        velocity: u8,
        position: u8,
    ) -> Result<Self, Error> {
        if !(8..=2048).contains(&count) || !count.is_power_of_two() || scene > 3 {
            return Err(Error::InvalidInput);
        }
        let mut world = World::new(Config {
            substeps,
            velocity_iterations: velocity,
            fixed_position_iterations: position,
            ..Config::default()
        })?;
        let levels = if scene == 3 { 1 } else { 4 };
        let cells = count / levels;
        let mut columns = 1;
        while columns * columns < cells {
            columns *= 2;
        }
        let depth = cells / columns;
        let mut floor = Body::new(
            BodyId(1),
            Shape::Box(V(5000.0, 16.0, 5000.0)),
            V(0.0, -16.0, 0.0),
            0.0,
        );
        floor.friction = 1.0;
        world.add_body(floor)?;
        let mut starts = Vec::with_capacity(count);
        // Stack fixtures have four levels at every size; carpet has one.
        // The 32-box stack is 4 x 2 x 4, with the Pages crate size/mass/material.
        for level in 0..levels {
            for row in 0..depth {
                for col in 0..columns {
                    let p = V(
                        (col as f64 - (columns - 1) as f64 * 0.5) * 36.0,
                        18.0 + level as f64 * 36.0,
                        (row as f64 - (depth - 1) as f64 * 0.5) * 36.0,
                    );
                    let mut b = Body::new(
                        BodyId(BOX_START + starts.len() as u64),
                        Shape::Box(V(18.0, 18.0, 18.0)),
                        p,
                        2.0,
                    );
                    b.friction = 1.0;
                    b.sleep_allowed = scene == 2;
                    world.add_body(b)?;
                    starts.push(p);
                }
            }
        }
        Ok(Self {
            world,
            count,
            scene,
            tick: 0,
            next_id: SHOT_START,
            starts,
            snapshot: Vec::new(),
            metrics: [0.0; 12],
            shots: 0,
            retired: 0,
            error: false,
        })
    }
    fn shoot_wave(&mut self, kind: u32) -> Result<(), Error> {
        let cells = self.count / 4;
        let mut columns = 1;
        while columns * columns < cells {
            columns *= 2;
        }
        let depth = cells / columns;
        // One projectile per four columns of the front face. Wave schedule/strength never
        // depends on solver settings; projectile creation and retirement are timed as workload.
        for col in (0..columns).step_by(4) {
            let x = (col as f64 - (columns - 1) as f64 * 0.5) * 36.0 + 4.0;
            let shape = match kind {
                0 => Shape::Sphere(3.0),
                1 => Shape::Box(V(1.0, 1.0, 9.0)),
                _ => Shape::Box(V(3.0, 3.0, 3.0)),
            };
            let mut p = Body::new(
                BodyId(self.next_id),
                shape,
                V(x, 50.0, (depth - 1) as f64 * 18.0 + 180.0),
                1.0,
            );
            p.velocity = V(0.0, 0.0, -5760.0);
            p.friction = 0.0;
            p.ccd = true;
            p.retire_on_impact = true;
            p.rotation_locked = kind == 1;
            p.sleep_allowed = false;
            self.world.add_body(p)?;
            self.next_id += 1;
            self.shots += 1;
        }
        Ok(())
    }
    pub fn step(&mut self) -> Result<(), Error> {
        if self.scene == 1 && [60, 120, 180].contains(&self.tick) {
            self.shoot_wave(self.tick / 60 - 1)?;
        }
        let r = self.world.step(DT)?;
        self.retired += r.retired.len() as u32;
        let departing: Vec<_> = self
            .world
            .bodies()
            .filter(|b| b.id.0 >= SHOT_START && b.position.abs().max_component() > 4500.0)
            .map(|b| b.id)
            .collect();
        for id in departing {
            self.world.remove_body(id);
        }
        self.tick += 1;
        Ok(())
    }
    /// Separate from timed step: observe full state and independent SAT overlap, not only
    /// the solver's pre-correction penetration counter. This never changes physical state.
    pub fn observe(&mut self) {
        self.snapshot.clear();
        let mut boxes = Vec::with_capacity(self.count);
        self.metrics = [0.0; 12];
        self.metrics[0] = 1.0; // finite, normalized and expected body inventory
        let mut energy = 0.0;
        for b in self.world.bodies() {
            let h = b.shape.half_extents();
            let q = b.orientation;
            let state = [
                b.id.0 as f64,
                b.position.0,
                b.position.1,
                b.position.2,
                h.0,
                h.1,
                h.2,
                q.0,
                q.1,
                q.2,
                q.3,
                b.velocity.0,
                b.velocity.1,
                b.velocity.2,
                b.angular_velocity.0,
                b.angular_velocity.1,
                b.angular_velocity.2,
                f64::from(b.is_sleeping()),
                b.mass,
                self.world.elapsed_seconds(),
                f64::from(b.rotation_locked),
            ];
            if !state.iter().all(|x| x.is_finite())
                || ((q.0 * q.0 + q.1 * q.1 + q.2 * q.2 + q.3 * q.3) - 1.0).abs() > 1e-8
            {
                self.metrics[0] = 0.0;
            }
            self.snapshot.extend(state);
            if (BOX_START..BOX_START + self.count as u64).contains(&b.id.0) {
                let a = ObservedBox::new(b);
                self.metrics[1] = self.metrics[1].max(-a.lo.1);
                self.metrics[3] += f64::from(!b.is_sleeping());
                let old = self.starts[(b.id.0 - BOX_START) as usize];
                self.metrics[6] = self.metrics[6].max((b.position - old).length());
                self.metrics[7] = self.metrics[7]
                    .max((b.velocity.length() + b.angular_velocity.length() * h.length()).abs());
                energy += 0.5 * b.mass * b.velocity.dot(b.velocity);
                let w = q.inverse_rotate(b.angular_velocity);
                energy += b.mass / 6.0
                    * ((h.1 * h.1 + h.2 * h.2) * w.0 * w.0
                        + (h.0 * h.0 + h.2 * h.2) * w.1 * w.1
                        + (h.0 * h.0 + h.1 * h.1) * w.2 * w.2);
                boxes.push(a);
            }
        }
        self.metrics[9] = boxes.len() as f64;
        if boxes.len() != self.count || self.world.body(BodyId(1)).is_none() {
            self.metrics[0] = 0.0;
        }
        self.metrics[8] = energy;
        boxes.sort_by(|a, b| a.lo.0.total_cmp(&b.lo.0).then(a.id.cmp(&b.id)));
        let mut touching = vec![false; boxes.len()];
        for i in 0..boxes.len() {
            for j in i + 1..boxes.len() {
                if boxes[j].lo.0 > boxes[i].hi.0 + 0.02 {
                    break;
                }
                if boxes[j].lo.1 > boxes[i].hi.1 + 0.02
                    || boxes[i].lo.1 > boxes[j].hi.1 + 0.02
                    || boxes[j].lo.2 > boxes[i].hi.2 + 0.02
                    || boxes[i].lo.2 > boxes[j].hi.2 + 0.02
                {
                    continue;
                }
                let d = overlap(&boxes[i], &boxes[j]);
                if d >= -0.02 {
                    self.metrics[2] = self.metrics[2].max(d);
                    self.metrics[4] += 1.0;
                    touching[i] = true;
                    touching[j] = true;
                }
            }
        }
        self.metrics[5] = touching.iter().filter(|&&v| v).count() as f64;
        self.metrics[10] = self.shots as f64;
        self.metrics[11] = self.retired as f64;
    }
    fn stat(&self, index: u32) -> f64 {
        let r = &self.world.last_report;
        match index {
            0 => r.substeps as f64,
            1 => r.impulse_iterations as f64,
            2 => r.position.passes as f64,
            3 => r.contact_points as f64,
            4 => r.pair_tests as f64,
            5 => r.narrow_tests as f64,
            6 => r.integrated_bodies as f64,
            7 => r.convergence.constraint_visits as f64,
            8 => r.convergence.capped_substeps as f64,
            9 => r.convergence.converged_substeps as f64,
            10 => r.position.contact_tests as f64,
            11 => r.position.corrections as f64,
            12 => r.swept_contacts as f64,
            13 => r.bookkeeping.scratch_retained_bytes as f64,
            14 => self.world.elapsed_seconds(),
            _ => f64::NAN,
        }
    }
}
struct ObservedBox {
    id: u64,
    p: V,
    h: V,
    axes: [V; 3],
    lo: V,
    hi: V,
}
impl ObservedBox {
    fn new(b: &Body) -> Self {
        let axes = [
            b.orientation.rotate(V::X),
            b.orientation.rotate(V::Y),
            b.orientation.rotate(V::Z),
        ];
        let h = b.shape.half_extents();
        let extent = axes[0].abs() * h.0 + axes[1].abs() * h.1 + axes[2].abs() * h.2;
        Self {
            id: b.id.0,
            p: b.position,
            h,
            axes,
            lo: b.position - extent,
            hi: b.position + extent,
        }
    }
}
/// Independent overlap-distance oracle. Full 15-axis OBB SAT; does not reuse engine manifolds.
fn overlap(a: &ObservedBox, b: &ObservedBox) -> f64 {
    let mut axes = [V::ZERO; 15];
    axes[..3].copy_from_slice(&a.axes);
    axes[3..6].copy_from_slice(&b.axes);
    for i in 0..3 {
        for j in 0..3 {
            axes[6 + i * 3 + j] = a.axes[i].cross(b.axes[j]);
        }
    }
    let mut penetration = f64::INFINITY;
    for axis in axes {
        let length = axis.length();
        if length < 1e-10 {
            continue;
        }
        let n = axis / length;
        let radius = |b: &ObservedBox| {
            b.axes[0].dot(n).abs() * b.h.0
                + b.axes[1].dot(n).abs() * b.h.1
                + b.axes[2].dot(n).abs() * b.h.2
        };
        let d = radius(a) + radius(b) - (b.p - a.p).dot(n).abs();
        penetration = penetration.min(d);
        if d < -0.02 {
            return d;
        }
    }
    penetration
}
thread_local! { static FIXTURE: RefCell<Option<Fixture>>=const {RefCell::new(None)}; }
#[unsafe(no_mangle)]
pub extern "C" fn budget_reset(
    count: u32,
    scene: u32,
    substeps: u32,
    velocity: u32,
    position: u32,
) -> i32 {
    let Ok(s) = u8::try_from(substeps) else {
        return -1;
    };
    let Ok(v) = u8::try_from(velocity) else {
        return -1;
    };
    let Ok(p) = u8::try_from(position) else {
        return -1;
    };
    match Fixture::new(count as usize, scene, s, v, p) {
        Ok(f) => {
            FIXTURE.with(|w| *w.borrow_mut() = Some(f));
            0
        }
        Err(_) => -1,
    }
}
#[unsafe(no_mangle)]
pub extern "C" fn budget_step() -> i32 {
    FIXTURE.with(|w| {
        let mut state = w.borrow_mut();
        let Some(f) = state.as_mut() else {
            return -1;
        };
        if f.error {
            return -2;
        }
        if f.step().is_err() {
            f.error = true;
            return -2;
        }
        0
    })
}
#[unsafe(no_mangle)]
pub extern "C" fn budget_observe() -> usize {
    FIXTURE.with(|w| {
        let mut s = w.borrow_mut();
        s.as_mut().map_or(0, |f| {
            f.observe();
            f.snapshot.as_ptr() as usize
        })
    })
}
#[unsafe(no_mangle)]
pub extern "C" fn budget_snapshot_len() -> usize {
    FIXTURE.with(|w| w.borrow().as_ref().map_or(0, |f| f.snapshot.len()))
}
#[unsafe(no_mangle)]
pub extern "C" fn budget_metric(index: u32) -> f64 {
    FIXTURE.with(|w| {
        w.borrow()
            .as_ref()
            .and_then(|f| f.metrics.get(index as usize))
            .copied()
            .unwrap_or(f64::NAN)
    })
}
#[unsafe(no_mangle)]
pub extern "C" fn budget_stat(index: u32) -> f64 {
    FIXTURE.with(|w| w.borrow().as_ref().map_or(f64::NAN, |f| f.stat(index)))
}

/// Records the actual default used by this diagnostic crate; no consumer-owned solver policy.
#[unsafe(no_mangle)]
pub extern "C" fn budget_convergence_scope() -> u32 {
    match Config::default().convergence_scope {
        physics_engine::approximate::ConvergenceScope::WholeWorld => 0,
        physics_engine::approximate::ConvergenceScope::ContactIslands => 1,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use physics_engine::approximate::Quaternion;
    #[test]
    fn independent_sat_covers_separation_touch_overlap_and_rotation() {
        let a = Body::new(BodyId(1), Shape::Box(V(1.0, 1.0, 1.0)), V::ZERO, 1.0);
        for (x, expected) in [(2.0, 0.0), (1.5, 0.5), (3.0, -1.0)] {
            let mut b = a.clone();
            b.id = BodyId(2);
            b.position.0 = x;
            assert!(
                (overlap(&ObservedBox::new(&a), &ObservedBox::new(&b)) - expected).abs() < 1e-12
            );
        }
        let mut b = a.clone();
        b.id = BodyId(2);
        b.position.0 = 2.0;
        b.orientation = Quaternion(
            0.0,
            0.0,
            (std::f64::consts::FRAC_PI_8).sin(),
            (std::f64::consts::FRAC_PI_8).cos(),
        );
        assert!(
            (overlap(&ObservedBox::new(&a), &ObservedBox::new(&b)) - (2.0_f64.sqrt() - 1.0)).abs()
                < 1e-12
        );
    }
    #[test]
    fn invalid_reset_is_transactional_and_ranges_are_checked() {
        assert_eq!(budget_reset(32, 0, 4, 8, 2), 0);
        budget_step();
        let before = budget_stat(14);
        for config in [
            (31, 0, 4, 8, 2),
            (32, 4, 4, 8, 2),
            (32, 0, 0, 8, 2),
            (32, 0, 1, 0, 2),
            (32, 0, 4, 8, 9),
            (32, 0, 256, 8, 2),
        ] {
            assert_eq!(
                budget_reset(config.0, config.1, config.2, config.3, config.4),
                -1
            );
            assert_eq!(budget_stat(14), before);
        }
    }
    #[test]
    fn observations_are_read_only_and_sustained_load_does_not_sleep() {
        let mut f = Fixture::new(32, 0, 4, 8, 2).unwrap();
        for _ in 0..60 {
            f.step().unwrap();
        }
        let bodies = f.world.bodies().cloned().collect::<Vec<_>>();
        let time = f.world.elapsed_seconds();
        f.observe();
        assert_eq!(f.metrics[0], 1.0);
        assert_eq!(f.metrics[3], 32.0);
        assert!(f.metrics[5] >= 24.0);
        assert_eq!(bodies, f.world.bodies().cloned().collect::<Vec<_>>());
        assert_eq!(time, f.world.elapsed_seconds());
    }
    #[test]
    fn quality_oracle_detects_deep_overlap_even_if_floor_projection_succeeds() {
        let mut f = Fixture::new(8, 0, 4, 8, 2).unwrap();
        let id = BodyId(100);
        let mut b = f.world.remove_body(id).unwrap();
        b.position = V(-18.0, 54.0, 0.0);
        f.world.add_body(b).unwrap();
        f.observe();
        assert!(f.metrics[2] > 1.8);
    }
    #[test]
    fn mixed_shots_use_ccd_and_consume_time_with_low_budget() {
        let mut f = Fixture::new(8, 1, 1, 1, 0).unwrap();
        for _ in 0..200 {
            f.step().unwrap();
        }
        f.observe();
        assert_eq!(f.shots, 3);
        assert!((f.world.elapsed_seconds() - 200.0 * DT).abs() < 1e-10);
    }
    #[test]
    fn repeated_cases_have_identical_full_state_and_actual_work() {
        let mut a = Fixture::new(16, 1, 2, 2, 1).unwrap();
        let mut b = Fixture::new(16, 1, 2, 2, 1).unwrap();
        for _ in 0..220 {
            a.step().unwrap();
            b.step().unwrap();
            a.observe();
            b.observe();
            assert_eq!(a.snapshot, b.snapshot);
            assert_eq!(a.metrics, b.metrics);
            for i in 0..15 {
                assert_eq!(a.stat(i), b.stat(i));
            }
        }
    }
}
