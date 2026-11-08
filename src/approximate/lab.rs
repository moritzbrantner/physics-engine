//! Prototype solver variants for the #545 comparison. Compiled only under one `solver-*` feature;
//! ordinary builds keep the scalar `convergence::solve_rows`.
//!
//! All variants copy the selected rows and the velocities they touch into compact arrays for one
//! `solve_rows` call and write them back at the end, or before a residual check.
//! - `solver-hot`: the same row order and arithmetic as the scalar solver, so traces must match
//!   it bit for bit. Shows what layout alone is worth.
//! - `solver-manifold`: rows of one pair, up to four, are solved together against one velocity
//!   snapshot (relaxed Jacobi). Runs of one row stay scalar.
//! - `solver-colored`: four rows with no written body in common share a batch, as Rapier does.
//!   Rows that fill no batch, joint rows and non-finite rows run scalar after the batches.
//!
//! The wide kernels square friction in f32 instead of going through f64 as `solve_row` does.
use super::convergence::{Convergence, Exit, Outcome, Rows, projected_residual_rows};
use super::{Body, Constraint, PreparedResponse, Real, Report, Vector};
use std::cell::RefCell;
#[cfg(any(feature = "solver-manifold", feature = "solver-colored"))]
use wide::f32x4;

const NO_SLOT: u32 = u32::MAX;
#[cfg(any(feature = "solver-manifold", feature = "solver-colored"))]
const WIDE_STEP: u32 = 1 << 31;

#[derive(Clone, Copy)]
struct Velocity {
    linear: Vector,
    angular: Vector,
}

#[derive(Clone, Copy)]
struct Hot {
    a: u32,
    b: u32,
    /// Position in the selected rows.
    origin: u32,
    inverse_mass: [Real; 2],
    n: Vector,
    ra: Vector,
    rb: Vector,
    t1: Vector,
    t2: Vector,
    angular: [[Vector; 3]; 2],
    normal_mass: Real,
    tangent_mass: [Real; 2],
    bias: Real,
    friction: Real,
    normal_impulse: Real,
    tangent_impulse: [Real; 2],
    finite_response: bool,
    spin: bool,
    response: [bool; 2],
    /// Per side: positive mass, so the side can have a contact velocity at all.
    moves: [bool; 2],
}

#[derive(Default)]
struct Lab {
    contact_rows: usize,
    velocity: Vec<Velocity>,
    body: Vec<u32>,
    slot: Vec<u32>,
    hot: Vec<Hot>,
    #[cfg(any(feature = "solver-manifold", feature = "solver-colored"))]
    wide: Vec<Wide>,
    /// Rows waiting for a batch, during `schedule`.
    #[cfg(any(feature = "solver-manifold", feature = "solver-colored"))]
    pool: Vec<Hot>,
    /// An index into `hot`, or into `wide` with `WIDE_STEP` set, in solve order.
    #[cfg(any(feature = "solver-manifold", feature = "solver-colored"))]
    steps: Vec<u32>,
    #[cfg(feature = "solver-colored")]
    written: Vec<u64>,
    #[cfg(feature = "solver-colored")]
    read: Vec<u64>,
    #[cfg(feature = "solver-colored")]
    tail: Vec<u32>,
}

std::thread_local! {
    static LAB: RefCell<Lab> = RefCell::new(Lab {
        contact_rows: usize::MAX,
        ..Lab::default()
    });
}

/// Rows from this index are joint rows, which the wide variants leave scalar and last.
pub(super) fn set_contact_rows(rows: usize) {
    LAB.with_borrow_mut(|lab| lab.contact_rows = rows);
}

#[inline(always)]
fn velocity_at(v: Velocity, r: Vector, spin: bool, moves: bool) -> Vector {
    // `contact_velocity`'s early return, which spares floor rows the cross product.
    if !moves {
        return Vector::ZERO;
    }
    v.linear
        + if spin {
            v.angular.cross(r)
        } else {
            Vector::ZERO
        }
}

#[inline(always)]
fn apply<const NORMAL: bool, const TANGENTS: bool>(
    velocity: &mut [Velocity],
    c: &Hot,
    impulse: [Real; 3],
) {
    let j = if !NORMAL {
        c.t1 * impulse[1] + c.t2 * impulse[2]
    } else if TANGENTS {
        c.n * impulse[0] + c.t1 * impulse[1] + c.t2 * impulse[2]
    } else {
        c.n * impulse[0]
    };
    for (side, i, sign) in [(0, c.a, -1.0), (1, c.b, 1.0)] {
        if c.response[side] {
            let v = &mut velocity[i as usize];
            v.linear += j * (sign * c.inverse_mass[side]);
            if c.spin {
                let [n, t1, t2] = c.angular[side];
                let angular = if !NORMAL {
                    t1 * impulse[1] + t2 * impulse[2]
                } else if TANGENTS {
                    n * impulse[0] + t1 * impulse[1] + t2 * impulse[2]
                } else {
                    n * impulse[0]
                };
                v.angular += angular * sign;
            }
        }
    }
}

#[derive(Default, Clone, Copy)]
struct Change {
    impulse: [Real; 3],
    velocity_scale: [Real; 3],
    impulse_scale: Real,
    normal_mass: Real,
    tangent_mass: [Real; 2],
}

// `convergence::solve_row`, field for field, over the compact row and velocities.
#[inline(always)]
fn solve_hot<const READ_CHANGE: bool>(velocity: &mut [Velocity], c: &mut Hot) -> Change {
    let relative = |velocity: &[Velocity], c: &Hot| {
        velocity_at(velocity[c.b as usize], c.rb, c.spin, c.moves[1])
            - velocity_at(velocity[c.a as usize], c.ra, c.spin, c.moves[0])
    };
    let before = relative(velocity, c);
    let vn = before.dot(c.n);
    let next = (c.normal_impulse + (c.bias - vn) * c.normal_mass).max(0.0);
    let dj = next - c.normal_impulse;
    c.normal_impulse = next;
    let rel = if dj == 0.0 && c.finite_response {
        before
    } else {
        apply::<true, false>(velocity, c, [dj, 0.0, 0.0]);
        relative(velocity, c)
    };
    let vt = [rel.dot(c.t1), rel.dot(c.t2)];
    let mut tangent = [
        c.tangent_impulse[0] - vt[0] * c.tangent_mass[0],
        c.tangent_impulse[1] - vt[1] * c.tangent_mass[1],
    ];
    let len = (f64::from(tangent[0]) * f64::from(tangent[0])
        + f64::from(tangent[1]) * f64::from(tangent[1]))
    .sqrt() as Real;
    let limit = c.friction * c.normal_impulse;
    if len > limit && len > 0.0 {
        tangent[0] *= limit / len;
        tangent[1] *= limit / len;
    }
    let dt = [
        tangent[0] - c.tangent_impulse[0],
        tangent[1] - c.tangent_impulse[1],
    ];
    c.tangent_impulse = tangent;
    if c.finite_response {
        if dt != [0.0; 2] {
            apply::<false, true>(velocity, c, [0.0, dt[0], dt[1]]);
        }
    } else {
        apply::<true, true>(velocity, c, [0.0, dt[0], dt[1]]);
    }
    if READ_CHANGE {
        Change {
            impulse: [dj, dt[0], dt[1]],
            velocity_scale: [vn.abs().max(c.bias.abs()), vt[0].abs(), vt[1].abs()],
            impulse_scale: c
                .normal_impulse
                .abs()
                .max(c.tangent_impulse[0].abs())
                .max(c.tangent_impulse[1].abs()),
            normal_mass: c.normal_mass,
            tangent_mass: c.tangent_mass,
        }
    } else {
        Change::default()
    }
}

// The one place the wide type is named: everything above and below works through `V3`.
#[cfg(any(feature = "solver-manifold", feature = "solver-colored"))]
mod v3 {
    use super::{Vector, f32x4};

    #[derive(Clone, Copy)]
    pub(super) struct V3 {
        pub x: f32x4,
        pub y: f32x4,
        pub z: f32x4,
    }
    impl V3 {
        pub const ZERO: Self = Self {
            x: f32x4::ZERO,
            y: f32x4::ZERO,
            z: f32x4::ZERO,
        };
        pub fn from_lanes(v: [Vector; 4]) -> Self {
            Self {
                x: f32x4::new([v[0].0, v[1].0, v[2].0, v[3].0]),
                y: f32x4::new([v[0].1, v[1].1, v[2].1, v[3].1]),
                z: f32x4::new([v[0].2, v[1].2, v[2].2, v[3].2]),
            }
        }
        pub fn splat(v: Vector) -> Self {
            Self {
                x: f32x4::splat(v.0),
                y: f32x4::splat(v.1),
                z: f32x4::splat(v.2),
            }
        }
        pub fn lane_sum(self) -> Vector {
            Vector(
                self.x.reduce_add(),
                self.y.reduce_add(),
                self.z.reduce_add(),
            )
        }
        pub fn lanes(self) -> [Vector; 4] {
            let (x, y, z) = (self.x.to_array(), self.y.to_array(), self.z.to_array());
            [0, 1, 2, 3].map(|i| Vector(x[i], y[i], z[i]))
        }
        #[inline(always)]
        pub fn dot(self, b: Self) -> f32x4 {
            self.x * b.x + self.y * b.y + self.z * b.z
        }
        #[inline(always)]
        pub fn cross(self, b: Self) -> Self {
            Self {
                x: self.y * b.z - self.z * b.y,
                y: self.z * b.x - self.x * b.z,
                z: self.x * b.y - self.y * b.x,
            }
        }
        #[inline(always)]
        pub fn scale(self, s: f32x4) -> Self {
            Self {
                x: self.x * s,
                y: self.y * s,
                z: self.z * s,
            }
        }
    }
    impl core::ops::Add for V3 {
        type Output = Self;
        #[inline(always)]
        fn add(self, b: Self) -> Self {
            Self {
                x: self.x + b.x,
                y: self.y + b.y,
                z: self.z + b.z,
            }
        }
    }
    impl core::ops::Sub for V3 {
        type Output = Self;
        #[inline(always)]
        fn sub(self, b: Self) -> Self {
            Self {
                x: self.x - b.x,
                y: self.y - b.y,
                z: self.z - b.z,
            }
        }
    }
}
#[cfg(any(feature = "solver-manifold", feature = "solver-colored"))]
use v3::V3;

/// Four rows' coefficients. An unused lane is all zero, so it solves to a zero impulse.
#[cfg(any(feature = "solver-manifold", feature = "solver-colored"))]
#[derive(Clone, Copy)]
struct Wide {
    a: [u32; 4],
    b: [u32; 4],
    origin: [u32; 4],
    lanes: u8,
    n: V3,
    ra: V3,
    rb: V3,
    t1: V3,
    t2: V3,
    angular: [[V3; 3]; 2],
    inverse_mass: [f32x4; 2],
    spin: f32x4,
    normal_mass: f32x4,
    tangent_mass: [f32x4; 2],
    bias: f32x4,
    friction: f32x4,
    normal_impulse: f32x4,
    tangent_impulse: [f32x4; 2],
    #[cfg(feature = "solver-manifold")]
    omega: f32x4,
}

#[cfg(any(feature = "solver-manifold", feature = "solver-colored"))]
#[inline(always)]
fn lane(rows: &[&Hot], f: impl Fn(&Hot) -> Real) -> f32x4 {
    f32x4::new([0, 1, 2, 3].map(|i| rows.get(i).map_or(0.0, |c| f(c))))
}

#[cfg(any(feature = "solver-manifold", feature = "solver-colored"))]
#[inline(always)]
fn vector(rows: &[&Hot], f: impl Fn(&Hot) -> Vector) -> V3 {
    V3::from_lanes([0, 1, 2, 3].map(|i| rows.get(i).map_or(Vector::ZERO, |c| f(c))))
}

#[cfg(any(feature = "solver-manifold", feature = "solver-colored"))]
#[inline(always)]
fn ids(rows: &[&Hot], f: impl Fn(&Hot) -> u32) -> [u32; 4] {
    [0, 1, 2, 3].map(|i| rows.get(i).map_or(0, |c| f(c)))
}

#[cfg(any(feature = "solver-manifold", feature = "solver-colored"))]
#[inline]
fn pack(rows: &[&Hot]) -> Wide {
    let angular = |side: usize, axis: usize| {
        vector(rows, |c| {
            if c.response[side] && c.spin {
                c.angular[side][axis]
            } else {
                Vector::ZERO
            }
        })
    };
    let inverse_mass = |side: usize| {
        lane(rows, |c| {
            if c.response[side] {
                c.inverse_mass[side]
            } else {
                0.0
            }
        })
    };
    Wide {
        a: ids(rows, |c| c.a),
        b: ids(rows, |c| c.b),
        origin: ids(rows, |c| c.origin),
        lanes: rows.len() as u8,
        n: vector(rows, |c| c.n),
        ra: vector(rows, |c| c.ra),
        rb: vector(rows, |c| c.rb),
        t1: vector(rows, |c| c.t1),
        t2: vector(rows, |c| c.t2),
        angular: [0, 1].map(|side| [0, 1, 2].map(|axis| angular(side, axis))),
        inverse_mass: [inverse_mass(0), inverse_mass(1)],
        spin: lane(rows, |c| {
            if c.spin {
                f32::from_bits(u32::MAX)
            } else {
                0.0
            }
        }),
        normal_mass: lane(rows, |c| c.normal_mass),
        tangent_mass: [
            lane(rows, |c| c.tangent_mass[0]),
            lane(rows, |c| c.tangent_mass[1]),
        ],
        bias: lane(rows, |c| c.bias),
        friction: lane(rows, |c| c.friction),
        normal_impulse: lane(rows, |c| c.normal_impulse),
        tangent_impulse: [
            lane(rows, |c| c.tangent_impulse[0]),
            lane(rows, |c| c.tangent_impulse[1]),
        ],
        #[cfg(feature = "solver-manifold")]
        omega: f32x4::splat(relaxation(rows)),
    }
}

/// The Jacobi step size: Gershgorin's bound on the largest eigenvalue of the manifold's
/// normalized Delassus matrix, mapped so that eigenvalue lands at 1.5 (stable below 2).
#[cfg(feature = "solver-manifold")]
fn relaxation(rows: &[&Hot]) -> Real {
    let mut bound: Real = 1.0;
    for (i, ci) in rows.iter().enumerate() {
        let mut sum: Real = 0.0;
        for (j, cj) in rows.iter().enumerate() {
            let mut d = 0.0;
            if ci.response[0] {
                d += ci.inverse_mass[0] * ci.n.dot(cj.n)
                    + if ci.spin {
                        cj.ra.cross(cj.n).dot(ci.angular[0][0])
                    } else {
                        0.0
                    };
            }
            if ci.response[1] {
                d += ci.inverse_mass[1] * ci.n.dot(cj.n)
                    + if ci.spin {
                        cj.rb.cross(cj.n).dot(ci.angular[1][0])
                    } else {
                        0.0
                    };
            }
            if i != j {
                sum += d.abs() * ci.normal_mass;
            }
        }
        bound = bound.max(1.0 + sum);
    }
    (1.5 / bound).min(1.0)
}

#[cfg(any(feature = "solver-manifold", feature = "solver-colored"))]
#[inline(always)]
fn friction_clamp(x: f32x4, y: f32x4, limit: f32x4) -> [f32x4; 2] {
    // The replacement for `solve_row`'s f64 squares: f32 squares and one wide sqrt. Impulses
    // above about 1.8e19 overflow to an infinite length and clamp to zero.
    let len = (x * x + y * y).sqrt();
    let over = len.simd_gt(limit) & len.simd_gt(f32x4::ZERO);
    let scale = over.bitselect(limit / len, f32x4::ONE);
    [x * scale, y * scale]
}

#[cfg(any(feature = "solver-manifold", feature = "solver-colored"))]
fn changes(w: &Wide, dj: f32x4, dt: [f32x4; 2], vn: f32x4, vt: [f32x4; 2]) -> [Change; 4] {
    let (dj, dt0, dt1) = (dj.to_array(), dt[0].to_array(), dt[1].to_array());
    let (vn, vt0, vt1) = (vn.to_array(), vt[0].to_array(), vt[1].to_array());
    let (bias, nm) = (w.bias.to_array(), w.normal_mass.to_array());
    let (tm0, tm1) = (w.tangent_mass[0].to_array(), w.tangent_mass[1].to_array());
    let (ni, ti0, ti1) = (
        w.normal_impulse.to_array(),
        w.tangent_impulse[0].to_array(),
        w.tangent_impulse[1].to_array(),
    );
    [0, 1, 2, 3].map(|i| Change {
        impulse: [dj[i], dt0[i], dt1[i]],
        velocity_scale: [vn[i].abs().max(bias[i].abs()), vt0[i].abs(), vt1[i].abs()],
        impulse_scale: ni[i].abs().max(ti0[i].abs()).max(ti1[i].abs()),
        normal_mass: nm[i],
        tangent_mass: [tm0[i], tm1[i]],
    })
}

#[cfg(feature = "solver-colored")]
#[inline(always)]
fn solve_batch<const READ_CHANGE: bool>(velocity: &mut [Velocity], w: &mut Wide) -> [Change; 4] {
    let gather = |ids: [u32; 4]| {
        let v = ids.map(|i| velocity[i as usize]);
        (
            V3::from_lanes(v.map(|v| v.linear)),
            V3::from_lanes(v.map(|v| v.angular)),
        )
    };
    let ((mut la, mut wa), (mut lb, mut wb)) = (gather(w.a), gather(w.b));
    let spin = |v: V3, s: f32x4| V3 {
        x: v.x & s,
        y: v.y & s,
        z: v.z & s,
    };
    let relative = |la: V3, wa: V3, lb: V3, wb: V3, w: &Wide| {
        (lb + spin(wb.cross(w.rb), w.spin)) - (la + spin(wa.cross(w.ra), w.spin))
    };
    let vn = relative(la, wa, lb, wb, w).dot(w.n);
    let next = (w.normal_impulse + (w.bias - vn) * w.normal_mass).max(f32x4::ZERO);
    let dj = next - w.normal_impulse;
    w.normal_impulse = next;
    let j = w.n.scale(dj);
    la = la - j.scale(w.inverse_mass[0]);
    lb = lb + j.scale(w.inverse_mass[1]);
    wa = wa - w.angular[0][0].scale(dj);
    wb = wb + w.angular[1][0].scale(dj);
    let rel = relative(la, wa, lb, wb, w);
    let vt = [rel.dot(w.t1), rel.dot(w.t2)];
    let [x, y] = friction_clamp(
        w.tangent_impulse[0] - vt[0] * w.tangent_mass[0],
        w.tangent_impulse[1] - vt[1] * w.tangent_mass[1],
        w.friction * w.normal_impulse,
    );
    let dt = [x - w.tangent_impulse[0], y - w.tangent_impulse[1]];
    w.tangent_impulse = [x, y];
    let j = w.t1.scale(dt[0]) + w.t2.scale(dt[1]);
    la = la - j.scale(w.inverse_mass[0]);
    lb = lb + j.scale(w.inverse_mass[1]);
    wa = wa - (w.angular[0][1].scale(dt[0]) + w.angular[0][2].scale(dt[1]));
    wb = wb + (w.angular[1][1].scale(dt[0]) + w.angular[1][2].scale(dt[1]));
    let (la, wa, lb, wb) = (la.lanes(), wa.lanes(), lb.lanes(), wb.lanes());
    for lane in 0..4 {
        velocity[w.a[lane] as usize] = Velocity {
            linear: la[lane],
            angular: wa[lane],
        };
        velocity[w.b[lane] as usize] = Velocity {
            linear: lb[lane],
            angular: wb[lane],
        };
    }
    if READ_CHANGE {
        changes(w, dj, dt, vn, vt)
    } else {
        [Change::default(); 4]
    }
}

#[cfg(feature = "solver-manifold")]
#[inline(always)]
fn solve_batch<const READ_CHANGE: bool>(velocity: &mut [Velocity], w: &mut Wide) -> [Change; 4] {
    let [ia, ib] = w.inverse_mass.map(|m| m.to_array()[0]);
    let (a, b) = (w.a[0] as usize, w.b[0] as usize);
    let (mut va, mut vb) = (velocity[a], velocity[b]);
    let relative = |va: Velocity, vb: Velocity, w: &Wide| {
        let wa = V3::splat(va.angular).cross(w.ra);
        let wb = V3::splat(vb.angular).cross(w.rb);
        let spin = |v: V3| V3 {
            x: v.x & w.spin,
            y: v.y & w.spin,
            z: v.z & w.spin,
        };
        (V3::splat(vb.linear) + spin(wb)) - (V3::splat(va.linear) + spin(wa))
    };
    let vn = relative(va, vb, w).dot(w.n);
    let next = (w.normal_impulse + (w.bias - vn) * w.normal_mass).max(f32x4::ZERO);
    let dj = (next - w.normal_impulse) * w.omega;
    w.normal_impulse = w.normal_impulse + dj;
    let sum = w.n.scale(dj).lane_sum();
    va.linear -= sum * ia;
    vb.linear += sum * ib;
    va.angular -= w.angular[0][0].scale(dj).lane_sum();
    vb.angular += w.angular[1][0].scale(dj).lane_sum();
    let rel = relative(va, vb, w);
    let vt = [rel.dot(w.t1), rel.dot(w.t2)];
    let [x, y] = friction_clamp(
        w.tangent_impulse[0] - vt[0] * w.tangent_mass[0],
        w.tangent_impulse[1] - vt[1] * w.tangent_mass[1],
        w.friction * w.normal_impulse,
    );
    let dt = [
        (x - w.tangent_impulse[0]) * w.omega,
        (y - w.tangent_impulse[1]) * w.omega,
    ];
    w.tangent_impulse = [w.tangent_impulse[0] + dt[0], w.tangent_impulse[1] + dt[1]];
    let sum = (w.t1.scale(dt[0]) + w.t2.scale(dt[1])).lane_sum();
    va.linear -= sum * ia;
    vb.linear += sum * ib;
    va.angular -= (w.angular[0][1].scale(dt[0]) + w.angular[0][2].scale(dt[1])).lane_sum();
    vb.angular += (w.angular[1][1].scale(dt[0]) + w.angular[1][2].scale(dt[1])).lane_sum();
    velocity[a] = va;
    velocity[b] = vb;
    if READ_CHANGE {
        changes(w, dj, dt, vn, vt)
    } else {
        [Change::default(); 4]
    }
}

impl Lab {
    fn slot_of(&mut self, bodies: &[Body], index: usize) -> u32 {
        if self.slot[index] == NO_SLOT {
            let b = &bodies[index];
            // `contact_velocity`: mass-zero geometry never moves.
            let velocity = if b.mass == 0.0 {
                Velocity {
                    linear: Vector::ZERO,
                    angular: Vector::ZERO,
                }
            } else {
                Velocity {
                    linear: b.velocity,
                    angular: b.angular_velocity,
                }
            };
            self.slot[index] = self.body.len() as u32;
            self.body.push(index as u32);
            self.velocity.push(velocity);
            #[cfg(feature = "solver-colored")]
            {
                self.written.push(0);
                self.read.push(0);
            }
        }
        self.slot[index]
    }

    fn hot_row(
        &mut self,
        bodies: &[Body],
        responses: &[PreparedResponse],
        c: &Constraint,
        origin: u32,
    ) -> Hot {
        Hot {
            a: self.slot_of(bodies, c.a),
            b: self.slot_of(bodies, c.b),
            origin,
            inverse_mass: [responses[c.a].inverse_mass, responses[c.b].inverse_mass],
            n: c.n,
            ra: c.ra,
            rb: c.rb,
            t1: c.t1,
            t2: c.t2,
            angular: c.angular,
            normal_mass: c.normal_mass,
            tangent_mass: c.tangent_mass,
            bias: c.bias,
            friction: c.friction,
            normal_impulse: c.normal_impulse,
            tangent_impulse: c.tangent_impulse,
            finite_response: c.finite_response,
            spin: c.spin,
            response: c.response,
            moves: [bodies[c.a].mass != 0.0, bodies[c.b].mass != 0.0],
        }
    }

    #[cfg(feature = "solver-hot")]
    fn schedule<R: Rows>(
        &mut self,
        bodies: &[Body],
        responses: &[PreparedResponse],
        constraints: &[Constraint],
        rows: R,
    ) {
        for n in 0..rows.len() {
            let row = self.hot_row(bodies, responses, &constraints[rows.index(n)], n as u32);
            self.hot.push(row);
        }
    }

    #[cfg(feature = "solver-manifold")]
    fn schedule<R: Rows>(
        &mut self,
        bodies: &[Body],
        responses: &[PreparedResponse],
        constraints: &[Constraint],
        rows: R,
    ) {
        let mut n = 0;
        while n < rows.len() {
            let first = rows.index(n);
            let c = &constraints[first];
            let mut end = n;
            self.pool.clear();
            while end < rows.len() && self.pool.len() < 4 {
                let index = rows.index(end);
                let d = &constraints[index];
                if d.a != c.a
                    || d.b != c.b
                    || !d.finite_response
                    || index >= self.contact_rows
                    || !c.finite_response
                {
                    break;
                }
                let row = self.hot_row(bodies, responses, d, end as u32);
                self.pool.push(row);
                end += 1;
            }
            let count = self.pool.len();
            if count >= 2 {
                let mut refs = [&self.pool[0]; 4];
                for (slot, row) in refs.iter_mut().zip(&self.pool) {
                    *slot = row;
                }
                let wide = pack(&refs[..count]);
                self.steps.push(WIDE_STEP | self.wide.len() as u32);
                self.wide.push(wide);
                n = end;
            } else {
                let row = self.hot_row(bodies, responses, c, n as u32);
                self.steps.push(self.hot.len() as u32);
                self.hot.push(row);
                n += 1;
            }
        }
    }

    // Greedy in row order over 64 buckets. A full bucket becomes a batch and frees its bodies,
    // so the result depends only on the stable row order.
    #[cfg(feature = "solver-colored")]
    fn schedule<R: Rows>(
        &mut self,
        bodies: &[Body],
        responses: &[PreparedResponse],
        constraints: &[Constraint],
        rows: R,
    ) {
        let mut buckets = [[0u32; 4]; 64];
        let mut counts = [0u8; 64];
        self.tail.clear();
        self.pool.clear();
        for n in 0..rows.len() {
            let index = rows.index(n);
            let row = self.hot_row(bodies, responses, &constraints[index], n as u32);
            let (a, b) = (row.a as usize, row.b as usize);
            let mut blocked = self.written[a] | self.written[b];
            if row.response[0] {
                blocked |= self.read[a];
            }
            if row.response[1] {
                blocked |= self.read[b];
            }
            if index >= self.contact_rows || !row.finite_response || blocked == u64::MAX {
                self.tail.push(self.hot.len() as u32);
                self.hot.push(row);
                continue;
            }
            let color = (!blocked).trailing_zeros() as usize;
            let bit = 1u64 << color;
            for (side, slot) in [(0, a), (1, b)] {
                if row.response[side] {
                    self.written[slot] |= bit;
                } else {
                    self.read[slot] |= bit;
                }
            }
            buckets[color][usize::from(counts[color])] = self.pool.len() as u32;
            self.pool.push(row);
            counts[color] += 1;
            if counts[color] == 4 {
                counts[color] = 0;
                let batch = buckets[color].map(|i| &self.pool[i as usize]);
                let wide = pack(&batch);
                let touched = batch.map(|row| [row.a as usize, row.b as usize]);
                self.steps.push(WIDE_STEP | self.wide.len() as u32);
                self.wide.push(wide);
                for slot in touched.into_iter().flatten() {
                    self.written[slot] &= !bit;
                    self.read[slot] &= !bit;
                }
            }
        }
        for color in 0..64 {
            for &at in &buckets[color][..usize::from(counts[color])] {
                self.tail.push(self.hot.len() as u32);
                self.hot.push(self.pool[at as usize]);
            }
        }
        // Stable row order for the scalar rows that follow the batches.
        let hot = &self.hot;
        self.tail.sort_unstable_by_key(|&h| hot[h as usize].origin);
        self.steps.extend_from_slice(&self.tail);
    }

    fn load<R: Rows>(
        &mut self,
        bodies: &[Body],
        responses: &[PreparedResponse],
        constraints: &[Constraint],
        rows: R,
    ) {
        if self.slot.len() < bodies.len() {
            self.slot.resize(bodies.len(), NO_SLOT);
        }
        self.schedule(bodies, responses, constraints, rows);
    }

    fn sync<R: Rows>(&self, bodies: &mut [Body], constraints: &mut [Constraint], rows: R) {
        for (&index, v) in self.body.iter().zip(&self.velocity) {
            let b = &mut bodies[index as usize];
            if b.mass != 0.0 {
                b.velocity = v.linear;
                b.angular_velocity = v.angular;
            }
        }
        for c in &self.hot {
            let row = &mut constraints[rows.index(c.origin as usize)];
            row.normal_impulse = c.normal_impulse;
            row.tangent_impulse = c.tangent_impulse;
        }
        #[cfg(any(feature = "solver-manifold", feature = "solver-colored"))]
        for w in &self.wide {
            let ni = w.normal_impulse.to_array();
            let (t0, t1) = (
                w.tangent_impulse[0].to_array(),
                w.tangent_impulse[1].to_array(),
            );
            for lane in 0..usize::from(w.lanes) {
                let row = &mut constraints[rows.index(w.origin[lane] as usize)];
                row.normal_impulse = ni[lane];
                row.tangent_impulse = [t0[lane], t1[lane]];
            }
        }
    }

    fn clear(&mut self) {
        for &index in &self.body {
            self.slot[index as usize] = NO_SLOT;
        }
        self.body.clear();
        self.velocity.clear();
        self.hot.clear();
        #[cfg(any(feature = "solver-manifold", feature = "solver-colored"))]
        {
            self.wide.clear();
            self.steps.clear();
        }
        #[cfg(feature = "solver-colored")]
        {
            self.written.clear();
            self.read.clear();
        }
    }

    #[cfg(feature = "solver-hot")]
    fn pass<const PROBE: bool>(
        &mut self,
        tolerance: Convergence,
        probe: &mut Probe,
        report: &mut Report,
    ) {
        for c in &mut self.hot {
            if PROBE && probe.open {
                report.convergence.delta_constraint_checks += 1;
                probe.check(solve_hot::<true>(&mut self.velocity, c), tolerance);
            } else {
                solve_hot::<false>(&mut self.velocity, c);
            }
        }
    }

    #[cfg(any(feature = "solver-manifold", feature = "solver-colored"))]
    fn pass<const PROBE: bool>(
        &mut self,
        tolerance: Convergence,
        probe: &mut Probe,
        report: &mut Report,
    ) {
        for &step in &self.steps {
            if step & WIDE_STEP != 0 {
                let w = &mut self.wide[(step & !WIDE_STEP) as usize];
                if PROBE && probe.open {
                    report.convergence.delta_constraint_checks += u64::from(w.lanes);
                    let change = solve_batch::<true>(&mut self.velocity, w);
                    for c in &change[..usize::from(w.lanes)] {
                        probe.check(*c, tolerance);
                    }
                } else {
                    solve_batch::<false>(&mut self.velocity, w);
                }
            } else {
                let c = &mut self.hot[step as usize];
                if PROBE && probe.open {
                    report.convergence.delta_constraint_checks += 1;
                    probe.check(solve_hot::<true>(&mut self.velocity, c), tolerance);
                } else {
                    solve_hot::<false>(&mut self.velocity, c);
                }
            }
        }
    }
}

struct Probe {
    open: bool,
    max_delta: Real,
}
impl Probe {
    #[inline(always)]
    fn check(&mut self, c: Change, tolerance: Convergence) {
        self.open = self.open
            && tolerance.small(
                c.impulse[0],
                c.normal_mass,
                c.impulse_scale,
                c.velocity_scale[0],
            )
            && tolerance.small(
                c.impulse[1],
                c.tangent_mass[0],
                c.impulse_scale,
                c.velocity_scale[1],
            )
            && tolerance.small(
                c.impulse[2],
                c.tangent_mass[1],
                c.impulse_scale,
                c.velocity_scale[2],
            );
        if self.open {
            self.max_delta = self
                .max_delta
                .max(c.impulse[0].abs())
                .max(c.impulse[1].abs())
                .max(c.impulse[2].abs());
        }
    }
}

/// The variant counterpart of the scalar `solve_rows`, with the same passes, probes and exits.
pub(super) fn solve_rows<const EARLY: bool, const START: u8, const PREFIX: bool, R: Rows>(
    bodies: &mut [Body],
    responses: &[PreparedResponse],
    constraints: &mut [Constraint],
    rows: R,
    iterations: u8,
    tolerance: Convergence,
    report: &mut Report,
) -> Outcome {
    if EARLY && rows.len() == 0 {
        return Outcome {
            passes: 0,
            exit: Exit::Empty,
        };
    }
    LAB.with_borrow_mut(|lab| {
        super::phase_timing::timed(super::phase_timing::Phase::Prepare, || {
            lab.load(bodies, responses, constraints, rows);
        });
        let mut outcome = Outcome {
            passes: iterations,
            exit: if EARLY { Exit::Capped } else { Exit::Fixed },
        };
        for pass in START..iterations {
            report.convergence.constraint_visits += rows.len() as u64;
            let completed = u32::from(pass) + 1;
            let candidate = EARLY
                && completed >= 2
                && completed.is_power_of_two()
                && completed < u32::from(iterations);
            let mut probe = Probe {
                open: candidate,
                max_delta: 0.0,
            };
            if candidate {
                report.convergence.probe_passes += 1;
                lab.pass::<true>(tolerance, &mut probe, report);
            } else {
                lab.pass::<false>(tolerance, &mut probe, report);
            }
            if probe.open {
                lab.sync(bodies, constraints, rows);
                if let Some(residual) = projected_residual_rows(
                    bodies,
                    (0..rows.len()).map(|n| &constraints[rows.index(n)]),
                    tolerance,
                    &mut report.convergence,
                ) {
                    report.convergence.max_exit_impulse_delta = report
                        .convergence
                        .max_exit_impulse_delta
                        .max(probe.max_delta);
                    report.convergence.max_exit_velocity_residual =
                        report.convergence.max_exit_velocity_residual.max(residual);
                    outcome = Outcome {
                        passes: pass + 1,
                        exit: Exit::Converged,
                    };
                    break;
                }
            }
            if PREFIX && pass == 1 {
                outcome = Outcome {
                    passes: 2,
                    exit: Exit::Pending,
                };
                break;
            }
        }
        super::phase_timing::timed(super::phase_timing::Phase::Prepare, || {
            lab.sync(bodies, constraints, rows);
            lab.clear();
        });
        outcome
    })
}
