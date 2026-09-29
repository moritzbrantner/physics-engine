//! Small f64 math surface for the fixed-step experiment; no integer state in this path.
use crate::{Vec3i, numeric::Scalar};
use std::ops::{Add, AddAssign, Div, Mul, Neg, Sub, SubAssign};

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Vector(pub Scalar, pub Scalar, pub Scalar);
impl Vector {
    pub const ZERO: Self = Self(0.0, 0.0, 0.0);
    pub const X: Self = Self(1.0, 0.0, 0.0);
    pub const Y: Self = Self(0.0, 1.0, 0.0);
    pub const Z: Self = Self(0.0, 0.0, 1.0);
    pub fn dot(self, b: Self) -> Scalar {
        self.0 * b.0 + self.1 * b.1 + self.2 * b.2
    }
    pub fn cross(self, b: Self) -> Self {
        Self(
            self.1 * b.2 - self.2 * b.1,
            self.2 * b.0 - self.0 * b.2,
            self.0 * b.1 - self.1 * b.0,
        )
    }
    pub fn length(self) -> Scalar {
        self.dot(self).sqrt()
    }
    pub fn unit(self) -> Self {
        let n = self.length();
        if n > 1e-12 { self / n } else { Self::ZERO }
    }
    pub fn at(self, i: usize) -> Scalar {
        [self.0, self.1, self.2][i]
    }
    pub fn finite(self) -> bool {
        self.0.is_finite() && self.1.is_finite() && self.2.is_finite()
    }
    pub fn abs(self) -> Self {
        Self(self.0.abs(), self.1.abs(), self.2.abs())
    }
    pub fn max_component(self) -> Scalar {
        self.0.max(self.1).max(self.2)
    }
    pub fn min_component(self) -> Scalar {
        self.0.min(self.1).min(self.2)
    }
    pub fn min(self, b: Self) -> Self {
        Self(self.0.min(b.0), self.1.min(b.1), self.2.min(b.2))
    }
    pub fn max(self, b: Self) -> Self {
        Self(self.0.max(b.0), self.1.max(b.1), self.2.max(b.2))
    }
    pub fn component_mul(self, b: Self) -> Self {
        Self(self.0 * b.0, self.1 * b.1, self.2 * b.2)
    }
}
impl From<Vec3i> for Vector {
    fn from(v: Vec3i) -> Self {
        Self(v.x as Scalar, v.y as Scalar, v.z as Scalar)
    }
}
impl Add for Vector {
    type Output = Self;
    fn add(self, b: Self) -> Self {
        Self(self.0 + b.0, self.1 + b.1, self.2 + b.2)
    }
}
impl Sub for Vector {
    type Output = Self;
    fn sub(self, b: Self) -> Self {
        Self(self.0 - b.0, self.1 - b.1, self.2 - b.2)
    }
}
impl Neg for Vector {
    type Output = Self;
    fn neg(self) -> Self {
        Self(-self.0, -self.1, -self.2)
    }
}
impl Mul<Scalar> for Vector {
    type Output = Self;
    fn mul(self, k: Scalar) -> Self {
        Self(self.0 * k, self.1 * k, self.2 * k)
    }
}
impl Div<Scalar> for Vector {
    type Output = Self;
    fn div(self, k: Scalar) -> Self {
        Self(self.0 / k, self.1 / k, self.2 / k)
    }
}
impl AddAssign for Vector {
    fn add_assign(&mut self, b: Self) {
        *self = *self + b;
    }
}
impl SubAssign for Vector {
    fn sub_assign(&mut self, b: Self) {
        *self = *self - b;
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Quaternion(pub Scalar, pub Scalar, pub Scalar, pub Scalar);
impl Default for Quaternion {
    fn default() -> Self {
        Self::IDENTITY
    }
}
impl Quaternion {
    pub const IDENTITY: Self = Self(0.0, 0.0, 0.0, 1.0);
    pub fn normalized(self) -> Self {
        let l = (self.0 * self.0 + self.1 * self.1 + self.2 * self.2 + self.3 * self.3).sqrt();
        Self(self.0 / l, self.1 / l, self.2 / l, self.3 / l)
    }
    pub fn finite(self) -> bool {
        [self.0, self.1, self.2, self.3]
            .iter()
            .all(|x| x.is_finite())
    }
    pub fn rotate(self, v: Vector) -> Vector {
        let q = Vector(self.0, self.1, self.2);
        let t = q.cross(v) * 2.0;
        v + t * self.3 + q.cross(t)
    }
    pub fn inverse_rotate(self, v: Vector) -> Vector {
        Self(-self.0, -self.1, -self.2, self.3).rotate(v)
    }
    pub fn integrate(self, w: Vector, dt: Scalar) -> Self {
        // First-order quaternion derivative, normalized once per substep.
        let q = Vector(self.0, self.1, self.2);
        let dq = (w * self.3 + w.cross(q)) * (0.5 * dt);
        Self(
            self.0 + dq.0,
            self.1 + dq.1,
            self.2 + dq.2,
            self.3 - 0.5 * dt * w.dot(q),
        )
        .normalized()
    }
    pub fn axes(self) -> [Vector; 3] {
        [
            self.rotate(Vector::X),
            self.rotate(Vector::Y),
            self.rotate(Vector::Z),
        ]
    }
}
