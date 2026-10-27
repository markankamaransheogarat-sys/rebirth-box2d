//! 2D Vector and 2x2 Matrix mathematics for physics simulations.

use core::ops::{Add, AddAssign, Mul, MulAssign, Neg, Sub, SubAssign};

/// Floating point mathematical helpers supporting both `std` and `no_std` (via `libm`).
#[inline]
pub(crate) fn sqrt(x: f32) -> f32 {
    #[cfg(feature = "std")]
    {
        x.sqrt()
    }
    #[cfg(all(not(feature = "std"), feature = "libm"))]
    {
        libm::sqrtf(x)
    }
    #[cfg(all(not(feature = "std"), not(feature = "libm")))]
    {
        // Fallback or compile error if neither is selected
        core::primitive::f32::from_bits((x.to_bits() as f64).sqrt() as u32)
    }
}

#[inline]
pub(crate) fn sin_cos(angle: f32) -> (f32, f32) {
    #[cfg(feature = "std")]
    {
        (angle.sin(), angle.cos())
    }
    #[cfg(all(not(feature = "std"), feature = "libm"))]
    {
        (libm::sinf(angle), libm::cosf(angle))
    }
}

#[inline]
pub(crate) fn abs(x: f32) -> f32 {
    #[cfg(feature = "std")]
    {
        x.abs()
    }
    #[cfg(not(feature = "std"))]
    {
        if x < 0.0 {
            -x
        } else {
            x
        }
    }
}

/// A 2D column vector.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct Vec2 {
    pub x: f32,
    pub y: f32,
}

impl Vec2 {
    pub const ZERO: Self = Self { x: 0.0, y: 0.0 };

    #[inline]
    pub const fn new(x: f32, y: f32) -> Self {
        Self { x, y }
    }

    #[inline]
    pub fn length_squared(self) -> f32 {
        self.x * self.x + self.y * self.y
    }

    #[inline]
    pub fn length(self) -> f32 {
        sqrt(self.length_squared())
    }

    #[inline]
    pub fn dot(self, other: Self) -> f32 {
        self.x * other.x + self.y * other.y
    }

    /// 2D cross product yielding scalar: `a.x * b.y - a.y * b.x`.
    #[inline]
    pub fn cross(self, other: Self) -> f32 {
        self.x * other.y - self.y * other.x
    }

    /// Cross product of a vector and a scalar: `v x s = Vec2(s * v.y, -s * v.x)`.
    #[inline]
    pub fn cross_scalar(self, s: f32) -> Self {
        Self::new(s * self.y, -s * self.x)
    }

    /// Component-wise absolute value.
    #[inline]
    pub fn abs(self) -> Self {
        Self::new(abs(self.x), abs(self.y))
    }
}

/// Cross product of a vector and a scalar: `v x s = Vec2(s * v.y, -s * v.x)`.
#[inline]
pub fn cross_vec_scalar(v: Vec2, s: f32) -> Vec2 {
    Vec2::new(s * v.y, -s * v.x)
}

/// Cross product of a scalar and a vector: `s x v = Vec2(-s * v.y, s * v.x)`.
/// Represents tangential velocity due to angular rotation: `w x r`.
#[inline]
pub fn cross_scalar_vec(s: f32, v: Vec2) -> Vec2 {
    Vec2::new(-s * v.y, s * v.x)
}

impl Add for Vec2 {
    type Output = Self;
    #[inline]
    fn add(self, rhs: Self) -> Self {
        Self::new(self.x + rhs.x, self.y + rhs.y)
    }
}

impl AddAssign for Vec2 {
    #[inline]
    fn add_assign(&mut self, rhs: Self) {
        self.x += rhs.x;
        self.y += rhs.y;
    }
}

impl Sub for Vec2 {
    type Output = Self;
    #[inline]
    fn sub(self, rhs: Self) -> Self {
        Self::new(self.x - rhs.x, self.y - rhs.y)
    }
}

impl SubAssign for Vec2 {
    #[inline]
    fn sub_assign(&mut self, rhs: Self) {
        self.x -= rhs.x;
        self.y -= rhs.y;
    }
}

impl Neg for Vec2 {
    type Output = Self;
    #[inline]
    fn neg(self) -> Self {
        Self::new(-self.x, -self.y)
    }
}

impl Mul<f32> for Vec2 {
    type Output = Self;
    #[inline]
    fn mul(self, rhs: f32) -> Self {
        Self::new(self.x * rhs, self.y * rhs)
    }
}

impl Mul<Vec2> for f32 {
    type Output = Vec2;
    #[inline]
    fn mul(self, rhs: Vec2) -> Vec2 {
        Vec2::new(self * rhs.x, self * rhs.y)
    }
}

impl MulAssign<f32> for Vec2 {
    #[inline]
    fn mul_assign(&mut self, rhs: f32) {
        self.x *= rhs;
        self.y *= rhs;
    }
}

/// A 2x2 column-major matrix.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct Mat22 {
    pub col1: Vec2,
    pub col2: Vec2,
}

impl Mat22 {
    pub const IDENTITY: Self = Self {
        col1: Vec2::new(1.0, 0.0),
        col2: Vec2::new(0.0, 1.0),
    };

    pub const ZERO: Self = Self {
        col1: Vec2::ZERO,
        col2: Vec2::ZERO,
    };

    #[inline]
    pub const fn from_cols(col1: Vec2, col2: Vec2) -> Self {
        Self { col1, col2 }
    }

    /// Construct a 2D rotation matrix from an angle in radians.
    #[inline]
    pub fn from_angle(angle: f32) -> Self {
        let (s, c) = sin_cos(angle);
        Self {
            col1: Vec2::new(c, s),
            col2: Vec2::new(-s, c),
        }
    }

    #[inline]
    pub fn transpose(self) -> Self {
        Self {
            col1: Vec2::new(self.col1.x, self.col2.x),
            col2: Vec2::new(self.col1.y, self.col2.y),
        }
    }

    #[inline]
    pub fn invert(self) -> Self {
        let a = self.col1.x;
        let b = self.col2.x;
        let c = self.col1.y;
        let d = self.col2.y;
        let det = a * d - b * c;
        if abs(det) < 1e-12 {
            return Self::ZERO;
        }
        let inv_det = 1.0 / det;
        Self {
            col1: Vec2::new(inv_det * d, -inv_det * c),
            col2: Vec2::new(-inv_det * b, inv_det * a),
        }
    }

    #[inline]
    pub fn abs(self) -> Self {
        Self {
            col1: self.col1.abs(),
            col2: self.col2.abs(),
        }
    }
}

impl Mul<Vec2> for Mat22 {
    type Output = Vec2;
    #[inline]
    fn mul(self, v: Vec2) -> Vec2 {
        Vec2::new(
            self.col1.x * v.x + self.col2.x * v.y,
            self.col1.y * v.x + self.col2.y * v.y,
        )
    }
}

impl Mul for Mat22 {
    type Output = Self;
    #[inline]
    fn mul(self, rhs: Self) -> Self {
        Self {
            col1: self * rhs.col1,
            col2: self * rhs.col2,
        }
    }
}

impl Add for Mat22 {
    type Output = Self;
    #[inline]
    fn add(self, rhs: Self) -> Self {
        Self {
            col1: self.col1 + rhs.col1,
            col2: self.col2 + rhs.col2,
        }
    }
}

#[inline]
pub fn clamp(val: f32, min_val: f32, max_val: f32) -> f32 {
    if val < min_val {
        min_val
    } else if val > max_val {
        max_val
    } else {
        val
    }
}

#[inline]
pub fn min(a: f32, b: f32) -> f32 {
    if a < b {
        a
    } else {
        b
    }
}

#[inline]
pub fn max(a: f32, b: f32) -> f32 {
    if a > b {
        a
    } else {
        b
    }
}
