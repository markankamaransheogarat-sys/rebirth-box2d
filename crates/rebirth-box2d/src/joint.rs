//! Pin (revolute) joint constraint between two bodies.

use crate::arbiter::get_two_mut;
use crate::body::{Body, BodyId};
use crate::math::{cross_scalar_vec, Mat22, Vec2};

/// A strongly typed unique identifier for a Joint.
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct JointId(pub usize);

/// Revolute joint constraining two bodies to share a common pivot anchor.
#[derive(Clone, Debug)]
pub struct Joint {
    pub body1: BodyId,
    pub body2: BodyId,
    pub local_anchor1: Vec2,
    pub local_anchor2: Vec2,
    pub r1: Vec2,
    pub r2: Vec2,
    pub m: Mat22,
    pub bias: Vec2,
    pub p: Vec2,
    pub bias_factor: f32,
    pub softness: f32,
}

impl Joint {
    /// Construct a joint attaching body1 and body2 at the given world-space anchor point.
    pub fn new(b1: BodyId, b2: BodyId, anchor: Vec2, bodies: &[Body]) -> Self {
        let body1 = &bodies[b1.0];
        let body2 = &bodies[b2.0];

        let rot1 = Mat22::from_angle(body1.rotation);
        let rot2 = Mat22::from_angle(body2.rotation);

        let local_anchor1 = rot1.transpose() * (anchor - body1.position);
        let local_anchor2 = rot2.transpose() * (anchor - body2.position);

        Self {
            body1: b1,
            body2: b2,
            local_anchor1,
            local_anchor2,
            r1: Vec2::ZERO,
            r2: Vec2::ZERO,
            m: Mat22::ZERO,
            bias: Vec2::ZERO,
            p: Vec2::ZERO,
            bias_factor: 0.2,
            softness: 0.0,
        }
    }

    /// Builder method to tune constraint softness (spring compliance).
    #[inline]
    pub fn with_softness(mut self, softness: f32) -> Self {
        self.softness = softness;
        self
    }

    /// Builder method to tune position correction bias factor.
    #[inline]
    pub fn with_bias_factor(mut self, bias_factor: f32) -> Self {
        self.bias_factor = bias_factor;
        self
    }

    /// Precompute effective joint mass matrix and apply warm starting.
    pub fn pre_step(
        &mut self,
        bodies: &mut [Body],
        inv_dt: f32,
        warm_starting: bool,
        position_correction: bool,
    ) {
        let (b1, b2) = get_two_mut(bodies, self.body1.0, self.body2.0);

        let rot1 = Mat22::from_angle(b1.rotation);
        let rot2 = Mat22::from_angle(b2.rotation);

        self.r1 = rot1 * self.local_anchor1;
        self.r2 = rot2 * self.local_anchor2;

        let mut k1 = Mat22::default();
        k1.col1.x = b1.inv_mass + b2.inv_mass;
        k1.col2.y = b1.inv_mass + b2.inv_mass;

        let mut k2 = Mat22::default();
        k2.col1.x = b1.inv_i * self.r1.y * self.r1.y;
        k2.col2.x = -b1.inv_i * self.r1.x * self.r1.y;
        k2.col1.y = -b1.inv_i * self.r1.x * self.r1.y;
        k2.col2.y = b1.inv_i * self.r1.x * self.r1.x;

        let mut k3 = Mat22::default();
        k3.col1.x = b2.inv_i * self.r2.y * self.r2.y;
        k3.col2.x = -b2.inv_i * self.r2.x * self.r2.y;
        k3.col1.y = -b2.inv_i * self.r2.x * self.r2.y;
        k3.col2.y = b2.inv_i * self.r2.x * self.r2.x;

        let mut k = k1 + k2 + k3;
        k.col1.x += self.softness;
        k.col2.y += self.softness;

        self.m = k.invert();

        let p1 = b1.position + self.r1;
        let p2 = b2.position + self.r2;
        let dp = p2 - p1;

        if position_correction {
            self.bias = -self.bias_factor * inv_dt * dp;
        } else {
            self.bias = Vec2::ZERO;
        }

        if warm_starting {
            b1.velocity -= self.p * b1.inv_mass;
            b1.angular_velocity -= b1.inv_i * self.r1.cross(self.p);

            b2.velocity += self.p * b2.inv_mass;
            b2.angular_velocity += b2.inv_i * self.r2.cross(self.p);
        } else {
            self.p = Vec2::ZERO;
        }
    }

    /// Iteratively resolve constraint velocity error and accumulate impulse.
    pub fn apply_impulse(&mut self, bodies: &mut [Body]) {
        let (b1, b2) = get_two_mut(bodies, self.body1.0, self.body2.0);

        let dv = (b2.velocity + cross_scalar_vec(b2.angular_velocity, self.r2))
            - (b1.velocity + cross_scalar_vec(b1.angular_velocity, self.r1));

        let impulse = self.m * (self.bias - dv - self.p * self.softness);

        b1.velocity -= impulse * b1.inv_mass;
        b1.angular_velocity -= b1.inv_i * self.r1.cross(impulse);

        b2.velocity += impulse * b2.inv_mass;
        b2.angular_velocity += b2.inv_i * self.r2.cross(impulse);

        self.p += impulse;
    }
}
