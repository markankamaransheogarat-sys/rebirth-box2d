//! Rigid body definition and state.

use crate::math::Vec2;

/// A strongly typed unique identifier for a Body within a World.
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct BodyId(pub usize);

/// A rigid 2D rectangular body.
#[derive(Clone, Debug, PartialEq)]
pub struct Body {
    pub position: Vec2,
    pub rotation: f32,

    pub velocity: Vec2,
    pub angular_velocity: f32,

    pub force: Vec2,
    pub torque: f32,

    pub width: Vec2,

    pub friction: f32,
    pub mass: f32,
    pub inv_mass: f32,
    pub i: f32,
    pub inv_i: f32,
}

impl Default for Body {
    fn default() -> Self {
        Self {
            position: Vec2::ZERO,
            rotation: 0.0,
            velocity: Vec2::ZERO,
            angular_velocity: 0.0,
            force: Vec2::ZERO,
            torque: 0.0,
            width: Vec2::new(1.0, 1.0),
            friction: 0.2,
            mass: f32::MAX,
            inv_mass: 0.0,
            i: f32::MAX,
            inv_i: 0.0,
        }
    }
}

impl Body {
    /// Create a dynamic or static rectangular body with the specified dimensions and mass.
    /// If `mass >= f32::MAX` or `mass <= 0.0`, the body is treated as static (infinite mass/inertia).
    pub fn new(width: Vec2, mass: f32) -> Self {
        let mut body = Self::default();
        body.set(width, mass);
        body
    }

    /// Create an immovable static rectangular body (ground, platform, wall).
    pub fn new_static(width: Vec2) -> Self {
        Self::new(width, f32::MAX)
    }

    /// Reconfigure the dimensions and mass of this body.
    pub fn set(&mut self, width: Vec2, mass: f32) {
        self.width = width;
        self.mass = mass;

        if mass < f32::MAX && mass > 0.0 {
            self.inv_mass = 1.0 / mass;
            self.i = mass * (width.x * width.x + width.y * width.y) / 12.0;
            self.inv_i = 1.0 / self.i;
        } else {
            self.mass = f32::MAX;
            self.inv_mass = 0.0;
            self.i = f32::MAX;
            self.inv_i = 0.0;
        }
    }

    #[inline]
    pub fn with_position(mut self, pos: Vec2) -> Self {
        self.position = pos;
        self
    }

    #[inline]
    pub fn with_rotation(mut self, rot: f32) -> Self {
        self.rotation = rot;
        self
    }

    #[inline]
    pub fn with_velocity(mut self, vel: Vec2) -> Self {
        self.velocity = vel;
        self
    }

    #[inline]
    pub fn with_angular_velocity(mut self, ang_vel: f32) -> Self {
        self.angular_velocity = ang_vel;
        self
    }

    #[inline]
    pub fn with_friction(mut self, friction: f32) -> Self {
        self.friction = friction;
        self
    }

    #[inline]
    pub fn add_force(&mut self, f: Vec2) {
        self.force += f;
    }

    #[inline]
    pub fn add_torque(&mut self, t: f32) {
        self.torque += t;
    }

    #[inline]
    pub fn is_static(&self) -> bool {
        self.inv_mass == 0.0
    }
}
