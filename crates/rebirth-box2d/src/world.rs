//! The physics simulation World coordinating bodies, joints, broadphase, and sequential impulse solver.

#[cfg(not(feature = "std"))]
use alloc::collections::BTreeMap;
#[cfg(not(feature = "std"))]
use alloc::vec::Vec;

#[cfg(feature = "std")]
use std::collections::BTreeMap;

use crate::arbiter::{Arbiter, ArbiterKey};
use crate::body::{Body, BodyId};
use crate::collide::{collide, Contact};
use crate::joint::{Joint, JointId};
use crate::math::{sqrt, Vec2};

/// The primary simulation container holding bodies, joints, and collision contacts.
#[derive(Clone, Debug)]
pub struct World {
    pub bodies: Vec<Body>,
    pub joints: Vec<Joint>,
    pub arbiters: BTreeMap<ArbiterKey, Arbiter>,
    pub gravity: Vec2,
    pub iterations: usize,
    pub accumulate_impulses: bool,
    pub warm_starting: bool,
    pub position_correction: bool,
}

impl Default for World {
    fn default() -> Self {
        Self::new(Vec2::new(0.0, -10.0), 10)
    }
}

impl World {
    /// Construct a new simulation world with given gravity vector and velocity solver iterations per step.
    pub fn new(gravity: Vec2, iterations: usize) -> Self {
        Self {
            bodies: Vec::new(),
            joints: Vec::new(),
            arbiters: BTreeMap::new(),
            gravity,
            iterations,
            accumulate_impulses: true,
            warm_starting: true,
            position_correction: true,
        }
    }

    /// Add a body to the world, returning its strongly typed `BodyId`.
    pub fn add_body(&mut self, body: Body) -> BodyId {
        let id = BodyId(self.bodies.len());
        self.bodies.push(body);
        id
    }

    /// Add a joint constraint between two bodies.
    pub fn add_joint(&mut self, joint: Joint) -> JointId {
        let id = JointId(self.joints.len());
        self.joints.push(joint);
        id
    }

    /// Get immutable reference to a body by ID.
    #[inline]
    pub fn body(&self, id: BodyId) -> &Body {
        &self.bodies[id.0]
    }

    /// Get mutable reference to a body by ID.
    #[inline]
    pub fn body_mut(&mut self, id: BodyId) -> &mut Body {
        &mut self.bodies[id.0]
    }

    /// Get immutable reference to a joint by ID.
    #[inline]
    pub fn joint(&self, id: JointId) -> &Joint {
        &self.joints[id.0]
    }

    /// Get mutable reference to a joint by ID.
    #[inline]
    pub fn joint_mut(&mut self, id: JointId) -> &mut Joint {
        &mut self.joints[id.0]
    }

    /// Remove all bodies, joints, and contacts.
    pub fn clear(&mut self) {
        self.bodies.clear();
        self.joints.clear();
        self.arbiters.clear();
    }

    /// Execute the broadphase collision detection: find overlapping bodies and update contact caches.
    pub fn broad_phase(&mut self) {
        let num_bodies = self.bodies.len();
        let mut contacts_buf = [Contact::default(); 2];

        for i in 0..num_bodies {
            for j in (i + 1)..num_bodies {
                let bi = &self.bodies[i];
                let bj = &self.bodies[j];

                if bi.is_static() && bj.is_static() {
                    continue;
                }

                let num_contacts = collide(&mut contacts_buf, bi, bj);
                let key = ArbiterKey::new(BodyId(i), BodyId(j));

                if num_contacts > 0 {
                    let friction = sqrt(bi.friction * bj.friction);
                    let warm_starting = self.warm_starting;

                    self.arbiters
                        .entry(key)
                        .and_modify(|arb| {
                            arb.update(&contacts_buf, num_contacts, warm_starting);
                        })
                        .or_insert_with(|| Arbiter {
                            body1: key.body1,
                            body2: key.body2,
                            contacts: contacts_buf,
                            num_contacts,
                            friction,
                        });
                } else {
                    self.arbiters.remove(&key);
                }
            }
        }
    }

    /// Advance the physics simulation by time `dt` seconds.
    pub fn step(&mut self, dt: f32) {
        let inv_dt = if dt > 0.0 { 1.0 / dt } else { 0.0 };

        // 1. Broadphase collision detection and contact caching
        self.broad_phase();

        // 2. Integrate forces to predict velocities
        for b in &mut self.bodies {
            if b.is_static() {
                continue;
            }

            b.velocity += (self.gravity + b.force * b.inv_mass) * dt;
            b.angular_velocity += (b.torque * b.inv_i) * dt;
        }

        // 3. Pre-step contacts
        for arb in self.arbiters.values_mut() {
            arb.pre_step(
                &mut self.bodies,
                inv_dt,
                self.accumulate_impulses,
                self.position_correction,
            );
        }

        // 4. Pre-step joints
        for j in &mut self.joints {
            j.pre_step(
                &mut self.bodies,
                inv_dt,
                self.warm_starting,
                self.position_correction,
            );
        }

        // 5. Sequential impulse solver iterations
        for _ in 0..self.iterations {
            for arb in self.arbiters.values_mut() {
                arb.apply_impulse(&mut self.bodies, self.accumulate_impulses);
            }

            for j in &mut self.joints {
                j.apply_impulse(&mut self.bodies);
            }
        }

        // 6. Integrate velocities to update positions
        for b in &mut self.bodies {
            if b.is_static() {
                continue;
            }

            b.position += b.velocity * dt;
            b.rotation += b.angular_velocity * dt;

            b.force = Vec2::ZERO;
            b.torque = 0.0;
        }
    }
}
