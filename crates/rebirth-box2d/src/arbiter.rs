//! Arbiter manages contact persistence, impulse accumulation, and friction solver between a pair of bodies.

use crate::body::{Body, BodyId};
use crate::collide::{collide, Contact};
use crate::math::{clamp, cross_scalar_vec, max, min, sqrt};

/// Helper to safely borrow two distinct mutable elements from a slice without unsafe code.
#[inline]
pub(crate) fn get_two_mut<T>(slice: &mut [T], a: usize, b: usize) -> (&mut T, &mut T) {
    assert!(a != b, "indices must be distinct");
    if a < b {
        let (first, second) = slice.split_at_mut(b);
        (&mut first[a], &mut second[0])
    } else {
        let (first, second) = slice.split_at_mut(a);
        (&mut second[0], &mut first[b])
    }
}

/// Canonical key identifying a collision pair between two bodies.
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct ArbiterKey {
    pub body1: BodyId,
    pub body2: BodyId,
}

impl ArbiterKey {
    #[inline]
    pub fn new(b1: BodyId, b2: BodyId) -> Self {
        if b1 < b2 {
            Self {
                body1: b1,
                body2: b2,
            }
        } else {
            Self {
                body1: b2,
                body2: b1,
            }
        }
    }
}

/// Holds active contact points and cached impulses between two bodies.
#[derive(Clone, Debug)]
pub struct Arbiter {
    pub body1: BodyId,
    pub body2: BodyId,
    pub contacts: [Contact; 2],
    pub num_contacts: usize,
    pub friction: f32,
}

impl Arbiter {
    /// Create a new arbiter and immediately test for contacts between the two bodies.
    pub fn new(b1: BodyId, b2: BodyId, bodies: &[Body]) -> Self {
        let (id1, id2) = if b1 < b2 { (b1, b2) } else { (b2, b1) };
        let mut contacts = [Contact::default(); 2];
        let num_contacts = collide(&mut contacts, &bodies[id1.0], &bodies[id2.0]);
        let friction = sqrt(bodies[id1.0].friction * bodies[id2.0].friction);

        Self {
            body1: id1,
            body2: id2,
            contacts,
            num_contacts,
            friction,
        }
    }

    /// Update contact points while retaining warm-started impulses from matching features.
    pub fn update(&mut self, new_contacts: &[Contact; 2], num_new: usize, warm_starting: bool) {
        let mut merged = [Contact::default(); 2];

        for i in 0..num_new {
            let c_new = &new_contacts[i];
            let mut matched_idx = None;

            for j in 0..self.num_contacts {
                if c_new.feature.as_u32() == self.contacts[j].feature.as_u32() {
                    matched_idx = Some(j);
                    break;
                }
            }

            if let Some(k) = matched_idx {
                let mut c = *c_new;
                let c_old = &self.contacts[k];
                if warm_starting {
                    c.pn = c_old.pn;
                    c.pt = c_old.pt;
                    c.pnb = c_old.pnb;
                } else {
                    c.pn = 0.0;
                    c.pt = 0.0;
                    c.pnb = 0.0;
                }
                merged[i] = c;
            } else {
                merged[i] = *c_new;
            }
        }

        self.contacts[..num_new].copy_from_slice(&merged[..num_new]);
        self.num_contacts = num_new;
    }

    /// Precompute effective masses and apply warm-started impulses before solver iterations.
    pub fn pre_step(
        &mut self,
        bodies: &mut [Body],
        inv_dt: f32,
        accumulate_impulses: bool,
        position_correction: bool,
    ) {
        const ALLOWED_PENETRATION: f32 = 0.01;
        let bias_factor = if position_correction { 0.2 } else { 0.0 };

        let (b1, b2) = get_two_mut(bodies, self.body1.0, self.body2.0);

        for i in 0..self.num_contacts {
            let c = &mut self.contacts[i];

            let r1 = c.position - b1.position;
            let r2 = c.position - b2.position;

            // Precompute normal mass
            let rn1 = r1.dot(c.normal);
            let rn2 = r2.dot(c.normal);
            let mut k_normal = b1.inv_mass + b2.inv_mass;
            k_normal += b1.inv_i * (r1.dot(r1) - rn1 * rn1) + b2.inv_i * (r2.dot(r2) - rn2 * rn2);
            c.mass_normal = if k_normal > 0.0 { 1.0 / k_normal } else { 0.0 };

            // Precompute tangent (friction) mass
            let tangent = c.normal.cross_scalar(1.0);
            let rt1 = r1.dot(tangent);
            let rt2 = r2.dot(tangent);
            let mut k_tangent = b1.inv_mass + b2.inv_mass;
            k_tangent += b1.inv_i * (r1.dot(r1) - rt1 * rt1) + b2.inv_i * (r2.dot(r2) - rt2 * rt2);
            c.mass_tangent = if k_tangent > 0.0 {
                1.0 / k_tangent
            } else {
                0.0
            };

            c.bias = -bias_factor * inv_dt * min(0.0, c.separation + ALLOWED_PENETRATION);

            if accumulate_impulses {
                let p = c.normal * c.pn + tangent * c.pt;

                b1.velocity -= p * b1.inv_mass;
                b1.angular_velocity -= b1.inv_i * r1.cross(p);

                b2.velocity += p * b2.inv_mass;
                b2.angular_velocity += b2.inv_i * r2.cross(p);
            }
        }
    }

    /// Iteratively resolve contact normal and friction impulses.
    pub fn apply_impulse(&mut self, bodies: &mut [Body], accumulate_impulses: bool) {
        let (b1, b2) = get_two_mut(bodies, self.body1.0, self.body2.0);

        for i in 0..self.num_contacts {
            let c = &mut self.contacts[i];
            c.r1 = c.position - b1.position;
            c.r2 = c.position - b2.position;

            // Relative velocity at contact point
            let dv = (b2.velocity + cross_scalar_vec(b2.angular_velocity, c.r2))
                - (b1.velocity + cross_scalar_vec(b1.angular_velocity, c.r1));

            // Normal impulse
            let vn = dv.dot(c.normal);
            let mut dpn = c.mass_normal * (-vn + c.bias);

            if accumulate_impulses {
                let pn0 = c.pn;
                c.pn = max(pn0 + dpn, 0.0);
                dpn = c.pn - pn0;
            } else {
                dpn = max(dpn, 0.0);
            }

            let pn = c.normal * dpn;
            b1.velocity -= pn * b1.inv_mass;
            b1.angular_velocity -= b1.inv_i * c.r1.cross(pn);

            b2.velocity += pn * b2.inv_mass;
            b2.angular_velocity += b2.inv_i * c.r2.cross(pn);

            // Relative velocity for friction
            let dv = (b2.velocity + cross_scalar_vec(b2.angular_velocity, c.r2))
                - (b1.velocity + cross_scalar_vec(b1.angular_velocity, c.r1));

            let tangent = c.normal.cross_scalar(1.0);
            let vt = dv.dot(tangent);
            let mut dpt = c.mass_tangent * (-vt);

            if accumulate_impulses {
                let max_pt = self.friction * c.pn;
                let old_pt = c.pt;
                c.pt = clamp(old_pt + dpt, -max_pt, max_pt);
                dpt = c.pt - old_pt;
            } else {
                let max_pt = self.friction * dpn;
                dpt = clamp(dpt, -max_pt, max_pt);
            }

            let pt = tangent * dpt;
            b1.velocity -= pt * b1.inv_mass;
            b1.angular_velocity -= b1.inv_i * c.r1.cross(pt);

            b2.velocity += pt * b2.inv_mass;
            b2.angular_velocity += b2.inv_i * c.r2.cross(pt);
        }
    }
}
