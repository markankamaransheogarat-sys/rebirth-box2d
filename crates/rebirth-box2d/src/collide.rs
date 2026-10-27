//! Box-Box collision detection using the Separating Axis Theorem (SAT) and edge clipping.

use crate::body::Body;
use crate::math::{Mat22, Vec2};

pub const NO_EDGE: u8 = 0;
pub const EDGE1: u8 = 1;
pub const EDGE2: u8 = 2;
pub const EDGE3: u8 = 3;
pub const EDGE4: u8 = 4;

#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct FeaturePair {
    pub in_edge1: u8,
    pub out_edge1: u8,
    pub in_edge2: u8,
    pub out_edge2: u8,
}

impl FeaturePair {
    #[inline]
    pub const fn new(in_edge1: u8, out_edge1: u8, in_edge2: u8, out_edge2: u8) -> Self {
        Self {
            in_edge1,
            out_edge1,
            in_edge2,
            out_edge2,
        }
    }

    #[inline]
    pub const fn as_u32(&self) -> u32 {
        (self.in_edge1 as u32)
            | ((self.out_edge1 as u32) << 8)
            | ((self.in_edge2 as u32) << 16)
            | ((self.out_edge2 as u32) << 24)
    }

    #[inline]
    pub fn flip(&mut self) {
        core::mem::swap(&mut self.in_edge1, &mut self.in_edge2);
        core::mem::swap(&mut self.out_edge1, &mut self.out_edge2);
    }
}

/// A contact point between two colliding bodies.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct Contact {
    pub position: Vec2,
    pub normal: Vec2,
    pub r1: Vec2,
    pub r2: Vec2,
    pub separation: f32,
    /// Accumulated normal impulse
    pub pn: f32,
    /// Accumulated tangent (friction) impulse
    pub pt: f32,
    /// Accumulated normal impulse for position bias
    pub pnb: f32,
    pub mass_normal: f32,
    pub mass_tangent: f32,
    pub bias: f32,
    pub feature: FeaturePair,
}

#[derive(Copy, Clone, Debug, Default)]
struct ClipVertex {
    v: Vec2,
    fp: FeaturePair,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum Axis {
    FaceAX,
    FaceAY,
    FaceBX,
    FaceBY,
}

#[inline]
fn clip_segment_to_line(
    v_out: &mut [ClipVertex; 2],
    v_in: &[ClipVertex; 2],
    normal: Vec2,
    offset: f32,
    clip_edge: u8,
) -> usize {
    let mut num_out = 0;

    let distance0 = normal.dot(v_in[0].v) - offset;
    let distance1 = normal.dot(v_in[1].v) - offset;

    if distance0 <= 0.0 {
        v_out[num_out] = v_in[0];
        num_out += 1;
    }
    if distance1 <= 0.0 {
        v_out[num_out] = v_in[1];
        num_out += 1;
    }

    if distance0 * distance1 < 0.0 {
        let interp = distance0 / (distance0 - distance1);
        let mut pt = ClipVertex {
            v: v_in[0].v + (v_in[1].v - v_in[0].v) * interp,
            fp: FeaturePair::default(),
        };

        if distance0 > 0.0 {
            pt.fp = v_in[0].fp;
            pt.fp.in_edge1 = clip_edge;
            pt.fp.in_edge2 = NO_EDGE;
        } else {
            pt.fp = v_in[1].fp;
            pt.fp.out_edge1 = clip_edge;
            pt.fp.out_edge2 = NO_EDGE;
        }

        v_out[num_out] = pt;
        num_out += 1;
    }

    num_out
}

fn compute_incident_edge(c: &mut [ClipVertex; 2], h: Vec2, pos: Vec2, rot: Mat22, normal: Vec2) {
    let rot_t = rot.transpose();
    let n = -(rot_t * normal);
    let n_abs = n.abs();

    if n_abs.x > n_abs.y {
        if n.x > 0.0 {
            c[0].v = Vec2::new(h.x, -h.y);
            c[0].fp.in_edge2 = EDGE3;
            c[0].fp.out_edge2 = EDGE4;

            c[1].v = Vec2::new(h.x, h.y);
            c[1].fp.in_edge2 = EDGE4;
            c[1].fp.out_edge2 = EDGE1;
        } else {
            c[0].v = Vec2::new(-h.x, h.y);
            c[0].fp.in_edge2 = EDGE1;
            c[0].fp.out_edge2 = EDGE2;

            c[1].v = Vec2::new(-h.x, -h.y);
            c[1].fp.in_edge2 = EDGE2;
            c[1].fp.out_edge2 = EDGE3;
        }
    } else if n.y > 0.0 {
        c[0].v = Vec2::new(h.x, h.y);
        c[0].fp.in_edge2 = EDGE4;
        c[0].fp.out_edge2 = EDGE1;

        c[1].v = Vec2::new(-h.x, h.y);
        c[1].fp.in_edge2 = EDGE1;
        c[1].fp.out_edge2 = EDGE2;
    } else {
        c[0].v = Vec2::new(-h.x, -h.y);
        c[0].fp.in_edge2 = EDGE2;
        c[0].fp.out_edge2 = EDGE3;

        c[1].v = Vec2::new(h.x, -h.y);
        c[1].fp.in_edge2 = EDGE3;
        c[1].fp.out_edge2 = EDGE4;
    }

    c[0].v = pos + rot * c[0].v;
    c[1].v = pos + rot * c[1].v;
}

/// Compute contacts between two rectangular bodies using SAT.
/// The normal points from body A to body B.
/// Returns the number of contact points found (0, 1, or 2).
pub fn collide(contacts: &mut [Contact; 2], body_a: &Body, body_b: &Body) -> usize {
    let h_a = body_a.width * 0.5;
    let h_b = body_b.width * 0.5;

    let pos_a = body_a.position;
    let pos_b = body_b.position;

    let rot_a = Mat22::from_angle(body_a.rotation);
    let rot_b = Mat22::from_angle(body_b.rotation);

    let rot_a_t = rot_a.transpose();
    let rot_b_t = rot_b.transpose();

    let dp = pos_b - pos_a;
    let d_a = rot_a_t * dp;
    let d_b = rot_b_t * dp;

    let c = rot_a_t * rot_b;
    let abs_c = c.abs();
    let abs_c_t = abs_c.transpose();

    // Box A faces
    let face_a = d_a.abs() - h_a - abs_c * h_b;
    if face_a.x > 0.0 || face_a.y > 0.0 {
        return 0;
    }

    // Box B faces
    let face_b = d_b.abs() - abs_c_t * h_a - h_b;
    if face_b.x > 0.0 || face_b.y > 0.0 {
        return 0;
    }

    // Find best axis
    let mut axis = Axis::FaceAX;
    let mut separation = face_a.x;
    let mut normal = if d_a.x > 0.0 { rot_a.col1 } else { -rot_a.col1 };

    let relative_tol = 0.95;
    let absolute_tol = 0.01;

    if face_a.y > relative_tol * separation + absolute_tol * h_a.y {
        axis = Axis::FaceAY;
        separation = face_a.y;
        normal = if d_a.y > 0.0 { rot_a.col2 } else { -rot_a.col2 };
    }

    if face_b.x > relative_tol * separation + absolute_tol * h_b.x {
        axis = Axis::FaceBX;
        separation = face_b.x;
        normal = if d_b.x > 0.0 { rot_b.col1 } else { -rot_b.col1 };
    }

    if face_b.y > relative_tol * separation + absolute_tol * h_b.y {
        axis = Axis::FaceBY;
        normal = if d_b.y > 0.0 { rot_b.col2 } else { -rot_b.col2 };
    }

    let front_normal: Vec2;
    let side_normal: Vec2;
    let mut incident_edge = [ClipVertex::default(); 2];
    let front: f32;
    let neg_side: f32;
    let pos_side: f32;
    let neg_edge: u8;
    let pos_edge: u8;

    match axis {
        Axis::FaceAX => {
            front_normal = normal;
            front = pos_a.dot(front_normal) + h_a.x;
            side_normal = rot_a.col2;
            let side = pos_a.dot(side_normal);
            neg_side = -side + h_a.y;
            pos_side = side + h_a.y;
            neg_edge = EDGE3;
            pos_edge = EDGE1;
            compute_incident_edge(&mut incident_edge, h_b, pos_b, rot_b, front_normal);
        }
        Axis::FaceAY => {
            front_normal = normal;
            front = pos_a.dot(front_normal) + h_a.y;
            side_normal = rot_a.col1;
            let side = pos_a.dot(side_normal);
            neg_side = -side + h_a.x;
            pos_side = side + h_a.x;
            neg_edge = EDGE2;
            pos_edge = EDGE4;
            compute_incident_edge(&mut incident_edge, h_b, pos_b, rot_b, front_normal);
        }
        Axis::FaceBX => {
            front_normal = -normal;
            front = pos_b.dot(front_normal) + h_b.x;
            side_normal = rot_b.col2;
            let side = pos_b.dot(side_normal);
            neg_side = -side + h_b.y;
            pos_side = side + h_b.y;
            neg_edge = EDGE3;
            pos_edge = EDGE1;
            compute_incident_edge(&mut incident_edge, h_a, pos_a, rot_a, front_normal);
        }
        Axis::FaceBY => {
            front_normal = -normal;
            front = pos_b.dot(front_normal) + h_b.y;
            side_normal = rot_b.col1;
            let side = pos_b.dot(side_normal);
            neg_side = -side + h_b.x;
            pos_side = side + h_b.x;
            neg_edge = EDGE2;
            pos_edge = EDGE4;
            compute_incident_edge(&mut incident_edge, h_a, pos_a, rot_a, front_normal);
        }
    }

    let mut clip_points1 = [ClipVertex::default(); 2];
    let mut clip_points2 = [ClipVertex::default(); 2];

    let np1 = clip_segment_to_line(
        &mut clip_points1,
        &incident_edge,
        -side_normal,
        neg_side,
        neg_edge,
    );
    if np1 < 2 {
        return 0;
    }

    let np2 = clip_segment_to_line(
        &mut clip_points2,
        &clip_points1,
        side_normal,
        pos_side,
        pos_edge,
    );
    if np2 < 2 {
        return 0;
    }

    let mut num_contacts = 0;
    for pt in &clip_points2 {
        let sep = front_normal.dot(pt.v) - front;
        if sep <= 0.0 {
            let mut c = Contact {
                separation: sep,
                normal,
                position: pt.v - front_normal * sep,
                feature: pt.fp,
                ..Default::default()
            };

            if axis == Axis::FaceBX || axis == Axis::FaceBY {
                c.feature.flip();
            }

            contacts[num_contacts] = c;
            num_contacts += 1;
        }
    }

    num_contacts
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_collide_ground_box() {
        let ground = Body::new_static(Vec2::new(100.0, 20.0)).with_position(Vec2::new(0.0, -10.0));
        let crate_b = Body::new(Vec2::new(1.0, 1.0), 200.0).with_position(Vec2::new(0.0, 0.45));
        let mut contacts = [Contact::default(); 2];
        let n = collide(&mut contacts, &ground, &crate_b);
        assert_eq!(n, 2);
    }
}
