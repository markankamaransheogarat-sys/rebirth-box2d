//! WebAssembly bindings for rebirth-box2d.

use rebirth_box2d::{Body, BodyId, Joint, Mat22, Vec2, World};
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub struct WasmWorld {
    world: World,
}

#[wasm_bindgen]
impl WasmWorld {
    #[wasm_bindgen(constructor)]
    pub fn new(gravity_x: f32, gravity_y: f32, iterations: u32) -> Self {
        Self {
            world: World::new(Vec2::new(gravity_x, gravity_y), iterations as usize),
        }
    }

    pub fn set_gravity(&mut self, gx: f32, gy: f32) {
        self.world.gravity = Vec2::new(gx, gy);
    }

    pub fn set_iterations(&mut self, iters: u32) {
        self.world.iterations = iters as usize;
    }

    pub fn set_accumulate_impulses(&mut self, val: bool) {
        self.world.accumulate_impulses = val;
    }

    pub fn set_warm_starting(&mut self, val: bool) {
        self.world.warm_starting = val;
    }

    pub fn set_position_correction(&mut self, val: bool) {
        self.world.position_correction = val;
    }

    pub fn add_box(&mut self, x: f32, y: f32, w: f32, h: f32, mass: f32, friction: f32) -> usize {
        let b = Body::new(Vec2::new(w, h), mass)
            .with_position(Vec2::new(x, y))
            .with_friction(friction);
        self.world.add_body(b).0
    }

    pub fn add_joint(
        &mut self,
        b1: usize,
        b2: usize,
        anchor_x: f32,
        anchor_y: f32,
        softness: f32,
        bias_factor: f32,
    ) -> usize {
        let j = Joint::new(
            BodyId(b1),
            BodyId(b2),
            Vec2::new(anchor_x, anchor_y),
            &self.world.bodies,
        )
        .with_softness(softness)
        .with_bias_factor(bias_factor);
        self.world.add_joint(j).0
    }

    pub fn step(&mut self, dt: f32) {
        self.world.step(dt);
    }

    pub fn clear(&mut self) {
        self.world.clear();
    }

    pub fn body_count(&self) -> usize {
        self.world.bodies.len()
    }

    pub fn joint_count(&self) -> usize {
        self.world.joints.len()
    }

    pub fn arbiter_count(&self) -> usize {
        self.world.arbiters.len()
    }

    /// Returns body state as packed flat float array:
    /// [x, y, rotation, width_x, width_y, is_static, velocity_x, velocity_y, ...]
    /// 8 floats per body.
    pub fn get_bodies_data(&self) -> Vec<f32> {
        let count = self.world.bodies.len();
        let mut data = Vec::with_capacity(count * 8);

        for b in &self.world.bodies {
            data.push(b.position.x);
            data.push(b.position.y);
            data.push(b.rotation);
            data.push(b.width.x);
            data.push(b.width.y);
            data.push(if b.is_static() { 1.0 } else { 0.0 });
            data.push(b.velocity.x);
            data.push(b.velocity.y);
        }

        data
    }

    /// Returns joint endpoints as packed flat float array:
    /// [x1, y1, x2, y2, ...]
    /// 4 floats per joint.
    pub fn get_joints_data(&self) -> Vec<f32> {
        let count = self.world.joints.len();
        let mut data = Vec::with_capacity(count * 4);

        for j in &self.world.joints {
            let b1 = &self.world.bodies[j.body1.0];
            let b2 = &self.world.bodies[j.body2.0];

            let rot1 = Mat22::from_angle(b1.rotation);
            let rot2 = Mat22::from_angle(b2.rotation);

            let p1 = b1.position + rot1 * j.local_anchor1;
            let p2 = b2.position + rot2 * j.local_anchor2;

            data.push(p1.x);
            data.push(p1.y);
            data.push(p2.x);
            data.push(p2.y);
        }

        data
    }

    /// Test which body contains point (x, y). Returns body index or -1 if none.
    pub fn find_body_at(&self, x: f32, y: f32) -> i32 {
        let pt = Vec2::new(x, y);

        // Check top-most (reverse order)
        for (i, b) in self.world.bodies.iter().enumerate().rev() {
            if b.is_static() {
                continue;
            }

            let rot = Mat22::from_angle(b.rotation);
            let local_p = rot.transpose() * (pt - b.position);
            let half_w = b.width * 0.5;

            if local_p.x >= -half_w.x
                && local_p.x <= half_w.x
                && local_p.y >= -half_w.y
                && local_p.y <= half_w.y
            {
                return i as i32;
            }
        }

        -1
    }

    pub fn set_body_position(&mut self, body_idx: usize, x: f32, y: f32) {
        if body_idx < self.world.bodies.len() {
            self.world.bodies[body_idx].position = Vec2::new(x, y);
        }
    }

    pub fn set_body_velocity(&mut self, body_idx: usize, vx: f32, vy: f32) {
        if body_idx < self.world.bodies.len() {
            self.world.bodies[body_idx].velocity = Vec2::new(vx, vy);
        }
    }

    pub fn apply_impulse(&mut self, body_idx: usize, ix: f32, iy: f32) {
        if body_idx < self.world.bodies.len() {
            let inv_m = self.world.bodies[body_idx].inv_mass;
            self.world.bodies[body_idx].velocity += Vec2::new(ix, iy) * inv_m;
        }
    }

    /// Load standard Erin Catto demo scenes.
    pub fn load_demo(&mut self, index: u32) {
        self.world.clear();

        match index {
            0 => self.demo_single_box(),
            1 => self.demo_pendulum(),
            2 => self.demo_friction(),
            3 => self.demo_stack(),
            4 => self.demo_pyramid(),
            5 => self.demo_teeter(),
            6 => self.demo_suspension_bridge(),
            7 => self.demo_dominos(),
            _ => self.demo_pyramid(),
        }
    }

    fn demo_single_box(&mut self) {
        // Ground
        self.add_box(0.0, -10.0, 100.0, 20.0, f32::MAX, 0.2);
        // Single crate
        self.add_box(0.0, 4.0, 1.0, 1.0, 200.0, 0.2);
    }

    fn demo_pendulum(&mut self) {
        let b1 = self.add_box(0.0, -10.0, 100.0, 20.0, f32::MAX, 0.2);
        let b2 = self.add_box(9.0, 11.0, 1.0, 1.0, 100.0, 0.2);
        self.add_joint(b1, b2, 0.0, 11.0, 0.0, 0.2);
    }

    fn demo_friction(&mut self) {
        self.add_box(0.0, -10.0, 100.0, 20.0, f32::MAX, 0.2);

        // Slopes
        let mut r1 = Body::new_static(Vec2::new(13.0, 0.25))
            .with_position(Vec2::new(-2.0, 11.0))
            .with_rotation(-0.25);
        r1.friction = 0.2;
        self.world.add_body(r1);

        let w1 = Body::new_static(Vec2::new(0.25, 1.0)).with_position(Vec2::new(5.25, 9.5));
        self.world.add_body(w1);

        let r2 = Body::new_static(Vec2::new(13.0, 0.25))
            .with_position(Vec2::new(2.0, 7.0))
            .with_rotation(0.25);
        self.world.add_body(r2);

        let w2 = Body::new_static(Vec2::new(0.25, 1.0)).with_position(Vec2::new(-5.25, 5.5));
        self.world.add_body(w2);

        let r3 = Body::new_static(Vec2::new(13.0, 0.25))
            .with_position(Vec2::new(-2.0, 3.0))
            .with_rotation(-0.25);
        self.world.add_body(r3);

        let frictions = [0.75, 0.5, 0.35, 0.1, 0.0];
        for (i, &f) in frictions.iter().enumerate() {
            let x = -7.5 + 2.0 * i as f32;
            self.add_box(x, 14.0, 0.5, 0.5, 25.0, f);
        }
    }

    fn demo_stack(&mut self) {
        self.add_box(0.0, -10.0, 100.0, 20.0, f32::MAX, 0.2);

        for i in 0..10 {
            let jitter = (i as f32 * 0.02) - 0.01;
            self.add_box(jitter, 0.51 + 1.05 * i as f32, 1.0, 1.0, 1.0, 0.2);
        }
    }

    fn demo_pyramid(&mut self) {
        self.add_box(0.0, -10.0, 100.0, 20.0, f32::MAX, 0.2);

        let mut x = Vec2::new(-6.0, 0.75);
        for i in 0..10 {
            let mut y = x;
            for _ in i..10 {
                self.add_box(y.x, y.y, 1.0, 1.0, 10.0, 0.2);
                y += Vec2::new(1.125, 0.0);
            }
            x += Vec2::new(0.5625, 1.5);
        }
    }

    fn demo_teeter(&mut self) {
        let b1 = self.add_box(0.0, -10.0, 100.0, 20.0, f32::MAX, 0.2);
        let b2 = self.add_box(0.0, 1.0, 12.0, 0.25, 100.0, 0.2);

        self.add_box(-5.0, 2.0, 0.5, 0.5, 25.0, 0.2);
        self.add_box(-5.5, 2.0, 0.5, 0.5, 25.0, 0.2);
        self.add_box(5.5, 15.0, 1.0, 1.0, 100.0, 0.2);

        self.add_joint(b1, b2, 0.0, 1.0, 0.0, 0.2);
    }

    fn demo_suspension_bridge(&mut self) {
        self.add_box(0.0, -10.0, 100.0, 20.0, f32::MAX, 0.2);

        let num_planks = 15;
        let mass = 50.0;
        let time_step = 1.0 / 60.0;

        let mut plank_ids = Vec::with_capacity(num_planks);
        for i in 0..num_planks {
            let id = self.add_box(-8.5 + 1.25 * i as f32, 5.0, 1.0, 0.25, mass, 0.2);
            plank_ids.push(id);
        }

        let frequency_hz = 2.0;
        let damping_ratio = 0.7;
        let omega = 2.0 * core::f32::consts::PI * frequency_hz;
        let d = 2.0 * mass * damping_ratio * omega;
        let k = mass * omega * omega;
        let softness = 1.0 / (d + time_step * k);
        let bias_factor = time_step * k / (d + time_step * k);

        let ground_id = 0;

        for i in 0..num_planks {
            let b1 = if i == 0 { ground_id } else { plank_ids[i - 1] };
            let b2 = plank_ids[i];
            let anchor_x = -9.125 + 1.25 * i as f32;
            self.add_joint(b1, b2, anchor_x, 5.0, softness, bias_factor);
        }

        let last_plank = plank_ids[num_planks - 1];
        let anchor_x = -9.125 + 1.25 * num_planks as f32;
        self.add_joint(last_plank, ground_id, anchor_x, 5.0, softness, bias_factor);
    }

    fn demo_dominos(&mut self) {
        self.add_box(0.0, -10.0, 100.0, 20.0, f32::MAX, 0.2);

        // Floor platform
        self.add_box(-1.5, 4.0, 12.0, 0.5, f32::MAX, 0.2);

        // 10 dominos standing
        for i in 0..10 {
            self.add_box(-6.0 + 1.0 * i as f32, 4.75, 0.1, 1.0, 10.0, 0.1);
        }

        // Toppler box falling from above on the first domino
        let toppler = Body::new(Vec2::new(0.5, 0.5), 10.0)
            .with_position(Vec2::new(-6.8, 6.0))
            .with_velocity(Vec2::new(2.0, 0.0));
        self.world.add_body(toppler);
    }
}
