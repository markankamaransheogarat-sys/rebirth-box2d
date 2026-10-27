//! # rebirth-box2d
//!
//! A pure, 100% safe Rust port of Erin Catto's classic **Box2D-Lite** 2D physics engine.
//!
//! Features:
//! - `#![forbid(unsafe_code)]` — strictly zero unsafe code.
//! - `no_std` compatible (with `default-features = false` and `features = ["libm"]`).
//! - **Sequential Impulses**: Fast, robust constraint solving for resting contact stacks and friction.
//! - **Warm Starting**: Cached normal and friction impulses across simulation steps for stack stability.
//! - **Revolute Joints**: Soft spring compliance, bias position correction, and pivot constraints.
//! - **Zero Dependencies**: Independent algorithmic core with optional `libm` for embedded targets.
//!
//! ## Quick Start
//! ```rust
//! use rebirth_box2d::{World, Body, Vec2};
//!
//! // Create world with earth gravity (y pointing down: -9.81 m/s^2)
//! let mut world = World::new(Vec2::new(0.0, -9.81), 10);
//!
//! // Add static ground platform (width: 100m, height: 20m)
//! let ground = Body::new_static(Vec2::new(100.0, 20.0))
//!     .with_position(Vec2::new(0.0, -10.0));
//! world.add_body(ground);
//!
//! // Add dynamic falling crate (1m x 1m, 10kg)
//! let crate_body = Body::new(Vec2::new(1.0, 1.0), 10.0)
//!     .with_position(Vec2::new(0.0, 5.0));
//! let crate_id = world.add_body(crate_body);
//!
//! // Simulate 60 steps at 60 Hz (1 second)
//! for _ in 0..60 {
//!     world.step(1.0 / 60.0);
//! }
//!
//! // Crate has safely landed on top of ground (ground top = 0.0, crate half-height = 0.5)
//! assert!(world.body(crate_id).position.y > 0.49);
//! ```

#![forbid(unsafe_code)]
#![cfg_attr(not(feature = "std"), no_std)]

#[cfg(not(feature = "std"))]
extern crate alloc;

pub mod arbiter;
pub mod body;
pub mod collide;
pub mod joint;
pub mod math;
pub mod world;

pub use arbiter::{Arbiter, ArbiterKey};
pub use body::{Body, BodyId};
pub use collide::{collide, Contact, FeaturePair};
pub use joint::{Joint, JointId};
pub use math::{Mat22, Vec2};
pub use world::World;
