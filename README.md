# rebirth-box2d

[![crates.io](https://img.shields.io/crates/v/rebirth-box2d.svg)](https://crates.io/crates/rebirth-box2d)
[![docs.rs](https://docs.rs/rebirth-box2d/badge.svg)](https://docs.rs/rebirth-box2d)
[![CI](https://github.com/markankamaransheogarat-sys/rebirth-box2d/actions/workflows/ci.yml/badge.svg)](https://github.com/markankamaransheogarat-sys/rebirth-box2d/actions)
[![Demo](https://img.shields.io/badge/live-demo%20(WASM)-4fd1c5)](https://markankamaransheogarat-sys.github.io/rebirth-box2d/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Safety: forbid unsafe](https://img.shields.io/badge/unsafe-forbidden-success.svg)](#safety)

A 100% safe Rust, zero-dependency reincarnation of Erin Catto's legendary **Box2D-Lite** 2D physics engine.

Featuring **Sequential Impulses**, **Persistent Contact Caching**, **Warm Starting**, **Revolute Joint Constraints**, and `no_std` + **WebAssembly** support.

Part of the [Open-Source Necromancer](https://github.com/markankamaransheogarat-sys/oss-necromancer) initiative: reviving foundational game mathematics with modern safety, comprehensive testing, and instant WebAssembly demos.

---

### [🎮 Live Interactive WebAssembly Demo](https://markankamaransheogarat-sys.github.io/rebirth-box2d/)

Play with 8 interactive scenes (Pyramid, Suspension Bridge, Dominos, Teeter-Totter, Pendulum, Vertical Stack, Friction Ramps, Single Crate) right in your browser with mouse drag & throw, real-time telemetry, and bomb launching.

---

## Features

- **Strictly Safe**: `#![forbid(unsafe_code)]` with zero raw pointer dereferences or undefined behavior.
- **Sequential Impulses**: Stable solver for resting contact stacks, impacts, and Coulomb friction.
- **Warm Starting**: Caches normal and friction impulses across simulation frames using `FeaturePair` keying.
- **Revolute Pin Joints**: Bilateral position constraints with adjustable spring compliance (`softness`) and Baumgarte bias stabilization.
- **Embedded & `no_std` Ready**: Compile without the standard library on bare-metal microcontrollers (e.g. ARM Cortex-M4F) via `features = ["libm"]`.
- **Zero Unnecessary Dependencies**: Pure mathematical core with no heavyweight third-party runtime crates.
- **WebAssembly Ready**: Efficient flat array buffers for high-framerate HTML5 Canvas / WebGL rendering.

---

## Measured Performance (Criterion)

Benchmarked on AMD/Intel x86_64 (`cargo bench`, 100 samples):

| Benchmark Scenario | Description | Measured Step Time | Steps / Second |
| :--- | :--- | :---: | :---: |
| `pyramid_78_boxes` | 12-tier pyramid (78 dynamic boxes + ground), 10 solver iterations | **148.0 µs** | ~6,750 Hz |
| `suspension_bridge_15_planks` | 15 interconnected planks with 16 spring-damper revolute joints | **10.6 µs** | ~94,300 Hz |

*Note: Measured with `opt-level = 3`, `lto = "fat"`, `codegen-units = 1`. No head-to-head comparison with original C++ code is published.*

---

## Quick Start

Add `rebirth-box2d` to your `Cargo.toml`:

```toml
[dependencies]
rebirth-box2d = "0.1.0"
```

### Basic Simulation

```rust
use rebirth_box2d::{World, Body, Vec2};

fn main() {
    // 1. Initialize world with gravity (y pointing down: -9.81 m/s^2)
    let mut world = World::new(Vec2::new(0.0, -9.81), 10);

    // 2. Add static ground platform (100m wide, 20m tall at y = -10.0 -> top surface at y = 0.0)
    let ground = Body::new_static(Vec2::new(100.0, 20.0))
        .with_position(Vec2::new(0.0, -10.0));
    world.add_body(ground);

    // 3. Add dynamic crate (1m x 1m, 10 kg, dropped from y = 5.0m)
    let crate_box = Body::new(Vec2::new(1.0, 1.0), 10.0)
        .with_position(Vec2::new(0.0, 5.0));
    let crate_id = world.add_body(crate_box);

    // 4. Step physics at 60 Hz (1 second simulation)
    for _ in 0..60 {
        world.step(1.0 / 60.0);
    }

    // Crate has safely landed on the ground
    let body = world.body(crate_id);
    println!("Crate resting position: ({:.2}, {:.2})", body.position.x, body.position.y);
    assert!((body.position.y - 0.5).abs() < 0.05);
}
```

---

## Pin / Revolute Joints

```rust
use rebirth_box2d::{World, Body, Joint, Vec2};

let mut world = World::new(Vec2::new(0.0, -9.81), 10);

// Static ceiling anchor
let ceiling = Body::new_static(Vec2::new(10.0, 1.0))
    .with_position(Vec2::new(0.0, 10.0));
let ceiling_id = world.add_body(ceiling);

// Pendulum bob (1m x 1m, 50kg)
let bob = Body::new(Vec2::new(1.0, 1.0), 50.0)
    .with_position(Vec2::new(5.0, 10.0));
let bob_id = world.add_body(bob);

// Connect with a revolute joint pinned at (0.0, 10.0)
let joint = Joint::new(ceiling_id, bob_id, Vec2::new(0.0, 10.0), &world.bodies)
    .with_softness(0.0)       // Rigid constraint
    .with_bias_factor(0.2);   // Baumgarte position correction
world.add_joint(joint);
```

---

## Feature Flags

| Feature | Description | Default |
| :--- | :--- | :---: |
| `std` | Uses standard library float operations. | **Yes** |
| `libm` | Uses `libm` software float math for pure `no_std` embedded targets. | No |

For embedded targets (e.g. `thumbv7em-none-eabihf`):
```toml
[dependencies]
rebirth-box2d = { version = "0.1.0", default-features = false, features = ["libm"] }
```

---

## Attribution & Origins

This library is a faithful safe Rust port of **[Box2D-Lite](https://github.com/erincatto/box2d-lite)** created by **Erin Catto** in 2006 for his GDC tutorial *"Iterative Dynamics with Temporal Coherence"*.

The original C++ implementation is licensed under the MIT License:
```text
Copyright (c) 2006-2009 Erin Catto http://www.gphysics.com
```

The Rust port preserves the algorithmic core, Separating Axis Theorem (SAT) clipping, impulse accumulation, and warm starting while providing 100% memory safety, `no_std` versatility, and modern WebAssembly bindings.

---

## License

Dual-attributed under the [MIT License](LICENSE).
