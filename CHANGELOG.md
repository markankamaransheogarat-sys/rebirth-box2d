# Changelog

All notable changes to `rebirth-box2d` will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.1.0] - 2026-10-27

### Added
- Complete safe Rust (`#![forbid(unsafe_code)]`) rewrite of Erin Catto's classic Box2D-Lite 2D physics engine.
- 2D mathematical vector and matrix algebra (`Vec2`, `Mat22`) with support for rotation, inversion, cross products, and clamping.
- Box-Box collision detection using Separating Axis Theorem (SAT) and incident edge clipping.
- Persistent contact caching with `FeaturePair` key matching for warm starting.
- Sequential impulse velocity constraint solver for resting stacks, impacts, and Coulomb friction.
- Position bias stabilization to eliminate overlap without velocity bounce artifacts.
- Revolute/pin joint constraints with spring compliance (`softness`) and Baumgarte position stabilization (`bias_factor`).
- `no_std` support with optional `libm` software float math (tested on Cortex-M4F microcontroller target).
- WebAssembly bindings (`rebirth-box2d-wasm`) with packed flat data arrays for high-performance canvas rendering.
- Interactive HTML5 Canvas browser demo with 8 classic Erin Catto scenes: Pyramid, Suspension Bridge, Dominos, Teeter-Totter, Pendulum, Vertical Stack, Friction Ramps, and Single Crate.
- Criterion benchmarks: 78-box pyramid simulated in ~148 µs/step; 15-plank suspension bridge in ~10.6 µs/step.
- CI matrix covering Linux, macOS, and Windows with clippy and formatting checks.
