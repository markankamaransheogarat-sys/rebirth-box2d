# Contributing to rebirth-box2d

Thank you for your interest in contributing to `rebirth-box2d`!

## Principles

1. **Safety First**: This library strictly enforces `#![forbid(unsafe_code)]`. No unsafe blocks, no exceptions.
2. **Deterministic Physics**: Physics step results must remain reproducible and stable across platforms.
3. **Zero Unnecessary Dependencies**: The core `rebirth-box2d` crate depends only on optional `libm` for `no_std`.
4. **Honest Attribution**: All credit to Erin Catto for the seminal Box2D-Lite algorithms.

## Development Workflow

1. Clone and navigate to the project directory:
   ```bash
   cd crates/rebirth-box2d
   ```
2. Run test suite:
   ```bash
   cargo test --workspace --all-features
   cargo test -p rebirth-box2d --no-default-features --features libm --lib
   ```
3. Run linter:
   ```bash
   cargo clippy --workspace --all-targets --all-features -- -D warnings
   cargo fmt --all -- --check
   ```
4. Build WebAssembly demo:
   ```bash
   cargo build -p rebirth-box2d-wasm --target wasm32-unknown-unknown --release
   wasm-bindgen target/wasm32-unknown-unknown/release/rebirth_box2d_wasm.wasm --out-dir web/pkg --target web
   ```
5. Clean up build caches when finished to respect disk space limits:
   ```bash
   cargo clean
   ```
