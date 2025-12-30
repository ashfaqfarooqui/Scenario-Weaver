# Phase 1: Project Setup

**Prerequisites**: None (starting from scratch)

**Duration**: 1-2 days

---

## Context

This is the foundation phase. We're setting up a Rust project with all necessary dependencies and directory structure.

**Why this phase**: Without proper setup, later phases can't be implemented. We need:
- Rust project structure
- Z3 SMT solver integration (core dependency)
- Serialization libraries (YAML input, JSON output)
- Error handling framework
- Testing infrastructure

**What problem it solves**: Establishes development environment and toolchain for all subsequent work.

---

## Goals

- [ ] Create Rust project with cargo
- [ ] Configure dependencies in Cargo.toml
- [ ] Set up module structure
- [ ] Create basic error types
- [ ] Verify Z3 integration works
- [ ] Create example files directory

---

## Implementation Steps

### Step 1: Create Rust Project

```bash
cargo new carla-scenario-generator --lib
cd carla-scenario-generator
```

**Why `--lib`**: We're building a library with a CLI frontend, not just a binary.

### Step 2: Configure Cargo.toml

Create comprehensive dependency list:

```toml
[package]
name = "carla-scenario-generator"
version = "0.1.0"
edition = "2021"

[[bin]]
name = "carla-scenario-gen"
path = "src/main.rs"

[lib]
name = "carla_scenario_generator"
path = "src/lib.rs"

[dependencies]
# Z3 SMT Solver - core constraint solving
z3 = { version = "0.12", features = ["static-link-z3"] }

# Serialization - YAML input, JSON output
serde = { version = "1.0", features = ["derive"] }
serde_yaml = "0.9"
serde_json = "1.0"

# Utilities
chrono = { version = "0.4", features = ["serde"] }  # Timestamps
uuid = { version = "1.6", features = ["v4", "serde"] }  # Scenario IDs
anyhow = "1.0"  # Error handling convenience
thiserror = "1.0"  # Custom error types

# CLI (will be used in Phase 12)
clap = { version = "4.4", features = ["derive"] }

# Logging (will be used in Phase 12)
tracing = "0.1"
tracing-subscriber = "0.3"

[dev-dependencies]
pretty_assertions = "1.4"  # Better test output
```

**Key dependencies explained**:
- **z3**: The SMT solver - most important dependency
- **serde**: Rust's serialization framework (derive macros auto-generate parsers)
- **anyhow/thiserror**: Rust error handling best practices
- **clap**: Modern CLI argument parsing

**Important**: `static-link-z3` feature embeds Z3 in the binary (no runtime dependency)

### Step 3: Create Module Structure

```bash
mkdir -p src/{dsl,ltl,solver,scenario}
mkdir -p examples
mkdir -p tests/fixtures
```

Create module files:

**src/lib.rs**:
```rust
//! CARLA Scenario Generator
//!
//! Generate driving test scenarios from high-level specifications using
//! Linear Temporal Logic (LTL) + Z3 SMT solver.

pub mod dsl;
pub mod ltl;
pub mod solver;
pub mod scenario;
pub mod error;

use anyhow::Result;

/// Main entry point for scenario generation (to be implemented in Phase 10)
pub fn generate_scenarios(yaml_content: &str, num_scenarios: usize) -> Result<Vec<scenario::model::Scenario>> {
    todo!("To be implemented in Phase 10")
}
```

**src/main.rs**:
```rust
//! CLI for CARLA Scenario Generator

use clap::Parser;
use std::path::PathBuf;
use anyhow::Result;

#[derive(Parser)]
#[command(name = "carla-scenario-gen")]
#[command(about = "Generate CARLA driving scenarios using LTL + Z3", long_about = None)]
struct Cli {
    /// Input YAML specification file
    #[arg(short, long)]
    input: PathBuf,

    /// Output JSON file (or directory for multiple scenarios)
    #[arg(short, long)]
    output: PathBuf,

    /// Enable verbose logging
    #[arg(short, long)]
    verbose: bool,
}

fn main() -> Result<()> {
    println!("CARLA Scenario Generator - To be implemented in Phase 12");
    println!("Run tests with: cargo test");
    Ok(())
}
```

**src/dsl/mod.rs**:
```rust
//! DSL (Domain-Specific Language) module
//!
//! Parses YAML specifications into structured types

pub mod types;
pub mod parser;
```

**src/ltl/mod.rs**:
```rust
//! LTL (Linear Temporal Logic) module
//!
//! Defines LTL formulas and generates them from DSL specs

pub mod formula;
pub mod generator;
```

**src/solver/mod.rs**:
```rust
//! Solver module
//!
//! Z3 SMT solver integration and constraint encoding

pub mod encoder;
pub mod physics;
pub mod multi_solve;
```

**src/scenario/mod.rs**:
```rust
//! Scenario module
//!
//! Output data structures and extraction from Z3 models

pub mod model;
pub mod extractor;
```

### Step 4: Create Error Types

**src/error.rs**:
```rust
//! Error types for the scenario generator

use thiserror::Error;

#[derive(Error, Debug)]
pub enum ScenarioGenError {
    #[error("Failed to parse YAML: {0}")]
    YamlParse(#[from] serde_yaml::Error),

    #[error("Failed to serialize JSON: {0}")]
    JsonSerialize(#[from] serde_json::Error),

    #[error("Z3 solver returned UNSAT - no valid scenario exists")]
    Unsatisfiable,

    #[error("Invalid DSL specification: {0}")]
    InvalidSpec(String),

    #[error("LTL formula generation failed: {0}")]
    LTLGeneration(String),

    #[error("Z3 encoding failed: {0}")]
    Z3Encoding(String),

    #[error("Scenario extraction failed: {0}")]
    ExtractionFailed(String),

    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),
}

pub type Result<T> = std::result::Result<T, ScenarioGenError>;
```

**Why thiserror**: Generates boilerplate error code, `#[from]` auto-converts errors.

### Step 5: Create Placeholder Module Files

Create empty placeholder files for later phases:

```bash
touch src/dsl/types.rs
touch src/dsl/parser.rs
touch src/ltl/formula.rs
touch src/ltl/generator.rs
touch src/solver/encoder.rs
touch src/solver/physics.rs
touch src/solver/multi_solve.rs
touch src/scenario/model.rs
touch src/scenario/extractor.rs
```

Add minimal placeholder content to each:

**src/dsl/types.rs**:
```rust
//! DSL data structures (Phase 2)
```

**src/dsl/parser.rs**:
```rust
//! YAML parser (Phase 2)
```

(Repeat for others)

### Step 6: Verify Z3 Integration

Create a simple test to ensure Z3 works:

**tests/integration_test.rs**:
```rust
//! Integration tests

use z3::*;

#[test]
fn test_z3_basic() {
    // Verify Z3 is working
    let cfg = Config::new();
    let ctx = Context::new(&cfg);
    let solver = Solver::new(&ctx);

    // Simple constraint: x > 0
    let x = ast::Int::new_const(&ctx, "x");
    let zero = ast::Int::from_i64(&ctx, 0);
    solver.assert(&x.gt(&zero));

    assert_eq!(solver.check(), SatResult::Sat);

    let model = solver.get_model().unwrap();
    let x_val = model.eval(&x, true).unwrap().as_i64().unwrap();
    assert!(x_val > 0);

    println!("Z3 found: x = {}", x_val);
}
```

**Why this test**: Verifies Z3 library is correctly linked and functional.

### Step 7: Create Example Files Directory

**examples/cut_in_left.yaml** (simple placeholder):
```yaml
# Cut-in from left scenario (to be implemented in Phase 2)
scenario_type: cut_in_left
```

### Step 8: Create README

**README.md**:
```markdown
# CARLA Scenario Generator

Generate driving test scenarios from high-level specifications using LTL + Z3.

## Status

🚧 **Under Development** - See Implementation_plan.md for roadmap

## Quick Start

```bash
# Build
cargo build

# Run tests
cargo test

# Run (when complete)
cargo run -- -i examples/cut_in_left.yaml -o output.json
```

## Documentation

- `Implementation_plan.md`: Master implementation plan
- `design_decisions.md`: Design rationale and alternatives
- `plans/`: Phase-by-phase implementation guides
```

---

## Success Criteria

### Verification Steps

1. **Project compiles**:
   ```bash
   cargo build
   ```
   Should complete without errors.

2. **Z3 test passes**:
   ```bash
   cargo test test_z3_basic
   ```
   Should print "Z3 found: x = <some positive number>"

3. **Directory structure is correct**:
   ```bash
   tree src/
   ```
   Should show dsl/, ltl/, solver/, scenario/ modules

4. **No warnings**:
   ```bash
   cargo clippy
   ```
   Should be clean (or only minor warnings)

### Checklist

- [ ] `cargo build` succeeds
- [ ] `cargo test` passes (Z3 integration test)
- [ ] All module directories exist
- [ ] Cargo.toml has all dependencies
- [ ] Error types compile
- [ ] README exists
- [ ] Example directory created

---

## Common Issues

### Issue: Z3 linking fails

**Symptom**: Error about missing Z3 library

**Solution**: Ensure `static-link-z3` feature is enabled in Cargo.toml. This embeds Z3 in the binary.

### Issue: Compilation errors in z3 crate

**Symptom**: C++ compiler errors

**Solution**: May need to install build dependencies:
```bash
# Ubuntu/Debian
sudo apt install build-essential cmake

# macOS
xcode-select --install
```

### Issue: serde derive errors

**Symptom**: Macro errors

**Solution**: Ensure `features = ["derive"]` is on serde dependency.

---

## Testing

```bash
# Run all tests
cargo test

# Run specific test
cargo test test_z3_basic

# With output
cargo test -- --nocapture

# Check code
cargo clippy

# Format code
cargo fmt
```

---

## Next Phase

Once this phase is complete and all tests pass:

**→ Continue to [Phase 2: DSL Layer](phase_02_dsl.md)**

Phase 2 will define the DSL data structures and YAML parser.

---

## Notes for AI Agents

**What you just built**:
- Rust project structure
- All necessary dependencies
- Module skeleton
- Error handling framework
- Z3 integration verified

**What you can now do**:
- Write tests for components
- Import dependencies in modules
- Build on this foundation

**What's still TODO**:
- All actual logic (coming in Phases 2-12)
- CLI implementation (Phase 12)
- Real tests (Phase 2+)

**If stuck**: Check that `cargo test` passes before proceeding to Phase 2.
