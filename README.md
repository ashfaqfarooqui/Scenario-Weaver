# CARLA Scenario Generator

Automatically generate diverse, safety-critical driving test scenarios from high-level specifications using Linear Temporal Logic (LTL) and Z3 SMT solver.

## Features

- **Declarative YAML-based scenario specifications**
- **Automatic constraint solving with Z3**
- **Built-in safety validation** (TTC, minimum distance)
- **Multiple coordinate systems** - Cartesian (x, y) and Bicycle (x, y, θ, v) models
- **Kinematic bicycle model** - Realistic vehicle dynamics with heading tracking and steering constraints
- **Multiple diverse scenario generation**
- **Adversarial scenario generation** - Generate scenarios that violate safety constraints for testing edge cases
- **Per-constraint control** - Enforce, violate, or ignore each constraint independently
- **JSON output** compatible with CARLA simulator

## Quick Start

### Installation

```bash
# Clone repository
git clone <repo-url>
cd carla-scenario-generator

# Build
cargo build --release
```

### Generate a Single Scenario

```bash
cargo run --release -- -i examples/cut_in_left.yaml -o output/
```

This will:
1. Parse the YAML specification
2. Generate LTL constraints
3. Solve with Z3
4. Output scenario files to the directory (JSON + XOSC + SVG + GIF)

### Generate Multiple Scenarios

```bash
# Generate 5 different scenarios to a directory
cargo run --release -- -i examples/cut_in_left.yaml -o scenarios/ -n 5
```

This creates `scenarios/scenario_0.json`, `scenarios/scenario_1.json`, etc.

### Enable Verbose Logging

```bash
cargo run --release -- -i examples/cut_in_left.yaml -o output/ -v
```

## Creating Custom Scenarios

This project supports two levels of customization:

### 1. Creating New Scenario Instances (YAML)

Define new test scenarios by creating YAML specification files. This requires no Rust programming.

**See**: [CREATING_SCENARIOS.md - Part 1: YAML Specification](CREATING_SCENARIOS.md#part-1-yaml-scenario-specification)

Quick example:
```yaml
scenario_type: cut_in_left
time_step: 0.1
duration: 10.0

actors:
  - id: ego
    role: ego
    lane: 1
    position: 50.0
    speed: 15.0
    direction: 1
    acceleration: [-8.0, 3.0]

  - id: npc
    role: npc
    lane: 0
    position: [60.0, 80.0]
    speed: [16.0, 20.0]
    direction: 1
    acceleration: [-8.0, 3.0]

    lane_change:
      enabled: true
      direction: right
      start_time: [2.5, 3.5]
      duration: [3.0, 4.0]

road:
  num_lanes: 2
  lane_width: 3.5
  lane_directions: [1, 1]

min_ttc: 3.0
min_distance: 5.0
```

### 2. Implementing New Scenario Types (Rust)

Extend the codebase with entirely new scenario types (e.g., roundabout, parking, intersection).

**See**: [CREATING_SCENARIOS.md - Part 2: Implementing Scenario Types](CREATING_SCENARIOS.md#part-2-implementing-new-scenario-types)

The implementation uses a trait-based plugin architecture - add a new scenario type by:
1. Adding to `ScenarioType` enum
2. Implementing `ScenarioModel` trait
3. Creating YAML examples

**For complete documentation, see [CREATING_SCENARIOS.md](CREATING_SCENARIOS.md).**

---

## Adversarial Scenario Generation (NEW!)

Generate scenarios that **intentionally violate safety constraints** to test autonomous vehicle edge cases and failure modes.

### Quick Example - CLI Override

```bash
# Generate scenarios that violate ALL safety constraints
cargo run --release -- -i examples/cut_in_left.yaml -o adversarial/ --adversarial
```

### YAML Configuration

Control which constraints to violate:

```yaml
# examples/cut_in_left_adversarial_ttc.yaml
scenario_type: cut_in_left
# ... standard config ...

min_ttc: 3.0
min_distance: 5.0

# Per-constraint modes: enforce, violate, or ignore
constraint_modes:
  min_ttc: violate       # Find TTC violations (< 3.0s)
  min_distance: enforce  # Maintain safe distance (≥ 5.0m)

num_scenarios: 10
```

**Result:** 10 scenarios where TTC is violated but distance is maintained.

### Shorthand Mode

```yaml
# Violate all constraints
constraint_modes: violate_all

# Ignore all constraints (maximum freedom)
constraint_modes: ignore_all

# Enforce all constraints (default, can be omitted)
constraint_modes: enforce_all
```

### Use Cases

- **Test emergency systems** - Validate braking/collision avoidance
- **Find edge cases** - Discover worst-case scenarios
- **ML training data** - Generate diverse datasets with violations
- **Compliance testing** - Document safety system behavior under hazards (ISO 26262)

**📖 See [README_ADVERSARIAL.md](README_ADVERSARIAL.md) for detailed documentation, architecture, and extension guide.**

---

## Coordinate Systems

The generator supports three coordinate systems for modeling vehicle dynamics:

### Cartesian (x, y) - Default

Point-mass model with separate x and y velocities. Best for general use and backward compatibility.

```yaml
coordinate_system: cartesian  # or omit (default)
```

### Bicycle Model (x, y, θ, v)

Kinematic bicycle model with heading tracking and steering constraints. Provides realistic vehicle dynamics with turn radius enforcement.

```yaml
coordinate_system: bicycle

bicycle_config:
  default_wheelbase: 2.7              # meters (typical sedan)
  default_max_steering_angle: 0.6     # radians (~34°)
  default_max_steering_rate: 0.5      # rad/s

actors:
  - id: ego
    # ... standard configuration ...
    # Uses defaults from bicycle_config

  - id: npc
    # Optional: Override defaults per actor
    bicycle_params:
      wheelbase: 2.9              # Larger vehicle (SUV)
      max_steering_angle: 0.5     # Less maneuverable
      max_steering_rate: 0.4      # Slower steering
```

**Bicycle model dynamics** (small angle approximation):
- State: (x, y, θ, v) - position, heading angle, speed
- Controls: (a, δ) - acceleration, steering angle
- Enforces steering limits, heading bounds (±30°), and minimum turn radius

**Examples:**
- `examples/bicycle_lane_change.yaml` - Highway cut-in with bicycle dynamics
- `examples/bicycle_straight.yaml` - Simple scenario with bicycle model

---

## YAML Specification Format

Create a YAML file describing your scenario:

```yaml
scenario_type: cut_in_left

# Time configuration
time_step: 0.5        # 0.5 second discretization
duration: 10.0        # 10 second scenario

# Actor specifications (generic actor system)
actors:
  # Ego vehicle (controlled by AV under test)
  - id: ego
    role: ego
    lane: 1                  # right lane
    position: [45.0, 55.0]   # 45-55 meters from start (Z3 chooses)
    speed: [14.0, 16.0]      # 14-16 m/s (Z3 chooses)
    acceleration: [-8.0, 3.0]  # -8.0 to 3.0 m/s² (braking to acceleration)

  # NPC vehicle (background actor)
  - id: npc
    role: npc
    lane: 0                  # left lane
    position: [60.0, 80.0]   # start 60-80m from start (Z3 chooses)
    speed: [12.0, 14.0]      # slightly slower (Z3 chooses)
    acceleration: [-8.0, 3.0]  # -8.0 to 3.0 m/s²
    behavior:
      cut_in_time: [2.5, 7.5]  # cut in between 2.5-7.5 seconds

# Safety constraints
min_ttc: 3.0              # minimum 3 second time-to-collision
min_distance: 5.0         # minimum 5 meter distance
lane_width: 3.5           # 3.5 meter lane width

# Generation settings
num_scenarios: 1          # generate 1 scenario (or use -n flag)
```

### Value Formats

- **Fixed values**: `position: 50.0` - Z3 must use exactly 50.0
- **Ranges**: `position: [45.0, 55.0]` - Z3 chooses any value in range
- **Behavior parameters**: Scenario-specific values in the `behavior` map (e.g., `cut_in_time`)

## Output Formats

The generator automatically produces scenarios in **four formats**: JSON, OpenSCENARIO (.xosc), SVG, and GIF.

### Quad Output

**Single scenario mode:**
```bash
cargo run --release -- -i examples/cut_in_left.yaml -o output/
# Creates: output/scenario.json + scenario.xosc + scenario.svg + scenario.gif
```

**Multiple scenario mode:**
```bash
cargo run --release -- -i examples/cut_in_left.yaml -o scenarios/ -n 5
# Creates: scenarios/scenario_0.json + scenario_0.xosc + scenario_0.svg + scenario_0.gif
#          scenarios/scenario_1.json + scenario_1.xosc + scenario_1.svg + scenario_1.gif
#          ... (5 quadruplets total)
```

### JSON Format

Complete actor trajectories with validation metrics:

```json
{
  "scenario_id": "uuid-here",
  "scenario_type": "cut_in_left",
  "time_step": 0.5,
  "duration": 10.0,
  "actors": [
    {
      "id": "ego",
      "role": "ego",
      "states": [
        {
          "time": 0.0,
          "position": { "x": 50.0, "y": 5.25 },
          "velocity": { "vx": 15.0, "vy": 0.0 },
          "lane": 1
        },
        ...
      ]
    },
    {
      "id": "npc",
      "role": "npc",
      "states": [ ... ]
    }
  ],
  "validation": {
    "min_ttc": 3.5,
    "min_distance": 8.2,
    "all_constraints_satisfied": true,
    "safety_violations": []
  }
}
```

### OpenSCENARIO Format (.xosc)

Valid OpenSCENARIO XML files for simulator compatibility:
- Standard OpenSCENARIO 1.0+ structure
- File header with scenario metadata
- Vehicle entities for all actors
- Trajectory data embedded in description field
- Compatible with CARLA and other OpenSCENARIO-compliant simulators

**Programmatic export:**
```rust
use carla_scenario_generator::{generate_single_scenario, export_scenario_to_xosc};

let yaml = std::fs::read_to_string("scenario.yaml")?;
let scenario = generate_single_scenario(&yaml)?;

// Export to XOSC
let xosc_xml = export_scenario_to_xosc(&scenario)?;
std::fs::write("scenario.xosc", xosc_xml)?;
```

### SVG Visualization Format (.svg)

Static vector graphic showing the complete scenario trajectory:
- **Road layout** with lane markings
- **Complete trajectories** for all actors from start to end
- **Vehicle positions** at initial and final states
- **Safety metrics** displayed in header (Min TTC, Min Distance, Status)
- **Violation markers** if safety constraints were violated
- **Scalable vector graphics** - perfect quality at any zoom level

Features:
- Opens in any web browser or image viewer
- Ideal for documentation and reports
- Shows the "big picture" of the scenario

**Programmatic export:**
```rust
use carla_scenario_generator::{generate_single_scenario, export_scenario_to_svg};

let yaml = std::fs::read_to_string("scenario.yaml")?;
let scenario = generate_single_scenario(&yaml)?;

// Export to SVG
let svg = export_scenario_to_svg(&scenario)?;
std::fs::write("scenario.svg", svg)?;
```

### GIF Animation Format (.gif)

Animated visualization showing vehicles moving through the scenario in real-time:
- **10 FPS animation** showing trajectory evolution over time
- **Fading trajectory trails** showing motion history
- **Real-time metrics overlay** (current time, TTC, distance, status)
- **Violation highlighting** with red circles when safety constraints are violated
- **Road surface** with lane markings
- **Vehicle rectangles** with heading arrows
- **Infinite loop** playback

Features:
- Works everywhere (browsers, Slack, GitHub, email, etc.)
- ~900KB file size for typical 10-second scenarios
- No player required - animates automatically
- Shows temporal dynamics and motion patterns
- Ideal for sharing and presentations

**Programmatic export:**
```rust
use carla_scenario_generator::{generate_single_scenario, export_scenario_to_gif};

let yaml = std::fs::read_to_string("scenario.yaml")?;
let scenario = generate_single_scenario(&yaml)?;

// Export to GIF
let gif_bytes = export_scenario_to_gif(&scenario)?;
std::fs::write("scenario.gif", gif_bytes)?;
```

## CLI Options

```bash
carla-scenario-gen [OPTIONS] --input <FILE> --output <DIR>

Options:
  -i, --input <FILE>     Input YAML specification file
  -o, --output <DIR>     Output directory for generated scenarios
  -n, --num <NUM>        Number of scenarios to generate (overrides YAML)
  -v, --verbose          Enable verbose logging
      --adversarial      Override constraint modes to violate all safety constraints
  -h, --help             Print help
  -V, --version          Print version
```

## Creating New Scenario Types

The scenario generator uses a **plugin system** that makes adding new scenario types simple. Adding a new scenario requires only **3 steps**:

### Overview

Each scenario type implements the `ScenarioModel` trait, which defines:
- **Validation**: Scenario-specific requirements (e.g., number of actors, behavior parameters)
- **Behavior LTL**: Temporal logic defining the scenario behavior
- **Safety**: Optional custom safety constraints (default: pairwise TTC/distance)
- **Z3 constraints**: Optional custom Z3 assertions (default: none)

### Step-by-Step Example

Let's add a new "lane change" scenario where an NPC changes lanes ahead of the ego vehicle.

#### Step 1: Create the Scenario Implementation

Create `src/scenarios/lane_change.rs`:

```rust
//! Lane change scenario model

use crate::scenarios::ScenarioModel;
use crate::dsl::types::ScenarioSpec;
use crate::ltl::formula::{LTLFormula, Proposition};
use anyhow::Result;

/// Lane change scenario model
pub struct LaneChangeModel;

impl ScenarioModel for LaneChangeModel {
    fn validate(&self, spec: &ScenarioSpec) -> Result<()> {
        // Require exactly 2 actors (ego + 1 npc)
        if spec.actors.len() != 2 {
            anyhow::bail!("Lane change requires exactly 2 actors, found {}", spec.actors.len());
        }

        let npc = &spec.npcs()[0];

        // Require lane_change_time parameter
        if !npc.behavior.contains_key("lane_change_time") {
            anyhow::bail!("NPC missing 'lane_change_time' in behavior map");
        }

        Ok(())
    }

    fn generate_ltl(&self, spec: &ScenarioSpec) -> Result<LTLFormula> {
        let ego = spec.ego().map_err(|e| anyhow::anyhow!(e))?;
        let npc = &spec.npcs()[0];

        let ego_id = ego.id.as_str();
        let npc_id = npc.id.as_str();

        // Initial conditions: ego and NPC start in different lanes
        let init = LTLFormula::Atom(Proposition::InLane {
            actor: ego_id.to_string(),
            lane: ego.lane,
        })
        .and(LTLFormula::Atom(Proposition::InLane {
            actor: npc_id.to_string(),
            lane: npc.lane,
        }));

        // Behavior: NPC eventually changes to ego's lane
        let behavior = LTLFormula::Atom(Proposition::InLane {
            actor: npc_id.to_string(),
            lane: ego.lane,
        })
        .eventually();

        Ok(init.and(behavior))
    }
}
```

#### Step 2: Register the Scenario Type

Add the variant to `ScenarioType` enum in `src/dsl/types.rs`:

```rust
#[derive(Debug, Clone, Copy, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum ScenarioType {
    CutInLeft,
    CutInRight,
    LaneChange,  // NEW
}
```

Update the `Display` implementation:

```rust
impl std::fmt::Display for ScenarioType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ScenarioType::CutInLeft => write!(f, "cut_in_left"),
            ScenarioType::CutInRight => write!(f, "cut_in_right"),
            ScenarioType::LaneChange => write!(f, "lane_change"),  // NEW
        }
    }
}
```

Add the dispatch logic in `get_model()`:

```rust
pub fn get_model(&self) -> Box<dyn crate::scenarios::ScenarioModel> {
    match self {
        ScenarioType::CutInLeft => Box::new(crate::scenarios::cut_in_left::CutInLeftModel),
        ScenarioType::CutInRight => Box::new(crate::scenarios::cut_in_right::CutInRightModel),
        ScenarioType::LaneChange => Box::new(crate::scenarios::lane_change::LaneChangeModel),  // NEW
    }
}
```

#### Step 3: Export the Module

Add to `src/scenarios/mod.rs`:

```rust
pub mod cut_in_left;
pub mod cut_in_right;
pub mod lane_change;  // NEW
```

That's it! Your new scenario type is now available.

### Create Example YAML

Create `examples/lane_change.yaml`:

```yaml
scenario_type: lane_change

time_step: 0.5
duration: 10.0

actors:
  - id: ego
    role: ego
    lane: 0
    position: 50.0
    speed: 15.0
    acceleration: [-8.0, 3.0]

  - id: npc
    role: npc
    lane: 1
    position: [60.0, 80.0]
    speed: [12.0, 14.0]
    acceleration: [-8.0, 3.0]
    behavior:
      lane_change_time: [3.0, 7.0]  # Lane change between 3-7 seconds

min_ttc: 3.0
min_distance: 5.0
lane_width: 3.5
num_scenarios: 5
```

### Test Your New Scenario

```bash
# Generate scenarios
cargo run --release -- -i examples/lane_change.yaml -o scenarios/ -n 5

# Run tests
cargo test lane_change
```

### Advanced Features

#### Custom Safety Constraints

Override `generate_safety()` to customize safety behavior:

```rust
impl ScenarioModel for LaneChangeModel {
    // ... other methods ...

    fn generate_safety(&self, spec: &ScenarioSpec) -> Result<LTLFormula> {
        // Custom safety logic
        // For example: only enforce distance, ignore TTC
        Ok(custom_distance_constraint(spec))
    }
}
```

#### Custom Z3 Constraints

Override `add_z3_constraints()` to add scenario-specific Z3 assertions:

```rust
fn add_z3_constraints(
    &self,
    spec: &ScenarioSpec,
    encoder: &crate::solver::Z3Encoder,
    solver: &z3::Solver,
    horizon: usize,
) -> Result<()> {
    // Add custom Z3 constraints
    // For example: constrain the lane change timing
    Ok(())
}
```

### Examples

See existing implementations for reference:
- **Cut-in left**: `src/scenarios/cut_in_left.rs` - NPC cuts in from left lane
- **Cut-in right**: `src/scenarios/cut_in_right.rs` - NPC cuts in from right lane

### Multi-Actor Support

The system supports **1 ego + N NPCs**. Access actors in your implementation:

```rust
let ego = spec.ego()?;  // Get the single ego actor
let npcs = spec.npcs(); // Get all NPCs (returns Vec<&ActorSpec>)

// Access specific actor by ID
let actor = spec.get_actor("npc1")?;
```

Pairwise safety constraints are automatically generated for all actor combinations unless you override `generate_safety()`.

## Development

### Run Tests

```bash
# All tests
cargo test

# Unit tests only
cargo test --lib

# Integration tests only
cargo test --test integration_test

# Specific test
cargo test test_generate_single_scenario
```

### Code Quality

```bash
# Check for warnings
cargo clippy

# Format code
cargo fmt

# Build documentation
cargo doc --open
```

## Architecture

The generator uses a **modular, trait-based plugin system** for both scenario types and coordinate systems:

### Pipeline

1. **DSL Parser** (`src/dsl/`) - Parse YAML into structured specification
2. **Scenario Model** (`src/scenarios/`) - Trait-based scenario implementations
3. **LTL Generator** (`src/ltl/`) - Convert specification to temporal logic constraints
4. **Z3 Encoder** (`src/solver/`) - Coordinate-specific encoders via trait objects
   - `GenericEncoder` facade dispatches to `CartesianEncoder` or `BicycleEncoder`
   - Coordinate-specific logic in `src/solver/encoders/cartesian.rs` and `bicycle.rs`
5. **Scenario Extractor** (`src/scenario/`) - Extract solution as JSON trajectories

### Encoder Architecture

The encoder system uses a **trait-based plugin architecture**:

- **`CoordinateEncoder<B>` trait**: Defines interface for coordinate-specific encoders
  - Variable creation (`create_variables`)
  - Kinematics encoding (`encode_kinematics`)
  - Constraint encoding (`encode_velocity_constraints`, `encode_acceleration_constraints`, etc.)
  - Variable accessors (`get_longitudinal_pos`, `get_lateral_pos`, `get_longitudinal_vel`, etc.)
  - Trajectory extraction (`extract_actor_trajectory`)

- **`GenericEncoder<B>`**: Thin facade that coordinates encoding
  - Holds `Box<dyn CoordinateEncoder<B>>` trait object
  - Dispatches to appropriate encoder based on `scenario.coordinate_system`
  - Maintains coordinate-agnostic logic (LTL encoding, validation metrics)

- **Coordinate-specific implementations**:
  - `CartesianEncoder<B>`: (x, y) coordinate system with smooth lane transition interpolation
    - Linear interpolation of lateral position during lane changes
    - Lateral acceleration bounded to 2.0 m/s² during transitions
    - Lateral velocity bounded to 2.0 m/s (realistic for vehicles)
  - `BicycleEncoder<B>`: (x, y, θ, v) kinematic bicycle model with heading tracking
    - Steering angle bounds, heading angle bounds (±30°), steering rate limits
    - Turn radius enforcement based on wheelbase and max steering angle
    - Small angle approximation for efficient solving

**Benefits:**
- Clean separation of coordinate system logic
- Easy to add new coordinate systems (just implement the trait)
- Type-safe dispatch at construction time
- Backward compatible with existing scenario code

### Key Design Features

- **Generic actor system**: Supports 1 ego + N NPCs with dynamic constraints
- **Trait-based plugins**: Each scenario type implements `ScenarioModel` trait
- **Coordinate system plugins**: Each coordinate system implements `CoordinateEncoder` trait
- **Type-safe dispatch**: Enum-based scenario types and coordinate systems (no string registry)
- **Automatic safety**: Pairwise TTC and distance constraints by default
- **Multi-scenario diversity**: Blocking clauses force diverse solutions
- **Constraint modes**: Enforce, violate, or ignore each safety constraint independently

### ScenarioModel Trait

All scenario types implement the `ScenarioModel` trait:

```rust
pub trait ScenarioModel: Send + Sync {
    // Validate scenario-specific requirements
    fn validate(&self, spec: &ScenarioSpec) -> Result<()>;

    // Generate behavioral LTL (required)
    fn generate_ltl(&self, spec: &ScenarioSpec) -> Result<LTLFormula>;

    // Generate safety constraints (optional, has default)
    fn generate_safety(&self, spec: &ScenarioSpec) -> Result<LTLFormula> {
        Ok(generate_default_safety(spec))
    }

    // Add Z3 constraints (optional, has default)
    fn add_z3_constraints(...) -> Result<()> { Ok(()) }
}
```

## Documentation

- **[README_ADVERSARIAL.md](README_ADVERSARIAL.md)**: Complete guide to adversarial scenario generation (uses, architecture, extensions)
- `Implementation_plan.md`: Master implementation plan
- `design_decisions.md`: Design rationale and alternatives
- `plans/`: Phase-by-phase implementation guides
- `QUICK_START.md`: Getting started guide

## Requirements

- Rust 1.70+
- Z3 SMT solver (installed via cargo)

## License

See LICENSE file
