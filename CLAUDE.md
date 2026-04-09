# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ScenarioWeaver: Automatically generate diverse, safety-critical driving test scenarios from YAML specifications using Linear Temporal Logic (LTL) and Z3 SMT solver. Supports both safe scenario generation and adversarial generation (scenarios that intentionally violate safety constraints for testing edge cases).

### Road Support

The generator currently supports **single-road scenarios** with the following features:

- **RoadSpec**: Single road with configurable lanes, width, and directions
- **Bidirectional lanes**: Each lane can have forward (+1) or backward (-1) direction
- **Lane Direction Constraints**: Automatic velocity constraints based on lane direction (forward lanes require vx >= 0, backward lanes require vx <= 0)
- **Dynamic road length**: Auto-calculated based on scenario duration if not specified

Example single-road specification:

```yaml
road:
  num_lanes: 4
  lane_width: 3.5
  lane_directions: [1, 1, -1, -1]  # 2 forward, 2 backward
  road_length: 400.0  # Optional - auto-calculated if omitted
```

**Note**: Multi-road networks with junctions are planned but not yet implemented. Future support would include:
- Multiple named roads with world positions and headings
- Road connections and junctions (T-junctions, crossroads)
- OpenDRIVE (.xodr) export for complex road networks

## Common Commands

### Build & Run

```bash
# Build project
cargo build --release

# Generate single scenario (creates output/scenario.json, .xosc, .svg, .gif)
cargo run --release -- -i examples/cut_in_left.yaml -o output/

# Generate multiple scenarios (creates quintuplets for each)
cargo run --release -- -i examples/cut_in_left.yaml -o scenarios/ -n 5

# Generate adversarial scenarios (violate safety constraints)
cargo run --release -- -i examples/cut_in_left.yaml -o adversarial/ --adversarial

# Generate scenarios using Bicycle model (kinematic with heading tracking)
cargo run --release -- -i examples/bicycle_lane_change.yaml -o bicycle_output/

# Enable verbose logging
cargo run --release -- -i examples/cut_in_left.yaml -o output/ -v
```

### Testing

```bash
# All tests
cargo test

# Unit tests only
cargo test --lib

# Integration tests only
cargo test --test integration_test

# Specific test
cargo test test_generate_single_scenario

# Run tests with output
cargo test -- --nocapture
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

## Architecture Overview

The generator uses a **pipeline architecture** that transforms YAML specifications into concrete JSON scenarios:

```
YAML Input → DSL Parser → LTL Generator → Z3 Encoder → Z3 Solver → Scenario Extractor → JSON Output
```

### Core Modules

1. **DSL Module** (`src/dsl/`)
   - **Purpose**: Parse and validate YAML specifications
   - **Key files**:
     - `types.rs`: Core data structures (`ScenarioSpec`, `ActorSpec`, `RoadSpec`, `ConstraintModes`)
     - `parser.rs`: YAML parsing logic
   - **Important**: Supports both fixed values and ranges (e.g., `position: 50.0` or `position: [45.0, 55.0]`)
   - **Constraint modes**: `Enforce` (default safe), `Violate` (adversarial), `Ignore` (unconstrained)
   - **Road Support**: Single `RoadSpec` with lanes, width, directions, and optional length

2. **LTL Module** (`src/ltl/`)
   - **Purpose**: Convert high-level specifications to temporal logic formulas
   - **Key files**:
     - `formula.rs`: LTL formula AST (`Always`, `Eventually`, `And`, `Or`, `Proposition`)
     - `generator.rs`: Generates LTL from `ScenarioSpec`
   - **Key insight**:
     - `Enforce` mode → `G(constraint)` (Always safe)
     - `Violate` mode → `F(NOT constraint)` (Eventually violates)
     - `Ignore` mode → constraint omitted

3. **Solver Module** (`src/solver/`)
   - **Purpose**: Encode LTL + physics into Z3 SMT constraints using coordinate-specific encoders
   - **Key files**:
     - `encoder.rs`: GenericEncoder facade that dispatches to coordinate-specific encoders
     - `coordinate_encoder.rs`: CoordinateEncoder trait defining encoder interface
     - `encoders/cartesian.rs`: CartesianEncoder for (x, y) coordinate system
     - `encoders/bicycle.rs`: BicycleEncoder for (x, y, θ, v) kinematic bicycle model
     - `physics.rs`: Kinematic constraint helpers
     - `multi_solve.rs`: Generate multiple diverse scenarios using blocking clauses
   - **Encoder Architecture**: Trait-based plugin system
     - `CoordinateEncoder<B>` trait defines interface for all coordinate-specific encoders
     - `GenericEncoder<B>` holds `Box<dyn CoordinateEncoder<B>>` and dispatches calls
     - Coordinate system selected via `scenario.coordinate_system` in YAML
   - **Two-layer encoding**:
     - Layer 1: LTL temporal operators expanded over time steps (all modes)
     - Layer 2: Direct Z3 assertions (only for `Enforce` mode, skipped for `Violate`/`Ignore`)
   - **Variables**: Position, velocity, and lane variables for each actor at each time step
     - Cartesian: `positions_x`, `positions_y`, `velocities_x`, `velocities_y`, `lanes`
     - Bicycle: `positions_x`, `positions_y`, `heading_theta`, `speed_v`, `steering_delta`, `accelerations`, `lanes`
   - **Velocity Direction Constraints**: `encode_lane_velocity_constraints()` enforces lane direction (forward: longitudinal vel >= 0, backward: <= 0)

4. **Scenario Module** (`src/scenario/`)
   - **Purpose**: Extract solutions from Z3 models and validate
   - **Key files**:
     - `model.rs`: Output data structures (`Scenario`, `ActorTrajectory`, `ValidationInfo`)
     - `extractor.rs`: Extract trajectories from Z3 model, compute metrics
   - **Validation**: Computes min TTC, min distance, detects violations with timestamps

### Data Flow Example (Cut-in Scenario)

1. **Input YAML**: Ego in lane 1 at 50m/15m/s, NPC in lane 0, smooth lane change between 2.5-3.5s over 3-4s, min_ttc=3.0s
2. **DSL Parser**: Creates `ScenarioSpec` with `lane_change` configuration for NPC
3. **LTL Generator**: Creates formula like `G(lane_0) UNTIL cut_in_time AND G(ttc > 3.0)`
4. **Z3 Encoder**:
   - Creates variables for time steps (e.g., 100 steps for 10s / 0.1s)
   - Encodes kinematics: `px[t+1] = px[t] + vx[t] * dt`
   - **Cartesian mode**: Encodes smooth lane transition with linear interpolation of lateral position
     - Before lane change: `py = lane * lane_width + lane_width/2`
     - During transition: `py = source + progress * (target - source)`
     - After lane change: `py = new_lane * lane_width + lane_width/2`
   - Encodes LTL: `G(ttc > 3.0)` → `ttc[0] > 3.0 ∧ ttc[1] > 3.0 ∧ ... ∧ ttc[100] > 3.0`
5. **Z3 Solver**: Finds values for all variables satisfying constraints
6. **Extractor**: Pulls position/velocity at each time step, validates safety, outputs JSON

### Lane Change Configuration

Actors with lane changes must specify the `lane_changes` configuration in the YAML. Multiple sequential lane changes are supported, enabling complex maneuvers like overtaking.

**Single lane change (cut-in):**
```yaml
actors:
  - id: npc
    role: npc
    lane: 0
    position: [20.0, 80.0]
    speed: [16.0, 20.0]
    direction: 1
    acceleration: [-8.0, 3.0]

    lane_changes:
      - direction: right         # Lane change direction (left or right)
        start_time: [2.5, 3.5]  # When to start the lane change
        duration: [3.0, 4.0]    # How long the transition takes
```

**Multiple lane changes (overtake):**
```yaml
actors:
  - id: npc
    role: npc
    lane: 1                    # Same lane as ego (starts behind)
    position: [25.0, 35.0]
    speed: [18.0, 22.0]
    direction: 1
    acceleration: [-3.0, 4.0]

    # Two sequential lane changes for overtake maneuver
    lane_changes:
      # First: move left into passing lane
      - direction: left
        start_time: [2.0, 3.0]
        duration: [1.5, 2.0]
      # Second: move right back to original lane
      - direction: right
        start_time: [7.0, 8.0]
        duration: [1.5, 2.0]
```

**Key behaviors:**
- **Empty vec**: No lane changes (actor stays in initial lane)
- **Presence in vec**: Lane change is enabled (no `enabled` field needed)
- **Multiple entries**: Sequential lane changes processed in order
- **Validation**: Lane changes must not overlap (one must end before next starts)
- **Cartesian system**: Lateral position linearly interpolates between lane centers over the duration
  - Lateral velocity: ~1.2 m/s for 3.5m lane change over 3s (realistic)
  - Lateral acceleration bounded to 2.0 m/s²

#### Physics Constraints (Cartesian)

The Cartesian encoder enforces realistic vehicle physics during lane changes to prevent physically impossible sideways-only motion:

**Velocity Ratio Constraint**: `|vy| <= k * |vx|` where k = 0.15
- Ensures lateral velocity is always much smaller than forward velocity
- Prevents sideways-only motion (vehicles can't slide like a hockey puck)
- Corresponds to max heading angle of ~8.5° (comfortable highway driving)
- Applied at every time step during lane change period
- Non-linear constraint (uses Z3's NRA - non-linear real arithmetic)

**Implementation** (`src/solver/encoders/cartesian.rs:281-310`):
```rust
// During lane change: start_step..=end_step
let k = Real::from_rational(15_i64, 100_i64);  // 0.15 ratio
for t in start_step..=end_step {
    let vx_t = &self.velocities_x[actor_id][t];
    let vy_t = &self.velocities_y[actor_id][t];

    // Compute |vx| * k (handle forward/backward lanes)
    let abs_vx = if actor.direction == 1 { vx_t } else { -vx_t };
    let max_vy = &abs_vx * &k;

    // Enforce: -max_vy <= vy <= max_vy
    self.backend.assert(&vy_t.ge(&-&max_vy));
    self.backend.assert(&vy_t.le(&max_vy));
}
```

**Implications**:
- Vehicles must maintain sufficient forward speed during lane changes
- Very low speeds + short durations may result in UNSAT (scenario impossible)
- Example: 8 m/s forward speed allows max 1.2 m/s lateral velocity
- For 3.5m lane change at 8 m/s, minimum duration: ~3 seconds
- At 20 m/s forward speed: max lateral velocity = 3.0 m/s (allows ~1.2s lane change)

**Performance**:
- Adds non-linear constraints but Z3 handles them efficiently
- Typical generation time: 1-2 seconds (no significant slowdown)
- Tested with highway speeds (15-25 m/s) and typical lane change durations (3-4s)

**Testing**:
- `tests/cartesian_physics_test.rs` verifies velocity ratios in all scenarios
- All generated scenarios maintain heading angles < 8.5°
- No sideways-only motion detected in any test case

### Bicycle Model Configuration

Actors using the bicycle coordinate system model vehicle dynamics with heading tracking and steering constraints.

**YAML configuration** for bicycle scenarios:
```yaml
# Select bicycle coordinate system
coordinate_system: bicycle

# Scenario-level defaults for all actors
bicycle_config:
  default_wheelbase: 2.7              # meters (typical sedan)
  default_max_steering_angle: 0.6     # radians (~34°)
  default_max_steering_rate: 0.5      # rad/s

actors:
  - id: ego
    role: ego
    lane: 1
    position: 50.0
    speed: 15.0
    acceleration: [-8.0, 3.0]
    direction: 1
    # Uses defaults from bicycle_config

  - id: npc
    role: npc
    lane: 0
    position: [60.0, 80.0]
    speed: [16.0, 20.0]
    acceleration: [-8.0, 3.0]
    direction: 1
    # Optional: Override default bicycle params for this actor
    bicycle_params:
      wheelbase: 2.9              # Larger vehicle (SUV)
      max_steering_angle: 0.5     # Less maneuverable
      max_steering_rate: 0.4      # Slower steering

    lane_change:
      enabled: true
      direction: right
      start_time: [2.5, 3.5]
      duration: [3.0, 4.0]
```

**Kinematic bicycle model dynamics** (small angle approximation):
- State: `(x, y, θ, v)` where θ is heading angle, v is speed
- Controls: `(a, δ)` where a is longitudinal acceleration, δ is steering angle
- Equations:
  ```
  dx/dt = v * cos(θ) ≈ v        (small angle: cos(θ) ≈ 1)
  dy/dt = v * sin(θ) ≈ v * θ    (small angle: sin(θ) ≈ θ)
  dθ/dt = (v/L) * tan(δ) ≈ (v/L) * δ    (small angle: tan(δ) ≈ δ)
  dv/dt = a
  ```

**Constraints enforced**:
- Steering angle bounds: `-δ_max ≤ δ ≤ δ_max`
- Heading angle bounds: `-π/6 ≤ θ ≤ π/6` (±30° for small angle validity)
- Steering rate: `|δ[t+1] - δ[t]| ≤ max_steering_rate * dt`
- Speed: `v ≥ 0` (always positive)
- Turn radius: `R_min = L / δ_max` (e.g., 2.7m / 0.6 rad ≈ 4.5m)

**Validation**:
- If `coordinate_system: bicycle` but no `bicycle_params` (neither default nor per-actor) → error
- If `wheelbase <= 0` or `max_steering_angle <= 0` → error
- Pedestrians in bicycle scenarios use simplified model (no steering)

**Trajectory output**:
- Extracts (x, y, θ, v, δ) from Z3 model
- Converts to Cartesian velocities: `vx ≈ v`, `vy ≈ v * θ` (small angle)
- JSON output format unchanged (still has `position: {x, y}`, `velocity: {vx, vy}`)

**Use cases**:
- Realistic vehicle dynamics with heading tracking
- Scenarios requiring minimum turn radius constraints
- Testing with physical steering angle limitations
- Highway lane changes with smooth heading transitions

**Limitations**:
- Valid for |θ| < 30° (small angle approximation)
- May return UNSAT if lane change duration too short for turn radius
- Lateral acceleration in output set to 0 (could be computed from v²*θ/L if needed)

See `examples/bicycle_lane_change.yaml` and `examples/bicycle_straight.yaml` for complete examples.

## Key Design Patterns

### Adversarial Generation

- **Motivation**: Test autonomous vehicles against edge cases and safety violations
- **Implementation**: Three constraint modes per constraint (TTC, distance)
  - `Enforce`: Standard safe generation (default)
  - `Violate`: Find scenarios violating this constraint
  - `Ignore`: Omit constraint entirely
- **Usage in YAML**:

  ```yaml
  constraint_modes:
    min_ttc: violate       # Find TTC violations
    min_distance: enforce  # But maintain safe distance
  ```

- **CLI shortcut**: `--adversarial` flag overrides all to `violate_all`

### Multi-Scenario Generation

- **Technique**: Blocking clauses to force diversity
- **Implementation** (`solver/multi_solve.rs`):
  1. Generate first scenario
  2. Add blocking clause excluding previous solution
  3. Solve again for different scenario
  4. Repeat for N scenarios
- **Diversity**: Focuses on NPC position and cut-in time as distinguishing features

### Value Ranges in DSL

- **Supports both**: `position: 50.0` (fixed) or `position: [45.0, 55.0]` (range)
- **Implementation**: `ValueOrRange<f64>` enum in `dsl/types.rs`
- **Z3 encoding**: Fixed values → equality constraint, ranges → inequality constraints

### Encoder Architecture

The encoder system uses a **trait-based plugin architecture** to support multiple coordinate systems:

- **`CoordinateEncoder<B>` trait** (`src/solver/coordinate_encoder.rs`)
  - Defines interface for all coordinate-specific encoders
  - Key methods:
    - `create_variables()`: Create Z3 variables for all actors at all time steps
    - `encode_kinematics()`: Encode motion equations (position/velocity updates)
    - `encode_initial_conditions()`: Encode starting positions and velocities
    - `encode_velocity_constraints()`: Enforce velocity bounds
    - `encode_acceleration_constraints()`: Enforce acceleration bounds
    - `get_longitudinal_pos()`: Get position variable for actor at time step
    - `get_lateral_pos()`: Get lateral position variable
    - `get_longitudinal_vel()`: Get velocity variable
    - `get_lateral_vel()`: Get lateral velocity variable
    - `get_lane_var()`: Get lane assignment variable
    - `extract_actor_trajectory()`: Extract trajectory from Z3 model

- **`GenericEncoder<B>`** (`src/solver/encoder.rs`)
  - Thin facade that coordinates encoding process
  - Constructor chooses encoder based on `spec.coordinate_system`
  - Holds `Box<dyn CoordinateEncoder<B>>` trait object
  - Coordinate-agnostic methods:
    - `encode_ltl()`: Encode LTL formulas (works for all coordinate systems)
    - `encode_safety()`: Encode safety constraints (TTC, distance)
    - `extract_scenario()`: Extract complete scenario from Z3 model
    - `compute_validation_metrics()`: Compute TTC, distance metrics

- **Coordinate-specific implementations** (`src/solver/encoders/`):
  - `cartesian.rs`: CartesianEncoder for (x, y) coordinates
    - Variables: `positions_x`, `positions_y`, `velocities_x`, `velocities_y`, `lanes`
    - Lane coupling: `py = lane * lane_width + lane_width/2`
    - Use case: Unstructured environments, backward compatibility
  - `bicycle.rs`: BicycleEncoder for (x, y, θ, v) kinematic bicycle model
    - Variables: `positions_x`, `positions_y`, `heading_theta`, `speed_v`, `steering_delta`, `accelerations`, `lanes`
    - Dynamics (small angle approximation): dx/dt = v, dy/dt = v*θ, dθ/dt = (v/L)*δ, dv/dt = a
    - Constraints: Steering angle bounds, heading angle bounds (±30°), steering rate limits, turn radius enforcement
    - Use case: Realistic vehicle dynamics with heading tracking and steering constraints

**Benefits:**
- Clean separation of coordinate system logic
- Easy to add new coordinate systems (just implement the trait)
- Type-safe dispatch at construction time
- Backward compatible with existing scenario code

**Working with Encoder Accessor Methods:**

When working with Z3 variables in LTL or constraints, always use accessor methods:

```rust
// CORRECT: Use accessor methods
let px = encoder.get_longitudinal_pos("actor_id", time);
let py = encoder.get_lateral_pos("actor_id", time);
let lane = encoder.get_lane_var("actor_id", time);

// WRONG: Direct field access (no longer exists)
let px = &encoder.positions_x["actor_id"][time];  // ERROR!
```

**Adding a New Coordinate System:**

To add a new coordinate system:

1. Create new file: `src/solver/encoders/mycoords.rs`
2. Implement `CoordinateEncoder<B>` trait for `MyCoordsEncoder<B>`
3. Add variant to `CoordinateSystem` enum in `src/dsl/types.rs` (if not already present)
4. Update `GenericEncoder::with_backend` in `src/solver/encoder.rs` to dispatch to your encoder
5. Add tests to verify encoder works correctly
6. Update documentation (this file and README.md)

**Coordinate System Selection:**

The coordinate system is selected in the YAML specification:

```yaml
coordinate_system: cartesian  # or bicycle
```

This determines which encoder is instantiated by `GenericEncoder::with_backend`.

### Available Propositions

The system supports the following atomic propositions for expressing scenario constraints:

#### Vehicle Positioning (4)
- `InLane { actor, lane }` - Actor in specific lane
- `Ahead { actor1, actor2 }` - Longitudinal ordering (px1 > px2)
- `DistanceGT { actor1, actor2, distance }` - Longitudinal distance > threshold
- `TTCGT { actor1, actor2, ttc }` - Time-to-collision > threshold (same-lane only)

#### Velocity Constraints (2)
- `VelocityGT { actor, velocity }` - Actor's longitudinal speed exceeds threshold (linear: |vx| > velocity)
  - Use case: Speed limit violations (violate mode), highway merging (enforce minimum)
  - Uses longitudinal velocity magnitude, matching YAML "speed" semantics
  - Z3 complexity: Low (linear constraint)
- `VelocityLT { actor, velocity }` - Actor's longitudinal speed below threshold (linear: |vx| < velocity)
  - Use case: School zones (enforce max speed), parking scenarios
  - Uses longitudinal velocity magnitude, matching YAML "speed" semantics
  - Z3 complexity: Low (linear constraint)

#### Lateral Positioning (3)
- `LateralDistanceGT { actor1, actor2, distance }` - Perpendicular distance > threshold
  - Linear constraint: |py1 - py2| > distance
  - Use case: Multi-lane safety (side-by-side clearance), parallel parking
  - Z3 complexity: Low
- `OnLeftOf { actor1, actor2 }` - Actor1 laterally left of Actor2 (py1 > py2)
  - Use case: Lane discipline, passing side specification
  - Z3 complexity: Very Low
- `OnRightOf { actor1, actor2 }` - Actor1 laterally right of Actor2 (py1 < py2)
  - Use case: Lane discipline, lateral ordering
  - Z3 complexity: Very Low

#### Relative Velocity (1)
- `RelativeVelocityGT { actor1, actor2, velocity }` - Speed difference exceeds threshold
  - Linear constraint: |vx1 - vx2| > velocity
  - Use case: Unsafe following (too fast relative to leader), overtaking constraints
  - Z3 complexity: Low

#### Pedestrian-Specific (6)
- `OnSidewalk { actor, side }` - Pedestrian on sidewalk (left/right)
- `CrossingRoad { actor }` - Pedestrian on road surface
- `Distance2DGT { actor1, actor2, distance }` - Euclidean distance (slow - quadratic constraints)
- `ManhattanDistanceGT { actor1, actor2, distance }` - Manhattan distance (faster - linear)
- `RectangularDistanceGT { actor1, actor2, threshold_x, threshold_y }` - Rectangular safety box (fastest)
- `PedestrianTTCGT { ego, pedestrian, ttc }` - Perpendicular crossing TTC

### New YAML Configuration Options

The following new constraint modes and thresholds are available:

```yaml
# Velocity constraints (optional)
max_velocity: 22.0        # Maximum speed in m/s (enforce: all vehicles under limit)
min_velocity: 10.0        # Minimum speed in m/s (enforce: all vehicles above limit)

# Lateral distance constraint (optional)
min_lateral_distance: 2.5 # Minimum lateral separation in meters

# Relative velocity constraint (optional)
max_relative_velocity: 10.0 # Maximum speed difference in m/s

# Constraint modes
constraint_modes:
  min_ttc: enforce
  min_distance: enforce
  max_velocity: enforce      # NEW: Control speed limit enforcement
  min_velocity: ignore       # NEW: Control minimum speed (default: ignore)
  min_lateral_distance: enforce  # NEW: Control lateral distance (default: ignore)
  max_relative_velocity: enforce  # NEW: Control relative velocity (default: ignore)
  max_acceleration: enforce
```

### Example: Speed Limit Violation Scenario

```yaml
scenario_type: cut_in_left
time_step: 0.5
duration: 10.0
num_scenarios: 1

actors:
  - id: ego
    role: ego
    lane: 1
    position: [0.0, 20.0]
    speed: [25.0, 30.0]  # Speeding - above the 22 m/s limit
    acceleration: [-5.0, 3.0]

  - id: npc
    role: npc
    lane: 0
    position: [40.0, 60.0]
    speed: [15.0, 20.0]  # Within limit
    acceleration: [-4.0, 2.0]
    behavior:
      cut_in_time: [3.0, 6.0]

road:
  num_lanes: 2
  lane_width: 3.5
  lane_directions: [1, 1]

min_ttc: 3.0
min_distance: 5.0

# Speed limit: 22 m/s (approximately 50 mph)
max_velocity: 22.0

# Constraint modes: violate max_velocity (adversarial - find speeding scenario)
# while maintaining safety (enforce TTC and distance)
constraint_modes:
  min_ttc: enforce         # Must maintain safe TTC
  min_distance: enforce    # Must maintain safe distance
  max_velocity: violate    # MUST violate speed limit (adversarial)
```

See `examples/speed_limit_violation.yaml`, `examples/school_zone.yaml`, `examples/multi_lane_safety.yaml`, and `examples/unsafe_following.yaml` for complete examples.

## Important Implementation Notes

### When Adding New Constraints

Follow this sequence (see README_ADVERSARIAL.md §"Extending the System" for details):

1. Add to `ConstraintModes::Detailed` in `dsl/types.rs`
2. Add getter method to `ConstraintModes`
3. Add threshold field to `ScenarioSpec`
4. Add proposition variant to `Proposition` enum in `ltl/formula.rs`
5. Update `LTLGenerator::safety_constraints()` in `ltl/generator.rs` to handle all three modes
6. Add Z3 encoding to all coordinate encoders:
   - `src/solver/encoders/cartesian.rs`: Implement in `encode_proposition()` and add direct assertions in `encode_safety()` (only for `Enforce` mode)
   - `src/solver/encoders/bicycle.rs`: Implement in `encode_proposition()` and add direct assertions in `encode_safety()` (only for `Enforce` mode)
   - Use accessor methods like `get_longitudinal_pos()`, `get_lateral_pos()` instead of direct field access
7. Update YAML example files
8. Add tests (unit tests for each encoder, integration tests for full pipeline)

### Z3 Encoding Best Practices

- **Use rational arithmetic**: `Real::from_real(ctx, (value * 10.0) as i32, 10)` for decimals
- **Time-bound everything**: Variables indexed by time step `0..=horizon`
- **Avoid conflicts**: Don't add direct assertions for `Violate`/`Ignore` modes (LTL encoding handles it)
- **Performance**: Coarser time steps and shorter durations solve faster

### Testing Considerations

- **UNSAT scenarios**: If Z3 returns UNSAT, constraints conflict (e.g., impossible to violate TTC with given parameters)
- **Fixture files**: Use `tests/fixtures/` for YAML test files
- **Integration tests**: Test full pipeline from YAML → JSON in `tests/integration_test.rs`

## Scenario Types

Currently supports:

- **cut_in_left**: NPC starts in left lane, cuts into ego's lane

Future extensions would add new scenario types by:

1. Adding variant to `ScenarioType` enum
2. Implementing type-specific LTL in `LTLGenerator`
3. Adding examples

## Output Formats

The generator produces scenarios in **four formats** automatically:

### JSON Output

- **Metadata**: scenario_id (UUID), type, duration, time_step
- **Actor trajectories**: Position, velocity, lane at each time step
- **Validation metrics**: min_ttc, min_distance, all_constraints_satisfied
- **Safety violations**: List of violations with timestamps (for adversarial scenarios)

### OpenSCENARIO (.xosc) Output

- **Format**: Valid OpenSCENARIO 1.0+ XML
- **Module**: `src/scenario/xosc_exporter.rs`
- **Functions**:
  - `export_to_xosc(scenario: &Scenario) -> Result<String>`
  - `export_to_xosc_with_road_file(scenario, xodr_path) -> Result<String>`
- **Structure**:
  - FileHeader with scenario metadata and author "ScenarioWeaver"
  - RoadNetwork section (optional reference to OpenDRIVE file)
  - Entities section with vehicle definitions
  - Storyboard with trajectory-based actions for each actor
  - StopTrigger based on scenario duration
- **Usage**: Automatically generated; also available via public API
- **Note**: Pedestrians are exported as vehicles due to openscenario-rs library limitations

### SVG Visualization (.svg) Output

- **Format**: Static vector graphic visualization
- **Module**: `src/scenario/svg_visualizer.rs`
- **Function**: `export_to_svg(scenario: &Scenario) -> Result<String>`
- **Structure**:
  - Single-road surface with lane markings
  - Complete vehicle trajectories from start to end
  - Vehicle markers at initial and final positions
  - Metrics bar with safety information (TTC, distance, status)
  - Legend with color key
  - Violation markers if constraints violated
- **Usage**: Automatically generated alongside JSON/XOSC/GIF; also available via public API `export_scenario_to_svg()`

### GIF Animation (.gif) Output

- **Format**: Animated GIF showing trajectory evolution
- **Module**: `src/scenario/gif_animator.rs`
- **Function**: `export_to_gif(scenario: &Scenario) -> Result<Vec<u8>>`
- **Features**:
  - 10 FPS animation showing vehicles moving through scenario
  - Fading trajectory trails showing motion history
  - Real-time metrics overlay (current time, TTC, distance, status)
  - Violation highlighting with red circles
  - Road surface with lane markings
  - Vehicle rectangles with heading arrows
  - Infinite loop playback
- **Implementation**:
  - Uses `image`, `gif`, `imageproc`, and `ab_glyph` crates
  - Generates one frame per time step (discrete state visualization)
  - Frame delay: 100ms (10 FPS)
  - Font: Embedded DejaVu Sans TTF in `assets/DejaVuSans.ttf`
  - Coordinate transformation reused from SVG visualizer
  - Per-frame metrics computation for real-time overlay
- **File Size**: ~900KB for typical 10-second scenario (101 frames)
- **Usage**: Automatically generated alongside JSON/XOSC/SVG; also available via public API `export_scenario_to_gif()`

### Planned: OpenDRIVE (.xodr) Export

OpenDRIVE export for road networks is planned but not yet implemented. Future support would include:
- Road definitions with geometry
- Lane sections with proper structure
- Support for multi-road networks and junctions

### Visualization Export Implementation Details

**OpenSCENARIO (xosc_exporter.rs):**

- Creates complete OpenSCENARIO structure using openscenario-rs builder
- Trajectory-based actions for each actor
- Optional reference to OpenDRIVE road file via RoadNetwork section
- Key functions: `export_to_xosc()`, `export_to_xosc_with_road_file()`, `build_trajectory()`, `build_init_actions()`
- Limitation: Pedestrians exported as vehicles (openscenario-rs library constraint)

**SVG (svg_visualizer.rs):**

- 1200x600 canvas with configurable margins
- Dynamic bounds calculation from trajectory data
- Single-road rendering with lane markings
- Coordinate transformation: scenario coords → SVG viewport
- Color scheme: Green (ego), Blue (NPC), Red (violations)
- Displays metrics, legend, and violation markers

**GIF (gif_animator.rs):**

- Reuses SVG coordinate system and color scheme
- `GifAnimator` struct with embedded font and configuration
- Single-road rendering with lane markings
- Frame rendering pipeline: background → road → lanes → trails → vehicles → violations → metrics
- Error handling via `ScenarioGenError::GifExport` variant

**Testing:**

- Unit tests verify export functionality and coordinate transformations
- Integration tests verify end-to-end export for single and multiple scenarios
- Physics tests verify vehicle dynamics (velocity ratios, acceleration bounds)

## Dependencies

- **z3 0.13**: SMT solver (requires system Z3 library)
- **serde/serde_yaml/serde_json**: YAML input, JSON output
- **openscenario-rs 0.2.0**: OpenSCENARIO XML generation (builder feature)
- **svg 0.17**: SVG generation for static visualizations
- **image 0.25**: Image manipulation for GIF frames
- **gif 0.13**: GIF encoding
- **imageproc 0.25**: Text rendering on images
- **ab_glyph 0.2**: Font loading (compatible with imageproc)
- **clap**: CLI argument parsing
- **tracing**: Structured logging
- **uuid, chrono**: IDs and timestamps

## Development Workflow

When making changes:

1. Modify code in `src/`
2. Run `cargo fmt` and `cargo clippy`
3. Add/update tests
4. Run `cargo test` to verify
5. Test with example YAML files: `cargo run -- -i examples/cut_in_left.yaml -o test_output/`
6. Verify all 4 outputs are generated in the directory: JSON, XOSC, SVG, and GIF
7. For adversarial changes, test both modes: normal and `--adversarial`
8. For bicycle model scenarios, test: `cargo run -- -i examples/bicycle_lane_change.yaml -o bicycle_output/`

# commiting the code

we plan to commit regularly dependeing on the feature implemented with relavant messages so we know the changes made. The message should not contain a reference to claude, nor should it contain emojis

## File Organization

- `src/main.rs`: CLI entry point with JSON/XOSC/SVG/GIF output
- `src/lib.rs`: Public API (`generate_single_scenario`, `generate_multiple_scenarios`, `export_scenario_to_xosc`, `export_scenario_to_xosc_with_road_file`, `export_scenario_to_svg`, `export_scenario_to_gif`)
- `src/solver/encoder.rs`: GenericEncoder facade that dispatches to coordinate-specific encoders
- `src/solver/coordinate_encoder.rs`: CoordinateEncoder trait defining encoder interface
- `src/solver/encoders/cartesian.rs`: CartesianEncoder for (x, y) coordinate system
- `src/solver/encoders/bicycle.rs`: BicycleEncoder for (x, y, θ, v) kinematic bicycle model
- `src/dsl/types.rs`: DSL data structures including `RoadSpec` for single-road scenarios
- `src/dsl/parser.rs`: YAML parsing and validation
- `src/scenario/xosc_exporter.rs`: OpenSCENARIO export module
- `src/scenario/svg_visualizer.rs`: SVG static visualization module (single-road)
- `src/scenario/gif_animator.rs`: GIF animation export module (single-road)
- `assets/DejaVuSans.ttf`: Embedded font for GIF text rendering
- `examples/`: YAML specification examples (cut_in_left.yaml, bicycle_lane_change.yaml, etc.)
- `tests/`: Integration tests with fixture files
- `plans/`: Implementation plan documentation (historical)
- `README.md`: User-facing documentation
- `README_ADVERSARIAL.md`: Detailed adversarial generation guide with architecture and extension instructions
- `CLAUDE.md`: This file - AI assistant contributor guide
