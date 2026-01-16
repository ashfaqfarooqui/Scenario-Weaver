# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

CARLA Scenario Generator: Automatically generate diverse, safety-critical driving test scenarios from YAML specifications using Linear Temporal Logic (LTL) and Z3 SMT solver. Supports both safe scenario generation and adversarial generation (scenarios that intentionally violate safety constraints for testing edge cases).

### Multi-Road Network Support

THis is something planned but not implemented yet.
The generator supports complex road networks with multiple named roads, connections, and junctions:

- **RoadNetwork**: Define multiple named roads with world positions and headings
- **ExtendedRoadSpec**: Each road has id, lanes, origin (x, y), heading, and length
- **RoadConnection**: Define predecessor/successor relationships between roads
- **Junctions**: T-junctions and crossroads with automatic geometry calculation
- **Lane Direction Constraints**: Automatic velocity constraints based on lane direction
- **OpenDRIVE Export**: Full road network exported to .xodr format

Example multi-road specification:

```yaml
roads:
  roads:
    - id: main_road
      num_lanes: 4
      lane_width: 3.5
      lane_directions: [1, 1, -1, -1]  # 2 forward, 2 backward
      length: 400.0
      origin: { x: 0.0, y: 0.0 }
      heading: 0.0  # East

    - id: side_road
      num_lanes: 2
      lane_width: 3.0
      lane_directions: [1, -1]
      length: 150.0
      origin: { x: 200.0, y: -50.0 }
      heading: 1.5708  # North

  junctions:
    - id: t_junction_1
      junction_type: t_junction
      main_road: main_road
      incoming_roads: [side_road]
      position: 200.0
      side: right
```

See `examples/t_junction.yaml` and `examples/crossroads.yaml` for junction examples.

## Common Commands

### Build & Run

```bash
# Build project
cargo build --release

# Generate single scenario (creates output/scenario.json, .xosc, .xodr, .svg, .gif)
cargo run --release -- -i examples/cut_in_left.yaml -o output/

# Generate multiple scenarios (creates quintuplets for each)
cargo run --release -- -i examples/cut_in_left.yaml -o scenarios/ -n 5

# Generate adversarial scenarios (violate safety constraints)
cargo run --release -- -i examples/cut_in_left.yaml -o adversarial/ --adversarial

# Generate junction scenarios
cargo run --release -- -i examples/t_junction.yaml -o junction_output/

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
     - `road_network.rs`: Multi-road network types (`RoadNetwork`, `ExtendedRoadSpec`, `Junction`, `RoadConnection`)
   - **Important**: Supports both fixed values and ranges (e.g., `position: 50.0` or `position: [45.0, 55.0]`)
   - **Constraint modes**: `Enforce` (default safe), `Violate` (adversarial), `Ignore` (unconstrained)
   - **Road Network**: `RoadNetwork` contains named roads with world positions, connections, and junctions
   - **Junction Types**: `TJunction` (main + incoming road) and `Crossroads` (3+ roads meeting)

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
     - `encoders/frenet.rs`: FrenetEncoder for (s, t) coordinate system
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
     - Frenet: `frenet_s`, `frenet_t`, `frenet_vs`, `frenet_vt`, `frenet_lane`
   - **Velocity Direction Constraints**: `encode_lane_velocity_constraints()` enforces lane direction (forward: longitudinal vel >= 0, backward: <= 0)

4. **Scenario Module** (`src/scenario/`)
   - **Purpose**: Extract solutions from Z3 models and validate
   - **Key files**:
     - `model.rs`: Output data structures (`Scenario`, `ActorTrajectory`, `ValidationInfo`)
     - `extractor.rs`: Extract trajectories from Z3 model, compute metrics
   - **Validation**: Computes min TTC, min distance, detects violations with timestamps

### Data Flow Example (Cut-in Scenario)

1. **Input YAML**: Ego in lane 1 at 50m/15m/s, NPC in lane 0, cuts in between 2.5-7.5s, min_ttc=3.0s
2. **DSL Parser**: Creates `ScenarioSpec` with ranges for Z3 to choose
3. **LTL Generator**: Creates formula like `G(lane_0) UNTIL cut_in_time AND G(ttc > 3.0)`
4. **Z3 Encoder**:
   - Creates variables for 20 time steps (10s / 0.5s)
   - Encodes kinematics: `px[t+1] = px[t] + vx[t] * dt`
   - Encodes LTL: `G(ttc > 3.0)` → `ttc[0] > 3.0 ∧ ttc[1] > 3.0 ∧ ... ∧ ttc[20] > 3.0`
5. **Z3 Solver**: Finds values for all variables satisfying constraints
6. **Extractor**: Pulls position/velocity at each time step, validates safety, outputs JSON

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
  - `frenet.rs`: FrenetEncoder for (s, t) coordinates
    - Variables: `frenet_s`, `frenet_t`, `frenet_vs`, `frenet_vt`, `frenet_lane`
    - Smooth lane changes with lateral velocity constraints
    - Use case: Road-based scenarios with lane changes

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
5. Implement coordinate conversion in `ReferenceLine` if needed
6. Add tests to verify encoder works correctly
7. Update documentation (this file and README.md)

**Coordinate System Selection:**

The coordinate system is selected in the YAML specification:

```yaml
coordinate_system: frenet  # or cartesian
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
6. Add Z3 encoding to both coordinate encoders:
   - `src/solver/encoders/cartesian.rs`: Implement in `encode_proposition()` and add direct assertions in `encode_safety()` (only for `Enforce` mode)
   - `src/solver/encoders/frenet.rs`: Implement in `encode_proposition()` and add direct assertions in `encode_safety()` (only for `Enforce` mode)
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

The generator produces scenarios in **five formats** automatically:

### JSON Output

- **Metadata**: scenario_id (UUID), type, duration, time_step
- **Actor trajectories**: Position, velocity, lane, road_id at each time step
- **Validation metrics**: min_ttc, min_distance, all_constraints_satisfied
- **Safety violations**: List of violations with timestamps (for adversarial scenarios)

### OpenSCENARIO (.xosc) Output

- **Format**: Valid OpenSCENARIO 1.0+ XML
- **Module**: `src/scenario/xosc_exporter.rs`
- **Functions**:
  - `export_to_xosc(scenario: &Scenario) -> Result<String>`
  - `export_to_xosc_with_road_file(scenario, xodr_path) -> Result<String>`
- **Structure**:
  - FileHeader with scenario metadata and author "CARLA Scenario Generator"
  - RoadNetwork section with link to OpenDRIVE file
  - Entities section with vehicle definitions
  - Storyboard with trajectory-based actions for each actor
  - StopTrigger based on scenario duration
- **Usage**: Automatically generated with reference to .xodr file; also available via public API

### OpenDRIVE (.xodr) Output

- **Format**: Valid OpenDRIVE 1.7 XML
- **Module**: `src/scenario/xodr_exporter.rs`
- **Function**: `export_to_xodr(scenario: &Scenario, spec: &ScenarioSpec) -> Result<String>`
- **Structure**:
  - Header with revision info and bounding box
  - Road definitions with geometry (straight lines)
  - Lane sections with proper left/right/center structure
  - Road links for connected roads
  - Junction definitions for T-junctions and crossroads
- **Usage**: Automatically generated; referenced by .xosc file for complete scenario
 this is planned but not implemented

### SVG Visualization (.svg) Output

- **Format**: Static vector graphic visualization
- **Module**: `src/scenario/svg_visualizer.rs`
- **Function**: `export_to_svg(scenario: &Scenario) -> Result<String>`
- **Structure**:
  - Multi-road surface with lane markings
  - Junction boxes for T-junctions and crossroads
  - Complete vehicle trajectories from start to end
  - Vehicle markers at initial and final positions
  - Metrics bar with safety information (TTC, distance, status)
  - Legend with color key
  - Violation markers if constraints violated
- **Usage**: Automatically generated alongside JSON/XOSC/XODR; also available via public API `export_scenario_to_svg()`

### GIF Animation (.gif) Output

- **Format**: Animated GIF showing trajectory evolution
- **Module**: `src/scenario/gif_animator.rs`
- **Function**: `export_to_gif(scenario: &Scenario) -> Result<Vec<u8>>`
- **Features**:
  - 10 FPS animation showing vehicles moving through scenario
  - Fading trajectory trails showing motion history
  - Real-time metrics overlay (current time, TTC, distance, status)
  - Violation highlighting with red circles
  - Multi-road surface with lane markings
  - Junction rendering (T-junctions and crossroads)
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

### Visualization Export Implementation Details

**OpenSCENARIO (xosc_exporter.rs):**

- Creates complete OpenSCENARIO structure using openscenario-rs builder
- Trajectory-based actions for each actor
- References OpenDRIVE road file via RoadNetwork section
- Key functions: `export_to_xosc()`, `export_to_xosc_with_road_file()`, `build_trajectory()`, `build_init_actions()`

**OpenDRIVE (xodr_exporter.rs):**

- Uses `opendrive` crate for XML generation
- Supports single roads and multi-road networks
- Lane mapping: forward lanes → right side (negative IDs), backward → left side (positive IDs)
- Junction generation for T-junctions and crossroads with lane links
- Key functions: `export_to_xodr()`, `build_opendrive_from_network()`, `build_junction()`

**SVG (svg_visualizer.rs):**

- 1200x600 canvas with configurable margins
- Dynamic bounds calculation from trajectory data (supports multi-road networks)
- Junction rendering as filled polygons
- Coordinate transformation: scenario coords → SVG viewport
- Color scheme: Green (ego), Blue (NPC), Red (violations), Gray (junctions)

**GIF (gif_animator.rs):**

- Reuses SVG coordinate system and color scheme
- `GifAnimator` struct with embedded font and configuration
- Junction rendering support for multi-road scenarios
- Frame rendering pipeline: background → roads → junctions → lanes → trails → vehicles → violations → metrics
- Error handling via `ScenarioGenError::GifExport` variant

**Testing:**

- Unit tests verify export functionality and coordinate transformations
- Integration tests verify end-to-end export for single and multiple scenarios
- Junction geometry tests for T-junctions and crossroads

## Dependencies

- **z3 0.13**: SMT solver (requires system Z3 library)
- **serde/serde_yaml/serde_json**: YAML input, JSON output
- **openscenario-rs 0.2.0**: OpenSCENARIO XML generation (builder feature)
- **opendrive**: OpenDRIVE XML generation for road networks
- **svg 0.17**: SVG generation for static visualizations
- **image 0.25**: Image manipulation for GIF frames
- **gif 0.13**: GIF encoding
- **imageproc 0.25**: Text rendering on images
- **ab_glyph 0.2**: Font loading (compatible with imageproc)
- **uom**: Units of measurement for OpenDRIVE (Length, Angle)
- **vec1**: Non-empty vector type required by opendrive crate
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
6. Verify all 5 outputs are generated in the directory: JSON, XOSC, XODR, SVG, and GIF
7. For junction scenarios, test: `cargo run -- -i examples/t_junction.yaml -o junction_output/`
8. For adversarial changes, test both modes: normal and `--adversarial`

# commiting the code

we plan to commit regularly dependeing on the feature implemented with relavant messages so we know the changes made. The message should not contain a reference to claude, nor should it contain emojis

## File Organization

- `src/main.rs`: CLI entry point with quintuplet JSON/XOSC/XODR/SVG/GIF output
- `src/lib.rs`: Public API (`generate_single_scenario`, `generate_multiple_scenarios`, `export_scenario_to_xosc`, `export_scenario_to_xosc_with_road_file`, `export_scenario_to_xodr`, `export_scenario_to_svg`, `export_scenario_to_gif`)
- `src/solver/encoder.rs`: GenericEncoder facade that dispatches to coordinate-specific encoders
- `src/solver/coordinate_encoder.rs`: CoordinateEncoder trait defining encoder interface
- `src/solver/encoders/cartesian.rs`: CartesianEncoder for (x, y) coordinate system
- `src/solver/encoders/frenet.rs`: FrenetEncoder for (s, t) coordinate system
- `src/dsl/road_network.rs`: Road network types (`RoadNetwork`, `ExtendedRoadSpec`, `Junction`, `RoadConnection`)
- `src/scenario/xosc_exporter.rs`: OpenSCENARIO export module
- `src/scenario/xodr_exporter.rs`: OpenDRIVE export module (road networks, junctions)
- `src/scenario/svg_visualizer.rs`: SVG static visualization module (multi-road, junctions)
- `src/scenario/gif_animator.rs`: GIF animation export module (multi-road, junctions)
- `assets/DejaVuSans.ttf`: Embedded font for GIF text rendering
- `examples/`: YAML specification examples (including t_junction.yaml, crossroads.yaml)
- `roads/`: Reusable road specification templates
- `tests/`: Integration tests with fixture files
- `plans/`: Implementation plan documentation (historical)
- `roadplan/`: Road network implementation plan and reports
- `README.md`: User-facing documentation
- `README_ADVERSARIAL.md`: Detailed adversarial generation guide with architecture and extension instructions
- `CLAUDE.md`: This file - AI assistant contributor guide
