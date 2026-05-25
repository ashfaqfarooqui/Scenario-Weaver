# Creating Scenarios in ScenarioWeaver

This guide covers everything you need to create and extend scenarios in the ScenarioWeaver. The guide is split into two parts:

- **Part 1: YAML Scenario Specification** - For domain experts who want to define test scenarios without writing code
- **Part 2: Implementing New Scenario Types** - For Rust developers who want to extend the codebase with new scenario types

---

# Part 1: YAML Scenario Specification

This section is for testing engineers, domain experts, and anyone who wants to define driving test scenarios using declarative YAML files without needing Rust programming knowledge.

## 1.1 Introduction

### What is a Scenario Specification?

A scenario specification is a declarative description of a driving test case. You describe:
- **What** the scenario looks like (actors, road, initial conditions)
- **What** constraints must be satisfied (safety requirements, timing)
- **How** constraints should be enforced (enforce/violate/ignore modes)

The generator automatically finds concrete scenarios that match your specification using constraint solving with Z3.

### YAML-Based Declarative Approach

Instead of writing imperative code like "move actor 2 meters left, accelerate for 3 seconds", you write declarative constraints like "NPC must change lanes between 2-4 seconds while maintaining 3 seconds TTC".

Example:
```yaml
scenario_type: cut_in_left
actors:
  - id: npc
    lane_changes:
      - start_time: [2.0, 4.0]  # Solver chooses when
        duration: [3.0, 4.0]     # Solver chooses duration
min_ttc: 3.0                   # Constraint: always maintain 3s TTC
```

The solver finds values for timing, speeds, and positions that satisfy all constraints.

### Generation Pipeline Overview

```
YAML Input → DSL Parser → LTL Generator → Z3 Encoder → Z3 Solver → Scenario Extractor → JSON Output
     ↓            ↓              ↓              ↓            ↓               ↓              ↓
  Your spec   Validation   Temporal Logic   Constraints   Solving    Extract values   6 formats
                                                                                        (JSON/XOSC/XODR/
                                                                                         SVG/GIF/OpenLabel)
```

## 1.2 Complete YAML Reference Guide

### Required Fields

Every scenario specification must include these fields:

#### `scenario_type` (required)
Type: `string`
Values: `cut_in_left`, `cut_in_right`, `overtake_left`, `pedestrian_crossing`

The behavioral pattern for this scenario. Determines what LTL formulas are generated.

```yaml
scenario_type: cut_in_left
```

#### `time_step` (required)
Type: `float` (seconds)
Default: `0.1`
Range: `0.01` to `1.0` (typically)

The time discretization interval. Smaller values give smoother trajectories but slower solving.

```yaml
time_step: 0.1  # 10 Hz update rate
```

#### `duration` (required)
Type: `float` (seconds)
Range: `5.0` to `20.0` (typically)

Total scenario duration. Longer durations give more degrees of freedom but slower solving.

```yaml
duration: 10.0  # 10-second scenario
```

#### `actors` (required)
Type: `array` of actor specifications

List of all actors (vehicles/pedestrians) in the scenario. Must include at least one ego vehicle. See Actor Configuration section for details.

```yaml
actors:
  - id: ego
    role: ego
    # ... more fields
  - id: npc
    role: npc
    # ... more fields
```

#### `road` (required)
Type: `object` (RoadSpec)

Road configuration with lanes and directions.

```yaml
road:
  num_lanes: 3           # Total number of lanes
  lane_width: 3.5        # Width per lane in meters
  lane_directions: [1, 1, -1]  # +1 forward, -1 backward
  road_length: 400.0     # Optional: auto-calculated if omitted
```

#### `min_ttc` (required)
Type: `float` (seconds)

Minimum time-to-collision threshold. Enforced between all actor pairs based on `constraint_modes.min_ttc`.

```yaml
min_ttc: 3.0  # Require 3 seconds TTC
```

#### `min_distance` (required)
Type: `float` (meters)

Minimum longitudinal distance threshold. Enforced between all actor pairs based on `constraint_modes.min_distance`.

```yaml
min_distance: 5.0  # Require 5 meters separation
```

### Optional Fields - Coordinate Systems

#### `coordinate_system` (optional)
Type: `string`
Values: `cartesian` (default), `bicycle`

Which kinematic model to use for vehicle motion.

```yaml
coordinate_system: cartesian  # Simple (x,y) with discrete lanes (default)
# OR
coordinate_system: bicycle    # Kinematic bicycle model with heading tracking
```

**Cartesian**: Fast solving, lane-based motion, good for highway scenarios.
**Bicycle**: Realistic steering constraints, heading tracking, minimum turn radius.

#### `bicycle_config` (optional, required if `coordinate_system: bicycle`)
Type: `object` (BicycleConfig)

Default bicycle model parameters for all actors. Individual actors can override with `bicycle_params`.

```yaml
bicycle_config:
  default_wheelbase: 2.7              # meters (typical sedan)
  default_max_steering_angle: 0.6     # radians (~34 degrees)
  default_max_steering_rate: 0.5      # rad/s
```

### Optional Fields - Safety Constraints

#### `max_velocity` (optional)
Type: `float` (m/s)

Maximum speed limit for all actors. Controlled by `constraint_modes.max_velocity`.

```yaml
max_velocity: 22.0  # ~50 mph speed limit
```

#### `min_velocity` (optional)
Type: `float` (m/s)

Minimum speed requirement for all actors. Controlled by `constraint_modes.min_velocity`.

```yaml
min_velocity: 10.0  # Highway minimum speed
```

#### `min_lateral_distance` (optional)
Type: `float` (meters)

Minimum side-by-side distance between actors. Controlled by `constraint_modes.min_lateral_distance`.

```yaml
min_lateral_distance: 2.5  # Multi-lane safety clearance
```

#### `max_relative_velocity` (optional)
Type: `float` (m/s)

Maximum speed difference between any two actors. Controlled by `constraint_modes.max_relative_velocity`.

```yaml
max_relative_velocity: 10.0  # Prevent unsafe speed differences
```

### Optional Fields - Constraint Modes

#### `constraint_modes` (optional)
Type: `object` or `string`

Controls how each constraint is enforced. Can be detailed (per-constraint) or shorthand (all constraints).

**Detailed syntax**:
```yaml
constraint_modes:
  min_ttc: enforce               # Must maintain TTC > threshold (default)
  min_distance: enforce          # Must maintain distance > threshold (default)
  max_velocity: violate          # Must violate speed limit (adversarial)
  min_velocity: ignore           # No minimum speed constraint (default)
  min_lateral_distance: ignore   # No lateral distance constraint (default)
  max_relative_velocity: enforce # Must maintain relative speed < threshold
  max_acceleration: enforce      # Actor acceleration bounds (default)
```

**Shorthand syntax**:
```yaml
constraint_modes: enforce_all  # All constraints enforced (default)
# OR
constraint_modes: violate_all  # All constraints violated (adversarial)
# OR
constraint_modes: ignore_all   # All constraints ignored (maximum freedom)
```

**Mode meanings**:
- `enforce`: Constraint must always hold (G constraint in LTL)
- `violate`: Constraint must be violated at some point (F NOT constraint in LTL) - for adversarial generation
- `ignore`: Constraint not included in formula - for unconstrained generation

### Optional Fields - Optimization

#### `optimization_target` (optional)
Type: `string`
Values: `none` (default), `minimize_ttc`, `minimize_distance`, `minimize_severity`, `maximize_ttc`

Find worst-case or best-case scenarios by optimizing metrics.

```yaml
optimization_target: minimize_ttc  # Find closest call scenario
```

#### `num_scenarios` (optional)
Type: `integer`
Default: `1`

Number of diverse scenarios to generate using blocking clauses.

```yaml
num_scenarios: 10  # Generate 10 different scenarios
```

### Value Types: Fixed vs Ranges

Many numeric fields accept either fixed values or ranges:

**Fixed value** - Exact constraint:
```yaml
position: 50.0        # Must be exactly 50.0m
speed: 15.0           # Must be exactly 15.0 m/s
```

**Range** - Solver chooses within bounds:
```yaml
position: [45.0, 55.0]      # Solver picks between 45-55m
speed: [14.0, 16.0]         # Solver picks between 14-16 m/s
start_time: [2.0, 4.0]      # Solver picks when to start
```

Fields supporting ranges: `position`, `speed`, `acceleration`, `lane_changes[].start_time`, `lane_changes[].duration`

## 1.3 Actor Configuration Patterns

### Ego Vehicle Setup

Every scenario requires exactly one ego vehicle (the AV under test):

```yaml
actors:
  - id: ego                    # Unique identifier
    role: ego                  # Actor role (required)
    lane: 1                    # Starting lane (0-indexed)
    position: 50.0             # Starting position (meters along road)
    speed: 15.0                # Starting speed (m/s)
    direction: 1               # Lane direction: +1 forward, -1 backward
    acceleration: [-8.0, 3.0]  # Acceleration bounds [min, max] (m/s^2)
```

### NPC Vehicle Setup

Background vehicles that interact with ego:

```yaml
actors:
  - id: npc                    # Unique identifier
    role: npc                  # Actor role
    lane: 0                    # Starting lane
    position: [60.0, 80.0]     # Solver chooses position in range
    speed: [16.0, 20.0]        # Solver chooses speed in range
    direction: 1               # Lane direction
    acceleration: [-8.0, 3.0]  # Acceleration bounds
```

### Lane Change Configuration

Actors that change lanes must include `lane_changes` configuration:

```yaml
actors:
  - id: npc
    role: npc
    lane: 0
    # ... other fields ...

    lane_changes:
      - direction: right         # Direction: 'left' or 'right'
        start_time: [2.5, 3.5]   # When lane change starts (seconds)
        duration: [3.0, 4.0]     # How long transition takes (seconds)
```

**Lane change behavior**:
- **Cartesian system**: Lateral position linearly interpolates between lane centers
  - Lateral velocity: ~1.2 m/s for 3.5m lane change over 3s
  - Velocity ratio constraint: |vy| <= 0.15 * |vx| (prevents sideways sliding)
  - Lateral acceleration bounded to 2.0 m/s²
- **Bicycle system**: Steering angle and heading change smoothly
  - Constrained by `max_steering_angle` and `max_steering_rate`
  - Minimum turn radius enforced

### Bicycle Model Parameters

When using `coordinate_system: bicycle`, actors can override default parameters:

```yaml
actors:
  - id: npc
    role: npc
    # ... other fields ...

    bicycle_params:
      wheelbase: 2.9              # Vehicle wheelbase (meters)
      max_steering_angle: 0.5     # Max steering angle (radians)
      max_steering_rate: 0.4      # Max steering rate (rad/s)
```

If `bicycle_params` is omitted, actor uses values from scenario-level `bicycle_config`.

### Pedestrian Setup

Pedestrians use different physics parameters:

```yaml
actors:
  - id: pedestrian
    role: pedestrian           # Pedestrian role
    position: [10.0, 20.0]     # Starting position (meters)
    speed: [1.0, 1.5]          # Walking/running speed (m/s)
    acceleration: [-1.0, 1.0]  # Pedestrian acceleration bounds

    behavior:
      crossing_start_time: [3.0, 5.0]  # When to start crossing
      target_side: "right"              # Which sidewalk to reach
```

**Pedestrian speed constants**:
- Walk: 0.5 - 1.41 m/s
- Run: 2.0 - 3.54 m/s
- Max acceleration: 1.0 m/s²

### Behavior Field Usage

The `behavior` field is a JSON map for scenario-specific parameters. Contents vary by scenario type:

**Cut-in scenarios** (cut_in_left, cut_in_right):
```yaml
behavior: {}  # No longer used - use lane_changes config instead
```

**Overtake scenario** (overtake_left):
```yaml
behavior:
  overtake_start_time: [2.0, 3.0]  # When to move to passing lane
  overtake_end_time: [6.0, 8.0]    # When to return to original lane
```

**Pedestrian crossing** (pedestrian_crossing):
```yaml
behavior:
  crossing_start_time: [3.0, 5.0]  # When to start crossing
  target_side: "right"              # Target sidewalk ("left" or "right")
```

## 1.4 Constraint Configuration

### Safety Constraints (TTC and Distance)

Time-to-collision and minimum distance are enforced between **all actor pairs**:

```yaml
min_ttc: 3.0         # Pairwise TTC constraint
min_distance: 5.0    # Pairwise distance constraint

constraint_modes:
  min_ttc: enforce        # Must maintain TTC > 3.0s at all times
  min_distance: enforce   # Must maintain distance > 5.0m at all times
```

For adversarial generation (intentionally violate safety):
```yaml
constraint_modes:
  min_ttc: violate        # Must have TTC < 3.0s at some point
  min_distance: enforce   # But still maintain safe distance
```

### Velocity Constraints

**Speed limits** (max_velocity):
```yaml
max_velocity: 22.0  # Speed limit in m/s (~50 mph)

constraint_modes:
  max_velocity: enforce  # All actors must stay under limit (default)
  # OR
  max_velocity: violate  # Find scenarios where actors exceed limit (adversarial)
```

**Minimum speed** (min_velocity):
```yaml
min_velocity: 10.0  # Minimum speed in m/s

constraint_modes:
  min_velocity: enforce  # All actors must stay above minimum
  # OR
  min_velocity: ignore   # No minimum speed requirement (default)
```

### Lateral Distance Constraints

Enforce side-by-side clearance between actors:

```yaml
min_lateral_distance: 2.5  # Minimum lateral separation (meters)

constraint_modes:
  min_lateral_distance: enforce  # Require clearance in multi-lane scenarios
  # OR
  min_lateral_distance: ignore   # No lateral constraint (default)
```

### Relative Velocity Constraints

Control speed differences between actors:

```yaml
max_relative_velocity: 10.0  # Max speed difference (m/s)

constraint_modes:
  max_relative_velocity: enforce  # Prevent unsafe speed differences
  # OR
  max_relative_velocity: violate  # Find scenarios with large speed differences (adversarial)
```

### Constraint Modes for Adversarial Generation

Adversarial generation creates scenarios that intentionally violate safety constraints for edge case testing:

**Example: Speed limit violation while maintaining safety**
```yaml
max_velocity: 22.0
min_ttc: 3.0
min_distance: 5.0

constraint_modes:
  max_velocity: violate    # MUST exceed speed limit
  min_ttc: enforce         # BUT maintain safe TTC
  min_distance: enforce    # AND maintain safe distance
```

**Example: Find unsafe following scenarios**
```yaml
max_relative_velocity: 10.0
min_distance: 5.0

constraint_modes:
  max_relative_velocity: violate  # MUST have large speed difference
  min_distance: enforce            # BUT maintain minimum distance
```

**CLI shortcut** for full adversarial mode:
```bash
cargo run --release -- -i examples/cut_in_left.yaml -o adversarial/ --adversarial
```
This overrides all constraint modes to `violate`.

## 1.5 Complete YAML Template

Copy this template to create new scenarios:

```yaml
# Scenario type (required)
scenario_type: cut_in_left  # cut_in_left | cut_in_right | overtake_left | pedestrian_crossing

# Time configuration (required)
time_step: 0.1     # Time discretization (seconds)
duration: 10.0     # Total scenario duration (seconds)

# Road specification (required)
road:
  num_lanes: 3                 # Total number of lanes
  lane_width: 3.5              # Width per lane (meters)
  lane_directions: [1, 1, -1]  # +1 forward, -1 backward per lane
  road_length: 400.0           # Optional: auto-calculated if omitted

# Coordinate system (optional, default: cartesian)
coordinate_system: cartesian  # cartesian | bicycle

# Bicycle model configuration (optional, required if coordinate_system: bicycle)
# bicycle_config:
#   default_wheelbase: 2.7              # meters
#   default_max_steering_angle: 0.6     # radians
#   default_max_steering_rate: 0.5      # rad/s

# Actor specifications (required)
actors:
  # Ego vehicle (required)
  - id: ego
    role: ego
    lane: 1                    # Starting lane (0-indexed)
    position: 50.0             # Fixed value or [min, max] range
    speed: 15.0                # m/s (fixed or range)
    direction: 1               # +1 forward, -1 backward
    acceleration: [-8.0, 3.0]  # [min, max] bounds (m/s^2)
    # bicycle_params:          # Optional: override defaults for this actor
    #   wheelbase: 2.7
    #   max_steering_angle: 0.6
    #   max_steering_rate: 0.5

  # NPC vehicle (example)
  - id: npc
    role: npc
    lane: 0
    position: [60.0, 80.0]     # Solver chooses in range
    speed: [16.0, 20.0]        # Solver chooses in range
    direction: 1
    acceleration: [-8.0, 3.0]

    # Lane change configuration (if applicable)
    # lane_changes:
    #   - direction: right       # left | right
    #     start_time: [2.5, 3.5] # When to start (seconds)
    #     duration: [3.0, 4.0]   # How long transition takes (seconds)

    # Scenario-specific behavior (varies by scenario_type)
    # behavior:
    #   overtake_start_time: [2.0, 3.0]  # For overtake scenarios
    #   overtake_end_time: [6.0, 8.0]

# Safety constraints (required)
min_ttc: 3.0          # Minimum time-to-collision (seconds)
min_distance: 5.0     # Minimum longitudinal distance (meters)

# Optional safety constraints
# max_velocity: 22.0              # Speed limit (m/s)
# min_velocity: 10.0              # Minimum speed (m/s)
# min_lateral_distance: 2.5       # Side-by-side clearance (meters)
# max_relative_velocity: 10.0     # Max speed difference (m/s)

# Constraint enforcement modes (optional, defaults to enforce_all)
constraint_modes:
  min_ttc: enforce               # enforce | violate | ignore
  min_distance: enforce
  max_velocity: enforce          # Only if max_velocity specified
  min_velocity: ignore           # Only if min_velocity specified
  min_lateral_distance: ignore   # Only if min_lateral_distance specified
  max_relative_velocity: ignore  # Only if max_relative_velocity specified
  max_acceleration: enforce

# OR use shorthand:
# constraint_modes: enforce_all  # enforce_all | violate_all | ignore_all

# Optimization target (optional, default: none)
# optimization_target: none      # none | minimize_ttc | minimize_distance | minimize_severity | maximize_ttc

# Generation settings (optional)
num_scenarios: 1  # Number of diverse scenarios to generate
```

## 1.6 Example Scenarios by Type

### Cut-In Left

NPC starts in left lane, cuts into ego's lane:

```yaml
scenario_type: cut_in_left
time_step: 0.1
duration: 10.0

road:
  num_lanes: 3
  lane_width: 3.5
  lane_directions: [1, 1, -1]

actors:
  - id: ego
    role: ego
    lane: 1                    # Right lane
    position: [0.0, 55.0]
    speed: [14.0, 16.0]
    direction: 1
    acceleration: [-8.0, 3.0]

  - id: npc
    role: npc
    lane: 0                    # Left lane
    position: [20.0, 80.0]
    speed: [16.0, 20.0]
    direction: 1
    acceleration: [-8.0, 3.0]

    lane_changes:
      - direction: right         # Cut from left to right
        start_time: [1.4, 3.5]
        duration: [3.0, 4.0]

min_ttc: 3.0
min_distance: 5.0
num_scenarios: 5
```

### Overtake Left

NPC starts behind ego, overtakes via left lane, returns ahead:

```yaml
scenario_type: overtake_left
time_step: 0.5
duration: 15.0

road:
  num_lanes: 2
  lane_width: 3.5
  lane_directions: [1, 1]

actors:
  - id: ego
    role: ego
    lane: 1                    # Right lane
    position: 50.0
    speed: 15.0
    direction: 1
    acceleration: [-3.0, 2.0]

  - id: npc
    role: npc
    lane: 1                    # Starts in SAME lane as ego
    position: [30.0, 40.0]     # Behind ego
    speed: [18.0, 22.0]        # Faster than ego
    direction: 1
    acceleration: [-3.0, 4.0]

    behavior:
      overtake_start_time: [2.0, 3.0]  # When to move to left lane
      overtake_end_time: [6.0, 8.0]    # When to return to right lane

min_ttc: 2.0
min_distance: 5.0
```

### Pedestrian Crossing

Pedestrian crosses road while ego approaches:

```yaml
scenario_type: pedestrian_crossing
time_step: 0.1
duration: 10.0

road:
  num_lanes: 2
  lane_width: 3.5
  lane_directions: [1, 1]

actors:
  - id: ego
    role: ego
    lane: 0
    position: [0.0, 30.0]
    speed: [10.0, 15.0]
    direction: 1
    acceleration: [-8.0, 3.0]

  - id: pedestrian
    role: pedestrian
    position: [80.0, 100.0]    # Crossing point ahead of ego
    speed: [1.0, 1.5]          # Walking speed
    acceleration: [-1.0, 1.0]

    behavior:
      crossing_start_time: [3.0, 5.0]  # When to start crossing
      target_side: "right"              # Which sidewalk

min_ttc: 2.0
min_distance: 3.0
```

### Adversarial Scenario (Speed Limit Violation)

Generate scenarios where actors exceed speed limit but maintain safety:

```yaml
scenario_type: cut_in_left
time_step: 0.5
duration: 10.0

road:
  num_lanes: 2
  lane_width: 3.5
  lane_directions: [1, 1]

actors:
  - id: ego
    role: ego
    lane: 1
    position: [0.0, 20.0]
    speed: [25.0, 30.0]        # Intentionally above limit
    direction: 1
    acceleration: [-5.0, 3.0]

  - id: npc
    role: npc
    lane: 0
    position: [40.0, 60.0]
    speed: [15.0, 20.0]        # Within limit
    direction: 1
    acceleration: [-4.0, 2.0]

    lane_changes:
      - direction: right
        start_time: [3.0, 6.0]
        duration: [3.0, 4.0]

min_ttc: 3.0
min_distance: 5.0
max_velocity: 22.0             # 22 m/s speed limit (~50 mph)

constraint_modes:
  min_ttc: enforce             # Must maintain safety
  min_distance: enforce        # Must maintain distance
  max_velocity: violate        # MUST violate speed limit (adversarial)
```

### Bicycle Model Lane Change

Realistic vehicle dynamics with heading tracking:

```yaml
scenario_type: cut_in_left
time_step: 0.1
duration: 10.0
coordinate_system: bicycle     # Use bicycle model

bicycle_config:
  default_wheelbase: 2.7
  default_max_steering_angle: 0.6
  default_max_steering_rate: 0.5

road:
  num_lanes: 2
  lane_width: 3.5
  lane_directions: [1, 1]

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

    bicycle_params:            # Override defaults
      wheelbase: 2.9           # Larger vehicle (SUV)
      max_steering_angle: 0.5
      max_steering_rate: 0.4

    lane_changes:
      - direction: right
        start_time: [2.5, 3.5]
        duration: [3.0, 4.0]

min_ttc: 3.0
min_distance: 5.0
```

## 1.7 Common Issues & Troubleshooting

### UNSAT (Unsatisfiable) Scenarios

**Symptom**: Generator reports "No satisfying solution found (UNSAT)"

**Causes**:
1. **Conflicting constraints**: Cannot satisfy all constraints simultaneously
   - Example: Require TTC > 5s but lane change duration < 1s
2. **Physically impossible**: Constraints violate physics limits
   - Example: Lane change too fast for vehicle acceleration/steering limits
3. **Timing conflicts**: Time windows don't allow required behavior
   - Example: Overtake start_time.max >= end_time.min
4. **Over-constrained ranges**: Ranges too narrow for solver
   - Example: position: [50.0, 50.1] with many other constraints

**Solutions**:
- **Relax constraints**: Widen ranges, lower safety thresholds
- **Increase duration**: Give more time for behaviors to unfold
- **Increase time_step**: Coarser discretization (0.5s instead of 0.1s)
- **Check timing**: Ensure start < end for all time windows
- **Use ignore mode**: Set some constraint modes to `ignore`
- **Simplify**: Start with minimal constraints, add incrementally

**Example fix**:
```yaml
# Before (UNSAT):
time_step: 0.1
duration: 5.0
min_ttc: 5.0
lane_changes:
  - start_time: [1.0, 2.0]
    duration: [1.0, 1.5]

# After (SAT):
time_step: 0.2            # Coarser
duration: 10.0            # Longer
min_ttc: 3.0              # Relaxed
lane_changes:
  - start_time: [2.0, 4.0]  # Wider window
    duration: [3.0, 4.0]    # More realistic
```

### Validation Errors

**Symptom**: Parser rejects YAML before solving

**Common errors**:

1. **Missing required field**:
   ```
   Error: Missing field 'actors'
   ```
   Solution: Add all required fields (see template in section 1.5)

2. **Invalid scenario type**:
   ```
   Error: Unknown scenario type: 'merge_right'
   ```
   Solution: Use one of: `cut_in_left`, `cut_in_right`, `overtake_left`, `pedestrian_crossing`

3. **Invalid lane change config**:
   ```
   Error: Cut-in-left requires lane_changes configuration
   ```
   Solution: Add `lane_changes` block to NPC actor

4. **Timing validation**:
   ```
   Error: overtake_start_time.max must be less than overtake_end_time.min
   ```
   Solution: Ensure time ranges don't overlap

5. **Lane direction mismatch**:
   ```
   Error: lane_directions length (2) must equal num_lanes (3)
   ```
   Solution: Ensure `lane_directions` array has `num_lanes` elements

6. **Bicycle model missing config**:
   ```
   Error: coordinate_system bicycle requires bicycle_config
   ```
   Solution: Add `bicycle_config` section or per-actor `bicycle_params`

### Performance Tips

**Slow solving** (>10 seconds):

1. **Increase time_step**: 0.5s instead of 0.1s (5x speedup)
2. **Decrease duration**: 8s instead of 15s
3. **Simplify constraints**: Use `ignore` mode for non-critical constraints
4. **Avoid non-linear constraints**:
   - Use `cartesian` instead of `bicycle` if heading not needed
   - Use `ManhattanDistanceGT` instead of `Distance2DGT` for pedestrians
5. **Narrow ranges**: More specific ranges solve faster
   - `position: [50.0, 60.0]` faster than `position: [0.0, 200.0]`

**Example optimization**:
```yaml
# Slow configuration:
time_step: 0.05           # Very fine discretization
duration: 20.0            # Long duration
coordinate_system: bicycle # More complex

# Fast configuration:
time_step: 0.2            # Coarser (4x speedup)
duration: 10.0            # Shorter (2x speedup)
coordinate_system: cartesian # Simpler (2x speedup)
# Combined: ~16x faster solving
```

### When Scenarios are Impossible to Satisfy

Some constraint combinations are fundamentally impossible:

**Example 1: Contradictory modes**
```yaml
min_ttc: 3.0
constraint_modes:
  min_ttc: enforce
  min_ttc: violate  # ERROR: Can't both enforce AND violate
```

**Example 2: Physics violations**
```yaml
# Bicycle model: minimum turn radius too large for lane change
bicycle_config:
  default_wheelbase: 2.7
  default_max_steering_angle: 0.1  # Very small angle -> large turn radius

lane_changes:
  - duration: [1.0, 2.0]  # Too fast for minimum turn radius
```

**Example 3: Impossible adversarial scenario**
```yaml
# Cannot violate TTC if actors never meet
actors:
  - id: ego
    lane: 0
    speed: 30.0  # Fast
  - id: npc
    lane: 1
    speed: 5.0   # Slow, different lane

constraint_modes:
  min_ttc: violate  # Impossible - they're too far apart in speed/space
```

**Solution**: Re-think scenario design or accept that some combinations are infeasible.

---

# Part 2: Implementing New Scenario Types

This section is for Rust developers who want to extend the codebase with entirely new scenario types (e.g., roundabout navigation, parking scenarios, intersection crossing).

## 2.1 Architecture Overview

### ScenarioType Enum and Plugin System

The generator uses a **trait-based plugin architecture** for scenario types:

```
ScenarioType enum (src/dsl/types.rs:310-342)
     ↓
get_model() dispatcher
     ↓
Box<dyn ScenarioModel> trait object
     ↓
Scenario-specific implementation (src/scenarios/*.rs)
```

**Key files**:
- `src/dsl/types.rs`: `ScenarioType` enum and dispatcher
- `src/scenarios/mod.rs`: `ScenarioModel` trait definition
- `src/scenarios/cut_in_left.rs`: Example implementation
- `src/scenarios/overtake_left.rs`: Example implementation (uses all 4 trait methods)
- `src/scenarios/pedestrian_crossing.rs`: Example pedestrian scenario

### ScenarioModel Trait Interface

```rust
pub trait ScenarioModel: Send + Sync {
    fn validate(&self, spec: &ScenarioSpec) -> Result<()>;
    fn generate_ltl(&self, spec: &ScenarioSpec) -> Result<LTLFormula>;
    fn generate_safety(&self, spec: &ScenarioSpec) -> Result<LTLFormula>;
    fn add_z3_constraints(&self, spec: &ScenarioSpec, encoder: &Z3Encoder,
                          backend: &dyn Z3Backend, horizon: usize) -> Result<()>;
}
```

### Data Flow: YAML → JSON

```
1. YAML Input (examples/*.yaml)
        ↓
2. DSL Parser (src/dsl/parser.rs)
   - Parse YAML → ScenarioSpec struct
   - Validate structure
        ↓
3. Scenario Model (src/scenarios/*.rs)
   - validate() checks scenario-specific requirements
   - generate_ltl() creates temporal logic formula
        ↓
4. LTL Generator (src/ltl/generator.rs)
   - Combines scenario LTL + safety constraints
   - Applies constraint modes (enforce/violate/ignore)
        ↓
5. Z3 Encoder (src/solver/encoder.rs + encoders/*.rs)
   - Selects coordinate-specific encoder (Cartesian/Bicycle)
   - encode_kinematics() - position/velocity updates
   - encode_ltl() - expand temporal operators over time
   - encode_safety() - direct Z3 assertions (Enforce mode only)
   - add_z3_constraints() - custom scenario assertions
        ↓
6. Z3 Solver (via z3 crate)
   - Finds satisfying assignment for all variables
        ↓
7. Scenario Extractor (src/scenario/extractor.rs)
   - Extracts trajectories from Z3 model
   - Computes validation metrics (TTC, distance)
   - Checks for violations
        ↓
8. Output (src/main.rs + exporters)
   - JSON: scenario.json
   - OpenSCENARIO: scenario.xosc
   - SVG: scenario.svg
   - GIF: scenario.gif
```

### Role of Each Module

**DSL Module** (`src/dsl/`):
- Defines all data structures (`ScenarioSpec`, `ActorSpec`, `RoadSpec`)
- Parses YAML → structs
- Basic validation (field presence, types)

**Scenarios Module** (`src/scenarios/`):
- One file per scenario type
- Implements `ScenarioModel` trait
- Scenario-specific validation and LTL generation

**LTL Module** (`src/ltl/`):
- `formula.rs`: LTL AST (`Always`, `Eventually`, `Until`, `Proposition`)
- `generator.rs`: Orchestrates LTL generation (scenario + safety)

**Solver Module** (`src/solver/`):
- `encoder.rs`: `GenericEncoder` facade that dispatches to encoders
- `coordinate_encoder.rs`: `CoordinateEncoder` trait interface
- `encoders/cartesian.rs`: Cartesian (x, y) encoder
- `encoders/bicycle.rs`: Bicycle (x, y, θ, v) encoder
- `physics.rs`: Kinematic constraint helpers
- `multi_solve.rs`: Generate multiple scenarios with blocking clauses

**Scenario Module** (`src/scenario/`):
- `model.rs`: Output data structures (`Scenario`, `ActorTrajectory`)
- `extractor.rs`: Extract trajectories from Z3 model
- `xosc_exporter.rs`: Export to OpenSCENARIO format
- `svg_visualizer.rs`: Export to SVG visualization
- `gif_animator.rs`: Export to GIF animation

## 2.2 ScenarioModel Trait Deep Dive

### Method 1: validate()

**Signature**:
```rust
fn validate(&self, spec: &ScenarioSpec) -> Result<()>
```

**Purpose**: Validate scenario-specific requirements before LTL generation.

**When to use**:
- Check actor count (e.g., "exactly 2 actors")
- Check actor roles (e.g., "must have ego + npc")
- Check required behavior fields (e.g., "NPC needs overtake_start_time")
- Check lane relationships (e.g., "NPC must start in same lane as ego")
- Validate timing relationships (e.g., "start_time.max < end_time.min")
- Check lane change configuration (e.g., "must have non-empty lane_changes")

**What NOT to check**:
- Physics feasibility (Z3 will return UNSAT if impossible)
- Generic field presence (DSL parser handles this)
- Acceleration/velocity bounds (handled by encoder)

**Default implementation**: Returns `Ok(())` (no validation)

**Example** (from `cut_in_left.rs:16-34`):
```rust
impl ScenarioModel for CutInLeftModel {
    fn validate(&self, spec: &ScenarioSpec) -> Result<()> {
        // Check actor count
        if spec.actors.len() != 2 {
            return Err(ScenarioGenError::InvalidSpec(format!(
                "Cut-in-left requires exactly 2 actors, found {}",
                spec.actors.len()
            )));
        }

        let npc = &spec.npcs()[0];

        // Check lane change configuration
        if npc.lane_changes.is_empty() {
            return Err(ScenarioGenError::InvalidSpec(
                "Cut-in-left requires lane_changes configuration".to_string(),
            ));
        }

        Ok(())
    }
}
```

### Method 2: generate_ltl() (REQUIRED)

**Signature**:
```rust
fn generate_ltl(&self, spec: &ScenarioSpec) -> Result<LTLFormula>
```

**Purpose**: Generate the temporal logic formula defining scenario behavior.

**Must implement**: This is the only required method with no default implementation.

**What to include**:
- Initial conditions (lane assignments, relative positions)
- Temporal behavior (lane changes, overtaking maneuvers)
- Sequencing constraints (A UNTIL B, Eventually C)

**What NOT to include**:
- Safety constraints (TTC, distance) - handled by `generate_safety()`
- Constraint mode logic (enforce/violate/ignore) - handled by LTL generator

**Structure pattern**:
```rust
fn generate_ltl(&self, spec: &ScenarioSpec) -> Result<LTLFormula> {
    let ego = spec.ego()?;
    let npc = &spec.npcs()[0];

    // 1. Initial conditions
    let init = self.initial_conditions(spec, &ego.id, &npc.id);

    // 2. Behavioral constraints
    let behavior = self.scenario_behavior(spec, &ego.id, &npc.id);

    // 3. Combine
    Ok(init.and(behavior))
}
```

**Example** (from `cut_in_left.rs:37-51`):
```rust
fn generate_ltl(&self, spec: &ScenarioSpec) -> Result<LTLFormula> {
    let ego = spec.ego().map_err(|e| ScenarioGenError::InvalidSpec(e))?;
    let npc = &spec.npcs()[0];

    let ego_id = ego.id.as_str();
    let npc_id = npc.id.as_str();

    // Initial conditions
    let init = self.initial_conditions(spec, ego_id, npc_id);

    // Cut-in behavior
    let behavior = self.cut_in_behavior(spec, ego_id, npc_id);

    Ok(init.and(behavior))
}
```

### Method 3: generate_safety() (Optional)

**Signature**:
```rust
fn generate_safety(&self, spec: &ScenarioSpec) -> Result<LTLFormula>
```

**Purpose**: Generate safety constraints (TTC, distance, velocity).

**Default implementation**: Generates pairwise safety for all actor pairs using `generate_default_safety()`.

**When to override**:
- Need different actor pairing (not all pairs)
- Need asymmetric constraints (different thresholds per pair)
- Need to exclude certain safety checks for this scenario type

**When NOT to override**: The default works for 99% of scenarios.

**Default behavior** (from `scenarios/mod.rs:52-232`):
- Pairwise TTC constraints based on `constraint_modes.min_ttc`
- Pairwise distance constraints based on `constraint_modes.min_distance`
- Pairwise lateral distance based on `constraint_modes.min_lateral_distance`
- Pairwise relative velocity based on `constraint_modes.max_relative_velocity`
- Per-actor velocity constraints (`max_velocity`, `min_velocity`)
- Respects `Enforce`/`Violate`/`Ignore` modes

**Example override** (custom safety for specific actors):
```rust
fn generate_safety(&self, spec: &ScenarioSpec) -> Result<LTLFormula> {
    let ego = spec.ego()?;
    let npc = &spec.npcs()[0];

    // Only generate safety between ego and npc, not other pairs
    let ttc = LTLFormula::Atom(Proposition::TTCGT {
        actor1: ego.id.clone(),
        actor2: npc.id.clone(),
        ttc: spec.min_ttc,
    }).always();

    let distance = LTLFormula::Atom(Proposition::DistanceGT {
        actor1: ego.id.clone(),
        actor2: npc.id.clone(),
        distance: spec.min_distance,
    }).always();

    Ok(ttc.and(distance))
}
```

### Method 4: add_z3_constraints() (Optional)

**Signature**:
```rust
fn add_z3_constraints(
    &self,
    spec: &ScenarioSpec,
    encoder: &Z3Encoder,
    backend: &dyn Z3Backend,
    horizon: usize,
) -> Result<()>
```

**Purpose**: Add custom Z3 assertions beyond what LTL encoding provides.

**Default implementation**: Does nothing (returns `Ok(())`).

**When to use**:
- Enforce specific values at specific time steps (e.g., "lane = 0 at t=0")
- Add implication constraints (e.g., "if lane==X then ahead")
- Restrict possible values (e.g., "lane must be 0 or 1 only")
- Complex timing windows that LTL can't express precisely
- Prevent oscillation or unwanted behaviors

**When NOT to use**:
- Simple temporal patterns (use LTL instead)
- Safety constraints (use `generate_safety()` instead)
- Physics constraints (encoder handles these)

**Access encoder variables** using accessor methods:
```rust
let px = encoder.get_longitudinal_pos("actor_id", time_step);
let py = encoder.get_lateral_pos("actor_id", time_step);
let lane = encoder.get_lane_var("actor_id", time_step);
let vx = encoder.get_longitudinal_vel("actor_id", time_step);
```

**Add assertions**:
```rust
backend.assert(&px.gt(&Real::from_real(&spec.time_step, 50, 1))); // px > 50.0
backend.assert(&lane.eq(&Int::from_i64(0))); // lane == 0
```

**Example** (from `overtake_left.rs:97-214`):
```rust
fn add_z3_constraints(
    &self,
    spec: &ScenarioSpec,
    encoder: &Z3Encoder,
    backend: &dyn Z3Backend,
    horizon: usize,
) -> Result<()> {
    let ego = spec.ego()?;
    let npc = &spec.npcs()[0];
    let original_lane = ego.lane;
    let passing_lane = ego.lane - 1;

    // Parse timing parameters from behavior map
    let start_time: ValueOrRange = /* ... */;
    let end_time: ValueOrRange = /* ... */;

    let start_min_step = (start_time.min() / spec.time_step).ceil() as usize;
    let end_max_step = (end_time.max() / spec.time_step).floor() as usize;

    // PHASE 1: Before overtake - NPC must be in original lane
    for t in 0..start_min_step {
        let lane_t = encoder.get_lane_var(&npc.id, t);
        backend.assert(&lane_t.eq(&Int::from_i64(original_lane as i64)));
    }

    // PHASE 3: After overtake - NPC must be back in original lane
    for t in end_max_step..=horizon {
        let lane_t = encoder.get_lane_var(&npc.id, t);
        backend.assert(&lane_t.eq(&Int::from_i64(original_lane as i64)));
    }

    // Position constraint: NPC must be ahead before returning
    for t in /* return window */ {
        let npc_px = encoder.get_longitudinal_pos(&npc.id, t);
        let ego_px = encoder.get_longitudinal_pos(&ego.id, t);
        let lane_t = encoder.get_lane_var(&npc.id, t);

        let in_original = lane_t.eq(&Int::from_i64(original_lane as i64));
        let npc_ahead = npc_px.gt(ego_px);

        // If in original lane, must be ahead
        backend.assert(&in_original.implies(&npc_ahead));
    }

    Ok(())
}
```

## 2.3 LTL Pattern Library

### Available Propositions (21 total)

All propositions defined in `src/ltl/formula.rs:26-120`:

#### Vehicle Positioning (4)
```rust
Proposition::InLane { actor: String, lane: usize }
// Actor is in specific lane

Proposition::Ahead { actor1: String, actor2: String }
// actor1 longitudinally ahead of actor2 (px1 > px2)

Proposition::DistanceGT { actor1: String, actor2: String, distance: f64 }
// Longitudinal distance > threshold (linear constraint)

Proposition::TTCGT { actor1: String, actor2: String, ttc: f64 }
// Time-to-collision > threshold (same-lane only)
```

#### Velocity Constraints (2)
```rust
Proposition::VelocityGT { actor: String, velocity: f64 }
// Longitudinal speed > threshold (|vx| > velocity)

Proposition::VelocityLT { actor: String, velocity: f64 }
// Longitudinal speed < threshold (|vx| < velocity)
```

#### Lateral Positioning (3)
```rust
Proposition::LateralDistanceGT { actor1: String, actor2: String, distance: f64 }
// Lateral distance > threshold (|py1 - py2| > distance)

Proposition::OnLeftOf { actor1: String, actor2: String }
// actor1 left of actor2 (py1 > py2)

Proposition::OnRightOf { actor1: String, actor2: String }
// actor1 right of actor2 (py1 < py2)
```

#### Relative Velocity (1)
```rust
Proposition::RelativeVelocityGT { actor1: String, actor2: String, velocity: f64 }
// Speed difference > threshold (|vx1 - vx2| > velocity)
```

#### Pedestrian-Specific (6)
```rust
Proposition::OnSidewalk { actor: String, side: String }
// Pedestrian on sidewalk (side: "left" or "right")

Proposition::CrossingRoad { actor: String }
// Pedestrian on road surface (between sidewalks)

Proposition::Distance2DGT { actor1: String, actor2: String, distance: f64 }
// 2D Euclidean distance > threshold (quadratic - slow)

Proposition::ManhattanDistanceGT { actor1: String, actor2: String, distance: f64 }
// Manhattan distance > threshold (|dx| + |dy| > distance) (linear - fast)

Proposition::RectangularDistanceGT { actor1: String, actor2: String,
                                     threshold_x: f64, threshold_y: f64 }
// Rectangular safety box: |dx| > tx OR |dy| > ty (linear - fastest)

Proposition::PedestrianTTCGT { ego: String, pedestrian: String, ttc: f64 }
// Perpendicular crossing TTC > threshold
```

### Temporal Operators

Defined in `src/ltl/formula.rs:122-157`:

```rust
// Logical operators
formula.and(other)      // φ ∧ ψ (both true)
formula.or(other)       // φ ∨ ψ (at least one true)
formula.negate()        // ¬φ (negation)
formula.implies(other)  // φ → ψ (implication)

// Temporal operators
formula.always()        // G φ (globally - true at all future times)
formula.eventually()    // F φ (finally - true at some future time)
formula.until(other)    // φ U ψ (φ true until ψ becomes true)
formula.next()          // X φ (next - true at next time step)
```

### Initial Conditions Pattern

Establish starting configuration:

```rust
fn initial_conditions(&self, spec: &ScenarioSpec, ego_id: &str, npc_id: &str) -> LTLFormula {
    let ego = spec.ego().unwrap();
    let npc = &spec.npcs()[0];

    // Both actors in specific lanes
    LTLFormula::Atom(Proposition::InLane {
        actor: ego_id.to_string(),
        lane: ego.lane,
    })
    .and(LTLFormula::Atom(Proposition::InLane {
        actor: npc_id.to_string(),
        lane: npc.lane,
    }))
    // NPC ahead of ego
    .and(LTLFormula::Atom(Proposition::Ahead {
        actor1: npc_id.to_string(),
        actor2: ego_id.to_string(),
    }))
}
```

### Two-Phase Behavior Pattern

State A holds UNTIL transition to state B:

```rust
// Example: Cut-in behavior
// NPC stays in lane 0 UNTIL it moves to lane 1
fn cut_in_behavior(&self, spec: &ScenarioSpec, npc_id: &str) -> LTLFormula {
    let initial_lane = 0;
    let target_lane = 1;

    let in_initial = LTLFormula::Atom(Proposition::InLane {
        actor: npc_id.to_string(),
        lane: initial_lane,
    });

    let in_target = LTLFormula::Atom(Proposition::InLane {
        actor: npc_id.to_string(),
        lane: target_lane,
    });

    // A UNTIL B pattern
    in_initial.until(in_target)
}
```

### Three-Phase Behavior Pattern

State A → state B → state C:

```rust
// Example: Overtake behavior (from overtake_left.rs:244-273)
// Phase 1: original lane UNTIL Phase 2: passing lane, then Phase 3: return and ahead
fn overtake_behavior(&self, spec: &ScenarioSpec, ego_id: &str, npc_id: &str) -> LTLFormula {
    let original_lane = 1;
    let passing_lane = 0;

    let in_original = LTLFormula::Atom(Proposition::InLane {
        actor: npc_id.to_string(),
        lane: original_lane,
    });

    let in_passing = LTLFormula::Atom(Proposition::InLane {
        actor: npc_id.to_string(),
        lane: passing_lane,
    });

    let npc_ahead = LTLFormula::Atom(Proposition::Ahead {
        actor1: npc_id.to_string(),
        actor2: ego_id.to_string(),
    });

    // Phase 1→2: Stay in original UNTIL entering passing
    let phase_1_to_2 = in_original.clone().until(in_passing);

    // Phase 3: Eventually return to original AND be ahead
    let return_ahead = in_original.and(npc_ahead).eventually();

    phase_1_to_2.and(return_ahead)
}
```

### Continuous Constraint Pattern

Constraint holds throughout scenario:

```rust
// Example: Pedestrian always crossing (from pedestrian_crossing scenario)
fn crossing_behavior(&self, pedestrian_id: &str) -> LTLFormula {
    let crossing = LTLFormula::Atom(Proposition::CrossingRoad {
        actor: pedestrian_id.to_string(),
    });

    // Eventually start crossing, then always crossing
    crossing.eventually().and(crossing.always())
}
```

### When to Use LTL vs Direct Z3 Constraints

**Use LTL when**:
- Expressing temporal patterns (UNTIL, EVENTUALLY, ALWAYS)
- Constraint applies across all/most time steps
- Want constraint mode support (enforce/violate/ignore)
- Relationship between propositions

**Use direct Z3 constraints when**:
- Specific time steps need specific values
- Complex implications or conditionals
- Restricting solution space (e.g., "lane must be 0 or 1 only")
- LTL encoding too coarse (bounded model checking limitations)
- Preventing oscillation or unwanted transitions

**Example: Lane persistence after cut-in**
```rust
// LTL encoding: F(G(InLane(npc, 1)))
// Problem: In bounded LTL, solver can delay until last step

// Solution: Use direct Z3 (in add_z3_constraints):
// After cut-in completes at step T, force lane=1 for all t > T
```

## 2.4 Step-by-Step Tutorial: Adding "MergeRight" Scenario

This section walks through implementing a complete new scenario type: a vehicle merging from an on-ramp onto a highway.

### Scenario Description

**Behavior**: NPC vehicle starts in a merge lane (on-ramp), accelerates, and merges into the highway lane where ego is traveling.

**Actors**:
- Ego: Traveling in highway lane (lane 1)
- NPC: Starts in merge lane (lane 0), merges into lane 1

**Constraints**:
- Merge must occur within specified time window
- Safe distance and TTC must be maintained
- NPC must be traveling forward (direction=1)

### Step 1: Define the Scenario

Before writing code, clearly define:

1. **Initial conditions**:
   - Ego in lane 1 (highway lane)
   - NPC in lane 0 (merge lane)
   - NPC behind ego longitudinally

2. **Behavior**:
   - NPC stays in lane 0 initially
   - NPC transitions to lane 1 (merge) at `merge_time`
   - NPC stays in lane 1 after merge

3. **Behavior parameters** (in YAML `behavior` field):
   - `merge_time`: When to start merging (range: [min, max])
   - No duration needed (use lane_changes config instead)

### Step 2: Add to ScenarioType Enum

**File**: `src/dsl/types.rs`

Add new variant to `ScenarioType` enum (around line 310):

```rust
#[derive(Debug, Clone, Copy, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum ScenarioType {
    CutInLeft,
    CutInRight,
    OvertakeLeft,
    PedestrianCrossing,
    MergeRight,  // NEW
}
```

Add to `Display` impl (around line 317):

```rust
impl std::fmt::Display for ScenarioType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ScenarioType::CutInLeft => write!(f, "cut_in_left"),
            ScenarioType::CutInRight => write!(f, "cut_in_right"),
            ScenarioType::OvertakeLeft => write!(f, "overtake_left"),
            ScenarioType::PedestrianCrossing => write!(f, "pedestrian_crossing"),
            ScenarioType::MergeRight => write!(f, "merge_right"),  // NEW
        }
    }
}
```

Add to `get_model()` dispatcher (around line 328):

```rust
impl ScenarioType {
    pub fn get_model(&self) -> Box<dyn crate::scenarios::ScenarioModel> {
        match self {
            ScenarioType::CutInLeft => Box::new(crate::scenarios::cut_in_left::CutInLeftModel),
            ScenarioType::CutInRight => Box::new(crate::scenarios::cut_in_right::CutInRightModel),
            ScenarioType::OvertakeLeft => Box::new(crate::scenarios::overtake_left::OvertakeLeftModel),
            ScenarioType::PedestrianCrossing => Box::new(crate::scenarios::pedestrian_crossing::PedestrianCrossingModel),
            ScenarioType::MergeRight => Box::new(crate::scenarios::merge_right::MergeRightModel),  // NEW
        }
    }
}
```

### Step 3: Create Scenario Model File

**File**: `src/scenarios/merge_right.rs`

```rust
//! Merge from right scenario model
//!
//! In this scenario, an NPC vehicle starts in a merge lane (on-ramp, lane 0),
//! accelerates, and merges into the highway lane (lane 1) where ego is traveling.

use crate::dsl::types::ScenarioSpec;
use crate::error::{Result, ScenarioGenError};
use crate::ltl::formula::{LTLFormula, Proposition};
use crate::scenarios::ScenarioModel;

/// Merge from right scenario model
pub struct MergeRightModel;

impl ScenarioModel for MergeRightModel {
    fn validate(&self, spec: &ScenarioSpec) -> Result<()> {
        // Validate exactly 2 actors (ego + 1 npc)
        if spec.actors.len() != 2 {
            return Err(ScenarioGenError::InvalidSpec(format!(
                "Merge-right requires exactly 2 actors, found {}",
                spec.actors.len()
            )));
        }

        let ego = spec.ego().map_err(|e| ScenarioGenError::InvalidSpec(e))?;
        let npc = &spec.npcs()[0];

        // Validate ego is in lane 1 (highway)
        if ego.lane != 1 {
            return Err(ScenarioGenError::InvalidSpec(format!(
                "Merge-right requires ego in lane 1 (highway), found lane {}",
                ego.lane
            )));
        }

        // Validate NPC is in lane 0 (merge lane)
        if npc.lane != 0 {
            return Err(ScenarioGenError::InvalidSpec(format!(
                "Merge-right requires NPC in lane 0 (merge lane), found lane {}",
                npc.lane
            )));
        }

        // Validate lane change configuration exists
        if npc.lane_changes.is_empty() {
            return Err(ScenarioGenError::InvalidSpec(
                "Merge-right requires lane_changes configuration".to_string(),
            ));
        }

        Ok(())
    }

    fn generate_ltl(&self, spec: &ScenarioSpec) -> Result<LTLFormula> {
        let ego = spec.ego().map_err(|e| ScenarioGenError::InvalidSpec(e))?;
        let npc = &spec.npcs()[0];

        let ego_id = ego.id.as_str();
        let npc_id = npc.id.as_str();

        // Initial conditions
        let init = self.initial_conditions(spec, ego_id, npc_id);

        // Merge behavior
        let behavior = self.merge_behavior(spec, ego_id, npc_id);

        Ok(init.and(behavior))
    }
}

impl MergeRightModel {
    fn initial_conditions(&self, spec: &ScenarioSpec, ego_id: &str, npc_id: &str) -> LTLFormula {
        let ego = spec.ego().unwrap();
        let npc = &spec.npcs()[0];

        // Ego in highway lane (lane 1)
        let ego_lane = LTLFormula::Atom(Proposition::InLane {
            actor: ego_id.to_string(),
            lane: ego.lane,
        });

        // NPC in merge lane (lane 0)
        let npc_lane = LTLFormula::Atom(Proposition::InLane {
            actor: npc_id.to_string(),
            lane: npc.lane,
        });

        // NPC behind ego initially
        let ego_ahead = LTLFormula::Atom(Proposition::Ahead {
            actor1: ego_id.to_string(),
            actor2: npc_id.to_string(),
        });

        ego_lane.and(npc_lane).and(ego_ahead)
    }

    fn merge_behavior(&self, spec: &ScenarioSpec, _ego_id: &str, npc_id: &str) -> LTLFormula {
        let merge_lane = 0;
        let highway_lane = 1;

        // NPC stays in merge lane UNTIL transitioning to highway lane
        let in_merge = LTLFormula::Atom(Proposition::InLane {
            actor: npc_id.to_string(),
            lane: merge_lane,
        });

        let in_highway = LTLFormula::Atom(Proposition::InLane {
            actor: npc_id.to_string(),
            lane: highway_lane,
        });

        // Two-phase pattern: merge lane UNTIL highway lane
        in_merge.until(in_highway)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dsl::types::{
        ActorRole, ActorSpec, ConstraintModes, LaneChangeConfig, LaneChangeDirection,
        OptimizationTarget, ScenarioType, ValueOrRange,
    };
    use std::collections::HashMap;

    fn create_test_spec() -> ScenarioSpec {
        let npc_lane_change = LaneChangeConfig {
            direction: LaneChangeDirection::Right,
            start_time: ValueOrRange::Range([3.0, 5.0]),
            duration: ValueOrRange::Range([2.0, 3.0]),
        };

        ScenarioSpec {
            scenario_type: ScenarioType::MergeRight,
            time_step: 0.1,
            duration: 10.0,
            actors: vec![
                ActorSpec {
                    id: "ego".to_string(),
                    role: ActorRole::Ego,
                    lane: 1, // Highway lane
                    position: ValueOrRange::Value(50.0),
                    speed: ValueOrRange::Value(20.0),
                    acceleration: ValueOrRange::Range([-8.0, 3.0]),
                    direction: 1,
                    behavior: HashMap::new(),
                    lane_changes: vec![],
                    bicycle_params: None,
                },
                ActorSpec {
                    id: "npc".to_string(),
                    role: ActorRole::Npc,
                    lane: 0, // Merge lane
                    position: ValueOrRange::Range([30.0, 40.0]),
                    speed: ValueOrRange::Range([15.0, 18.0]),
                    acceleration: ValueOrRange::Range([-5.0, 4.0]),
                    direction: 1,
                    behavior: HashMap::new(),
                    lane_changes: vec![npc_lane_change],
                    bicycle_params: None,
                },
            ],
            min_ttc: 3.0,
            min_distance: 10.0,
            road: None,
            lane_width: 3.5,
            num_scenarios: 1,
            constraint_modes: ConstraintModes::default(),
            optimization_target: OptimizationTarget::None,
            max_acceleration: None,
            max_deceleration: None,
            max_velocity: None,
            min_velocity: None,
            min_lateral_distance: None,
            max_relative_velocity: None,
            max_lateral_acceleration: 2.0,
            coordinate_system: crate::dsl::types::CoordinateSystem::Cartesian,
            bicycle_config: None,
        }
    }

    #[test]
    fn test_merge_right_validate_success() {
        let model = MergeRightModel;
        let spec = create_test_spec();
        assert!(model.validate(&spec).is_ok());
    }

    #[test]
    fn test_merge_right_validate_wrong_ego_lane() {
        let model = MergeRightModel;
        let mut spec = create_test_spec();
        spec.actors[0].lane = 0; // Ego in wrong lane
        assert!(model.validate(&spec).is_err());
    }

    #[test]
    fn test_merge_right_validate_wrong_npc_lane() {
        let model = MergeRightModel;
        let mut spec = create_test_spec();
        spec.actors[1].lane = 2; // NPC in wrong lane
        assert!(model.validate(&spec).is_err());
    }

    #[test]
    fn test_merge_right_validate_missing_lane_change() {
        let model = MergeRightModel;
        let mut spec = create_test_spec();
        spec.actors[1].lane_changes = vec![];
        assert!(model.validate(&spec).is_err());
    }

    #[test]
    fn test_merge_right_generate_ltl() {
        let model = MergeRightModel;
        let spec = create_test_spec();
        let formula = model.generate_ltl(&spec);
        assert!(formula.is_ok());

        let formula_str = format!("{}", formula.unwrap());
        assert!(formula_str.contains("InLane"));
        assert!(formula_str.contains("Ahead"));
    }
}
```

### Step 4: Register in Module System

**File**: `src/scenarios/mod.rs`

Add module declaration (around line 234):

```rust
pub mod cut_in_left;
pub mod cut_in_right;
pub mod overtake_left;
pub mod pedestrian_crossing;
pub mod merge_right;  // NEW
```

### Step 5: Create YAML Example

**File**: `examples/merge_right.yaml`

```yaml
scenario_type: merge_right

# Time configuration
time_step: 0.1
duration: 10.0

# Road specification
road:
  num_lanes: 2
  lane_width: 3.5
  lane_directions: [1, 1]  # Both lanes forward

# Actor specifications
actors:
  # Ego vehicle (traveling on highway)
  - id: ego
    role: ego
    lane: 1                    # Highway lane
    position: 50.0
    speed: 20.0                # Constant highway speed
    direction: 1
    acceleration: [-8.0, 3.0]

  # NPC vehicle (merging from on-ramp)
  - id: npc
    role: npc
    lane: 0                    # Merge lane (on-ramp)
    position: [30.0, 40.0]     # Behind ego
    speed: [15.0, 18.0]        # Slower initially
    direction: 1
    acceleration: [-5.0, 4.0]  # Can accelerate to match traffic

    # Lane change configuration
    lane_changes:
      - direction: right         # Merge into highway (lane 0 -> lane 1)
        start_time: [3.0, 5.0]   # Merge between 3-5 seconds
        duration: [2.0, 3.0]     # Merge takes 2-3 seconds

# Safety constraints
min_ttc: 3.0          # Maintain safe time-to-collision
min_distance: 10.0    # Maintain safe distance (higher for highway)

# Generation settings
num_scenarios: 5
```

### Step 6: Add Integration Test

**File**: `tests/integration_test.rs`

Add test function:

```rust
#[test]
fn test_generate_merge_right_scenario() {
    let yaml_path = "examples/merge_right.yaml";
    let output_dir = "test_outputs/merge_right/";

    // Ensure output directory exists
    std::fs::create_dir_all(output_dir).unwrap();

    // Generate scenario
    let result = generate_single_scenario(yaml_path, output_dir);
    assert!(result.is_ok(), "Failed to generate merge_right scenario: {:?}", result.err());

    // Verify outputs exist
    let json_path = format!("{}scenario.json", output_dir);
    let xosc_path = format!("{}scenario.xosc", output_dir);
    let svg_path = format!("{}scenario.svg", output_dir);
    let gif_path = format!("{}scenario.gif", output_dir);

    assert!(std::path::Path::new(&json_path).exists(), "JSON output missing");
    assert!(std::path::Path::new(&xosc_path).exists(), "XOSC output missing");
    assert!(std::path::Path::new(&svg_path).exists(), "SVG output missing");
    assert!(std::path::Path::new(&gif_path).exists(), "GIF output missing");

    // Parse and validate JSON
    let json_content = std::fs::read_to_string(&json_path).unwrap();
    let scenario: serde_json::Value = serde_json::from_str(&json_content).unwrap();

    assert_eq!(scenario["scenario_type"], "merge_right");
    assert_eq!(scenario["actors"].as_array().unwrap().len(), 2);
}
```

Also add fixture file:

**File**: `tests/fixtures/merge_right.yaml`

(Same content as `examples/merge_right.yaml`)

### Step 7: Test the Implementation

```bash
# Build
cargo build --release

# Run unit tests
cargo test merge_right

# Run integration test
cargo test test_generate_merge_right_scenario

# Generate example scenario
cargo run --release -- -i examples/merge_right.yaml -o outputs/merge_right/

# Verify outputs
ls outputs/merge_right/
# Should see: scenario.json, scenario.xosc, scenario.svg, scenario.gif

# Inspect JSON
cat outputs/merge_right/scenario.json | jq .
```

### Step 8: Document the New Scenario Type

Update `README.md` to list the new scenario type:

```markdown
## Supported Scenario Types

- **cut_in_left**: NPC cuts into ego's lane from the left
- **cut_in_right**: NPC cuts into ego's lane from the right
- **overtake_left**: NPC overtakes ego via left lane
- **pedestrian_crossing**: Pedestrian crosses road in front of ego
- **merge_right**: NPC merges from on-ramp into highway (NEW)
```

Update this guide (`CREATING_SCENARIOS.md`) with the merge_right example.

## 2.5 Validation Patterns

### Actor Count Validation

```rust
fn validate(&self, spec: &ScenarioSpec) -> Result<()> {
    // Exactly 2 actors
    if spec.actors.len() != 2 {
        return Err(ScenarioGenError::InvalidSpec(format!(
            "Scenario requires exactly 2 actors, found {}",
            spec.actors.len()
        )));
    }

    // At least 2 actors
    if spec.actors.len() < 2 {
        return Err(ScenarioGenError::InvalidSpec(
            "Scenario requires at least 2 actors".to_string()
        ));
    }

    Ok(())
}
```

### Role Validation

```rust
fn validate(&self, spec: &ScenarioSpec) -> Result<()> {
    // Must have exactly one ego
    let ego = spec.ego().map_err(|e| ScenarioGenError::InvalidSpec(e))?;

    // Must have at least one NPC
    if spec.npcs().is_empty() {
        return Err(ScenarioGenError::InvalidSpec(
            "Scenario requires at least one NPC actor".to_string()
        ));
    }

    // Must have pedestrian
    let pedestrians: Vec<_> = spec.actors.iter()
        .filter(|a| matches!(a.role, ActorRole::Pedestrian))
        .collect();
    if pedestrians.is_empty() {
        return Err(ScenarioGenError::InvalidSpec(
            "Scenario requires at least one pedestrian".to_string()
        ));
    }

    Ok(())
}
```

### Lane Constraint Validation

```rust
fn validate(&self, spec: &ScenarioSpec) -> Result<()> {
    let ego = spec.ego()?;
    let npc = &spec.npcs()[0];

    // NPC must start in same lane as ego
    if npc.lane != ego.lane {
        return Err(ScenarioGenError::InvalidSpec(format!(
            "NPC must start in same lane as ego. Ego: {}, NPC: {}",
            ego.lane, npc.lane
        )));
    }

    // Ego must not be in leftmost lane (need left lane for passing)
    if ego.lane == 0 {
        return Err(ScenarioGenError::InvalidSpec(
            "Ego must not be in leftmost lane (lane 0) - need passing lane".to_string()
        )));
    }

    Ok(())
}
```

### Behavior Field Validation

```rust
fn validate(&self, spec: &ScenarioSpec) -> Result<()> {
    let npc = &spec.npcs()[0];

    // Check required behavior fields exist
    if !npc.behavior.contains_key("overtake_start_time") {
        return Err(ScenarioGenError::InvalidSpec(
            "NPC missing 'overtake_start_time' in behavior map".to_string()
        ));
    }
    if !npc.behavior.contains_key("overtake_end_time") {
        return Err(ScenarioGenError::InvalidSpec(
            "NPC missing 'overtake_end_time' in behavior map".to_string()
        ));
    }

    // Parse and validate JSON values
    let start_time_json = npc.behavior.get("overtake_start_time").unwrap();
    let end_time_json = npc.behavior.get("overtake_end_time").unwrap();

    let start_time: ValueOrRange = serde_json::from_value(start_time_json.clone())
        .map_err(|e| ScenarioGenError::InvalidSpec(
            format!("Failed to parse overtake_start_time: {}", e)
        ))?;
    let end_time: ValueOrRange = serde_json::from_value(end_time_json.clone())
        .map_err(|e| ScenarioGenError::InvalidSpec(
            format!("Failed to parse overtake_end_time: {}", e)
        ))?;

    Ok(())
}
```

### Timing Relationship Validation

```rust
fn validate(&self, spec: &ScenarioSpec) -> Result<()> {
    let npc = &spec.npcs()[0];

    // Parse timing from behavior
    let start_time: ValueOrRange = /* parse from behavior */;
    let end_time: ValueOrRange = /* parse from behavior */;

    // Ensure non-overlapping windows: start.max < end.min
    if start_time.max() >= end_time.min() {
        return Err(ScenarioGenError::InvalidSpec(format!(
            "start_time.max ({}) must be less than end_time.min ({})",
            start_time.max(),
            end_time.min()
        )));
    }

    // Ensure timing fits within scenario duration
    if end_time.max() > spec.duration {
        return Err(ScenarioGenError::InvalidSpec(format!(
            "end_time.max ({}) exceeds scenario duration ({})",
            end_time.max(),
            spec.duration
        )));
    }

    Ok(())
}
```

### Lane Change Configuration Validation

```rust
fn validate(&self, spec: &ScenarioSpec) -> Result<()> {
    let npc = &spec.npcs()[0];

    // Check lane_changes is non-empty
    if npc.lane_changes.is_empty() {
        return Err(ScenarioGenError::InvalidSpec(
            "NPC missing lane_changes configuration".to_string()
        ));
    }

    // Validate direction is compatible with scenario
    use crate::dsl::types::LaneChangeDirection;
    if npc.lane_changes[0].direction != LaneChangeDirection::Right {
        return Err(ScenarioGenError::InvalidSpec(
            "Scenario requires lane_changes[0].direction=right".to_string()
        ));
    }

    Ok(())
}
```

## 2.6 Coordinate System Integration

### How Encoders Work

The solver module uses a **trait-based encoder system** with two implementations:

1. **CartesianEncoder** (`src/solver/encoders/cartesian.rs`):
   - Variables: `positions_x[actor][t]`, `positions_y[actor][t]`, `velocities_x[actor][t]`, `velocities_y[actor][t]`, `lanes[actor][t]`
   - Lane coupling: `py = lane * lane_width + lane_width/2`
   - Kinematics: `px[t+1] = px[t] + vx[t] * dt`
   - Use case: Highway scenarios, simple lane-based motion

2. **BicycleEncoder** (`src/solver/encoders/bicycle.rs`):
   - Variables: `positions_x[actor][t]`, `positions_y[actor][t]`, `heading_theta[actor][t]`, `speed_v[actor][t]`, `steering_delta[actor][t]`, `accelerations[actor][t]`, `lanes[actor][t]`
   - Kinematics (small angle): `dx/dt = v`, `dy/dt = v*θ`, `dθ/dt = (v/L)*δ`, `dv/dt = a`
   - Constraints: Steering angle/rate bounds, heading angle bounds, turn radius
   - Use case: Realistic vehicle dynamics, scenarios requiring heading tracking

### When Scenarios Need Coordinate-System Awareness

**Most scenarios are coordinate-agnostic**: They only care about lanes and relative positions.

**Examples of coordinate-agnostic scenarios**:
- Cut-in (left/right): Lane transitions handled uniformly
- Merge: Lane transitions handled uniformly
- Overtake: Lane transitions handled uniformly

**When you MIGHT need coordinate awareness**:
- Scenario requires specific heading angles
- Scenario depends on turn radius constraints
- Scenario uses steering angle in behavior specification

### Lane Change Behavior (Handled Uniformly)

Both encoders handle lane changes automatically based on `lane_changes` configuration:

- **Cartesian**: `encode_lane_change_constraints()` in `src/solver/encoders/cartesian.rs`
  - Lateral position interpolates linearly between lane centers
  - Velocity ratio constraint: |vy| <= 0.15 * |vx|
  - Lateral acceleration bounded to 2.0 m/s²

- **Bicycle**: `encode_lane_change_constraints()` in `src/solver/encoders/bicycle.rs`
  - Heading angle changes smoothly to align with target lane
  - Steering angle and rate constrained
  - Turn radius enforced via wheelbase and max steering angle

**No scenario code needed**: Lane changes work automatically for both coordinate systems.

### Accessing Encoder Variables (Use Accessor Methods)

**NEVER access encoder fields directly**. Use accessor methods:

```rust
fn add_z3_constraints(
    &self,
    spec: &ScenarioSpec,
    encoder: &Z3Encoder,
    backend: &dyn Z3Backend,
    horizon: usize,
) -> Result<()> {
    let actor_id = "npc";
    let t = 5;

    // CORRECT: Use accessor methods
    let px = encoder.get_longitudinal_pos(actor_id, t);
    let py = encoder.get_lateral_pos(actor_id, t);
    let vx = encoder.get_longitudinal_vel(actor_id, t);
    let vy = encoder.get_lateral_vel(actor_id, t);
    let lane = encoder.get_lane_var(actor_id, t);

    // Use variables in assertions
    backend.assert(&px.gt(&Real::from_real(&spec.time_step, 100, 1)));

    Ok(())
}
```

**Available accessor methods** (in `src/solver/encoder.rs` and `coordinate_encoder.rs`):
- `get_longitudinal_pos(actor, t)`: Position along road
- `get_lateral_pos(actor, t)`: Position perpendicular to road
- `get_longitudinal_vel(actor, t)`: Velocity along road
- `get_lateral_vel(actor, t)`: Velocity perpendicular to road
- `get_lane_var(actor, t)`: Lane assignment (integer)

These methods work for **both** Cartesian and Bicycle encoders.

## 2.7 Testing Strategy

### Unit Tests in Scenario File

Every scenario file should have a `#[cfg(test)]` module:

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use crate::dsl::types::*;
    use std::collections::HashMap;

    fn create_test_spec() -> ScenarioSpec {
        // Create minimal valid spec for this scenario type
        ScenarioSpec {
            scenario_type: ScenarioType::MergeRight,
            // ... all required fields ...
        }
    }

    #[test]
    fn test_validate_success() {
        let model = MergeRightModel;
        let spec = create_test_spec();
        assert!(model.validate(&spec).is_ok());
    }

    #[test]
    fn test_validate_wrong_actor_count() {
        let model = MergeRightModel;
        let mut spec = create_test_spec();
        spec.actors.pop(); // Remove one actor
        assert!(model.validate(&spec).is_err());
    }

    #[test]
    fn test_generate_ltl() {
        let model = MergeRightModel;
        let spec = create_test_spec();
        let result = model.generate_ltl(&spec);
        assert!(result.is_ok());

        // Check LTL structure
        let formula = result.unwrap();
        let formula_str = format!("{}", formula);
        assert!(formula_str.contains("InLane"));
        assert!(formula_str.contains("Ahead"));
    }
}
```

**Test coverage**:
1. `test_validate_success`: Valid spec passes validation
2. `test_validate_*`: Each validation rule has a test that triggers the error
3. `test_generate_ltl`: LTL generation succeeds and contains expected propositions

### Integration Tests

**File**: `tests/integration_test.rs`

Test end-to-end generation:

```rust
#[test]
fn test_generate_merge_right_scenario() {
    let yaml_path = "tests/fixtures/merge_right.yaml";
    let output_dir = "test_outputs/merge_right/";

    // Ensure output directory exists
    std::fs::create_dir_all(output_dir).unwrap();

    // Generate scenario
    let result = generate_single_scenario(yaml_path, output_dir);
    assert!(result.is_ok(), "Generation failed: {:?}", result.err());

    // Verify all outputs exist
    assert!(Path::new(&format!("{}scenario.json", output_dir)).exists());
    assert!(Path::new(&format!("{}scenario.xosc", output_dir)).exists());
    assert!(Path::new(&format!("{}scenario.svg", output_dir)).exists());
    assert!(Path::new(&format!("{}scenario.gif", output_dir)).exists());

    // Parse JSON and validate structure
    let json = std::fs::read_to_string(format!("{}scenario.json", output_dir)).unwrap();
    let scenario: serde_json::Value = serde_json::from_str(&json).unwrap();

    assert_eq!(scenario["scenario_type"], "merge_right");
    assert!(scenario["actors"].is_array());
    assert!(scenario["validation"]["all_constraints_satisfied"].as_bool().unwrap());
}
```

### Fixture File Patterns

**File**: `tests/fixtures/merge_right.yaml`

Use minimal but realistic configurations:

```yaml
scenario_type: merge_right
time_step: 0.5      # Coarser for faster testing
duration: 10.0

road:
  num_lanes: 2
  lane_width: 3.5
  lane_directions: [1, 1]

actors:
  - id: ego
    role: ego
    lane: 1
    position: 50.0
    speed: 20.0
    direction: 1
    acceleration: [-8.0, 3.0]

  - id: npc
    role: npc
    lane: 0
    position: [30.0, 40.0]
    speed: [15.0, 18.0]
    direction: 1
    acceleration: [-5.0, 4.0]

    lane_changes:
      - direction: right
        start_time: [3.0, 5.0]
        duration: [2.0, 3.0]

min_ttc: 3.0
min_distance: 10.0
num_scenarios: 1    # Single scenario for tests
```

### Testing Adversarial Modes

Test both enforce and violate modes:

```rust
#[test]
fn test_merge_right_adversarial_ttc() {
    let yaml_path = "tests/fixtures/merge_right_adversarial.yaml";
    let output_dir = "test_outputs/merge_right_adversarial/";

    std::fs::create_dir_all(output_dir).unwrap();
    let result = generate_single_scenario(yaml_path, output_dir);
    assert!(result.is_ok());

    // Parse JSON
    let json = std::fs::read_to_string(format!("{}scenario.json", output_dir)).unwrap();
    let scenario: serde_json::Value = serde_json::from_str(&json).unwrap();

    // Verify TTC violation occurred
    let violations = scenario["validation"]["violations"].as_array().unwrap();
    let has_ttc_violation = violations.iter()
        .any(|v| v["constraint"].as_str().unwrap().contains("TTC"));

    assert!(has_ttc_violation, "Expected TTC violation in adversarial mode");
}
```

**Fixture**: `tests/fixtures/merge_right_adversarial.yaml`

```yaml
# Same as merge_right.yaml, but with:
constraint_modes:
  min_ttc: violate
  min_distance: enforce
```

### Testing with Different Coordinate Systems

Test both Cartesian and Bicycle modes:

```rust
#[test]
fn test_merge_right_bicycle_model() {
    let yaml_path = "tests/fixtures/merge_right_bicycle.yaml";
    let output_dir = "test_outputs/merge_right_bicycle/";

    std::fs::create_dir_all(output_dir).unwrap();
    let result = generate_single_scenario(yaml_path, output_dir);
    assert!(result.is_ok());

    // Parse and verify bicycle-specific fields
    let json = std::fs::read_to_string(format!("{}scenario.json", output_dir)).unwrap();
    let scenario: serde_json::Value = serde_json::from_str(&json).unwrap();

    // Bicycle mode should have reasonable heading angles
    let npc_traj = &scenario["actors"][1]["trajectory"];
    for waypoint in npc_traj.as_array().unwrap() {
        let vx = waypoint["velocity"]["vx"].as_f64().unwrap();
        let vy = waypoint["velocity"]["vy"].as_f64().unwrap();

        // Heading angle should be small (< 30 degrees for small angle approximation)
        if vx.abs() > 0.1 {
            let heading = (vy / vx).atan();
            assert!(heading.abs() < 0.52, "Heading angle too large: {} rad", heading);
        }
    }
}
```

**Fixture**: `tests/fixtures/merge_right_bicycle.yaml`

```yaml
coordinate_system: bicycle

bicycle_config:
  default_wheelbase: 2.7
  default_max_steering_angle: 0.6
  default_max_steering_rate: 0.5

# ... rest of config ...
```

## 2.8 Advanced Topics

### Using Behavior HashMap for Custom Parameters

The `behavior` field is a flexible JSON map for scenario-specific parameters:

```rust
// In YAML:
behavior:
  overtake_start_time: [2.0, 3.0]
  overtake_end_time: [6.0, 8.0]
  custom_flag: true

// In Rust (validate method):
fn validate(&self, spec: &ScenarioSpec) -> Result<()> {
    let npc = &spec.npcs()[0];

    // Parse timing parameters
    let start_json = npc.behavior.get("overtake_start_time")
        .ok_or_else(|| ScenarioGenError::InvalidSpec("Missing start_time".into()))?;

    let start_time: ValueOrRange = serde_json::from_value(start_json.clone())
        .map_err(|e| ScenarioGenError::InvalidSpec(format!("Parse error: {}", e)))?;

    // Parse boolean flag
    let custom_flag = npc.behavior.get("custom_flag")
        .and_then(|v| v.as_bool())
        .unwrap_or(false);

    if custom_flag {
        // Custom logic
    }

    Ok(())
}
```

**Best practices**:
- Document required behavior fields in scenario file comments
- Validate presence in `validate()` method
- Use `ValueOrRange` for timing/numeric parameters
- Use `bool` for flags
- Provide sensible defaults with `.unwrap_or(default)`

### Overriding Default Safety Constraints

Override `generate_safety()` for custom safety logic:

```rust
fn generate_safety(&self, spec: &ScenarioSpec) -> Result<LTLFormula> {
    let ego = spec.ego()?;
    let npc = &spec.npcs()[0];

    // Custom: Only enforce TTC during merge phase
    let merge_start_step = /* compute from behavior */;
    let merge_end_step = /* compute from behavior */;

    // Before merge: no TTC constraint
    // During merge: strict TTC constraint
    // After merge: relaxed TTC constraint

    // Note: This requires Z3 assertions, not LTL
    // Return tautology here, implement in add_z3_constraints()

    Ok(LTLFormula::Atom(Proposition::InLane {
        actor: ego.id.clone(),
        lane: ego.lane,
    }).or(LTLFormula::Atom(Proposition::InLane {
        actor: ego.id.clone(),
        lane: ego.lane,
    }).negate()))
}

fn add_z3_constraints(...) -> Result<()> {
    // Add phase-specific safety constraints here
    for t in merge_start_step..=merge_end_step {
        // Strict TTC during merge
        let ttc_constraint = /* compute TTC at step t */;
        backend.assert(&ttc_constraint);
    }
    Ok(())
}
```

### Adding Custom Z3 Constraints Beyond LTL

Use `add_z3_constraints()` for complex constraints:

**Example 1: Speed ramp-up during merge**
```rust
fn add_z3_constraints(...) -> Result<()> {
    let npc_id = "npc";
    let merge_start_step = 30;
    let merge_end_step = 50;

    // NPC must accelerate during merge
    for t in merge_start_step..merge_end_step {
        let vx_t = encoder.get_longitudinal_vel(npc_id, t);
        let vx_t1 = encoder.get_longitudinal_vel(npc_id, t + 1);

        // Speed must increase (or at least not decrease much)
        let min_accel = Real::from_real(&spec.time_step, -1, 10); // -0.1 m/s^2
        let accel_bound = &vx_t1 - &vx_t;
        backend.assert(&accel_bound.ge(&min_accel));
    }

    Ok(())
}
```

**Example 2: Position ordering constraints**
```rust
fn add_z3_constraints(...) -> Result<()> {
    let ego_id = "ego";
    let npc_id = "npc";

    // At end of scenario, NPC must be ahead of ego
    let final_step = horizon;
    let npc_px = encoder.get_longitudinal_pos(npc_id, final_step);
    let ego_px = encoder.get_longitudinal_pos(ego_id, final_step);

    backend.assert(&npc_px.gt(ego_px));

    Ok(())
}
```

**Example 3: Lane restriction (prevent impossible transitions)**
```rust
fn add_z3_constraints(...) -> Result<()> {
    let npc_id = "npc";
    let allowed_lanes = vec![0, 1]; // Only lanes 0 and 1

    for t in 0..=horizon {
        let lane_t = encoder.get_lane_var(npc_id, t);

        // lane == 0 OR lane == 1
        let in_lane_0 = lane_t.eq(&Int::from_i64(0));
        let in_lane_1 = lane_t.eq(&Int::from_i64(1));
        backend.assert(&Bool::or(&[&in_lane_0, &in_lane_1]));
    }

    Ok(())
}
```

### Multi-Actor Scenarios (>2 Actors)

Handle scenarios with more than 2 actors:

```rust
fn validate(&self, spec: &ScenarioSpec) -> Result<()> {
    // Allow 3+ actors
    if spec.actors.len() < 3 {
        return Err(ScenarioGenError::InvalidSpec(
            "Multi-merge requires at least 3 actors".to_string()
        ));
    }

    // Must have 1 ego + 2+ NPCs
    if spec.npcs().len() < 2 {
        return Err(ScenarioGenError::InvalidSpec(
            "Multi-merge requires at least 2 NPCs".to_string()
        ));
    }

    Ok(())
}

fn generate_ltl(&self, spec: &ScenarioSpec) -> Result<LTLFormula> {
    let ego = spec.ego()?;
    let npcs = spec.npcs();

    // Initial: All actors in specific lanes
    let mut init = LTLFormula::Atom(Proposition::InLane {
        actor: ego.id.clone(),
        lane: ego.lane,
    });

    for npc in &npcs {
        init = init.and(LTLFormula::Atom(Proposition::InLane {
            actor: npc.id.clone(),
            lane: npc.lane,
        }));
    }

    // Behavior: Each NPC merges independently
    let mut behaviors = Vec::new();
    for npc in &npcs {
        let merge_behavior = /* generate for this NPC */;
        behaviors.push(merge_behavior);
    }

    let combined_behavior = behaviors.into_iter()
        .reduce(|acc, b| acc.and(b))
        .unwrap();

    Ok(init.and(combined_behavior))
}
```

**Pairwise safety**: The default `generate_safety()` already handles all pairs correctly.

### Optimization Targets

Use optimization to find worst-case or best-case scenarios:

```yaml
# In YAML:
optimization_target: minimize_ttc  # Find closest call

# Or:
optimization_target: minimize_distance  # Find closest approach

# Or:
optimization_target: minimize_severity  # Minimize both (weighted)
```

**In Rust**: Optimization is handled automatically by the solver. No scenario-specific code needed.

**Use cases**:
- `minimize_ttc`: Find scenarios with lowest TTC (still > threshold if enforced)
- `minimize_distance`: Find scenarios with closest approach
- `minimize_severity`: Worst-case scenario (minimize both TTC and distance)
- `maximize_ttc`: Find safest scenario (opposite of minimize)

## 2.9 Reference: Complete Code Example

Here's the complete implementation of the MergeRight scenario from the tutorial:

**File**: `src/scenarios/merge_right.rs` (Complete, 203 lines)

```rust
//! Merge from right scenario model
//!
//! In this scenario, an NPC vehicle starts in a merge lane (on-ramp, lane 0),
//! accelerates, and merges into the highway lane (lane 1) where ego is traveling.
//!
//! ## Behavior
//! - Initial: Ego in lane 1 (highway), NPC in lane 0 (merge lane), NPC behind ego
//! - Transition: NPC changes from lane 0 to lane 1 at merge_time
//! - Final: NPC in lane 1, maintaining safe distance and TTC
//!
//! ## YAML Configuration
//! ```yaml
//! scenario_type: merge_right
//! actors:
//!   - id: ego
//!     role: ego
//!     lane: 1  # Highway lane
//!   - id: npc
//!     role: npc
//!     lane: 0  # Merge lane
//!     lane_changes:
//!       - direction: right
//!         start_time: [3.0, 5.0]
//!         duration: [2.0, 3.0]
//! ```

use crate::dsl::types::ScenarioSpec;
use crate::error::{Result, ScenarioGenError};
use crate::ltl::formula::{LTLFormula, Proposition};
use crate::scenarios::ScenarioModel;

/// Merge from right scenario model
pub struct MergeRightModel;

impl ScenarioModel for MergeRightModel {
    fn validate(&self, spec: &ScenarioSpec) -> Result<()> {
        // Validate exactly 2 actors (ego + 1 npc)
        if spec.actors.len() != 2 {
            return Err(ScenarioGenError::InvalidSpec(format!(
                "Merge-right requires exactly 2 actors, found {}",
                spec.actors.len()
            )));
        }

        let ego = spec.ego().map_err(|e| ScenarioGenError::InvalidSpec(e))?;
        let npc = &spec.npcs()[0];

        // Validate ego is in lane 1 (highway)
        if ego.lane != 1 {
            return Err(ScenarioGenError::InvalidSpec(format!(
                "Merge-right requires ego in lane 1 (highway), found lane {}",
                ego.lane
            )));
        }

        // Validate NPC is in lane 0 (merge lane)
        if npc.lane != 0 {
            return Err(ScenarioGenError::InvalidSpec(format!(
                "Merge-right requires NPC in lane 0 (merge lane), found lane {}",
                npc.lane
            )));
        }

        // Validate lane change configuration exists
        if npc.lane_changes.is_empty() {
            return Err(ScenarioGenError::InvalidSpec(
                "Merge-right requires lane_changes configuration".to_string(),
            ));
        }

        // Validate lane change direction is 'right'
        use crate::dsl::types::LaneChangeDirection;
        if npc.lane_changes[0].direction != LaneChangeDirection::Right {
            return Err(ScenarioGenError::InvalidSpec(
                "Merge-right requires lane_changes[0].direction=right".to_string(),
            ));
        }

        Ok(())
    }

    fn generate_ltl(&self, spec: &ScenarioSpec) -> Result<LTLFormula> {
        let ego = spec.ego().map_err(|e| ScenarioGenError::InvalidSpec(e))?;
        let npc = &spec.npcs()[0];

        let ego_id = ego.id.as_str();
        let npc_id = npc.id.as_str();

        // Initial conditions
        let init = self.initial_conditions(spec, ego_id, npc_id);

        // Merge behavior
        let behavior = self.merge_behavior(spec, ego_id, npc_id);

        Ok(init.and(behavior))
    }

    // Use default generate_safety() - pairwise TTC and distance
    // Use default add_z3_constraints() - no custom Z3 needed
}

impl MergeRightModel {
    /// Generate initial conditions LTL
    fn initial_conditions(&self, spec: &ScenarioSpec, ego_id: &str, npc_id: &str) -> LTLFormula {
        let ego = spec.ego().unwrap();
        let npc = &spec.npcs()[0];

        // Ego in highway lane (lane 1)
        let ego_lane = LTLFormula::Atom(Proposition::InLane {
            actor: ego_id.to_string(),
            lane: ego.lane,
        });

        // NPC in merge lane (lane 0)
        let npc_lane = LTLFormula::Atom(Proposition::InLane {
            actor: npc_id.to_string(),
            lane: npc.lane,
        });

        // NPC behind ego initially (ego is ahead)
        let ego_ahead = LTLFormula::Atom(Proposition::Ahead {
            actor1: ego_id.to_string(),
            actor2: npc_id.to_string(),
        });

        ego_lane.and(npc_lane).and(ego_ahead)
    }

    /// Generate merge behavior LTL
    fn merge_behavior(&self, spec: &ScenarioSpec, _ego_id: &str, npc_id: &str) -> LTLFormula {
        let merge_lane = 0;
        let highway_lane = 1;

        // NPC stays in merge lane UNTIL transitioning to highway lane
        let in_merge = LTLFormula::Atom(Proposition::InLane {
            actor: npc_id.to_string(),
            lane: merge_lane,
        });

        let in_highway = LTLFormula::Atom(Proposition::InLane {
            actor: npc_id.to_string(),
            lane: highway_lane,
        });

        // Two-phase pattern: merge lane UNTIL highway lane
        // Lane change timing and smoothness handled by encoder via lane_changes config
        in_merge.until(in_highway)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dsl::types::{
        ActorRole, ActorSpec, ConstraintModes, LaneChangeConfig, LaneChangeDirection,
        OptimizationTarget, ScenarioType, ValueOrRange, CoordinateSystem,
    };
    use std::collections::HashMap;

    fn create_test_spec() -> ScenarioSpec {
        let npc_lane_change = LaneChangeConfig {
            direction: LaneChangeDirection::Right,
            start_time: ValueOrRange::Range([3.0, 5.0]),
            duration: ValueOrRange::Range([2.0, 3.0]),
        };

        ScenarioSpec {
            scenario_type: ScenarioType::MergeRight,
            time_step: 0.1,
            duration: 10.0,
            actors: vec![
                ActorSpec {
                    id: "ego".to_string(),
                    role: ActorRole::Ego,
                    lane: 1, // Highway lane
                    position: ValueOrRange::Value(50.0),
                    speed: ValueOrRange::Value(20.0),
                    acceleration: ValueOrRange::Range([-8.0, 3.0]),
                    direction: 1,
                    behavior: HashMap::new(),
                    lane_changes: vec![],
                    bicycle_params: None,
                },
                ActorSpec {
                    id: "npc".to_string(),
                    role: ActorRole::Npc,
                    lane: 0, // Merge lane
                    position: ValueOrRange::Range([30.0, 40.0]),
                    speed: ValueOrRange::Range([15.0, 18.0]),
                    acceleration: ValueOrRange::Range([-5.0, 4.0]),
                    direction: 1,
                    behavior: HashMap::new(),
                    lane_changes: vec![npc_lane_change],
                    bicycle_params: None,
                },
            ],
            min_ttc: 3.0,
            min_distance: 10.0,
            road: None,
            lane_width: 3.5,
            num_scenarios: 1,
            constraint_modes: ConstraintModes::default(),
            optimization_target: OptimizationTarget::None,
            max_acceleration: None,
            max_deceleration: None,
            max_velocity: None,
            min_velocity: None,
            min_lateral_distance: None,
            max_relative_velocity: None,
            max_lateral_acceleration: 2.0,
            coordinate_system: CoordinateSystem::Cartesian,
            bicycle_config: None,
        }
    }

    #[test]
    fn test_merge_right_validate_success() {
        let model = MergeRightModel;
        let spec = create_test_spec();
        assert!(model.validate(&spec).is_ok());
    }

    #[test]
    fn test_merge_right_validate_wrong_actor_count() {
        let model = MergeRightModel;
        let mut spec = create_test_spec();
        spec.actors.pop();
        assert!(model.validate(&spec).is_err());
    }

    #[test]
    fn test_merge_right_validate_wrong_ego_lane() {
        let model = MergeRightModel;
        let mut spec = create_test_spec();
        spec.actors[0].lane = 0; // Ego in wrong lane
        assert!(model.validate(&spec).is_err());
    }

    #[test]
    fn test_merge_right_validate_wrong_npc_lane() {
        let model = MergeRightModel;
        let mut spec = create_test_spec();
        spec.actors[1].lane = 2; // NPC in wrong lane
        assert!(model.validate(&spec).is_err());
    }

    #[test]
    fn test_merge_right_validate_missing_lane_change() {
        let model = MergeRightModel;
        let mut spec = create_test_spec();
        spec.actors[1].lane_changes = vec![];
        assert!(model.validate(&spec).is_err());
    }

    #[test]
    fn test_merge_right_validate_wrong_lane_change_direction() {
        let model = MergeRightModel;
        let mut spec = create_test_spec();
        spec.actors[1].lane_changes[0].direction = LaneChangeDirection::Left;
        assert!(model.validate(&spec).is_err());
    }

    #[test]
    fn test_merge_right_generate_ltl() {
        let model = MergeRightModel;
        let spec = create_test_spec();
        let formula = model.generate_ltl(&spec);
        assert!(formula.is_ok());

        let formula_str = format!("{}", formula.unwrap());
        assert!(formula_str.contains("InLane"));
        assert!(formula_str.contains("Ahead"));
    }
}
```

**Corresponding YAML**: `examples/merge_right.yaml` (see Step 5 in tutorial)

**Integration Test**: See Step 6 in tutorial.

## 2.10 Troubleshooting Implementation Issues

### Compilation Errors

**Error**: "cannot find type `ScenarioGenError` in this scope"
**Solution**: Add import: `use crate::error::{Result, ScenarioGenError};`

**Error**: "no method named `ego` found for `ScenarioSpec`"
**Solution**: Add import: `use crate::dsl::types::ScenarioSpec;`

**Error**: "trait `ScenarioModel` is not implemented for `MergeRightModel`"
**Solution**: Ensure all trait methods are implemented, especially `generate_ltl()` (required method).

**Error**: "expected `Result<LTLFormula>`, found `LTLFormula`"
**Solution**: Wrap return value in `Ok()`: `Ok(init.and(behavior))`

### UNSAT with New Constraints

**Symptom**: Scenario generates UNSAT after adding custom Z3 constraints.

**Debugging steps**:
1. **Remove custom constraints**: Comment out `add_z3_constraints()` body. Does it work?
2. **Check constraint logic**: Are constraints too restrictive?
3. **Check time windows**: Do constraints apply at impossible time steps?
4. **Check variable bounds**: Are value ranges too narrow?

**Common mistakes**:
```rust
// WRONG: Constraining all time steps to impossible values
for t in 0..=horizon {
    let lane_t = encoder.get_lane_var(npc_id, t);
    backend.assert(&lane_t.eq(&Int::from_i64(0))); // NPC can't change lanes!
}

// RIGHT: Only constrain specific phases
for t in 0..start_step {
    let lane_t = encoder.get_lane_var(npc_id, t);
    backend.assert(&lane_t.eq(&Int::from_i64(0))); // NPC in lane 0 before merge
}
```

### Performance Issues with Complex LTL

**Symptom**: Solving takes >30 seconds or times out.

**Solutions**:
1. **Simplify LTL**: Use fewer temporal operators
   - Replace nested `eventually()` with direct Z3 constraints
   - Replace `always()` with Z3 constraints at specific steps
2. **Reduce horizon**: Use coarser `time_step` or shorter `duration`
3. **Narrow ranges**: More specific value ranges solve faster
4. **Avoid non-linear constraints**: Use linear propositions where possible

**Example optimization**:
```rust
// SLOW: Nested temporal operators
let complex = eventually1.and(eventually2).and(always1).and(always2);

// FAST: Use Z3 for some constraints
let simple = eventually1; // Keep one temporal operator
// Move always1, always2 to add_z3_constraints()
```

### Debugging Z3 Encoding

**Enable verbose logging**:
```bash
cargo run --release -- -i examples/merge_right.yaml -o output/ -v
```

**Add debug prints in scenario code**:
```rust
fn add_z3_constraints(...) -> Result<()> {
    tracing::info!("Adding custom constraints for merge_right");
    tracing::debug!("Merge window: {} to {}", start_step, end_step);

    // ... constraints ...

    Ok(())
}
```

**Check Z3 assertions**: If solving hangs, Z3 may be stuck on conflicting constraints. Simplify incrementally.

---

## Summary

This guide covered:

**Part 1 (YAML Specification)**:
- Complete YAML reference with all required and optional fields
- Actor configuration patterns (ego, NPC, pedestrian, lane change, bicycle model)
- Constraint configuration (safety, velocity, lateral, adversarial modes)
- Complete YAML template and examples by scenario type
- Troubleshooting UNSAT scenarios, validation errors, and performance issues

**Part 2 (Implementing Scenario Types)**:
- Architecture overview (ScenarioType enum, ScenarioModel trait, data flow)
- ScenarioModel trait deep dive (all 4 methods with examples)
- LTL pattern library (21 propositions, temporal operators, common patterns)
- Step-by-step tutorial: Adding "MergeRight" scenario (complete walkthrough)
- Validation patterns, coordinate system integration, testing strategy
- Advanced topics (behavior HashMap, custom Z3 constraints, multi-actor scenarios, optimization)
- Complete code example and troubleshooting

For additional details, see:
- `docs/adversarial-generation.md` - Detailed adversarial generation guide
- `README.md` - User-facing quick start documentation
