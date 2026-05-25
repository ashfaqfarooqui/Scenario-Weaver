# ScenarioWeaver User Guide

ScenarioWeaver generates diverse, safety-critical driving test scenarios from declarative YAML specifications. You describe what you want -- actors, roads, constraints, lane changes -- and the tool uses Linear Temporal Logic (LTL) and the Z3 SMT solver to find concrete trajectories that satisfy (or intentionally violate) your constraints.

Key capabilities:

- **Declarative specs**: describe scenarios in YAML, not code
- **Constraint solving**: Z3 finds physically valid trajectories automatically
- **Adversarial generation**: intentionally violate safety constraints to find edge cases
- **Optimization**: find worst-case scenarios (minimum TTC, closest approach)
- **Six output formats**: JSON, OpenSCENARIO, OpenDRIVE, SVG, GIF, OpenLabel
- **Multiple scenarios**: generate diverse variants from a single spec using blocking clauses

This guide is for testing engineers, simulation developers, and anyone who wants to generate driving scenarios without writing solver code.

---

## Installation

### Prerequisites

- Rust 1.70+ (install via [rustup](https://rustup.rs/))
- Z3 SMT solver system library

### Install Z3

Ubuntu/Debian:

```bash
sudo apt-get install libz3-dev
```

macOS:

```bash
brew install z3
```

### Build ScenarioWeaver

```bash
git clone <repository-url>
cd ScenarioGenerationWorkspace/main
cargo build --release
```

Verify the build:

```bash
cargo test
```

The binary is at `target/release/scenario-weaver`.

---

## Quick Start

Generate a scenario from one of the bundled examples:

```bash
cargo run --release -- -i examples/cut_in_left.yaml -o my_first_scenario/
```

This produces six files in `my_first_scenario/`:

| File | What it contains |
|------|-----------------|
| `scenario_1.json` | Full trajectory data, validation metrics |
| `scenario_1.xosc` | OpenSCENARIO XML for simulator import |
| `scenario_1.xodr` | OpenDRIVE road network |
| `scenario_1.svg` | Static visualization (open in browser) |
| `scenario_1.gif` | Animated visualization at 10 FPS |
| `scenario_1.ol.json` | OpenLabel metadata |

Open the SVG in a browser to see the complete trajectories. Open the GIF to watch the scenario play out.

---

## Understanding Output Formats

### JSON (.json)

Complete machine-readable scenario data. Contains:

- **Metadata**: scenario ID (UUID), type, duration, time step
- **Actor trajectories**: position `(x, y)`, velocity `(vx, vy)`, acceleration, lane assignment at every time step
- **Validation**: minimum TTC, minimum distance, whether all constraints were satisfied
- **Violations**: list of constraint violations with timestamps (useful for adversarial scenarios)

Use this for post-processing, analysis pipelines, or feeding into custom tools.

### OpenSCENARIO (.xosc)

Industry-standard format for driving scenario description. Compatible with CARLA and other OpenSCENARIO-compliant simulators. Contains vehicle entities with trajectory-based actions and a storyboard with stop trigger based on scenario duration.

Note: pedestrians are exported as vehicles due to a limitation in the openscenario-rs library.

### OpenDRIVE (.xodr)

Industry-standard road network description. Describes the straight road geometry with lane structure matching your road spec. Pair with the `.xosc` file for a complete simulation setup.

### SVG (.svg)

Static vector graphic showing the road layout, lane markings, complete vehicle trajectories, and safety metrics. Opens in any browser. Good for reports and documentation.

### GIF (.gif)

Animated visualization at 10 FPS showing vehicles moving through the scenario. Includes real-time metrics overlay (current time, TTC, distance, constraint status) and violation highlighting. Typically ~900KB for a 10-second scenario. Works in browsers, Slack, GitHub, email.

### OpenLabel (.ol.json)

OpenLabel 1.0.0 metadata with semantic tags for scenario type, actor roles, road type, and behaviors. Useful for organizing generated scenario datasets.

---

## Writing Your First YAML Scenario

A scenario spec has five parts: time configuration, road definition, actors, safety constraints, and generation settings.

Here is a complete working example -- a cut-in scenario where an NPC changes from the left lane into the ego vehicle's lane:

```yaml
scenario_type: cut_in_left

# Time configuration
time_step: 0.1          # seconds between trajectory points
duration: 10.0           # total scenario length in seconds

# Road definition
road:
  num_lanes: 2
  lane_width: 3.5        # meters
  lane_directions: [1, 1]  # both lanes go forward

# Actors
actors:
  - id: ego
    role: ego
    lane: 1              # right lane (0-indexed)
    position: [40.0, 60.0]  # Z3 picks a value in this range (meters)
    speed: [14.0, 16.0]     # Z3 picks a value in this range (m/s)
    direction: 1             # forward
    acceleration: [-8.0, 3.0]  # allowed acceleration range (m/s^2)

  - id: npc
    role: npc
    lane: 0              # left lane
    position: [50.0, 80.0]
    speed: [16.0, 20.0]
    direction: 1
    acceleration: [-8.0, 3.0]

    lane_changes:
      - direction: right        # cut from lane 0 to lane 1
        start_time: [2.0, 4.0]  # when the lane change begins
        duration: [3.0, 4.0]    # how long the transition takes

# Safety constraints
min_ttc: 3.0       # minimum time-to-collision (seconds)
min_distance: 5.0  # minimum longitudinal distance (meters)

# How many scenarios to generate
num_scenarios: 1
```

Run it:

```bash
cargo run --release -- -i my_scenario.yaml -o output/
```

### Key concepts

**Ranges vs fixed values**: Use `position: 50.0` for a fixed value or `position: [40.0, 60.0]` for a range. When you give a range, Z3 chooses a value that satisfies all constraints.

**Lanes are 0-indexed**: Lane 0 is the leftmost lane. Lane directions are specified per-lane: `1` = forward, `-1` = backward.

**Lane changes**: Defined as a list under `lane_changes`. Each entry has a `direction` (left/right), `start_time`, and `duration`. Multiple entries create sequential lane changes (e.g., for overtaking).

---

## CLI Reference

```
scenario-weaver [OPTIONS] --input <FILE> --output <DIR>

Options:
  -i, --input <FILE>       Input YAML specification file (required)
  -o, --output <DIR>       Output directory for generated scenarios (required)
  -n, --num <N>            Number of scenarios to generate (overrides YAML)
  -v, --verbose            Enable verbose/debug logging
      --adversarial        Override all constraint modes to violate
      --optimize <TARGET>  Optimization target: min-ttc, min-distance,
                           min-severity, max-ttc
  -h, --help               Print help
  -V, --version            Print version
```

### Common usage patterns

Generate a single scenario:

```bash
cargo run --release -- -i spec.yaml -o output/
```

Generate 10 diverse scenarios from one spec:

```bash
cargo run --release -- -i spec.yaml -o output/ -n 10
```

Generate adversarial scenarios (violate all safety constraints):

```bash
cargo run --release -- -i spec.yaml -o output/ --adversarial
```

Find the worst-case scenario (minimum TTC):

```bash
cargo run --release -- -i spec.yaml -o output/ --optimize min-ttc
```

Debug a failing spec with verbose logging:

```bash
cargo run --release -- -i spec.yaml -o output/ -v
```

---

## Scenario Types

### cut_in_left

NPC starts in a lane to the left of ego and cuts into ego's lane.

```yaml
scenario_type: cut_in_left

actors:
  - id: ego
    role: ego
    lane: 1
    position: [40.0, 60.0]
    speed: [14.0, 16.0]
    direction: 1
    acceleration: [-8.0, 3.0]

  - id: npc
    role: npc
    lane: 0                    # left of ego
    position: [50.0, 80.0]
    speed: [16.0, 20.0]
    direction: 1
    acceleration: [-8.0, 3.0]
    lane_changes:
      - direction: right       # cuts right into ego's lane
        start_time: [2.0, 4.0]
        duration: [3.0, 4.0]
```

### cut_in_right

NPC starts in a lane to the right of ego and cuts into ego's lane.

```yaml
scenario_type: cut_in_right

actors:
  - id: ego
    role: ego
    lane: 0                    # ego in left lane
    position: [45.0, 55.0]
    speed: [14.0, 16.0]
    direction: 1
    acceleration: [-8.0, 3.0]

  - id: npc
    role: npc
    lane: 1                    # right of ego
    position: [60.0, 80.0]
    speed: [12.0, 14.0]
    direction: 1
    acceleration: [-8.0, 3.0]
    lane_changes:
      - direction: left        # cuts left into ego's lane
        start_time: [2.5, 7.5]
        duration: [3.0, 4.0]
```

### overtake_left

NPC starts behind ego in the same lane, moves to the left passing lane, overtakes, and returns to the original lane. Requires two sequential lane changes.

```yaml
scenario_type: overtake_left

time_step: 0.5
duration: 12.0               # longer duration for complete maneuver

road:
  num_lanes: 3
  lane_width: 3.5
  lane_directions: [1, 1, 1]

actors:
  - id: ego
    role: ego
    lane: 1
    position: 50.0
    speed: [14.0, 16.0]
    direction: 1
    acceleration: [-2.0, 1.0]

  - id: npc
    role: npc
    lane: 1                    # same lane as ego, starts behind
    position: [25.0, 35.0]
    speed: [18.0, 22.0]       # faster than ego
    direction: 1
    acceleration: [-3.0, 4.0]
    lane_changes:
      - direction: left        # move to passing lane
        start_time: [2.0, 3.0]
        duration: [1.5, 2.0]
      - direction: right       # return to original lane
        start_time: [7.0, 8.0]
        duration: [1.5, 2.0]
```

### pedestrian_crossing

Pedestrian crosses the road while ego approaches. Pedestrians have special configuration for walking mode and crossing direction.

```yaml
scenario_type: pedestrian_crossing

time_step: 0.3
duration: 10.0

road:
  num_lanes: 2
  lane_width: 3.5
  lane_directions: [1, 1]

actors:
  - id: ego
    role: ego
    lane: 0
    position: [0.0, 40.0]
    speed: [8.0, 12.0]
    direction: 1
    acceleration: [-5.0, 2.0]

  - id: pedestrian
    role: pedestrian
    lane: 0                    # ignored for pedestrians
    position: [10.0, 60.0]
    speed: [0.8, 1.5]         # walking speed
    direction: 1
    acceleration: [-1.0, 1.0]
    behavior:
      walking_mode: walk       # walk, run, or hesitate
      direction: left_to_right # or right_to_left
```

---

## Constraint Modes and Adversarial Generation

Every safety constraint operates in one of three modes:

| Mode | LTL encoding | Meaning |
|------|-------------|---------|
| `enforce` | `G(constraint)` | Constraint must hold at every time step (default) |
| `violate` | `F(NOT constraint)` | Constraint must be violated at least once |
| `ignore` | omitted | Constraint is not included |

### Per-constraint control

Set modes individually in YAML:

```yaml
min_ttc: 3.0
min_distance: 5.0
max_velocity: 22.0

constraint_modes:
  min_ttc: enforce           # always safe TTC
  min_distance: violate      # find scenarios that get too close
  max_velocity: ignore       # don't care about speed
```

The following optional threshold fields activate their constraint when present:

| Field | Mode-controlled? | Description |
|-------|-----------------|-------------|
| `max_velocity` | Yes (`constraint_modes.max_velocity`) | Global speed limit (m/s) |
| `min_velocity` | Yes (`constraint_modes.min_velocity`) | Global minimum speed (m/s) |
| `min_lateral_distance` | Yes (`constraint_modes.min_lateral_distance`) | Side-by-side clearance (m) |
| `max_relative_velocity` | Yes (`constraint_modes.max_relative_velocity`) | Max speed difference between actors (m/s) |
| `max_acceleration` | Yes (`constraint_modes.max_acceleration`) | Max longitudinal acceleration (m/s²) |
| `max_deceleration` | No — always enforced when present | Max deceleration (must be negative, e.g. `-8.0`) |
| `max_lateral_acceleration` | No — always enforced when present | Max lateral acceleration during lane changes (default: `2.0` m/s²) |

### Shorthand modes

Apply the same mode to all constraints:

```yaml
constraint_modes: enforce_all   # all constraints enforced
constraint_modes: violate_all   # all constraints violated
constraint_modes: ignore_all    # no constraints
```

### CLI override

The `--adversarial` flag overrides all constraint modes to `violate`:

```bash
cargo run --release -- -i spec.yaml -o output/ --adversarial
```

### Use cases

- **Testing emergency braking**: violate `min_ttc` to generate near-collision scenarios
- **Edge case discovery**: violate `min_distance` to find close-approach situations
- **ML training data**: generate both safe and unsafe scenarios for classifier training
- **Speed limit testing**: violate `max_velocity` while enforcing other constraints

---

## Coordinate Systems

### Cartesian (default)

Simple `(x, y)` coordinate system with linear kinematics. Fast to solve. Good for most highway and urban scenarios.

```yaml
coordinate_system: cartesian   # or just omit this line
```

### Bicycle

Kinematic bicycle model with heading tracking: `(x, y, theta, v)`. Adds steering angle constraints, turn radius enforcement, and heading bounds. Use when you need realistic vehicle dynamics.

```yaml
coordinate_system: bicycle

bicycle_config:
  default_wheelbase: 2.7              # meters
  default_max_steering_angle: 0.6     # radians (~34 degrees)
  default_max_steering_rate: 0.5      # rad/s

actors:
  - id: ego
    role: ego
    lane: 1
    position: 50.0
    speed: 15.0
    direction: 1
    acceleration: [-8.0, 3.0]
    # Uses defaults from bicycle_config

  - id: npc
    role: npc
    lane: 0
    position: [60.0, 80.0]
    speed: [16.0, 20.0]
    direction: 1
    acceleration: [-8.0, 3.0]
    # Override defaults for this actor:
    bicycle_params:
      wheelbase: 2.9
      max_steering_angle: 0.5
      max_steering_rate: 0.4
```

The bicycle model uses small-angle approximations valid for heading angles under 30 degrees. It may return UNSAT if lane change durations are too short for the vehicle's minimum turn radius.

---

## Road Import

Reuse road definitions across scenarios by importing from external YAML files in the `roads/` directory:

```yaml
imports:
  - roads/3_lane_highway.yaml

scenario_type: cut_in_left
# ... rest of scenario spec (no need to define road: block)
```

Available road definitions:

| File | Description |
|------|-------------|
| `roads/2_lane_rural.yaml` | 2-lane rural road |
| `roads/3_lane_highway.yaml` | 3-lane highway (2 forward, 1 backward) |
| `roads/4_lane_bidirectional.yaml` | 4-lane bidirectional (2 forward, 2 backward) |

A road definition file looks like this:

```yaml
num_lanes: 3
lane_width: 3.75
lane_directions: [1, 1, -1]
```

---

## Optimization Mode

Instead of finding any satisfying scenario, optimization mode finds the best (or worst) one:

```bash
cargo run --release -- -i spec.yaml -o output/ --optimize min-ttc
```

| Target | What it finds |
|--------|--------------|
| `min-ttc` | Scenario with smallest time-to-collision (worst-case near-miss) |
| `min-distance` | Scenario with closest approach distance |
| `min-severity` | Minimizes both TTC and distance (weighted combination) |
| `max-ttc` | Safest possible scenario (maximum TTC) |

Optimization uses Z3's Optimize solver instead of the standard Solver. It may take longer than standard generation.

---

## Using the Rust API

Add ScenarioWeaver as a dependency and use it programmatically:

### Generate from YAML string

```rust
use scenario_weaver::generate_single_scenario;

let yaml = std::fs::read_to_string("scenario.yaml").unwrap();
let scenario = generate_single_scenario(&yaml).unwrap();

println!("Scenario ID: {}", scenario.id);
println!("Min TTC: {:?}", scenario.validation.as_ref().map(|v| v.min_ttc));
```

### Generate from a pre-parsed spec

```rust
use scenario_weaver::{generate_single_scenario_from_spec, dsl};

let yaml = std::fs::read_to_string("scenario.yaml").unwrap();
let mut spec = dsl::parser::parse_yaml(&yaml).unwrap();

// Modify the spec programmatically
spec.duration = 15.0;

let scenario = generate_single_scenario_from_spec(spec).unwrap();
```

### Generate multiple diverse scenarios

```rust
use scenario_weaver::generate_multiple_scenarios;

let yaml = std::fs::read_to_string("scenario.yaml").unwrap();
let scenarios = generate_multiple_scenarios(&yaml, 5, None::<fn(usize, &_) -> _>).unwrap();

println!("Generated {} scenarios", scenarios.len());
```

### Export to all formats

```rust
use scenario_weaver::{
    generate_single_scenario,
    export_scenario_to_xosc, export_scenario_to_xodr,
    export_scenario_to_svg, export_scenario_to_gif,
    export_scenario_to_openlabel,
};

let yaml = std::fs::read_to_string("scenario.yaml").unwrap();
let scenario = generate_single_scenario(&yaml).unwrap();

let json = serde_json::to_string_pretty(&scenario).unwrap();
std::fs::write("scenario.json", json).unwrap();

let xosc = export_scenario_to_xosc(&scenario).unwrap();
std::fs::write("scenario.xosc", xosc).unwrap();

let xodr = export_scenario_to_xodr(&scenario).unwrap();
std::fs::write("scenario.xodr", xodr).unwrap();

let svg = export_scenario_to_svg(&scenario).unwrap();
std::fs::write("scenario.svg", svg).unwrap();

let gif = export_scenario_to_gif(&scenario).unwrap();
std::fs::write("scenario.gif", gif).unwrap();

let openlabel = export_scenario_to_openlabel(&scenario).unwrap();
std::fs::write("scenario.ol.json", openlabel).unwrap();
```

### Custom GIF resolution

```rust
use scenario_weaver::{export_scenario_to_gif_with_resolution, Resolution};

let gif = export_scenario_to_gif_with_resolution(&scenario, Resolution::High).unwrap();
```

### Link OpenSCENARIO with OpenDRIVE

```rust
use scenario_weaver::export_scenario_to_xosc_with_road_file;

let xosc = export_scenario_to_xosc_with_road_file(&scenario, "scenario.xodr").unwrap();
```

---

## Troubleshooting

### UNSAT (no solution found)

The solver could not find trajectories satisfying all constraints. Common causes:

- **Conflicting constraints**: e.g., enforcing min TTC of 10s with actors starting 5m apart at high speed
- **Ranges too narrow**: widen position, speed, or lane change timing ranges
- **Physically impossible**: lane change duration too short for the speed (especially with bicycle model)
- **Adversarial + enforce conflict**: violating one constraint may make another impossible to enforce

Fix: widen ranges, relax constraints, increase duration, or use `ignore` mode for non-essential constraints.

### Slow solving

- **Reduce time steps**: use `time_step: 0.5` instead of `0.1` (20 steps vs 100 for a 10s scenario)
- **Shorten duration**: 10s scenarios solve faster than 30s
- **Use cartesian**: the bicycle model adds non-linear constraints that slow solving
- **Simplify constraints**: fewer constraint modes = faster solving

### Missing Z3 library

```
error: could not find native static library `z3`
```

Install the Z3 development library for your platform (see Installation section above).

### Validation errors in YAML

- Every scenario needs at least one actor with `role: ego`
- `lane_directions` length must match `num_lanes`
- Lane indices must be within `0..num_lanes`
- `direction` must be `1` (forward) or `-1` (backward)
- Bicycle model requires `bicycle_config` at scenario level or `bicycle_params` per actor

---

## Example Gallery

All examples are in the `examples/` directory.

| File | Description |
|------|-------------|
| `cut_in_left.yaml` | Basic cut-in from left lane, 3-lane road |
| `cut_in_right.yaml` | Cut-in from right lane, 2-lane road |
| `cut_in_left_adversarial_all.yaml` | Cut-in with all constraints violated |
| `cut_in_left_adversarial_ttc.yaml` | Cut-in with only TTC violated |
| `cut_in_right_bicycle.yaml` | Right cut-in, bicycle model |
| `overtake_left.yaml` | NPC overtakes ego via left lane (two lane changes) |
| `overtake_with_opposite.yaml` | Overtake with oncoming traffic |
| `pedestrian_crossing.yaml` | Pedestrian crosses road |
| `bicycle_lane_change.yaml` | Bicycle model lane change |
| `speed_limit_violation.yaml` | Adversarial speed limit violation |
| `multi_lane_safety.yaml` | Lateral distance constraints |
| `unsafe_following.yaml` | Relative velocity constraints |
| `simple_bidirectional.yaml` | Bidirectional road scenario |
| `head_on_collision.yaml` | Head-on collision scenario |
| `head_on_near_miss.yaml` | Head-on near-miss |
| `with_import.yaml` | Road import from external file |

---

## Further Reading

- [CREATING_SCENARIOS.md](CREATING_SCENARIOS.md) -- guide for extending ScenarioWeaver with new scenario types and constraints
- [docs/z3_constraints.md](z3_constraints.md) -- detailed reference for Z3 constraint encoding internals
