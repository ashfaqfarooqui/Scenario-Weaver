# CARLA Scenario Generator

Automatically generate diverse, safety-critical driving test scenarios from high-level specifications using Linear Temporal Logic (LTL) and Z3 SMT solver.

## Features

- Declarative YAML-based scenario specifications
- Automatic constraint solving with Z3
- Built-in safety validation (TTC, minimum distance)
- Multiple diverse scenario generation
- JSON output compatible with CARLA simulator

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
cargo run --release -- -i examples/cut_in_left.yaml -o output.json
```

This will:
1. Parse the YAML specification
2. Generate LTL constraints
3. Solve with Z3
4. Output a scenario JSON file

### Generate Multiple Scenarios

```bash
# Generate 5 different scenarios to a directory
cargo run --release -- -i examples/cut_in_left.yaml -o scenarios/ -n 5
```

This creates `scenarios/scenario_0.json`, `scenarios/scenario_1.json`, etc.

### Enable Verbose Logging

```bash
cargo run --release -- -i examples/cut_in_left.yaml -o output.json -v
```

## YAML Specification Format

Create a YAML file describing your scenario:

```yaml
scenario_type: cut_in_left

# Time configuration
time_step: 0.5        # 0.5 second discretization
duration: 10.0        # 10 second scenario

# Ego vehicle (controlled by AV under test)
ego:
  lane: 1             # right lane
  position: 50.0      # 50 meters from start
  speed: 15.0         # 15 m/s (54 km/h)

# NPC vehicle (background actor)
npc:
  lane: 0             # left lane
  position: [60.0, 80.0]   # start 60-80m from start (Z3 chooses)
  speed: [12.0, 14.0]      # slightly slower (Z3 chooses)
  cut_in_time: [2.5, 7.5]  # cut in between 2.5-7.5 seconds

# Safety constraints
min_ttc: 3.0              # minimum 3 second time-to-collision
min_distance: 5.0         # minimum 5 meter distance
lane_width: 3.5           # 3.5 meter lane width

# Generation settings
num_scenarios: 1          # generate 1 scenario (or use -n flag)
```

## Output Format

Generated scenarios are JSON files with complete actor trajectories:

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

## CLI Options

```bash
carla-scenario-gen [OPTIONS] --input <FILE> --output <PATH>

Options:
  -i, --input <FILE>     Input YAML specification file
  -o, --output <PATH>    Output JSON file (or directory for multiple scenarios)
  -n, --num <NUM>        Number of scenarios to generate (overrides YAML)
  -v, --verbose          Enable verbose logging
  -h, --help             Print help
  -V, --version          Print version
```

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

The generator pipeline:

1. **DSL Parser** (`src/dsl/`) - Parse YAML into structured specification
2. **LTL Generator** (`src/ltl/`) - Convert specification to temporal logic constraints
3. **Z3 Encoder** (`src/solver/`) - Encode LTL + physics + safety into Z3 constraints
4. **Scenario Extractor** (`src/scenario/`) - Extract solution as JSON trajectories

## Documentation

- `Implementation_plan.md`: Master implementation plan
- `design_decisions.md`: Design rationale and alternatives
- `plans/`: Phase-by-phase implementation guides
- `QUICK_START.md`: Getting started guide

## Requirements

- Rust 1.70+
- Z3 SMT solver (installed via cargo)

## License

See LICENSE file
