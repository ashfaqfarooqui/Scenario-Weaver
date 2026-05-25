# ScenarioWeaver

Automatically generate diverse, safety-critical driving test scenarios from high-level YAML specifications using Linear Temporal Logic (LTL) and the Z3 SMT solver.

## Features

- **Declarative YAML specs** — describe actors, lanes, speeds, and constraints; Z3 finds valid trajectories
- **Safety validation** — built-in TTC and minimum-distance checks at every time step
- **Adversarial generation** — intentionally violate constraints to test edge cases and failure modes
- **Per-constraint control** — enforce, violate, or ignore each constraint independently
- **Multiple coordinate systems** — Cartesian (x, y) and kinematic Bicycle (x, y, θ, v)
- **Multi-scenario diversity** — blocking clauses force structurally different solutions
- **Six output formats** — JSON, OpenSCENARIO, OpenDRIVE, SVG, GIF, OpenLabel

## Requirements

- Rust 1.70+
- Z3 SMT solver (pulled automatically via Cargo)

## Install

```bash
git clone <repo-url>
cd ScenarioGenerationWorkspace/main
cargo build --release
```

## Quick Start

```bash
# Single scenario → output/scenario.{json,xosc,xodr,svg,gif,ol.json}
cargo run --release -- -i examples/cut_in_left.yaml -o output/

# Five diverse scenarios
cargo run --release -- -i examples/cut_in_left.yaml -o scenarios/ -n 5

# Adversarial (violate all safety constraints)
cargo run --release -- -i examples/cut_in_left.yaml -o adversarial/ --adversarial

# Bicycle model
cargo run --release -- -i examples/bicycle_lane_change.yaml -o output/

# Verbose logging
cargo run --release -- -i examples/cut_in_left.yaml -o output/ -v
```

## CLI Options

```
scenario-weaver [OPTIONS] --input <FILE> --output <DIR>

Options:
  -i, --input <FILE>          Input YAML specification file
  -o, --output <DIR>          Output directory
  -n, --num <NUM>             Number of scenarios to generate (overrides YAML)
  -v, --verbose               Enable verbose logging
      --adversarial           Override all constraint modes to violate
      --optimize <TARGET>     Optimization target: min-ttc | min-distance | min-severity | max-ttc
  -h, --help                  Print help
  -V, --version               Print version
```

## YAML Specification

```yaml
scenario_type: cut_in_left
time_step: 0.1
duration: 10.0

actors:
  - id: ego
    role: ego
    lane: 1
    position: 50.0          # fixed value
    speed: 15.0
    direction: 1
    acceleration: [-8.0, 3.0]

  - id: npc
    role: npc
    lane: 0
    position: [60.0, 80.0]  # range — Z3 chooses
    speed: [16.0, 20.0]
    direction: 1
    acceleration: [-8.0, 3.0]
    lane_changes:
      - direction: right
        start_time: [2.5, 3.5]
        duration: [3.0, 4.0]

road:
  num_lanes: 2
  lane_width: 3.5
  lane_directions: [1, 1]   # 1 = forward, -1 = backward

min_ttc: 3.0
min_distance: 5.0
```

**Value formats:**
- `position: 50.0` — Z3 must use exactly 50.0
- `position: [45.0, 55.0]` — Z3 chooses any value in range

Road definitions can be shared across files using `imports: [roads/3_lane_highway.yaml]`.

## Output Formats

Outputs JSON, OpenSCENARIO (.xosc), OpenDRIVE (.xodr), SVG, GIF, and OpenLabel (.ol.json) automatically.
See [docs/output-formats.md](docs/output-formats.md) for format details and programmatic Rust API.

## Development

```bash
# Run all tests
cargo test

# Unit tests only
cargo test --lib

# Integration tests only
cargo test --test integration_test

# Linting and formatting
cargo clippy
cargo fmt

# Build API docs
cargo doc --open
```

## Documentation

| Document | Contents |
|----------|----------|
| [docs/output-formats.md](docs/output-formats.md) | JSON schema, XOSC/XODR/SVG/GIF/OpenLabel details, Rust API |
| [docs/adversarial-generation.md](docs/adversarial-generation.md) | Constraint modes, YAML config, use cases |
| [docs/coordinate-systems.md](docs/coordinate-systems.md) | Cartesian vs Bicycle model, dynamics, configuration |
| [docs/scenario-types.md](docs/scenario-types.md) | Adding new scenario types in Rust (step-by-step) |
| [docs/architecture.md](docs/architecture.md) | Pipeline, encoder architecture, module map |
| [docs/z3_constraints.md](docs/z3_constraints.md) | Z3 constraint reference for both encoders |
| [CREATING_SCENARIOS.md](docs/CREATING_SCENARIOS.md) | YAML specification guide and scenario authoring |

## License

See LICENSE file.

## Acknowledgement

<p align="center">
<img src="assets/synergies.svg" alt="Synergies logo" width="200"/>
</p>

This package is developed as part of the [SYNERGIES](https://synergies-ccam.eu/) project.

<p align="center">
<img src="assets/funded_by_eu.svg" alt="Funded by EU" width="200"/>
</p>

Funded by the European Union. Views and opinions expressed are however those of the author(s) only and do not necessarily reflect those of the European Union or the European Climate, Infrastructure and Environment Executive Agency (CINEA). Neither the European Union nor the granting authority can be held responsible for them.
