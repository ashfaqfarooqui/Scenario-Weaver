# CARLA Scenario Generator

Generate driving test scenarios from high-level specifications using LTL + Z3.

## Status

Under Development - See Implementation_plan.md for roadmap

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
