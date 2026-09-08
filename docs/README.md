# ScenarioWeaver Documentation

← [Back to project README](../README.md)

ScenarioWeaver turns a high-level YAML specification into concrete, safety-critical driving
scenarios: you declare the actors, road, and constraints, and the Z3 solver finds trajectories
that satisfy them. These documents cover how to run it, how to write specifications, how the
system fits together, and how to extend it.

## Start here

| Document | What it covers |
| --- | --- |
| [user-guide.md](user-guide.md) | Install, build, quick start, and the full CLI reference |
| [authoring-scenarios.md](authoring-scenarios.md) | A step-by-step walkthrough: build a cut-in scenario from scratch |

## Writing scenarios

| Document | What it covers |
| --- | --- |
| [yaml-reference.md](yaml-reference.md) | The complete YAML schema, field by field |
| [adversarial-generation.md](adversarial-generation.md) | Enforce / violate / ignore constraint modes and `--adversarial` |
| [optimizer.md](optimizer.md) | Optimization targets (minimize/maximize TTC, distance, severity) |
| [coordinate-systems.md](coordinate-systems.md) | Cartesian vs bicycle motion models, and the pedestrian sub-model |
| [output-formats.md](output-formats.md) | The six output files: JSON, OpenSCENARIO, OpenDRIVE, SVG, GIF, OpenLABEL |

## Understanding and extending the system

| Document | What it covers |
| --- | --- |
| [architecture.md](architecture.md) | The end-to-end pipeline, the encoder plugin system, and how the solver works |
| [creating-scenario-types.md](creating-scenario-types.md) | Adding a new scenario type in Rust |
| [z3_constraints.md](z3_constraints.md) | Advanced contributor reference: the SMT constraint encoding in detail |
