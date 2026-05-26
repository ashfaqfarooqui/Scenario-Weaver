# Optimizer

## Overview

The optimizer component finds *optimal* scenarios by replacing the standard Z3 `Solver` with Z3 `Optimize`. While the normal generation path finds any satisfying assignment, the optimizer steers the solver toward an extremal value of a chosen metric (e.g., minimum distance between actors, worst time-to-collision proxy).

Use the optimizer when you need a specific boundary scenario rather than an arbitrary valid one — for example, the closest possible approach that still satisfies all constraints, or the safest scenario with the largest gap.

## Usage

### CLI

```sh
scenario-weaver generate --optimize <target> scenario.yaml
```

### YAML field

```yaml
optimization_target: min-distance
```

### Available targets

| CLI value | Description |
|-----------|-------------|
| `min-distance` | Minimize the closest physical approach between actors |
| `min-ttc` | Minimize a linear proxy of time-to-collision |
| `min-severity` | Maximize relative approach speed (most severe interaction) |
| `max-ttc` | Maximize the minimum gap (safest scenario) |

## Optimization Targets

| Target | CLI Value | Objective (LRA) | What It Finds |
|--------|-----------|-----------------|---------------|
| MinimizeDistance | `min-distance` | minimize min\_t \|px\_i - px\_j\| when same lane | Closest physical approach between actors |
| MinimizeTtc | `min-ttc` | minimize (distance - dt * closing\_speed) | Scenario with worst time-to-collision proxy (small gap + fast approach) |
| MinimizeSeverity | `min-severity` | maximize \|vx\_i - vx\_j\| when same lane | Scenario with highest relative approach speed |
| MaximizeTtc | `max-ttc` | maximize min\_t \|px\_i - px\_j\| when same lane | Safest scenario (largest minimum gap) |

## Design Decisions

### LRA only

All objectives are expressed in Linear Real Arithmetic. True TTC requires dividing distance by closing speed, which introduces Non-linear Real Arithmetic (NRA). NRA makes Z3 significantly slower and less predictable. The `min-ttc` target uses a linear proxy instead.

### Single scalar objective

Each target optimizes exactly one scalar value. Lexicographic multi-priority objectives (e.g., minimize distance then maximize severity) are not yet supported.

### EncoderAccessor trait

The `EncoderAccessor` trait provides backend-agnostic access to Z3 variables (positions, velocities, time-step constants). This allows the same scenario-specific constraint logic to work with both the `Solver` and `Optimizer` backends without duplication.

## How It Works

1. The encoding pipeline runs identically to the normal path: kinematics, LTL temporal constraints, and scenario-specific constraints are all asserted.
2. After all constraints are in place, the optimizer adds an objective function corresponding to the chosen target.
3. Z3 `Optimize` searches for a satisfying assignment that extremizes the objective.
4. The optimal value is extracted from the resulting Z3 model and reported alongside the generated scenario.

## Known Limitations

- Each target optimizes a single scalar value. Future work: lexicographic multi-priority objectives.
- `min-ttc` is a linear proxy, not actual TTC (actual TTC requires NRA division).
- Multi-scenario diversity (`-n` flag) is not supported in optimizer mode.
- The optimizer may be slower than plain SAT solving for complex scenarios with many actors or long time horizons.

## Architecture

Key files:

| File | Contents |
|------|----------|
| `src/solver/backend.rs` | `OptimizerBackend` struct wrapping Z3 `Optimize` |
| `src/solver/encoder.rs` | `impl GenericEncoder<OptimizerBackend>` with objective encoding |
| `src/solver/encoder.rs` | `EncoderAccessor` trait for backend-agnostic variable access |
| `src/lib.rs` | `generate_with_optimizer()` orchestration entry point |
