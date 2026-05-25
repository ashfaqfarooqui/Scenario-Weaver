# Encoder Test Coverage Analysis

## Module Overview

`src/solver/encoder.rs` implements `GenericEncoder<B>`, the facade that orchestrates all Z3 SMT constraint encoding. It dispatches coordinate-specific work (kinematics, variables, lane coupling) to a `Box<dyn CoordinateEncoder<B>>` and directly handles:

- LTL formula encoding (temporal operators expanded over discrete time steps)
- Proposition encoding (atomic constraints like TTC, distance, velocity)
- Safety constraint encoding (direct Z3 assertions for Enforce mode)
- TTC computation (two-case closing-speed logic)
- Validation metrics (post-solve TTC/distance/acceleration analysis)
- Optimization objectives (min-TTC, min/max-distance)

## Current Test Coverage

### Unit tests in `src/solver/encoder.rs`

The inline `#[cfg(test)]` module contains 13 tests:

| Test | What it verifies |
|------|-----------------|
| `test_encoder_creation` | Horizon = duration / time_step |
| `test_create_variables` | Accessor methods (`get_longitudinal_pos`, etc.) return valid Z3 variables |
| `test_encode_initial_conditions` | Z3 model contains correct initial position/velocity values |
| `test_lane_position_coupling` | Lateral position = lane * lane_width + lane_width/2 |
| `test_kinematics` | px[t+1] = px[t] + vx[t] * dt holds in Z3 model |
| `test_ltl_encoding_simple` | InLane atom is satisfiable |
| `test_ltl_encoding_eventually` | F(InLane) is satisfiable |
| `test_ltl_encoding_always` | G(InLane) is satisfiable |
| `test_ltl_encoding_until` | Until operator semantics (phi holds until psi) |
| `test_safety_constraints` | Basic safety encoding is satisfiable |
| `test_full_cut_in_scenario` | Full scenario with LTL generation solves successfully |
| `test_scenario_extraction` | Extracted scenario serializes to valid JSON |
| `test_velocity_propositions_linear` | VelocityGT and VelocityLT encode correctly |

### Integration tests in `tests/`

| File | Coverage area |
|------|--------------|
| `integration_test.rs` | End-to-end YAML to scenario pipeline |
| `comprehensive_test.rs` | All scenario types, adversarial modes, all exporters |
| `cartesian_physics_test.rs` | Velocity ratio constraints, heading angle bounds |
| `bidirectional_test.rs` | Backward lane velocity constraints |

### Related unit tests in other modules

- `src/solver/backend.rs` -- SolverBackend and OptimizerBackend trait implementations
- `src/solver/multi_solve.rs` -- Blocking clauses and multi-scenario diversity
- `src/solver/encoder_utils.rs` -- `LaneChangeSteps` struct construction

## Identified Coverage Gaps

### Proposition encoding

Only `VelocityGT` and `VelocityLT` have direct unit tests. The following propositions lack Z3-backed verification:

- `DistanceGT` (longitudinal distance threshold)
- `TTCGT` (time-to-collision threshold)
- `LateralDistanceGT` (perpendicular separation)
- `OnLeftOf` / `OnRightOf` (lateral ordering)
- `RelativeVelocityGT` (speed difference)
- `ManhattanDistanceGT` (L1 distance)
- `RectangularDistanceGT` (box-shaped safety zone)
- `Distance2DGT` (Euclidean distance -- quadratic)
- `PedestrianTTCGT` (perpendicular crossing TTC)
- `OnSidewalk` / `CrossingRoad` (pedestrian position)

### TTC constraint logic

The two-case TTC encoding (actor1 behind actor2, actor2 behind actor1) is only exercised implicitly through full scenario generation. No isolated test verifies the closing-speed formula or the conditional branching.

### Validation metrics

`compute_validation_metrics` computes min TTC, min distance, and acceleration violations from a solved model. It has no direct unit tests -- correctness is only checked indirectly via integration tests that assert `all_constraints_satisfied`.

### Optimizer encoder

`encode_objective` (used with `--optimize min-ttc`) and the distance-based objectives have no direct tests. The `OptimizerBackend` trait is tested in `backend.rs`, but the objective encoding logic in `encoder.rs` is not.

### Encoder utils Z3 correctness

Helper functions `encode_y_proximity_constraint` and `encode_same_lane_constraint` (used for TTC applicability) have no Z3-backed tests verifying their constraint semantics.

### Edge cases

- LTL `Next` operator at the horizon boundary (t = horizon - 1)
- UNSAT detection when constraints are mutually exclusive
- Empty actor list or single-actor scenarios

## Test Plan

Planned additions (16 new tests):

**Group 1: Proposition encoding correctness (7 tests)**
- `test_distance_gt_proposition` -- DistanceGT satisfiable/unsatisfiable cases
- `test_lateral_distance_gt_proposition` -- LateralDistanceGT encoding
- `test_on_left_of_proposition` -- OnLeftOf lateral ordering
- `test_on_right_of_proposition` -- OnRightOf lateral ordering
- `test_relative_velocity_gt_proposition` -- RelativeVelocityGT encoding
- `test_manhattan_distance_gt_proposition` -- ManhattanDistanceGT encoding
- `test_rectangular_distance_gt_proposition` -- RectangularDistanceGT box constraint

**Group 2: TTC constraint encoding (2 tests)**
- `test_ttc_gt_closing_speed` -- TTC formula with known closing speed
- `test_ttc_gt_diverging_actors` -- TTC infinite when actors move apart

**Group 3: Validation metrics (3 tests)**
- `test_validation_metrics_safe_scenario` -- all_constraints_satisfied = true
- `test_validation_metrics_violation_detected` -- violation timestamps populated
- `test_validation_metrics_acceleration_bounds` -- max_acceleration violation flagged

**Group 4: Optimizer encoder (2 tests)**
- `test_optimize_min_ttc` -- objective minimizes TTC value
- `test_optimize_max_distance` -- objective maximizes separation

**Group 5: Edge cases (2 tests)**
- `test_ltl_next_at_horizon_boundary` -- Next at last step does not panic
- `test_conflicting_constraints_unsat` -- contradictory spec returns UNSAT

## Running Tests

```bash
# All solver encoder unit tests
cargo test --lib -- solver::encoder::tests

# All tests including integration
cargo test

# With output
cargo test -- --nocapture

# Single test by name
cargo test test_velocity_propositions_linear
```
