# Examples

This directory contains 16 YAML scenario specifications covering the full range of ScenarioWeaver features.

## Running an Example

```bash
cargo run --release -- -i examples/cut_in_left.yaml -o output/
```

Each run produces six output files: `.json`, `.xosc`, `.xodr`, `.svg`, `.gif`, `.ol.json`.

---

## Core Scenarios

### cut_in_left.yaml
NPC in the left lane cuts right into the ego's lane on a 3-lane bidirectional road.
Canonical reference example. Generates 5 diverse scenarios.
```bash
cargo run --release -- -i examples/cut_in_left.yaml -o output/
```

### cut_in_right.yaml
Mirror of cut_in_left: NPC in the right lane cuts left into the ego's lane.
```bash
cargo run --release -- -i examples/cut_in_right.yaml -o output/
```

### overtake_left.yaml
NPC overtakes ego using two sequential lane changes: move left into passing lane, accelerate past, then return right. Demonstrates multi-step lane change maneuvers.
```bash
cargo run --release -- -i examples/overtake_left.yaml -o output/
```

### pedestrian_crossing.yaml
Pedestrian crosses a two-lane road while the ego vehicle approaches. The only pedestrian scenario type.
```bash
cargo run --release -- -i examples/pedestrian_crossing.yaml -o output/
```

---

## Bicycle Model Scenarios

These use the kinematic bicycle model (`coordinate_system: bicycle`) with heading tracking and steering constraints.

### bicycle_lane_change.yaml
Cut-in-left scenario using the bicycle model. Demonstrates realistic vehicle dynamics with wheelbase, max steering angle, and steering rate configuration.
```bash
cargo run --release -- -i examples/bicycle_lane_change.yaml -o output/
```

### cut_in_right_bicycle.yaml
Right-side cut-in using the bicycle model on a 3-lane highway. Mirror of `bicycle_lane_change.yaml`.
```bash
cargo run --release -- -i examples/cut_in_right_bicycle.yaml -o output/
```

---

## Adversarial Scenarios

These intentionally violate safety constraints to generate edge cases for AV testing.

### cut_in_left_adversarial_all.yaml
Cut-in with `constraint_modes: violate_all` — both TTC and distance constraints are violated. Generates 5 worst-case scenarios.
```bash
cargo run --release -- -i examples/cut_in_left_adversarial_all.yaml -o output/
```

### cut_in_left_adversarial_ttc.yaml
Selective adversarial: violates TTC while enforcing minimum distance. Demonstrates fine-grained constraint mode control.
```bash
cargo run --release -- -i examples/cut_in_left_adversarial_ttc.yaml -o output/
```

### head_on_collision.yaml
Oncoming NPC on a 4-lane bidirectional road changes into the ego's lane with safety constraints violated. Closing speed ~40 m/s.
```bash
cargo run --release -- -i examples/head_on_collision.yaml -o output/
```

### speed_limit_violation.yaml
Ego exceeds a 22 m/s speed limit (`max_velocity: violate`) while TTC and distance remain enforced. Demonstrates `VelocityGT` proposition.
```bash
cargo run --release -- -i examples/speed_limit_violation.yaml -o output/
```

### unsafe_following.yaml
NPC cuts in with a relative speed >10 m/s above the ego (`max_relative_velocity: violate`). Demonstrates `RelativeVelocityGT` proposition.
```bash
cargo run --release -- -i examples/unsafe_following.yaml -o output/
```

### multi_lane_safety.yaml
NPC lane change violates the lateral distance threshold (`min_lateral_distance: violate`) while TTC and longitudinal distance stay enforced. Demonstrates `LateralDistanceGT` proposition.
```bash
cargo run --release -- -i examples/multi_lane_safety.yaml -o output/
```

---

## Bidirectional Road Scenarios

### simple_bidirectional.yaml
Basic cut-in on a 4-lane bidirectional road (2 forward, 2 backward). Both actors travel in the forward direction.
```bash
cargo run --release -- -i examples/simple_bidirectional.yaml -o output/
```

### head_on_near_miss.yaml
Oncoming NPC on a bidirectional road changes lanes with safety enforced — the safe counterpart to `head_on_collision.yaml`.
```bash
cargo run --release -- -i examples/head_on_near_miss.yaml -o output/
```

### overtake_with_opposite.yaml
NPC cuts into ego's lane on a 3-lane road with one oncoming lane. Tests mixed-direction lane configurations.
```bash
cargo run --release -- -i examples/overtake_with_opposite.yaml -o output/
```

---

## Feature Demonstrations

### with_import.yaml
Cut-in scenario that imports its road definition from `roads/4_lane_bidirectional.yaml`. Demonstrates the road library import feature.
```bash
cargo run --release -- -i examples/with_import.yaml -o output/
```

---

## Coordinate Systems

All examples use one of two coordinate systems set via `coordinate_system` in the YAML:

| System | Variables | Use case |
|--------|-----------|----------|
| `cartesian` (default) | x, y, vx, vy | General scenarios, fast solving |
| `bicycle` | x, y, θ, v, δ | Realistic dynamics with heading and steering |

The bicycle model requires a `bicycle_config` block:
```yaml
coordinate_system: bicycle
bicycle_config:
  default_wheelbase: 2.7
  default_max_steering_angle: 0.6
  default_max_steering_rate: 0.5
```

---

## Output Format

All scenarios (both coordinate systems) produce Cartesian output for compatibility with visualization and simulation tools:

```json
{
  "time": 1.0,
  "position": {"x": 65.0, "y": 1.75},
  "velocity": {"vx": 15.0, "vy": 0.0},
  "acceleration": {"ax": 0.0, "ay": 0.0},
  "lane": 1
}
```
