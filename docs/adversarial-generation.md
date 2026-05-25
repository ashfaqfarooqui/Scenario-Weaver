# Adversarial Scenario Generation

← [Back to README](../README.md)

Adversarial generation produces scenarios that **intentionally violate safety constraints** — useful for testing autonomous vehicle edge cases, emergency systems, and failure modes.

## Quick Start

```bash
# Violate ALL safety constraints
cargo run --release -- -i examples/cut_in_left.yaml -o adversarial/ --adversarial
```

The `--adversarial` flag overrides all constraint modes to `violate`.

---

## Per-Constraint YAML Configuration

For fine-grained control, set `constraint_modes` in your YAML:

```yaml
scenario_type: cut_in_left
time_step: 0.5
duration: 10.0

actors:
  - id: ego
    role: ego
    lane: 1
    position: [0.0, 20.0]
    speed: [14.0, 16.0]
    acceleration: [-8.0, 3.0]

  - id: npc
    role: npc
    lane: 0
    position: [60.0, 80.0]
    speed: [16.0, 20.0]
    acceleration: [-8.0, 3.0]
    lane_changes:
      - direction: right
        start_time: [2.5, 3.5]
        duration: [3.0, 4.0]

road:
  num_lanes: 2
  lane_width: 3.5
  lane_directions: [1, 1]

min_ttc: 3.0
min_distance: 5.0

# Per-constraint modes
constraint_modes:
  min_ttc: violate       # Find TTC violations (< 3.0s)
  min_distance: enforce  # Maintain safe distance (≥ 5.0m)
```

**Result:** Scenarios where TTC is violated but minimum distance is maintained.

---

## Constraint Modes

Each constraint accepts one of three modes:

| Mode | Behaviour |
|------|-----------|
| `enforce` | Constraint must always hold (`G(constraint)`) |
| `violate` | Constraint must be violated at some point (`F(NOT constraint)`) |
| `ignore` | Constraint is omitted entirely |

### Shorthand

```yaml
constraint_modes: violate_all   # Violate every constraint
constraint_modes: ignore_all    # Omit every constraint (maximum freedom)
constraint_modes: enforce_all   # Enforce all (default, can be omitted)
```

---

## Available Constraint Modes

```yaml
constraint_modes:
  min_ttc: enforce              # Time-to-collision
  min_distance: enforce         # Longitudinal distance
  max_velocity: enforce         # Speed limit
  min_velocity: ignore          # Minimum speed (default: ignore)
  min_lateral_distance: ignore  # Side-by-side clearance (default: ignore)
  max_relative_velocity: ignore # Speed difference between actors (default: ignore)
  max_acceleration: enforce     # Acceleration bounds
```

The following optional threshold fields activate the corresponding constraint when present in the YAML:

| Field | Type | Description |
|-------|------|-------------|
| `max_velocity` | `f64` (m/s) | Global speed limit for all actors |
| `min_velocity` | `f64` (m/s) | Global minimum speed for all actors |
| `min_lateral_distance` | `f64` (m) | Minimum side-by-side clearance between actors |
| `max_relative_velocity` | `f64` (m/s) | Maximum speed difference between any two actors |
| `max_acceleration` | `f64` (m/s²) | Maximum longitudinal acceleration (positive) |
| `max_deceleration` | `f64` (m/s²) | Maximum deceleration (must be a negative value, e.g. `-8.0`) |
| `max_lateral_acceleration` | `f64` (m/s²) | Maximum lateral acceleration during lane changes (default: `2.0`) |

Note: `max_deceleration` and `max_lateral_acceleration` do not have corresponding `constraint_modes` entries — they are always enforced when present.

---

## Speed Limit Violation Example

```yaml
scenario_type: cut_in_left
time_step: 0.5
duration: 10.0

actors:
  - id: ego
    role: ego
    lane: 1
    position: [0.0, 20.0]
    speed: [25.0, 30.0]   # Speeding — above the 22 m/s limit
    acceleration: [-5.0, 3.0]

  - id: npc
    role: npc
    lane: 0
    position: [40.0, 60.0]
    speed: [15.0, 20.0]
    acceleration: [-4.0, 2.0]
    lane_changes:
      - direction: right
        start_time: [3.0, 6.0]
        duration: [3.0, 4.0]

road:
  num_lanes: 2
  lane_width: 3.5
  lane_directions: [1, 1]

min_ttc: 3.0
min_distance: 5.0
max_velocity: 22.0   # ~50 mph speed limit

constraint_modes:
  min_ttc: enforce
  min_distance: enforce
  max_velocity: violate   # Must exceed speed limit
```

See also: `examples/speed_limit_violation.yaml`, `examples/school_zone.yaml`, `examples/multi_lane_safety.yaml`, `examples/unsafe_following.yaml`.

---

## Use Cases

- **Emergency system testing** — Validate braking and collision avoidance under near-miss conditions
- **Edge case discovery** — Find worst-case scenarios within a parameter space
- **ML training data** — Generate diverse datasets that include safety violations
- **Compliance testing** — Document safety system behaviour under hazards (ISO 26262)
