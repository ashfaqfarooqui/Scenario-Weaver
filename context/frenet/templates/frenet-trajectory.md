# Frenet Trajectory Template

## Overview
Standard template for generating Frenet-based vehicle trajectories in test-4.

## Template Structure

```yaml
scenario:
  type: "lane_change"
  coordinate_mode: "frenet"  # or "cartesian" for legacy

reference_line:
  source: "map_file"  # or "waypoints"
  data: "town01_centerline.json"
  # or
  waypoints:
    - { x: 0.0, y: 0.0 }
    - { x: 10.0, y: 10.0 }
    - { x: 20.0, y: 20.0 }

vehicle:
  id: "ego_vehicle"

  initial_state:
    s: 0.0           # Longitudinal position (m)
    t: 0.0           # Lateral offset (m, negative=right)
    s_d: 15.0        # Longitudinal velocity (m/s)
    t_d: 0.0         # Lateral velocity (m/s)
    s_dd: 0.0        # Longitudinal acceleration (m/s²)
    t_dd: 0.0        # Lateral acceleration (m/s²)

  target_state:
    s: 100.0         # Target longitudinal position
    t: 3.5           # Target lateral offset (left lane)
    s_d: 15.0        # Target longitudinal velocity
    t_d: 0.0         # Target lateral velocity
    s_dd: 0.0
    t_dd: 0.0

  duration: 6.0      # Maneuver duration (seconds)

  method: "quintic_polynomial"  # Only method for Phase 1

constraints:
  lane_width: 3.5    # Lane width (m)
  max_lateral_acceleration: 2.0  # m/s²
  max_lateral_jerk: 0.5         # m/s³
  min_velocity: 5.0             # m/s
  max_velocity: 30.0            # m/s

  obstacles:
    - id: "static_obstacle_1"
      s: 50.0
      t: 0.0
      radius: 1.5
      type: "static"

output:
  resolution: 0.1    # Time step (seconds)
  format: "carla_transform"  # or "frenet_state", "cartesian"
```

## Required Fields

| Field | Type | Description |
|-------|------|-------------|
| `scenario.type` | string | "lane_change", "lane_follow", "merge" |
| `scenario.coordinate_mode` | string | "frenet" or "cartesian" |
| `reference_line` | object | Waypoints or map file |
| `vehicle.initial_state.s` | float | Starting longitudinal position |
| `vehicle.initial_state.t` | float | Starting lateral offset |
| `vehicle.initial_state.s_d` | float | Starting longitudinal velocity |
| `vehicle.target_state.s` | float | Target longitudinal position |
| `vehicle.target_state.t` | float | Target lateral offset |
| `vehicle.duration` | float | Maneuver duration (seconds) |
| `output.resolution` | float | Time step for trajectory points |

## Optional Fields

| Field | Default | Description |
|-------|---------|-------------|
| `vehicle.initial_state.t_d` | 0.0 | Initial lateral velocity |
| `vehicle.initial_state.s_dd` | 0.0 | Initial longitudinal acceleration |
| `vehicle.initial_state.t_dd` | 0.0 | Initial lateral acceleration |
| `vehicle.method` | "quintic_polynomial" | Trajectory generation method |
| `constraints.lane_width` | 3.5 | Lane width (meters) |
| `constraints.max_lateral_acceleration` | 2.0 | Comfortable threshold |
| `constraints.max_lateral_jerk` | 0.5 | Comfortable threshold |
| `output.format` | "carla_transform" | Output format |

## Examples

### Example 1: Simple Lane Change
```yaml
scenario:
  type: "lane_change"
  coordinate_mode: "frenet"

reference_line:
  source: "map_file"
  data: "highway.json"

vehicle:
  id: "ego"

  initial_state:
    s: 0.0
    t: 0.0       # Right lane
    s_d: 20.0    # 72 km/h

  target_state:
    s: 150.0
    t: 3.5       # Left lane
    s_d: 20.0

  duration: 5.0

output:
  resolution: 0.1
```

**Result**: Smooth 5-second lane change at 20 m/s

### Example 2: Lane Following
```yaml
scenario:
  type: "lane_follow"
  coordinate_mode: "frenet"

reference_line:
  source: "map_file"
  data: "road.json"

vehicle:
  id: "ego"

  initial_state:
    s: 0.0
    t: 0.0       # Center of lane
    s_d: 15.0    # 54 km/h

  target_state:
    s: 500.0     # Follow lane for 500m
    t: 0.0       # Stay in center
    s_d: 15.0

  duration: 33.3  # 500m / 15 m/s

output:
  resolution: 0.1
```

**Result**: Straight line following in Frenet, converted to curved path in Cartesian

### Example 3: Emergency Lane Change
```yaml
scenario:
  type: "lane_change"
  coordinate_mode: "frenet"

reference_line:
  source: "map_file"
  data: "highway.json"

vehicle:
  id: "ego"

  initial_state:
    s: 0.0
    t: 0.0
    s_d: 25.0    # 90 km/h (high speed)

  target_state:
    s: 50.0      # Quick change (shorter distance)
    t: 3.5
    s_d: 25.0

  duration: 2.0  # Fast!

constraints:
  max_lateral_acceleration: 5.0  # Emergency limit
  max_lateral_jerk: 2.0

output:
  resolution: 0.05  # Higher resolution for fast maneuver
```

**Result**: Quick lane change for obstacle avoidance, higher lateral acceleration

### Example 4: Acceleration While Changing Lanes
```yaml
scenario:
  type: "lane_change"
  coordinate_mode: "frenet"

reference_line:
  source: "map_file"
  data: "highway.json"

vehicle:
  id: "ego"

  initial_state:
    s: 0.0
    t: 0.0
    s_d: 10.0    # Slow start

  target_state:
    s: 100.0
    t: 3.5
    s_d: 20.0    # Accelerate to 20 m/s

  duration: 6.0

constraints:
  max_longitudinal_acceleration: 2.0
  max_lateral_acceleration: 2.0

output:
  resolution: 0.1
```

**Result**: Simultaneous lateral transition and longitudinal acceleration

## Output Format Examples

### CARLA Transform Output
```json
[
  {
    "time": 0.0,
    "location": {"x": 0.0, "y": 0.0, "z": 0.0},
    "rotation": {"pitch": 0.0, "yaw": 0.0, "roll": 0.0}
  },
  {
    "time": 0.1,
    "location": {"x": 1.5, "y": 0.02, "z": 0.0},
    "rotation": {"pitch": 0.0, "yaw": 0.01, "roll": 0.0}
  }
]
```

### Frenet State Output
```json
[
  {
    "time": 0.0,
    "s": 0.0,
    "t": 0.0,
    "s_d": 15.0,
    "t_d": 0.0,
    "s_dd": 0.0,
    "t_dd": 0.0
  },
  {
    "time": 0.1,
    "s": 1.5,
    "t": 0.02,
    "s_d": 15.0,
    "t_d": 0.4,
    "s_dd": 0.0,
    "t_dd": 2.4
  }
]
```

## Variations

### Variation 1: With Z3 Constraints
```yaml
scenario:
  type: "lane_change"
  coordinate_mode: "frenet"

# ... initial/target state ...

z3_integration:
  enabled: true
  waypoint_tolerance: 0.5  # Allow Z3 to vary waypoints by ±0.5m
  collision_avoidance: true
  min_vehicle_distance: 5.0
```

### Variation 2: Multiple Vehicles
```yaml
vehicles:
  - id: "ego"
    initial_state: { s: 0.0, t: 0.0, s_d: 15.0 }
    target_state: { s: 100.0, t: 3.5, s_d: 15.0 }
    duration: 6.0

  - id: "other_vehicle"
    initial_state: { s: 20.0, t: 0.0, s_d: 15.0 }  # 20m ahead
    target_state: { s: 120.0, t: -3.5, s_d: 15.0 }  # Opposite direction
    duration: 6.0

constraints:
  collision_avoidance: true
  min_distance: 2.0
```

## Context Dependencies
- **domain/core-concepts.md**: Frenet coordinate definitions
- **processes/trajectory-generation.md**: Generation workflow
- **standards/smoothness-criteria.md**: Validation requirements

## Best Practices
- Always specify duration for predictable behavior
- Use emergency constraints only for obstacle avoidance
- Keep lateral velocity at 0 for clean lane changes
- Validate output smoothness before use in CARLA
- Use appropriate resolution (0.1s for normal, 0.05s for fast maneuvers)
- Test generated trajectory against constraints
