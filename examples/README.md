# Coordinate System Examples

This directory contains example YAML specifications demonstrating different coordinate systems for scenario generation.

## Overview

The system supports two coordinate systems via the `coordinate_system` field in YAML specifications:

### Cartesian (default)
- Uses traditional x, y coordinates
- Direct position and velocity variables in Z3
- Smooth lane changes using linear interpolation
- Useful for general scenarios

### Bicycle
- Uses kinematic bicycle model with heading tracking (x, y, θ, v)
- Realistic vehicle dynamics with steering constraints
- Enforces turn radius and steering rate limits
- Recommended for realistic vehicle behavior

## Coordinate System Architecture

The encoder system uses a **trait-based plugin architecture**:

1. **`CoordinateEncoder<B>` trait**: Defines interface for all coordinate-specific encoders
2. **`GenericEncoder<B>`**: Facade that dispatches to appropriate encoder based on `coordinate_system` field
3. **Coordinate-specific implementations**:
   - `CartesianEncoder`: Z3 variables are `positions_x`, `positions_y`, `velocities_x`, `velocities_y`, `lanes`
   - `BicycleEncoder`: Z3 variables are `positions_x`, `positions_y`, `heading_theta`, `speed_v`, `steering_delta`, `accelerations`, `lanes`

## Specifying Coordinate System

Add the `coordinate_system` field to your YAML:

```yaml
# Use Cartesian coordinates (default)
coordinate_system: cartesian

actors:
  - id: ego
    lane: 1
    position: [0.0, 20.0]
    speed: 15.0
```

Or specify Bicycle model:

```yaml
# Use Bicycle model
coordinate_system: bicycle

bicycle_config:
  default_wheelbase: 2.7
  default_max_steering_angle: 0.6
  default_max_steering_rate: 0.5

actors:
  - id: ego
    lane: 1
    position: [0.0, 20.0]
    speed: 15.0
```

## Examples

### cut_in_left.yaml
Basic cut-in scenario using Cartesian coordinates

### t_junction.yaml
T-junction scenario demonstrating multi-road networks

### crossroads.yaml
Crossroads scenario with four-way intersection

## Output Format

The JSON output contains coordinates in Cartesian format for visualization and CARLA:

```json
{
  "time": 1.0,
  "position": {"x": 65.0, "y": 1.75},
  "velocity": {"vx": 15.0, "vy": 0.0},
  "acceleration": {"ax": 0.0, "ay": 0.0},
  "lane": 1
}
```

Both coordinate systems (Cartesian and Bicycle) output in this format for consistency with visualization tools and CARLA.

## Running Examples

Generate a single scenario with Cartesian coordinates:
```bash
cargo run --release -- -i examples/cut_in_left.yaml -o output/
```

Generate with Bicycle model:
```bash
# Add to YAML: coordinate_system: bicycle
cargo run --release -- -i examples/bicycle_lane_change.yaml -o output/
```
