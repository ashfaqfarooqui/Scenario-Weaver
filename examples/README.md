# Coordinate System Examples

This directory contains example YAML specifications demonstrating different coordinate systems for scenario generation.

## Overview

The system supports two coordinate systems via the `coordinate_system` field in YAML specifications:

### Frenet (default)
- Uses longitudinal (s) and lateral (t) coordinates relative to reference line
- Smooth lane changes using lateral velocity constraints in Z3
- Better approximates real-world driving behavior
- Recommended for road-based scenarios

### Cartesian
- Uses traditional x, y coordinates
- Direct position and velocity variables in Z3
- Backward compatible with existing YAML files
- Useful for unstructured environments

## Coordinate System Architecture

The encoder system uses a **trait-based plugin architecture**:

1. **`CoordinateEncoder<B>` trait**: Defines interface for all coordinate-specific encoders
2. **`GenericEncoder<B>`**: Facade that dispatches to appropriate encoder based on `coordinate_system` field
3. **Coordinate-specific implementations**:
   - `FrenetEncoder`: Z3 variables are `frenet_s`, `frenet_t`, `frenet_vs`, `frenet_vt`, `frenet_lane`
   - `CartesianEncoder`: Z3 variables are `positions_x`, `positions_y`, `velocities_x`, `velocities_y`, `lanes`

## Specifying Coordinate System

Add the `coordinate_system` field to your YAML:

```yaml
# Use Frenet coordinates (default)
coordinate_system: frenet

actors:
  - id: ego
    lane: 1
    position: [0.0, 20.0]
    speed: 15.0
```

Or explicitly specify Cartesian:

```yaml
# Use Cartesian coordinates
coordinate_system: cartesian

actors:
  - id: ego
    lane: 1
    position: [0.0, 20.0]
    speed: 15.0
```

## Frenet Coordinate Details

When using `coordinate_system: frenet`, the encoder:

1. **Creates Frenet variables**: `frenet_s` (longitudinal), `frenet_t` (lateral), `frenet_vs`, `frenet_vt`
2. **Encodes kinematics**: `s[t+1] = s[t] + vs[t] * dt`, `t[t+1] = t[t] + vt[t] * dt`
3. **Applies lateral velocity constraints**: Bounds on `vt` ensure smooth lane changes
4. **Exports trajectories**: JSON output contains Frenet coordinates converted to Cartesian for visualization

### Smooth Lane Changes

Smoothness is enforced via Z3 constraints on lateral velocity:
- **Lateral velocity bounds**: `vt_min <= vt[t] <= vt_max` prevents sudden movements
- **Integer lane assignment**: Vehicles constrained to discrete lanes
- **No quintic pre-generation**: Z3 directly solves for all variables (simpler architecture)

## Examples

### cut_in_left.yaml
Basic cut-in scenario (uses default Frenet coordinates)

### t_junction.yaml
T-junction scenario demonstrating multi-road networks

### crossroads.yaml
Crossroads scenario with four-way intersection

## Output Format

The JSON output contains coordinates in Cartesian format (for visualization and CARLA), regardless of which coordinate system was used for solving:

```json
{
  "time": 1.0,
  "position": {"x": 65.0, "y": 1.75},
  "velocity": {"vx": 15.0, "vy": 0.0},
  "acceleration": {"ax": 0.0, "ay": 0.0},
  "lane": 1
}
```

**Note:** Frenet coordinates are used internally during solving (if `coordinate_system: frenet`), but output is always converted to Cartesian for consistency with visualization tools and CARLA.

## Running Examples

Generate a single scenario:
```bash
cargo run --release -- -i examples/cut_in_left.yaml -o output/
```

Generate with Frenet coordinates (explicit):
```bash
# Add to YAML: coordinate_system: frenet
cargo run --release -- -i examples/your_scenario.yaml -o output/
```

Generate with Cartesian coordinates:
```bash
# Add to YAML: coordinate_system: cartesian
cargo run --release -- -i examples/your_scenario.yaml -o output/
```
