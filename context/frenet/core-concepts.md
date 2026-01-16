# Frenet Coordinate System: Core Concepts

## Overview
Frenet coordinate system is a curvilinear coordinate system commonly used in autonomous driving and path planning. It describes positions relative to a reference curve (typically the lane centerline), rather than absolute Cartesian coordinates.

## Definition

### Coordinate Axes
- **s (longitudinal)**: Arc length along the reference line from a reference point
- **t (lateral)**: Perpendicular offset from the reference line (signed: positive = left, negative = right)
- **θ (heading)**: Deviation from the reference line tangent direction

### Reference Line
A smooth curve that serves as the longitudinal reference, typically:
- Lane centerline
- Road centerline
- Pre-planned path segment
- Spline or polynomial curve

## Key Attributes

### Advantages Over Cartesian
- **Intuitive**: Natural representation of lane following and lane changes
- **Decoupled**: Longitudinal and lateral motions can be treated independently
- **Constrained**: Lane boundaries become simple bounds on t
- **Simpler**: Collision avoidance and path planning are more straightforward

### Limitations
- **Curve-dependent**: Requires a well-defined reference line
- **Singularity**: Can have issues at sharp curves or discontinuities
- **Conversion**: Requires Frenet-Cartesian transformations for rendering/physics

## Conversion Formulas

### Frenet to Cartesian
Given a reference line point at (x_r(s), y_r(s)) with tangent angle θ_r(s):

```
x(s, t) = x_r(s) - t * sin(θ_r(s))
y(s, t) = y_r(s) + t * cos(θ_r(s))
```

### Cartesian to Frenet
For a point (x, y), find s that minimizes the perpendicular distance to the reference line, then:

```
t = ±√[(x - x_r(s))² + (y - y_r(s))²]
θ = atan2(y - y_r(s), x - x_r(s)) - θ_r(s)
```

## Business Rules

### When to Use Frenet Coordinates
1. **Lane following**: Vehicle stays primarily on a reference line
2. **Lane changes**: Smooth lateral transitions between lanes
3. **Path planning**: Generating trajectories with constraints
4. **Traffic scenarios**: Multi-vehicle interactions on structured roads

### When to Prefer Cartesian
1. **Open areas**: Parking lots, intersections, off-road
2. **Unstructured environments**: No clear reference line
3. **Free navigation**: Point-to-point navigation without road constraints

## Relationships

### Depends on
- **Reference line definition**: Spline, polynomial, or waypoints
- **Path interpolation**: Continuous curve representation
- **Coordinate conversion**: Frenet-Cartesian transformations

### Used by
- **Path planning algorithms**: Generate trajectories in Frenet space
- **Motion control**: Track desired s-t trajectories
- **Scenario generation**: Define vehicle positions and movements
- **Constraint solving**: Apply lane boundary and collision constraints

## Examples

### Simple Lane Following
```yaml
# Vehicle staying in lane at 50m down the road
position:
  s: 50.0    # 50m along the lane
  t: 0.0     # Center of lane
  theta: 0.0 # Aligned with lane

# Vehicle 2m to the right of lane center
position:
  s: 30.0
  t: -2.0    # Negative = right side
  theta: 0.05 # Slight heading offset
```

### Lane Change from Left to Right
```yaml
# Start position (left lane)
start:
  s: 0.0
  t: 3.5     # 3.5m left of reference (center line)

# End position (right lane)
end:
  s: 100.0   # 100m downstream
  t: -3.5    # 3.5m right of reference

# Interpolated positions
waypoints:
  - {s: 0.0, t: 3.5}
  - {s: 25.0, t: 2.0}
  - {s: 50.0, t: 0.0}  # Cross reference line
  - {s: 75.0, t: -2.0}
  - {s: 100.0, t: -3.5}
```

## Common Patterns

### Polynomial Trajectory in Frenet
Use quintic (5th-order) polynomials for smooth lane changes:
- Longitudinal: s(t) = a₀ + a₁t + a₂t² + a₃t³ + a₄t⁴ + a₅t⁵
- Lateral: t(t) = b₀ + b₁t + b₂t² + b₃t³ + b₄t⁴ + b₅t⁵

Coefficients determined by boundary conditions:
- Position, velocity, acceleration at start and end

### Multi-Lane Reference
For highways with multiple lanes, define reference line as:
- Center of road (lane 0)
- Or center of current lane (lane-specific reference)
- Allows consistent t across lane changes

### Time-Varying t
For dynamic scenarios, treat t as function of time:
- t(s) = f(s): lateral position varies with longitudinal distance
- Or t(time): lateral position varies with time (for interaction scenarios)
