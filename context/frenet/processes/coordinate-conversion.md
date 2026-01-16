# Coordinate Conversion Process

## Overview
Procedures for converting between Frenet (s, t, θ) and Cartesian (x, y, yaw) coordinate systems.

## When to Use
- Frenet → Cartesian: Rendering in CARLA, physics simulation
- Cartesian → Frenet: Processing sensor data, user input in Cartesian

## Prerequisites
- Reference line loaded and interpolated
- Reference heading function available

## Process: Frenet to Cartesian

### Step 1: Find Reference Point
**Action**: Interpolate reference line at longitudinal position s

```rust
let ref_point = reference_line.interpolate(frenet.s);
let (x_r, y_r) = (ref_point.x, ref_point.y);
```

### Step 2: Get Reference Heading
**Action**: Calculate tangent direction at position s

```rust
let yaw_r = reference_line.get_heading(frenet.s);
```

### Step 3: Calculate Cartesian Coordinates
**Action**: Apply Frenet transformation formulas

```rust
let x = x_r - frenet.t * yaw_r.sin();
let y = y_r + frenet.t * yaw_r.cos();
let yaw = yaw_r + frenet.theta;  // Add heading deviation
```

### Step 4: Return Cartesian Point
**Output**: (x, y, yaw) in CARLA coordinate system

```rust
let cartesian = CartesianPoint { x, y, yaw };
```

## Process: Cartesian to Frenet

### Step 1: Find Closest Point on Reference Line
**Action**: Project point onto reference line to find s

```rust
// Binary search or Newton-Raphson iteration
let mut s = 0.0;
for _ in 0..100 {  // Max 100 iterations
    let ref_point = reference_line.interpolate(s);
    let dx = cartesian.x - ref_point.x;
    let dy = cartesian.y - ref_point.y;
    let tangent = reference_line.get_tangent(s);

    // Orthogonal projection
    let projection = dx * tangent.0 + dy * tangent.1;
    let new_s = s + projection;

    if (new_s - s).abs() < 1e-6 {
        s = new_s;
        break;
    }
    s = new_s;
}
```

### Step 2: Calculate Lateral Offset
**Action**: Compute perpendicular distance from reference line

```rust
let ref_point = reference_line.interpolate(s);
let dx = cartesian.x - ref_point.x;
let dy = cartesian.y - ref_point.y;

// Signed distance (positive = left, negative = right)
let yaw_r = reference_line.get_heading(s);
let t = dx * yaw_r.cos() + dy * yaw_r.sin();
```

### Step 3: Calculate Heading Deviation
**Action**: Compute angle between velocity and reference heading

```rust
let theta = cartesian.yaw - yaw_r;

// Normalize to [-π, π]
while theta > PI { theta -= 2.0 * PI; }
while theta < -PI { theta += 2.0 * PI; }
```

### Step 4: Return Frenet Point
**Output**: (s, t, θ) in Frenet coordinate system

```rust
let frenet = FrenetPoint { s, t, theta };
```

## Validation

### Frenet → Cartesian Validation
```rust
#[test]
fn test_frenet_to_cartesian_accuracy() {
    let ref_line = ReferenceLine::straight_along_x();

    let frenet = FrenetPoint { s: 50.0, t: 3.5, theta: 0.0 };
    let cartesian = frenet.to_cartesian(&ref_line);

    // For straight line: should be exact
    assert!((cartesian.x - 50.0).abs() < 1e-6);
    assert!((cartesian.y - 3.5).abs() < 1e-6);
}
```

### Cartesian → Frenet Validation
```rust
#[test]
fn test_cartesian_to_frenet_accuracy() {
    let ref_line = ReferenceLine::straight_along_x();

    let cartesian = CartesianPoint { x: 50.0, y: 3.5, yaw: 0.0 };
    let frenet = FrenetPoint::from_cartesian(cartesian, &ref_line);

    assert!((frenet.s - 50.0).abs() < 1e-3);
    assert!((frenet.t - 3.5).abs() < 1e-3);
}
```

### Roundtrip Validation
```rust
#[test]
fn test_roundtrip_conversion() {
    let ref_line = ReferenceLine::from_waypoints(&[
        (0.0, 0.0), (10.0, 10.0), (20.0, 0.0)
    ]);

    let original = FrenetPoint { s: 15.0, t: 2.5, theta: 0.1 };

    // Frenet → Cartesian → Frenet
    let cartesian = original.to_cartesian(&ref_line);
    let recovered = FrenetPoint::from_cartesian(cartesian, &ref_line);

    // Should return to original (within tolerance)
    assert!((recovered.s - original.s).abs() < 1e-3);
    assert!((recovered.t - original.t).abs() < 1e-3);
    assert!((recovered.theta - original.theta).abs() < 1e-3);
}
```

## Decision Points

### If projection iteration doesn't converge:
- Increase max iterations
- Check reference line quality
- Use spatial index for initial guess

### If lateral offset is unexpected:
- Verify reference line orientation (tangent direction)
- Check coordinate system handedness
- Validate reference line continuity

### If heading deviation is large:
- Check if vehicle is driving backwards
- Verify reference line heading calculation
- Consider normalizing angles differently

## Performance Optimization

### Cache Reference Line Interpolation
```rust
struct CachedReferenceLine {
    ref_line: Arc<ReferenceLine>,
    cache: LruCache<f64, (f64, f64, f64)>,  // s → (x, y, yaw)
}
```

### Spatial Index for Cartesian → Frenet
```rust
struct SpatialReferenceLine {
    segments: Vec<ReferenceSegment>,
    kd_tree: KdTree,  // For initial s guess
}
```

## Context Dependencies
- **domain/core-concepts.md**: Coordinate system definitions
- **domain/quintic-polynomial.md**: Trajectory structures
- **standards/conversion-accuracy.md**: Accuracy requirements

## Common Issues

### Issue: Roundtrip error too large (> 1cm)
**Cause**: Reference line too coarse, interpolation errors

**Solution**: Increase reference line resolution, use higher-order interpolation

### Issue: Heading deviation incorrect
**Cause**: Wrong tangent direction or angle normalization

**Solution**: Verify tangent calculation, use atan2 for robust angle handling

### Issue: Projection iteration slow
**Cause**: Poor initial guess, many iterations needed

**Solution**: Use spatial index for initial s, limit max iterations

## Examples

### Example 1: Straight Road
```rust
// Reference line along x-axis
let ref_line = ReferenceLine::straight_along_x();

// Frenet: 50m down, 3.5m left
let frenet = FrenetPoint { s: 50.0, t: 3.5, theta: 0.0 };

// Convert to Cartesian
let cartesian = frenet.to_cartesian(&ref_line);
// Result: (x=50.0, y=3.5, yaw=0.0)
```

### Example 2: Curved Road
```rust
// 90° curve (arc)
let ref_line = ReferenceLine::arc(0.0, 0.0, 100.0, 0.0, PI/2);

// Frenet: halfway through curve (s = 50)
let frenet = FrenetPoint { s: 50.0, t: 0.0, theta: 0.0 };

// Convert to Cartesian
let cartesian = frenet.to_cartesian(&ref_line);
// Result: At 45° on circle, approximately (70.7, 70.7)
```

### Example 3: Multiple Conversions
```rust
// Batch conversion for trajectory
let trajectory: Vec<Transform> = frenet_points.iter()
    .map(|fp| fp.to_cartesian(&ref_line))
    .map(|(x, y, yaw)| Transform {
        location: Location { x, y, z: 0.0 },
        rotation: Rotation { pitch: 0.0, yaw, roll: 0.0 },
    })
    .collect();
```
