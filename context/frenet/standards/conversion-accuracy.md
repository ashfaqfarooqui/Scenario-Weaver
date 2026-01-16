# Conversion Accuracy Standards

## Overview
Accuracy requirements and validation methods for Frenet ↔ Cartesian coordinate conversion.

## Quality Criteria

### Criterion 1: Roundtrip Accuracy

**Description**: Converting Frenet → Cartesian → Frenet should return original values

**Thresholds**:
```yaml
roundtrip_accuracy:
  position_s: 1.0e-3    # 1mm accuracy in longitudinal
  position_t: 1.0e-3    # 1mm accuracy in lateral
  heading: 1.0e-3       # 0.06° accuracy in heading
```

**Measurement**:
```rust
fn test_roundtrip_accuracy(
    frenet: FrenetPoint,
    ref_line: &ReferenceLine
) -> AccuracyResult {
    let cartesian = frenet.to_cartesian(ref_line);
    let recovered = FrenetPoint::from_cartesian(cartesian, ref_line);

    AccuracyResult {
        s_error: (recovered.s - frenet.s).abs(),
        t_error: (recovered.t - frenet.t).abs(),
        theta_error: (recovered.theta - frenet.theta).abs(),
    }
}
```

**Failure Action**: Improve reference line resolution, check interpolation method

---

### Criterion 2: Reference Line Interpolation

**Description**: Reference line should be smooth and sufficiently dense

**Thresholds**:
```yaml
reference_line:
  min_waypoint_spacing: 5.0   # meters (max distance between waypoints)
  min_curvature_resolution: 0.1  # radians per meter (for curves)
  c2_continuous: true
```

**Measurement**:
```rust
fn validate_reference_line(ref_line: &ReferenceLine) -> bool {
    // Check C² continuity
    for s in (0..ref_line.length() as u32).step_by(5) {
        let t = s as f64;
        let yaw = ref_line.get_heading(t);
        let curvature = ref_line.get_curvature(t);

        // Curvature should be continuous
        if curvature.abs() > 1.0 {
            // Check nearby points
            let yaw_next = ref_line.get_heading(t + 1.0);
            if (yaw_next - yaw).abs() > PI / 4.0 {  // 45° over 1m = too sharp
                return false;
            }
        }
    }
    true
}
```

**Failure Action**: Increase waypoint density, use higher-order interpolation

---

### Criterion 3: Orthogonal Projection

**Description**: Cartesian → Frenet projection should find true closest point

**Thresholds**:
```yaml
projection:
  max_iterations: 100
  convergence_tolerance: 1.0e-6
  initial_guess_accuracy: 5.0   # meters
```

**Measurement**:
```rust
fn test_projection_accuracy(
    cartesian: CartesianPoint,
    ref_line: &ReferenceLine
) -> bool {
    let frenet = FrenetPoint::from_cartesian(cartesian, ref_line);

    // Verify this is indeed the closest point
    let s_test = frenet.s;
    let (x_r, y_r) = ref_line.interpolate(s_test);
    let dist_closest = ((cartesian.x - x_r).powi(2) +
                       (cartesian.y - y_r).powi(2)).sqrt();

    // Check nearby points
    for delta in -5.0..=5.0 {
        if delta == 0.0 { continue; }
        let s_neighbor = s_test + delta;
        if s_neighbor < 0.0 || s_neighbor > ref_line.length() {
            continue;
        }

        let (x_n, y_n) = ref_line.interpolate(s_neighbor);
        let dist_neighbor = ((cartesian.x - x_n).powi(2) +
                             (cartesian.y - y_n).powi(2)).sqrt();

        if dist_neighbor < dist_closest - 1e-3 {
            return false;  // Found closer point!
        }
    }

    true
}
```

**Failure Action**: Check convergence, increase iterations, use spatial index

---

## Scoring System

```yaml
conversion_score:
  roundtrip_accuracy: weight_40
  reference_line_quality: weight_30
  projection_accuracy: weight_30

  threshold: 9.0  # out of 10 for production
```

## Examples

### Pass Example (Straight Road)
```yaml
reference_line: straight_along_x
test_points:
  - { x: 50.0, y: 3.5, yaw: 0.0 }
  - { x: 100.0, y: -3.5, yaw: 0.1 }

results:
  roundtrip_errors:
    max_s_error: 1.0e-8 m ✓
    max_t_error: 1.0e-8 m ✓
    max_theta_error: 1.0e-10 rad ✓
  projection_accuracy: perfect (no iterations needed)
  conversion_score: 10.0/10
```

### Pass Example (Curved Road)
```yaml
reference_line: arc(radius=100m, angle=90°)
waypoint_density: 1 point per meter

results:
  roundtrip_errors:
    max_s_error: 2.0e-4 m ✓ (0.2mm)
    max_t_error: 5.0e-4 m ✓ (0.5mm)
    max_theta_error: 1.0e-3 rad ✓
  projection_accuracy: converged in 5 iterations ✓
  conversion_score: 9.8/10
```

### Fail Example (Sparse Reference Line)
```yaml
reference_line: waypoints every 20m
test_point: { x: 105.0, y: 3.0, yaw: 0.0 }

results:
  roundtrip_errors:
    s_error: 0.15 m ✗ (15cm error!)
    t_error: 0.08 m ✗
  projection: converged but inaccurate

action: Increase waypoint density to 5m or less
```

## Validation Rules

### Rule 1: Test Multiple Points
- Test at least 100 points randomly distributed
- Include points near reference line (t ≈ 0)
- Include points far from reference line (t = ±10m)
- Include points on curves (high curvature)

### Rule 2: Test Edge Cases
- Points at reference line start (s = 0)
- Points at reference line end (s = length)
- Points on sharp curves
- Points on inflection points

### Rule 3: Performance Validation
- Roundtrip conversion < 0.001ms per point
- Projection convergence < 0.01ms per point
- Reference line query < 0.0001ms per point

## Context Dependencies
- **domain/core-concepts.md**: Conversion formulas
- **processes/coordinate-conversion.md**: Conversion workflow
- **standards/smoothness-criteria.md**: Trajectory validation

## Best Practices
- Always validate reference line before use
- Use spatial index for projection initial guess
- Cache reference line interpolations
- Test on both straight and curved roads
- Measure roundtrip error at multiple points
- Consider higher-order interpolation for curves
