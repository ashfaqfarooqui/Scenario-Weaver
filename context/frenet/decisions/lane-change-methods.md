# Lane Change Method Support

## Overview
Choosing which lane change algorithms to support in test-4.

## Options

### Option 1: Quintic Polynomial Only
- Single method: 5th-order polynomial in Frenet space
- All lane changes use same algorithm
- Simple implementation

### Option 2: Multiple Methods
- Quintic polynomial
- Spline interpolation
- Bézier curves
- Optimal control
- User-selectable method

### Option 3: Adaptive Selection
- Automatically choose method based on scenario
- Quintic for simple lane changes
- Spline for complex maneuvers
- Optimization for obstacle avoidance

## Trade-offs

| Factor | Single Method | Multiple Methods | Adaptive |
|--------|---------------|------------------|----------|
| Code complexity | Low | High | Very High |
| Flexibility | Low | High | High |
| Testing effort | Low | Medium | High |
| User control | None | High | None |
| Performance | High | Variable | High |

## Recommendations

### Phase 1: Quintic Polynomial Only
- Implement proven algorithm from simple-scenario
- Focus on correctness and smoothness
- Validate thoroughly

### Phase 2: Add Spline Method (if needed)
- For scenarios requiring waypoints (e.g., multi-segment maneuvers)
- Spline interpolation through specified points
- Keep as optional alternative

### Not Recommended:
- Optimal control in Z3 (too complex, slow)
- Multiple methods initially (YAGNI principle)
- Automatic selection (hard to predict behavior)

## Quintic Polynomial Summary

### Why Quintic?
- 6 coefficients → match position, velocity, acceleration at both ends
- Guarantees C² continuity (continuous acceleration)
- Minimum jerk (smoothest possible)
- Closed-form solution (fast computation)

### Formulation
```
s(t) = a₀ + a₁t + a₂t² + a₃t³ + a₄t⁴ + a₅t⁵
t(t) = b₀ + b₁t + b₂t² + b₃t³ + b₄t⁴ + b₅t⁵
```

### Boundary Conditions
```rust
// At t = 0 (start)
s(0) = s_start,      t(0) = t_start
s'(0) = s_v_start,   t'(0) = t_v_start
s''(0) = s_a_start,  t''(0) = t_a_start

// At t = T (end)
s(T) = s_end,        t(T) = t_end
s'(T) = s_v_end,     t'(T) = t_v_end
s''(T) = s_a_end,    t''(T) = t_a_end
```

## Context Dependencies
- **domain/quintic-polynomial.md**: Algorithm details
- **standards/smoothness-criteria.md**: Smoothness requirements
- **processes/lane-change-workflow.md**: Complete workflow

## Examples

```rust
// Quintic polynomial lane change
let trajectory = QuinticTrajectory::lane_change(
    &start_frenet,   // s=0, t=0 (right lane)
    &end_frenet,     // s=100, t=3.5 (left lane)
    6.0,             // Duration: 6 seconds
);

// Verify smoothness
assert!(validate_c2_continuity(&trajectory));
assert!(validate_acceleration_limits(&trajectory, 2.0));  // 2 m/s² max
```

## Comparison to Spline Method

| Aspect | Quintic Polynomial | Spline Interpolation |
|--------|-------------------|---------------------|
| Smoothness | C² (optimal) | C² |
| Boundary control | Precise (6 DOF) | Loose |
| Computational cost | Very low (~0.01ms) | Medium (~0.1ms) |
| Flexibility | Low (one curve) | High (multiple segments) |
| Best use case | Simple lane change | Complex multi-waypoint |

## When to Add Spline Support
Consider adding spline support if:
- You need multi-segment trajectories (e.g., lane change → merge → lane change)
- You have pre-defined waypoints that must be followed exactly
- The path has complex geometry that quintic can't handle
