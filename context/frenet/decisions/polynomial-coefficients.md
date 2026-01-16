# Polynomial Coefficient Handling

## Overview
How to handle quintic polynomial coefficients in relation to Z3 constraint solving.

## Options

### Option 1: Pre-Solved Coefficients
- Calculate quintic coefficients before Z3
- Use Z3 only for timing/position adjustments
- Coefficients fixed, not variables

### Option 2: Z3-Solves Coefficients
- Polynomial coefficients as Z3 variables
- Z3 ensures constraints through coefficient values
- More flexibility, but more complex

### Option 3: Hybrid: Coefficients + Adjustments
- Base coefficients from quintic solution
- Z3 adds small offset functions
- Combine base + adjustment for final trajectory

## Trade-offs

| Factor | Pre-Solved | Z3-Solved | Hybrid |
|--------|------------|-----------|--------|
| Z3 complexity | Low | High | Medium |
| Solution quality | High (smooth) | Variable | Medium-High |
| Solving speed | Fast | Slow | Medium |
| Implementation | Simple | Complex | Medium |

## Recommendation
**Pre-solved coefficients with Z3 waypoint refinement**

### Rationale:
- Quintic polynomials already optimal for smoothness
- Z3 is not needed for coefficient optimization
- Waypoint refinement is sufficient for collision avoidance
- Much simpler than making coefficients variables

## Implementation

```rust
// Pre-solve coefficients (closed-form solution)
let lateral_coeffs = solve_quintic(
    (t_start, t_v_start, t_a_start),
    (t_end, t_v_end, t_a_end),
    duration
);

let longitudinal_coeffs = solve_quintic(
    (s_start, s_v_start, s_a_start),
    (s_end, s_v_end, s_a_end),
    duration
);

// Generate trajectory points from coefficients
let smooth_traj: Vec<FrenetState> = (0..=num_points)
    .map(|i| {
        let t = i as f64 * dt;
        FrenetState {
            s: evaluate_quintic(&longitudinal_coeffs, t).position,
            t: evaluate_quintic(&lateral_coeffs, t).position,
            // ... other state components
        }
    })
    .collect();

// Use Z3 only for waypoint refinement
let refined_traj = apply_z3_refinement(smooth_traj, constraints);
```

## Alternative for Advanced Use Cases
If future scenarios require complex constraints (e.g., dynamic obstacles), consider:
- Coefficient constraints as soft constraints in optimization
- Or use optimal control methods (MPC) instead of Z3

## Context Dependencies
- **domain/quintic-polynomial.md**: Quintic polynomial details
- **processes/trajectory-generation.md**: Trajectory generation pipeline
- **decisions/z3-strategy.md**: Z3 integration approach

## Examples

```rust
// Start and end conditions
let start = FrenetState {
    s: 0.0, t: 0.0,
    s_d: 15.0, t_d: 0.0,
    s_dd: 0.0, t_dd: 0.0,
};

let end = FrenetState {
    s: 100.0, t: 3.5,
    s_d: 15.0, t_d: 0.0,
    s_dd: 0.0, t_dd: 0.0,
};

// Pre-solve coefficients (fast: ~0.01ms)
let (s_coeffs, t_coeffs) = solve_quintic_lane_change(&start, &end, 6.0);

// Generate smooth trajectory
let trajectory = generate_from_coeffs(&s_coeffs, &t_coeffs, 6.0);
```
