# Frenet Trajectory Generation Process

## Overview
Step-by-step workflow for generating smooth trajectories using Frenet coordinates and quintic polynomials.

## When to Use
- Lane change scenarios
- Smooth lateral motion requirements
- Road-based autonomous driving scenarios

## Prerequisites
- Reference line loaded (spline from waypoints)
- Start and end Frenet states defined
- Duration specified (or longitudinal velocity known)

## Process Steps

### Step 1: Load Reference Line
**Action**: Create smooth spline from road waypoints

```rust
let waypoints = vec![(0.0, 0.0), (10.0, 10.0), (20.0, 20.0)];
let ref_line = CubicSpline::from_points(waypoints);
```

**Validation**: Reference line is C² continuous, spans scenario length

---

### Step 2: Define Start and End States
**Action**: Specify Frenet boundary conditions

```rust
let start = FrenetState {
    s: 0.0,              // Longitudinal position
    t: 0.0,              // Lateral offset (right lane)
    s_d: 15.0,           // Longitudinal velocity (m/s)
    t_d: 0.0,            // Lateral velocity (m/s)
    s_dd: 0.0,           // Longitudinal acceleration
    t_dd: 0.0,           // Lateral acceleration
};

let end = FrenetState {
    s: 100.0,            // 100m downstream
    t: 3.5,              // 3.5m left (left lane)
    s_d: 15.0,           // Same velocity
    t_d: 0.0,            // Zero lateral velocity
    s_dd: 0.0,
    t_dd: 0.0,
};
```

**Validation**: States are physically feasible, velocities within limits

---

### Step 3: Calculate Quintic Coefficients
**Action**: Solve 6x6 linear system for polynomial coefficients

```rust
let duration = 6.0;  // seconds

// Solve for longitudinal polynomial
let s_coeffs = QuinticSolver::solve(
    (start.s, start.s_d, start.s_dd),
    (end.s, end.s_d, end.s_dd),
    duration
);

// Solve for lateral polynomial
let t_coeffs = QuinticSolver::solve(
    (start.t, start.t_d, start.t_dd),
    (end.t, end.t_d, end.t_dd),
    duration
);
```

**Validation**: Coefficients are finite, no numerical overflow

---

### Step 4: Generate Trajectory Points
**Action**: Evaluate polynomials at desired time resolution

```rust
let dt = 0.1;  // 100ms resolution
let mut trajectory = Vec::new();

for i in 0..=((duration / dt) as usize) {
    let time = i as f64 * dt;

    // Evaluate quintic polynomials
    let s_state = evaluate_quintic(&s_coeffs, time);
    let t_state = evaluate_quintic(&t_coeffs, time);

    trajectory.push(FrenetState {
        s: s_state.position,
        t: t_state.position,
        s_d: s_state.velocity,
        t_d: t_state.velocity,
        s_dd: s_state.acceleration,
        t_dd: t_state.acceleration,
        s_ddd: s_state.jerk,
        t_ddd: t_state.jerk,
        theta: (t_state.velocity / s_state.velocity).atan(),
    });
}
```

**Output**: Trajectory with ~60 points for 6-second lane change

---

### Step 5: Validate Smoothness
**Action**: Check C² continuity and physical limits

```rust
// Validate C² continuity
assert!(validate_c2_continuity(&trajectory));

// Validate acceleration limits
assert!(validate_lateral_acceleration(&trajectory, 2.0));  // 2 m/s² max
assert!(validate_lateral_jerk(&trajectory, 0.5));  // 0.5 m/s³ max

// Validate lane boundaries
assert!(validate_lane_boundaries(&trajectory, 7.0));  // 7m road width
```

**Validation**: All checks pass

---

### Step 6: Convert to Cartesian (Optional)
**Action**: Transform to CARLA coordinates

```rust
let cartesian_traj: Vec<Transform> = trajectory.iter()
    .map(|fp| {
        let (x, y, yaw) = fp.to_cartesian(&ref_line);
        Transform {
            location: Location { x, y, z: 0.0 },
            rotation: Rotation { pitch: 0.0, yaw, roll: 0.0 },
        }
    })
    .collect();
```

**Output**: CARLA-ready trajectory

---

## Decision Points

### If smoothness validation fails:
- Increase lane change duration
- Reduce target velocity
- Check boundary condition feasibility

### If Z3 constraints needed:
- Go to **processes/z3-integration.md**
- Apply waypoint refinement around smooth trajectory

### If reference line unavailable:
- Fall back to Cartesian mode
- Or generate reference line from map data

## Context Dependencies
- **domain/core-concepts.md**: Frenet definitions
- **domain/quintic-polynomial.md**: Quintic solver details
- **standards/smoothness-criteria.md**: Validation rules
- **decisions/z3-strategy.md**: Z3 integration approach

## Success Criteria
- [x] Trajectory is C² continuous
- [x] Lateral acceleration < 2 m/s²
- [x] Lateral jerk < 0.5 m/s³
- [x] Lane boundaries respected
- [x] Conversion to Cartesian accurate (< 1mm)

## Common Issues

### Issue: Lateral acceleration too high
**Cause**: Lane change too fast or too aggressive

**Solution**: Increase duration or reduce lateral offset

### Issue: S-shaped trajectory (wiggle)
**Cause**: Incorrect boundary conditions (velocity not zero)

**Solution**: Set start/end lateral velocity to zero for clean lane change

### Issue: Jerk spikes at endpoints
**Cause**: Numerical precision issues in polynomial solver

**Solution**: Use higher precision floating point or adjust solver tolerance

## Examples

### Example 1: Simple Lane Change
```rust
let trajectory = generate_lane_change(
    &ref_line,
    &FrenetState::new(0.0, 0.0, 15.0, 0.0, 0.0, 0.0),
    &FrenetState::new(100.0, 3.5, 15.0, 0.0, 0.0, 0.0),
    6.0,
);

// Result: Smooth 6-second lane change, C² continuous
```

### Example 2: Velocity Change with Lane Change
```rust
let trajectory = generate_lane_change(
    &ref_line,
    &FrenetState::new(0.0, 0.0, 10.0, 0.0, 0.0, 0.0),
    &FrenetState::new(100.0, 3.5, 20.0, 0.0, 0.0, 0.0),
    6.0,
);

// Result: Accelerate from 10 to 20 m/s while changing lanes
```

### Example 3: Emergency Lane Change
```rust
let trajectory = generate_lane_change(
    &ref_line,
    &FrenetState::new(0.0, 0.0, 20.0, 0.0, 0.0, 0.0),
    &FrenetState::new(50.0, 3.5, 20.0, 0.0, 0.0, 0.0),
    2.5,  // Fast!
);

// Result: Quick lane change for obstacle avoidance
// Note: Higher lateral acceleration (~4 m/s²), within emergency limits
```
