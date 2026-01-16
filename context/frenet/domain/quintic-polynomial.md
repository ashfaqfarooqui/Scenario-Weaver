# Quintic Polynomial Algorithm

## Overview
The quintic (5th-order) polynomial is the optimal method for generating smooth lane changes in Frenet space. It guarantees C² continuity (continuous acceleration) and minimum jerk.

## Why Quintic?

### Properties
- 6 coefficients → can match position, velocity, acceleration at both ends
- Guarantees C² continuity (continuous acceleration)
- Minimizes integral of jerk squared (smoothest possible)
- Closed-form solution (fast computation: ~0.01ms)

### Mathematical Formulation
```
s(t) = a₀ + a₁t + a₂t² + a₃t³ + a₄t⁴ + a₅t⁵
t(t) = b₀ + b₁t + b₂t² + b₃t³ + b₄t⁴ + b₅t⁵
```

Where:
- s(t) = longitudinal position at time t
- t(t) = lateral position at time t
- a₀₋₅, b₀₋₅ = polynomial coefficients

## Boundary Conditions

### Start Conditions (t = 0)
```
s(0) = s_start      → Initial longitudinal position
s'(0) = s_v_start   → Initial longitudinal velocity
s''(0) = s_a_start  → Initial longitudinal acceleration

t(0) = t_start      → Initial lateral offset
t'(0) = t_v_start   → Initial lateral velocity
t''(0) = t_a_start  → Initial lateral acceleration
```

### End Conditions (t = T)
```
s(T) = s_end        → Final longitudinal position
s'(T) = s_v_end     → Final longitudinal velocity
s''(T) = s_a_end    → Final longitudinal acceleration

t(T) = t_end        → Final lateral offset
t'(T) = t_v_end     → Final lateral velocity
t''(T) = t_a_end    → Final lateral acceleration
```

## Solving for Coefficients

### Matrix Formulation
The 6 boundary conditions form a 6×6 linear system:

```
[A][coefficients] = [boundary_values]
```

For a generic polynomial P(t) = c₀ + c₁t + c₂t² + c₃t³ + c₄t⁴ + c₅t⁵:

**At t = 0:**
```
P(0) = c₀ = value_start
P'(0) = c₁ = velocity_start
P''(0) = 2c₂ = acceleration_start
```

**At t = T:**
```
P(T) = c₀ + c₁T + c₂T² + c₃T³ + c₄T⁴ + c₅T⁵ = value_end
P'(T) = c₁ + 2c₂T + 3c₃T² + 4c₄T³ + 5c₅T⁴ = velocity_end
P''(T) = 2c₂ + 6c₃T + 12c₄T² + 20c₅T³ = acceleration_end
```

### Pre-Computed Solution Matrix
Since the matrix structure is constant (only T varies), we can pre-compute the inverse:

```rust
struct QuinticPolynomial {
    a0: f64, a1: f64, a2: f64, a3: f64, a4: f64, a5: f64,
}

impl QuinticPolynomial {
    fn new(
        start: (f64, f64, f64),  // (position, velocity, acceleration)
        end: (f64, f64, f64),
        duration: f64,
    ) -> Self {
        let T = duration;
        let T2 = T * T;
        let T3 = T2 * T;
        let T4 = T3 * T;
        let T5 = T4 * T;

        // Pre-computed inverse solution (for efficiency)
        let a0 = start.0;
        let a1 = start.1;
        let a2 = start.2 / 2.0;

        let a3 = (20.0 * end.0 - 20.0 * start.0
                 - 8.0 * end.1 * T - 12.0 * start.1 * T
                 + 3.0 * start.2 * T2 - end.2 * T2) / (2.0 * T5);

        let a4 = (-30.0 * end.0 + 30.0 * start.0
                 + 14.0 * end.1 * T + 16.0 * start.1 * T
                 - 3.0 * start.2 * T2 + 2.0 * end.2 * T2) / (2.0 * T4);

        let a5 = (12.0 * end.0 - 12.0 * start.0
                 - 6.0 * end.1 * T - 6.0 * start.1 * T
                 + start.2 * T2 - end.2 * T2) / (2.0 * T3);

        Self { a0, a1, a2, a3, a4, a5 }
    }
}
```

## Evaluation

### Function and Derivatives
```rust
impl QuinticPolynomial {
    /// Evaluate polynomial at time t
    /// Returns (position, velocity, acceleration, jerk)
    fn evaluate(&self, t: f64) -> (f64, f64, f64, f64) {
        let t2 = t * t;
        let t3 = t2 * t;
        let t4 = t3 * t;
        let t5 = t4 * t;

        // Position
        let pos = self.a0 + self.a1 * t + self.a2 * t2 +
                  self.a3 * t3 + self.a4 * t4 + self.a5 * t5;

        // Velocity (first derivative)
        let vel = self.a1 + 2.0 * self.a2 * t +
                  3.0 * self.a3 * t2 + 4.0 * self.a4 * t3 +
                  5.0 * self.a5 * t4;

        // Acceleration (second derivative)
        let acc = 2.0 * self.a2 + 6.0 * self.a3 * t +
                  12.0 * self.a4 * t2 + 20.0 * self.a5 * t3;

        // Jerk (third derivative)
        let jerk = 6.0 * self.a3 + 24.0 * self.a4 * t +
                   60.0 * self.a5 * t2;

        (pos, vel, acc, jerk)
    }
}
```

## Context Dependencies
- **domain/core-concepts.md**: Frenet coordinate definitions
- **processes/trajectory-generation.md**: Complete workflow
- **decisions/polynomial-coefficients.md**: Coefficient handling strategy

## Examples

### Example 1: Simple Lane Change
```rust
// Start: t=0 (right lane), no lateral velocity/acceleration
let start = (0.0, 0.0, 0.0);

// End: t=3.5m (left lane), no lateral velocity/acceleration
let end = (3.5, 0.0, 0.0);

// Duration: 6 seconds
let lateral_poly = QuinticPolynomial::new(start, end, 6.0);

// Evaluate at t=3s (halfway through)
let (pos, vel, acc, jerk) = lateral_poly.evaluate(3.0);
// pos ≈ 1.75m (halfway across)
// vel ≈ max lateral velocity
// acc ≈ 0 (peak acceleration passed)
```

### Example 2: Velocity Change
```rust
// Accelerate longitudinally while maintaining constant lateral offset
let start_s = (0.0, 10.0, 0.0);   // 10 m/s
let end_s = (100.0, 20.0, 0.0);   // 20 m/s over 100m
let duration = 10.0;

let longitudinal_poly = QuinticPolynomial::new(start_s, end_s, duration);
```

### Example 3: Full Lane Change
```rust
// Lane change: right to left, 100m distance, 6s duration
let start_s = (0.0, 15.0, 0.0);   // Longitudinal start
let end_s = (100.0, 15.0, 0.0);    // Longitudinal end (constant velocity)

let start_t = (0.0, 0.0, 0.0);    // Right lane
let end_t = (3.5, 0.0, 0.0);      // Left lane

let s_poly = QuinticPolynomial::new(start_s, end_s, 6.0);
let t_poly = QuinticPolynomial::new(start_t, end_t, 6.0);

// Generate trajectory points
let dt = 0.1;
for i in 0..=60 {  // 6s / 0.1s = 60 steps
    let time = i as f64 * dt;
    let (s, s_d, s_dd, _) = s_poly.evaluate(time);
    let (t, t_d, t_dd, t_ddd) = t_poly.evaluate(time);
    // Use these Frenet state values
}
```

## Common Patterns

### Pattern 1: Zero Initial/Final Lateral Velocity
For clean lane changes, lateral velocity should be zero at start and end:
```rust
let start = (0.0, 0.0, 0.0);  // No lateral motion
let end = (3.5, 0.0, 0.0);    // No lateral motion
```

### Pattern 2: Constant Longitudinal Velocity
For typical lane changes, maintain constant longitudinal speed:
```rust
let velocity = 15.0;  // m/s
let distance = velocity * duration;

let start_s = (0.0, velocity, 0.0);
let end_s = (distance, velocity, 0.0);
```

### Pattern 3: Emergency Maneuver (Short Duration)
For obstacle avoidance, use shorter duration (2-3s):
```rust
let start = (0.0, 0.0, 0.0);
let end = (3.5, 0.0, 0.0);
let duration = 2.0;  // Fast!

// Higher lateral acceleration (~4 m/s²) but physically possible
```

## Performance
- Coefficient calculation: ~0.001ms
- Single evaluation: ~0.00001ms
- Full trajectory (100 points): ~0.01ms
