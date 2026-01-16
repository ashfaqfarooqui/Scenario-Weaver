# Smoothness Criteria

## Overview
Defines what constitutes "smooth" lateral motion and the validation rules to ensure realistic vehicle behavior.

## What is "Smooth" Motion?

### Characteristics
1. **Continuity**: Position, velocity, acceleration are continuous functions of time
2. **Human-like**: Comfortable lateral acceleration and jerk levels
3. **Physical**: Respects vehicle dynamics and limits
4. **Natural**: Gradual steering, no abrupt maneuvers (except emergencies)

### Mathematical Requirements

**Minimum: C² Continuity**
```rust
// Position is continuous
lim(t→t₀⁺) x(t) = lim(t→t₀⁻) x(t) = x(t₀)

// First derivative (velocity) is continuous
lim(t→t₀⁺) x'(t) = lim(t→t₀⁻) x'(t) = x'(t₀)

// Second derivative (acceleration) is continuous
lim(t→t₀⁺) x''(t) = lim(t→t₀⁻) x''(t) = x''(t₀)
```

## Quality Criteria

### Criterion 1: Lateral Acceleration

**Description**: Maximum lateral acceleration during maneuver

**Thresholds**:
```yaml
max_lateral_acceleration:
  comfortable: 2.0   # m/s² - comfortable for passengers
  acceptable: 3.0    # m/s² - noticeable but acceptable
  emergency: 5.0     # m/s² - for obstacle avoidance only
```

**Measurement**:
```rust
fn lateral_acceleration(frenet: &FrenetState, curvature: f64) -> f64 {
    // a_t = t_d² * curvature + t_dd
    frenet.t_d.powi(2) * curvature + frenet.t_dd
}
```

**Failure Action**: Increase maneuver duration or reduce lateral offset

---

### Criterion 2: Lateral Jerk

**Description**: Maximum rate of change of lateral acceleration

**Thresholds**:
```yaml
max_lateral_jerk:
  comfortable: 0.5   # m/s³ - very smooth
  acceptable: 1.0    # m/s³ - acceptable
  emergency: 2.0     # m/s³ - emergency maneuvers
```

**Measurement**:
```rust
fn lateral_jerk(frenet: &FrenetState) -> f64 {
    frenet.t_ddd  // Third derivative of t
}
```

**Failure Action**: Increase maneuver duration, check polynomial solver

---

### Criterion 3: Longitudinal Velocity Consistency

**Description**: Longitudinal velocity should not vary excessively

**Thresholds**:
```yaml
longitudinal_velocity:
  min: 5.0    # m/s (~18 km/h)
  max: 30.0   # m/s (~108 km/h)
  max_delta: 10.0  # m/s change during maneuver
  max_rate: 2.0    # m/s² accel/decel
```

**Measurement**:
```rust
fn validate_longitudinal_velocity(
    trajectory: &[FrenetState],
    limits: &VelocityLimits
) -> bool {
    for point in trajectory {
        if point.s_d < limits.min || point.s_d > limits.max {
            return false;
        }
    }
    true
}
```

**Failure Action**: Adjust target velocity or duration

---

### Criterion 4: Lane Boundary Compliance

**Description**: Vehicle must stay within lane (or transition smoothly)

**Thresholds**:
```yaml
lane_boundaries:
  lane_width: 3.5  # meters
  max_offset_during_change: lane_width * 1.5  # Allow spanning lanes
```

**Measurement**:
```rust
fn validate_lane_boundaries(
    trajectory: &[FrenetState],
    lane_width: f64
) -> bool {
    for point in trajectory {
        if point.t.abs() > lane_width * 1.5 {
            return false;
        }
    }
    true
}
```

**Failure Action**: Reduce lateral offset or increase road width

---

## Scoring System

```yaml
smoothness_score:
  lateral_acceleration: weight_30
  lateral_jerk: weight_20
  velocity_consistency: weight_20
  lane_compliance: weight_20
  c2_continuity: weight_10

  threshold: 8.0  # out of 10 for production
```

**Calculation**:
```rust
fn calculate_smoothness_score(
    trajectory: &[FrenetState],
    limits: &PhysicalLimits
) -> f64 {
    let a_t_score = score_lateral_acceleration(trajectory, limits);
    let jerk_score = score_lateral_jerk(trajectory, limits);
    let vel_score = score_velocity_consistency(trajectory, limits);
    let lane_score = score_lane_compliance(trajectory, limits);
    let c2_score = score_c2_continuity(trajectory);

    (a_t_score * 0.3 +
     jerk_score * 0.2 +
     vel_score * 0.2 +
     lane_score * 0.2 +
     c2_score * 0.1)
}
```

## Examples

### Pass Example (Comfortable Lane Change)
```yaml
trajectory:
  duration: 6.0s
  start: { s: 0, t: 0 }
  end: { s: 100, t: 3.5 }
  velocity: 15 m/s

results:
  max_lateral_acceleration: 1.8 m/s² ✓
  max_lateral_jerk: 0.4 m/s³ ✓
  c2_continuous: true ✓
  smoothness_score: 9.2/10
```

### Fail Example (Too Fast)
```yaml
trajectory:
  duration: 2.0s  # Too short!
  start: { s: 0, t: 0 }
  end: { s: 50, t: 3.5 }
  velocity: 20 m/s

results:
  max_lateral_acceleration: 5.2 m/s² ✗ (exceeds 2.0)
  max_lateral_jerk: 3.1 m/s³ ✗ (exceeds 0.5)
  smoothness_score: 3.5/10

action: Increase duration to 4-6s
```

### Emergency Lane Change (Acceptable)
```yaml
trajectory:
  duration: 1.5s  # Fast but necessary
  start: { s: 0, t: 0 }
  end: { s: 30, t: 3.5 }
  velocity: 20 m/s
  scenario: obstacle_avoidance

results:
  max_lateral_acceleration: 4.5 m/s² ✓ (within emergency limit)
  max_lateral_jerk: 1.8 m/s³ ✓ (within emergency limit)
  smoothness_score: 6.5/10 (acceptable for emergency)
```

## Common Patterns

### Pattern 1: Gradual Lane Change
- Duration: 4-6 seconds
- Lateral velocity: starts at 0, increases smoothly, returns to 0
- Lateral acceleration: bell-shaped curve, max ~1.5 m/s²

### Pattern 2: Quick Merge
- Duration: 2-3 seconds
- Higher lateral acceleration (~2.5 m/s²)
- Used when entering highway or merging

### Pattern 3: Emergency Avoidance
- Duration: 1-2 seconds
- Maximum lateral acceleration (~4-5 m/s²)
- Only for obstacle avoidance, not normal driving

## Context Dependencies
- **domain/core-concepts.md**: Frenet definitions
- **domain/quintic-polynomial.md**: Quintic algorithm (ensures C²)
- **processes/trajectory-generation.md**: Generation pipeline

## Best Practices
- Default to comfortable thresholds for normal scenarios
- Use acceptable thresholds for aggressive maneuvers
- Use emergency thresholds only for obstacle avoidance
- Always validate C² continuity regardless of mode
- Smoothness should be primary goal, not just constraint satisfaction
