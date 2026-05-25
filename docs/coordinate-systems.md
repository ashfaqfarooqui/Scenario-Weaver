# Coordinate Systems

← [Back to README](../README.md)

ScenarioWeaver supports two coordinate systems for modelling vehicle dynamics. Select one via the `coordinate_system` field in your YAML.

---

## Cartesian (x, y) — Default

Point-mass model with independent x and y velocities. Best for general use and backward compatibility.

```yaml
coordinate_system: cartesian  # or omit entirely
```

**Variables per actor per time step:** `x`, `y`, `vx`, `vy`, `lane`

**Lane coupling:** lateral position is tied to lane centre:
```
py = lane * lane_width + lane_width / 2
```

**Lane change physics:** During a lane change the lateral position linearly interpolates between lane centres. A velocity-ratio constraint prevents physically impossible sideways-only motion:

```
|vy| ≤ 0.15 * |vx|
```

This corresponds to a maximum heading angle of ~8.5°. At 15 m/s forward speed the maximum lateral velocity is 2.25 m/s, so a 3.5 m lane change takes at least ~1.6 s.

> If a scenario specifies a very short lane-change duration at low speed, Z3 may return UNSAT. Increase the duration or the actor's minimum speed.

---

## Bicycle Model (x, y, θ, v)

Kinematic bicycle model with heading tracking and steering constraints. Provides realistic vehicle dynamics with turn-radius enforcement.

```yaml
coordinate_system: bicycle

# Scenario-level defaults (required when using bicycle model)
bicycle_config:
  default_wheelbase: 2.7              # metres (typical sedan)
  default_max_steering_angle: 0.6     # radians (~34°)
  default_max_steering_rate: 0.5      # rad/s
```

**Variables per actor per time step:** `x`, `y`, `θ` (heading), `v` (speed), `δ` (steering angle), `a` (acceleration), `lane`

### Dynamics (small-angle approximation)

```
dx/dt  = v * cos(θ) ≈ v
dy/dt  = v * sin(θ) ≈ v * θ
dθ/dt  = (v / L) * tan(δ) ≈ (v / L) * δ
dv/dt  = a
```

### Constraints enforced

| Constraint | Expression |
|------------|------------|
| Steering angle bounds | `-δ_max ≤ δ ≤ δ_max` |
| Heading angle bounds | `-π/6 ≤ θ ≤ π/6` (±30°, required for small-angle validity) |
| Steering rate | `\|δ[t+1] - δ[t]\| ≤ max_steering_rate * dt` |
| Speed | `v ≥ 0` |
| Minimum turn radius | `R_min = L / δ_max` (e.g. 2.7 m / 0.6 rad ≈ 4.5 m) |

### Per-actor overrides

```yaml
actors:
  - id: npc
    role: npc
    # ...
    bicycle_params:
      wheelbase: 2.9              # Larger vehicle (SUV)
      max_steering_angle: 0.5     # Less maneuverable
      max_steering_rate: 0.4      # Slower steering
```

If `coordinate_system: bicycle` is set but no `bicycle_config` defaults and no per-actor `bicycle_params` are provided, the parser will return an error.

### Trajectory output

The JSON output format is unchanged — the extractor converts bicycle-model state to Cartesian velocities using the small-angle approximation:

```
vx ≈ v
vy ≈ v * θ
```

### Limitations

- Valid only for `|θ| < 30°` (small-angle approximation breaks down beyond this)
- May return UNSAT if the lane-change duration is too short for the vehicle's minimum turn radius
- Lateral acceleration in JSON output is set to 0 (could be computed as `v² * θ / L` if needed)

### Examples

- `examples/bicycle_lane_change.yaml` — Highway cut-in with bicycle dynamics

---

## Choosing a Coordinate System

| | Cartesian | Bicycle |
|---|---|---|
| Heading tracking | No | Yes |
| Steering constraints | No | Yes |
| Turn radius enforcement | No | Yes |
| Solver speed | Faster | Slightly slower |
| Best for | General scenarios, backward compat | Realistic dynamics, steering tests |
