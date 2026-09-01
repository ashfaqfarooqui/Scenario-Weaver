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

### Dynamics, and the linearisation

The kinematic bicycle model is

```
dx/dt  = v * cos(θ)
dy/dt  = v * sin(θ)
dθ/dt  = (v / L) * tan(δ)
dv/dt  = a
```

Two of those are **variable × variable** products. Z3 is asked to solve these
constraints alongside an integer `lane` variable, and a nonlinear mixed
integer-real problem (QF_NIRA) has no decision procedure — Z3 answers `unknown`
— while `Optimize`, which backs `--optimize`, has essentially no support for
nonlinear objectives. The encoder therefore stays inside linear real arithmetic
(QF_LRA), which costs two documented approximations:

**1. Small angle.** `cos(θ) ≈ 1`, `sin(θ) ≈ θ`, `tan(δ) ≈ δ`. The heading is
bounded to `|θ| ≤ atan(0.15) ≈ 8.5°` (see below), where `sin(θ) - θ` is 0.37 %
of `θ` and `1 - cos(θ)` is 1.1 %. A highway lane change at 16 m/s uses
`|δ| < 0.02 rad`, where the tangent error is under 0.02 %.

**2. Reference speed.** The `v` inside the two products is replaced by a
**constant** `v̄`: the midpoint of a speed bucket that `v[t]` is asserted to lie
inside. Buckets are 5 m/s wide by default and are derived per actor from the
speed the actor can actually reach (initial speed range widened by the
acceleration band over the horizon, clipped at 0 and at `max_velocity`), capped
at 8 buckets. The relative error on `vy` and on `dθ/dt` is at most half a bucket
over `v̄` — under 8 % at 16 m/s, and exactly zero for an actor whose reachable
speed span fits in one bucket. A disjunction of linear regimes is still QF_LRA:
each bucket contributes `lo ≤ v[t] < hi ⇒ (linear constraints)`, and the buckets
partition the line, so exactly one applies at each step.

So what is actually asserted is

```
px[t+1] = px[t] ± (v[t] + v[t+1]) * dt/2       (exact for piecewise-constant a)
py[t+1] = py[t] + (vy[t] + vy[t+1]) * dt/2
vy[t]   = v̄ * θ[t]
θ[t+1]  = θ[t] + (v̄ / L) * δ[t] * dt
v[t+1]  = v[t] + a[t] * dt
vy[t+1] = vy[t] + ay[t] * dt
```

Outside a lane change the encoder pins `vy = θ = δ = 0`, so the coupling is
inert there and no bucket machinery is emitted; a lane change is also required
to begin at zero heading, hence at zero lateral velocity.

The approximations are approximations of the *dynamics*, not of the exported
trajectory's internal consistency: `px`, `py`, `vx`, `vy`, `ax` and `ay` agree
with each other to machine precision, because `py` is integrated from the same
`vy` the heading defines.

### Constraints enforced

| Constraint | Expression |
|------------|------------|
| Heading drives lateral motion | `vy[t] = v̄ * θ[t]` |
| Steering drives heading | `θ[t+1] = θ[t] + (v̄ / L) * δ[t] * dt` |
| Steering angle bounds | `\|δ\| ≤ atan(L / R_min) = δ_max` |
| Heading angle bounds | `\|θ\| ≤ atan(0.15) ≈ 8.5°`, the same lateral/longitudinal ratio the Cartesian encoder uses |
| Steering rate | `\|δ[t+1] - δ[t]\| ≤ max_steering_rate * dt` |
| Heading rate (turn radius) | `\|θ[t+1] - θ[t]\| ≤ (v_max / R_min) * dt`, with `v_max` the actor's reachable top speed |
| Lateral velocity | `\|vy\| ≤ 0.15 * v` and `\|vy\| ≤ 2.0 m/s` — both the same as Cartesian |
| Lateral acceleration | `\|ay\| ≤ max_lateral_acceleration` |
| Speed | `v ≥ 0`, and `v ≤ max_velocity` when one is declared |
| Minimum turn radius | `R_min = L / tan(δ_max)` (e.g. 2.7 m / tan(0.6 rad) ≈ 3.95 m) |

> `actor.speed` is an **initial condition**, not a ceiling: it is the value (or
> range) the actor starts from. The velocity ceiling is the scenario-level
> `max_velocity`. Before this was fixed, `speed.max()` was asserted as a
> ceiling at every step, so an actor declaring `speed: 15.0` with
> `acceleration: [-8.0, 3.0]` could not accelerate at all, and identical YAML
> produced qualitatively different dynamics under the two coordinate systems.

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

The JSON output format is unchanged. The extractor converts bicycle-model state
to Cartesian velocities using the same small-angle approximation the encoder
asserts, so the exported numbers are the solver's, not a re-derivation:

```
vx = v            (cos θ ≈ 1)
vy = v̄ * θ        (sin θ ≈ θ, with v̄ the reference speed of the bucket
                   Z3 selected at that step)
ay = the ay solver variable, which drives vy
```

### Limitations

- The dynamics are linearised; see the two error bounds above. They are
  approximations of a real vehicle, not of the trajectory's internal
  consistency.
- Valid only while `|θ|` stays small, which the `atan(0.15)` heading bound
  enforces; a scenario needing a sharper heading than 8.5° is out of scope for
  this encoder.
- May return UNSAT if the lane-change duration is too short for the lateral
  velocity and acceleration envelopes to cover a lane width.
- `vx` ignores the `cos θ` factor, at most 1.1 % at the heading bound. Taking it
  into account would put the exported `vx` out of step with the longitudinal
  integration, which uses `v`.

### Examples

- `examples/bicycle_lane_change.yaml` — Highway cut-in with bicycle dynamics
- `examples/cut_in_right_bicycle.yaml` — Mirror-image cut-in on a 3-lane road

---

## Choosing a Coordinate System

| | Cartesian | Bicycle |
|---|---|---|
| Heading tracking | No | Yes — `θ` is asserted against `vy` |
| Steering constraints | No | Yes — `δ` is asserted against `θ` |
| Turn radius enforcement | No | Yes, via the heading-rate bound `v / R_min` |
| Lateral velocity envelope | `\|vy\| ≤ 0.15\|vx\|`, `\|vy\| ≤ 2.0 m/s` | the same two bounds |
| Solver speed | Faster | Slower; `bicycle_lane_change.yaml` solves in ~1.7 s against ~0.2 s |
| Best for | General scenarios, backward compat | Realistic dynamics, steering tests |

Given the same YAML, the two coordinate systems now produce comparable
dynamics. They are not identical — the bicycle model routes lateral motion
through a bounded heading rather than through a free `vy` — but neither one
permits motion the other forbids by an order of magnitude, which was the case
while `θ` was decorative.
