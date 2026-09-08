# Z3 Constraint Reference: Cartesian and Bicycle Encoders

> **Advanced / contributor reference.** This document walks through the Z3
> assertions each encoder emits, at the level of individual formulas and their
> arithmetic theory. It is written for someone about to modify
> `src/solver/encoders/*.rs` or debug a solver timeout, not for someone
> writing a scenario YAML. If you are authoring scenarios, start with
> [`architecture.md`](architecture.md) and [`coordinate-systems.md`](coordinate-systems.md)
> instead – this document assumes both as background and does not repeat
> their concept-level explanations. New scenario types are covered in
> [`creating-scenario-types.md`](creating-scenario-types.md).

Both encoders share one property that is easy to lose sight of while reading
formula-by-formula: every constraint in the standard pipeline is linear.
There is no non-linear arithmetic anywhere in a generated scenario's
encoding: not in the kinematics, not in the lane coupling, not in the TTC
proposition. That was not always true, and section 11 covers what changed and
why; sections 3-7 describe the encoding as it exists today.

---

## 1. Z3 Theory Primer

Z3 is an SMT (Satisfiability Modulo Theories) solver. It dispatches
constraints to specialized decision procedures depending on which *theory*
the constraints belong to:

| Theory | Full Name | What it covers | Solver used | Speed |
|--------|-----------|-----------------|-------------|-------|
| **QF_LRA** | Quantifier-Free Linear Real Arithmetic | Addition, subtraction, scalar multiplication, comparisons of Real variables | Simplex | Fast, polynomial time |
| **QF_LIA** | Quantifier-Free Linear Integer Arithmetic | Same as LRA but for Int variables | Branch-and-bound / Omega test | Fast |
| **QF_LIRA** | Mixed linear Int + Real | Linear combinations where an Int variable is coerced to Real (`to_real(n)`) and then only multiplied by constants | Combined Simplex + branch-and-bound | Fast, decidable |
| **QF_NRA** | Quantifier-Free Non-linear Real Arithmetic | Multiplication of two *symbolic* Real variables | NLSAT / CAD | Slow, can be exponential |
| **QF_NIRA** | Non-linear, with an Int variable in the same problem | Any of the above plus a genuinely non-linear term | No complete decision procedure; Z3 may answer `unknown` | N/A |

**Key rule:** `constant × symbolic_var` is linear, regardless of theory.
`symbolic_var × symbolic_var` is non-linear.

```
15.0 * theta[t]          -- linear  (15.0 is a constant rational)
min_ttc * rel_vel[t]     -- linear  (min_ttc is a YAML-supplied f64, fixed at encode time)
v[t] * theta[t]          -- non-linear  (both are free symbolic variables)
```

`QF_NIRA` is the trap this codebase is built to avoid. The lane index is a
symbolic `Int` in every scenario (there is no coordinate system without
lanes), so as soon as *any* constraint anywhere in the problem contains a
non-linear Real term, the combined problem is `QF_NIRA`, which has no
complete decision procedure – Z3 can and does answer `unknown` on it, and
`Optimize` has essentially no support for it at all. `QF_LIRA` alone (an Int
coerced to Real and then only scaled by a constant, as the Cartesian
encoder's lane-position coupling does) stays fully decidable. The distinction
the encoders care about at every step is therefore not "is this Real or
Int" but "does this term multiply two things Z3 doesn't already know the
value of."

Every technique in sections 3 and 4 (trapezoidal integration, the
lane-position bracket, the bicycle model's speed buckets) exists to keep
every asserted formula on the linear side of that line while still
expressing something that looks, and drives, like a real vehicle.

---

## 2. Encoding Pipeline

Both encoders are driven through the same call sequence from `src/lib.rs`
(the single-scenario and multi-scenario entry points call it identically):

```
Step 1  encoder.create_variables()
        -> allocate all Z3 variables for t = 0 .. horizon

Step 2  encoder.encode_initial_conditions()
        -> fix starting state at t = 0

Step 3  encoder.encode_kinematics()
        -> add motion equations linking each t to t+1
           [bicycle only: also calls encode_lane_coupling_with_lane_changes(),
            encode_bicycle_constraints(), and encode_heading_coupling() here]

Step 4  encoder.encode_velocity_constraints()
        -> see the per-encoder notes below; not a no-op for either encoder

Step 5  encoder.encode_acceleration_constraints()
        -> see the per-encoder notes below; not a no-op for either encoder

Step 6  encoder.encode_lane_velocity_constraints()
        -> integer lane bounds and single-lane-jump constraint

Step 7  encoder.encode_lateral_velocity_bounds()
        -> absolute |vy| cap, both encoders

Step 8  encoder.encode_ltl(formula)
        -> expand G / F / U operators over [0, horizon]

Step 9  encoder.encode_scenario_specific_constraints(model)
        -> delegates to the ScenarioModel's own add_z3_constraints()

Step 10 [blocking clauses, one per prior scenario in multi-scenario mode]
```

Steps 4 and 5 look identical from the call site, since the `CoordinateEncoder`
trait requires both encoders to implement them, but they do different
amounts of work per implementor, and it is worth being explicit about that
rather than assuming symmetry:

- **`encode_velocity_constraints()`** is a no-op for `CartesianEncoder`: the
  direction sign is asserted by `encode_lane_velocity_constraints()` instead,
  and a declared `max_velocity` is enforced coordinate-system-agnostically as
  a `VelocityLT` LTL proposition. For `BicycleEncoder` this method asserts
  that same `max_velocity` ceiling directly on the speed variable, the
  encoder's only *unconditional* enforcement of it, since not every spec
  reaches a `VelocityLT` atom.
- **`encode_acceleration_constraints()`** is a no-op for `CartesianEncoder`,
  which asserts its acceleration band inline inside `encode_kinematics()`.
  For `BicycleEncoder` this is where the acceleration band is asserted;
  `encode_kinematics()` does not do it there. Neither encoder forces a
  constant acceleration across the whole horizon any more (see §3.3 and
  §4.7 for why that constraint was removed).

The bicycle encoder additionally calls lane coupling, bicycle-specific bounds
(steering, heading, speed sign), and the heading-to-lateral-velocity coupling
from *inside* `encode_kinematics()`, in that order, because each stage
depends on state the previous one pins down (phase classification first,
then the coupling that reads it).

---

## 3. Cartesian Encoder

### 3.1 Variables (`create_variables()`)

Source: `src/solver/encoders/cartesian.rs`

Per actor, per time step `t ∈ [0, horizon]`:

```
px[a][t]   : Real   longitudinal position (m)
py[a][t]   : Real   lateral position (m)
vx[a][t]   : Real   longitudinal velocity (m/s), signed
vy[a][t]   : Real   lateral velocity (m/s)
ax[a][t]   : Real   longitudinal acceleration (m/s^2)
ay[a][t]   : Real   lateral acceleration (m/s^2)
lane[a][t] : Int    discrete lane index
```

Seven variables per actor per time step (six Real, one Int) – unchanged in
shape from the original design, though every one of the six Real variables
now participates in an equation (see §3.3): none of them is free-floating.

### 3.2 Initial Conditions (`encode_initial_conditions()`)

Source: `encode_actor_initial_state()`, `encoders::pedestrian::encode_pedestrian_initial_state()`

Vehicles and pedestrians take different initial-state paths, both called
from the same method:

| Variable | Vehicle | Pedestrian |
|----------|---------|------------|
| `lane[0]` | `lane[0] = lane_spec` (LIA) | `lane[0] = lane_spec` (LIA) |
| `px[0]` | fixed or ranged from `actor.position` (LRA) | same, from `actor.position` (LRA) |
| `vx[0]` | signed by `direction`, from `actor.speed` (LRA) | signed by `direction`, intersected with the walk/run speed cap (LRA) |
| `vy[0]` | `= 0` (not changing lanes at t=0) | **unconstrained** – a pedestrian may already be mid-crossing |
| `ax[0]` | fixed or ranged from `actor.acceleration` (LRA) | ranged from `actor.acceleration`, clamped to `[-1.0, 1.0]` m/s² |
| `ay[0]` | `= 0` | **unconstrained** |
| `py[0]` | via `encode_lane_position_coupling_at_time(0)`: `py[0] = to_real(lane[0])·lw + lw/2` (mixed LIA+LRA) | `py[0] = lane·lw + lw/2`, a constant (LRA) |

The pedestrian path leaving `vy[0]`/`ay[0]` free is deliberate, not an
oversight: a pedestrian crossing spec often wants the pedestrian already in
motion laterally at `t=0`, and pinning them to zero the way a vehicle is
pinned would make that unreachable.

### 3.3 Kinematics (`encode_kinematics()`)

Source: `src/solver/encoders/cartesian.rs`, delegating per-step arithmetic to
`encoders::pedestrian::encode_pedestrian_kinematics_step()`

The Cartesian encoder treats every actor, vehicle or pedestrian alike, as a 2D
point mass, and both use the *same* integration step, asserted once per axis
per time step:

```
vx[t+1] = vx[t] + ax[t]*dt                          LRA
vy[t+1] = vy[t] + ay[t]*dt                           LRA
px[t+1] = px[t] + (vx[t] + vx[t+1]) * dt/2           LRA  (trapezoidal)
py[t+1] = py[t] + (vy[t] + vy[t+1]) * dt/2           LRA  (trapezoidal)
```

The trapezoidal form is algebraically identical to the exact
constant-acceleration update `px + v*dt + 0.5*a*dt²` given the velocity
update asserted alongside it, and it is the cheaper of the two for Z3
because it couples position only to the velocity chain rather than to both
velocity and acceleration directly. It also closes a null space the older
forward-Euler position update left open: with `py` unconstrained between
updates, a stationary vehicle could satisfy `py[t+1] = py[t]` by picking any
`vy[t+1] = -vy[t]`, which let Z3 sawtooth a parked vehicle's lateral velocity
between its bounds at zero cost. Chaining `vy` to a *bounded* `ay` removes
that freedom.

`ax`/`ay` bounds are asserted alongside the integration step, in the same
loop, for every time step up to and including the horizon (not
`horizon - 1`) – the trapezoidal update reads `v[t+1]`, so `a[horizon]`
reaches into the encoding through the velocity chain and must not be left
free:

```
ax_min ≤ ax[t] ≤ ax_max     LRA   (vehicles: actor.acceleration range)
ay_min ≤ ay[t] ≤ ay_max     LRA   (max_lateral_acceleration, both signs)
```

There is no forced `ax[t+1] = ax[t]` constant-acceleration chain in either
encoder any more. It used to exist to prevent Z3 from oscillating
acceleration freely across the horizon; combined with a speed ceiling
computed from the wrong quantity, it instead made positive acceleration
mathematically unreachable for an entire scenario. The acceleration *range*
from the YAML spec is now the whole of the longitudinal envelope.

Pedestrians additionally get, at every step:

```
|vx| ≤ v_max_ped,  |vy| ≤ v_max_ped,  |vx|+|vy| ≤ sqrt(2)*v_max_ped      LRA (octagon)
-SIDEWALK_WIDTH ≤ py ≤ road_width + SIDEWALK_WIDTH                       LRA
```

The speed octagon (four linear half-planes cutting the corners off the
`|vx| ≤ v, |vy| ≤ v` box) approximates the physically-correct speed disk
`vx² + vy² ≤ v²` without the product of two symbolic variables the disk
would require. A plain box would let a pedestrian move at `sqrt(2)*v` on the
diagonal; the octagon instead admits that at most on the exact 45° line and
keeps every other heading, including straight-across-the-road, within `v`.
`v_max_ped` is `PEDESTRIAN_WALK_MAX_SPEED` (2.0 m/s) or
`PEDESTRIAN_RUN_MAX_SPEED` (5.0 m/s), selected from the `walking_mode`
behavior field. The lateral containment bound uses `SIDEWALK_WIDTH = 2.0` m
and is asserted unconditionally at every step, not only where an
`OnSidewalk` proposition happens to pin one instant – otherwise a pedestrian
is free to drift arbitrarily far past the sidewalk strip everywhere the LTL
formula doesn't literally say otherwise.

Lane-position coupling for non-pedestrian actors is handled separately, in
two forms:

```
Stable step:      py[t] = to_real(lane[t])·lw + lw/2      Mixed LIA+LRA, equality
Transition step:  |py[t] - (lane[t]·lw + lw/2)| ≤ lw/2     Mixed LIA+LRA, bracket
```

During a lane change `py` is between two lane centers and cannot equal
either exactly, so the equality form cannot be asserted there – but `lane`
must still name whichever lane's strip physically contains `py`. The bracket
form says exactly that, and unlike a schedule that pins `lane` to the source
value for the whole window and flips it only at the last step, it lets `py`
and `lane` disagree for at most the width of one lane, never a whole
manoeuvre's worth of it.

### 3.4 Lane Change Transition (`encode_smooth_lane_transition()`)

Source: `src/solver/encoders/cartesian.rs`

During the lane change window `[start_step, end_step]`:

| Assertion | Formula | Type |
|-----------|---------|------|
| Source position (soft) | `py[start] ∈ [src_center - 0.5, src_center + 0.5]` m | LRA |
| Target position (soft) | `py[end] ∈ [tgt_center - 0.5, tgt_center + 0.5]` m | LRA |
| Lane bracket, every step in the window | `\|py[t] - lane[t]·lw - lw/2\| ≤ lw/2` | Mixed LIA+LRA |
| Velocity ratio | `\|vy[t]\| ≤ k · \|vx[t]\|`, `k = 0.15` | LRA |

The velocity ratio is bounded from `start_step - 1`, not `start_step`: `py`
is pinned to the source lane center for every `t < start_step`, so
`py[start_step] = py[start_step - 1] + vy[start_step - 1]*dt` makes
`vy[start_step - 1]` the first lateral velocity actually free to move – bounding the ratio only from `start_step` onward left that one step covered
by nothing tighter than the flat `|vy| ≤ 2.0` cap (§3.8), and Z3 has been
observed to use exactly that slack. `k = 0.15` corresponds to a heading of
`atan(0.15) ≈ 8.5°`; for a backward-direction actor the sign of `vx` is
flipped before comparison so the ratio still reads as a positive magnitude
bound.

### 3.5–3.6 Velocity and Acceleration Constraint Methods

Both are no-ops for `CartesianEncoder` – see §2 for why, and where the
equivalent enforcement actually lives (`encode_lane_velocity_constraints()`
for direction, inline in `encode_kinematics()` for the acceleration band).

### 3.7 Lane and Velocity Direction Constraints (`encode_lane_velocity_constraints()`)

Source: `src/solver/encoders/cartesian.rs`

For every non-pedestrian actor, all time steps:

| Assertion | Formula | Type |
|-----------|---------|------|
| Direction | `vx[t] ≥ 0` (forward) or `vx[t] ≤ 0` (backward) | LRA |
| Lane lower bound | `lane[t] ≥ 0` | LIA |
| Lane upper bound | `lane[t] ≤ num_lanes - 1` | LIA |

Single-lane-jump, applied to every NPC and to the ego whenever it has a
declared lane change:

```
-1 ≤ lane[t+1] - lane[t] ≤ 1     LIA
```

An ego with no `lane_changes` declared is exempt from the jump constraint – it never needs it, since lane coupling already pins it to one lane for the
whole run.

### 3.8 Lateral Velocity Bounds (`encode_lateral_velocity_bounds()`)

Applied to every NPC and to an ego with declared lane changes:

```
-2.0 ≤ vy[t] ≤ 2.0     LRA
```

2.0 m/s is a hand-picked ceiling: a 3.5 m lane change completed smoothly over
3 s needs roughly 1.17 m/s of average lateral speed, so 2.0 leaves headroom
for a non-uniform profile without being loose enough to make the ratio bound
in §3.4 the only thing doing work.

### 3.9 Cartesian Constraint Count – Order of Magnitude

The exact count depends on the number of actors, the horizon, and how many
steps fall inside a lane-change window, and it is not worth pinning to a
specific number that the next kinematics change will falsify. As orders of
magnitude, for `A` actors and `H` steps:

| Stage | Order | Type |
|-------|-------|------|
| Initial conditions | `O(A)` | LRA/LIA/Mixed |
| Kinematics (4 equations/actor/step) | `O(A·H)` | LRA |
| Lane coupling (one bracket or equality per actor per step) | `O(A·H)` | Mixed LIA+LRA |
| Lane and direction bounds | `O(A·H)` | LRA/LIA |
| Lateral velocity bounds | `O(A·H)` | LRA |
| Lane-change ratio (only inside transition windows) | `O(A·W)`, `W` = window width | LRA |
| LTL safety, `G` over `[0,H]` | `O(A²·H)` for pairwise propositions | LRA (see §5) |

For a typical two-actor, 50-step cut-in the total is on the order of a
thousand assertions, and there is no non-linear term among them – every row
above is LRA, LIA, or their linear mix. Z3 solves scenarios like this in
well under two seconds.

---

## 4. Bicycle Encoder (Hybrid LRA)

The bicycle encoder gives vehicles a heading and a steering angle, the
whole reason to pick this coordinate system over Cartesian, while keeping
every assertion linear. The mechanism for that has changed since this
document last described it: the current encoder does not treat `vy` as an
independent variable bounded by a fixed ratio to `v`. It derives `vy` from
the heading, `vy = v̄ · θ`, where `v̄` is a constant standing in for the
symbolic speed `v`. Getting from "the exact bicycle model is non-linear" to
"this coupling is a constant times a variable" is the load-bearing idea in
this section, so it is worth stating precisely before the per-method detail.

### 4.1 The exact model, and the two approximations that linearize it

The exact kinematic bicycle model is:

```
dy/dt = v * sin(theta)                NRA (product of two symbolic variables)
dtheta/dt = (v / L) * tan(delta)      NRA (product of two symbolic variables)
```

With the `Int` lane variable present in every scenario, asserting either of
these as written puts the whole problem in `QF_NIRA` – no decision
procedure, and no `Optimize` support at all (§1). Two approximations buy
linearity back:

1. **Small angle.** `sin(θ) ≈ θ` and `tan(δ) ≈ δ`. The relative error on the
   first is `θ²/6`; the heading bound (§4.4) keeps `|θ| ≤ atan(0.15) ≈ 8.5°`,
   so the error is under 0.4% on `vy`. `δ` stays smaller still in practice: a highway lane change at 16 m/s uses `|δ| < 0.02` rad, where the tangent
   error is under 0.02%.
2. **Reference speed.** `v` in both products is replaced by the midpoint
   `v̄` of a *bucket* that `v[t]` is asserted to lie in, a constant fixed at
   encode time, not the symbolic `v[t]` itself. The relative error is then at
   most half a bucket width over `v̄`: under 8% at 16 m/s with the default
   5 m/s buckets, and exactly zero when the actor's whole reachable speed
   range fits inside one bucket. A disjunction over finitely many buckets,
   each holding a linear constraint, is still QF_LRA: the disjunction
   itself costs nothing extra because the buckets are asserted as a guarded
   implication, not a Z3-chosen case split (§4.3.3 covers why that
   distinction was worth 10x on solve time).

Both approximations affect the *dynamics* the encoder assumes, not whether
the exported trajectory is internally consistent: `px`, `py`, `vx`, `vy`,
`ax`, `ay` remain mutually consistent to machine precision, because `py` is
integrated from the very `vy` the coupling defines, using exact rational
arithmetic throughout.

### 4.2 Variables (`create_variables()`)

Source: `src/solver/encoders/bicycle.rs`

Per actor, per time step:

```
px[a][t]    : Real   longitudinal position (m)
py[a][t]    : Real   lateral position (m)
theta[a][t] : Real   heading angle (rad), deviation from nominal direction
v[a][t]     : Real   scalar speed (m/s), always >= 0
delta[a][t] : Real   front-wheel steering angle (rad)
a[a][t]     : Real   longitudinal acceleration (m/s^2)
lane[a][t]  : Int    discrete lane index
vy[a][t]    : Real   lateral velocity (m/s), derived: vy = v-bar * theta
ay[a][t]    : Real   lateral acceleration (m/s^2), chained to vy
```

Eight Real variables plus one Int, one more Real than the May-era design
(`ay` is now a genuine variable rather than a hard-coded zero at extraction, see §4.9). There is no separate signed `vx`: longitudinal velocity is the
magnitude `v` (always non-negative), and direction lives in the sign chosen
for the `px` update (§4.3.1). A `direction: -1` actor additionally gets a
`vx_signed[t] = direction * v[t]` variable (`direction` a Rust constant, so
still constant × variable) so that `get_longitudinal_vel()` returns a signed
quantity comparable with the Cartesian encoder's `vx`: every relative-
velocity and closing-speed computation in the shared LTL/objective code
subtracts two actors' longitudinal velocities and needs the sign to mean the
same thing on both sides of that subtraction.

### 4.3 Kinematics – All Linear (`encode_kinematics()`)

Source: `src/solver/encoders/bicycle.rs`

#### 4.3.1 Motion Equations

For every non-pedestrian actor, `t ∈ [0, horizon)`:

```
px[t+1] = px[t] + (v[t] + v[t+1]) * dt/2     LRA (forward, direction = 1)
px[t+1] = px[t] - (v[t] + v[t+1]) * dt/2     LRA (backward, direction = -1)
vy[t+1] = vy[t] + ay[t]*dt                   LRA
py[t+1] = py[t] + (vy[t] + vy[t+1]) * dt/2   LRA
v[t+1]  = v[t]  + a[t]*dt                    LRA
```

Same trapezoidal form as the Cartesian encoder (§3.3) and the same
rationale: it is exact for piecewise-constant acceleration given the
velocity update beside it, and chaining `vy` to a bounded `ay` closes the
same sawtooth null space a free `vy` would otherwise leave open.

Pedestrians in bicycle-coordinate scenarios go through the identical
point-mass helper the Cartesian encoder uses (`encoders::pedestrian`) – `speed_v` stands in for `vx`, `velocities_y` for `vy`. They have no heading
or steering and are excluded from every constraint in the rest of this
section.

#### 4.3.2 Phase-Specific Constraints

Every step is classified as **stable** (no lane change in progress) or
**lane-change** (inside a declared transition window), from the schedule
computed at encode time. For every non-pedestrian actor, all
`t ∈ [0, horizon]`:

```
Stable:  vy[t] = 0,  theta[t] = 0,  delta[t] = 0     LRA
```

No lateral motion, no heading deviation, no steering while driving straight.
Lane-change steps get the heading coupling instead (§4.3.3–4.3.4).

#### 4.3.3 Heading Rate Constraint

For steps where `t` or `t+1` falls in a lane change:

```
-R*dt ≤ theta[t+1] - theta[t] ≤ R*dt     LRA
```

`R = v_ceiling / R_min`, where `v_ceiling` is the actor's reachable speed
span's upper end (the initial speed range widened by the acceleration band
over the whole horizon, clipped to `max_velocity` when declared – not the
initial `speed.max()` alone, which would understate how fast a still-
accelerating vehicle can turn) and `R_min = wheelbase / tan(delta_max)` is
the exact minimum turn radius. Both are Rust `f64` constants computed once
per actor before any Z3 term is built, so the bound stays linear.

#### 4.3.4 Heading Coupling (`encode_heading_coupling()`)

This is the constraint that actually ties `θ` and `δ` to the vehicle's
motion – the reason to have a bicycle model at all. For every step where the
heading is allowed to be non-zero (the lane-change windows identified in
§4.3.2) and for each reference-speed bucket `[lo, hi)` with midpoint `v̄`:

```
lo ≤ v[t] < hi   =>   vy[t] = v̄ * theta[t]                    LRA (guarded)
                 =>   theta[t+1] = theta[t] + (v̄/L) * delta[t] * dt   LRA (guarded)
```

Buckets are built by `speed_buckets()`: the actor's reachable speed span
(§4.3.3) is tiled into spans of width `SPEED_BUCKET_WIDTH = 5.0` m/s, capped
at `MAX_SPEED_BUCKETS = 8` buckets total (an actor with a wide acceleration
range gets wider buckets instead of more of them, trading fidelity for
solver time past that cap). The buckets are half-open and partition the
whole real line (the outermost two are left open at their far edges), so
exactly one guard holds at every step and the four-way antecedent above
never needs to be asserted as a disjunction. That distinction is between
**propagation** (Simplex derives the one bucket that already holds from the
asserted bounds on `v[t]`, a single deterministic pass) and **search** (Z3
must try candidate buckets, backtracking on failure, because nothing in the
formula picks one for it), and it is not academic: an earlier draft let Z3
choose the bucket via a free Bool selector per bucket per step, turning 36
lane-change steps into an `8^36` search that took 12.6 s on
`bicycle_lane_change.yaml`, against 1.2 s for the guarded (propagated) form
here.

One step is handled specially: the step immediately before a lane-change
window opens has `theta[t]` and `delta[t]` both pinned to zero by the stable
phase, so `theta[t+1] = theta[t]` holds trivially there for any reference
speed, and the encoder asserts it directly without touching the bucket
machinery. This is what forces every lane change to begin from zero heading.

At extraction time (§4.8), `reference_speed_at()` re-evaluates the same
guard predicates against the solved model to recover which bucket Z3
selected, so the exported `vy` is read back from the same rationals the
solver reasoned over rather than recomputed from a rounded `f64` speed – avoiding disagreement right at a bucket boundary.

### 4.4 Bicycle-Specific Constraints (`encode_bicycle_constraints()`)

Source: `src/solver/encoders/bicycle.rs`, called from inside
`encode_kinematics()`

For every non-pedestrian actor, all `t ∈ [0, horizon]`:

| Assertion | Formula | Type | Reason |
|-----------|---------|------|--------|
| Steering bound | `\|δ[t]\| ≤ δ_max`, `δ_max = atan(wheelbase / R_min)` | LRA | Written through the turn radius so `R_min` is a real quantity in the encoding, not a method with no caller; algebraically identical to `\|δ\| ≤ max_steering_angle` |
| Heading bound | `\|θ[t]\| ≤ atan(0.15) ≈ 8.5°` | LRA | Matches the Cartesian encoder's `\|vy\| ≤ 0.15·\|vx\|` envelope exactly, rather than a separately-chosen angle |
| Speed non-negative | `v[t] ≥ 0` | LRA | `v` is a magnitude |

For `t ∈ [0, horizon)`:

```
-max_steering_rate*dt ≤ delta[t+1] - delta[t] ≤ max_steering_rate*dt     LRA
```

Default bicycle parameters (from `BicycleConfig`, used when an actor has no
per-actor `bicycle_params`): wheelbase 2.7 m, `max_steering_angle` 0.6 rad
(~34°), `max_steering_rate` 0.5 rad/s. The heading bound is derived from the
lateral-velocity ratio (`atan(0.15)`, ≈0.149 rad), not from
`max_steering_angle` directly – the two used to be different numbers
(±30°, `sin(π/6) = 0.5`) picked to match a lateral-velocity ratio the
bicycle encoder no longer uses; keeping one number shared between the two
coordinate systems means the same YAML now produces the same lateral speed
envelope under either.

### 4.5 Lane Coupling (`encode_lane_coupling_with_lane_changes()`)

Source: `src/solver/encoders/bicycle.rs`

Uses **concrete** lane indices computed from the YAML lane-change schedule
at encode time, never a symbolic `Int` inside an arithmetic expression – the
bicycle encoder's lane coupling is pure LRA/LIA with no mixed coercion at
all, unlike the Cartesian encoder's `to_real(lane)·lw`.

For a stable stretch at lane `L` (a Rust `usize`):

```
py[t] >= L * lw            LRA
py[t] <= (L+1) * lw         LRA
lane[t] = L                  LIA
```

During a transition window (`encode_smooth_lane_transition_bicycle()`):

```
py[start] in [src_center - 0.5, src_center + 0.5]     LRA (soft)
py[end]   in [tgt_center - 0.5, tgt_center + 0.5]      LRA (soft)
```

Rather than pinning `lane` to `source` for the whole window and `target`
only at the last step (which let `py` reach the target lane's center
seconds before `lane` acknowledged it), the encoder asserts a two-way case
split at every step of the window:

```
lane[t] = source OR lane[t] = target                                LIA
lane[t] = source  =>  |py[t] - source_center| <= lw/2                LRA
lane[t] = target  =>  |py[t] - target_center| <= lw/2                LRA
```

Because only two lanes are reachable in one window, this is expressed as a
disjunction over exactly those two rather than the general
`|py - lane·lw - lw/2| <= lw/2` bracket the Cartesian encoder uses (which
needs a symbolic `lane` and the mixed coercion that comes with it); it is
strictly stronger, since it also rules out a third lane index the schedule
never intended. Alongside it:

```
|vy[t]| <= k * v[t],  k = 0.15     LRA
0 <= py[t] <= num_lanes * lw       LRA (road bounds)
```

`k` is the same `LATERAL_VELOCITY_RATIO` constant the heading bound in §4.4
derives from – one number shared with the Cartesian encoder, not two
independently chosen ones.

### 4.6–4.7 Velocity and Acceleration Constraint Methods

`encode_velocity_constraints()` asserts `v[t] ≤ max_velocity` for every
non-pedestrian actor at every step, when `spec.max_velocity` is declared (see §2 for why this is the one place that bound is unconditionally
enforced). `encode_acceleration_constraints()` asserts the acceleration range
bound (`a_min ≤ a[t] ≤ a_max`) for every actor at every step and nothing
else: there is no forced `a[t+1] = a[t]` here either (§3.3 explains the
Cartesian-side removal; the bicycle-side removal fixed the same defect,
combined with a speed ceiling read from the wrong field, which made positive
acceleration unreachable for the whole run regardless of the declared
range).

### 4.8 Lane and Velocity Constraints (`encode_lane_velocity_constraints()`)

Lane bounds (`0 ≤ lane[t] ≤ num_lanes - 1`, LIA) apply to every actor.
Single-lane-jump (`-1 ≤ lane[t+1] - lane[t] ≤ 1`, LIA) applies to every
non-pedestrian actor **including ego** – unlike the Cartesian encoder, which
exempts an ego with no declared lane changes. There is no direction-based
velocity assertion here (`v ≥ 0` etc.); direction is encoded entirely in the
sign chosen for the `px` update in §4.3.1, and `v ≥ 0` itself is asserted in
§4.4.

### 4.9 Lateral Velocity Bounds (`encode_lateral_velocity_bounds()`)

The absolute cap here is the same `|vy| ≤ 2.0` m/s the Cartesian encoder
applies (§3.8), for the same reason. It is a genuine *second* bound, tighter
in practice than the heading-derived one at low speed and looser at high
speed: the heading coupling already limits `vy` to `0.1489 * v̄` through
`|θ| ≤ atan(0.15)`, so the two together give `vy` the smaller of a
speed-proportional bound and a flat 2.0 m/s ceiling. Both bounds are real
and both are asserted here: this method used to be an empty body, with a
comment attributing the bound to the heading constraints alone, from before
the heading was wired to `vy` at all (§4.3.4) and so could not have bounded
anything.

### 4.10 Bicycle Constraint Count – Order of Magnitude

As with the Cartesian encoder (§3.9), pinning an exact total invites drift.
The structural difference from Cartesian worth keeping in mind:

- Kinematics, phase pinning, and bicycle-specific bounds are `O(A·H)`, same
  as Cartesian's equivalent stages.
- Heading coupling is `O(A·W·N)`, where `W` is the total number of steps
  across all of an actor's lane-change windows (zero contribution outside
  them: no buckets, no guards) and `N` is that actor's bucket count
  (`≤ MAX_SPEED_BUCKETS = 8`). A scenario with short or no lane changes pays
  almost nothing here; one with wide speed ranges and long transitions pays
  proportionally more.
- Every term in every stage remains linear. There is no NRA contribution
  from the bicycle model itself, at any horizon or bucket count: the whole
  point of §4.1's two approximations.

In practice, scenarios of the size this project's example corpus uses (tens
of steps, one or two lane changes, two to four actors) solve in well under a
second; see §10 for what actually drives solve time now that neither
encoder contributes non-linear terms.

---

## 5. LTL Expansion (shared by both encoders)

Source: `src/ltl/encode.rs` (`encode_ltl_bounded()`, `encode_proposition()`)

### 5.1 Temporal Operators

```
G(phi)  [Always]     ->  phi[0] AND phi[1] AND ... AND phi[horizon]
F(phi)  [Eventually] ->  phi[0] OR  phi[1] OR  ... OR  phi[horizon]
phi U psi [Until]    ->  psi[t] OR (phi[t] AND (phi U psi at t+1))     (recursive)
```

`G` is the most common – it is how `enforce` mode works, and it produces
`horizon + 1` copies of whatever proposition it wraps.

### 5.2 Proposition Catalog

Source: `src/ltl/formula.rs` (the `Proposition` enum), `src/ltl/encode.rs`
(`encode_proposition()`)

| Proposition | Formula | Type |
|-------------|---------|------|
| `InLane(a, L)` | `lane[a][t] = L` | LIA |
| `Ahead(a1, a2)` | `px[a1][t] > px[a2][t]` (or `<`, chosen by shared travel direction – see below) | LRA |
| `DistanceGT(a1,a2,d)` | `same_lane ⟹ \|px1-px2\| ≥ d` | LRA |
| `TTCGT(a1,a2,ttc)` | guarded TTC bound, `ttc` a YAML constant | LRA |
| `Approaching(follower,leader)` | `px_lead > px_follow ∧ (vx_follow - vx_lead) > ε` | LRA |
| `OnSidewalk(a, side)` | `py` inside the sidewalk strip on the named side | LRA |
| `CrossingRoad(a)` | `0 ≤ py[a][t] ≤ road_width` | LRA |
| `RectangularDistanceGT(a1,a2,tx,ty)` | `\|dx\| ≥ tx ∨ \|dy\| ≥ ty` | LRA |
| `PedestrianTTCGT(ego,ped,ttc)` | guarded TTC bound for perpendicular crossing | LRA |
| `PedestrianTTCGuard(ego,ped)` | the antecedent of `PedestrianTTCGT`, named standalone | LRA |
| `VelocityGT(a,v)` / `VelocityLT(a,v)` | `\|vx[a][t]\| ≥ v` / `≤ v` | LRA |
| `LateralDistanceGT(a1,a2,d)` | `\|py1-py2\| ≥ d` | LRA |
| `RelativeVelocityGT(a1,a2,v)` | `\|vx1-vx2\| > v` | LRA |

**Every proposition in the current catalog is linear.** This is a change
from the encoding this document previously described: `TTCGT` and
`PedestrianTTCGT` look like they multiply two symbolic quantities
(`ttc * relative_velocity`), but `ttc` is a scalar the scenario spec fixes
before encoding starts: it becomes a Z3 rational constant the moment
`encode_ttc_constraint()` builds `real_from_f64(min_ttc)`, not a free
variable. `min_ttc_val * rel_vel` is therefore constant × variable, exactly
like every other bound in this document. Four propositions this document
used to list (`Distance2DGT`, `ManhattanDistanceGT`, `OnLeftOf`, `OnRightOf`)
have since been removed from the enum entirely, since they were never emitted by
any scenario type, so they cost real-arithmetic decisions in the compiler
and reviewer attention for zero encoding effect.

`Approaching` and `PedestrianTTCGuard` exist because a guarded implication
like `G(TTCGT(...))` is satisfied vacuously by two actors that never
converge – an `enforce`d `min_ttc` then constrains nothing at all unless
something else forces the guard true somewhere. Each scenario type that
relies on a TTC bound also asserts `G(condition ⟹ Guard(...))` with a
condition the scenario already forces true (e.g., a lane match, or
`F(CrossingRoad(pedestrian))`), so the TTC bound cannot be satisfied by
never triggering it.

`Ahead(a1, a2)` reads its comparison direction from the *pair*, not from
`a1` alone: for two actors travelling the same direction it is that shared
direction; for a pair travelling opposite directions (as `head_on` and
`overtake_left` both construct) it falls back to the fixed road frame
(`+x`). Reading the frame off `a1` alone would make `Ahead(a,b)` and
`Ahead(b,a)` both satisfiable at once for a mixed-direction pair, which
breaks the antisymmetry these scenario types depend on.

### 5.3 Constraint Modes

```
enforce (default)   G(atom)              -- must hold at every step
violate              F(NOT atom)          -- must fail at some step
ignore                (nothing asserted)
```

`push_constraint()` (`src/scenarios/mod.rs`) additionally tracks each atom's
polarity (`Positive`/`Negated`) so that, e.g., a `min_velocity` bound, whose
*safe* condition is "speed at or above the floor", negates correctly under
`violate` even though the atom itself is phrased as the safe condition
rather than the unsafe one. `ignore` returns without pushing anything.

---

## 6. Multi-Scenario Blocking Clauses

Source: `src/solver/multi_solve.rs` (`create_blocking_clause()`)

Between scenarios in `num_scenarios > 1` mode, a blocking clause rules out
regenerating a near-duplicate of a prior scenario. For every non-ego actor:

```
px_close = |px[a][0] - prev_px[a][0]| <= 0.5     LRA
vx_close = |vx[a][0] - prev_vx[a][0]| <= 0.2     LRA
```

For a pedestrian, the lateral pair is checked too:

```
py_close = |py[a][0] - prev_py[a][0]| <= 0.5     LRA
vy_close = |vy[a][0] - prev_vy[a][0]| <= 0.2     LRA
```

and the actor counts as "close to the prior solution" only if all of
`px_close, vx_close, py_close, vy_close` hold; a vehicle actor uses only the
longitudinal pair. The overall blocking clause is the disjunction of
`NOT(all axes close)` across every non-ego actor – at least one actor must
differ from the previous scenario on the axes checked for its role. All
LRA; adds a handful of assertions per prior scenario.

---

## 7. TTC and Distance Constraints (Y-Proximity Encoding)

Source: `src/solver/encoder.rs` (`encode_ttc_constraint()`,
`encode_approaching()`), `src/solver/encoder_utils.rs`
(`encode_same_lane_constraint()`)

TTC and longitudinal-distance propositions gate on a shared "same lane"
predicate rather than a bare discrete lane match:

```
same_lane = (lane[a1][t] = lane[a2][t])                    LIA
            OR (|py[a1][t] - py[a2][t]| < lane_width)       LRA
```

The lateral-proximity disjunct matters specifically during a lane-change
transition: `lane` is derived from `py` (§3.3, §4.5) and can lag `py` by up
to one lane width inside a transition window, so a discrete-only match would
let two vehicles pass within a lane width of each other, mid-manoeuvre,
without either safety proposition ever engaging. The same predicate backs
`DistanceGT`, `TTCGT`, and the optimizer's `directed_conflict()` – one
definition, not three that could silently diverge.

```
TTC constraint (both directions of "who's ahead"):
  same_lane AND actor1_ahead AND actor2_faster
    => (px1 - px2) >= min_ttc * (vx2 - vx1)      LRA (min_ttc is a constant)
  same_lane AND actor2_ahead AND actor1_faster
    => (px2 - px1) >= min_ttc * (vx1 - vx2)      LRA

Distance constraint:
  same_lane => (px1 - px2 >= min_dist) OR (px2 - px1 >= min_dist)     LRA
```

`Approaching(follower, leader)` (§5.2) is deliberately lane-free: it is used
as the *antecedent* of a guarded implication, and hoisting the (disjunctive)
lane test into the hypothesis rather than the consequent turned a measured
103-second solve into 14 seconds on a `cut_in_left` corpus – a disjunction Z3
must satisfy inside a consequent is search, the same disjunction as a
hypothesis is propagation.

---

## 8. Trajectory Extraction

Source: `extract_actor_trajectory()` in both `cartesian.rs` and
`bicycle.rs`

The Cartesian encoder's extraction is the identity map: `px, py, vx, vy, ax,
ay, lane` are read straight out of the solved model.

The bicycle encoder's extraction reconstructs the Cartesian output fields
from the bicycle state:

| Output field | Source |
|---------------|--------|
| `position.x`, `position.y` | `px[t]`, `py[t]`, read directly |
| `velocity.vx` | `longitudinal_vel[t]`, i.e. `direction * v[t]` for a vehicle, so it agrees in sign with the `px` update in §4.3.1 |
| `velocity.vy` | `v̄ * theta[t]`, where `v̄` is the reference speed `reference_speed_at()` recovers for that step (§4.3.4); falls back to the raw `v[t]` where the heading is pinned to zero and no bucket was asserted, which gives the same answer either way since `theta = 0` there |
| `acceleration.ax` | `a[t] * direction_sign`, direction-signed to match the signed `vx` above; `d(vx)/dt` is `direction * dv/dt`, not `dv/dt` alone, once `vx` itself carries the sign |
| `acceleration.ay` | `ay[t]`, read directly, a real solver variable and not a hard-coded zero |
| `theta[t]` | extracted internally, not part of the exported `State` |

`vx ≈ v` (not `v * cos(theta)`) is a small-angle choice made to match the
`px` integration in §4.3.1, which itself uses `v`, not `v * cos(theta)`; at
`|θ| ≤ atan(0.15)` the omitted `cos(θ)` factor is within 1.1% of 1, and
using it in extraction without also using it in the kinematics would put the
exported velocity out of step with the exported position.

---

## 9. Cartesian vs Bicycle – Side by Side

| Property | Cartesian | Bicycle (Hybrid LRA) |
|----------|-----------|-----------------------|
| Variables per actor per step | 7 (6 Real + 1 Int) | 8 Real + 1 Int (9 for a `direction: -1` vehicle) |
| Non-linear terms, anywhere | 0 | 0 |
| Lane coupling | `to_real(lane) * lw` (mixed LIA+LRA) | Concrete `L * lw` (pure LRA/LIA, no coercion) |
| Lateral velocity | Independent `vy`, ratio-bounded by `vx` | Derived: `vy = v̄ * θ`, plus the same ratio and a flat cap |
| Heading tracking | None (implicit, via the `vy`/`vx` ratio) | Explicit `θ`, `δ` variables, driving `vy` |
| Steering constraints | None | Angle and rate limits, both linear |
| Acceleration profile | Range bound only, no forced constant-`a` | Same |
| Direction encoding | `vx ≥ 0` / `vx ≤ 0` | Sign baked into the `px` update |
| Phase-specific pinning | Implicit (lane coupling holds `py`/`vy` still) | Explicit: `vy = θ = δ = 0` outside lane changes |

---

## 10. Performance Guide

With no non-linear term anywhere in the standard pipeline, "avoid NRA" is no
longer the operative piece of advice it once was. What now drives solve time:

| Factor | Effect |
|--------|--------|
| Number of actor pairs | Pairwise safety propositions are `O(A²)` per time step |
| Horizon length | Every stage in §3.9/§4.10 is at least linear in `H` |
| Lane-change window width and bucket count (bicycle only) | Heading coupling cost is `O(A·W·N)` – long transitions on wide-speed-range actors cost more, not more than that |
| Guarded vs. Z3-chosen disjunctions | A hypothesis-side disjunction is propagation; the same disjunction in a consequent, or as a free Bool selector, is search (§4.3.4, §7) |
| `min_ttc: ignore` / `min_distance: ignore` | Removes those propositions' `O(A²·H)` contribution outright |

### UNSAT diagnostics

If Z3 returns UNSAT, common causes remain kinematic rather than theoretic:

1. **Lane change too short.** With a `0.15` lateral ratio and a `2.0` m/s
   absolute cap, the achievable lateral speed at low `v` is `0.15*v`; a 3.5 m
   lane change then needs at least `3.5 / (0.15*v)` seconds – at 15 m/s,
   about 1.6 s, not the sub-second figure the old, more permissive ratio
   would have implied.
2. **Conflicting safety constraints.** A tight `min_ttc`/`min_distance` can
   conflict with a lane-change schedule that necessarily brings two actors
   laterally close for a window.
3. **A speed range too narrow given the acceleration band and horizon**, so
   no trajectory satisfies both the range and the kinematics.

---

## 11. Historical Note: Two Rounds of Linearization

The bicycle encoder has gone through two designs, not one, and it is worth
naming both so a change to the current one isn't mistaken for reintroducing
the first.

**Round 1 – the exact model.** The original encoder asserted `vy = v*θ` and
`θ[t+1] = θ[t] + (v*δ/L)*dt` directly, both products of two symbolic
variables. With the lane `Int` in every problem this was `QF_NIRA`, which
has no decision procedure; Z3 could not reliably solve even ten-step
scenarios.

**Round 2: an independent lateral velocity.** The fix at the time made `vy` an
independent variable bounded by a fixed ratio to `v` (`|vy| ≤ k*v`, `k =
0.5`), used a linear rate bound for `θ`, and pinned `θ = δ = 0` outside lane
changes. This eliminated the non-linear terms, but `θ` and `δ` were now
related to the vehicle's actual lateral motion by nothing at all: deleting
every `θ`/`δ` constraint would not have changed a single exported number,
and the `k = 0.5` ratio (a ±30° heading) admitted lateral speeds, 8 m/s at
16 m/s forward speed, that the Cartesian encoder would never have allowed on the
same YAML.

**Round 3: the current design (§4).** The reference-speed-bucket coupling
in §4.3.4 makes `θ` and `δ` load-bearing: `vy` is now *derived* from `θ`
rather than bounded independently of it, and `k` was brought down to `0.15`
to match the Cartesian encoder exactly. The linearization technique that
makes this possible, a disjunction of guarded linear constraints, one per
speed bucket, standing in for one non-linear one, is the piece this
document did not previously describe, because it did not exist yet.
