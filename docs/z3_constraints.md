# Z3 Constraint Reference: Cartesian and Bicycle Encoders

This document walks through every Z3 assertion added during scenario generation,
organized by pipeline stage, for both the Cartesian and Bicycle coordinate systems.
It explains what type of arithmetic each constraint uses and why that matters for
solver performance.

---

## 1. Z3 Theory Primer

Z3 is an SMT (Satisfiability Modulo Theories) solver. It dispatches constraints to
specialized sub-solvers depending on which *theory* the constraints belong to:

| Theory | Full Name | What it covers | Solver used | Speed |
|--------|-----------|----------------|-------------|-------|
| **LRA** | Linear Real Arithmetic | Addition, subtraction, scalar multiplication, comparisons of Real variables | Simplex | Very fast — polynomial time |
| **LIA** | Linear Integer Arithmetic | Same as LRA but for Int variables | Omega test / branch-and-bound | Fast |
| **NRA** | Non-linear Real Arithmetic | Any multiplication of two *symbolic* Real variables | NLSAT / CAD | Slow — can be exponential |
| **Mixed LIA+LRA** | Combined | `to_real(int_var) * real_const` | Combined DPLL(T) | Moderate |

**Key rule:** `constant × symbolic_var` is **LRA**. `symbolic_var × symbolic_var` is **NRA**.

Example:
```
15.0 * theta[t]          -- LRA  (15.0 is a constant rational)
v[t] * theta[t]          -- NRA  (both are free symbolic variables)
0.5 * v[t]               -- LRA  (0.5 is a constant rational)
```

If Z3 sees *any* NRA term in the formula, it hands the whole problem to NLSAT,
which can hang for problems that would be trivial under Simplex.

---

## 2. Encoding Pipeline

Both encoders follow the same call sequence (from `src/solver/multi_solve.rs`
and `src/lib.rs`):

```
Step 1  encoder.create_variables()
        → allocate all Z3 variables for t = 0 … horizon

Step 2  encoder.encode_initial_conditions()
        → fix starting state at t = 0

Step 3  encoder.encode_kinematics()
        → add motion equations linking each t to t+1
          [bicycle only: also calls encode_lane_coupling_with_lane_changes()
           and encode_bicycle_constraints() here]

Step 4  encoder.encode_velocity_constraints()
        → bound speed upper limit for all actors at all time steps

Step 5  encoder.encode_acceleration_constraints()
        → bound acceleration range and enforce constant-a for all actors

Step 6  encoder.encode_lane_velocity_constraints()
        → integer lane bounds and single-lane-jump constraint

Step 7  encoder.encode_lateral_velocity_bounds()
        → [Cartesian: bound vy; Bicycle: no-op]

Step 8  encoder.encode_ltl(formula)
        → expand G / F operators over [0, horizon]

Step 9  encoder.encode_scenario_specific_constraints(model)
        → scenario-type-specific Z3 constraints (currently no-ops for cut-in)

Step 10 [blocking clauses, one per prior scenario in multi-scenario mode]
```

Steps 4 and 5 (`encode_velocity_constraints` and `encode_acceleration_constraints`)
are called from the pipeline *after* kinematics. The bicycle encoder also calls
lane coupling and bicycle-specific bounds from *inside* `encode_kinematics()`.

---

## 3. Cartesian Encoder

### 3.1 Variables (`create_variables()`)

Source: `src/solver/encoders/cartesian.rs`

Per actor, per time step `t ∈ [0, horizon]` (horizon+1 values each):

```
px[a][t]   : Real   longitudinal position (m)
py[a][t]   : Real   lateral position (m)
vx[a][t]   : Real   longitudinal velocity (m/s)
vy[a][t]   : Real   lateral velocity (m/s)
ax[a][t]   : Real   longitudinal acceleration (m/s²)
ay[a][t]   : Real   lateral acceleration (m/s²)
lane[a][t] : Int    discrete lane index
```

**7 variables per actor per time step** (6 Real + 1 Int).

For 2 actors and horizon = 50 (5 s at 0.1 s/step): **714 variables** total.

---

### 3.2 Initial Conditions (`encode_initial_conditions()`)

Source: `src/solver/encoders/cartesian.rs`, `encode_actor_initial_state()`

For each actor at `t = 0`:

| Variable | Condition | Formula | Type |
|----------|-----------|---------|------|
| `lane[0]` | always | `lane[0] = lane_spec` | LIA |
| `px[0]` | fixed value | `px[0] = pos` | LRA |
| `px[0]` | range | `pos_min ≤ px[0] ≤ pos_max` | LRA |
| `vx[0]` | fixed, forward | `vx[0] = +speed` | LRA |
| `vx[0]` | range, forward | `speed_min ≤ vx[0] ≤ speed_max` | LRA |
| `vx[0]` | range, backward | `-speed_max ≤ vx[0] ≤ -speed_min` | LRA |
| `vy[0]` | always | `vy[0] = 0` | LRA |
| `ax[0]` | fixed value | `ax[0] = accel` | LRA |
| `ax[0]` | range | `accel_min ≤ ax[0] ≤ accel_max` | LRA |
| `ay[0]` | always | `ay[0] = 0` | LRA |
| `py[0]` | always (via lane coupling) | `py[0] = lane[0]·lw + lw/2` | Mixed LIA+LRA |

**~11 assertions per actor.**

---

### 3.3 Kinematics (`encode_kinematics()`)

Source: `src/solver/encoders/cartesian.rs`

For each actor, for each `t ∈ [0, horizon)`:

| Assertion | Formula | Type | Notes |
|-----------|---------|------|-------|
| vx update | `vx[t+1] = vx[t] + ax[t]·dt` | **LRA** | dt is a constant rational |
| px update | `px[t+1] = px[t] + vx[t]·dt` | **LRA** | dt is a constant rational |
| py update | `py[t+1] = py[t] + vy[t]·dt` | **LRA** | |
| Lane-position coupling | `py[t] = to_real(lane[t])·lw + lw/2` | **Mixed LIA+LRA** | lane is Int → to_real() → multiply by constant lw |
| Ego lateral velocity | `vy[t] = 0` | **LRA** | Ego stays in lane |

For pedestrians, additional `vy` and `ay` update equations apply (also LRA).

**Lane-position coupling** is the only place an integer variable appears in an
arithmetic expression with reals. `to_real(lane[t]) * lw` is linear (lw is
constant), so this stays in the combined LRA+LIA theory rather than NRA.

**No `vy[t] = ...` or `ay[t] = ...` kinematics for NPC vehicles without lane changes** —
lateral velocity is only non-zero during transitions.

---

### 3.4 Lane Change Transition (`encode_smooth_lane_transition()`)

Source: `src/solver/encoders/cartesian.rs`

During the lane change window `[start_step, end_step]`:

| Assertion | Formula | Type | Notes |
|-----------|---------|------|-------|
| Source position (soft) | `py[start] ≈ src_center ± 0.5m` | LRA | 2 bounds |
| Target position (soft) | `py[end] ≈ tgt_center ± 0.5m` | LRA | 2 bounds |
| Lane assignment pre-change | `lane[t] = src_lane` for `t < end_step` | LIA | |
| Lane assignment at end | `lane[end_step] = tgt_lane` | LIA | |
| Lateral accel bounds | `-2.0 ≤ ay[t] ≤ 2.0` | LRA | per step in window |
| **Velocity ratio constraint** | `|vy[t]| ≤ 0.15 · |vx[t]|` | **LRA** | k=0.15, constant × variable |

The velocity ratio constraint uses `k = 0.15` (corresponding to ~8.5° max heading).
`0.15 * vx[t]` is LRA because 0.15 is a constant. For forward lanes, `abs_vx = vx`
(already positive); for backward lanes, `abs_vx = -vx`.

---

### 3.5 Velocity Constraints (`encode_velocity_constraints()`)

Source: `src/solver/encoders/cartesian.rs`

For each actor, all time steps:

| Assertion | Formula | Type |
|-----------|---------|------|
| Speed upper bound | `vx[t] ≤ speed_max` | LRA |

(Lower bound is handled by lane direction constraints.)

---

### 3.6 Acceleration Constraints (`encode_acceleration_constraints()`)

Source: `src/solver/encoders/cartesian.rs`

For each actor, all time steps:

| Assertion | Formula | Type |
|-----------|---------|------|
| Accel lower bound | `ax[t] ≥ accel_min` | LRA |
| Accel upper bound | `ax[t] ≤ accel_max` | LRA |
| Constant acceleration | `ax[t+1] = ax[t]` | LRA |

The constant-acceleration chain forces Z3 to pick a single acceleration value for
the entire horizon. Combined with `vx[t+1] = vx[t] + ax[t]·dt`, the speed profile
becomes monotonically linear. This prevents the solver from oscillating `a[t]`
freely (which caused jagged speed profiles).

---

### 3.7 Lane and Velocity Direction Constraints (`encode_lane_velocity_constraints()`)

Source: `src/solver/encoders/cartesian.rs`

For each actor, all time steps:

| Assertion | Formula | Type |
|-----------|---------|------|
| Forward lane direction | `vx[t] ≥ 0` | LRA |
| Backward lane direction | `vx[t] ≤ 0` | LRA |
| Lane lower bound | `lane[t] ≥ 0` | LIA |
| Lane upper bound | `lane[t] ≤ num_lanes - 1` | LIA |

For non-pedestrian actors, single-lane-jump:

| Assertion | Formula | Type |
|-----------|---------|------|
| Jump constraint | `-1 ≤ lane[t+1] - lane[t] ≤ 1` | LIA |

---

### 3.8 Lateral Velocity Bounds (`encode_lateral_velocity_bounds()`)

For non-ego actors, all time steps:
```
-2.0 ≤ vy[t] ≤ 2.0       LRA
```

---

### 3.9 Cartesian Constraint Count (cut_in_left, 2 actors, 50 steps)

| Stage | Count | Type |
|-------|-------|------|
| Variables | 714 total | — |
| Initial conditions | ~22 | LRA/LIA/Mixed |
| Kinematics (px, py, vx per step) | 3 × 50 × 2 = 300 | LRA |
| Lane coupling per step | 51 × 2 = 102 | Mixed LIA+LRA |
| Velocity upper bounds | 51 × 2 = 102 | LRA |
| Acceleration bounds + constant-a | (2 + 1) × 51 × 2 = 306 | LRA |
| Velocity direction | 51 × 2 = 102 | LRA |
| Lane bounds | 51 × 2 × 2 = 204 | LIA |
| Single-lane-jump | 50 × 2 = 100 | LIA |
| Lateral velocity bounds (NPC) | 51 = 51 | LRA |
| Lane change ratio (window only) | ~30 × 2 = 60 | LRA |
| LTL safety (G TTC, 51 steps) | 51 | NRA (via TTCGT) |
| LTL safety (G Distance, 51 steps) | 51 | LRA |
| **Total** | **~1,450** | ~96% LRA/LIA, ~4% NRA |

Z3 handles Cartesian scenarios in < 2 seconds because NRA is limited to
TTC propositions (which use `distance > ttc · rel_vel`).

---

## 4. Bicycle Encoder (Hybrid LRA)

The bicycle encoder uses a **hybrid LRA approach** that retains the bicycle model's
value (heading tracking, steering constraints, turn radius limits) while keeping
all kinematic constraints in LRA for efficient solving.

### Design Principle

The original bicycle encoder encoded the kinematic bicycle model directly:
```
vy[t] = v[t] · θ[t]                    NRA — product of two symbolic variables
θ[t+1] = θ[t] + (v[t]·δ[t]/L)·dt      NRA — product of two symbolic variables
```

These NRA terms caused Z3 to hang even on 10-step problems.

The hybrid approach eliminates NRA by:
1. Making `vy` an **independent variable** (not derived from `v·θ`)
2. Bounding `vy` with a **linear ratio constraint**: `|vy| ≤ k · v`
3. Using **linear rate bounds** for heading instead of NRA dynamics
4. Enforcing **phase-specific constraints**: `vy=0, θ=0, δ=0` during straight driving

This preserves the physical realism of the bicycle model (vehicles have heading,
steering angle limits, turn radius constraints) while staying entirely in LRA.

---

### 4.1 Variables (`create_variables()`)

Source: `src/solver/encoders/bicycle.rs`

Per actor, per time step `t ∈ [0, horizon]`:

```
px[a][t]    : Real   longitudinal position (m)
py[a][t]    : Real   lateral position (m)
theta[a][t] : Real   heading angle (rad) — deviation from nominal direction
v[a][t]     : Real   scalar speed (m/s, always ≥ 0)
delta[a][t] : Real   front-wheel steering angle (rad)
a[a][t]     : Real   longitudinal acceleration (m/s²)
lane[a][t]  : Int    discrete lane index
vy[a][t]    : Real   lateral velocity (m/s) — independent variable
```

**8 variables per actor per time step** (7 Real + 1 Int).

For 2 actors and horizon = 50: **816 variables** total (vs 714 for Cartesian).

Note: `theta` represents deviation from the actor's nominal direction (0 for both
forward and backward vehicles). There is no separate `vx` variable — longitudinal
velocity is `v` (always positive). Direction is encoded in the `px` kinematics.

---

### 4.2 Initial Conditions (`encode_initial_conditions()`)

Source: `src/solver/encoders/bicycle.rs`, `encode_actor_initial_state()`

For each actor at `t = 0`:

| Variable | Condition | Formula | Type |
|----------|-----------|---------|------|
| `lane[0]` | always | `lane[0] = lane_spec` | LIA |
| `px[0]` | fixed value | `px[0] = pos` | LRA |
| `px[0]` | range | `pos_min ≤ px[0] ≤ pos_max` | LRA |
| `py[0]` | always | `py[0] = lane·lw + lw/2` | **LRA** (constant, not mixed) |
| `v[0]` | fixed value | `v[0] = speed` | LRA |
| `v[0]` | range | `speed_min ≤ v[0] ≤ speed_max` | LRA |
| `theta[0]` | always | `theta[0] = 0` | LRA |
| `delta[0]` | always | `delta[0] = 0` (straight) | LRA |
| `a[0]` | fixed value | `a[0] = accel` | LRA |
| `a[0]` | range | `accel_min ≤ a[0] ≤ accel_max` | LRA |

**~10 assertions per actor.**

Key differences from Cartesian:
- `py[0]` uses a **constant** computed from `lane_spec` (a Rust `usize`), not from
  the symbolic `lane[0]` variable. Pure LRA, no integer conversion.
- `theta[0] = 0` for all actors regardless of direction. Backward direction is
  handled in the `px` kinematics (`px[t+1] = px[t] - v[t]·dt`), keeping `θ`
  near zero so the small-angle approximation remains valid.

---

### 4.3 Kinematics — All LRA (`encode_kinematics()`)

Source: `src/solver/encoders/bicycle.rs`

This is the critical stage. The hybrid approach ensures **zero NRA terms** in
the kinematic equations.

#### 4.3.1 Motion Equations

For each non-pedestrian actor, for each `t ∈ [0, horizon)`:

| Assertion | Formula | Type | Notes |
|-----------|---------|------|-------|
| px update (forward) | `px[t+1] = px[t] + v[t]·dt` | **LRA** | dt is constant |
| px update (backward) | `px[t+1] = px[t] - v[t]·dt` | **LRA** | direction is a Rust constant |
| py update | `py[t+1] = py[t] + vy[t]·dt` | **LRA** | vy is independent |
| v update | `v[t+1] = v[t] + a[t]·dt` | **LRA** | dt is constant |

**3 LRA assertions per actor per step.** No NRA anywhere.

Note: `vy` is an independent variable, NOT derived from `v·θ`. This is the key
design decision that eliminates NRA.

#### 4.3.2 Phase-Specific Constraints

The encoder computes lane change schedules from the YAML spec and classifies
each time step as either **stable** (straight driving) or **lane change**.

For each non-pedestrian actor, for all `t ∈ [0, horizon]`:

**Stable phase** (not in any lane change):

| Assertion | Formula | Type | Reason |
|-----------|---------|------|--------|
| No lateral motion | `vy[t] = 0` | LRA | Vehicle drives straight |
| No heading deviation | `theta[t] = 0` | LRA | Aligned with road |
| No steering | `delta[t] = 0` | LRA | Wheels straight |

**Lane change phase** — ratio bounds are set in `encode_smooth_lane_transition_bicycle()`.

#### 4.3.3 Heading Rate Constraint

For steps where `t` or `t+1` is in a lane change:

| Assertion | Formula | Type | Notes |
|-----------|---------|------|-------|
| Heading rate bound | `-R·dt ≤ θ[t+1] - θ[t] ≤ R·dt` | **LRA** | R is a Rust constant |

Where `R = v_max · δ_max / L`:
- `v_max` = actor's speed upper bound (from YAML)
- `δ_max` = max steering angle (from bicycle config)
- `L` = wheelbase (from bicycle config)

This is the **linearized heading dynamics**. The original NRA equation was
`θ[t+1] = θ[t] + (v[t]·δ[t]/L)·dt` — a product of two symbolic variables.
The linearized bound uses a constant `R` computed at the Rust level, keeping
the constraint in LRA.

The bound is conservative: it uses `v_max` (worst case) rather than `v[t]`
(symbolic). At lower speeds, the actual heading rate would be smaller, so
the bound allows more freedom than the exact model.

---

### 4.4 Bicycle-Specific Constraints (`encode_bicycle_constraints()`)

Source: `src/solver/encoders/bicycle.rs`

Called from inside `encode_kinematics()` after the motion equations.

For each non-pedestrian actor, for all `t ∈ [0, horizon]`:

| Assertion | Formula | Type | Reason |
|-----------|---------|------|--------|
| Steering lower | `δ[t] ≥ -δ_max` | LRA | Physical steering limit |
| Steering upper | `δ[t] ≤ +δ_max` | LRA | Physical steering limit |
| Heading lower | `θ[t] ≥ -π/6` | LRA | Small-angle validity (30°) |
| Heading upper | `θ[t] ≤ +π/6` | LRA | Small-angle validity (30°) |
| Speed non-negative | `v[t] ≥ 0` | LRA | Speed is a magnitude |

For each `t ∈ [0, horizon)`:

| Assertion | Formula | Type | Reason |
|-----------|---------|------|--------|
| Steering rate lower | `δ[t+1] - δ[t] ≥ -max_rate·dt` | LRA | Smooth steering |
| Steering rate upper | `δ[t+1] - δ[t] ≤ +max_rate·dt` | LRA | Smooth steering |

Default values from YAML (`bicycle_config`):
- `δ_max = 0.6 rad` (~34°)
- `max_steering_rate = 0.5 rad/s` → max change per step = `0.5 × dt`
- Heading bound: `±π/6 ≈ ±0.524 rad` (±30°)

**All LRA.** 5 + 2 assertions per actor per step.

---

### 4.5 Lane Coupling (`encode_lane_coupling_with_lane_changes()`)

Source: `src/solver/encoders/bicycle.rs`

Called from inside `encode_kinematics()`. Uses **concrete lane indices** computed
from the YAML spec at encode time — no symbolic Int variables appear in arithmetic.

#### For actors without lane changes:

For all `t ∈ [0, horizon]`, where `L = initial_lane` (a Rust `usize`):

```
py[t] ≥ L · lw              LRA  (constant lower bound)
py[t] ≤ (L+1) · lw          LRA  (constant upper bound)
lane[t] = L                  LIA  (discrete lane variable tied to position)
```

#### For actors with lane changes (3 phases):

**Phase 1 — before lane change** (`t ∈ [0, start_step)`):
```
py[t] ≥ L · lw              LRA
py[t] ≤ (L+1) · lw          LRA
lane[t] = L                  LIA
```

**Phase 2 — during transition** (`t ∈ [start_step, end_step]`),
handled by `encode_smooth_lane_transition_bicycle()`:
```
py[start] ≈ src_center ± 0.5m       LRA  (soft start constraint)
py[end] ≈ tgt_center ± 0.5m         LRA  (soft end constraint)
lane[t] = src_lane  (for t < end)    LIA  (discrete lane during transition)
lane[end] = tgt_lane                  LIA  (discrete lane at end)
|vy[t]| ≤ 0.5 · v[t]                LRA  (velocity ratio bound)
py[t] ≥ 0                            LRA  (road boundary)
py[t] ≤ num_lanes · lw               LRA  (road boundary)
```

The velocity ratio uses `k = 0.5`, corresponding to the `±30°` heading bound
(`sin(π/6) = 0.5`). This is more permissive than the Cartesian encoder's `k = 0.15`
because the bicycle model uses heading/steering constraints for realism rather
than a tight `vy` ratio. `0.5 · v[t]` is LRA (constant times variable).

**Phase 3 — after lane change** (`t ∈ [end_step+1, horizon]`), where `L' = L ± 1`:
```
py[t] ≥ L' · lw              LRA
py[t] ≤ (L'+1) · lw          LRA
lane[t] = L'                  LIA
```

All phase boundaries and lane indices are Rust constants computed before any
Z3 assertions are made. **This entire stage is pure LRA/LIA — no mixed
Int×Real multiplication.**

---

### 4.6 Velocity Constraints (`encode_velocity_constraints()`)

Source: `src/solver/encoders/bicycle.rs`

For each actor, all time steps:

| Assertion | Formula | Type |
|-----------|---------|------|
| Speed upper bound | `v[t] ≤ speed_max` | LRA |

Where `speed_max = actor.speed.max()` from the YAML specification.

---

### 4.7 Acceleration Constraints (`encode_acceleration_constraints()`)

Source: `src/solver/encoders/bicycle.rs`

For each actor, for all `t ∈ [0, horizon]`:

```
a[t] ≥ a_min    LRA
a[t] ≤ a_max    LRA
```

For each `t ∈ [0, horizon)` — **constant acceleration enforcement**:

```
a[t+1] = a[t]   LRA
```

This chain forces Z3 to pick a single acceleration value for the entire horizon.
Combined with `v[t+1] = v[t] + a[t]·dt`, the speed profile becomes monotonically
linear. This prevents the solver from oscillating `a[t]` freely.

---

### 4.8 Lane and Velocity Constraints (`encode_lane_velocity_constraints()`)

Source: `src/solver/encoders/bicycle.rs`

For each actor, all `t`:

| Assertion | Formula | Type | Notes |
|-----------|---------|------|-------|
| Lane lower | `lane[t] ≥ 0` | LIA | |
| Lane upper | `lane[t] ≤ num_lanes - 1` | LIA | |

For all non-pedestrian actors, each `t ∈ [0, horizon)`:

| Assertion | Formula | Type | Notes |
|-----------|---------|------|-------|
| Jump constraint | `-1 ≤ lane[t+1] - lane[t] ≤ 1` | LIA | No multi-lane jumps |

Note: This applies to **all** non-pedestrian actors including ego, unlike
the Cartesian encoder which only applies it to non-ego actors.

**No direction-based velocity constraint** (`vx ≥ 0` etc.) exists in the bicycle
encoder. Direction is encoded via the sign in `px` kinematics:
- Forward (`direction = 1`): `px[t+1] = px[t] + v[t]·dt`
- Backward (`direction = -1`): `px[t+1] = px[t] - v[t]·dt`

The speed `v[t]` is always non-negative (enforced in `encode_bicycle_constraints()`).

---

### 4.9 Lateral Velocity Bounds (`encode_lateral_velocity_bounds()`)

Currently a no-op for the bicycle encoder. Lateral velocity bounds are handled
by the phase-specific constraints:
- Stable phase: `vy[t] = 0`
- Lane change: `|vy[t]| ≤ 0.5 · v[t]`

---

### 4.10 Bicycle Constraint Count

#### `bicycle_minimal.yaml` (10 steps, 2 actors, dt=0.5s)

| Stage | Count | Type |
|-------|-------|------|
| Variables | 176 total | — |
| Initial conditions | ~20 | LRA/LIA |
| Kinematics (px, py, v) | 3 × 10 × 2 = 60 | LRA |
| Phase constraints (stable: vy=0, θ=0, δ=0) | ~50 | LRA |
| Heading rate (lane change steps only) | ~10 | LRA |
| Bicycle bounds (steering, heading, speed) | 5 × 11 × 2 = 110 | LRA |
| Steering rate | 2 × 10 × 2 = 40 | LRA |
| Lane coupling (py bounds + lane var) | ~66 | LRA/LIA |
| Lane change transition (ratio + road bounds) | ~20 | LRA/LIA |
| Velocity upper bounds | 11 × 2 = 22 | LRA |
| Acceleration bounds + constant-a | (2 + 1) × 11 × 2 = 66 | LRA |
| Lane bounds + jump constraints | ~42 | LIA |
| LTL (min_ttc/min_distance: ignore) | 0 | — |
| **Total** | **~506** | **100% LRA/LIA** |

#### `cut_in_left_bicycle.yaml` (100 steps, 2 actors, dt=0.1s)

| Stage | Count | Type |
|-------|-------|------|
| Variables | 1,616 total | — |
| Kinematics (px, py, v) | 3 × 100 × 2 = 600 | LRA |
| Phase constraints | ~400 | LRA |
| Heading rate | ~120 | LRA |
| Bicycle bounds + steering rate | ~1,400 | LRA |
| Lane coupling + transitions | ~400 | LRA/LIA |
| Velocity + acceleration bounds | ~700 | LRA |
| Lane constraints | ~400 | LIA |
| LTL safety (G TTC, 101 steps) | 101 | NRA (via TTCGT) |
| LTL safety (G Distance, 101 steps) | 101 | LRA |
| **Total** | **~4,200** | ~97.5% LRA/LIA, ~2.5% NRA |

The only NRA in the bicycle encoder comes from `TTCGT` propositions in the
LTL formula (`distance > ttc · rel_vel`). Setting `min_ttc: ignore` removes
all NRA entirely.

---

## 5. LTL Expansion (shared by both encoders)

Source: `src/solver/encoder.rs`, `encode_ltl_bounded()`

### 5.1 Temporal Operators

```
G(φ)  [Always]     →  φ[0] ∧ φ[1] ∧ … ∧ φ[horizon]   (51 assertions for 50 steps)
F(φ)  [Eventually] →  φ[0] ∨ φ[1] ∨ … ∨ φ[horizon]   (1 disjunction, 51 clauses)
φ U ψ [Until]      →  ψ[t] ∨ (φ[t] ∧ φU ψ at t+1)     (recursive expansion)
```

The `Always` (G) operator is the most common — it's how `Enforce` mode works.
For a 50-step horizon, one `G(φ)` produces 51 individual assertions.

### 5.2 Proposition Types

Source: `src/solver/encoder.rs`, `encode_proposition()`

| Proposition | Formula | Type | When used |
|-------------|---------|------|-----------|
| `InLane(a, L)` | `lane[a][t] = L` | LIA | Behavior formulas |
| `Ahead(a1, a2)` | `px[a1][t] > px[a2][t]` | LRA | Behavior formulas |
| `DistanceGT(a1,a2,d)` | `|px1-px2| > d` | LRA | Safety (min_distance) |
| `TTCGT(a1,a2,ttc)` | `dist > ttc · rel_vel` (with implication) | **NRA** | Safety (min_ttc) |
| `VelocityGT(a,v)` | `|vx[a][t]| > v` | LRA | Speed limits |
| `VelocityLT(a,v)` | `|vx[a][t]| < v` | LRA | Speed limits |
| `LateralDistanceGT(a1,a2,d)` | `|py1-py2| > d` | LRA | Side clearance |
| `RelativeVelocityGT(a1,a2,v)` | `|vx1-vx2| > v` | LRA | Following distance |
| `OnLeftOf(a1,a2)` | `py[a1][t] > py[a2][t]` | LRA | Lateral ordering |
| `OnRightOf(a1,a2)` | `py[a1][t] < py[a2][t]` | LRA | Lateral ordering |
| `Distance2DGT(a1,a2,d)` | `(dx)² + (dy)² > d²` | **NRA** | Pedestrian safety |
| `ManhattanDistanceGT(a1,a2,d)` | `|dx| + |dy| > d` | LRA | Pedestrian safety |
| `RectangularDistanceGT(...)` | `|dx| > tx OR |dy| > ty` | LRA | Pedestrian safety |
| `PedestrianTTCGT(...)` | `ped_px - ego_px > ttc · ego_vx` | **NRA** | Pedestrian crossing |

**NRA propositions**: `TTCGT`, `Distance2DGT`, `PedestrianTTCGT`.
All others are LRA or LIA.

`TTCGT` is the most impactful because it is used in default safety constraints
and gets expanded to `horizon + 1` assertions by `G(TTCGT(...))`.

### 5.3 Constraint Modes

For each safety constraint (TTC, distance, etc.), three modes are available:

| Mode | Behavior | LTL formula |
|------|----------|-------------|
| `enforce` (default) | Must hold at all times | `G(constraint)` |
| `violate` | Must be violated at some point | `F(NOT constraint)` |
| `ignore` | No constraint added | (nothing) |

Setting `min_ttc: ignore` removes all TTCGT assertions, potentially eliminating
all NRA from the entire problem.

---

## 6. Multi-Scenario Blocking Clauses

Source: `src/solver/multi_solve.rs`, `create_blocking_clause()`

Between scenarios (for `num_scenarios > 1`), a blocking clause prevents re-generating
the same scenario. For each non-ego actor, it defines a "same solution" region:

```
px_same[i] = (prev_px[i] - 0.5 ≤ px[i][0] ≤ prev_px[i] + 0.5)   LRA
vx_same[i] = (prev_vx[i] - 0.2 ≤ vx[i][0] ≤ prev_vx[i] + 0.2)   LRA
```

The blocking clause asserts that at least one actor must be *outside* its region:

```
¬(px_same[0] ∧ vx_same[0]) ∨ ¬(px_same[1] ∧ vx_same[1]) ∨ …     Boolean/LRA
```

All LRA. Adds ~4–6 assertions per prior scenario.

---

## 7. TTC and Distance Constraints (Bicycle-specific encoding)

Source: `src/solver/encoders/bicycle.rs`, `encode_ttc_constraint()` and
`encode_distance_constraint()`

Both the TTC and distance constraints in the bicycle encoder use an enhanced
"same lane" condition that handles lane change transitions:

```
same_lane = lane[a1][t] == lane[a2][t]           LIA (discrete match)
         OR (|py[a1] - py[a2]| < lane_width)     LRA (y-proximity)
```

The y-proximity check uses AND to correctly encode absolute value:
```
(py1 - py2 < lw) AND (py2 - py1 < lw)           LRA
```

**TTC constraint** (NRA due to `distance > ttc · relative_velocity`):
```
If same_lane AND actor1_ahead AND actor2_faster:
    (px1 - px2) > min_ttc · (v2 - v1)            NRA
If same_lane AND actor2_ahead AND actor1_faster:
    (px2 - px1) > min_ttc · (v1 - v2)            NRA
```

**Distance constraint** (LRA):
```
If same_lane:
    (px1 - px2 ≥ min_dist) OR (px2 - px1 ≥ min_dist)    LRA
```

---

## 8. Trajectory Extraction

Source: `src/solver/encoders/bicycle.rs`, `extract_actor_trajectory()`

After Z3 finds a satisfying model, the bicycle encoder extracts trajectories
and converts to the common Cartesian output format:

| Extracted variable | Source | Output field |
|--------------------|--------|-------------|
| `px` | `positions_x[actor][t]` | `position.x` |
| `py` | `positions_y[actor][t]` | `position.y` |
| `vy` | `velocities_y[actor][t]` | `velocity.vy` |
| `v` | `speed_v[actor][t]` | `velocity.vx` (vx ≈ v, small angle) |
| `a` | `accelerations[actor][t]` | `acceleration.ax` |
| `lane` | `lanes[actor][t]` | `lane` |
| `theta` | `heading_theta[actor][t]` | (extracted but not in output) |

Note: `vy` is extracted directly from the `velocities_y` variable (the independent
lateral velocity), not computed from `v · theta`. Lateral acceleration `ay` is
set to 0 in the output (could be computed from steering if needed).

---

## 9. Cartesian vs Bicycle — Side-by-Side

| Property | Cartesian | Bicycle (Hybrid LRA) |
|----------|-----------|----------------------|
| Variables per actor per step | 7 (6R + 1I) | 8 (7R + 1I) |
| Total variables (2 actors, 50 steps) | 714 | 816 |
| Kinematic NRA terms per step | 0 | **0** |
| NRA during lane change | 0 (vy ratio is LRA) | **0** (vy ratio is LRA) |
| NRA from TTCGT (G, 51 steps) | 51 | 51 |
| Lane coupling method | `to_real(lane) * lw` (Mixed LIA+LRA) | Concrete `L * lw` (pure LRA) |
| Lateral velocity constraint | `|vy| ≤ 0.15·|vx|` (k=0.15, ~8.5°) | `|vy| ≤ 0.5·v` (k=0.5, ~30°) |
| Acceleration profile | Constant (`a[t+1] = a[t]`) | Constant (`a[t+1] = a[t]`) |
| Direction encoding | `vx ≥ 0` or `vx ≤ 0` | `px ± v·dt` (sign in kinematics) |
| Heading tracking | No (implicit via vy/vx ratio) | Yes (θ, δ variables with bounds) |
| Steering constraints | No | Yes (angle + rate limits) |
| Phase-specific constraints | No | Yes (stable: vy=θ=δ=0) |
| Typical solve time (10 steps) | < 1 s | < 0.1 s |
| Typical solve time (100 steps) | < 2 s | < 1 s |

---

## 10. Performance Guide

### When bicycle scenarios solve quickly

| Condition | Why it helps |
|-----------|-------------|
| `constraint_modes: {min_ttc: ignore}` | Removes all TTCGT NRA assertions |
| Short horizon (`duration: 5.0`, `time_step: 0.5`) | Fewer variables and constraints |
| Fixed speed (`speed: 15.0`) with zero accel | More constrained = smaller search space |
| All three conditions combined | Problem is fully LRA — Simplex only |

### When bicycle scenarios may be slow

| Condition | Why it hurts |
|-----------|-------------|
| `min_ttc: enforce` with many actors | TTCGT is NRA, expanded for each actor pair × time step |
| Very long horizon (200+ steps) | Linear growth in constraint count |
| Very tight lane change (duration < lane_width / (k · v_min)) | May be UNSAT if vehicle can't traverse lane width fast enough |

### UNSAT diagnostics

If Z3 returns UNSAT for a bicycle scenario, common causes:
1. **Lane change too short**: At speed `v`, max lateral velocity is `0.5·v`. A 3.5m lane change requires at least `3.5 / (0.5·v)` seconds. At 15 m/s: min duration ≈ 0.47s.
2. **Conflicting safety constraints**: TTC and distance constraints may conflict with the lane change schedule.
3. **Speed range too narrow with acceleration**: Constant acceleration + velocity upper bound can create impossible trajectories.

---

## 11. Historical Note: NRA Elimination

The original bicycle encoder (pre-fix) used the exact kinematic bicycle model:

```
vy[t] = v[t] · θ[t]                        NRA — caused Z3 to invoke NLSAT
θ[t+1] = θ[t] + (v[t]·δ[t]/L)·dt          NRA — caused Z3 to invoke NLSAT
```

This produced **202 NRA assertions** for a 2-actor, 50-step scenario (vs 0 now).
Combined with TTCGT propositions, the total NRA count exceeded 300 assertions,
making Z3 unable to solve even 10-step problems within 60 seconds.

The fix replaced these with:
- Independent `vy` variable + linear ratio bound `|vy| ≤ 0.5·v` (LRA)
- Linear heading rate bound `|Δθ| ≤ R·dt` where R is a Rust constant (LRA)
- Phase-specific constraints that anchor stable phases to `vy=θ=δ=0` (LRA)

Result: Bicycle scenarios now solve in < 1 second for typical configurations,
comparable to or faster than the Cartesian encoder.
