# Z3 Constraint Solving Strategy

## Overview
How Z3 constraint solving should interact with Frenet coordinate generation.

## Options

### Option 1: Solve Entirely in Frenet Space
- Z3 variables: s(t), t(t), θ(t)
- Constraints: lane boundaries, collision (in Frenet)
- No Cartesian conversion during solving

### Option 2: Solve Entirely in Cartesian Space
- Frenet only for pre/post processing
- Z3 solves in Cartesian as currently
- Frenet used only to generate initial conditions

### Option 3: Hybrid: Frenet Generation + Z3 Refinement
- Generate smooth Frenet trajectory (quintic polynomial)
- Convert to waypoints
- Use Z3 to adjust within tolerance for constraints
- Combine: smooth base + Z3 refinements

## Trade-offs

| Factor | Frenet Solving | Cartesian Solving | Hybrid |
|--------|----------------|-------------------|--------|
| Smoothness guarantee | High (if no Z3) | Low | Medium-High |
| Constraint accuracy | Low (collision) | High | High |
| Computational cost | Low | High | Medium |
| Implementation complexity | Low | Low (current) | High |
| Flexibility | Low | High | High |

## Challenges with Frenet-Space Solving

### 1. Collision Detection Complexity
```rust
// Easy in Cartesian
let dist_sq = (x1 - x2).powi(2) + (y1 - y2).powi(2);

// Complex in Frenet (reference lines may differ!)
// Only accurate if vehicles share similar reference line
// For different lanes or roads, need Cartesian conversion
```

### 2. Road Curvature Effects
- On curves, same s doesn't mean same longitudinal position
- Lateral distance depends on curvature
- Collision constraints become non-linear

## Recommendation
**Hybrid: Frenet generation + Z3 refinement with tight tolerances**

### Rationale:
- Quintic polynomials guarantee smooth lateral motion
- Z3 handles complex constraints (collision, timing)
- Tight tolerance prevents Z3 from breaking smoothness
- Best balance of both approaches

## Implementation (COMPLETED)

**Status:** Implemented as coordinate-specific encoders with trait-based architecture

The actual implementation uses **Option 1: Solve Entirely in Frenet Space** for Frenet coordinate system scenarios, and **Option 2: Solve Entirely in Cartesian Space** for Cartesian scenarios. The "hybrid" approach is achieved at the encoder level, not within a single solving process.

### Actual Implementation Strategy

1. **Coordinate-specific encoders** (`src/solver/encoders/`):
   - `FrenetEncoder`: Z3 variables are `frenet_s`, `frenet_t`, `frenet_vs`, `frenet_vt` at each time step
   - `CartesianEncoder`: Z3 variables are `positions_x`, `positions_y`, `velocities_x`, `velocities_y` at each time step

2. **Trait-based dispatch** (`src/solver/encoder.rs`):
   - `GenericEncoder` facade chooses appropriate encoder based on `spec.coordinate_system`
   - All constraint encoding uses coordinate-agnostic accessor methods

3. **Collision detection in Cartesian** (even for Frenet scenarios):
   - TTC and distance constraints computed by converting to Cartesian for evaluation
   - Z3 constraints encoded in native coordinate system
   - Metrics computed in Cartesian for consistency

### FrenetEncoder Implementation

```rust
pub struct FrenetEncoder<B: Backend> {
    // Z3 variables for each actor at each time step
    frenet_s: HashMap<String, Vec<dyn Ast>>,
    frenet_t: HashMap<String, Vec<dyn Ast>>,
    frenet_vs: HashMap<String, Vec<dyn Ast>>,
    frenet_vt: HashMap<String, Vec<dyn Ast>>,
    frenet_lane: HashMap<String, Vec<dyn Ast>>,
    // ...
}

impl<B: Backend> CoordinateEncoder<B> for FrenetEncoder<B> {
    fn encode_kinematics(&self, backend: &B, spec: &ScenarioSpec) -> Result<()> {
        // Encode: s[t+1] = s[t] + vs[t] * dt
        // Encode: t[t+1] = t[t] + vt[t] * dt
        // Encode lateral velocity bounds for smooth lane changes
    }
}
```

### Key Differences from Original Design

1. **No hybrid within single solve**: Each scenario solves entirely in one coordinate system
2. **No quintic pre-generation**: Z3 directly solves for Frenet variables with smoothness constraints
3. **No waypoint conversion**: Coordinate conversion only happens during trajectory extraction/output
4. **Trait-based architecture**: Clean separation between coordinate systems rather than hybrid approach

### Smoothness Constraints

Instead of quintic polynomial pre-generation, smoothness is enforced via Z3 constraints:

- **Lateral velocity bounds**: `vt_min <= vt[t] <= vt_max` prevents sudden lateral movements
- **Lateral acceleration limits**: Derived from consecutive `vt` values
- **Lane assignment**: Integer variables constrain vehicles to discrete lanes

### Benefits of Implemented Design

- **Simpler architecture**: No pre-generation + refinement pipeline
- **Direct Z3 solving**: All constraints encoded directly in Z3
- **Coordinate consistency**: No back-and-forth conversion during solving
- **Trait-based extensibility**: Easy to add new coordinate systems

## Context Dependencies
- **domain/core-concepts.md**: Frenet definitions
- **processes/trajectory-generation.md**: Quintic trajectory generation
- **decisions/polynomial-coefficients.md**: Coefficient handling

## Examples

```rust
// Generate smooth trajectory
let smooth = generate_smooth_lane_change(&start, &end, 6.0);

// Add Z3 constraints (e.g., avoid obstacle at s=50)
let obstacle = Obstacle { s: 50.0, t: 1.75, radius: 2.0 };
let result = apply_z3_constraints(&smooth, &[obstacle], &ref_line);
```
