# Coordinate System Architecture Decision

## Overview
Choosing the coordinate system architecture for Frenet integration into test-4.

## Options

### Option 1: Frenet-Only
- All positions stored as (s, t, θ)
- No Cartesian storage internal to system
- Convert to Cartesian only for CARLA output

### Option 2: Cartesian-Only (Status Quo)
- Keep current Cartesian system
- Frenet calculations as pre-processing only
- No internal Frenet representation

### Option 3: Hybrid/Dual System
- Support both coordinate systems
- Runtime selection based on scenario type
- Conversion between representations as needed

## Trade-offs

| Factor | Frenet-Only | Hybrid | Cartesian-Only |
|--------|-------------|---------|----------------|
| Code simplicity | High | Low | High |
| Refactoring effort | High | Medium | None |
| Scenario flexibility | Low | High | Low (for road scenarios) |
| Constraint complexity | Low | Medium | High |
| Backward compatibility | No | Yes | Yes |
| Performance | High | Medium | High |

## Recommendation
**Hybrid system with default Frenet for road scenarios**

### Rationale:
- Allows gradual migration from Cartesian to Frenet
- Supports both road scenarios (Frenet) and unstructured areas (Cartesian)
- Provides backward compatibility for existing scenarios
- Can eventually deprecate Cartesian if Frenet proves superior

## Implementation (COMPLETED)

**Status:** Implemented as trait-based encoder architecture

The hybrid system is implemented using a **trait-based plugin architecture**:

### 1. CoordinateSystem Enum (`src/dsl/types.rs`)

```rust
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default, Serialize, Deserialize)]
pub enum CoordinateSystem {
    #[default]
    Frenet,   // (s, t) coordinates - default for road scenarios
    Cartesian, // (x, y) coordinates - for unstructured environments
}
```

**Key difference from design:** Uses `CoordinateSystem` enum (not `CoordinateMode`) in the DSL types. The `Position` enum approach was not used in favor of coordinate-specific encoder implementations.

### 2. CoordinateEncoder Trait (`src/solver/coordinate_encoder.rs`)

Defines interface for all coordinate-specific encoders:

```rust
pub trait CoordinateEncoder<B: Backend> {
    // Variable creation
    fn create_variables(&mut self, ctx: &Context, actors: &[ActorSpec], horizon: usize) -> Result<()>;

    // Kinematics encoding
    fn encode_kinematics(&self, backend: &B, spec: &ScenarioSpec) -> Result<()>;
    fn encode_initial_conditions(&self, backend: &B, spec: &ScenarioSpec) -> Result<()>;

    // Constraint encoding
    fn encode_velocity_constraints(&self, backend: &B, spec: &ScenarioSpec) -> Result<()>;
    fn encode_acceleration_constraints(&self, backend: &B, spec: &ScenarioSpec) -> Result<()>;
    fn encode_lane_velocity_constraints(&self, backend: &B, spec: &ScenarioSpec) -> Result<()>;
    fn encode_lateral_velocity_bounds(&self, backend: &B, spec: &ScenarioSpec) -> Result<()>;

    // Variable accessors (coordinate-agnostic interface)
    fn get_longitudinal_pos(&self, actor_id: &str, time: usize) -> &Dynamic<B::Dynamic>;
    fn get_lateral_pos(&self, actor_id: &str, time: usize) -> &Dynamic<B::Dynamic>;
    fn get_longitudinal_vel(&self, actor_id: &str, time: usize) -> &Dynamic<B::Dynamic>;
    fn get_lateral_vel(&self, actor_id: &str, time: usize) -> &Dynamic<B::Dynamic>;
    fn get_lane_var(&self, actor_id: &str, time: usize) -> &Dynamic<B::Dynamic>;

    // Trajectory extraction
    fn extract_actor_trajectory(&self, model: &Model, spec: &ScenarioSpec, actor_id: &str) -> Result<ActorTrajectory>;
}
```

### 3. GenericEncoder Facade (`src/solver/encoder.rs`)

Thin facade that dispatches to coordinate-specific encoders:

```rust
pub struct GenericEncoder<B: Backend> {
    coordinate_system: CoordinateSystem,
    encoder: Box<dyn CoordinateEncoder<B>>,
    // Coordinate-agnostic fields...
}

impl<B: Backend> GenericEncoder<B> {
    pub fn with_backend(backend: &B, spec: &ScenarioSpec) -> Result<Self> {
        let encoder: Box<dyn CoordinateEncoder<B>> = match spec.coordinate_system {
            CoordinateSystem::Frenet => Box::new(FrenetEncoder::new(backend, spec)?),
            CoordinateSystem::Cartesian => Box::new(CartesianEncoder::new(backend, spec)?),
        };
        // ...
    }
}
```

### 4. Coordinate-Specific Encoders

**CartesianEncoder** (`src/solver/encoders/cartesian.rs`):
- Variables: `positions_x`, `positions_y`, `velocities_x`, `velocities_y`, `lanes`
- Lane coupling: `py = lane * lane_width + lane_width/2`
- Use case: Unstructured environments, backward compatibility

**FrenetEncoder** (`src/solver/encoders/frenet.rs`):
- Variables: `frenet_s`, `frenet_t`, `frenet_vs`, `frenet_vt`, `frenet_lane`
- Smooth lane changes with lateral velocity constraints
- Use case: Road-based scenarios with lane changes

### 5. YAML Configuration

Coordinate system selected in scenario specification:

```yaml
coordinate_system: frenet  # or cartesian (default: frenet)
```

### Key Differences from Original Design

1. **No `Position` enum**: Instead of a single enum with variants, coordinate systems are handled entirely by separate encoder implementations
2. **Trait-based dispatch**: `GenericEncoder` uses trait objects (`Box<dyn CoordinateEncoder<B>>`) for runtime polymorphism
3. **Accessor methods**: Coordinate-agnostic access via trait methods (`get_longitudinal_pos()`, etc.) instead of direct field access
4. **No `CoordinateMode`**: Uses `CoordinateSystem` enum in DSL types instead

### Benefits of Implemented Design

- **Clean separation**: Each coordinate system in its own file (~700 lines each vs ~2360 lines in monolithic encoder)
- **Type-safe dispatch**: Enum-based selection at construction time
- **Extensibility**: Add new coordinate systems by implementing the trait
- **Backward compatible**: All existing scenarios work without modification

## Context Dependencies
- **domain/core-concepts.md**: Frenet coordinate definitions
- **processes/coordinate-conversion.md**: Frenet ↔ Cartesian conversion

## Examples

### YAML Configuration

```yaml
# Specify Frenet coordinates (default)
coordinate_system: frenet

actors:
  - id: ego
    lane: 1
    position: [0.0, 20.0]  # Frenet (s) range
    speed: 15.0

road:
  num_lanes: 2
  lane_width: 3.5
```

### Encoder Usage

```rust
// Encoder automatically selected based on coordinate_system
let encoder = GenericEncoder::with_backend(&backend, &spec)?;

// Accessor methods work for both coordinate systems
let px = encoder.get_longitudinal_pos("ego", 0);  // s (Frenet) or x (Cartesian)
let py = encoder.get_lateral_pos("ego", 0);      // t (Frenet) or y (Cartesian)

// Coordinate-agnostic LTL encoding
encoder.encode_ltl(&backend, &ltl_formula)?;
```

### Coordinate Conversion (for output)

```rust
// Trajectory extraction returns coordinates in system specified
let trajectory = encoder.extract_actor_trajectory(&model, &spec, "ego")?;

// Convert to Cartesian for CARLA/visualization if needed
let cartesian_traj = convert_trajectory_to_cartesian(&trajectory, &ref_line)?;
```
