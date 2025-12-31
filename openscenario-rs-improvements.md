# OpenSCENARIO-rs Builder Improvements

## Problem Statement

The `openscenario-rs` library (v0.2.0) is missing critical builder support for trajectory-based actions, making it difficult to programmatically create scenarios with `FollowTrajectoryAction`. This document outlines the gaps and provides guidance for implementing the missing builders.

**Repository**: https://github.com/ashfaqfarooqui/openscenario-rs
**Local checkout**: `~/.cargo/git/checkouts/openscenario-rs-b34f2df995ce1c44/6dd97b7/`

---

## Current State Analysis

### What Exists ✅

**Movement Action Builders** (`src/builder/actions/movement.rs`):
- ✅ `SpeedActionBuilder` - Set absolute/relative speed
- ✅ `TeleportActionBuilder` - Instant position changes

**Init Builders** (`src/builder/init/`):
- ✅ `InitBuilder` - Initialize scenarios
- ✅ `InitActionBuilder` - Add init actions

**Pattern Established**:
- Detached builder pattern for complex nested structures
- Fluent API with chainable methods
- Validation before building
- Clear separation of concerns

### What's Missing ❌

**Critical Gaps**:
1. ❌ `TrajectoryBuilder` - Build trajectory definitions
2. ❌ `PolylineBuilder` - Build polyline shapes
3. ❌ `VertexBuilder` - Build trajectory vertices
4. ❌ `FollowTrajectoryActionBuilder` - Build follow trajectory actions
5. ❌ Integration with detached maneuver builders

**Impact**:
- Cannot programmatically create trajectory-based scenarios
- Must manually construct complex nested types
- No validation or type safety for trajectories
- Inconsistent API compared to other actions

---

## Required Implementations

### 1. TrajectoryBuilder

**File**: `src/builder/actions/trajectory.rs` (NEW)

**Purpose**: Build `Trajectory` objects with polyline/nurbs/clothoid shapes

**API Design**:
```rust
pub struct TrajectoryBuilder {
    name: Option<String>,
    closed: bool,
    shape: Option<Shape>,
}

impl TrajectoryBuilder {
    pub fn new() -> Self
    pub fn name(self, name: &str) -> Self
    pub fn closed(self, closed: bool) -> Self
    pub fn polyline(self) -> PolylineBuilder
    pub fn build(self) -> BuilderResult<Trajectory>
}
```

**Reference Types** (already exist in `src/types/`):
- `types::actions::movement::Trajectory`
- `types::basic::Shape`

**Pattern to Follow**: Look at `SpeedActionBuilder` in `movement.rs` for validation and builder patterns

---

### 2. PolylineBuilder

**File**: Same as TrajectoryBuilder (`src/builder/actions/trajectory.rs`)

**Purpose**: Build polyline shapes with vertices

**API Design**:
```rust
pub struct PolylineBuilder {
    parent: TrajectoryBuilder,
    vertices: Vec<Vertex>,
}

impl PolylineBuilder {
    pub fn add_vertex(self) -> VertexBuilder
    pub fn finish(self) -> TrajectoryBuilder
}
```

**Reference Types**:
- `types::basic::Polyline`
- `types::basic::Vertex`

**Key Challenge**: Managing parent-child builder relationships while allowing fluent chaining

---

### 3. VertexBuilder

**Purpose**: Build individual trajectory vertices with time and position

**API Design**:
```rust
pub struct VertexBuilder {
    parent: PolylineBuilder,
    time: Option<f64>,
    position: Option<Position>,
}

impl VertexBuilder {
    pub fn time(self, time: f64) -> Self
    pub fn position(self, position: Position) -> Self
    pub fn world_position(self, x: f64, y: f64, z: f64, h: f64) -> Self
    pub fn finish(self) -> BuilderResult<PolylineBuilder>
}
```

**Integration Point**: Should reuse existing `PositionBuilder` from `src/builder/positions/`

**Reference Types**:
- `types::basic::Vertex`
- `types::positions::Position`

---

### 4. FollowTrajectoryActionBuilder

**Purpose**: Build FollowTrajectoryAction with trajectory reference

**API Design**:
```rust
pub struct FollowTrajectoryActionBuilder {
    entity_ref: Option<String>,
    trajectory: Option<Trajectory>,
    following_mode: Option<String>,
}

impl FollowTrajectoryActionBuilder {
    pub fn new() -> Self
    pub fn for_entity(self, entity_ref: &str) -> Self
    pub fn with_trajectory(self, trajectory: Trajectory) -> Self
    pub fn following_mode(self, mode: &str) -> Self  // "follow", "position"
    pub fn build_action(self) -> BuilderResult<PrivateAction>
}

impl ActionBuilder for FollowTrajectoryActionBuilder { ... }
impl ManeuverAction for FollowTrajectoryActionBuilder { ... }
```

**Reference Types**:
- `types::actions::movement::FollowTrajectoryAction`
- `types::actions::movement::TrajectoryFollowingMode`

**Pattern**: Must implement `ActionBuilder` and `ManeuverAction` traits like other action builders

---

### 5. Detached Builder Integration

**Files to Modify**:
- `src/builder/storyboard/maneuver.rs` - Add trajectory action creation
- `src/builder/storyboard/event.rs` - Support trajectory events

**Add Methods**:
```rust
// In ManeuverBuilder or similar
pub fn create_follow_trajectory_action(&mut self) -> DetachedFollowTrajectoryActionBuilder
```

**Pattern**: Follow the existing detached pattern used for speed actions in `examples/cut_in_scenario_demo.rs:48-57`

---

## Where to Look

### Study These Files First

1. **Existing Action Builders**:
   - `src/builder/actions/movement.rs` - SpeedAction and TeleportAction patterns
   - `src/builder/actions/lateral.rs` - LaneChangeAction for complex action examples
   - `src/builder/actions/base.rs` - ActionBuilder trait definitions

2. **Detached Builder Pattern**:
   - `src/builder/storyboard/maneuver.rs` - How detached builders work
   - `examples/cut_in_scenario_demo.rs` - Real usage of detached pattern
   - `examples/alks_scenario_builder_demo.rs` - Complex scenario with multiple actions

3. **Type Definitions**:
   - `src/types/actions/movement.rs` - Lines 152-220 (Trajectory, FollowTrajectoryAction)
   - `src/types/basic.rs` - Shape, Polyline, Vertex definitions
   - `src/types/positions/mod.rs` - Position types used in vertices

4. **Validation Patterns**:
   - `src/builder/validation.rs` - How validation is done
   - `src/builder/error.rs` - Error types to use

---

## Implementation Checklist

### Phase 1: Core Trajectory Builders
- [ ] Create `src/builder/actions/trajectory.rs`
- [ ] Implement `TrajectoryBuilder` with basic fields
- [ ] Implement `PolylineBuilder` with vertex collection
- [ ] Implement `VertexBuilder` with time and position
- [ ] Add validation for required fields
- [ ] Add unit tests for builders

### Phase 2: Action Builder
- [ ] Implement `FollowTrajectoryActionBuilder`
- [ ] Implement `ActionBuilder` trait
- [ ] Implement `ManeuverAction` trait
- [ ] Add validation (trajectory required, entity_ref required)
- [ ] Add unit tests

### Phase 3: Integration
- [ ] Export builders from `src/builder/actions/mod.rs`
- [ ] Add to `src/builder/mod.rs` public API
- [ ] Update documentation
- [ ] Add integration test in `tests/`

### Phase 4: Detached Pattern
- [ ] Add `create_follow_trajectory_action()` to maneuver builders
- [ ] Create `DetachedFollowTrajectoryActionBuilder`
- [ ] Implement attach pattern
- [ ] Add example demonstrating usage

---

## Critical Design Patterns to Follow

### 1. Builder Validation Pattern
```rust
impl TrajectoryBuilder {
    fn validate(&self) -> BuilderResult<()> {
        if self.name.is_none() {
            return Err(BuilderError::validation_error("Trajectory name is required"));
        }
        if self.shape.is_none() {
            return Err(BuilderError::validation_error("Trajectory shape is required"));
        }
        Ok(())
    }
}
```

### 2. Parent-Child Builder Pattern
```rust
pub struct ChildBuilder {
    parent: ParentBuilder,
    // ... child fields
}

impl ChildBuilder {
    pub fn finish(self) -> BuilderResult<ParentBuilder> {
        // Validate child
        // Add child data to parent
        Ok(self.parent)
    }
}
```

### 3. Type Conversion Pattern
Look at how `SpeedActionBuilder::build_action()` creates the final type:
- Unwrap validated options
- Create nested type structure
- Wrap in appropriate enum variants
- Return PrivateAction

### 4. Detached Builder Pattern
Study `examples/cut_in_scenario_demo.rs:46-59`:
```rust
let mut detached_action = maneuver.create_action_type();
detached_action.configure();
detached_action.attach_to_detached(&mut maneuver)?;
```

---

## Testing Strategy

### Unit Tests (in `trajectory.rs`)
```rust
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_trajectory_builder_basic() {
        let trajectory = TrajectoryBuilder::new()
            .name("test_trajectory")
            .closed(false)
            .polyline()
                .add_vertex()
                    .time(0.0)
                    .world_position(0.0, 0.0, 0.0, 0.0)
                    .finish()
                    .unwrap()
                .add_vertex()
                    .time(1.0)
                    .world_position(10.0, 0.0, 0.0, 0.0)
                    .finish()
                    .unwrap()
                .finish()
            .build()
            .unwrap();

        assert_eq!(trajectory.name, "test_trajectory");
        assert!(!trajectory.closed);
    }

    #[test]
    fn test_trajectory_validation_fails_without_name() {
        let result = TrajectoryBuilder::new()
            .polyline()
            .finish()
            .build();

        assert!(result.is_err());
    }
}
```

### Integration Test (in `tests/`)
Create a complete scenario with trajectory action and verify XML output

---

## Expected Usage After Implementation

```rust
use openscenario_rs::ScenarioBuilder;
use openscenario_rs::builder::StoryboardBuilder;

// Create scenario with trajectory
let scenario = ScenarioBuilder::new()
    .with_header("Trajectory Test", "Test")
    .with_entities()
    .add_vehicle("ego", |v| v.car());

// Create trajectory
let trajectory = TrajectoryBuilder::new()
    .name("ego_trajectory")
    .closed(false)
    .polyline()
        .add_vertex()
            .time(0.0)
            .world_position(0.0, 0.0, 0.0, 0.0)
            .finish()?
        .add_vertex()
            .time(1.0)
            .world_position(10.0, 0.0, 0.0, 0.0)
            .finish()?
        .finish()
    .build()?;

// Add to storyboard using detached pattern
let mut storyboard = StoryboardBuilder::new(scenario);
let mut story = storyboard.add_story_simple("main");
let mut act = story.create_act("movement");
let mut maneuver = act.create_maneuver("follow_path", "ego");

let trajectory_action = maneuver
    .create_follow_trajectory_action()
    .with_trajectory(trajectory)
    .following_mode("follow");

trajectory_action.attach_to_detached(&mut maneuver)?;
maneuver.attach_to_detached(&mut act);
act.attach_to(&mut story);

let scenario = storyboard.finish().build()?;
```

---

## Potential Issues to Watch For

### 1. Lifetime Management
The detached builder pattern uses lifetimes extensively. Be careful with:
- Mutable borrows in detached builders
- Parent-child relationships
- Builder consumption vs borrowing

**Solution**: Study `src/builder/storyboard/maneuver.rs` carefully

### 2. Type Complexity
`FollowTrajectoryAction` has multiple optional fields:
- `trajectory` (inline)
- `catalog_reference` (external)
- `trajectory_ref` (wrapper)

**Solution**: Start with inline trajectory only, add catalog support later

### 3. Position Types
Vertices can have different position types (World, Lane, Road, etc.)

**Solution**: Support WorldPosition first, add others incrementally

### 4. Shape Variants
Trajectory shapes can be Polyline, NURBS, or Clothoid

**Solution**: Implement Polyline only initially (most common)

---

## Success Criteria

Implementation is complete when:
- ✅ All builders compile without errors
- ✅ Unit tests pass for each builder
- ✅ Integration test creates valid OpenSCENARIO XML
- ✅ XML validates against OpenSCENARIO 1.0+ schema
- ✅ Example code in this document runs successfully
- ✅ Documentation is added to all public APIs
- ✅ Follows existing code style and patterns

---

## Additional Resources

**OpenSCENARIO Specification**:
- Latest spec: https://www.asam.net/standards/detail/openscenario/
- Trajectory section: Part of PrivateAction → RoutingAction → FollowTrajectoryAction

**Similar Implementations**:
- Look for OpenSCENARIO implementations in other languages for reference
- Check how other Rust builder libraries handle nested structures

**Rust Builder Patterns**:
- Study the `derive_builder` crate for comparison
- Review Rust API guidelines for builder patterns

---

## Questions for AI Agent

When implementing, consider:
1. Should VertexBuilder support all Position types or just WorldPosition?
2. Should we validate trajectory continuity (time ordering)?
3. How to handle optional trajectory parameters (speed profile, etc.)?
4. Should we add convenience methods for common trajectory patterns (straight line, arc)?
5. How to integrate with catalog-based trajectory references?

---

## Estimated Scope

- **Core builders**: ~300-400 lines
- **Tests**: ~200-300 lines
- **Integration**: ~100 lines
- **Documentation**: ~200 lines
- **Total**: ~800-1000 lines of code

**Complexity**: Medium-High (requires understanding of detached builder pattern)

**Time Estimate**: 4-8 hours for experienced Rust developer, longer for learning the codebase

---

## Contact & Contribution

If implementing this, consider:
1. Opening an issue on the openscenario-rs repository first
2. Discussing API design with maintainers
3. Submitting incremental PRs (core builders → actions → detached pattern)
4. Adding examples demonstrating the new functionality

This would be a valuable contribution to the OpenSCENARIO Rust ecosystem!
