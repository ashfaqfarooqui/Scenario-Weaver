# Phase 4: Scenario Model

**Prerequisites**: Phase 3 complete (LTL formulas defined)

**Duration**: 0.5 day

---

## Context

The scenario model defines the output format - JSON data structures representing complete driving scenarios with actor trajectories, positions, velocities, and validation info.

**Why this phase**: We need a clear schema for the output before we start extracting scenarios from Z3. This defines what a "scenario" looks like in our system.

**What problem it solves**: Provides type-safe, serializable representation of scenario solutions that can be validated, visualized, or transformed to other formats (e.g., OpenSCENARIO later).

---

## Goals

- [ ] Define Scenario output structure
- [ ] Define ActorTrajectory with state sequence
- [ ] Define State (position, velocity, lane at a time step)
- [ ] Define ValidationInfo for constraint checking
- [ ] Implement JSON serialization
- [ ] Create example expected output
- [ ] Write unit tests

---

## Implementation Steps

### Step 1: Define Scenario Model

**File**: `src/scenario/model.rs`

```rust
//! Scenario output data structures

use serde::{Deserialize, Serialize};

/// Complete scenario with all actor trajectories
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Scenario {
    /// Unique scenario identifier
    pub scenario_id: String,

    /// Type of scenario (e.g., "cut_in_left")
    pub scenario_type: String,

    /// Time discretization step (seconds)
    pub time_step: f64,

    /// Total duration (seconds)
    pub duration: f64,

    /// All actors and their trajectories
    pub actors: Vec<ActorTrajectory>,

    /// Validation information
    pub validation: ValidationInfo,
}

/// Trajectory of a single actor through the scenario
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ActorTrajectory {
    /// Actor identifier (e.g., "ego", "npc")
    pub id: String,

    /// Actor role
    pub role: String,

    /// Sequence of states over time
    pub states: Vec<State>,
}

/// State of an actor at a specific time
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct State {
    /// Time (seconds from start)
    pub time: f64,

    /// Position in world coordinates
    pub position: Position,

    /// Velocity
    pub velocity: Velocity,

    /// Current lane
    pub lane: usize,
}

/// 2D position
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Position {
    /// Longitudinal position (along road, meters)
    pub x: f64,

    /// Lateral position (across lanes, meters)
    pub y: f64,
}

/// 2D velocity
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Velocity {
    /// Longitudinal velocity (m/s)
    pub vx: f64,

    /// Lateral velocity (m/s, non-zero during lane changes)
    pub vy: f64,
}

/// Validation information for the scenario
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ValidationInfo {
    /// Minimum time-to-collision observed (seconds)
    pub min_ttc: f64,

    /// Minimum longitudinal distance observed (meters)
    pub min_distance: f64,

    /// Whether all constraints were satisfied
    pub all_constraints_satisfied: bool,

    /// List of any safety violations detected
    #[serde(default)]
    pub safety_violations: Vec<String>,
}

impl Scenario {
    /// Create a new scenario with basic metadata
    pub fn new(
        scenario_type: String,
        time_step: f64,
        duration: f64,
    ) -> Self {
        Self {
            scenario_id: uuid::Uuid::new_v4().to_string(),
            scenario_type,
            time_step,
            duration,
            actors: Vec::new(),
            validation: ValidationInfo {
                min_ttc: f64::INFINITY,
                min_distance: f64::INFINITY,
                all_constraints_satisfied: false,
                safety_violations: Vec::new(),
            },
        }
    }

    /// Add an actor trajectory to the scenario
    pub fn add_actor(&mut self, trajectory: ActorTrajectory) {
        self.actors.push(trajectory);
    }

    /// Get trajectory for a specific actor
    pub fn get_actor(&self, id: &str) -> Option<&ActorTrajectory> {
        self.actors.iter().find(|a| a.id == id)
    }

    /// Compute validation metrics from trajectories
    pub fn compute_validation(&mut self, min_ttc_required: f64, min_dist_required: f64) {
        // This will be implemented in Phase 9 when we have actual trajectories
        // For now, just placeholder
        self.validation.all_constraints_satisfied = true;
    }
}

impl ActorTrajectory {
    /// Create a new actor trajectory
    pub fn new(id: String, role: String) -> Self {
        Self {
            id,
            role,
            states: Vec::new(),
        }
    }

    /// Add a state to the trajectory
    pub fn add_state(&mut self, state: State) {
        self.states.push(state);
    }

    /// Get state at a specific time index
    pub fn state_at(&self, index: usize) -> Option<&State> {
        self.states.get(index)
    }

    /// Get the number of time steps
    pub fn num_steps(&self) -> usize {
        self.states.len()
    }
}

impl State {
    /// Create a new state
    pub fn new(time: f64, position: Position, velocity: Velocity, lane: usize) -> Self {
        Self {
            time,
            position,
            velocity,
            lane,
        }
    }
}

impl Position {
    /// Create a new position
    pub fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    /// Compute Euclidean distance to another position
    pub fn distance_to(&self, other: &Position) -> f64 {
        ((self.x - other.x).powi(2) + (self.y - other.y).powi(2)).sqrt()
    }

    /// Longitudinal distance (x-axis only)
    pub fn longitudinal_distance_to(&self, other: &Position) -> f64 {
        (self.x - other.x).abs()
    }

    /// Lateral distance (y-axis only)
    pub fn lateral_distance_to(&self, other: &Position) -> f64 {
        (self.y - other.y).abs()
    }
}

impl Velocity {
    /// Create a new velocity
    pub fn new(vx: f64, vy: f64) -> Self {
        Self { vx, vy }
    }

    /// Compute speed (magnitude of velocity)
    pub fn speed(&self) -> f64 {
        (self.vx.powi(2) + self.vy.powi(2)).sqrt()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_scenario_creation() {
        let mut scenario = Scenario::new(
            "cut_in_left".to_string(),
            0.5,
            10.0,
        );

        assert_eq!(scenario.scenario_type, "cut_in_left");
        assert_eq!(scenario.time_step, 0.5);
        assert_eq!(scenario.duration, 10.0);
        assert!(!scenario.scenario_id.is_empty());
    }

    #[test]
    fn test_actor_trajectory() {
        let mut traj = ActorTrajectory::new("ego".to_string(), "ego".to_string());

        let state = State::new(
            0.0,
            Position::new(50.0, 5.25),
            Velocity::new(15.0, 0.0),
            1,
        );

        traj.add_state(state);
        assert_eq!(traj.num_steps(), 1);
        assert_eq!(traj.state_at(0).unwrap().time, 0.0);
    }

    #[test]
    fn test_position_distance() {
        let p1 = Position::new(0.0, 0.0);
        let p2 = Position::new(3.0, 4.0);

        assert_eq!(p1.distance_to(&p2), 5.0);
        assert_eq!(p1.longitudinal_distance_to(&p2), 3.0);
        assert_eq!(p1.lateral_distance_to(&p2), 4.0);
    }

    #[test]
    fn test_velocity_speed() {
        let v = Velocity::new(3.0, 4.0);
        assert_eq!(v.speed(), 5.0);
    }

    #[test]
    fn test_json_serialization() {
        let mut scenario = Scenario::new(
            "cut_in_left".to_string(),
            0.5,
            10.0,
        );

        let mut ego_traj = ActorTrajectory::new("ego".to_string(), "ego".to_string());
        ego_traj.add_state(State::new(
            0.0,
            Position::new(50.0, 5.25),
            Velocity::new(15.0, 0.0),
            1,
        ));

        scenario.add_actor(ego_traj);

        // Test serialization
        let json = serde_json::to_string_pretty(&scenario).unwrap();
        println!("Serialized scenario:\n{}", json);

        // Test deserialization
        let deserialized: Scenario = serde_json::from_str(&json).unwrap();
        assert_eq!(deserialized.scenario_type, "cut_in_left");
        assert_eq!(deserialized.actors.len(), 1);
    }
}
```

### Step 2: Create Example Expected Output

**File**: `examples/expected_output.json`

```json
{
  "scenario_id": "a7b3c5d9-1e4f-4a8b-9c2d-3e5f6a7b8c9d",
  "scenario_type": "cut_in_left",
  "time_step": 0.5,
  "duration": 10.0,
  "actors": [
    {
      "id": "ego",
      "role": "ego",
      "states": [
        {
          "time": 0.0,
          "position": { "x": 50.0, "y": 5.25 },
          "velocity": { "vx": 15.0, "vy": 0.0 },
          "lane": 1
        },
        {
          "time": 0.5,
          "position": { "x": 57.5, "y": 5.25 },
          "velocity": { "vx": 15.0, "vy": 0.0 },
          "lane": 1
        },
        {
          "time": 1.0,
          "position": { "x": 65.0, "y": 5.25 },
          "velocity": { "vx": 15.0, "vy": 0.0 },
          "lane": 1
        }
      ]
    },
    {
      "id": "npc",
      "role": "npc",
      "states": [
        {
          "time": 0.0,
          "position": { "x": 72.5, "y": 1.75 },
          "velocity": { "vx": 13.0, "vy": 0.0 },
          "lane": 0
        },
        {
          "time": 0.5,
          "position": { "x": 79.0, "y": 1.75 },
          "velocity": { "vx": 13.0, "vy": 0.0 },
          "lane": 0
        },
        {
          "time": 4.5,
          "position": { "x": 131.0, "y": 3.5 },
          "velocity": { "vx": 13.0, "vy": 1.4 },
          "lane": 0
        },
        {
          "time": 5.0,
          "position": { "x": 137.5, "y": 5.25 },
          "velocity": { "vx": 13.0, "vy": 0.0 },
          "lane": 1
        }
      ]
    }
  ],
  "validation": {
    "min_ttc": 3.8,
    "min_distance": 6.2,
    "all_constraints_satisfied": true,
    "safety_violations": []
  }
}
```

### Step 3: Update Module Exports

**src/scenario/mod.rs**:
```rust
//! Scenario module

pub mod model;
pub mod extractor;

pub use model::{Scenario, ActorTrajectory, State, Position, Velocity, ValidationInfo};
```

### Step 4: Add Placeholder for Extractor

**src/scenario/extractor.rs**:
```rust
//! Scenario extraction from Z3 models (to be implemented in Phase 9)

use crate::scenario::model::Scenario;

/// Extract scenario from Z3 model (placeholder)
pub fn extract_scenario() -> Scenario {
    // Will be implemented in Phase 9
    Scenario::new("placeholder".to_string(), 0.5, 10.0)
}
```

---

## Success Criteria

### Verification Steps

1. **Unit tests pass**:
   ```bash
   cargo test scenario
   ```

2. **JSON serialization works**:
   ```bash
   cargo test test_json_serialization -- --nocapture
   ```
   Should print valid JSON

3. **Example JSON is valid**:
   ```bash
   # Validate JSON structure
   cat examples/expected_output.json | python -m json.tool
   ```

4. **Helper methods work**:
   ```bash
   cargo test test_position_distance
   cargo test test_velocity_speed
   ```

### Checklist

- [ ] All model structs defined
- [ ] JSON serialization/deserialization works
- [ ] Helper methods implemented (distance, speed, etc.)
- [ ] Example JSON file created
- [ ] Unit tests pass
- [ ] Code compiles without warnings

---

## Testing

```bash
# Run scenario model tests
cargo test scenario -- --nocapture

# Test JSON serialization
cargo test test_json_serialization -- --nocapture

# Check example JSON
python -m json.tool < examples/expected_output.json
```

---

## Common Issues

### Issue: serde serialization fails

**Symptom**: JSON output missing fields or has wrong format

**Solution**: Ensure all fields have `#[derive(Serialize, Deserialize)]` and check for `#[serde(default)]` where needed.

### Issue: UUID not found

**Symptom**: `uuid::Uuid` not recognized

**Solution**: Ensure `uuid` is in Cargo.toml dependencies with features = ["v4", "serde"]

---

## Next Phase

Once this phase is complete and all tests pass:

**→ Continue to [Phase 5: Z3 Foundation](phase_05_z3_foundation.md)**

Phase 5 will set up Z3 integration and create variables for the scenario.

---

## Notes for AI Agents

**What you just built**:
- Complete JSON output schema
- Type-safe scenario representation
- Helper methods for geometric calculations
- Example expected output

**What you can now do**:
- Serialize scenarios to JSON
- Work with trajectory data
- Validate scenario structure

**What's next**:
- Phase 5-8: Build Z3 encoder that produces data matching this schema
- Phase 9: Extract Z3 solutions into this schema
