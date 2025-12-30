# Phase 2: DSL Layer

**Prerequisites**: Phase 1 complete (project setup)

**Duration**: 1 day

---

## Context

The DSL (Domain-Specific Language) layer provides the user-facing interface for scenario specification. Users write simple YAML files describing scenarios at a high level, and we parse them into structured Rust types.

**Why this phase**: We need a way for users to specify scenarios without writing LTL formulas or Z3 constraints manually. The DSL hides complexity and provides an intuitive interface.

**What problem it solves**: Bridges the gap between user intent and formal specification.

---

## Goals

- [ ] Define DSL data structures (ScenarioSpec, ActorSpec, etc.)
- [ ] Implement YAML parsing using serde
- [ ] Add input validation
- [ ] Write unit tests for parsing
- [ ] Test with example YAML file

---

## Implementation Steps

### Step 1: Define DSL Types

**File**: `src/dsl/types.rs`

```rust
//! DSL data structures for scenario specification

use serde::{Deserialize, Serialize};

/// Root scenario specification
#[derive(Debug, Clone, Deserialize)]
pub struct ScenarioSpec {
    pub scenario_type: ScenarioType,
    pub time_step: f64,              // seconds per discretization step
    pub duration: f64,               // total scenario duration (seconds)
    pub ego: ActorSpec,
    pub npc: NpcSpec,
    pub min_ttc: f64,                // minimum time-to-collision (seconds)
    pub min_distance: f64,           // minimum longitudinal distance (meters)
    pub lane_width: f64,             // meters
    pub num_scenarios: usize,        // 1 for single, N for multiple
}

#[derive(Debug, Clone, Copy, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum ScenarioType {
    CutInLeft,
}

/// Ego vehicle specification (fixed parameters)
#[derive(Debug, Clone, Deserialize)]
pub struct ActorSpec {
    pub lane: usize,
    pub position: f64,               // meters from start
    pub speed: f64,                  // m/s
}

/// NPC vehicle specification (with ranges for solver)
#[derive(Debug, Clone, Deserialize)]
pub struct NpcSpec {
    pub lane: usize,
    pub position: ValueOrRange,      // starting position
    pub speed: ValueOrRange,         // velocity
    pub cut_in_time: ValueOrRange,   // when to perform lane change (seconds)
}

/// Value that can be either fixed or a range for Z3 to solve
#[derive(Debug, Clone, Deserialize)]
#[serde(untagged)]
pub enum ValueOrRange {
    Value(f64),
    Range([f64; 2]),                 // [min, max]
}

impl ValueOrRange {
    /// Get minimum value
    pub fn min(&self) -> f64 {
        match self {
            ValueOrRange::Value(v) => *v,
            ValueOrRange::Range([min, _]) => *min,
        }
    }

    /// Get maximum value
    pub fn max(&self) -> f64 {
        match self {
            ValueOrRange::Value(v) => *v,
            ValueOrRange::Range([_, max]) => *max,
        }
    }

    /// Check if this is a fixed value
    pub fn is_fixed(&self) -> bool {
        matches!(self, ValueOrRange::Value(_))
    }
}

impl ScenarioSpec {
    /// Validate the specification
    pub fn validate(&self) -> Result<(), String> {
        // Time parameters
        if self.time_step <= 0.0 {
            return Err("time_step must be positive".to_string());
        }
        if self.duration <= 0.0 {
            return Err("duration must be positive".to_string());
        }
        if self.duration < self.time_step {
            return Err("duration must be >= time_step".to_string());
        }

        // Safety constraints
        if self.min_ttc <= 0.0 {
            return Err("min_ttc must be positive".to_string());
        }
        if self.min_distance <= 0.0 {
            return Err("min_distance must be positive".to_string());
        }
        if self.lane_width <= 0.0 {
            return Err("lane_width must be positive".to_string());
        }

        // Generation parameters
        if self.num_scenarios == 0 {
            return Err("num_scenarios must be at least 1".to_string());
        }

        // Actor parameters
        if self.ego.speed <= 0.0 {
            return Err("ego speed must be positive".to_string());
        }
        if self.npc.speed.min() <= 0.0 {
            return Err("npc speed must be positive".to_string());
        }

        // Range validity
        if let ValueOrRange::Range([min, max]) = self.npc.position {
            if min >= max {
                return Err("npc position range invalid: min >= max".to_string());
            }
        }
        if let ValueOrRange::Range([min, max]) = self.npc.speed {
            if min >= max {
                return Err("npc speed range invalid: min >= max".to_string());
            }
        }
        if let ValueOrRange::Range([min, max]) = self.npc.cut_in_time {
            if min >= max {
                return Err("npc cut_in_time range invalid: min >= max".to_string());
            }
            if max > self.duration {
                return Err("npc cut_in_time max exceeds scenario duration".to_string());
            }
        }

        Ok(())
    }

    /// Get the number of time steps in the scenario
    pub fn num_time_steps(&self) -> usize {
        (self.duration / self.time_step).ceil() as usize
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_value_or_range_fixed() {
        let val = ValueOrRange::Value(10.0);
        assert_eq!(val.min(), 10.0);
        assert_eq!(val.max(), 10.0);
        assert!(val.is_fixed());
    }

    #[test]
    fn test_value_or_range_range() {
        let val = ValueOrRange::Range([5.0, 15.0]);
        assert_eq!(val.min(), 5.0);
        assert_eq!(val.max(), 15.0);
        assert!(!val.is_fixed());
    }

    #[test]
    fn test_num_time_steps() {
        let spec = create_test_spec();
        assert_eq!(spec.num_time_steps(), 20); // 10.0 / 0.5 = 20
    }

    fn create_test_spec() -> ScenarioSpec {
        ScenarioSpec {
            scenario_type: ScenarioType::CutInLeft,
            time_step: 0.5,
            duration: 10.0,
            ego: ActorSpec {
                lane: 1,
                position: 50.0,
                speed: 15.0,
            },
            npc: NpcSpec {
                lane: 0,
                position: ValueOrRange::Range([60.0, 80.0]),
                speed: ValueOrRange::Range([12.0, 14.0]),
                cut_in_time: ValueOrRange::Range([2.5, 7.5]),
            },
            min_ttc: 3.0,
            min_distance: 5.0,
            lane_width: 3.5,
            num_scenarios: 1,
        }
    }
}
```

**Key design choices**:
- `#[serde(untagged)]` on `ValueOrRange`: Allows YAML to be either `10.0` or `[5.0, 15.0]`
- `validate()` method: Catches invalid inputs early
- Helper methods (`min()`, `max()`, `num_time_steps()`): Make later code cleaner

### Step 2: Implement YAML Parser

**File**: `src/dsl/parser.rs`

```rust
//! YAML parser for DSL specifications

use super::types::ScenarioSpec;
use crate::error::{Result, ScenarioGenError};

/// Parse YAML string into ScenarioSpec
pub fn parse_yaml(yaml_content: &str) -> Result<ScenarioSpec> {
    let spec: ScenarioSpec = serde_yaml::from_str(yaml_content)
        .map_err(|e| ScenarioGenError::YamlParse(e))?;

    // Validate the parsed specification
    spec.validate()
        .map_err(|msg| ScenarioGenError::InvalidSpec(msg))?;

    Ok(spec)
}

/// Parse YAML file into ScenarioSpec
pub fn parse_yaml_file(path: &std::path::Path) -> Result<ScenarioSpec> {
    let content = std::fs::read_to_string(path)?;
    parse_yaml(&content)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_valid_yaml() {
        let yaml = r#"
scenario_type: cut_in_left

time_step: 0.5
duration: 10.0

ego:
  lane: 1
  position: 50.0
  speed: 15.0

npc:
  lane: 0
  position: [60.0, 80.0]
  speed: [12.0, 14.0]
  cut_in_time: [2.5, 7.5]

min_ttc: 3.0
min_distance: 5.0
lane_width: 3.5

num_scenarios: 1
"#;

        let spec = parse_yaml(yaml).unwrap();
        assert_eq!(spec.scenario_type, super::super::types::ScenarioType::CutInLeft);
        assert_eq!(spec.time_step, 0.5);
        assert_eq!(spec.ego.lane, 1);
        assert_eq!(spec.npc.position.min(), 60.0);
    }

    #[test]
    fn test_parse_invalid_time_step() {
        let yaml = r#"
scenario_type: cut_in_left
time_step: -0.5
duration: 10.0
ego: { lane: 1, position: 50.0, speed: 15.0 }
npc: { lane: 0, position: 60.0, speed: 13.0, cut_in_time: 5.0 }
min_ttc: 3.0
min_distance: 5.0
lane_width: 3.5
num_scenarios: 1
"#;

        let result = parse_yaml(yaml);
        assert!(result.is_err());
        if let Err(ScenarioGenError::InvalidSpec(msg)) = result {
            assert!(msg.contains("time_step"));
        } else {
            panic!("Expected InvalidSpec error");
        }
    }

    #[test]
    fn test_parse_fixed_values() {
        let yaml = r#"
scenario_type: cut_in_left
time_step: 0.5
duration: 10.0
ego: { lane: 1, position: 50.0, speed: 15.0 }
npc: { lane: 0, position: 65.0, speed: 13.0, cut_in_time: 5.0 }
min_ttc: 3.0
min_distance: 5.0
lane_width: 3.5
num_scenarios: 1
"#;

        let spec = parse_yaml(yaml).unwrap();
        assert!(spec.npc.position.is_fixed());
        assert_eq!(spec.npc.position.min(), 65.0);
    }
}
```

### Step 3: Update Module Exports

**src/dsl/mod.rs**:
```rust
//! DSL (Domain-Specific Language) module

pub mod types;
pub mod parser;

pub use types::{ScenarioSpec, ScenarioType, ActorSpec, NpcSpec, ValueOrRange};
pub use parser::{parse_yaml, parse_yaml_file};
```

### Step 4: Create Example YAML

**examples/cut_in_left.yaml**:
```yaml
scenario_type: cut_in_left

# Time configuration
time_step: 0.5        # 0.5 second discretization
duration: 10.0        # 10 second scenario

# Ego vehicle (controlled by AV under test)
ego:
  lane: 1             # right lane
  position: 50.0      # 50 meters from start
  speed: 15.0         # 15 m/s (54 km/h)

# NPC vehicle (background actor)
npc:
  lane: 0             # left lane
  position: [60.0, 80.0]   # start 60-80m from start (Z3 chooses)
  speed: [12.0, 14.0]      # slightly slower (Z3 chooses)
  cut_in_time: [2.5, 7.5]  # cut in between 2.5-7.5 seconds

# Safety constraints
min_ttc: 3.0              # minimum 3 second time-to-collision
min_distance: 5.0         # minimum 5 meter distance
lane_width: 3.5           # 3.5 meter lane width

# Generation settings
num_scenarios: 1          # generate 1 scenario (change to 5 for multiple)
```

### Step 5: Write Integration Test

**tests/integration_test.rs** (add to existing file):
```rust
use carla_scenario_generator::dsl;

#[test]
fn test_parse_example_yaml() {
    let yaml_path = std::path::Path::new("examples/cut_in_left.yaml");
    assert!(yaml_path.exists(), "Example YAML file should exist");

    let spec = dsl::parse_yaml_file(yaml_path)
        .expect("Should parse example YAML successfully");

    // Verify basic properties
    assert_eq!(spec.scenario_type, dsl::ScenarioType::CutInLeft);
    assert_eq!(spec.time_step, 0.5);
    assert_eq!(spec.duration, 10.0);
    assert_eq!(spec.num_time_steps(), 20);

    // Verify ego
    assert_eq!(spec.ego.lane, 1);
    assert_eq!(spec.ego.position, 50.0);
    assert_eq!(spec.ego.speed, 15.0);

    // Verify npc
    assert_eq!(spec.npc.lane, 0);
    assert_eq!(spec.npc.position.min(), 60.0);
    assert_eq!(spec.npc.position.max(), 80.0);
    assert!(!spec.npc.position.is_fixed());
}
```

---

## Success Criteria

### Verification Steps

1. **Unit tests pass**:
   ```bash
   cargo test dsl
   ```

2. **Integration test passes**:
   ```bash
   cargo test test_parse_example_yaml
   ```

3. **Example YAML parses successfully**:
   ```bash
   cargo test test_parse_example_yaml -- --nocapture
   ```

4. **Validation catches errors**:
   ```bash
   cargo test test_parse_invalid
   ```

### Checklist

- [ ] All DSL types defined in `types.rs`
- [ ] `ValueOrRange` handles both fixed and range values
- [ ] `ScenarioSpec::validate()` catches invalid inputs
- [ ] YAML parser in `parser.rs` works
- [ ] Example YAML file parses without errors
- [ ] Unit tests pass
- [ ] Integration test passes
- [ ] Code compiles without warnings (`cargo clippy`)

---

## Testing

```bash
# Run DSL tests
cargo test dsl

# Run with output
cargo test dsl -- --nocapture

# Test specific function
cargo test test_value_or_range_fixed

# Check code quality
cargo clippy --all-targets
```

---

## Common Issues

### Issue: serde deserialization fails

**Symptom**: "missing field" or "invalid type" errors

**Solution**: Check YAML structure matches Rust structs exactly. Field names must match (use `#[serde(rename = "...")]` if needed).

### Issue: ValueOrRange not parsing correctly

**Symptom**: Always parsing as Range even for fixed values

**Solution**: Ensure `#[serde(untagged)]` is on the enum. serde tries variants in order.

---

## Next Phase

Once this phase is complete and all tests pass:

**→ Continue to [Phase 3: LTL Layer](phase_03_ltl.md)**

Phase 3 will define LTL formulas and generate them from DSL specs.

---

## Notes for AI Agents

**What you just built**:
- DSL data structures matching simplified YAML format
- YAML parser with validation
- Helper methods for later phases
- Example YAML file

**What you can now do**:
- Parse user input
- Validate scenario specifications
- Access scenario parameters programmatically

**What's next**:
- Generate LTL formulas from these specs (Phase 3)
- Use these types throughout the codebase
