//! YAML parser for DSL specifications

use super::types::{ActorRole, ActorSpec, ConstraintModes, ScenarioSpec, ScenarioType, ValueOrRange};
use crate::error::{Result, ScenarioGenError};
use serde::Deserialize;
use std::collections::HashMap;

/// Parse YAML string into ScenarioSpec
pub fn parse_yaml(yaml_content: &str) -> Result<ScenarioSpec> {
    // Try new format first
    match serde_yaml::from_str::<ScenarioSpec>(yaml_content) {
        Ok(spec) => {
            spec.validate().map_err(ScenarioGenError::InvalidSpec)?;
            Ok(spec)
        }
        Err(_) => {
            // Fall back to legacy format
            let legacy: LegacyScenarioSpec = serde_yaml::from_str(yaml_content)
                .map_err(ScenarioGenError::YamlParse)?;
            let spec = ScenarioSpec::from(legacy);
            spec.validate().map_err(ScenarioGenError::InvalidSpec)?;
            Ok(spec)
        }
    }
}

// Legacy format support
#[derive(Deserialize)]
struct LegacyScenarioSpec {
    scenario_type: ScenarioType,
    time_step: f64,
    duration: f64,
    ego: LegacyActorSpec,
    npc: LegacyNpcSpec,
    min_ttc: f64,
    min_distance: f64,
    lane_width: f64,
    #[serde(default)]
    constraint_modes: ConstraintModes,
    #[serde(default)]
    max_acceleration: Option<f64>,
    #[serde(default)]
    max_deceleration: Option<f64>,
    num_scenarios: usize,
}

#[derive(Deserialize)]
struct LegacyActorSpec {
    lane: usize,
    position: ValueOrRange,
    speed: ValueOrRange,
    acceleration: ValueOrRange,
}

#[derive(Deserialize)]
struct LegacyNpcSpec {
    lane: usize,
    position: ValueOrRange,
    speed: ValueOrRange,
    acceleration: ValueOrRange,
    cut_in_time: ValueOrRange,
}

impl From<LegacyScenarioSpec> for ScenarioSpec {
    fn from(legacy: LegacyScenarioSpec) -> Self {
        let ego_actor = ActorSpec {
            id: "ego".to_string(),
            role: ActorRole::Ego,
            lane: legacy.ego.lane,
            position: legacy.ego.position,
            speed: legacy.ego.speed,
            acceleration: legacy.ego.acceleration,
            behavior: HashMap::new(),
        };

        let mut npc_behavior = HashMap::new();
        npc_behavior.insert(
            "cut_in_time".to_string(),
            serde_json::to_value(legacy.npc.cut_in_time).unwrap(),
        );

        let npc_actor = ActorSpec {
            id: "npc".to_string(),
            role: ActorRole::Npc,
            lane: legacy.npc.lane,
            position: legacy.npc.position,
            speed: legacy.npc.speed,
            acceleration: legacy.npc.acceleration,
            behavior: npc_behavior,
        };

        ScenarioSpec {
            scenario_type: legacy.scenario_type,
            time_step: legacy.time_step,
            duration: legacy.duration,
            actors: vec![ego_actor, npc_actor],
            min_ttc: legacy.min_ttc,
            min_distance: legacy.min_distance,
            lane_width: legacy.lane_width,
            constraint_modes: legacy.constraint_modes,
            max_acceleration: legacy.max_acceleration,
            max_deceleration: legacy.max_deceleration,
            num_scenarios: legacy.num_scenarios,
        }
    }
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
  acceleration: [-8.0, 3.0]

npc:
  lane: 0
  position: [60.0, 80.0]
  speed: [12.0, 14.0]
  cut_in_time: [2.5, 7.5]
  acceleration: [-8.0, 3.0]

min_ttc: 3.0
min_distance: 5.0
lane_width: 3.5

num_scenarios: 1
"#;

        let spec = parse_yaml(yaml).unwrap();
        assert_eq!(
            spec.scenario_type,
            super::super::types::ScenarioType::CutInLeft
        );
        assert_eq!(spec.time_step, 0.5);
        assert_eq!(spec.ego().lane, 1);
        assert_eq!(spec.npcs().next().unwrap().position.min(), 60.0);
    }

    #[test]
    fn test_parse_invalid_time_step() {
        let yaml = r#"
scenario_type: cut_in_left
time_step: -0.5
duration: 10.0
ego: { lane: 1, position: 50.0, speed: 15.0, acceleration: [-8.0, 3.0] }
npc: { lane: 0, position: 60.0, speed: 13.0, cut_in_time: 5.0, acceleration: [-8.0, 3.0] }
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
ego: { lane: 1, position: 50.0, speed: 15.0, acceleration: [-8.0, 3.0] }
npc: { lane: 0, position: 65.0, speed: 13.0, cut_in_time: 5.0, acceleration: [-8.0, 3.0] }
min_ttc: 3.0
min_distance: 5.0
lane_width: 3.5
num_scenarios: 1
"#;

        let spec = parse_yaml(yaml).unwrap();
        let npc = spec.npcs().next().unwrap();
        assert!(npc.position.is_fixed());
        assert_eq!(npc.position.min(), 65.0);
    }
}
