//! YAML parser for DSL specifications

use super::types::ScenarioSpec;
use crate::error::{Result, ScenarioGenError};

/// Parse YAML string into ScenarioSpec
pub fn parse_yaml(yaml_content: &str) -> Result<ScenarioSpec> {
    let spec: ScenarioSpec =
        serde_yaml::from_str(yaml_content).map_err(ScenarioGenError::YamlParse)?;
    spec.validate().map_err(ScenarioGenError::InvalidSpec)?;
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

actors:
  - id: ego
    role: ego
    lane: 1
    position: 50.0
    speed: 15.0
    acceleration: [-8.0, 3.0]
  - id: npc
    role: npc
    lane: 0
    position: [60.0, 80.0]
    speed: [12.0, 14.0]
    acceleration: [-8.0, 3.0]
    behavior:
      cut_in_time: [2.5, 7.5]

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
actors:
  - id: ego
    role: ego
    lane: 1
    position: 50.0
    speed: 15.0
    acceleration: [-8.0, 3.0]
  - id: npc
    role: npc
    lane: 0
    position: 60.0
    speed: 13.0
    acceleration: [-8.0, 3.0]
    behavior:
      cut_in_time: 5.0
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
actors:
  - id: ego
    role: ego
    lane: 1
    position: 50.0
    speed: 15.0
    acceleration: [-8.0, 3.0]
  - id: npc
    role: npc
    lane: 0
    position: 65.0
    speed: 13.0
    acceleration: [-8.0, 3.0]
    behavior:
      cut_in_time: 5.0
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
