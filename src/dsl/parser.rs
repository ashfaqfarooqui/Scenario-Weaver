//! YAML parser for DSL specifications

use super::types::ScenarioSpec;
use crate::error::{Result, ScenarioGenError};
use std::path::Path;

/// Parse YAML string into ScenarioSpec
pub fn parse_yaml(yaml_content: &str) -> Result<ScenarioSpec> {
    let spec: ScenarioSpec =
        serde_yml::from_str(yaml_content).map_err(ScenarioGenError::YamlParse)?;

    // Validate the parsed specification
    spec.validate().map_err(ScenarioGenError::InvalidSpec)?;

    Ok(spec)
}

/// Parse YAML file into ScenarioSpec with import preprocessing
pub fn parse_yaml_file(path: &Path) -> Result<ScenarioSpec> {
    let content = std::fs::read_to_string(path)?;

    // Preprocess imports relative to the file's directory
    let base_dir = path.parent().unwrap_or_else(|| Path::new("."));
    let preprocessed = preprocess_imports(&content, base_dir)?;

    parse_yaml(&preprocessed)
}

/// Preprocess YAML content to handle imports
///
/// Supports simple import syntax:
/// ```yaml
/// imports:
///   - roads/4_lane_bidirectional.yaml
///   - another_file.yaml
/// ```
///
/// Imported content is merged into the main YAML (flattened).
/// Currently only supports importing road specifications.
fn preprocess_imports(yaml_content: &str, base_dir: &Path) -> Result<String> {
    // Parse as generic YAML to check for imports
    let mut value: serde_yml::Value =
        serde_yml::from_str(yaml_content).map_err(ScenarioGenError::YamlParse)?;

    // Collect import paths first to avoid borrow checker issues
    let import_paths: Vec<String> =
        if let Some(imports) = value.get("imports").and_then(|v| v.as_sequence()) {
            imports
                .iter()
                .filter_map(|entry| entry.as_str())
                .map(|s| s.to_string())
                .collect()
        } else {
            Vec::new()
        };

    // Process collected imports
    for import_path in import_paths {
        let full_path = base_dir.join(&import_path);

        // Read imported file
        let imported_content = std::fs::read_to_string(&full_path).map_err(|e| {
            ScenarioGenError::InvalidSpec(format!("Failed to read import '{}': {}", import_path, e))
        })?;

        // Parse imported YAML
        let imported: serde_yml::Value = serde_yml::from_str(&imported_content).map_err(|e| {
            ScenarioGenError::InvalidSpec(format!(
                "Failed to parse import '{}': {}",
                import_path, e
            ))
        })?;

        // Merge imported content into main value
        // Currently only supports importing road specs
        if let Some(road) = imported.get("road") {
            if value.get("road").is_none() {
                value
                    .as_mapping_mut()
                    .unwrap()
                    .insert(serde_yml::Value::String("road".to_string()), road.clone());
            }
        } else if imported.get("num_lanes").is_some() {
            // Import file is a road spec (flat structure)
            if value.get("road").is_none() {
                value.as_mapping_mut().unwrap().insert(
                    serde_yml::Value::String("road".to_string()),
                    imported.clone(),
                );
            }
        }
    }

    // Remove imports field from final output if it exists
    if value.get("imports").is_some() {
        value
            .as_mapping_mut()
            .unwrap()
            .remove(serde_yml::Value::String("imports".to_string()));
    }

    // Convert back to YAML string
    serde_yml::to_string(&value).map_err(ScenarioGenError::YamlParse)
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
        assert_eq!(spec.ego().unwrap().lane, 1);
        assert_eq!(spec.npcs()[0].position.min(), 60.0);
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
        let npc = spec.npcs()[0];
        assert!(npc.position.is_fixed());
        assert_eq!(npc.position.min(), 65.0);
    }

    #[test]
    fn test_preprocess_imports_no_imports() {
        let yaml = r#"
scenario_type: cut_in_left
time_step: 0.5
duration: 10.0
"#;

        let result = preprocess_imports(yaml, Path::new("."));
        assert!(result.is_ok());

        // Should return unchanged YAML (just reformatted)
        let processed = result.unwrap();
        assert!(processed.contains("scenario_type"));
        assert!(processed.contains("cut_in_left"));
    }

    #[test]
    fn test_preprocess_imports_with_road_spec() {
        use std::io::Write;
        use tempfile::TempDir;

        // Create temporary directory with road file
        let temp_dir = TempDir::new().unwrap();
        let road_path = temp_dir.path().join("test_road.yaml");

        let mut road_file = std::fs::File::create(&road_path).unwrap();
        write!(
            road_file,
            "num_lanes: 4\nlane_width: 3.5\nlane_directions: [1, 1, -1, -1]\n"
        )
        .unwrap();

        let yaml = format!(
            r#"
imports:
  - test_road.yaml
scenario_type: cut_in_left
time_step: 0.5
duration: 10.0
actors:
  - id: ego
    role: ego
    lane: 0
    position: 50.0
    speed: 15.0
    acceleration: [-8.0, 3.0]
  - id: npc
    role: npc
    lane: 1
    position: 60.0
    speed: 13.0
    acceleration: [-8.0, 3.0]
    behavior:
      cut_in_time: 5.0
min_ttc: 3.0
min_distance: 5.0
num_scenarios: 1
"#
        );

        let result = preprocess_imports(&yaml, temp_dir.path());
        assert!(result.is_ok());

        let processed = result.unwrap();
        // Should contain road spec
        assert!(processed.contains("road"));
        assert!(processed.contains("num_lanes"));
        // Should not contain imports field
        assert!(!processed.contains("imports"));

        // Parse the processed YAML
        let spec = parse_yaml(&processed).unwrap();
        assert_eq!(spec.get_num_lanes(), 4);
        assert_eq!(spec.get_lane_width(), 3.5);
        assert_eq!(spec.get_lane_direction(0), 1);
        assert_eq!(spec.get_lane_direction(2), -1);
    }

    #[test]
    fn test_parse_yaml_file_with_imports() {
        use std::io::Write;
        use tempfile::TempDir;

        // Create temporary directory
        let temp_dir = TempDir::new().unwrap();

        // Create road spec file
        let road_path = temp_dir.path().join("bidirectional.yaml");
        let mut road_file = std::fs::File::create(&road_path).unwrap();
        write!(
            road_file,
            "num_lanes: 2\nlane_width: 3.0\nlane_directions: [1, -1]\n"
        )
        .unwrap();

        // Create main scenario file with import
        let scenario_path = temp_dir.path().join("scenario.yaml");
        let mut scenario_file = std::fs::File::create(&scenario_path).unwrap();
        write!(
            scenario_file,
            r#"imports:
  - bidirectional.yaml

scenario_type: cut_in_left
time_step: 0.5
duration: 10.0

actors:
  - id: ego
    role: ego
    lane: 0
    position: 50.0
    speed: 15.0
    acceleration: [-8.0, 3.0]

  - id: npc
    role: npc
    lane: 1
    position: 150.0
    speed: 15.0
    acceleration: [-2.0, 0.0]
    behavior:
      cut_in_time: [2.5, 7.5]

min_ttc: 3.0
min_distance: 5.0
num_scenarios: 1
"#
        )
        .unwrap();

        // Parse file with imports
        let spec = parse_yaml_file(&scenario_path).unwrap();

        // Verify imported road spec
        assert_eq!(spec.get_num_lanes(), 2);
        assert_eq!(spec.get_lane_width(), 3.0);
        assert_eq!(spec.get_lane_direction(0), 1);
        assert_eq!(spec.get_lane_direction(1), -1);

        // Verify scenario content
        assert_eq!(spec.actors.len(), 2);
        assert_eq!(spec.ego().unwrap().lane, 0);
    }
}
