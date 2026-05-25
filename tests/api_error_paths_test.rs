//! Integration tests for ScenarioWeaver public API error paths

use scenario_weaver::*;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Arc;

const VALID_YAML: &str = r#"
scenario_type: cut_in_left
time_step: 0.5
duration: 5.0

road:
  num_lanes: 2
  lane_width: 3.5
  lane_directions: [1, 1]

actors:
  - id: ego
    role: ego
    lane: 1
    position: 50.0
    speed: 15.0
    direction: 1
    acceleration: [-8.0, 3.0]
  - id: npc
    role: npc
    lane: 0
    direction: 1
    position: 70.0
    speed: 15.0
    acceleration: [-8.0, 3.0]
    lane_changes:
      - direction: right
        start_time: [2.0, 3.0]
        duration: [3.0, 4.0]

min_ttc: 3.0
min_distance: 5.0
num_scenarios: 1
"#;

#[test]
fn test_generate_single_scenario_invalid_yaml() {
    let result = generate_single_scenario("{{{{not yaml");
    assert!(result.is_err(), "Invalid YAML should return Err");
}

#[test]
fn test_generate_single_scenario_missing_actors() {
    let yaml = r#"
scenario_type: cut_in_left
time_step: 0.5
duration: 5.0
road:
  num_lanes: 2
  lane_width: 3.5
  lane_directions: [1, 1]
min_ttc: 3.0
min_distance: 5.0
"#;
    let result = generate_single_scenario(yaml);
    assert!(result.is_err(), "YAML with no actors should return Err");
}

#[test]
fn test_generate_single_scenario_empty_yaml() {
    let result = generate_single_scenario("");
    assert!(result.is_err(), "Empty string should return Err");
}

#[test]
fn test_generate_multiple_scenarios_zero_count() {
    let result = generate_multiple_scenarios(
        VALID_YAML,
        0,
        None::<fn(usize, &scenario::model::Scenario) -> error::Result<()>>,
    );
    // Zero count should either return Ok with empty vec or Err
    match result {
        Ok(scenarios) => assert!(scenarios.is_empty(), "Zero count should produce empty vec"),
        Err(_) => {} // Also acceptable
    }
}

#[test]
fn test_generate_multiple_scenarios_invalid_yaml() {
    let result = generate_multiple_scenarios(
        "{{{{not yaml",
        3,
        None::<fn(usize, &scenario::model::Scenario) -> error::Result<()>>,
    );
    assert!(result.is_err(), "Invalid YAML should return Err");
}

#[test]
fn test_export_scenario_to_svg_valid() {
    let scenario = generate_single_scenario(VALID_YAML).expect("Should generate scenario");
    let svg = export_scenario_to_svg(&scenario).expect("Should export to SVG");
    assert!(svg.contains("<svg"), "SVG output should contain <svg tag");
}

#[test]
fn test_export_scenario_to_xodr_valid() {
    let scenario = generate_single_scenario(VALID_YAML).expect("Should generate scenario");
    let xodr = export_scenario_to_xodr(&scenario).expect("Should export to XODR");
    assert!(
        xodr.contains("OpenDRIVE"),
        "XODR output should contain OpenDRIVE"
    );
}

#[test]
fn test_export_scenario_to_openlabel_valid() {
    let scenario = generate_single_scenario(VALID_YAML).expect("Should generate scenario");
    let json_str = export_scenario_to_openlabel(&scenario).expect("Should export to OpenLABEL");
    let parsed: serde_json::Value =
        serde_json::from_str(&json_str).expect("Should be valid JSON");
    assert!(
        parsed.get("openlabel").is_some(),
        "JSON should have 'openlabel' key"
    );
}

#[test]
fn test_export_scenario_to_gif_with_resolution() {
    let scenario = generate_single_scenario(VALID_YAML).expect("Should generate scenario");
    let gif_bytes = export_scenario_to_gif_with_resolution(&scenario, Resolution::High)
        .expect("Should export GIF with custom resolution");
    assert_eq!(&gif_bytes[0..6], b"GIF89a", "Should have GIF89a header");
}

#[test]
fn test_export_scenario_to_xosc_with_road_file() {
    let scenario = generate_single_scenario(VALID_YAML).expect("Should generate scenario");
    let road_path = "my_road_network.xodr";
    let xosc = export_scenario_to_xosc_with_road_file(&scenario, road_path)
        .expect("Should export XOSC with road file");
    assert!(
        xosc.contains(road_path),
        "XOSC should contain the road file path"
    );
}

#[test]
fn test_generate_single_scenario_conflicting_constraints() {
    // Actor must be in lane 5 on a 2-lane road
    let yaml = r#"
scenario_type: cut_in_left
time_step: 0.5
duration: 5.0

road:
  num_lanes: 2
  lane_width: 3.5
  lane_directions: [1, 1]

actors:
  - id: ego
    role: ego
    lane: 5
    position: 50.0
    speed: 15.0
    direction: 1
    acceleration: [-8.0, 3.0]
  - id: npc
    role: npc
    lane: 0
    direction: 1
    position: 70.0
    speed: 15.0
    acceleration: [-8.0, 3.0]
    lane_changes:
      - direction: right
        start_time: [2.0, 3.0]
        duration: [3.0, 4.0]

min_ttc: 3.0
min_distance: 5.0
num_scenarios: 1
"#;
    let result = generate_single_scenario(yaml);
    assert!(
        result.is_err(),
        "Conflicting constraints should return Err (Unsatisfiable or InvalidSpec)"
    );
}

#[test]
fn test_generate_callback_invoked() {
    let count = Arc::new(AtomicUsize::new(0));
    let count_clone = count.clone();

    let callback = move |_idx: usize, _scenario: &scenario::model::Scenario| -> error::Result<()> {
        count_clone.fetch_add(1, Ordering::SeqCst);
        Ok(())
    };

    let scenarios = generate_multiple_scenarios(VALID_YAML, 2, Some(callback))
        .expect("Should generate scenarios");

    let invocations = count.load(Ordering::SeqCst);
    assert_eq!(
        invocations,
        scenarios.len(),
        "Callback should be invoked once per generated scenario"
    );
}
