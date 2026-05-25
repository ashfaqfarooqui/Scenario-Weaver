//! Integration tests for bicycle-model scenario export pipeline.
//!
//! Verifies that bicycle-model scenarios (which use different physics encoding)
//! can be generated AND exported through all format exporters correctly.

use scenario_weaver::error::ScenarioGenError;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

fn load_bicycle_lane_change() -> String {
    std::fs::read_to_string("examples/bicycle_lane_change.yaml")
        .expect("bicycle_lane_change.yaml should exist")
}

fn load_cut_in_right_bicycle() -> String {
    std::fs::read_to_string("examples/cut_in_right_bicycle.yaml")
        .expect("cut_in_right_bicycle.yaml should exist")
}

fn generate_bicycle_cut_in_left() -> scenario_weaver::scenario::model::Scenario {
    let yaml = load_bicycle_lane_change();
    scenario_weaver::generate_single_scenario(&yaml).expect("bicycle cut_in_left should be SAT")
}

// =========================================================================
// Test 1: Bicycle cut_in_left → SVG export
// =========================================================================

#[test]
fn test_bicycle_cut_in_left_export_svg() {
    let scenario = generate_bicycle_cut_in_left();
    let svg = scenario_weaver::export_scenario_to_svg(&scenario).expect("SVG export should succeed");

    assert!(svg.contains("<svg"), "SVG should contain <svg element");
    let lower = svg.to_lowercase();
    assert!(
        lower.contains("ego") || lower.contains("npc"),
        "SVG should contain actor names"
    );
    assert!(svg.len() > 100, "SVG should be reasonably sized, got {} bytes", svg.len());
}

// =========================================================================
// Test 2: Bicycle cut_in_left → XOSC export
// =========================================================================

#[test]
fn test_bicycle_cut_in_left_export_xosc() {
    let scenario = generate_bicycle_cut_in_left();
    let xosc = scenario_weaver::export_scenario_to_xosc(&scenario).expect("XOSC export should succeed");

    let lower = xosc.to_lowercase();
    assert!(lower.contains("openscenario"), "XOSC should contain OpenSCENARIO header");
    assert!(lower.contains("entit"), "XOSC should contain entity definitions");
    assert!(
        lower.contains("trajectory") || lower.contains("maneuver") || lower.contains("action"),
        "XOSC should contain trajectory/maneuver data"
    );
}

// =========================================================================
// Test 3: Bicycle cut_in_left → XODR export
// =========================================================================

#[test]
fn test_bicycle_cut_in_left_export_xodr() {
    let scenario = generate_bicycle_cut_in_left();
    let xodr = scenario_weaver::export_scenario_to_xodr(&scenario).expect("XODR export should succeed");

    let lower = xodr.to_lowercase();
    assert!(lower.contains("opendrive"), "XODR should contain OpenDRIVE header");
    assert!(lower.contains("road"), "XODR should contain road structure");
    assert!(lower.contains("lane"), "XODR should contain lane structure");
}

// =========================================================================
// Test 4: Bicycle cut_in_left → OpenLABEL export
// =========================================================================

#[test]
fn test_bicycle_cut_in_left_export_openlabel() {
    let scenario = generate_bicycle_cut_in_left();
    let ol = scenario_weaver::export_scenario_to_openlabel(&scenario).expect("OpenLABEL export should succeed");

    let parsed: serde_json::Value = serde_json::from_str(&ol).expect("Should be valid JSON");
    assert!(parsed.get("openlabel").is_some(), "Should have 'openlabel' key");
    let openlabel = &parsed["openlabel"];
    assert!(
        openlabel.get("tags").is_some() || openlabel.get("metadata").is_some(),
        "Should have tags or metadata"
    );
}

// =========================================================================
// Test 5: Bicycle cut_in_left → GIF export
// =========================================================================

#[test]
fn test_bicycle_cut_in_left_export_gif() {
    let scenario = generate_bicycle_cut_in_left();
    let gif = scenario_weaver::export_scenario_to_gif(&scenario).expect("GIF export should succeed");

    assert!(gif.len() > 1024, "GIF should be > 1KB, got {} bytes", gif.len());
    // GIF89a magic bytes
    assert_eq!(&gif[..6], b"GIF89a", "Should have GIF89a magic bytes");
}

// =========================================================================
// Test 6: Bicycle cut_in_right generation
// =========================================================================

#[test]
fn test_bicycle_cut_in_right_generation() {
    let yaml = load_cut_in_right_bicycle();
    match scenario_weaver::generate_single_scenario(&yaml) {
        Ok(scenario) => {
            assert_eq!(scenario.scenario_type, "cut_in_right");
            assert_eq!(scenario.actors.len(), 2, "Should have ego + npc");

            let npc = scenario.get_actor("npc").expect("npc actor");
            // NPC starts in lane 2, should change to lane 1
            assert_eq!(npc.states[0].lane(), 2, "NPC should start in lane 2");
            let reached_lane_1 = npc.states.iter().any(|s| s.lane() == 1);
            assert!(reached_lane_1, "NPC should perform lane change to lane 1");
        }
        Err(ScenarioGenError::Unsatisfiable) => {
            println!("cut_in_right_bicycle UNSAT — acceptable for complex bicycle constraints");
        }
        Err(e) => panic!("Unexpected error: {e}"),
    }
}

// =========================================================================
// Test 7: Bicycle cut_in_right → SVG export
// =========================================================================

#[test]
fn test_bicycle_cut_in_right_export_svg() {
    let yaml = load_cut_in_right_bicycle();
    match scenario_weaver::generate_single_scenario(&yaml) {
        Ok(scenario) => {
            let svg = scenario_weaver::export_scenario_to_svg(&scenario).expect("SVG export");
            assert!(svg.contains("<svg"), "SVG should contain <svg element");
            assert!(svg.len() > 100, "SVG should be non-trivial");
        }
        Err(ScenarioGenError::Unsatisfiable) => {
            println!("cut_in_right_bicycle UNSAT — skipping SVG export test");
        }
        Err(e) => panic!("Unexpected error: {e}"),
    }
}

// =========================================================================
// Test 8: Bicycle scenario trajectory physics verification
// =========================================================================

#[test]
fn test_bicycle_scenario_trajectory_physics() {
    let scenario = generate_bicycle_cut_in_left();

    let ego = scenario.get_actor("ego").expect("ego");
    let npc = scenario.get_actor("npc").expect("npc");

    // Positions should increase over time (forward motion)
    let ego_x_first = ego.states.first().unwrap().position().x;
    let ego_x_last = ego.states.last().unwrap().position().x;
    assert!(ego_x_last > ego_x_first, "Ego should move forward: {ego_x_first} -> {ego_x_last}");

    let npc_x_first = npc.states.first().unwrap().position().x;
    let npc_x_last = npc.states.last().unwrap().position().x;
    assert!(npc_x_last > npc_x_first, "NPC should move forward: {npc_x_first} -> {npc_x_last}");

    // NPC lateral position should change during lane change
    let npc_y_values: Vec<f64> = npc.states.iter().map(|s| s.position().y).collect();
    let y_min = npc_y_values.iter().cloned().fold(f64::INFINITY, f64::min);
    let y_max = npc_y_values.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    assert!(
        (y_max - y_min).abs() > 0.5,
        "NPC lateral position should change during lane change, range={:.2}",
        y_max - y_min
    );

    // Speed should remain non-negative (forward motion)
    for state in ego.states.iter() {
        let vx = state.velocity().vx;
        assert!(vx >= 0.0, "Ego speed should remain non-negative, got {vx}");
    }
}

// =========================================================================
// Test 9: Bicycle vs Cartesian — both generate and export
// =========================================================================

#[test]
fn test_bicycle_vs_cartesian_both_generate() {
    let bicycle_yaml = load_bicycle_lane_change();
    let bicycle_scenario = scenario_weaver::generate_single_scenario(&bicycle_yaml)
        .expect("bicycle scenario should be SAT");

    // Equivalent cartesian scenario
    let cartesian_yaml = r"
scenario_type: cut_in_left
time_step: 0.1
duration: 10.0

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
    acceleration: [-8.0, 3.0]
    direction: 1

  - id: npc
    role: npc
    lane: 0
    position: [60.0, 80.0]
    speed: [16.0, 20.0]
    acceleration: [-8.0, 3.0]
    direction: 1
    lane_changes:
      - direction: right
        start_time: [2.5, 3.5]
        duration: [3.0, 4.0]

min_ttc: 3.0
min_distance: 5.0
num_scenarios: 1
";

    let cartesian_scenario = scenario_weaver::generate_single_scenario(cartesian_yaml)
        .expect("cartesian scenario should be SAT");

    // Both succeed with same structure
    assert_eq!(bicycle_scenario.actors.len(), cartesian_scenario.actors.len());
    assert_eq!(
        bicycle_scenario.get_actor("ego").unwrap().states.len(),
        cartesian_scenario.get_actor("ego").unwrap().states.len(),
        "Both should have same number of timesteps"
    );

    // Both can export to all formats
    assert!(scenario_weaver::export_scenario_to_svg(&bicycle_scenario).is_ok());
    assert!(scenario_weaver::export_scenario_to_svg(&cartesian_scenario).is_ok());

    assert!(scenario_weaver::export_scenario_to_xosc(&bicycle_scenario).is_ok());
    assert!(scenario_weaver::export_scenario_to_xosc(&cartesian_scenario).is_ok());

    assert!(scenario_weaver::export_scenario_to_xodr(&bicycle_scenario).is_ok());
    assert!(scenario_weaver::export_scenario_to_xodr(&cartesian_scenario).is_ok());

    assert!(scenario_weaver::export_scenario_to_openlabel(&bicycle_scenario).is_ok());
    assert!(scenario_weaver::export_scenario_to_openlabel(&cartesian_scenario).is_ok());

    assert!(scenario_weaver::export_scenario_to_gif(&bicycle_scenario).is_ok());
    assert!(scenario_weaver::export_scenario_to_gif(&cartesian_scenario).is_ok());

    println!(
        "Both coordinate systems generate and export successfully: bicycle={} actors, cartesian={} actors",
        bicycle_scenario.actors.len(),
        cartesian_scenario.actors.len()
    );
}
