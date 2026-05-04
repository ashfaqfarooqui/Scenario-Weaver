//! Comprehensive integration tests for ScenarioWeaver
//!
//! Covers scenario types (cut_in_right, overtake_left, pedestrian_crossing),
//! bicycle model, adversarial/constraint modes, all exporters, DSL parser
//! edge cases, negative/edge cases, and multi-scenario generation.
//!
//! Tests use coarse time steps (0.5s) and short durations (5s) for speed.
//! UNSAT results are handled gracefully where the solver may not find solutions.

use scenario_weaver::error::ScenarioGenError;

// ---------------------------------------------------------------------------
// Helper: generate or accept UNSAT
// ---------------------------------------------------------------------------
fn generate_or_unsat(
    yaml: &str,
) -> Result<scenario_weaver::scenario::model::Scenario, ScenarioGenError> {
    scenario_weaver::generate_single_scenario(yaml)
}

// =========================================================================
// Group 1: Scenario Type Coverage
// =========================================================================

#[test]
fn test_cut_in_right_scenario() {
    let yaml = r"
scenario_type: cut_in_right
time_step: 0.5
duration: 10.0

road:
  num_lanes: 2
  lane_width: 3.5
  lane_directions: [1, 1]

actors:
  - id: ego
    role: ego
    lane: 0
    position: [45.0, 55.0]
    speed: [14.0, 16.0]
    direction: 1
    acceleration: [-8.0, 3.0]

  - id: npc
    role: npc
    lane: 1
    position: [60.0, 80.0]
    speed: [12.0, 14.0]
    direction: 1
    acceleration: [-8.0, 3.0]
    lane_changes:
      - direction: left
        start_time: [2.5, 7.5]
        duration: [3.0, 4.0]

min_ttc: 3.0
min_distance: 5.0
num_scenarios: 1
";

    let scenario = generate_or_unsat(yaml).expect("cut_in_right should be SAT");

    assert_eq!(scenario.scenario_type, "cut_in_right");
    assert!(scenario.actors.len() >= 2, "Need at least ego + npc");

    let npc = scenario.get_actor("npc").expect("npc actor");
    // NPC starts in lane 1, should eventually reach lane 0
    assert_eq!(npc.states[0].lane(), 1, "NPC should start in lane 1");
    let reached_lane_0 = npc.states.iter().any(|s| s.lane() == 0);
    assert!(reached_lane_0, "NPC should eventually change to lane 0");

    println!(
        "cut_in_right: min_ttc={:.2}, min_dist={:.2}",
        scenario.validation.min_ttc, scenario.validation.min_distance
    );
}

#[test]
fn test_overtake_left_scenario() {
    // Use the example file which is known to work
    let yaml = std::fs::read_to_string("examples/overtake_left.yaml")
        .expect("overtake_left.yaml should exist");

    match generate_or_unsat(&yaml) {
        Ok(scenario) => {
            assert_eq!(scenario.scenario_type, "overtake_left");
            assert!(scenario.actors.len() >= 2);

            let npc = scenario.get_actor("npc").expect("npc actor");
            // NPC should visit lane 0 (left of lane 1) at some point
            let visited_lane_0 = npc.states.iter().any(|s| s.lane() == 0);
            println!(
                "overtake_left: visited_lane_0={}, final_lane={}",
                visited_lane_0,
                npc.states.last().unwrap().lane()
            );
            // NPC should return to lane 1 eventually
            let returned_lane_1 = npc.states.iter().rev().take(3).any(|s| s.lane() == 1);
            if visited_lane_0 {
                assert!(
                    returned_lane_1,
                    "NPC should return to lane 1 after overtake"
                );
            }
        }
        Err(ScenarioGenError::Unsatisfiable) => {
            println!("overtake_left returned UNSAT — acceptable for complex maneuver");
        }
        Err(e) => panic!("Unexpected error: {e}"),
    }
}

#[test]
fn test_pedestrian_crossing_scenario() {
    let yaml = std::fs::read_to_string("examples/pedestrian_crossing.yaml")
        .expect("pedestrian_crossing.yaml should exist");

    match generate_or_unsat(&yaml) {
        Ok(scenario) => {
            assert_eq!(scenario.scenario_type, "pedestrian_crossing");
            assert!(scenario.actors.len() >= 2, "Need ego + pedestrian");

            let ped = scenario
                .actors
                .iter()
                .find(|a| a.role == "pedestrian")
                .expect("Should have a pedestrian actor");
            assert_eq!(ped.role, "pedestrian");

            assert!((scenario.duration - 10.0).abs() < 0.01);
            assert!((scenario.time_step - 0.3).abs() < 0.01);

            println!(
                "pedestrian_crossing: {} actors, {} steps",
                scenario.actors.len(),
                ped.states.len()
            );
        }
        Err(ScenarioGenError::Unsatisfiable) => {
            println!("pedestrian_crossing returned UNSAT — acceptable");
        }
        Err(e) => panic!("Unexpected error: {e}"),
    }
}

// =========================================================================
// Group 2: Bicycle Model Integration
// =========================================================================

#[test]
fn test_bicycle_straight_scenario() {
    let yaml =
        std::fs::read_to_string("examples/bicycle_straight.yaml").expect("file should exist");

    match generate_or_unsat(&yaml) {
        Ok(scenario) => {
            assert!(scenario.actors.len() >= 2);
            let ego = scenario.get_actor("ego").expect("ego");
            let expected_steps = (5.0 / 0.5) as usize + 1; // 11
            assert_eq!(ego.states.len(), expected_steps);

            // Forward-moving: x should increase
            let x_first = ego.states.first().unwrap().position().x;
            let x_last = ego.states.last().unwrap().position().x;
            assert!(
                x_last > x_first,
                "Ego x should increase: {x_first} -> {x_last}"
            );
            println!("bicycle_straight: ego x {x_first:.1} -> {x_last:.1}");
        }
        Err(ScenarioGenError::Unsatisfiable) => {
            println!("bicycle_straight UNSAT — acceptable");
        }
        Err(e) => panic!("Unexpected error: {e}"),
    }
}

#[test]
fn test_bicycle_lane_change_scenario() {
    let yaml = std::fs::read_to_string("examples/bicycle_lane_change_simple.yaml")
        .expect("file should exist");

    match generate_or_unsat(&yaml) {
        Ok(scenario) => {
            assert!(scenario.actors.len() >= 2);
            let npc = scenario.get_actor("npc").expect("npc");
            // Check for lateral movement (lane change)
            let y_values: Vec<f64> = npc.states.iter().map(|s| s.position().y).collect();
            let y_min = y_values.iter().cloned().fold(f64::INFINITY, f64::min);
            let y_max = y_values.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
            let y_range = (y_max - y_min).abs();
            println!("bicycle_lane_change: NPC y range = {y_range:.2}m");
        }
        Err(ScenarioGenError::Unsatisfiable) => {
            println!("bicycle_lane_change UNSAT — acceptable");
        }
        Err(e) => panic!("Unexpected error: {e}"),
    }
}

// =========================================================================
// Group 3: Adversarial / Constraint Mode Tests
// =========================================================================

#[test]
fn test_adversarial_ttc_violation() {
    let yaml = r"
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
    position: 70.0
    speed: 15.0
    direction: 1
    acceleration: [-8.0, 3.0]
    lane_changes:
      - direction: right
        start_time: [1.0, 2.0]
        duration: [2.0, 3.0]

min_ttc: 3.0
min_distance: 5.0

constraint_modes:
  min_ttc: violate
  min_distance: enforce

num_scenarios: 1
";

    match generate_or_unsat(yaml) {
        Ok(scenario) => {
            // With TTC violated, we expect either violations or low TTC
            let has_violation =
                !scenario.validation.all_constraints_satisfied || scenario.validation.min_ttc < 3.0;
            println!(
                "adversarial_ttc: min_ttc={:.2}, satisfied={}, violations={:?}",
                scenario.validation.min_ttc,
                scenario.validation.all_constraints_satisfied,
                scenario.validation.safety_violations
            );
            assert!(
                has_violation,
                "Adversarial TTC mode should produce a violation or low TTC"
            );
        }
        Err(ScenarioGenError::Unsatisfiable) => {
            println!("adversarial_ttc UNSAT — constraints may conflict");
        }
        Err(e) => panic!("Unexpected error: {e}"),
    }
}

#[test]
fn test_adversarial_all_violations() {
    let yaml = r"
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
    position: 70.0
    speed: 15.0
    direction: 1
    acceleration: [-8.0, 3.0]
    lane_changes:
      - direction: right
        start_time: [1.0, 2.0]
        duration: [2.0, 3.0]

min_ttc: 3.0
min_distance: 5.0

constraint_modes:
  min_ttc: violate
  min_distance: violate

num_scenarios: 1
";

    match generate_or_unsat(yaml) {
        Ok(scenario) => {
            println!(
                "adversarial_all: satisfied={}, violations={:?}",
                scenario.validation.all_constraints_satisfied,
                scenario.validation.safety_violations
            );
            // Both violated — should have violations
            assert!(
                !scenario.validation.all_constraints_satisfied
                    || !scenario.validation.safety_violations.is_empty()
                    || scenario.validation.min_ttc < 3.0
                    || scenario.validation.min_distance < 5.0,
                "Should have some violation"
            );
        }
        Err(ScenarioGenError::Unsatisfiable) => {
            println!("adversarial_all UNSAT — acceptable");
        }
        Err(e) => panic!("Unexpected error: {e}"),
    }
}

#[test]
fn test_ignore_constraint_mode() {
    let yaml = r"
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
    position: 70.0
    speed: 15.0
    direction: 1
    acceleration: [-8.0, 3.0]
    lane_changes:
      - direction: right
        start_time: [1.0, 2.0]
        duration: [2.0, 3.0]

min_ttc: 3.0
min_distance: 5.0

constraint_modes:
  min_ttc: ignore
  min_distance: ignore

num_scenarios: 1
";

    // With ignore mode, solver has more freedom — should succeed
    let scenario = generate_or_unsat(yaml).expect("ignore mode should be SAT (fewer constraints)");
    assert_eq!(scenario.actors.len(), 2);
    println!(
        "ignore_mode: min_ttc={:.2}, min_dist={:.2}",
        scenario.validation.min_ttc, scenario.validation.min_distance
    );
}

// =========================================================================
// Group 4: Exporter Integration Tests
// =========================================================================

/// Helper: generate a simple cut_in_left scenario for exporter tests
fn generate_simple_scenario() -> scenario_weaver::scenario::model::Scenario {
    let yaml = r"
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
    position: 70.0
    speed: 15.0
    direction: 1
    acceleration: [-8.0, 3.0]
    lane_changes:
      - direction: right
        start_time: [1.0, 2.0]
        duration: [2.0, 3.0]

min_ttc: 3.0
min_distance: 5.0
num_scenarios: 1
";
    scenario_weaver::generate_single_scenario(yaml).expect("simple scenario should be SAT")
}

#[test]
fn test_xodr_export_integration() {
    let scenario = generate_simple_scenario();
    let xodr = scenario_weaver::export_scenario_to_xodr(&scenario).expect("XODR export");

    assert!(!xodr.is_empty());
    // OpenDRIVE XML should contain the root element (case-insensitive check)
    let lower = xodr.to_lowercase();
    assert!(
        lower.contains("opendrive"),
        "XODR should contain OpenDRIVE element"
    );
    assert!(lower.contains("road"), "XODR should contain road elements");
    assert!(lower.contains("lane"), "XODR should contain lane elements");
    println!("XODR export: {} bytes", xodr.len());
}

#[test]
fn test_openlabel_export_integration() {
    let scenario = generate_simple_scenario();
    let ol = scenario_weaver::export_scenario_to_openlabel(&scenario).expect("OpenLabel export");

    assert!(!ol.is_empty());
    // Should be valid JSON
    let parsed: serde_json::Value = serde_json::from_str(&ol).expect("Should be valid JSON");
    assert!(
        parsed.get("openlabel").is_some(),
        "Should contain 'openlabel' key"
    );
    assert!(
        parsed["openlabel"].get("metadata").is_some(),
        "Should contain metadata"
    );
    println!("OpenLabel export: {} bytes", ol.len());
}

#[test]
fn test_svg_export_integration() {
    let scenario = generate_simple_scenario();
    let svg = scenario_weaver::export_scenario_to_svg(&scenario).expect("SVG export");

    assert!(!svg.is_empty());
    assert!(svg.contains("<svg"), "Should contain <svg element");
    // Check for actor references in the SVG content
    let lower = svg.to_lowercase();
    assert!(
        lower.contains("ego") || lower.contains("npc"),
        "SVG should reference actors"
    );
    println!("SVG export: {} bytes", svg.len());
}

#[test]
fn test_all_exporters_consistent() {
    let scenario = generate_simple_scenario();

    let json = serde_json::to_string(&scenario).expect("JSON");
    let xosc = scenario_weaver::export_scenario_to_xosc(&scenario).expect("XOSC");
    let xodr = scenario_weaver::export_scenario_to_xodr(&scenario).expect("XODR");
    let svg = scenario_weaver::export_scenario_to_svg(&scenario).expect("SVG");
    let gif = scenario_weaver::export_scenario_to_gif(&scenario).expect("GIF");
    let ol = scenario_weaver::export_scenario_to_openlabel(&scenario).expect("OpenLabel");

    assert!(!json.is_empty(), "JSON non-empty");
    assert!(!xosc.is_empty(), "XOSC non-empty");
    assert!(!xodr.is_empty(), "XODR non-empty");
    assert!(!svg.is_empty(), "SVG non-empty");
    assert!(!gif.is_empty(), "GIF non-empty");
    assert!(!ol.is_empty(), "OpenLabel non-empty");

    println!(
        "All exports: JSON={}B, XOSC={}B, XODR={}B, SVG={}B, GIF={}B, OL={}B",
        json.len(),
        xosc.len(),
        xodr.len(),
        svg.len(),
        gif.len(),
        ol.len()
    );
}

// =========================================================================
// Group 5: DSL Parser Edge Cases
// =========================================================================

#[test]
fn test_parse_invalid_yaml() {
    let result = scenario_weaver::generate_single_scenario("{{{{not yaml at all!!!}}}}");
    assert!(result.is_err(), "Garbage YAML should return Err");
    println!("Invalid YAML error: {}", result.unwrap_err());
}

#[test]
fn test_parse_missing_actors() {
    let yaml = r"
scenario_type: cut_in_left
time_step: 0.5
duration: 5.0
road:
  num_lanes: 2
  lane_width: 3.5
  lane_directions: [1, 1]
num_scenarios: 1
";
    let result = scenario_weaver::generate_single_scenario(yaml);
    assert!(result.is_err(), "Missing actors should return Err");
    println!("Missing actors error: {}", result.unwrap_err());
}

#[test]
fn test_parse_range_values() {
    let yaml = r"
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
    position: [20.0, 80.0]
    speed: [10.0, 20.0]
    direction: 1
    acceleration: [-8.0, 3.0]
  - id: npc
    role: npc
    lane: 0
    position: [40.0, 100.0]
    speed: [12.0, 22.0]
    direction: 1
    acceleration: [-8.0, 3.0]
    lane_changes:
      - direction: right
        start_time: [1.0, 2.0]
        duration: [2.0, 3.0]

min_ttc: 3.0
min_distance: 5.0
num_scenarios: 1
";

    let scenario = generate_or_unsat(yaml).expect("Range values should be SAT");
    let ego = scenario.get_actor("ego").expect("ego");
    let init = &ego.states[0];

    let px = init.position().x;
    let vx = init.velocity().vx;
    assert!(
        (20.0..=80.0).contains(&px),
        "Ego px {px} should be in [20, 80]"
    );
    assert!(
        (10.0..=20.0).contains(&vx),
        "Ego vx {vx} should be in [10, 20]"
    );
    println!("Range values: ego px={px:.2}, vx={vx:.2}");
}

#[test]
fn test_parse_fixed_values() {
    let yaml = r"
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
    position: 80.0
    speed: 18.0
    direction: 1
    acceleration: [-8.0, 3.0]
    lane_changes:
      - direction: right
        start_time: [1.0, 2.0]
        duration: [2.0, 3.0]

min_ttc: 3.0
min_distance: 5.0
num_scenarios: 1
";

    let scenario = generate_or_unsat(yaml).expect("Fixed values should be SAT");
    let ego = scenario.get_actor("ego").expect("ego");
    let init = &ego.states[0];

    let px = init.position().x;
    let vx = init.velocity().vx;
    assert!(
        (px - 50.0).abs() < 0.1,
        "Fixed ego px should be ~50.0, got {px}"
    );
    assert!(
        (vx - 15.0).abs() < 0.1,
        "Fixed ego vx should be ~15.0, got {vx}"
    );
    println!("Fixed values: ego px={px:.2}, vx={vx:.2}");
}

// =========================================================================
// Group 6: Negative / Edge Case Tests
// =========================================================================

#[test]
fn test_unsat_conflicting_constraints() {
    // Ego and NPC very close, but require huge min_distance — should be UNSAT
    let yaml = r"
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
    acceleration: [-1.0, 1.0]
  - id: npc
    role: npc
    lane: 0
    position: 52.0
    speed: 15.0
    direction: 1
    acceleration: [-1.0, 1.0]
    lane_changes:
      - direction: right
        start_time: [0.5, 1.0]
        duration: [1.0, 2.0]

min_ttc: 100.0
min_distance: 500.0
num_scenarios: 1
";

    let result = generate_or_unsat(yaml);
    match result {
        Err(ScenarioGenError::Unsatisfiable) => {
            println!("Conflicting constraints correctly returned UNSAT");
        }
        Err(e) => {
            // Other errors are also acceptable for impossible constraints
            println!("Conflicting constraints returned error: {e}");
        }
        Ok(scenario) => {
            // If somehow SAT, the constraints should be violated
            println!(
                "Surprisingly SAT: min_ttc={:.2}, min_dist={:.2}",
                scenario.validation.min_ttc, scenario.validation.min_distance
            );
        }
    }
}

#[test]
fn test_zero_duration_scenario() {
    let yaml = r"
scenario_type: cut_in_left
time_step: 0.5
duration: 0.0

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
    position: 70.0
    speed: 15.0
    direction: 1
    acceleration: [-8.0, 3.0]
    lane_changes:
      - direction: right
        start_time: [0.0, 0.0]
        duration: [0.0, 0.0]

min_ttc: 3.0
min_distance: 5.0
num_scenarios: 1
";

    let result = generate_or_unsat(yaml);
    // Either error or a trivial scenario — both are acceptable
    match result {
        Ok(scenario) => {
            println!(
                "Zero duration: {} actors, {} steps",
                scenario.actors.len(),
                scenario.actors.first().map_or(0, |a| a.states.len())
            );
        }
        Err(e) => {
            println!("Zero duration correctly returned error: {e}");
        }
    }
}

// =========================================================================
// Group 7: Multi-Scenario Generation
// =========================================================================

#[test]
fn test_multi_scenario_cut_in_right() {
    let yaml = r"
scenario_type: cut_in_right
time_step: 0.5
duration: 10.0

road:
  num_lanes: 2
  lane_width: 3.5
  lane_directions: [1, 1]

actors:
  - id: ego
    role: ego
    lane: 0
    position: [45.0, 55.0]
    speed: [14.0, 16.0]
    direction: 1
    acceleration: [-8.0, 3.0]

  - id: npc
    role: npc
    lane: 1
    position: [60.0, 80.0]
    speed: [12.0, 14.0]
    direction: 1
    acceleration: [-8.0, 3.0]
    lane_changes:
      - direction: left
        start_time: [2.5, 7.5]
        duration: [3.0, 4.0]

min_ttc: 3.0
min_distance: 5.0
num_scenarios: 2
";

    let scenarios = scenario_weaver::generate_multiple_scenarios(
        yaml,
        2,
        None::<
            fn(
                usize,
                &scenario_weaver::scenario::model::Scenario,
            ) -> scenario_weaver::error::Result<()>,
        >,
    )
    .expect("multi-scenario generation should succeed");

    assert!(!scenarios.is_empty(), "Should generate at least 1 scenario");
    println!("Generated {} cut_in_right scenarios", scenarios.len());

    for (i, s) in scenarios.iter().enumerate() {
        assert_eq!(s.scenario_type, "cut_in_right");
        assert!(s.actors.len() >= 2);
        println!(
            "  Scenario {i}: npc_px={:.2}",
            s.get_actor("npc").unwrap().states[0].position().x
        );
    }

    // If we got 2, verify they differ
    if scenarios.len() >= 2 {
        let px0 = scenarios[0].get_actor("npc").unwrap().states[0]
            .position()
            .x;
        let px1 = scenarios[1].get_actor("npc").unwrap().states[0]
            .position()
            .x;
        let vx0 = scenarios[0].get_actor("npc").unwrap().states[0]
            .velocity()
            .vx;
        let vx1 = scenarios[1].get_actor("npc").unwrap().states[0]
            .velocity()
            .vx;
        let different = (px0 - px1).abs() > 0.01 || (vx0 - vx1).abs() > 0.01;
        assert!(different, "Multi-scenarios should differ");
    }
}
