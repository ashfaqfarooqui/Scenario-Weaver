//! Integration tests for CARLA Scenario Generator
//!
//! These tests verify the end-to-end functionality of the scenario generator.

use carla_scenario_generator::dsl;
use z3::*;

#[test]
fn test_z3_basic() {
    // Verify Z3 is working
    let cfg = Config::new();
    z3::with_z3_config(&cfg, || {
        let solver = Solver::new();

        // Simple constraint: x > 0
        let x = ast::Int::new_const("x");
        let zero = ast::Int::from_i64(0);
        solver.assert(&x.gt(&zero));

        assert_eq!(solver.check(), SatResult::Sat);

        let model = solver.get_model().unwrap();
        let x_val = model.eval(&x, true).unwrap().as_i64().unwrap();
        assert!(x_val > 0);

        println!("Z3 found: x = {}", x_val);
    });
}

#[test]
fn test_parse_example_yaml() {
    let yaml_path = std::path::Path::new("examples/cut_in_left.yaml");
    assert!(yaml_path.exists(), "Example YAML file should exist");

    let spec = dsl::parse_yaml_file(yaml_path).expect("Should parse example YAML successfully");

    // Verify basic properties
    assert_eq!(spec.scenario_type, dsl::ScenarioType::CutInLeft);
    assert_eq!(spec.time_step, 0.1);
    assert_eq!(spec.duration, 10.0);
    assert_eq!(spec.num_time_steps(), 100);

    // Verify ego (now supports ranges)
    let ego = spec.ego().expect("Should have ego actor");
    assert_eq!(ego.lane, 1);
    assert_eq!(ego.position.min(), 0.0);
    assert_eq!(ego.position.max(), 55.0);
    assert_eq!(ego.speed.min(), 14.0);
    assert_eq!(ego.speed.max(), 16.0);

    // Verify npc
    let npcs = spec.npcs();
    assert_eq!(npcs.len(), 1);
    let npc = npcs[0];
    assert_eq!(npc.lane, 0);
    assert_eq!(npc.position.min(), 60.0);
    assert_eq!(npc.position.max(), 80.0);
    assert!(!npc.position.is_fixed());
}

#[test]
fn test_generate_single_scenario_integration() {
    // Read example YAML
    let yaml_content = std::fs::read_to_string("examples/cut_in_left.yaml")
        .expect("Should read example YAML file");

    // Generate scenario
    let scenario = carla_scenario_generator::generate_single_scenario(&yaml_content)
        .expect("Should generate scenario successfully");

    // Verify basic structure
    assert_eq!(scenario.scenario_type, "cut_in_left");
    assert_eq!(scenario.time_step, 0.1);
    assert_eq!(scenario.duration, 10.0);
    assert_eq!(scenario.actors.len(), 2);

    // Verify ego actor
    let ego = scenario.get_actor("ego").expect("Should have ego actor");
    assert_eq!(ego.role, "ego");
    assert_eq!(ego.states.len(), 101); // 0..=100 time steps (10s / 0.1s)

    // Verify ego initial conditions (should be within ranges)
    let ego_initial = &ego.states[0];
    assert_eq!(ego_initial.lane, 1);
    assert!(
        ego_initial.position.x >= 0.0 && ego_initial.position.x <= 55.0,
        "Ego position {} should be in range [0.0, 55.0]",
        ego_initial.position.x
    );
    assert!(
        ego_initial.velocity.vx >= 14.0 && ego_initial.velocity.vx <= 16.0,
        "Ego speed {} should be in range [14.0, 16.0]",
        ego_initial.velocity.vx
    );

    // Verify NPC actor
    let npc = scenario.get_actor("npc").expect("Should have npc actor");
    assert_eq!(npc.role, "npc");
    assert_eq!(npc.states.len(), 101);

    // Verify NPC initial conditions
    let npc_initial = &npc.states[0];
    assert_eq!(npc_initial.lane, 0);
    assert!(npc_initial.position.x >= 60.0 && npc_initial.position.x <= 80.0);
    assert!(npc_initial.velocity.vx >= 16.0 && npc_initial.velocity.vx <= 20.0);

    // Verify NPC is initially ahead of ego
    assert!(npc_initial.position.x > ego_initial.position.x);

    // Verify NPC eventually changes to lane 1
    let mut found_lane_change = false;
    for state in &npc.states {
        if state.lane == 1 {
            found_lane_change = true;
            break;
        }
    }
    assert!(found_lane_change, "NPC should eventually change to lane 1");

    // Verify validation metrics
    assert!(scenario.validation.min_ttc >= 3.0 || scenario.validation.min_ttc > 100.0);
    assert!(scenario.validation.min_distance >= 5.0 || scenario.validation.min_distance > 100.0);

    println!("Generated scenario:");
    println!("  Scenario ID: {}", scenario.scenario_id);
    println!("  Min TTC: {:.2}s", scenario.validation.min_ttc);
    println!("  Min Distance: {:.2}m", scenario.validation.min_distance);
    println!(
        "  All constraints satisfied: {}",
        scenario.validation.all_constraints_satisfied
    );
}

#[test]
fn test_generate_multiple_scenarios_integration() {
    // Read example YAML
    let yaml_content = std::fs::read_to_string("examples/cut_in_left.yaml")
        .expect("Should read example YAML file");

    // Generate 3 scenarios
    let scenarios = carla_scenario_generator::generate_multiple_scenarios(&yaml_content, 3)
        .expect("Should generate multiple scenarios successfully");

    // Should have at least 1 scenario (might be less than 3 if solution space is limited)
    assert!(!scenarios.is_empty());
    println!("Generated {} scenarios", scenarios.len());

    // Verify each scenario
    for (i, scenario) in scenarios.iter().enumerate() {
        println!("\n--- Scenario {} ---", i);

        // Basic structure
        assert_eq!(scenario.scenario_type, "cut_in_left");
        assert_eq!(scenario.actors.len(), 2);

        // Get actors
        let ego = scenario.get_actor("ego").unwrap();
        let npc = scenario.get_actor("npc").unwrap();

        // Initial conditions
        let ego_initial = &ego.states[0];
        let npc_initial = &npc.states[0];

        println!(
            "  NPC initial: px={:.2}, vx={:.2}, lane={}",
            npc_initial.position.x, npc_initial.velocity.vx, npc_initial.lane
        );

        // Verify NPC eventually changes lanes
        let mut found_lane_change = false;
        for state in &npc.states {
            if state.lane == 1 {
                found_lane_change = true;
                break;
            }
        }
        assert!(found_lane_change, "NPC should change to lane 1");

        // Verify NPC is ahead initially
        assert!(npc_initial.position.x > ego_initial.position.x);
    }

    // Verify scenarios are different (if we have at least 2)
    if scenarios.len() >= 2 {
        let npc0 = scenarios[0].get_actor("npc").unwrap();
        let npc1 = scenarios[1].get_actor("npc").unwrap();

        let px0 = npc0.states[0].position.x;
        let vx0 = npc0.states[0].velocity.vx;
        let px1 = npc1.states[0].position.x;
        let vx1 = npc1.states[0].velocity.vx;

        let different = (px0 - px1).abs() > 0.01 || (vx0 - vx1).abs() > 0.01;
        assert!(
            different,
            "Scenarios should have different initial conditions"
        );

        println!("\nVerified scenarios are different:");
        println!("  Scenario 0: px={:.2}, vx={:.2}", px0, vx0);
        println!("  Scenario 1: px={:.2}, vx={:.2}", px1, vx1);
    }
}

#[test]
fn test_scenario_json_serialization() {
    // Generate a scenario
    let yaml_content = std::fs::read_to_string("examples/cut_in_left.yaml")
        .expect("Should read example YAML file");

    let scenario = carla_scenario_generator::generate_single_scenario(&yaml_content)
        .expect("Should generate scenario successfully");

    // Serialize to JSON
    let json = serde_json::to_string_pretty(&scenario).expect("Should serialize to JSON");

    // Verify JSON is valid
    assert!(!json.is_empty());
    assert!(json.contains("scenario_id"));
    assert!(json.contains("scenario_type"));
    assert!(json.contains("actors"));
    assert!(json.contains("validation"));

    // Verify we can deserialize it back
    let deserialized: carla_scenario_generator::scenario::model::Scenario =
        serde_json::from_str(&json).expect("Should deserialize from JSON");

    assert_eq!(deserialized.scenario_type, scenario.scenario_type);
    assert_eq!(deserialized.actors.len(), scenario.actors.len());

    println!("JSON serialization test passed");
}

#[test]
fn test_xosc_export() {
    // Generate a scenario
    let yaml_content = std::fs::read_to_string("examples/cut_in_left.yaml")
        .expect("Should read example YAML file");

    let scenario = carla_scenario_generator::generate_single_scenario(&yaml_content)
        .expect("Should generate scenario successfully");

    // Export to XOSC
    let xosc_xml = carla_scenario_generator::export_scenario_to_xosc(&scenario)
        .expect("Should export to XOSC format");

    // Validate XML structure
    assert!(!xosc_xml.is_empty(), "XOSC XML should not be empty");
    assert!(
        xosc_xml.contains("<?xml"),
        "XOSC should contain XML declaration"
    );
    assert!(
        xosc_xml.contains("OpenSCENARIO"),
        "XOSC should contain OpenSCENARIO root element"
    );
    assert!(
        xosc_xml.contains("CARLA Scenario Generator"),
        "XOSC should contain author info"
    );

    // Verify scenario-specific content in description
    assert!(
        xosc_xml.contains("cut_in_left"),
        "XOSC should mention scenario type"
    );
    assert!(
        xosc_xml.contains(&scenario.scenario_id),
        "XOSC should contain scenario ID"
    );
    assert!(xosc_xml.contains("ego"), "XOSC should mention ego actor");
    assert!(xosc_xml.contains("npc"), "XOSC should mention npc actor");

    // Verify structure contains FileHeader
    assert!(
        xosc_xml.contains("FileHeader"),
        "XOSC should have FileHeader element"
    );

    println!("XOSC export test passed");
    println!("Generated XML length: {} bytes", xosc_xml.len());
}

#[test]
fn test_xosc_export_multiple() {
    // Generate multiple scenarios
    let yaml_content = std::fs::read_to_string("examples/cut_in_left.yaml")
        .expect("Should read example YAML file");

    let scenarios = carla_scenario_generator::generate_multiple_scenarios(&yaml_content, 3)
        .expect("Should generate multiple scenarios successfully");

    assert!(
        !scenarios.is_empty(),
        "Should generate at least one scenario"
    );

    // Export each scenario to XOSC
    for (i, scenario) in scenarios.iter().enumerate() {
        let xosc_xml = carla_scenario_generator::export_scenario_to_xosc(scenario)
            .expect("Should export scenario to XOSC");

        // Validate each XOSC output
        assert!(
            xosc_xml.contains("OpenSCENARIO"),
            "Scenario {} XOSC should contain OpenSCENARIO element",
            i
        );
        assert!(
            xosc_xml.contains(&scenario.scenario_id),
            "Scenario {} XOSC should contain its scenario ID",
            i
        );

        println!("Scenario {} XOSC export: {} bytes", i, xosc_xml.len());
    }

    // Verify XOSC outputs are different (if we have multiple scenarios)
    if scenarios.len() >= 2 {
        let xosc0 = carla_scenario_generator::export_scenario_to_xosc(&scenarios[0])
            .expect("Should export scenario 0");
        let xosc1 = carla_scenario_generator::export_scenario_to_xosc(&scenarios[1])
            .expect("Should export scenario 1");

        assert_ne!(
            xosc0, xosc1,
            "Different scenarios should produce different XOSC outputs"
        );
        println!("Verified XOSC outputs are unique");
    }

    println!("Multiple XOSC export test passed");
}

#[test]
fn test_gif_export_integration() {
    println!("\n=== Testing GIF Export ===");

    // Read example YAML
    let yaml_content = std::fs::read_to_string("examples/cut_in_left.yaml")
        .expect("Should read example YAML file");

    // Generate scenario
    let scenario = carla_scenario_generator::generate_single_scenario(&yaml_content)
        .expect("Should generate scenario successfully");

    println!("Generated scenario: {}", scenario.scenario_id);
    println!("  Type: {}", scenario.scenario_type);
    println!("  Duration: {}s", scenario.duration);
    println!("  Time steps: {}", scenario.actors[0].states.len());

    // Export to GIF
    let gif_bytes = carla_scenario_generator::export_scenario_to_gif(&scenario)
        .expect("Should export scenario to GIF");

    println!("Generated GIF with {} bytes", gif_bytes.len());

    // Verify GIF format
    assert_eq!(&gif_bytes[0..3], b"GIF", "Should have GIF magic bytes");
    assert_eq!(
        &gif_bytes[3..6],
        b"89a",
        "Should have GIF89a version identifier"
    );
    assert!(gif_bytes.len() > 1024, "GIF should be at least 1KB");

    // Optional: Write to temp file for manual inspection
    let temp_path = std::env::temp_dir().join("test_scenario.gif");
    std::fs::write(&temp_path, &gif_bytes).expect("Should write temp GIF file");
    println!("Test GIF written to: {:?}", temp_path);

    println!("GIF export test passed");
}

#[test]
fn test_gif_export_with_violations() {
    println!("\n=== Testing GIF Export with Violations ===");

    // Create adversarial YAML content (violate safety constraints)
    let yaml_content = r#"
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
    acceleration: [-8.0, 3.0]
  - id: npc
    role: npc
    lane: 0
    position: 70.0
    speed: 15.0
    acceleration: [-8.0, 3.0]
    behavior:
      cut_in_time: [2.0, 3.0]

min_ttc: 3.0
min_distance: 5.0

constraint_modes:
  min_ttc: violate
  min_distance: enforce

num_scenarios: 1
"#;

    // Generate adversarial scenario
    let scenario = carla_scenario_generator::generate_single_scenario(yaml_content)
        .expect("Should generate adversarial scenario");

    println!("Generated adversarial scenario: {}", scenario.scenario_id);
    println!(
        "  All constraints satisfied: {}",
        scenario.validation.all_constraints_satisfied
    );
    println!(
        "  Violations: {}",
        scenario.validation.safety_violations.len()
    );

    if !scenario.validation.safety_violations.is_empty() {
        println!("  Safety violations:");
        for violation in &scenario.validation.safety_violations {
            println!("    - {}", violation);
        }
    }

    // Export to GIF (should handle violations gracefully)
    let gif_bytes = carla_scenario_generator::export_scenario_to_gif(&scenario)
        .expect("Should export adversarial scenario to GIF");

    println!("Generated adversarial GIF with {} bytes", gif_bytes.len());

    // Verify GIF format
    assert_eq!(&gif_bytes[0..3], b"GIF");
    assert!(gif_bytes.len() > 1024);

    // Optional: Write to temp file
    let temp_path = std::env::temp_dir().join("test_adversarial_scenario.gif");
    std::fs::write(&temp_path, &gif_bytes).expect("Should write temp GIF file");
    println!("Adversarial GIF written to: {:?}", temp_path);

    println!("Adversarial GIF export test passed");
}
