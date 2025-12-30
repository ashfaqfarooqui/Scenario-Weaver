//! Integration tests for CARLA Scenario Generator
//!
//! These tests verify the end-to-end functionality of the scenario generator.

use carla_scenario_generator::dsl;
use z3::*;

#[test]
fn test_z3_basic() {
    // Verify Z3 is working
    let cfg = Config::new();
    let ctx = Context::new(&cfg);
    let solver = Solver::new(&ctx);

    // Simple constraint: x > 0
    let x = ast::Int::new_const(&ctx, "x");
    let zero = ast::Int::from_i64(&ctx, 0);
    solver.assert(&x.gt(&zero));

    assert_eq!(solver.check(), SatResult::Sat);

    let model = solver.get_model().unwrap();
    let x_val = model.eval(&x, true).unwrap().as_i64().unwrap();
    assert!(x_val > 0);

    println!("Z3 found: x = {}", x_val);
}

#[test]
fn test_parse_example_yaml() {
    let yaml_path = std::path::Path::new("examples/cut_in_left.yaml");
    assert!(yaml_path.exists(), "Example YAML file should exist");

    let spec = dsl::parse_yaml_file(yaml_path).expect("Should parse example YAML successfully");

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
    assert_eq!(scenario.time_step, 0.5);
    assert_eq!(scenario.duration, 10.0);
    assert_eq!(scenario.actors.len(), 2);

    // Verify ego actor
    let ego = scenario.get_actor("ego").expect("Should have ego actor");
    assert_eq!(ego.role, "ego");
    assert_eq!(ego.states.len(), 21); // 0..=20 time steps

    // Verify ego initial conditions
    let ego_initial = &ego.states[0];
    assert_eq!(ego_initial.lane, 1);
    assert_eq!(ego_initial.position.x, 50.0);
    assert_eq!(ego_initial.velocity.vx, 15.0);

    // Verify NPC actor
    let npc = scenario.get_actor("npc").expect("Should have npc actor");
    assert_eq!(npc.role, "npc");
    assert_eq!(npc.states.len(), 21);

    // Verify NPC initial conditions
    let npc_initial = &npc.states[0];
    assert_eq!(npc_initial.lane, 0);
    assert!(npc_initial.position.x >= 60.0 && npc_initial.position.x <= 80.0);
    assert!(npc_initial.velocity.vx >= 12.0 && npc_initial.velocity.vx <= 14.0);

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
