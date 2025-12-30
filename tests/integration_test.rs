//! Integration tests

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

use carla_scenario_generator::dsl;

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
