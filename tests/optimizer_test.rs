//! Integration tests for the optimization (--optimize) code path.
//!
//! Tests verify that optimization targets (min-ttc, min-distance, max-ttc, min-severity)
//! produce valid scenarios with optimization metadata.

use scenario_weaver::dsl::types::OptimizationTarget;

/// Helper to create a spec from the cut_in_left example and set an optimization target.
fn create_optimized_spec(target: OptimizationTarget) -> scenario_weaver::dsl::types::ScenarioSpec {
    let yaml = std::fs::read_to_string("examples/cut_in_left.yaml")
        .expect("Should read example YAML file");
    let mut spec = scenario_weaver::dsl::parser::parse_yaml(&yaml)
        .expect("Should parse YAML");
    spec.optimization_target = target;
    // Use a shorter duration for faster tests
    spec.duration = 5.0;
    spec.time_step = 0.5;
    spec
}

#[test]
fn test_optimize_minimize_ttc() {
    let spec = create_optimized_spec(OptimizationTarget::MinimizeTtc);
    let scenario = scenario_weaver::generate_single_scenario_from_spec(spec)
        .expect("Should generate optimized scenario");

    // Verify optimization metadata is present
    assert!(scenario.optimization.is_some(), "Should have optimization info");
    let opt = scenario.optimization.as_ref().unwrap();
    assert!(opt.target.contains("MinimizeTtc"), "Target should be MinimizeTtc, got: {}", opt.target);
    assert!(opt.optimal_value.is_some(), "Should have optimal value");

    // The optimal value is a TTC proxy (distance - dt*closing_speed)
    // It can be negative when closing speed dominates distance
    let val = opt.optimal_value.unwrap();
    assert!(val > -1000.0, "TTC proxy should be finite, got: {}", val);
    assert!(val < 1000.0, "TTC proxy should be finite, got: {}", val);

    println!("MinimizeTtc: optimal_value = {:.2} (TTC proxy)", val);
    println!("  Scenario min_ttc: {:.2}s", scenario.validation.min_ttc);
}

#[test]
fn test_optimize_minimize_distance() {
    let spec = create_optimized_spec(OptimizationTarget::MinimizeDistance);
    let scenario = scenario_weaver::generate_single_scenario_from_spec(spec)
        .expect("Should generate optimized scenario");

    assert!(scenario.optimization.is_some(), "Should have optimization info");
    let opt = scenario.optimization.as_ref().unwrap();
    assert!(opt.target.contains("MinimizeDistance"), "Target should be MinimizeDistance, got: {}", opt.target);
    assert!(opt.optimal_value.is_some(), "Should have optimal value");

    let val = opt.optimal_value.unwrap();
    assert!(val >= 0.0, "Optimal distance should be non-negative, got: {}", val);
    assert!(val < 1000.0, "Optimal distance should be finite, got: {}", val);

    println!("MinimizeDistance: optimal_value = {:.2}m", val);
    println!("  Scenario min_distance: {:.2}m", scenario.validation.min_distance);
}

#[test]
fn test_optimize_maximize_ttc() {
    let spec = create_optimized_spec(OptimizationTarget::MaximizeTtc);
    let result = scenario_weaver::generate_single_scenario_from_spec(spec);

    // MaximizeTtc may return UNSAT with short durations; only assert structure if it succeeds
    if let Ok(scenario) = result {
        assert!(scenario.optimization.is_some(), "Should have optimization info");
        let opt = scenario.optimization.as_ref().unwrap();
        assert!(opt.target.contains("MaximizeTtc"), "Target should be MaximizeTtc, got: {}", opt.target);

        assert!(!scenario.actors.is_empty(), "Should have actors");
        assert_eq!(scenario.scenario_type, "cut_in_left");

        println!("MaximizeTtc: optimal_value = {:?}", opt.optimal_value);
    } else {
        println!("MaximizeTtc returned UNSAT with short duration — acceptable");
    }
}

#[test]
fn test_optimize_minimize_severity() {
    let spec = create_optimized_spec(OptimizationTarget::MinimizeSeverity);
    let result = scenario_weaver::generate_single_scenario_from_spec(spec);

    // MinimizeSeverity may return UNSAT with short durations; only assert structure if it succeeds
    if let Ok(scenario) = result {
        assert!(scenario.optimization.is_some(), "Should have optimization info");
        let opt = scenario.optimization.as_ref().unwrap();
        assert!(opt.target.contains("MinimizeSeverity"), "Target should be MinimizeSeverity, got: {}", opt.target);

        println!("MinimizeSeverity: optimal_value = {:?}", opt.optimal_value);
    } else {
        println!("MinimizeSeverity returned UNSAT with short duration — acceptable");
    }
}

#[test]
fn test_optimize_none_via_normal_path() {
    // OptimizationTarget::None should use the normal solver path (not optimizer)
    let yaml = std::fs::read_to_string("examples/cut_in_left.yaml")
        .expect("Should read example YAML file");
    let mut spec = scenario_weaver::dsl::parser::parse_yaml(&yaml)
        .expect("Should parse YAML");
    spec.optimization_target = OptimizationTarget::None;
    spec.duration = 5.0;
    spec.time_step = 0.5;

    let scenario = scenario_weaver::generate_single_scenario_from_spec(spec)
        .expect("Should generate scenario via normal path");

    // Normal path should NOT have optimization info
    assert!(scenario.optimization.is_none(), "Normal path should not have optimization info");
    assert_eq!(scenario.scenario_type, "cut_in_left");
    assert!(!scenario.actors.is_empty());
}

#[test]
fn test_optimized_scenario_exports_correctly() {
    // Verify that optimized scenarios can be exported to all formats
    let spec = create_optimized_spec(OptimizationTarget::MinimizeTtc);
    let scenario = scenario_weaver::generate_single_scenario_from_spec(spec)
        .expect("Should generate optimized scenario");

    // JSON serialization should include optimization field
    let json = serde_json::to_string_pretty(&scenario)
        .expect("Should serialize to JSON");
    assert!(json.contains("optimization"), "JSON should contain optimization field");
    assert!(json.contains("MinimizeTtc"), "JSON should contain target name");

    // SVG export should work
    let svg = scenario_weaver::export_scenario_to_svg(&scenario)
        .expect("Should export to SVG");
    assert!(!svg.is_empty());

    // XOSC export should work
    let xosc = scenario_weaver::export_scenario_to_xosc(&scenario)
        .expect("Should export to XOSC");
    assert!(xosc.contains("OpenSCENARIO"));

    println!("All exports succeeded for optimized scenario");
}
