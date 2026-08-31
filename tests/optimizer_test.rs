//! Integration tests for the optimization (--optimize) code path.
//!
//! Tests verify that optimization targets (min-ttc, min-distance, max-ttc, min-severity)
//! produce valid scenarios with optimization metadata.

mod common;

use scenario_weaver::dsl::types::OptimizationTarget;

/// Helper to create a spec from the cut_in_left example and set an optimization target.
fn create_optimized_spec(target: OptimizationTarget) -> scenario_weaver::dsl::types::ScenarioSpec {
    let mut spec = common::parse_example("cut_in_left.yaml");
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
    assert!(
        scenario.optimization.is_some(),
        "Should have optimization info"
    );
    let opt = scenario.optimization.as_ref().unwrap();
    assert!(
        opt.target.contains("MinimizeTtc"),
        "Target should be MinimizeTtc, got: {}",
        opt.target
    );
    assert!(opt.optimal_value.is_some(), "Should have optimal value");

    // The optimal value is a TTC proxy (distance - dt*closing_speed)
    // It can be negative when closing speed dominates distance
    let val = opt.optimal_value.unwrap();
    assert!(val > -1000.0, "TTC proxy should be finite, got: {}", val);
    assert!(val < 1000.0, "TTC proxy should be finite, got: {}", val);

    println!("MinimizeTtc: optimal_value = {:.2} (TTC proxy)", val);
    println!("  Scenario min_ttc: {:?}", scenario.validation.min_ttc);
}

#[test]
fn test_optimize_minimize_distance() {
    let spec = create_optimized_spec(OptimizationTarget::MinimizeDistance);
    let scenario = scenario_weaver::generate_single_scenario_from_spec(spec)
        .expect("Should generate optimized scenario");

    assert!(
        scenario.optimization.is_some(),
        "Should have optimization info"
    );
    let opt = scenario.optimization.as_ref().unwrap();
    assert!(
        opt.target.contains("MinimizeDistance"),
        "Target should be MinimizeDistance, got: {}",
        opt.target
    );
    assert!(opt.optimal_value.is_some(), "Should have optimal value");

    let val = opt.optimal_value.unwrap();
    assert!(
        val >= 0.0,
        "Optimal distance should be non-negative, got: {}",
        val
    );
    assert!(
        val < 1000.0,
        "Optimal distance should be finite, got: {}",
        val
    );

    println!("MinimizeDistance: optimal_value = {:.2}m", val);
    println!(
        "  Scenario min_distance: {:?}",
        scenario.validation.min_distance
    );
}

#[test]
fn test_optimize_maximize_ttc() {
    let spec = create_optimized_spec(OptimizationTarget::MaximizeTtc);
    let scenario = common::generate_spec_or_fail(spec);

    let opt = scenario
        .optimization
        .as_ref()
        .expect("Should have optimization info");
    assert!(
        opt.target.contains("MaximizeTtc"),
        "Target should be MaximizeTtc, got: {}",
        opt.target
    );
    let val = opt
        .optimal_value
        .expect("MaximizeTtc should report an optimal value");
    assert!(val.is_finite(), "optimal value should be finite, got {val}");

    assert!(!scenario.actors.is_empty(), "Should have actors");
    assert_eq!(scenario.scenario_type, "cut_in_left");
}

#[test]
fn test_optimize_minimize_severity() {
    let spec = create_optimized_spec(OptimizationTarget::MinimizeSeverity);
    let scenario = common::generate_spec_or_fail(spec);

    let opt = scenario
        .optimization
        .as_ref()
        .expect("Should have optimization info");
    assert!(
        opt.target.contains("MinimizeSeverity"),
        "Target should be MinimizeSeverity, got: {}",
        opt.target
    );
    let val = opt
        .optimal_value
        .expect("MinimizeSeverity should report an optimal value");
    assert!(val.is_finite(), "optimal value should be finite, got {val}");
}

#[test]
fn test_optimize_none_via_normal_path() {
    // OptimizationTarget::None should use the normal solver path (not optimizer)
    let mut spec = common::parse_example("cut_in_left.yaml");
    spec.optimization_target = OptimizationTarget::None;
    spec.duration = 5.0;
    spec.time_step = 0.5;

    let scenario = scenario_weaver::generate_single_scenario_from_spec(spec)
        .expect("Should generate scenario via normal path");

    // Normal path should NOT have optimization info
    assert!(
        scenario.optimization.is_none(),
        "Normal path should not have optimization info"
    );
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
    let json = serde_json::to_string_pretty(&scenario).expect("Should serialize to JSON");
    assert!(
        json.contains("optimization"),
        "JSON should contain optimization field"
    );
    assert!(
        json.contains("MinimizeTtc"),
        "JSON should contain target name"
    );

    // SVG export should work
    let svg = scenario_weaver::export_scenario_to_svg(&scenario).expect("Should export to SVG");
    assert!(!svg.is_empty());

    // XOSC export should work
    let xosc = scenario_weaver::export_scenario_to_xosc(&scenario).expect("Should export to XOSC");
    assert!(xosc.contains("OpenSCENARIO"));

    println!("All exports succeeded for optimized scenario");
}

#[test]
fn test_objectives_produce_distinct_results() {
    use std::collections::HashSet;

    let targets = [
        OptimizationTarget::MinimizeDistance,
        OptimizationTarget::MinimizeTtc,
        OptimizationTarget::MinimizeSeverity,
    ];

    let mut values: Vec<f64> = Vec::new();
    for &target in &targets {
        let spec = create_optimized_spec(target);
        let scenario = common::generate_spec_or_fail(spec);
        let opt = scenario
            .optimization
            .as_ref()
            .unwrap_or_else(|| panic!("{target:?} should record optimization info"));
        values.push(
            opt.optimal_value
                .unwrap_or_else(|| panic!("{target:?} should report an optimal value")),
        );
    }

    assert_eq!(
        values.len(),
        targets.len(),
        "every objective must produce a result"
    );

    let unique: HashSet<String> = values.iter().map(|v| format!("{v:.4}")).collect();
    assert!(
        unique.len() >= 2,
        "Objectives should produce distinct optimal values, got: {values:?}"
    );
}

#[test]
fn test_optimizer_pedestrian_crossing() {
    let mut spec = common::parse_example("pedestrian_crossing.yaml");
    spec.optimization_target = OptimizationTarget::MinimizeDistance;
    spec.duration = 5.0;
    spec.time_step = 0.5;

    let scenario = common::generate_spec_or_fail(spec);

    let opt = scenario
        .optimization
        .as_ref()
        .expect("Should have optimization info");
    let val = opt
        .optimal_value
        .expect("MinimizeDistance should report an optimal value");
    assert!(
        val >= 0.0 && val.is_finite(),
        "minimised distance must be a finite non-negative length, got {val}"
    );

    let ped = scenario
        .actors
        .iter()
        .find(|a| a.id == "pedestrian")
        .expect("Should have pedestrian actor");
    for state in &ped.states {
        assert_eq!(
            state.lane(),
            0,
            "Pedestrian lane should be fixed to 0 by scenario constraints"
        );
    }
}

/// A single-step horizon leaves no room for `cut_in_left`'s lane change
/// (`start_time: [2.5, 7.5]`), so no model can exist. Committed as an
/// expectation rather than an "acceptable either way" branch.
#[test]
fn test_optimizer_minimal_horizon_is_infeasible() {
    let mut spec = common::parse_example("cut_in_left.yaml");
    spec.optimization_target = OptimizationTarget::MinimizeDistance;
    spec.duration = 0.5;
    spec.time_step = 0.5;

    common::assert_infeasible(
        spec,
        "a 0.5 s horizon cannot contain the lane change the scenario requires",
    );
}
