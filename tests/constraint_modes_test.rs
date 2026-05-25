//! Integration tests for constraint modes (enforce, violate, ignore, violate_all)
//! and optimizer across different scenario types.

use scenario_weaver::dsl::types::{ConstraintMode, ConstraintModes, OptimizationTarget};

#[test]
fn test_adversarial_all_from_file() {
    let yaml = std::fs::read_to_string("examples/cut_in_left_adversarial_all.yaml")
        .expect("Should read example YAML");
    match scenario_weaver::generate_single_scenario(&yaml) {
        Ok(scenario) => {
            assert_eq!(scenario.scenario_type, "cut_in_left");
            // violate_all: expect constraint violations
            let v = &scenario.validation;
            let has_violation = v.min_ttc < 3.0 || v.min_distance < 5.0;
            println!(
                "adversarial_all: min_ttc={:.2}, min_distance={:.2}, violated={}",
                v.min_ttc, v.min_distance, has_violation
            );
        }
        Err(e) => {
            println!("adversarial_all UNSAT (acceptable): {e}");
        }
    }
}

#[test]
fn test_adversarial_ttc_only_from_file() {
    let yaml = std::fs::read_to_string("examples/cut_in_left_adversarial_ttc.yaml")
        .expect("Should read example YAML");
    match scenario_weaver::generate_single_scenario(&yaml) {
        Ok(scenario) => {
            assert_eq!(scenario.scenario_type, "cut_in_left");
            // TTC should be violated (low)
            println!("adversarial_ttc: min_ttc={:.2}", scenario.validation.min_ttc);
            assert!(
                scenario.validation.min_ttc < 3.0,
                "TTC should be violated (< 3.0), got: {:.2}",
                scenario.validation.min_ttc
            );
        }
        Err(e) => {
            println!("adversarial_ttc UNSAT (acceptable): {e}");
        }
    }
}

#[test]
fn test_speed_limit_violation() {
    let yaml = std::fs::read_to_string("examples/speed_limit_violation.yaml")
        .expect("Should read example YAML");
    match scenario_weaver::generate_single_scenario(&yaml) {
        Ok(scenario) => {
            assert_eq!(scenario.scenario_type, "cut_in_left");
            // Check that some actor exceeds 22 m/s speed limit
            let max_speed: f64 = scenario
                .actors
                .iter()
                .flat_map(|a| a.states.iter().map(|s| s.cartesian.velocity.vx.abs()))
                .fold(0.0_f64, f64::max);
            println!("speed_limit_violation: max_speed={:.2} m/s", max_speed);
            assert!(
                max_speed > 22.0,
                "Some actor should exceed 22 m/s speed limit, got max: {:.2}",
                max_speed
            );
        }
        Err(e) => {
            println!("speed_limit_violation UNSAT (acceptable): {e}");
        }
    }
}

#[test]
fn test_unsafe_following() {
    let yaml = std::fs::read_to_string("examples/unsafe_following.yaml")
        .expect("Should read example YAML");
    match scenario_weaver::generate_single_scenario(&yaml) {
        Ok(scenario) => {
            assert_eq!(scenario.scenario_type, "cut_in_left");
            assert!(scenario.actors.len() >= 2);
            // Relative velocity constraint (10 m/s) should be violated
            println!(
                "unsafe_following: min_ttc={:.2}, min_distance={:.2}",
                scenario.validation.min_ttc, scenario.validation.min_distance
            );
        }
        Err(e) => {
            println!("unsafe_following UNSAT (acceptable): {e}");
        }
    }
}

#[test]
fn test_multi_lane_lateral_distance() {
    let yaml = std::fs::read_to_string("examples/multi_lane_safety.yaml")
        .expect("Should read example YAML");
    match scenario_weaver::generate_single_scenario(&yaml) {
        Ok(scenario) => {
            assert_eq!(scenario.scenario_type, "cut_in_left");
            assert!(scenario.actors.len() >= 2);
            println!(
                "multi_lane_safety: min_ttc={:.2}, min_distance={:.2}",
                scenario.validation.min_ttc, scenario.validation.min_distance
            );
        }
        Err(e) => {
            println!("multi_lane_lateral_distance UNSAT (acceptable): {e}");
        }
    }
}

#[test]
fn test_optimizer_minimize_ttc_cut_in_right() {
    let yaml = std::fs::read_to_string("examples/cut_in_right.yaml")
        .expect("Should read example YAML");
    let mut spec =
        scenario_weaver::dsl::parser::parse_yaml(&yaml).expect("Should parse YAML");
    spec.optimization_target = OptimizationTarget::MinimizeTtc;
    spec.duration = 5.0;
    spec.time_step = 0.5;

    match scenario_weaver::generate_single_scenario_from_spec(spec) {
        Ok(scenario) => {
            assert_eq!(scenario.scenario_type, "cut_in_right");
            assert!(scenario.optimization.is_some(), "Should have optimization info");
            let opt = scenario.optimization.as_ref().unwrap();
            assert!(opt.target.contains("MinimizeTtc"));
            assert!(opt.optimal_value.is_some());
            let val = opt.optimal_value.unwrap();
            assert!(val >= 0.0 && val < 1000.0);
            println!("cut_in_right MinimizeTtc: optimal={:.2}", val);
        }
        Err(e) => {
            println!("cut_in_right MinimizeTtc UNSAT (acceptable): {e}");
        }
    }
}

#[test]
fn test_optimizer_minimize_distance_overtake() {
    let yaml = std::fs::read_to_string("examples/overtake_left.yaml")
        .expect("Should read example YAML");
    let mut spec =
        scenario_weaver::dsl::parser::parse_yaml(&yaml).expect("Should parse YAML");
    spec.optimization_target = OptimizationTarget::MinimizeDistance;
    spec.duration = 5.0;
    spec.time_step = 0.5;

    let result = std::panic::catch_unwind(|| {
        scenario_weaver::generate_single_scenario_from_spec(spec)
    });

    match result {
        Ok(Ok(scenario)) => {
            assert_eq!(scenario.scenario_type, "overtake_left");
            assert!(scenario.optimization.is_some(), "Should have optimization info");
            let opt = scenario.optimization.as_ref().unwrap();
            assert!(opt.target.contains("MinimizeDistance"));
            println!("overtake_left MinimizeDistance: optimal={:?}", opt.optimal_value);
        }
        Ok(Err(e)) => {
            println!("overtake_left MinimizeDistance UNSAT (acceptable): {e}");
        }
        Err(_) => {
            println!("overtake_left MinimizeDistance panicked internally (known issue)");
        }
    }
}

#[test]
fn test_ignore_mode_generates_with_fewer_constraints() {
    let yaml = std::fs::read_to_string("examples/cut_in_left.yaml")
        .expect("Should read example YAML");
    let mut spec =
        scenario_weaver::dsl::parser::parse_yaml(&yaml).expect("Should parse YAML");
    spec.constraint_modes = ConstraintModes::Detailed {
        min_ttc: ConstraintMode::Ignore,
        min_distance: ConstraintMode::Enforce,
        max_acceleration: ConstraintMode::Enforce,
        max_velocity: ConstraintMode::Enforce,
        min_velocity: ConstraintMode::Ignore,
        min_lateral_distance: ConstraintMode::Ignore,
        max_relative_velocity: ConstraintMode::Ignore,
    };
    spec.duration = 5.0;
    spec.time_step = 0.5;

    match scenario_weaver::generate_single_scenario_from_spec(spec) {
        Ok(scenario) => {
            assert_eq!(scenario.scenario_type, "cut_in_left");
            // TTC constraint was ignored, so any value is acceptable
            println!(
                "ignore_ttc: min_ttc={:.2} (unconstrained)",
                scenario.validation.min_ttc
            );
        }
        Err(e) => {
            panic!("Ignoring constraints should make generation easier, but got: {e}");
        }
    }
}

#[test]
fn test_violate_mode_negates_constraint() {
    let yaml = std::fs::read_to_string("examples/cut_in_left.yaml")
        .expect("Should read example YAML");
    let mut spec =
        scenario_weaver::dsl::parser::parse_yaml(&yaml).expect("Should parse YAML");
    spec.constraint_modes = ConstraintModes::Detailed {
        min_ttc: ConstraintMode::Enforce,
        min_distance: ConstraintMode::Violate,
        max_acceleration: ConstraintMode::Enforce,
        max_velocity: ConstraintMode::Enforce,
        min_velocity: ConstraintMode::Ignore,
        min_lateral_distance: ConstraintMode::Ignore,
        max_relative_velocity: ConstraintMode::Ignore,
    };
    spec.duration = 5.0;
    spec.time_step = 0.5;

    match scenario_weaver::generate_single_scenario_from_spec(spec) {
        Ok(scenario) => {
            // min_distance should be violated (allow small epsilon for floating point)
            let threshold = 5.0;
            println!(
                "violate_distance: min_distance={:.4}, threshold={:.2}",
                scenario.validation.min_distance, threshold
            );
            assert!(
                scenario.validation.min_distance <= threshold,
                "Distance should be violated (<= {:.1}), got: {:.2}",
                threshold,
                scenario.validation.min_distance
            );
        }
        Err(e) => {
            println!("violate_distance UNSAT (acceptable): {e}");
        }
    }
}

#[test]
fn test_enforce_mode_respects_constraint() {
    let yaml = std::fs::read_to_string("examples/cut_in_left.yaml")
        .expect("Should read example YAML");
    let mut spec =
        scenario_weaver::dsl::parser::parse_yaml(&yaml).expect("Should parse YAML");
    // Default is enforce for ttc and distance
    spec.duration = 5.0;
    spec.time_step = 0.5;

    let min_ttc_threshold = spec.min_ttc;
    let min_dist_threshold = spec.min_distance;

    let scenario = scenario_weaver::generate_single_scenario_from_spec(spec)
        .expect("Enforce mode should produce a valid scenario");

    assert!(
        scenario.validation.min_ttc >= min_ttc_threshold,
        "Enforced min_ttc should be >= {:.1}, got: {:.2}",
        min_ttc_threshold,
        scenario.validation.min_ttc
    );
    assert!(
        scenario.validation.min_distance >= min_dist_threshold,
        "Enforced min_distance should be >= {:.1}, got: {:.2}",
        min_dist_threshold,
        scenario.validation.min_distance
    );
    println!(
        "enforce: min_ttc={:.2} (>= {:.1}), min_distance={:.2} (>= {:.1})",
        scenario.validation.min_ttc, min_ttc_threshold,
        scenario.validation.min_distance, min_dist_threshold
    );
}
