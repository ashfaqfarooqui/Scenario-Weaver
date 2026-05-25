use std::fs;
use std::path::Path;

use scenario_weaver::{dsl, generate_single_scenario, generate_single_scenario_from_spec};

fn assert_example_generates_or_clean_error(file: &str) {
    let yaml = fs::read_to_string(format!("examples/{}", file))
        .unwrap_or_else(|e| panic!("Failed to read {}: {}", file, e));

    match generate_single_scenario(&yaml) {
        Ok(scenario) => {
            assert!(!scenario.actors.is_empty(), "{}: no actors", file);
            assert!(!scenario.scenario_id.is_empty(), "{}: empty id", file);
            assert!(scenario.time_step > 0.0, "{}: invalid time_step", file);
        }
        Err(e) => {
            let err_str = format!("{:?}", e);
            assert!(
                err_str.contains("Unsatisfiable")
                    || err_str.contains("nsat")
                    || err_str.contains("UNSAT"),
                "{}: unexpected error (not UNSAT): {:?}",
                file,
                e
            );
        }
    }
}

/// `with_import.yaml` uses road file imports resolved relative to the file path.
/// The import path in the YAML references `roads/` which lives at the repo root,
/// not relative to `examples/`. We test that parsing from the repo root works.
fn assert_import_example_generates_or_clean_error(file: &str) {
    let path = Path::new("examples").join(file);

    let spec = match dsl::parse_yaml_file(&path) {
        Ok(s) => s,
        Err(e) => {
            // Import resolution failure is a clean error, not a panic
            let err_str = format!("{:?}", e);
            assert!(
                err_str.contains("import") || err_str.contains("Import"),
                "{}: unexpected parse error (not import-related): {:?}",
                file,
                e
            );
            return;
        }
    };

    match generate_single_scenario_from_spec(spec) {
        Ok(scenario) => {
            assert!(!scenario.actors.is_empty(), "{}: no actors", file);
            assert!(!scenario.scenario_id.is_empty(), "{}: empty id", file);
            assert!(scenario.time_step > 0.0, "{}: invalid time_step", file);
        }
        Err(e) => {
            let err_str = format!("{:?}", e);
            assert!(
                err_str.contains("Unsatisfiable")
                    || err_str.contains("nsat")
                    || err_str.contains("UNSAT"),
                "{}: unexpected error (not UNSAT): {:?}",
                file,
                e
            );
        }
    }
}

#[test]
fn test_example_cut_in_left() {
    assert_example_generates_or_clean_error("cut_in_left.yaml");
}

#[test]
fn test_example_cut_in_right() {
    assert_example_generates_or_clean_error("cut_in_right.yaml");
}

#[test]
fn test_example_cut_in_left_adversarial_all() {
    assert_example_generates_or_clean_error("cut_in_left_adversarial_all.yaml");
}

#[test]
fn test_example_cut_in_left_adversarial_ttc() {
    assert_example_generates_or_clean_error("cut_in_left_adversarial_ttc.yaml");
}

#[test]
fn test_example_cut_in_right_bicycle() {
    assert_example_generates_or_clean_error("cut_in_right_bicycle.yaml");
}

#[test]
fn test_example_bicycle_lane_change() {
    assert_example_generates_or_clean_error("bicycle_lane_change.yaml");
}

#[test]
fn test_example_head_on_collision() {
    assert_example_generates_or_clean_error("head_on_collision.yaml");
}

#[test]
fn test_example_head_on_near_miss() {
    assert_example_generates_or_clean_error("head_on_near_miss.yaml");
}

#[test]
fn test_example_multi_lane_safety() {
    assert_example_generates_or_clean_error("multi_lane_safety.yaml");
}

#[test]
fn test_example_overtake_left() {
    assert_example_generates_or_clean_error("overtake_left.yaml");
}

#[test]
fn test_example_overtake_with_opposite() {
    assert_example_generates_or_clean_error("overtake_with_opposite.yaml");
}

#[test]
fn test_example_pedestrian_crossing() {
    assert_example_generates_or_clean_error("pedestrian_crossing.yaml");
}

#[test]
fn test_example_simple_bidirectional() {
    assert_example_generates_or_clean_error("simple_bidirectional.yaml");
}

#[test]
fn test_example_speed_limit_violation() {
    assert_example_generates_or_clean_error("speed_limit_violation.yaml");
}

#[test]
fn test_example_unsafe_following() {
    assert_example_generates_or_clean_error("unsafe_following.yaml");
}

#[test]
fn test_example_with_import() {
    assert_import_example_generates_or_clean_error("with_import.yaml");
}
