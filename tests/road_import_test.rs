//! Integration tests for road import/library functionality

use std::path::{Path, PathBuf};

use scenario_weaver::dsl::parse_yaml_file;
use scenario_weaver::generate_single_scenario_from_spec;

fn project_root() -> &'static Path {
    Path::new(env!("CARGO_MANIFEST_DIR"))
}

/// Create a temp YAML file at the project root so relative imports resolve correctly.
/// The `with_import.yaml` example uses `roads/...` relative to CWD/project root.
/// Uses a unique suffix to avoid race conditions between parallel tests.
fn write_import_yaml_at_root(suffix: &str) -> PathBuf {
    let path = project_root().join(format!("_test_import_tmp_{}.yaml", suffix));
    let content = std::fs::read_to_string(project_root().join("examples/with_import.yaml"))
        .expect("should read example");
    std::fs::write(&path, &content).unwrap();
    path
}

fn cleanup_tmp(suffix: &str) {
    let _ = std::fs::remove_file(project_root().join(format!("_test_import_tmp_{}.yaml", suffix)));
}

#[test]
fn test_with_import_yaml_parses() {
    let path = write_import_yaml_at_root("parses");
    let result = parse_yaml_file(&path);
    cleanup_tmp("parses");
    let spec = result.expect("should parse with_import.yaml");

    let road = spec.road.expect("spec should have road after import");
    assert_eq!(road.num_lanes, 4);
    assert_eq!(road.lane_width, 3.5);
    assert_eq!(road.lane_directions, vec![1, 1, -1, -1]);
}

#[test]
fn test_with_import_generates_scenario() {
    let path = write_import_yaml_at_root("generates");
    let result = parse_yaml_file(&path);
    cleanup_tmp("generates");
    let spec = result.expect("should parse");

    let scenario = generate_single_scenario_from_spec(spec).expect("should generate scenario");
    assert!(scenario.duration > 0.0);
}

#[test]
fn test_road_files_are_valid_yaml() {
    let roads_dir = project_root().join("roads");

    let files = [
        ("2_lane_rural.yaml", 2, 3.0, vec![1, -1]),
        ("3_lane_highway.yaml", 3, 3.75, vec![1, 1, -1]),
        ("4_lane_bidirectional.yaml", 4, 3.5, vec![1, 1, -1, -1]),
    ];

    for (filename, expected_lanes, expected_width, expected_dirs) in &files {
        let content = std::fs::read_to_string(roads_dir.join(filename))
            .unwrap_or_else(|_| panic!("should read {}", filename));

        let road: scenario_weaver::dsl::types::RoadSpec =
            serde_yml::from_str(&content).unwrap_or_else(|_| panic!("should parse {}", filename));

        assert_eq!(road.num_lanes, *expected_lanes, "lanes mismatch in {}", filename);
        assert_eq!(road.lane_width, *expected_width, "width mismatch in {}", filename);
        assert_eq!(road.lane_directions, *expected_dirs, "directions mismatch in {}", filename);
    }
}

#[test]
fn test_import_nonexistent_road_file() {
    let yaml = r#"
imports:
  - roads/nonexistent_road.yaml

scenario_type: cut_in_left
time_step: 0.5
duration: 10.0

road:
  num_lanes: 2
  lane_width: 3.5
  lane_directions: [1, -1]

actors:
  - id: ego
    role: ego
    lane: 0
    position: 50.0
    speed: 15.0
    direction: 1
"#;

    let tmp_file = project_root().join("_test_bad_import_tmp.yaml");
    std::fs::write(&tmp_file, yaml).unwrap();

    let result = parse_yaml_file(&tmp_file);
    let _ = std::fs::remove_file(&tmp_file);
    assert!(result.is_err(), "should error on nonexistent import");
}

#[test]
fn test_road_spec_from_import_has_lanes() {
    let path = write_import_yaml_at_root("has_lanes");
    let result = parse_yaml_file(&path);
    cleanup_tmp("has_lanes");
    let spec = result.expect("should parse");

    let road = spec.road.expect("should have road");
    assert!(road.num_lanes > 0);
    assert!(road.lane_width > 0.0);
    assert!(!road.lane_directions.is_empty());
    assert_eq!(road.lane_directions.len(), road.num_lanes);
}

#[test]
fn test_imported_road_matches_file_content() {
    let road_file = project_root().join("roads/4_lane_bidirectional.yaml");
    let road_content = std::fs::read_to_string(&road_file).unwrap();
    let road_direct: scenario_weaver::dsl::types::RoadSpec =
        serde_yml::from_str(&road_content).unwrap();

    let path = write_import_yaml_at_root("matches");
    let result = parse_yaml_file(&path);
    cleanup_tmp("matches");
    let spec = result.expect("should parse");
    let road_imported = spec.road.expect("should have road");

    assert_eq!(road_imported.num_lanes, road_direct.num_lanes);
    assert_eq!(road_imported.lane_width, road_direct.lane_width);
    assert_eq!(road_imported.lane_directions, road_direct.lane_directions);
}
