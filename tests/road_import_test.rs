//! Integration tests for road import/library functionality

mod common;

use std::path::PathBuf;

use common::project_root;
use scenario_weaver::dsl::parse_yaml_file;

/// Directory used for temp copies of `with_import.yaml`, below.
///
/// Deliberately **not** `examples/`: `tests/common::example_paths()` globs every
/// `examples/*.yaml` for the corpus-wide smoke tests, so dropping scratch files in
/// there would pull them into that corpus. It also is not the project root: since
/// SW-13, `with_import.yaml` declares `imports: ../roads/4_lane_bidirectional.yaml`,
/// resolved relative to the YAML's own directory (as `examples/with_import.yaml`
/// really is), so the copy needs to sit exactly one directory below the repo root,
/// the same depth as `examples/`, for `../roads/...` to land on `roads/` at the root.
fn tmp_import_dir() -> PathBuf {
    let dir = project_root().join("_test_tmp_imports");
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

/// Create a temp copy of `with_import.yaml` at [`tmp_import_dir`] so its
/// `imports: ../roads/...` resolves exactly as it does for the real
/// `examples/with_import.yaml`.
///
/// Uses a unique suffix to avoid race conditions between parallel tests.
fn write_import_yaml_at_matching_depth(suffix: &str) -> PathBuf {
    let path = tmp_import_dir().join(format!("with_import_{}.yaml", suffix));
    std::fs::write(&path, common::load_example("with_import.yaml")).unwrap();
    path
}

fn cleanup_tmp(suffix: &str) {
    let _ = std::fs::remove_file(tmp_import_dir().join(format!("with_import_{}.yaml", suffix)));
}

#[test]
fn test_with_import_yaml_parses() {
    let path = write_import_yaml_at_matching_depth("parses");
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
    let path = write_import_yaml_at_matching_depth("generates");
    let result = parse_yaml_file(&path);
    cleanup_tmp("generates");
    let spec = result.expect("should parse");

    let scenario = common::generate_spec_or_fail(spec);
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

        assert_eq!(
            road.num_lanes, *expected_lanes,
            "lanes mismatch in {}",
            filename
        );
        assert_eq!(
            road.lane_width, *expected_width,
            "width mismatch in {}",
            filename
        );
        assert_eq!(
            road.lane_directions, *expected_dirs,
            "directions mismatch in {}",
            filename
        );
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
    let path = write_import_yaml_at_matching_depth("has_lanes");
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

    let path = write_import_yaml_at_matching_depth("matches");
    let result = parse_yaml_file(&path);
    cleanup_tmp("matches");
    let spec = result.expect("should parse");
    let road_imported = spec.road.expect("should have road");

    assert_eq!(road_imported.num_lanes, road_direct.num_lanes);
    assert_eq!(road_imported.lane_width, road_direct.lane_width);
    assert_eq!(road_imported.lane_directions, road_direct.lane_directions);
}
