//! Export coverage integration tests.
//!
//! Ensures ALL scenario types can be exported to ALL formats.

mod common;

use scenario_weaver::scenario::model::Scenario;
use scenario_weaver::{
    export_scenario_to_gif, export_scenario_to_openlabel, export_scenario_to_svg,
    export_scenario_to_xodr, export_scenario_to_xosc,
};

use common::artifacts::{assert_valid_xodr, assert_valid_xosc};

/// Generate one example, failing loudly rather than returning `Option` and
/// letting the caller "skip".
fn generate_from_file(file: &str) -> Scenario {
    common::generate_example(file)
}

// ---------------------------------------------------------------------------
// Real .xosc / .xodr validation (SW-05), replacing the substring checks this
// file used to run (e.g. `xosc.contains("OpenSCENARIO") || xosc.contains("<?xml")`,
// true of nearly any XML document — see FINDINGS.md T3). The exhaustive,
// example-corpus-driven version of these checks lives in
// `tests/artifact_validation_test.rs`; this is the minimal per-format check
// so each scenario-type/format pair below still asserts something real about
// its own output rather than importing the whole other suite.
//
// `assert_valid_xosc` / `assert_valid_xodr` moved to
// `tests/common/artifacts.rs` (SW-07 dedup) — this file, `bicycle_export_test.rs`
// and `artifact_validation_test.rs` all had near-verbatim copies.
// ---------------------------------------------------------------------------

// ===========================================================================
// cut_in_right
// ===========================================================================

#[test]
fn test_cut_in_right_export_svg() {
    let scenario = generate_from_file("cut_in_right.yaml");
    let svg = export_scenario_to_svg(&scenario).unwrap();
    assert!(svg.contains("<svg"));
    let lower = svg.to_lowercase();
    assert!(lower.contains("ego") || lower.contains("npc") || lower.contains("cut_in_right"));
}

#[test]
fn test_cut_in_right_export_xodr() {
    let scenario = generate_from_file("cut_in_right.yaml");
    let xodr = export_scenario_to_xodr(&scenario).unwrap();
    assert_valid_xodr(&xodr, "cut_in_right.yaml");
}

#[test]
fn test_cut_in_right_export_xosc() {
    let scenario = generate_from_file("cut_in_right.yaml");
    let xosc = export_scenario_to_xosc(&scenario).unwrap();
    assert_valid_xosc(&xosc, &scenario, "cut_in_right.yaml");
}

#[test]
fn test_cut_in_right_export_openlabel() {
    let scenario = generate_from_file("cut_in_right.yaml");
    let json_str = export_scenario_to_openlabel(&scenario).unwrap();
    let json: serde_json::Value = serde_json::from_str(&json_str).expect("valid JSON");
    assert!(json.get("openlabel").is_some());
    assert!(json["openlabel"].get("metadata").is_some());
}

#[test]
fn test_cut_in_right_export_gif() {
    let scenario = generate_from_file("cut_in_right.yaml");
    let gif = export_scenario_to_gif(&scenario).unwrap();
    assert!(gif.len() > 1024, "GIF should be > 1KB");
    assert_eq!(&gif[..6], b"GIF89a");
}

#[test]
fn test_cut_in_right_export_json() {
    let scenario = generate_from_file("cut_in_right.yaml");
    let json = serde_json::to_string(&scenario).unwrap();
    assert!(json.contains("cut_in_right"));
}

// ===========================================================================
// overtake_left
// ===========================================================================

#[test]
fn test_overtake_left_export_svg() {
    let scenario = generate_from_file("overtake_left.yaml");
    let svg = export_scenario_to_svg(&scenario).unwrap();
    assert!(svg.contains("<svg"));
    let lower = svg.to_lowercase();
    assert!(lower.contains("ego") || lower.contains("npc") || lower.contains("overtake"));
}

#[test]
fn test_overtake_left_export_xodr() {
    let scenario = generate_from_file("overtake_left.yaml");
    let xodr = export_scenario_to_xodr(&scenario).unwrap();
    assert_valid_xodr(&xodr, "overtake_left.yaml");
}

#[test]
fn test_overtake_left_export_xosc() {
    let scenario = generate_from_file("overtake_left.yaml");
    let xosc = export_scenario_to_xosc(&scenario).unwrap();
    assert_valid_xosc(&xosc, &scenario, "overtake_left.yaml");
}

#[test]
fn test_overtake_left_export_openlabel() {
    let scenario = generate_from_file("overtake_left.yaml");
    let json_str = export_scenario_to_openlabel(&scenario).unwrap();
    let json: serde_json::Value = serde_json::from_str(&json_str).expect("valid JSON");
    assert!(json.get("openlabel").is_some());
    assert!(json["openlabel"].get("metadata").is_some());
}

#[test]
fn test_overtake_left_export_gif() {
    let scenario = generate_from_file("overtake_left.yaml");
    let gif = export_scenario_to_gif(&scenario).unwrap();
    assert!(gif.len() > 1024, "GIF should be > 1KB");
    assert_eq!(&gif[..6], b"GIF89a");
}

#[test]
fn test_overtake_left_export_json() {
    let scenario = generate_from_file("overtake_left.yaml");
    let json = serde_json::to_string(&scenario).unwrap();
    assert!(json.contains("overtake_left"));
}

// ===========================================================================
// pedestrian_crossing
// ===========================================================================

#[test]
fn test_pedestrian_crossing_export_svg() {
    let scenario = generate_from_file("pedestrian_crossing.yaml");
    let svg = export_scenario_to_svg(&scenario).unwrap();
    assert!(svg.contains("<svg"));
    let lower = svg.to_lowercase();
    assert!(lower.contains("ego") || lower.contains("pedestrian"));
}

#[test]
fn test_pedestrian_crossing_export_xodr() {
    let scenario = generate_from_file("pedestrian_crossing.yaml");
    let xodr = export_scenario_to_xodr(&scenario).unwrap();
    assert_valid_xodr(&xodr, "pedestrian_crossing.yaml");
}

#[test]
fn test_pedestrian_crossing_export_xosc() {
    let scenario = generate_from_file("pedestrian_crossing.yaml");
    let xosc = export_scenario_to_xosc(&scenario).unwrap();
    assert_valid_xosc(&xosc, &scenario, "pedestrian_crossing.yaml");
    assert!(
        xosc.contains("pedestrian") || xosc.contains("Pedestrian"),
        "pedestrian_crossing.yaml: xosc should export the pedestrian as a Pedestrian entity"
    );
}

#[test]
fn test_pedestrian_crossing_export_openlabel() {
    let scenario = generate_from_file("pedestrian_crossing.yaml");
    let json_str = export_scenario_to_openlabel(&scenario).unwrap();
    let json: serde_json::Value = serde_json::from_str(&json_str).expect("valid JSON");
    assert!(json.get("openlabel").is_some());
    assert!(json["openlabel"].get("metadata").is_some());
}

#[test]
fn test_pedestrian_crossing_export_gif() {
    let scenario = generate_from_file("pedestrian_crossing.yaml");
    let gif = export_scenario_to_gif(&scenario).unwrap();
    assert!(gif.len() > 1024, "GIF should be > 1KB");
    assert_eq!(&gif[..6], b"GIF89a");
}

#[test]
fn test_pedestrian_crossing_export_json() {
    let scenario = generate_from_file("pedestrian_crossing.yaml");
    let json = serde_json::to_string(&scenario).unwrap();
    assert!(json.contains("pedestrian_crossing"));
}

// ===========================================================================
// head_on
// ===========================================================================

#[test]
fn test_head_on_export_svg() {
    let scenario = generate_from_file("head_on_near_miss.yaml");
    let svg = export_scenario_to_svg(&scenario).unwrap();
    assert!(svg.contains("<svg"));
}

#[test]
fn test_head_on_export_xodr() {
    let scenario = generate_from_file("head_on_near_miss.yaml");
    let xodr = export_scenario_to_xodr(&scenario).unwrap();
    assert_valid_xodr(&xodr, "head_on_near_miss.yaml");
}

#[test]
fn test_head_on_export_xosc() {
    let scenario = generate_from_file("head_on_near_miss.yaml");
    let xosc = export_scenario_to_xosc(&scenario).unwrap();
    assert_valid_xosc(&xosc, &scenario, "head_on_near_miss.yaml");
}

#[test]
fn test_head_on_export_openlabel() {
    let scenario = generate_from_file("head_on_near_miss.yaml");
    let json_str = export_scenario_to_openlabel(&scenario).unwrap();
    let json: serde_json::Value = serde_json::from_str(&json_str).expect("valid JSON");
    assert!(json.get("openlabel").is_some());
    assert!(json["openlabel"].get("metadata").is_some());
}

#[test]
fn test_head_on_export_gif() {
    let scenario = generate_from_file("head_on_near_miss.yaml");
    let gif = export_scenario_to_gif(&scenario).unwrap();
    assert!(gif.len() > 1024, "GIF should be > 1KB");
    assert_eq!(&gif[..6], b"GIF89a");
}

#[test]
fn test_head_on_export_json() {
    let scenario = generate_from_file("head_on_near_miss.yaml");
    let json = serde_json::to_string(&scenario).unwrap();
    assert!(json.contains("head_on"));
}

// ===========================================================================
// SW-17 E4/D4: .ol.json <-> .xosc join by actor name
// ===========================================================================

/// The whole point of D4 (Route A): the `.ol.json` for a scenario must be
/// joinable to its sibling `.xosc` by actor name. This reads the *real*
/// exported `.xosc` back through `openscenario_rs::parse_from_str` (not a
/// string/regex check — the SW-15/SW-18 lesson is that a comment claiming
/// two artifacts agree is not a substitute for a test that reads both) and
/// compares its `<ScenarioObject name="...">` set against the `.ol.json`
/// `objects[*].name` set, for a 3-actor scenario (`head_on_near_miss.yaml`:
/// ego, slow_npc, oncoming_npc — see `tests/head_on_test.rs`).
#[test]
fn test_openlabel_objects_join_xosc_entities_by_name() {
    let scenario = generate_from_file("head_on_near_miss.yaml");

    let xosc = export_scenario_to_xosc(&scenario).unwrap();
    let doc = openscenario_rs::parse_from_str(&xosc)
        .expect("head_on_near_miss.yaml: xosc did not round-trip through parse_from_str");
    let entities = doc
        .entities
        .as_ref()
        .expect("head_on_near_miss.yaml: round-tripped xosc has no <Entities>");
    let mut xosc_names: Vec<String> = entities
        .scenario_objects
        .iter()
        .map(|so| {
            so.name
                .as_literal()
                .map(String::as_str)
                .unwrap_or("")
                .to_string()
        })
        .collect();
    xosc_names.sort();

    let ol_json_str = export_scenario_to_openlabel(&scenario).unwrap();
    let ol_json: serde_json::Value = serde_json::from_str(&ol_json_str).unwrap();
    let objects = ol_json["openlabel"]["objects"]
        .as_object()
        .expect("openlabel.objects must be present (SW-17 E4)");
    let mut ol_names: Vec<String> = objects
        .values()
        .map(|o| o["name"].as_str().unwrap().to_string())
        .collect();
    ol_names.sort();

    assert_eq!(
        xosc_names, ol_names,
        "head_on_near_miss.yaml: .ol.json object names must match .xosc \
         ScenarioObject names exactly, or the two artifacts cannot be joined"
    );
    assert_eq!(
        xosc_names.len(),
        3,
        "expected 3 actors (ego, slow_npc, oncoming_npc)"
    );

    // And the frames actually carry per-timestep data, not just object stubs.
    let frames = ol_json["openlabel"]["frames"]
        .as_object()
        .expect("openlabel.frames must be present (SW-17 E4)");
    assert!(!frames.is_empty(), "frames must not be empty");
    let frame0 = frames.get("0").expect("frame 0 must exist");
    assert_eq!(
        frame0["objects"].as_object().unwrap().len(),
        3,
        "frame 0 must carry all 3 actors' positions"
    );
}
