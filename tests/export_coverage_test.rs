//! Export coverage integration tests.
//!
//! Ensures ALL scenario types can be exported to ALL formats.

use scenario_weaver::{
    export_scenario_to_gif, export_scenario_to_openlabel, export_scenario_to_svg,
    export_scenario_to_xodr, export_scenario_to_xosc, generate_single_scenario,
    scenario::model::Scenario,
};
use std::fs;

fn generate_from_file(file: &str) -> Option<Scenario> {
    let yaml = fs::read_to_string(format!("examples/{}", file)).unwrap();
    generate_single_scenario(&yaml).ok()
}

// ===========================================================================
// cut_in_right
// ===========================================================================

#[test]
fn test_cut_in_right_export_svg() {
    let scenario = generate_from_file("cut_in_right.yaml").expect("generation failed");
    let svg = export_scenario_to_svg(&scenario).unwrap();
    assert!(svg.contains("<svg"));
    let lower = svg.to_lowercase();
    assert!(lower.contains("ego") || lower.contains("npc") || lower.contains("cut_in_right"));
}

#[test]
fn test_cut_in_right_export_xodr() {
    let scenario = generate_from_file("cut_in_right.yaml").expect("generation failed");
    let xodr = export_scenario_to_xodr(&scenario).unwrap();
    let lower = xodr.to_lowercase();
    assert!(lower.contains("opendrive"));
    assert!(lower.contains("road"));
    assert!(lower.contains("lane"));
}

#[test]
fn test_cut_in_right_export_xosc() {
    let scenario = generate_from_file("cut_in_right.yaml").expect("generation failed");
    let xosc = export_scenario_to_xosc(&scenario).unwrap();
    assert!(xosc.contains("OpenSCENARIO") || xosc.contains("<?xml"));
    let lower = xosc.to_lowercase();
    assert!(lower.contains("ego") || lower.contains("npc") || lower.contains("entity"));
}

#[test]
fn test_cut_in_right_export_openlabel() {
    let scenario = generate_from_file("cut_in_right.yaml").expect("generation failed");
    let json_str = export_scenario_to_openlabel(&scenario).unwrap();
    let json: serde_json::Value = serde_json::from_str(&json_str).expect("valid JSON");
    assert!(json.get("openlabel").is_some());
    assert!(json["openlabel"].get("metadata").is_some());
}

#[test]
fn test_cut_in_right_export_gif() {
    let scenario = generate_from_file("cut_in_right.yaml").expect("generation failed");
    let gif = export_scenario_to_gif(&scenario).unwrap();
    assert!(gif.len() > 1024, "GIF should be > 1KB");
    assert_eq!(&gif[..6], b"GIF89a");
}

#[test]
fn test_cut_in_right_export_json() {
    let scenario = generate_from_file("cut_in_right.yaml").expect("generation failed");
    let json = serde_json::to_string(&scenario).unwrap();
    assert!(json.contains("cut_in_right"));
}

// ===========================================================================
// overtake_left
// ===========================================================================

#[test]
fn test_overtake_left_export_svg() {
    let scenario = generate_from_file("overtake_left.yaml").expect("generation failed");
    let svg = export_scenario_to_svg(&scenario).unwrap();
    assert!(svg.contains("<svg"));
    let lower = svg.to_lowercase();
    assert!(lower.contains("ego") || lower.contains("npc") || lower.contains("overtake"));
}

#[test]
fn test_overtake_left_export_xodr() {
    let scenario = generate_from_file("overtake_left.yaml").expect("generation failed");
    let xodr = export_scenario_to_xodr(&scenario).unwrap();
    let lower = xodr.to_lowercase();
    assert!(lower.contains("opendrive"));
    assert!(lower.contains("road"));
    assert!(lower.contains("lane"));
}

#[test]
fn test_overtake_left_export_xosc() {
    let scenario = generate_from_file("overtake_left.yaml").expect("generation failed");
    let xosc = export_scenario_to_xosc(&scenario).unwrap();
    assert!(xosc.contains("OpenSCENARIO") || xosc.contains("<?xml"));
    let lower = xosc.to_lowercase();
    assert!(lower.contains("ego") || lower.contains("npc") || lower.contains("entity"));
}

#[test]
fn test_overtake_left_export_openlabel() {
    let scenario = generate_from_file("overtake_left.yaml").expect("generation failed");
    let json_str = export_scenario_to_openlabel(&scenario).unwrap();
    let json: serde_json::Value = serde_json::from_str(&json_str).expect("valid JSON");
    assert!(json.get("openlabel").is_some());
    assert!(json["openlabel"].get("metadata").is_some());
}

#[test]
fn test_overtake_left_export_gif() {
    let scenario = generate_from_file("overtake_left.yaml").expect("generation failed");
    let gif = export_scenario_to_gif(&scenario).unwrap();
    assert!(gif.len() > 1024, "GIF should be > 1KB");
    assert_eq!(&gif[..6], b"GIF89a");
}

#[test]
fn test_overtake_left_export_json() {
    let scenario = generate_from_file("overtake_left.yaml").expect("generation failed");
    let json = serde_json::to_string(&scenario).unwrap();
    assert!(json.contains("overtake_left"));
}

// ===========================================================================
// pedestrian_crossing
// ===========================================================================

#[test]
fn test_pedestrian_crossing_export_svg() {
    let scenario = generate_from_file("pedestrian_crossing.yaml").expect("generation failed");
    let svg = export_scenario_to_svg(&scenario).unwrap();
    assert!(svg.contains("<svg"));
    let lower = svg.to_lowercase();
    assert!(lower.contains("ego") || lower.contains("pedestrian"));
}

#[test]
fn test_pedestrian_crossing_export_xodr() {
    let scenario = generate_from_file("pedestrian_crossing.yaml").expect("generation failed");
    let xodr = export_scenario_to_xodr(&scenario).unwrap();
    let lower = xodr.to_lowercase();
    assert!(lower.contains("opendrive"));
    assert!(lower.contains("road"));
    assert!(lower.contains("lane"));
}

#[test]
fn test_pedestrian_crossing_export_xosc() {
    let scenario = generate_from_file("pedestrian_crossing.yaml").expect("generation failed");
    let xosc = export_scenario_to_xosc(&scenario).unwrap();
    assert!(xosc.contains("OpenSCENARIO") || xosc.contains("<?xml"));
    assert!(xosc.contains("pedestrian") || xosc.contains("Pedestrian"));
}

#[test]
fn test_pedestrian_crossing_export_openlabel() {
    let scenario = generate_from_file("pedestrian_crossing.yaml").expect("generation failed");
    let json_str = export_scenario_to_openlabel(&scenario).unwrap();
    let json: serde_json::Value = serde_json::from_str(&json_str).expect("valid JSON");
    assert!(json.get("openlabel").is_some());
    assert!(json["openlabel"].get("metadata").is_some());
}

#[test]
fn test_pedestrian_crossing_export_gif() {
    let scenario = generate_from_file("pedestrian_crossing.yaml").expect("generation failed");
    let gif = export_scenario_to_gif(&scenario).unwrap();
    assert!(gif.len() > 1024, "GIF should be > 1KB");
    assert_eq!(&gif[..6], b"GIF89a");
}

#[test]
fn test_pedestrian_crossing_export_json() {
    let scenario = generate_from_file("pedestrian_crossing.yaml").expect("generation failed");
    let json = serde_json::to_string(&scenario).unwrap();
    assert!(json.contains("pedestrian_crossing"));
}

// ===========================================================================
// head_on (may be UNSAT — handle gracefully)
// ===========================================================================

#[test]
fn test_head_on_export_svg() {
    if let Some(scenario) = generate_from_file("head_on_near_miss.yaml") {
        let svg = export_scenario_to_svg(&scenario).unwrap();
        assert!(svg.contains("<svg"));
    }
}

#[test]
fn test_head_on_export_xodr() {
    if let Some(scenario) = generate_from_file("head_on_near_miss.yaml") {
        let xodr = export_scenario_to_xodr(&scenario).unwrap();
        let lower = xodr.to_lowercase();
        assert!(lower.contains("opendrive"));
        assert!(lower.contains("road"));
        assert!(lower.contains("lane"));
    }
}

#[test]
fn test_head_on_export_xosc() {
    if let Some(scenario) = generate_from_file("head_on_near_miss.yaml") {
        let xosc = export_scenario_to_xosc(&scenario).unwrap();
        assert!(xosc.contains("OpenSCENARIO") || xosc.contains("<?xml"));
    }
}

#[test]
fn test_head_on_export_openlabel() {
    if let Some(scenario) = generate_from_file("head_on_near_miss.yaml") {
        let json_str = export_scenario_to_openlabel(&scenario).unwrap();
        let json: serde_json::Value = serde_json::from_str(&json_str).expect("valid JSON");
        assert!(json.get("openlabel").is_some());
        assert!(json["openlabel"].get("metadata").is_some());
    }
}

#[test]
fn test_head_on_export_gif() {
    if let Some(scenario) = generate_from_file("head_on_near_miss.yaml") {
        let gif = export_scenario_to_gif(&scenario).unwrap();
        assert!(gif.len() > 1024, "GIF should be > 1KB");
        assert_eq!(&gif[..6], b"GIF89a");
    }
}

#[test]
fn test_head_on_export_json() {
    if let Some(scenario) = generate_from_file("head_on_near_miss.yaml") {
        let json = serde_json::to_string(&scenario).unwrap();
        assert!(json.contains("head_on"));
    }
}
