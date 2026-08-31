//! Export coverage integration tests.
//!
//! Ensures ALL scenario types can be exported to ALL formats.

mod common;

use std::path::PathBuf;
use std::sync::Mutex;

use libxml::parser::Parser as XmlParser;
use libxml::schemas::{SchemaParserContext, SchemaValidationContext};

use scenario_weaver::scenario::model::Scenario;
use scenario_weaver::{
    export_scenario_to_gif, export_scenario_to_openlabel, export_scenario_to_svg,
    export_scenario_to_xodr, export_scenario_to_xosc,
};

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
// Duplicated (not shared via `tests/common`, which SW-06 owns) — see the
// SW-05 report's "Notes for the next agent" for a proposal to fold this into
// `tests/common` once this issue lands.
// ---------------------------------------------------------------------------

/// See `tests/artifact_validation_test.rs` module docs for why the XSD is
/// vendored here rather than resolved from the sibling `openscenario-rs`
/// checkout.
fn xsd_path() -> PathBuf {
    common::project_root().join("tests/schemas/OpenSCENARIO.xsd")
}

/// See `tests/artifact_validation_test.rs`: libxml's schema validation
/// context is documented as unsafe across threads, so this is serialized.
static XSD_LOCK: Mutex<()> = Mutex::new(());

/// Assert `xosc` is well-formed XML, passes XSD validation against the real
/// OpenSCENARIO schema, and round-trips through `parse_from_str` with its
/// entity count matching `scenario.actors`.
fn assert_valid_xosc(xosc: &str, scenario: &Scenario, label: &str) {
    {
        let _guard = XSD_LOCK
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        let doc = XmlParser::default()
            .parse_string(xosc)
            .unwrap_or_else(|e| panic!("{label}: xosc not well-formed XML: {e:?}"));
        let mut xsd_parser =
            SchemaParserContext::from_file(xsd_path().to_str().expect("utf8 path"));
        let mut xsd = SchemaValidationContext::from_parser(&mut xsd_parser)
            .unwrap_or_else(|e| panic!("{label}: failed to load XSD: {e:?}"));
        xsd.validate_document(&doc)
            .unwrap_or_else(|errors| panic!("{label}: xosc failed XSD validation: {errors:?}"));
    }

    let parsed = openscenario_rs::parse_from_str(xosc)
        .unwrap_or_else(|e| panic!("{label}: xosc did not round-trip through parse_from_str: {e}"));
    let entities = parsed
        .entities
        .unwrap_or_else(|| panic!("{label}: round-tripped xosc has no <Entities>"));
    assert_eq!(
        entities.scenario_objects.len(),
        scenario.actors.len(),
        "{label}: entity count not preserved across the xosc round-trip"
    );
}

/// Assert `xodr` is well-formed OpenDRIVE that structurally parses back
/// through the `opendrive` crate (catches an unclosed element or a
/// malformed `<geometry>`/`<laneSection>` that a substring check cannot).
fn assert_valid_xodr(xodr: &str, label: &str) {
    let doc = opendrive::core::OpenDrive::from_xml_str(xodr)
        .unwrap_or_else(|e| panic!("{label}: xodr did not parse: {e}"));
    assert!(
        !doc.road.is_empty(),
        "{label}: xodr parsed but has no <road> elements"
    );
}

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
