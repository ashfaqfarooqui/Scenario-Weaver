//! Shared XSD/structural validation helpers for generated `.xosc` / `.xodr`
//! artifacts (SW-07 dedup).
//!
//! Before this module existed, `assert_valid_xosc`, `assert_valid_xodr`,
//! `xsd_path` and `XSD_LOCK` were pasted near-verbatim into
//! `tests/artifact_validation_test.rs`, `tests/export_coverage_test.rs` and
//! `tests/bicycle_export_test.rs` — SW-05 could not fold them in at the time
//! because it was not allowed to touch `tests/common/`. This is that fold.
//!
//! ## Why the schema is vendored
//!
//! `openscenario-rs` bundles `Schema/OpenSCENARIO.xsd`, but that path lives
//! at the sibling path dependency `../../Workspace_OpenScenario-rs/main`,
//! which a test here must not hardcode (SW-01 is removing that path
//! dependency). `OpenSCENARIO.xsd` is a single self-contained file (no
//! `xsd:include`s), so it was copied byte-for-byte into
//! `tests/schemas/OpenSCENARIO.xsd`.
//!
//! ## Why the mutex
//!
//! libxml's `SchemaValidationContext` is documented (libxml 0.3.8
//! `tests/schema_tests.rs`) as unsafe to use concurrently from multiple
//! threads in libxml2 >= 2.12. `cargo nextest run` (what `scripts/check.sh`
//! prefers) isolates every test in its own process, so this does not matter
//! there; a plain `cargo test` run puts every test in a binary on a shared
//! thread pool, so this lock keeps that path from flaking. **Keep it** even
//! though nextest doesn't need it — a plain `cargo test` run still does.

#![allow(dead_code)] // each test crate uses a different subset

use std::path::PathBuf;
use std::sync::Mutex;

use libxml::parser::Parser as XmlParser;
use libxml::schemas::{SchemaParserContext, SchemaValidationContext};

use scenario_weaver::scenario::model::Scenario;

/// See module docs: serializes every XSD validation across this whole test
/// binary. Shared by every test file, not one static per file, so a
/// `cargo test` run of the whole crate is serialized on exactly one lock.
pub static XSD_LOCK: Mutex<()> = Mutex::new(());

/// Path to the vendored OpenSCENARIO schema.
#[must_use]
pub fn xsd_path() -> PathBuf {
    super::project_root().join("tests/schemas/OpenSCENARIO.xsd")
}

/// Validate one `.xosc` document against the bundled OpenSCENARIO XSD,
/// returning `Err` with a message instead of panicking.
///
/// Fails loudly (via `Err`, not a silent skip) if the schema itself cannot
/// be loaded — a broken `tests/schemas/OpenSCENARIO.xsd` must not present as
/// "every example passed validation".
pub fn validate_against_xsd(xml: &str, label: &str) -> Result<(), String> {
    let _guard = XSD_LOCK
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner);

    let path = xsd_path();
    let doc = XmlParser::default()
        .parse_string(xml)
        .map_err(|e| format!("{label}: not well-formed XML: {e:?}"))?;

    let mut xsd_parser =
        SchemaParserContext::from_file(path.to_str().expect("schema path is valid UTF-8"));
    let mut xsd = SchemaValidationContext::from_parser(&mut xsd_parser).map_err(|errors| {
        format!(
            "failed to load {}: {}",
            path.display(),
            error_messages(&errors).join("; ")
        )
    })?;

    xsd.validate_document(&doc)
        .map_err(|errors| format!("{label}: {}", error_messages(&errors).join("; ")))
}

fn error_messages(errors: &[libxml::error::StructuredError]) -> Vec<String> {
    errors
        .iter()
        .map(|e| {
            e.message
                .clone()
                .unwrap_or_else(|| "<no message>".to_string())
        })
        .collect()
}

/// Assert `xosc` passes XSD validation and round-trips through
/// `openscenario_rs::parse_from_str` with entity count matching
/// `scenario.actors` — replaces the substring checks that did not actually
/// verify the XML was valid OpenSCENARIO (SW-05 / `FINDINGS.md` T3).
pub fn assert_valid_xosc(xosc: &str, scenario: &Scenario, label: &str) {
    validate_against_xsd(xosc, label)
        .unwrap_or_else(|e| panic!("{label}: xosc failed XSD validation: {e}"));

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
pub fn assert_valid_xodr(xodr: &str, label: &str) {
    let doc = opendrive::core::OpenDrive::from_xml_str(xodr)
        .unwrap_or_else(|e| panic!("{label}: xodr did not parse: {e}"));
    assert!(
        !doc.road.is_empty(),
        "{label}: xodr parsed but has no <road> elements"
    );
}
