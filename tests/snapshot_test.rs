//! Golden/snapshot tests of generated JSON, `.xosc` and `.xodr` (SW-07,
//! item 2).
//!
//! ## Why this exists
//!
//! SW-08 lands next and changes numeric solver output across the board.
//! Without a snapshot baseline, that change is invisible until something
//! else notices — with one, `cargo insta review` turns it into a
//! reviewable diff. This file is the pre-SW-08 baseline: commit the
//! snapshots alongside this file.
//!
//! ## Coverage
//!
//! A representative subset, not all 22 examples (kept small so the suite
//! stays fast): `cut_in_left.yaml` (cartesian, lane change),
//! `bicycle_lane_change.yaml` (bicycle model) and
//! `pedestrian_crossing.yaml` (pedestrian physics).
//!
//! ## Determinism and redaction
//!
//! Z3 output here is fully deterministic (verified byte-identical across
//! runs — see `SW-07-test-tooling.md`), so nothing in a snapshot diff is
//! ever flakiness; a diff always means an edit changed behaviour. Three
//! fields are the exception and **must** be redacted rather than snapshotted
//! raw, since `src/` may not be touched to inject a fixed clock/ID:
//!
//! - `Scenario::scenario_id` (`src/scenario/model.rs`, `Uuid::new_v4`) —
//!   appears in the JSON export directly, and twice more in the `.xosc`'s
//!   embedded description (`src/scenario/xosc_exporter.rs::build_scenario_description`).
//! - The `.xosc` `FileHeader`'s `date` field — stamped by
//!   `openscenario-rs`'s `ScenarioBuilder::with_header` via `chrono::Utc::now()`
//!   (not this crate's `src/`, but still present in every `.xosc` this
//!   crate emits).
//! - The `.xodr` `<header date="...">` field
//!   (`src/scenario/xodr_exporter.rs`, `chrono::Utc::now()`).
//!
//! Both redaction patterns below are content filters (regex, not JSON-path
//! redactions), so they apply uniformly to the JSON, `.xosc` and `.xodr`
//! snapshots without needing format-specific handling.

mod common;

use insta::assert_snapshot;

use scenario_weaver::scenario::model::Scenario;
use scenario_weaver::{export_scenario_to_xodr, export_scenario_to_xosc};

/// Build the insta settings that redact the two nondeterministic patterns
/// documented in the module docs. Applied via `.bind(...)` around every
/// assertion in this file so no raw UUID or timestamp ever reaches a
/// committed snapshot.
fn redacted_settings() -> insta::Settings {
    let mut settings = insta::Settings::clone_current();
    // Uuid::new_v4().to_string(): lowercase-hex 8-4-4-4-12.
    settings.add_filter(
        r"[0-9a-fA-F]{8}-[0-9a-fA-F]{4}-[0-9a-fA-F]{4}-[0-9a-fA-F]{4}-[0-9a-fA-F]{12}",
        "[uuid]",
    );
    // Utc::now() rendered either as "%Y-%m-%dT%H:%M:%S" (xodr_exporter.rs,
    // openscenario-rs's FileHeader builder) or full RFC3339 with a
    // fractional-second/offset tail (belt and braces for any format that
    // shows up in generated output).
    settings.add_filter(
        r"\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}(\.\d+)?(Z|[+-]\d{2}:\d{2})?",
        "[timestamp]",
    );
    settings
}

/// Snapshot one example's generated JSON, `.xosc` and `.xodr`, under a
/// shared `snapshot_name` prefix.
fn snapshot_example(file: &str, snapshot_name: &str) {
    let settings = redacted_settings();
    settings.bind(|| {
        let scenario: Scenario = common::generate_example(file);

        let json = serde_json::to_string_pretty(&scenario)
            .unwrap_or_else(|e| panic!("{file}: failed to serialize scenario to JSON: {e}"));
        assert_snapshot!(format!("{snapshot_name}_json"), json);

        let xosc = export_scenario_to_xosc(&scenario)
            .unwrap_or_else(|e| panic!("{file}: xosc export failed: {e}"));
        assert_snapshot!(format!("{snapshot_name}_xosc"), xosc);

        let xodr = export_scenario_to_xodr(&scenario)
            .unwrap_or_else(|e| panic!("{file}: xodr export failed: {e}"));
        assert_snapshot!(format!("{snapshot_name}_xodr"), xodr);
    });
}

/// Cartesian coordinate system, with a lane change (`cut_in_left.yaml`).
#[test]
fn snapshot_cut_in_left_cartesian() {
    snapshot_example("cut_in_left.yaml", "cut_in_left");
}

/// Bicycle model coordinate system (`bicycle_lane_change.yaml`).
#[test]
fn snapshot_bicycle_lane_change() {
    snapshot_example("bicycle_lane_change.yaml", "bicycle_lane_change");
}

/// Pedestrian physics (`pedestrian_crossing.yaml`).
#[test]
fn snapshot_pedestrian_crossing() {
    snapshot_example("pedestrian_crossing.yaml", "pedestrian_crossing");
}
