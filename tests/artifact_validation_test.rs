//! Validate the artifacts we actually ship (SW-05).
//!
//! Before this file, `.xosc` and `.xodr` output was never parsed, let alone
//! schema-validated — `tests/export_coverage_test.rs` only ran substring
//! checks like `xosc.contains("OpenSCENARIO") || xosc.contains("<?xml")`,
//! which is true of nearly any XML document. This file:
//!
//! 1. XSD-validates every generated `.xosc` against the real OpenSCENARIO
//!    schema.
//! 2. Round-trips every `.xosc` through `openscenario_rs::parse_from_str`
//!    and checks entity count, actor names and trajectory vertex count
//!    survive.
//! 3. Structurally walks every `.xodr`: geometry continuity, road length
//!    vs. the sum of its geometries, lane-section `s` monotonicity, and
//!    that every lane section has a `<center>`.
//! 4. Parses every `.svg` as XML and checks its `viewBox` contains every
//!    actor marker drawn onto it.
//!
//! Driven by `common::solvable_examples()` — the example corpus discovered
//! from disk via `common::example_paths()` — never a hand-written list, so a
//! new example is swept in automatically. Artifacts are generated fresh into
//! memory (nothing is read from or written to `output/`, which is stale and
//! gitignored).
//!
//! ## Locating the XSD
//!
//! `openscenario-rs` bundles `Schema/OpenSCENARIO.xsd`, but that path lives
//! at the sibling path dependency `../../Workspace_OpenScenario-rs/main`,
//! which is explicitly not something a test here may hardcode (SW-01 is
//! removing that path dependency; a test must not depend on its location).
//! `OpenSCENARIO.xsd` is a single self-contained file (no `xsd:include`s —
//! verified by grep), so it was copied byte-for-byte into
//! `tests/schemas/OpenSCENARIO.xsd` alongside the ASAM `NOTICE` file that
//! accompanies it. This makes the test independent of the sibling checkout
//! and reproducible off this machine, at the cost of needing a manual
//! re-copy if the schema is ever upgraded (there is no automated sync; the
//! two schema copies can drift, and nothing currently detects that).

mod common;

use std::collections::HashMap;
use std::path::PathBuf;
use std::sync::Mutex;

use libxml::parser::Parser as XmlParser;
use libxml::schemas::{SchemaParserContext, SchemaValidationContext};
use uom::si::length::meter;

use scenario_weaver::scenario::model::Scenario;
use scenario_weaver::{export_scenario_to_svg, export_scenario_to_xodr, export_scenario_to_xosc};

/// Numeric tolerance for geometry-continuity comparisons.
///
/// Not `tests/common/invariants.rs::TOL` (1e-6, calibrated to Z3's exact
/// rationals): the xodr/xosc exporters do their own f64 arithmetic
/// (`road_length = max_x * 1.2`, trigonometric lane offsets) on top of the
/// solver's output, so a slightly looser but still tight bound is used here.
const EPS: f64 = 1e-6;

/// libxml's `SchemaValidationContext` is documented (libxml 0.3.8
/// `tests/schema_tests.rs`) as unsafe to use concurrently from multiple
/// threads in libxml2 >= 2.12. `cargo nextest run` (what `scripts/check.sh`
/// prefers) isolates every test in its own process, so this does not matter
/// there; a plain `cargo test` run puts every test in this binary on a
/// shared thread pool, so this lock keeps that path from flaking.
static XSD_LOCK: Mutex<()> = Mutex::new(());

fn xsd_path() -> PathBuf {
    common::project_root().join("tests/schemas/OpenSCENARIO.xsd")
}

/// Validate one `.xosc` document against the bundled OpenSCENARIO XSD.
///
/// Fails loudly (rather than silently skipping) if the schema itself cannot
/// be loaded — a broken `tests/schemas/OpenSCENARIO.xsd` must not present as
/// "every example passed validation".
fn validate_against_xsd(xml: &str, label: &str) -> Result<(), String> {
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

// ---------------------------------------------------------------------------
// .xosc: XSD validation
// ---------------------------------------------------------------------------

/// Every solvable example's `.xosc` output passes XSD validation against the
/// real OpenSCENARIO schema.
///
/// Baseline before this test existed: the 40 committed (stale) artifacts
/// under `output/` were independently confirmed to fail 40/40
/// (`xmllint --noout --schema ... `, `Element 'RoadNetwork': This element is
/// not expected`) — see FINDINGS.md T3. This test proves current source
/// output is not in that state.
#[test]
fn every_example_xosc_passes_xsd_validation() {
    let mut failures = Vec::new();
    let mut ok_count = 0usize;

    for (name, spec) in common::solvable_examples() {
        let (scenario, _spec) = common::generate_spec_with_spec(spec);
        let xosc = export_scenario_to_xosc(&scenario)
            .unwrap_or_else(|e| panic!("{name}: xosc export failed: {e}"));

        match validate_against_xsd(&xosc, name) {
            Ok(()) => ok_count += 1,
            Err(msg) => failures.push(msg),
        }
    }

    assert!(
        failures.is_empty(),
        "{ok_count} passed, {} failed XSD validation:\n{}",
        failures.len(),
        failures.join("\n")
    );
}

// ---------------------------------------------------------------------------
// .xosc: round-trip through openscenario_rs::parse_from_str
// ---------------------------------------------------------------------------

/// Total vertex count of every actor's trajectory in a parsed `.xosc`,
/// keyed by actor id.
///
/// Walks `Storyboard -> Story -> Act -> ManeuverGroup -> Maneuver -> Event ->
/// Action -> PrivateAction -> RoutingAction -> FollowTrajectoryAction`. The
/// exporter (`src/scenario/xosc_exporter.rs`) gives every actor its own Act
/// with a single-entity ManeuverGroup (`actors.entity_refs[0]`), so the
/// first (and only) entity ref on a maneuver group identifies which actor
/// that act's trajectory belongs to.
fn xosc_trajectory_vertex_counts(
    doc: &openscenario_rs::types::scenario::storyboard::OpenScenario,
) -> HashMap<String, usize> {
    let mut out = HashMap::new();
    let Some(storyboard) = &doc.storyboard else {
        return out;
    };

    for story in &storyboard.stories {
        for act in &story.acts {
            for group in &act.maneuver_groups {
                let Some(actor_id) = group
                    .actors
                    .entity_refs
                    .first()
                    .and_then(|e| e.entity_ref.as_literal())
                else {
                    continue;
                };

                let mut vertex_count = 0usize;
                for maneuver in &group.maneuvers {
                    for event in &maneuver.events {
                        for action in &event.actions {
                            if let Some(vertices) = action
                                .private_action
                                .as_ref()
                                .and_then(|pa| pa.routing_action.as_ref())
                                .and_then(|ra| ra.follow_trajectory_action.as_ref())
                                .and_then(|fta| fta.trajectory.as_ref())
                                .and_then(|traj| traj.shape.polyline.as_ref())
                            {
                                vertex_count += vertices.vertices.len();
                            }
                        }
                    }
                }
                out.insert(actor_id.clone(), vertex_count);
            }
        }
    }

    out
}

/// Every solvable example's `.xosc` round-trips through
/// `openscenario_rs::parse_from_str` with entity count, actor names, and
/// per-actor trajectory vertex count preserved.
///
/// This catches structural errors XSD validation alone misses: XSD checks
/// "is this a legal document", not "does it say the same thing the
/// `Scenario` we started from said".
#[test]
fn every_example_xosc_round_trips_with_structure_preserved() {
    for (name, spec) in common::solvable_examples() {
        let (scenario, _spec) = common::generate_spec_with_spec(spec);
        let xosc = export_scenario_to_xosc(&scenario)
            .unwrap_or_else(|e| panic!("{name}: xosc export failed: {e}"));

        let doc = openscenario_rs::parse_from_str(&xosc).unwrap_or_else(|e| {
            panic!("{name}: xosc did not round-trip through parse_from_str: {e}")
        });

        let entities = doc
            .entities
            .as_ref()
            .unwrap_or_else(|| panic!("{name}: round-tripped document has no <Entities>"));
        assert_eq!(
            entities.scenario_objects.len(),
            scenario.actors.len(),
            "{name}: entity count not preserved"
        );

        let mut parsed_names: Vec<&str> = entities
            .scenario_objects
            .iter()
            .map(|so| so.name.as_literal().map_or("", String::as_str))
            .collect();
        parsed_names.sort_unstable();
        let mut actor_ids: Vec<&str> = scenario.actors.iter().map(|a| a.id.as_str()).collect();
        actor_ids.sort_unstable();
        assert_eq!(parsed_names, actor_ids, "{name}: actor names not preserved");

        let vertex_counts = xosc_trajectory_vertex_counts(&doc);
        for actor in &scenario.actors {
            let got = *vertex_counts.get(actor.id.as_str()).unwrap_or_else(|| {
                panic!(
                    "{name}: no FollowTrajectoryAction found for actor '{}'",
                    actor.id
                )
            });
            assert_eq!(
                got,
                actor.states.len(),
                "{name}: actor '{}' trajectory vertex count not preserved ({} states in, {} vertices out)",
                actor.id,
                actor.states.len(),
                got
            );
        }
    }
}

// ---------------------------------------------------------------------------
// .xodr: structural walk
// ---------------------------------------------------------------------------

fn xodr_for(scenario: &Scenario, label: &str) -> opendrive::core::OpenDrive {
    let xodr = export_scenario_to_xodr(scenario)
        .unwrap_or_else(|e| panic!("{label}: xodr export failed: {e}"));
    opendrive::core::OpenDrive::from_xml_str(&xodr)
        .unwrap_or_else(|e| panic!("{label}: xodr did not parse: {e}"))
}

/// Every solvable example's `.xodr`: geometry continuity (`s + length` of
/// one `<geometry>` equals the next `s`), road `length` equal to the sum of
/// its geometries' lengths, lane-section `s` values monotonic, and every
/// `<laneSection>` has a `<center>`.
///
/// The last point is enforced by the `opendrive` crate's own type
/// (`LaneSection::center: Center`, not `Option<Center>`) rather than
/// asserted here: a `<laneSection>` missing `<center>` would fail to parse
/// at all, so the `xodr_for` call above already proves it for every example
/// that reaches this point.
#[test]
fn every_example_xodr_structure_is_well_formed() {
    for (name, spec) in common::solvable_examples() {
        let (scenario, _spec) = common::generate_spec_with_spec(spec);
        let doc = xodr_for(&scenario, name);

        for road in &doc.road {
            let geometries: Vec<_> = road.plan_view.geometry.iter().collect();
            for pair in geometries.windows(2) {
                let end = pair[0].s.get::<meter>() + pair[0].length.get::<meter>();
                let next_s = pair[1].s.get::<meter>();
                assert!(
                    (end - next_s).abs() < EPS,
                    "{name}: road '{}' geometry discontinuity: segment ends at s={end}, next starts at s={next_s}",
                    road.id
                );
            }

            let sum_of_lengths: f64 = geometries.iter().map(|g| g.length.get::<meter>()).sum();
            let road_length = road.length.get::<meter>();
            assert!(
                (sum_of_lengths - road_length).abs() < EPS,
                "{name}: road '{}' length {road_length} != sum of geometry lengths {sum_of_lengths}",
                road.id
            );

            let mut prev_s: Option<f64> = None;
            for section in &road.lanes.lane_section {
                if let Some(prev) = prev_s {
                    assert!(
                        section.s + EPS >= prev,
                        "{name}: road '{}' lane-section s not monotonic: {prev} then {}",
                        road.id,
                        section.s
                    );
                }
                prev_s = Some(section.s);
                // `center` is present by construction — see doc comment above.
                let _ = &section.center;
            }
        }
    }
}

/// Road `length` must be at least as large as the furthest longitudinal
/// position (`x`) any actor reaches — otherwise a trajectory runs off the
/// end of the road a downstream simulator loaded it onto.
///
/// See "What the new validation found" in the SW-05 report: this is
/// expected to fail for some examples (M12) and is `#[ignore]`d rather than
/// weakened.
#[test]
#[ignore = "SW-16: road length can be shorter than the trajectories it carries \
            (M12) — export_to_xodr's compute_road_length() takes the RoadSpec's \
            road_length when set (parser.rs defaults it to duration * 30.0) \
            rather than the actor trajectories' actual max x, and the 20% \
            trajectory-derived fallback buffer is not applied when road_length \
            is Some. Observed: cut_in_left_optimize_max_ttc.yaml road length \
            300.00m < furthest actor x 303.26m; \
            cut_in_left_optimize_min_severity.yaml road length 300.00m < \
            furthest actor x 402.62m."]
fn every_example_xodr_road_length_covers_trajectories() {
    let mut failures = Vec::new();
    for (name, spec) in common::solvable_examples() {
        let (scenario, _spec) = common::generate_spec_with_spec(spec);
        let doc = xodr_for(&scenario, name);
        let road_length = doc.road[0].length.get::<meter>();

        let max_x = scenario
            .actors
            .iter()
            .flat_map(|a| a.states.iter())
            .map(|s| s.position().x)
            .fold(f64::MIN, f64::max);

        if max_x > road_length + EPS {
            failures.push(format!(
                "{name}: road length {road_length:.2}m < furthest actor x {max_x:.2}m"
            ));
        }
    }
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

/// Every actor position stays within the road surface the `.xodr` describes:
/// `y` (lateral) within `[0, num_lanes * lane_width]`, matching the mapping
/// `xodr_exporter::build_lane_section` uses (forward lanes project into
/// `[0, n_forward * lane_width]`, backward lanes into
/// `[n_forward * lane_width, num_lanes * lane_width]`).
///
/// See "What the new validation found" in the SW-05 report: pedestrian
/// scenarios are expected to fail this (E3), since a pedestrian crossing the
/// road legitimately starts and ends off the drivable surface (on a
/// sidewalk the `.xodr` does not model). `#[ignore]`d rather than weakened —
/// weakening it would also stop catching a vehicle actor drifting off the
/// road, which is the failure mode this check exists for.
#[test]
#[ignore = "SW-16: pedestrian scenarios put actors outside the road surface \
            (E3) — the xodr exporter only models the drivable lanes, and a \
            pedestrian crossing them starts/ends off that surface by design. \
            Observed: pedestrian_crossing.yaml 'pedestrian' reaches y=13.48 \
            against a [0, 7.00] road surface; pedestrian_running.yaml \
            'runner' reaches y=8.39 against [0, 7.00]; \
            pedestrian_wide_road.yaml 'ped' reaches y=14.29 against \
            [0, 10.50]."]
fn every_example_actors_stay_within_road_surface() {
    let mut failures = Vec::new();
    for (name, spec) in common::solvable_examples() {
        let (scenario, _spec) = common::generate_spec_with_spec(spec);
        let road_width = scenario.road.num_lanes as f64 * scenario.road.lane_width;

        for actor in &scenario.actors {
            for state in &actor.states {
                let y = state.position().y;
                if y < -EPS || y > road_width + EPS {
                    failures.push(format!(
                        "{name}: actor '{}' y={y:.2} outside road surface [0, {road_width:.2}]",
                        actor.id
                    ));
                }
            }
        }
    }
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

// ---------------------------------------------------------------------------
// .svg: parses as XML, viewBox contains every actor marker
// ---------------------------------------------------------------------------

/// `(x, y, width, height)` from an `<svg viewBox="...">` attribute, plus the
/// `(cx, cy)` of every `<circle>` in the document (vehicles, pedestrians,
/// and legend markers alike — the legend is drawn inside the same fixed
/// canvas, so it is a legitimate part of "every marker this document
/// draws").
#[allow(clippy::type_complexity)]
fn parse_svg_markers(svg: &str, label: &str) -> ((f64, f64, f64, f64), Vec<(f64, f64)>) {
    use quick_xml::events::Event;
    use quick_xml::reader::Reader;

    let mut reader = Reader::from_str(svg);
    let mut view_box = None;
    let mut circles = Vec::new();

    loop {
        let event = reader
            .read_event()
            .unwrap_or_else(|e| panic!("{label}: svg did not parse as XML: {e}"));
        match event {
            Event::Eof => break,
            Event::Start(e) | Event::Empty(e) => {
                let local = e.local_name();
                let local = std::str::from_utf8(local.as_ref()).unwrap_or_default();
                if local == "svg" && view_box.is_none() {
                    for attr in e.attributes().flatten() {
                        if attr.key.as_ref() == b"viewBox" {
                            let value = String::from_utf8_lossy(&attr.value).into_owned();
                            let nums: Vec<f64> = value
                                .split_whitespace()
                                .filter_map(|s| s.parse().ok())
                                .collect();
                            assert_eq!(
                                nums.len(),
                                4,
                                "{label}: viewBox should have 4 numbers, got '{value}'"
                            );
                            view_box = Some((nums[0], nums[1], nums[2], nums[3]));
                        }
                    }
                } else if local == "circle" {
                    let mut cx = None;
                    let mut cy = None;
                    for attr in e.attributes().flatten() {
                        let value = String::from_utf8_lossy(&attr.value).into_owned();
                        match attr.key.as_ref() {
                            b"cx" => cx = value.parse::<f64>().ok(),
                            b"cy" => cy = value.parse::<f64>().ok(),
                            _ => {}
                        }
                    }
                    if let (Some(cx), Some(cy)) = (cx, cy) {
                        circles.push((cx, cy));
                    }
                }
            }
            _ => {}
        }
    }

    let view_box = view_box.unwrap_or_else(|| panic!("{label}: svg has no <svg viewBox=...>"));
    (view_box, circles)
}

/// Every solvable example's `.svg` parses as well-formed XML, and every
/// actor/legend marker (`<circle cx cy>`) drawn onto it falls inside the
/// declared `viewBox`.
#[test]
fn every_example_svg_viewbox_contains_actor_markers() {
    for (name, spec) in common::solvable_examples() {
        let (scenario, _spec) = common::generate_spec_with_spec(spec);
        let svg = export_scenario_to_svg(&scenario)
            .unwrap_or_else(|e| panic!("{name}: svg export failed: {e}"));

        let ((vb_x, vb_y, vb_w, vb_h), circles) = parse_svg_markers(&svg, name);
        assert!(
            !circles.is_empty(),
            "{name}: svg has no <circle> markers to check"
        );

        for (cx, cy) in circles {
            assert!(
                cx >= vb_x - EPS && cx <= vb_x + vb_w + EPS,
                "{name}: marker cx={cx} outside viewBox x-range [{vb_x}, {}]",
                vb_x + vb_w
            );
            assert!(
                cy >= vb_y - EPS && cy <= vb_y + vb_h + EPS,
                "{name}: marker cy={cy} outside viewBox y-range [{vb_y}, {}]",
                vb_y + vb_h
            );
        }
    }
}
