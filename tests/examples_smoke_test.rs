//! Corpus sweep over `examples/*.yaml`.
//!
//! The example set is discovered by reading the directory, never from a
//! hand-maintained list, and every example carries a committed expectation in
//! `common::EXAMPLE_EXPECTATIONS`. Adding an example without an expectation
//! fails `test_expectations_cover_every_example`.

mod common;

use common::{Expect, KNOWN_BROKEN_EXAMPLES};
use std::collections::BTreeSet;

/// The expectation table plus the known-broken list must describe exactly the
/// files on disk — no extras, no omissions.
#[test]
fn test_expectations_cover_every_example() {
    let on_disk: BTreeSet<String> = common::example_names().into_iter().collect();

    let declared: BTreeSet<String> = common::EXAMPLE_EXPECTATIONS
        .iter()
        .map(|(name, _)| (*name).to_string())
        .chain(KNOWN_BROKEN_EXAMPLES.iter().map(|(n, _)| (*n).to_string()))
        .collect();

    let uncovered: Vec<&String> = on_disk.difference(&declared).collect();
    let stale: Vec<&String> = declared.difference(&on_disk).collect();

    assert!(
        uncovered.is_empty(),
        "examples with no committed expectation: {uncovered:?} \
         — add them to common::EXAMPLE_EXPECTATIONS"
    );
    assert!(
        stale.is_empty(),
        "expectations for examples that no longer exist: {stale:?}"
    );
    assert_eq!(
        on_disk.len(),
        declared.len(),
        "expectation table and examples/ disagree"
    );
}

/// Every example must produce exactly the outcome it is committed to.
///
/// All examples are run before failing, so one regression does not hide the
/// others.
#[test]
fn test_every_example_matches_its_expectation() {
    let mut failures: Vec<String> = Vec::new();

    for &(name, expect) in common::EXAMPLE_EXPECTATIONS {
        let spec = common::parse_example(name);
        match common::check_expectation(name, spec, expect) {
            Ok(line) => println!("{line}"),
            Err(msg) => failures.push(msg),
        }
    }

    assert!(
        failures.is_empty(),
        "{} of {} examples did not match their committed expectation:\n  {}",
        failures.len(),
        common::EXAMPLE_EXPECTATIONS.len(),
        failures.join("\n  ")
    );
}

/// Solvable examples must yield trajectories that are actually populated —
/// a scenario with actors but no states would otherwise pass every export test.
#[test]
fn test_solvable_examples_have_populated_trajectories() {
    for &(name, expect) in common::EXAMPLE_EXPECTATIONS {
        if expect != Expect::Solvable {
            continue;
        }
        let scenario = common::generate_example(name);
        // Mirrors ScenarioSpec::num_time_steps: ceil(duration / time_step) intervals,
        // hence one more state than intervals.
        let expected_steps = (scenario.duration / scenario.time_step).ceil() as usize + 1;
        for actor in &scenario.actors {
            assert_eq!(
                actor.states.len(),
                expected_steps,
                "{name}: actor {} has {} states, expected {expected_steps} \
                 for duration={} time_step={}",
                actor.id,
                actor.states.len(),
                scenario.duration,
                scenario.time_step
            );
        }
    }
}

/// `examples/with_import.yaml` documents `cargo run -- -i examples/with_import.yaml`,
/// but `imports: roads/4_lane_bidirectional.yaml` is resolved relative to the
/// YAML's own directory while `roads/` lives at the repo root. The example is
/// therefore unloadable from the path it advertises.
///
/// `tests/road_import_test.rs` works around this by copying the file to the repo
/// root before parsing; this test asserts the behaviour that is actually wanted.
#[test]
#[ignore = "SW-21: examples/with_import.yaml import path resolves relative to examples/, but roads/ is at the repo root"]
fn test_with_import_example_loads_from_its_own_directory() {
    let path = common::example_path("with_import.yaml");
    let spec = scenario_weaver::dsl::parse_yaml_file(&path)
        .unwrap_or_else(|e| panic!("with_import.yaml should resolve its own imports: {e}"));

    let road = spec.road.as_ref().expect("import should supply a road");
    assert_eq!(road.num_lanes, 4);
    assert_eq!(road.lane_directions, vec![1, 1, -1, -1]);

    let scenario = common::generate_spec_or_fail(spec);
    assert!(!scenario.actors.is_empty());
}
