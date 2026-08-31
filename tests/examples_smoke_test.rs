//! Corpus sweep over `examples/*.yaml`.
//!
//! The example set is discovered by reading the directory, never from a
//! hand-maintained list, and every example carries a committed expectation in
//! `common::EXAMPLE_EXPECTATIONS`. Adding an example without an expectation
//! fails `test_expectations_cover_every_example`.

mod common;

use common::{Expect, KNOWN_BROKEN_EXAMPLES};
use scenario_weaver::dsl::types::{ConstraintMode, ScenarioSpec};
use std::collections::BTreeSet;

/// Every solvable example whose spec declares `mode` for the constraint selected
/// by `select`, paired with its parsed spec.
///
/// Derived from `EXAMPLE_EXPECTATIONS` and the spec itself rather than from a
/// hand-written list, so an example that changes its constraint modes — or a new
/// example that declares `enforce` — is picked up automatically.
fn examples_declaring(
    select: fn(&ScenarioSpec) -> ConstraintMode,
    mode: ConstraintMode,
) -> Vec<(&'static str, ScenarioSpec)> {
    common::EXAMPLE_EXPECTATIONS
        .iter()
        .filter(|(_, expect)| *expect == Expect::Solvable)
        .map(|(name, _)| (*name, common::parse_example(name)))
        .filter(|(_, spec)| select(spec) == mode)
        .collect()
}

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

/// An example that declares `min_ttc: enforce` must actually produce a measured
/// `min_ttc`. Enforcing a constraint the validator never evaluates is not
/// enforcement.
///
/// Before SW-04 these examples shipped `"min_ttc": 999.0` — a fiction that
/// satisfied any `>=` threshold. SW-04 made the state honest (the field is now
/// absent from the JSON), but honest and correct are different things: nothing
/// alerts on it, which is what this test is for.
///
/// Observed failures, all with `all_constraints_satisfied: true` and an empty
/// violation list: `bicycle_lane_change`, `cut_in_left`,
/// `cut_in_left_optimize_min_severity`, `cut_in_right`, `multi_lane_safety`,
/// `simple_bidirectional`, `speed_limit_violation`.
///
/// Cause (SW-10): the `lane` variable lags the lateral position, so during a
/// lane change the two actors are never recorded in the same lane at a step
/// where they are also approaching. `compute_validation_metrics` gates TTC on
/// `state1.lane() == state2.lane()`, so it never evaluates. In `cut_in_left` the
/// npc reaches lane 1's centre (`y=4.70`) at t=4.0 s while `lane` still reads 0,
/// and by the time `lane` flips at t=5.0 s both actors have `vx=0.00`.
#[test]
#[ignore = "SW-10: lane variable lags lateral position, so same-lane TTC is never evaluated for 7 enforce-mode examples"]
fn test_enforce_min_ttc_examples_produce_a_measured_ttc() {
    let mut failures: Vec<String> = Vec::new();

    let candidates = examples_declaring(
        |spec| spec.constraint_modes.min_ttc(),
        ConstraintMode::Enforce,
    );
    assert!(
        !candidates.is_empty(),
        "no example declares min_ttc: enforce — the corpus or the selector is wrong"
    );

    for (name, spec) in &candidates {
        let scenario = common::generate_example(name);
        match scenario.validation.min_ttc {
            Some(ttc) => println!("{name}: min_ttc={ttc:.4} (threshold {})", spec.min_ttc),
            None => failures.push(format!(
                "{name}: declares min_ttc: enforce (threshold {}) but min_ttc was never \
                 evaluated; all_constraints_satisfied={}, safety_violations={:?}",
                spec.min_ttc,
                scenario.validation.all_constraints_satisfied,
                scenario.validation.safety_violations
            )),
        }
    }

    assert!(
        failures.is_empty(),
        "{} of {} examples declaring min_ttc: enforce never evaluated it:\n  {}",
        failures.len(),
        candidates.len(),
        failures.join("\n  ")
    );
}

/// An example that declares `min_distance: enforce` must end up satisfying its
/// own threshold, with the metric actually measured.
///
/// This passes today across the whole corpus. It exists because SW-04 briefly
/// believed `overtake_left.yaml` violated it — the metric reads
/// `min_distance = 4.0826` with `all_constraints_satisfied: true`, which looks
/// like a breach until you notice that example declares `min_distance: 4.0`, not
/// the corpus-typical 5.0. The invariant was never actually broken, but nothing
/// in the suite was checking it either, so the question could only be settled by
/// hand. Now it is checked, against each example's own declared threshold.
#[test]
fn test_enforce_min_distance_examples_meet_their_threshold() {
    let mut failures: Vec<String> = Vec::new();

    let candidates = examples_declaring(
        |spec| spec.constraint_modes.min_distance(),
        ConstraintMode::Enforce,
    );
    assert!(
        !candidates.is_empty(),
        "no example declares min_distance: enforce — the corpus or the selector is wrong"
    );

    for (name, spec) in &candidates {
        let scenario = common::generate_example(name);
        let threshold = spec.min_distance;
        match scenario.validation.min_distance {
            Some(d) if d >= threshold => println!("{name}: min_distance={d:.4} >= {threshold}"),
            Some(d) => failures.push(format!(
                "{name}: declares min_distance: enforce (threshold {threshold}) but measured \
                 {d:.4}; all_constraints_satisfied={}, safety_violations={:?}",
                scenario.validation.all_constraints_satisfied,
                scenario.validation.safety_violations
            )),
            None => failures.push(format!(
                "{name}: declares min_distance: enforce (threshold {threshold}) but \
                 min_distance was never evaluated"
            )),
        }
    }

    assert!(
        failures.is_empty(),
        "{} of {} examples declaring min_distance: enforce did not satisfy it:\n  {}",
        failures.len(),
        candidates.len(),
        failures.join("\n  ")
    );
}
