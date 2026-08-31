//! Corpus sweep over `examples/*.yaml`.
//!
//! The example set is discovered by reading the directory, never from a
//! hand-maintained list, and every example carries a committed expectation in
//! `common::EXAMPLE_EXPECTATIONS`. Adding an example without an expectation
//! fails `test_expectations_cover_every_example`.

mod common;

use common::{
    check_scenario_invariants, examples_declaring, is_known_broken, Expect, Invariant,
    KNOWN_BROKEN_EXAMPLES, KNOWN_BROKEN_INVARIANTS,
};
use scenario_weaver::dsl::types::ConstraintMode;
use std::collections::{BTreeMap, BTreeSet};

/// Generate every solvable example once and collect every invariant breach,
/// keyed by example name.
///
/// One sweep, reused by each per-invariant test below, because generation — not
/// checking — is what costs.
fn corpus_violations() -> Vec<(&'static str, Vec<common::Violation>)> {
    common::solvable_examples()
        .into_iter()
        .map(|(name, spec)| {
            let scenario = common::generate_example(name);
            let found = check_scenario_invariants(&scenario, &spec);
            (name, found)
        })
        .collect()
}

/// Assert that `invariant` holds across the whole corpus, reporting every
/// breach at once rather than stopping at the first.
fn assert_corpus_invariant(invariant: Invariant) {
    let sweep = corpus_violations();
    let mut failing = 0usize;
    let mut lines: Vec<String> = Vec::new();

    for (name, found) in &sweep {
        let mine: Vec<&common::Violation> =
            found.iter().filter(|v| v.invariant == invariant).collect();
        if mine.is_empty() {
            continue;
        }
        failing += 1;
        // One example can breach an invariant at every step of every actor;
        // the first few carry the diagnosis, the rest are the same story.
        lines.push(format!(
            "{name}: {} breach(es), first 3:\n      {}",
            mine.len(),
            mine.iter()
                .take(3)
                .map(|v| v.detail.clone())
                .collect::<Vec<_>>()
                .join("\n      ")
        ));
    }

    assert!(
        lines.is_empty(),
        "{invariant} fails on {failing} of {} examples:\n  {}",
        sweep.len(),
        lines.join("\n  ")
    );
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

// ---------------------------------------------------------------------------
// Scenario invariants across the corpus (SW-06)
// ---------------------------------------------------------------------------

/// The two things this repository asserts about every scenario it generates
/// today, swept over the whole corpus, plus the proof that the known-broken
/// baseline is not stale.
///
/// `common::generate_example` already applies the non-baseline invariants to
/// every scenario the suite produces, so the first half of this test is a
/// belt-and-braces statement of that at corpus scope. The second half is the
/// ratchet: every entry in `KNOWN_BROKEN_INVARIANTS` must still be violated by
/// at least one example. When SW-08, SW-10 or SW-12 lands, its entry stops
/// being violated, this test fails, and the entry has to be deleted — which
/// promotes that invariant to enforced everywhere in one edit.
#[test]
fn test_known_broken_invariants_are_still_broken() {
    let sweep = corpus_violations();

    let leaked: Vec<String> = sweep
        .iter()
        .flat_map(|(name, found)| {
            found
                .iter()
                .filter(|v| !is_known_broken(v.invariant))
                .map(move |v| format!("{name}: {v}"))
        })
        .collect();
    assert!(
        leaked.is_empty(),
        "invariants outside the known-broken baseline were violated:\n  {}",
        leaked.join("\n  ")
    );

    let mut counts: BTreeMap<String, usize> = BTreeMap::new();
    for (_, found) in &sweep {
        for v in found {
            *counts.entry(v.invariant.to_string()).or_default() += 1;
        }
    }
    println!("corpus invariant breaches by invariant: {counts:?}");

    let stale: Vec<&str> = KNOWN_BROKEN_INVARIANTS
        .iter()
        .filter(|(inv, _)| {
            !sweep
                .iter()
                .any(|(_, found)| found.iter().any(|v| v.invariant == *inv))
        })
        .map(|(_, why)| *why)
        .collect();
    assert!(
        stale.is_empty(),
        "{} entry/entries in KNOWN_BROKEN_INVARIANTS are no longer violated by any \
         example — the owning issue has landed, so delete them from the baseline and \
         let the invariant be enforced everywhere:\n  {}",
        stale.len(),
        stale.join("\n  ")
    );
}

/// Envelope compliance and model-vs-extraction agreement, at corpus scope.
///
/// Both hold today. They are the half of the invariant set that is live, and
/// this is where a regression in either surfaces as one named failure rather
/// than as forty scattered ones.
#[test]
fn test_envelope_compliance_across_the_corpus() {
    assert_corpus_invariant(Invariant::Envelope);
}

#[test]
fn test_extraction_agreement_across_the_corpus() {
    assert_corpus_invariant(Invariant::ExtractionAgreement);
}

/// `p[i+1] = p[i] + v[i]·dt + ½·a[i]·dt²` and `v[i+1] = v[i] + a[i]·dt`, on both
/// axes, at 1e-6.
///
/// Fails on all 21 solvable examples, for two distinct reasons:
///
/// - **H1 / SW-08.** The position update is forward Euler. The residual against
///   the constant-acceleration form is exactly `½·a·dt²` and the forward-Euler
///   residual is 0.0 to the last bit, on every example — this is not a numerical
///   artefact, the term is simply absent. `cut_in_left_adversarial_all`:
///   `px[t=0.50] = 57.5` where `p + v·dt + ½·a·dt² = 57.875`, a 0.375 m error
///   per step against a `min_distance` threshold of 5 m.
/// - **C2 / SW-09.** For vehicles, `ay` is never connected to `vy`: the
///   `vy[t+1] = vy[t] + ay[t]·dt` assertion sits inside a
///   `if role == Pedestrian` branch. `cut_in_left`'s npc steps
///   `vy = 0 → -2.0 → +2.0` with `ay = 0.0` throughout, and later
///   `+1.64 → -1.685`. The `ay` in the JSON and the `.xosc` is fiction, and
///   `max_lateral_acceleration` bounds a variable that constrains nothing.
#[test]
#[ignore = "SW-08 (position update drops the a*dt^2/2 term) and SW-09 (lateral acceleration is a free variable for vehicles)"]
fn test_kinematic_consistency_across_the_corpus() {
    assert_corpus_invariant(Invariant::Kinematics);
}

/// `Enforce ⟹ metric ≥ threshold`, `Violate ⟹ metric < threshold` strictly.
///
/// Two failure shapes:
///
/// - **SW-12.** `violate` is satisfied by equality.
///   `cut_in_left_adversarial_all` declares `min_ttc: violate` against a
///   threshold of 3 and reports a measured `min_ttc` of exactly 3.000000000.
///   Meeting a bound is not violating it.
/// - **SW-10.** `enforce` passes on examples where the metric was never
///   evaluated, because the lane variable lags lateral position and no
///   same-lane step ever exists — see
///   `test_enforce_min_ttc_examples_produce_a_measured_ttc`. Plus one genuine
///   breach: `head_on_near_miss` declares `min_ttc: enforce` at 2 s and
///   measures 1.05 s, with `all_constraints_satisfied` reported as true.
#[test]
#[ignore = "SW-12 (violate is satisfied by equality) and SW-10 (enforce passes on a metric that was never evaluated)"]
fn test_constraint_mode_semantics_across_the_corpus() {
    assert_corpus_invariant(Invariant::ConstraintModes);
}

/// Lateral position on the road surface, and `lane` consistent with `y`.
///
/// H2 / SW-10: `lane` is pinned on a schedule while `py` is pinned only at the
/// two endpoints of a lane change, so mid-manoeuvre the two disagree —
/// `cut_in_left_optimize_min_ttc`'s npc is recorded in lane 0 (centre 1.75)
/// with `py = 4.5`, a 2.75 m error against a 1.75 m half-width, i.e. it is
/// physically in the next lane but a full lane-width outside the one it is
/// recorded in. Every consumer keyed on `lane` — the same-lane TTC test,
/// `compute_effective_dist`, `compute_validation_metrics` — treats the actors
/// as separated during exactly the window a cut-in scenario is about.
#[test]
#[ignore = "SW-10: the lane variable lags lateral position through a lane change (H2); SW-08 also shifts lane centres by 5 cm (C1)"]
fn test_lane_road_containment_across_the_corpus() {
    assert_corpus_invariant(Invariant::Containment);
}

/// No scenario in which every vehicle is parked for a majority of the horizon.
///
/// Fails on all three pedestrian examples: the ego brakes at the encoder's
/// floor and then sits still — `pedestrian_crossing` has it at `vx = 0` for 29
/// of 35 steps (8.7 s of a 10 s horizon) — and `pedestrian_wide_road` reports
/// `all_constraints_satisfied: true` while doing it, because a stationary
/// vehicle trivially satisfies every distance and TTC threshold. A scenario
/// whose ego never reaches the pedestrian is not the scenario the YAML asks
/// for.
#[test]
#[ignore = "SW-12: the ego brakes to a standstill for a majority of the horizon on all three pedestrian examples and the result is still reported as satisfying every constraint"]
fn test_forward_progress_across_the_corpus() {
    assert_corpus_invariant(Invariant::ForwardProgress);
}

/// The whole point, stated once without the baseline: every scenario this
/// repository generates satisfies every invariant.
///
/// This is what green looks like when SW-08, SW-09, SW-10 and SW-12 have all
/// landed. Until then it is the standing record of what is left — run it with
/// `cargo test --no-fail-fast -- --ignored` to see the current state, and
/// prefer the four per-invariant tests above when you want the failure
/// attributed to one issue.
#[test]
#[ignore = "SW-08/SW-09 (kinematics), SW-10 (containment), SW-12 (constraint modes, forward progress) — the union of the four per-invariant tests above"]
fn test_every_scenario_satisfies_every_invariant() {
    for (name, spec) in common::solvable_examples() {
        let scenario = common::generate_example(name);
        common::assert_all_scenario_invariants(&scenario, &spec);
    }
}
