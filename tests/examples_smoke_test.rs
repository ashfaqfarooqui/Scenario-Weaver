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
/// and `imports:` is resolved relative to the YAML's own directory
/// (`examples/`), not the repo root. SW-13 fixed the example to declare
/// `imports: ../roads/4_lane_bidirectional.yaml` accordingly (and wired
/// `main.rs` through `parse_yaml_file`, which is what actually resolves
/// `imports:` — see `src/main.rs`), so this now exercises the real path.
#[test]
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
/// Before SW-10 this failed on 8 of the 15 candidates, and the cause was the
/// lane lag: `compute_validation_metrics` gated TTC on
/// `state1.lane() == state2.lane()`, and the `lane` variable was pinned on a
/// schedule, so during a lane change no same-lane step existed at all.
/// That half is fixed — `lane` is derived from `py` now, and the gate is the
/// same `encode_same_lane_constraint` predicate the encoder asserts.
/// `bicycle_lane_change`, `simple_bidirectional` and `unsafe_following` began
/// reporting a measured TTC as a direct result.
///
/// The 7 that remain fail for a different reason, and it is not a measurement
/// bug: in the solution Z3 returns, no actor ever approaches another *in its
/// own lane*, so TTC is genuinely undefined. Nothing in the encoding forces a
/// conflict — `Always(TTCGT(...))` is an implication, vacuously true when
/// nobody is closing — and on `cut_in_left` both vehicles simply brake to a
/// standstill 98 m apart (`Invariant::ForwardProgress`, SW-12) while the npc's
/// cut-in is executed perfectly and 63 m ahead. Making the declared conflict
/// actually happen is SW-12's forward-progress work, not a same-lane test.
#[test]
#[ignore = "SW-12: nothing forces a closing conflict, so Z3 answers with a scenario in \
            which no actor ever approaches another in its own lane and TTC is genuinely \
            undefined — on cut_in_left both vehicles brake to a standstill. The SW-10 half \
            (lane lagging lateral position, so no same-lane step existed at all) is fixed: \
            cut_in_left now spends 54 of 101 steps with both actors in lane 1 and reports a \
            min_distance of 63.49 m measured over that window, where before the fix it \
            reported 170.92 m measured over the handful of steps the lag left behind"]
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

/// Examples whose `min_distance: enforce` declaration the encoder does not
/// actually assert, each with the issue that owns the fix.
///
/// A ratchet in the same shape as `KNOWN_BROKEN_INVARIANTS`: an entry that
/// stops failing must be deleted, and
/// `test_enforce_min_distance_examples_meet_their_threshold` below fails if it
/// is not. Never add an entry to silence a new failure.
/// Empty since SW-12: `HeadOnModel::generate_safety` used to build TTC and
/// distance constraints for the ego <-> oncoming pair *only* — under the
/// comment "Only ego <-> oncoming gets the requested constraint mode" — while
/// `compute_validation_metrics` measured every pair, so the breach
/// `head_on_near_miss` reported was on ego <-> slow_npc, a pair the encoder
/// had never constrained. `generate_safety` now applies the declared
/// `Enforce` mode to every pair, exactly as `generate_default_safety` does.
const MIN_DISTANCE_NOT_ASSERTED: &[(&str, &str)] = &[];

/// An example that declares `min_distance: enforce` must end up satisfying its
/// own threshold, with the metric actually measured.
///
/// It exists because SW-04 briefly believed `overtake_left.yaml` violated it —
/// the metric reads `min_distance = 4.0826` with
/// `all_constraints_satisfied: true`, which looks like a breach until you
/// notice that example declares `min_distance: 4.0`, not the corpus-typical
/// 5.0. The invariant was never actually broken, but nothing in the suite was
/// checking it either, so the question could only be settled by hand. Now it is
/// checked, against each example's own declared threshold.
///
/// `MIN_DISTANCE_NOT_ASSERTED` carries the one example the encoder does not
/// assert the constraint for, and is checked both ways.
#[test]
fn test_enforce_min_distance_examples_meet_their_threshold() {
    let mut failures: Vec<String> = Vec::new();
    let mut fixed: Vec<String> = Vec::new();

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
        let excused = MIN_DISTANCE_NOT_ASSERTED
            .iter()
            .find(|(n, _)| n == name)
            .map(|(_, why)| *why);

        // `>= threshold - TOL`, not a bare `>=`. Since SW-12 the encoder
        // asserts `|dx| >= min_distance` non-strictly (so that `Violate` can
        // negate it strictly), and Z3 answers *on* the boundary: `dx` comes
        // back as exactly 5 as a rational. Rendering the two positions as
        // `f64` and subtracting does not always round back to 5 —
        // `cut_in_right_bicycle` and `simple_bidirectional` both report
        // 4.999999999999999 — so a bare `>=` fails an example the solver
        // proved. `TOL` is 1e-6, the rational-to-double rounding error
        // `tests/common/invariants.rs` documents and uses for the same
        // quantities; the discrepancy it excuses here is 1e-15.
        let met = match scenario.validation.min_distance {
            Some(d) if d >= threshold - common::TOL => {
                println!("{name}: min_distance={d:.4} >= {threshold}");
                true
            }
            Some(d) => {
                if excused.is_none() {
                    failures.push(format!(
                        "{name}: declares min_distance: enforce (threshold {threshold}) but \
                         measured {d:.4}; all_constraints_satisfied={}, \
                         safety_violations={:?}",
                        scenario.validation.all_constraints_satisfied,
                        scenario.validation.safety_violations
                    ));
                }
                false
            }
            None => {
                if excused.is_none() {
                    failures.push(format!(
                        "{name}: declares min_distance: enforce (threshold {threshold}) but \
                         min_distance was never evaluated"
                    ));
                }
                false
            }
        };

        if met {
            if let Some(why) = excused {
                fixed.push(format!("{name} — listed as: {why}"));
            }
        }
    }

    assert!(
        failures.is_empty(),
        "{} of {} examples declaring min_distance: enforce did not satisfy it:\n  {}",
        failures.len(),
        candidates.len(),
        failures.join("\n  ")
    );
    assert!(
        fixed.is_empty(),
        "{} entr(ies) in MIN_DISTANCE_NOT_ASSERTED now satisfy their threshold — delete \
         them rather than leaving the list to rot:\n  {}",
        fixed.len(),
        fixed.join("\n  ")
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
/// Failed on all 21 solvable examples when this was written, for two distinct
/// reasons. One is fixed:
///
/// - **H1 / SW-08 — fixed.** The position update was forward Euler; the
///   residual against the constant-acceleration form was exactly `½·a·dt²` and
///   the forward-Euler residual 0.0 to the last bit, on every example
///   (`cut_in_left_adversarial_all`: `px[t=0.50] = 57.5` where
///   `p + v·dt + ½·a·dt² = 57.875`, 0.375 m per step against a `min_distance`
///   threshold of 5 m). The update is now written trapezoidally,
///   `p[t+1] = p[t] + (v[t] + v[t+1])·dt/2`, which is the same constraint given
///   the velocity update and keeps position coupled only to the velocity chain.
///   Zero `px` and zero `vx` breaches remain corpus-wide, and the three
///   pedestrian examples satisfy the whole invariant.
/// - **C2 / SW-09 — fixed.** For vehicles, `ay` was never connected to `vy`:
///   the `vy[t+1] = vy[t] + ay[t]·dt` assertion sat inside a
///   `if role == Pedestrian` branch while the `py` update below it applied to
///   everyone. `cut_in_left`'s npc stepped `vy = 0 → -2.0 → +2.0` with
///   `ay = 0.0` throughout, so the `ay` in the JSON and the `.xosc` was fiction
///   and `max_lateral_acceleration` bounded a variable that constrained
///   nothing. Both axes now integrate the accelerations reported beside them,
///   through one shared point-mass step in `solver::encoders::pedestrian` that
///   the Cartesian encoder uses for every actor and the Bicycle encoder uses
///   for pedestrians. 234 breaches corpus-wide before, 0 after.
#[test]
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
/// The SW-10 half is fixed. `lane` is no longer pinned on a schedule: it is
/// derived from `py` at every step of a lane change
/// (`|py - lane*w - w/2| <= w/2`), so the two can no longer disagree. Every
/// one of the 19 vehicle examples passes this invariant now; before the fix
/// `cut_in_left` alone breached it at 16 of 101 steps with a worst error of
/// 3.46 m against a 1.75 m half-width.
///
/// What is left is pedestrians, and it is a different defect: E3 / SW-16
/// encodes `OnSidewalk` as the unbounded half-plane
/// `py > lane_width * num_lanes`, so a crossing pedestrian ends up parked
/// outside the road surface entirely (`pedestrian_crossing`: `py = 7.85` on a
/// road of `[0, 7]`) with its `lane` still reading the lane it set off from.
/// No amount of lane-vs-`py` coupling fixes a `py` that is off the road.
#[test]
#[ignore = "SW-16 (E3): pedestrians park outside the road surface, so `lane` cannot agree \
            with `py` for them. The SW-10 half — lane lagging lateral position through a \
            lane change — is fixed and all 19 vehicle examples pass"]
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
#[ignore = "SW-09 (kinematics, lateral only since SW-08), SW-10 (containment), \
            SW-12 (constraint modes, forward progress) — the union of the four \
            per-invariant tests above"]
fn test_every_scenario_satisfies_every_invariant() {
    for (name, spec) in common::solvable_examples() {
        let scenario = common::generate_example(name);
        common::assert_all_scenario_invariants(&scenario, &spec);
    }
}
