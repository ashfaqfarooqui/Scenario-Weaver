//! Integration tests for constraint modes (enforce, violate, ignore, violate_all)
//! and the optimizer, across scenario types.
//!
//! Every generation here has a committed expectation: either it must produce a
//! model, or it must be proved to have none. Nothing is accepted "either way".

mod common;

use scenario_weaver::dsl::types::{ConstraintMode, ConstraintModes, OptimizationTarget};
use scenario_weaver::scenario::model::Scenario;

/// Largest |v_a − v_b| over the shared timeline of two actors.
fn max_relative_velocity(scenario: &Scenario, a: &str, b: &str) -> f64 {
    let a = scenario.get_actor(a).expect("actor a");
    let b = scenario.get_actor(b).expect("actor b");
    a.states
        .iter()
        .zip(b.states.iter())
        .map(|(sa, sb)| (sa.velocity().vx - sb.velocity().vx).abs())
        .fold(0.0_f64, f64::max)
}

/// Smallest |y_a − y_b| over the shared timeline of two actors.
fn min_lateral_separation(scenario: &Scenario, a: &str, b: &str) -> f64 {
    let a = scenario.get_actor(a).expect("actor a");
    let b = scenario.get_actor(b).expect("actor b");
    a.states
        .iter()
        .zip(b.states.iter())
        .map(|(sa, sb)| (sa.position().y - sb.position().y).abs())
        .fold(f64::INFINITY, f64::min)
}

/// `cut_in_left.yaml` with the horizon shortened, for the tests that only need
/// to exercise a constraint mode rather than the full manoeuvre.
fn short_cut_in_left() -> scenario_weaver::dsl::types::ScenarioSpec {
    let mut spec = common::parse_example("cut_in_left.yaml");
    spec.duration = 5.0;
    spec.time_step = 0.5;
    spec
}

// ---------------------------------------------------------------------------
// Adversarial examples: the named constraint must actually end up violated
// ---------------------------------------------------------------------------

#[test]
fn test_adversarial_all_from_file() {
    let scenario = common::generate_example("cut_in_left_adversarial_all.yaml");

    assert_eq!(scenario.scenario_type, "cut_in_left");
    assert!(
        scenario.validation.min_distance.is_some_and(|d| d < 5.0),
        "violate_all should drive min_distance below the 5.0 m threshold, got {:?}",
        scenario.validation.min_distance
    );
    assert!(
        !scenario.validation.all_constraints_satisfied,
        "violate_all should report the scenario as violating its constraints"
    );
    assert!(
        !scenario.validation.safety_violations.is_empty(),
        "violate_all should enumerate the violations it produced"
    );
}

#[test]
fn test_adversarial_ttc_only_from_file() {
    let scenario = common::generate_example("cut_in_left_adversarial_ttc.yaml");

    assert_eq!(scenario.scenario_type, "cut_in_left");
    assert!(
        scenario.validation.min_ttc.is_some_and(|ttc| ttc < 3.0),
        "TTC should be violated (< 3.0), got {:?}",
        scenario.validation.min_ttc
    );
}

#[test]
fn test_speed_limit_violation() {
    let scenario = common::generate_example("speed_limit_violation.yaml");

    assert_eq!(scenario.scenario_type, "cut_in_left");
    let max_speed: f64 = scenario
        .actors
        .iter()
        .flat_map(|a| a.states.iter().map(|s| s.velocity().vx.abs()))
        .fold(0.0_f64, f64::max);
    assert!(
        max_speed > 22.0,
        "some actor should exceed the 22 m/s speed limit, got max {max_speed:.4}"
    );
}

#[test]
fn test_unsafe_following() {
    // The example puts `max_relative_velocity: 10.0` in violate mode, so the
    // produced trajectory must actually exceed that difference somewhere.
    let scenario = common::generate_example("unsafe_following.yaml");

    assert_eq!(scenario.scenario_type, "cut_in_left");
    assert_eq!(scenario.actors.len(), 2);

    let max_rel = max_relative_velocity(&scenario, "ego", "npc");
    assert!(
        max_rel > 10.0,
        "violate mode on max_relative_velocity should exceed 10.0 m/s, got {max_rel:.4}"
    );
}

#[test]
fn test_multi_lane_lateral_distance() {
    // The example puts `min_lateral_distance: 1.5` in violate mode, so the two
    // vehicles must actually come closer than that laterally.
    let scenario = common::generate_example("multi_lane_safety.yaml");

    assert_eq!(scenario.scenario_type, "cut_in_left");
    assert_eq!(scenario.actors.len(), 2);

    let min_lat = min_lateral_separation(&scenario, "ego", "npc");
    assert!(
        min_lat < 1.5,
        "violate mode on min_lateral_distance should breach 1.5 m, got {min_lat:.4}"
    );
}

// ---------------------------------------------------------------------------
// Optimizer paths
// ---------------------------------------------------------------------------

/// `cut_in_right` under `MinimizeTtc`, at a horizon its manoeuvre fits in.
///
/// The horizon matters and is not free to shorten: the npc's lane change is
/// scheduled at the midpoint of `start_time: [2.5, 7.5]`, i.e. t = 5.0 s, so
/// 10 s at 0.5 s steps puts the whole window (steps 10..17) inside the
/// trajectory. See `test_cut_in_right_truncated_to_its_start_step_is_rejected_by_validation`
/// for the truncated variant this test used to carry.
#[test]
fn test_optimizer_minimize_ttc_cut_in_right() {
    let mut spec = common::parse_example("cut_in_right.yaml");
    spec.optimization_target = OptimizationTarget::MinimizeTtc;
    spec.time_step = 0.5;

    let scenario = common::generate_spec_or_fail(spec);

    assert_eq!(scenario.scenario_type, "cut_in_right");
    let opt = scenario
        .optimization
        .as_ref()
        .expect("optimizer path must record optimization metadata");
    assert!(opt.target.contains("MinimizeTtc"), "got {}", opt.target);
    let val = opt
        .optimal_value
        .expect("optimizer must report an optimal value");
    // Since SW-14 the objective is a TTC in seconds, not the old
    // `|Δpx| − dt·|Δvx|` metre-scale proxy: a non-negative, finite time.
    assert!(
        val.is_finite() && val >= 0.0 && val < 1000.0,
        "minimised TTC out of range: {val}"
    );
    // `min-ttc` is a certified lower bound (SW-14), so the trajectory the
    // optimiser returned cannot be safer than the optimum it reported.
    let measured = scenario
        .validation
        .min_ttc
        .expect("cut_in_right must report a measured min_ttc");
    assert!(
        val <= measured + 1e-6,
        "optimiser reported {val:.6} s but the trajectory's min_ttc is {measured:.6} s"
    );
}

/// Truncating `cut_in_right` to 5 s puts the npc's lane change at step 10 of a
/// 10-step horizon — it would begin at the final state and so span no simulated
/// step at all. The cut-in can therefore not happen.
///
/// This spec used to reach the solver and produce a model, and a wrong one
/// (SW-25): the transition encoder discarded the window while
/// `encode_lane_coupling_with_lane_changes` skipped it as a lane change, so
/// step 10 was left with *no* lateral constraint — the npc sat at `py = 5.00`
/// (inside lane 1) with `lane = 0`, and `MinimizeTtc` chose that lane
/// precisely because a free `lane` let it claim to be sharing the ego's.
///
/// SW-25's fix made the encoding total, so this spec then had to prove
/// infeasible through the solver instead (`assert_infeasible`). SW-29 wires
/// `ScenarioSpec::validate` into the generation path, and this is exactly the
/// spec that check exists to catch: it now names the field before ever
/// reaching the solver, which is why this asserts a validation rejection
/// (`assert_invalid_spec`) rather than a solver UNSAT — a clearer error for
/// the same "no model" fact. `optimization_target` is set anyway to keep this
/// as close as possible to the optimizer test above it; validation rejects
/// before the optimizer ever runs, so it has no bearing on the outcome here.
#[test]
fn test_cut_in_right_truncated_to_its_start_step_is_rejected_by_validation() {
    let mut spec = common::parse_example("cut_in_right.yaml");
    spec.optimization_target = OptimizationTarget::MinimizeTtc;
    spec.duration = 5.0;
    spec.time_step = 0.5;

    common::assert_invalid_spec(
        spec,
        "lane change starts at step 10 (t = 5.000 s), at or past the scenario horizon",
    );
}

#[test]
fn test_optimizer_minimize_distance_overtake() {
    // The full 12 s horizon is required: overtake_left's second lane change
    // starts at t ∈ [7.0, 8.0], so a shortened horizon is genuinely infeasible
    // (see test_overtake_left_is_rejected_by_validation_below_its_manoeuvre_horizon).
    let mut spec = common::parse_example("overtake_left.yaml");
    spec.optimization_target = OptimizationTarget::MinimizeDistance;

    let scenario = common::generate_spec_or_fail(spec);

    assert_eq!(scenario.scenario_type, "overtake_left");
    let opt = scenario
        .optimization
        .as_ref()
        .expect("optimizer path must record optimization metadata");
    assert!(
        opt.target.contains("MinimizeDistance"),
        "got {}",
        opt.target
    );
    let val = opt
        .optimal_value
        .expect("optimizer must report an optimal value");
    assert!(
        val >= 0.0 && val.is_finite(),
        "minimised distance must be a finite non-negative length, got {val}"
    );
    // The objective is the minimum inter-actor distance, so it must not exceed
    // the distance the validator measured on the very trajectory it chose.
    let measured_min_distance = scenario
        .validation
        .min_distance
        .expect("min_distance must be measured to compare it against the objective");
    assert!(
        val <= measured_min_distance + 1e-6,
        "optimiser reported {val:.6} but the trajectory's min_distance is {measured_min_distance:.6}"
    );
}

/// Truncating `overtake_left` to 5 s removes the window its second lane change
/// needs (`start_time: [7.0, 8.0]`), so no model can exist. This used to be
/// swallowed by an `Err(_) => println!` arm, then (before SW-29) proved
/// infeasible only by asking the solver.
///
/// SW-29 wires `ScenarioSpec::validate` into the generation path, and a lane
/// change scheduled past the horizon is exactly what that check rejects, so
/// this spec is now invalid before it ever reaches the solver. What this test
/// was actually proving — that the second lane change's window doesn't fit —
/// still holds: the rejection message names the same fact (step 15 at 7.5 s
/// is at or past the 10-step / 5 s horizon), just earlier and more precisely
/// than a solver UNSAT would have.
#[test]
fn test_overtake_left_is_rejected_by_validation_below_its_manoeuvre_horizon() {
    let mut spec = common::parse_example("overtake_left.yaml");
    spec.duration = 5.0;
    spec.time_step = 0.5;

    common::assert_invalid_spec(
        spec,
        "lane change starts at step 15 (t = 7.500 s), at or past the scenario horizon",
    );
}

// ---------------------------------------------------------------------------
// enforce / violate / ignore
// ---------------------------------------------------------------------------

#[test]
fn test_ignore_mode_generates_with_fewer_constraints() {
    let mut spec = short_cut_in_left();
    spec.constraint_modes = ConstraintModes::Detailed {
        min_ttc: ConstraintMode::Ignore,
        min_distance: ConstraintMode::Enforce,
        max_acceleration: ConstraintMode::Enforce,
        max_velocity: ConstraintMode::Enforce,
        min_velocity: ConstraintMode::Ignore,
        min_lateral_distance: ConstraintMode::Ignore,
        max_relative_velocity: ConstraintMode::Ignore,
    };

    let scenario = common::generate_spec_or_fail(spec);

    assert_eq!(scenario.scenario_type, "cut_in_left");
    assert_eq!(scenario.actors.len(), 2);
    // min_distance is still enforced, so it must hold even in ignore-TTC mode.
    assert!(
        scenario.validation.min_distance.is_some_and(|d| d >= 5.0),
        "min_distance stayed in enforce mode, got {:?}",
        scenario.validation.min_distance
    );
}

/// `Violate` must *negate* the constraint, not satisfy it at the boundary.
///
/// Observed with `min_distance: violate` against a 5.0 m threshold:
/// `min_distance = 5.0000` — exactly on the boundary, i.e. the constraint still
/// holds. The old assertion used `<=`, which accepted that. The correct
/// assertion is a strict `<`.
///
/// Fixed by SW-12. `DistanceGT` lowered to a *strict* `|dx| > d`, so the
/// negation `Violate` asserts is `|dx| <= d` — satisfied by `d` exactly. The
/// validator, meanwhile, calls `distance < min_distance` a breach, so the
/// solver's answer was a distance the validator reported as *safe* for a
/// constraint the spec asked to have violated. The lowering is non-strict now
/// (`|dx| >= d`, which is the validator's own definition of safe), and the
/// negation is therefore `|dx| < d`, strictly.
#[test]
fn test_violate_mode_negates_constraint() {
    let mut spec = short_cut_in_left();
    let threshold = spec.min_distance;
    spec.constraint_modes = ConstraintModes::Detailed {
        min_ttc: ConstraintMode::Enforce,
        min_distance: ConstraintMode::Violate,
        max_acceleration: ConstraintMode::Enforce,
        max_velocity: ConstraintMode::Enforce,
        min_velocity: ConstraintMode::Ignore,
        min_lateral_distance: ConstraintMode::Ignore,
        max_relative_velocity: ConstraintMode::Ignore,
    };

    let scenario = common::generate_spec_or_fail(spec);

    println!(
        "SW-12 violate mode: min_distance = {:?} against threshold {threshold:.2}",
        scenario.validation.min_distance
    );
    assert!(
        scenario
            .validation
            .min_distance
            .is_some_and(|d| d < threshold),
        "violate mode must strictly negate min_distance (< {threshold:.2}), got {:?} \
         — equality means the constraint was satisfied, not violated",
        scenario.validation.min_distance
    );
}

/// `Enforce` must hold against a value that was actually measured.
///
/// The metrics are `Option<f64>`: `None` means "never evaluated". The `expect`s
/// below are the guard that used to be `< 999.0` — if a metric is not computed,
/// this test fails, rather than passing vacuously on a sentinel that satisfies
/// any `>=` threshold.
///
/// The fixture is `head_on_near_miss.yaml`: it declares both modes as
/// `enforce` and both metrics are measured (TTC 4.85 s against a 2.0 s
/// threshold, distance 23.31 m against 5.0 m).
///
/// It is the *fourth* fixture this test has had to move to, and — SW-22 — it
/// should be the last. `simple_bidirectional.yaml` stopped producing a measured
/// TTC when SW-12 added the forward-progress bound and Z3 moved to a different,
/// equally valid model — as, in the same change, did `cut_in_right.yaml`, which
/// was briefly its replacement; that had replaced
/// `speed_limit_violation.yaml`, which stopped when SW-10 reconciled the
/// encoder's and the validator's same-lane predicates; that had replaced
/// `unsafe_following.yaml`, which stopped when SW-09 chained `vy` to `ay`; and
/// that had replaced `overtake_with_opposite.yaml`, which stopped when SW-08
/// corrected the lane centres and the position integration. Every time, the
/// mechanism was the same: nothing in the encoding *required* the two actors to
/// be closing on each other, so whether a TTC existed to measure was decided by
/// which satisfying model Z3 happened to return.
///
/// **A conflict is now required rather than hoped for.** Every one of those five
/// rotated-away fixtures is a `cut_in_left` or `cut_in_right` spec, and
/// `scenarios::cut_in_conflict` (SW-22) makes the NPC merge in front of an ego
/// that is gaining — an implication guarded by a lane membership the template
/// already forces, not the `F(some pair is closing)` disjunction SW-12 measured
/// at over 500 s on `cut_in_left`. All five report a measured TTC again:
/// `simple_bidirectional` 3.00 s, `speed_limit_violation` 3.00 s,
/// `unsafe_following` 3.00 s, `overtake_with_opposite` 3.00 s, `cut_in_right`
/// 3.00 s (the smallest of its five scenarios). The corpus-wide form of this
/// test, `examples_smoke_test::test_enforce_min_ttc_examples_produce_a_measured_ttc`,
/// is no longer `#[ignore]`d and now covers all of them at once, so the next
/// fixture rotation would be a test failure rather than a quiet edit here.
///
/// `head_on_near_miss.yaml` stays as the fixture for the one thing that has
/// survived every previous rotation: its closing pair is **structural**. The ego
/// and the oncoming actor travel in opposite directions down the same road, so
/// they approach each other in every model there is — no encoding choice can
/// make that pair stop closing without making the example unsolvable. Its
/// margins are comfortable too (TTC 4.85 s against 2.0 s, distance 23.31 m
/// against 5.0 m), where several of the alternatives sit on their thresholds
/// exactly and are one rounding step from a spurious failure.
#[test]
fn test_enforce_mode_respects_constraint() {
    // Declares `min_ttc: enforce` and `min_distance: enforce`.
    let spec = common::parse_example("head_on_near_miss.yaml");
    assert_eq!(spec.constraint_modes.min_ttc(), ConstraintMode::Enforce);
    assert_eq!(
        spec.constraint_modes.min_distance(),
        ConstraintMode::Enforce
    );
    let min_ttc_threshold = spec.min_ttc;
    let min_dist_threshold = spec.min_distance;

    let scenario = common::generate_spec_or_fail(spec);

    let min_ttc = scenario
        .validation
        .min_ttc
        .expect("min_ttc must be a measured value, not left unevaluated");
    assert!(
        min_ttc >= min_ttc_threshold,
        "enforced min_ttc should be >= {min_ttc_threshold:.1}, got {min_ttc:.4}"
    );

    let min_distance = scenario
        .validation
        .min_distance
        .expect("min_distance must be a measured value, not left unevaluated");
    assert!(
        min_distance >= min_dist_threshold,
        "enforced min_distance should be >= {min_dist_threshold:.1}, got {min_distance:.4}"
    );
}

// ---------------------------------------------------------------------------
// min_distance is lane-guarded (SW-10 / H7)
// ---------------------------------------------------------------------------

/// `min_distance` applies to actors sharing a lane, not to every pair on the
/// road.
///
/// The two actors here start side by side — both at `x` in `[0, 5]`, so
/// `|px_ego - px_npc| <= 5` at t=0, far inside the declared 40 m threshold —
/// but in *different* lanes, and the npc only merges into the ego's lane at
/// t=14 s, by which time it is far ahead.
///
/// Before SW-10, `Proposition::DistanceGT` lowered to a bare
/// `|px1 - px2| > d` with no lane guard, so two cars in different lanes still
/// had to be 40 m apart longitudinally and this spec was UNSAT (verified
/// against the pre-SW-10 binary). `compute_validation_metrics` meanwhile has
/// always gated `min_distance` on the pair sharing a lane, so the encoder was
/// enforcing a constraint the validator would never have checked, and
/// over-constraining every multi-lane spec in the process.
///
/// Both sides now use `encoder_utils::encode_same_lane_constraint`.
#[test]
fn test_min_distance_is_lane_guarded() {
    let yaml = r#"
    scenario_type: cut_in_left
    time_step: 0.5
    duration: 20.0
    num_scenarios: 1

    actors:
      - id: ego
        role: ego
        lane: 1
        position: [0.0, 5.0]
        speed: [15.0, 16.0]
        direction: 1
        acceleration: [-0.5, 0.5]

      - id: npc
        role: npc
        lane: 0
        position: [0.0, 5.0]
        speed: [20.0, 22.0]
        direction: 1
        acceleration: [-0.5, 0.5]
        lane_changes:
          - direction: right
            start_time: [14.0, 14.0]
            duration: [2.0, 2.0]

    road:
      num_lanes: 2
      lane_width: 3.5
      lane_directions: [1, 1]

    min_ttc: 3.0
    min_distance: 40.0

    constraint_modes:
      min_ttc: enforce
      min_distance: enforce

    max_lateral_acceleration: 3.0"#;

    let (scenario, spec) = common::generate_yaml_with_spec(yaml);

    // The pair really is closer than min_distance while in different lanes:
    // the guard is doing work, not passing vacuously.
    let ego = scenario.get_actor("ego").expect("ego");
    let npc = scenario.get_actor("npc").expect("npc");
    let gap_at_start = (ego.states[0].position().x - npc.states[0].position().x).abs();
    assert!(
        gap_at_start < spec.min_distance,
        "the fixture is meant to start the pair inside min_distance ({} m) in \
         different lanes; measured {gap_at_start:.4} m",
        spec.min_distance
    );
    assert_ne!(
        ego.states[0].lane(),
        npc.states[0].lane(),
        "the fixture is meant to start the pair in different lanes"
    );

    // And the metric the validator reports still honours the threshold, because
    // it is measured over the same-lane steps only.
    let measured = scenario
        .validation
        .min_distance
        .expect("min_distance must be measured: the pair does share a lane after the merge");
    assert!(
        measured >= spec.min_distance,
        "same-lane min_distance should be >= {:.1}, got {measured:.4}",
        spec.min_distance
    );
}
