//! Integration tests for the optimization (--optimize) code path.
//!
//! Tests verify that optimization targets (min-ttc, min-distance, max-ttc, min-severity)
//! produce valid scenarios with optimization metadata, and that each target's
//! objective is the quantity the validator then reports (SW-14).

mod common;

use scenario_weaver::dsl::types::{OptimizationTarget, ScenarioSpec};
use scenario_weaver::scenario::model::Scenario;

/// Helper to create a spec from the cut_in_left example and set an optimization target.
fn create_optimized_spec(target: OptimizationTarget) -> scenario_weaver::dsl::types::ScenarioSpec {
    let mut spec = common::parse_example("cut_in_left.yaml");
    spec.optimization_target = target;
    // Use a shorter duration for faster tests
    spec.duration = 5.0;
    spec.time_step = 0.5;
    spec
}

#[test]
fn test_optimize_minimize_ttc() {
    let spec = create_optimized_spec(OptimizationTarget::MinimizeTtc);
    let scenario = common::generate_spec_or_fail(spec);

    // Verify optimization metadata is present
    assert!(
        scenario.optimization.is_some(),
        "Should have optimization info"
    );
    let opt = scenario.optimization.as_ref().unwrap();
    assert!(
        opt.target.contains("MinimizeTtc"),
        "Target should be MinimizeTtc, got: {}",
        opt.target
    );
    assert!(opt.optimal_value.is_some(), "Should have optimal value");

    // Since SW-14 the optimal value is a time-to-collision in seconds, read off
    // the `TTC_LEVELS` ladder, and is a certified *upper* bound on the min-TTC of
    // the trajectory shipped with it.
    let val = opt.optimal_value.unwrap();
    assert!(val > 0.0, "TTC should be a positive duration, got: {}", val);
    assert!(
        val <= 60.0,
        "TTC should not exceed the ladder top, got: {}",
        val
    );

    println!("MinimizeTtc: optimal_value = {:.2}s (min TTC)", val);
    println!("  Scenario min_ttc: {:?}", scenario.validation.min_ttc);
}

#[test]
fn test_optimize_minimize_distance() {
    let spec = create_optimized_spec(OptimizationTarget::MinimizeDistance);
    let scenario = common::generate_spec_or_fail(spec);

    assert!(
        scenario.optimization.is_some(),
        "Should have optimization info"
    );
    let opt = scenario.optimization.as_ref().unwrap();
    assert!(
        opt.target.contains("MinimizeDistance"),
        "Target should be MinimizeDistance, got: {}",
        opt.target
    );
    assert!(opt.optimal_value.is_some(), "Should have optimal value");

    let val = opt.optimal_value.unwrap();
    assert!(
        val >= 0.0,
        "Optimal distance should be non-negative, got: {}",
        val
    );
    assert!(
        val < 1000.0,
        "Optimal distance should be finite, got: {}",
        val
    );

    println!("MinimizeDistance: optimal_value = {:.2}m", val);
    println!(
        "  Scenario min_distance: {:?}",
        scenario.validation.min_distance
    );
}

#[test]
fn test_optimize_maximize_ttc() {
    let spec = create_optimized_spec(OptimizationTarget::MaximizeTtc);
    let scenario = common::generate_spec_or_fail(spec);

    let opt = scenario
        .optimization
        .as_ref()
        .expect("Should have optimization info");
    assert!(
        opt.target.contains("MaximizeTtc"),
        "Target should be MaximizeTtc, got: {}",
        opt.target
    );
    let val = opt
        .optimal_value
        .expect("MaximizeTtc should report an optimal value");
    assert!(val.is_finite(), "optimal value should be finite, got {val}");

    assert!(!scenario.actors.is_empty(), "Should have actors");
    assert_eq!(scenario.scenario_type, "cut_in_left");
}

#[test]
fn test_optimize_minimize_severity() {
    let spec = create_optimized_spec(OptimizationTarget::MinimizeSeverity);
    let scenario = common::generate_spec_or_fail(spec);

    let opt = scenario
        .optimization
        .as_ref()
        .expect("Should have optimization info");
    assert!(
        opt.target.contains("MinimizeSeverity"),
        "Target should be MinimizeSeverity, got: {}",
        opt.target
    );
    let val = opt
        .optimal_value
        .expect("MinimizeSeverity should report an optimal value");
    assert!(val.is_finite(), "optimal value should be finite, got {val}");
}

#[test]
fn test_optimize_none_via_normal_path() {
    // OptimizationTarget::None should use the normal solver path (not optimizer)
    let mut spec = common::parse_example("cut_in_left.yaml");
    spec.optimization_target = OptimizationTarget::None;
    spec.duration = 5.0;
    spec.time_step = 0.5;

    let scenario = common::generate_spec_or_fail(spec);

    // Normal path should NOT have optimization info
    assert!(
        scenario.optimization.is_none(),
        "Normal path should not have optimization info"
    );
    assert_eq!(scenario.scenario_type, "cut_in_left");
    assert!(!scenario.actors.is_empty());
}

#[test]
fn test_optimized_scenario_exports_correctly() {
    // Verify that optimized scenarios can be exported to all formats
    let spec = create_optimized_spec(OptimizationTarget::MinimizeTtc);
    let scenario = common::generate_spec_or_fail(spec);

    // JSON serialization should include optimization field
    let json = serde_json::to_string_pretty(&scenario).expect("Should serialize to JSON");
    assert!(
        json.contains("optimization"),
        "JSON should contain optimization field"
    );
    assert!(
        json.contains("MinimizeTtc"),
        "JSON should contain target name"
    );

    // SVG export should work
    let svg = scenario_weaver::export_scenario_to_svg(&scenario).expect("Should export to SVG");
    assert!(!svg.is_empty());

    // XOSC export should work
    let xosc = scenario_weaver::export_scenario_to_xosc(&scenario).expect("Should export to XOSC");
    assert!(xosc.contains("OpenSCENARIO"));

    println!("All exports succeeded for optimized scenario");
}

#[test]
fn test_objectives_produce_distinct_results() {
    use std::collections::HashSet;

    let targets = [
        OptimizationTarget::MinimizeDistance,
        OptimizationTarget::MinimizeTtc,
        OptimizationTarget::MinimizeSeverity,
    ];

    let mut values: Vec<f64> = Vec::new();
    for &target in &targets {
        let spec = create_optimized_spec(target);
        let scenario = common::generate_spec_or_fail(spec);
        let opt = scenario
            .optimization
            .as_ref()
            .unwrap_or_else(|| panic!("{target:?} should record optimization info"));
        values.push(
            opt.optimal_value
                .unwrap_or_else(|| panic!("{target:?} should report an optimal value")),
        );
    }

    assert_eq!(
        values.len(),
        targets.len(),
        "every objective must produce a result"
    );

    let unique: HashSet<String> = values.iter().map(|v| format!("{v:.4}")).collect();
    assert!(
        unique.len() >= 2,
        "Objectives should produce distinct optimal values, got: {values:?}"
    );
}

#[test]
fn test_optimizer_pedestrian_crossing() {
    let mut spec = common::parse_example("pedestrian_crossing.yaml");
    spec.optimization_target = OptimizationTarget::MinimizeDistance;
    spec.duration = 5.0;
    spec.time_step = 0.5;

    let scenario = common::generate_spec_or_fail(spec);

    let opt = scenario
        .optimization
        .as_ref()
        .expect("Should have optimization info");
    let val = opt
        .optimal_value
        .expect("MinimizeDistance should report an optimal value");
    assert!(
        val >= 0.0 && val.is_finite(),
        "minimised distance must be a finite non-negative length, got {val}"
    );

    let ped = scenario
        .actors
        .iter()
        .find(|a| a.id == "pedestrian")
        .expect("Should have pedestrian actor");
    for state in &ped.states {
        assert_eq!(
            state.lane(),
            0,
            "Pedestrian lane should be fixed to 0 by scenario constraints"
        );
    }
}

/// A single-step horizon leaves no room for `cut_in_left`'s lane change
/// (`start_time: [2.5, 7.5]`), so no model can exist. Committed as an
/// expectation rather than an "acceptable either way" branch.
#[test]
fn test_optimizer_minimal_horizon_is_infeasible() {
    let mut spec = common::parse_example("cut_in_left.yaml");
    spec.optimization_target = OptimizationTarget::MinimizeDistance;
    spec.duration = 0.5;
    spec.time_step = 0.5;

    common::assert_infeasible(
        spec,
        "a 0.5 s horizon cannot contain the lane change the scenario requires",
    );
}

// ---------------------------------------------------------------------------
// The optimality property (SW-06 / finding M16)
// ---------------------------------------------------------------------------

/// The objective values `GenericEncoder::encode_objective` optimises, recomputed
/// from an extracted `Scenario`.
///
/// Mirrored from the objective region of `src/solver/encoder.rs`:
/// `compute_effective_dist`, `compute_effective_closing_speed`,
/// `collect_directed_conflicts`, `TTC_LEVELS` and the `9999` sentinel returned
/// for a pair that is not in the same lane. **If an objective's definition
/// changes, this module must move with it** — it is the only check that the
/// reported `optimal_value` matches the shipped trajectory.
mod objective {
    use super::{OptimizationTarget, Scenario};

    /// The `big_val` the encoder substitutes when two actors are not in the
    /// same lane, so an out-of-lane pair cannot look like the closest approach.
    const BIG: f64 = 9999.0;

    /// `src/solver/encoder.rs::TTC_LEVELS` — the ladder of constant TTC levels the
    /// two TTC objectives are expressed over.
    const TTC_LEVELS: [f64; 21] = [
        0.25, 0.5, 0.75, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 5.0, 6.0, 8.0, 10.0, 12.5, 15.0, 20.0,
        25.0, 30.0, 40.0, 60.0,
    ];

    /// `src/solver/encoder.rs::TTC_CLOSING_SPEED_EPSILON`, which itself mirrors the
    /// `epsilon` in `compute_validation_metrics`.
    const CLOSING_SPEED_EPSILON: f64 = 0.01;

    /// Slack for the rational→`f64` rounding described by `Encoder::METRIC_TOL`.
    ///
    /// The encoder asserts `gap ≤ T · closing` over exact rationals; recovering the
    /// TTC from the extracted doubles by *dividing* does not round back to the same
    /// number, so an exactly-satisfying solution recomputes as `3.0000000000000004`.
    /// Without this slack the ladder lookup would jump a whole level.
    const LADDER_TOL: f64 = 1e-6;

    /// `src/solver/encoder_utils.rs::encode_same_lane_constraint`: a discrete lane
    /// match **or** lateral overlap within one lane width. This is also the predicate
    /// `compute_validation_metrics` uses, which is the point — before SW-14 the
    /// objectives used the discrete half alone and scored a different set of states
    /// than the tool reported on.
    fn same_lane(scenario: &Scenario, a: usize, b: usize, t: usize) -> bool {
        let (s1, s2) = (&scenario.actors[a].states[t], &scenario.actors[b].states[t]);
        s1.lane() == s2.lane()
            || (s1.position().y - s2.position().y).abs() < scenario.road.lane_width
    }

    /// Same-lane longitudinal gap at one step, or `BIG`.
    fn effective_dist(scenario: &Scenario, a: usize, b: usize, t: usize) -> f64 {
        if same_lane(scenario, a, b, t) {
            let (s1, s2) = (&scenario.actors[a].states[t], &scenario.actors[b].states[t]);
            (s1.position().x - s2.position().x).abs()
        } else {
            BIG
        }
    }

    /// `|vx_a - vx_b|` when same-lane, else 0 — the encoder's severity term.
    fn effective_closing_speed(scenario: &Scenario, a: usize, b: usize, t: usize) -> f64 {
        if same_lane(scenario, a, b, t) {
            let (s1, s2) = (&scenario.actors[a].states[t], &scenario.actors[b].states[t]);
            (s1.velocity().vx - s2.velocity().vx).abs()
        } else {
            0.0
        }
    }

    /// The smallest TTC over every directed conflict in the scenario, or `INFINITY`
    /// when no pair is ever same-lane, ordered and closing — the encoder's
    /// `collect_directed_conflicts`, and the validator's own TTC block.
    pub fn min_ttc(scenario: &Scenario) -> f64 {
        let n = scenario.actors.len();
        let mut best = f64::INFINITY;
        for a in 0..n {
            for b in (a + 1)..n {
                let steps = scenario.actors[a]
                    .states
                    .len()
                    .min(scenario.actors[b].states.len());
                for t in 0..steps {
                    if !same_lane(scenario, a, b, t) {
                        continue;
                    }
                    for (follow, lead) in [(a, b), (b, a)] {
                        let sf = &scenario.actors[follow].states[t];
                        let sl = &scenario.actors[lead].states[t];
                        let gap = sl.position().x - sf.position().x;
                        let closing = sf.velocity().vx - sl.velocity().vx;
                        if gap > 0.0 && closing >= CLOSING_SPEED_EPSILON {
                            best = best.min(gap / closing);
                        }
                    }
                }
            }
        }
        best
    }

    /// The lowest ladder level that `ttc` does not exceed — what
    /// `encode_minimize_ttc_objective` reports (`obj = T_m`, the smallest `T_m` with a
    /// step at or below it), saturating at the top of the ladder.
    pub fn ladder_ceiling(ttc: f64) -> f64 {
        let top = TTC_LEVELS[TTC_LEVELS.len() - 1];
        TTC_LEVELS
            .iter()
            .copied()
            .find(|&level| ttc <= level + LADDER_TOL)
            .unwrap_or(top)
    }

    /// The highest ladder level that `ttc` clears — what
    /// `encode_maximize_ttc_objective` reports (`obj = T_M`, the largest `T_M` every
    /// step clears), and 0 when even the lowest level is breached.
    pub fn ladder_floor(ttc: f64) -> f64 {
        TTC_LEVELS
            .iter()
            .copied()
            .filter(|&level| ttc >= level - LADDER_TOL)
            .next_back()
            .unwrap_or(0.0)
    }

    fn fold_pairs(
        scenario: &Scenario,
        init: f64,
        term: fn(&Scenario, usize, usize, usize) -> f64,
        combine: fn(f64, f64) -> f64,
    ) -> f64 {
        let n = scenario.actors.len();
        let mut acc = init;
        for a in 0..n {
            for b in (a + 1)..n {
                let steps = scenario.actors[a]
                    .states
                    .len()
                    .min(scenario.actors[b].states.len());
                for t in 0..steps {
                    acc = combine(acc, term(scenario, a, b, t));
                }
            }
        }
        acc
    }

    /// The value `target` is trying to move, computed from a finished scenario.
    pub fn value(scenario: &Scenario, target: OptimizationTarget) -> f64 {
        match target {
            OptimizationTarget::MinimizeDistance => {
                fold_pairs(scenario, f64::INFINITY, effective_dist, f64::min)
            }
            OptimizationTarget::MinimizeTtc => ladder_ceiling(min_ttc(scenario)),
            OptimizationTarget::MaximizeTtc => ladder_floor(min_ttc(scenario)),
            OptimizationTarget::MinimizeSeverity => fold_pairs(
                scenario,
                f64::NEG_INFINITY,
                effective_closing_speed,
                f64::max,
            ),
            OptimizationTarget::None => f64::NAN,
        }
    }

    /// True when `target` pushes its objective up rather than down.
    ///
    /// `MinimizeSeverity` is a maximiser despite its name: severity correlates
    /// with impact speed, so the encoder maximises the highest same-lane closing
    /// speed. Renaming the variant is SW-14's recommendation but reaches
    /// `src/lib.rs`, outside that issue's edit set.
    pub fn is_maximiser(target: OptimizationTarget) -> bool {
        matches!(
            target,
            OptimizationTarget::MaximizeTtc | OptimizationTarget::MinimizeSeverity
        )
    }
}

/// All four optimizer targets.
const ALL_TARGETS: [OptimizationTarget; 4] = [
    OptimizationTarget::MinimizeTtc,
    OptimizationTarget::MinimizeDistance,
    OptimizationTarget::MinimizeSeverity,
    OptimizationTarget::MaximizeTtc,
];

/// The targets whose optimality property is asserted — all four.
///
/// `MinimizeDistance` used to be split out into an `#[ignore]`d test against
/// SW-14, on a measurement taken on the SW-08 branch (optimised 123.45 against
/// a plain-solver 45.83, i.e. `Optimize` returning a model it had not proved
/// optimal). Re-measured at `dfb58a1` that no longer reproduces — both paths
/// land on 5.000000 — so the entry is retired rather than suppressed, and the
/// property is asserted for every target.
const OPTIMAL_TARGETS: [OptimizationTarget; 4] = ALL_TARGETS;

/// The optimality check for one target, as a list of failure descriptions.
fn optimality_failures(base: &ScenarioSpec, targets: &[OptimizationTarget]) -> Vec<String> {
    let mut failures: Vec<String> = Vec::new();
    for target in targets.iter().copied() {
        let (optimized, plain) = optimality_gap(base, target);
        let maximiser = objective::is_maximiser(target);
        let ok = if maximiser {
            optimized >= plain - common::TOL
        } else {
            optimized <= plain + common::TOL
        };

        println!(
            "{target:?} ({}): optimized={optimized:.6}, unoptimized={plain:.6}, delta={:+.6}",
            if maximiser { "maximise" } else { "minimise" },
            optimized - plain
        );

        if !ok {
            failures.push(format!(
                "{target:?} is a {} but optimising made its objective worse: \
                 optimized={optimized:.9}, unoptimized={plain:.9}",
                if maximiser { "maximiser" } else { "minimiser" }
            ));
        }
    }
    failures
}

/// Run one spec twice — once through the Optimize backend, once through the
/// plain Solver — and compare the objective the target claims to move.
///
/// The baseline matters: `OptimizationTarget::None` takes an entirely different
/// code path (`generate_with_solver`), so this compares the optimizer against
/// the thing it is supposed to improve on, not against itself.
fn optimality_gap(base: &ScenarioSpec, target: OptimizationTarget) -> (f64, f64) {
    let mut optimized_spec = base.clone();
    optimized_spec.optimization_target = target;
    let optimized = common::generate_spec_or_fail(optimized_spec);

    let mut plain_spec = base.clone();
    plain_spec.optimization_target = OptimizationTarget::None;
    let unoptimized = common::generate_spec_or_fail(plain_spec);

    (
        objective::value(&optimized, target),
        objective::value(&unoptimized, target),
    )
}

/// The optimizer's defining property: optimising must not produce a worse
/// objective than not optimising.
///
/// Nothing asserted this before. `test_optimize_minimize_ttc` checked
/// `val > -1000.0 && val < 1000.0` — a finiteness check standing in for an
/// optimality check, which an optimizer that returned the first satisfying
/// model it found would pass. Two solver calls and one comparison per target is
/// the whole of what actually validates this code path.
///
/// The baseline matters: `OptimizationTarget::None` takes an entirely different
/// code path (`generate_with_solver`), so this compares the optimizer against
/// the thing it is supposed to improve on, not against itself. Both paths run
/// the same `encode_*` sequence in `lib.rs`, so the two runs share a constraint
/// set and the property must hold however tight that set is.
///
/// All four targets hold since SW-14.
#[test]
fn test_optimization_never_worsens_its_own_objective() {
    let base = create_optimized_spec(OptimizationTarget::None);
    let failures = optimality_failures(&base, &OPTIMAL_TARGETS);

    assert!(
        failures.is_empty(),
        "{} of {} optimizer targets failed the optimality property:\n  {}",
        failures.len(),
        OPTIMAL_TARGETS.len(),
        failures.join("\n  ")
    );
}

/// Every target must report an optimal value, and it must be the value of the
/// objective in the scenario that was actually extracted.
///
/// A reported `optimal_value` that does not match the trajectory shipped
/// alongside it is worse than none: it is the number a consumer would trust.
#[test]
fn test_reported_optimal_value_matches_the_extracted_scenario() {
    let mut failures: Vec<String> = Vec::new();

    for target in ALL_TARGETS {
        let spec = create_optimized_spec(target);
        let scenario = common::generate_spec_or_fail(spec);
        let opt = scenario
            .optimization
            .as_ref()
            .unwrap_or_else(|| panic!("{target:?} should record optimization info"));
        let reported = opt
            .optimal_value
            .unwrap_or_else(|| panic!("{target:?} should report an optimal value"));
        let recomputed = objective::value(&scenario, target);

        println!("{target:?}: reported={reported:.6}, from trajectories={recomputed:.6}");
        if (reported - recomputed).abs() > common::TOL {
            failures.push(format!(
                "{target:?}: optimization.optimal_value = {reported:.9} but the extracted \
                 trajectories give {recomputed:.9}"
            ));
        }
    }

    assert!(
        failures.is_empty(),
        "{} of {} targets report an optimal value that the extracted scenario does not \
         support:\n  {}",
        failures.len(),
        ALL_TARGETS.len(),
        failures.join("\n  ")
    );
}

// ---------------------------------------------------------------------------
// SW-14 — the objective must be the quantity the tool reports
// ---------------------------------------------------------------------------

/// Every `optimization_target` value must parse **from YAML**.
///
/// `docs/optimizer.md:20` shipped `optimization_target: min-distance` — the *CLI*
/// spelling in a *YAML* snippet — for as long as it did precisely because nothing
/// exercised this field through the parser. `OptimizationTarget` is
/// `#[serde(rename_all = "snake_case")]`, so the CLI's kebab-case spellings are not
/// valid YAML; this pins both halves, the five that must parse and the four that
/// must not.
#[test]
fn test_all_optimization_target_values_parse_from_yaml() {
    let base = std::fs::read_to_string("examples/cut_in_left.yaml")
        .expect("examples/cut_in_left.yaml should be readable");

    let expected = [
        ("none", OptimizationTarget::None),
        ("minimize_ttc", OptimizationTarget::MinimizeTtc),
        ("minimize_distance", OptimizationTarget::MinimizeDistance),
        ("minimize_severity", OptimizationTarget::MinimizeSeverity),
        ("maximize_ttc", OptimizationTarget::MaximizeTtc),
    ];

    for (yaml_value, want) in expected {
        let yaml = format!("{base}\noptimization_target: {yaml_value}\n");
        let spec = scenario_weaver::dsl::parser::parse_yaml(&yaml)
            .unwrap_or_else(|e| panic!("`optimization_target: {yaml_value}` must parse: {e}"));
        assert_eq!(
            spec.optimization_target, want,
            "`optimization_target: {yaml_value}` parsed to the wrong variant"
        );
        println!("optimization_target: {yaml_value} -> {want:?}");
    }

    // The CLI spellings are kebab-case and must NOT be accepted as YAML.
    for cli_value in ["min-ttc", "min-distance", "min-severity", "max-ttc"] {
        let yaml = format!("{base}\noptimization_target: {cli_value}\n");
        assert!(
            scenario_weaver::dsl::parser::parse_yaml(&yaml).is_err(),
            "`optimization_target: {cli_value}` is the CLI spelling and must not parse as YAML"
        );
        println!("optimization_target: {cli_value} -> rejected (CLI spelling)");
    }
}

/// The `docs/optimizer.md` YAML snippet, verbatim, must parse.
///
/// The doc used to show `min-distance`; this reads the fence out of the document
/// itself so the two cannot drift apart again.
#[test]
fn test_optimizer_doc_yaml_snippet_parses() {
    let doc = std::fs::read_to_string("docs/optimizer.md").expect("docs/optimizer.md should exist");
    let snippet = doc
        .lines()
        .find(|l| l.trim_start().starts_with("optimization_target:"))
        .expect("docs/optimizer.md should show an `optimization_target:` line");

    let base = std::fs::read_to_string("examples/cut_in_left.yaml")
        .expect("examples/cut_in_left.yaml should be readable");
    let yaml = format!("{base}\n{snippet}\n");

    scenario_weaver::dsl::parser::parse_yaml(&yaml).unwrap_or_else(|e| {
        panic!("the YAML snippet in docs/optimizer.md does not parse: `{snippet}`: {e}")
    });
    println!("docs/optimizer.md snippet parses: `{snippet}`");
}

/// A hand-built two-actor scenario at one step, for scoring objectives directly.
fn two_actor_state(gap: f64, closing: f64) -> Scenario {
    use scenario_weaver::dsl::types::RoadSpec;
    use scenario_weaver::scenario::model::{
        Acceleration, ActorTrajectory, Position, State, Velocity,
    };

    let road = RoadSpec {
        num_lanes: 2,
        lane_width: 3.5,
        lane_directions: vec![1, 1],
        road_length: None,
    };
    let mut scenario = Scenario::new("cut_in_left".to_string(), 0.5, 0.5, road);

    // Both in lane 0; `follow` is at x = 0 closing on `lead` at x = gap.
    let mut follow = ActorTrajectory::new("follow".to_string(), "ego".to_string());
    follow.add_state(State::new(
        0.0,
        Position::new(0.0, 1.75),
        Velocity::new(closing, 0.0),
        Acceleration::new(0.0, 0.0),
        0,
    ));
    let mut lead = ActorTrajectory::new("lead".to_string(), "npc".to_string());
    lead.add_state(State::new(
        0.0,
        Position::new(gap, 1.75),
        Velocity::new(0.0, 0.0),
        Acceleration::new(0.0, 0.0),
        0,
    ));

    scenario.add_actor(follow);
    scenario.add_actor(lead);
    scenario
}

/// The `min-ttc` objective must be monotone in TTC — SW-14/M3's counterexample.
///
/// The old objective was `|Δpx| − dt·|Δvx|`, minimised. With `dt = 0.5` it scored
///
/// | state | true TTC | old proxy |
/// |---|---|---|
/// | `d = 2 m, v = 0` | ∞ | 2.0 |
/// | `d = 50 m, v = 20 m/s` | 2.5 s | 40.0 |
///
/// so *minimising* it preferred the **infinite**-TTC state — it inverted the ranking
/// it claimed to produce. The ladder objective scores the same two states in
/// seconds, and a minimiser must now prefer the 2.5 s one.
#[test]
fn test_minimize_ttc_objective_is_monotone_in_ttc() {
    let stationary = two_actor_state(2.0, 0.0);
    let closing = two_actor_state(50.0, 20.0);

    let stationary_ttc = objective::min_ttc(&stationary);
    let closing_ttc = objective::min_ttc(&closing);
    assert!(
        stationary_ttc.is_infinite(),
        "(d=2, v=0) is not approaching at all, so its TTC is infinite, got {stationary_ttc}"
    );
    assert!(
        (closing_ttc - 2.5).abs() < common::TOL,
        "(d=50, v=20) has a 2.5 s TTC, got {closing_ttc}"
    );

    let stationary_obj = objective::value(&stationary, OptimizationTarget::MinimizeTtc);
    let closing_obj = objective::value(&closing, OptimizationTarget::MinimizeTtc);

    println!(
        "MinimizeTtc objective: (d=2, v=0) -> {stationary_obj}, (d=50, v=20) -> {closing_obj}"
    );
    assert!(
        closing_obj < stationary_obj,
        "a minimiser of TTC must prefer (d=50, v=20) [TTC 2.5 s, objective {closing_obj}] over \
         (d=2, v=0) [TTC infinite, objective {stationary_obj}]"
    );

    // And the maximiser must rank them the other way round.
    let stationary_max = objective::value(&stationary, OptimizationTarget::MaximizeTtc);
    let closing_max = objective::value(&closing, OptimizationTarget::MaximizeTtc);
    println!(
        "MaximizeTtc objective: (d=2, v=0) -> {stationary_max}, (d=50, v=20) -> {closing_max}"
    );
    assert!(
        stationary_max > closing_max,
        "a maximiser of TTC must prefer the infinite-TTC state"
    );
}

/// The reported `optimal_value` must bound the TTC the validator then measures.
///
/// This is the property SW-12 found missing: under `--optimize` the same spec
/// reported `Min TTC: 12.37 s` in cartesian and `44.76 s` in bicycle for `max-ttc`,
/// and `3.00 s` against `32.49 s` for `min-ttc` — whatever the objective was
/// optimising, it was not the quantity the validator measured. Both TTC targets now
/// score the validator's own TTC, so the objective bounds it in a known direction:
///
/// - `MinimizeTtc` asserts a step at or below its level, so `measured ≤ optimal`.
/// - `MaximizeTtc` asserts every step clears its level, so `optimal ≤ measured`.
#[test]
fn test_ttc_objectives_bound_the_reported_ttc() {
    for (target, spec_ttc_is_upper_bound) in [
        (OptimizationTarget::MinimizeTtc, true),
        (OptimizationTarget::MaximizeTtc, false),
    ] {
        let spec = create_optimized_spec(target);
        let scenario = common::generate_spec_or_fail(spec);
        let reported = scenario
            .optimization
            .as_ref()
            .and_then(|o| o.optimal_value)
            .unwrap_or_else(|| panic!("{target:?} should report an optimal value"));
        let measured = objective::min_ttc(&scenario);

        println!(
            "{target:?}: optimal_value={reported:.6}s, min TTC of the trajectory={measured:.6}s"
        );
        if spec_ttc_is_upper_bound {
            assert!(
                measured <= reported + common::TOL,
                "{target:?} claims min TTC ≤ {reported} but the trajectory measures {measured}"
            );
        } else {
            assert!(
                reported <= measured + common::TOL,
                "{target:?} claims min TTC ≥ {reported} but the trajectory measures {measured}"
            );
        }
    }
}
