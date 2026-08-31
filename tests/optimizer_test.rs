//! Integration tests for the optimization (--optimize) code path.
//!
//! Tests verify that optimization targets (min-ttc, min-distance, max-ttc, min-severity)
//! produce valid scenarios with optimization metadata.

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

    // The optimal value is a TTC proxy (distance - dt*closing_speed)
    // It can be negative when closing speed dominates distance
    let val = opt.optimal_value.unwrap();
    assert!(val > -1000.0, "TTC proxy should be finite, got: {}", val);
    assert!(val < 1000.0, "TTC proxy should be finite, got: {}", val);

    println!("MinimizeTtc: optimal_value = {:.2} (TTC proxy)", val);
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
/// Each is a deliberate linear proxy — TTC is `distance / closing_speed`, and
/// division would put the whole encoding in NRA — so the proxy, not TTC itself,
/// is what "optimal" means here. Mirrored from `src/solver/encoder.rs`:
/// `compute_effective_dist`, `compute_ttc_proxy`,
/// `compute_effective_closing_speed`, and the `9999` sentinel they return for a
/// pair that is not in the same lane.
mod objective {
    use super::{OptimizationTarget, Scenario};

    /// The `big_val` the encoder substitutes when two actors are not in the
    /// same lane, so an out-of-lane pair cannot look like the closest approach.
    const BIG: f64 = 9999.0;

    /// Same-lane longitudinal gap at one step, or `BIG`.
    fn effective_dist(scenario: &Scenario, a: usize, b: usize, t: usize) -> f64 {
        let (s1, s2) = (&scenario.actors[a].states[t], &scenario.actors[b].states[t]);
        if s1.lane() == s2.lane() {
            (s1.position().x - s2.position().x).abs()
        } else {
            BIG
        }
    }

    /// `|vx_a - vx_b|` when same-lane, else 0 — the encoder's severity term.
    fn effective_closing_speed(scenario: &Scenario, a: usize, b: usize, t: usize) -> f64 {
        let (s1, s2) = (&scenario.actors[a].states[t], &scenario.actors[b].states[t]);
        if s1.lane() == s2.lane() {
            (s1.velocity().vx - s2.velocity().vx).abs()
        } else {
            0.0
        }
    }

    /// `distance - dt * closing_speed` when same-lane, else `BIG`.
    fn ttc_proxy(scenario: &Scenario, a: usize, b: usize, t: usize) -> f64 {
        let (s1, s2) = (&scenario.actors[a].states[t], &scenario.actors[b].states[t]);
        if s1.lane() == s2.lane() {
            (s1.position().x - s2.position().x).abs()
                - scenario.time_step * (s1.velocity().vx - s2.velocity().vx).abs()
        } else {
            BIG
        }
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
            // Both distance targets score the same quantity — the smallest
            // same-lane gap — and differ only in which way they push it.
            OptimizationTarget::MinimizeDistance | OptimizationTarget::MaximizeTtc => {
                fold_pairs(scenario, f64::INFINITY, effective_dist, f64::min)
            }
            OptimizationTarget::MinimizeTtc => {
                fold_pairs(scenario, f64::INFINITY, ttc_proxy, f64::min)
            }
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
    /// speed. `MaximizeTtc` maximises the smallest same-lane gap.
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

/// The optimizer's defining property, for every target: optimising must not
/// produce a worse objective than not optimising.
///
/// Nothing asserted this before. `test_optimize_minimize_ttc` checked
/// `val > -1000.0 && val < 1000.0` — a finiteness check standing in for an
/// optimality check, which an optimizer that returned the first satisfying
/// model it found would pass. Two solver calls and one comparison per target is
/// the whole of what actually validates this code path.
///
/// Eight solver calls, and it holds today: on this spec every target's
/// objective is at least as good optimised as unoptimised.
#[test]
fn test_optimization_never_worsens_its_own_objective() {
    let base = create_optimized_spec(OptimizationTarget::None);
    let mut failures: Vec<String> = Vec::new();

    for target in ALL_TARGETS {
        let (optimized, plain) = optimality_gap(&base, target);
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

    assert!(
        failures.is_empty(),
        "{} of {} optimizer targets failed the optimality property:\n  {}",
        failures.len(),
        ALL_TARGETS.len(),
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
