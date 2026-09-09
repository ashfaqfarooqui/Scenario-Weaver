//! `min_lateral_distance` on a pedestrian crossing: encoded, guarded on
//! longitudinal relevance, and measured the same way.
//!
//! **SW-32** made `PedestrianCrossingModel::generate_safety` stop silently
//! dropping the field — before it, a spec that set `min_lateral_distance` in
//! any constraint mode got exactly the same encoding as one that omitted it.
//!
//! **SW-51** made the constraint mean something. SW-32 lowered it as an
//! *unguarded* `|py_ego − py_ped| >= d` at every step, and for a perpendicular
//! crossing that is self-contradictory: the pedestrian's path runs through
//! `py_ego`, so a continuous crossing has `|dy| = 0` at some instant. It was
//! satisfiable only because the trajectory is *sampled*, with a ceiling of
//! `(v_ped × time_step) / 2` — the half-hop the pedestrian takes across the
//! forbidden band between two samples. Measured at HEAD before the fix:
//!
//! ```text
//! pedestrian_crossing.yaml (v_ped <= 1.5, time_step 0.30)  d = 0.22  SAT
//! pedestrian_crossing.yaml (v_ped <= 1.5, time_step 0.30)  d = 0.25  UNSAT
//! pedestrian_crossing.yaml (v_ped <= 1.5, time_step 0.15)  d = 0.22  UNSAT
//! pedestrian_crossing.yaml (v_ped <= 1.5, time_step 0.15)  d = 0.11  SAT
//! pedestrian_wide_road.yaml (v_ped <= 1.4, time_step 0.40) d = 0.25  SAT
//! pedestrian_wide_road.yaml (v_ped <= 1.4, time_step 0.20) d = 0.25  UNSAT
//! ```
//!
//! — a feasible set that is a function of the discretisation, not of the
//! physics, and in which every value a user would plausibly author (`2.0`, the
//! same order as the `min_distance: 2.0` beside it in the example) is UNSAT by
//! construction. **This file used to encode that ceiling as its own threshold**
//! (0.4 m, recalibrated to 0.25 m when SW-45 fixed the walking speed). The
//! number was the symptom; the guard is the finding, and the thresholds below
//! are deliberately physically meaningful values rather than half-hops.
//!
//! The constraint is now
//!
//! ```text
//! G( |dx| <= W  =>  |dy| >= min_lateral_distance )
//! ```
//!
//! with `W = pedestrian_crossing::lateral_relevance_window(spec)` — lowered as
//! the equivalent box `RectangularDistanceGT { threshold_x: W, threshold_y: d }`.
//!
//! `pedestrian_wide_road.yaml` is used for most cases rather than
//! `pedestrian_crossing.yaml` because it puts the ego in lane 1 and the
//! pedestrian in lane 0, so the pair's `py` values start further apart and the
//! crossing has more road to cover; `pedestrian_crossing.yaml` carries the
//! sampling-independence case, which is where its finer `time_step` matters.

mod common;

use scenario_weaver::dsl::types::{ConstraintMode, ConstraintModes, ScenarioSpec};
use scenario_weaver::scenario::model::Scenario;

/// Every constraint but `min_lateral_distance` ignored, so only the
/// constraint under test can affect satisfiability or its reported metric.
fn modes_isolating_lateral(min_lateral_distance: ConstraintMode) -> ConstraintModes {
    ConstraintModes::Detailed {
        min_ttc: ConstraintMode::Ignore,
        min_distance: ConstraintMode::Ignore,
        max_acceleration: ConstraintMode::Ignore,
        max_velocity: ConstraintMode::Ignore,
        min_velocity: ConstraintMode::Ignore,
        min_lateral_distance,
        max_relative_velocity: ConstraintMode::Ignore,
    }
}

/// The longitudinal relevance window the encoder used for this spec.
///
/// Re-derived from the spec rather than imported, because
/// `lateral_relevance_window` is `pub(crate)` — an integration test crate
/// cannot see it. Keeping the arithmetic here in one place and asserting
/// against it is the closest an integration test gets to the coupling check
/// `metrics.rs::test_pedestrian_box_thresholds_share_one_definition` does for
/// the other box; if the two ever disagree, every case below that leans on the
/// guard's *position* fails rather than silently passing.
fn relevance_window(spec: &ScenarioSpec) -> f64 {
    let ego = spec.ego().expect("ego actor");
    (spec.min_ttc * ego.speed.max()).max(spec.min_distance / 2.0)
}

/// `(|dx|, |dy|)` between the pedestrian and the ego at every step.
fn separations(scenario: &Scenario, ped_id: &str) -> Vec<(f64, f64)> {
    let ped = scenario.get_actor(ped_id).expect("pedestrian actor");
    let ego = scenario.get_actor("ego").expect("ego actor");
    ped.states
        .iter()
        .zip(&ego.states)
        .map(|(p, e)| {
            (
                (p.position().x - e.position().x).abs(),
                (p.position().y - e.position().y).abs(),
            )
        })
        .collect()
}

/// A physically meaningful lateral clearance — the same order as the
/// `min_distance` sitting beside it in both example specs, and roughly a
/// vehicle's half-width plus a margin. Under the pre-SW-51 unguarded lowering
/// every value at this scale was UNSAT by construction.
const MEANINGFUL_CLEARANCE: f64 = 2.0;

/// A physically meaningful value is now satisfiable, and the shipped
/// trajectory honours the guarded property step by step.
///
/// This is the regression test for SW-51's core claim. Against the pre-fix
/// (unguarded) lowering this spec is `Unsatisfiable` — verified by running it —
/// because the pedestrian has to walk through `py_ego` and the unguarded form
/// forbids `|dy| < 2.0` at every step of that walk.
#[test]
fn a_meaningful_min_lateral_distance_is_satisfiable_and_guarded() {
    let mut spec = common::parse_example("pedestrian_wide_road.yaml");
    spec.constraint_modes = modes_isolating_lateral(ConstraintMode::Enforce);
    spec.min_lateral_distance = Some(MEANINGFUL_CLEARANCE);
    let window = relevance_window(&spec);

    let scenario = scenario_weaver::generate_single_scenario_from_spec(spec.clone())
        .unwrap_or_else(|e| panic!("expected solvable at a meaningful lateral floor: {e}"));

    let pairs = separations(&scenario, "ped");

    // The guarded property itself, at every step.
    for (t, (dx, dy)) in pairs.iter().enumerate() {
        assert!(
            *dx >= window - 1e-6 || *dy >= MEANINGFUL_CLEARANCE - 1e-6,
            "step {t}: |dx|={dx:.4} is inside the {window:.2} m relevance window, so \
             min_lateral_distance: enforce demanded |dy| >= {MEANINGFUL_CLEARANCE}, but \
             |dy|={dy:.4}"
        );
    }

    // ...and the guard is not vacuous: some step really is inside the window,
    // so `enforce` is doing work rather than being satisfied by the ego always
    // being somewhere else.
    let guarded_steps = pairs.iter().filter(|(dx, _)| *dx < window).count();
    assert!(
        guarded_steps > 0,
        "no step fell inside the {window:.2} m relevance window, so the enforced \
         constraint was vacuously true"
    );

    // ...and the pedestrian really does traverse the ego's lateral position —
    // which is the whole reason the unguarded form could not work. It may only
    // do so outside the window.
    let closest = pairs
        .iter()
        .map(|(_, dy)| *dy)
        .fold(f64::INFINITY, f64::min);
    assert!(
        closest < MEANINGFUL_CLEARANCE,
        "the pedestrian never came within {MEANINGFUL_CLEARANCE} m of the ego laterally \
         (closest {closest:.4} m), so this scenario is not a crossing and the guard was \
         never tested"
    );
}

/// The feasible set no longer depends on the sampling rate.
///
/// This is the criterion that separates a real fix from a re-tuned constant.
/// Pre-fix, `pedestrian_crossing.yaml` at `time_step: 0.30` topped out at
/// `d = 0.22` and at `time_step: 0.15` at `d = 0.11` — halving the step halved
/// the achievable separation, because the ceiling was `(v_ped × time_step) / 2`
/// and nothing else. Post-fix the same physically meaningful `d` holds at both.
#[test]
fn satisfiability_does_not_depend_on_the_time_step() {
    for time_step in [0.3, 0.15] {
        let mut spec = common::parse_example("pedestrian_crossing.yaml");
        spec.constraint_modes = modes_isolating_lateral(ConstraintMode::Enforce);
        spec.min_lateral_distance = Some(MEANINGFUL_CLEARANCE);
        spec.time_step = time_step;
        let window = relevance_window(&spec);

        let scenario = scenario_weaver::generate_single_scenario_from_spec(spec.clone())
            .unwrap_or_else(|e| {
                panic!(
                    "min_lateral_distance: {MEANINGFUL_CLEARANCE} must be satisfiable \
                     independently of time_step, but time_step={time_step} gave {e}"
                )
            });

        for (t, (dx, dy)) in separations(&scenario, "pedestrian").iter().enumerate() {
            assert!(
                *dx >= window - 1e-6 || *dy >= MEANINGFUL_CLEARANCE - 1e-6,
                "time_step={time_step}, step {t}: |dx|={dx:.4} inside the {window:.2} m \
                 window but |dy|={dy:.4}"
            );
        }
    }
}

/// `Violate` produces a real near miss, not a satisfied antecedent-is-false.
///
/// The atom is `A => B`, so its negation is `A AND NOT B` — the solver has to
/// bring the ego *inside* the relevance window and only then close the lateral
/// gap. A trajectory that satisfied the negation by keeping the ego far away
/// would be vacuous, and this asserts it does not: some step carries both
/// halves at once.
#[test]
fn violating_min_lateral_distance_enters_the_guarded_region() {
    let mut spec = common::parse_example("pedestrian_wide_road.yaml");
    spec.constraint_modes = modes_isolating_lateral(ConstraintMode::Violate);
    spec.min_lateral_distance = Some(MEANINGFUL_CLEARANCE);
    let window = relevance_window(&spec);

    let scenario = scenario_weaver::generate_single_scenario_from_spec(spec.clone())
        .unwrap_or_else(|e| panic!("expected solvable: {e}"));

    let pairs = separations(&scenario, "ped");
    let breach = pairs
        .iter()
        .enumerate()
        .find(|(_, (dx, dy))| *dx < window && *dy < MEANINGFUL_CLEARANCE);

    let Some((t, (dx, dy))) = breach else {
        let closest = pairs
            .iter()
            .map(|(_, dy)| *dy)
            .fold(f64::INFINITY, f64::min);
        panic!(
            "min_lateral_distance: violate must produce a step that is both inside the \
             {window:.2} m longitudinal window and under {MEANINGFUL_CLEARANCE} m laterally; \
             closest lateral approach anywhere was {closest:.4} m, so the negation was \
             satisfied vacuously"
        );
    };
    assert!(
        *dx < window && *dy < MEANINGFUL_CLEARANCE,
        "step {t}: |dx|={dx:.4}, |dy|={dy:.4}"
    );
}

/// The guard is a relevance window, not an escape hatch.
///
/// Widen the window past anything the ego can traverse in the horizon and the
/// guarded constraint degenerates back into the unguarded one — at which point
/// an unmeetable threshold is still reported `Unsatisfiable`, exactly as every
/// other safety constraint is when it cannot be met.
///
/// `min_ttc` is the knob because `W = max(min_ttc * ego_speed_max,
/// min_distance / 2)`: at `min_ttc: 100` the window is 1200 m against the
/// at-most 120 m the ego covers in this spec's 10 s, so every step of every
/// trajectory is inside it. Its constraint *mode* stays `Ignore`, so no TTC
/// constraint is asserted and satisfiability turns on the lateral field alone.
#[test]
fn an_unmeetable_min_lateral_distance_is_still_unsatisfiable() {
    let mut spec = common::parse_example("pedestrian_wide_road.yaml");
    spec.constraint_modes = modes_isolating_lateral(ConstraintMode::Enforce);
    spec.min_ttc = 100.0;
    // The road is 3 lanes * 3.5 m plus sidewalks; with the window covering the
    // whole scenario, a crossing cannot keep 20 m of lateral separation.
    spec.min_lateral_distance = Some(20.0);

    let result = scenario_weaver::generate_single_scenario_from_spec(spec);
    assert!(
        matches!(
            result,
            Err(scenario_weaver::error::ScenarioGenError::Unsatisfiable)
        ),
        "expected Unsatisfiable for an unmeetable min_lateral_distance under a \
         scenario-wide relevance window, got {result:?}"
    );
}

/// And with that same scenario-wide window, even a *meaningful* value is
/// unsatisfiable — because a crossing pedestrian must traverse `py_ego`.
///
/// This pins the geometry SW-51 is built on: the guard is not what makes
/// `2.0 m` reachable in general, it is what makes it reachable *where lateral
/// separation is a safety property*. Remove the window and the old
/// impossibility returns.
#[test]
fn a_scenario_wide_window_reproduces_the_unguarded_impossibility() {
    let mut spec = common::parse_example("pedestrian_wide_road.yaml");
    spec.constraint_modes = modes_isolating_lateral(ConstraintMode::Enforce);
    spec.min_ttc = 100.0;
    spec.min_lateral_distance = Some(MEANINGFUL_CLEARANCE);

    let result = scenario_weaver::generate_single_scenario_from_spec(spec);
    assert!(
        matches!(
            result,
            Err(scenario_weaver::error::ScenarioGenError::Unsatisfiable)
        ),
        "with a relevance window covering the whole scenario the guarded constraint is \
         the unguarded one, which a perpendicular crossing cannot satisfy at \
         {MEANINGFUL_CLEARANCE} m; got {result:?}"
    );
}

/// The validator measures the guarded property, not the unguarded one.
///
/// `compute_validation_metrics` is the only independent check of a generated
/// scenario against its spec. If it kept measuring the unguarded `|dy| >= d`
/// it would report the entire approach phase — where the encoder deliberately
/// asserts nothing — as a stream of lateral-distance violations, and an
/// `enforce`d spec the solver satisfied perfectly would ship with
/// `all_constraints_satisfied: false`. That is the same false positive the
/// pedestrian distance-box branch exists to remove.
#[test]
fn the_validator_measures_the_guarded_property() {
    let mut spec = common::parse_example("pedestrian_wide_road.yaml");
    spec.constraint_modes = modes_isolating_lateral(ConstraintMode::Enforce);
    spec.min_lateral_distance = Some(MEANINGFUL_CLEARANCE);

    let scenario = scenario_weaver::generate_single_scenario_from_spec(spec)
        .expect("expected solvable at a meaningful lateral floor");

    let lateral: Vec<&String> = scenario
        .validation
        .safety_violations
        .iter()
        .filter(|v| v.contains("Lateral distance violation"))
        .collect();
    assert!(
        lateral.is_empty(),
        "the encoder satisfied the guarded constraint, so the validator must report no \
         lateral-distance violation; got {lateral:?}"
    );
    assert!(
        scenario.validation.all_constraints_satisfied,
        "all_constraints_satisfied must be true: {:?}",
        scenario.validation.safety_violations
    );
}

/// ...and it still reports one when the guarded property is genuinely broken.
///
/// The mirror of the test above: a `violate`d spec drives the pair into the
/// relevance window with the lateral gap closed, and the validator must see it.
/// Without this, "no violations reported" would be equally consistent with a
/// check that never fires.
#[test]
fn the_validator_reports_a_guarded_violation_under_violate_mode() {
    let mut spec = common::parse_example("pedestrian_wide_road.yaml");
    spec.constraint_modes = modes_isolating_lateral(ConstraintMode::Violate);
    spec.min_lateral_distance = Some(MEANINGFUL_CLEARANCE);

    let scenario =
        scenario_weaver::generate_single_scenario_from_spec(spec).expect("expected solvable");

    let lateral: Vec<&String> = scenario
        .validation
        .safety_violations
        .iter()
        .filter(|v| v.contains("Lateral distance violation"))
        .collect();
    assert!(
        !lateral.is_empty(),
        "min_lateral_distance: violate produced a breach of the guarded property, so the \
         validator must report it; got {:?}",
        scenario.validation.safety_violations
    );
}
