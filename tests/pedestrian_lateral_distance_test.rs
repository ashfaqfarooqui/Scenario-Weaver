//! SW-32: `min_lateral_distance` must not be silently ignored by
//! `pedestrian_crossing`.
//!
//! `PedestrianCrossingModel::generate_safety` replaces
//! `generate_default_safety` wholesale and, before this fix, never once read
//! `spec.min_lateral_distance` — a spec that set it, in any constraint mode,
//! got exactly the same encoding as one that omitted it. `enforced_...`
//! below fails against the pre-fix code for exactly that reason: nothing
//! forces any lateral separation, so the closest approach the solver happens
//! to find is well under the requested floor. `violating_...` is the mirror
//! case: nothing forbids the constraint's own negation from being trivially
//! true, so it also passed accidentally before the fix — the closest
//! approach was almost certainly already under the threshold with no
//! `Violate`-mode constraint driving it there.
//!
//! `pedestrian_wide_road.yaml` is used rather than `pedestrian_crossing.yaml`
//! because it puts the ego in lane 1 and the pedestrian in lane 0: their `py`
//! values start one `lane_width` apart instead of coinciding.
//! `pedestrian_crossing.yaml` declares both actors in lane 0, and a
//! pedestrian's initial `py` is anchored to its own declared lane's centre
//! (`encode_pedestrian_initial_state`, `src/solver/encoders/cartesian.rs` —
//! fenced, not touched here) exactly as a vehicle's is — so there the two
//! actors start at *exactly* the same `py`, and demanding any positive
//! `min_lateral_distance` at every step is correctly unsatisfiable
//! regardless of the lowering. That is the spec asking for something the
//! declared actor placement already rules out, not a defect in the
//! lowering: verified separately (not asserted here) that the same
//! constraint on `pedestrian_wide_road.yaml`, where the actors do not
//! coincide, is satisfiable for a modest threshold and correctly becomes
//! `Unsatisfiable` once the threshold is large enough that the crossing
//! cannot honour it — i.e. it fails loudly instead of succeeding silently,
//! which is the fix this issue asks for either way.

mod common;

use scenario_weaver::dsl::types::{ConstraintMode, ConstraintModes};

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

/// Closest lateral approach between the pedestrian and the ego, in metres.
fn min_lateral_separation(scenario: &scenario_weaver::scenario::model::Scenario) -> f64 {
    let ped = scenario.get_actor("ped").expect("pedestrian actor");
    let ego = scenario.get_actor("ego").expect("ego actor");
    ped.states
        .iter()
        .zip(&ego.states)
        .map(|(p, e)| (p.position().y - e.position().y).abs())
        .fold(f64::INFINITY, f64::min)
}

#[test]
fn enforced_min_lateral_distance_actually_holds() {
    let mut spec = common::parse_example("pedestrian_wide_road.yaml");
    spec.constraint_modes = modes_isolating_lateral(ConstraintMode::Enforce);
    // 0.25 m is above the ~0.09 m closest approach this same example settles on
    // with every constraint ignored (verified separately), so a pass here means
    // the constraint did something rather than being silently dropped.
    //
    // Why 0.25 and not the 0.4 this used to demand: the ego sits stationary in
    // the middle lane (py = 5.25) and the pedestrian must cross straight through
    // it, so the pair's lateral separation is bounded by how far `py_ped` can
    // step across `py_ego` in one sample. At the honest walking speed (SW-45:
    // the crossing runs at the authored 1.4 m/s, not the 2.0 m/s walk cap the
    // buggy encoder used) the largest lateral step is 1.4 · 0.4 = 0.56 m, so the
    // best a full crossing can straddle the ego's lane is ±0.28 m; 0.4 m is
    // physically unreachable and 0.25 m is the meaningful floor just inside it.
    // The old 0.4 m only held because the pre-SW-45 pedestrian hopped the
    // forbidden band in a single 0.8 m step — a sampling artifact of the wrong
    // crossing speed, not real clearance.
    spec.min_lateral_distance = Some(0.25);

    let scenario = scenario_weaver::generate_single_scenario_from_spec(spec.clone())
        .unwrap_or_else(|e| panic!("expected solvable with a modest lateral floor: {e}"));

    let min_lat = min_lateral_separation(&scenario);
    assert!(
        min_lat >= 0.25 - 1e-6,
        "min_lateral_distance: enforce demanded >= 0.25 m of lateral separation at every \
         step, but the closest approach found was {min_lat:.4} m — the constraint did \
         nothing"
    );
}

#[test]
fn violating_min_lateral_distance_actually_produces_a_close_approach() {
    let mut spec = common::parse_example("pedestrian_wide_road.yaml");
    spec.constraint_modes = modes_isolating_lateral(ConstraintMode::Violate);
    spec.min_lateral_distance = Some(2.0);

    let scenario = scenario_weaver::generate_single_scenario_from_spec(spec.clone())
        .unwrap_or_else(|e| panic!("expected solvable: {e}"));

    let min_lat = min_lateral_separation(&scenario);
    assert!(
        min_lat < 2.0 - 1e-6,
        "min_lateral_distance: violate demanded the pair come within 2.0 m of each other \
         at some step, but the closest approach found was {min_lat:.4} m"
    );
}

/// An `Enforce`d threshold too large for the scenario to honour must be
/// reported as unsatisfiable, not silently dropped — the same behaviour
/// every other safety constraint gets when it cannot be met.
#[test]
fn an_unmeetable_min_lateral_distance_is_reported_unsatisfiable() {
    let mut spec = common::parse_example("pedestrian_wide_road.yaml");
    spec.constraint_modes = modes_isolating_lateral(ConstraintMode::Enforce);
    // The road is 3 lanes * 3.5 m = 10.5 m wide; demanding 20 m of lateral
    // separation at every step cannot be satisfied by any crossing.
    spec.min_lateral_distance = Some(20.0);

    let result = scenario_weaver::generate_single_scenario_from_spec(spec);
    assert!(
        matches!(
            result,
            Err(scenario_weaver::error::ScenarioGenError::Unsatisfiable)
        ),
        "expected Unsatisfiable for an unmeetable min_lateral_distance, got {result:?}"
    );
}
