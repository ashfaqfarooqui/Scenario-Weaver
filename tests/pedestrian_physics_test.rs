//! Pedestrian scenario generation, end to end.
//!
//! This file was the only place in the suite that looked at the numbers a
//! generated trajectory contains rather than at whether one was returned. It
//! still is the pedestrian-specific home for that, but the checks themselves
//! now come from `common::invariants`, so the same rules apply to every actor
//! in every scenario type instead of to pedestrians in three examples.
//!
//! Two things changed in the process, both of which made checks stricter:
//!
//! - **Tolerance `0.1` → `common::TOL` (1e-6).** Against `dt = 0.5` the old
//!   figure was about ten percent of slack, wide enough to hide a sign error on
//!   a small `vy` — exactly the defect C2 turns out to produce. Z3 works in
//!   exact rationals, so there was never anything for that slack to absorb.
//! - **The acceleration bound is no longer transcribed.**
//!   `test_pedestrian_crossing_acceleration_bounds` compared against a literal
//!   `1.0` while the speed test correctly imported
//!   `PEDESTRIAN_WALK_MAX_SPEED`; the invariant reads
//!   `PEDESTRIAN_MAX_ACCELERATION` / `PEDESTRIAN_MAX_DECELERATION` from the
//!   same module the encoder does, and the vehicle bounds off the spec.
//!
//! The Euler-integration checks survive as `Invariant::Kinematics`, generalised
//! to both axes, to velocity as well as position, and to every actor. They
//! failed when this file was written and pass since SW-08 corrected the
//! position update: on these three examples the pedestrian's `vy` is chained to
//! its `ay`, so the second-order term applies on both axes and the invariant
//! holds at 1e-6. The two tests still `#[ignore]`d here are SW-10's and
//! SW-12's, not kinematics.

mod common;

use common::Invariant;
use scenario_weaver::dsl::types::{
    ActorRole, BicycleConfig, CoordinateSystem, PEDESTRIAN_RUN_MAX_SPEED, PEDESTRIAN_WALK_MAX_SPEED,
};
use scenario_weaver::scenario::model::Scenario;

/// Generate an example and hand back the spec it came from, so bounds are
/// checked against what the scenario declares rather than against literals.
fn generate_from_file(name: &str) -> (Scenario, scenario_weaver::dsl::types::ScenarioSpec) {
    common::generate_example_with_spec(name)
}

// ─── Basic pedestrian crossing (walk mode) ───

#[test]
fn test_pedestrian_crossing_generates_successfully() {
    let (scenario, _) = generate_from_file("pedestrian_crossing.yaml");

    assert_eq!(scenario.actors.len(), 2);
    let ped = scenario
        .get_actor("pedestrian")
        .expect("Should have pedestrian actor");
    assert_eq!(ped.role, ActorRole::Pedestrian);
    assert!(!ped.states.is_empty());
}

/// Position and velocity must integrate the accelerations reported beside them.
///
/// Holds since SW-08: the position update carries the `½·a·dt²` term on both
/// axes for pedestrians, whose `vy` is chained to `ay`, so the residual against
/// `p + v·dt + ½·a·dt²` is zero instead of being exactly `½·a·dt²`. It used
/// to fail at 1e-6 and pass only at the old `0.1` tolerance.
#[test]
fn test_pedestrian_crossing_kinematics_consistency() {
    let (scenario, spec) = generate_from_file("pedestrian_crossing.yaml");
    common::assert_invariant(&scenario, &spec, Invariant::Kinematics);
}

/// Same invariant, stated for the scenario as a whole: `Invariant::Kinematics`
/// covers `v[i+1] = v[i] + a[i]·dt` on both axes for every actor, which is
/// where C2 would show up — a vehicle's `vy` stepping `0 → -2.0 → +2.0` with
/// `ay = 0.0` throughout. The only vehicle in this example never changes lane,
/// so its `vy` is pinned to zero and C2 has nothing to corrupt here. The corpus
/// version, `examples_smoke_test::test_kinematic_consistency_across_the_corpus`,
/// is where C2 still bites; it stays `#[ignore]`d against SW-09.
#[test]
fn test_pedestrian_crossing_velocity_consistency() {
    let (scenario, spec) = generate_from_file("pedestrian_crossing.yaml");
    common::assert_invariant(&scenario, &spec, Invariant::Kinematics);
}

/// Speed and acceleration envelopes, both read from the code the encoder reads.
///
/// `Invariant::Envelope` checks `|vx|` and `|vy|` against the pedestrian
/// ceiling selected by this actor's own `walking_mode`, and `ax`/`ay` against
/// `[PEDESTRIAN_MAX_DECELERATION, PEDESTRIAN_MAX_ACCELERATION]` — no literal
/// `1.0`, and no need to know here whether the pedestrian walks or runs.
#[test]
fn test_pedestrian_crossing_speed_bounds() {
    let (scenario, spec) = generate_from_file("pedestrian_crossing.yaml");
    common::assert_invariant(&scenario, &spec, Invariant::Envelope);

    // The example declares a walking pedestrian, so the ceiling the invariant
    // applied must be the walking one. Asserted here because the invariant is
    // deliberately agnostic about which mode a given actor is in.
    let ped = scenario.get_actor("pedestrian").expect("pedestrian actor");
    assert!(
        ped.states
            .iter()
            .all(|s| s.velocity().vx.abs() <= PEDESTRIAN_WALK_MAX_SPEED + common::TOL),
        "pedestrian_crossing declares a walking pedestrian, so no state may exceed \
         PEDESTRIAN_WALK_MAX_SPEED ({PEDESTRIAN_WALK_MAX_SPEED})"
    );
}

#[test]
fn test_pedestrian_crossing_acceleration_bounds() {
    let (scenario, spec) = generate_from_file("pedestrian_crossing.yaml");
    common::assert_invariant(&scenario, &spec, Invariant::Envelope);
}

#[test]
fn test_pedestrian_crosses_laterally() {
    let (scenario, _) = generate_from_file("pedestrian_crossing.yaml");
    let ped = scenario.get_actor("pedestrian").expect("pedestrian actor");

    // Pedestrian should move laterally (py should change over time)
    let py_start = ped.states[0].position().y;
    let py_end = ped.states.last().expect("at least one state").position().y;

    assert!(
        (py_end - py_start).abs() > 1.0,
        "Pedestrian should cross laterally: py_start={py_start:.2}, py_end={py_end:.2}, \
         delta={:.2}",
        (py_end - py_start).abs()
    );
}

// ─── Running pedestrian ───

#[test]
fn test_pedestrian_running_generates_successfully() {
    let (scenario, _) = generate_from_file("pedestrian_running.yaml");

    assert_eq!(scenario.actors.len(), 2);
    let ped = scenario
        .get_actor("runner")
        .expect("Should have runner actor");
    assert_eq!(ped.role, ActorRole::Pedestrian);
}

#[test]
fn test_pedestrian_running_speed_bounds() {
    let (scenario, spec) = generate_from_file("pedestrian_running.yaml");
    common::assert_invariant(&scenario, &spec, Invariant::Envelope);

    // `walking_mode: run` must actually raise the ceiling, and the running
    // pedestrian must actually use some of it — otherwise the mode is decorative
    // and the envelope check would pass on a walking trajectory.
    let runner = scenario.get_actor("runner").expect("runner actor");
    let fastest = runner
        .states
        .iter()
        .map(|s| s.velocity().speed())
        .fold(0.0_f64, f64::max);
    assert!(
        fastest <= PEDESTRIAN_RUN_MAX_SPEED * std::f64::consts::SQRT_2 + common::TOL,
        "runner speed {fastest:.4} exceeds the running box's diagonal"
    );
    assert!(
        fastest > PEDESTRIAN_WALK_MAX_SPEED,
        "pedestrian_running declares walking_mode: run but the runner never exceeds the \
         walking ceiling ({PEDESTRIAN_WALK_MAX_SPEED}); fastest was {fastest:.4}"
    );
}

#[test]
fn test_pedestrian_running_kinematics_consistency() {
    let (scenario, spec) = generate_from_file("pedestrian_running.yaml");
    common::assert_invariant(&scenario, &spec, Invariant::Kinematics);
}

// ─── Wide road pedestrian ───

#[test]
fn test_pedestrian_wide_road_generates_successfully() {
    let (scenario, _) = generate_from_file("pedestrian_wide_road.yaml");

    assert_eq!(scenario.actors.len(), 2);
    let ped = scenario
        .get_actor("ped")
        .expect("Should have pedestrian actor");
    assert_eq!(ped.role, ActorRole::Pedestrian);

    // Ego should be in middle lane
    let ego = scenario.get_actor("ego").expect("ego actor");
    assert_eq!(ego.states[0].lane(), 1);
}

#[test]
fn test_pedestrian_wide_road_kinematics_consistency() {
    let (scenario, spec) = generate_from_file("pedestrian_wide_road.yaml");
    common::assert_invariant(&scenario, &spec, Invariant::Kinematics);
}

#[test]
fn test_pedestrian_wide_road_crosses_multiple_lanes() {
    let (scenario, spec) = generate_from_file("pedestrian_wide_road.yaml");
    let ped = scenario.get_actor("ped").expect("pedestrian actor");

    // The distance to beat is one lane width, read from the spec rather than
    // transcribed: this example's road is wider than the default.
    let lane_width = spec.get_lane_width();
    let py_start = ped.states[0].position().y;
    let py_end = ped.states.last().expect("at least one state").position().y;

    assert!(
        (py_end - py_start).abs() > lane_width,
        "Pedestrian should cross at least one lane width ({lane_width} m) on a \
         {}-lane road: delta_py={:.2}",
        spec.get_num_lanes(),
        (py_end - py_start).abs()
    );
}

/// A pedestrian must stay on the road surface, and the lane it is recorded in
/// must be the lane its `y` puts it in.
///
/// Fails on `py` leaving the road surface (finding E3): `OnSidewalk` is encoded
/// as the unbounded half-plane `py > lane_width * num_lanes`, so the pedestrian
/// parks up to 6.1 m off the road (`pedestrian_crossing`: `py = 7.85` on a road
/// of `[0, 7]`) with `lane` still reading the lane it started in. A `py` that is
/// off the road cannot agree with any lane index.
///
/// This is no longer the SW-10 lane lag. `lane` is derived from `py` at every
/// step now, and all 19 vehicle examples satisfy this invariant.
#[test]
#[ignore = "SW-16 (E3): pedestrians are steered outside the road surface, so no lane index \
            can agree with their py. The SW-10 half — the lane variable lagging lateral \
            position — is fixed"]
fn test_pedestrian_stays_on_the_road_and_in_its_recorded_lane() {
    for name in [
        "pedestrian_crossing.yaml",
        "pedestrian_running.yaml",
        "pedestrian_wide_road.yaml",
    ] {
        let (scenario, spec) = generate_from_file(name);
        common::assert_invariant(&scenario, &spec, Invariant::Containment);
    }
}

// ─── Ego vehicle sanity checks ───

/// The ego must actually travel while the pedestrian crosses.
///
/// Fails: on all three pedestrian examples the ego brakes at the encoder's
/// floor and then sits still for a majority of the horizon
/// (`pedestrian_crossing`: 29 of 35 steps, 8.7 s of 10 s), which trivially
/// satisfies every distance and TTC threshold — `pedestrian_wide_road` reports
/// `all_constraints_satisfied: true` while doing it. The weaker
/// "ends further along than it started" form below still passes, which is why
/// it took an invariant to see this.
#[test]
#[ignore = "SW-12: the ego brakes to a standstill for a majority of the horizon on every pedestrian example and the result is still reported as satisfying every constraint"]
fn test_ego_makes_forward_progress_during_pedestrian_crossing() {
    for name in [
        "pedestrian_crossing.yaml",
        "pedestrian_running.yaml",
        "pedestrian_wide_road.yaml",
    ] {
        let (scenario, spec) = generate_from_file(name);
        common::assert_invariant(&scenario, &spec, Invariant::ForwardProgress);
    }
}

#[test]
fn test_ego_moves_forward_during_pedestrian_crossing() {
    let (scenario, _) = generate_from_file("pedestrian_crossing.yaml");
    let ego = scenario.get_actor("ego").expect("ego actor");

    // Ego should move forward (px increasing over time)
    let px_start = ego.states[0].position().x;
    let px_end = ego.states.last().expect("at least one state").position().x;

    assert!(
        px_end > px_start,
        "Ego should move forward: px_start={px_start:.2}, px_end={px_end:.2}"
    );
}

#[test]
fn test_ego_stays_in_lane_during_pedestrian_crossing() {
    let (scenario, _) = generate_from_file("pedestrian_crossing.yaml");
    let ego = scenario.get_actor("ego").expect("ego actor");

    // The ego performs no lane change in this scenario, so its lateral velocity
    // is not merely small — it is exactly zero, and is checked as such.
    for state in &ego.states {
        assert!(
            state.velocity().vy.abs() <= common::TOL,
            "Ego makes no lane change here, so vy must be 0 at t={:.2}s: got {:.9}",
            state.time,
            state.velocity().vy
        );
    }
}

// ─── Pedestrians under `coordinate_system: bicycle` (SW-09 / finding C3) ───

/// `pedestrian_crossing.yaml` re-solved in the bicycle coordinate system.
fn pedestrian_crossing_bicycle() -> (Scenario, scenario_weaver::dsl::types::ScenarioSpec) {
    let mut spec = common::parse_example("pedestrian_crossing.yaml");
    spec.coordinate_system = CoordinateSystem::Bicycle;
    // The bicycle encoder needs wheelbase/steering parameters for its vehicles.
    spec.bicycle_config = Some(BicycleConfig {
        default_wheelbase: 2.7,
        default_max_steering_angle: 0.6,
        default_max_steering_rate: 0.5,
    });
    let scenario = common::generate_spec_or_fail(spec.clone());
    (scenario, spec)
}

/// A pedestrian under `coordinate_system: bicycle` obeys the same kinematics
/// as one under `cartesian`.
///
/// Before SW-09 `encoders/bicycle.rs` `continue`d past every pedestrian: no
/// `px`, `py` or `v` update was ever asserted, so a pedestrian's position at
/// `t > 0` was entirely unconstrained and Z3 was free to teleport them to
/// wherever the safety propositions were cheapest to satisfy. Nothing in
/// `ScenarioSpec::validate` rejected the combination, so it was reachable from
/// user YAML. They now go through the same point-mass helpers the Cartesian
/// encoder uses.
#[test]
fn test_pedestrian_under_bicycle_has_constrained_kinematics() {
    let (scenario, spec) = pedestrian_crossing_bicycle();
    common::assert_invariant(&scenario, &spec, Invariant::Kinematics);

    let ped = scenario.get_actor("pedestrian").expect("pedestrian actor");
    let dt = scenario.time_step;

    // Explicitly: consecutive positions integrate the reported velocities, so
    // no step is a teleport. The bound is generous on purpose — the point is
    // that a bound exists at all, where before any jump was admissible.
    for w in ped.states.windows(2) {
        let (s, next) = (&w[0], &w[1]);
        let step = (next.position().x - s.position().x).hypot(next.position().y - s.position().y);
        let max_step = PEDESTRIAN_WALK_MAX_SPEED * dt * std::f64::consts::SQRT_2 + common::TOL;
        assert!(
            step <= max_step,
            "pedestrian moved {step:.6} m in one {dt}s step at t={:.2}s, above the \
             {max_step:.6} m the speed box allows",
            next.time
        );
    }
}

/// The pedestrian's lateral acceleration in the bicycle system is a real solver
/// variable, not the hard-coded `ay = 0.0` extraction used to report.
#[test]
fn test_pedestrian_under_bicycle_reports_a_real_lateral_acceleration() {
    let (scenario, _) = pedestrian_crossing_bicycle();
    let ped = scenario.get_actor("pedestrian").expect("pedestrian actor");
    let dt = scenario.time_step;

    for w in ped.states.windows(2) {
        let (s, next) = (&w[0], &w[1]);
        let expected = s.velocity().vy + s.acceleration().ay * dt;
        assert!(
            (next.velocity().vy - expected).abs() <= common::TOL,
            "vy[t={:.2}s] = {:.9} but vy + ay·dt = {expected:.9}",
            next.time,
            next.velocity().vy
        );
    }
}

// ─── Separation (SW-09, note from SW-03) ───

/// A pedestrian scenario must not resolve to zero separation.
///
/// `examples/pedestrian_crossing.yaml` declares both `min_ttc` and
/// `min_distance` as `ignore`, so nothing in the encoding forbids a collision;
/// what forbids it is that the two actors now have kinematics that cannot put
/// them in the same place. Recorded because SW-03 observed this same input
/// under a `MinimizeDistance` objective resolving to `min_distance = 0.00`
/// while reporting `all_constraints_satisfied = false`.
#[test]
fn test_pedestrian_scenarios_keep_a_nonzero_separation() {
    for (name, ped_id) in [
        ("pedestrian_crossing.yaml", "pedestrian"),
        ("pedestrian_running.yaml", "runner"),
        ("pedestrian_wide_road.yaml", "ped"),
    ] {
        let (scenario, _) = generate_from_file(name);
        let ped = scenario.get_actor(ped_id).expect("pedestrian actor");
        let ego = scenario.get_actor("ego").expect("ego actor");

        let closest = ped
            .states
            .iter()
            .zip(&ego.states)
            .map(|(p, e)| (p.position().x - e.position().x).hypot(p.position().y - e.position().y))
            .fold(f64::INFINITY, f64::min);

        assert!(
            closest > common::TOL,
            "{name}: pedestrian and ego coincide (closest approach {closest:.9} m)"
        );
    }
}
