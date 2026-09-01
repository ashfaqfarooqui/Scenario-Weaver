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
use scenario_weaver::dsl::types::{PEDESTRIAN_RUN_MAX_SPEED, PEDESTRIAN_WALK_MAX_SPEED};
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
    assert_eq!(ped.role, "pedestrian");
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
    assert_eq!(ped.role, "pedestrian");
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
    assert_eq!(ped.role, "pedestrian");

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
/// Fails: the pedestrian's `py` runs a full lane width outside the lane the
/// extractor records for it, and on the wide-road example it leaves the road
/// surface entirely (finding E3).
#[test]
#[ignore = "SW-10: the lane variable lags lateral position, so a crossing pedestrian is recorded in a lane it is not in"]
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
