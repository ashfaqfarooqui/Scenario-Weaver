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

/// SW-45: the authored `speed:` governs the *crossing* (the lateral velocity
/// `vy`), not an along-road drift, and the along-road velocity `vx` of a
/// crossing pedestrian is ≈0.
///
/// Before the fix the crossing ran at the raw walk cap (2.0 m/s) regardless of
/// the authored `speed:` [0.8, 1.5], the authored speed was pinned to `vx`
/// where it decayed to a standstill and could sign-flip (a pedestrian walking
/// backwards along the road), and `vy` was free of the authored bound
/// entirely. Concretely, before: `vx: 0.8 → 0.5 → 0.2 → 0 …` and
/// `vy: 2.0 → 2.0 → 2.0 …` peaking at 2.0.
#[test]
fn test_pedestrian_crossing_speed_is_the_lateral_crossing_speed() {
    let (scenario, spec) = generate_from_file("pedestrian_crossing.yaml");
    let ped_spec = spec
        .actors
        .iter()
        .find(|a| a.role == ActorRole::Pedestrian)
        .expect("pedestrian in spec");
    let authored_max = ped_spec.speed.max();
    let authored_min = ped_spec.speed.min();

    let ped = scenario.get_actor("pedestrian").expect("pedestrian actor");

    // vx ≈ 0 at every step: no along-road drift.
    for s in &ped.states {
        assert!(
            s.velocity().vx.abs() <= common::TOL,
            "vx should be ≈0 for a crossing pedestrian, got {:.6} at t={:.2}",
            s.velocity().vx,
            s.time
        );
    }

    // The crossing speed lives on vy: it never exceeds the authored max (it
    // used to sit at the 2.0 walk cap, above the authored 1.5), and the sign
    // follows the left_to_right crossing direction (vy ≥ 0 — never backwards).
    for s in &ped.states {
        assert!(
            s.velocity().vy >= -common::TOL,
            "left_to_right crossing must not move backwards: vy={:.6} at t={:.2}",
            s.velocity().vy,
            s.time
        );
        assert!(
            s.velocity().vy.abs() <= authored_max + common::TOL,
            "crossing speed vy={:.6} exceeds the authored max {authored_max} at t={:.2} \
             (the pre-fix trajectory ran at the 2.0 walk cap)",
            s.velocity().vy,
            s.time
        );
    }

    // And the crossing actually happens at the authored speed: the peak |vy|
    // reaches into the authored band, rather than the crossing being carried
    // by vx.
    let peak_vy = ped
        .states
        .iter()
        .map(|s| s.velocity().vy.abs())
        .fold(0.0_f64, f64::max);
    assert!(
        peak_vy >= authored_min - common::TOL,
        "the crossing speed vy should reach the authored band [{authored_min}, {authored_max}]; \
         peak |vy| was only {peak_vy:.6}"
    );
}

/// SW-46: a pedestrian who crosses the road must *arrive at the far kerb and
/// stay there*, not graze it for one step and drift back into the road.
///
/// Before the fix the crossing goal was `F(OnSidewalk(far))`, which pins the
/// far kerb at one step and leaves `py` free everywhere else: the shipped
/// `pedestrian_crossing` reached `py ≈ 7.0` for a single step and walked back to
/// `py ≈ 4.78` — a standstill in the *middle* of a `[0, 7]` road. This asserts
/// the pedestrian ends past the far-kerb centre (`road_width + SIDEWALK/2`) and
/// is settled there across the tail, for all three examples.
#[test]
fn test_pedestrian_arrives_and_settles_on_the_far_kerb() {
    use scenario_weaver::solver::encoder::SIDEWALK_WIDTH;

    for (file, ped_id) in [
        ("pedestrian_crossing.yaml", "pedestrian"),
        ("pedestrian_running.yaml", "runner"),
        ("pedestrian_wide_road.yaml", "ped"),
    ] {
        let (scenario, spec) = generate_from_file(file);
        let road_width = spec.get_lane_width() * spec.get_num_lanes() as f64;
        // All three examples cross left_to_right, so the far kerb is the right
        // one and the kerb centre is at road_width + SIDEWALK/2.
        let kerb_centre = road_width + SIDEWALK_WIDTH / 2.0;

        let ped = scenario.get_actor(ped_id).expect("pedestrian actor");
        let py_end = ped.states.last().expect("states").position().y;

        // Arrived: the final position is past the far-kerb centre, not merely a
        // hair onto the sidewalk and not back in the road.
        assert!(
            py_end >= kerb_centre - common::TOL,
            "{file}: pedestrian must end past the far-kerb centre {kerb_centre:.3}, \
             ended at py={py_end:.3} (pre-fix it drifted back to mid-road)"
        );

        // Settled: the tail does not dip back below the kerb centre.
        let tail = 2usize.min(ped.states.len());
        for s in &ped.states[ped.states.len() - tail..] {
            assert!(
                s.position().y >= kerb_centre - common::TOL,
                "{file}: pedestrian left the far kerb before the end: py={:.3} < {kerb_centre:.3} \
                 at t={:.2}",
                s.position().y,
                s.time
            );
        }
    }
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

    // `walking_mode: run` raises the crossing-speed ceiling above the walking
    // cap. Post-SW-45 the crossing speed lives on `vy` (not `vx`), so the run
    // ceiling applies to `vy`.
    //
    // The previous form of this test asserted `speed() > walk_cap` strictly.
    // That only passed because of the SW-45 bug it is now independent of: the
    // authored speed used to be pinned to `vx` as well, so `speed() =
    // hypot(vx, vy)` reached ~2.83 even when `vy` sat at the 2.0 cap. With `vx`
    // now correctly ≈0, the runner (authored `[2.0, 3.0]`, floor == the 2.0
    // walk cap) may legitimately cross at exactly its authored floor — Z3 is
    // not obliged to spend the full authored ceiling over a generous horizon.
    // So this asserts the crossing reaches the authored floor and never
    // exceeds the *run* ceiling; that the run ceiling is genuinely raised
    // above the walk cap (a walker would be clamped) is proved directly in
    // `encoders::pedestrian`'s `test_bounds_step_running_mode_higher_speed`.
    let runner_spec = spec
        .actors
        .iter()
        .find(|a| a.role == ActorRole::Pedestrian)
        .expect("runner in spec");
    let authored_min = runner_spec.speed.min();
    let authored_max = runner_spec.speed.max();
    assert!(
        authored_max > PEDESTRIAN_WALK_MAX_SPEED,
        "pedestrian_running must author a speed above the walk cap for run mode to matter"
    );

    let runner = scenario.get_actor("runner").expect("runner actor");
    let fastest_vy = runner
        .states
        .iter()
        .map(|s| s.velocity().vy.abs())
        .fold(0.0_f64, f64::max);
    assert!(
        fastest_vy <= PEDESTRIAN_RUN_MAX_SPEED + common::TOL,
        "runner crossing speed {fastest_vy:.4} exceeds the running ceiling"
    );
    assert!(
        fastest_vy >= authored_min - common::TOL,
        "the runner should cross at its authored speed (>= {authored_min}); \
         fastest |vy| was {fastest_vy:.4}"
    );
    // vx carries no speed any more.
    assert!(
        runner
            .states
            .iter()
            .all(|s| s.velocity().vx.abs() <= common::TOL),
        "runner vx should be ≈0 (crossing speed is on vy)"
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

// ─── Braking to rest at the horizon (SW-40) ───

/// A declared braking manoeuvre whose ego is at rest at the last step.
///
/// `acceleration: [-5.0, -1.9]` is strictly negative, so the spec itself
/// requires the ego to shed at least 1.9 m/s² at *every* step: from 20 m/s it
/// arrives at the horizon at 1 m/s at the very fastest, and the trajectory the
/// solver returns brakes linearly to exactly 0. Everything else here is
/// ordinary — `min_ttc` and `min_distance` are enforced, the ego stops 10 m
/// short of a pedestrian crossing 110 m ahead.
const BRAKE_TO_REST_YAML: &str = r"
scenario_type: pedestrian_crossing
time_step: 0.5
duration: 10.0
road:
  num_lanes: 2
  lane_width: 3.5
  lane_directions: [1, 1]
actors:
  - id: ego
    role: ego
    lane: 0
    position: 0.0
    speed: 20.0
    direction: 1
    acceleration: [-5.0, -1.9]
  - id: ped
    role: pedestrian
    lane: 0
    position: 110.0
    speed: [0.8, 1.4]
    direction: 1
    acceleration: [-0.8, 0.8]
    behavior:
      walking_mode: walk
      direction: left_to_right
min_ttc: 1.5
min_distance: 1.5
num_scenarios: 1
";

/// SW-40: the SW-39 terminal-speed floor must not forbid a stop the spec asks for.
///
/// `TERMINAL_SPEED_FRACTION` requires `vx[H] >= 0.5 * speed.min()` — 10 m/s
/// here — of every vehicle, on the argument that an emergency stop is "a dip,
/// not an ending state". That argument covers a stop in the middle of the
/// horizon and not one at it. This spec's declared acceleration band forces the
/// second: the fastest reachable terminal speed is
/// `speed.max() + acceleration.max() * duration` = `20 - 1.9 * 10` = 1 m/s, so
/// the floor contradicts the declared dynamics rather than rejecting an
/// implausible trajectory, and the whole spec was UNSAT.
///
/// The SW-12 displacement floor is *not* what excluded it and is still checked
/// here: the triangular profile covers 100 m against the 100 m that floor
/// demands. Verified against the pre-fix encoder — with the terminal bound
/// asserted unconditionally this spec is UNSAT, with only the displacement
/// floor it is SAT.
#[test]
fn test_declared_braking_manoeuvre_may_end_at_rest() {
    let scenario = common::generate_or_fail(BRAKE_TO_REST_YAML);
    let ego = scenario.get_actor("ego").expect("ego actor");

    let initial = ego.states[0].velocity().vx;
    let final_vx = ego.states.last().expect("nonempty").velocity().vx;
    let travelled = ego.states.last().expect("nonempty").position().x - ego.states[0].position().x;

    assert!(
        final_vx <= 1.0 + common::TOL,
        "the declared acceleration band caps the terminal speed at 1.0 m/s, got {final_vx} \
         (vx series: {:?})",
        ego.states
            .iter()
            .map(|s| s.velocity().vx)
            .collect::<Vec<_>>()
    );
    assert!(
        travelled >= 0.5 * initial * scenario.duration - common::TOL,
        "the SW-12 displacement floor still applies: {travelled} m covered, \
         {} m required",
        0.5 * initial * scenario.duration
    );
}
