//! SW-44 — the pedestrian safety box is sampled, so the ego could tunnel
//! through it between two steps.
//!
//! `RectangularDistanceGT` asserts `|dx| >= min_distance/2 OR |dy| >=
//! min_distance/1.5` at each discrete step. An ego at 20 m/s covers 10 m in a
//! `time_step: 0.5` against a longitudinal half-box of a metre or two, so it
//! can satisfy the box at every sampled step and still have driven straight
//! through the pedestrian in between — and `compute_validation_metrics`, which
//! sampled the same way, reported the result safe.
//!
//! Both halves are checked here, because a fix to either one alone leaves the
//! other unable to see the case: the encoder must not produce such a
//! trajectory, and the validator must flag one if it ever meets one.
//!
//! The specs are deliberately extreme — a single lane, a fast ego, a large
//! `min_distance` — because that is what makes the lateral escape hatch
//! unavailable and the longitudinal jump the only way through. Nothing about
//! the defect needs the extremes; they just make it reproducible in one solve.

mod common;

/// A 20 m/s ego on a single-lane road, sampled every 0.5 s: 10 m of travel per
/// step against a `threshold_x` of 2.5 m.
const FAST_EGO: &str = r#"
scenario_type: pedestrian_crossing
time_step: 0.5
duration: 8.0
road:
  num_lanes: 1
  lane_width: 3.5
  lane_directions: [1]
actors:
  - id: ego
    role: ego
    lane: 0
    position: 0.0
    speed: 20.0
    direction: 1
    acceleration: [-1.0, 1.0]
  - id: pedestrian
    role: pedestrian
    lane: 0
    position: [70.0, 90.0]
    speed: [0.8, 1.5]
    direction: 1
    acceleration: [-1.0, 1.0]
    behavior:
      walking_mode: walk
      direction: left_to_right
constraint_modes:
  min_ttc: ignore
  min_distance: MODE
min_ttc: 2.0
min_distance: MIN_DISTANCE
num_scenarios: 1
"#;

fn generate(mode: &str, min_distance: &str) -> scenario_weaver::scenario::model::Scenario {
    let yaml = FAST_EGO
        .replace("MODE", mode)
        .replace("MIN_DISTANCE", min_distance);
    let spec = scenario_weaver::dsl::parser::parse_yaml(&yaml)
        .unwrap_or_else(|e| panic!("cannot parse scenario YAML: {e}"));
    scenario_weaver::generate_single_scenario_from_spec(spec)
        .unwrap_or_else(|e| panic!("expected a solvable scenario, solver returned: {e}"))
}

/// The encoder half. With `min_distance: 5.0` enforced, `threshold_x` is 2.5 m
/// and `threshold_y` is 3.33 m; the pedestrian can clear the lateral half-box
/// (the far kerb is 3.75 m from the ego's lane centre) but only just, so the
/// cheapest way through used to be the longitudinal jump.
///
/// At the pre-SW-44 tree this fails at the step where `dx` goes
/// `-2.500 → +3.750` with `|dy|` = 3.105 then 3.500 — under the 3.333 m
/// threshold at the first of the two, so the ego crossed the pedestrian's
/// longitudinal position from inside the lateral half-box. Both sampled steps
/// satisfy the box: the first on `|dx| >= 2.5` exactly, the second on `|dy|`.
#[test]
fn test_the_ego_cannot_pass_the_pedestrian_through_the_safety_box() {
    let scenario = generate("enforce", "5.0");
    let threshold_y = 5.0 / 1.5;

    let ego = scenario.get_actor("ego").expect("ego trajectory");
    let ped = scenario
        .get_actor("pedestrian")
        .expect("pedestrian trajectory");

    let dx = |t: usize| ego.states[t].position().x - ped.states[t].position().x;
    let dy = |t: usize| ego.states[t].position().y - ped.states[t].position().y;

    let mut passes = 0_usize;
    for t in 0..ego.states.len() - 1 {
        // A strict sign change: the ego was behind the pedestrian at one
        // sample and in front of it at the next, so it passed *between* the
        // two and there is no sample of that instant. `dx == 0` at a sample is
        // not this case — the pass is then directly observed, and the box at
        // that step already demands the lateral clearance, which is why Z3 is
        // free to (and does) land there.
        if dx(t) * dx(t + 1) >= 0.0 {
            continue;
        }
        passes += 1;
        let clear = |d: f64| d.abs() >= threshold_y;
        assert!(
            clear(dy(t)) && clear(dy(t + 1)),
            "between t={t} and t={}: dx {:.3} → {:.3} crosses the pedestrian while dy \
             {:.3} → {:.3} is inside the {threshold_y:.3} m lateral half-box — the ego drove \
             through the safety box between two samples of it",
            t + 1,
            dx(t),
            dx(t + 1),
            dy(t),
            dy(t + 1)
        );
    }

    // If the ego never reached the pedestrian at all, the loop above proved
    // nothing. It does reach it — post-fix it passes with a sample landing
    // exactly on `dx == 0`, where the box at that step supplies the lateral
    // clearance itself and there is no unsampled instant to argue about; the
    // strict-crossing loop is therefore empty *because* the solver stopped
    // jumping the box, not because the ego stayed behind.
    let behind = (0..ego.states.len()).any(|t| dx(t) < 0.0);
    let ahead = (0..ego.states.len()).any(|t| dx(t) > 0.0);
    assert!(
        behind && ahead,
        "the ego never passed the pedestrian ({passes} strict crossings, behind={behind}, \
         ahead={ahead}), so no tunnelling was possible to rule out"
    );
    assert!(
        !scenario
            .validation
            .safety_violations
            .iter()
            .any(|v| v.contains("tunnelling")),
        "the validator disagrees with the encoder: {:?}",
        scenario.validation.safety_violations
    );
}

/// The validator half, on a trajectory the encoder is *allowed* to produce.
///
/// `min_distance: ignore` asserts no box at all, so nothing stops the ego
/// driving through the pedestrian — and it does. What used to be missing is
/// that `compute_validation_metrics` sampled the box exactly the way the
/// encoder asserts it, so it reported the result safe. With `min_distance:
/// 6.0` the lateral half-box is 4.0 m and the far kerb is 3.75 m from the
/// ego's lane centre, so no lateral clearance exists anywhere on this road and
/// every crossing of the pedestrian's `px` is a tunnelling event.
///
/// At the pre-SW-44 tree this fails: `safety_violations` is empty.
#[test]
fn test_a_tunnelling_trajectory_is_reported_by_the_validator() {
    let scenario = generate("ignore", "6.0");

    let tunnelling: Vec<_> = scenario
        .validation
        .safety_violations
        .iter()
        .filter(|v| v.contains("tunnelling"))
        .collect();

    assert!(
        !tunnelling.is_empty(),
        "the ego passed the pedestrian with no lateral clearance available on this road, \
         and the validator reported no tunnelling: {:?}",
        scenario.validation.safety_violations
    );
}

/// The `Violate` half of the pedestrian constraint-mode invariant
/// (`tests/common/invariants.rs::check_pedestrian_constraint_modes`, rewritten
/// under SW-44 to read the propositions `pedestrian_crossing.rs` actually
/// lowers rather than a same-lane longitudinal scalar that means nothing for a
/// perpendicular crossing).
///
/// Nothing in the suite generated a `violate`d pedestrian scenario through the
/// shared invariant set, so that arm had never been exercised. Here it is:
/// `generate_or_fail` runs the whole invariant set, and the run fails unless
/// `compute_validation_metrics` reports a real breach of the distance box.
#[test]
fn test_a_violated_pedestrian_min_distance_is_a_breach_the_validator_reports() {
    let yaml = FAST_EGO
        .replace("MODE", "violate")
        .replace("MIN_DISTANCE", "2.0");
    let scenario = common::generate_or_fail(&yaml);

    assert!(
        scenario
            .validation
            .safety_violations
            .iter()
            .any(|v| v.contains("Pedestrian distance-box violation")),
        "expected a reported box breach, got {:?}",
        scenario.validation.safety_violations
    );
}
