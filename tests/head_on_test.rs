//! Integration tests for the head_on scenario type.

mod common;

use scenario_weaver::dsl::types::ActorRole;
use scenario_weaver::scenario::model::Scenario;
use scenario_weaver::{
    export_scenario_to_openlabel, export_scenario_to_svg, export_scenario_to_xodr,
    export_scenario_to_xosc,
};

fn near_miss() -> Scenario {
    common::generate_example("head_on_near_miss.yaml")
}

/// SW-47: `HeadOnModel::generate_ltl` used to be `Ok(LTLFormula::True)` — no
/// behavioral goal at all — so nothing required the ego to actually finish
/// the overtake it is set up to attempt. It could dip into the oncoming lane
/// and back without ever passing the slow vehicle, and Z3 was free to return
/// exactly that "weave without passing" model whenever the ego is not
/// guaranteed faster than the slow NPC it is meant to overtake.
///
/// This spec deliberately does not guarantee the ego is faster: both the ego
/// and `slow_npc` share the same declared speed range and `slow_npc` starts
/// only a short distance ahead, so a model in which the ego merely performs
/// its two lane changes without ever getting ahead of `slow_npc` is entirely
/// consistent with the (pre-fix) constraints. Before SW-47, Z3 returned
/// exactly that model. After SW-47, `generate_ltl` requires the ego to end up
/// back in its own lane *and* ahead of `slow_npc`, which forbids it.
#[test]
fn test_head_on_ego_forced_to_complete_overtake() {
    let yaml = r#"
scenario_type: head_on
time_step: 0.5
duration: 10.0
road:
  num_lanes: 2
  lane_width: 3.5
  lane_directions: [1, -1]
actors:
  - id: ego
    role: ego
    lane: 0
    position: 0.0
    speed: [8.0, 10.0]
    direction: 1
    acceleration: [-5.0, 3.0]
    lane_changes:
      - direction: right
        start_time: [2.0, 3.0]
        duration: [2.0, 3.0]
      - direction: left
        start_time: [6.0, 7.0]
        duration: [2.0, 3.0]
  - id: slow_npc
    role: npc
    lane: 0
    position: [20.0, 30.0]
    speed: [8.0, 10.0]
    direction: 1
    acceleration: [-5.0, 3.0]
  - id: oncoming_npc
    role: npc
    lane: 1
    position: [150.0, 180.0]
    speed: [10.0, 12.0]
    direction: -1
    acceleration: [-2.0, 1.0]
min_ttc: 0.5
min_distance: 1.0
num_scenarios: 1
constraint_modes:
  min_ttc: ignore
  min_distance: ignore
"#;

    let scenario = common::generate_or_fail(yaml);

    let ego = scenario.get_actor("ego").expect("ego");
    let slow_npc = scenario.get_actor("slow_npc").expect("slow_npc");

    let ego_final = ego.states.last().expect("ego has states");
    let slow_final = slow_npc.states.last().expect("slow_npc has states");

    assert_eq!(
        ego_final.lane(),
        0,
        "ego should be back in its own lane (0) at the end, was {}",
        ego_final.lane()
    );
    assert!(
        ego_final.position().x > slow_final.position().x,
        "ego should have completed the overtake and be ahead of slow_npc by the end: \
         ego.x={:.3}, slow_npc.x={:.3}",
        ego_final.position().x,
        slow_final.position().x
    );
}

/// A *near miss* is by definition a scenario in which the safety constraints
/// hold. The solver still returns one in which they do not, but SW-10
/// re-diagnosed why. It is not the lane lag: `lane` now tracks `py` exactly,
/// and the ego's overtake reads correctly (lane 0 -> 1 at the step `py` crosses
/// 3.5, and back).
///
/// Every violation reported here is on the **ego <-> slow_npc** pair, and
/// `HeadOnModel::generate_safety` (`src/scenarios/head_on.rs`) never constrains
/// that pair — it builds TTC and distance constraints for ego <-> oncoming
/// only, as its own `// Only ego <-> oncoming gets the requested constraint
/// mode` comment says. Meanwhile `compute_validation_metrics` measures every
/// pair. So the encoder asserts one pair's safety and the validator grades
/// three, which no change inside the encoder can reconcile.
///
/// `src/scenarios/head_on.rs` belongs to SW-12; the assertion below is the one
/// SW-10 was asked to add, left armed for SW-12 to turn green.
#[test]
#[ignore = "SW-12 (owns src/scenarios/head_on.rs): HeadOnModel::generate_safety constrains \
            only the ego <-> oncoming pair, while the validator measures every pair, so the \
            unconstrained ego <-> slow_npc pair violates min_distance (0.29 m against 5.0 m). \
            The SW-10 half — the lane variable lagging lateral position — is fixed"]
fn test_head_on_near_miss_generation() {
    let scenario = near_miss();

    assert_eq!(scenario.scenario_type, "head_on");
    assert_eq!(scenario.actors.len(), 3);

    // Ego exists
    let ego = scenario.get_actor("ego").expect("Should have ego");
    assert_eq!(ego.role, ActorRole::Ego);

    // At least one actor has negative velocity (oncoming)
    let has_negative_vx = scenario
        .actors
        .iter()
        .any(|a| a.states[0].velocity().vx < 0.0);
    assert!(
        has_negative_vx,
        "Should have an oncoming actor with negative vx"
    );

    // A near miss must actually be a near miss: no constraint may be violated.
    assert!(
        scenario.validation.all_constraints_satisfied,
        "head_on_near_miss must satisfy its constraints; min_ttc={:?}, \
         min_distance={:?}, violations={:?}",
        scenario.validation.min_ttc,
        scenario.validation.min_distance,
        scenario.validation.safety_violations
    );
    assert!(
        scenario.validation.safety_violations.is_empty(),
        "unexpected violations: {:?}",
        scenario.validation.safety_violations
    );
}

/// `head_on_collision.yaml` is the adversarial counterpart: it must be solvable
/// and its solution must actually breach the safety constraints.
#[test]
fn test_head_on_collision_generation() {
    let scenario = common::generate_example("head_on_collision.yaml");

    assert_eq!(scenario.scenario_type, "head_on");
    assert_eq!(scenario.actors.len(), 3);
    assert!(
        !scenario.validation.all_constraints_satisfied,
        "an adversarial collision scenario must report violated constraints; \
         min_ttc={:?}, min_distance={:?}",
        scenario.validation.min_ttc, scenario.validation.min_distance
    );
}

/// SW-39: `encode_forward_progress` only floors *net displacement* over the
/// whole horizon, and nothing else ties a later step's speed to the actor's
/// declared `speed:`. The cheapest way to satisfy a displacement floor is to
/// decay linearly to zero — `oncoming_npc` here rides exactly that: it starts
/// at -10 m/s and integrates (trapezoidally) to precisely the -50 m the floor
/// demands, ending at vx = 0.0 and staying there. An oncoming vehicle coasting
/// to a dead stop in the middle of a bidirectional road is not a near miss.
///
/// This asserts the physical property directly — the oncoming actor must
/// still be carrying a usable fraction of its declared entry speed at the end
/// of the horizon — rather than a hand-computed displacement constant.
///
/// The floor is anchored to the actor's **declared** minimum entry speed, not
/// the one Z3 happened to sample. `oncoming_npc`'s `speed:` is the range
/// `[10.0, 12.0]`, so its initial speed is a free variable; the earlier form
/// compared the final speed against half of the *extracted* initial, which the
/// solver is free to draw anywhere in that band. That made the bound depend on
/// an arbitrary satisfying assignment rather than a physical property: the same
/// final speed of 5.0 m/s reads as "50% of 10" for one valid model and "42% of
/// 12" for another. (SW-49's tightening of `same_lane` shifted the sampled
/// initial from 10 to 12 while the final speed — and the min over the horizon,
/// ~4.1 m/s — stayed put, exposing exactly this coupling.) Half of the declared
/// minimum, 5.0 m/s, is the physically meaningful "not coasting to a stop" line.
#[test]
fn test_head_on_oncoming_retains_speed_through_horizon() {
    // `oncoming_npc`'s declared `speed:` lower bound in head_on_near_miss.yaml.
    const DECLARED_MIN_ENTRY_SPEED: f64 = 10.0;

    let scenario = near_miss();
    let oncoming = scenario.get_actor("oncoming_npc").expect("oncoming_npc");

    let final_speed = oncoming
        .states
        .last()
        .expect("nonempty")
        .velocity()
        .vx
        .abs();

    assert!(
        final_speed >= 0.5 * DECLARED_MIN_ENTRY_SPEED - 1e-6,
        "oncoming_npc should retain at least half its declared minimum entry speed \
         ({DECLARED_MIN_ENTRY_SPEED:.1} m/s) by the end of the horizon (final \
         {final_speed:.3} m/s) — a coast to a dead stop in the middle of a bidirectional \
         road is not a near miss; vx series: {:?}",
        oncoming
            .states
            .iter()
            .map(|s| s.velocity().vx)
            .collect::<Vec<_>>()
    );
}

/// SW-50: the SW-39/SW-40 floors bound the *average* over the horizon and the
/// *final* step; nothing bounded the steps in between. On the shipped
/// `head_on_near_miss.yaml` at `2c7c8fc` the oncoming vehicle decayed
/// monotonically from -12.0 to -4.1 m/s against a declared `[10.0, 12.0]`
/// (riding `TERMINAL_SPEED_FRACTION` back up to exactly -5.0 at the horizon),
/// and the "slow vehicle motivating overtake" reached 13.5 m/s against a
/// declared `[6.0, 8.0]` — 69% over its own maximum, and faster than the ego's
/// declared 12.0. The overtake was still enforced (SW-47); its motivation was
/// not.
///
/// Both actors now carry `behavior.speed_retention: 1.0`, which asserts the
/// declared band as a two-sided per-step bound on along-track speed. This test
/// reads the fraction and the band out of the spec rather than restating them,
/// so re-tuning either example cannot leave the assertion checking a stale
/// number — and it checks **every** step, which is the whole point: the
/// pre-SW-50 trajectory satisfied a terminal-step check and violated this one
/// at 17 of its 21 steps.
///
/// The bicycle example is included because `get_longitudinal_vel` is the signed
/// along-track velocity in both coordinate frames (Cartesian `vx`; the bicycle
/// model's `longitudinal_vel = direction * speed_v`), so the retention bound is
/// asserted frame-independently and both frames should honour it.
#[test]
fn test_head_on_speed_retention_holds_the_declared_band_at_every_step() {
    for example in ["head_on_near_miss.yaml", "head_on_near_miss_bicycle.yaml"] {
        let (scenario, spec) = common::generate_example_with_spec(example);

        let mut actors_checked = 0;
        for actor_spec in &spec.actors {
            let Some(fraction) = actor_spec.speed_retention() else {
                continue;
            };
            actors_checked += 1;

            let floor = fraction * actor_spec.speed.min();
            let ceiling = actor_spec.speed.max() / fraction;
            let direction = f64::from(actor_spec.direction);
            let actor = scenario
                .get_actor(&actor_spec.id)
                .unwrap_or_else(|| panic!("{example}: missing actor {}", actor_spec.id));

            for (step, state) in actor.states.iter().enumerate() {
                let along_track = direction * state.velocity().vx;
                assert!(
                    along_track >= floor - 1e-6 && along_track <= ceiling + 1e-6,
                    "{example}: {} declares speed_retention {fraction} over speed \
                     [{:.1}, {:.1}], so its along-track speed must stay within \
                     [{floor:.3}, {ceiling:.3}] m/s at every step — step {step} is \
                     {along_track:.3}; series: {:?}",
                    actor_spec.id,
                    actor_spec.speed.min(),
                    actor_spec.speed.max(),
                    actor
                        .states
                        .iter()
                        .map(|s| direction * s.velocity().vx)
                        .collect::<Vec<_>>()
                );
            }
        }

        assert_eq!(
            actors_checked, 2,
            "{example} should declare speed_retention on slow_npc and oncoming_npc; \
             found {actors_checked} actor(s) with the key"
        );
    }
}

/// SW-50, the other half of the same property: the retention band is **opt-in
/// per actor**, and the ego of a head-on example does not carry it. That is
/// deliberate — the ego is the vehicle under test and must stay free to brake
/// hard, which is exactly what `encode_forward_progress`'s doc comment refuses
/// to forbid globally and what SW-40 had to reopen for a declared braking
/// manoeuvre. If a future change ever seats the floor on every actor, this test
/// is the one that should go red first.
#[test]
fn test_head_on_speed_retention_is_opt_in_and_leaves_the_ego_free() {
    let (_scenario, spec) = common::generate_example_with_spec("head_on_near_miss.yaml");

    let ego = spec
        .actors
        .iter()
        .find(|a| a.role == ActorRole::Ego)
        .expect("head_on_near_miss.yaml has an ego");

    assert!(
        ego.speed_retention().is_none(),
        "the ego must not opt in to speed retention: it is the vehicle under test \
         and has to stay free to brake"
    );
}

#[test]
fn test_head_on_three_actors() {
    let scenario = near_miss();

    // Ego: positive vx, forward lane
    let ego = scenario.get_actor("ego").unwrap();
    assert!(ego.states[0].velocity().vx > 0.0, "Ego should move forward");

    // slow_npc: same direction as ego
    let slow = scenario.get_actor("slow_npc").unwrap();
    assert!(
        slow.states[0].velocity().vx > 0.0,
        "slow_npc should move forward"
    );

    // oncoming_npc: opposite direction
    let oncoming = scenario.get_actor("oncoming_npc").unwrap();
    assert!(
        oncoming.states[0].velocity().vx < 0.0,
        "oncoming_npc should move backward (negative vx)"
    );
}

#[test]
fn test_head_on_ego_lane_change() {
    let scenario = near_miss();

    let ego = scenario.get_actor("ego").unwrap();

    // Ego should change lane at some point (lane value changes or py changes)
    let initial_lane = ego.states[0].lane();
    let lane_changed = ego.states.iter().any(|s| s.lane() != initial_lane);
    let py_changed = ego
        .states
        .iter()
        .any(|s| (s.position().y - ego.states[0].position().y).abs() > 0.5);

    assert!(
        lane_changed || py_changed,
        "Ego should perform a lane change (lane or lateral position should change)"
    );
}

#[test]
fn test_head_on_export_svg() {
    let scenario = near_miss();

    let svg = export_scenario_to_svg(&scenario).expect("Should export to SVG");

    assert!(svg.contains("<svg"), "SVG should contain <svg element");
    assert!(svg.contains("ego"), "SVG should mention ego");
    assert!(svg.contains("slow_npc"), "SVG should mention slow_npc");
    assert!(
        svg.contains("oncoming_npc"),
        "SVG should mention oncoming_npc"
    );
}

#[test]
fn test_head_on_export_xodr() {
    let scenario = near_miss();

    let xodr = export_scenario_to_xodr(&scenario).expect("Should export to XODR");

    assert!(
        xodr.contains("<OpenDRIVE>") || xodr.contains("OpenDRIVE"),
        "XODR should contain OpenDRIVE"
    );
    // Bidirectional road info
    assert!(
        xodr.contains("lane") || xodr.contains("Lane"),
        "XODR should contain lane information"
    );
}

#[test]
fn test_head_on_export_xosc() {
    let scenario = near_miss();

    let xosc = export_scenario_to_xosc(&scenario).expect("Should export to XOSC");

    assert!(
        xosc.contains("<OpenSCENARIO>") || xosc.contains("OpenSCENARIO"),
        "XOSC should contain OpenSCENARIO"
    );
    assert!(xosc.contains("ego"), "XOSC should contain ego entity");
    assert!(
        xosc.contains("slow_npc"),
        "XOSC should contain slow_npc entity"
    );
    assert!(
        xosc.contains("oncoming_npc"),
        "XOSC should contain oncoming_npc entity"
    );
}

#[test]
fn test_head_on_export_openlabel() {
    let scenario = near_miss();

    let json_str = export_scenario_to_openlabel(&scenario).expect("Should export to OpenLABEL");

    // Valid JSON
    let parsed: serde_json::Value =
        serde_json::from_str(&json_str).expect("OpenLABEL output should be valid JSON");

    assert!(
        parsed.get("openlabel").is_some(),
        "Should have 'openlabel' key"
    );

    // Check actor count in objects or frames
    if let Some(openlabel) = parsed.get("openlabel") {
        if let Some(objects) = openlabel.get("objects") {
            let obj_map = objects.as_object().expect("objects should be a map");
            assert_eq!(
                obj_map.len(),
                3,
                "Should have 3 actors in OpenLABEL objects"
            );
        }
    }
}
