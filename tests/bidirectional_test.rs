//! Integration tests for bidirectional traffic scenarios

mod common;

use scenario_weaver::dsl::types::ActorRole;

#[test]
fn test_simple_bidirectional_scenario() {
    // Test basic bidirectional road with forward lanes
    let yaml = r#"
scenario_type: cut_in_left
time_step: 0.5
duration: 10.0

road:
  num_lanes: 4
  lane_width: 3.5
  lane_directions: [1, 1, -1, -1]

actors:
  - id: ego
    role: ego
    lane: 1
    position: 50.0
    speed: 15.0
    direction: 1
    acceleration: [-8.0, 3.0]

  - id: npc
    role: npc
    lane: 0
    position: [60.0, 80.0]
    speed: [12.0, 14.0]
    direction: 1
    acceleration: [-8.0, 3.0]
    lane_changes:
      - direction: right
        start_time: [2.5, 7.5]
        duration: [3.0, 4.0]

min_ttc: 3.0
min_distance: 5.0
num_scenarios: 1
"#;

    let scenario = common::generate_or_fail(yaml);

    // Verify basic properties
    assert_eq!(scenario.actors.len(), 2);
    assert_eq!(scenario.time_step, 0.5);
    assert_eq!(scenario.duration, 10.0);

    // Verify ego trajectory
    let ego = scenario.actors.iter().find(|a| a.id == "ego").unwrap();
    assert_eq!(ego.role, ActorRole::Ego);

    // All ego velocities should be positive (forward lane)
    for state in &ego.states {
        assert!(
            state.velocity().vx >= 0.0,
            "Ego velocity should be non-negative in forward lane, got {}",
            state.velocity().vx
        );
    }

    // Verify NPC trajectory
    let npc = scenario.actors.iter().find(|a| a.id == "npc").unwrap();
    assert_eq!(npc.role, ActorRole::Npc);

    // All NPC velocities should be positive (forward lane)
    for state in &npc.states {
        assert!(
            state.velocity().vx >= 0.0,
            "NPC velocity should be non-negative in forward lane, got {}",
            state.velocity().vx
        );
    }

    // Verify safety constraints are satisfied
    assert!(
        scenario.validation.all_constraints_satisfied,
        "Safety constraints should be satisfied"
    );
}

#[test]
fn test_backward_lane_velocity() {
    // Test vehicle in backward lane has negative velocity
    let yaml = r#"
scenario_type: cut_in_left
time_step: 0.5
duration: 10.0

road:
  num_lanes: 4
  lane_width: 3.5
  lane_directions: [1, 1, -1, -1]

actors:
  - id: ego
    role: ego
    lane: 1
    position: 50.0
    speed: 15.0
    direction: 1
    acceleration: [-8.0, 3.0]

  - id: npc
    role: npc
    lane: 2
    position: 150.0
    speed: 16.0
    direction: -1
    acceleration: [-2.0, 0.0]
    lane_changes:
      - direction: right
        start_time: [2.5, 7.5]
        duration: [3.0, 4.0]

min_ttc: 3.0
min_distance: 5.0
num_scenarios: 1
# SW-28. This used to put the ego in lane 0 while the NPC's lane change
# (lane 2 -> lane 1) never reached it, so the pair never shared a lane at
# all and neither metric was ever evaluated — exactly the defect
# `ScenarioSpec::validate` now rejects at parse time (see the check next to
# the lane-change target computation in `dsl::types::ScenarioSpec::validate`).
# The ego moved to lane 1 so the NPC's merge actually lands on it, which is
# now a real (if oncoming) cut-in.
#
# That leaves the same structural conflict `test_lane_direction_consistency`
# and `test_narrow_rural_road` document: an oncoming pair forced to cross by
# `ForwardProgress` cannot carry a non-vacuous `enforce`d `min_ttc` (the gap
# hits zero while still closing, the only escape is not sharing the lane
# while approaching, and `TTCGT` is undefined there either way) — asserting
# it made this UNSAT (measured). `min_distance` is unaffected by that and
# stays `enforce`d, as in both sibling tests.
constraint_modes:
  min_ttc: ignore
"#;

    let scenario = common::generate_or_fail(yaml);

    // Verify ego (forward lane)
    let ego = scenario.actors.iter().find(|a| a.id == "ego").unwrap();
    for (i, state) in ego.states.iter().enumerate() {
        assert!(
            state.velocity().vx >= 0.0,
            "Ego velocity at t={} should be non-negative (forward lane), got {}",
            i,
            state.velocity().vx
        );
    }

    // Verify NPC (backward lane)
    let npc = scenario.actors.iter().find(|a| a.id == "npc").unwrap();
    for (i, state) in npc.states.iter().enumerate() {
        assert!(
            state.velocity().vx <= 0.0,
            "NPC velocity at t={} should be non-positive (backward lane), got {}",
            i,
            state.velocity().vx
        );
    }
}

#[test]
fn test_lane_direction_consistency() {
    // Test that velocity direction is consistent with lane direction throughout trajectory
    let yaml = r#"
scenario_type: cut_in_left
time_step: 0.5
duration: 10.0

road:
  num_lanes: 2
  lane_width: 3.0
  lane_directions: [1, -1]

actors:
  - id: ego
    role: ego
    lane: 0
    position: 50.0
    speed: 20.0
    direction: 1
    acceleration: [-8.0, 3.0]

  - id: npc
    role: npc
    lane: 1
    position: 150.0
    speed: 18.0
    direction: -1
    acceleration: [-2.0, 0.0]
    lane_changes:
      - direction: right
        start_time: [2.5, 7.5]
        duration: [3.0, 4.0]

min_ttc: 3.0
min_distance: 5.0
num_scenarios: 1
# SW-22. These three specs are oncoming pairs, and an `enforce`d `min_ttc` on an
# oncoming pair that must cross is only satisfiable *vacuously* — which is the
# defect SW-22 removed, so it now shows up as an
# `Invariant::ConstraintModes` breach rather than passing silently.
#
# Derivation, not trial and error: forward progress (SW-12) requires each vehicle
# to cover `0.5 * speed.min() * duration`, so over the 10 s horizon the ego must
# travel at least half its declared speed x 10 s, and likewise the NPC,
# toward each other. Their initial separation is smaller than that sum, so they
# *must* cross. While they share a lane, `min_ttc >= 3 s` demands
# `gap >= 3 * closing`, and at the crossing the gap goes to zero with the closing
# speed still positive — so the only models left are the ones where the pair does
# not share a lane while approaching, and then TTC is never defined. Both
# outcomes fail `enforce`; there is no satisfying model in between. Asserting the
# conflict anyway made these UNSAT (measured).
#
# `ignore` is the mode this spec can actually honour. Nothing these tests assert
# depends on it: they check that velocity sign follows lane direction. The
# `enforce` was default boilerplate that had never been evaluated on this spec.
constraint_modes:
  min_ttc: ignore
"#;

    let scenario = common::generate_or_fail(yaml);

    // Verify velocity signs throughout entire trajectory
    let ego = scenario.actors.iter().find(|a| a.id == "ego").unwrap();
    let npc = scenario.actors.iter().find(|a| a.id == "npc").unwrap();

    // Ego in forward lane (0) - all positive velocities
    assert!(
        ego.states.iter().all(|s| s.velocity().vx >= 0.0),
        "Ego should maintain positive velocity in forward lane"
    );

    // NPC in backward lane (1) - all negative velocities
    assert!(
        npc.states.iter().all(|s| s.velocity().vx <= 0.0),
        "NPC should maintain negative velocity in backward lane"
    );
}

#[test]
fn test_multiple_bidirectional_scenarios() {
    // Test generating multiple diverse bidirectional scenarios
    let yaml = r#"
scenario_type: cut_in_left
time_step: 0.5
duration: 10.0

road:
  num_lanes: 4
  lane_width: 3.5
  lane_directions: [1, 1, -1, -1]

actors:
  - id: ego
    role: ego
    lane: 1
    position: [45.0, 55.0]
    speed: [14.0, 16.0]
    direction: 1
    acceleration: [-8.0, 3.0]

  - id: npc
    role: npc
    lane: 0
    position: [60.0, 80.0]
    speed: [12.0, 14.0]
    direction: 1
    acceleration: [-8.0, 3.0]
    lane_changes:
      - direction: right
        start_time: [2.5, 7.5]
        duration: [3.0, 4.0]

min_ttc: 3.0
min_distance: 5.0
num_scenarios: 3
"#;

    let scenarios = common::generate_multiple_or_fail(yaml, 3);
    assert_eq!(scenarios.len(), 3, "Should generate 3 scenarios");

    // Verify all scenarios have valid velocity directions
    for (idx, scenario) in scenarios.iter().enumerate() {
        let ego = scenario.actors.iter().find(|a| a.id == "ego").unwrap();
        let npc = scenario.actors.iter().find(|a| a.id == "npc").unwrap();

        // Both in forward lanes
        assert!(
            ego.states.iter().all(|s| s.velocity().vx >= 0.0),
            "Scenario {} ego should have positive velocity",
            idx
        );
        assert!(
            npc.states.iter().all(|s| s.velocity().vx >= 0.0),
            "Scenario {} npc should have positive velocity",
            idx
        );

        // Verify safety
        assert!(
            scenario.validation.all_constraints_satisfied,
            "Scenario {} should satisfy safety constraints",
            idx
        );
    }

    // Verify the scenarios actually differ, using the SW-52 diversity metric rather
    // than an unsubstantiated comment. This used to say "scenarios may have similar
    // initial conditions, this is acceptable, the main goal is testing bidirectional
    // road support" and check nothing at all. It is still true this test's main goal
    // is bidirectional road support, not diversity — hence a loose bound (> 0.0, not
    // a specific target) — but "nothing measures it" and "acceptable" are different
    // claims, and only the metric can tell them apart.
    let spec = scenario_weaver::dsl::parser::parse_yaml(yaml)
        .unwrap_or_else(|e| panic!("cannot parse scenario YAML: {e}"));
    let diversity = scenario_weaver::scenario::scenario_diversity(&spec, &scenarios);
    let min_dist = diversity
        .min_pairwise_distance
        .expect("3 scenarios must yield a min pairwise distance");
    assert!(
        min_dist > 0.0,
        "the 3 generated scenarios should not be trajectory-identical (min pairwise \
         L∞ distance was {min_dist})"
    );
    println!("{diversity}");
}

#[test]
fn test_three_lane_highway() {
    // Test 3-lane highway configuration (2 forward, 1 backward)
    let yaml = r#"
scenario_type: cut_in_left
time_step: 0.5
duration: 10.0

road:
  num_lanes: 3
  lane_width: 3.75
  lane_directions: [1, 1, -1]

actors:
  - id: ego
    role: ego
    lane: 1
    position: 50.0
    speed: 20.0
    direction: 1
    acceleration: [-8.0, 3.0]

  - id: npc
    role: npc
    lane: 0
    position: [60.0, 80.0]
    speed: [18.0, 22.0]
    direction: 1
    acceleration: [-8.0, 3.0]
    lane_changes:
      - direction: right
        start_time: [2.5, 7.5]
        duration: [3.0, 4.0]

min_ttc: 3.0
min_distance: 5.0
num_scenarios: 1
"#;

    let scenario = common::generate_or_fail(yaml);

    // Both actors in forward lanes
    for actor in &scenario.actors {
        assert!(
            actor.states.iter().all(|s| s.velocity().vx >= 0.0),
            "Actor {} should have positive velocity in forward lane",
            actor.id
        );
    }
}

#[test]
fn test_narrow_rural_road() {
    // Test 2-lane narrow rural road (1 forward, 1 backward)
    let yaml = r#"
scenario_type: cut_in_left
time_step: 0.5
duration: 10.0

road:
  num_lanes: 2
  lane_width: 3.0
  lane_directions: [1, -1]

actors:
  - id: ego
    role: ego
    lane: 0
    position: 50.0
    speed: 15.0
    direction: 1
    acceleration: [-8.0, 3.0]

  - id: npc
    role: npc
    lane: 1
    position: 120.0
    speed: 14.0
    direction: -1
    acceleration: [-2.0, 0.0]
    lane_changes:
      - direction: right
        start_time: [2.5, 7.5]
        duration: [3.0, 4.0]

min_ttc: 3.0
min_distance: 5.0
num_scenarios: 1
# SW-22. These three specs are oncoming pairs, and an `enforce`d `min_ttc` on an
# oncoming pair that must cross is only satisfiable *vacuously* — which is the
# defect SW-22 removed, so it now shows up as an
# `Invariant::ConstraintModes` breach rather than passing silently.
#
# Derivation, not trial and error: forward progress (SW-12) requires each vehicle
# to cover `0.5 * speed.min() * duration`, so over the 10 s horizon the ego must
# travel at least half its declared speed x 10 s, and likewise the NPC,
# toward each other. Their initial separation is smaller than that sum, so they
# *must* cross. While they share a lane, `min_ttc >= 3 s` demands
# `gap >= 3 * closing`, and at the crossing the gap goes to zero with the closing
# speed still positive — so the only models left are the ones where the pair does
# not share a lane while approaching, and then TTC is never defined. Both
# outcomes fail `enforce`; there is no satisfying model in between. Asserting the
# conflict anyway made these UNSAT (measured).
#
# `ignore` is the mode this spec can actually honour. Nothing these tests assert
# depends on it: they check that velocity sign follows lane direction. The
# `enforce` was default boilerplate that had never been evaluated on this spec.
constraint_modes:
  min_ttc: ignore
"#;

    let scenario = common::generate_or_fail(yaml);

    let ego = scenario.actors.iter().find(|a| a.id == "ego").unwrap();
    let npc = scenario.actors.iter().find(|a| a.id == "npc").unwrap();

    // Ego forward, NPC backward
    assert!(
        ego.states.iter().all(|s| s.velocity().vx >= 0.0),
        "Ego in forward lane"
    );
    assert!(
        npc.states.iter().all(|s| s.velocity().vx <= 0.0),
        "NPC in backward lane"
    );
}
