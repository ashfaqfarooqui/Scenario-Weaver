//! Integration tests for pedestrian scenario generation
//!
//! These tests run full end-to-end scenario generation using example YAML files
//! and verify that pedestrian physics are correct in the generated trajectories.

use scenario_weaver;
use scenario_weaver::dsl::types::{PEDESTRIAN_RUN_MAX_SPEED, PEDESTRIAN_WALK_MAX_SPEED};

/// Helper: generate scenario from a YAML file, panic on failure
fn generate_from_file(path: &str) -> scenario_weaver::scenario::model::Scenario {
    let yaml_content = std::fs::read_to_string(path)
        .unwrap_or_else(|e| panic!("Failed to read {}: {}", path, e));
    scenario_weaver::generate_single_scenario(&yaml_content)
        .unwrap_or_else(|e| panic!("Failed to generate scenario from {}: {:?}", path, e))
}

// ─── Basic pedestrian crossing (walk mode) ───

#[test]
fn test_pedestrian_crossing_generates_successfully() {
    let scenario = generate_from_file("examples/pedestrian_crossing.yaml");

    assert_eq!(scenario.actors.len(), 2);
    let ped = scenario.get_actor("pedestrian").expect("Should have pedestrian actor");
    assert_eq!(ped.role, "pedestrian");
    assert!(!ped.states.is_empty());
}

#[test]
fn test_pedestrian_crossing_kinematics_consistency() {
    let scenario = generate_from_file("examples/pedestrian_crossing.yaml");
    let ped = scenario.get_actor("pedestrian").unwrap();
    let dt = scenario.time_step;

    // Verify position updates match velocity * dt (Euler integration)
    for i in 0..ped.states.len() - 1 {
        let s = &ped.states[i];
        let s_next = &ped.states[i + 1];

        let expected_px = s.cartesian.position.x + s.cartesian.velocity.vx * dt;
        let expected_py = s.cartesian.position.y + s.cartesian.velocity.vy * dt;

        assert!(
            (s_next.cartesian.position.x - expected_px).abs() < 0.1,
            "px mismatch at t={:.1}s: got {:.4}, expected {:.4} (vx={:.4})",
            s.time, s_next.cartesian.position.x, expected_px, s.cartesian.velocity.vx
        );
        assert!(
            (s_next.cartesian.position.y - expected_py).abs() < 0.1,
            "py mismatch at t={:.1}s: got {:.4}, expected {:.4} (vy={:.4})",
            s.time, s_next.cartesian.position.y, expected_py, s.cartesian.velocity.vy
        );
    }
}

#[test]
fn test_pedestrian_crossing_velocity_consistency() {
    let scenario = generate_from_file("examples/pedestrian_crossing.yaml");
    let ped = scenario.get_actor("pedestrian").unwrap();
    let dt = scenario.time_step;

    // Verify velocity updates match acceleration * dt
    for i in 0..ped.states.len() - 1 {
        let s = &ped.states[i];
        let s_next = &ped.states[i + 1];

        let expected_vx = s.cartesian.velocity.vx + s.cartesian.acceleration.ax * dt;
        let expected_vy = s.cartesian.velocity.vy + s.cartesian.acceleration.ay * dt;

        assert!(
            (s_next.cartesian.velocity.vx - expected_vx).abs() < 0.1,
            "vx mismatch at t={:.1}s: got {:.4}, expected {:.4} (ax={:.4})",
            s.time, s_next.cartesian.velocity.vx, expected_vx, s.cartesian.acceleration.ax
        );
        assert!(
            (s_next.cartesian.velocity.vy - expected_vy).abs() < 0.1,
            "vy mismatch at t={:.1}s: got {:.4}, expected {:.4} (ay={:.4})",
            s.time, s_next.cartesian.velocity.vy, expected_vy, s.cartesian.acceleration.ay
        );
    }
}

#[test]
fn test_pedestrian_crossing_speed_bounds() {
    let scenario = generate_from_file("examples/pedestrian_crossing.yaml");
    let ped = scenario.get_actor("pedestrian").unwrap();

    // Walking pedestrian: |vx| <= PEDESTRIAN_WALK_MAX_SPEED, |vy| <= PEDESTRIAN_WALK_MAX_SPEED
    let tolerance = 0.05; // Small numerical tolerance

    for state in &ped.states {
        assert!(
            state.cartesian.velocity.vx.abs() <= PEDESTRIAN_WALK_MAX_SPEED + tolerance,
            "vx out of bounds at t={:.1}s: {:.4} > {:.4}",
            state.time, state.cartesian.velocity.vx.abs(), PEDESTRIAN_WALK_MAX_SPEED
        );
        assert!(
            state.cartesian.velocity.vy.abs() <= PEDESTRIAN_WALK_MAX_SPEED + tolerance,
            "vy out of bounds at t={:.1}s: {:.4} > {:.4}",
            state.time, state.cartesian.velocity.vy.abs(), PEDESTRIAN_WALK_MAX_SPEED
        );
    }
}

#[test]
fn test_pedestrian_crossing_acceleration_bounds() {
    let scenario = generate_from_file("examples/pedestrian_crossing.yaml");
    let ped = scenario.get_actor("pedestrian").unwrap();

    // Pedestrian acceleration should be clamped to [-1.0, 1.0]
    let tolerance = 0.05;

    for state in &ped.states {
        assert!(
            state.cartesian.acceleration.ax.abs() <= 1.0 + tolerance,
            "ax out of bounds at t={:.1}s: {:.4}",
            state.time, state.cartesian.acceleration.ax
        );
        assert!(
            state.cartesian.acceleration.ay.abs() <= 1.0 + tolerance,
            "ay out of bounds at t={:.1}s: {:.4}",
            state.time, state.cartesian.acceleration.ay
        );
    }
}

#[test]
fn test_pedestrian_crosses_laterally() {
    let scenario = generate_from_file("examples/pedestrian_crossing.yaml");
    let ped = scenario.get_actor("pedestrian").unwrap();

    // Pedestrian should move laterally (py should change over time)
    let py_start = ped.states[0].cartesian.position.y;
    let py_end = ped.states.last().unwrap().cartesian.position.y;

    assert!(
        (py_end - py_start).abs() > 1.0,
        "Pedestrian should cross laterally: py_start={:.2}, py_end={:.2}, delta={:.2}",
        py_start, py_end, (py_end - py_start).abs()
    );
}

// ─── Running pedestrian ───

#[test]
fn test_pedestrian_running_generates_successfully() {
    let scenario = generate_from_file("examples/pedestrian_running.yaml");

    assert_eq!(scenario.actors.len(), 2);
    let ped = scenario.get_actor("runner").expect("Should have runner actor");
    assert_eq!(ped.role, "pedestrian");
}

#[test]
fn test_pedestrian_running_speed_bounds() {
    let scenario = generate_from_file("examples/pedestrian_running.yaml");
    let ped = scenario.get_actor("runner").unwrap();

    // Running pedestrian: |vx| <= PEDESTRIAN_RUN_MAX_SPEED, |vy| <= PEDESTRIAN_RUN_MAX_SPEED
    let tolerance = 0.05;

    for state in &ped.states {
        assert!(
            state.cartesian.velocity.vx.abs() <= PEDESTRIAN_RUN_MAX_SPEED + tolerance,
            "vx out of bounds at t={:.1}s: {:.4} > {:.4}",
            state.time, state.cartesian.velocity.vx.abs(), PEDESTRIAN_RUN_MAX_SPEED
        );
        assert!(
            state.cartesian.velocity.vy.abs() <= PEDESTRIAN_RUN_MAX_SPEED + tolerance,
            "vy out of bounds at t={:.1}s: {:.4} > {:.4}",
            state.time, state.cartesian.velocity.vy.abs(), PEDESTRIAN_RUN_MAX_SPEED
        );
    }
}

#[test]
fn test_pedestrian_running_kinematics_consistency() {
    let scenario = generate_from_file("examples/pedestrian_running.yaml");
    let ped = scenario.get_actor("runner").unwrap();
    let dt = scenario.time_step;

    for i in 0..ped.states.len() - 1 {
        let s = &ped.states[i];
        let s_next = &ped.states[i + 1];

        let expected_px = s.cartesian.position.x + s.cartesian.velocity.vx * dt;
        let expected_py = s.cartesian.position.y + s.cartesian.velocity.vy * dt;

        assert!(
            (s_next.cartesian.position.x - expected_px).abs() < 0.1,
            "px mismatch at t={:.1}s: got {:.4}, expected {:.4}",
            s.time, s_next.cartesian.position.x, expected_px
        );
        assert!(
            (s_next.cartesian.position.y - expected_py).abs() < 0.1,
            "py mismatch at t={:.1}s: got {:.4}, expected {:.4}",
            s.time, s_next.cartesian.position.y, expected_py
        );
    }
}

// ─── Wide road pedestrian ───

#[test]
fn test_pedestrian_wide_road_generates_successfully() {
    let scenario = generate_from_file("examples/pedestrian_wide_road.yaml");

    assert_eq!(scenario.actors.len(), 2);
    let ped = scenario.get_actor("ped").expect("Should have pedestrian actor");
    assert_eq!(ped.role, "pedestrian");

    // Ego should be in middle lane
    let ego = scenario.get_actor("ego").unwrap();
    assert_eq!(ego.states[0].cartesian.lane, 1);
}

#[test]
fn test_pedestrian_wide_road_kinematics_consistency() {
    let scenario = generate_from_file("examples/pedestrian_wide_road.yaml");
    let ped = scenario.get_actor("ped").unwrap();
    let dt = scenario.time_step;

    for i in 0..ped.states.len() - 1 {
        let s = &ped.states[i];
        let s_next = &ped.states[i + 1];

        let expected_px = s.cartesian.position.x + s.cartesian.velocity.vx * dt;
        let expected_py = s.cartesian.position.y + s.cartesian.velocity.vy * dt;

        assert!(
            (s_next.cartesian.position.x - expected_px).abs() < 0.1,
            "px mismatch at t={:.1}s: got {:.4}, expected {:.4}",
            s.time, s_next.cartesian.position.x, expected_px
        );
        assert!(
            (s_next.cartesian.position.y - expected_py).abs() < 0.1,
            "py mismatch at t={:.1}s: got {:.4}, expected {:.4}",
            s.time, s_next.cartesian.position.y, expected_py
        );
    }
}

#[test]
fn test_pedestrian_wide_road_crosses_multiple_lanes() {
    let scenario = generate_from_file("examples/pedestrian_wide_road.yaml");
    let ped = scenario.get_actor("ped").unwrap();

    // On a 3-lane road (10.5m wide), the pedestrian should cross a significant distance
    let py_start = ped.states[0].cartesian.position.y;
    let py_end = ped.states.last().unwrap().cartesian.position.y;

    // Should cross at least 1 lane width (3.5m) on a 3-lane road
    assert!(
        (py_end - py_start).abs() > 3.0,
        "Pedestrian should cross at least one lane width on wide road: delta_py={:.2}",
        (py_end - py_start).abs()
    );
}

// ─── Ego vehicle sanity checks ───

#[test]
fn test_ego_moves_forward_during_pedestrian_crossing() {
    let scenario = generate_from_file("examples/pedestrian_crossing.yaml");
    let ego = scenario.get_actor("ego").unwrap();

    // Ego should move forward (px increasing over time)
    let px_start = ego.states[0].cartesian.position.x;
    let px_end = ego.states.last().unwrap().cartesian.position.x;

    assert!(
        px_end > px_start,
        "Ego should move forward: px_start={:.2}, px_end={:.2}",
        px_start, px_end
    );
}

#[test]
fn test_ego_stays_in_lane_during_pedestrian_crossing() {
    let scenario = generate_from_file("examples/pedestrian_crossing.yaml");
    let ego = scenario.get_actor("ego").unwrap();

    // Ego should stay in its lane (vy should be ~0)
    for state in &ego.states {
        assert!(
            state.cartesian.velocity.vy.abs() < 0.01,
            "Ego vy should be ~0 at t={:.1}s: got {:.4}",
            state.time, state.cartesian.velocity.vy
        );
    }
}
