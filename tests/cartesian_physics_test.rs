use scenario_weaver;
use std::time::Instant;

#[test]
fn test_velocity_ratio_during_lane_change() {
    let yaml_content =
        std::fs::read_to_string("examples/cut_in_left.yaml").expect("Should read example YAML");

    let scenario = scenario_weaver::generate_single_scenario(&yaml_content)
        .expect("Should generate scenario");

    let npc = scenario
        .actors
        .iter()
        .find(|a| a.id == "npc")
        .expect("Should have NPC");

    // Find lane change period (when vy != 0)
    let max_ratio = 0.15;
    let tolerance = 0.01; // Allow small numerical error

    let mut found_lane_change = false;

    for state in &npc.states {
        let vx = state.velocity().vx.abs();
        let vy = state.velocity().vy.abs();

        if vy > 0.01 {
            // During lane change (vy non-zero)
            found_lane_change = true;
            let ratio = vy / vx;
            assert!(
                ratio <= max_ratio + tolerance,
                "At t={:.1}s: vy/vx = {:.4} exceeds max ratio {:.4} (vx={:.2}, vy={:.2})",
                state.time,
                ratio,
                max_ratio,
                vx,
                vy
            );
        }
    }

    assert!(
        found_lane_change,
        "Should have detected lane change with vy != 0"
    );
}

#[test]
fn test_no_sideways_only_motion() {
    let yaml_content =
        std::fs::read_to_string("examples/cut_in_left.yaml").expect("Should read example YAML");

    let scenario = scenario_weaver::generate_single_scenario(&yaml_content)
        .expect("Should generate scenario");

    let npc = scenario
        .actors
        .iter()
        .find(|a| a.id == "npc")
        .expect("Should have NPC");

    for state in &npc.states {
        let vx = state.velocity().vx.abs();
        let vy = state.velocity().vy.abs();

        if vy > 0.01 {
            // If moving laterally
            assert!(
                vx > vy * 2.0, // Forward speed >> lateral speed
                "At t={:.1}s: vx={:.2} should be much greater than vy={:.2}",
                state.time,
                vx,
                vy
            );
        }
    }
}

#[test]
fn test_heading_angle_during_lane_change() {
    let yaml_content =
        std::fs::read_to_string("examples/cut_in_left.yaml").expect("Should read example YAML");

    let scenario = scenario_weaver::generate_single_scenario(&yaml_content)
        .expect("Should generate scenario");

    let npc = scenario
        .actors
        .iter()
        .find(|a| a.id == "npc")
        .expect("Should have NPC");

    let max_heading_degrees: f64 = 10.0; // Comfortable limit
    let max_heading_radians = max_heading_degrees.to_radians();

    for state in &npc.states {
        let vx = state.velocity().vx;
        let vy = state.velocity().vy;

        if vx.abs() > 0.1 {
            // Avoid division by very small numbers
            let heading = (vy / vx).atan().abs();

            assert!(
                heading <= max_heading_radians,
                "At t={:.1}s: heading angle {:.2}° exceeds max {:.2}° (vx={:.2}, vy={:.2})",
                state.time,
                heading.to_degrees(),
                max_heading_degrees,
                vx,
                vy
            );
        }
    }
}

#[test]
fn test_solving_performance_with_ratio_constraint() {
    let yaml_content =
        std::fs::read_to_string("examples/cut_in_left.yaml").expect("Should read example YAML");

    let start = Instant::now();
    let scenario = scenario_weaver::generate_single_scenario(&yaml_content)
        .expect("Should generate scenario");
    let duration = start.elapsed();

    println!(
        "Generation time with ratio constraint: {:.2}s",
        duration.as_secs_f64()
    );

    // Should complete in reasonable time (balanced priority)
    assert!(
        duration.as_secs() < 10,
        "Generation took {:.2}s, should be < 10s for balanced performance",
        duration.as_secs_f64()
    );

    // Verify physics correctness
    let npc = scenario
        .actors
        .iter()
        .find(|a| a.id == "npc")
        .expect("Should have NPC");
    let has_lane_change = npc.states.iter().any(|s| s.velocity().vy.abs() > 0.01);
    assert!(has_lane_change, "Should generate lane change scenario");
}

#[test]
fn test_multiple_scenarios_maintain_physics() {
    use tempfile::TempDir;

    let yaml_content =
        std::fs::read_to_string("examples/cut_in_left.yaml").expect("Should read example YAML");

    // Use temporary directory for output
    let temp_dir = TempDir::new().expect("Should create temp dir");
    let _output_dir = temp_dir.path();

    // Parse spec to create scenarios manually
    let max_ratio = 0.15;
    let tolerance = 0.01;

    for i in 0..3 {
        let scenario = scenario_weaver::generate_single_scenario(&yaml_content)
            .expect(&format!("Should generate scenario {}", i));

        let npc = scenario
            .actors
            .iter()
            .find(|a| a.id == "npc")
            .expect("Should have NPC");

        let mut found_lane_change = false;

        for state in &npc.states {
            let vx = state.velocity().vx.abs();
            let vy = state.velocity().vy.abs();

            if vy > 0.01 {
                found_lane_change = true;
                let ratio = vy / vx;
                assert!(
                    ratio <= max_ratio + tolerance,
                    "Scenario {}: At t={:.1}s: vy/vx = {:.4} exceeds max ratio {:.4}",
                    i,
                    state.time,
                    ratio,
                    max_ratio
                );
            }
        }

        assert!(found_lane_change, "Scenario {} should have lane change", i);
    }
}
