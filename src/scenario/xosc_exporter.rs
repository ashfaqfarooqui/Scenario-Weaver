//! OpenSCENARIO (.xosc) export functionality
//!
//! Converts internal Scenario data structures to OpenSCENARIO XML format.
//! This implementation creates a minimal but valid OpenSCENARIO file with
//! basic vehicle entities and trajectory information in the description.

use crate::error::Result;
use crate::scenario::model::{Scenario, State};
use openscenario_rs::ScenarioBuilder;

/// Export a scenario to OpenSCENARIO XML format
///
/// Generates a minimal OpenSCENARIO file with:
/// - File header with scenario metadata
/// - Vehicle entities for all actors
/// - Trajectory data embedded in description for reference
///
/// Note: This is a simplified implementation that creates valid OpenSCENARIO structure
/// without full trajectory actions. The trajectory data is preserved in the description
/// field for reference and can be extended in future versions.
pub fn export_to_xosc(scenario: &Scenario) -> Result<String> {
    // Build scenario description with embedded trajectory summary
    let description = build_scenario_description(scenario);

    // Create basic OpenSCENARIO structure using the builder
    let mut builder = ScenarioBuilder::new()
        .with_header(&description, "CARLA Scenario Generator")
        .with_entities();

    // Add vehicle entity for each actor
    for actor in &scenario.actors {
        builder = builder.add_vehicle(&actor.id, |vehicle| vehicle.car());
    }

    let openscenario = builder.build().map_err(|e| {
        crate::error::ScenarioGenError::XoscExport(format!(
            "Failed to build OpenSCENARIO structure: {}",
            e
        ))
    })?;

    // Serialize to XML string
    let xml = openscenario_rs::serialize_to_string(&openscenario).map_err(|e| {
        crate::error::ScenarioGenError::XoscExport(format!("XML serialization failed: {}", e))
    })?;

    Ok(xml)
}

/// Build a detailed scenario description with trajectory summary
///
/// Embeds key scenario information in the description field including:
/// - Scenario type and ID
/// - Time parameters
/// - Actor count and roles
/// - Trajectory summary (initial/final positions and speeds)
fn build_scenario_description(scenario: &Scenario) -> String {
    let mut desc = format!(
        "Scenario: {} (ID: {})\nType: {}\nDuration: {}s, Time step: {}s\n\nActors: {}",
        scenario.scenario_id,
        scenario.scenario_id,
        scenario.scenario_type,
        scenario.duration,
        scenario.time_step,
        scenario.actors.len()
    );

    // Add trajectory summary for each actor
    for actor in &scenario.actors {
        if let (Some(first), Some(last)) = (actor.states.first(), actor.states.last()) {
            desc.push_str(&format!(
                "\n\n{} ({}):\n  Start: pos=({:.1}, {:.1}) vel=({:.1}, {:.1}) lane={}\n  End: pos=({:.1}, {:.1}) vel=({:.1}, {:.1}) lane={}",
                actor.id,
                actor.role,
                first.position.x, first.position.y,
                first.velocity.vx, first.velocity.vy,
                first.lane,
                last.position.x, last.position.y,
                last.velocity.vx, last.velocity.vy,
                last.lane
            ));
        }
    }

    // Add validation info
    desc.push_str(&format!(
        "\n\nValidation:\n  Min TTC: {:.2}s\n  Min distance: {:.2}m\n  Constraints satisfied: {}",
        scenario.validation.min_ttc,
        scenario.validation.min_distance,
        scenario.validation.all_constraints_satisfied
    ));

    desc
}

/// Compute heading angle from velocity vector
///
/// Uses atan2(vy, vx) to compute heading in radians.
/// Heading follows OpenSCENARIO convention:
/// - 0 radians = East (+X direction)
/// - π/2 radians = North (+Y direction)
#[allow(dead_code)]
fn compute_heading(state: &State) -> f64 {
    state.velocity.vy.atan2(state.velocity.vx)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scenario::model::{ActorTrajectory, Position, Velocity};

    #[test]
    fn test_compute_heading() {
        // East (along X axis)
        let state = State::new(0.0, Position::new(0.0, 0.0), Velocity::new(15.0, 0.0), 1);
        assert!((compute_heading(&state) - 0.0).abs() < 1e-10);

        // North (along Y axis)
        let state = State::new(0.0, Position::new(0.0, 0.0), Velocity::new(0.0, 15.0), 1);
        assert!((compute_heading(&state) - std::f64::consts::FRAC_PI_2).abs() < 1e-10);

        // West (negative X)
        let state = State::new(0.0, Position::new(0.0, 0.0), Velocity::new(-15.0, 0.0), 1);
        assert!((compute_heading(&state) - std::f64::consts::PI).abs() < 1e-10);

        // South (negative Y)
        let state = State::new(0.0, Position::new(0.0, 0.0), Velocity::new(0.0, -15.0), 1);
        assert!((compute_heading(&state) + std::f64::consts::FRAC_PI_2).abs() < 1e-10);

        // Northeast (45 degrees)
        let state = State::new(0.0, Position::new(0.0, 0.0), Velocity::new(10.0, 10.0), 1);
        assert!((compute_heading(&state) - std::f64::consts::FRAC_PI_4).abs() < 1e-10);
    }

    #[test]
    fn test_export_to_xosc() {
        let mut scenario = Scenario::new("cut_in_left".to_string(), 0.5, 2.0);

        // Create simple ego trajectory with 2 states
        let mut ego = ActorTrajectory::new("ego".to_string(), "ego".to_string());
        ego.add_state(State::new(
            0.0,
            Position::new(0.0, 5.25),
            Velocity::new(15.0, 0.0),
            1,
        ));
        ego.add_state(State::new(
            0.5,
            Position::new(7.5, 5.25),
            Velocity::new(15.0, 0.0),
            1,
        ));

        scenario.add_actor(ego);

        let xml = export_to_xosc(&scenario).expect("Export should succeed");

        // Basic validation
        assert!(xml.contains("<?xml"));
        assert!(xml.contains("OpenSCENARIO"));
        assert!(xml.contains("CARLA Scenario Generator"));
        assert!(xml.contains("cut_in_left"));
    }

    #[test]
    fn test_scenario_description() {
        let mut scenario = Scenario::new("test_scenario".to_string(), 0.5, 10.0);

        let mut ego = ActorTrajectory::new("ego".to_string(), "ego".to_string());
        ego.add_state(State::new(
            0.0,
            Position::new(0.0, 5.0),
            Velocity::new(15.0, 0.0),
            1,
        ));
        ego.add_state(State::new(
            10.0,
            Position::new(150.0, 5.0),
            Velocity::new(15.0, 0.0),
            1,
        ));

        scenario.add_actor(ego);

        let desc = build_scenario_description(&scenario);

        assert!(desc.contains("test_scenario"));
        assert!(desc.contains("Duration: 10s"));
        assert!(desc.contains("ego"));
        assert!(desc.contains("Start:"));
        assert!(desc.contains("End:"));
    }
}
