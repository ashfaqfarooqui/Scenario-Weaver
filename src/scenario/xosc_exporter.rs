//! OpenSCENARIO (.xosc) export functionality
//!
//! Converts internal Scenario data structures to OpenSCENARIO XML format
//! with complete trajectory-based actions using openscenario-rs builders.

use crate::error::Result;
use crate::scenario::model::{Scenario, State};
use openscenario_rs::builder::actions::trajectory::TrajectoryBuilder;
use openscenario_rs::builder::init::InitActionBuilder;
use openscenario_rs::builder::positions::WorldPositionBuilder;
use openscenario_rs::builder::StoryboardBuilder;
use openscenario_rs::ScenarioBuilder;

/// Export a scenario to OpenSCENARIO XML format
///
/// Generates a complete OpenSCENARIO file with:
/// - File header with scenario metadata
/// - Vehicle entities for all actors
/// - Init actions setting initial positions and velocities
/// - Storyboard with trajectory following actions for all actors
///
/// This implementation uses the full openscenario-rs builder API to create
/// proper trajectory-based scenarios that can be executed by simulators.
pub fn export_to_xosc(scenario: &Scenario) -> Result<String> {
    // Build scenario description for the header
    let description = build_scenario_description(scenario);

    // Create basic scenario structure with entities
    let mut builder = ScenarioBuilder::new()
        .with_header(&description, "CARLA Scenario Generator")
        .with_entities();

    // Add entities for each actor
    // TODO: openscenario-rs library limitation - no direct add_pedestrian() method
    // The library only supports add_catalog_pedestrian() which requires external catalog files.
    // To properly support pedestrians, the openscenario-rs library needs to add:
    //   - pub fn add_pedestrian<F>(name: &str, config: F) -> Self
    //   - PedestrianBuilder with methods like .adult(), .child(), etc.
    // For now, we export all actors (including pedestrians) as vehicles.
    for actor in &scenario.actors {
        builder = builder.add_vehicle(&actor.id, |vehicle| vehicle.car());
    }

    // Build storyboard with init actions and trajectories
    let mut storyboard_builder = StoryboardBuilder::new(builder);

    // Add init actions for all actors (position + speed)
    let init_actions = build_init_actions(scenario)?;
    storyboard_builder = storyboard_builder.with_init_actions(init_actions);

    let mut story_builder = storyboard_builder.add_story_simple("main_story");

    // For each actor, create a separate Act with trajectory action
    // This avoids the library limitation where all maneuvers in an Act
    // are placed in the same ManeuverGroup, which causes esmini conflicts
    for actor in &scenario.actors {
        // Build trajectory from actor states
        let trajectory = build_trajectory(actor)?;

        // Create a separate act for this actor
        let act_name = format!("{}_trajectory_act", actor.id);
        let mut act = story_builder.create_act(&act_name);

        // Create maneuver for this actor
        let maneuver_name = format!("{}_maneuver", actor.id);
        let mut maneuver = act.create_maneuver(&maneuver_name, &actor.id);

        // Create follow trajectory action
        let trajectory_action = maneuver
            .create_follow_trajectory_action()
            .with_trajectory(trajectory)
            .following_mode_follow();

        // Attach action to maneuver (detached pattern)
        trajectory_action
            .attach_to_detached(&mut maneuver)
            .map_err(|e| {
                crate::error::ScenarioGenError::XoscExport(format!(
                    "Failed to attach trajectory action: {}",
                    e
                ))
            })?;

        // Attach maneuver to act
        maneuver.attach_to_detached(&mut act);

        // Attach act to story
        act.attach_to(&mut story_builder);
    }

    // Finish the story to add it to the storyboard
    story_builder.finish();

    // Add stop trigger based on scenario duration
    let storyboard_builder = storyboard_builder
        .stop_after_time(scenario.duration)
        .map_err(|e| {
            crate::error::ScenarioGenError::XoscExport(format!("Failed to add stop trigger: {}", e))
        })?;

    // Build the final scenario
    let openscenario = storyboard_builder.finish().build().map_err(|e| {
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

/// Build a trajectory from an actor's state sequence
fn build_trajectory(
    actor: &crate::scenario::model::ActorTrajectory,
) -> Result<openscenario_rs::types::actions::movement::Trajectory> {
    let mut polyline_builder = TrajectoryBuilder::new()
        .name(&format!("{}_trajectory", actor.id))
        .closed(false)
        .polyline();

    // Add vertex for each state
    for state in &actor.states {
        let heading = compute_heading(state);

        polyline_builder = polyline_builder
            .add_vertex()
            .time(state.time)
            .world_position(state.position.x, state.position.y, 0.0, heading)
            .finish()
            .map_err(|e| {
                crate::error::ScenarioGenError::XoscExport(format!(
                    "Failed to add trajectory vertex: {}",
                    e
                ))
            })?;
    }

    // Finish polyline and build trajectory
    let trajectory = polyline_builder.finish().build().map_err(|e| {
        crate::error::ScenarioGenError::XoscExport(format!("Failed to build trajectory: {}", e))
    })?;

    Ok(trajectory)
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
fn compute_heading(state: &State) -> f64 {
    state.velocity.vy.atan2(state.velocity.vx)
}

/// Build init actions for all actors (position + speed)
///
/// Creates Init section with Private actions for each actor:
/// - TeleportAction: Sets initial world position
/// - SpeedAction: Sets initial speed from velocity magnitude
fn build_init_actions(scenario: &Scenario) -> Result<openscenario_rs::types::scenario::init::Init> {
    let mut init_builder = InitActionBuilder::new();

    for actor in &scenario.actors {
        // Get initial state
        let initial_state = actor.states.first().ok_or_else(|| {
            crate::error::ScenarioGenError::XoscExport(format!("Actor {} has no states", actor.id))
        })?;

        // Calculate speed from velocity magnitude
        let speed = (initial_state.velocity.vx.powi(2) + initial_state.velocity.vy.powi(2)).sqrt();

        // Calculate heading from velocity
        let heading = compute_heading(initial_state);

        // Create world position
        let position = WorldPositionBuilder::new()
            .at_coordinates(initial_state.position.x, initial_state.position.y, 0.0)
            .with_heading(heading)
            .build()
            .map_err(|e| {
                crate::error::ScenarioGenError::XoscExport(format!(
                    "Failed to build world position: {}",
                    e
                ))
            })?;

        // Add speed action first, then teleport (to match reference format)
        init_builder = init_builder
            .add_speed_action(&actor.id, speed)
            .add_teleport_action(&actor.id, position);
    }

    init_builder.build().map_err(|e| {
        crate::error::ScenarioGenError::XoscExport(format!("Failed to build init actions: {}", e))
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scenario::model::{Acceleration, ActorTrajectory, Position, Velocity};

    #[test]
    fn test_compute_heading() {
        // East (along X axis)
        let state = State::new(
            0.0,
            Position::new(0.0, 0.0),
            Velocity::new(15.0, 0.0),
            Acceleration::new(0.0, 0.0),
            1,
        );
        assert!((compute_heading(&state) - 0.0).abs() < 1e-10);

        // North (along Y axis)
        let state = State::new(
            0.0,
            Position::new(0.0, 0.0),
            Velocity::new(0.0, 15.0),
            Acceleration::new(0.0, 0.0),
            1,
        );
        assert!((compute_heading(&state) - std::f64::consts::FRAC_PI_2).abs() < 1e-10);

        // West (negative X)
        let state = State::new(
            0.0,
            Position::new(0.0, 0.0),
            Velocity::new(-15.0, 0.0),
            Acceleration::new(0.0, 0.0),
            1,
        );
        assert!((compute_heading(&state) - std::f64::consts::PI).abs() < 1e-10);

        // South (negative Y)
        let state = State::new(
            0.0,
            Position::new(0.0, 0.0),
            Velocity::new(0.0, -15.0),
            Acceleration::new(0.0, 0.0),
            1,
        );
        assert!((compute_heading(&state) + std::f64::consts::FRAC_PI_2).abs() < 1e-10);

        // Northeast (45 degrees)
        let state = State::new(
            0.0,
            Position::new(0.0, 0.0),
            Velocity::new(10.0, 10.0),
            Acceleration::new(0.0, 0.0),
            1,
        );
        assert!((compute_heading(&state) - std::f64::consts::FRAC_PI_4).abs() < 1e-10);
    }

    #[test]
    fn test_export_to_xosc() {
        let mut scenario = Scenario::new("cut_in_left".to_string(), 0.5, 2.0);

        // Create simple ego trajectory with 3 states
        let mut ego = ActorTrajectory::new("ego".to_string(), "ego".to_string());
        ego.add_state(State::new(
            0.0,
            Position::new(0.0, 5.25),
            Velocity::new(15.0, 0.0),
            Acceleration::new(0.0, 0.0),
            1,
        ));
        ego.add_state(State::new(
            0.5,
            Position::new(7.5, 5.25),
            Velocity::new(15.0, 0.0),
            Acceleration::new(0.0, 0.0),
            1,
        ));
        ego.add_state(State::new(
            1.0,
            Position::new(15.0, 5.25),
            Velocity::new(15.0, 0.0),
            Acceleration::new(0.0, 0.0),
            1,
        ));

        scenario.add_actor(ego);

        let xml = export_to_xosc(&scenario).expect("Export should succeed");

        // Basic validation
        assert!(xml.contains("<?xml"));
        assert!(xml.contains("OpenSCENARIO"));
        assert!(xml.contains("CARLA Scenario Generator"));
        assert!(xml.contains("cut_in_left"));

        // Verify entities
        assert!(xml.contains("<Entities"));
        assert!(xml.contains("ego"));

        // Verify storyboard structure
        assert!(xml.contains("<Storyboard"));
        assert!(xml.contains("<Story"));
    }

    #[test]
    fn test_scenario_description() {
        let mut scenario = Scenario::new("test_scenario".to_string(), 0.5, 10.0);

        let mut ego = ActorTrajectory::new("ego".to_string(), "ego".to_string());
        ego.add_state(State::new(
            0.0,
            Position::new(0.0, 5.0),
            Velocity::new(15.0, 0.0),
            Acceleration::new(0.0, 0.0),
            1,
        ));
        ego.add_state(State::new(
            10.0,
            Position::new(150.0, 5.0),
            Velocity::new(15.0, 0.0),
            Acceleration::new(0.0, 0.0),
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
