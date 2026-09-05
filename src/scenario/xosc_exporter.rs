//! OpenSCENARIO (.xosc) export functionality
//!
//! Converts internal Scenario data structures to OpenSCENARIO XML format
//! with complete trajectory-based actions using openscenario-rs builders.

use crate::dsl::types::{ActorRole, RoadSpec};
use crate::error::Result;
use crate::scenario::lane_ids::{lane_index_to_xodr_id, XODR_ROAD_ID};
use crate::scenario::model::{Scenario, State};
use crate::scenario::xodr_exporter::compute_road_geometry;
use openscenario_rs::builder::actions::trajectory::TrajectoryBuilder;
use openscenario_rs::builder::init::InitActionBuilder;
use openscenario_rs::builder::positions::{LanePositionBuilder, PositionBuilder};
use openscenario_rs::builder::StoryboardBuilder;
use openscenario_rs::types::basic::Double;
use openscenario_rs::types::catalogs::locations::CatalogLocations;
use openscenario_rs::types::enums::ReferenceContext;
use openscenario_rs::types::positions::{Orientation, Position};
use openscenario_rs::types::road::RoadNetwork;
use openscenario_rs::ScenarioBuilder;

/// The OpenSCENARIO revision this project targets, declared explicitly
/// (SW-15 M13). `1.3` is what `ScenarioBuilder::with_header` already
/// defaults to; calling `with_revision` makes that a deliberate choice
/// instead of an accident of the dependency's default, and gives future
/// changes to that default nothing to silently break.
const OSC_REV_MAJOR: u16 = 1;
const OSC_REV_MINOR: u16 = 3;

/// Vehicle bounding-box dimensions used for every exported `PassengerCar`
/// entity (SW-15 H8). These match `openscenario_rs::VehicleBuilder::car()`'s
/// own preset exactly — made explicit here, via `with_dimensions`, so a
/// change to that dependency default cannot silently change what this crate
/// ships. The DSL does not yet carry per-actor dimensions (see the SW-15
/// report: `ActorSpec` is built via plain struct literals with no
/// `..Default::default()` in `src/scenarios/*.rs` and `src/solver/*.rs`, both
/// out of this issue's touch list, so adding required fields there is not a
/// same-issue-sized change), so every vehicle uses this single constant set.
/// `min_distance` therefore remains centre-to-centre, not bumper-to-bumper —
/// unchanged by this issue, tracked as a solver-side follow-up.
const VEHICLE_LENGTH_M: f64 = 4.5;
const VEHICLE_WIDTH_M: f64 = 1.8;
const VEHICLE_HEIGHT_M: f64 = 1.4;

/// Pedestrian bounding-box dimensions, matching
/// `openscenario_rs::PedestrianBuilder::pedestrian()`'s own preset (see the
/// `VEHICLE_*_M` doc comment above for why these are constants rather than
/// DSL-configurable yet).
const PEDESTRIAN_LENGTH_M: f64 = 0.6;
const PEDESTRIAN_WIDTH_M: f64 = 0.6;
const PEDESTRIAN_HEIGHT_M: f64 = 1.8;

/// Export a scenario to OpenSCENARIO XML format
///
/// Generates a complete OpenSCENARIO file with:
/// - File header with scenario metadata
/// - Vehicle entities for all actors
/// - Init actions setting initial positions and velocities
/// - Storyboard with trajectory following actions for all actors
pub fn export_to_xosc(scenario: &Scenario) -> Result<String> {
    export_to_xosc_impl(scenario, None)
}

/// Export a scenario to OpenSCENARIO XML format with an OpenDRIVE road reference
///
/// Same as [`export_to_xosc`] but embeds a `<RoadNetwork><LogicFile>` reference to
/// the given OpenDRIVE file path.  Use a relative path (e.g. `"scenario.xodr"`) so
/// the .xosc and .xodr files can be moved together without breaking the reference.
pub fn export_to_xosc_with_road_file(scenario: &Scenario, xodr_path: &str) -> Result<String> {
    export_to_xosc_impl(scenario, Some(xodr_path))
}

fn export_to_xosc_impl(scenario: &Scenario, road_file: Option<&str>) -> Result<String> {
    // Build scenario description for the header
    let description = build_scenario_description(scenario);

    // The `.xodr`'s `s=0` is not always world `x=0` -- SW-16/M12 pulls the
    // road's start back to cover a backward-direction actor's negative `x`.
    // Every `LanePosition` below must subtract this same offset so its `s`
    // means the same physical point the companion `.xodr` does.
    let (road_start_x, _) = compute_road_geometry(scenario);

    // Create basic scenario structure with entities
    let header_builder = ScenarioBuilder::new()
        .with_header(&description, "ScenarioWeaver")
        .with_revision(OSC_REV_MAJOR, OSC_REV_MINOR);

    let header_builder = if let Some(path) = road_file {
        header_builder.with_road_file(path)
    } else {
        header_builder.with_road_network(RoadNetwork::default())
    };

    let header_builder = header_builder.with_catalog_locations(CatalogLocations::default());

    let mut builder = header_builder.with_entities();

    // Add entities for each actor
    for actor in &scenario.actors {
        if actor.role == ActorRole::Pedestrian {
            builder = builder.add_pedestrian(&actor.id, |ped| {
                ped.pedestrian().with_dimensions(
                    PEDESTRIAN_LENGTH_M,
                    PEDESTRIAN_WIDTH_M,
                    PEDESTRIAN_HEIGHT_M,
                )
            });
        } else {
            builder = builder.add_vehicle(&actor.id, |vehicle| {
                vehicle
                    .car()
                    .with_dimensions(VEHICLE_LENGTH_M, VEHICLE_WIDTH_M, VEHICLE_HEIGHT_M)
            });
        }
    }

    // Build storyboard with init actions and trajectories
    let mut storyboard_builder = StoryboardBuilder::new(builder);

    // Add init actions for all actors (position + speed)
    let init_actions = build_init_actions(scenario, road_start_x)?;
    storyboard_builder = storyboard_builder.with_init_actions(init_actions);

    let mut story_builder = storyboard_builder.add_story_simple("main_story");

    // For each actor, create a separate Act with trajectory action
    // This avoids the library limitation where all maneuvers in an Act
    // are placed in the same ManeuverGroup, which causes esmini conflicts
    for actor in &scenario.actors {
        // Build trajectory from actor states
        let trajectory = build_trajectory(actor, &scenario.road, road_start_x)?;

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
///
/// Each vertex is a `LanePosition` referencing the same lane id the
/// companion `.xodr` uses for this physical lane (SW-15 E1), rather than a
/// bare `WorldPosition` that shares no identifier with the road network.
fn build_trajectory(
    actor: &crate::scenario::model::ActorTrajectory,
    road: &RoadSpec,
    road_start_x: f64,
) -> Result<openscenario_rs::types::actions::movement::Trajectory> {
    let mut polyline_builder = TrajectoryBuilder::new()
        .name(&format!("{}_trajectory", actor.id))
        .closed(false)
        .polyline();

    // Add vertex for each state
    for state in &actor.states {
        let heading = compute_heading(state);
        let position = lane_position(
            road,
            state.cartesian.lane,
            state.position().x,
            state.position().y,
            heading,
            road_start_x,
        )?;

        polyline_builder = polyline_builder
            .add_vertex()
            .time(state.time)
            .position(position)
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

/// Build a `LanePosition` for the given world coordinates, referencing the
/// lane id `lane_index_to_xodr_id` derives for the same physical lane in the
/// companion `.xodr` (SW-15 E1). Heading is carried via `<Orientation h="…">`
/// since `LanePosition` (unlike `WorldPosition`) has no direct heading
/// attribute.
///
/// `s` is the actor's world `x` relative to the exported road's own start:
/// `xodr_exporter::compute_road_geometry` does not always put that start at
/// `x=0` -- a backward-direction actor reaching `x<0` pulls it back to cover
/// that excursion (SW-16/M12) -- so `s` here is `x - road_start_x`, the same
/// offset the `.xodr` file's `s=0` sits at. `road_start_x` is `.0` of that
/// same `compute_road_geometry` call, computed once in
/// `export_to_xosc_impl` and threaded down here (SW-20), so this file cannot
/// silently disagree with the `.xodr` it is meant to reference. This was
/// previously flagged rather than fixed because `xodr_exporter.rs` was
/// fenced to SW-16; SW-20's `lane_ids` addendum lifted that fence for this
/// specific fix.
///
/// `z` is never emitted: the exported road never carries an elevation
/// profile (`xodr_exporter::export_to_xodr` always sets
/// `elevation_profile: None`), so there is no elevation to encode.
fn lane_position(
    road: &RoadSpec,
    lane: usize,
    x: f64,
    y: f64,
    heading: f64,
    road_start_x: f64,
) -> Result<Position> {
    let xodr_lane_id = lane_index_to_xodr_id(road, lane);
    let lane_center = lane as f64 * road.lane_width + road.lane_width / 2.0;
    let offset = y - lane_center;

    let mut position = LanePositionBuilder::new()
        .road(XODR_ROAD_ID)
        .lane(&xodr_lane_id.to_string())
        .s(x - road_start_x)
        .offset(offset)
        .finish()
        .map_err(|e| {
            crate::error::ScenarioGenError::XoscExport(format!(
                "Failed to build lane position: {}",
                e
            ))
        })?;

    if let Some(lane_position) = position.lane_position.as_mut() {
        lane_position.orientation = Some(Orientation {
            h: Some(Double::literal(heading)),
            p: None,
            r: None,
            reference_context: Some(ReferenceContext::Absolute),
        });
    }

    Ok(position)
}

/// Build a detailed scenario description with trajectory summary
///
/// Embeds key scenario information in the description field including:
/// - Scenario type and ID
/// - Time parameters
/// - Actor count and roles
/// - Trajectory summary (initial/final positions and speeds)
fn build_scenario_description(scenario: &Scenario) -> String {
    let desc = format!(
        "Scenario: {} (ID: {})\nType: {}\nDuration: {}s, Time step: {}s",
        scenario.scenario_id,
        scenario.scenario_id,
        scenario.scenario_type,
        scenario.duration,
        scenario.time_step,
    );

    desc
}

/// Compute heading angle from velocity vector
///
/// Uses atan2(vy, vx) to compute heading in radians.
/// Heading follows OpenSCENARIO convention:
/// - 0 radians = East (+X direction)
/// - π/2 radians = North (+Y direction)
fn compute_heading(state: &State) -> f64 {
    state.velocity().vy.atan2(state.velocity().vx)
}

/// Build init actions for all actors (position + speed)
///
/// Creates Init section with Private actions for each actor:
/// - TeleportAction: Sets initial world position
/// - SpeedAction: Sets initial speed from velocity magnitude
fn build_init_actions(
    scenario: &Scenario,
    road_start_x: f64,
) -> Result<openscenario_rs::types::scenario::init::Init> {
    let mut init_builder = InitActionBuilder::new();

    for actor in &scenario.actors {
        // Get initial state
        let initial_state = actor.states.first().ok_or_else(|| {
            crate::error::ScenarioGenError::XoscExport(format!("Actor {} has no states", actor.id))
        })?;

        // Calculate speed from velocity magnitude
        let speed =
            (initial_state.velocity().vx.powi(2) + initial_state.velocity().vy.powi(2)).sqrt();

        // Calculate heading from velocity
        let heading = compute_heading(initial_state);

        // Lane-referenced initial position (SW-15 E1), matching the
        // trajectory vertices below so the whole file is self-consistent.
        let position = lane_position(
            &scenario.road,
            initial_state.cartesian.lane,
            initial_state.position().x,
            initial_state.position().y,
            heading,
            road_start_x,
        )?;

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
        let road = crate::dsl::types::RoadSpec {
            num_lanes: 2,
            lane_width: 3.5,
            lane_directions: vec![1, 1],
            road_length: None,
        };
        let mut scenario = Scenario::new("cut_in_left".to_string(), 0.5, 2.0, road);

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
        assert!(xml.contains("ScenarioWeaver"));
        assert!(xml.contains("cut_in_left"));

        // Verify entities
        assert!(xml.contains("<Entities"));
        assert!(xml.contains("ego"));

        // Verify storyboard structure
        assert!(xml.contains("<Storyboard"));
        assert!(xml.contains("<Story"));
    }

    /// SW-15 E1: the exported trajectory and initial teleport reference a
    /// lane id, not just a bare world position, and that id matches what
    /// `lane_index_to_xodr_id` (shared with the `.xodr` exporter) derives
    /// for the same lane index.
    #[test]
    fn test_export_to_xosc_contains_lane_position() {
        let road = crate::dsl::types::RoadSpec {
            num_lanes: 2,
            lane_width: 3.5,
            lane_directions: vec![1, 1],
            road_length: None,
        };
        let mut scenario = Scenario::new("cut_in_left".to_string(), 0.5, 1.0, road.clone());

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
        scenario.add_actor(ego);

        let xml = export_to_xosc(&scenario).expect("export should succeed");

        assert!(
            xml.contains("LanePosition"),
            "expected a LanePosition element in the exported .xosc"
        );

        let expected_lane_id = crate::scenario::lane_ids::lane_index_to_xodr_id(&road, 1);
        assert_eq!(
            expected_lane_id, -1,
            "lane 1 of an all-forward 2-lane road is xodr id -1"
        );
        assert!(
            xml.contains(&format!("laneId=\"{expected_lane_id}\"")),
            "expected laneId=\"{expected_lane_id}\" in:\n{xml}"
        );
        assert!(
            xml.contains(&format!("roadId=\"{XODR_ROAD_ID}\"")),
            "expected roadId=\"{XODR_ROAD_ID}\""
        );

        // No bare WorldPosition should remain now that every position is
        // lane-referenced.
        assert!(
            !xml.contains("WorldPosition"),
            "expected WorldPosition to be fully replaced by LanePosition"
        );
    }

    /// SW-20 (`lane_ids` addendum): a backward-direction actor that reaches
    /// `x<0` makes `xodr_exporter::compute_road_geometry` pull the exported
    /// road's start back to cover it (SW-16/M12), so the `.xodr`'s `s=0` is
    /// no longer world `x=0`. `LanePosition`'s `s` must follow that same
    /// offset -- this is the case no example in `examples/` currently
    /// exercises (none reaches negative `x`), so it is covered here
    /// directly instead.
    #[test]
    fn test_lane_position_s_follows_road_start_for_negative_x() {
        let road = crate::dsl::types::RoadSpec {
            num_lanes: 1,
            lane_width: 3.5,
            lane_directions: vec![-1],
            road_length: None,
        };
        let mut scenario = Scenario::new("backward_actor".to_string(), 0.5, 2.0, road.clone());

        // A backward-direction (direction = -1) actor driving from x=5 to
        // x=-10: reaches well past the origin.
        let mut actor = ActorTrajectory::new("back".to_string(), "npc".to_string());
        actor.add_state(State::new(
            0.0,
            Position::new(5.0, 1.75),
            Velocity::new(-15.0, 0.0),
            Acceleration::new(0.0, 0.0),
            0,
        ));
        actor.add_state(State::new(
            1.0,
            Position::new(-10.0, 1.75),
            Velocity::new(-15.0, 0.0),
            Acceleration::new(0.0, 0.0),
            0,
        ));
        scenario.add_actor(actor);

        let (road_start_x, _) = compute_road_geometry(&scenario);
        assert!(
            road_start_x < 0.0,
            "a trajectory reaching x=-10 should pull the road start below 0, got {road_start_x}"
        );

        // `lane_position` is this file's single point of `s` computation;
        // check it directly against the offset `compute_road_geometry` just
        // reported, for both the point exactly at the pulled-back road start
        // and one 5 m past it.
        let heading = 0.0;
        let at_start = lane_position(&road, 0, road_start_x, 1.75, heading, road_start_x)
            .expect("lane_position should succeed");
        let s_at_start = at_start
            .lane_position
            .as_ref()
            .and_then(|lp| lp.s.as_literal())
            .copied()
            .expect("s should be a literal");
        assert!(
            (s_at_start - 0.0).abs() < 1e-9,
            "world x == road_start_x must map to s=0, got {s_at_start}"
        );

        let past_start = lane_position(&road, 0, road_start_x + 5.0, 1.75, heading, road_start_x)
            .expect("lane_position should succeed");
        let s_past_start = past_start
            .lane_position
            .as_ref()
            .and_then(|lp| lp.s.as_literal())
            .copied()
            .expect("s should be a literal");
        assert!(
            (s_past_start - 5.0).abs() < 1e-9,
            "world x == road_start_x + 5 must map to s=5, got {s_past_start}"
        );
    }

    /// SW-15 H8: vehicle dimensions are emitted explicitly (not left to the
    /// dependency's implicit `.car()` default) and match the constants this
    /// file documents as the DSL's assumed default.
    #[test]
    fn test_export_to_xosc_vehicle_dimensions_present() {
        let road = crate::dsl::types::RoadSpec {
            num_lanes: 2,
            lane_width: 3.5,
            lane_directions: vec![1, 1],
            road_length: None,
        };
        let mut scenario = Scenario::new("cut_in_left".to_string(), 0.5, 1.0, road);
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
        scenario.add_actor(ego);

        let xml = export_to_xosc(&scenario).expect("export should succeed");

        assert!(xml.contains(&format!("length=\"{VEHICLE_LENGTH_M}\"")));
        assert!(xml.contains(&format!("width=\"{VEHICLE_WIDTH_M}\"")));
        assert!(xml.contains(&format!("height=\"{VEHICLE_HEIGHT_M}\"")));
    }

    /// SW-15 M13: the declared revision is the one this file explicitly
    /// requests via `with_revision`, not an accidental dependency default.
    #[test]
    fn test_export_to_xosc_declares_explicit_revision() {
        let road = crate::dsl::types::RoadSpec {
            num_lanes: 2,
            lane_width: 3.5,
            lane_directions: vec![1, 1],
            road_length: None,
        };
        let mut scenario = Scenario::new("cut_in_left".to_string(), 0.5, 1.0, road);
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
        scenario.add_actor(ego);

        let xml = export_to_xosc(&scenario).expect("export should succeed");

        assert!(xml.contains(&format!("revMajor=\"{OSC_REV_MAJOR}\"")));
        assert!(xml.contains(&format!("revMinor=\"{OSC_REV_MINOR}\"")));
    }

    #[test]
    fn test_scenario_description() {
        let road = crate::dsl::types::RoadSpec {
            num_lanes: 2,
            lane_width: 3.5,
            lane_directions: vec![1, 1],
            road_length: None,
        };
        let mut scenario = Scenario::new("test_scenario".to_string(), 0.5, 10.0, road);

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
    }
}
