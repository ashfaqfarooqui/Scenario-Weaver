//! OpenDRIVE (.xodr) export functionality
//!
//! Converts RoadSpec and Scenario data to OpenDRIVE XML format
//! for use with CARLA and other driving simulators.

use crate::dsl::types::{RoadSpec, ScenarioSpec};
use crate::error::{Result, ScenarioGenError};
use crate::scenario::model::Scenario;

use opendrive::core::additional_data::AdditionalData;
use opendrive::core::header::Header;
use opendrive::core::OpenDrive;
use opendrive::lane::center::Center;
use opendrive::lane::center_lane::CenterLane;
use opendrive::lane::lane_choice::LaneChoice;
use opendrive::lane::lane_section::LaneSection;
use opendrive::lane::lane_type::LaneType;
use opendrive::lane::lanes::Lanes;
use opendrive::lane::left::Left;
use opendrive::lane::left_lane::LeftLane;
use opendrive::lane::right::Right;
use opendrive::lane::right_lane::RightLane;
use opendrive::lane::width::Width;
use opendrive::lane::Lane;
use opendrive::road::geometry::geometry_type::GeometryType;
use opendrive::road::geometry::line::Line;
use opendrive::road::geometry::plan_view::PlanView;
use opendrive::road::geometry::Geometry;
use opendrive::road::Road;
use uom::si::angle::radian;
use uom::si::f64::{Angle, Length};
use uom::si::length::meter;
use vec1::Vec1;

/// Configuration for OpenDRIVE export
#[derive(Debug, Clone)]
pub struct XodrExportConfig {
    /// Road length in meters (derived from scenario if None)
    pub road_length: Option<f64>,
    /// Buffer to add beyond trajectory bounds (default: 50.0)
    pub buffer: f64,
}

impl Default for XodrExportConfig {
    fn default() -> Self {
        Self {
            road_length: None,
            buffer: 50.0,
        }
    }
}

/// Export scenario road to OpenDRIVE format
pub fn export_to_xodr(scenario: &Scenario, spec: &ScenarioSpec) -> Result<String> {
    export_to_xodr_with_config(scenario, spec, XodrExportConfig::default())
}

/// Export with custom configuration
pub fn export_to_xodr_with_config(
    scenario: &Scenario,
    spec: &ScenarioSpec,
    config: XodrExportConfig,
) -> Result<String> {
    let road_spec = spec
        .road
        .clone()
        .unwrap_or_else(|| create_default_road_spec(spec));

    let road_length = config
        .road_length
        .or(road_spec.length)
        .unwrap_or_else(|| compute_road_length(scenario, config.buffer));

    // Build OpenDRIVE structure
    let opendrive = build_opendrive(&road_spec, road_length)?;

    opendrive
        .to_xml_string()
        .map_err(|e| ScenarioGenError::XodrExport(format!("Failed to serialize OpenDRIVE: {}", e)))
}

fn create_default_road_spec(spec: &ScenarioSpec) -> RoadSpec {
    RoadSpec {
        num_lanes: 2,
        lane_width: spec.lane_width,
        lane_directions: vec![1, 1], // All forward
        length: None,
    }
}

fn compute_road_length(scenario: &Scenario, buffer: f64) -> f64 {
    let mut max_x: f64 = 0.0;
    let mut min_x: f64 = f64::MAX;

    for actor in &scenario.actors {
        for state in &actor.states {
            max_x = max_x.max(state.position.x);
            min_x = min_x.min(state.position.x);
        }
    }

    // Handle edge case where min_x is still MAX (no states)
    if min_x == f64::MAX {
        min_x = 0.0;
    }

    (max_x - min_x).abs() + 2.0 * buffer
}

fn build_opendrive(road_spec: &RoadSpec, road_length: f64) -> Result<OpenDrive> {
    let header = Header {
        rev_major: 1,
        rev_minor: 7,
        name: Some("generated_road".to_string()),
        version: Some("1.0".to_string()),
        date: Some(chrono::Utc::now().format("%Y-%m-%dT%H:%M:%S").to_string()),
        north: Some(Length::new::<meter>(0.0)),
        south: Some(Length::new::<meter>(0.0)),
        east: Some(Length::new::<meter>(road_length)),
        west: Some(Length::new::<meter>(0.0)),
        vendor: Some("CARLA Scenario Generator".to_string()),
        geo_reference: None,
        offset: None,
        additional_data: AdditionalData::default(),
    };

    let road = build_road(road_spec, road_length)?;

    Ok(OpenDrive {
        header,
        road: vec![road],
        controller: vec![],
        junction: vec![],
        junction_group: vec![],
        station: vec![],
        additional_data: AdditionalData::default(),
    })
}

fn build_road(road_spec: &RoadSpec, road_length: f64) -> Result<Road> {
    // Build plan view with straight line geometry
    let geometry = Geometry {
        hdg: Angle::new::<radian>(0.0),
        length: Length::new::<meter>(road_length),
        s: Length::new::<meter>(0.0),
        x: Length::new::<meter>(0.0),
        y: Length::new::<meter>(0.0),
        r#type: GeometryType::Line(Line {}),
        additional_data: AdditionalData::default(),
    };

    let plan_view = PlanView {
        geometry: Vec1::new(geometry),
        additional_data: AdditionalData::default(),
    };

    // Build lane section
    let lane_section = build_lane_section(road_spec)?;

    let lanes = Lanes {
        lane_offset: vec![],
        lane_section: Vec1::new(lane_section),
        additional_data: AdditionalData::default(),
    };

    Ok(Road {
        id: "1".to_string(),
        junction: "-1".to_string(),
        length: Length::new::<meter>(road_length),
        name: Some("main_road".to_string()),
        rule: None,
        link: None,
        r#type: vec![],
        plan_view,
        elevation_profile: None,
        lateral_profile: None,
        lanes,
        objects: None,
        signals: None,
        surface: None,
        railroad: None,
        additional_data: AdditionalData::default(),
    })
}

fn build_lane_section(road_spec: &RoadSpec) -> Result<LaneSection> {
    // OpenDRIVE lane convention:
    // - Center lane has ID 0 (reference line, no width)
    // - Right lanes have negative IDs (-1, -2, ...) - travel in road direction (forward)
    // - Left lanes have positive IDs (1, 2, ...) - travel against road direction (backward)
    //
    // Our lane_directions convention:
    // - +1 = forward (same as road direction) -> maps to right lanes (negative IDs)
    // - -1 = backward (against road direction) -> maps to left lanes (positive IDs)

    let mut left_lanes: Vec<LeftLane> = Vec::new();
    let mut right_lanes: Vec<RightLane> = Vec::new();

    // Separate lanes by direction
    let mut forward_count = 0i64;
    let mut backward_count = 0i64;

    for (_, &direction) in road_spec.lane_directions.iter().enumerate() {
        let lane = create_base_lane(road_spec.lane_width);

        if direction == -1 {
            // Backward (against road direction) -> left side (positive IDs)
            backward_count += 1;
            left_lanes.push(LeftLane {
                id: backward_count,
                base: lane,
            });
        } else {
            // Forward (with road direction) -> right side (negative IDs)
            forward_count += 1;
            right_lanes.push(RightLane {
                id: -forward_count,
                base: lane,
            });
        }
    }

    // Build center lane (ID 0, no width)
    let center_lane = CenterLane {
        id: 0,
        base: Lane {
            link: None,
            choice: vec![],
            road_mark: vec![],
            material: vec![],
            speed: vec![],
            access: vec![],
            height: vec![],
            rule: vec![],
            level: Some(false),
            r#type: LaneType::None,
            additional_data: AdditionalData::default(),
        },
    };

    let center = Center {
        lane: Vec1::new(center_lane),
        additional_data: AdditionalData::default(),
    };

    // Build left section (backward lanes)
    let left = if left_lanes.is_empty() {
        None
    } else {
        Some(Left {
            lane: Vec1::try_from_vec(left_lanes)
                .map_err(|_| ScenarioGenError::XodrExport("No left lanes to create".to_string()))?,
            additional_data: AdditionalData::default(),
        })
    };

    // Build right section (forward lanes)
    let right = if right_lanes.is_empty() {
        None
    } else {
        Some(Right {
            lane: Vec1::try_from_vec(right_lanes).map_err(|_| {
                ScenarioGenError::XodrExport("No right lanes to create".to_string())
            })?,
            additional_data: AdditionalData::default(),
        })
    };

    Ok(LaneSection {
        s: 0.0,
        single_side: None,
        left,
        center,
        right,
        additional_data: AdditionalData::default(),
    })
}

fn create_base_lane(lane_width: f64) -> Lane {
    let width = Width {
        a: lane_width,
        b: 0.0,
        c: 0.0,
        d: 0.0,
        s_offset: Length::new::<meter>(0.0),
    };

    Lane {
        link: None,
        choice: vec![LaneChoice::Width(width)],
        road_mark: vec![],
        material: vec![],
        speed: vec![],
        access: vec![],
        height: vec![],
        rule: vec![],
        level: Some(false),
        r#type: LaneType::Driving,
        additional_data: AdditionalData::default(),
    }
}

/// Map internal lane index to OpenDRIVE lane ID
///
/// Our lanes: [0, 1, 2, 3] with directions [1, 1, -1, -1]
/// OpenDRIVE: right lanes (-1, -2) for forward, left lanes (1, 2) for backward
#[allow(dead_code)]
pub fn map_lane_to_opendrive_id(lane_index: usize, lane_directions: &[i32]) -> Option<i64> {
    if lane_index >= lane_directions.len() {
        return None;
    }

    let direction = lane_directions[lane_index];

    // Count how many lanes of the same direction come before this one
    let mut count = 0i64;
    for (i, &dir) in lane_directions.iter().enumerate() {
        if dir == direction {
            count += 1;
            if i == lane_index {
                break;
            }
        }
    }

    if direction == -1 {
        // Backward -> left side (positive IDs)
        Some(count)
    } else {
        // Forward -> right side (negative IDs)
        Some(-count)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dsl::types::{
        ActorRole, ActorSpec, ConstraintModes, OptimizationTarget, ScenarioType, ValueOrRange,
    };
    use crate::scenario::model::{ActorTrajectory, Position, State, ValidationInfo, Velocity};
    use std::collections::HashMap;

    #[test]
    fn test_compute_road_length() {
        let scenario = create_test_scenario_with_positions(0.0, 100.0);
        let length = compute_road_length(&scenario, 50.0);
        assert!((length - 200.0).abs() < 0.1); // 100m span + 2*50m buffer
    }

    #[test]
    fn test_compute_road_length_empty() {
        let scenario = Scenario {
            scenario_id: "test".to_string(),
            scenario_type: "cut_in_left".to_string(),
            duration: 10.0,
            time_step: 0.5,
            actors: vec![],
            validation: ValidationInfo {
                min_ttc: 10.0,
                min_distance: 100.0,
                all_constraints_satisfied: true,
                safety_violations: vec![],
                max_acceleration: 0.0,
                max_deceleration: 0.0,
                acceleration_violations: vec![],
            },
        };
        let length = compute_road_length(&scenario, 50.0);
        assert!((length - 100.0).abs() < 0.1); // 0m span + 2*50m buffer
    }

    #[test]
    fn test_lane_mapping_forward_only() {
        let directions = vec![1, 1];
        assert_eq!(map_lane_to_opendrive_id(0, &directions), Some(-1));
        assert_eq!(map_lane_to_opendrive_id(1, &directions), Some(-2));
    }

    #[test]
    fn test_lane_mapping_backward_only() {
        let directions = vec![-1, -1];
        assert_eq!(map_lane_to_opendrive_id(0, &directions), Some(1));
        assert_eq!(map_lane_to_opendrive_id(1, &directions), Some(2));
    }

    #[test]
    fn test_lane_mapping_bidirectional() {
        let directions = vec![1, 1, -1, -1];
        // Forward lanes
        assert_eq!(map_lane_to_opendrive_id(0, &directions), Some(-1));
        assert_eq!(map_lane_to_opendrive_id(1, &directions), Some(-2));
        // Backward lanes
        assert_eq!(map_lane_to_opendrive_id(2, &directions), Some(1));
        assert_eq!(map_lane_to_opendrive_id(3, &directions), Some(2));
    }

    #[test]
    fn test_lane_mapping_out_of_bounds() {
        let directions = vec![1, 1];
        assert_eq!(map_lane_to_opendrive_id(5, &directions), None);
    }

    #[test]
    fn test_export_produces_valid_xml() {
        let spec = create_test_spec();
        let scenario = create_test_scenario();

        let xml = export_to_xodr(&scenario, &spec).unwrap();

        assert!(xml.contains("<?xml"));
        assert!(xml.contains("OpenDRIVE"));
        assert!(xml.contains("header"));
        assert!(xml.contains("road"));
        assert!(xml.contains("lane"));
    }

    #[test]
    fn test_export_with_road_spec() {
        let mut spec = create_test_spec();
        spec.road = Some(RoadSpec {
            num_lanes: 4,
            lane_width: 3.5,
            lane_directions: vec![1, 1, -1, -1],
            length: Some(500.0),
        });

        let scenario = create_test_scenario();
        let xml = export_to_xodr(&scenario, &spec).unwrap();

        // Check for multiple lanes
        assert!(xml.contains("lane"));
        // Check for road length
        assert!(xml.contains("5e2") || xml.contains("500"));
    }

    fn create_test_spec() -> ScenarioSpec {
        ScenarioSpec {
            scenario_type: ScenarioType::CutInLeft,
            time_step: 0.5,
            duration: 10.0,
            actors: vec![
                ActorSpec {
                    id: "ego".to_string(),
                    role: ActorRole::Ego,
                    road_id: None,
                    lane: 1,
                    position: ValueOrRange::Value(50.0),
                    speed: ValueOrRange::Value(15.0),
                    acceleration: ValueOrRange::Range([-8.0, 3.0]),
                    behavior: HashMap::new(),
                },
                ActorSpec {
                    id: "npc".to_string(),
                    role: ActorRole::Npc,
                    road_id: None,
                    lane: 0,
                    position: ValueOrRange::Range([60.0, 80.0]),
                    speed: ValueOrRange::Range([12.0, 14.0]),
                    acceleration: ValueOrRange::Range([-8.0, 3.0]),
                    behavior: {
                        let mut map = HashMap::new();
                        map.insert("cut_in_time".to_string(), serde_json::json!([2.5, 7.5]));
                        map
                    },
                },
            ],
            min_ttc: 3.0,
            min_distance: 5.0,
            roads: Default::default(),
            road: None,
            lane_width: 3.5,
            num_scenarios: 1,
            constraint_modes: ConstraintModes::default(),
            max_acceleration: None,
            max_deceleration: None,
            optimization_target: OptimizationTarget::None,
        }
    }

    fn create_test_scenario() -> Scenario {
        create_test_scenario_with_positions(50.0, 100.0)
    }

    fn create_test_scenario_with_positions(min_x: f64, max_x: f64) -> Scenario {
        Scenario {
            scenario_id: "test-123".to_string(),
            scenario_type: "cut_in_left".to_string(),
            duration: 10.0,
            time_step: 0.5,
            actors: vec![ActorTrajectory {
                id: "ego".to_string(),
                role: "ego".to_string(),
                states: vec![
                    State {
                        time: 0.0,
                        position: Position { x: min_x, y: 5.0 },
                        velocity: Velocity { vx: 15.0, vy: 0.0 },
                        acceleration: crate::scenario::model::Acceleration { ax: 0.0, ay: 0.0 },
                        lane: 1,
                        road_id: None,
                    },
                    State {
                        time: 5.0,
                        position: Position { x: max_x, y: 5.0 },
                        velocity: Velocity { vx: 15.0, vy: 0.0 },
                        acceleration: crate::scenario::model::Acceleration { ax: 0.0, ay: 0.0 },
                        lane: 1,
                        road_id: None,
                    },
                ],
            }],
            validation: ValidationInfo {
                min_ttc: 5.0,
                min_distance: 10.0,
                all_constraints_satisfied: true,
                safety_violations: vec![],
                max_acceleration: 0.0,
                max_deceleration: 0.0,
                acceleration_violations: vec![],
            },
        }
    }
}
