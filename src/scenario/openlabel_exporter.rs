//! OpenLabel 1.0.0 export for scenario metadata and semantic tags

use std::collections::BTreeMap;

use chrono::Utc;
use serde::Serialize;

use crate::error::{Result, ScenarioGenError};
use crate::scenario::model::Scenario;

// ---------------------------------------------------------------------------
// JSON structure
// ---------------------------------------------------------------------------

#[derive(Serialize)]
struct OpenLabelFile {
    openlabel: OpenLabelRoot,
}

#[derive(Serialize)]
struct OpenLabelRoot {
    metadata: OpenLabelMetadata,
    ontologies: BTreeMap<String, OpenLabelOntology>,
    tags: BTreeMap<String, OpenLabelTag>,
}

#[derive(Serialize)]
struct OpenLabelOntology {
    uri: &'static str,
}

#[derive(Serialize)]
struct OpenLabelMetadata {
    schema_version: &'static str,
    file_version: &'static str,
    annotator: &'static str,
    comment: String,
    #[serde(rename = "ScenarioId")]
    scenario_id: String,
    #[serde(rename = "Name")]
    name: String,
    #[serde(rename = "Description")]
    description: String,
    #[serde(rename = "Image")]
    image: &'static str,
    #[serde(rename = "ScenarioDatabase")]
    scenario_database: &'static str,
    #[serde(rename = "CreateDate")]
    create_date: String,
    #[serde(rename = "ModifyDate")]
    modify_date: String,
    #[serde(rename = "Creator")]
    creator: &'static str,
    #[serde(rename = "Generator")]
    generator: GeneratorInfo,
}

#[derive(Serialize)]
struct GeneratorInfo {
    #[serde(rename = "Name")]
    name: &'static str,
    #[serde(rename = "Version")]
    version: &'static str,
}

#[derive(Serialize)]
struct OpenLabelTag {
    #[serde(rename = "type")]
    tag_type: String,
    ontology_uid: &'static str,
    #[serde(skip_serializing_if = "Option::is_none")]
    tag_data: Option<TagData>,
}

#[derive(Serialize)]
struct TagData {
    num: Vec<TagValue>,
}

#[derive(Serialize)]
struct TagValue {
    #[serde(rename = "type")]
    val_type: &'static str,
    val: u32,
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

/// Export a scenario to OpenLabel 1.0.0 JSON format.
pub fn export_to_openlabel(scenario: &Scenario) -> Result<String> {
    let now = Utc::now().to_rfc3339();
    let scenario_id = format!("SW-{}", scenario.scenario_id.to_uppercase());
    let comment = format!(
        "Generated {} scenario with {} actor(s)",
        scenario.scenario_type,
        scenario.actors.len()
    );

    let metadata = OpenLabelMetadata {
        schema_version: "1.0.0",
        file_version: "1.0",
        annotator: "ScenarioWeaver",
        comment: comment.clone(),
        scenario_id: scenario_id.clone(),
        name: scenario_id,
        description: comment,
        image: "",
        scenario_database: "SCENARIOWEAVER",
        create_date: now.clone(),
        modify_date: now,
        creator: "ScenarioWeaver_2026",
        generator: GeneratorInfo {
            name: "ScenarioWeaver",
            version: env!("CARGO_PKG_VERSION"),
        },
    };

    let mut ontologies = BTreeMap::new();
    ontologies.insert(
        "0".to_string(),
        OpenLabelOntology {
            uri: "https://openlabel.asam.net/V1-0-0/ontologies/",
        },
    );

    let tags = build_tags(scenario);

    let file = OpenLabelFile {
        openlabel: OpenLabelRoot {
            metadata,
            ontologies,
            tags,
        },
    };

    serde_json::to_string_pretty(&file)
        .map_err(|e| ScenarioGenError::OpenLabelExport(e.to_string()))
}

// ---------------------------------------------------------------------------
// Tag helpers
// ---------------------------------------------------------------------------

fn build_tags(scenario: &Scenario) -> BTreeMap<String, OpenLabelTag> {
    let mut tags: Vec<OpenLabelTag> = Vec::new();

    // Motion: map scenario type to ontology motion tag
    let motion_tag = match scenario.scenario_type.as_str() {
        t if t.contains("cut_in") => "MotionCutIn",
        t if t.contains("cut_out") => "MotionCutOut",
        t if t.contains("overtake") => "MotionOvertake",
        _ => "MotionDrive",
    };
    tags.push(simple_tag(motion_tag));

    // Motion: lane change direction(s) detected from trajectories
    if has_lane_change_left(scenario) {
        tags.push(simple_tag("MotionLaneChangeLeft"));
    }
    if has_lane_change_right(scenario) {
        tags.push(simple_tag("MotionLaneChangeRight"));
    }

    // Motion: accelerate/decelerate detected from trajectory ax values
    if has_acceleration(scenario) {
        tags.push(simple_tag("MotionAccelerate"));
    }
    if has_deceleration(scenario) {
        tags.push(simple_tag("MotionDecelerate"));
    }

    // Vehicle types — only when at least one non-pedestrian actor exists
    if scenario.actors.iter().any(|a| a.role != "pedestrian") {
        tags.push(simple_tag("VehicleCar"));
    }

    // Human roles — conditional on actor roles
    if scenario.actors.iter().any(|a| a.role == "ego") {
        tags.push(simple_tag("HumanDriver"));
    }
    if scenario.actors.iter().any(|a| a.role == "pedestrian") {
        tags.push(simple_tag("HumanPedestrian"));
    }

    // Road type — inferred from lane count and directionality
    let all_same_direction = scenario
        .road
        .lane_directions
        .windows(2)
        .all(|w| w[0] == w[1]);
    let road_type_tag = if scenario.road.num_lanes >= 3 && all_same_direction {
        "RoadTypeMotorway"
    } else if scenario.road.num_lanes == 2 && all_same_direction {
        "RoadTypeDistributor"
    } else {
        "RoadTypeMinor"
    };
    tags.push(simple_tag(road_type_tag));

    // Lane travel direction — always emit the category tag, then directional tags
    tags.push(simple_tag("LaneSpecificationTravelDirection"));
    if scenario.road.lane_directions.iter().any(|&d| d == 1) {
        tags.push(simple_tag("TravelDirectionRight"));
    }
    if scenario.road.lane_directions.iter().any(|&d| d == -1) {
        tags.push(simple_tag("TravelDirectionLeft"));
    }

    // Special structure — pedestrian crossing
    if scenario.scenario_type.contains("pedestrian") {
        tags.push(simple_tag("SpecialStructurePedestrianCrossing"));
    }

    // Zone tags
    if scenario.scenario_type.contains("school") {
        tags.push(simple_tag("ZoneSchool"));
    }

    // Lane count with tag_data
    tags.push(OpenLabelTag {
        tag_type: "LaneSpecificationLaneCount".to_string(),
        ontology_uid: "0",
        tag_data: Some(TagData {
            num: vec![TagValue {
                val_type: "value",
                val: scenario.road.num_lanes as u32,
            }],
        }),
    });

    tags.into_iter()
        .enumerate()
        .map(|(i, t)| (i.to_string(), t))
        .collect()
}

/// Returns true if any actor has any state with ax > 0.5 (meaningful acceleration).
fn has_acceleration(scenario: &Scenario) -> bool {
    scenario
        .actors
        .iter()
        .any(|actor| actor.states.iter().any(|s| s.cartesian.acceleration.ax > 0.5))
}

/// Returns true if any actor has any state with ax < -0.5 (meaningful deceleration).
fn has_deceleration(scenario: &Scenario) -> bool {
    scenario
        .actors
        .iter()
        .any(|actor| actor.states.iter().any(|s| s.cartesian.acceleration.ax < -0.5))
}

fn simple_tag(name: &str) -> OpenLabelTag {
    OpenLabelTag {
        tag_type: name.to_string(),
        ontology_uid: "0",
        tag_data: None,
    }
}

/// Returns true if any actor moves to a higher-numbered lane (left change).
fn has_lane_change_left(scenario: &Scenario) -> bool {
    scenario.actors.iter().any(|actor| {
        actor
            .states
            .windows(2)
            .any(|w| w[1].get_lane() > w[0].get_lane())
    })
}

/// Returns true if any actor moves to a lower-numbered lane (right change).
fn has_lane_change_right(scenario: &Scenario) -> bool {
    scenario.actors.iter().any(|actor| {
        actor
            .states
            .windows(2)
            .any(|w| w[1].get_lane() < w[0].get_lane())
    })
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dsl::types::RoadSpec;
    use crate::scenario::model::{
        Acceleration, ActorTrajectory, Position, Scenario, State, ValidationInfo, Velocity,
    };

    fn make_scenario(all_satisfied: bool) -> Scenario {
        let road = RoadSpec {
            num_lanes: 2,
            lane_width: 3.5,
            lane_directions: vec![1, 1],
            road_length: None,
        };
        let mut scenario = Scenario::new("cut_in_left".to_string(), 0.1, 5.0, road);
        scenario.validation = ValidationInfo {
            min_ttc: 3.0,
            min_distance: 10.0,
            all_constraints_satisfied: all_satisfied,
            safety_violations: vec![],
            max_acceleration: 2.0,
            max_deceleration: -3.0,
            acceleration_violations: vec![],
        };

        let mut ego = ActorTrajectory::new("ego".to_string(), "ego".to_string());
        ego.add_state(State::new(
            0.0,
            Position::new(50.0, 5.25),
            Velocity::new(15.0, 0.0),
            Acceleration::new(0.0, 0.0),
            1,
        ));
        scenario.add_actor(ego);

        let mut npc = ActorTrajectory::new("npc".to_string(), "npc".to_string());
        npc.add_state(State::new(
            0.0,
            Position::new(70.0, 1.75),
            Velocity::new(18.0, 0.0),
            Acceleration::new(0.0, 0.0),
            0,
        ));
        scenario.add_actor(npc);

        scenario
    }

    fn make_scenario_with_lane_change(left: bool) -> Scenario {
        let road = RoadSpec {
            num_lanes: 2,
            lane_width: 3.5,
            lane_directions: vec![1, 1],
            road_length: None,
        };
        let mut scenario = Scenario::new("cut_in_left".to_string(), 0.1, 5.0, road);
        scenario.validation = ValidationInfo {
            min_ttc: 3.0,
            min_distance: 10.0,
            all_constraints_satisfied: true,
            safety_violations: vec![],
            max_acceleration: 2.0,
            max_deceleration: -3.0,
            acceleration_violations: vec![],
        };

        let mut npc = ActorTrajectory::new("npc".to_string(), "npc".to_string());
        let (start_lane, end_lane) = if left { (0, 1) } else { (1, 0) };
        npc.add_state(State::new(
            0.0,
            Position::new(70.0, 1.75),
            Velocity::new(18.0, 0.0),
            Acceleration::new(0.0, 0.0),
            start_lane,
        ));
        npc.add_state(State::new(
            0.1,
            Position::new(71.8, 3.5),
            Velocity::new(18.0, 1.0),
            Acceleration::new(0.0, 0.0),
            end_lane,
        ));
        scenario.add_actor(npc);
        scenario
    }

    #[test]
    fn test_export_produces_valid_json() {
        let scenario = make_scenario(true);
        let result = export_to_openlabel(&scenario).expect("export should succeed");
        let parsed: serde_json::Value = serde_json::from_str(&result).expect("must be valid JSON");

        assert_eq!(parsed["openlabel"]["metadata"]["schema_version"], "1.0.0");
        assert!(parsed["openlabel"]["metadata"]["ScenarioId"]
            .as_str()
            .unwrap()
            .starts_with("SW-"));
    }

    #[test]
    fn test_ontologies_section_present() {
        let scenario = make_scenario(true);
        let result = export_to_openlabel(&scenario).unwrap();
        let parsed: serde_json::Value = serde_json::from_str(&result).unwrap();

        assert_eq!(
            parsed["openlabel"]["ontologies"]["0"]["uri"],
            "https://openlabel.asam.net/V1-0-0/ontologies/"
        );
    }

    #[test]
    fn test_ontology_tags_always_present() {
        // make_scenario: 2-lane unidirectional [1,1] with ego + npc actors
        let scenario = make_scenario(true);
        let result = export_to_openlabel(&scenario).unwrap();
        let parsed: serde_json::Value = serde_json::from_str(&result).unwrap();

        let tags = parsed["openlabel"]["tags"].as_object().unwrap();
        let types: Vec<&str> = tags.values().map(|t| t["type"].as_str().unwrap()).collect();

        // 2-lane unidirectional → RoadTypeDistributor (not Motorway)
        assert!(types.contains(&"RoadTypeDistributor"), "expected RoadTypeDistributor, got: {:?}", types);
        assert!(!types.contains(&"RoadTypeMotorway"), "RoadTypeMotorway must not appear for 2-lane road");
        // ego actor present → VehicleCar and HumanDriver
        assert!(types.contains(&"VehicleCar"));
        assert!(types.contains(&"HumanDriver"));
        assert!(types.contains(&"MotionCutIn"));
        assert!(types.contains(&"LaneSpecificationLaneCount"));
        // old invented tags must be gone
        assert!(!types.contains(&"highway"));
        assert!(!types.contains(&"ego_vehicle"));
        assert!(!types.contains(&"npc_vehicle"));
    }

    #[test]
    fn test_road_type_motorway() {
        // 4-lane unidirectional → RoadTypeMotorway
        let road = RoadSpec {
            num_lanes: 4,
            lane_width: 3.5,
            lane_directions: vec![1, 1, 1, 1],
            road_length: None,
        };
        let mut scenario = Scenario::new("cut_in_left".to_string(), 0.1, 5.0, road);
        scenario.validation = ValidationInfo {
            min_ttc: 3.0, min_distance: 10.0, all_constraints_satisfied: true,
            safety_violations: vec![], max_acceleration: 2.0, max_deceleration: -3.0,
            acceleration_violations: vec![],
        };
        let mut ego = ActorTrajectory::new("ego".to_string(), "ego".to_string());
        ego.add_state(State::new(0.0, Position::new(50.0, 5.25), Velocity::new(15.0, 0.0), Acceleration::new(0.0, 0.0), 1));
        scenario.add_actor(ego);

        let result = export_to_openlabel(&scenario).unwrap();
        let parsed: serde_json::Value = serde_json::from_str(&result).unwrap();
        let tags = parsed["openlabel"]["tags"].as_object().unwrap();
        let types: Vec<&str> = tags.values().map(|t| t["type"].as_str().unwrap()).collect();
        assert!(types.contains(&"RoadTypeMotorway"), "expected RoadTypeMotorway for 4-lane unidirectional");
        assert!(!types.contains(&"RoadTypeDistributor"));
        assert!(!types.contains(&"RoadTypeMinor"));
    }

    #[test]
    fn test_road_type_minor_bidirectional() {
        // 4-lane bidirectional [1,1,-1,-1] → RoadTypeMinor
        let road = RoadSpec {
            num_lanes: 4,
            lane_width: 3.5,
            lane_directions: vec![1, 1, -1, -1],
            road_length: None,
        };
        let mut scenario = Scenario::new("cut_in_left".to_string(), 0.1, 5.0, road);
        scenario.validation = ValidationInfo {
            min_ttc: 3.0, min_distance: 10.0, all_constraints_satisfied: true,
            safety_violations: vec![], max_acceleration: 2.0, max_deceleration: -3.0,
            acceleration_violations: vec![],
        };
        let mut ego = ActorTrajectory::new("ego".to_string(), "ego".to_string());
        ego.add_state(State::new(0.0, Position::new(50.0, 5.25), Velocity::new(15.0, 0.0), Acceleration::new(0.0, 0.0), 1));
        scenario.add_actor(ego);

        let result = export_to_openlabel(&scenario).unwrap();
        let parsed: serde_json::Value = serde_json::from_str(&result).unwrap();
        let tags = parsed["openlabel"]["tags"].as_object().unwrap();
        let types: Vec<&str> = tags.values().map(|t| t["type"].as_str().unwrap()).collect();
        assert!(types.contains(&"RoadTypeMinor"), "expected RoadTypeMinor for bidirectional road");
        assert!(!types.contains(&"RoadTypeMotorway"));
        assert!(!types.contains(&"RoadTypeDistributor"));
    }

    #[test]
    fn test_pedestrian_crossing_tag() {
        let road = RoadSpec {
            num_lanes: 2,
            lane_width: 3.5,
            lane_directions: vec![1, 1],
            road_length: None,
        };
        let mut scenario = Scenario::new("pedestrian_crossing".to_string(), 0.1, 5.0, road);
        scenario.validation = ValidationInfo {
            min_ttc: 3.0, min_distance: 10.0, all_constraints_satisfied: true,
            safety_violations: vec![], max_acceleration: 2.0, max_deceleration: -3.0,
            acceleration_violations: vec![],
        };
        let mut ego = ActorTrajectory::new("ego".to_string(), "ego".to_string());
        ego.add_state(State::new(0.0, Position::new(50.0, 5.25), Velocity::new(15.0, 0.0), Acceleration::new(0.0, 0.0), 1));
        scenario.add_actor(ego);

        let result = export_to_openlabel(&scenario).unwrap();
        let parsed: serde_json::Value = serde_json::from_str(&result).unwrap();
        let tags = parsed["openlabel"]["tags"].as_object().unwrap();
        let types: Vec<&str> = tags.values().map(|t| t["type"].as_str().unwrap()).collect();
        assert!(types.contains(&"SpecialStructurePedestrianCrossing"));
    }

    #[test]
    fn test_travel_direction_tags_unidirectional() {
        // All +1 lanes → TravelDirectionRight only
        let scenario = make_scenario(true); // uses [1, 1]
        let result = export_to_openlabel(&scenario).unwrap();
        let parsed: serde_json::Value = serde_json::from_str(&result).unwrap();
        let tags = parsed["openlabel"]["tags"].as_object().unwrap();
        let types: Vec<&str> = tags.values().map(|t| t["type"].as_str().unwrap()).collect();
        assert!(types.contains(&"LaneSpecificationTravelDirection"));
        assert!(types.contains(&"TravelDirectionRight"));
        assert!(!types.contains(&"TravelDirectionLeft"));
    }

    #[test]
    fn test_travel_direction_tags_bidirectional() {
        // Mixed [1,1,-1,-1] → both TravelDirectionRight AND TravelDirectionLeft
        let road = RoadSpec {
            num_lanes: 4,
            lane_width: 3.5,
            lane_directions: vec![1, 1, -1, -1],
            road_length: None,
        };
        let mut scenario = Scenario::new("cut_in_left".to_string(), 0.1, 5.0, road);
        scenario.validation = ValidationInfo {
            min_ttc: 3.0, min_distance: 10.0, all_constraints_satisfied: true,
            safety_violations: vec![], max_acceleration: 2.0, max_deceleration: -3.0,
            acceleration_violations: vec![],
        };
        let mut ego = ActorTrajectory::new("ego".to_string(), "ego".to_string());
        ego.add_state(State::new(0.0, Position::new(50.0, 5.25), Velocity::new(15.0, 0.0), Acceleration::new(0.0, 0.0), 1));
        scenario.add_actor(ego);

        let result = export_to_openlabel(&scenario).unwrap();
        let parsed: serde_json::Value = serde_json::from_str(&result).unwrap();
        let tags = parsed["openlabel"]["tags"].as_object().unwrap();
        let types: Vec<&str> = tags.values().map(|t| t["type"].as_str().unwrap()).collect();
        assert!(types.contains(&"TravelDirectionRight"));
        assert!(types.contains(&"TravelDirectionLeft"));
    }

    #[test]
    fn test_motion_accelerate_decelerate() {
        let road = RoadSpec {
            num_lanes: 2,
            lane_width: 3.5,
            lane_directions: vec![1, 1],
            road_length: None,
        };
        let mut scenario = Scenario::new("cut_in_left".to_string(), 0.1, 5.0, road);
        scenario.validation = ValidationInfo {
            min_ttc: 3.0, min_distance: 10.0, all_constraints_satisfied: true,
            safety_violations: vec![], max_acceleration: 2.0, max_deceleration: -3.0,
            acceleration_violations: vec![],
        };

        // Actor with ax > 0.5 (accelerating)
        let mut ego = ActorTrajectory::new("ego".to_string(), "ego".to_string());
        ego.add_state(State::new(0.0, Position::new(50.0, 5.25), Velocity::new(15.0, 0.0), Acceleration::new(1.5, 0.0), 1));
        scenario.add_actor(ego);

        // Actor with ax < -0.5 (decelerating)
        let mut npc = ActorTrajectory::new("npc".to_string(), "npc".to_string());
        npc.add_state(State::new(0.0, Position::new(70.0, 1.75), Velocity::new(18.0, 0.0), Acceleration::new(-2.0, 0.0), 0));
        scenario.add_actor(npc);

        let result = export_to_openlabel(&scenario).unwrap();
        let parsed: serde_json::Value = serde_json::from_str(&result).unwrap();
        let tags = parsed["openlabel"]["tags"].as_object().unwrap();
        let types: Vec<&str> = tags.values().map(|t| t["type"].as_str().unwrap()).collect();
        assert!(types.contains(&"MotionAccelerate"), "expected MotionAccelerate for ax=1.5");
        assert!(types.contains(&"MotionDecelerate"), "expected MotionDecelerate for ax=-2.0");
    }

    #[test]
    fn test_no_vehicle_car_for_pedestrian_only() {
        let road = RoadSpec {
            num_lanes: 2,
            lane_width: 3.5,
            lane_directions: vec![1, 1],
            road_length: None,
        };
        let mut scenario = Scenario::new("pedestrian_crossing".to_string(), 0.1, 5.0, road);
        scenario.validation = ValidationInfo {
            min_ttc: 3.0, min_distance: 10.0, all_constraints_satisfied: true,
            safety_violations: vec![], max_acceleration: 2.0, max_deceleration: -3.0,
            acceleration_violations: vec![],
        };

        let mut ped = ActorTrajectory::new("ped".to_string(), "pedestrian".to_string());
        ped.add_state(State::new(0.0, Position::new(30.0, 0.0), Velocity::new(1.0, 0.0), Acceleration::new(0.0, 0.0), 0));
        scenario.add_actor(ped);

        let result = export_to_openlabel(&scenario).unwrap();
        let parsed: serde_json::Value = serde_json::from_str(&result).unwrap();
        let tags = parsed["openlabel"]["tags"].as_object().unwrap();
        let types: Vec<&str> = tags.values().map(|t| t["type"].as_str().unwrap()).collect();
        assert!(!types.contains(&"VehicleCar"), "VehicleCar must not appear in pedestrian-only scenario");
        assert!(types.contains(&"HumanPedestrian"));
    }

    #[test]
    fn test_lane_count_tag_data() {
        let scenario = make_scenario(true);
        let result = export_to_openlabel(&scenario).unwrap();
        let parsed: serde_json::Value = serde_json::from_str(&result).unwrap();

        let tags = parsed["openlabel"]["tags"].as_object().unwrap();
        let lane_tag = tags
            .values()
            .find(|t| t["type"] == "LaneSpecificationLaneCount")
            .expect("LaneSpecificationLaneCount tag must exist");

        assert_eq!(lane_tag["tag_data"]["num"][0]["val"], 2);
        assert_eq!(lane_tag["tag_data"]["num"][0]["type"], "value");
    }

    #[test]
    fn test_no_safety_critical_tag() {
        // safety_critical is not an ontology term; must never appear
        let scenario = make_scenario(false);
        let result = export_to_openlabel(&scenario).unwrap();
        let parsed: serde_json::Value = serde_json::from_str(&result).unwrap();

        let tags = parsed["openlabel"]["tags"].as_object().unwrap();
        let types: Vec<&str> = tags.values().map(|t| t["type"].as_str().unwrap()).collect();
        assert!(!types.contains(&"safety_critical"));
    }

    #[test]
    fn test_lane_change_left_detection() {
        let scenario = make_scenario_with_lane_change(true);
        let result = export_to_openlabel(&scenario).unwrap();
        let parsed: serde_json::Value = serde_json::from_str(&result).unwrap();

        let tags = parsed["openlabel"]["tags"].as_object().unwrap();
        let types: Vec<&str> = tags.values().map(|t| t["type"].as_str().unwrap()).collect();
        assert!(types.contains(&"MotionLaneChangeLeft"));
        assert!(!types.contains(&"MotionLaneChangeRight"));
    }

    #[test]
    fn test_lane_change_right_detection() {
        let scenario = make_scenario_with_lane_change(false);
        let result = export_to_openlabel(&scenario).unwrap();
        let parsed: serde_json::Value = serde_json::from_str(&result).unwrap();

        let tags = parsed["openlabel"]["tags"].as_object().unwrap();
        let types: Vec<&str> = tags.values().map(|t| t["type"].as_str().unwrap()).collect();
        assert!(types.contains(&"MotionLaneChangeRight"));
        assert!(!types.contains(&"MotionLaneChangeLeft"));
    }
}
