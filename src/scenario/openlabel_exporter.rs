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
    tags: BTreeMap<String, OpenLabelTag>,
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
    ontology_uid: &'static str,
    #[serde(rename = "type")]
    tag_type: String,
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

/// Export a scenario to OpenLabel 1.0.0 JSON format.
///
/// Generates a minimal but standards-compliant OpenLabel file containing
/// scenario metadata and semantic tags derived from the scenario content.
pub fn export_to_openlabel(scenario: &Scenario) -> Result<String> {
    let now = Utc::now().to_rfc3339();
    let scenario_id = format!("SCEN-{}", scenario.scenario_id.to_uppercase());
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

    let tags = build_tags(scenario);

    let file = OpenLabelFile {
        openlabel: OpenLabelRoot { metadata, tags },
    };

    serde_json::to_string_pretty(&file)
        .map_err(|e| ScenarioGenError::OpenLabelExport(e.to_string()))
}

// ---------------------------------------------------------------------------
// Tag helpers
// ---------------------------------------------------------------------------

fn build_tags(scenario: &Scenario) -> BTreeMap<String, OpenLabelTag> {
    let mut tag_list: Vec<&str> = Vec::new();

    // ODD: road type — always highway for single-road scenarios
    tag_list.push("highway");

    // Scenario category
    tag_list.push(&scenario.scenario_type);

    // Actor roles
    tag_list.push("ego_vehicle");

    if scenario.actors.iter().any(|a| a.role == "npc") {
        tag_list.push("npc_vehicle");
    }

    if scenario.actors.iter().any(|a| a.role == "pedestrian") {
        tag_list.push("pedestrian");
    }

    // Lane change present in trajectory
    if has_lane_change(scenario) {
        tag_list.push("lane_change");
    }

    // Adversarial / safety-critical
    if !scenario.validation.all_constraints_satisfied {
        tag_list.push("safety_critical");
    }

    // Multi-lane road
    if scenario.road.num_lanes > 2 {
        tag_list.push("multi_lane_road");
    }

    tag_list
        .into_iter()
        .enumerate()
        .map(|(i, t)| {
            (
                i.to_string(),
                OpenLabelTag {
                    ontology_uid: "0",
                    tag_type: t.to_string(),
                },
            )
        })
        .collect()
}

/// Returns true if any actor changes lane at least once during the scenario.
fn has_lane_change(scenario: &Scenario) -> bool {
    scenario.actors.iter().any(|actor| {
        actor
            .states
            .windows(2)
            .any(|w| w[0].get_lane() != w[1].get_lane())
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

    #[test]
    fn test_export_produces_valid_json() {
        let scenario = make_scenario(true);
        let result = export_to_openlabel(&scenario).expect("export should succeed");
        let parsed: serde_json::Value = serde_json::from_str(&result).expect("must be valid JSON");

        assert_eq!(parsed["openlabel"]["metadata"]["schema_version"], "1.0.0");
        assert!(parsed["openlabel"]["metadata"]["ScenarioId"]
            .as_str()
            .unwrap()
            .starts_with("SCEN-"));
    }

    #[test]
    fn test_highway_and_scenario_type_tags_always_present() {
        let scenario = make_scenario(true);
        let result = export_to_openlabel(&scenario).unwrap();
        let parsed: serde_json::Value = serde_json::from_str(&result).unwrap();

        let tags = parsed["openlabel"]["tags"].as_object().unwrap();
        let types: Vec<&str> = tags.values().map(|t| t["type"].as_str().unwrap()).collect();

        assert!(types.contains(&"highway"));
        assert!(types.contains(&"cut_in_left"));
        assert!(types.contains(&"ego_vehicle"));
        assert!(types.contains(&"npc_vehicle"));
    }

    #[test]
    fn test_safety_critical_tag_on_violation() {
        let scenario = make_scenario(false);
        let result = export_to_openlabel(&scenario).unwrap();
        let parsed: serde_json::Value = serde_json::from_str(&result).unwrap();

        let tags = parsed["openlabel"]["tags"].as_object().unwrap();
        let types: Vec<&str> = tags.values().map(|t| t["type"].as_str().unwrap()).collect();

        assert!(types.contains(&"safety_critical"));
    }

    #[test]
    fn test_no_safety_critical_tag_when_satisfied() {
        let scenario = make_scenario(true);
        let result = export_to_openlabel(&scenario).unwrap();
        let parsed: serde_json::Value = serde_json::from_str(&result).unwrap();

        let tags = parsed["openlabel"]["tags"].as_object().unwrap();
        let types: Vec<&str> = tags.values().map(|t| t["type"].as_str().unwrap()).collect();

        assert!(!types.contains(&"safety_critical"));
    }
}
