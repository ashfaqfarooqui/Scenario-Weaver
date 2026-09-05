//! OpenLabel 1.0.0 export for scenario metadata and semantic tags

use std::collections::BTreeMap;

use chrono::Utc;
use serde::Serialize;

use crate::dsl::types::ActorRole;
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
    objects: BTreeMap<String, OpenLabelObject>,
    frames: BTreeMap<String, OpenLabelFrame>,
    tags: BTreeMap<String, OpenLabelTag>,
}

#[derive(Serialize)]
struct OpenLabelOntology {
    uri: &'static str,
}

/// OpenLABEL `metadata` object. Only real ASAM OpenLABEL 1.0.0 schema fields
/// (`schema_version`, `file_version`, `annotator`, `comment`) sit at the top
/// level; everything this project adds that the schema does not define lives
/// under the namespaced `scenario_weaver` extension object instead of as
/// bare, non-schema PascalCase keys (SW-17 M14).
#[derive(Serialize)]
struct OpenLabelMetadata {
    schema_version: &'static str,
    file_version: &'static str,
    annotator: &'static str,
    comment: String,
    scenario_weaver: ScenarioWeaverExtension,
}

/// Non-schema extension metadata, namespaced under `metadata.scenario_weaver`
/// so it cannot be mistaken for part of the ASAM OpenLABEL schema itself.
///
/// `creator` is derived from the crate's own version at compile time
/// (`env!("CARGO_PKG_VERSION")`), never a hand-written year baked into a
/// `&'static str`, so it cannot go stale the way the old literal did (SW-17
/// M14). The old `Image` (always `""`)
/// and `ScenarioDatabase` (`"SCENARIOWEAVER"`, a name with no referent) fields
/// carried no real information and are dropped rather than namespaced. `Name`
/// and `Description` duplicated `scenario_id`/`comment` one level up and are
/// likewise dropped.
#[derive(Serialize)]
struct ScenarioWeaverExtension {
    scenario_id: String,
    create_date: String,
    modify_date: String,
    creator: String,
    generator: GeneratorInfo,
}

#[derive(Serialize)]
struct GeneratorInfo {
    name: &'static str,
    version: &'static str,
}

/// One tracked object (actor), keyed by a stable per-scenario numeric uid in
/// `OpenLabelRoot::objects`. `name` matches the corresponding `.xosc`
/// `<ScenarioObject name="...">` exactly (both derive from `ActorTrajectory::id`
/// via `xosc_exporter::export_to_xosc_impl`'s `add_vehicle`/`add_pedestrian`
/// calls), which is what makes the `.ol.json` and `.xosc` for one scenario
/// joinable by name (SW-17 E4 / D4).
#[derive(Serialize)]
struct OpenLabelObject {
    name: String,
    #[serde(rename = "type")]
    object_type: &'static str,
}

/// One timestep. Keyed by a stable per-scenario frame index in
/// `OpenLabelRoot::frames`.
#[derive(Serialize)]
struct OpenLabelFrame {
    frame_properties: OpenLabelFrameProperties,
    objects: BTreeMap<String, OpenLabelFrameObjectEntry>,
}

#[derive(Serialize)]
struct OpenLabelFrameProperties {
    timestamp: String,
}

#[derive(Serialize)]
struct OpenLabelFrameObjectEntry {
    object_data: OpenLabelObjectData,
}

#[derive(Serialize)]
struct OpenLabelObjectData {
    vec: Vec<OpenLabelVec>,
}

#[derive(Serialize)]
struct OpenLabelVec {
    name: &'static str,
    val: Vec<f64>,
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
        comment,
        scenario_weaver: ScenarioWeaverExtension {
            scenario_id,
            create_date: now.clone(),
            modify_date: now,
            creator: format!("ScenarioWeaver-{}", env!("CARGO_PKG_VERSION")),
            generator: GeneratorInfo {
                name: "ScenarioWeaver",
                version: env!("CARGO_PKG_VERSION"),
            },
        },
    };

    let mut ontologies = BTreeMap::new();
    ontologies.insert(
        "0".to_string(),
        OpenLabelOntology {
            uri: "https://openlabel.asam.net/V1-0-0/ontologies/",
        },
    );

    let (objects, frames) = build_objects_and_frames(scenario);
    let tags = build_tags(scenario);

    let file = OpenLabelFile {
        openlabel: OpenLabelRoot {
            metadata,
            ontologies,
            objects,
            frames,
            tags,
        },
    };

    serde_json::to_string_pretty(&file)
        .map_err(|e| ScenarioGenError::OpenLabelExport(e.to_string()))
}

// ---------------------------------------------------------------------------
// Tag helpers
// ---------------------------------------------------------------------------

/// Minimum absolute acceleration (m/s²) to emit MotionAccelerate/MotionDecelerate tags.
/// Chosen above typical sensor noise floor to avoid tagging near-constant-speed trajectories.
const ACCELERATION_TAG_THRESHOLD: f64 = 0.5;

fn build_tags(scenario: &Scenario) -> BTreeMap<String, OpenLabelTag> {
    debug_assert_eq!(
        scenario.road.num_lanes,
        scenario.road.lane_directions.len(),
        "RoadSpec invariant violated: num_lanes != lane_directions.len()"
    );

    let mut tags: Vec<OpenLabelTag> = Vec::new();

    // Motion: map scenario type to ontology motion tag
    let motion_tag = match scenario.scenario_type.as_str() {
        t if t.contains("cut_in") => "MotionCutIn",
        t if t.contains("cut_out") => "MotionCutOut",
        t if t.contains("overtake") => "MotionOvertake",
        t if t.contains("pedestrian") => "MotionWalk",
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
    if scenario
        .actors
        .iter()
        .any(|a| a.role != ActorRole::Pedestrian)
    {
        tags.push(simple_tag("VehicleCar"));
    }

    // Human roles — conditional on actor roles
    if scenario.actors.iter().any(|a| a.role == ActorRole::Ego) {
        tags.push(simple_tag("HumanDriver"));
    }
    if scenario
        .actors
        .iter()
        .any(|a| a.role == ActorRole::Pedestrian)
    {
        tags.push(simple_tag("HumanPedestrian"));
    }

    // SW-17 M14: a `RoadTypeMotorway`/`RoadTypeDistributor`/`RoadTypeMinor` heuristic
    // used to be inferred here from lane count and directionality alone and pushed
    // under the ontology URI as if it were a real classification. It was not: a
    // 3-lane `cut_in_left` and a 2-lane `pedestrian_crossing` landed in different
    // buckets purely from lane count, with nothing behind the label but a guess (see
    // MASTER.md / FINDINGS.md M14). ASAM OpenLABEL's ontology mechanism is meant for
    // vocabularies a project actually defines and stands behind (confirmed against
    // https://www.asam.net/standards/detail/openlabel/: "organizations can import
    // their own ontologies... rather than relying on predefined classifications"),
    // not a proxy computed from unrelated fields. Emitting no road-type tag is more
    // honest than emitting a wrong one. A `road_class` field on `RoadSpec` would let
    // this be reintroduced correctly.

    // Lane travel direction — only emit when every lane agrees on a single
    // direction. A bidirectional road (mixed `+1`/`-1`) has no single travel
    // direction to report; emitting both `TravelDirectionRight` and
    // `TravelDirectionLeft` for every such road conveyed nothing (SW-17 M14).
    let lane_count = scenario.road.lane_directions.len();
    if lane_count > 0 {
        let all_right = scenario.road.lane_directions.iter().all(|&d| d == 1);
        let all_left = scenario.road.lane_directions.iter().all(|&d| d == -1);
        if all_right {
            tags.push(simple_tag("LaneSpecificationTravelDirection"));
            tags.push(simple_tag("TravelDirectionRight"));
        } else if all_left {
            tags.push(simple_tag("LaneSpecificationTravelDirection"));
            tags.push(simple_tag("TravelDirectionLeft"));
        }
    }

    // Special structure — pedestrian crossing
    if scenario.scenario_type.contains("pedestrian") {
        tags.push(simple_tag("SpecialStructurePedestrianCrossing"));
    }

    // ZoneSchool: emit when a school_zone scenario type is added

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

    tags.into_iter().map(|t| (t.tag_type.clone(), t)).collect()
}

/// Returns true if any actor has any state with ax > threshold (meaningful acceleration).
fn has_acceleration(scenario: &Scenario) -> bool {
    scenario.actors.iter().any(|actor| {
        actor
            .states
            .iter()
            .any(|s| s.cartesian.acceleration.ax > ACCELERATION_TAG_THRESHOLD)
    })
}

/// Returns true if any actor has any state with ax < -threshold (meaningful deceleration).
fn has_deceleration(scenario: &Scenario) -> bool {
    scenario.actors.iter().any(|actor| {
        actor
            .states
            .iter()
            .any(|s| s.cartesian.acceleration.ax < -ACCELERATION_TAG_THRESHOLD)
    })
}

fn simple_tag(name: &str) -> OpenLabelTag {
    OpenLabelTag {
        tag_type: name.to_string(),
        ontology_uid: "0",
        tag_data: None,
    }
}

/// Returns true if any actor moves to a lower-numbered lane (a **left**
/// change).
///
/// SW-17 E5: this used to call an *increasing* lane index "left", which
/// disagreed with both the DSL and the encoder. `RoadSpec` lane index counts
/// up with y (`cartesian.rs`'s `encode_lane_position_coupling_at_time`:
/// `py = lane * lane_width + lane_width / 2`, so lane 0 is at `y = 1.7` and
/// lane 1 is at `y = 5.2` for a 3.5 m lane — verified, not assumed), and
/// `cartesian.rs`'s `encode_smooth_lane_transition` maps
/// `LaneChangeDirection::Right` to `lane_delta = actor_direction` — `+1` for
/// the default forward-travelling actor — i.e. **`Right` increases the lane
/// index**. `types.rs`'s `LaneChangeDirection` doc comment now says the same
/// thing; this function was the one place still disagreeing with the
/// encoder it exists to describe.
fn has_lane_change_left(scenario: &Scenario) -> bool {
    scenario.actors.iter().any(|actor| {
        actor
            .states
            .windows(2)
            .any(|w| w[1].get_lane() < w[0].get_lane())
    })
}

/// Returns true if any actor moves to a higher-numbered lane (a **right**
/// change). See [`has_lane_change_left`] for why this is the increasing
/// direction.
fn has_lane_change_right(scenario: &Scenario) -> bool {
    scenario.actors.iter().any(|actor| {
        actor
            .states
            .windows(2)
            .any(|w| w[1].get_lane() > w[0].get_lane())
    })
}

// ---------------------------------------------------------------------------
// Objects / frames (SW-17 E4, decision D4 — Route A)
// ---------------------------------------------------------------------------

/// Build the `objects` and `frames` sections: one object per actor (named to
/// match the `.xosc` `<ScenarioObject name="...">` for the same actor) and
/// one frame per timestep carrying each actor's position at that time, so a
/// `.ol.json` can be joined to its sibling `.xosc`/`.json` by actor name.
///
/// Frames are built defensively rather than assuming every actor has the
/// same number of states at the same times: the frame count is the max
/// state count across actors, and a frame's timestamp is read from whichever
/// actor has a state at that index first. Every current example scenario
/// advances all actors in lockstep over the same horizon (verified via
/// `tests/artifact_validation_test.rs`'s per-actor vertex-count check on the
/// `.xosc` path), so in practice every actor contributes to every frame.
fn build_objects_and_frames(
    scenario: &Scenario,
) -> (
    BTreeMap<String, OpenLabelObject>,
    BTreeMap<String, OpenLabelFrame>,
) {
    let mut objects = BTreeMap::new();
    for (idx, actor) in scenario.actors.iter().enumerate() {
        objects.insert(
            idx.to_string(),
            OpenLabelObject {
                name: actor.id.clone(),
                object_type: if actor.role == ActorRole::Pedestrian {
                    "pedestrian"
                } else {
                    "vehicle"
                },
            },
        );
    }

    let max_frames = scenario
        .actors
        .iter()
        .map(|a| a.states.len())
        .max()
        .unwrap_or(0);

    let mut frames = BTreeMap::new();
    for frame_idx in 0..max_frames {
        let mut frame_objects = BTreeMap::new();
        let mut timestamp: Option<f64> = None;
        for (idx, actor) in scenario.actors.iter().enumerate() {
            let Some(state) = actor.states.get(frame_idx) else {
                continue;
            };
            timestamp.get_or_insert(state.time);
            frame_objects.insert(
                idx.to_string(),
                OpenLabelFrameObjectEntry {
                    object_data: OpenLabelObjectData {
                        vec: vec![OpenLabelVec {
                            name: "position",
                            val: vec![state.cartesian.position.x, state.cartesian.position.y],
                        }],
                    },
                },
            );
        }
        frames.insert(
            frame_idx.to_string(),
            OpenLabelFrame {
                frame_properties: OpenLabelFrameProperties {
                    timestamp: format!("{:.6}", timestamp.unwrap_or(0.0)),
                },
                objects: frame_objects,
            },
        );
    }

    (objects, frames)
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
            min_ttc: Some(3.0),
            min_distance: Some(10.0),
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

    /// `to_higher_lane`: whether the npc's lane index increases (`0 -> 1`,
    /// which E5 fixed to be a *right* change, matching `cartesian.rs`) or
    /// decreases (`1 -> 0`, a *left* change).
    fn make_scenario_with_lane_change(to_higher_lane: bool) -> Scenario {
        let road = RoadSpec {
            num_lanes: 2,
            lane_width: 3.5,
            lane_directions: vec![1, 1],
            road_length: None,
        };
        let mut scenario = Scenario::new("cut_in_left".to_string(), 0.1, 5.0, road);
        scenario.validation = ValidationInfo {
            min_ttc: Some(3.0),
            min_distance: Some(10.0),
            all_constraints_satisfied: true,
            safety_violations: vec![],
            max_acceleration: 2.0,
            max_deceleration: -3.0,
            acceleration_violations: vec![],
        };

        let mut npc = ActorTrajectory::new("npc".to_string(), "npc".to_string());
        let (start_lane, end_lane) = if to_higher_lane { (0, 1) } else { (1, 0) };
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
        assert!(
            parsed["openlabel"]["metadata"]["scenario_weaver"]["scenario_id"]
                .as_str()
                .unwrap()
                .starts_with("SW-")
        );
    }

    #[test]
    fn test_creator_has_no_fabricated_year() {
        // M14: `creator` used to be a hardcoded literal with a baked-in
        // calendar year. It must now be derived from the crate version instead.
        let scenario = make_scenario(true);
        let result = export_to_openlabel(&scenario).unwrap();
        let parsed: serde_json::Value = serde_json::from_str(&result).unwrap();
        let creator = parsed["openlabel"]["metadata"]["scenario_weaver"]["creator"]
            .as_str()
            .unwrap();
        assert_eq!(
            creator,
            format!("ScenarioWeaver-{}", env!("CARGO_PKG_VERSION"))
        );
    }

    #[test]
    fn test_no_duplicated_name_description_fields() {
        // M14: `Name`/`Description` used to duplicate `ScenarioId`/`comment`.
        let scenario = make_scenario(true);
        let result = export_to_openlabel(&scenario).unwrap();
        let parsed: serde_json::Value = serde_json::from_str(&result).unwrap();
        let metadata = parsed["openlabel"]["metadata"].as_object().unwrap();
        assert!(!metadata.contains_key("Name"));
        assert!(!metadata.contains_key("Description"));
        assert!(!metadata.contains_key("Image"));
        assert!(!metadata.contains_key("ScenarioDatabase"));
        // No non-schema PascalCase keys sitting directly under metadata any
        // more; everything project-specific is namespaced under
        // `scenario_weaver`.
        for key in metadata.keys() {
            assert!(
                key.chars().next().is_none_or(|c| !c.is_uppercase()),
                "metadata key '{key}' is not snake_case"
            );
        }
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
    fn test_road_type_heuristic_removed() {
        // SW-17 M14: the RoadType* lane-count heuristic shipped a guess under
        // the ontology URI. It is gone; no RoadType* tag is emitted at all,
        // for any lane configuration, until a real `road_class` field exists.
        let road = RoadSpec {
            num_lanes: 4,
            lane_width: 3.5,
            lane_directions: vec![1, 1, 1, 1],
            road_length: None,
        };
        let mut scenario = Scenario::new("cut_in_left".to_string(), 0.1, 5.0, road);
        scenario.validation = ValidationInfo {
            min_ttc: Some(3.0),
            min_distance: Some(10.0),
            all_constraints_satisfied: true,
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

        let result = export_to_openlabel(&scenario).unwrap();
        let parsed: serde_json::Value = serde_json::from_str(&result).unwrap();
        let tags = parsed["openlabel"]["tags"].as_object().unwrap();
        let types: Vec<&str> = tags.values().map(|t| t["type"].as_str().unwrap()).collect();
        assert!(!types.contains(&"RoadTypeMotorway"));
        assert!(!types.contains(&"RoadTypeDistributor"));
        assert!(!types.contains(&"RoadTypeMinor"));
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
            min_ttc: Some(3.0),
            min_distance: Some(10.0),
            all_constraints_satisfied: true,
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
        // Mixed [1,1,-1,-1] → SW-17 M14: neither TravelDirectionRight nor
        // TravelDirectionLeft, since a bidirectional road has no single
        // travel direction to report. Emitting both (the pre-fix behaviour)
        // conveyed nothing.
        let road = RoadSpec {
            num_lanes: 4,
            lane_width: 3.5,
            lane_directions: vec![1, 1, -1, -1],
            road_length: None,
        };
        let mut scenario = Scenario::new("cut_in_left".to_string(), 0.1, 5.0, road);
        scenario.validation = ValidationInfo {
            min_ttc: Some(3.0),
            min_distance: Some(10.0),
            all_constraints_satisfied: true,
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

        let result = export_to_openlabel(&scenario).unwrap();
        let parsed: serde_json::Value = serde_json::from_str(&result).unwrap();
        let tags = parsed["openlabel"]["tags"].as_object().unwrap();
        let types: Vec<&str> = tags.values().map(|t| t["type"].as_str().unwrap()).collect();
        assert!(!types.contains(&"TravelDirectionRight"));
        assert!(!types.contains(&"TravelDirectionLeft"));
        assert!(!types.contains(&"LaneSpecificationTravelDirection"));
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
            min_ttc: Some(3.0),
            min_distance: Some(10.0),
            all_constraints_satisfied: true,
            safety_violations: vec![],
            max_acceleration: 2.0,
            max_deceleration: -3.0,
            acceleration_violations: vec![],
        };

        // Actor with ax > 0.5 (accelerating)
        let mut ego = ActorTrajectory::new("ego".to_string(), "ego".to_string());
        ego.add_state(State::new(
            0.0,
            Position::new(50.0, 5.25),
            Velocity::new(15.0, 0.0),
            Acceleration::new(1.5, 0.0),
            1,
        ));
        scenario.add_actor(ego);

        // Actor with ax < -0.5 (decelerating)
        let mut npc = ActorTrajectory::new("npc".to_string(), "npc".to_string());
        npc.add_state(State::new(
            0.0,
            Position::new(70.0, 1.75),
            Velocity::new(18.0, 0.0),
            Acceleration::new(-2.0, 0.0),
            0,
        ));
        scenario.add_actor(npc);

        let result = export_to_openlabel(&scenario).unwrap();
        let parsed: serde_json::Value = serde_json::from_str(&result).unwrap();
        let tags = parsed["openlabel"]["tags"].as_object().unwrap();
        let types: Vec<&str> = tags.values().map(|t| t["type"].as_str().unwrap()).collect();
        assert!(
            types.contains(&"MotionAccelerate"),
            "expected MotionAccelerate for ax=1.5"
        );
        assert!(
            types.contains(&"MotionDecelerate"),
            "expected MotionDecelerate for ax=-2.0"
        );
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
            min_ttc: Some(3.0),
            min_distance: Some(10.0),
            all_constraints_satisfied: true,
            safety_violations: vec![],
            max_acceleration: 2.0,
            max_deceleration: -3.0,
            acceleration_violations: vec![],
        };

        let mut ped = ActorTrajectory::new("ped".to_string(), "pedestrian".to_string());
        ped.add_state(State::new(
            0.0,
            Position::new(30.0, 0.0),
            Velocity::new(1.0, 0.0),
            Acceleration::new(0.0, 0.0),
            0,
        ));
        scenario.add_actor(ped);

        let result = export_to_openlabel(&scenario).unwrap();
        let parsed: serde_json::Value = serde_json::from_str(&result).unwrap();
        let tags = parsed["openlabel"]["tags"].as_object().unwrap();
        let types: Vec<&str> = tags.values().map(|t| t["type"].as_str().unwrap()).collect();
        assert!(
            !types.contains(&"VehicleCar"),
            "VehicleCar must not appear in pedestrian-only scenario"
        );
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
    fn test_lane_change_to_higher_index_tags_right() {
        // SW-17 E5: increasing lane index matches `cartesian.rs`'s
        // `LaneChangeDirection::Right` (a forward actor's `Right => lane+1`),
        // so it must produce `MotionLaneChangeRight`, not Left.
        let scenario = make_scenario_with_lane_change(true);
        let result = export_to_openlabel(&scenario).unwrap();
        let parsed: serde_json::Value = serde_json::from_str(&result).unwrap();

        let tags = parsed["openlabel"]["tags"].as_object().unwrap();
        let types: Vec<&str> = tags.values().map(|t| t["type"].as_str().unwrap()).collect();
        assert!(types.contains(&"MotionLaneChangeRight"));
        assert!(!types.contains(&"MotionLaneChangeLeft"));
    }

    #[test]
    fn test_lane_change_to_lower_index_tags_left() {
        let scenario = make_scenario_with_lane_change(false);
        let result = export_to_openlabel(&scenario).unwrap();
        let parsed: serde_json::Value = serde_json::from_str(&result).unwrap();

        let tags = parsed["openlabel"]["tags"].as_object().unwrap();
        let types: Vec<&str> = tags.values().map(|t| t["type"].as_str().unwrap()).collect();
        assert!(types.contains(&"MotionLaneChangeLeft"));
        assert!(!types.contains(&"MotionLaneChangeRight"));
    }

    /// SW-17 E5 headline check: `examples/cut_in_left.yaml` declares
    /// `direction: right` for the npc's only lane change (lane 0 -> lane 1,
    /// per its own comment). The exported `.ol.json` for that scenario must
    /// tag `MotionLaneChangeRight`, not `MotionLaneChangeLeft`.
    #[test]
    fn test_cut_in_left_yaml_direction_right_tags_right() {
        let scenario = make_scenario_with_lane_change(true);
        let result = export_to_openlabel(&scenario).unwrap();
        let parsed: serde_json::Value = serde_json::from_str(&result).unwrap();
        let tags = parsed["openlabel"]["tags"].as_object().unwrap();
        let types: Vec<&str> = tags.values().map(|t| t["type"].as_str().unwrap()).collect();
        assert!(
            types.contains(&"MotionLaneChangeRight"),
            "direction: right (lane 0 -> 1) must tag MotionLaneChangeRight, not Left"
        );
    }

    #[test]
    fn test_empty_lane_directions() {
        // lane_directions: vec![] → no RoadType*, no TravelDirectionRight/Left, no panic
        let road = RoadSpec {
            num_lanes: 0,
            lane_width: 3.5,
            lane_directions: vec![],
            road_length: None,
        };
        let mut scenario = Scenario::new("cut_in_left".to_string(), 0.1, 5.0, road);
        scenario.validation = ValidationInfo {
            min_ttc: Some(3.0),
            min_distance: Some(10.0),
            all_constraints_satisfied: true,
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
            0,
        ));
        scenario.add_actor(ego);

        let result = export_to_openlabel(&scenario).unwrap();
        let parsed: serde_json::Value = serde_json::from_str(&result).unwrap();
        let tags = parsed["openlabel"]["tags"].as_object().unwrap();
        let types: Vec<&str> = tags.values().map(|t| t["type"].as_str().unwrap()).collect();
        assert!(
            !types.contains(&"RoadTypeMinor")
                && !types.contains(&"RoadTypeMotorway")
                && !types.contains(&"RoadTypeDistributor"),
            "no RoadType* tag must appear (heuristic removed, SW-17 M14)"
        );
        assert!(
            !types.contains(&"TravelDirectionRight"),
            "TravelDirectionRight must not appear for empty lane_directions"
        );
        assert!(
            !types.contains(&"TravelDirectionLeft"),
            "TravelDirectionLeft must not appear for empty lane_directions"
        );
        assert!(
            !types.contains(&"LaneSpecificationTravelDirection"),
            "LaneSpecificationTravelDirection must not appear for empty lane_directions"
        );
    }

    #[test]
    fn test_pedestrian_scenario_motion_tag() {
        let road = RoadSpec {
            num_lanes: 2,
            lane_width: 3.5,
            lane_directions: vec![1, 1],
            road_length: None,
        };
        let mut scenario = Scenario::new("pedestrian_crossing".to_string(), 0.1, 5.0, road);
        scenario.validation = ValidationInfo {
            min_ttc: Some(3.0),
            min_distance: Some(10.0),
            all_constraints_satisfied: true,
            safety_violations: vec![],
            max_acceleration: 2.0,
            max_deceleration: -3.0,
            acceleration_violations: vec![],
        };
        let mut ped = ActorTrajectory::new("ped".to_string(), "pedestrian".to_string());
        ped.add_state(State::new(
            0.0,
            Position::new(30.0, 0.0),
            Velocity::new(1.0, 0.0),
            Acceleration::new(0.0, 0.0),
            0,
        ));
        scenario.add_actor(ped);

        let result = export_to_openlabel(&scenario).unwrap();
        let parsed: serde_json::Value = serde_json::from_str(&result).unwrap();
        let tags = parsed["openlabel"]["tags"].as_object().unwrap();
        let types: Vec<&str> = tags.values().map(|t| t["type"].as_str().unwrap()).collect();
        assert!(
            types.contains(&"MotionWalk"),
            "expected MotionWalk for pedestrian_crossing scenario"
        );
        assert!(
            !types.contains(&"MotionDrive"),
            "MotionDrive must not appear for pedestrian scenario"
        );
    }

    #[test]
    fn test_tag_keys_are_type_strings() {
        // Tag map keys must be the tag type string, not positional integers
        let scenario = make_scenario(true);
        let result = export_to_openlabel(&scenario).unwrap();
        let parsed: serde_json::Value = serde_json::from_str(&result).unwrap();
        let tags = parsed["openlabel"]["tags"].as_object().unwrap();
        // Keys should be type strings like "MotionCutIn", not "0", "1", "2"
        assert!(
            tags.contains_key("MotionCutIn"),
            "expected key 'MotionCutIn' in tags map"
        );
        assert!(
            tags.contains_key("LaneSpecificationLaneCount"),
            "expected key 'LaneSpecificationLaneCount' in tags map"
        );
        assert!(
            !tags.contains_key("0"),
            "positional integer keys must not appear"
        );
    }

    #[test]
    fn test_objects_present_and_named_after_actor_ids() {
        // SW-17 E4 (D4, Route A): one object per actor, named after the
        // actor's `id` — the same string the `.xosc` exporter uses as its
        // `<ScenarioObject name="...">` (see xosc_exporter.rs's
        // `add_vehicle`/`add_pedestrian` calls, both keyed on `actor.id`).
        let scenario = make_scenario(true); // "ego" + "npc"
        let result = export_to_openlabel(&scenario).unwrap();
        let parsed: serde_json::Value = serde_json::from_str(&result).unwrap();
        let objects = parsed["openlabel"]["objects"].as_object().unwrap();
        assert_eq!(objects.len(), 2);
        let names: std::collections::BTreeSet<&str> = objects
            .values()
            .map(|o| o["name"].as_str().unwrap())
            .collect();
        assert_eq!(
            names,
            std::collections::BTreeSet::from(["ego", "npc"]),
            "object names must match actor ids exactly"
        );
    }

    #[test]
    fn test_frames_carry_per_timestep_positions() {
        let scenario = make_scenario_with_lane_change(true); // 2 states, "npc"
        let result = export_to_openlabel(&scenario).unwrap();
        let parsed: serde_json::Value = serde_json::from_str(&result).unwrap();
        let frames = parsed["openlabel"]["frames"].as_object().unwrap();
        assert_eq!(frames.len(), 2, "expected one frame per timestep");

        let frame0 = &frames["0"];
        assert_eq!(frame0["frame_properties"]["timestamp"], "0.000000");
        let obj0 = frame0["objects"].as_object().unwrap();
        assert_eq!(obj0.len(), 1);
        let (_, entry) = obj0.iter().next().unwrap();
        let pos = entry["object_data"]["vec"][0]["val"].as_array().unwrap();
        assert_eq!(pos[0].as_f64().unwrap(), 70.0);
        assert_eq!(pos[1].as_f64().unwrap(), 1.75);

        let frame1 = &frames["1"];
        assert_eq!(frame1["frame_properties"]["timestamp"], "0.100000");
        let obj1 = frame1["objects"].as_object().unwrap();
        let (_, entry1) = obj1.iter().next().unwrap();
        let pos1 = entry1["object_data"]["vec"][0]["val"].as_array().unwrap();
        assert_eq!(pos1[0].as_f64().unwrap(), 71.8);
        assert_eq!(pos1[1].as_f64().unwrap(), 3.5);
    }
}
