//! OpenDRIVE (.xodr) road network export
//!
//! Converts the internal `RoadSpec` into an OpenDRIVE 1.7 road network file.
//! The output describes a single straight road with lanes matching the
//! scenario specification.  Simulators like CARLA can load this alongside
//! the companion .xosc file.

use crate::error::{Result, ScenarioGenError};
use crate::scenario::model::Scenario;
use opendrive::{
    core::{additional_data::AdditionalData, header::Header, OpenDrive},
    lane::{
        center::Center, center_lane::CenterLane, lane_choice::LaneChoice,
        lane_section::LaneSection, lane_type::LaneType, lanes::Lanes, left::Left,
        left_lane::LeftLane, right::Right, right_lane::RightLane, width::Width, Lane,
    },
    road::{
        geometry::{geometry_type::GeometryType, line::Line, plan_view::PlanView, Geometry},
        Road,
    },
};
use uom::si::{angle::radian, f64::Angle, f64::Length, length::meter};
use vec1::{vec1, Size0Error, Vec1};

/// Export a scenario to OpenDRIVE XML format
///
/// Generates a single straight road whose lane count, width, and directions
/// match the scenario's `RoadSpec`.  The road length comes from `RoadSpec`
/// when set, or is estimated from the actor trajectories.
///
/// # Errors
/// Returns an error if XML serialization fails.
pub fn export_to_xodr(scenario: &Scenario) -> Result<String> {
    let road_length = compute_road_length(scenario);

    let header = Header {
        rev_major: 1,
        rev_minor: 7,
        name: Some(scenario.scenario_type.clone()),
        version: Some("1.0".to_string()),
        date: Some(chrono::Utc::now().format("%Y-%m-%dT%H:%M:%S").to_string()),
        vendor: Some("CARLA Scenario Generator".to_string()),
        ..Default::default()
    };

    // Place the reference line at y = n_forward * lane_width so that forward
    // (right) lanes project downward into y > 0 space, matching the scenario's
    // lane coordinate formula: py = lane * lane_width + lane_width/2.
    let n_forward = scenario
        .road
        .lane_directions
        .iter()
        .filter(|&&d| d == 1)
        .count();
    let reference_y = n_forward as f64 * scenario.road.lane_width;

    let geometry = Geometry {
        s: Length::new::<meter>(0.0),
        x: Length::new::<meter>(0.0),
        y: Length::new::<meter>(reference_y),
        hdg: Angle::new::<radian>(0.0),
        length: Length::new::<meter>(road_length),
        r#type: GeometryType::Line(Line {}),
        additional_data: AdditionalData::default(),
    };

    let plan_view = PlanView {
        geometry: vec1![geometry],
        additional_data: AdditionalData::default(),
    };

    let lane_section = build_lane_section(scenario)?;

    let lanes = Lanes {
        lane_offset: vec![],
        lane_section: vec1![lane_section],
        additional_data: AdditionalData::default(),
    };

    let road = Road {
        id: "0".to_string(),
        junction: "-1".to_string(),
        length: Length::new::<meter>(road_length),
        name: Some(scenario.scenario_type.clone()),
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
    };

    let mut opendrive = OpenDrive::default();
    opendrive.header = header;
    opendrive.road.push(road);

    opendrive.to_xml_string().map_err(|e| {
        ScenarioGenError::ExtractionFailed(format!("OpenDRIVE serialization failed: {e}"))
    })
}

/// Determine the road length from the spec or estimate from actor trajectories.
fn compute_road_length(scenario: &Scenario) -> f64 {
    if let Some(len) = scenario.road.road_length {
        return len;
    }

    // Derive from max longitudinal position observed across all actors, plus a
    // 20% buffer so vehicles always remain on the road.
    let max_x = scenario
        .actors
        .iter()
        .flat_map(|a| a.states.iter())
        .filter_map(|s| s.cartesian.as_ref().map(|c| c.position.x))
        .fold(0.0_f64, f64::max);

    if max_x > 0.0 {
        max_x * 1.2
    } else {
        // Last-resort fallback: duration × 30 m/s with buffer
        scenario.duration * 30.0 * 1.2
    }
}

/// Build the single `LaneSection` from the scenario's `RoadSpec`.
///
/// Forward lanes (`direction == 1`) become right-side lanes (IDs -1, -2, …).
/// Backward lanes (`direction == -1`) become left-side lanes (IDs +1, +2, …).
fn build_lane_section(scenario: &Scenario) -> Result<LaneSection> {
    let road = &scenario.road;

    let center = Center {
        lane: vec1![CenterLane {
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
        }],
        additional_data: AdditionalData::default(),
    };

    let mut right_lanes: Vec<RightLane> = Vec::new(); // forward (+1) → negative IDs
    let mut left_lanes: Vec<LeftLane> = Vec::new(); // backward (−1) → positive IDs

    // Count forward lanes so IDs can be assigned outermost-first.
    // OpenDRIVE ID -1 is innermost (closest to center/reference line),
    // -n is outermost.  The scenario's lane 0 is at the lowest y (outermost
    // right), so it gets the most negative ID.
    let n_forward = road.lane_directions.iter().filter(|&&d| d == 1).count() as i64;
    let mut right_id: i64 = -n_forward; // start outermost, count toward -1
    let mut left_id: i64 = 1;

    for &direction in &road.lane_directions {
        let base = driving_lane(road.lane_width);
        if direction == 1 {
            right_lanes.push(RightLane { id: right_id, base });
            right_id += 1; // move inward toward -1
        } else {
            left_lanes.push(LeftLane { id: left_id, base });
            left_id += 1;
        }
    }

    let left = if left_lanes.is_empty() {
        None
    } else {
        Some(Left {
            lane: Vec1::try_from_vec(left_lanes).map_err(|_: Size0Error| {
                ScenarioGenError::ExtractionFailed("left lane list was empty".to_string())
            })?,
            additional_data: AdditionalData::default(),
        })
    };

    let right = if right_lanes.is_empty() {
        None
    } else {
        Some(Right {
            lane: Vec1::try_from_vec(right_lanes).map_err(|_: Size0Error| {
                ScenarioGenError::ExtractionFailed("right lane list was empty".to_string())
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

/// Build a driving lane with constant width (polynomial a=width, b=c=d=0).
fn driving_lane(lane_width: f64) -> Lane {
    Lane {
        link: None,
        choice: vec![LaneChoice::Width(Width {
            a: lane_width,
            b: 0.0,
            c: 0.0,
            d: 0.0,
            s_offset: Length::new::<meter>(0.0),
        })],
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dsl::types::RoadSpec;
    use crate::scenario::model::Scenario;

    fn make_scenario(directions: Vec<i32>, road_length: Option<f64>) -> Scenario {
        Scenario::new(
            "test".to_string(),
            0.1,
            10.0,
            RoadSpec {
                num_lanes: directions.len(),
                lane_width: 3.5,
                lane_directions: directions,
                road_length,
            },
        )
    }

    #[test]
    fn test_xodr_all_forward() {
        let scenario = make_scenario(vec![1, 1], Some(200.0));
        let xml = export_to_xodr(&scenario).expect("export succeeded");
        assert!(xml.contains("<OpenDRIVE"));
        assert!(xml.contains("driving"));
    }

    #[test]
    fn test_xodr_bidirectional() {
        let scenario = make_scenario(vec![1, 1, -1, -1], Some(400.0));
        let xml = export_to_xodr(&scenario).expect("export succeeded");
        assert!(xml.contains("<OpenDRIVE"));
        assert!(xml.contains("driving"));
    }

    #[test]
    fn test_xodr_road_length_fallback() {
        let scenario = make_scenario(vec![1, 1], None);
        let xml = export_to_xodr(&scenario).expect("export succeeded");
        assert!(xml.contains("<OpenDRIVE"));
    }
}
