//! OpenDRIVE (.xodr) road network export
//!
//! Converts the internal `RoadSpec` into an OpenDRIVE 1.7 road network file.
//! The output describes a single straight road with lanes matching the
//! scenario specification.  Simulators like CARLA can load this alongside
//! the companion .xosc file.

use crate::error::{Result, ScenarioGenError};
use crate::scenario::model::Scenario;
use crate::solver::encoder::SIDEWALK_WIDTH;
use opendrive::{
    core::{additional_data::AdditionalData, header::Header, OpenDrive},
    lane::{
        center::Center,
        center_lane::CenterLane,
        lane_choice::LaneChoice,
        lane_section::LaneSection,
        lane_type::LaneType,
        lanes::Lanes,
        left::Left,
        left_lane::LeftLane,
        right::Right,
        right_lane::RightLane,
        road_mark::{color::Color, type_simplified::TypeSimplified, RoadMark},
        width::Width,
        Lane,
    },
    road::{
        geometry::{geometry_type::GeometryType, line::Line, plan_view::PlanView, Geometry},
        road_type::RoadType,
        road_type_e::RoadTypeE,
        speed::{MaxSpeed, Speed},
        unit::SpeedUnit,
        Road,
    },
};
use uom::si::{angle::radian, f64::Angle, f64::Length, length::meter};
use vec1::{vec1, Size0Error, Vec1};

/// Export a scenario to OpenDRIVE XML format
///
/// Generates a single straight road whose lane count, width, and directions
/// match the scenario's `RoadSpec`.  The road's longitudinal extent covers at
/// least `RoadSpec::road_length` (when set) and always covers every actor's
/// observed `x` position, including negative `x` from backward-direction
/// actors (M12).
///
/// # Errors
/// Returns an error if XML serialization fails.
pub fn export_to_xodr(scenario: &Scenario) -> Result<String> {
    let (start_x, road_length) = compute_road_geometry(scenario);

    let header = Header {
        rev_major: 1,
        rev_minor: 7,
        name: Some(scenario.scenario_type.clone()),
        version: Some("1.0".to_string()),
        date: Some(chrono::Utc::now().format("%Y-%m-%dT%H:%M:%S").to_string()),
        vendor: Some("ScenarioWeaver".to_string()),
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
        x: Length::new::<meter>(start_x),
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
        r#type: vec![road_type(scenario)],
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

/// Determine the road's start `x` and `length` so that it covers at least the
/// spec's `road_length` (when set) and always covers every actor's observed
/// `x` extent, with a padding margin on both ends.
///
/// Two defects this fixes (M12):
/// - Previously `road_length` short-circuited the trajectory-derived estimate
///   entirely, so a spec with an explicit (or `parser.rs`-defaulted) length
///   shorter than the trajectories it carried silently shipped that way —
///   `overtake_with_opposite` reached `x=267.5` on a `length="3.0e2"` — no
///   defect there, but `cut_in_left_optimize_max_ttc` reached `x=303.26` on
///   the same 300 m default. Now the final length is
///   `max(spec_length, trajectory_extent * margin)`, so it can only grow.
/// - Previously the geometry always started at `x=0`, so a backward-direction
///   actor reaching `x<0` landed at a negative `s`, outside the road's
///   `[0, length]` range. Now the geometry's start `x` (and therefore `s=0`)
///   is pulled back to cover the most negative observed `x`, with padding.
fn compute_road_geometry(scenario: &Scenario) -> (f64, f64) {
    let spec_length = scenario
        .road
        .road_length
        .unwrap_or(scenario.duration * 30.0);

    let mut bounds: Option<(f64, f64)> = None;
    for actor in &scenario.actors {
        for state in &actor.states {
            let x = state.cartesian.position.x;
            bounds = Some(match bounds {
                None => (x, x),
                Some((min_x, max_x)) => (min_x.min(x), max_x.max(x)),
            });
        }
    }

    let Some((min_x, max_x)) = bounds else {
        // No trajectory data at all: fall back to the spec/default length
        // starting at the origin, same as before.
        return (0.0, spec_length);
    };

    let span = (max_x - min_x).max(0.0);
    let margin = (span * 0.2).max(1.0);
    // Never push the start past the origin for an all-forward, all-positive-x
    // scenario — this keeps existing (non-negative) examples' geometry
    // starting at x=0, matching prior output.
    let start_x = (min_x - margin).min(0.0);
    let end_x = max_x + margin;
    let trajectory_length = end_x - start_x;

    (start_x, spec_length.max(trajectory_length))
}

/// Determine the (right, left) sidewalk widths needed to cover every actor's
/// observed lateral excursion off the drivable surface, floored at
/// `SIDEWALK_WIDTH`.
///
/// The right sidewalk covers `py < 0`; the left covers
/// `py > num_lanes * lane_width`. `OnSidewalk`'s bound in
/// `src/solver/encoder.rs` only pins the single instant an `eventually`
/// proposition is satisfied — nothing stops the pedestrian from drifting
/// further before or after that instant — so the exporter cannot assume the
/// excursion stays within `SIDEWALK_WIDTH` and must measure it directly, the
/// same way `compute_road_geometry` measures the longitudinal extent (M12).
fn sidewalk_widths(scenario: &Scenario) -> (f64, f64) {
    const MARGIN: f64 = 0.5;

    let road_width = scenario.road.num_lanes as f64 * scenario.road.lane_width;
    let mut max_right_excess = 0.0_f64; // py < 0, magnitude of overshoot
    let mut max_left_excess = 0.0_f64; // py > road_width, magnitude of overshoot

    for actor in &scenario.actors {
        for state in &actor.states {
            let y = state.cartesian.position.y;
            if y < 0.0 {
                max_right_excess = max_right_excess.max(-y);
            } else if y > road_width {
                max_left_excess = max_left_excess.max(y - road_width);
            }
        }
    }

    (
        (max_right_excess + MARGIN).max(SIDEWALK_WIDTH),
        (max_left_excess + MARGIN).max(SIDEWALK_WIDTH),
    )
}

/// Build the `<type>` element carrying the road's speed limit.
///
/// The limit is derived from the fastest longitudinal speed any actor
/// actually reaches, rounded up to the next 5 m/s (minimum 30 m/s), so the
/// exported limit is never lower than what the trajectories require.
fn road_type(scenario: &Scenario) -> RoadType {
    let max_speed = scenario
        .actors
        .iter()
        .flat_map(|a| a.states.iter())
        .map(|s| s.cartesian.velocity.vx.abs())
        .fold(0.0_f64, f64::max);

    let limit = ((max_speed / 5.0).ceil() * 5.0).max(30.0);

    RoadType {
        speed: Some(Speed {
            max: MaxSpeed::Limit(limit),
            unit: Some(SpeedUnit::MetersPerSecond),
        }),
        country: None,
        s: Length::new::<meter>(0.0),
        r#type: RoadTypeE::Rural,
        additional_data: AdditionalData::default(),
    }
}

/// A broken white `<roadMark>` for a driving/sidewalk lane's outer edge.
fn edge_road_mark() -> RoadMark {
    RoadMark {
        sway: vec![],
        r#type: None,
        explicit: None,
        color: Color::White,
        height: None,
        lane_change: None,
        material: None,
        s_offset: Length::new::<meter>(0.0),
        type_simplified: TypeSimplified::Broken,
        weight: None,
        width: Some(Length::new::<meter>(0.12)),
        additional_data: AdditionalData::default(),
    }
}

/// A solid yellow `<roadMark>` for the centre lane (separates opposing traffic).
fn center_road_mark() -> RoadMark {
    RoadMark {
        sway: vec![],
        r#type: None,
        explicit: None,
        color: Color::Yellow,
        height: None,
        lane_change: None,
        material: None,
        s_offset: Length::new::<meter>(0.0),
        type_simplified: TypeSimplified::Solid,
        weight: None,
        width: Some(Length::new::<meter>(0.12)),
        additional_data: AdditionalData::default(),
    }
}

/// Build the single `LaneSection` from the scenario's `RoadSpec`.
///
/// Forward lanes (`direction == 1`) become right-side lanes (IDs -1, -2, …).
/// Backward lanes (`direction == -1`) become left-side lanes (IDs +1, +2, …).
/// `RoadSpec::validate` guarantees `lane_directions` is a single
/// forward-then-backward block (SW-16 E2), which is what makes this
/// assignment agree with the scenario's `py = lane*lane_width +
/// lane_width/2` mapping.
///
/// A `LaneType::Sidewalk` strip is added just outside the driving lanes on
/// both sides — the region `OnSidewalk` bounds its half-plane to in
/// `src/solver/encoder.rs` (SW-16 E3) — so a pedestrian standing there is on
/// something the `.xodr` actually describes. Its width is
/// `max(SIDEWALK_WIDTH, observed excursion + margin)` (see
/// [`sidewalk_widths`]): `SIDEWALK_WIDTH` bounds where `OnSidewalk` can
/// become true, but that is a pointwise ("eventually") proposition, not an
/// "always" one, so nothing stops a pedestrian from drifting further after
/// satisfying it — the exporter widens the drawn sidewalk to cover whatever
/// the trajectory actually does, the same trajectory-driven pattern M12
/// uses for road length.
fn build_lane_section(scenario: &Scenario) -> Result<LaneSection> {
    let road = &scenario.road;
    let (right_sidewalk_width, left_sidewalk_width) = sidewalk_widths(scenario);

    let center = Center {
        lane: vec1![CenterLane {
            id: 0,
            base: Lane {
                link: None,
                choice: vec![],
                road_mark: vec![center_road_mark()],
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

    // Right-side sidewalk: covers `py < 0`, i.e. one lane-width further out
    // than the outermost right (driving) lane, or -1 if there are no
    // driving lanes on the right at all.
    let right_sidewalk_id = right_lanes.first().map_or(-1, |l| l.id - 1);
    right_lanes.insert(
        0,
        RightLane {
            id: right_sidewalk_id,
            base: sidewalk_lane(right_sidewalk_width),
        },
    );

    // Left-side sidewalk: covers `py > road_width`, i.e. one lane-width
    // further out than the outermost left (driving) lane, or 1 if there are
    // no driving lanes on the left at all.
    let left_sidewalk_id = left_lanes.last().map_or(1, |l| l.id + 1);
    left_lanes.push(LeftLane {
        id: left_sidewalk_id,
        base: sidewalk_lane(left_sidewalk_width),
    });

    let left = Some(Left {
        lane: Vec1::try_from_vec(left_lanes).map_err(|_: Size0Error| {
            ScenarioGenError::ExtractionFailed("left lane list was empty".to_string())
        })?,
        additional_data: AdditionalData::default(),
    });

    let right = Some(Right {
        lane: Vec1::try_from_vec(right_lanes).map_err(|_: Size0Error| {
            ScenarioGenError::ExtractionFailed("right lane list was empty".to_string())
        })?,
        additional_data: AdditionalData::default(),
    });

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
        road_mark: vec![edge_road_mark()],
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

/// Build a sidewalk lane of the given width (polynomial a=width, b=c=d=0).
fn sidewalk_lane(width: f64) -> Lane {
    Lane {
        link: None,
        choice: vec![LaneChoice::Width(Width {
            a: width,
            b: 0.0,
            c: 0.0,
            d: 0.0,
            s_offset: Length::new::<meter>(0.0),
        })],
        road_mark: vec![edge_road_mark()],
        material: vec![],
        speed: vec![],
        access: vec![],
        height: vec![],
        rule: vec![],
        level: Some(false),
        r#type: LaneType::Sidewalk,
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

    /// SW-16 M12: a backward-direction actor reaching `x < 0` must stay
    /// within the exported road's `s` range (`geometry.x` ..
    /// `geometry.x + length`), not land at a negative, out-of-range `s`.
    #[test]
    fn test_xodr_negative_x_actor_stays_within_road_s_range() {
        use crate::scenario::model::{Acceleration, ActorTrajectory, Position, State, Velocity};

        let mut scenario = make_scenario(vec![1, -1], Some(100.0));
        let mut backward = ActorTrajectory::new("oncoming".to_string(), "npc".to_string());
        // Travels from x=10 down to x=-50 (backward direction).
        backward.add_state(State::new(
            0.0,
            Position::new(10.0, 5.25),
            Velocity::new(-10.0, 0.0),
            Acceleration::new(0.0, 0.0),
            1,
        ));
        backward.add_state(State::new(
            6.0,
            Position::new(-50.0, 5.25),
            Velocity::new(-10.0, 0.0),
            Acceleration::new(0.0, 0.0),
            1,
        ));
        scenario.add_actor(backward);

        let (start_x, length) = compute_road_geometry(&scenario);
        let end_x = start_x + length;

        assert!(
            start_x <= -50.0,
            "road must start at or before the actor's most negative x=-50.0, got start_x={start_x}"
        );
        assert!(
            end_x >= 10.0,
            "road must end at or after the actor's furthest x=10.0, got end_x={end_x}"
        );

        // The exported geometry's x must match start_x exactly (s=0 there).
        let xml = export_to_xodr(&scenario).expect("export succeeded");
        let doc =
            opendrive::core::OpenDrive::from_xml_str(&xml).expect("xodr round-trips through parse");
        let geom_x = doc.road[0].plan_view.geometry[0]
            .x
            .get::<uom::si::length::meter>();
        assert!((geom_x - start_x).abs() < 1e-9);
    }

    /// SW-16 E3: the exported sidewalk widens to cover a pedestrian's actual
    /// lateral excursion, rather than assuming a fixed width is always
    /// enough (the encoder's `OnSidewalk` bound only pins one instant, not
    /// the whole trajectory — see `sidewalk_widths`'s doc comment).
    #[test]
    fn test_sidewalk_widths_grow_to_cover_actual_excursion() {
        use crate::scenario::model::{Acceleration, ActorTrajectory, Position, State, Velocity};

        let mut scenario = make_scenario(vec![1, 1], Some(100.0)); // road_width = 7.0
        let mut ped = ActorTrajectory::new("ped".to_string(), "pedestrian".to_string());
        // Drifts far onto the left sidewalk: road_width=7.0, y reaches 20.0.
        ped.add_state(State::new(
            0.0,
            Position::new(50.0, 8.0),
            Velocity::new(0.0, 2.0),
            Acceleration::new(0.0, 0.0),
            0,
        ));
        ped.add_state(State::new(
            5.0,
            Position::new(50.0, 20.0),
            Velocity::new(0.0, 2.0),
            Acceleration::new(0.0, 0.0),
            0,
        ));
        scenario.add_actor(ped);

        let (right_width, left_width) = sidewalk_widths(&scenario);
        // Excess beyond road_width=7.0 is 20.0-7.0=13.0; must be covered plus margin.
        assert!(
            left_width >= 13.0,
            "left sidewalk width {left_width} must cover the 13.0m excursion"
        );
        // Right side never visited: stays at the SIDEWALK_WIDTH floor.
        assert!((right_width - SIDEWALK_WIDTH).abs() < 1e-9);
    }
}
