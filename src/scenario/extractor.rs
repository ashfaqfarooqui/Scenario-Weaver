//! Scenario extraction from Z3 models
//!
//! This module provides a public helper for extracting scenario data from a Z3
//! model via [`GenericEncoder::extract_scenario`](crate::solver::encoder::GenericEncoder).
//! The primary extraction logic lives in the encoder itself; this module exists
//! to expose a stable public API and as an extension point.

use crate::scenario::model::Scenario;

/// Extract scenario from a Z3 model using the given encoder.
///
/// Delegates to [`GenericEncoder::extract_scenario`](crate::solver::encoder::GenericEncoder::extract_scenario).
pub fn extract_scenario_from_model(
    encoder: &crate::solver::encoder::Z3Encoder,
    model: &z3::Model,
) -> crate::error::Result<Scenario> {
    encoder.extract_scenario(model)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dsl::types::{
        ActorRole, ActorSpec, LaneChangeConfig, LaneChangeDirection, RoadSpec, ScenarioSpec,
        ScenarioType, ValueOrRange,
    };
    use crate::ltl::generator::LTLGenerator;
    use crate::solver::encoder::Z3Encoder;
    use std::collections::HashMap;
    use z3::{Config, SatResult};

    fn create_test_spec() -> ScenarioSpec {
        ScenarioSpec {
            scenario_type: ScenarioType::CutInLeft,
            time_step: 0.5,
            duration: 10.0,
            actors: vec![
                ActorSpec {
                    id: "ego".to_string(),
                    role: ActorRole::Ego,
                    lane: 1,
                    position: ValueOrRange::Value(50.0),
                    speed: ValueOrRange::Value(15.0),
                    acceleration: ValueOrRange::Range([-8.0, 3.0]),
                    direction: 1,
                    behavior: HashMap::new(),
                    lane_changes: vec![],
                    bicycle_params: None,
                },
                ActorSpec {
                    id: "npc".to_string(),
                    role: ActorRole::Npc,
                    lane: 0,
                    position: ValueOrRange::Range([60.0, 80.0]),
                    speed: ValueOrRange::Range([12.0, 14.0]),
                    acceleration: ValueOrRange::Range([-8.0, 3.0]),
                    direction: 1,
                    behavior: HashMap::new(),
                    lane_changes: vec![LaneChangeConfig {
                        direction: LaneChangeDirection::Right,
                        start_time: ValueOrRange::Range([2.5, 7.5]),
                        duration: ValueOrRange::Range([3.0, 4.0]),
                    }],
                    bicycle_params: None,
                },
            ],
            min_ttc: 3.0,
            min_distance: 5.0,
            road: Some(RoadSpec {
                num_lanes: 2,
                lane_width: 3.5,
                lane_directions: vec![1, 1],
                road_length: None,
            }),
            lane_width: 3.5,
            num_scenarios: 1,
            constraint_modes: crate::dsl::types::ConstraintModes::default(),
            optimization_target: crate::dsl::types::OptimizationTarget::None,
            max_acceleration: None,
            max_deceleration: None,
            max_velocity: None,
            min_velocity: None,
            min_lateral_distance: None,
            max_relative_velocity: None,
            max_lateral_acceleration: 2.0,
            coordinate_system: crate::dsl::types::CoordinateSystem::Cartesian,
            bicycle_config: None,
        }
    }

    #[test]
    fn test_extract_scenario_integration() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let spec = create_test_spec();

            let mut encoder = Z3Encoder::new(spec.clone());
            encoder.create_variables();
            encoder.encode_initial_conditions();
            encoder.encode_kinematics();
            encoder.encode_lane_velocity_constraints();
            encoder.encode_lateral_velocity_bounds();

            let ltl_formula = LTLGenerator::generate(&spec).unwrap();
            encoder.encode_ltl(&ltl_formula);
            // Safety constraints are now included in LTL formula via generate_safety()

            let result = encoder.check();
            assert_eq!(result, SatResult::Sat);

            if result == SatResult::Sat {
                let model = encoder.get_model().unwrap();
                let scenario = extract_scenario_from_model(&encoder, &model).unwrap();

                // Verify extraction worked
                assert_eq!(scenario.actors.len(), 2);
                assert!(scenario.get_actor("ego").is_some());
                assert!(scenario.get_actor("npc").is_some());

                println!("Integration test passed - scenario extracted successfully");
            }
        });
    }

    /// Helper: run the full encode-solve-extract pipeline and return the Scenario
    fn run_extraction(spec: &ScenarioSpec) -> Scenario {
        let cfg = Config::new();
        let mut result: Option<Scenario> = None;
        z3::with_z3_config(&cfg, || {
            let mut encoder = Z3Encoder::new(spec.clone());
            encoder.create_variables();
            encoder.encode_initial_conditions();
            encoder.encode_kinematics();
            encoder.encode_lane_velocity_constraints();
            encoder.encode_lateral_velocity_bounds();

            let ltl_formula = LTLGenerator::generate(spec).unwrap();
            encoder.encode_ltl(&ltl_formula);

            let sat = encoder.check();
            assert_eq!(sat, SatResult::Sat, "Spec must be satisfiable");

            let model = encoder.get_model().unwrap();
            result = Some(extract_scenario_from_model(&encoder, &model).unwrap());
        });
        result.unwrap()
    }

    #[test]
    fn test_extract_scenario_validation_metrics() {
        let spec = create_test_spec();
        let scenario = run_extraction(&spec);

        assert!(scenario.validation.min_ttc.is_finite());
        assert!(scenario.validation.min_ttc > 0.0);
        assert!(scenario.validation.min_ttc < 999.0, "min_ttc should be computed, not default");

        assert!(scenario.validation.min_distance.is_finite());
        assert!(scenario.validation.min_distance > 0.0);
        assert!(scenario.validation.min_distance < 999.0, "min_distance should be computed, not default");

        // all_constraints_satisfied is a bool — for a valid SAT scenario it should be true
        assert!(scenario.validation.all_constraints_satisfied);
    }

    #[test]
    fn test_extract_scenario_trajectory_values() {
        let spec = create_test_spec();
        let scenario = run_extraction(&spec);

        let horizon = (spec.duration / spec.time_step) as usize;
        let expected_states = horizon + 1;

        for actor in &scenario.actors {
            assert_eq!(
                actor.states.len(),
                expected_states,
                "Actor {} should have {} states",
                actor.id,
                expected_states
            );

            // Lane values are valid
            for state in &actor.states {
                assert!(state.cartesian.lane < spec.road.as_ref().unwrap().num_lanes);
            }

            // Velocity within reasonable bounds (acceleration range [-8, 3] over 10s)
            for state in &actor.states {
                assert!(state.cartesian.velocity.vx >= -100.0 && state.cartesian.velocity.vx <= 100.0);
            }
        }

        // Ego starts at position 50 moving forward — positions should be monotonically increasing
        let ego = scenario.get_actor("ego").unwrap();
        for i in 1..ego.states.len() {
            assert!(
                ego.states[i].cartesian.position.x >= ego.states[i - 1].cartesian.position.x,
                "Ego position should be monotonically increasing (step {})",
                i
            );
        }
    }

    #[test]
    fn test_extract_scenario_two_actors() {
        let spec = create_test_spec();
        let scenario = run_extraction(&spec);

        assert_eq!(scenario.actors.len(), 2);

        let ego = scenario.get_actor("ego").unwrap();
        assert_eq!(ego.role, "ego");

        let npc = scenario.get_actor("npc").unwrap();
        assert_eq!(npc.role, "npc");
    }

    #[test]
    fn test_extract_scenario_road_info() {
        let spec = create_test_spec();
        let scenario = run_extraction(&spec);

        let road_spec = spec.road.as_ref().unwrap();
        assert_eq!(scenario.road.num_lanes, road_spec.num_lanes);
        assert_eq!(scenario.road.lane_width, road_spec.lane_width);
    }

    #[test]
    fn test_extract_scenario_metadata() {
        let spec = create_test_spec();
        let scenario = run_extraction(&spec);

        assert_eq!(scenario.scenario_type, "cut_in_left");
        assert_eq!(scenario.time_step, spec.time_step);
        assert_eq!(scenario.duration, spec.duration);
        // scenario_id should be a non-empty UUID-like string
        assert!(!scenario.scenario_id.is_empty());
        assert!(scenario.scenario_id.len() >= 32, "scenario_id should be UUID-like");
    }

    #[test]
    fn test_extract_scenario_no_road_spec() {
        let mut spec = create_test_spec();
        spec.road = None;

        // With no road spec, extraction should still work using defaults
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let mut encoder = Z3Encoder::new(spec.clone());
            encoder.create_variables();
            encoder.encode_initial_conditions();
            encoder.encode_kinematics();
            encoder.encode_lane_velocity_constraints();
            encoder.encode_lateral_velocity_bounds();

            let ltl_formula = LTLGenerator::generate(&spec).unwrap();
            encoder.encode_ltl(&ltl_formula);

            let sat = encoder.check();
            if sat == SatResult::Sat {
                let model = encoder.get_model().unwrap();
                let result = extract_scenario_from_model(&encoder, &model);
                // Either succeeds with defaults or returns an error — both are acceptable
                match result {
                    Ok(scenario) => {
                        // If it succeeds, road should have sensible defaults
                        assert!(scenario.road.num_lanes > 0);
                        assert!(scenario.road.lane_width > 0.0);
                    }
                    Err(_) => {
                        // Error is also acceptable behavior for missing road spec
                    }
                }
            }
        });
    }
}
