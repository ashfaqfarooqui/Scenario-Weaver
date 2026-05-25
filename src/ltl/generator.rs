//! LTL formula generation from DSL specifications

use crate::dsl::types::ScenarioSpec;
use crate::error::Result;

/// Generates LTL formulas from a [`ScenarioSpec`] by combining behavioral
/// constraints (from the scenario model) with safety constraints (TTC, distance, etc.).
pub struct LTLGenerator;

impl LTLGenerator {
    /// Generate the complete LTL formula for a scenario.
    ///
    /// Combines the scenario-specific behavioral formula (lane changes, ordering)
    /// with pairwise safety constraints (TTC, distance) according to the
    /// configured [`ConstraintModes`](crate::dsl::types::ConstraintModes).
    ///
    /// **Canonical validation point.** All public generation paths
    /// (`generate_single_scenario_from_spec`, `generate_multiple_scenarios_from_spec`)
    /// funnel through here, so validation runs exactly once per call.
    /// Callers must not call `model.validate()` themselves before invoking this.
    pub fn generate(spec: &ScenarioSpec) -> Result<crate::ltl::formula::LTLFormula> {
        let model = spec.scenario_type.get_model();
        model.validate(spec)?;

        let behavior = model.generate_ltl(spec)?;

        // generate_safety() uses the trait default (pairwise TTC + distance) unless the
        // scenario type overrides it.
        let safety = model.generate_safety(spec)?;

        Ok(behavior.and(safety))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dsl::types::{
        ActorRole, ActorSpec, CoordinateSystem, ConstraintMode, ConstraintModes,
        LaneChangeConfig, LaneChangeDirection, OptimizationTarget, RoadSpec, ValueOrRange,
    };
    use crate::ltl::formula::LTLFormula;
    use std::collections::HashMap;

    fn create_test_spec() -> ScenarioSpec {
        ScenarioSpec {
            scenario_type: crate::dsl::types::ScenarioType::CutInLeft,
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
            road: None,
            lane_width: 3.5,
            num_scenarios: 1,
            constraint_modes: ConstraintModes::default(),
            optimization_target: OptimizationTarget::None,
            max_acceleration: None,
            max_deceleration: None,
            max_velocity: None,
            min_velocity: None,
            min_lateral_distance: None,
            max_relative_velocity: None,
            max_lateral_acceleration: 2.0,
            coordinate_system: CoordinateSystem::Cartesian,
            bicycle_config: None,
        }
    }

    #[test]
    fn test_generate_cut_in_left() {
        let spec = create_test_spec();
        let formula = LTLGenerator::generate(&spec).unwrap();

        println!("Generated LTL formula:");
        println!("{}", formula);

        // Should be a conjunction (AND)
        assert!(matches!(formula, LTLFormula::And(_, _)));
    }

    #[test]
    fn test_cut_in_left_formula_structure() {
        let spec = create_test_spec();
        let formula = LTLGenerator::generate(&spec).unwrap();

        let formula_str = format!("{}", formula);
        assert!(formula_str.contains("InLane"));
        assert!(formula_str.contains("Ahead"));
        assert!(formula_str.contains("G(")); // Safety constraints
    }

    fn create_cut_in_right_spec() -> ScenarioSpec {
        ScenarioSpec {
            scenario_type: crate::dsl::types::ScenarioType::CutInRight,
            time_step: 0.5,
            duration: 10.0,
            actors: vec![
                ActorSpec {
                    id: "ego".to_string(),
                    role: ActorRole::Ego,
                    lane: 0,
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
                    lane: 1,
                    position: ValueOrRange::Range([60.0, 80.0]),
                    speed: ValueOrRange::Range([12.0, 14.0]),
                    acceleration: ValueOrRange::Range([-8.0, 3.0]),
                    direction: 1,
                    behavior: HashMap::new(),
                    lane_changes: vec![LaneChangeConfig {
                        direction: LaneChangeDirection::Left,
                        start_time: ValueOrRange::Range([2.5, 7.5]),
                        duration: ValueOrRange::Range([3.0, 4.0]),
                    }],
                    bicycle_params: None,
                },
            ],
            min_ttc: 3.0,
            min_distance: 5.0,
            road: None,
            lane_width: 3.5,
            num_scenarios: 1,
            constraint_modes: ConstraintModes::default(),
            optimization_target: OptimizationTarget::None,
            max_acceleration: None,
            max_deceleration: None,
            max_velocity: None,
            min_velocity: None,
            min_lateral_distance: None,
            max_relative_velocity: None,
            max_lateral_acceleration: 2.0,
            coordinate_system: CoordinateSystem::Cartesian,
            bicycle_config: None,
        }
    }

    #[test]
    fn test_generate_cut_in_right() {
        let spec = create_cut_in_right_spec();
        let formula = LTLGenerator::generate(&spec).unwrap();

        let formula_str = format!("{}", formula);
        assert!(formula_str.contains("InLane"));
        assert!(matches!(formula, LTLFormula::And(_, _)));
    }

    fn create_overtake_left_spec() -> ScenarioSpec {
        ScenarioSpec {
            scenario_type: crate::dsl::types::ScenarioType::OvertakeLeft,
            time_step: 0.5,
            duration: 10.0,
            actors: vec![
                ActorSpec {
                    id: "ego".to_string(),
                    role: ActorRole::Ego,
                    lane: 1,
                    position: ValueOrRange::Value(50.0),
                    speed: ValueOrRange::Value(15.0),
                    acceleration: ValueOrRange::Range([-3.0, 2.0]),
                    direction: 1,
                    behavior: HashMap::new(),
                    lane_changes: vec![],
                    bicycle_params: None,
                },
                ActorSpec {
                    id: "npc".to_string(),
                    role: ActorRole::Npc,
                    lane: 1,
                    position: ValueOrRange::Range([30.0, 40.0]),
                    speed: ValueOrRange::Range([18.0, 22.0]),
                    acceleration: ValueOrRange::Range([-3.0, 4.0]),
                    direction: 1,
                    behavior: HashMap::new(),
                    lane_changes: vec![
                        LaneChangeConfig {
                            direction: LaneChangeDirection::Left,
                            start_time: ValueOrRange::Range([2.0, 3.0]),
                            duration: ValueOrRange::Range([1.5, 2.0]),
                        },
                        LaneChangeConfig {
                            direction: LaneChangeDirection::Right,
                            start_time: ValueOrRange::Range([6.0, 7.0]),
                            duration: ValueOrRange::Range([1.5, 2.0]),
                        },
                    ],
                    bicycle_params: None,
                },
            ],
            min_ttc: 2.0,
            min_distance: 5.0,
            road: None,
            lane_width: 3.5,
            num_scenarios: 1,
            constraint_modes: ConstraintModes::default(),
            optimization_target: OptimizationTarget::None,
            max_acceleration: None,
            max_deceleration: None,
            max_velocity: None,
            min_velocity: None,
            min_lateral_distance: None,
            max_relative_velocity: None,
            max_lateral_acceleration: 2.0,
            coordinate_system: CoordinateSystem::Cartesian,
            bicycle_config: None,
        }
    }

    #[test]
    fn test_generate_overtake_left() {
        let spec = create_overtake_left_spec();
        let formula = LTLGenerator::generate(&spec).unwrap();

        assert!(matches!(formula, LTLFormula::And(_, _)));
        let formula_str = format!("{}", formula);
        assert!(formula_str.contains("InLane"));
    }

    fn create_head_on_spec() -> ScenarioSpec {
        ScenarioSpec {
            scenario_type: crate::dsl::types::ScenarioType::HeadOn,
            time_step: 0.5,
            duration: 10.0,
            actors: vec![
                ActorSpec {
                    id: "ego".to_string(),
                    role: ActorRole::Ego,
                    lane: 0,
                    position: ValueOrRange::Value(0.0),
                    speed: ValueOrRange::Value(15.0),
                    acceleration: ValueOrRange::Range([-5.0, 3.0]),
                    direction: 1,
                    behavior: HashMap::new(),
                    lane_changes: vec![
                        LaneChangeConfig {
                            direction: LaneChangeDirection::Right,
                            start_time: ValueOrRange::Range([2.0, 3.0]),
                            duration: ValueOrRange::Range([2.0, 3.0]),
                        },
                        LaneChangeConfig {
                            direction: LaneChangeDirection::Left,
                            start_time: ValueOrRange::Range([6.0, 7.0]),
                            duration: ValueOrRange::Range([2.0, 3.0]),
                        },
                    ],
                    bicycle_params: None,
                },
                ActorSpec {
                    id: "slow_npc".to_string(),
                    role: ActorRole::Npc,
                    lane: 0,
                    position: ValueOrRange::Range([40.0, 60.0]),
                    speed: ValueOrRange::Range([8.0, 10.0]),
                    acceleration: ValueOrRange::Range([-3.0, 1.0]),
                    direction: 1,
                    behavior: HashMap::new(),
                    lane_changes: vec![],
                    bicycle_params: None,
                },
                ActorSpec {
                    id: "oncoming_npc".to_string(),
                    role: ActorRole::Npc,
                    lane: 1,
                    position: ValueOrRange::Range([150.0, 180.0]),
                    speed: ValueOrRange::Range([12.0, 15.0]),
                    acceleration: ValueOrRange::Range([-2.0, 1.0]),
                    direction: -1,
                    behavior: HashMap::new(),
                    lane_changes: vec![],
                    bicycle_params: None,
                },
            ],
            min_ttc: 2.0,
            min_distance: 5.0,
            road: Some(RoadSpec {
                num_lanes: 2,
                lane_width: 3.5,
                lane_directions: vec![1, -1],
                road_length: None,
            }),
            lane_width: 3.5,
            num_scenarios: 1,
            constraint_modes: ConstraintModes::default(),
            optimization_target: OptimizationTarget::None,
            max_acceleration: None,
            max_deceleration: None,
            max_velocity: None,
            min_velocity: None,
            min_lateral_distance: None,
            max_relative_velocity: None,
            max_lateral_acceleration: 2.0,
            coordinate_system: CoordinateSystem::Cartesian,
            bicycle_config: None,
        }
    }

    #[test]
    fn test_generate_head_on() {
        let spec = create_head_on_spec();
        let formula = LTLGenerator::generate(&spec).unwrap();

        let formula_str = format!("{}", formula);
        // HeadOn returns True for behavioral LTL but adds safety constraints
        assert!(formula_str.contains("G("));
    }

    fn create_pedestrian_crossing_spec() -> ScenarioSpec {
        let mut pedestrian_behavior = HashMap::new();
        pedestrian_behavior
            .insert("direction".to_string(), serde_json::json!("left_to_right"));

        ScenarioSpec {
            scenario_type: crate::dsl::types::ScenarioType::PedestrianCrossing,
            time_step: 0.5,
            duration: 10.0,
            actors: vec![
                ActorSpec {
                    id: "ego".to_string(),
                    role: ActorRole::Ego,
                    lane: 0,
                    position: ValueOrRange::Value(0.0),
                    speed: ValueOrRange::Value(10.0),
                    acceleration: ValueOrRange::Range([-3.0, 2.0]),
                    direction: 1,
                    behavior: HashMap::new(),
                    lane_changes: vec![],
                    bicycle_params: None,
                },
                ActorSpec {
                    id: "pedestrian".to_string(),
                    role: ActorRole::Pedestrian,
                    lane: 0,
                    position: ValueOrRange::Value(50.0),
                    speed: ValueOrRange::Range([0.8, 1.5]),
                    acceleration: ValueOrRange::Range([-1.0, 1.0]),
                    direction: 1,
                    behavior: pedestrian_behavior,
                    lane_changes: vec![],
                    bicycle_params: None,
                },
            ],
            min_ttc: 2.0,
            min_distance: 2.0,
            road: None,
            lane_width: 3.5,
            num_scenarios: 1,
            constraint_modes: ConstraintModes::default(),
            optimization_target: OptimizationTarget::None,
            max_acceleration: None,
            max_deceleration: None,
            max_velocity: None,
            min_velocity: None,
            min_lateral_distance: None,
            max_relative_velocity: None,
            max_lateral_acceleration: 2.0,
            coordinate_system: CoordinateSystem::Cartesian,
            bicycle_config: None,
        }
    }

    #[test]
    fn test_generate_pedestrian_crossing() {
        let spec = create_pedestrian_crossing_spec();
        let formula = LTLGenerator::generate(&spec).unwrap();

        let formula_str = format!("{}", formula);
        assert!(formula_str.contains("PedestrianTTC"));
    }

    #[test]
    fn test_generate_validation_failure() {
        let mut spec = create_test_spec();
        // Remove lane changes from NPC — CutInLeft requires at least one
        spec.actors[1].lane_changes = vec![];

        let result = LTLGenerator::generate(&spec);
        assert!(result.is_err());
        let err_str = format!("{}", result.unwrap_err());
        assert!(err_str.contains("lane_change"));
    }

    #[test]
    fn test_generate_with_constraint_mode_ignore() {
        let mut spec = create_test_spec();
        spec.constraint_modes = ConstraintModes::Detailed {
            min_ttc: ConstraintMode::Ignore,
            min_distance: ConstraintMode::Ignore,
            max_acceleration: ConstraintMode::Ignore,
            max_velocity: ConstraintMode::Ignore,
            min_velocity: ConstraintMode::Ignore,
            min_lateral_distance: ConstraintMode::Ignore,
            max_relative_velocity: ConstraintMode::Ignore,
        };

        let formula = LTLGenerator::generate(&spec).unwrap();
        let formula_str = format!("{}", formula);
        // With all constraints ignored, no TTC or distance references
        assert!(!formula_str.contains("TTC"));
        assert!(!formula_str.contains("DistanceGT"));
    }

    #[test]
    fn test_generate_with_constraint_mode_violate() {
        let mut spec = create_test_spec();
        spec.constraint_modes = ConstraintModes::Detailed {
            min_ttc: ConstraintMode::Enforce,
            min_distance: ConstraintMode::Violate,
            max_acceleration: ConstraintMode::Enforce,
            max_velocity: ConstraintMode::Enforce,
            min_velocity: ConstraintMode::Ignore,
            min_lateral_distance: ConstraintMode::Ignore,
            max_relative_velocity: ConstraintMode::Ignore,
        };

        let formula = LTLGenerator::generate(&spec).unwrap();
        let formula_str = format!("{}", formula);
        // Violate mode negates the constraint: F(NOT DistanceGT(...))
        assert!(formula_str.contains("DistanceGT"));
        assert!(formula_str.contains("\u{00ac}")); // ¬ negation
    }
}
