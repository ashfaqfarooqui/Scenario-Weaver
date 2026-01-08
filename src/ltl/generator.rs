//! LTL formula generation from DSL specifications

use crate::dsl::types::ScenarioSpec;

pub struct LTLGenerator;

impl LTLGenerator {
    /// Generate LTL formula from scenario specification using ScenarioModel trait
    pub fn generate(spec: &ScenarioSpec) -> crate::ltl::formula::LTLFormula {
        // Get scenario model
        let model = spec.scenario_type.get_model();

        // Validate scenario-specific requirements
        if let Err(e) = model.validate(spec) {
            panic!("Scenario validation failed: {}", e);
        }

        // Generate behavioral LTL
        let behavior = model
            .generate_ltl(spec)
            .expect("Failed to generate behavioral LTL");

        // Generate safety constraints (uses trait default or override)
        let safety = model
            .generate_safety(spec)
            .expect("Failed to generate safety constraints");

        // Combine: behavior AND safety
        behavior.and(safety)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dsl::types::{
        ActorRole, ActorSpec, ConstraintModes, OptimizationTarget, ValueOrRange,
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
            optimization_target: OptimizationTarget::None,
            max_acceleration: None,
            max_deceleration: None,
        }
    }

    #[test]
    fn test_generate_cut_in_left() {
        let spec = create_test_spec();
        let formula = LTLGenerator::generate(&spec);

        println!("Generated LTL formula:");
        println!("{}", formula);

        // Should be a conjunction (AND)
        assert!(matches!(formula, LTLFormula::And(_, _)));
    }

    #[test]
    fn test_cut_in_left_formula_structure() {
        let spec = create_test_spec();
        let formula = LTLGenerator::generate(&spec);

        let formula_str = format!("{}", formula);
        assert!(formula_str.contains("InLane"));
        assert!(formula_str.contains("Ahead"));
        assert!(formula_str.contains("G(")); // Safety constraints
    }
}
