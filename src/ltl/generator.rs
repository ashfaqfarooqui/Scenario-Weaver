//! LTL formula generation from DSL specifications

use crate::dsl::types::ScenarioSpec;
use crate::error::Result;

pub struct LTLGenerator;

impl LTLGenerator {
    /// Generate LTL formula from scenario specification using ScenarioModel trait
    pub fn generate(spec: &ScenarioSpec) -> Result<crate::ltl::formula::LTLFormula> {
        // Get scenario model
        let model = spec.scenario_type.get_model();

        // Validate scenario-specific requirements
        model.validate(spec)?;

        // Generate behavioral LTL
        let behavior = model.generate_ltl(spec)?;

        // Generate safety constraints (uses trait default or override)
        let safety = model.generate_safety(spec)?;

        // Combine: behavior AND safety
        Ok(behavior.and(safety))
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
        use crate::dsl::types::CoordinateSystem;
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
                    lane_change: None,
                },
                ActorSpec {
                    id: "npc".to_string(),
                    role: ActorRole::Npc,
                    lane: 0,
                    position: ValueOrRange::Range([60.0, 80.0]),
                    speed: ValueOrRange::Range([12.0, 14.0]),
                    acceleration: ValueOrRange::Range([-8.0, 3.0]),
                    direction: 1,
                    behavior: {
                        let mut map = HashMap::new();
                        map.insert("cut_in_time".to_string(), serde_json::json!([2.5, 7.5]));
                        map
                    },
                    lane_change: None,
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
            coordinate_system: CoordinateSystem::default(),
            reference_line: None,
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
}
