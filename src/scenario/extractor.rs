//! Scenario extraction from Z3 models
//!
//! This module provides utilities for extracting scenario data from Z3 models.
//! The main extraction logic is implemented in the Z3Encoder (Phase 9).

use crate::scenario::model::Scenario;

/// Extract scenario from Z3 model
///
/// Note: The actual extraction is implemented in Z3Encoder::extract_scenario()
/// This module exists to provide a clean public API and future extensions.
pub fn extract_scenario_from_model<'ctx>(
    encoder: &crate::solver::encoder::Z3Encoder<'ctx>,
    model: &z3::Model<'ctx>,
) -> Scenario {
    encoder.extract_scenario(model)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dsl::types::{ActorRole, ActorSpec, ScenarioSpec, ScenarioType, ValueOrRange};
    use crate::ltl::generator::LTLGenerator;
    use crate::solver::encoder::Z3Encoder;
    use std::collections::HashMap;
    use z3::{Config, Context, SatResult};

    fn create_test_spec() -> ScenarioSpec {
        let ego = ActorSpec {
            id: "ego".to_string(),
            role: ActorRole::Ego,
            lane: 1,
            position: ValueOrRange::Value(50.0),
            speed: ValueOrRange::Value(15.0),
            acceleration: ValueOrRange::Range([-8.0, 3.0]),
            behavior: HashMap::new(),
        };

        let mut npc_behavior = HashMap::new();
        npc_behavior.insert(
            "cut_in_time".to_string(),
            serde_json::json!([2.5, 7.5]),
        );

        let npc = ActorSpec {
            id: "npc".to_string(),
            role: ActorRole::Npc,
            lane: 0,
            position: ValueOrRange::Range([60.0, 80.0]),
            speed: ValueOrRange::Range([12.0, 14.0]),
            acceleration: ValueOrRange::Range([-8.0, 3.0]),
            behavior: npc_behavior,
        };

        ScenarioSpec {
            scenario_type: ScenarioType::CutInLeft,
            time_step: 0.5,
            duration: 10.0,
            actors: vec![ego, npc],
            min_ttc: 3.0,
            min_distance: 5.0,
            lane_width: 3.5,
            num_scenarios: 1,
            constraint_modes: crate::dsl::types::ConstraintModes::default(),
            max_acceleration: None,
            max_deceleration: None,
        }
    }

    #[test]
    fn test_extract_scenario_integration() {
        let cfg = Config::new();
        let ctx = Context::new(&cfg);
        let spec = create_test_spec();

        let mut encoder = Z3Encoder::new(&ctx, spec.clone());
        encoder.create_variables();
        encoder.encode_initial_conditions();
        encoder.encode_kinematics();

        let ltl_formula = LTLGenerator::generate(&spec);
        encoder.encode_ltl(&ltl_formula);
        encoder.encode_safety();

        let result = encoder.check();
        assert_eq!(result, SatResult::Sat);

        if result == SatResult::Sat {
            let model = encoder.get_model().unwrap();
            let scenario = extract_scenario_from_model(&encoder, &model);

            // Verify extraction worked
            assert_eq!(scenario.actors.len(), 2);
            assert!(scenario.get_actor("ego").is_some());
            assert!(scenario.get_actor("npc").is_some());

            println!("Integration test passed - scenario extracted successfully");
        }
    }
}
