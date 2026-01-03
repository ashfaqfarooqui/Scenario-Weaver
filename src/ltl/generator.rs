//! LTL formula generation from DSL specifications

use crate::dsl::types::ScenarioSpec;
use crate::ltl::formula::LTLFormula;
use crate::ltl::plugin::REGISTRY;

pub struct LTLGenerator;

impl LTLGenerator {
    /// Generate LTL formula from scenario specification using plugin system
    pub fn generate(spec: &ScenarioSpec) -> LTLFormula {
        REGISTRY
            .lock()
            .unwrap()
            .generate(spec)
            .expect("Failed to generate LTL formula")
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dsl::types::{ActorRole, ActorSpec, ScenarioType, ValueOrRange};
    use std::collections::HashMap;

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
    fn test_generate_via_plugin_system() {
        let spec = create_test_spec();
        let formula = LTLGenerator::generate(&spec);

        println!("Generated LTL formula via plugin system:");
        println!("{}", formula);

        // Should be a conjunction (AND)
        assert!(matches!(formula, LTLFormula::And(_, _)));
    }
}
