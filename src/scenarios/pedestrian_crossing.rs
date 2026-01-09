//! Pedestrian crossing scenario model
//!
//! Simplified model: A pedestrian crosses perpendicular to the ego vehicle's path.
//! The pedestrian starts at an initial lateral position and moves to cross the road.
//! No complex sidewalk or timing constraints - just basic crossing behavior.

use crate::dsl::types::ScenarioSpec;
use crate::ltl::formula::{LTLFormula, Proposition};
use crate::scenarios::ScenarioModel;
use anyhow::Result;

/// Pedestrian crossing scenario model
pub struct PedestrianCrossingModel;

impl ScenarioModel for PedestrianCrossingModel {
    fn validate(&self, spec: &ScenarioSpec) -> Result<()> {
        use crate::dsl::types::ActorRole;

        // Validate exactly 2 actors (1 ego vehicle, 1 pedestrian)
        if spec.actors.len() != 2 {
            anyhow::bail!(
                "Pedestrian crossing requires exactly 2 actors, found {}",
                spec.actors.len()
            );
        }

        // Validate roles
        let _ego = spec.ego().map_err(|e| anyhow::anyhow!(e))?;
        let pedestrian = &spec.npcs()[0];

        if pedestrian.role != ActorRole::Pedestrian {
            anyhow::bail!(
                "Second actor must be pedestrian, found {:?}",
                pedestrian.role
            );
        }

        Ok(())
    }

    fn generate_ltl(&self, spec: &ScenarioSpec) -> Result<LTLFormula> {
        let ego = spec.ego().map_err(|e| anyhow::anyhow!(e))?;
        let ego_id = ego.id.as_str();

        // Simple LTL: Ego stays in its lane
        let ego_in_lane = LTLFormula::Atom(Proposition::InLane {
            actor: ego_id.to_string(),
            lane: ego.lane,
        });

        // That's it - just keep ego in lane, let physics handle the rest
        Ok(ego_in_lane)
    }

    fn add_z3_constraints(
        &self,
        spec: &ScenarioSpec,
        encoder: &crate::solver::Z3Encoder,
        backend: &dyn crate::solver::Z3Backend,
        horizon: usize,
    ) -> Result<()> {
        use crate::dsl::ActorRole;
        use z3::ast::Real;

        let npcs = spec.npcs();
        let pedestrian = npcs
            .iter()
            .find(|a| a.role == ActorRole::Pedestrian)
            .ok_or_else(|| anyhow::anyhow!("No pedestrian actor found"))?;
        let pedestrian_id = &pedestrian.id;

        let lane_width = spec.get_lane_width();
        let num_lanes = spec.get_num_lanes();
        let road_width = lane_width * num_lanes as f64;

        // Determine crossing direction based on lane field (0=left, 1=right)
        let (start_y, end_y) = if pedestrian.lane == 0 {
            // Start left of road, end right of road
            (-1.0, road_width + 1.0)
        } else {
            // Start right of road, end left of road
            (road_width + 1.0, -1.0)
        };

        // Constraint: Pedestrian must cross from start to end
        // Initial position: near start_y
        let py_0 = &encoder.positions_y[pedestrian_id][0];
        let start_y_real = Real::from_rational((start_y * 10.0) as i64, 10_i64);
        let start_margin = Real::from_rational(5_i64, 10_i64); // 0.5m margin
        backend.assert(&py_0.ge(&(&start_y_real - &start_margin)));
        backend.assert(&py_0.le(&(&start_y_real + &start_margin)));

        // Final position: near end_y
        let py_final = &encoder.positions_y[pedestrian_id][horizon];
        let end_y_real = Real::from_rational((end_y * 10.0) as i64, 10_i64);
        let end_margin = Real::from_rational(5_i64, 10_i64); // 0.5m margin
        backend.assert(&py_final.ge(&(&end_y_real - &end_margin)));
        backend.assert(&py_final.le(&(&end_y_real + &end_margin)));

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dsl::types::{
        ActorRole, ActorSpec, ConstraintModes, OptimizationTarget, ValueOrRange,
    };
    use std::collections::HashMap;

    fn create_test_spec() -> ScenarioSpec {
        let ego_behavior = HashMap::new();
        let pedestrian_behavior = HashMap::new();

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
                    behavior: ego_behavior,
                },
                ActorSpec {
                    id: "pedestrian".to_string(),
                    role: ActorRole::Pedestrian,
                    lane: 0,
                    position: ValueOrRange::Value(50.0),
                    speed: ValueOrRange::Range([0.8, 1.5]),
                    acceleration: ValueOrRange::Range([-1.0, 1.0]),
                    behavior: pedestrian_behavior,
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
        }
    }

    #[test]
    fn test_pedestrian_validate_success() {
        let model = PedestrianCrossingModel;
        let spec = create_test_spec();
        assert!(model.validate(&spec).is_ok());
    }

    #[test]
    fn test_pedestrian_validate_wrong_role() {
        let model = PedestrianCrossingModel;
        let mut spec = create_test_spec();
        spec.actors[1].role = ActorRole::Npc; // Change pedestrian to NPC
        assert!(model.validate(&spec).is_err());
    }

    #[test]
    fn test_pedestrian_generate_ltl() {
        let model = PedestrianCrossingModel;
        let spec = create_test_spec();
        let formula = model.generate_ltl(&spec);
        assert!(formula.is_ok());

        let formula_str = format!("{}", formula.unwrap());
        assert!(formula_str.contains("InLane"));
    }
}
