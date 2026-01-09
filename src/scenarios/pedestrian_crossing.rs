//! Pedestrian crossing scenario model
//!
//! In this scenario, a pedestrian starts on one side of the road (sidewalk)
//! and crosses perpendicular to the ego vehicle's path. The pedestrian
//! starts on the sidewalk (lateral position outside road boundaries),
//! waits, then crosses to the opposite sidewalk.

use crate::dsl::types::{ScenarioSpec, ValueOrRange};
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

        // Validate behavior parameters exist
        if !pedestrian.behavior.contains_key("crossing_time") {
            anyhow::bail!("Pedestrian missing 'crossing_time' in behavior map");
        }

        Ok(())
    }

    fn generate_ltl(&self, spec: &ScenarioSpec) -> Result<LTLFormula> {
        let ego = spec.ego().map_err(|e| anyhow::anyhow!(e))?;
        let pedestrian = &spec.npcs()[0];

        let ego_id = ego.id.as_str();
        let pedestrian_id = pedestrian.id.as_str();

        // Initial conditions
        let init = self.initial_conditions(spec, ego_id, pedestrian_id);

        // Crossing behavior
        let behavior = self.crossing_behavior(spec, ego_id, pedestrian_id);

        Ok(init.and(behavior))
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
        let initial_side = if pedestrian.lane == 0 {
            "left"
        } else {
            "right"
        };

        // Parse crossing_time from behavior
        let crossing_time_json = pedestrian
            .behavior
            .get("crossing_time")
            .ok_or_else(|| anyhow::anyhow!("Pedestrian missing 'crossing_time' in behavior"))?;

        let crossing_time: ValueOrRange = serde_json::from_value(crossing_time_json.clone())
            .map_err(|e| anyhow::anyhow!("Failed to parse crossing_time: {}", e))?;

        let time_step = spec.time_step;
        let (min_time, max_time) = match crossing_time {
            ValueOrRange::Value(t) => (t, t),
            ValueOrRange::Range([min, max]) => (min, max),
        };

        // Convert time to time step indices
        let min_step = (min_time / time_step).ceil() as usize;
        let max_step = (max_time / time_step).floor() as usize;

        let lane_width = spec.get_lane_width();
        let num_lanes = spec.get_num_lanes();
        let road_width = lane_width * num_lanes as f64;

        // Constraint: pedestrian on initial sidewalk before crossing_time_min
        let road_width_real = Real::from_rational((road_width * 10.0) as i64, 10_i64);

        for t in 0..min_step.saturating_sub(1) {
            let py_t = &encoder.positions_y[pedestrian_id][t];

            if initial_side == "left" {
                // Left sidewalk: py <= -0.4 (allow small margin)
                let sidewalk_limit = Real::from_rational(-4_i64, 10_i64); // -0.4
                let on_sidewalk_left = py_t.le(&sidewalk_limit);
                backend.assert(&on_sidewalk_left);
            } else {
                // Right sidewalk: py >= road_width + 0.4
                let sidewalk_limit = &road_width_real + &Real::from_rational(4_i64, 10_i64); // road_width + 0.4
                let on_sidewalk_right = py_t.ge(&sidewalk_limit);
                backend.assert(&on_sidewalk_right);
            }
        }

        // Constraint: pedestrian on opposite sidewalk after crossing_time_max
        for t in max_step..=horizon {
            let py_t = &encoder.positions_y[pedestrian_id][t];

            if initial_side == "left" {
                // Crossed to right: py >= road_width + 0.4
                let sidewalk_limit = &road_width_real + &Real::from_rational(4_i64, 10_i64);
                let on_opposite = py_t.ge(&sidewalk_limit);
                backend.assert(&on_opposite);
            } else {
                // Crossed to left: py <= -0.4
                let sidewalk_limit = Real::from_rational(-4_i64, 10_i64);
                let on_opposite = py_t.le(&sidewalk_limit);
                backend.assert(&on_opposite);
            }
        }

        Ok(())
    }
}

impl PedestrianCrossingModel {
    fn initial_conditions(
        &self,
        spec: &ScenarioSpec,
        ego_id: &str,
        pedestrian_id: &str,
    ) -> LTLFormula {
        let ego = spec.ego().unwrap();
        let pedestrian = spec.npcs()[0];

        let initial_side = if pedestrian.lane == 0 {
            "left"
        } else {
            "right"
        };

        // Ego in lane
        let ego_in_lane = LTLFormula::Atom(Proposition::InLane {
            actor: ego_id.to_string(),
            lane: ego.lane,
        });

        // Pedestrian on initial sidewalk
        let pedestrian_on_sidewalk = LTLFormula::Atom(Proposition::OnSidewalk {
            actor: pedestrian_id.to_string(),
            side: initial_side.to_string(),
        });

        ego_in_lane.and(pedestrian_on_sidewalk)
    }

    fn crossing_behavior(
        &self,
        _spec: &ScenarioSpec,
        _ego_id: &str,
        pedestrian_id: &str,
    ) -> LTLFormula {
        let initial_side = "left";
        let opposite_side = "right";

        // Pedestrian stays on initial sidewalk UNTIL crossing to opposite sidewalk
        // Note: Sidewalk persistence after crossing is enforced by direct Z3 constraints
        LTLFormula::Atom(Proposition::OnSidewalk {
            actor: pedestrian_id.to_string(),
            side: initial_side.to_string(),
        })
        .until(LTLFormula::Atom(Proposition::OnSidewalk {
            actor: pedestrian_id.to_string(),
            side: opposite_side.to_string(),
        }))
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
        let mut pedestrian_behavior = HashMap::new();
        pedestrian_behavior.insert("crossing_time".to_string(), serde_json::json!([2.5, 5.5]));

        ScenarioSpec {
            scenario_type: crate::dsl::types::ScenarioType::PedestrianCrossing,
            time_step: 0.1,
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
                    acceleration: ValueOrRange::Range([-0.5, 0.5]),
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
    fn test_pedestrian_validate_missing_crossing_time() {
        let model = PedestrianCrossingModel;
        let mut spec = create_test_spec();
        spec.actors[1].behavior.clear(); // Remove crossing_time
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
        assert!(formula_str.contains("OnSidewalk"));
    }
}
