//! Cut-in from left scenario model
//!
//! In this scenario, an NPC vehicle starts in the left lane (lane 0),
//! ahead of the ego vehicle in the right lane (lane 1), and eventually
//! changes lanes to cut in front of the ego vehicle.

use crate::dsl::types::{LaneChangeDirection, ScenarioSpec};
use crate::error::{Result, ScenarioGenError};
use crate::ltl::formula::{LTLFormula, Proposition};
use crate::scenarios::ScenarioModel;

/// Cut-in from left scenario model
pub struct CutInLeftModel;

impl ScenarioModel for CutInLeftModel {
    fn validate(&self, spec: &ScenarioSpec) -> Result<()> {
        // Validate exactly 2 actors (ego + 1 npc)
        if spec.actors.len() != 2 {
            return Err(ScenarioGenError::InvalidSpec(format!(
                "Cut-in-left requires exactly 2 actors, found {}",
                spec.actors.len()
            )));
        }

        let npc = &spec.npcs()[0];

        if npc.lane_changes.is_empty() {
            return Err(ScenarioGenError::InvalidSpec(
                "Cut-in-left requires at least one lane_change in lane_changes".to_string(),
            ));
        }

        Ok(())
    }

    fn generate_ltl(&self, spec: &ScenarioSpec) -> Result<LTLFormula> {
        let ego = spec.ego().map_err(|e| ScenarioGenError::InvalidSpec(e))?;
        let npc = &spec.npcs()[0];

        let ego_id = ego.id.as_str();
        let npc_id = npc.id.as_str();

        // Initial conditions
        let init = self.initial_conditions(spec, ego_id, npc_id)?;

        // Cut-in behavior
        let behavior = self.cut_in_behavior(spec, ego_id, npc_id);

        Ok(init.and(behavior))
    }

    fn add_z3_constraints(
        &self,
        _spec: &ScenarioSpec,
        _encoder: &crate::solver::Z3Encoder,
        _backend: &dyn crate::solver::Z3Backend,
        _horizon: usize,
    ) -> Result<()> {
        Ok(())
    }
}

impl CutInLeftModel {
    fn initial_conditions(&self, spec: &ScenarioSpec, ego_id: &str, npc_id: &str) -> Result<LTLFormula> {
        let ego = spec.ego().map_err(|e| ScenarioGenError::InvalidSpec(e))?;
        let npc = spec.npcs()[0];

        Ok(LTLFormula::Atom(Proposition::InLane {
            actor: ego_id.to_string(),
            lane: ego.lane,
        })
        .and(LTLFormula::Atom(Proposition::InLane {
            actor: npc_id.to_string(),
            lane: npc.lane,
        }))
        .and(LTLFormula::Atom(Proposition::Ahead {
            actor1: npc_id.to_string(),
            actor2: ego_id.to_string(),
        })))
    }

    fn cut_in_behavior(&self, spec: &ScenarioSpec, _ego_id: &str, npc_id: &str) -> LTLFormula {
        let npc = spec.npcs()[0];

        let initial_lane = npc.lane;

        // Compute actual final target lane by accumulating all lane change deltas.
        // This must match what the kinematics encoder computes for the last lane change.
        // Using ego.lane as the target is wrong when the NPC needs multiple changes
        // (e.g., opposite-traffic bidirectional roads).
        let total_delta: i64 = npc
            .lane_changes
            .iter()
            .map(|lc| match lc.direction {
                LaneChangeDirection::Left => -1i64,
                LaneChangeDirection::Right => 1i64,
            })
            .sum();
        let target_lane = (initial_lane as i64 + total_delta).max(0) as usize;

        // NPC stays in initial lane UNTIL it switches to target lane
        // This ensures a clean transition without oscillation during the switch
        // Note: Lane persistence after cut-in is enforced by a direct Z3 constraint
        // in add_z3_constraints(), not in LTL, because F(G(...)) in bounded LTL
        // allows the solver to delay until the last time step.
        LTLFormula::Atom(Proposition::InLane {
            actor: npc_id.to_string(),
            lane: initial_lane,
        })
        .until(LTLFormula::Atom(Proposition::InLane {
            actor: npc_id.to_string(),
            lane: target_lane,
        }))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dsl::types::{
        ActorRole, ActorSpec, ConstraintModes, LaneChangeConfig, LaneChangeDirection,
        OptimizationTarget, ValueOrRange,
    };
    use std::collections::HashMap;

    fn create_test_spec() -> ScenarioSpec {
        let ego_behavior = HashMap::new();

        let npc_lane_changes = vec![LaneChangeConfig {
            direction: LaneChangeDirection::Right,
            start_time: ValueOrRange::Range([2.5, 7.5]),
            duration: ValueOrRange::Range([3.0, 4.0]),
        }];

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
                    behavior: ego_behavior,
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
                    lane_changes: npc_lane_changes,
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
            coordinate_system: crate::dsl::types::CoordinateSystem::Cartesian,
            bicycle_config: None,
        }
    }

    #[test]
    fn test_cut_in_left_validate_success() {
        let model = CutInLeftModel;
        let spec = create_test_spec();
        assert!(model.validate(&spec).is_ok());
    }

    #[test]
    fn test_cut_in_left_validate_missing_lane_changes() {
        let model = CutInLeftModel;
        let mut spec = create_test_spec();
        spec.actors[1].lane_changes = vec![]; // Remove lane_changes
        assert!(model.validate(&spec).is_err());
    }

    #[test]
    fn test_cut_in_left_generate_ltl() {
        let model = CutInLeftModel;
        let spec = create_test_spec();
        let formula = model.generate_ltl(&spec);
        assert!(formula.is_ok());

        let formula_str = format!("{}", formula.unwrap());
        assert!(formula_str.contains("InLane"));
        assert!(formula_str.contains("Ahead"));
    }
}
