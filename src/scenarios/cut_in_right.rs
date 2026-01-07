//! Cut-in from right scenario model
//!
//! In this scenario, an NPC vehicle starts in the right lane (lane 1),
//! ahead of the ego vehicle in the left lane (lane 0), and eventually
//! changes lanes to cut in front of the ego vehicle.

use crate::scenarios::ScenarioModel;
use crate::dsl::types::ScenarioSpec;
use crate::ltl::formula::{LTLFormula, Proposition};
use anyhow::Result;

/// Cut-in from right scenario model
pub struct CutInRightModel;

impl ScenarioModel for CutInRightModel {
    fn validate(&self, spec: &ScenarioSpec) -> Result<()> {
        // Validate exactly 2 actors (ego + 1 npc)
        if spec.actors.len() != 2 {
            anyhow::bail!(
                "Cut-in-right requires exactly 2 actors, found {}",
                spec.actors.len()
            );
        }

        let npc = &spec.npcs()[0];

        // Validate behavior parameters exist
        if !npc.behavior.contains_key("cut_in_time") {
            anyhow::bail!("NPC missing 'cut_in_time' in behavior map");
        }

        Ok(())
    }

    fn generate_ltl(&self, spec: &ScenarioSpec) -> Result<LTLFormula> {
        let ego = spec.ego().map_err(|e| anyhow::anyhow!(e))?;
        let npc = &spec.npcs()[0];

        let ego_id = ego.id.as_str();
        let npc_id = npc.id.as_str();

        // Initial conditions
        let init = self.initial_conditions(spec, ego_id, npc_id);

        // Cut-in behavior
        let behavior = self.cut_in_behavior(spec, ego_id, npc_id);

        Ok(init.and(behavior))
    }

    fn add_z3_constraints(
        &self,
        spec: &ScenarioSpec,
        encoder: &crate::solver::Z3Encoder,
        solver: &z3::Solver,
        horizon: usize,
    ) -> Result<()> {
        use z3::ast::{Ast, Int};

        let ego = spec.ego().map_err(|e| anyhow::anyhow!(e))?;
        let npc = &spec.npcs()[0];
        let target_lane = ego.lane;
        let npc_id = &npc.id;

        // Lane persistence: once NPC is in target lane, it must stay there
        // This is critical because the UNTIL operator only requires reaching the target
        // lane once, but doesn't enforce staying there.
        //
        // We enforce: for all pairs of consecutive time steps, if we're in target at t,
        // we cannot transition back to initial lane at t+1
        let initial_lane = npc.lane;
        for t in 0..horizon {
            let lane_t = &encoder.lanes[npc_id][t];
            let lane_t1 = &encoder.lanes[npc_id][t + 1];
            let target_val = Int::from_i64(encoder.ctx, target_lane as i64);
            let initial_val = Int::from_i64(encoder.ctx, initial_lane as i64);

            // If lane[t] == target_lane, then lane[t+1] != initial_lane
            // (This is stronger than just requiring lane[t+1] == target)
            let in_target = lane_t._eq(&target_val);
            let not_back_to_initial = lane_t1._eq(&initial_val).not();
            let no_return = in_target.implies(&not_back_to_initial);

            solver.assert(&no_return);

            // Also add the positive constraint: if in target, stay in target
            let stays_in_target = lane_t1._eq(&target_val);
            let persistence = in_target.implies(&stays_in_target);
            solver.assert(&persistence);
        }

        Ok(())
    }
}

impl CutInRightModel {
    fn initial_conditions(&self, spec: &ScenarioSpec, ego_id: &str, npc_id: &str) -> LTLFormula {
        let ego = spec.ego().unwrap();
        let npc = spec.npcs()[0];

        LTLFormula::Atom(Proposition::InLane {
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
        }))
    }

    fn cut_in_behavior(&self, spec: &ScenarioSpec, _ego_id: &str, npc_id: &str) -> LTLFormula {
        let ego = spec.ego().unwrap();
        let npc = spec.npcs()[0];

        let target_lane = ego.lane;
        let initial_lane = npc.lane;

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
    use crate::dsl::types::{ActorRole, ActorSpec, ValueOrRange, ConstraintModes};
    use std::collections::HashMap;

    fn create_test_spec() -> ScenarioSpec {
        let ego_behavior = HashMap::new();
        let mut npc_behavior = HashMap::new();
        npc_behavior.insert("cut_in_time".to_string(), serde_json::json!([2.5, 7.5]));

        ScenarioSpec {
            scenario_type: crate::dsl::types::ScenarioType::CutInRight,
            time_step: 0.5,
            duration: 10.0,
            actors: vec![
                ActorSpec {
                    id: "ego".to_string(),
                    role: ActorRole::Ego,
                    lane: 0,  // Ego in LEFT lane for cut-in-right
                    position: ValueOrRange::Value(50.0),
                    speed: ValueOrRange::Value(15.0),
                    acceleration: ValueOrRange::Range([-8.0, 3.0]),
                    behavior: ego_behavior,
                },
                ActorSpec {
                    id: "npc".to_string(),
                    role: ActorRole::Npc,
                    lane: 1,  // NPC in RIGHT lane for cut-in-right
                    position: ValueOrRange::Range([60.0, 80.0]),
                    speed: ValueOrRange::Range([12.0, 14.0]),
                    acceleration: ValueOrRange::Range([-8.0, 3.0]),
                    behavior: npc_behavior,
                },
            ],
            min_ttc: 3.0,
            min_distance: 5.0,
            road: None,
            lane_width: 3.5,
            num_scenarios: 1,
            constraint_modes: ConstraintModes::default(),
            max_acceleration: None,
            max_deceleration: None,
        }
    }

    #[test]
    fn test_cut_in_right_validate_success() {
        let model = CutInRightModel;
        let spec = create_test_spec();
        assert!(model.validate(&spec).is_ok());
    }

    #[test]
    fn test_cut_in_right_validate_missing_cut_in_time() {
        let model = CutInRightModel;
        let mut spec = create_test_spec();
        spec.actors[1].behavior.clear(); // Remove cut_in_time
        assert!(model.validate(&spec).is_err());
    }

    #[test]
    fn test_cut_in_right_generate_ltl() {
        let model = CutInRightModel;
        let spec = create_test_spec();
        let formula = model.generate_ltl(&spec);
        assert!(formula.is_ok());

        let formula_str = format!("{}", formula.unwrap());
        assert!(formula_str.contains("InLane"));
        assert!(formula_str.contains("Ahead"));
    }
}
