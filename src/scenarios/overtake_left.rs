//! Overtake from left scenario model
//!
//! In this scenario, an NPC vehicle starts behind the ego vehicle in the same lane,
//! moves to the left lane (passing lane), accelerates to pass the ego, then returns
//! to the original lane ahead of the ego vehicle.

use crate::dsl::types::{ScenarioSpec, ValueOrRange};
use crate::error::{Result, ScenarioGenError};
use crate::ltl::formula::{LTLFormula, Proposition};
use crate::scenarios::ScenarioModel;

/// Overtake from left scenario model
pub struct OvertakeLeftModel;

impl ScenarioModel for OvertakeLeftModel {
    fn validate(&self, spec: &ScenarioSpec) -> Result<()> {
        // Validate exactly 2 actors (ego + 1 npc)
        if spec.actors.len() != 2 {
            return Err(ScenarioGenError::InvalidSpec(format!(
                "Overtake-left requires exactly 2 actors, found {}",
                spec.actors.len()
            )));
        }

        let ego = spec.ego().map_err(|e| ScenarioGenError::InvalidSpec(e))?;
        let npc = &spec.npcs()[0];

        // Validate NPC starts in same lane as ego
        if npc.lane != ego.lane {
            return Err(ScenarioGenError::InvalidSpec(format!(
                "Overtake-left requires NPC to start in same lane as ego. Ego lane: {}, NPC lane: {}",
                ego.lane,
                npc.lane
            )));
        }

        // Validate passing lane exists (left of current lane)
        if ego.lane == 0 {
            return Err(ScenarioGenError::InvalidSpec(
                "Overtake-left requires a lane to the left. Ego is in lane 0, no left lane available".to_string()
            ));
        }

        // Validate behavior parameters exist
        if !npc.behavior.contains_key("overtake_start_time") {
            return Err(ScenarioGenError::InvalidSpec(
                "NPC missing 'overtake_start_time' in behavior map".to_string(),
            ));
        }
        if !npc.behavior.contains_key("overtake_end_time") {
            return Err(ScenarioGenError::InvalidSpec(
                "NPC missing 'overtake_end_time' in behavior map".to_string(),
            ));
        }

        // Parse and validate timing: start_time.max < end_time.min
        let start_time_json = npc.behavior.get("overtake_start_time").unwrap();
        let end_time_json = npc.behavior.get("overtake_end_time").unwrap();

        let start_time: ValueOrRange =
            serde_json::from_value(start_time_json.clone()).map_err(|e| {
                ScenarioGenError::InvalidSpec(format!("Failed to parse overtake_start_time: {}", e))
            })?;
        let end_time: ValueOrRange =
            serde_json::from_value(end_time_json.clone()).map_err(|e| {
                ScenarioGenError::InvalidSpec(format!("Failed to parse overtake_end_time: {}", e))
            })?;

        // Ensure start_time.max < end_time.min for valid timing
        if start_time.max() >= end_time.min() {
            return Err(ScenarioGenError::InvalidSpec(format!(
                "overtake_start_time.max ({}) must be less than overtake_end_time.min ({})",
                start_time.max(),
                end_time.min()
            )));
        }

        Ok(())
    }

    fn generate_ltl(&self, spec: &ScenarioSpec) -> Result<LTLFormula> {
        let ego = spec.ego().map_err(|e| ScenarioGenError::InvalidSpec(e))?;
        let npc = &spec.npcs()[0];

        let ego_id = ego.id.as_str();
        let npc_id = npc.id.as_str();

        // Initial conditions
        let init = self.initial_conditions(spec, ego_id, npc_id);

        // Overtake behavior (three-phase)
        let behavior = self.overtake_behavior(spec, ego_id, npc_id);

        Ok(init.and(behavior))
    }

    fn add_z3_constraints(
        &self,
        spec: &ScenarioSpec,
        encoder: &crate::solver::Z3Encoder,
        backend: &dyn crate::solver::Z3Backend,
        horizon: usize,
    ) -> Result<()> {
        use z3::ast::{Bool, Int};

        let ego = spec.ego().map_err(|e| ScenarioGenError::InvalidSpec(e))?;
        let npc = &spec.npcs()[0];
        let original_lane = ego.lane;
        let passing_lane = ego.lane - 1; // Left lane (lower number)
        let npc_id = &npc.id;
        let ego_id = &ego.id;

        // Parse timing parameters
        let start_time_json = npc.behavior.get("overtake_start_time").ok_or_else(|| {
            ScenarioGenError::InvalidSpec("NPC missing 'overtake_start_time'".to_string())
        })?;
        let end_time_json = npc.behavior.get("overtake_end_time").ok_or_else(|| {
            ScenarioGenError::InvalidSpec("NPC missing 'overtake_end_time'".to_string())
        })?;

        let start_time: ValueOrRange =
            serde_json::from_value(start_time_json.clone()).map_err(|e| {
                ScenarioGenError::InvalidSpec(format!("Failed to parse overtake_start_time: {}", e))
            })?;
        let end_time: ValueOrRange =
            serde_json::from_value(end_time_json.clone()).map_err(|e| {
                ScenarioGenError::InvalidSpec(format!("Failed to parse overtake_end_time: {}", e))
            })?;

        let time_step = spec.time_step;

        // Convert times to step indices
        let start_min_step = (start_time.min() / time_step).ceil() as usize;
        let start_max_step = (start_time.max() / time_step).floor() as usize;
        let end_min_step = (end_time.min() / time_step).ceil() as usize;
        let end_max_step = (end_time.max() / time_step).floor() as usize;

        let original_val = Int::from_i64(original_lane as i64);
        let passing_val = Int::from_i64(passing_lane as i64);

        // ===== PHASE 1: Before overtake_start_time.min =====
        // NPC must be in original lane (same as ego)
        for t in 0..start_min_step.saturating_sub(1) {
            let lane_t = encoder.get_lane_var(npc_id, t);
            backend.assert(&lane_t.eq(&original_val));
        }

        // ===== PHASE 2: Between start_max and end_min =====
        // NPC must be in passing lane (guaranteed passing window)
        for t in start_max_step..=end_min_step.min(horizon) {
            let lane_t = encoder.get_lane_var(npc_id, t);
            backend.assert(&lane_t.eq(&passing_val));
        }

        // ===== PHASE 3: After overtake_end_time.max =====
        // NPC must be back in original lane
        for t in end_max_step..=horizon {
            let lane_t = encoder.get_lane_var(npc_id, t);
            backend.assert(&lane_t.eq(&original_val));
        }

        // ===== CRITICAL: Position constraint - NPC must be ahead before returning =====
        // At return time window, if NPC is in original lane, it must be ahead of ego
        for t in end_min_step.saturating_sub(1)..=end_max_step.min(horizon) {
            let npc_px = encoder.get_longitudinal_pos(npc_id, t);
            let ego_px = encoder.get_longitudinal_pos(ego_id, t);
            let lane_t = encoder.get_lane_var(npc_id, t);

            // If NPC is returning to original lane at this step, it must be ahead
            let in_original = lane_t.eq(&original_val);
            let npc_ahead = npc_px.gt(ego_px);

            // in_original => npc_ahead
            backend.assert(&in_original.implies(&npc_ahead));
        }

        // ===== Lane transition constraints =====
        // Prevent oscillation: only allow transitions in correct direction

        for t in 0..horizon {
            let lane_t = encoder.get_lane_var(npc_id, t);
            let lane_t1 = encoder.get_lane_var(npc_id, t + 1);

            // Once in passing lane before end window, stay in passing lane
            if t < end_min_step.saturating_sub(1) {
                let in_passing = lane_t.eq(&passing_val);
                let stays_passing = lane_t1.eq(&passing_val);
                backend.assert(&in_passing.implies(&stays_passing));
            }

            // Once back in original lane after start window, stay in original lane
            if t >= start_max_step {
                let in_original = lane_t.eq(&original_val);
                let stays_original = lane_t1.eq(&original_val);
                backend.assert(&in_original.implies(&stays_original));
            }
        }

        // ===== Initial position constraint: NPC behind ego =====
        let npc_px_0 = encoder.get_longitudinal_pos(npc_id, 0);
        let ego_px_0 = encoder.get_longitudinal_pos(ego_id, 0);
        backend.assert(&npc_px_0.lt(ego_px_0));

        // ===== Lane restriction: NPC can only be in original or passing lane =====
        // This prevents the NPC from using arbitrary lanes during transition windows
        for t in 0..=horizon {
            let lane_t = encoder.get_lane_var(npc_id, t);
            let in_original = lane_t.eq(&original_val);
            let in_passing = lane_t.eq(&passing_val);
            backend.assert(&Bool::or(&[&in_original, &in_passing]));
        }

        Ok(())
    }
}

impl OvertakeLeftModel {
    /// Generate initial conditions LTL
    fn initial_conditions(&self, spec: &ScenarioSpec, ego_id: &str, npc_id: &str) -> LTLFormula {
        let ego = spec.ego().unwrap();
        let npc = &spec.npcs()[0];

        // Both start in the same lane
        let ego_lane = LTLFormula::Atom(Proposition::InLane {
            actor: ego_id.to_string(),
            lane: ego.lane,
        });

        let npc_lane = LTLFormula::Atom(Proposition::InLane {
            actor: npc_id.to_string(),
            lane: npc.lane,
        });

        // NPC is behind ego initially (ego is ahead of NPC)
        let ego_ahead = LTLFormula::Atom(Proposition::Ahead {
            actor1: ego_id.to_string(),
            actor2: npc_id.to_string(),
        });

        ego_lane.and(npc_lane).and(ego_ahead)
    }

    /// Generate three-phase overtake behavior LTL
    fn overtake_behavior(&self, spec: &ScenarioSpec, ego_id: &str, npc_id: &str) -> LTLFormula {
        let ego = spec.ego().unwrap();
        let original_lane = ego.lane;
        let passing_lane = ego.lane - 1; // Left lane

        // Phase 1: NPC in original lane
        let in_original = LTLFormula::Atom(Proposition::InLane {
            actor: npc_id.to_string(),
            lane: original_lane,
        });

        // Phase 2: NPC in passing lane
        let in_passing = LTLFormula::Atom(Proposition::InLane {
            actor: npc_id.to_string(),
            lane: passing_lane,
        });

        // Phase 3: NPC ahead of ego (for the return condition)
        let npc_ahead = LTLFormula::Atom(Proposition::Ahead {
            actor1: npc_id.to_string(),
            actor2: ego_id.to_string(),
        });

        // The overtake behavior:
        // 1. Stay in original lane UNTIL entering passing lane
        // 2. Eventually return to original lane AND be ahead of ego
        let phase_1_to_2 = in_original.clone().until(in_passing);
        let return_ahead = in_original.and(npc_ahead).eventually();

        phase_1_to_2.and(return_ahead)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dsl::types::{
        ActorRole, ActorSpec, ConstraintModes, OptimizationTarget, ScenarioType, ValueOrRange,
    };
    use std::collections::HashMap;

    fn create_test_spec() -> ScenarioSpec {
        let ego_behavior = HashMap::new();
        let mut npc_behavior = HashMap::new();
        npc_behavior.insert(
            "overtake_start_time".to_string(),
            serde_json::json!([2.0, 3.0]),
        );
        npc_behavior.insert(
            "overtake_end_time".to_string(),
            serde_json::json!([6.0, 8.0]),
        );

        ScenarioSpec {
            scenario_type: ScenarioType::OvertakeLeft,
            time_step: 0.5,
            duration: 10.0,
            actors: vec![
                ActorSpec {
                    id: "ego".to_string(),
                    role: ActorRole::Ego,
                    lane: 1, // Ego in right lane
                    position: ValueOrRange::Value(50.0),
                    speed: ValueOrRange::Value(15.0),
                    acceleration: ValueOrRange::Range([-3.0, 2.0]),
                    direction: 1,
                    behavior: ego_behavior,
                    lane_change: None,
                    bicycle_params: None,
                },
                ActorSpec {
                    id: "npc".to_string(),
                    role: ActorRole::Npc,
                    lane: 1, // NPC starts in SAME lane as ego
                    position: ValueOrRange::Range([30.0, 40.0]), // Behind ego
                    speed: ValueOrRange::Range([18.0, 22.0]), // Faster than ego
                    acceleration: ValueOrRange::Range([-3.0, 4.0]),
                    direction: 1,
                    behavior: npc_behavior,
                    lane_change: None,
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
            coordinate_system: crate::dsl::types::CoordinateSystem::Cartesian,
            bicycle_config: None,
        }
    }

    #[test]
    fn test_overtake_left_validate_success() {
        let model = OvertakeLeftModel;
        let spec = create_test_spec();
        assert!(model.validate(&spec).is_ok());
    }

    #[test]
    fn test_overtake_left_validate_missing_start_time() {
        let model = OvertakeLeftModel;
        let mut spec = create_test_spec();
        spec.actors[1].behavior.remove("overtake_start_time");
        assert!(model.validate(&spec).is_err());
    }

    #[test]
    fn test_overtake_left_validate_missing_end_time() {
        let model = OvertakeLeftModel;
        let mut spec = create_test_spec();
        spec.actors[1].behavior.remove("overtake_end_time");
        assert!(model.validate(&spec).is_err());
    }

    #[test]
    fn test_overtake_left_validate_invalid_timing() {
        let model = OvertakeLeftModel;
        let mut spec = create_test_spec();
        // Set start_time.max > end_time.min (overlapping windows)
        spec.actors[1].behavior.insert(
            "overtake_start_time".to_string(),
            serde_json::json!([5.0, 7.0]),
        );
        spec.actors[1].behavior.insert(
            "overtake_end_time".to_string(),
            serde_json::json!([6.0, 8.0]),
        );
        assert!(model.validate(&spec).is_err());
    }

    #[test]
    fn test_overtake_left_validate_wrong_lane() {
        let model = OvertakeLeftModel;
        let mut spec = create_test_spec();
        spec.actors[1].lane = 0; // NPC in different lane than ego
        assert!(model.validate(&spec).is_err());
    }

    #[test]
    fn test_overtake_left_validate_no_left_lane() {
        let model = OvertakeLeftModel;
        let mut spec = create_test_spec();
        spec.actors[0].lane = 0; // Ego in leftmost lane
        spec.actors[1].lane = 0; // NPC also in leftmost lane
        assert!(model.validate(&spec).is_err());
    }

    #[test]
    fn test_overtake_left_generate_ltl() {
        let model = OvertakeLeftModel;
        let spec = create_test_spec();
        let formula = model.generate_ltl(&spec);
        assert!(formula.is_ok());

        let formula_str = format!("{}", formula.unwrap());
        assert!(formula_str.contains("InLane"));
        assert!(formula_str.contains("Ahead"));
    }
}
