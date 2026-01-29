//! Overtake from left scenario model
//!
//! In this scenario, an NPC vehicle starts behind the ego vehicle in the same lane,
//! moves to the left lane (passing lane), accelerates to pass the ego, then returns
//! to the original lane ahead of the ego vehicle.
//!
//! The overtake is expressed as two sequential lane changes:
//! 1. Left lane change (into passing lane)
//! 2. Right lane change (back to original lane)

use crate::dsl::types::{LaneChangeDirection, ScenarioSpec};
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

        // Validate lane_changes configuration (two lane changes: left then right)
        if npc.lane_changes.len() != 2 {
            return Err(ScenarioGenError::InvalidSpec(format!(
                "Overtake-left requires exactly 2 lane_changes for NPC (left, right), found {}",
                npc.lane_changes.len()
            )));
        }

        // First lane change must be left (into passing lane)
        if npc.lane_changes[0].direction != LaneChangeDirection::Left {
            return Err(ScenarioGenError::InvalidSpec(
                "Overtake-left: first lane change must be 'left' (into passing lane)".to_string(),
            ));
        }

        // Second lane change must be right (back to original lane)
        if npc.lane_changes[1].direction != LaneChangeDirection::Right {
            return Err(ScenarioGenError::InvalidSpec(
                "Overtake-left: second lane change must be 'right' (back to original lane)"
                    .to_string(),
            ));
        }

        // Validate timing: first lane change must end before second starts
        let first_end_max =
            npc.lane_changes[0].start_time.max() + npc.lane_changes[0].duration.max();
        let second_start_min = npc.lane_changes[1].start_time.min();
        if first_end_max >= second_start_min {
            return Err(ScenarioGenError::InvalidSpec(format!(
                "Lane changes must not overlap: first ends at max {} but second starts at min {}",
                first_end_max, second_start_min
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
        _spec: &ScenarioSpec,
        _encoder: &crate::solver::Z3Encoder,
        _backend: &dyn crate::solver::Z3Backend,
        _horizon: usize,
    ) -> Result<()> {
        // Lane change constraints are now handled by the encoder
        // based on lane_changes config in the actor spec.
        // The two sequential lane changes (left then right) express
        // the overtake behavior declaratively.
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
        ActorRole, ActorSpec, ConstraintModes, LaneChangeConfig, LaneChangeDirection,
        OptimizationTarget, ScenarioType, ValueOrRange,
    };
    use std::collections::HashMap;

    fn create_test_spec() -> ScenarioSpec {
        let ego_behavior = HashMap::new();

        // Two lane changes for overtake: left (into passing lane), then right (back)
        let npc_lane_changes = vec![
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
        ];

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
                    lane_changes: vec![],
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
                    behavior: HashMap::new(),
                    lane_changes: npc_lane_changes,
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
    fn test_overtake_left_validate_missing_lane_changes() {
        let model = OvertakeLeftModel;
        let mut spec = create_test_spec();
        spec.actors[1].lane_changes = vec![]; // No lane changes
        assert!(model.validate(&spec).is_err());
    }

    #[test]
    fn test_overtake_left_validate_single_lane_change() {
        let model = OvertakeLeftModel;
        let mut spec = create_test_spec();
        // Only one lane change (should be two)
        spec.actors[1].lane_changes = vec![LaneChangeConfig {
            direction: LaneChangeDirection::Left,
            start_time: ValueOrRange::Range([2.0, 3.0]),
            duration: ValueOrRange::Range([1.5, 2.0]),
        }];
        assert!(model.validate(&spec).is_err());
    }

    #[test]
    fn test_overtake_left_validate_wrong_direction_order() {
        let model = OvertakeLeftModel;
        let mut spec = create_test_spec();
        // Wrong order: right then left (should be left then right)
        spec.actors[1].lane_changes = vec![
            LaneChangeConfig {
                direction: LaneChangeDirection::Right,
                start_time: ValueOrRange::Range([2.0, 3.0]),
                duration: ValueOrRange::Range([1.5, 2.0]),
            },
            LaneChangeConfig {
                direction: LaneChangeDirection::Left,
                start_time: ValueOrRange::Range([6.0, 7.0]),
                duration: ValueOrRange::Range([1.5, 2.0]),
            },
        ];
        assert!(model.validate(&spec).is_err());
    }

    #[test]
    fn test_overtake_left_validate_overlapping_lane_changes() {
        let model = OvertakeLeftModel;
        let mut spec = create_test_spec();
        // Overlapping: first ends at 5.0 max but second starts at 4.0 min
        spec.actors[1].lane_changes = vec![
            LaneChangeConfig {
                direction: LaneChangeDirection::Left,
                start_time: ValueOrRange::Range([2.0, 3.0]),
                duration: ValueOrRange::Range([1.5, 2.0]),
            },
            LaneChangeConfig {
                direction: LaneChangeDirection::Right,
                start_time: ValueOrRange::Range([4.0, 5.0]), // Overlaps with first
                duration: ValueOrRange::Range([1.5, 2.0]),
            },
        ];
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
