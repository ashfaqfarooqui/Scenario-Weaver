//! Head-on overtake scenario model
//!
//! In this scenario, the ego vehicle overtakes a slower vehicle on a two-lane
//! bidirectional road by temporarily entering the oncoming traffic lane. An
//! oncoming vehicle in that lane creates a head-on collision risk.
//!
//! The overtake is expressed as two sequential lane changes on the ego:
//! 1. Right lane change (into oncoming lane at higher index)
//! 2. Left lane change (back to original lane)
//!
//! Actors:
//!   - ego: overtaker, starts in forward lane, performs 2 lane changes
//!   - slow NPC: same lane as ego, same direction, ahead of ego
//!   - oncoming NPC: in the passing lane, opposite direction

use crate::dsl::types::{ConstraintMode, LaneChangeDirection, ScenarioSpec};
use crate::error::{Result, ScenarioGenError};
use crate::ltl::formula::{LTLFormula, Proposition};
use crate::scenarios::ScenarioModel;

/// Head-on overtake scenario model
pub(crate) struct HeadOnModel;

impl ScenarioModel for HeadOnModel {
    fn validate(&self, spec: &ScenarioSpec) -> Result<()> {
        // Require exactly 3 actors (ego + 2 NPCs)
        if spec.actors.len() != 3 {
            return Err(ScenarioGenError::InvalidSpec(format!(
                "Head-on scenario requires exactly 3 actors (ego + 2 NPCs), found {}",
                spec.actors.len()
            )));
        }

        let ego = spec.ego().map_err(ScenarioGenError::InvalidSpec)?;
        let npcs = spec.npcs();

        if npcs.len() != 2 {
            return Err(ScenarioGenError::InvalidSpec(format!(
                "Head-on scenario requires exactly 2 NPCs, found {}",
                npcs.len()
            )));
        }

        // Ego must have exactly 2 lane changes: right then left
        if ego.lane_changes.len() != 2 {
            return Err(ScenarioGenError::InvalidSpec(format!(
                "Head-on scenario requires exactly 2 lane_changes on ego (right, left), found {}",
                ego.lane_changes.len()
            )));
        }

        if ego.lane_changes[0].direction != LaneChangeDirection::Right {
            return Err(ScenarioGenError::InvalidSpec(
                "Head-on scenario: ego's first lane change must be 'right' (into oncoming lane)"
                    .to_string(),
            ));
        }

        if ego.lane_changes[1].direction != LaneChangeDirection::Left {
            return Err(ScenarioGenError::InvalidSpec(
                "Head-on scenario: ego's second lane change must be 'left' (return to own lane)"
                    .to_string(),
            ));
        }

        // Passing lane must exist (ego.lane + 1 must be valid)
        let num_lanes = spec.road.as_ref().map_or(2, |r| r.num_lanes);
        if ego.lane + 1 >= num_lanes {
            return Err(ScenarioGenError::InvalidSpec(format!(
                "Head-on scenario: no lane to the right of ego (lane {}). Need at least {} lanes.",
                ego.lane,
                ego.lane + 2
            )));
        }

        // Find the slow NPC (same lane, same direction as ego)
        let slow_npc = npcs
            .iter()
            .find(|n| n.lane == ego.lane && n.direction == ego.direction);

        if slow_npc.is_none() {
            return Err(ScenarioGenError::InvalidSpec(
                "Head-on scenario: one NPC must be in the same lane and direction as ego (slow vehicle)"
                    .to_string(),
            ));
        }

        // Find the oncoming NPC (in the passing lane, opposite direction)
        let passing_lane = ego.lane + 1;
        let oncoming_npc = npcs
            .iter()
            .find(|n| n.lane == passing_lane && n.direction != ego.direction);

        if oncoming_npc.is_none() {
            return Err(ScenarioGenError::InvalidSpec(format!(
                "Head-on scenario: one NPC must be in lane {} (passing lane) with opposite direction",
                passing_lane
            )));
        }

        // Validate timing: first lane change must end before second starts
        let first_end_max =
            ego.lane_changes[0].start_time.max() + ego.lane_changes[0].duration.max();
        let second_start_min = ego.lane_changes[1].start_time.min();
        if first_end_max > second_start_min {
            return Err(ScenarioGenError::InvalidSpec(format!(
                "Ego lane changes must not overlap: first ends at max {} but second starts at min {}",
                first_end_max, second_start_min
            )));
        }

        Ok(())
    }

    fn generate_ltl(&self, spec: &ScenarioSpec) -> Result<LTLFormula> {
        let ego = spec.ego().map_err(ScenarioGenError::InvalidSpec)?;

        // Minimal behavioral LTL: just a tautology.
        // The lane changes and initial positions are fully handled by the
        // kinematics encoder from the YAML config. No extra LTL needed.
        let tautology = LTLFormula::Atom(Proposition::InLane {
            actor: ego.id.clone(),
            lane: ego.lane,
        })
        .or(LTLFormula::Atom(Proposition::InLane {
            actor: ego.id.clone(),
            lane: ego.lane,
        })
        .negate());

        Ok(tautology)
    }

    /// Custom safety generation for head-on scenario.
    ///
    /// Only applies TTC/distance constraints to the ego↔oncoming pair.
    /// Other pairs are left unconstrained — the kinematics (positions, speeds)
    /// are already set by the encoder from the YAML config.
    fn generate_safety(&self, spec: &ScenarioSpec) -> Result<LTLFormula> {
        let ego = spec.ego().map_err(ScenarioGenError::InvalidSpec)?;
        let npcs = spec.npcs();

        let ego_id = &ego.id;
        let passing_lane = ego.lane + 1;

        // Identify the oncoming NPC
        let oncoming_npc = npcs
            .iter()
            .find(|n| n.lane == passing_lane && n.direction != ego.direction)
            .ok_or_else(|| ScenarioGenError::InvalidSpec(
                "Head-on scenario requires an oncoming actor (direction=-1)".to_string()
            ))?;
        let oncoming_id = &oncoming_npc.id;

        let mut constraints = Vec::new();

        let ttc_mode = spec.constraint_modes.min_ttc();
        let dist_mode = spec.constraint_modes.min_distance();

        // Only ego ↔ oncoming gets the requested constraint mode
        Self::add_pair_constraints(
            &mut constraints,
            ego_id,
            oncoming_id,
            spec.min_ttc,
            spec.min_distance,
            ttc_mode,
            dist_mode,
        );

        if constraints.is_empty() {
            // Tautology
            Ok(LTLFormula::Atom(Proposition::InLane {
                actor: ego_id.clone(),
                lane: ego.lane,
            })
            .or(LTLFormula::Atom(Proposition::InLane {
                actor: ego_id.clone(),
                lane: ego.lane,
            })
            .negate()))
        } else {
            // constraints is non-empty here due to the is_empty() check above
            Ok(LTLFormula::conjunction(constraints))
        }
    }

    fn add_z3_constraints(
        &self,
        _spec: &ScenarioSpec,
        _encoder: &crate::solver::Z3Encoder,
        _backend: &dyn crate::solver::Z3Backend,
        _horizon: usize,
    ) -> Result<()> {
        // Lane change constraints are handled by the encoder based on
        // lane_changes config in the actor spec.
        Ok(())
    }
}

impl HeadOnModel {
    /// Add TTC and distance constraints for a single actor pair
    fn add_pair_constraints(
        constraints: &mut Vec<LTLFormula>,
        actor1: &str,
        actor2: &str,
        min_ttc: f64,
        min_distance: f64,
        ttc_mode: ConstraintMode,
        dist_mode: ConstraintMode,
    ) {
        match ttc_mode {
            ConstraintMode::Enforce => {
                constraints.push(
                    LTLFormula::Atom(Proposition::TTCGT {
                        actor1: actor1.to_string(),
                        actor2: actor2.to_string(),
                        ttc: min_ttc,
                    })
                    .always(),
                );
            }
            ConstraintMode::Violate => {
                constraints.push(
                    LTLFormula::Atom(Proposition::TTCGT {
                        actor1: actor1.to_string(),
                        actor2: actor2.to_string(),
                        ttc: min_ttc,
                    })
                    .negate()
                    .eventually(),
                );
            }
            ConstraintMode::Ignore => {}
        }

        match dist_mode {
            ConstraintMode::Enforce => {
                constraints.push(
                    LTLFormula::Atom(Proposition::DistanceGT {
                        actor1: actor1.to_string(),
                        actor2: actor2.to_string(),
                        distance: min_distance,
                    })
                    .always(),
                );
            }
            ConstraintMode::Violate => {
                constraints.push(
                    LTLFormula::Atom(Proposition::DistanceGT {
                        actor1: actor1.to_string(),
                        actor2: actor2.to_string(),
                        distance: min_distance,
                    })
                    .negate()
                    .eventually(),
                );
            }
            ConstraintMode::Ignore => {}
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dsl::types::{
        ActorRole, ActorSpec, ConstraintModes, LaneChangeConfig, LaneChangeDirection,
        OptimizationTarget, RoadSpec, ScenarioType, ValueOrRange,
    };
    use std::collections::HashMap;

    fn create_test_spec() -> ScenarioSpec {
        let ego_lane_changes = vec![
            LaneChangeConfig {
                direction: LaneChangeDirection::Right,
                start_time: ValueOrRange::Range([2.0, 3.0]),
                duration: ValueOrRange::Range([2.0, 3.0]),
            },
            LaneChangeConfig {
                direction: LaneChangeDirection::Left,
                start_time: ValueOrRange::Range([6.0, 7.0]),
                duration: ValueOrRange::Range([2.0, 3.0]),
            },
        ];

        ScenarioSpec {
            scenario_type: ScenarioType::HeadOn,
            time_step: 0.5,
            duration: 10.0,
            actors: vec![
                ActorSpec {
                    id: "ego".to_string(),
                    role: ActorRole::Ego,
                    lane: 0,
                    position: ValueOrRange::Value(0.0),
                    speed: ValueOrRange::Value(15.0),
                    acceleration: ValueOrRange::Range([-5.0, 3.0]),
                    direction: 1,
                    behavior: HashMap::new(),
                    lane_changes: ego_lane_changes,
                    bicycle_params: None,
                },
                ActorSpec {
                    id: "slow_npc".to_string(),
                    role: ActorRole::Npc,
                    lane: 0, // Same lane as ego
                    position: ValueOrRange::Range([40.0, 60.0]),
                    speed: ValueOrRange::Range([8.0, 10.0]),
                    acceleration: ValueOrRange::Range([-3.0, 1.0]),
                    direction: 1, // Same direction as ego
                    behavior: HashMap::new(),
                    lane_changes: vec![],
                    bicycle_params: None,
                },
                ActorSpec {
                    id: "oncoming_npc".to_string(),
                    role: ActorRole::Npc,
                    lane: 1, // Passing lane (oncoming)
                    position: ValueOrRange::Range([150.0, 180.0]),
                    speed: ValueOrRange::Range([12.0, 15.0]),
                    acceleration: ValueOrRange::Range([-2.0, 1.0]),
                    direction: -1, // Opposite direction
                    behavior: HashMap::new(),
                    lane_changes: vec![],
                    bicycle_params: None,
                },
            ],
            min_ttc: 2.0,
            min_distance: 5.0,
            road: Some(RoadSpec {
                num_lanes: 2,
                lane_width: 3.5,
                lane_directions: vec![1, -1],
                road_length: None,
            }),
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
    fn test_head_on_validate_success() {
        let model = HeadOnModel;
        let spec = create_test_spec();
        let result = model.validate(&spec);
        assert!(result.is_ok(), "Validation failed: {:?}", result.err());
    }

    #[test]
    fn test_head_on_validate_wrong_actor_count() {
        let model = HeadOnModel;
        let mut spec = create_test_spec();
        spec.actors.pop(); // Remove one NPC
        assert!(model.validate(&spec).is_err());
    }

    #[test]
    fn test_head_on_validate_no_ego_lane_changes() {
        let model = HeadOnModel;
        let mut spec = create_test_spec();
        spec.actors[0].lane_changes = vec![];
        assert!(model.validate(&spec).is_err());
    }

    #[test]
    fn test_head_on_validate_wrong_direction_order() {
        let model = HeadOnModel;
        let mut spec = create_test_spec();
        // Swap: left then right (should be right then left)
        spec.actors[0].lane_changes = vec![
            LaneChangeConfig {
                direction: LaneChangeDirection::Left,
                start_time: ValueOrRange::Range([2.0, 3.0]),
                duration: ValueOrRange::Range([2.0, 3.0]),
            },
            LaneChangeConfig {
                direction: LaneChangeDirection::Right,
                start_time: ValueOrRange::Range([6.0, 7.0]),
                duration: ValueOrRange::Range([2.0, 3.0]),
            },
        ];
        assert!(model.validate(&spec).is_err());
    }

    #[test]
    fn test_head_on_validate_overlapping_lane_changes() {
        let model = HeadOnModel;
        let mut spec = create_test_spec();
        // Overlapping: first ends at 6.0 max but second starts at 5.0 min
        spec.actors[0].lane_changes = vec![
            LaneChangeConfig {
                direction: LaneChangeDirection::Right,
                start_time: ValueOrRange::Range([2.0, 3.0]),
                duration: ValueOrRange::Range([2.0, 3.0]),
            },
            LaneChangeConfig {
                direction: LaneChangeDirection::Left,
                start_time: ValueOrRange::Range([5.0, 6.0]),
                duration: ValueOrRange::Range([2.0, 3.0]),
            },
        ];
        assert!(model.validate(&spec).is_err());
    }

    #[test]
    fn test_head_on_validate_no_slow_npc() {
        let model = HeadOnModel;
        let mut spec = create_test_spec();
        // Move the slow NPC to a different lane
        spec.actors[1].lane = 1;
        spec.actors[1].direction = -1;
        assert!(model.validate(&spec).is_err());
    }

    #[test]
    fn test_head_on_validate_no_oncoming_npc() {
        let model = HeadOnModel;
        let mut spec = create_test_spec();
        // Make the oncoming NPC same direction as ego
        spec.actors[2].direction = 1;
        assert!(model.validate(&spec).is_err());
    }

    #[test]
    fn test_head_on_generate_ltl() {
        let model = HeadOnModel;
        let spec = create_test_spec();
        let formula = model.generate_ltl(&spec);
        assert!(formula.is_ok());

        // LTL is a tautology (kinematics handle everything); just verify it generates
        let formula_str = format!("{}", formula.unwrap());
        assert!(formula_str.contains("InLane")); // Tautology uses InLane
    }
}
