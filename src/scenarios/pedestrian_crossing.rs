//! Pedestrian crossing scenario model
//!
//! Simplified model: A pedestrian crosses perpendicular to the ego vehicle's path.
//! The pedestrian starts at an initial lateral position and moves to cross the road.
//! No complex sidewalk or timing constraints - just basic crossing behavior.

use crate::dsl::types::ScenarioSpec;
use crate::error::{Result, ScenarioGenError};
use crate::ltl::formula::{LTLFormula, Proposition};
use crate::scenarios::ScenarioModel;

/// Pedestrian crossing scenario model
pub struct PedestrianCrossingModel;

impl ScenarioModel for PedestrianCrossingModel {
    fn validate(&self, spec: &ScenarioSpec) -> Result<()> {
        use crate::dsl::types::ActorRole;

        // Validate exactly 2 actors (1 ego vehicle, 1 pedestrian)
        if spec.actors.len() != 2 {
            return Err(ScenarioGenError::InvalidSpec(format!(
                "Pedestrian crossing requires exactly 2 actors, found {}",
                spec.actors.len()
            )));
        }

        // Validate roles
        let _ego = spec.ego().map_err(|e| ScenarioGenError::InvalidSpec(e))?;
        let pedestrian = &spec.npcs()[0];

        if pedestrian.role != ActorRole::Pedestrian {
            return Err(ScenarioGenError::InvalidSpec(format!(
                "Second actor must be pedestrian, found {:?}",
                pedestrian.role
            )));
        }

        // Validate direction field exists and is valid
        let direction = pedestrian
            .behavior
            .get("direction")
            .and_then(|v| v.as_str())
            .ok_or_else(|| {
                ScenarioGenError::InvalidSpec(
                    "Pedestrian missing 'direction' in behavior".to_string(),
                )
            })?;

        match direction {
            "left_to_right" | "right_to_left" => Ok(()),
            _ => {
                return Err(ScenarioGenError::InvalidSpec(format!(
                    "Invalid direction '{}': must be 'left_to_right' or 'right_to_left'",
                    direction
                )))
            }
        }
    }

    fn generate_safety(&self, spec: &ScenarioSpec) -> Result<LTLFormula> {
        use crate::dsl::types::{ActorRole, ConstraintMode};

        let ego = spec.ego().map_err(|e| ScenarioGenError::InvalidSpec(e))?;
        let npcs = spec.npcs();
        let pedestrian = npcs
            .iter()
            .find(|a| a.role == ActorRole::Pedestrian)
            .ok_or_else(|| ScenarioGenError::InvalidSpec("No pedestrian found".to_string()))?;

        let mut constraints = Vec::new();

        // Rectangular safety box (simplest linear constraint, very fast Z3 solving)
        // For perpendicular crossing: lateral distance is more critical than longitudinal
        // Using threshold/1.5 gives conservative safety (~1.3m for 2m threshold)
        if spec.constraint_modes.min_distance() == ConstraintMode::Enforce {
            let dist = LTLFormula::Atom(Proposition::RectangularDistanceGT {
                actor1: ego.id.clone(),
                actor2: pedestrian.id.clone(),
                threshold_x: spec.min_distance / 2.0, // Longitudinal: half the threshold
                threshold_y: spec.min_distance / 1.5, // Lateral: slightly more conservative
            })
            .always();
            constraints.push(dist);
        }

        // Pedestrian-specific TTC (perpendicular crossing)
        if spec.constraint_modes.min_ttc() == ConstraintMode::Enforce {
            let ttc = LTLFormula::Atom(Proposition::PedestrianTTCGT {
                ego: ego.id.clone(),
                pedestrian: pedestrian.id.clone(),
                ttc: spec.min_ttc,
            })
            .always();
            constraints.push(ttc);
        }

        if constraints.is_empty() {
            // Return tautology
            Ok(LTLFormula::Atom(Proposition::InLane {
                actor: ego.id.clone(),
                lane: ego.lane,
            })
            .or(LTLFormula::Atom(Proposition::InLane {
                actor: ego.id.clone(),
                lane: ego.lane,
            })
            .negate()))
        } else {
            Ok(constraints.into_iter().reduce(|acc, c| acc.and(c)).unwrap())
        }
    }

    fn generate_ltl(&self, spec: &ScenarioSpec) -> Result<LTLFormula> {
        use crate::dsl::types::ActorRole;

        let ego = spec.ego().map_err(|e| ScenarioGenError::InvalidSpec(e))?;
        let ego_id = ego.id.as_str();

        // Ego stays in its lane
        let ego_in_lane = LTLFormula::Atom(Proposition::InLane {
            actor: ego_id.to_string(),
            lane: ego.lane,
        });

        // Get pedestrian
        let npcs = spec.npcs();
        let pedestrian = npcs
            .iter()
            .find(|a| a.role == ActorRole::Pedestrian)
            .ok_or_else(|| {
                ScenarioGenError::InvalidSpec("No pedestrian found in spec".to_string())
            })?;
        let ped_id = &pedestrian.id;

        // Determine crossing direction from behavior field
        let direction = pedestrian
            .behavior
            .get("direction")
            .and_then(|v| v.as_str())
            .ok_or_else(|| {
                ScenarioGenError::InvalidSpec(
                    "Pedestrian missing 'direction' in behavior".to_string(),
                )
            })?;

        let opposite_side = match direction {
            "left_to_right" => "right",
            "right_to_left" => "left",
            _ => {
                return Err(ScenarioGenError::InvalidSpec(format!(
                    "Invalid direction '{}': must be 'left_to_right' or 'right_to_left'",
                    direction
                )))
            }
        };

        // Multi-stage crossing using sequential implications
        // Stage 1: Eventually starts crossing (enters road)
        let crossing_road = LTLFormula::Atom(Proposition::CrossingRoad {
            actor: ped_id.clone(),
        });

        // Stage 2: Eventually reaches opposite sidewalk
        let on_opposite_sidewalk = LTLFormula::Atom(Proposition::OnSidewalk {
            actor: ped_id.clone(),
            side: opposite_side.to_string(),
        });

        // Combine: Eventually cross road AND eventually reach opposite side
        // This allows: start on initial → move to road → move to opposite
        let enters_road = crossing_road.clone().eventually();
        let reaches_opposite = on_opposite_sidewalk.eventually();

        let full_crossing = enters_road.and(reaches_opposite);

        // Combine ego constraint with pedestrian crossing behavior
        Ok(ego_in_lane.and(full_crossing))
    }

    fn add_z3_constraints(
        &self,
        spec: &ScenarioSpec,
        encoder: &crate::solver::Z3Encoder,
        backend: &dyn crate::solver::Z3Backend,
        horizon: usize,
    ) -> Result<()> {
        use crate::dsl::ActorRole;
        use z3::ast::{Int, Real};

        // Get pedestrian actor
        let npcs = spec.npcs();
        let pedestrian = npcs
            .iter()
            .find(|a| a.role == ActorRole::Pedestrian)
            .ok_or_else(|| {
                ScenarioGenError::InvalidSpec("No pedestrian actor found".to_string())
            })?;

        let pedestrian_id = &pedestrian.id;

        // Fix pedestrian lane to 0 throughout the scenario
        // The lane field has no semantic meaning for pedestrians - only lateral
        // position (py) matters for crossing detection. Fixing it to 0 prevents
        // Z3 from generating spurious lane values (e.g., 2, 3, 4).
        let zero_lane = Int::from_i64(0_i64);
        for t in 0..=horizon {
            let lane_t = encoder.get_lane_var(pedestrian_id, t);
            backend.assert(&lane_t.eq(&zero_lane));
        }

        // Check if pedestrian has "hesitate" walking mode

        if let Some(walking_mode) = pedestrian.behavior.get("walking_mode") {
            if walking_mode == "hesitate" {
                let pedestrian_id = &pedestrian.id;

                // For hesitate mode: Force pedestrian to slow down significantly
                // at some point during the middle of the scenario (e.g., 40%-60% through)
                let start_hesitate = (horizon as f64 * 0.4) as usize;
                let end_hesitate = (horizon as f64 * 0.6) as usize;

                // At least one time step in this range should have very low speed
                // Create a disjunction: at least one time step has speed < 0.2 m/s
                let slow_threshold_sq = 0.04; // (0.2 m/s)^2
                let threshold_real =
                    Real::from_rational((slow_threshold_sq * 100.0) as i64, 100_i64);

                let mut slow_constraints = vec![];
                for t in start_hesitate..end_hesitate {
                    let vx_t = encoder.get_longitudinal_vel(pedestrian_id, t);
                    let vy_t = encoder.get_lateral_vel(pedestrian_id, t);
                    let vx_sq = vx_t * vx_t;
                    let vy_sq = vy_t * vy_t;
                    let speed_sq = &vx_sq + &vy_sq;
                    // speed^2 < threshold
                    slow_constraints.push(speed_sq.lt(&threshold_real));
                }

                // At least one of these must be true (OR them together)
                if !slow_constraints.is_empty() {
                    let slow_constraint_refs: Vec<_> = slow_constraints.iter().collect();
                    let hesitate_constraint = z3::ast::Bool::or(&slow_constraint_refs);
                    backend.assert(&hesitate_constraint);
                }
            }
        }

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
        let mut pedestrian_behavior = HashMap::new();
        pedestrian_behavior.insert("direction".to_string(), serde_json::json!("left_to_right"));

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
                    direction: 1,
                    behavior: ego_behavior,
                    lane_change: None,
                    bicycle_params: None,
                },
                ActorSpec {
                    id: "pedestrian".to_string(),
                    role: ActorRole::Pedestrian,
                    lane: 0,
                    position: ValueOrRange::Value(50.0),
                    speed: ValueOrRange::Range([0.8, 1.5]),
                    acceleration: ValueOrRange::Range([-1.0, 1.0]),
                    direction: 1,
                    behavior: pedestrian_behavior,
                    lane_change: None,
                    bicycle_params: None,
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
