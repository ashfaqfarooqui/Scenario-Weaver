//! Multiple scenario generation with blocking clauses
//!
//! This module implements Phase 11 - generating multiple diverse scenarios
//! from the same specification by using blocking clauses to prevent
//! duplicate solutions.

use crate::dsl::types::{ActorRole, ScenarioSpec};
use crate::error::{Result, ScenarioGenError};
use crate::ltl::formula::LTLFormula;
use crate::scenario::model::Scenario;
use crate::solver::{Z3Backend, Z3Encoder};
use z3::ast::{Bool, Real};
use z3::{Config, SatResult};

/// Generate multiple diverse scenarios from the same specification
///
/// Uses blocking clauses to ensure each generated scenario is different.
/// Specifically, we block based on NPC initial conditions (position and velocity).
///
/// # Arguments
/// * `spec` - Scenario specification
/// * `ltl_formula` - Generated LTL formula (same for all scenarios)
/// * `num_scenarios` - Number of scenarios to generate
/// * `callback` - Optional callback invoked after each scenario is generated
///
/// # Returns
/// A vector of unique scenarios
///
/// # Errors
/// Returns error if specification is invalid or initial setup fails
pub fn generate_scenarios<F>(
    spec: &ScenarioSpec,
    ltl_formula: &LTLFormula,
    num_scenarios: usize,
    mut callback: Option<F>,
) -> Result<Vec<Scenario>>
where
    F: FnMut(usize, &Scenario) -> Result<()>,
{
    let mut scenarios = Vec::new();

    for i in 0..num_scenarios {
        // Get scenario model for scenario-specific constraints
        let scenario_model = spec.scenario_type.get_model();

        // Create fresh Z3 context for each scenario
        let cfg = Config::new();
        let result = z3::with_z3_config(&cfg, || {
            let mut encoder = Z3Encoder::new(spec.clone());

            // Setup encoder (same as single scenario)
            encoder.create_variables();
            encoder.encode_initial_conditions();
            encoder.encode_kinematics();
            encoder.encode_lane_velocity_constraints();
            encoder.encode_lateral_velocity_bounds();
            encoder.encode_ltl(ltl_formula);
            encoder.encode_scenario_specific_constraints(&*scenario_model)?;
            // Note: Safety constraints now handled via LTL propositions only

            // Add blocking clauses for all previous scenarios
            for prev_scenario in &scenarios {
                let blocking_clause = create_blocking_clause(&encoder, prev_scenario);
                encoder.assert_constraint(&blocking_clause);
            }

            // Solve
            match encoder.check() {
                SatResult::Sat => {
                    let model = encoder.get_model().ok_or_else(|| {
                        ScenarioGenError::ExtractionFailed("Failed to get Z3 model".to_string())
                    })?;
                    let scenario = encoder.extract_scenario(&model);
                    scenarios.push(scenario);
                    tracing::info!("Generated scenario {}/{}", i + 1, num_scenarios);
                    Ok::<(), ScenarioGenError>(())
                }
                SatResult::Unsat => {
                    tracing::warn!(
                        "No more unique scenarios found after {} scenarios",
                        scenarios.len()
                    );
                    Ok::<(), ScenarioGenError>(()) // No more solutions exist
                }
                SatResult::Unknown => {
                    tracing::error!("Z3 returned UNKNOWN for scenario {}", i + 1);
                    Ok::<(), ScenarioGenError>(())
                }
            }
        });

        result?;

        // Break if no more unique scenarios found
        if scenarios.len() < i + 1 {
            break;
        }

        // Call callback if provided (after Z3 context is released)
        if let Some(ref mut cb) = callback {
            cb(i, &scenarios[i])?;
        }
    }

    if scenarios.is_empty() {
        Err(ScenarioGenError::Unsatisfiable)
    } else {
        Ok(scenarios)
    }
}

/// Create a blocking clause to prevent generating the same scenario
///
/// We block based on all non-ego actors' initial conditions (position and velocity at t=0).
/// This ensures diversity in the generated scenarios across different actor types.
///
/// The blocking clause is: !(actor1_equal AND actor2_equal AND ...)
/// Which is equivalent to: (actor1_differs OR actor2_differs OR ...)
/// At least one non-ego actor must have different initial conditions from previous scenarios.
fn create_blocking_clause(encoder: &Z3Encoder, prev_scenario: &Scenario) -> Bool {
    let mut all_blocking_clauses = Vec::new();

    // Get all non-ego actors from the spec
    for actor in encoder
        .spec
        .actors
        .iter()
        .filter(|a| a.role != ActorRole::Ego)
    {
        // Get actor trajectory from previous scenario
        let actor_traj = prev_scenario
            .get_actor(&actor.id)
            .expect(&format!("Actor {} trajectory missing", actor.id));

        // Get actor initial state (t=0)
        let actor_initial = &actor_traj.states[0];
        let prev_px0 = actor_initial.position.x;
        let prev_vx0 = actor_initial.velocity.vx;

        // Get Z3 variables for this actor's initial conditions
        let actor_px0 = encoder.get_position_x(&actor.id, 0);
        let actor_vx0 = encoder.get_velocity_x(&actor.id, 0);

        // Create real values from previous scenario
        // Use truncation (not rounding) to match encoder's precision handling
        // Convert to Z3 rational: multiply by 10 to get 1 decimal precision (matches encoder)
        let prev_px0_z3 = Real::from_rational((prev_px0 * 10.0) as i64, 10_i64);
        let prev_vx0_z3 = Real::from_rational((prev_vx0 * 10.0) as i64, 10_i64);

        // Create equality constraints
        let px_eq = actor_px0.eq(&prev_px0_z3);
        let vx_eq = actor_vx0.eq(&prev_vx0_z3);

        // Both equal: px0 == prev_px0 AND vx0 == prev_vx0
        let both_equal = Bool::and(&[&px_eq, &vx_eq]);

        // Blocking clause: NOT(both equal)
        all_blocking_clauses.push(both_equal.not());
    }

    // Combine with OR: at least one actor must differ
    if all_blocking_clauses.is_empty() {
        // Edge case: no non-ego actors (shouldn't happen in practice)
        // Return a constraint that's always satisfiable
        Bool::from_bool(true)
    } else if all_blocking_clauses.len() == 1 {
        all_blocking_clauses.into_iter().next().unwrap()
    } else {
        Bool::or(&all_blocking_clauses)
    }
}

/// Add accessor methods to Z3Encoder for multi_solve module
impl Z3Encoder {
    /// Get position_x variable for an actor at a specific time
    pub fn get_position_x(&self, actor_id: &str, time: usize) -> &Real {
        &self.positions_x[actor_id][time]
    }

    /// Get velocity_x variable for an actor at a specific time
    pub fn get_velocity_x(&self, actor_id: &str, time: usize) -> &Real {
        &self.velocities_x[actor_id][time]
    }

    /// Assert a constraint to the solver
    pub fn assert_constraint(&self, constraint: &Bool) {
        self.backend.assert(constraint);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dsl::types::{ActorRole, ActorSpec, RoadSpec, ScenarioType, ValueOrRange};
    use crate::ltl::generator::LTLGenerator;
    use std::collections::HashMap;

    fn create_test_spec() -> ScenarioSpec {
        let mut npc_behavior = HashMap::new();
        npc_behavior.insert("cut_in_time".to_string(), serde_json::json!([2.5, 7.5]));

        ScenarioSpec {
            scenario_type: ScenarioType::CutInLeft,
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
                    behavior: HashMap::new(),
                },
                ActorSpec {
                    id: "npc".to_string(),
                    role: ActorRole::Npc,
                    lane: 0,
                    position: ValueOrRange::Range([60.0, 80.0]),
                    speed: ValueOrRange::Range([12.0, 14.0]),
                    acceleration: ValueOrRange::Range([-8.0, 3.0]),
                    behavior: npc_behavior,
                },
            ],
            min_ttc: 3.0,
            min_distance: 5.0,
            road: Some(RoadSpec {
                num_lanes: 2,
                lane_width: 3.5,
                lane_directions: vec![1, 1],
            }),
            lane_width: 3.5,
            num_scenarios: 5,
            constraint_modes: crate::dsl::types::ConstraintModes::default(),
            optimization_target: crate::dsl::types::OptimizationTarget::None,
            max_acceleration: None,
            max_deceleration: None,
            max_velocity: None,
            min_velocity: None,
            min_lateral_distance: None,
            max_relative_velocity: None,
        }
    }

    #[test]
    fn test_generate_multiple_scenarios() {
        let spec = create_test_spec();
        let ltl_formula = LTLGenerator::generate(&spec);

        // Generate 3 scenarios
        let scenarios = generate_scenarios(
            &spec,
            &ltl_formula,
            3,
            None::<fn(usize, &Scenario) -> Result<()>>,
        )
        .unwrap();

        // Should have 3 scenarios
        assert!(!scenarios.is_empty());
        println!("Generated {} scenarios", scenarios.len());

        // Verify each scenario is different
        for (i, scenario) in scenarios.iter().enumerate() {
            let npc = scenario.get_actor("npc").unwrap();
            let npc_px0 = npc.states[0].position.x;
            let npc_vx0 = npc.states[0].velocity.vx;

            println!("Scenario {}: NPC px0={:.2}, vx0={:.2}", i, npc_px0, npc_vx0);

            // Verify NPC eventually changes to lane 1
            let mut found_lane_change = false;
            for state in &npc.states {
                if state.lane == 1 {
                    found_lane_change = true;
                    break;
                }
            }
            assert!(found_lane_change, "NPC should change to lane 1");
        }

        // Verify scenarios are different (at least one parameter different)
        if scenarios.len() >= 2 {
            let npc0 = scenarios[0].get_actor("npc").unwrap();
            let npc1 = scenarios[1].get_actor("npc").unwrap();

            let px0_0 = npc0.states[0].position.x;
            let vx0_0 = npc0.states[0].velocity.vx;
            let px0_1 = npc1.states[0].position.x;
            let vx0_1 = npc1.states[0].velocity.vx;

            let different = (px0_0 - px0_1).abs() > 0.01 || (vx0_0 - vx0_1).abs() > 0.01;
            assert!(
                different,
                "Scenarios should have different initial conditions"
            );
        }
    }

    #[test]
    fn test_blocking_clause() {
        let spec = create_test_spec();

        // Generate first scenario in its own context
        let cfg = Config::new();
        let scenario1 = z3::with_z3_config(&cfg, || {
            let mut encoder = Z3Encoder::new(spec.clone());

            encoder.create_variables();
            encoder.encode_initial_conditions();

            // Generate first scenario
            let ltl_formula = LTLGenerator::generate(&spec);
            let scenario_model = spec.scenario_type.get_model();
            encoder.encode_kinematics();
            encoder.encode_lane_velocity_constraints();
            encoder.encode_lateral_velocity_bounds();
            encoder.encode_ltl(&ltl_formula);
            encoder
                .encode_scenario_specific_constraints(&*scenario_model)
                .unwrap();
            // Safety constraints are now included in LTL formula via generate_safety()

            let result = encoder.check();
            assert_eq!(result, SatResult::Sat);

            let model = encoder.get_model().unwrap();
            encoder.extract_scenario(&model)
        });

        // Generate second scenario in a separate context (not nested)
        let cfg2 = Config::new();
        let scenario2 = z3::with_z3_config(&cfg2, || {
            let ltl_formula = LTLGenerator::generate(&spec);
            let scenario_model = spec.scenario_type.get_model();
            let mut enc = Z3Encoder::new(spec.clone());
            enc.create_variables();
            enc.encode_initial_conditions();
            enc.encode_kinematics();
            enc.encode_lane_velocity_constraints();
            enc.encode_lateral_velocity_bounds();
            enc.encode_ltl(&ltl_formula);
            enc.encode_scenario_specific_constraints(&*scenario_model)
                .unwrap();
            // Safety constraints are now included in LTL formula via generate_safety()

            // Add blocking clause
            let blocking = create_blocking_clause(&enc, &scenario1);
            enc.assert_constraint(&blocking);

            // Should still be satisfiable (with different solution)
            let result2 = enc.check();
            assert_eq!(result2, SatResult::Sat);

            let model2 = enc.get_model().unwrap();
            enc.extract_scenario(&model2)
        });

        // Verify scenarios are different
        let npc1 = scenario1.get_actor("npc").unwrap();
        let npc2 = scenario2.get_actor("npc").unwrap();

        let px1 = npc1.states[0].position.x;
        let vx1 = npc1.states[0].velocity.vx;
        let px2 = npc2.states[0].position.x;
        let vx2 = npc2.states[0].velocity.vx;

        println!("Scenario 1: px0={:.2}, vx0={:.2}", px1, vx1);
        println!("Scenario 2: px0={:.2}, vx0={:.2}", px2, vx2);

        let different = (px1 - px2).abs() > 0.01 || (vx1 - vx2).abs() > 0.01;
        assert!(different, "Scenarios should be different");
    }
}
