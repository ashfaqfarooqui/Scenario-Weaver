//! Multiple scenario generation with blocking clauses
//!
//! This module implements Phase 11 - generating multiple diverse scenarios
//! from the same specification by using blocking clauses to prevent
//! duplicate solutions.

use crate::dsl::types::ScenarioSpec;
use crate::error::{Result, ScenarioGenError};
use crate::ltl::formula::LTLFormula;
use crate::scenario::model::Scenario;
use crate::solver::Z3Encoder;
use z3::ast::{Ast, Bool, Real};
use z3::{Config, Context, SatResult};

/// Generate multiple diverse scenarios from the same specification
///
/// Uses blocking clauses to ensure each generated scenario is different.
/// Specifically, we block based on NPC initial conditions (position and velocity).
///
/// # Arguments
/// * `spec` - Scenario specification
/// * `ltl_formula` - Generated LTL formula (same for all scenarios)
/// * `num_scenarios` - Number of scenarios to generate
///
/// # Returns
/// A vector of unique scenarios
///
/// # Errors
/// Returns error if specification is invalid or initial setup fails
pub fn generate_scenarios(
    spec: &ScenarioSpec,
    ltl_formula: &LTLFormula,
    num_scenarios: usize,
) -> Result<Vec<Scenario>> {
    let mut scenarios = Vec::new();

    for i in 0..num_scenarios {
        // Create fresh Z3 context for each scenario
        let cfg = Config::new();
        let ctx = Context::new(&cfg);
        let mut encoder = Z3Encoder::new(&ctx, spec.clone());

        // Setup encoder (same as single scenario)
        encoder.create_variables();
        encoder.encode_initial_conditions();
        encoder.encode_kinematics();
        encoder.encode_ltl(ltl_formula);
        encoder.encode_safety();

        // Add blocking clauses for all previous scenarios
        for prev_scenario in &scenarios {
            let blocking_clause = create_blocking_clause(&ctx, &encoder, prev_scenario);
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
            }
            SatResult::Unsat => {
                tracing::warn!(
                    "No more unique scenarios found after {} scenarios",
                    scenarios.len()
                );
                break; // No more solutions exist
            }
            SatResult::Unknown => {
                tracing::error!("Z3 returned UNKNOWN for scenario {}", i + 1);
                break;
            }
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
/// We block based on NPC initial conditions (position and velocity at t=0).
/// This ensures diversity in the generated scenarios.
///
/// The blocking clause is: !(px0 == prev_px0 AND vx0 == prev_vx0)
/// Which is equivalent to: (px0 != prev_px0 OR vx0 != prev_vx0)
fn create_blocking_clause<'ctx>(
    ctx: &'ctx Context,
    encoder: &Z3Encoder<'ctx>,
    prev_scenario: &Scenario,
) -> Bool<'ctx> {
    // Get NPC trajectory from previous scenario
    let npc_traj = prev_scenario
        .get_actor("npc")
        .expect("NPC trajectory missing");

    // Get NPC initial state (t=0)
    let npc_initial = &npc_traj.states[0];
    let prev_px0 = npc_initial.position.x;
    let prev_vx0 = npc_initial.velocity.vx;

    // Get Z3 variables for NPC initial conditions
    let npc_px0 = encoder.get_position_x("npc", 0);
    let npc_vx0 = encoder.get_velocity_x("npc", 0);

    // Create real values from previous scenario
    // We use a tolerance to handle floating point precision
    // Convert to Z3 rational: multiply by 100 to get 2 decimal precision
    let prev_px0_z3 = Real::from_real(ctx, (prev_px0 * 100.0).round() as i32, 100);
    let prev_vx0_z3 = Real::from_real(ctx, (prev_vx0 * 100.0).round() as i32, 100);

    // Create equality constraints
    let px_eq = npc_px0._eq(&prev_px0_z3);
    let vx_eq = npc_vx0._eq(&prev_vx0_z3);

    // Both equal: px0 == prev_px0 AND vx0 == prev_vx0
    let both_equal = Bool::and(ctx, &[&px_eq, &vx_eq]);

    // Blocking clause: NOT(both equal)
    // This forces at least one of them to be different
    both_equal.not()
}

/// Add accessor methods to Z3Encoder for multi_solve module
impl<'ctx> Z3Encoder<'ctx> {
    /// Get position_x variable for an actor at a specific time
    pub fn get_position_x(&self, actor_id: &str, time: usize) -> &Real<'ctx> {
        &self.positions_x[actor_id][time]
    }

    /// Get velocity_x variable for an actor at a specific time
    pub fn get_velocity_x(&self, actor_id: &str, time: usize) -> &Real<'ctx> {
        &self.velocities_x[actor_id][time]
    }

    /// Assert a constraint to the solver
    pub fn assert_constraint(&self, constraint: &Bool<'ctx>) {
        self.solver.assert(constraint);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dsl::types::{ActorSpec, ActorRole, ScenarioType, ValueOrRange};
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
            lane_width: 3.5,
            num_scenarios: 5,
            constraint_modes: crate::dsl::types::ConstraintModes::default(),
            max_acceleration: None,
            max_deceleration: None,
        }
    }

    #[test]
    fn test_generate_multiple_scenarios() {
        let spec = create_test_spec();
        let ltl_formula = LTLGenerator::generate(&spec);

        // Generate 3 scenarios
        let scenarios = generate_scenarios(&spec, &ltl_formula, 3).unwrap();

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
        let cfg = Config::new();
        let ctx = Context::new(&cfg);
        let mut encoder = Z3Encoder::new(&ctx, spec.clone());

        encoder.create_variables();
        encoder.encode_initial_conditions();

        // Generate first scenario
        let ltl_formula = LTLGenerator::generate(&spec);
        encoder.encode_kinematics();
        encoder.encode_ltl(&ltl_formula);
        encoder.encode_safety();

        let result = encoder.check();
        assert_eq!(result, SatResult::Sat);

        let model = encoder.get_model().unwrap();
        let scenario1 = encoder.extract_scenario(&model);

        // Create fresh context and encoder for second scenario
        let cfg2 = Config::new();
        let ctx2 = Context::new(&cfg2);
        let mut encoder2 = Z3Encoder::new(&ctx2, spec);

        encoder2.create_variables();
        encoder2.encode_initial_conditions();
        encoder2.encode_kinematics();
        encoder2.encode_ltl(&ltl_formula);
        encoder2.encode_safety();

        // Add blocking clause
        let blocking = create_blocking_clause(&ctx2, &encoder2, &scenario1);
        encoder2.assert_constraint(&blocking);

        // Should still be satisfiable (with different solution)
        let result2 = encoder2.check();
        assert_eq!(result2, SatResult::Sat);

        let model2 = encoder2.get_model().unwrap();
        let scenario2 = encoder2.extract_scenario(&model2);

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
