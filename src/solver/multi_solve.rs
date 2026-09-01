//! Multiple scenario generation with blocking clauses
//!
//! Generates multiple diverse scenarios from the same specification by using
//! blocking clauses to prevent duplicate solutions. Diversity is enforced on
//! NPC initial position and velocity.

use crate::dsl::types::{ActorRole, OptimizationTarget as DslOptimizationTarget, ScenarioSpec};
use crate::error::{Result, ScenarioGenError};
use crate::ltl::formula::LTLFormula;
use crate::scenario::model::{OptimizationInfo, Scenario};
use crate::scenarios::ScenarioModel;
use crate::solver::backend::{
    OptimizationTarget as BackendOptimizationTarget, OptimizerBackend, Z3Backend,
};
use crate::solver::encoder_utils::real_from_f64;
use crate::solver::{GenericEncoder, Z3Encoder};
use z3::ast::{Bool, Real};
use z3::{Config, SatResult};

/// Outcome of a single Z3 solve attempt, kept distinguishable all the way out to the
/// caller. Folding `Unknown` into `Unsat` here is exactly the bug SW-13/H9 fixes: a
/// solver timeout or incompleteness result is not a proof that no scenario exists.
enum SolveOutcome {
    Sat(Box<Scenario>),
    Unsat,
    Unknown,
}

/// Standard constraint-encoding pipeline shared by every solve attempt (both the SAT
/// and optimizer backends, single- and multi-scenario). Centralised so the two
/// backends cannot silently diverge in which constraints they encode.
fn encode_standard_pipeline<B: Z3Backend + 'static>(
    encoder: &mut GenericEncoder<B>,
    ltl_formula: &LTLFormula,
    scenario_model: &dyn ScenarioModel,
) -> Result<()> {
    encoder.create_variables();
    encoder.encode_initial_conditions();
    encoder.encode_kinematics();
    encoder.encode_velocity_constraints();
    encoder.encode_acceleration_constraints();
    encoder.encode_lane_velocity_constraints();
    encoder.encode_lateral_velocity_bounds();
    encoder.encode_ltl(ltl_formula);
    encoder.encode_scenario_specific_constraints(scenario_model)?;
    // Safety constraints are encoded via LTL propositions inside encode_ltl().
    Ok(())
}

/// Drive the generate-N-diverse-scenarios loop, given a closure that runs one solve
/// attempt (already including blocking clauses against everything generated so far).
///
/// Behaviour, made explicit (previously undocumented, see SW-13/H9):
/// - `Unsat` on any iteration stops the loop — "no more unique scenarios exist" is a
///   legitimate, successful terminal state, so a partial result (e.g. `-n 10` yielding
///   3) is returned as `Ok` with only a `warn!`.
/// - `Unknown` on any iteration also stops the loop, but is NOT treated as equivalent
///   to `Unsat`: if it happens before anything was generated, the caller gets
///   `ScenarioGenError::SolverUnknown`, not `Unsatisfiable` — the two are different
///   diagnoses (timeout/incompleteness vs. a proof of no solution). If some scenarios
///   were already generated, they are still returned as a (loudly logged) partial
///   success, on the same reasoning as the `Unsat` case.
fn drive_generation<F>(
    num_scenarios: usize,
    mut callback: Option<F>,
    mut solve_one: impl FnMut(&[Scenario]) -> Result<SolveOutcome>,
    unknown_label: &str,
) -> Result<Vec<Scenario>>
where
    F: FnMut(usize, &Scenario) -> Result<()>,
{
    let mut scenarios = Vec::new();
    let mut saw_unknown = false;

    for i in 0..num_scenarios {
        let scenario = match solve_one(&scenarios)? {
            SolveOutcome::Sat(scenario) => *scenario,
            SolveOutcome::Unsat => {
                tracing::warn!(
                    "No more unique scenarios found after {} scenarios",
                    scenarios.len()
                );
                break;
            }
            SolveOutcome::Unknown => {
                tracing::error!(
                    "Z3 {} returned UNKNOWN for scenario {}",
                    unknown_label,
                    i + 1
                );
                saw_unknown = true;
                break;
            }
        };

        tracing::info!("Generated scenario {}/{}", i + 1, num_scenarios);

        // Call callback if provided (after Z3 context is released)
        if let Some(ref mut cb) = callback {
            cb(i, &scenario)?;
        }

        scenarios.push(scenario);
    }

    if scenarios.is_empty() {
        return Err(if saw_unknown {
            ScenarioGenError::SolverUnknown
        } else {
            ScenarioGenError::Unsatisfiable
        });
    }

    if saw_unknown {
        tracing::warn!(
            "Returning {} scenario(s): Z3 returned UNKNOWN partway through generation. \
             This is a timeout/incompleteness signal, not proof that no further unique \
             scenarios exist.",
            scenarios.len()
        );
    }

    Ok(scenarios)
}

/// Generate multiple diverse scenarios from the same specification
///
/// Uses blocking clauses to ensure each generated scenario is different.
/// Specifically, we block based on NPC initial conditions (position and velocity).
///
/// Honours `spec.optimization_target` (SW-13/D4): previously this always went through
/// the plain SAT solver, so `--optimize` set on the CLI was silently dropped whenever
/// `num_scenarios > 1`. It now runs each of the N solves through the Z3 optimizer
/// backend instead, with the same blocking clauses enforcing diversity between them.
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
/// Returns error if specification is invalid, initial setup fails, or (for an
/// unrecognised optimization target) the target cannot be mapped to a backend target.
pub fn generate_scenarios<F>(
    spec: &ScenarioSpec,
    ltl_formula: &LTLFormula,
    num_scenarios: usize,
    callback: Option<F>,
) -> Result<Vec<Scenario>>
where
    F: FnMut(usize, &Scenario) -> Result<()>,
{
    match spec.optimization_target {
        DslOptimizationTarget::None => drive_generation(
            num_scenarios,
            callback,
            |prev_scenarios| solve_one_sat(spec, ltl_formula, prev_scenarios),
            "solver",
        ),
        target => {
            let backend_target = crate::dsl_target_to_backend_target(target)?;
            drive_generation(
                num_scenarios,
                callback,
                |prev_scenarios| {
                    solve_one_optimized(spec, ltl_formula, backend_target, target, prev_scenarios)
                },
                "optimizer",
            )
        }
    }
}

/// Run one SAT-backend solve attempt (blocking clauses against `prev_scenarios` already
/// generated), classified into a [`SolveOutcome`].
fn solve_one_sat(
    spec: &ScenarioSpec,
    ltl_formula: &LTLFormula,
    prev_scenarios: &[Scenario],
) -> Result<SolveOutcome> {
    let scenario_model = spec.scenario_type.get_model();
    let cfg = Config::new();
    z3::with_z3_config(&cfg, || {
        let mut encoder = Z3Encoder::new(spec.clone());
        encode_standard_pipeline(&mut encoder, ltl_formula, &*scenario_model)?;

        for prev_scenario in prev_scenarios {
            let blocking_clause = create_blocking_clause(&encoder, prev_scenario)?;
            encoder.assert_constraint(&blocking_clause);
        }

        match encoder.check() {
            SatResult::Sat => {
                let model = encoder.get_model().ok_or_else(|| {
                    ScenarioGenError::ExtractionFailed("Failed to get Z3 model".to_string())
                })?;
                let scenario = encoder.extract_scenario(&model)?;
                Ok(SolveOutcome::Sat(Box::new(scenario)))
            }
            SatResult::Unsat => Ok(SolveOutcome::Unsat),
            SatResult::Unknown => Ok(SolveOutcome::Unknown),
        }
    })
}

/// Run one optimizer-backend solve attempt (blocking clauses against `prev_scenarios`
/// already generated, plus the objective for `backend_target`), classified into a
/// [`SolveOutcome`]. On `Sat`, the scenario's `optimization` field is populated exactly
/// as the single-scenario optimizer path (`generate_with_optimizer` in `lib.rs`) does.
fn solve_one_optimized(
    spec: &ScenarioSpec,
    ltl_formula: &LTLFormula,
    backend_target: BackendOptimizationTarget,
    dsl_target: DslOptimizationTarget,
    prev_scenarios: &[Scenario],
) -> Result<SolveOutcome> {
    let scenario_model = spec.scenario_type.get_model();
    let cfg = Config::new();
    z3::with_z3_config(&cfg, || {
        let mut encoder =
            GenericEncoder::with_backend(spec.clone(), OptimizerBackend::new(backend_target));
        encode_standard_pipeline(&mut encoder, ltl_formula, &*scenario_model)?;

        for prev_scenario in prev_scenarios {
            let blocking_clause = create_blocking_clause(&encoder, prev_scenario)?;
            encoder.assert_constraint(&blocking_clause);
        }

        encoder.encode_objective();

        match encoder.check() {
            SatResult::Sat => {
                let model = encoder.get_model().ok_or_else(|| {
                    ScenarioGenError::ExtractionFailed("Failed to get Z3 model".to_string())
                })?;
                encoder.extract_optimal_value(&model);
                let opt_val = encoder.get_optimal_value();

                let mut scenario = encoder.extract_scenario(&model)?;
                scenario.optimization = Some(OptimizationInfo {
                    target: format!("{:?}", dsl_target),
                    optimal_value: opt_val,
                });

                Ok(SolveOutcome::Sat(Box::new(scenario)))
            }
            SatResult::Unsat => Ok(SolveOutcome::Unsat),
            SatResult::Unknown => Ok(SolveOutcome::Unknown),
        }
    })
}

/// Create a blocking clause to prevent generating the same scenario
///
/// We block based on all non-ego actors' initial conditions (position and velocity at t=0).
/// This ensures diversity in the generated scenarios across different actor types.
///
/// Uses position_x and velocity_x for all coordinate systems (Cartesian and Bicycle both use x-axis).
///
/// The blocking clause is: !(actor1_equal AND actor2_equal AND ...)
/// Which is equivalent to: (actor1_differs OR actor2_differs OR ...)
/// At least one non-ego actor must have different initial conditions from previous scenarios.
fn create_blocking_clause<B: Z3Backend + 'static>(
    encoder: &GenericEncoder<B>,
    prev_scenario: &Scenario,
) -> Result<Bool> {
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
            .ok_or_else(|| ScenarioGenError::ActorNotFound(actor.id.clone()))?;

        // Get actor initial state (t=0)
        let actor_initial = &actor_traj.states[0];

        // Block based on position_x and velocity_x (works for both Cartesian and Bicycle)
        let blocking_clause = {
            let prev_px0 = actor_initial.position().x;
            let prev_vx0 = actor_initial.velocity().vx;

            let actor_px0 = encoder.get_position_x(&actor.id, 0);
            let actor_vx0 = encoder.get_velocity_x(&actor.id, 0);

            let prev_px0_z3 = real_from_f64(prev_px0);
            let prev_vx0_z3 = real_from_f64(prev_vx0);

            // Tolerance bands for diversity (block near-duplicates)
            let pos_tolerance = Real::from_rational(5_i64, 10_i64); // 0.5m
            let vel_tolerance = Real::from_rational(2_i64, 10_i64); // 0.2 m/s

            let px_close = Bool::and(&[
                &actor_px0.ge(&(&prev_px0_z3 - &pos_tolerance)),
                &actor_px0.le(&(&prev_px0_z3 + &pos_tolerance)),
            ]);

            let vx_close = Bool::and(&[
                &actor_vx0.ge(&(&prev_vx0_z3 - &vel_tolerance)),
                &actor_vx0.le(&(&prev_vx0_z3 + &vel_tolerance)),
            ]);

            // For pedestrians, also block lateral (y-axis) initial conditions
            if actor.role == ActorRole::Pedestrian {
                let prev_py0 = actor_initial.position().y;
                let prev_vy0 = actor_initial.velocity().vy;

                let actor_py0 = encoder.get_position_y(&actor.id, 0);
                let actor_vy0 = encoder.get_velocity_y(&actor.id, 0);

                let prev_py0_z3 = real_from_f64(prev_py0);
                let prev_vy0_z3 = real_from_f64(prev_vy0);

                let py_close = Bool::and(&[
                    &actor_py0.ge(&(&prev_py0_z3 - &pos_tolerance)),
                    &actor_py0.le(&(&prev_py0_z3 + &pos_tolerance)),
                ]);

                let vy_close = Bool::and(&[
                    &actor_vy0.ge(&(&prev_vy0_z3 - &vel_tolerance)),
                    &actor_vy0.le(&(&prev_vy0_z3 + &vel_tolerance)),
                ]);

                // Block if all four are within tolerance
                let all_close = Bool::and(&[&px_close, &vx_close, &py_close, &vy_close]);
                all_close.not()
            } else {
                // For vehicles, block if both longitudinal values are within tolerance
                let both_close = Bool::and(&[&px_close, &vx_close]);
                both_close.not()
            }
        };

        all_blocking_clauses.push(blocking_clause);
    }

    // Combine with OR: at least one actor must differ
    if all_blocking_clauses.is_empty() {
        Ok(Bool::from_bool(true))
    } else if all_blocking_clauses.len() == 1 {
        Ok(all_blocking_clauses
            .into_iter()
            .next()
            .expect("len checked above"))
    } else {
        Ok(Bool::or(&all_blocking_clauses))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dsl::types::{
        ActorRole, ActorSpec, LaneChangeConfig, LaneChangeDirection, RoadSpec, ScenarioType,
        ValueOrRange,
    };
    use crate::ltl::generator::LTLGenerator;
    use std::collections::HashMap;

    /// SW-13/H9: `SatResult::Unknown` must never be reported to the caller as
    /// `Unsatisfiable` — a solver timeout/incompleteness is a different diagnosis
    /// from a proof that no scenario exists. Exercised at the `drive_generation`
    /// level with a stubbed solve result, since forcing a real Z3 `unknown` would
    /// require deliberately leaving the decidable linear-arithmetic fragment
    /// (SW-11's territory), which this issue does not touch.
    #[test]
    fn unknown_on_first_attempt_reports_solver_unknown_not_unsatisfiable() {
        let result: Result<Vec<Scenario>> = drive_generation(
            1,
            None::<fn(usize, &Scenario) -> Result<()>>,
            |_prev_scenarios| Ok(SolveOutcome::Unknown),
            "solver",
        );

        match result {
            Err(ScenarioGenError::SolverUnknown) => {}
            other => panic!("expected Err(SolverUnknown), got {other:?}"),
        }
    }

    /// Unsat, by contrast, is a legitimate proof there is no (further) solution and
    /// must still map to `Unsatisfiable` when nothing was generated.
    #[test]
    fn unsat_on_first_attempt_still_reports_unsatisfiable() {
        let result: Result<Vec<Scenario>> = drive_generation(
            1,
            None::<fn(usize, &Scenario) -> Result<()>>,
            |_prev_scenarios| Ok(SolveOutcome::Unsat),
            "solver",
        );

        assert!(
            matches!(result, Err(ScenarioGenError::Unsatisfiable)),
            "expected Err(Unsatisfiable), got {result:?}"
        );
    }

    /// Minimal placeholder `Scenario` for `SolveOutcome::Sat` stubs — its contents are
    /// irrelevant to the `drive_generation` control-flow tests, only its presence.
    fn dummy_scenario() -> Scenario {
        Scenario {
            scenario_id: "test".to_string(),
            scenario_type: "cut_in_left".to_string(),
            time_step: 0.5,
            duration: 1.0,
            road: RoadSpec {
                num_lanes: 1,
                lane_width: 3.5,
                lane_directions: vec![1],
                road_length: None,
            },
            actors: vec![],
            validation: crate::scenario::model::ValidationInfo {
                min_ttc: None,
                min_distance: None,
                all_constraints_satisfied: true,
                safety_violations: vec![],
                max_acceleration: 0.0,
                max_deceleration: 0.0,
                acceleration_violations: vec![],
            },
            optimization: None,
        }
    }

    /// A partial result (some scenarios generated, then Unknown) is still returned
    /// as `Ok` — documented behaviour, see `drive_generation`'s doc comment — but the
    /// caller has no way to tell from the `Ok` alone that generation was cut short by
    /// an UNKNOWN rather than by genuinely running out of unique solutions. This test
    /// pins the current decision (partial success) so a future change to it is
    /// deliberate, not silent.
    #[test]
    fn unknown_after_partial_success_returns_scenarios_generated_so_far() {
        let mut attempt = 0;
        let result: Result<Vec<Scenario>> = drive_generation(
            3,
            None::<fn(usize, &Scenario) -> Result<()>>,
            |_prev_scenarios| {
                attempt += 1;
                if attempt == 1 {
                    Ok(SolveOutcome::Sat(Box::new(dummy_scenario())))
                } else {
                    Ok(SolveOutcome::Unknown)
                }
            },
            "solver",
        );

        let scenarios = result.expect("partial success should be Ok, not Err");
        assert_eq!(scenarios.len(), 1);
    }

    fn create_test_spec() -> ScenarioSpec {
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
                    direction: 1,
                    behavior: HashMap::new(),
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
                    lane_changes: vec![LaneChangeConfig {
                        direction: LaneChangeDirection::Right,
                        start_time: ValueOrRange::Range([2.5, 7.5]),
                        duration: ValueOrRange::Range([3.0, 4.0]),
                    }],
                    bicycle_params: None,
                },
            ],
            min_ttc: 3.0,
            min_distance: 5.0,
            road: Some(RoadSpec {
                num_lanes: 2,
                lane_width: 3.5,
                lane_directions: vec![1, 1],
                road_length: None,
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
            max_lateral_acceleration: 2.0,
            coordinate_system: crate::dsl::types::CoordinateSystem::Cartesian,
            bicycle_config: None,
        }
    }

    #[test]
    fn test_generate_multiple_scenarios() {
        let spec = create_test_spec();
        let ltl_formula = LTLGenerator::generate(&spec).unwrap();

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
            let npc_px0 = npc.states[0].position().x;
            let npc_vx0 = npc.states[0].velocity().vx;

            println!("Scenario {}: NPC px0={:.2}, vx0={:.2}", i, npc_px0, npc_vx0);

            // Verify NPC eventually changes to lane 1
            let mut found_lane_change = false;
            for state in &npc.states {
                if state.lane() == 1 {
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

            let px0_0 = npc0.states[0].position().x;
            let vx0_0 = npc0.states[0].velocity().vx;
            let px0_1 = npc1.states[0].position().x;
            let vx0_1 = npc1.states[0].velocity().vx;

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
            let ltl_formula = LTLGenerator::generate(&spec).unwrap();
            let scenario_model = spec.scenario_type.get_model();
            encoder.encode_kinematics();
            encoder.encode_velocity_constraints();
            encoder.encode_acceleration_constraints();
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
            encoder.extract_scenario(&model).unwrap()
        });

        // Generate second scenario in a separate context (not nested)
        let cfg2 = Config::new();
        let scenario2 = z3::with_z3_config(&cfg2, || {
            let ltl_formula = LTLGenerator::generate(&spec).unwrap();
            let scenario_model = spec.scenario_type.get_model();
            let mut enc = Z3Encoder::new(spec.clone());
            enc.create_variables();
            enc.encode_initial_conditions();
            enc.encode_kinematics();
            enc.encode_velocity_constraints();
            enc.encode_acceleration_constraints();
            enc.encode_lane_velocity_constraints();
            enc.encode_lateral_velocity_bounds();
            enc.encode_ltl(&ltl_formula);
            enc.encode_scenario_specific_constraints(&*scenario_model)
                .unwrap();
            // Safety constraints are now included in LTL formula via generate_safety()

            // Add blocking clause
            let blocking = create_blocking_clause(&enc, &scenario1).unwrap();
            enc.assert_constraint(&blocking);

            // Should still be satisfiable (with different solution)
            let result2 = enc.check();
            assert_eq!(result2, SatResult::Sat);

            let model2 = enc.get_model().unwrap();
            enc.extract_scenario(&model2).unwrap()
        });

        // Verify scenarios are different
        let npc1 = scenario1.get_actor("npc").unwrap();
        let npc2 = scenario2.get_actor("npc").unwrap();

        let px1 = npc1.states[0].position().x;
        let vx1 = npc1.states[0].velocity().vx;
        let px2 = npc2.states[0].position().x;
        let vx2 = npc2.states[0].velocity().vx;

        println!("Scenario 1: px0={:.2}, vx0={:.2}", px1, vx1);
        println!("Scenario 2: px0={:.2}, vx0={:.2}", px2, vx2);

        let different = (px1 - px2).abs() > 0.01 || (vx1 - vx2).abs() > 0.01;
        assert!(different, "Scenarios should be different");
    }
}
