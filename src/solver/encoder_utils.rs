//! Shared utilities for coordinate encoders
//!
//! This module provides common helper functions used by both CartesianEncoder
//! and BicycleEncoder to reduce code duplication.

use std::collections::HashMap;
use z3::ast::{Bool, Int, Real};
use z3::Model;

use crate::dsl::types::{ActorRole, LaneChangeDirection, ScenarioSpec};
use crate::error::{Result, ScenarioGenError};

/// Resolved lane change timing for an actor, expressed as discrete time-step indices.
#[derive(Debug, Clone)]
pub struct LaneChangeSteps {
    /// Direction of the lane change (left or right)
    pub direction: LaneChangeDirection,
    /// Time step when the lane change starts
    pub start_step: usize,
    /// Time step when the lane change ends
    pub end_step: usize,
}

/// Extract a real value from Z3 model
///
/// Handles rationals and complex expressions with fallback to decimal approximation.
/// This is the more robust version from BicycleEncoder that handles edge cases.
pub fn extract_real(model: &Model, var: &Real) -> Result<f64> {
    let ast = model.eval(var, true).ok_or_else(|| {
        ScenarioGenError::Z3ModelParsing("Failed to evaluate real variable".to_string())
    })?;

    // First try to extract as rational directly
    if let Some(rational) = ast.as_rational() {
        let (num, denom) = rational;
        return Ok(num as f64 / denom as f64);
    }

    // If not a simple rational, try as_real() which handles more complex expressions
    #[allow(deprecated)]
    if let Some((num, denom)) = ast.as_real() {
        return Ok(num as f64 / denom as f64);
    }

    // As a last resort, use Z3's decimal approximation for complex expressions
    let decimal_str = ast.approx(10); // 10 decimal places precision
    decimal_str.parse::<f64>().map_err(|e| {
        ScenarioGenError::Z3ModelParsing(format!(
            "Failed to parse decimal approximation '{}' for expression {}: {}",
            decimal_str, ast, e
        ))
    })
}

/// Extract a non-negative integer value from a Z3 model variable.
pub fn extract_int(model: &Model, var: &Int) -> Result<usize> {
    let ast = model.eval(var, true).ok_or_else(|| {
        ScenarioGenError::Z3ModelParsing("Failed to evaluate int variable".to_string())
    })?;

    if let Some(val) = ast.as_i64() {
        if val < 0 {
            return Err(ScenarioGenError::Z3ModelParsing(format!(
                "Expected non-negative integer, got: {}",
                val
            )));
        }
        Ok(val as usize)
    } else {
        Err(ScenarioGenError::Z3ModelParsing(format!(
            "Expected integer value, got: {}",
            ast
        )))
    }
}

/// Collect lane change data for all actors, converting time ranges to step ranges
///
/// Returns a HashMap from actor_id to Vec of lane change steps.
/// Only includes actors that are not pedestrians and have lane changes configured.
pub fn collect_lane_change_data(
    spec: &ScenarioSpec,
    horizon: usize,
) -> HashMap<String, Vec<LaneChangeSteps>> {
    let dt = spec.time_step;

    spec.actors
        .iter()
        .filter(|a| a.role != ActorRole::Pedestrian)
        .filter(|a| !a.lane_changes.is_empty())
        .map(|a| {
            let changes: Vec<LaneChangeSteps> = a
                .lane_changes
                .iter()
                .map(|lc| {
                    let start_min = lc.start_time.min();
                    let start_max = lc.start_time.max();
                    let duration_min = lc.duration.min();
                    let duration_max = lc.duration.max();

                    let start_step_min = (start_min / dt) as usize;
                    let start_step_max = (start_max / dt) as usize;
                    let duration_steps_min = (duration_min / dt) as usize;
                    let duration_steps_max = (duration_max / dt) as usize;

                    // Use midpoint for now (TODO: make solver variables)
                    let start_step = usize::midpoint(start_step_min, start_step_max);
                    let duration_steps = usize::midpoint(duration_steps_min, duration_steps_max);
                    let end_step = (start_step + duration_steps).min(horizon);

                    LaneChangeSteps {
                        direction: lc.direction,
                        start_step,
                        end_step,
                    }
                })
                .collect();
            (a.id.clone(), changes)
        })
        .collect()
}

/// Encode "same lane" check for two actors using y-position proximity
///
/// This function creates a Z3 Bool that is true when two actors are in the
/// same lateral space (i.e., |py1 - py2| < lane_width).
///
/// IMPORTANT: This uses AND (not OR) to correctly check absolute value:
/// |py1 - py2| < lane_width is equivalent to:
///   (py1 - py2 < lane_width) AND (py2 - py1 < lane_width)
///
/// Using OR would be incorrect because:
/// - If py1 - py2 = 5.0 and lane_width = 3.5
/// - py_diff_pos = 5.0, so 5.0 < 3.5 is FALSE
/// - py_diff_neg = -5.0, so -5.0 < 3.5 is TRUE (always true for negative values!)
/// - OR would incorrectly return TRUE
///
/// With AND:
/// - Both conditions must be true
/// - This correctly requires the actual distance to be less than lane_width
pub fn encode_y_proximity_constraint(py1: &Real, py2: &Real, lane_width: f64) -> Bool {
    let lane_width_real = Real::from_rational((lane_width * 10.0) as i64, 10_i64);
    let py_diff_pos = py1 - py2;
    let py_diff_neg = py2 - py1;

    // FIXED: Use AND to properly check |py1 - py2| < lane_width
    // Both (py1-py2) < lane_width AND (py2-py1) < lane_width must be true
    Bool::and(&[
        &py_diff_pos.lt(&lane_width_real),
        &py_diff_neg.lt(&lane_width_real),
    ])
}

/// Encode combined "same lane" constraint (discrete lane match OR y-proximity)
///
/// Returns true if actors are in the same lane either by:
/// 1. Having the same discrete lane value, OR
/// 2. Having lateral positions within one lane width of each other
pub fn encode_same_lane_constraint(
    lane1: &Int,
    lane2: &Int,
    py1: &Real,
    py2: &Real,
    lane_width: f64,
) -> Bool {
    let same_lane_discrete = lane1.eq(lane2);
    let y_proximity = encode_y_proximity_constraint(py1, py2, lane_width);

    // Consider "same lane" if either discrete lanes match OR y-positions are close
    Bool::or(&[&same_lane_discrete, &y_proximity])
}

#[cfg(test)]
mod tests {
    use super::*;
    use z3::ast::Ast;
    use z3::{Config, SatResult, Solver};

    use crate::dsl::types::{
        ActorRole, ActorSpec, ConstraintModes, LaneChangeConfig, LaneChangeDirection,
        OptimizationTarget, ScenarioSpec, ScenarioType, ValueOrRange,
    };

    #[test]
    fn test_lane_change_steps_struct() {
        let lcs = LaneChangeSteps {
            direction: LaneChangeDirection::Right,
            start_step: 10,
            end_step: 20,
        };
        assert_eq!(lcs.start_step, 10);
        assert_eq!(lcs.end_step, 20);
    }

    fn make_spec(actors: Vec<ActorSpec>) -> ScenarioSpec {
        ScenarioSpec {
            scenario_type: ScenarioType::CutInLeft,
            time_step: 0.1,
            duration: 5.0,
            actors,
            min_ttc: 2.0,
            min_distance: 5.0,
            road: None,
            lane_width: 3.5,
            num_scenarios: 1,
            constraint_modes: ConstraintModes::default(),
            max_acceleration: None,
            max_deceleration: None,
            optimization_target: OptimizationTarget::None,
            max_velocity: None,
            min_velocity: None,
            min_lateral_distance: None,
            max_relative_velocity: None,
            max_lateral_acceleration: 2.0,
            coordinate_system: Default::default(),
            bicycle_config: None,
        }
    }

    fn make_actor(id: &str, role: ActorRole, lane_changes: Vec<LaneChangeConfig>) -> ActorSpec {
        ActorSpec {
            id: id.to_string(),
            role,
            lane: 1,
            position: ValueOrRange::Value(0.0),
            speed: ValueOrRange::Value(10.0),
            acceleration: ValueOrRange::Value(0.0),
            direction: 1,
            behavior: Default::default(),
            lane_changes,
            bicycle_params: None,
        }
    }

    #[test]
    fn test_collect_single_fixed_lane_change() {
        let lc = LaneChangeConfig {
            direction: LaneChangeDirection::Left,
            start_time: ValueOrRange::Value(1.0),
            duration: ValueOrRange::Value(2.0),
        };
        let actor = make_actor("npc1", ActorRole::Npc, vec![lc]);
        let spec = make_spec(vec![actor]);
        let result = collect_lane_change_data(&spec, 50);
        let steps = &result["npc1"];
        assert_eq!(steps.len(), 1);
        assert_eq!(steps[0].start_step, 10);
        assert_eq!(steps[0].end_step, 30);
        assert_eq!(steps[0].direction, LaneChangeDirection::Left);
    }

    #[test]
    fn test_collect_range_valued_lane_change_uses_midpoint() {
        let lc = LaneChangeConfig {
            direction: LaneChangeDirection::Right,
            start_time: ValueOrRange::Range([1.0, 3.0]),
            duration: ValueOrRange::Range([1.0, 3.0]),
        };
        let actor = make_actor("npc1", ActorRole::Npc, vec![lc]);
        let spec = make_spec(vec![actor]);
        let result = collect_lane_change_data(&spec, 50);
        let steps = &result["npc1"];
        assert_eq!(steps[0].start_step, 20);
        assert_eq!(steps[0].end_step, 40);
    }

    #[test]
    fn test_collect_no_lane_changes_not_in_result() {
        let actor = make_actor("npc1", ActorRole::Npc, vec![]);
        let spec = make_spec(vec![actor]);
        let result = collect_lane_change_data(&spec, 50);
        assert!(result.is_empty());
    }

    #[test]
    fn test_collect_pedestrian_filtered_out() {
        let lc = LaneChangeConfig {
            direction: LaneChangeDirection::Left,
            start_time: ValueOrRange::Value(1.0),
            duration: ValueOrRange::Value(2.0),
        };
        let actor = make_actor("ped1", ActorRole::Pedestrian, vec![lc]);
        let spec = make_spec(vec![actor]);
        let result = collect_lane_change_data(&spec, 50);
        assert!(result.is_empty());
    }

    #[test]
    fn test_collect_end_step_clamped_to_horizon() {
        let lc = LaneChangeConfig {
            direction: LaneChangeDirection::Left,
            start_time: ValueOrRange::Value(4.0),
            duration: ValueOrRange::Value(3.0),
        };
        let actor = make_actor("npc1", ActorRole::Npc, vec![lc]);
        let spec = make_spec(vec![actor]);
        let result = collect_lane_change_data(&spec, 50);
        assert_eq!(result["npc1"][0].end_step, 50);
    }

    #[test]
    fn test_collect_multiple_lane_changes_one_actor() {
        let lc1 = LaneChangeConfig {
            direction: LaneChangeDirection::Left,
            start_time: ValueOrRange::Value(1.0),
            duration: ValueOrRange::Value(1.0),
        };
        let lc2 = LaneChangeConfig {
            direction: LaneChangeDirection::Right,
            start_time: ValueOrRange::Value(3.0),
            duration: ValueOrRange::Value(1.0),
        };
        let actor = make_actor("npc1", ActorRole::Npc, vec![lc1, lc2]);
        let spec = make_spec(vec![actor]);
        let result = collect_lane_change_data(&spec, 50);
        let steps = &result["npc1"];
        assert_eq!(steps.len(), 2);
        assert_eq!(steps[0].direction, LaneChangeDirection::Left);
        assert_eq!(steps[0].start_step, 10);
        assert_eq!(steps[0].end_step, 20);
        assert_eq!(steps[1].direction, LaneChangeDirection::Right);
        assert_eq!(steps[1].start_step, 30);
        assert_eq!(steps[1].end_step, 40);
    }

    #[test]
    fn test_collect_multiple_actors() {
        let lc1 = LaneChangeConfig {
            direction: LaneChangeDirection::Left,
            start_time: ValueOrRange::Value(1.0),
            duration: ValueOrRange::Value(1.0),
        };
        let lc2 = LaneChangeConfig {
            direction: LaneChangeDirection::Right,
            start_time: ValueOrRange::Value(2.0),
            duration: ValueOrRange::Value(1.5),
        };
        let actor1 = make_actor("npc1", ActorRole::Npc, vec![lc1]);
        let actor2 = make_actor("npc2", ActorRole::Npc, vec![lc2]);
        let spec = make_spec(vec![actor1, actor2]);
        let result = collect_lane_change_data(&spec, 50);
        assert_eq!(result.len(), 2);
        assert!(result.contains_key("npc1"));
        assert!(result.contains_key("npc2"));
    }

    #[test]
    fn test_extract_real_fixed_value() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let solver = Solver::new();
            let x = Real::new_const("x");
            let val = Real::from_rational(7, 2);
            solver.assert(&x._eq(&val));
            assert_eq!(solver.check(), SatResult::Sat);
            let model = solver.get_model().unwrap();
            let result = extract_real(&model, &x).unwrap();
            assert!((result - 3.5).abs() < 1e-9);
        });
    }

    #[test]
    fn test_extract_real_integer_value() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let solver = Solver::new();
            let x = Real::new_const("x");
            let val = Real::from_rational(5, 1);
            solver.assert(&x._eq(&val));
            assert_eq!(solver.check(), SatResult::Sat);
            let model = solver.get_model().unwrap();
            let result = extract_real(&model, &x).unwrap();
            assert!((result - 5.0).abs() < 1e-9);
        });
    }

    #[test]
    fn test_extract_int_positive() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let solver = Solver::new();
            let x = Int::new_const("x");
            let val = Int::from_i64(42);
            solver.assert(&x._eq(&val));
            assert_eq!(solver.check(), SatResult::Sat);
            let model = solver.get_model().unwrap();
            let result = extract_int(&model, &x).unwrap();
            assert_eq!(result, 42);
        });
    }

    #[test]
    fn test_extract_int_zero() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let solver = Solver::new();
            let x = Int::new_const("x");
            let val = Int::from_i64(0);
            solver.assert(&x._eq(&val));
            assert_eq!(solver.check(), SatResult::Sat);
            let model = solver.get_model().unwrap();
            let result = extract_int(&model, &x).unwrap();
            assert_eq!(result, 0);
        });
    }

    #[test]
    fn test_y_proximity_sat_when_close() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let solver = Solver::new();
            let py1 = Real::new_const("py1");
            let py2 = Real::new_const("py2");
            solver.assert(&py1._eq(&Real::from_rational(10, 10)));
            solver.assert(&py2._eq(&Real::from_rational(20, 10)));
            solver.assert(&encode_y_proximity_constraint(&py1, &py2, 3.5));
            assert_eq!(solver.check(), SatResult::Sat);
        });
    }

    #[test]
    fn test_y_proximity_unsat_when_far() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let solver = Solver::new();
            let py1 = Real::new_const("py1");
            let py2 = Real::new_const("py2");
            solver.assert(&py1._eq(&Real::from_rational(0, 1)));
            solver.assert(&py2._eq(&Real::from_rational(50, 10)));
            solver.assert(&encode_y_proximity_constraint(&py1, &py2, 3.5));
            assert_eq!(solver.check(), SatResult::Unsat);
        });
    }

    #[test]
    fn test_same_lane_discrete_match_sat() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let solver = Solver::new();
            let lane1 = Int::new_const("lane1");
            let lane2 = Int::new_const("lane2");
            let py1 = Real::new_const("py1");
            let py2 = Real::new_const("py2");
            solver.assert(&lane1._eq(&Int::from_i64(2)));
            solver.assert(&lane2._eq(&Int::from_i64(2)));
            solver.assert(&py1._eq(&Real::from_rational(0, 1)));
            solver.assert(&py2._eq(&Real::from_rational(100, 1)));
            solver.assert(&encode_same_lane_constraint(&lane1, &lane2, &py1, &py2, 3.5));
            assert_eq!(solver.check(), SatResult::Sat);
        });
    }

    #[test]
    fn test_same_lane_proximity_match_sat() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let solver = Solver::new();
            let lane1 = Int::new_const("lane1");
            let lane2 = Int::new_const("lane2");
            let py1 = Real::new_const("py1");
            let py2 = Real::new_const("py2");
            solver.assert(&lane1._eq(&Int::from_i64(1)));
            solver.assert(&lane2._eq(&Int::from_i64(2)));
            solver.assert(&py1._eq(&Real::from_rational(10, 10)));
            solver.assert(&py2._eq(&Real::from_rational(20, 10)));
            solver.assert(&encode_same_lane_constraint(&lane1, &lane2, &py1, &py2, 3.5));
            assert_eq!(solver.check(), SatResult::Sat);
        });
    }

    #[test]
    fn test_same_lane_different_lanes_far_y_unsat() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let solver = Solver::new();
            let lane1 = Int::new_const("lane1");
            let lane2 = Int::new_const("lane2");
            let py1 = Real::new_const("py1");
            let py2 = Real::new_const("py2");
            solver.assert(&lane1._eq(&Int::from_i64(1)));
            solver.assert(&lane2._eq(&Int::from_i64(3)));
            solver.assert(&py1._eq(&Real::from_rational(0, 1)));
            solver.assert(&py2._eq(&Real::from_rational(50, 1)));
            solver.assert(&encode_same_lane_constraint(&lane1, &lane2, &py1, &py2, 3.5));
            assert_eq!(solver.check(), SatResult::Unsat);
        });
    }
}
