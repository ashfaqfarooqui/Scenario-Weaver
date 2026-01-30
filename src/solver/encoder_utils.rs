//! Shared utilities for coordinate encoders
//!
//! This module provides common helper functions used by both CartesianEncoder
//! and BicycleEncoder to reduce code duplication.

use std::collections::HashMap;
use z3::ast::{Bool, Int, Real};
use z3::Model;

use crate::dsl::types::{ActorRole, LaneChangeDirection, ScenarioSpec};
use crate::error::{Result, ScenarioGenError};

/// Parsed lane change data for an actor
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

/// Extract an integer value from Z3 model
pub fn extract_int(model: &Model, var: &Int) -> Result<usize> {
    let ast = model.eval(var, true).ok_or_else(|| {
        ScenarioGenError::Z3ModelParsing("Failed to evaluate int variable".to_string())
    })?;

    if let Some(val) = ast.as_i64() {
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
                    let start_step = (start_step_min + start_step_max) / 2;
                    let duration_steps = (duration_steps_min + duration_steps_max) / 2;
                    let end_step = (start_step + duration_steps).min(horizon);

                    LaneChangeSteps {
                        direction: lc.direction.clone(),
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
}
