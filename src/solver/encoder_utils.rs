//! Shared utilities for coordinate encoders
//!
//! This module provides common helper functions used by both CartesianEncoder
//! and BicycleEncoder to reduce code duplication.

use std::collections::HashMap;
use z3::ast::{Bool, Int, Real};
use z3::Model;

use crate::dsl::types::{
    lane_change_start_past_horizon, ActorRole, LaneChangeDirection, ScenarioSpec,
};
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

/// Maximum number of digits kept after the decimal point by
/// [`rational_from_f64`].
///
/// Twelve is chosen to be wider than any literal a user writes in a
/// `ScenarioSpec` (positions, speeds and thresholds are quoted to one or two
/// decimals) while staying far inside `i64`: the largest denominator this can
/// produce is `10^12`, so any value up to ~9.2e6 in magnitude is always
/// representable at full precision, and gcd reduction means "nice" values
/// never get near that bound.
pub const MAX_DECIMAL_DIGITS: usize = 12;

/// Convert an `f64` to an exact `(numerator, denominator)` rational.
///
/// # Precision
///
/// The value is rendered with Rust's **shortest round-trip decimal**
/// representation and that decimal is converted exactly, then reduced by its
/// gcd. So `3.5` becomes `7/2`, `2.55` becomes `51/20` and `0.1` becomes
/// `1/10` — the number the user wrote, not a truncation of it.
///
/// This is deliberately *not* the exact binary value of the double. `2.55` as
/// an IEEE double is `2.549999999999999822...`, whose exact rational has a
/// 51-bit numerator over a power of two; feeding rationals of that size into
/// Z3 for every constant in the encoding costs solver time to represent an
/// artefact of binary floating point rather than anything the specification
/// says. The shortest round-trip decimal is the unique shortest decimal that
/// parses back to the same double, which is precisely "what the user typed".
///
/// Values that need more than [`MAX_DECIMAL_DIGITS`] digits after the point —
/// only ever intermediate results computed inside the encoder, never spec
/// literals — are **rounded** (not truncated) to that many digits, and the
/// precision is dropped further if the result would not fit in `i64`.
/// Non-finite inputs yield `(0, 1)`; `ScenarioSpec::validate` rejects them
/// upstream.
///
/// Replaces the previous `(x * 10.0) as i64 / 10` idiom, which truncated
/// *toward zero* — silently turning `min_ttc: 2.55` into 2.5 and, for negative
/// bounds, tightening rather than loosening them (`-2.75` into `-2.7`).
#[must_use]
pub fn rational_from_f64(value: f64) -> (i64, i64) {
    if !value.is_finite() {
        return (0, 1);
    }

    // Shortest round-trip decimal. `f64`'s `Display` never uses exponent
    // notation, but fall back to a fixed-precision rendering if that ever
    // changes so the digit-string parse below stays correct.
    let shortest = format!("{value}");
    let shortest = if shortest.contains(['e', 'E']) {
        format!("{:.*}", MAX_DECIMAL_DIGITS, value)
    } else {
        shortest
    };
    let frac_len = shortest.find('.').map_or(0, |dot| shortest.len() - dot - 1);
    let start = frac_len.min(MAX_DECIMAL_DIGITS);

    // Try the most precise rendering that fits in i64, giving up decimals one
    // at a time. Terminates: at zero decimals the numerator is the integer
    // part, and a finite f64 too large for i64 falls through to the clamp.
    for digits in (0..=start).rev() {
        // The shortest rendering is used as-is only when it is already within
        // the precision cap; otherwise it is re-rendered (and so rounded).
        let rendered = if digits == frac_len {
            shortest.clone()
        } else {
            format!("{:.*}", digits, value)
        };
        if let Some(pair) = parse_decimal_exact(&rendered) {
            return pair;
        }
    }

    // |value| exceeds i64::MAX; nothing sane reaches here.
    (value.clamp(i64::MIN as f64, i64::MAX as f64) as i64, 1)
}

/// Parse a plain decimal string into a gcd-reduced `(num, den)`, or `None` if
/// either part overflows `i64`.
fn parse_decimal_exact(s: &str) -> Option<(i64, i64)> {
    let negative = s.starts_with('-');
    let unsigned = s.trim_start_matches(['-', '+']);
    let (int_part, frac_part) = unsigned.split_once('.').unwrap_or((unsigned, ""));

    let mut digits = String::with_capacity(int_part.len() + frac_part.len());
    digits.push_str(int_part);
    digits.push_str(frac_part);
    let mut num: i128 = digits.parse().ok()?;
    let mut den: i128 = 10_i128.checked_pow(u32::try_from(frac_part.len()).ok()?)?;

    let divisor = gcd_i128(num, den);
    num /= divisor;
    den /= divisor;
    if negative {
        num = -num;
    }

    Some((i64::try_from(num).ok()?, i64::try_from(den).ok()?))
}

/// Greatest common divisor of two non-negative-after-abs `i128`s.
fn gcd_i128(a: i128, b: i128) -> i128 {
    let (mut a, mut b) = (a.abs(), b.abs());
    while b != 0 {
        let t = b;
        b = a % b;
        a = t;
    }
    if a == 0 {
        1
    } else {
        a
    }
}

/// Convert an `f64` into a Z3 `Real` exactly, via [`rational_from_f64`].
///
/// This is the single conversion point for every numeric constant that enters
/// the encoding. See [`rational_from_f64`] for the precision contract.
#[must_use]
pub fn real_from_f64(value: f64) -> Real {
    let (num, den) = rational_from_f64(value);
    // NOT `Real::from_rational`: despite its `i64` parameters that constructor
    // casts both to C `int` on the way to `Z3_mk_real`, so anything outside
    // i32 wraps silently. `pi/6` at full precision is 261799387799/500000000000
    // and came back out of Z3 as a wrapped-around number, turning
    // `bicycle_lane_change` UNSAT. `from_rational_str` goes through
    // `Z3_mk_numeral`, which parses the literal at arbitrary precision.
    Real::from_rational_str(&num.to_string(), &den.to_string()).unwrap_or_else(|| {
        // Unreachable: `Z3_mk_numeral` only rejects a malformed literal, and
        // "<i64> / <i64>" never is. The i32-clamped constructor is exact for
        // every value that fits it, which is the whole realistic range.
        Real::from_rational(num, den)
    })
}

/// Extract a real value from Z3 model
///
/// Handles rationals, with a fallback to Z3's decimal approximation for
/// values `Z3_get_numeral_small` cannot express in two `i64`s (an irrational
/// algebraic number, or a rational whose numerator or denominator overflows).
///
/// There used to be a second `if let Some((num, denom)) = ast.as_real()` branch
/// between the two below, repeating the same division. It was dead code:
/// `Real::as_real` is `#[deprecated]` and its body in z3 0.19.7 is exactly
/// `self.as_rational()`, so it re-called the function that had just returned
/// `None` one line above and could never itself return `Some`. That is why the
/// wave-1 mutation baseline found this function's division surviving mutation
/// to both `*` and `%` despite 26 live callers — the surviving mutants were in
/// the unreachable copy, and no test could have killed them. The reachable
/// division is pinned by `test_extract_real_division_not_product_or_remainder`.
pub fn extract_real(model: &Model, var: &Real) -> Result<f64> {
    let ast = model.eval(var, true).ok_or_else(|| {
        ScenarioGenError::Z3ModelParsing("Failed to evaluate real variable".to_string())
    })?;

    // Z3 hands back exact rationals for everything this encoder builds.
    if let Some((num, denom)) = ast.as_rational() {
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
                .filter_map(|lc| {
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

                    // Skip lane changes that begin beyond the scenario horizon.
                    // Must match the `>=` used by `ScenarioSpec::validate` and by
                    // `CartesianEncoder::encode_smooth_lane_transition`'s own
                    // `start_step >= self.horizon` guard — a bare `>` here would be
                    // one off from those. This call and `ScenarioSpec::validate`
                    // share `lane_change_start_past_horizon` and cannot drift apart;
                    // `cartesian.rs`'s copy is still a separate `>=` literal —
                    // numerically identical today, but not wired to the shared
                    // predicate.
                    if lane_change_start_past_horizon(start_step, horizon) {
                        return None;
                    }

                    let duration_steps = usize::midpoint(duration_steps_min, duration_steps_max);
                    let end_step = (start_step + duration_steps).min(horizon);

                    Some(LaneChangeSteps {
                        direction: lc.direction,
                        start_step,
                        end_step,
                    })
                })
                .collect();
            (a.id.clone(), changes)
        })
        .collect()
}

/// One non-pedestrian actor's initial-condition fields, collected up front so
/// `encode_initial_conditions` can iterate them without holding a borrow of
/// `spec` across the `&mut self` calls that assert each actor's constraints
/// (the two coordinate encoders would otherwise each build this same 10-tuple
/// from the same filter independently; this is the shared extraction).
pub struct VehicleInitialState {
    pub actor_id: String,
    pub lane: usize,
    pub pos_min: f64,
    pub pos_max: f64,
    pub speed_min: f64,
    pub speed_max: f64,
    pub accel_min: f64,
    pub accel_max: f64,
    pub role: ActorRole,
    pub direction: i32,
}

/// Collect [`VehicleInitialState`] for every non-pedestrian actor in `spec`.
///
/// Pedestrians are excluded because both encoders route them through the
/// separate `encode_pedestrian_initial_state` path instead of
/// `encode_actor_initial_state`.
#[must_use]
pub fn collect_vehicle_initial_state(spec: &ScenarioSpec) -> Vec<VehicleInitialState> {
    spec.actors
        .iter()
        .filter(|actor| actor.role != ActorRole::Pedestrian)
        .map(|actor| VehicleInitialState {
            actor_id: actor.id.clone(),
            lane: actor.lane,
            pos_min: actor.position.min(),
            pos_max: actor.position.max(),
            speed_min: actor.speed.min(),
            speed_max: actor.speed.max(),
            accel_min: actor.acceleration.min(),
            accel_max: actor.acceleration.max(),
            role: actor.role,
            direction: actor.direction,
        })
        .collect()
}

/// Rounding budget for evaluating the same-lane predicate in `f64`, in metres.
///
/// # The boundary, decided
///
/// The predicate is `lane1 == lane2 || |py1 - py2| < lane_width`, and the
/// boundary is **excluded**: two actors exactly one lane width apart laterally
/// are in *adjacent* lanes, not the same one, so they are not a conflict pair.
/// That is not an incidental case. `py = lane * lane_width + lane_width / 2`
/// puts every pair of neighbouring lane centres at exactly `lane_width`, so the
/// boundary is where the whole corpus sits whenever nobody is mid-manoeuvre.
///
/// # Which arithmetic is canonical
///
/// The **exact** one. [`encode_same_lane_constraint`] evaluates the predicate
/// over Z3's rationals and the solver's model is ground truth; everything else
/// — `Z3Encoder::compute_validation_metrics`, `tests/common/invariants.rs` —
/// is *checking* that model from an extracted trajectory that has already been
/// rounded to `f64`. So the exact side is left exactly as written, and the
/// `f64` side is the one that has to be taught to reproduce it.
///
/// # Why the `f64` side needs a band
///
/// A bare `f64` `<` against `lane_width` disagrees with the exact evaluation
/// *on the boundary*, and only there. At `lane_width = 3.2` the lane centres
/// are 1.6 and 4.8: exact arithmetic gets `3.2 < 3.2` = false and asserts no
/// `min_distance` for the pair, while `4.8 - 1.6` in `f64` is
/// 3.1999999999999997, gets `true`, and reports the untouched gap as a breach.
/// Measured on `examples/cut_in_left.yaml` with `lane_width = 3.2` at
/// `93f2ab2`: `min_distance` 2.02 m against an enforced 5.0 m,
/// `all_constraints_satisfied: false` — a constraint the solver never asserted,
/// reported as violated.
///
/// So [`same_lane_f64`] compares against `lane_width - LANE_OVERLAP_EPS`. This
/// is the same rounding budget, applied in the same one-sided way and for the
/// same reason, as `Z3Encoder::METRIC_TOL` and `tests/common/invariants.rs::TOL`:
/// the error it absorbs is rational-to-double rounding (~1e-15 here), nine
/// orders below the band, not modelling slack.
///
/// # The residual, stated
///
/// The band makes the `f64` overlap set a strict *subset* of the exact one:
/// they differ only for pairs whose lateral separation lies in
/// `[lane_width - 1e-6, lane_width)`. That direction is deliberate. The
/// validator can no longer report a breach on a pair the encoder left
/// unconstrained, which is the defect above; what remains is that a `violate`
/// scenario whose *only* conflicting steps sat inside a one-micrometre lateral
/// band would go unreported — a state no example produces and none could
/// meaningfully distinguish from adjacent lanes.
///
/// Shifting the exact side by the same epsilon was measured as the alternative
/// and rejected: it turns every lane-width literal Z3 sees from `7/2` into
/// `3499999/1000000` (against the whole rationale of [`MAX_DECIMAL_DIGITS`]),
/// and it churned the model of nearly every example in `examples/` — a
/// corpus-wide output change to fix a boundary nothing but the `f64` reader
/// ever got wrong.
pub const LANE_OVERLAP_EPS: f64 = 1e-6;

/// The lateral distance below which two actors count as sharing a lane, as the
/// `f64` side must test it.
///
/// See [`LANE_OVERLAP_EPS`]: the predicate's threshold is `lane_width` with the
/// boundary excluded, and this is that threshold minus the `f64` rounding
/// budget. Every `f64` evaluation of the predicate must go through
/// [`same_lane_f64`] rather than re-typing the comparison.
#[must_use]
pub fn lane_overlap_threshold(lane_width: f64) -> f64 {
    lane_width - LANE_OVERLAP_EPS
}

/// The `f64` twin of [`encode_same_lane_constraint`], for code holding an
/// extracted trajectory rather than a Z3 model.
///
/// `lane1 == lane2 || |py1 - py2| < lane_overlap_threshold(lane_width)` — the
/// same disjunction and the same boundary policy as the exact side, so the
/// validator cannot report on a pair the encoder left unconstrained. See
/// [`LANE_OVERLAP_EPS`] for the boundary decision and the residual.
#[must_use]
pub fn same_lane_f64(lane1: usize, lane2: usize, py1: f64, py2: f64, lane_width: f64) -> bool {
    lane1 == lane2 || (py1 - py2).abs() < lane_overlap_threshold(lane_width)
}

/// Encode "same lane" check for two actors using y-position proximity
///
/// This function creates a Z3 Bool that is true when two actors are in the
/// same lateral space (i.e., |py1 - py2| < lane_width, boundary excluded).
/// This is the **canonical** evaluation of the predicate, in exact rationals;
/// [`same_lane_f64`] is the `f64` twin that has to agree with it, and
/// [`LANE_OVERLAP_EPS`] documents the boundary and how the two are kept in
/// step.
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
/// - This correctly requires the actual distance to be less than the threshold
pub fn encode_y_proximity_constraint(py1: &Real, py2: &Real, lane_width: f64) -> Bool {
    // Strict, against `lane_width` itself: exact rationals need no guard band,
    // and adjacent lane centres — which sit at exactly `lane_width` — are
    // excluded by the strictness. `same_lane_f64` is what has to work to get
    // the same answer. See `LANE_OVERLAP_EPS`.
    let lane_width_real = real_from_f64(lane_width);
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
/// 2. Having lateral positions strictly within one lane width of each other
///
/// [`same_lane_f64`] is the `f64` twin of this predicate; the two must stay in
/// lockstep — see [`LANE_OVERLAP_EPS`].
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

    // -----------------------------------------------------------------
    // rational_from_f64 / real_from_f64
    // -----------------------------------------------------------------

    #[test]
    fn test_rational_exact_for_spec_literals() {
        // Every one of these is a value the old `(x * 10.0) as i64 / 10`
        // idiom got wrong or represented at an inconsistent precision.
        for (value, expected) in [
            (3.5_f64, (7_i64, 2_i64)), // lane_width
            (1.75, (7, 4)),            // lane_width / 2 — was 1.7
            (3.25, (13, 4)),           // was 3.2
            (2.55, (51, 20)),          // min_ttc — was 2.5
            (60.07, (6007, 100)),      // position — was 60.0
            (0.1, (1, 10)),            // time_step
            (0.05, (1, 20)),           // time_step / 2
            (16.0, (16, 1)),
            (0.0, (0, 1)),
            (-0.0, (0, 1)),
        ] {
            assert_eq!(
                rational_from_f64(value),
                expected,
                "rational_from_f64({value})"
            );
        }
    }

    #[test]
    fn test_rational_negative_bounds_are_not_tightened() {
        // The old truncation moved negative bounds *inward*: -2.75 became
        // -2.7, a constraint tighter than the user asked for.
        assert_eq!(rational_from_f64(-2.75), (-11, 4));
        assert_eq!(rational_from_f64(-8.0), (-8, 1));
        assert_eq!(rational_from_f64(-0.15), (-3, 20));
        let (num, den) = rational_from_f64(-2.75);
        assert!((num as f64 / den as f64) <= -2.75, "bound must not tighten");
    }

    #[test]
    fn test_rational_round_trips_to_the_same_double() {
        for value in [
            3.5,
            3.25,
            2.55,
            -2.75,
            0.1,
            0.3,
            60.07,
            1e-6,
            1234.5678,
            -0.000_000_5,
        ] {
            let (num, den) = rational_from_f64(value);
            assert!(
                ((num as f64 / den as f64) - value).abs() <= value.abs() * 1e-12 + 1e-12,
                "{value} round-tripped as {num}/{den}"
            );
        }
    }

    /// Pins the fractional-digit count in `rational_from_f64`.
    ///
    /// Most values cannot detect an error there: when the count is at most
    /// `MAX_DECIMAL_DIGITS` the loop's first iteration reuses the shortest
    /// rendering verbatim, so any miscount is masked. This value is chosen so
    /// it cannot be: its shortest representation has 13 fractional digits, one
    /// past the cap, so the reuse shortcut cannot fire on the first iteration
    /// and the digit count actually selects the rendering precision. A count
    /// computed as `len / dot` instead of `len - dot` yields 3 here rather
    /// than 13, i.e. `1234.568` — off by 1.1e-4, far outside the tolerance.
    #[test]
    fn test_rational_digit_count_selects_the_rendering_precision() {
        let value = 1234.567_890_123_456_7_f64;
        assert_eq!(
            format!("{value}").split('.').nth(1).map(str::len),
            Some(13),
            "this test only bites while the value has one more fractional digit than the cap"
        );

        let (num, den) = rational_from_f64(value);
        let got = num as f64 / den as f64;
        assert!(
            (got - value).abs() < 1e-11,
            "{value} came back as {num}/{den} = {got}"
        );
        assert!(
            den > 1,
            "a fractional value must not be rounded to a whole number ({num}/{den})"
        );
    }

    #[test]
    fn test_rational_is_reduced() {
        // gcd reduction keeps the rationals handed to Z3 small.
        assert_eq!(rational_from_f64(2.5), (5, 2));
        assert_eq!(rational_from_f64(100.0), (100, 1));
        assert_eq!(rational_from_f64(0.125), (1, 8));
    }

    #[test]
    fn test_rational_beyond_max_decimals_is_rounded_not_truncated() {
        // 1/3 needs more digits than MAX_DECIMAL_DIGITS; the result is the
        // rounded 12-digit value, and rounding (not truncation) means the
        // error is at most half an ulp of the last kept digit.
        let (num, den) = rational_from_f64(1.0 / 3.0);
        assert!(den <= 10_i64.pow(MAX_DECIMAL_DIGITS as u32));
        assert!(((num as f64 / den as f64) - 1.0 / 3.0).abs() < 5e-13);

        // Rounding up, where truncation would have gone the other way.
        let (num, den) = rational_from_f64(0.999_999_999_999_9);
        assert!((num as f64 / den as f64 - 1.0).abs() < 1e-12);
    }

    #[test]
    fn test_rational_handles_non_finite_and_huge() {
        assert_eq!(rational_from_f64(f64::NAN), (0, 1));
        assert_eq!(rational_from_f64(f64::INFINITY), (0, 1));
        assert_eq!(rational_from_f64(f64::NEG_INFINITY), (0, 1));
        // Larger than i64 can hold at any precision: falls through to the
        // clamp rather than panicking or wrapping.
        let (_, den) = rational_from_f64(1e30);
        assert_eq!(den, 1);
    }

    #[test]
    fn test_real_from_f64_matches_the_rational_in_z3() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let solver = Solver::new();
            let x = Real::new_const("x");
            solver.assert(&x._eq(&real_from_f64(2.55)));
            // 2.55 exactly, not the 2.5 the old truncation produced.
            solver.assert(&x._eq(&Real::from_rational(51, 20)));
            assert_eq!(solver.check(), SatResult::Sat);
            let model = solver.get_model().unwrap();
            assert!((extract_real(&model, &x).unwrap() - 2.55).abs() < 1e-12);
        });
    }

    #[test]
    fn test_real_from_f64_lane_centres_are_exact() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let solver = Solver::new();
            // py = lane * lane_width + lane_width / 2, the cartesian encoder's
            // lane-position coupling, for lane_width = 3.5 and lane_width = 3.25.
            for (lane_width, lane, centre) in
                [(3.5_f64, 0_i64, 1.75_f64), (3.5, 1, 5.25), (3.25, 1, 4.875)]
            {
                let py = Real::new_const(format!("py_{lane_width}_{lane}"));
                let expected = Real::from_int(&Int::from_i64(lane)) * real_from_f64(lane_width)
                    + real_from_f64(lane_width / 2.0);
                solver.assert(&py._eq(&expected));
                assert_eq!(solver.check(), SatResult::Sat);
                let model = solver.get_model().unwrap();
                let got = extract_real(&model, &py).unwrap();
                assert!(
                    (got - centre).abs() < 1e-12,
                    "lane {lane} of width {lane_width}: got {got}, want {centre}"
                );
            }
        });
    }

    /// `Real::from_rational` takes `i64` but casts both arguments to C `int`
    /// on the way to `Z3_mk_real`, so a rational outside i32 wraps silently.
    /// `real_from_f64` must not do that: `pi/6` reduces to
    /// 261799387799/500000000000, and building it with `from_rational` turned
    /// `bicycle_lane_change` UNSAT because the heading bound came back as a
    /// wrapped-around number.
    #[test]
    fn test_real_from_f64_survives_denominators_beyond_i32() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            for value in [
                std::f64::consts::PI / 6.0, // 261799387799/500000000000
                1.0 / 3.0,
                15.0 * 0.6 / 2.7 * 0.1, // bicycle max heading change per step
                20.0 * 0.6 / 2.7 * 0.1,
            ] {
                let (num, den) = rational_from_f64(value);
                assert!(
                    den > i64::from(i32::MAX) || num.abs() > i64::from(i32::MAX),
                    "{value} = {num}/{den} does not exercise the i32 boundary"
                );
                let solver = Solver::new();
                let x = Real::new_const("x");
                solver.assert(&x._eq(&real_from_f64(value)));
                assert_eq!(solver.check(), SatResult::Sat);
                let model = solver.get_model().unwrap();
                let got = extract_real(&model, &x).unwrap();
                assert!(
                    (got - value).abs() < 1e-11,
                    "{value} came back from Z3 as {got}"
                );
            }
        });
    }

    // -----------------------------------------------------------------
    // extract_real
    // -----------------------------------------------------------------

    /// `extract_real` divides numerator by denominator. The wave-1 mutation
    /// baseline found that division surviving mutation to both `*` and `%`
    /// despite 26 live callers, because every existing test used a value
    /// where the three operators are hard to tell apart or the branch was
    /// never reached. These cases separate them: for 7/3, `/` gives 2.333…,
    /// `*` gives 21 and `%` gives 1.
    #[test]
    fn test_extract_real_division_not_product_or_remainder() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            for (num, den, expected) in [
                (7_i32, 3_i32, 7.0 / 3.0),
                (1, 8, 0.125),
                (-51, 20, -2.55),
                (0, 7, 0.0),
                (5, 1, 5.0),
            ] {
                let solver = Solver::new();
                let x = Real::new_const("x");
                solver.assert(&x._eq(&Real::from_rational(num.into(), den.into())));
                assert_eq!(solver.check(), SatResult::Sat);
                let model = solver.get_model().unwrap();
                let got = extract_real(&model, &x).unwrap();
                assert!(
                    (got - expected).abs() < 1e-12,
                    "{num}/{den}: got {got}, want {expected}"
                );
            }
        });
    }

    /// `extract_real` on a value the solver derives rather than one asserted
    /// verbatim, so the model holds a computed rational.
    #[test]
    fn test_extract_real_derived_rational() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let solver = Solver::new();
            let x = Real::new_const("x");
            let y = Real::new_const("y");
            // 3x = 1  and  y = x + 1/4  =>  x = 1/3, y = 7/12
            solver.assert(&(&x * Real::from_rational(3, 1))._eq(&Real::from_rational(1, 1)));
            solver.assert(&y._eq(&(&x + Real::from_rational(1, 4))));
            assert_eq!(solver.check(), SatResult::Sat);
            let model = solver.get_model().unwrap();
            assert!((extract_real(&model, &x).unwrap() - 1.0 / 3.0).abs() < 1e-12);
            assert!((extract_real(&model, &y).unwrap() - 7.0 / 12.0).abs() < 1e-12);
        });
    }

    /// Round-trip: an `f64` through [`real_from_f64`] and back out through
    /// [`extract_real`] is the same number.
    #[test]
    fn test_real_from_f64_extract_real_round_trip() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            for value in [3.5_f64, 1.75, 3.25, 2.55, -2.75, 0.1, 60.07, -0.15, 0.0] {
                let solver = Solver::new();
                let x = Real::new_const("x");
                solver.assert(&x._eq(&real_from_f64(value)));
                assert_eq!(solver.check(), SatResult::Sat);
                let model = solver.get_model().unwrap();
                let got = extract_real(&model, &x).unwrap();
                assert!(
                    (got - value).abs() < 1e-12,
                    "{value} round-tripped as {got}"
                );
            }
        });
    }

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

    /// The function's contract is `|py1 - py2| < lane_width`, and the two
    /// subtractions are what implement it. The sat/unsat pair below does not
    /// pin them: at `py1 = 1.0, py2 = 2.0` the sum (3.0) is also under a 3.5 m
    /// lane width, so mutating either `-` to `+` still passes. `py1 = 4.0,
    /// py2 = 1.0` separates them — the difference is 3.0 (same lane) while the
    /// sum is 5.0 and the quotient 4.0, both outside the lane width, so this
    /// case is SAT only for the real operator.
    #[test]
    fn test_y_proximity_pins_the_difference_not_the_sum_or_quotient() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let solver = Solver::new();
            let py1 = Real::new_const("py1");
            let py2 = Real::new_const("py2");
            solver.assert(&py1.eq(&real_from_f64(4.0)));
            solver.assert(&py2.eq(&real_from_f64(1.0)));
            solver.assert(&encode_y_proximity_constraint(&py1, &py2, 3.5));
            assert_eq!(
                solver.check(),
                SatResult::Sat,
                "|4.0 - 1.0| = 3.0 is inside a 3.5 m lane width"
            );
        });
    }

    /// The mirror image: the difference is outside the lane width while the
    /// quotient is inside it, so this is UNSAT only for the real operator.
    #[test]
    fn test_y_proximity_unsat_when_difference_is_wide_but_quotient_is_not() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let solver = Solver::new();
            let py1 = Real::new_const("py1");
            let py2 = Real::new_const("py2");
            solver.assert(&py1.eq(&real_from_f64(8.0)));
            solver.assert(&py2.eq(&real_from_f64(4.0)));
            solver.assert(&encode_y_proximity_constraint(&py1, &py2, 3.5));
            assert_eq!(
                solver.check(),
                SatResult::Unsat,
                "|8.0 - 4.0| = 4.0 is outside a 3.5 m lane width, though 8/4 = 2 is not"
            );
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
            solver.assert(&encode_same_lane_constraint(
                &lane1, &lane2, &py1, &py2, 3.5,
            ));
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
            solver.assert(&encode_same_lane_constraint(
                &lane1, &lane2, &py1, &py2, 3.5,
            ));
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
            solver.assert(&encode_same_lane_constraint(
                &lane1, &lane2, &py1, &py2, 3.5,
            ));
            assert_eq!(solver.check(), SatResult::Unsat);
        });
    }

    /// The exact predicate and its `f64` twin must return the same
    /// answer for every lane centre in the corpus's lane-width range —
    /// including the widths where `f64` subtraction of two lane centres
    /// undershoots (3.2 is the one that broke: `4.8 - 1.6` is
    /// 3.1999999999999997, so a bare `<` called adjacent lanes "same lane"
    /// while Z3, in exact rationals, did not).
    #[test]
    fn test_same_lane_f64_agrees_with_exact_at_every_lane_centre() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            for width_tenths in 25_i64..=45 {
                let w = width_tenths as f64 / 10.0;
                for l1 in 0_usize..4 {
                    for l2 in 0_usize..4 {
                        let c1 = l1 as f64 * w + w / 2.0;
                        let c2 = l2 as f64 * w + w / 2.0;

                        let solver = Solver::new();
                        let lane1 = Int::new_const("lane1");
                        let lane2 = Int::new_const("lane2");
                        let py1 = Real::new_const("py1");
                        let py2 = Real::new_const("py2");
                        solver.assert(&lane1._eq(&Int::from_i64(l1 as i64)));
                        solver.assert(&lane2._eq(&Int::from_i64(l2 as i64)));
                        // The exact lane centre, as the encoder pins it:
                        // lane * w + w/2, not the f64 rounding of the product.
                        solver.assert(&py1._eq(
                            &(Real::from_int(&Int::from_i64(l1 as i64)) * real_from_f64(w)
                                + real_from_f64(w / 2.0)),
                        ));
                        solver.assert(&py2._eq(
                            &(Real::from_int(&Int::from_i64(l2 as i64)) * real_from_f64(w)
                                + real_from_f64(w / 2.0)),
                        ));
                        solver.assert(&encode_same_lane_constraint(&lane1, &lane2, &py1, &py2, w));
                        let exact = solver.check() == SatResult::Sat;

                        assert_eq!(
                            exact,
                            same_lane_f64(l1, l2, c1, c2, w),
                            "lane_width {w}: lanes {l1} (py {c1}) and {l2} (py {c2}) — \
                             exact says {exact}, f64 disagrees"
                        );
                    }
                }
            }
        });
    }

    /// The boundary itself, stated as a test: exactly one lane width apart is
    /// *adjacent*, not same-lane, on both sides of the tool.
    #[test]
    fn test_same_lane_f64_excludes_exactly_one_lane_width() {
        assert!(
            !same_lane_f64(0, 1, 1.6, 4.8, 3.2),
            "|4.8 - 1.6| is one lane width at 3.2 m: adjacent lanes, not a conflict pair"
        );
        assert!(
            same_lane_f64(0, 1, 1.6, 4.79, 3.2),
            "10 cm inside the lane width is an overlap"
        );
        assert!(
            same_lane_f64(2, 2, 0.0, 100.0, 3.2),
            "a discrete lane match is same-lane regardless of py"
        );
    }

    #[test]
    fn test_collect_lane_change_data_filters_beyond_horizon() {
        // Lane change starts at 6.0s with time_step=0.5 → start_step = 12
        // With horizon=10, start_step > horizon so filter_map returns None
        let lc = LaneChangeConfig {
            direction: LaneChangeDirection::Right,
            start_time: ValueOrRange::Value(6.0),
            duration: ValueOrRange::Value(1.0),
        };
        let mut spec = make_spec(vec![make_actor("npc1", ActorRole::Npc, vec![lc])]);
        spec.time_step = 0.5;
        let result = collect_lane_change_data(&spec, 10);
        // Actor appears in result (has lane_change config) but vec is empty
        // because start_step = 6.0/0.5 = 12 > 10 (horizon)
        assert!(
            result["npc1"].is_empty(),
            "Lane change starting at step 12 should be filtered when horizon=10"
        );
    }
}
