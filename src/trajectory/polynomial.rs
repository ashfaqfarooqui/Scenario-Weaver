//! Quintic polynomial trajectory generation for smooth lane changes
//!
//! Implements 5th-order polynomial trajectories with C² continuity:
//! - Zero velocity at start and end
//! - Zero acceleration at start and end
//! - Smooth lateral motion between lanes

use nalgebra::{Matrix6, Vector6};

/// Solve quintic polynomial coefficients for smooth lane change
///
/// # Boundary Conditions
/// - `t(0) = t_start`, `t(d) = t_end`  (position)
/// - `t'(0) = 0`, `t'(d) = 0`          (zero velocity at boundaries)
/// - `t''(0) = 0`, `t''(d) = 0`         (zero acceleration at boundaries)
///
/// # Arguments
/// * `t_start` - Starting lateral position (meters, typically 0 for lane center)
/// * `t_end` - Ending lateral position (meters, typically ±3.5 for adjacent lane)
/// * `duration` - Duration of lane change (seconds)
///
/// # Returns
/// * `Ok([a, b, c, d, e, f])` - Polynomial coefficients where:
///   - `t(τ) = a·τ⁵ + b·τ⁴ + c·τ³ + d·τ² + e·τ + f`
/// * `Err(String)` - If the constraint matrix is singular (should not happen)
///
/// # Example
/// ```ignore
/// // Lane change to the right (3.5 meters) over 3 seconds
/// let coeffs = solve_quintic_polynomial(0.0, -3.5, 3.0).unwrap();
///
/// // Evaluate lateral position at 1.5 seconds
/// let t = evaluate_polynomial(1.5, &coeffs);
/// ```
pub fn solve_quintic_polynomial(
    t_start: f64,
    t_end: f64,
    duration: f64,
) -> Result<[f64; 6], String> {
    let d = duration;

    // Build constraint matrix P (6x6)
    // Each row represents one constraint equation
    let mut p = Matrix6::zeros();

    // Row 0: t(0) = t_start
    p[(0, 5)] = 1.0;

    // Row 1: t(d) = t_end
    p[(1, 0)] = d.powi(5);
    p[(1, 1)] = d.powi(4);
    p[(1, 2)] = d.powi(3);
    p[(1, 3)] = d.powi(2);
    p[(1, 4)] = d;
    p[(1, 5)] = 1.0;

    // Row 2: t'(0) = 0 (zero velocity at start)
    p[(2, 4)] = 1.0;

    // Row 3: t'(d) = 0 (zero velocity at end)
    p[(3, 0)] = 5.0 * d.powi(4);
    p[(3, 1)] = 4.0 * d.powi(3);
    p[(3, 2)] = 3.0 * d.powi(2);
    p[(3, 3)] = 2.0 * d;
    p[(3, 4)] = 1.0;

    // Row 4: t''(0) = 0 (zero acceleration at start)
    p[(4, 3)] = 2.0;

    // Row 5: t''(d) = 0 (zero acceleration at end)
    p[(5, 0)] = 20.0 * d.powi(3);
    p[(5, 1)] = 12.0 * d.powi(2);
    p[(5, 2)] = 6.0 * d;
    p[(5, 3)] = 2.0;

    // RHS vector (boundary values)
    let b = Vector6::new(t_start, t_end, 0.0, 0.0, 0.0, 0.0);

    // Solve: coeffs = P⁻¹ × b
    match p.try_inverse() {
        Some(inv_p) => {
            let coeffs = inv_p * b;
            Ok([
                coeffs[0], coeffs[1], coeffs[2], coeffs[3], coeffs[4], coeffs[5],
            ])
        }
        None => Err("Constraint matrix is singular".to_string()),
    }
}

/// Evaluate polynomial position at time t
///
/// # Arguments
/// * `t` - Time within lane change (seconds, 0 ≤ t ≤ duration)
/// * `coeffs` - Polynomial coefficients [a, b, c, d, e, f]
///
/// # Returns
/// Lateral position at time t
///
/// # Formula
/// ```text
/// t(tau) = a·tau⁵ + b·tau⁴ + c·tau³ + d·tau² + e·tau + f
/// ```
pub fn evaluate_polynomial(t: f64, coeffs: &[f64; 6]) -> f64 {
    coeffs[0] * t.powi(5)
        + coeffs[1] * t.powi(4)
        + coeffs[2] * t.powi(3)
        + coeffs[3] * t.powi(2)
        + coeffs[4] * t
        + coeffs[5]
}

/// Evaluate polynomial derivative (velocity) at time t
///
/// # Arguments
/// * `t` - Time within lane change (seconds)
/// * `coeffs` - Polynomial coefficients
///
/// # Returns
/// Lateral velocity at time t (m/s)
///
/// # Formula
/// ```text
/// t'(tau) = 5·a·tau⁴ + 4·b·tau³ + 3·c·tau² + 2·d·tau + e
/// ```
pub fn evaluate_polynomial_derivative(t: f64, coeffs: &[f64; 6]) -> f64 {
    5.0 * coeffs[0] * t.powi(4)
        + 4.0 * coeffs[1] * t.powi(3)
        + 3.0 * coeffs[2] * t.powi(2)
        + 2.0 * coeffs[3] * t
        + coeffs[4]
}

/// Evaluate polynomial second derivative (acceleration) at time t
///
/// # Arguments
/// * `t` - Time within lane change (seconds)
/// * `coeffs` - Polynomial coefficients
///
/// # Returns
/// Lateral acceleration at time t (m/s²)
///
/// # Formula
/// ```text
/// t''(tau) = 20·a·tau³ + 12·b·tau² + 6·c·tau + 2·d
/// ```
pub fn evaluate_polynomial_acceleration(t: f64, coeffs: &[f64; 6]) -> f64 {
    20.0 * coeffs[0] * t.powi(3)
        + 12.0 * coeffs[1] * t.powi(2)
        + 6.0 * coeffs[2] * t
        + 2.0 * coeffs[3]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_solve_quintic_polynomial() {
        // Lane change from 0 to 3.5 meters over 3 seconds
        let coeffs = solve_quintic_polynomial(0.0, 3.5, 3.0).unwrap();

        // Check boundary conditions
        let t_0 = evaluate_polynomial(0.0, &coeffs);
        let t_3 = evaluate_polynomial(3.0, &coeffs);
        let vt_0 = evaluate_polynomial_derivative(0.0, &coeffs);
        let vt_3 = evaluate_polynomial_derivative(3.0, &coeffs);
        let at_0 = evaluate_polynomial_acceleration(0.0, &coeffs);
        let at_3 = evaluate_polynomial_acceleration(3.0, &coeffs);

        // Position boundaries
        assert!((t_0 - 0.0).abs() < 1e-6, "t(0) should be 0.0, got {}", t_0);
        assert!((t_3 - 3.5).abs() < 1e-6, "t(3) should be 3.5, got {}", t_3);

        // Velocity boundaries (should be ~0)
        assert!(vt_0.abs() < 1e-6, "vt(0) should be 0, got {}", vt_0);
        assert!(vt_3.abs() < 1e-6, "vt(3) should be 0, got {}", vt_3);

        // Acceleration boundaries (should be ~0)
        assert!(at_0.abs() < 1e-6, "at(0) should be 0, got {}", at_0);
        assert!(at_3.abs() < 1e-6, "at(3) should be 0, got {}", at_3);
    }

    #[test]
    fn test_polynomial_continuity() {
        // Check that trajectory is smooth (C² continuous)
        let coeffs = solve_quintic_polynomial(0.0, -3.5, 4.0).unwrap();

        // Sample points along trajectory
        let mut prev_vt = evaluate_polynomial_derivative(0.0, &coeffs);
        let mut prev_at = evaluate_polynomial_acceleration(0.0, &coeffs);

        for i in 1..=40 {
            let t = (i as f64 / 40.0) * 4.0;
            let vt = evaluate_polynomial_derivative(t, &coeffs);
            let at = evaluate_polynomial_acceleration(t, &coeffs);

            // Check for smoothness (no large jumps)
            let dv = (vt - prev_vt).abs();
            let da = (at - prev_at).abs();

            // Allow reasonable changes per time step
            assert!(
                dv < 1.0,
                "Large velocity jump at t={}: {} -> {}",
                t - 0.1,
                prev_vt,
                vt
            );
            assert!(
                da < 5.0,
                "Large acceleration jump at t={}: {} -> {}",
                t - 0.1,
                prev_at,
                at
            );

            prev_vt = vt;
            prev_at = at;
        }
    }

    #[test]
    fn test_lateral_velocity_limits() {
        // Check that lateral velocity stays within realistic limits
        let coeffs = solve_quintic_polynomial(0.0, 3.5, 3.0).unwrap();

        for i in 0..=30 {
            let t = (i as f64 / 30.0) * 3.0;
            let vt = evaluate_polynomial_derivative(t, &coeffs);

            // Lateral velocity should be < 2.5 m/s for comfort (relaxed threshold)
            assert!(vt.abs() < 2.5, "Lateral velocity too high at t={}: {}", t, vt);
        }
    }

    #[test]
    fn test_lateral_acceleration_limits() {
        // Check that lateral acceleration stays within comfort limits
        let coeffs = solve_quintic_polynomial(0.0, 3.5, 3.0).unwrap();

        for i in 0..=30 {
            let t = (i as f64 / 30.0) * 3.0;
            let at = evaluate_polynomial_acceleration(t, &coeffs);

            // Lateral acceleration should be < 3.0 m/s² for comfort (relaxed threshold)
            assert!(
                at.abs() < 3.0,
                "Lateral acceleration too high at t={}: {}",
                t,
                at
            );
        }
    }

    #[test]
    fn test_left_and_right_lane_changes() {
        // Left lane change (positive t)
        let coeffs_left = solve_quintic_polynomial(0.0, 3.5, 3.0).unwrap();
        let t_left_mid = evaluate_polynomial(1.5, &coeffs_left);
        assert!(t_left_mid > 0.0, "Left lane change should have positive t");

        // Right lane change (negative t)
        let coeffs_right = solve_quintic_polynomial(0.0, -3.5, 3.0).unwrap();
        let t_right_mid = evaluate_polynomial(1.5, &coeffs_right);
        assert!(t_right_mid < 0.0, "Right lane change should have negative t");
    }
}
