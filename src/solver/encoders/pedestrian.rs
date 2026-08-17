//! Shared pedestrian encoder helpers
//!
//! Provides reusable functions for encoding pedestrian dynamics in Z3.
//! Pedestrians use a simple 2D point-mass model (no steering, no heading).
//! Both CartesianEncoder and BicycleEncoder can call these helpers.

use z3::ast::{Int, Real};
use z3::Model;

use crate::dsl::types::{
    ActorSpec, PEDESTRIAN_MAX_ACCELERATION, PEDESTRIAN_MAX_DECELERATION,
    PEDESTRIAN_RUN_MAX_SPEED, PEDESTRIAN_WALK_MAX_SPEED,
};
use crate::error::Result;
use crate::scenario::model::{
    Acceleration, ActorTrajectory, CartesianState, Position, State, Velocity,
};
use crate::solver::backend::Z3Backend;
use crate::solver::encoder_utils::{extract_int, extract_real};

/// Encode the initial state constraints for a pedestrian at t=0.
///
/// - `px[0]`: range or fixed from `actor.position`
/// - `py[0]`: computed from `actor.lane * lane_width + lane_width / 2.0`
/// - `vx[0]`: bounded by `[-max_speed, +max_speed]` (pedestrians aren't locked to a direction)
/// - `vy[0]`: left UNCONSTRAINED (pedestrian may already be crossing)
/// - `ax[0]`: bounded by actor acceleration range, clamped to pedestrian limits
/// - `ay[0]`: left UNCONSTRAINED
#[allow(clippy::too_many_arguments)]
pub fn encode_pedestrian_initial_state<B: Z3Backend>(
    backend: &B,
    px: &[Real],
    py: &[Real],
    vx: &[Real],
    _vy: &[Real],
    ax: &[Real],
    _ay: &[Real],
    actor: &ActorSpec,
    lane_width: f64,
) {
    // px[0]: position range or fixed
    let pos_min = actor.position.min();
    let pos_max = actor.position.max();
    if (pos_min - pos_max).abs() < 1e-6 {
        let pos_val = Real::from_rational((pos_min * 10.0) as i64, 10_i64);
        backend.assert(&px[0].eq(&pos_val));
    } else {
        let min_val = Real::from_rational((pos_min * 10.0) as i64, 10_i64);
        let max_val = Real::from_rational((pos_max * 10.0) as i64, 10_i64);
        backend.assert(&px[0].ge(&min_val));
        backend.assert(&px[0].le(&max_val));
    }

    // py[0]: lateral position from lane center
    let py_initial = actor.lane as f64 * lane_width + lane_width / 2.0;
    let py_val = Real::from_rational((py_initial * 100.0).round() as i64, 100_i64);
    backend.assert(&py[0].eq(&py_val));

    // vx[0]: bounded by [-max_speed, +max_speed]
    let max_speed = actor
        .behavior
        .get("walking_mode")
        .map_or(PEDESTRIAN_WALK_MAX_SPEED, |mode| match mode.as_str() {
            Some("run") => PEDESTRIAN_RUN_MAX_SPEED,
            _ => PEDESTRIAN_WALK_MAX_SPEED,
        });
    let max_speed_real = Real::from_rational((max_speed * 10.0) as i64, 10_i64);
    let neg_max_speed_real = Real::from_rational(((-max_speed) * 10.0) as i64, 10_i64);
    backend.assert(&vx[0].ge(&neg_max_speed_real));
    backend.assert(&vx[0].le(&max_speed_real));

    // vy[0]: UNCONSTRAINED (pedestrian may already be crossing)

    // ax[0]: bounded by actor acceleration range, clamped to pedestrian limits
    let accel_min = actor.acceleration.min().max(PEDESTRIAN_MAX_DECELERATION);
    let accel_max = actor.acceleration.max().min(PEDESTRIAN_MAX_ACCELERATION);
    if (accel_min - accel_max).abs() < 1e-6 {
        let accel_val = Real::from_rational((accel_min * 10.0) as i64, 10_i64);
        backend.assert(&ax[0].eq(&accel_val));
    } else {
        let min_val = Real::from_rational((accel_min * 10.0) as i64, 10_i64);
        let max_val = Real::from_rational((accel_max * 10.0) as i64, 10_i64);
        backend.assert(&ax[0].ge(&min_val));
        backend.assert(&ax[0].le(&max_val));
    }

    // ay[0]: UNCONSTRAINED
}

/// Encode one kinematics timestep for a pedestrian (simple 2D point-mass).
///
/// Asserts:
/// - `px_t1 = px_t + vx_t * dt`
/// - `py_t1 = py_t + vy_t * dt`
/// - `vx_t1 = vx_t + ax_t * dt`
/// - `vy_t1 = vy_t + ay_t * dt`
#[allow(clippy::too_many_arguments)]
pub fn encode_pedestrian_kinematics_step<B: Z3Backend>(
    backend: &B,
    px_t: &Real,
    px_t1: &Real,
    py_t: &Real,
    py_t1: &Real,
    vx_t: &Real,
    vx_t1: &Real,
    vy_t: &Real,
    vy_t1: &Real,
    ax_t: &Real,
    ay_t: &Real,
    dt: &Real,
) {
    // px_t1 = px_t + vx_t * dt
    let expected_px = px_t + &(vx_t * dt);
    backend.assert(&px_t1.eq(&expected_px));

    // py_t1 = py_t + vy_t * dt
    let expected_py = py_t + &(vy_t * dt);
    backend.assert(&py_t1.eq(&expected_py));

    // vx_t1 = vx_t + ax_t * dt
    let expected_vx = vx_t + &(ax_t * dt);
    backend.assert(&vx_t1.eq(&expected_vx));

    // vy_t1 = vy_t + ay_t * dt
    let expected_vy = vy_t + &(ay_t * dt);
    backend.assert(&vy_t1.eq(&expected_vy));
}

/// Encode per-step bounds for a pedestrian's velocity and acceleration.
///
/// - Acceleration bounds: clamp actor range to `[-1.0, +1.0]` for both axes
/// - Speed box constraint: `|vx| <= max_speed` AND `|vy| <= max_speed`
pub fn encode_pedestrian_bounds_step<B: Z3Backend>(
    backend: &B,
    vx_t: &Real,
    vy_t: &Real,
    ax_t: &Real,
    ay_t: &Real,
    actor: &ActorSpec,
) {
    // Acceleration bounds: clamp to pedestrian limits [-1.0, +1.0]
    let accel_min = actor.acceleration.min().max(PEDESTRIAN_MAX_DECELERATION);
    let accel_max = actor.acceleration.max().min(PEDESTRIAN_MAX_ACCELERATION);
    let ax_min_real = Real::from_rational((accel_min * 10.0) as i64, 10_i64);
    let ax_max_real = Real::from_rational((accel_max * 10.0) as i64, 10_i64);

    backend.assert(&ax_t.ge(&ax_min_real));
    backend.assert(&ax_t.le(&ax_max_real));
    backend.assert(&ay_t.ge(&ax_min_real));
    backend.assert(&ay_t.le(&ax_max_real));

    // Speed box constraint: |vx| <= max_speed AND |vy| <= max_speed
    let max_speed = actor
        .behavior
        .get("walking_mode")
        .map_or(PEDESTRIAN_WALK_MAX_SPEED, |mode| match mode.as_str() {
            Some("run") => PEDESTRIAN_RUN_MAX_SPEED,
            _ => PEDESTRIAN_WALK_MAX_SPEED,
        });

    let max_speed_real = Real::from_rational((max_speed * 10.0) as i64, 10_i64);
    let neg_max_speed_real = Real::from_rational(((-max_speed) * 10.0) as i64, 10_i64);

    backend.assert(&vx_t.ge(&neg_max_speed_real));
    backend.assert(&vx_t.le(&max_speed_real));
    backend.assert(&vy_t.ge(&neg_max_speed_real));
    backend.assert(&vy_t.le(&max_speed_real));
}

/// Extract a pedestrian's trajectory from the Z3 model.
///
/// Builds an `ActorTrajectory` with `role = "pedestrian"` by reading
/// position, velocity, acceleration, and lane values at each timestep.
#[allow(clippy::too_many_arguments)]
pub fn extract_pedestrian_trajectory(
    model: &Model,
    actor_id: &str,
    px: &[Real],
    py: &[Real],
    vx: &[Real],
    vy: &[Real],
    ax: &[Real],
    ay: &[Real],
    lanes: &[Int],
    horizon: usize,
    dt: f64,
) -> Result<ActorTrajectory> {
    let mut trajectory = ActorTrajectory::new(actor_id.to_string(), "pedestrian".to_string());

    for t in 0..=horizon {
        let time = t as f64 * dt;

        let px_val = extract_real(model, &px[t])?;
        let py_val = extract_real(model, &py[t])?;
        let vx_val = extract_real(model, &vx[t])?;
        let vy_val = extract_real(model, &vy[t])?;
        let ax_val = extract_real(model, &ax[t])?;
        let ay_val = extract_real(model, &ay[t])?;
        let lane_val = extract_int(model, &lanes[t])?;

        let state = State {
            time,
            cartesian: CartesianState {
                position: Position::new(px_val, py_val),
                velocity: Velocity::new(vx_val, vy_val),
                acceleration: Acceleration::new(ax_val, ay_val),
                lane: lane_val,
            },
        };

        trajectory.add_state(state);
    }

    Ok(trajectory)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dsl::types::{ActorRole, ActorSpec, ValueOrRange};
    use crate::solver::backend::SolverBackend;
    use crate::solver::encoder_utils::extract_real;
    use std::collections::HashMap;
    use z3::{Config, SatResult};

    /// Helper: create a basic walking pedestrian spec
    fn make_pedestrian(id: &str, lane: usize, position: ValueOrRange) -> ActorSpec {
        ActorSpec {
            id: id.to_string(),
            role: ActorRole::Pedestrian,
            lane,
            position,
            speed: ValueOrRange::Range([0.5, 1.0]),
            acceleration: ValueOrRange::Range([-0.5, 0.5]),
            direction: 1,
            behavior: HashMap::new(),
            lane_changes: vec![],
            bicycle_params: None,
        }
    }

    /// Helper: create a running pedestrian spec
    fn make_running_pedestrian(id: &str, lane: usize, position: ValueOrRange) -> ActorSpec {
        let mut behavior = HashMap::new();
        behavior.insert(
            "walking_mode".to_string(),
            serde_json::Value::String("run".to_string()),
        );
        ActorSpec {
            id: id.to_string(),
            role: ActorRole::Pedestrian,
            lane,
            position,
            speed: ValueOrRange::Range([2.0, 3.0]),
            acceleration: ValueOrRange::Range([-1.0, 1.0]),
            direction: 1,
            behavior,
            lane_changes: vec![],
            bicycle_params: None,
        }
    }

    /// Helper to parse a Z3 real AST to f64
    fn eval_real_val(model: &z3::Model, var: &Real) -> f64 {
        extract_real(model, var).unwrap()
    }

    // ==================== encode_pedestrian_initial_state ====================

    #[test]
    fn test_initial_state_fixed_position() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let backend = SolverBackend::new();
            let actor = make_pedestrian("ped1", 1, ValueOrRange::Value(10.0));
            let lane_width = 3.5;

            // Create variables (horizon=0 means just 1 timestep)
            let px = vec![Real::new_const("px_0")];
            let py = vec![Real::new_const("py_0")];
            let vx = vec![Real::new_const("vx_0")];
            let vy = vec![Real::new_const("vy_0")];
            let ax = vec![Real::new_const("ax_0")];
            let ay = vec![Real::new_const("ay_0")];

            encode_pedestrian_initial_state(&backend, &px, &py, &vx, &vy, &ax, &ay, &actor, lane_width);

            assert_eq!(backend.check(), SatResult::Sat);
            let model = backend.get_model().unwrap();

            // px should be exactly 10.0
            let px_val = eval_real_val(&model, &px[0]);
            assert!((px_val - 10.0).abs() < 0.01, "px should be 10.0, got {}", px_val);

            // py should be lane*lane_width + lane_width/2 = 1*3.5 + 1.75 = 5.25
            let py_val = eval_real_val(&model, &py[0]);
            assert!((py_val - 5.25).abs() < 0.01, "py should be 5.25, got {}", py_val);

            // vx should be in [-1.41, 1.41] (PEDESTRIAN_WALK_MAX_SPEED)
            let vx_val = eval_real_val(&model, &vx[0]);
            assert!(
                vx_val >= -PEDESTRIAN_WALK_MAX_SPEED - 0.01
                    && vx_val <= PEDESTRIAN_WALK_MAX_SPEED + 0.01,
                "vx should be in [-{}, {}], got {}",
                PEDESTRIAN_WALK_MAX_SPEED, PEDESTRIAN_WALK_MAX_SPEED, vx_val
            );

            // ax should be in [-0.5, 0.5] (clamped to pedestrian limits)
            let ax_val = eval_real_val(&model, &ax[0]);
            assert!(
                ax_val >= -0.5 - 0.01 && ax_val <= 0.5 + 0.01,
                "ax should be in [-0.5, 0.5], got {}", ax_val
            );
        });
    }

    #[test]
    fn test_initial_state_range_position() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let backend = SolverBackend::new();
            let actor = make_pedestrian("ped1", 0, ValueOrRange::Range([5.0, 15.0]));
            let lane_width = 3.5;

            let px = vec![Real::new_const("px_0")];
            let py = vec![Real::new_const("py_0")];
            let vx = vec![Real::new_const("vx_0")];
            let vy = vec![Real::new_const("vy_0")];
            let ax = vec![Real::new_const("ax_0")];
            let ay = vec![Real::new_const("ay_0")];

            encode_pedestrian_initial_state(&backend, &px, &py, &vx, &vy, &ax, &ay, &actor, lane_width);

            assert_eq!(backend.check(), SatResult::Sat);
            let model = backend.get_model().unwrap();

            // px should be in [5.0, 15.0]
            let px_val = eval_real_val(&model, &px[0]);
            assert!(
                px_val >= 5.0 - 0.01 && px_val <= 15.0 + 0.01,
                "px should be in [5, 15], got {}", px_val
            );

            // py should be 0*3.5 + 1.75 = 1.75
            let py_val = eval_real_val(&model, &py[0]);
            assert!((py_val - 1.75).abs() < 0.01, "py should be 1.75, got {}", py_val);
        });
    }

    #[test]
    fn test_initial_state_vy_unconstrained() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let backend = SolverBackend::new();
            let actor = make_pedestrian("ped1", 0, ValueOrRange::Value(10.0));
            let lane_width = 3.5;

            let px = vec![Real::new_const("px_0")];
            let py = vec![Real::new_const("py_0")];
            let vx = vec![Real::new_const("vx_0")];
            let vy = vec![Real::new_const("vy_0")];
            let ax = vec![Real::new_const("ax_0")];
            let ay = vec![Real::new_const("ay_0")];

            encode_pedestrian_initial_state(&backend, &px, &py, &vx, &vy, &ax, &ay, &actor, lane_width);

            // Assert vy must be exactly 99.0 (way outside walking limits) to prove it's unconstrained
            let big_val = Real::from_rational(990, 10);
            backend.assert(&vy[0].eq(&big_val));

            assert_eq!(backend.check(), SatResult::Sat, "vy[0] should be unconstrained");
        });
    }

    #[test]
    fn test_initial_state_ay_unconstrained() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let backend = SolverBackend::new();
            let actor = make_pedestrian("ped1", 0, ValueOrRange::Value(10.0));
            let lane_width = 3.5;

            let px = vec![Real::new_const("px_0")];
            let py = vec![Real::new_const("py_0")];
            let vx = vec![Real::new_const("vx_0")];
            let vy = vec![Real::new_const("vy_0")];
            let ax = vec![Real::new_const("ax_0")];
            let ay = vec![Real::new_const("ay_0")];

            encode_pedestrian_initial_state(&backend, &px, &py, &vx, &vy, &ax, &ay, &actor, lane_width);

            // Assert ay must be 50.0 (way outside pedestrian limits) to prove it's unconstrained
            let big_val = Real::from_rational(500, 10);
            backend.assert(&ay[0].eq(&big_val));

            assert_eq!(backend.check(), SatResult::Sat, "ay[0] should be unconstrained");
        });
    }

    #[test]
    fn test_initial_state_running_speed_bounds() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let backend = SolverBackend::new();
            let actor = make_running_pedestrian("runner", 0, ValueOrRange::Value(5.0));
            let lane_width = 3.5;

            let px = vec![Real::new_const("px_0")];
            let py = vec![Real::new_const("py_0")];
            let vx = vec![Real::new_const("vx_0")];
            let vy = vec![Real::new_const("vy_0")];
            let ax = vec![Real::new_const("ax_0")];
            let ay = vec![Real::new_const("ay_0")];

            encode_pedestrian_initial_state(&backend, &px, &py, &vx, &vy, &ax, &ay, &actor, lane_width);

            // Try to force vx > PEDESTRIAN_RUN_MAX_SPEED => should be UNSAT
            let too_fast = Real::from_rational((PEDESTRIAN_RUN_MAX_SPEED * 10.0) as i64 + 1, 10);
            backend.assert(&vx[0].gt(&too_fast));

            assert_eq!(backend.check(), SatResult::Unsat, "vx > run_max_speed should be UNSAT");
        });
    }

    // ==================== encode_pedestrian_kinematics_step ====================

    #[test]
    fn test_kinematics_step_computes_correctly() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let backend = SolverBackend::new();

            let px_t = Real::new_const("px_0");
            let px_t1 = Real::new_const("px_1");
            let py_t = Real::new_const("py_0");
            let py_t1 = Real::new_const("py_1");
            let vx_t = Real::new_const("vx_0");
            let vx_t1 = Real::new_const("vx_1");
            let vy_t = Real::new_const("vy_0");
            let vy_t1 = Real::new_const("vy_1");
            let ax_t = Real::new_const("ax_0");
            let ay_t = Real::new_const("ay_0");
            let dt = Real::from_rational(5, 10); // 0.5s

            // Fix initial state: px=10, py=5, vx=1.0, vy=0.5, ax=0.2, ay=-0.1
            backend.assert(&px_t.eq(&Real::from_rational(100, 10)));
            backend.assert(&py_t.eq(&Real::from_rational(50, 10)));
            backend.assert(&vx_t.eq(&Real::from_rational(10, 10)));
            backend.assert(&vy_t.eq(&Real::from_rational(5, 10)));
            backend.assert(&ax_t.eq(&Real::from_rational(2, 10)));
            backend.assert(&ay_t.eq(&Real::from_rational(-1, 10)));

            encode_pedestrian_kinematics_step(
                &backend, &px_t, &px_t1, &py_t, &py_t1,
                &vx_t, &vx_t1, &vy_t, &vy_t1, &ax_t, &ay_t, &dt,
            );

            assert_eq!(backend.check(), SatResult::Sat);
            let model = backend.get_model().unwrap();

            // px_t1 = 10.0 + 1.0 * 0.5 = 10.5
            let px1 = eval_real_val(&model, &px_t1);
            assert!((px1 - 10.5).abs() < 0.01, "px_t1 should be 10.5, got {}", px1);

            // py_t1 = 5.0 + 0.5 * 0.5 = 5.25
            let py1 = eval_real_val(&model, &py_t1);
            assert!((py1 - 5.25).abs() < 0.01, "py_t1 should be 5.25, got {}", py1);

            // vx_t1 = 1.0 + 0.2 * 0.5 = 1.1
            let vx1 = eval_real_val(&model, &vx_t1);
            assert!((vx1 - 1.1).abs() < 0.01, "vx_t1 should be 1.1, got {}", vx1);

            // vy_t1 = 0.5 + (-0.1) * 0.5 = 0.45
            let vy1 = eval_real_val(&model, &vy_t1);
            assert!((vy1 - 0.45).abs() < 0.01, "vy_t1 should be 0.45, got {}", vy1);
        });
    }

    #[test]
    fn test_kinematics_multi_step_trajectory() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let backend = SolverBackend::new();
            let horizon = 4;
            let dt = Real::from_rational(5, 10); // 0.5s

            // Create variable arrays for 5 timesteps (horizon+1)
            let px: Vec<_> = (0..=horizon).map(|t| Real::new_const(format!("px_{}", t))).collect();
            let py: Vec<_> = (0..=horizon).map(|t| Real::new_const(format!("py_{}", t))).collect();
            let vx: Vec<_> = (0..=horizon).map(|t| Real::new_const(format!("vx_{}", t))).collect();
            let vy: Vec<_> = (0..=horizon).map(|t| Real::new_const(format!("vy_{}", t))).collect();
            let ax: Vec<_> = (0..horizon).map(|t| Real::new_const(format!("ax_{}", t))).collect();
            let ay: Vec<_> = (0..horizon).map(|t| Real::new_const(format!("ay_{}", t))).collect();

            // Fix initial state
            backend.assert(&px[0].eq(&Real::from_rational(0, 1)));
            backend.assert(&py[0].eq(&Real::from_rational(0, 1)));
            backend.assert(&vx[0].eq(&Real::from_rational(10, 10))); // 1.0 m/s
            backend.assert(&vy[0].eq(&Real::from_rational(5, 10)));  // 0.5 m/s

            // Fix constant acceleration
            for t in 0..horizon {
                backend.assert(&ax[t].eq(&Real::from_rational(0, 1))); // zero ax
                backend.assert(&ay[t].eq(&Real::from_rational(0, 1))); // zero ay
            }

            // Encode kinematics for each step
            for t in 0..horizon {
                encode_pedestrian_kinematics_step(
                    &backend, &px[t], &px[t + 1], &py[t], &py[t + 1],
                    &vx[t], &vx[t + 1], &vy[t], &vy[t + 1], &ax[t], &ay[t], &dt,
                );
            }

            assert_eq!(backend.check(), SatResult::Sat);
            let model = backend.get_model().unwrap();

            // With constant velocity (ax=ay=0), position should increase linearly
            // px at t=4: 0 + 1.0 * 0.5 * 4 = 2.0
            let px_final = eval_real_val(&model, &px[horizon]);
            assert!((px_final - 2.0).abs() < 0.01, "px[4] should be 2.0, got {}", px_final);

            // py at t=4: 0 + 0.5 * 0.5 * 4 = 1.0
            let py_final = eval_real_val(&model, &py[horizon]);
            assert!((py_final - 1.0).abs() < 0.01, "py[4] should be 1.0, got {}", py_final);
        });
    }

    // ==================== encode_pedestrian_bounds_step ====================

    #[test]
    fn test_bounds_step_enforces_speed_limit() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let backend = SolverBackend::new();
            let actor = make_pedestrian("ped1", 0, ValueOrRange::Value(0.0));

            let vx = Real::new_const("vx");
            let vy = Real::new_const("vy");
            let ax = Real::new_const("ax");
            let ay = Real::new_const("ay");

            encode_pedestrian_bounds_step(&backend, &vx, &vy, &ax, &ay, &actor);

            // Try to force vx > max walking speed => UNSAT
            let too_fast = Real::from_rational((PEDESTRIAN_WALK_MAX_SPEED * 10.0) as i64 + 1, 10);
            backend.assert(&vx.gt(&too_fast));

            assert_eq!(backend.check(), SatResult::Unsat, "vx > walk_max should be UNSAT");
        });
    }

    #[test]
    fn test_bounds_step_enforces_negative_speed_limit() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let backend = SolverBackend::new();
            let actor = make_pedestrian("ped1", 0, ValueOrRange::Value(0.0));

            let vx = Real::new_const("vx");
            let vy = Real::new_const("vy");
            let ax = Real::new_const("ax");
            let ay = Real::new_const("ay");

            encode_pedestrian_bounds_step(&backend, &vx, &vy, &ax, &ay, &actor);

            // Try to force vx < -max walking speed => UNSAT
            let too_neg = Real::from_rational(((-PEDESTRIAN_WALK_MAX_SPEED) * 10.0) as i64 - 1, 10);
            backend.assert(&vx.lt(&too_neg));

            assert_eq!(backend.check(), SatResult::Unsat, "vx < -walk_max should be UNSAT");
        });
    }

    #[test]
    fn test_bounds_step_enforces_acceleration_limit() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let backend = SolverBackend::new();
            let actor = make_pedestrian("ped1", 0, ValueOrRange::Value(0.0));

            let vx = Real::new_const("vx");
            let vy = Real::new_const("vy");
            let ax = Real::new_const("ax");
            let ay = Real::new_const("ay");

            encode_pedestrian_bounds_step(&backend, &vx, &vy, &ax, &ay, &actor);

            // The actor spec has acceleration [-0.5, 0.5], which is within pedestrian limits [-1, 1]
            // So the effective limit is [-0.5, 0.5]
            // Force ax > 0.5 => UNSAT
            let too_high = Real::from_rational(6, 10); // 0.6
            backend.assert(&ax.gt(&too_high));

            assert_eq!(backend.check(), SatResult::Unsat, "ax > 0.5 should be UNSAT");
        });
    }

    #[test]
    fn test_bounds_step_running_mode_higher_speed() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let backend = SolverBackend::new();
            let actor = make_running_pedestrian("runner", 0, ValueOrRange::Value(0.0));

            let vx = Real::new_const("vx");
            let vy = Real::new_const("vy");
            let ax = Real::new_const("ax");
            let ay = Real::new_const("ay");

            encode_pedestrian_bounds_step(&backend, &vx, &vy, &ax, &ay, &actor);

            // Force vx = 3.0 (above walk limit but below run limit) => should be SAT
            let v_3 = Real::from_rational(30, 10);
            backend.assert(&vx.eq(&v_3));

            assert_eq!(backend.check(), SatResult::Sat, "vx=3.0 should be SAT for runner");
        });
    }

    #[test]
    fn test_bounds_step_lateral_speed_limited() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let backend = SolverBackend::new();
            let actor = make_pedestrian("ped1", 0, ValueOrRange::Value(0.0));

            let vx = Real::new_const("vx");
            let vy = Real::new_const("vy");
            let ax = Real::new_const("ax");
            let ay = Real::new_const("ay");

            encode_pedestrian_bounds_step(&backend, &vx, &vy, &ax, &ay, &actor);

            // Force vy > walk max speed => UNSAT
            let too_fast = Real::from_rational((PEDESTRIAN_WALK_MAX_SPEED * 10.0) as i64 + 1, 10);
            backend.assert(&vy.gt(&too_fast));

            assert_eq!(backend.check(), SatResult::Unsat, "vy > walk_max should be UNSAT");
        });
    }

    // ==================== extract_pedestrian_trajectory ====================

    #[test]
    fn test_extract_trajectory_from_model() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let backend = SolverBackend::new();
            let horizon = 2;
            let dt_val = 0.5;
            let dt = Real::from_rational(5, 10);

            // Create variables
            let px: Vec<_> = (0..=horizon).map(|t| Real::new_const(format!("px_{}", t))).collect();
            let py: Vec<_> = (0..=horizon).map(|t| Real::new_const(format!("py_{}", t))).collect();
            let vx: Vec<_> = (0..=horizon).map(|t| Real::new_const(format!("vx_{}", t))).collect();
            let vy: Vec<_> = (0..=horizon).map(|t| Real::new_const(format!("vy_{}", t))).collect();
            let ax: Vec<_> = (0..=horizon).map(|t| Real::new_const(format!("ax_{}", t))).collect();
            let ay: Vec<_> = (0..=horizon).map(|t| Real::new_const(format!("ay_{}", t))).collect();
            let lanes: Vec<_> = (0..=horizon).map(|t| Int::new_const(format!("lane_{}", t))).collect();

            // Fix initial state and constant dynamics
            backend.assert(&px[0].eq(&Real::from_rational(0, 1)));
            backend.assert(&py[0].eq(&Real::from_rational(35, 10))); // 3.5
            backend.assert(&vx[0].eq(&Real::from_rational(10, 10))); // 1.0
            backend.assert(&vy[0].eq(&Real::from_rational(5, 10)));  // 0.5
            backend.assert(&ax[0].eq(&Real::from_rational(0, 1)));
            backend.assert(&ay[0].eq(&Real::from_rational(0, 1)));

            for t in 0..=horizon {
                backend.assert(&lanes[t].eq(&Int::from_i64(1)));
            }

            // Encode kinematics for 2 steps
            for t in 0..horizon {
                // Keep acceleration constant at 0 for later steps
                if t > 0 {
                    backend.assert(&ax[t].eq(&Real::from_rational(0, 1)));
                    backend.assert(&ay[t].eq(&Real::from_rational(0, 1)));
                }
                encode_pedestrian_kinematics_step(
                    &backend, &px[t], &px[t + 1], &py[t], &py[t + 1],
                    &vx[t], &vx[t + 1], &vy[t], &vy[t + 1], &ax[t], &ay[t], &dt,
                );
            }

            assert_eq!(backend.check(), SatResult::Sat);
            let model = backend.get_model().unwrap();

            let trajectory = extract_pedestrian_trajectory(
                &model, "ped1", &px, &py, &vx, &vy, &ax, &ay, &lanes, horizon, dt_val,
            ).unwrap();

            assert_eq!(trajectory.id, "ped1");
            assert_eq!(trajectory.role, "pedestrian");
            assert_eq!(trajectory.states.len(), 3); // horizon+1

            // Check first state
            let s0 = &trajectory.states[0];
            assert!((s0.time - 0.0).abs() < 0.001);
            assert!((s0.cartesian.position.x - 0.0).abs() < 0.01);
            assert!((s0.cartesian.position.y - 3.5).abs() < 0.01);
            assert!((s0.cartesian.velocity.vx - 1.0).abs() < 0.01);
            assert!((s0.cartesian.velocity.vy - 0.5).abs() < 0.01);
            assert_eq!(s0.cartesian.lane, 1);

            // Check second state (t=0.5)
            let s1 = &trajectory.states[1];
            assert!((s1.time - 0.5).abs() < 0.001);
            assert!((s1.cartesian.position.x - 0.5).abs() < 0.01);  // 0 + 1.0*0.5
            assert!((s1.cartesian.position.y - 3.75).abs() < 0.01); // 3.5 + 0.5*0.5

            // Check third state (t=1.0)
            let s2 = &trajectory.states[2];
            assert!((s2.time - 1.0).abs() < 0.001);
            assert!((s2.cartesian.position.x - 1.0).abs() < 0.01);  // 0.5 + 1.0*0.5
            assert!((s2.cartesian.position.y - 4.0).abs() < 0.01);  // 3.75 + 0.5*0.5
        });
    }

    // ==================== Integration test: full pedestrian lifecycle ====================

    #[test]
    fn test_full_pedestrian_encode_and_extract() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let backend = SolverBackend::new();
            let actor = make_pedestrian("ped_cross", 0, ValueOrRange::Value(20.0));
            let lane_width = 3.5;
            let horizon = 4;
            let dt_val = 0.5;
            let dt = Real::from_rational(5, 10);

            // Create variables
            let px: Vec<_> = (0..=horizon).map(|t| Real::new_const(format!("px_{}", t))).collect();
            let py: Vec<_> = (0..=horizon).map(|t| Real::new_const(format!("py_{}", t))).collect();
            let vx: Vec<_> = (0..=horizon).map(|t| Real::new_const(format!("vx_{}", t))).collect();
            let vy: Vec<_> = (0..=horizon).map(|t| Real::new_const(format!("vy_{}", t))).collect();
            let ax: Vec<_> = (0..=horizon).map(|t| Real::new_const(format!("ax_{}", t))).collect();
            let ay: Vec<_> = (0..=horizon).map(|t| Real::new_const(format!("ay_{}", t))).collect();
            let lanes: Vec<_> = (0..=horizon).map(|t| Int::new_const(format!("lane_{}", t))).collect();

            // Fix lanes (pedestrian stays on lane 0 throughout)
            for t in 0..=horizon {
                backend.assert(&lanes[t].eq(&Int::from_i64(0)));
            }

            // 1. Encode initial state
            encode_pedestrian_initial_state(&backend, &px, &py, &vx, &vy, &ax, &ay, &actor, lane_width);

            // 2. Encode kinematics and bounds for each step
            for t in 0..horizon {
                encode_pedestrian_kinematics_step(
                    &backend, &px[t], &px[t + 1], &py[t], &py[t + 1],
                    &vx[t], &vx[t + 1], &vy[t], &vy[t + 1], &ax[t], &ay[t], &dt,
                );
                encode_pedestrian_bounds_step(&backend, &vx[t], &vy[t], &ax[t], &ay[t], &actor);
            }
            // Also bound the final velocity step
            encode_pedestrian_bounds_step(
                &backend, &vx[horizon], &vy[horizon],
                &ax[horizon - 1], &ay[horizon - 1], &actor,
            );

            assert_eq!(backend.check(), SatResult::Sat, "Full pedestrian encoding should be SAT");
            let model = backend.get_model().unwrap();

            // 3. Extract trajectory
            let trajectory = extract_pedestrian_trajectory(
                &model, "ped_cross", &px, &py, &vx, &vy, &ax, &ay, &lanes, horizon, dt_val,
            ).unwrap();

            assert_eq!(trajectory.states.len(), horizon + 1);
            assert_eq!(trajectory.role, "pedestrian");

            // Verify physics consistency: position changes match velocity * dt
            for t in 0..horizon {
                let s = &trajectory.states[t];
                let s_next = &trajectory.states[t + 1];

                let expected_px = s.cartesian.position.x + s.cartesian.velocity.vx * dt_val;
                let expected_py = s.cartesian.position.y + s.cartesian.velocity.vy * dt_val;

                assert!(
                    (s_next.cartesian.position.x - expected_px).abs() < 0.05,
                    "px mismatch at t={}: {} vs expected {}",
                    t + 1, s_next.cartesian.position.x, expected_px
                );
                assert!(
                    (s_next.cartesian.position.y - expected_py).abs() < 0.05,
                    "py mismatch at t={}: {} vs expected {}",
                    t + 1, s_next.cartesian.position.y, expected_py
                );
            }

            // Verify speed bounds hold at every step
            for t in 0..=horizon {
                let s = &trajectory.states[t];
                assert!(
                    s.cartesian.velocity.vx.abs() <= PEDESTRIAN_WALK_MAX_SPEED + 0.05,
                    "vx out of bounds at t={}: {}", t, s.cartesian.velocity.vx
                );
                assert!(
                    s.cartesian.velocity.vy.abs() <= PEDESTRIAN_WALK_MAX_SPEED + 0.05,
                    "vy out of bounds at t={}: {}", t, s.cartesian.velocity.vy
                );
            }
        });
    }
}
