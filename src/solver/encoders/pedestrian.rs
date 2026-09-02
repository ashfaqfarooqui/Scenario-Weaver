//! Shared 2D point-mass encoder helpers
//!
//! Provides reusable functions for encoding pedestrian dynamics in Z3.
//! Pedestrians use a simple 2D point-mass model (no steering, no heading).
//!
//! Both `CartesianEncoder` and `BicycleEncoder` call these helpers, and the
//! Cartesian encoder additionally uses [`encode_pedestrian_kinematics_step`]
//! for its *vehicles*: the Cartesian model is a 2D point mass for every actor,
//! and before SW-09 the only thing that distinguished a vehicle from a
//! pedestrian there was that the vehicle's `vy` was never chained to its `ay`
//! (finding C2 — a free lateral acceleration that bounded nothing and put
//! fiction in the exported `.xosc`). There is now one integration step, used by
//! everything the Cartesian encoder emits.
//!
//! All conversions from `f64` go through [`real_from_f64`], which is exact for
//! every value a spec can hold; the ad-hoc `(x * 10.0) as i64` truncations this
//! module used to carry silently rounded, e.g., a 0.05 m/s bound to 0.0.

use z3::ast::{Int, Real};
use z3::Model;

use crate::dsl::types::{
    ActorSpec, PEDESTRIAN_MAX_ACCELERATION, PEDESTRIAN_MAX_DECELERATION, PEDESTRIAN_RUN_MAX_SPEED,
    PEDESTRIAN_WALK_MAX_SPEED,
};
use crate::error::Result;
use crate::scenario::model::{
    Acceleration, ActorTrajectory, CartesianState, Position, State, Velocity,
};
use crate::solver::backend::Z3Backend;
use crate::solver::encoder::SIDEWALK_WIDTH;
use crate::solver::encoder_utils::{extract_int, extract_real, real_from_f64};

/// The pedestrian's speed cap, selected by `behavior.walking_mode`.
fn pedestrian_max_speed(actor: &ActorSpec) -> f64 {
    actor
        .behavior
        .get("walking_mode")
        .map_or(PEDESTRIAN_WALK_MAX_SPEED, |mode| match mode.as_str() {
            Some("run") => PEDESTRIAN_RUN_MAX_SPEED,
            _ => PEDESTRIAN_WALK_MAX_SPEED,
        })
}

/// The actor's acceleration range, clamped to the pedestrian physics limits.
fn pedestrian_accel_range(actor: &ActorSpec) -> (f64, f64) {
    (
        actor.acceleration.min().max(PEDESTRIAN_MAX_DECELERATION),
        actor.acceleration.max().min(PEDESTRIAN_MAX_ACCELERATION),
    )
}

/// Encode the initial state constraints for a pedestrian at t=0.
///
/// - `px[0]`: range or fixed from `actor.position`
/// - `py[0]`: computed from `actor.lane * lane_width + lane_width / 2.0`
/// - `vx[0]`: the actor's own speed range, signed by `actor.direction` and
///   intersected with `[-max_speed, +max_speed]`
/// - `vy[0]`: left UNCONSTRAINED (pedestrian may already be crossing)
/// - `ax[0]`: bounded by actor acceleration range, clamped to pedestrian limits
/// - `ay[0]`: left UNCONSTRAINED
///
/// The `vx[0]` rule is deliberately the intersection and not just the speed
/// cap: the cap alone would discard `speed:` from the spec entirely, which is
/// what the Cartesian encoder used to honour on its own inline path.
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
        backend.assert(&px[0].eq(real_from_f64(pos_min)));
    } else {
        backend.assert(&px[0].ge(real_from_f64(pos_min)));
        backend.assert(&px[0].le(real_from_f64(pos_max)));
    }

    // py[0]: lateral position from lane center
    let py_initial = actor.lane as f64 * lane_width + lane_width / 2.0;
    backend.assert(&py[0].eq(real_from_f64(py_initial)));

    // vx[0]: the spec's speed range, signed by direction, capped by the
    // pedestrian's walking/running limit.
    let max_speed = pedestrian_max_speed(actor);
    let (speed_min, speed_max) = if actor.direction == 1 {
        (actor.speed.min(), actor.speed.max())
    } else {
        (-actor.speed.max(), -actor.speed.min())
    };
    let vx_min = speed_min.max(-max_speed);
    let vx_max = speed_max.min(max_speed);
    if (vx_min - vx_max).abs() < 1e-6 {
        backend.assert(&vx[0].eq(real_from_f64(vx_min)));
    } else {
        backend.assert(&vx[0].ge(real_from_f64(vx_min)));
        backend.assert(&vx[0].le(real_from_f64(vx_max)));
    }

    // vy[0]: UNCONSTRAINED (pedestrian may already be crossing)

    // ax[0]: bounded by actor acceleration range, clamped to pedestrian limits
    let (accel_min, accel_max) = pedestrian_accel_range(actor);
    if (accel_min - accel_max).abs() < 1e-6 {
        backend.assert(&ax[0].eq(real_from_f64(accel_min)));
    } else {
        backend.assert(&ax[0].ge(real_from_f64(accel_min)));
        backend.assert(&ax[0].le(real_from_f64(accel_max)));
    }

    // ay[0]: UNCONSTRAINED
}

/// Encode one kinematics timestep of the 2D point-mass model.
///
/// Asserts, on both axes, the exact constant-acceleration update written in
/// trapezoidal form:
/// - `px_t1 = px_t + (vx_t + vx_t1) * dt/2`  (≡ `px + vx*dt + ½*ax*dt²`)
/// - `py_t1 = py_t + (vy_t + vy_t1) * dt/2`  (≡ `py + vy*dt + ½*ay*dt²`)
/// - `vx_t1 = vx_t + ax_t * dt`
/// - `vy_t1 = vy_t + ay_t * dt`
///
/// The trapezoidal form is identical to the explicit one given the velocity
/// updates asserted alongside it, and is the cheaper of the two for Z3 because
/// it leaves position coupled only to the velocity chain (measured in SW-08).
/// Everything here is constant × variable, so the encoding stays in QF_LRA.
///
/// `half_dt` must be `dt / 2`; it is passed in rather than derived so the
/// caller builds both numerals once for the whole horizon.
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
    half_dt: &Real,
) {
    // vx_t1 = vx_t + ax_t * dt
    let expected_vx = vx_t + &(ax_t * dt);
    backend.assert(&vx_t1.eq(&expected_vx));

    // vy_t1 = vy_t + ay_t * dt
    let expected_vy = vy_t + &(ay_t * dt);
    backend.assert(&vy_t1.eq(&expected_vy));

    // px_t1 = px_t + (vx_t + vx_t1) * dt/2
    let expected_px = px_t + &((vx_t + vx_t1) * half_dt);
    backend.assert(&px_t1.eq(&expected_px));

    // py_t1 = py_t + (vy_t + vy_t1) * dt/2
    let expected_py = py_t + &((vy_t + vy_t1) * half_dt);
    backend.assert(&py_t1.eq(&expected_py));
}

/// Encode per-step bounds for a pedestrian's velocity and acceleration.
///
/// - Acceleration bounds: clamp actor range to `[-1.0, +1.0]` for both axes
/// - Speed octagon: `|vx| <= v`, `|vy| <= v` and `|vx| + |vy| <= sqrt(2)*v`
pub fn encode_pedestrian_bounds_step<B: Z3Backend>(
    backend: &B,
    vx_t: &Real,
    vy_t: &Real,
    ax_t: &Real,
    ay_t: &Real,
    actor: &ActorSpec,
) {
    // Acceleration bounds: clamp to pedestrian limits [-1.0, +1.0]
    let (accel_min, accel_max) = pedestrian_accel_range(actor);
    let ax_min_real = real_from_f64(accel_min);
    let ax_max_real = real_from_f64(accel_max);

    backend.assert(&ax_t.ge(&ax_min_real));
    backend.assert(&ax_t.le(&ax_max_real));
    backend.assert(&ay_t.ge(&ax_min_real));
    backend.assert(&ay_t.le(&ax_max_real));

    // Speed octagon (SW-12/M8):
    //     |vx| <= v,  |vy| <= v,  |vx| + |vy| <= sqrt(2)*v
    //
    // The last pair of half-planes is the whole change. Without them the
    // bound is a *box*, which contains the disk and lets a pedestrian walk at
    // sqrt(2)*v on the diagonal; `dsl::types` compensated by dividing the
    // speed constants by sqrt(2), which fixed the diagonal and broke every
    // other direction — a pedestrian crossing perpendicular to the road, the
    // dominant case here, was capped at 1.41 m/s instead of 2.0.
    //
    // `|vx| + |vy| <= c` is four linear constraints, one per sign
    // combination, asserted unconditionally: no Bool selector, no case split,
    // so this is propagation rather than search and the encoding stays in
    // QF_LRA. The disk `vx^2 + vy^2 <= v^2` would be exact but nonlinear —
    // QF_NRA with the Int lane variable in the same problem, where Z3 answers
    // `unknown` and `Optimize` has no support for the objective at all.
    let max_speed = pedestrian_max_speed(actor);
    let max_speed_real = real_from_f64(max_speed);
    let neg_max_speed_real = real_from_f64(-max_speed);

    backend.assert(&vx_t.ge(&neg_max_speed_real));
    backend.assert(&vx_t.le(&max_speed_real));
    backend.assert(&vy_t.ge(&neg_max_speed_real));
    backend.assert(&vy_t.le(&max_speed_real));

    let diag_real = real_from_f64(max_speed * std::f64::consts::SQRT_2);
    let sum = vx_t + vy_t;
    let diff = vx_t - vy_t;
    backend.assert(&sum.le(&diag_real));
    backend.assert(&diff.le(&diag_real));
    backend.assert(&(&Real::from_rational(0_i64, 1_i64) - &sum).le(&diag_real));
    backend.assert(&(&Real::from_rational(0_i64, 1_i64) - &diff).le(&diag_real));
}

/// Bound a pedestrian's lateral position to the drivable surface plus the
/// sidewalk margin on either side, at a single step.
///
/// `-SIDEWALK_WIDTH <= py <= road_width + SIDEWALK_WIDTH`. Constants only
/// (`road_width` is a per-spec constant the caller computes once from
/// `lane_width * num_lanes`), so this stays in QF_LRA.
///
/// This closes the gap SW-16 found and could not close from
/// `src/solver/encoder.rs`: `OnSidewalk` only appears inside `eventually(...)`,
/// so it pins `py` into the `SIDEWALK_WIDTH` strip at one instant and leaves
/// it unconstrained at every other. Reuses the same `SIDEWALK_WIDTH` the
/// `.xodr` exporter's `sidewalk_widths` floors against, rather than a second
/// copy of the constant (SW-23).
pub fn encode_pedestrian_lateral_containment<B: Z3Backend>(
    backend: &B,
    py_t: &Real,
    road_width: f64,
) {
    let lower = real_from_f64(-SIDEWALK_WIDTH);
    let upper = real_from_f64(road_width + SIDEWALK_WIDTH);
    backend.assert(&py_t.ge(&lower));
    backend.assert(&py_t.le(&upper));
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

            encode_pedestrian_initial_state(
                &backend, &px, &py, &vx, &vy, &ax, &ay, &actor, lane_width,
            );

            assert_eq!(backend.check(), SatResult::Sat);
            let model = backend.get_model().unwrap();

            // px should be exactly 10.0
            let px_val = eval_real_val(&model, &px[0]);
            assert!(
                (px_val - 10.0).abs() < 0.01,
                "px should be 10.0, got {}",
                px_val
            );

            // py should be lane*lane_width + lane_width/2 = 1*3.5 + 1.75 = 5.25
            let py_val = eval_real_val(&model, &py[0]);
            assert!(
                (py_val - 5.25).abs() < 0.01,
                "py should be 5.25, got {}",
                py_val
            );

            // vx is the actor's own speed range intersected with the walking
            // cap. `make_pedestrian` declares [0.5, 1.0], which sits inside
            // ±PEDESTRIAN_WALK_MAX_SPEED, so the cap is what is checked here.
            let vx_val = eval_real_val(&model, &vx[0]);
            assert!(
                vx_val >= -PEDESTRIAN_WALK_MAX_SPEED - 0.01
                    && vx_val <= PEDESTRIAN_WALK_MAX_SPEED + 0.01,
                "vx should be in [-{}, {}], got {}",
                PEDESTRIAN_WALK_MAX_SPEED,
                PEDESTRIAN_WALK_MAX_SPEED,
                vx_val
            );

            // ax should be in [-0.5, 0.5] (clamped to pedestrian limits)
            let ax_val = eval_real_val(&model, &ax[0]);
            assert!(
                ax_val >= -0.5 - 0.01 && ax_val <= 0.5 + 0.01,
                "ax should be in [-0.5, 0.5], got {}",
                ax_val
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

            encode_pedestrian_initial_state(
                &backend, &px, &py, &vx, &vy, &ax, &ay, &actor, lane_width,
            );

            assert_eq!(backend.check(), SatResult::Sat);
            let model = backend.get_model().unwrap();

            // px should be in [5.0, 15.0]
            let px_val = eval_real_val(&model, &px[0]);
            assert!(
                px_val >= 5.0 - 0.01 && px_val <= 15.0 + 0.01,
                "px should be in [5, 15], got {}",
                px_val
            );

            // py should be 0*3.5 + 1.75 = 1.75
            let py_val = eval_real_val(&model, &py[0]);
            assert!(
                (py_val - 1.75).abs() < 0.01,
                "py should be 1.75, got {}",
                py_val
            );
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

            encode_pedestrian_initial_state(
                &backend, &px, &py, &vx, &vy, &ax, &ay, &actor, lane_width,
            );

            // Assert vy must be exactly 99.0 (way outside walking limits) to prove it's unconstrained
            let big_val = Real::from_rational(990, 10);
            backend.assert(&vy[0].eq(&big_val));

            assert_eq!(
                backend.check(),
                SatResult::Sat,
                "vy[0] should be unconstrained"
            );
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

            encode_pedestrian_initial_state(
                &backend, &px, &py, &vx, &vy, &ax, &ay, &actor, lane_width,
            );

            // Assert ay must be 50.0 (way outside pedestrian limits) to prove it's unconstrained
            let big_val = Real::from_rational(500, 10);
            backend.assert(&ay[0].eq(&big_val));

            assert_eq!(
                backend.check(),
                SatResult::Sat,
                "ay[0] should be unconstrained"
            );
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

            encode_pedestrian_initial_state(
                &backend, &px, &py, &vx, &vy, &ax, &ay, &actor, lane_width,
            );

            // Try to force vx > PEDESTRIAN_RUN_MAX_SPEED => should be UNSAT
            let too_fast = Real::from_rational((PEDESTRIAN_RUN_MAX_SPEED * 10.0) as i64 + 1, 10);
            backend.assert(&vx[0].gt(&too_fast));

            assert_eq!(
                backend.check(),
                SatResult::Unsat,
                "vx > run_max_speed should be UNSAT"
            );
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
            let half_dt = Real::from_rational(25, 100); // dt / 2

            // Fix initial state: px=10, py=5, vx=1.0, vy=0.5, ax=0.2, ay=-0.1
            backend.assert(&px_t.eq(&Real::from_rational(100, 10)));
            backend.assert(&py_t.eq(&Real::from_rational(50, 10)));
            backend.assert(&vx_t.eq(&Real::from_rational(10, 10)));
            backend.assert(&vy_t.eq(&Real::from_rational(5, 10)));
            backend.assert(&ax_t.eq(&Real::from_rational(2, 10)));
            backend.assert(&ay_t.eq(&Real::from_rational(-1, 10)));

            encode_pedestrian_kinematics_step(
                &backend, &px_t, &px_t1, &py_t, &py_t1, &vx_t, &vx_t1, &vy_t, &vy_t1, &ax_t, &ay_t,
                &dt, &half_dt,
            );

            assert_eq!(backend.check(), SatResult::Sat);
            let model = backend.get_model().unwrap();

            // The position update is the exact constant-acceleration one,
            // p + v*dt + 1/2*a*dt^2, written trapezoidally. The old
            // expectations here were the forward-Euler p + v*dt, which is the
            // H1 defect SW-08 fixed; they differ by 1/2*a*dt^2 exactly.
            //
            // px_t1 = 10.0 + 1.0*0.5 + 0.5*0.2*0.25 = 10.525
            let px1 = eval_real_val(&model, &px_t1);
            assert!(
                (px1 - 10.525).abs() < 1e-9,
                "px_t1 should be 10.525, got {}",
                px1
            );

            // py_t1 = 5.0 + 0.5*0.5 + 0.5*(-0.1)*0.25 = 5.2375
            let py1 = eval_real_val(&model, &py_t1);
            assert!(
                (py1 - 5.2375).abs() < 1e-9,
                "py_t1 should be 5.2375, got {}",
                py1
            );

            // vx_t1 = 1.0 + 0.2 * 0.5 = 1.1
            let vx1 = eval_real_val(&model, &vx_t1);
            assert!((vx1 - 1.1).abs() < 0.01, "vx_t1 should be 1.1, got {}", vx1);

            // vy_t1 = 0.5 + (-0.1) * 0.5 = 0.45
            let vy1 = eval_real_val(&model, &vy_t1);
            assert!(
                (vy1 - 0.45).abs() < 0.01,
                "vy_t1 should be 0.45, got {}",
                vy1
            );
        });
    }

    #[test]
    fn test_kinematics_multi_step_trajectory() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let backend = SolverBackend::new();
            let horizon = 4;
            let dt = Real::from_rational(5, 10); // 0.5s
            let half_dt = Real::from_rational(25, 100); // dt / 2

            // Create variable arrays for 5 timesteps (horizon+1)
            let px: Vec<_> = (0..=horizon)
                .map(|t| Real::new_const(format!("px_{}", t)))
                .collect();
            let py: Vec<_> = (0..=horizon)
                .map(|t| Real::new_const(format!("py_{}", t)))
                .collect();
            let vx: Vec<_> = (0..=horizon)
                .map(|t| Real::new_const(format!("vx_{}", t)))
                .collect();
            let vy: Vec<_> = (0..=horizon)
                .map(|t| Real::new_const(format!("vy_{}", t)))
                .collect();
            let ax: Vec<_> = (0..horizon)
                .map(|t| Real::new_const(format!("ax_{}", t)))
                .collect();
            let ay: Vec<_> = (0..horizon)
                .map(|t| Real::new_const(format!("ay_{}", t)))
                .collect();

            // Fix initial state
            backend.assert(&px[0].eq(&Real::from_rational(0, 1)));
            backend.assert(&py[0].eq(&Real::from_rational(0, 1)));
            backend.assert(&vx[0].eq(&Real::from_rational(10, 10))); // 1.0 m/s
            backend.assert(&vy[0].eq(&Real::from_rational(5, 10))); // 0.5 m/s

            // Fix constant acceleration
            for t in 0..horizon {
                backend.assert(&ax[t].eq(&Real::from_rational(0, 1))); // zero ax
                backend.assert(&ay[t].eq(&Real::from_rational(0, 1))); // zero ay
            }

            // Encode kinematics for each step
            for t in 0..horizon {
                encode_pedestrian_kinematics_step(
                    &backend,
                    &px[t],
                    &px[t + 1],
                    &py[t],
                    &py[t + 1],
                    &vx[t],
                    &vx[t + 1],
                    &vy[t],
                    &vy[t + 1],
                    &ax[t],
                    &ay[t],
                    &dt,
                    &half_dt,
                );
            }

            assert_eq!(backend.check(), SatResult::Sat);
            let model = backend.get_model().unwrap();

            // With constant velocity (ax=ay=0), position should increase linearly
            // px at t=4: 0 + 1.0 * 0.5 * 4 = 2.0
            let px_final = eval_real_val(&model, &px[horizon]);
            assert!(
                (px_final - 2.0).abs() < 0.01,
                "px[4] should be 2.0, got {}",
                px_final
            );

            // py at t=4: 0 + 0.5 * 0.5 * 4 = 1.0
            let py_final = eval_real_val(&model, &py[horizon]);
            assert!(
                (py_final - 1.0).abs() < 0.01,
                "py[4] should be 1.0, got {}",
                py_final
            );
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

            assert_eq!(
                backend.check(),
                SatResult::Unsat,
                "vx > walk_max should be UNSAT"
            );
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

            assert_eq!(
                backend.check(),
                SatResult::Unsat,
                "vx < -walk_max should be UNSAT"
            );
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

            assert_eq!(
                backend.check(),
                SatResult::Unsat,
                "ax > 0.5 should be UNSAT"
            );
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

            assert_eq!(
                backend.check(),
                SatResult::Sat,
                "vx=3.0 should be SAT for runner"
            );
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

            assert_eq!(
                backend.check(),
                SatResult::Unsat,
                "vy > walk_max should be UNSAT"
            );
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
            let half_dt = Real::from_rational(25, 100); // dt / 2

            // Create variables
            let px: Vec<_> = (0..=horizon)
                .map(|t| Real::new_const(format!("px_{}", t)))
                .collect();
            let py: Vec<_> = (0..=horizon)
                .map(|t| Real::new_const(format!("py_{}", t)))
                .collect();
            let vx: Vec<_> = (0..=horizon)
                .map(|t| Real::new_const(format!("vx_{}", t)))
                .collect();
            let vy: Vec<_> = (0..=horizon)
                .map(|t| Real::new_const(format!("vy_{}", t)))
                .collect();
            let ax: Vec<_> = (0..=horizon)
                .map(|t| Real::new_const(format!("ax_{}", t)))
                .collect();
            let ay: Vec<_> = (0..=horizon)
                .map(|t| Real::new_const(format!("ay_{}", t)))
                .collect();
            let lanes: Vec<_> = (0..=horizon)
                .map(|t| Int::new_const(format!("lane_{}", t)))
                .collect();

            // Fix initial state and constant dynamics
            backend.assert(&px[0].eq(&Real::from_rational(0, 1)));
            backend.assert(&py[0].eq(&Real::from_rational(35, 10))); // 3.5
            backend.assert(&vx[0].eq(&Real::from_rational(10, 10))); // 1.0
            backend.assert(&vy[0].eq(&Real::from_rational(5, 10))); // 0.5
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
                    &backend,
                    &px[t],
                    &px[t + 1],
                    &py[t],
                    &py[t + 1],
                    &vx[t],
                    &vx[t + 1],
                    &vy[t],
                    &vy[t + 1],
                    &ax[t],
                    &ay[t],
                    &dt,
                    &half_dt,
                );
            }

            assert_eq!(backend.check(), SatResult::Sat);
            let model = backend.get_model().unwrap();

            let trajectory = extract_pedestrian_trajectory(
                &model, "ped1", &px, &py, &vx, &vy, &ax, &ay, &lanes, horizon, dt_val,
            )
            .unwrap();

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
            assert!((s1.cartesian.position.x - 0.5).abs() < 0.01); // 0 + 1.0*0.5
            assert!((s1.cartesian.position.y - 3.75).abs() < 0.01); // 3.5 + 0.5*0.5

            // Check third state (t=1.0)
            let s2 = &trajectory.states[2];
            assert!((s2.time - 1.0).abs() < 0.001);
            assert!((s2.cartesian.position.x - 1.0).abs() < 0.01); // 0.5 + 1.0*0.5
            assert!((s2.cartesian.position.y - 4.0).abs() < 0.01); // 3.75 + 0.5*0.5
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
            let half_dt = Real::from_rational(25, 100); // dt / 2

            // Create variables
            let px: Vec<_> = (0..=horizon)
                .map(|t| Real::new_const(format!("px_{}", t)))
                .collect();
            let py: Vec<_> = (0..=horizon)
                .map(|t| Real::new_const(format!("py_{}", t)))
                .collect();
            let vx: Vec<_> = (0..=horizon)
                .map(|t| Real::new_const(format!("vx_{}", t)))
                .collect();
            let vy: Vec<_> = (0..=horizon)
                .map(|t| Real::new_const(format!("vy_{}", t)))
                .collect();
            let ax: Vec<_> = (0..=horizon)
                .map(|t| Real::new_const(format!("ax_{}", t)))
                .collect();
            let ay: Vec<_> = (0..=horizon)
                .map(|t| Real::new_const(format!("ay_{}", t)))
                .collect();
            let lanes: Vec<_> = (0..=horizon)
                .map(|t| Int::new_const(format!("lane_{}", t)))
                .collect();

            // Fix lanes (pedestrian stays on lane 0 throughout)
            for t in 0..=horizon {
                backend.assert(&lanes[t].eq(&Int::from_i64(0)));
            }

            // 1. Encode initial state
            encode_pedestrian_initial_state(
                &backend, &px, &py, &vx, &vy, &ax, &ay, &actor, lane_width,
            );

            // 2. Encode kinematics and bounds for each step
            for t in 0..horizon {
                encode_pedestrian_kinematics_step(
                    &backend,
                    &px[t],
                    &px[t + 1],
                    &py[t],
                    &py[t + 1],
                    &vx[t],
                    &vx[t + 1],
                    &vy[t],
                    &vy[t + 1],
                    &ax[t],
                    &ay[t],
                    &dt,
                    &half_dt,
                );
                encode_pedestrian_bounds_step(&backend, &vx[t], &vy[t], &ax[t], &ay[t], &actor);
            }
            // Also bound the final velocity step
            encode_pedestrian_bounds_step(
                &backend,
                &vx[horizon],
                &vy[horizon],
                &ax[horizon - 1],
                &ay[horizon - 1],
                &actor,
            );

            assert_eq!(
                backend.check(),
                SatResult::Sat,
                "Full pedestrian encoding should be SAT"
            );
            let model = backend.get_model().unwrap();

            // 3. Extract trajectory
            let trajectory = extract_pedestrian_trajectory(
                &model,
                "ped_cross",
                &px,
                &py,
                &vx,
                &vy,
                &ax,
                &ay,
                &lanes,
                horizon,
                dt_val,
            )
            .unwrap();

            assert_eq!(trajectory.states.len(), horizon + 1);
            assert_eq!(trajectory.role, "pedestrian");

            // Verify physics consistency: the extracted trajectory must satisfy
            // the exact constant-acceleration update on both axes, and the
            // velocity update that goes with it. Asserted at 1e-9, not the 0.05
            // this used to allow against a forward-Euler expectation: Z3 values
            // are exact rationals, so the residual is zero to the last bit.
            let half_dt2 = 0.5 * dt_val * dt_val;
            for t in 0..horizon {
                let s = &trajectory.states[t];
                let s_next = &trajectory.states[t + 1];
                let (p, v, a) = (
                    &s.cartesian.position,
                    &s.cartesian.velocity,
                    &s.cartesian.acceleration,
                );

                let expected_px = p.x + v.vx * dt_val + a.ax * half_dt2;
                let expected_py = p.y + v.vy * dt_val + a.ay * half_dt2;

                assert!(
                    (s_next.cartesian.position.x - expected_px).abs() < 1e-9,
                    "px mismatch at t={}: {} vs expected {}",
                    t + 1,
                    s_next.cartesian.position.x,
                    expected_px
                );
                assert!(
                    (s_next.cartesian.position.y - expected_py).abs() < 1e-9,
                    "py mismatch at t={}: {} vs expected {}",
                    t + 1,
                    s_next.cartesian.position.y,
                    expected_py
                );
                assert!(
                    (s_next.cartesian.velocity.vx - (v.vx + a.ax * dt_val)).abs() < 1e-9,
                    "vx mismatch at t={}: {} vs expected {}",
                    t + 1,
                    s_next.cartesian.velocity.vx,
                    v.vx + a.ax * dt_val
                );
                assert!(
                    (s_next.cartesian.velocity.vy - (v.vy + a.ay * dt_val)).abs() < 1e-9,
                    "vy mismatch at t={}: {} vs expected {}",
                    t + 1,
                    s_next.cartesian.velocity.vy,
                    v.vy + a.ay * dt_val
                );
            }

            // Verify speed bounds hold at every step
            for t in 0..=horizon {
                let s = &trajectory.states[t];
                assert!(
                    s.cartesian.velocity.vx.abs() <= PEDESTRIAN_WALK_MAX_SPEED + 0.05,
                    "vx out of bounds at t={}: {}",
                    t,
                    s.cartesian.velocity.vx
                );
                assert!(
                    s.cartesian.velocity.vy.abs() <= PEDESTRIAN_WALK_MAX_SPEED + 0.05,
                    "vy out of bounds at t={}: {}",
                    t,
                    s.cartesian.velocity.vy
                );
            }
        });
    }
}
