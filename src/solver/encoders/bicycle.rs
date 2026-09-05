//! Bicycle model coordinate system encoder (hybrid LRA approach)
//!
//! Implements the CoordinateEncoder trait for kinematic bicycle model dynamics.
//! This encoder models vehicles with heading tracking, steering constraints,
//! and turn radius limitations while keeping all Z3 constraints in LRA
//! (linear real arithmetic) for efficient solving.
//!
//! State: (x, y, θ, v, δ) where θ is heading angle, v is speed, δ is steering
//! Controls: (a, δ) where a is longitudinal acceleration, δ is steering angle
//!
//! Hybrid approach ("Route B-lin", SW-11):
//! - Longitudinal dynamics are linear: dx/dt = v, dv/dt = a
//! - Lateral dynamics are driven by the heading: `vy[t] == v̄[t] * θ[t]`, where
//!   `v̄[t]` is a **constant** reference speed picked from a Bool-selected
//!   speed bucket containing `v[t]`. Constant × variable, so still QF_LRA.
//! - Heading follows the steering: `θ[t+1] == θ[t] + (v̄[t]/L) * δ[t] * dt`,
//!   again constant × variable.
//! - Steering is bounded by the turn radius: `|δ| <= atan(L / R_min)`.
//! - During stable phases: vy=0, θ=0, δ=0 (straight driving)
//! - During lane changes: the coupling above plus the Cartesian-matching
//!   lateral speed envelope (`|vy| <= 0.15*v` and `|vy| <= 2.0 m/s`).
//!
//! Before SW-11, θ and δ were related to nothing: `vy` was a free variable, θ
//! was pinned to zero outside lane changes and merely rate-bounded inside
//! them, and extraction threw θ away. Deleting every θ and δ would not have
//! changed a single output number.
//!
//! # The linearisation, and its error
//!
//! The exact kinematic bicycle model is `dy/dt = v*sin(θ)` and
//! `dθ/dt = (v/L)*tan(δ)`. Both are variable × variable products; with the
//! `Int` lane variable in the same problem that is QF_NIRA, which has no
//! decision procedure (Z3 answers `unknown`) and no usable `Optimize` support.
//! Two approximations buy linearity:
//!
//! 1. **Small angle.** `sin(θ) ≈ θ` and `tan(δ) ≈ δ`. The relative error is
//!    `θ²/6`; the heading bound keeps `|θ| <= atan(0.15) ≈ 8.5°`, so the error
//!    is at most 0.37 % on `vy`. `δ` is smaller still in practice — a highway
//!    lane change at 16 m/s uses `|δ| < 0.02 rad` — where the tangent error is
//!    under 0.02 %.
//! 2. **Reference speed.** `v` in the two products is replaced by the midpoint
//!    `v̄` of a speed bucket that `v[t]` is asserted to lie in. The relative
//!    error is therefore at most half a bucket width over `v̄`: under 8 % at
//!    16 m/s with the default 5 m/s buckets, and exactly zero when the actor's
//!    reachable speed span fits inside one bucket. A disjunction over buckets
//!    of linear constraints is still QF_LRA.
//!
//! Both errors are approximations of the *dynamics*, not of the exported
//! trajectory's internal consistency: `px`, `py`, `vx`, `vy`, `ax` and `ay`
//! remain mutually consistent to machine precision, because `py` is integrated
//! from the very `vy` the coupling defines.

use std::collections::HashMap;
use z3::ast::{Bool, Int, Real};
use z3::Model;

use crate::dsl::types::{ActorRole, ActorSpec, ScenarioSpec};
use crate::error::{Result, ScenarioGenError};
use crate::scenario::model::{
    Acceleration, ActorTrajectory, CartesianState, Position, State, Velocity,
};
use crate::solver::backend::Z3Backend;
use crate::solver::coordinate_encoder::CoordinateEncoder;
use crate::solver::encoder_utils::{
    collect_lane_change_data, extract_int, extract_real, real_from_f64,
};
use crate::solver::encoders::pedestrian::{
    encode_pedestrian_bounds_step, encode_pedestrian_initial_state,
    encode_pedestrian_kinematics_step, encode_pedestrian_lateral_containment,
    extract_pedestrian_trajectory,
};

/// Width of a reference-speed bucket, m/s.
///
/// Sets the linearisation error of `vy == v̄ * θ`: at most half a bucket,
/// relative to the bucket midpoint. Narrower buckets are more faithful and
/// cost one Bool per bucket per lane-change step.
const SPEED_BUCKET_WIDTH: f64 = 5.0;

/// Upper bound on the number of buckets emitted for one actor.
///
/// The reachable speed envelope of an actor with `acceleration: [-8, 3]` over
/// a 10 s horizon spans 110 m/s, which at 5 m/s a bucket would be 22 Bools per
/// step. Past this count the buckets are widened instead, trading fidelity for
/// solver time; the error bound in the module docs is then
/// `span / (2 * n * v̄)` rather than `SPEED_BUCKET_WIDTH / (2 * v̄)`.
const MAX_SPEED_BUCKETS: usize = 8;

/// Lateral/longitudinal velocity ratio during a lane change.
///
/// 0.15, the same constant `cartesian.rs` uses, corresponding to a heading of
/// `atan(0.15) ≈ 8.5°`. It was 0.5 here (a ±30° heading), justified in a
/// comment by the claim that "the bicycle model uses heading/steering
/// constraints for realism rather than a tight vy ratio" — which was false
/// while the heading constrained nothing. Now that the heading really does
/// drive `vy`, the two coordinate systems can and should use one number.
const LATERAL_VELOCITY_RATIO: f64 = 0.15;

/// Hard lateral speed ceiling, m/s — the same value and rationale as
/// `cartesian.rs::encode_lateral_velocity_bounds`.
const MAX_LATERAL_SPEED: f64 = 2.0;

/// Bicycle model coordinate system encoder
///
/// Uses (x, y, θ, v, δ) state variables with kinematic bicycle model dynamics.
/// Enforces steering angle limits and turn radius constraints.
pub struct BicycleEncoder<B: Z3Backend> {
    /// Z3 backend (Solver or Optimizer)
    backend: B,

    /// Scenario specification
    spec: ScenarioSpec,

    /// Number of time steps
    horizon: usize,

    // Variable maps: actor_id -> Vec<variable> (one per time step)
    /// Longitudinal positions (x coordinate, m)
    positions_x: HashMap<String, Vec<Real>>,

    /// Lateral positions (y coordinate, m)
    positions_y: HashMap<String, Vec<Real>>,

    /// Heading angles (θ, radians from +x axis)
    heading_theta: HashMap<String, Vec<Real>>,

    /// Speed (v, m/s, always >= 0)
    speed_v: HashMap<String, Vec<Real>>,

    /// Steering angles (δ, radians)
    steering_delta: HashMap<String, Vec<Real>>,

    /// Longitudinal accelerations (a, m/s²)
    accelerations: HashMap<String, Vec<Real>>,

    /// Lane numbers (integer)
    lanes: HashMap<String, Vec<Int>>,

    /// Lateral velocities (vy, m/s) — independent variables with linear bounds
    /// Bounded by |vy| <= k * v during lane changes, vy = 0 during stable phases
    velocities_y: HashMap<String, Vec<Real>>,

    /// Lateral accelerations (ay, m/s²)
    ///
    /// Added by SW-09. `vy` used to be an independent variable with nothing
    /// behind it, and extraction reported a hard-coded `ay = 0.0`, so the
    /// exported trajectory claimed zero lateral acceleration while `vy` stepped
    /// 0 -> -8 -> -3 m/s. `vy[t+1] = vy[t] + ay[t]*dt` now ties the two, with
    /// `|ay| <= spec.max_lateral_acceleration`.
    accelerations_y: HashMap<String, Vec<Real>>,

    /// Reference-speed buckets per actor (SW-11): half-open `[lo, hi)` spans
    /// paired with the constant `v̄` that stands in for the symbolic `v` in
    /// the heading coupling. They partition the real line, so exactly one
    /// contains any given `v[t]`.
    speed_buckets: HashMap<String, Vec<(f64, f64, f64)>>,

    /// Steps at which the heading coupling was asserted, per actor.
    ///
    /// False wherever `encode_kinematics` pins `vy = θ = δ = 0` — the coupling
    /// holds there for any reference speed, so nothing is emitted — and for
    /// pedestrians, who have no heading. Read at extraction to know whether the
    /// exported `vy` should come from θ or from the pinned zero.
    heading_coupled: HashMap<String, Vec<bool>>,
}

impl<B: Z3Backend> BicycleEncoder<B> {
    /// Create a new Bicycle encoder
    pub fn new(spec: ScenarioSpec, backend: B) -> Self {
        let horizon = spec.num_time_steps();

        Self {
            backend,
            spec,
            horizon,
            positions_x: HashMap::new(),
            positions_y: HashMap::new(),
            heading_theta: HashMap::new(),
            speed_v: HashMap::new(),
            steering_delta: HashMap::new(),
            accelerations: HashMap::new(),
            lanes: HashMap::new(),
            velocities_y: HashMap::new(),
            accelerations_y: HashMap::new(),
            speed_buckets: HashMap::new(),
            heading_coupled: HashMap::new(),
        }
    }

    /// Get bicycle parameters for an actor
    fn get_actor_bicycle_params(&self, actor_id: &str) -> Result<(f64, f64, f64)> {
        let actor = self.spec.get_actor(actor_id).ok_or_else(|| {
            ScenarioGenError::InvalidSpec(format!("Actor {} not found", actor_id))
        })?;

        let params = self.spec.get_bicycle_params(actor).ok_or_else(|| {
            ScenarioGenError::InvalidSpec(format!("No bicycle parameters for actor {}", actor_id))
        })?;

        Ok((
            params.wheelbase,
            params.max_steering_angle,
            params.max_steering_rate,
        ))
    }

    /// Encode initial state for a single actor (Bicycle-specific)
    fn encode_actor_initial_state(
        &mut self,
        actor_id: &str,
        lane: usize,
        pos_min: f64,
        pos_max: f64,
        speed_min: f64,
        speed_max: f64,
        accel_min: f64,
        accel_max: f64,
        _role: ActorRole,
        _direction: i32,
    ) {
        // Lane at t=0
        let lane_var = &self.lanes[actor_id][0];
        let lane_val = Int::from_i64(lane as i64);
        self.backend.assert(&lane_var.eq(&lane_val));

        // Position at t=0 (longitudinal)
        let px_var = &self.positions_x[actor_id][0];
        if (pos_min - pos_max).abs() < 1e-6 {
            // Fixed value
            let pos_val = real_from_f64(pos_min);
            self.backend.assert(&px_var.eq(&pos_val));
        } else {
            // Range
            let min_val = real_from_f64(pos_min);
            let max_val = real_from_f64(pos_max);
            self.backend.assert(&px_var.ge(&min_val));
            self.backend.assert(&px_var.le(&max_val));
        }

        // Lateral position at t=0 (computed from lane)
        let py_var = &self.positions_y[actor_id][0];
        let lane_width = self.spec.get_lane_width();
        let py_initial = lane as f64 * lane_width + lane_width / 2.0;
        let py_val = real_from_f64(py_initial);
        self.backend.assert(&py_var.eq(&py_val));

        // Speed at t=0 (always positive)
        let v_var = &self.speed_v[actor_id][0];
        if (speed_min - speed_max).abs() < 1e-6 {
            // Fixed value
            let speed_val = real_from_f64(speed_min);
            self.backend.assert(&v_var.eq(&speed_val));
        } else {
            // Range
            let min_val = real_from_f64(speed_min);
            let max_val = real_from_f64(speed_max);
            self.backend.assert(&v_var.ge(&min_val));
            self.backend.assert(&v_var.le(&max_val));
        }

        // Heading angle at t=0 (zero = aligned with nominal direction)
        // θ represents deviation from the actor's base direction, not absolute heading.
        // Backward direction is handled in the px kinematics (px ± v*dt), keeping
        // θ near zero so the small-angle approximation remains valid.
        let theta_var = &self.heading_theta[actor_id][0];
        let theta_val = Real::from_rational(0, 1);
        self.backend.assert(&theta_var.eq(&theta_val));

        // Steering angle at t=0 (straight)
        let delta_var = &self.steering_delta[actor_id][0];
        let delta_val = Real::from_rational(0, 1); // Straight (0 radians)
        self.backend.assert(&delta_var.eq(&delta_val));

        // Acceleration at t=0
        let a_var = &self.accelerations[actor_id][0];
        if (accel_min - accel_max).abs() < 1e-6 {
            // Fixed value
            let accel_val = real_from_f64(accel_min);
            self.backend.assert(&a_var.eq(&accel_val));
        } else {
            // Range
            let min_val = real_from_f64(accel_min);
            let max_val = real_from_f64(accel_max);
            self.backend.assert(&a_var.ge(&min_val));
            self.backend.assert(&a_var.le(&max_val));
        }
    }

    /// Encode bicycle-specific constraints (steering bounds, heading bounds, steering rate)
    fn encode_bicycle_constraints(&mut self) {
        for actor in &self.spec.actors {
            if actor.role == ActorRole::Pedestrian {
                // Pedestrians use simplified model (no steering)
                continue;
            }

            let actor_id = &actor.id;

            // Get bicycle parameters for this actor
            let Ok((wheelbase, _, max_steering_rate)) = self.get_actor_bicycle_params(actor_id)
            else {
                continue; // Skip if no params
            };

            // Steering angle bounds, written as the turn-radius constraint
            // `docs/coordinate-systems.md` has always advertised and no line of
            // this file previously enforced (SW-11/H6):
            //
            //   R >= R_min   <=>   |δ| <= atan(L / R_min)
            //
            // `BicycleParams::min_turn_radius` is `L / tan(δ_max)`, so this is
            // exactly `|δ| <= δ_max` — but routed through the radius, which is
            // what makes the radius a real quantity in the encoding instead of
            // a method with zero callers. Its formula was `L / δ_max` until
            // SW-11, 14 % off at the 0.6 rad default lock.
            let min_turn_radius = self
                .spec
                .get_actor(actor_id)
                .and_then(|a| self.spec.get_bicycle_params(a))
                .map_or(f64::INFINITY, |p| p.min_turn_radius());
            let delta_max = (wheelbase / min_turn_radius).atan();
            let delta_max_val = real_from_f64(delta_max);
            let delta_min_val = real_from_f64(-delta_max);

            // Heading angle bound. `sin(θ) ≈ θ` needs a small angle to be
            // honest, and the driving envelope is tighter than the maths: the
            // Cartesian encoder allows `|vy| <= 0.15*|vx|`, i.e. a heading of
            // atan(0.15). This used to be ±30° (sin(π/6) = 0.5, matching the
            // old k = 0.5 ratio), a 4.5 % small-angle error and a lateral
            // speed four times what Cartesian permits on the same YAML.
            let theta_max = LATERAL_VELOCITY_RATIO.atan();
            let theta_max_val = real_from_f64(theta_max);
            let theta_min_val = real_from_f64(-theta_max);

            for t in 0..=self.horizon {
                let delta_var = &self.steering_delta[actor_id][t];
                self.backend.assert(&delta_var.ge(&delta_min_val));
                self.backend.assert(&delta_var.le(&delta_max_val));

                let theta_var = &self.heading_theta[actor_id][t];
                self.backend.assert(&theta_var.ge(&theta_min_val));
                self.backend.assert(&theta_var.le(&theta_max_val));

                // Speed is always non-negative
                let v_var = &self.speed_v[actor_id][t];
                let zero = Real::from_rational(0, 1);
                self.backend.assert(&v_var.ge(&zero));
            }

            // Steering rate constraint: |δ[t+1] - δ[t]| <= max_steering_rate * dt
            let dt = self.spec.time_step;
            let max_delta_change = max_steering_rate * dt;
            let max_change_val = real_from_f64(max_delta_change);

            for t in 0..self.horizon {
                let delta_t = &self.steering_delta[actor_id][t];
                let delta_t1 = &self.steering_delta[actor_id][t + 1];
                let delta_diff = delta_t1 - delta_t;

                // |delta_diff| <= max_change
                // Encoded as: -max_change <= delta_diff <= max_change
                let neg_max_change_val = real_from_f64(-max_delta_change);
                self.backend.assert(&delta_diff.ge(&neg_max_change_val));
                self.backend.assert(&delta_diff.le(&max_change_val));
            }
        }
    }

    /// Encode lane-position coupling with lane change support.
    ///
    /// Uses CONSTANT lane indices (computed from the YAML spec + lane change schedule)
    /// to produce pure LRA bounds — avoids mixed integer-real arithmetic with symbolic
    /// lane variables which makes the NRA solver much slower.
    fn encode_lane_coupling_with_lane_changes(&mut self) {
        let lane_changes_data = collect_lane_change_data(&self.spec, self.horizon);
        let num_lanes = self.spec.get_num_lanes();

        // Collect actor IDs + initial lanes to avoid borrow checker issues
        let actor_data: Vec<_> = self
            .spec
            .actors
            .iter()
            .filter(|a| a.role != ActorRole::Pedestrian)
            .map(|a| (a.id.clone(), a.lane))
            .collect();

        for (actor_id, initial_lane) in actor_data {
            if let Some(changes) = lane_changes_data.get(&actor_id) {
                if changes.is_empty() {
                    // No lane changes: constant bounds for all time steps
                    for t in 0..=self.horizon {
                        self.encode_lane_position_bounds_const(&actor_id, t, initial_lane);
                    }
                } else {
                    // Phase 1: before first lane change — initial lane
                    let first_start = changes[0].start_step;
                    for t in 0..first_start.min(self.horizon + 1) {
                        self.encode_lane_position_bounds_const(&actor_id, t, initial_lane);
                    }

                    // Process each lane change and the stable phase that follows it
                    let mut current_lane = initial_lane as i32;
                    for (i, lc) in changes.iter().enumerate() {
                        // Compute target lane from direction
                        let lane_delta: i32 = match lc.direction {
                            crate::dsl::types::LaneChangeDirection::Right => 1,
                            crate::dsl::types::LaneChangeDirection::Left => -1,
                        };
                        let target_lane =
                            (current_lane + lane_delta).clamp(0, (num_lanes as i32) - 1);

                        // Encode transition with concrete lane indices
                        self.encode_smooth_lane_transition_bicycle(
                            &actor_id,
                            lc.start_step,
                            lc.end_step,
                            current_lane as usize,
                            target_lane as usize,
                        );

                        current_lane = target_lane;

                        // Stable phase after this change, until the next one or end
                        let next_start = if i + 1 < changes.len() {
                            changes[i + 1].start_step
                        } else {
                            self.horizon + 1
                        };
                        for t in (lc.end_step + 1)..next_start.min(self.horizon + 1) {
                            self.encode_lane_position_bounds_const(
                                &actor_id,
                                t,
                                current_lane as usize,
                            );
                        }
                    }
                }
            } else {
                // No lane changes configured for this actor
                for t in 0..=self.horizon {
                    self.encode_lane_position_bounds_const(&actor_id, t, initial_lane);
                }
            }
        }
    }

    /// Constrain py at time t to stay within the bounds of a known constant lane index.
    ///
    /// Uses rational constants only (pure LRA), avoiding the mixed integer-real
    /// arithmetic that arises from multiplying a symbolic Int lane variable by lane_width.
    fn encode_lane_position_bounds_const(&mut self, actor_id: &str, t: usize, lane: usize) {
        let lane_width = self.spec.get_lane_width();
        let py_var = &self.positions_y[actor_id][t];

        // Pin to lane center (consistent with cartesian encoder's lane coupling)
        let center_py = lane as f64 * lane_width + lane_width / 2.0;
        let center_val = real_from_f64(center_py);
        self.backend.assert(&py_var.eq(&center_val));

        // Also tie the discrete lane variable to this concrete lane
        let lane_var = &self.lanes[actor_id][t];
        let lane_val = Int::from_i64(lane as i64);
        self.backend.assert(&lane_var.eq(&lane_val));
    }

    /// Encode smooth lane transition for bicycle model using concrete lane indices.
    ///
    /// Uses the same cartesian-style pattern: constrain py near source center at start
    /// and near target center at end, with velocity ratio bounds during transition.
    /// Lane variables are set to source during transition and target at end.
    fn encode_smooth_lane_transition_bicycle(
        &mut self,
        actor_id: &str,
        start_step: usize,
        end_step: usize,
        source_lane: usize,
        target_lane: usize,
    ) {
        let lane_width = self.spec.get_lane_width();
        let end_clamped = end_step.min(self.horizon);

        // Compute lane centers
        let source_center = source_lane as f64 * lane_width + lane_width / 2.0;
        let target_center = target_lane as f64 * lane_width + lane_width / 2.0;

        let source_center_val = real_from_f64(source_center);
        let target_center_val = real_from_f64(target_center);
        let tolerance = Real::from_rational(5_i64, 10_i64); // 0.5m

        // Constrain py near source center at start
        let py_start = &self.positions_y[actor_id][start_step];
        self.backend
            .assert(&py_start.ge(&(&source_center_val - &tolerance)));
        self.backend
            .assert(&py_start.le(&(&source_center_val + &tolerance)));

        // Constrain py near target center at end
        let py_end = &self.positions_y[actor_id][end_clamped];
        self.backend
            .assert(&py_end.ge(&(&target_center_val - &tolerance)));
        self.backend
            .assert(&py_end.le(&(&target_center_val + &tolerance)));

        // Lane variable during the transition (SW-10/H2).
        //
        // The old encoding pinned `lane` on a schedule — `source` for every
        // step of the window, `target` only at the last one — while `py` was
        // constrained only at the two endpoints. Z3 reached the target centre
        // early and sat there while `lane` still read `source`; the finding
        // records t=5.0 and t=6.0 at y=5.25 (lane-1 centre) with lane=0.
        //
        // Instead, derive `lane` from `py`: `lane` must name the lane whose
        // lane_width-wide strip physically contains `py`. The cartesian
        // encoder writes this as |py - lane*w - w/2| <= w/2 directly, because
        // its source/target lanes are symbolic `Int`s. Here they are concrete
        // `usize`s, so the same relation is expressed as a two-way case split
        // over the only two lanes reachable in this window. That keeps this
        // file in pure LRA — no Int-to-Real coercion, as
        // `encode_lane_position_bounds_const` above is careful to avoid — and
        // is strictly stronger, since it also rules out any third lane index.
        //
        // The timing does not go away with the schedule: `py[start_step]` is
        // pinned within 0.5 m of the source centre and `py[end_clamped]`
        // within 0.5 m of the target centre above, and 0.5 m is inside the
        // half-width, so the case split forces lane == source at the first
        // step of the window and lane == target at the last, exactly as the
        // schedule's endpoints did.
        let source_lane_val = Int::from_i64(source_lane as i64);
        let target_lane_val = Int::from_i64(target_lane as i64);
        let half_width_val = real_from_f64(lane_width / 2.0);
        for t in start_step..=end_clamped {
            let lane_var = &self.lanes[actor_id][t];
            let py_var = &self.positions_y[actor_id][t];

            let at_source = lane_var.eq(&source_lane_val);
            let at_target = lane_var.eq(&target_lane_val);

            let off_source = py_var - &source_center_val;
            let in_source = Bool::and(&[
                &off_source.le(&half_width_val),
                &off_source.ge(&-&half_width_val),
            ]);

            let off_target = py_var - &target_center_val;
            let in_target = Bool::and(&[
                &off_target.le(&half_width_val),
                &off_target.ge(&-&half_width_val),
            ]);

            let case_split = Bool::or(&[&at_source, &at_target]);
            let source_consistent = at_source.implies(&in_source);
            let target_consistent = at_target.implies(&in_target);

            self.backend.assert(&case_split);
            self.backend.assert(&source_consistent);
            self.backend.assert(&target_consistent);
        }

        // Velocity ratio constraint during lane change: |vy| <= k * v.
        //
        // k is now `LATERAL_VELOCITY_RATIO` = 0.15, cartesian's value. It was
        // 0.5 — the sine of the old ±30° heading bound — on the argument that
        // "the bicycle model uses heading/steering constraints for realism
        // rather than a tight vy ratio", which was not true of a heading that
        // constrained nothing (SW-11/H5). At v = 16 m/s the old bound admitted
        // vy = 8.0 m/s: 28.8 km/h of pure sideways motion.
        //
        // This is the only bound here that reads the *symbolic* v, so it is the
        // one that stays tight when an actor slows below its bucket midpoint.
        let k = real_from_f64(LATERAL_VELOCITY_RATIO);

        for t in start_step..=end_clamped {
            let v_t = &self.speed_v[actor_id][t];
            let vy_t = &self.velocities_y[actor_id][t];

            // |vy| <= k * v (linear: constant k times variable v)
            let max_vy = v_t * &k;
            self.backend.assert(&vy_t.ge(&-&max_vy));
            self.backend.assert(&vy_t.le(&max_vy));
        }

        // Road bounds during transition
        let num_lanes = self.spec.get_num_lanes();
        let road_min = Real::from_rational(0, 1);
        let road_max = real_from_f64(num_lanes as f64 * lane_width);
        for t in start_step..=end_clamped {
            let py_t = &self.positions_y[actor_id][t];
            self.backend.assert(&py_t.ge(&road_min));
            self.backend.assert(&py_t.le(&road_max));
        }
    }

    /// The interval of speeds an actor can reach anywhere in the horizon.
    ///
    /// Derived from the spec, not from a single field: the initial speed range
    /// widened by the acceleration band over the full horizon, clipped below at
    /// zero (`v >= 0` is asserted) and above at `spec.max_velocity` when one is
    /// declared. Note that `actor.speed` is an *initial condition* — treating
    /// its max as a ceiling is exactly the H4 bug this issue fixes — so it is
    /// only the starting point here.
    fn reachable_speed_span(&self, actor: &ActorSpec) -> (f64, f64) {
        let t_end = self.horizon as f64 * self.spec.time_step;
        let lo = (actor.speed.min() + actor.acceleration.min() * t_end).max(0.0);
        let mut hi = actor.speed.max() + actor.acceleration.max() * t_end;
        if let Some(v_max) = self.spec.max_velocity {
            hi = hi.min(v_max);
        }
        (lo, hi.max(lo))
    }

    /// Reference-speed buckets for one actor, as `(lo, hi, v̄)` triples.
    ///
    /// The buckets tile [`Self::reachable_speed_span`] and their midpoints are
    /// the constants that stand in for the symbolic `v` in the heading
    /// coupling. The outermost edges are deliberately *not* asserted (see
    /// [`Self::encode_heading_coupling`]), so the tiling covers every real
    /// speed even if the span estimate above is wrong.
    fn speed_buckets(&self, actor: &ActorSpec) -> Vec<(f64, f64, f64)> {
        let (lo, hi) = self.reachable_speed_span(actor);
        let span = hi - lo;
        #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
        let n = ((span / SPEED_BUCKET_WIDTH).ceil().max(1.0) as usize).min(MAX_SPEED_BUCKETS);
        #[allow(clippy::cast_precision_loss)]
        let width = if span > 0.0 {
            span / n as f64
        } else {
            SPEED_BUCKET_WIDTH
        };
        #[allow(clippy::cast_precision_loss)]
        (0..n)
            .map(|i| {
                let b_lo = lo + i as f64 * width;
                let b_hi = b_lo + width;
                (b_lo, b_hi, f64::midpoint(b_lo, b_hi))
            })
            .collect()
    }

    /// The steps at which an actor's heading is allowed to be non-zero.
    ///
    /// `encode_kinematics` pins `vy = θ = δ = 0` outside lane changes, so the
    /// coupling is trivially satisfied there for any reference speed and no
    /// bucket machinery is emitted. Restricting it to the lane-change windows
    /// is what keeps the Bool count — and the solve time — small.
    fn heading_active_steps(&self, actor_id: &str) -> Vec<usize> {
        let lane_changes = collect_lane_change_data(&self.spec, self.horizon);
        let Some(changes) = lane_changes.get(actor_id) else {
            return Vec::new();
        };
        (0..=self.horizon)
            .filter(|t| {
                changes
                    .iter()
                    .any(|lc| *t >= lc.start_step && *t <= lc.end_step)
            })
            .collect()
    }

    /// Relate the heading and the steering to the motion — the whole point of
    /// having a bicycle model (SW-11/H6, decision D3).
    ///
    /// For every step where the heading may be non-zero, and for each
    /// reference-speed bucket `[lo, hi)` with midpoint `v̄`:
    ///
    /// ```text
    ///   lo <= v[t] < hi  =>  vy[t] == v̄ * θ[t]                     (dy/dt = v sinθ)
    ///                    &&  θ[t+1] == θ[t] + (v̄ / L) * δ[t] * dt  (dθ/dt = (v/L) tanδ)
    /// ```
    ///
    /// `v̄` and `v̄/L` are `f64` constants, so every product is constant ×
    /// variable and the encoding stays in QF_LRA. A variable × variable form of
    /// the same two equations, alongside the `Int` lane variable, would be
    /// QF_NIRA: no decision procedure, and no `--optimize`.
    ///
    /// The buckets partition the line — half-open, with the outermost edges
    /// left open — so exactly one antecedent holds at every step and no
    /// disjunction has to be asserted separately. That matters for solve time:
    /// an earlier draft introduced a Bool selector per bucket per step and let
    /// Z3 *choose* one, which turned 36 lane-change steps into an 8^36 search
    /// and took 12.6 s on `bicycle_lane_change.yaml` against 1.2 s for the
    /// guard form here, which the arithmetic solver simply propagates.
    ///
    /// On the step before a window opens both `θ[t]` and `δ[t]` are pinned to
    /// zero, so the update degenerates to `θ[t+1] == θ[t]` and needs no bucket;
    /// asserting it there is what forces a lane change to *begin* with zero
    /// heading and therefore zero lateral velocity.
    fn encode_heading_coupling(&mut self, dt: f64) {
        let dt_val = real_from_f64(dt);

        let actors: Vec<ActorSpec> = self
            .spec
            .actors
            .iter()
            .filter(|a| a.role != ActorRole::Pedestrian)
            .cloned()
            .collect();

        for actor in &actors {
            let actor_id = &actor.id;
            let Ok((wheelbase, _, _)) = self.get_actor_bicycle_params(actor_id) else {
                continue;
            };

            let active = self.heading_active_steps(actor_id);
            if active.is_empty() {
                continue;
            }
            let buckets = self.speed_buckets(actor);
            let last = buckets.len() - 1;

            for &t in &active {
                for (i, &(lo, hi, v_bar)) in buckets.iter().enumerate() {
                    let mut guard: Vec<Bool> = Vec::with_capacity(2);
                    let v_t = &self.speed_v[actor_id][t];
                    // Half-open [lo, hi); the outermost edges are left open so
                    // the buckets cover the whole line, not just the estimated
                    // reachable span.
                    if i > 0 {
                        let lo_val = real_from_f64(lo);
                        guard.push(v_t.ge(&lo_val));
                    }
                    if i < last {
                        let hi_val = real_from_f64(hi);
                        guard.push(v_t.lt(&hi_val));
                    }

                    let mut body: Vec<Bool> = Vec::with_capacity(2);

                    // vy[t] == v̄ * θ[t]
                    let v_bar_val = real_from_f64(v_bar);
                    let theta_t = &self.heading_theta[actor_id][t];
                    let vy_t = &self.velocities_y[actor_id][t];
                    body.push(vy_t.eq(&(theta_t * &v_bar_val)));

                    // θ[t+1] == θ[t] + (v̄ / L) * δ[t] * dt
                    if t < self.horizon {
                        let gain = real_from_f64(v_bar / wheelbase);
                        let delta_t = &self.steering_delta[actor_id][t];
                        let theta_t1 = &self.heading_theta[actor_id][t + 1];
                        let step = &(&(delta_t * &gain) * &dt_val);
                        body.push(theta_t1.eq(&(theta_t + step)));
                    }

                    let body = Bool::and(&body.iter().collect::<Vec<_>>());
                    if guard.is_empty() {
                        // A single bucket covering everything: no guard needed.
                        self.backend.assert(&body);
                    } else {
                        let guard = Bool::and(&guard.iter().collect::<Vec<_>>());
                        self.backend.assert(&guard.implies(&body));
                    }
                }
            }

            // Entry edge: the step before the window, where θ and δ are both
            // pinned to zero, so the heading update is bucket-independent.
            for t in 0..self.horizon {
                if !active.contains(&t) && active.contains(&(t + 1)) {
                    let theta_t = &self.heading_theta[actor_id][t];
                    let theta_t1 = &self.heading_theta[actor_id][t + 1];
                    self.backend.assert(&theta_t1.eq(theta_t));
                }
            }

            self.speed_buckets.insert(actor_id.clone(), buckets);
            if let Some(flags) = self.heading_coupled.get_mut(actor_id) {
                for t in active {
                    flags[t] = true;
                }
            }
        }
    }

    /// The reference speed that applies to `actor_id` at step `t`.
    ///
    /// `None` where the heading is pinned to zero, since no coupling was
    /// asserted there. Otherwise the midpoint of the one bucket containing
    /// `v[t]` — the buckets partition the line, so the lookup is total.
    ///
    /// The membership test is the *same* comparison the encoder asserted,
    /// evaluated in the model, so it is decided over Z3's exact rationals. The
    /// obvious alternative — comparing the extracted `f64` speed against the
    /// bucket edges — would disagree with the solver whenever a speed sits
    /// within a rounding step of an edge, and the exported `vy` would then be
    /// computed from the wrong `v̄` and stop matching the `py` integration.
    fn reference_speed_at(&self, model: &Model, actor_id: &str, t: usize) -> Option<f64> {
        if !*self.heading_coupled.get(actor_id)?.get(t)? {
            return None;
        }
        let buckets = self.speed_buckets.get(actor_id)?;
        let last = buckets.len().checked_sub(1)?;
        let v_t = &self.speed_v[actor_id][t];
        let holds = |b: &Bool| {
            model
                .eval(b, true)
                .and_then(|value| value.as_bool())
                .unwrap_or(false)
        };
        buckets
            .iter()
            .enumerate()
            .find(|(i, (lo, hi, _))| {
                let above = *i == 0 || {
                    let lo_val = real_from_f64(*lo);
                    holds(&v_t.ge(&lo_val))
                };
                let below = *i == last || {
                    let hi_val = real_from_f64(*hi);
                    holds(&v_t.lt(&hi_val))
                };
                above && below
            })
            .map(|(_, (_, _, v_bar))| *v_bar)
    }
}

impl<B: Z3Backend> CoordinateEncoder<B> for BicycleEncoder<B> {
    fn create_variables(&mut self, horizon: usize, spec: &ScenarioSpec) {
        for actor in &spec.actors {
            let actor_id = &actor.id;

            // Create variables for each time step (0 to horizon inclusive)
            let mut px_vars = Vec::new();
            let mut py_vars = Vec::new();
            let mut theta_vars = Vec::new();
            let mut v_vars = Vec::new();
            let mut delta_vars = Vec::new();
            let mut a_vars = Vec::new();
            let mut lane_vars = Vec::new();
            let mut vy_vars = Vec::new();
            let mut ay_vars = Vec::new();

            for t in 0..=horizon {
                px_vars.push(Real::new_const(format!("{}__px_{}", actor_id, t)));
                py_vars.push(Real::new_const(format!("{}__py_{}", actor_id, t)));
                theta_vars.push(Real::new_const(format!("{}__theta_{}", actor_id, t)));
                v_vars.push(Real::new_const(format!("{}__v_{}", actor_id, t)));
                delta_vars.push(Real::new_const(format!("{}__delta_{}", actor_id, t)));
                a_vars.push(Real::new_const(format!("{}__a_{}", actor_id, t)));
                lane_vars.push(Int::new_const(format!("{}__lane_{}", actor_id, t)));
                // Lateral velocity (independent variable with linear bounds)
                vy_vars.push(Real::new_const(format!("{}__vy_{}", actor_id, t)));
                ay_vars.push(Real::new_const(format!("{}__ay_{}", actor_id, t)));
            }

            self.positions_x.insert(actor_id.clone(), px_vars);
            self.positions_y.insert(actor_id.clone(), py_vars);
            self.heading_theta.insert(actor_id.clone(), theta_vars);
            self.speed_v.insert(actor_id.clone(), v_vars);
            self.steering_delta.insert(actor_id.clone(), delta_vars);
            self.accelerations.insert(actor_id.clone(), a_vars);
            self.lanes.insert(actor_id.clone(), lane_vars);
            self.velocities_y.insert(actor_id.clone(), vy_vars);
            self.accelerations_y.insert(actor_id.clone(), ay_vars);
            self.speed_buckets.insert(actor_id.clone(), Vec::new());
            self.heading_coupled
                .insert(actor_id.clone(), vec![false; horizon + 1]);
        }
    }

    fn encode_kinematics(&mut self, dt: f64) {
        let dt_val = real_from_f64(dt);
        // Half of dt, for the trapezoidal position updates below.
        let half_dt = real_from_f64(dt / 2.0);
        let zero = Real::from_rational(0, 1);

        // Collect lane change data to determine stable vs transition phases
        let lane_changes_data = collect_lane_change_data(&self.spec, self.horizon);

        // Same constant `cartesian.rs` computes for
        // `encode_pedestrian_lateral_containment` below.
        let road_width = self.spec.get_lane_width() * self.spec.get_num_lanes() as f64;

        // `max_lateral_acceleration` is a hard envelope with no `ConstraintMode`
        // of its own; it now bounds the acceleration that actually drives `vy`.
        let max_ay = real_from_f64(self.spec.max_lateral_acceleration);
        let neg_max_ay = real_from_f64(-self.spec.max_lateral_acceleration);

        // Pedestrians: the 2D point-mass model, shared with the Cartesian
        // encoder (`encoders::pedestrian`).
        //
        // This is the C3 fix. Every pedestrian-touching loop in this file used
        // to `continue` past them — no px, py or v update was ever asserted, so
        // a pedestrian's position at t > 0 was entirely unconstrained and the
        // solver was free to teleport them anywhere that satisfied the safety
        // propositions. `ScenarioSpec::validate` does not reject the
        // combination, so it was reachable from user YAML.
        //
        // The bicycle state maps onto the point-mass state directly:
        // `speed_v` is vx, `accelerations` is ax, `velocities_y` is vy, and
        // `accelerations_y` is ay. Heading and steering stay unconstrained and
        // unread for pedestrians, who have neither.
        for actor in &self.spec.actors {
            if actor.role != ActorRole::Pedestrian {
                continue;
            }
            let actor_id = &actor.id;
            for t in 0..=self.horizon {
                encode_pedestrian_bounds_step(
                    &self.backend,
                    &self.speed_v[actor_id][t],
                    &self.velocities_y[actor_id][t],
                    &self.accelerations[actor_id][t],
                    &self.accelerations_y[actor_id][t],
                    actor,
                );

                // `py` bounded to the drivable surface plus the sidewalk
                // margin, at every step (SW-24, following SW-23's cartesian
                // fix). Without this a bicycle-coordinate pedestrian drifts
                // arbitrarily far past the sidewalk strip; measured up to
                // py = 9.28 against a [-2, 9] envelope before this fix.
                encode_pedestrian_lateral_containment(
                    &self.backend,
                    &self.positions_y[actor_id][t],
                    road_width,
                );

                if t < self.horizon {
                    encode_pedestrian_kinematics_step(
                        &self.backend,
                        &self.positions_x[actor_id][t],
                        &self.positions_x[actor_id][t + 1],
                        &self.positions_y[actor_id][t],
                        &self.positions_y[actor_id][t + 1],
                        &self.speed_v[actor_id][t],
                        &self.speed_v[actor_id][t + 1],
                        &self.velocities_y[actor_id][t],
                        &self.velocities_y[actor_id][t + 1],
                        &self.accelerations[actor_id][t],
                        &self.accelerations_y[actor_id][t],
                        &dt_val,
                        &half_dt,
                    );
                }
            }
        }

        // Collect actor info to avoid borrow checker issues
        let actor_info: Vec<_> = self
            .spec
            .actors
            .iter()
            .map(|a| {
                (
                    a.id.clone(),
                    a.role,
                    a.direction,
                    self.reachable_speed_span(a).1,
                )
            })
            .collect();

        for (actor_id, role, direction, speed_ceiling) in &actor_info {
            if *role == ActorRole::Pedestrian {
                // Handled by the point-mass loop above.
                continue;
            }

            // Turn-radius bound on the heading rate, as a constant.
            //
            // On a circle of radius R the heading turns at v/R, so the tightest
            // radius the vehicle can hold gives the fastest it can turn:
            //   |dθ/dt| <= v_ceiling / R_min,   R_min = L / tan(δ_max).
            //
            // This was `speed.max() * δ_max / L` — the same quantity with two
            // errors: `speed.max()` is the *initial* speed spec (H4), and the
            // small-angle δ_max understates tan(δ_max) by 14 % at the 0.6 rad
            // default lock. `speed_ceiling` is now the reachable speed span's
            // upper end, so the bound stays valid for an actor that accelerates.
            let Some(params) = self
                .spec
                .get_actor(actor_id)
                .and_then(|a| self.spec.get_bicycle_params(a))
            else {
                continue;
            };
            let max_heading_rate = speed_ceiling / params.min_turn_radius();
            let max_theta_change = max_heading_rate * dt;
            let max_theta_change_val = real_from_f64(max_theta_change);
            let neg_max_theta_change_val = real_from_f64(-max_theta_change);

            // Determine which time steps are in a lane change
            let changes = lane_changes_data.get(actor_id.as_str());
            let is_in_lane_change = |t: usize| -> bool {
                if let Some(changes) = changes {
                    changes
                        .iter()
                        .any(|lc| t >= lc.start_step && t <= lc.end_step)
                } else {
                    false
                }
            };

            // Encode dynamics for each time step (all linear — no NRA)
            for t in 0..self.horizon {
                let px_t = &self.positions_x[actor_id][t];
                let py_t = &self.positions_y[actor_id][t];
                let v_t = &self.speed_v[actor_id][t];
                let a_t = &self.accelerations[actor_id][t];
                let vy_t = &self.velocities_y[actor_id][t];

                let px_t1 = &self.positions_x[actor_id][t + 1];
                let py_t1 = &self.positions_y[actor_id][t + 1];
                let v_t1 = &self.speed_v[actor_id][t + 1];

                // Longitudinal, exact for piecewise-constant acceleration:
                //   px[t+1] = px[t] ± (v[t]*dt + 0.5*a[t]*dt^2)
                // written in the equivalent trapezoidal form
                //   px[t+1] = px[t] ± (v[t] + v[t+1]) * dt/2,
                // which is the same constraint given the speed update asserted
                // below and keeps px coupled only to the speed chain (see the
                // matching comment in cartesian.rs). Still QF_LRA.
                // Direction handled here (not via heading angle)
                let px_step = (v_t + v_t1) * &half_dt;
                let px_next = if *direction == 1 {
                    px_t + &px_step
                } else {
                    px_t - &px_step
                };
                self.backend.assert(&px_t1.eq(&px_next));

                // Lateral, matching the longitudinal axis:
                //   vy[t+1] = vy[t] + ay[t]*dt
                //   py[t+1] = py[t] + (vy[t] + vy[t+1]) * dt/2
                // Before SW-09, `vy` was an independent variable with no
                // acceleration behind it and extraction reported ay = 0.0
                // regardless, so the trajectory in the JSON contradicted the
                // velocities printed beside it (npc: vy 0 -> -8 -> -3 m/s with
                // ay = 0.000). Chaining vy to a bounded ay also closes the null
                // space that made forward Euler necessary here: with vy free,
                // vy[t+1] = -vy[t] left py unchanged at zero cost.
                let ay_t = &self.accelerations_y[actor_id][t];
                let vy_t1 = &self.velocities_y[actor_id][t + 1];
                self.backend.assert(&vy_t1.eq(&(vy_t + &(ay_t * &dt_val))));

                let py_next = py_t + &((vy_t + vy_t1) * &half_dt);
                self.backend.assert(&py_t1.eq(&py_next));

                // Speed: v[t+1] = v[t] + a[t] * dt (linear)
                let v_next = v_t + &(a_t * &dt_val);
                self.backend.assert(&v_t1.eq(&v_next));
            }

            // Lateral acceleration envelope, horizon inclusive: the trapezoidal
            // py update reads vy[horizon], so ay[horizon] is reachable through
            // the chain and must not be left free to be extracted as anything.
            for t in 0..=self.horizon {
                let ay_t = &self.accelerations_y[actor_id][t];
                self.backend.assert(&ay_t.ge(&neg_max_ay));
                self.backend.assert(&ay_t.le(&max_ay));
            }

            // Phase-specific constraints for all time steps including horizon
            for t in 0..=self.horizon {
                let vy_t = &self.velocities_y[actor_id][t];
                let theta_t = &self.heading_theta[actor_id][t];
                let delta_t = &self.steering_delta[actor_id][t];

                if !is_in_lane_change(t) {
                    // Stable phase: straight driving
                    self.backend.assert(&vy_t.eq(&zero));
                    self.backend.assert(&theta_t.eq(&zero));
                    self.backend.assert(&delta_t.eq(&zero));
                }
                // Lane-change vy ratio bounds are set in encode_smooth_lane_transition_bicycle
            }

            // Heading rate constraint during lane changes: |θ[t+1] - θ[t]| <= max_rate * dt
            // This is linear (constant bound on variable differences)
            for t in 0..self.horizon {
                if is_in_lane_change(t) || is_in_lane_change(t + 1) {
                    let theta_t = &self.heading_theta[actor_id][t];
                    let theta_t1 = &self.heading_theta[actor_id][t + 1];
                    let theta_diff = theta_t1 - theta_t;
                    self.backend
                        .assert(&theta_diff.ge(&neg_max_theta_change_val));
                    self.backend.assert(&theta_diff.le(&max_theta_change_val));
                }
            }
        }

        // Encode lane change constraints and lane-position coupling
        self.encode_lane_coupling_with_lane_changes();

        // Encode bicycle-specific constraints (steering bounds, heading bounds, speed >= 0)
        self.encode_bicycle_constraints();

        // Tie θ and δ to the motion. Must run after the phase pinning above,
        // which is what tells it where the heading may be non-zero.
        self.encode_heading_coupling(dt);
    }

    fn encode_initial_conditions(&mut self) {
        // Pedestrians take the shared point-mass initial state. The bicycle
        // block below would otherwise pin their heading and steering to zero
        // and force a non-negative, direction-locked speed, none of which is
        // meaningful for an actor that has no wheels and crosses laterally.
        let pedestrians: Vec<ActorSpec> = self
            .spec
            .actors
            .iter()
            .filter(|a| a.role == ActorRole::Pedestrian)
            .cloned()
            .collect();
        let lane_width = self.spec.get_lane_width();
        for actor in &pedestrians {
            let actor_id = &actor.id;
            let lane_var = &self.lanes[actor_id][0];
            // `try_from` rather than `as`: a lane index is a `usize` and the
            // wrapping cast the rest of this file still uses is a live lint.
            let lane_val = Int::from_i64(i64::try_from(actor.lane).unwrap_or(i64::MAX));
            self.backend.assert(&lane_var.eq(&lane_val));
            encode_pedestrian_initial_state(
                &self.backend,
                &self.positions_x[actor_id],
                &self.positions_y[actor_id],
                &self.speed_v[actor_id],
                &self.velocities_y[actor_id],
                &self.accelerations[actor_id],
                &self.accelerations_y[actor_id],
                actor,
                lane_width,
            );
        }

        // Collect actor data first to avoid borrow checker issues
        let actor_data: Vec<_> = self
            .spec
            .actors
            .iter()
            .filter(|actor| actor.role != ActorRole::Pedestrian)
            .map(|actor| {
                (
                    actor.id.clone(),
                    actor.lane,
                    actor.position.min(),
                    actor.position.max(),
                    actor.speed.min(),
                    actor.speed.max(),
                    actor.acceleration.min(),
                    actor.acceleration.max(),
                    actor.role,
                    actor.direction,
                )
            })
            .collect();

        for (
            actor_id,
            lane,
            pos_min,
            pos_max,
            speed_min,
            speed_max,
            accel_min,
            accel_max,
            role,
            direction,
        ) in actor_data
        {
            self.encode_actor_initial_state(
                &actor_id, lane, pos_min, pos_max, speed_min, speed_max, accel_min, accel_max,
                role, direction,
            );
        }
    }

    fn encode_velocity_constraints(&mut self) {
        // SW-11/H4. This used to assert `v[t] <= actor.speed.max()` at every
        // step. `actor.speed` is the *initial condition* — `ValueOrRange`, the
        // band Z3 may pick the starting speed from — while the declared
        // velocity ceiling is `spec.max_velocity`, which is what
        // `tests/common/invariants.rs` checks and what `src/scenarios/mod.rs`
        // lowers to a `VelocityLT` proposition.
        //
        // The consequence was not cosmetic. With `speed: 15.0` the ceiling was
        // 15.0, so `a[t] <= 0` for the whole run regardless of the declared
        // `acceleration: [-8.0, 3.0]`; combined with the forced constant
        // acceleration below it, an actor could only coast or brake, and the
        // audit records an ego decelerating to a complete stop at t = 10 s in
        // the middle of a highway cut-in and the result being reported valid.
        // Identical YAML under `coordinate_system: cartesian` had no such cap.
        //
        // Cartesian applies no encoder-level speed envelope at all; matching it
        // exactly would drop this method to a no-op, but `spec.max_velocity` is
        // a hard envelope by construction, so it is asserted here when declared.
        let Some(v_max) = self.spec.max_velocity else {
            return;
        };
        let v_max_val = real_from_f64(v_max);

        for actor in &self.spec.actors {
            if actor.role == ActorRole::Pedestrian {
                // Pedestrians carry their own speed box from
                // `encoders::pedestrian`; a vehicle ceiling is not theirs.
                continue;
            }
            let actor_id = &actor.id;
            for t in 0..=self.horizon {
                let v_var = &self.speed_v[actor_id][t];
                self.backend.assert(&v_var.le(&v_max_val));
            }
        }
    }

    fn encode_acceleration_constraints(&mut self) {
        for actor in &self.spec.actors {
            let actor_id = &actor.id;
            let accel_min = actor.acceleration.min();
            let accel_max = actor.acceleration.max();

            let accel_min_val = real_from_f64(accel_min);
            let accel_max_val = real_from_f64(accel_max);

            for t in 0..=self.horizon {
                let a_var = &self.accelerations[actor_id][t];
                self.backend.assert(&a_var.ge(&accel_min_val));
                self.backend.assert(&a_var.le(&accel_max_val));
            }

            // The forced `a[t+1] == a[t]` that used to live here is gone
            // (SW-11/H4). It was justified as "smooth monotonic speed profiles
            // with no longitudinal jitter", but its real effect, next to the
            // `speed.max()` ceiling above, was to make acceleration impossible:
            // one value had to serve the whole run, and any positive value
            // breached the ceiling at some step. Cartesian imposes no such
            // constraint, so the same YAML produced qualitatively different
            // dynamics under the two coordinate systems. The acceleration band
            // asserted above is the whole of the longitudinal envelope now.
        }
    }

    fn extract_actor_trajectory(
        &self,
        model: &Model,
        actor_id: &str,
        role: &str,
    ) -> Result<ActorTrajectory> {
        // Pedestrians are a plain 2D point mass here, extracted by the shared
        // helper: (speed_v, velocities_y) are their (vx, vy) and
        // (accelerations, accelerations_y) their (ax, ay), all real solver
        // variables rather than the `ay = 0.0` the bicycle path hard-codes.
        if self
            .spec
            .get_actor(actor_id)
            .is_some_and(|a| a.role == ActorRole::Pedestrian)
        {
            return extract_pedestrian_trajectory(
                model,
                actor_id,
                &self.positions_x[actor_id],
                &self.positions_y[actor_id],
                &self.speed_v[actor_id],
                &self.velocities_y[actor_id],
                &self.accelerations[actor_id],
                &self.accelerations_y[actor_id],
                &self.lanes[actor_id],
                self.horizon,
                self.spec.time_step,
            );
        }

        let mut trajectory = ActorTrajectory::new(actor_id.to_string(), role.to_string());

        let dt = self.spec.time_step;

        // Extract trajectory at each time step
        for t in 0..=self.horizon {
            let time = t as f64 * dt;

            // Extract bicycle state variables using shared utilities
            let px = extract_real(model, &self.positions_x[actor_id][t])?;
            let py = extract_real(model, &self.positions_y[actor_id][t])?;
            let theta = extract_real(model, &self.heading_theta[actor_id][t])?;
            let v = extract_real(model, &self.speed_v[actor_id][t])?;
            let a = extract_real(model, &self.accelerations[actor_id][t])?;
            let lane = extract_int(model, &self.lanes[actor_id][t])?;

            // Lateral velocity from the heading, `vy = v̄ * sin(θ) ≈ v̄ * θ`,
            // which is what `docs/coordinate-systems.md` has always claimed the
            // extractor did. Until SW-11 this line read the free `vy` variable
            // and the line above was `let _theta = ...` — θ was extracted and
            // thrown away in the same breath.
            //
            // `v̄` is the reference speed of the bucket Z3 selected at this
            // step. Where the heading is pinned to zero no bucket exists and
            // `vy` is pinned to zero too, so any reference speed gives the same
            // answer and `v` is used. The encoder asserts `vy == v̄*θ` over
            // exact rationals, so this reproduces the solver's `vy` bit for bit
            // rather than approximating it — `test_theta_drives_vy` pins that.
            let v_bar = self.reference_speed_at(model, actor_id, t).unwrap_or(v);
            let vy = v_bar * theta;

            // vx ≈ v (small angle: cos(θ) ≈ 1; at |θ| <= atan(0.15) the error
            // is under 1.1 %, and taking it into account here would put the
            // exported vx out of step with the px integration, which uses v).
            let vx = v;

            // Lateral acceleration is a real solver variable now (SW-09), tied
            // to vy by vy[t+1] = vy[t] + ay[t]*dt, so it is read out rather
            // than reported as a hard-coded zero.
            let ax = a;
            let ay = extract_real(model, &self.accelerations_y[actor_id][t])?;

            let state = State {
                time,
                cartesian: CartesianState {
                    position: Position { x: px, y: py },
                    velocity: Velocity { vx, vy },
                    acceleration: Acceleration { ax, ay },
                    lane,
                },
            };

            trajectory.states.push(state);
        }

        Ok(trajectory)
    }

    fn get_longitudinal_pos(&self, actor_id: &str, time: usize) -> &Real {
        &self.positions_x[actor_id][time]
    }

    fn get_lateral_pos(&self, actor_id: &str, time: usize) -> &Real {
        &self.positions_y[actor_id][time]
    }

    fn get_longitudinal_vel(&self, actor_id: &str, time: usize) -> &Real {
        &self.speed_v[actor_id][time]
    }

    fn get_lane_var(&self, actor_id: &str, time: usize) -> &Int {
        &self.lanes[actor_id][time]
    }

    fn get_lateral_vel(&self, actor_id: &str, time: usize) -> &Real {
        // Return the derived lateral velocity (vy = v * θ)
        // This is constrained during kinematics encoding
        &self.velocities_y[actor_id][time]
    }

    fn encode_lane_velocity_constraints(&mut self) {
        // Encode lane bounds and single-lane-jump constraints
        let num_lanes = self.spec.get_num_lanes();
        let max_lane = Int::from_i64((num_lanes - 1) as i64);
        let zero_lane = Int::from_i64(0);

        for actor in &self.spec.actors {
            let actor_id = &actor.id;

            for t in 0..=self.horizon {
                let lane_var = &self.lanes[actor_id][t];

                // Lane bounds: 0 <= lane <= (num_lanes - 1)
                self.backend.assert(&lane_var.ge(&zero_lane));
                self.backend.assert(&lane_var.le(&max_lane));
            }
        }

        // Add single-lane-jump constraint: |lane[t+1] - lane[t]| <= 1
        // Prevents vehicles from jumping multiple lanes at once
        let one = Int::from_i64(1);
        let neg_one = Int::from_i64(-1);

        for actor in &self.spec.actors {
            if actor.role != ActorRole::Pedestrian {
                let actor_id = &actor.id;
                for t in 0..self.horizon {
                    let lane_t = &self.lanes[actor_id][t];
                    let lane_t1 = &self.lanes[actor_id][t + 1];
                    let diff = lane_t1 - lane_t;
                    // -1 <= diff <= 1
                    self.backend.assert(&diff.ge(&neg_one));
                    self.backend.assert(&diff.le(&one));
                }
            }
        }
    }

    fn encode_lateral_velocity_bounds(&mut self) {
        // SW-11/H5. This was an empty body whose comment claimed the bound was
        // "implicitly handled by steering angle and heading angle constraints".
        // It was not: θ and δ were related to `vy` by nothing at all, and the
        // only lateral limit in the encoder was the ratio `|vy| <= 0.5*v`,
        // which at v = 16 m/s admits vy = 8.0 m/s — 28.8 km/h sideways.
        //
        // The comment is now true as well as the bound: `encode_bicycle_
        // constraints` caps |θ| at atan(0.15) and `encode_heading_coupling`
        // asserts `vy == v̄*θ`, so the heading really does bound the lateral
        // speed, to `0.1489 * v̄`. What follows is the second, absolute cap —
        // the same 2.0 m/s `cartesian.rs::encode_lateral_velocity_bounds`
        // applies, and for the same reason: a 3.5 m lane change over 3 s needs
        // about 1.17 m/s, so 2.0 leaves room for a smooth profile and nothing
        // more.
        let max_vy = real_from_f64(MAX_LATERAL_SPEED);
        let neg_max_vy = real_from_f64(-MAX_LATERAL_SPEED);

        for actor in &self.spec.actors {
            if actor.role == ActorRole::Pedestrian {
                // Pedestrians cross laterally by definition; their envelope is
                // the point-mass speed box in `encoders::pedestrian`.
                continue;
            }
            let actor_id = &actor.id;
            for t in 0..=self.horizon {
                let vy_t = &self.velocities_y[actor_id][t];
                self.backend.assert(&vy_t.ge(&neg_max_vy));
                self.backend.assert(&vy_t.le(&max_vy));
            }
        }
    }

    fn backend(&self) -> &B {
        &self.backend
    }

    fn backend_mut(&mut self) -> &mut B {
        &mut self.backend
    }

    fn spec(&self) -> &ScenarioSpec {
        &self.spec
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dsl::types::{
        ActorRole, ActorSpec, BicycleConfig, BicycleParams, LaneChangeConfig, LaneChangeDirection,
        RoadSpec, ScenarioType, ValueOrRange,
    };
    use crate::solver::backend::SolverBackend;
    use crate::solver::coordinate_encoder::CoordinateEncoder;
    use z3::{Config, SatResult};

    fn create_bicycle_spec() -> ScenarioSpec {
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
                    bicycle_params: Some(BicycleParams {
                        wheelbase: 2.7,
                        max_steering_angle: 0.5,
                        max_steering_rate: 0.5,
                    }),
                },
                ActorSpec {
                    id: "npc".to_string(),
                    role: ActorRole::Npc,
                    lane: 0,
                    position: ValueOrRange::Range([60.0, 80.0]),
                    speed: ValueOrRange::Range([10.0, 20.0]),
                    acceleration: ValueOrRange::Range([-8.0, 3.0]),
                    direction: 1,
                    behavior: HashMap::new(),
                    lane_changes: vec![LaneChangeConfig {
                        direction: LaneChangeDirection::Right,
                        start_time: ValueOrRange::Range([2.5, 7.5]),
                        duration: ValueOrRange::Range([3.0, 4.0]),
                    }],
                    bicycle_params: Some(BicycleParams {
                        wheelbase: 2.7,
                        max_steering_angle: 0.5,
                        max_steering_rate: 0.5,
                    }),
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
            num_scenarios: 1,
            constraint_modes: crate::dsl::types::ConstraintModes::default(),
            optimization_target: crate::dsl::types::OptimizationTarget::None,
            max_acceleration: None,
            max_deceleration: None,
            max_velocity: None,
            min_velocity: None,
            min_lateral_distance: None,
            max_relative_velocity: None,
            max_lateral_acceleration: 2.0,
            coordinate_system: crate::dsl::types::CoordinateSystem::Bicycle,
            bicycle_config: Some(BicycleConfig {
                default_wheelbase: 2.7,
                default_max_steering_angle: 0.5,
                default_max_steering_rate: 0.5,
            }),
        }
    }

    fn eval_real(model: &z3::Model, var: &Real) -> f64 {
        crate::solver::encoder_utils::extract_real(model, var).unwrap()
    }

    fn eval_int(model: &z3::Model, var: &Int) -> i64 {
        crate::solver::encoder_utils::extract_int(model, var).unwrap() as i64
    }

    fn approx_eq(a: f64, b: f64, eps: f64) -> bool {
        (a - b).abs() < eps
    }

    #[test]
    fn test_bicycle_encoder_creation() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let spec = create_bicycle_spec();
            let expected_horizon = spec.num_time_steps();
            let backend = SolverBackend::new();
            let encoder = BicycleEncoder::new(spec, backend);

            // The horizon is derived from the spec, and no variables exist until
            // `create_variables` is called.
            assert_eq!(encoder.horizon, expected_horizon);
            assert_eq!(encoder.horizon, 20, "10.0 s at 0.5 s steps");
            assert!(encoder.positions_x.is_empty());
            assert!(encoder.positions_y.is_empty());
            assert!(encoder.heading_theta.is_empty());
            assert!(encoder.speed_v.is_empty());
            assert!(encoder.steering_delta.is_empty());
            assert!(encoder.accelerations.is_empty());
            assert!(encoder.lanes.is_empty());
            assert!(encoder.velocities_y.is_empty());
        });
    }

    #[test]
    fn test_create_variables() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let spec = create_bicycle_spec();
            let backend = SolverBackend::new();
            let mut encoder = BicycleEncoder::new(spec.clone(), backend);
            encoder.create_variables(20, &spec);

            assert_eq!(encoder.positions_x["ego"].len(), 21);
            assert_eq!(encoder.positions_y["npc"].len(), 21);
            assert_eq!(encoder.speed_v["ego"].len(), 21);
            assert_eq!(encoder.heading_theta["ego"].len(), 21);
            assert_eq!(encoder.steering_delta["npc"].len(), 21);
            assert_eq!(encoder.accelerations["ego"].len(), 21);
            assert_eq!(encoder.lanes["ego"].len(), 21);
            assert_eq!(encoder.velocities_y["npc"].len(), 21);

            let _ = encoder.get_longitudinal_pos("ego", 0);
            let _ = encoder.get_lateral_pos("npc", 10);
            let _ = encoder.get_longitudinal_vel("ego", 20);
            let _ = encoder.get_lateral_vel("npc", 5);
            let _ = encoder.get_lane_var("ego", 0);
        });
    }

    #[test]
    fn test_encode_initial_conditions() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let spec = create_bicycle_spec();
            let backend = SolverBackend::new();
            let mut encoder = BicycleEncoder::new(spec.clone(), backend);
            encoder.create_variables(20, &spec);
            encoder.encode_initial_conditions();

            assert_eq!(encoder.backend.check(), SatResult::Sat);
            let model = encoder.backend.get_model().unwrap();

            let ego_px = eval_real(&model, &encoder.positions_x["ego"][0]);
            assert!(approx_eq(ego_px, 50.0, 0.1), "ego px={}", ego_px);

            let ego_v = eval_real(&model, &encoder.speed_v["ego"][0]);
            assert!(approx_eq(ego_v, 15.0, 0.1), "ego v={}", ego_v);

            let ego_lane = eval_int(&model, &encoder.lanes["ego"][0]);
            assert_eq!(ego_lane, 1);
        });
    }

    #[test]
    fn test_encode_kinematics() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let spec = create_bicycle_spec();
            let backend = SolverBackend::new();
            let mut encoder = BicycleEncoder::new(spec.clone(), backend);
            encoder.create_variables(20, &spec);
            encoder.encode_initial_conditions();
            encoder.encode_kinematics(0.5);

            assert_eq!(encoder.backend.check(), SatResult::Sat);
            let model = encoder.backend.get_model().unwrap();

            let dt = 0.5;
            let px0 = eval_real(&model, &encoder.positions_x["ego"][0]);
            let px1 = eval_real(&model, &encoder.positions_x["ego"][1]);
            let v0 = eval_real(&model, &encoder.speed_v["ego"][0]);
            let v1 = eval_real(&model, &encoder.speed_v["ego"][1]);
            let a0 = eval_real(&model, &encoder.accelerations["ego"][0]);

            // v[1] = v[0] + a[0]*dt, exactly.
            assert!(
                approx_eq(v1, v0 + a0 * dt, 1e-9),
                "v1={v1} expected={}",
                v0 + a0 * dt
            );

            // px[1] = px[0] + v[0]*dt + 0.5*a[0]*dt^2 (direction = 1).
            // This test previously asserted the forward-Euler `px0 + v0*dt` at
            // a 0.1 tolerance, which is what the encoder used to assert
            // (SW-08/H1); the two differ by 0.5*a*dt^2, 1.0 m per step here at
            // a = -8, dt = 0.5, so the old form is not merely imprecise.
            let expected_px1 = px0 + v0 * dt + 0.5 * a0 * dt * dt;
            assert!(
                approx_eq(px1, expected_px1, 1e-9),
                "px1={px1} expected={expected_px1} (px0={px0}, v0={v0}, a0={a0})"
            );
            // Equivalently, the trapezoidal form the encoder asserts.
            assert!(
                approx_eq(px1, px0 + (v0 + v1) * dt / 2.0, 1e-9),
                "px1={px1} disagrees with the trapezoidal form"
            );
        });
    }

    #[test]
    fn test_velocity_constraints() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let spec = create_bicycle_spec();
            let backend = SolverBackend::new();
            let mut encoder = BicycleEncoder::new(spec.clone(), backend);
            encoder.create_variables(20, &spec);
            encoder.encode_initial_conditions();
            encoder.encode_velocity_constraints();

            // Add constraint that ego speed at t=5 is negative — should be UNSAT
            // because encode_kinematics (called via bicycle_constraints in kinematics)
            // enforces v >= 0. But velocity_constraints only adds upper bound.
            // The v >= 0 is in encode_bicycle_constraints called from encode_kinematics.
            // So let's encode kinematics too.
            encoder.encode_kinematics(0.5);

            // Now assert v < 0 at some step
            let neg = Real::from_rational(-1, 1);
            encoder.backend.assert(&encoder.speed_v["ego"][5].lt(&neg));

            assert_eq!(encoder.backend.check(), SatResult::Unsat);
        });
    }

    /// The acceleration band is the whole of the longitudinal envelope.
    ///
    /// This test used to assert `a[0] == a[1]`, the forced constant
    /// acceleration SW-11 removed. It is not weakened to make the removal pass:
    /// it now asserts the bound that is actually claimed (every `a[t]` inside
    /// the declared range) *and* that the profile is free to vary, which is the
    /// property whose absence made an actor with `acceleration: [-8.0, 3.0]`
    /// unable to accelerate at all next to the `speed.max()` ceiling.
    #[test]
    fn test_acceleration_constraints() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let spec = create_bicycle_spec();
            let backend = SolverBackend::new();
            let mut encoder = BicycleEncoder::new(spec.clone(), backend);
            encoder.create_variables(20, &spec);
            encoder.encode_initial_conditions();
            encoder.encode_acceleration_constraints();

            assert_eq!(encoder.backend.check(), SatResult::Sat);
            let model = encoder.backend.get_model().unwrap();

            for t in 0..=20 {
                let a = eval_real(&model, &encoder.accelerations["ego"][t]);
                assert!(
                    (-8.0 - 1e-9..=3.0 + 1e-9).contains(&a),
                    "a[{t}]={a} outside the declared [-8.0, 3.0]"
                );
            }

            // A non-constant profile is now reachable.
            let a0 = &encoder.accelerations["ego"][0];
            let a1 = &encoder.accelerations["ego"][1];
            encoder.backend.assert(&a0.eq(&real_from_f64(-8.0)));
            encoder.backend.assert(&a1.eq(&real_from_f64(3.0)));
            assert_eq!(
                encoder.backend.check(),
                SatResult::Sat,
                "a[t+1] == a[t] should no longer be forced"
            );
        });
    }

    /// Run the standard encoding pipeline (the order `src/lib.rs` and
    /// `src/solver/multi_solve.rs` use) so a test exercises what production
    /// encodes rather than a subset of it.
    fn encode_all(encoder: &mut BicycleEncoder<SolverBackend>, spec: &ScenarioSpec) {
        let horizon = spec.num_time_steps();
        encoder.create_variables(horizon, spec);
        encoder.encode_initial_conditions();
        encoder.encode_kinematics(spec.time_step);
        encoder.encode_velocity_constraints();
        encoder.encode_acceleration_constraints();
        encoder.encode_lane_velocity_constraints();
        encoder.encode_lateral_velocity_bounds();
    }

    /// SW-11/H4: `actor.speed` is an initial condition, not a ceiling.
    ///
    /// The ego declares `speed: 15.0` and `acceleration: [-8.0, 3.0]`. Before
    /// the fix `v[t] <= actor.speed.max() = 15.0` was asserted at every step,
    /// so no positive acceleration was consistent with the kinematics and the
    /// actor could only coast or brake; the audit records one coasting to a
    /// complete stop at t = 10 s in the middle of a highway cut-in.
    #[test]
    fn test_actor_can_accelerate_past_its_initial_speed() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let spec = create_bicycle_spec();
            assert_eq!(spec.actors[0].speed.max(), 15.0, "ego starts at 15 m/s");
            assert!(spec.max_velocity.is_none(), "no declared velocity ceiling");

            let mut encoder = BicycleEncoder::new(spec.clone(), SolverBackend::new());
            encode_all(&mut encoder, &spec);

            let horizon = spec.num_time_steps();
            let faster = real_from_f64(20.0);
            encoder
                .backend
                .assert(&encoder.speed_v["ego"][horizon].ge(&faster));

            assert_eq!(
                encoder.backend.check(),
                SatResult::Sat,
                "the ego must be able to reach 20 m/s from 15 m/s at a <= 3 m/s^2"
            );
        });
    }

    /// The declared ceiling is `spec.max_velocity`, and it is still enforced.
    #[test]
    fn test_max_velocity_is_the_ceiling() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let mut spec = create_bicycle_spec();
            spec.max_velocity = Some(18.0);

            let mut encoder = BicycleEncoder::new(spec.clone(), SolverBackend::new());
            encode_all(&mut encoder, &spec);

            let horizon = spec.num_time_steps();
            let over = real_from_f64(18.5);
            encoder
                .backend
                .assert(&encoder.speed_v["ego"][horizon].ge(&over));

            assert_eq!(
                encoder.backend.check(),
                SatResult::Unsat,
                "spec.max_velocity = 18.0 must bound v"
            );
        });
    }

    /// SW-11/H5: the lateral velocity is bounded comparably to Cartesian.
    ///
    /// The old encoding's only lateral limit was `|vy| <= 0.5*v`, so at
    /// v = 16 m/s a lateral velocity of 8.0 m/s — 28.8 km/h of pure sideways
    /// motion — was a legal solution. Cartesian's equivalent is a hard 2.0 m/s.
    #[test]
    fn test_lateral_velocity_is_bounded_like_cartesian() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let spec = create_bicycle_spec();
            // Mid-window step of the npc's lane change (the window is steps
            // 10..=17 for this spec), where vy is free to be non-zero at all.
            // The first step of a window is not usable: the heading coupling
            // makes a lane change begin at zero heading, hence zero vy.
            let lane_change_step = 13;

            for (target, expected) in [
                (8.0_f64, SatResult::Unsat),
                (3.0, SatResult::Unsat),
                (1.0, SatResult::Sat),
            ] {
                let mut encoder = BicycleEncoder::new(spec.clone(), SolverBackend::new());
                encode_all(&mut encoder, &spec);
                let vy = &encoder.velocities_y["npc"][lane_change_step];
                encoder.backend.assert(&vy.eq(&real_from_f64(target)));
                assert_eq!(
                    encoder.backend.check(),
                    expected,
                    "vy = {target} m/s at step {lane_change_step}"
                );
            }
        });
    }

    /// SW-11/H6: θ is asserted against `vy`, and drives it.
    ///
    /// Two properties, both of which failed before: `vy == v̄ * θ` holds at
    /// every step (θ is no longer decorative), and θ is genuinely non-zero
    /// during a lane change (the coupling is not satisfied vacuously by
    /// everything being pinned to zero).
    #[test]
    fn test_theta_drives_vy() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let spec = create_bicycle_spec();
            let mut encoder = BicycleEncoder::new(spec.clone(), SolverBackend::new());
            encode_all(&mut encoder, &spec);

            // Force a real lane displacement so the solver cannot answer with a
            // straight line.
            assert_eq!(encoder.backend.check(), SatResult::Sat);
            let model = encoder.backend.get_model().unwrap();

            let horizon = spec.num_time_steps();
            let mut saw_nonzero_theta = false;
            for t in 0..=horizon {
                let theta = eval_real(&model, &encoder.heading_theta["npc"][t]);
                let vy = eval_real(&model, &encoder.velocities_y["npc"][t]);
                let v = eval_real(&model, &encoder.speed_v["npc"][t]);
                let v_bar = encoder.reference_speed_at(&model, "npc", t).unwrap_or(v);

                assert!(
                    approx_eq(vy, v_bar * theta, 1e-9),
                    "t={t}: vy={vy} but v_bar*theta={} (v_bar={v_bar}, theta={theta})",
                    v_bar * theta
                );
                if theta.abs() > 1e-9 {
                    saw_nonzero_theta = true;
                }
            }
            assert!(
                saw_nonzero_theta,
                "the npc changes lanes, so its heading must leave zero"
            );
        });
    }

    /// The steering bound is the turn-radius bound, and the turn radius is the
    /// exact kinematic one.
    #[test]
    fn test_steering_is_bounded_by_the_turn_radius() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let spec = create_bicycle_spec();
            let params = spec.get_bicycle_params(&spec.actors[1]).unwrap();
            let r_min = params.min_turn_radius();
            assert!(
                approx_eq(r_min, 2.7 / 0.5_f64.tan(), 1e-12),
                "R_min must be L/tan(delta_max), got {r_min}"
            );

            let delta_max = (params.wheelbase / r_min).atan();
            let mut encoder = BicycleEncoder::new(spec.clone(), SolverBackend::new());
            encode_all(&mut encoder, &spec);
            encoder
                .backend
                .assert(&encoder.steering_delta["npc"][10].ge(&real_from_f64(delta_max * 1.01)));
            assert_eq!(encoder.backend.check(), SatResult::Unsat);
        });
    }

    /// The reference-speed buckets partition the line: any speed lands in
    /// exactly one, including speeds outside the estimated reachable span.
    #[test]
    fn test_speed_buckets_partition_the_line() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let spec = create_bicycle_spec();
            let encoder = BicycleEncoder::new(spec.clone(), SolverBackend::new());
            let buckets = encoder.speed_buckets(&spec.actors[1]);
            assert!(!buckets.is_empty());
            let last = buckets.len() - 1;
            for v in [0.0_f64, 1.0, 12.5, 17.5, 40.0, 1000.0] {
                let hits = buckets
                    .iter()
                    .enumerate()
                    .filter(|(i, (lo, hi, _))| (*i == 0 || v >= *lo) && (*i == last || v < *hi))
                    .count();
                assert_eq!(hits, 1, "v={v} landed in {hits} buckets");
            }
        });
    }

    #[test]
    fn test_lane_bounds() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let spec = create_bicycle_spec();
            let backend = SolverBackend::new();
            let mut encoder = BicycleEncoder::new(spec.clone(), backend);
            encoder.create_variables(20, &spec);
            encoder.encode_initial_conditions();
            encoder.encode_lane_velocity_constraints();

            assert_eq!(encoder.backend.check(), SatResult::Sat);
            let model = encoder.backend.get_model().unwrap();

            // Lane should be bounded [0, num_lanes-1] = [0, 1]
            for t in 0..=20 {
                let lane = eval_int(&model, &encoder.lanes["ego"][t]);
                assert!(lane >= 0 && lane <= 1, "lane[{}]={} out of bounds", t, lane);
            }
        });
    }

    #[test]
    fn test_full_bicycle_scenario() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let spec = create_bicycle_spec();
            let backend = SolverBackend::new();
            let mut encoder = BicycleEncoder::new(spec.clone(), backend);
            let horizon = spec.num_time_steps();
            encoder.create_variables(horizon, &spec);
            encoder.encode_initial_conditions();
            encoder.encode_kinematics(0.5);
            encoder.encode_velocity_constraints();
            encoder.encode_acceleration_constraints();
            encoder.encode_lane_velocity_constraints();

            assert_eq!(encoder.backend.check(), SatResult::Sat);
        });
    }

    #[test]
    fn test_extract_trajectory() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let spec = create_bicycle_spec();
            let backend = SolverBackend::new();
            let mut encoder = BicycleEncoder::new(spec.clone(), backend);
            let horizon = spec.num_time_steps();
            encoder.create_variables(horizon, &spec);
            encoder.encode_initial_conditions();
            encoder.encode_kinematics(0.5);
            encoder.encode_velocity_constraints();
            encoder.encode_acceleration_constraints();
            encoder.encode_lane_velocity_constraints();

            assert_eq!(encoder.backend.check(), SatResult::Sat);
            let model = encoder.backend.get_model().unwrap();

            let ego_traj = encoder
                .extract_actor_trajectory(&model, "ego", "ego")
                .unwrap();
            let npc_traj = encoder
                .extract_actor_trajectory(&model, "npc", "npc")
                .unwrap();

            // Should have horizon+1 states
            assert_eq!(ego_traj.states.len(), horizon + 1);
            assert_eq!(npc_traj.states.len(), horizon + 1);

            // Verify actor IDs
            assert_eq!(ego_traj.id, "ego");
            assert_eq!(npc_traj.id, "npc");
        });
    }
}
