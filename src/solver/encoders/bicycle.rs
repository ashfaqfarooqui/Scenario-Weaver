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
//! Hybrid approach:
//! - Longitudinal dynamics are linear: dx/dt = v, dv/dt = a
//! - Lateral dynamics use independent vy with linear ratio bounds: |vy| <= k * v
//! - Heading (θ) and steering (δ) are bounded variables with linear rate constraints
//!   (not coupled to position via NRA products)
//! - During stable phases: vy=0, θ=0, δ=0 (straight driving)
//! - During lane changes: vy bounded by velocity ratio, θ/δ bounded by rate limits

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
    encode_pedestrian_kinematics_step, extract_pedestrian_trajectory,
};

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
            let (_, max_steering_angle, max_steering_rate) =
                match self.get_actor_bicycle_params(actor_id) {
                    Ok(params) => params,
                    Err(_) => continue, // Skip if no params
                };

            // Steering angle bounds: -δ_max <= δ <= δ_max
            let delta_max_val = real_from_f64(max_steering_angle);
            let delta_min_val = real_from_f64(-max_steering_angle);

            for t in 0..=self.horizon {
                let delta_var = &self.steering_delta[actor_id][t];
                self.backend.assert(&delta_var.ge(&delta_min_val));
                self.backend.assert(&delta_var.le(&delta_max_val));

                // Heading angle bounds: -π/6 <= θ <= π/6 (±30° for small angle validity)
                let theta_var = &self.heading_theta[actor_id][t];
                let theta_max = std::f64::consts::PI / 6.0; // 30 degrees
                let theta_max_val = real_from_f64(theta_max);
                let theta_min_val = real_from_f64(-theta_max);
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

        // Velocity ratio constraint during lane change: |vy| <= k * v
        // k = 0.5 corresponds to the ±30° heading angle bound (sin(π/6) = 0.5)
        // This is more permissive than cartesian's k=0.15 because the bicycle model
        // uses heading/steering constraints for realism rather than a tight vy ratio.
        let k = Real::from_rational(5_i64, 10_i64);

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
        }
    }

    fn encode_kinematics(&mut self, dt: f64) {
        let dt_val = real_from_f64(dt);
        // Half of dt, for the trapezoidal position updates below.
        let half_dt = real_from_f64(dt / 2.0);
        let zero = Real::from_rational(0, 1);

        // Collect lane change data to determine stable vs transition phases
        let lane_changes_data = collect_lane_change_data(&self.spec, self.horizon);

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
            .map(|a| (a.id.clone(), a.role, a.direction, a.speed.max()))
            .collect();

        for (actor_id, role, direction, speed_max) in &actor_info {
            if *role == ActorRole::Pedestrian {
                // Handled by the point-mass loop above.
                continue;
            }

            // Get bicycle parameters for heading rate bound
            let (wheelbase, max_steering_angle, _) = match self.get_actor_bicycle_params(actor_id) {
                Ok(params) => params,
                Err(_) => continue,
            };

            // Compute max heading rate as a constant: v_max * delta_max / L
            let max_heading_rate = speed_max * max_steering_angle / wheelbase;
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
        for actor in &self.spec.actors {
            let actor_id = &actor.id;
            let speed_max = actor.speed.max();
            let speed_max_val = real_from_f64(speed_max);

            for t in 0..=self.horizon {
                let v_var = &self.speed_v[actor_id][t];
                self.backend.assert(&v_var.le(&speed_max_val));
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

            // Constant acceleration: a[t+1] = a[t] for all t.
            // Z3 picks one value in [a_min, a_max] and holds it for the full run,
            // producing smooth monotonic speed profiles with no longitudinal jitter.
            for t in 0..self.horizon {
                let a_t = &self.accelerations[actor_id][t];
                let a_t1 = &self.accelerations[actor_id][t + 1];
                self.backend.assert(&a_t1.eq(a_t));
            }
        }
    }

    fn encode_ttc_constraint(&self, actor1: &str, actor2: &str, min_ttc: f64, time: usize) -> Bool {
        let lane1 = &self.lanes[actor1][time];
        let lane2 = &self.lanes[actor2][time];

        let px1 = &self.positions_x[actor1][time];
        let px2 = &self.positions_x[actor2][time];
        let py1 = &self.positions_y[actor1][time];
        let py2 = &self.positions_y[actor2][time];

        // For bicycle model, speed_v is always >= 0. Multiply by actor direction
        // to get signed longitudinal velocity for correct relative velocity calculation.
        let actor1_dir = self.spec.get_actor(actor1).map_or(1, |a| a.direction);
        let actor2_dir = self.spec.get_actor(actor2).map_or(1, |a| a.direction);
        let raw_v1 = &self.speed_v[actor1][time];
        let raw_v2 = &self.speed_v[actor2][time];
        let v1 = if actor1_dir == 1 {
            raw_v1.clone()
        } else {
            -raw_v1
        };
        let v2 = if actor2_dir == 1 {
            raw_v2.clone()
        } else {
            -raw_v2
        };

        let min_ttc_val = real_from_f64(min_ttc);
        let epsilon = Real::from_rational(1_i64, 100_i64); // 0.01 m/s to avoid division by zero

        // Enhanced "same lane" condition: discrete lane match OR y-position proximity
        // This handles lane change transitions where discrete lane != smooth y-position
        let same_lane_discrete = lane1.eq(lane2);

        // Y-position proximity: |py1 - py2| < lane_width (vehicles in same lateral space)
        // FIXED: Use AND to properly check |py1 - py2| < lane_width
        // Both (py1-py2) < lane_width AND (py2-py1) < lane_width must be true
        // Using OR would be incorrect: if py1-py2 = 5.0 and lane_width = 3.5,
        // py2-py1 = -5.0 < 3.5 is TRUE, so OR would incorrectly return TRUE
        let lane_width = self.spec.get_lane_width();
        let lane_width_real = real_from_f64(lane_width);
        let py_diff_pos = py1 - py2;
        let py_diff_neg = py2 - py1;
        let y_proximity = Bool::and(&[
            &py_diff_pos.lt(&lane_width_real),
            &py_diff_neg.lt(&lane_width_real),
        ]);

        // Consider "same lane" if either discrete lanes match OR y-positions are close
        let same_lane = Bool::or(&[&same_lane_discrete, &y_proximity]);

        // Determine who is ahead and who is behind
        // Case 1: actor1 ahead, actor2 behind, actor2 faster
        // TTC = (px1 - px2) / (v2 - v1)
        let actor1_ahead = px1.gt(px2);
        let actor2_faster = v2.gt(&v1);
        let rel_vel_1 = &v2 - &v1;
        let distance_1 = px1 - px2;
        let collision_possible_1 =
            Bool::and(&[&actor1_ahead, &actor2_faster, &rel_vel_1.gt(&epsilon)]);
        // TTC > min_ttc means: distance / rel_vel > min_ttc
        // Equivalent to: distance > min_ttc * rel_vel
        let ttc_safe_1 = distance_1.gt(&(&min_ttc_val * &rel_vel_1));

        // Case 2: actor2 ahead, actor1 behind, actor1 faster
        // TTC = (px2 - px1) / (v1 - v2)
        let actor2_ahead = px2.gt(px1);
        let actor1_faster = v1.gt(&v2);
        let rel_vel_2 = &v1 - &v2;
        let distance_2 = px2 - px1;
        let collision_possible_2 =
            Bool::and(&[&actor2_ahead, &actor1_faster, &rel_vel_2.gt(&epsilon)]);
        let ttc_safe_2 = distance_2.gt(&(&min_ttc_val * &rel_vel_2));

        // Overall constraint:
        // If same_lane AND collision_possible_1, then ttc_safe_1
        // If same_lane AND collision_possible_2, then ttc_safe_2
        // Otherwise (not same lane OR no collision possible), constraint is automatically satisfied
        let case1_constraint = Bool::implies(
            &Bool::and(&[&same_lane, &collision_possible_1]),
            &ttc_safe_1,
        );
        let case2_constraint = Bool::implies(
            &Bool::and(&[&same_lane, &collision_possible_2]),
            &ttc_safe_2,
        );

        Bool::and(&[&case1_constraint, &case2_constraint])
    }

    fn encode_distance_constraint(
        &self,
        actor1: &str,
        actor2: &str,
        min_dist: f64,
        time: usize,
    ) -> Bool {
        let lane1 = &self.lanes[actor1][time];
        let lane2 = &self.lanes[actor2][time];

        let px1 = &self.positions_x[actor1][time];
        let px2 = &self.positions_x[actor2][time];
        let py1 = &self.positions_y[actor1][time];
        let py2 = &self.positions_y[actor2][time];

        let min_dist_val = real_from_f64(min_dist);

        // Enhanced "same lane" condition: discrete lane match OR y-position proximity
        // This handles lane change transitions where discrete lane != smooth y-position
        let same_lane_discrete = lane1.eq(lane2);

        // Y-position proximity: |py1 - py2| < lane_width (vehicles in same lateral space)
        // FIXED: Use AND to properly check |py1 - py2| < lane_width
        // Both (py1-py2) < lane_width AND (py2-py1) < lane_width must be true
        // Using OR would be incorrect: if py1-py2 = 5.0 and lane_width = 3.5,
        // py2-py1 = -5.0 < 3.5 is TRUE, so OR would incorrectly return TRUE
        let lane_width = self.spec.get_lane_width();
        let lane_width_real = real_from_f64(lane_width);
        let py_diff_pos = py1 - py2;
        let py_diff_neg = py2 - py1;
        let y_proximity = Bool::and(&[
            &py_diff_pos.lt(&lane_width_real),
            &py_diff_neg.lt(&lane_width_real),
        ]);

        // Consider "same lane" if either discrete lanes match OR y-positions are close
        let same_lane = Bool::or(&[&same_lane_discrete, &y_proximity]);

        // Distance constraint: |px1 - px2| >= min_dist
        // Equivalent to: (px1 - px2 >= min_dist) OR (px2 - px1 >= min_dist)
        let dist_fwd = (px1 - px2).ge(&min_dist_val);
        let dist_bwd = (px2 - px1).ge(&min_dist_val);
        let dist_safe = Bool::or(&[&dist_fwd, &dist_bwd]);

        // If same lane, then distance must be safe
        Bool::implies(&same_lane, &dist_safe)
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

        let mut trajectory = ActorTrajectory {
            id: actor_id.to_string(),
            role: role.to_string(),
            states: Vec::new(),
        };

        let dt = self.spec.time_step;

        // Extract trajectory at each time step
        for t in 0..=self.horizon {
            let time = t as f64 * dt;

            // Extract bicycle state variables using shared utilities
            let px = extract_real(model, &self.positions_x[actor_id][t])?;
            let py = extract_real(model, &self.positions_y[actor_id][t])?;
            let _theta = extract_real(model, &self.heading_theta[actor_id][t])?;
            let v = extract_real(model, &self.speed_v[actor_id][t])?;
            let a = extract_real(model, &self.accelerations[actor_id][t])?;
            let lane = extract_int(model, &self.lanes[actor_id][t])?;

            // Extract lateral velocity from the independent vy variable
            let vy = extract_real(model, &self.velocities_y[actor_id][t])?;

            // vx ≈ v (small angle: cos(θ) ≈ 1)
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
        // TODO: Implement lateral velocity bounds for bicycle model
        // This is implicitly handled by steering angle and heading angle constraints
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

            // Constant acceleration: a[0] == a[1]
            let a0 = eval_real(&model, &encoder.accelerations["ego"][0]);
            let a1 = eval_real(&model, &encoder.accelerations["ego"][1]);
            assert!(approx_eq(a0, a1, 0.1), "a0={} a1={}", a0, a1);
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
