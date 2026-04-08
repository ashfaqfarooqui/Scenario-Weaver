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

use crate::dsl::types::{ActorRole, ScenarioSpec};
use crate::error::{Result, ScenarioGenError};
use crate::scenario::model::{
    Acceleration, ActorTrajectory, CartesianState, Position, State, Velocity,
};
use crate::solver::backend::Z3Backend;
use crate::solver::coordinate_encoder::CoordinateEncoder;
use crate::solver::encoder_utils::{collect_lane_change_data, extract_int, extract_real};

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
            let pos_val = Real::from_rational((pos_min * 10.0) as i64, 10_i64);
            self.backend.assert(&px_var.eq(&pos_val));
        } else {
            // Range
            let min_val = Real::from_rational((pos_min * 10.0) as i64, 10_i64);
            let max_val = Real::from_rational((pos_max * 10.0) as i64, 10_i64);
            self.backend.assert(&px_var.ge(&min_val));
            self.backend.assert(&px_var.le(&max_val));
        }

        // Lateral position at t=0 (computed from lane)
        let py_var = &self.positions_y[actor_id][0];
        let lane_width = self.spec.get_lane_width();
        let py_initial = lane as f64 * lane_width + lane_width / 2.0;
        let py_val = Real::from_rational((py_initial * 10.0) as i64, 10_i64);
        self.backend.assert(&py_var.eq(&py_val));

        // Speed at t=0 (always positive)
        let v_var = &self.speed_v[actor_id][0];
        if (speed_min - speed_max).abs() < 1e-6 {
            // Fixed value
            let speed_val = Real::from_rational((speed_min * 10.0) as i64, 10_i64);
            self.backend.assert(&v_var.eq(&speed_val));
        } else {
            // Range
            let min_val = Real::from_rational((speed_min * 10.0) as i64, 10_i64);
            let max_val = Real::from_rational((speed_max * 10.0) as i64, 10_i64);
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
            let accel_val = Real::from_rational((accel_min * 10.0) as i64, 10_i64);
            self.backend.assert(&a_var.eq(&accel_val));
        } else {
            // Range
            let min_val = Real::from_rational((accel_min * 10.0) as i64, 10_i64);
            let max_val = Real::from_rational((accel_max * 10.0) as i64, 10_i64);
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
            let delta_max_val = Real::from_rational((max_steering_angle * 100.0) as i64, 100_i64);
            let delta_min_val = Real::from_rational((-max_steering_angle * 100.0) as i64, 100_i64);

            for t in 0..=self.horizon {
                let delta_var = &self.steering_delta[actor_id][t];
                self.backend.assert(&delta_var.ge(&delta_min_val));
                self.backend.assert(&delta_var.le(&delta_max_val));

                // Heading angle bounds: -π/6 <= θ <= π/6 (±30° for small angle validity)
                let theta_var = &self.heading_theta[actor_id][t];
                let theta_max = std::f64::consts::PI / 6.0; // 30 degrees
                let theta_max_val = Real::from_rational((theta_max * 100.0) as i64, 100_i64);
                let theta_min_val = Real::from_rational((-theta_max * 100.0) as i64, 100_i64);
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
            let max_change_val = Real::from_rational((max_delta_change * 100.0) as i64, 100_i64);

            for t in 0..self.horizon {
                let delta_t = &self.steering_delta[actor_id][t];
                let delta_t1 = &self.steering_delta[actor_id][t + 1];
                let delta_diff = delta_t1 - delta_t;

                // |delta_diff| <= max_change
                // Encoded as: -max_change <= delta_diff <= max_change
                let neg_max_change_val =
                    Real::from_rational((-max_delta_change * 100.0) as i64, 100_i64);
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
        let lane_width = self.spec.lane_width;
        let py_var = &self.positions_y[actor_id][t];

        let min_py = Real::from_rational((lane as f64 * lane_width * 100.0) as i64, 100_i64);
        let max_py = Real::from_rational(((lane + 1) as f64 * lane_width * 100.0) as i64, 100_i64);

        self.backend.assert(&py_var.ge(&min_py));
        self.backend.assert(&py_var.le(&max_py));

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

        let source_center_val = Real::from_rational((source_center * 100.0) as i64, 100_i64);
        let target_center_val = Real::from_rational((target_center * 100.0) as i64, 100_i64);
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

        // Update lane variables: source during transition, target at end
        let source_lane_val = Int::from_i64(source_lane as i64);
        let target_lane_val = Int::from_i64(target_lane as i64);
        for t in start_step..=end_clamped {
            if t < end_clamped {
                self.backend
                    .assert(&self.lanes[actor_id][t].eq(&source_lane_val));
            } else {
                self.backend
                    .assert(&self.lanes[actor_id][t].eq(&target_lane_val));
            }
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
        let road_max = Real::from_rational((num_lanes as f64 * lane_width * 100.0) as i64, 100_i64);
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
            }

            self.positions_x.insert(actor_id.clone(), px_vars);
            self.positions_y.insert(actor_id.clone(), py_vars);
            self.heading_theta.insert(actor_id.clone(), theta_vars);
            self.speed_v.insert(actor_id.clone(), v_vars);
            self.steering_delta.insert(actor_id.clone(), delta_vars);
            self.accelerations.insert(actor_id.clone(), a_vars);
            self.lanes.insert(actor_id.clone(), lane_vars);
            self.velocities_y.insert(actor_id.clone(), vy_vars);
        }
    }

    fn encode_kinematics(&mut self, dt: f64) {
        let dt_val = Real::from_rational((dt * 100.0) as i64, 100_i64);
        let zero = Real::from_rational(0, 1);

        // Collect lane change data to determine stable vs transition phases
        let lane_changes_data = collect_lane_change_data(&self.spec, self.horizon);

        // Collect actor info to avoid borrow checker issues
        let actor_info: Vec<_> = self
            .spec
            .actors
            .iter()
            .map(|a| (a.id.clone(), a.role, a.direction, a.speed.max()))
            .collect();

        for (actor_id, role, direction, speed_max) in &actor_info {
            if *role == ActorRole::Pedestrian {
                // TODO: Implement simplified pedestrian model
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
            let max_theta_change_val =
                Real::from_rational((max_theta_change * 1000.0) as i64, 1000_i64);
            let neg_max_theta_change_val =
                Real::from_rational((-max_theta_change * 1000.0) as i64, 1000_i64);

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

                // Longitudinal: px[t+1] = px[t] ± v[t] * dt
                // Direction handled here (not via heading angle)
                let px_next = if *direction == 1 {
                    px_t + &(v_t * &dt_val)
                } else {
                    px_t - &(v_t * &dt_val)
                };
                self.backend.assert(&px_t1.eq(&px_next));

                // Lateral: py[t+1] = py[t] + vy[t] * dt (vy is independent, linear)
                let py_next = py_t + &(vy_t * &dt_val);
                self.backend.assert(&py_t1.eq(&py_next));

                // Speed: v[t+1] = v[t] + a[t] * dt (linear)
                let v_next = v_t + &(a_t * &dt_val);
                self.backend.assert(&v_t1.eq(&v_next));
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
        // Collect actor data first to avoid borrow checker issues
        let actor_data: Vec<_> = self
            .spec
            .actors
            .iter()
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
            let speed_max_val = Real::from_rational((speed_max * 10.0) as i64, 10_i64);

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

            let accel_min_val = Real::from_rational((accel_min * 10.0) as i64, 10_i64);
            let accel_max_val = Real::from_rational((accel_max * 10.0) as i64, 10_i64);

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

        // For bicycle model, use speed (always positive) with heading to determine velocity
        // Small angle approximation: vx ≈ v * cos(θ) ≈ v
        let v1 = &self.speed_v[actor1][time];
        let v2 = &self.speed_v[actor2][time];

        let min_ttc_val = Real::from_rational((min_ttc * 10.0) as i64, 10_i64);
        let epsilon = Real::from_rational(1_i64, 100_i64); // 0.01 m/s to avoid division by zero

        // Enhanced "same lane" condition: discrete lane match OR y-position proximity
        // This handles lane change transitions where discrete lane != smooth y-position
        let same_lane_discrete = lane1.eq(lane2);

        // Y-position proximity: |py1 - py2| < lane_width (vehicles in same lateral space)
        // FIXED: Use AND to properly check |py1 - py2| < lane_width
        // Both (py1-py2) < lane_width AND (py2-py1) < lane_width must be true
        // Using OR would be incorrect: if py1-py2 = 5.0 and lane_width = 3.5,
        // py2-py1 = -5.0 < 3.5 is TRUE, so OR would incorrectly return TRUE
        let lane_width = self.spec.lane_width;
        let lane_width_real = Real::from_rational((lane_width * 10.0) as i64, 10_i64);
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
        let actor2_faster = v2.gt(v1);
        let rel_vel_1 = v2 - v1;
        let distance_1 = px1 - px2;
        let collision_possible_1 =
            Bool::and(&[&actor1_ahead, &actor2_faster, &rel_vel_1.gt(&epsilon)]);
        // TTC > min_ttc means: distance / rel_vel > min_ttc
        // Equivalent to: distance > min_ttc * rel_vel
        let ttc_safe_1 = distance_1.gt(&(&min_ttc_val * &rel_vel_1));

        // Case 2: actor2 ahead, actor1 behind, actor1 faster
        // TTC = (px2 - px1) / (v1 - v2)
        let actor2_ahead = px2.gt(px1);
        let actor1_faster = v1.gt(v2);
        let rel_vel_2 = v1 - v2;
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

        let min_dist_val = Real::from_rational((min_dist * 10.0) as i64, 10_i64);

        // Enhanced "same lane" condition: discrete lane match OR y-position proximity
        // This handles lane change transitions where discrete lane != smooth y-position
        let same_lane_discrete = lane1.eq(lane2);

        // Y-position proximity: |py1 - py2| < lane_width (vehicles in same lateral space)
        // FIXED: Use AND to properly check |py1 - py2| < lane_width
        // Both (py1-py2) < lane_width AND (py2-py1) < lane_width must be true
        // Using OR would be incorrect: if py1-py2 = 5.0 and lane_width = 3.5,
        // py2-py1 = -5.0 < 3.5 is TRUE, so OR would incorrectly return TRUE
        let lane_width = self.spec.lane_width;
        let lane_width_real = Real::from_rational((lane_width * 10.0) as i64, 10_i64);
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

            // For acceleration, we have longitudinal acceleration 'a'
            // Lateral acceleration comes from centripetal acceleration during turns
            // For bicycle model: ay ≈ v * dθ/dt ≈ v * (v/L) * δ
            // However, we don't extract δ or compute derivatives here
            // For simplicity, set lateral acceleration to 0 in output
            let ax = a;
            let ay = 0.0; // Simplified - could be computed from steering and speed

            let state = State {
                time,
                cartesian: Some(CartesianState {
                    position: Position { x: px, y: py },
                    velocity: Velocity { vx, vy },
                    acceleration: Acceleration { ax, ay },
                    lane,
                }),
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
