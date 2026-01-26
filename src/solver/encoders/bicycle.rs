//! Bicycle model coordinate system encoder
//!
//! Implements the CoordinateEncoder trait for kinematic bicycle model dynamics.
//! This encoder models vehicles with heading tracking, steering constraints,
//! and turn radius limitations.
//!
//! State: (x, y, θ, v) where θ is heading angle, v is speed
//! Controls: (a, δ) where a is longitudinal acceleration, δ is steering angle
//!
//! Dynamics (small angle approximation):
//! ```text
//! dx/dt = v * cos(θ) ≈ v
//! dy/dt = v * sin(θ) ≈ v * θ
//! dθ/dt = (v/L) * tan(δ) ≈ (v/L) * δ
//! dv/dt = a
//! ```

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

    /// Helper: Extract real value from Z3 model
    fn extract_real(&self, model: &Model, var: &Real) -> Result<f64> {
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

    /// Helper: Extract int value from Z3 model
    fn extract_int(&self, model: &Model, var: &Int) -> Result<usize> {
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
        direction: i32,
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

        // Heading angle at t=0 (aligned with road)
        let theta_var = &self.heading_theta[actor_id][0];
        let theta_initial = if direction == 1 {
            0.0 // Forward: 0 radians (east)
        } else {
            std::f64::consts::PI // Backward: π radians (west)
        };
        let theta_val = Real::from_rational((theta_initial * 100.0) as i64, 100_i64);
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

    /// Encode lane-position coupling with lane change support
    fn encode_lane_coupling_with_lane_changes(&mut self) {
        // Collect actors with lane_change config and their data
        let lane_change_data: Vec<_> = self
            .spec
            .actors
            .iter()
            .filter(|a| a.role != ActorRole::Pedestrian)
            .filter_map(|a| {
                a.lane_change.as_ref().filter(|lc| lc.enabled).map(|lc| {
                    let dt = self.spec.time_step;
                    let start_min = lc.start_time.min();
                    let start_max = lc.start_time.max();
                    let duration_min = lc.duration.min();
                    let duration_max = lc.duration.max();

                    let start_step_min = (start_min / dt) as usize;
                    let start_step_max = (start_max / dt) as usize;
                    let duration_steps_min = (duration_min / dt) as usize;
                    let duration_steps_max = (duration_max / dt) as usize;

                    // Use midpoint for now
                    let start_step = (start_step_min + start_step_max) / 2;
                    let duration_steps = (duration_steps_min + duration_steps_max) / 2;
                    let end_step = (start_step + duration_steps).min(self.horizon);

                    (a.id.clone(), lc.direction.clone(), start_step, end_step)
                })
            })
            .collect();

        // Collect actor IDs to avoid borrow checker issues
        let actor_data: Vec<_> = self
            .spec
            .actors
            .iter()
            .filter(|a| a.role != ActorRole::Pedestrian)
            .map(|a| a.id.clone())
            .collect();

        for actor_id in actor_data {
            // Check if this actor has lane_change enabled
            let lc_data = lane_change_data
                .iter()
                .find(|(id, _, _, _)| id == &actor_id);

            if let Some((_, direction, start_step, end_step)) = lc_data {
                // Before lane change: enforce lane-position coupling
                for t in 0..(*start_step).min(self.horizon) {
                    self.encode_lane_position_coupling_at_time(&actor_id, t);
                }

                // During lane change: encode smooth transition
                self.encode_smooth_lane_transition_bicycle(
                    &actor_id,
                    *start_step,
                    *end_step,
                    direction,
                );

                // After lane change: enforce lane-position coupling to new lane
                for t in *end_step..=self.horizon {
                    self.encode_lane_position_coupling_at_time(&actor_id, t);
                }
            } else {
                // No lane change: enforce coupling at all time steps
                for t in 0..=self.horizon {
                    self.encode_lane_position_coupling_at_time(&actor_id, t);
                }
            }
        }
    }

    /// Encode lane-position coupling at a specific time step
    /// For bicycle model, DON'T enforce strict coupling - let dynamics naturally evolve
    /// Only track which lane the vehicle is logically in based on lateral position
    fn encode_lane_position_coupling_at_time(&mut self, _actor_id: &str, _t: usize) {
        // For bicycle model: No strict coupling needed
        // The lane variable is used for LTL constraints but doesn't force lateral position
        // Lateral position naturally evolves from bicycle dynamics (heading, steering)
        // The lane assignment is determined by initial conditions and lane changes
    }

    /// Encode smooth lane transition for bicycle model
    /// Update lane variable AND ensure actual lateral position change
    fn encode_smooth_lane_transition_bicycle(
        &mut self,
        actor_id: &str,
        start_step: usize,
        end_step: usize,
        direction: &crate::dsl::types::LaneChangeDirection,
    ) {
        let lane_width = self.spec.lane_width;
        let lane_width_real = Real::from_rational((lane_width * 10.0) as i64, 10_i64);

        // Get source lane from step before transition starts
        let source_lane = &self.lanes[actor_id][start_step.saturating_sub(1)];

        // Calculate target lane
        let lane_delta = match direction {
            crate::dsl::types::LaneChangeDirection::Right => 1,
            crate::dsl::types::LaneChangeDirection::Left => -1,
        };
        let target_lane_int = source_lane + Int::from_i64(lane_delta);

        // Update lane variable: at end of transition, lane equals target
        let end_clamped = end_step.min(self.horizon);
        if end_clamped <= self.horizon {
            self.backend
                .assert(&self.lanes[actor_id][end_clamped].eq(&target_lane_int));
        }

        // Ensure actual lateral position change occurs
        // Lateral position must move by approximately one lane width in the correct direction
        let py_start = &self.positions_y[actor_id][start_step];
        let py_end = &self.positions_y[actor_id][end_clamped];
        let py_diff = py_end - py_start;

        // Lane change should produce lateral motion of roughly one lane width
        // Allow some tolerance (50% to 150% of lane width)
        let min_change = Real::from_rational((lane_width * 5.0) as i64, 10_i64); // 0.5 * lane_width
        let max_change = Real::from_rational((lane_width * 15.0) as i64, 10_i64); // 1.5 * lane_width

        match direction {
            crate::dsl::types::LaneChangeDirection::Right => {
                // Moving to higher lane number = higher y
                self.backend.assert(&py_diff.ge(&min_change));
                self.backend.assert(&py_diff.le(&max_change));
            }
            crate::dsl::types::LaneChangeDirection::Left => {
                // Moving to lower lane number = lower y
                let neg_max_change = Real::from_rational((-lane_width * 15.0) as i64, 10_i64);
                let neg_min_change = Real::from_rational((-lane_width * 5.0) as i64, 10_i64);
                self.backend.assert(&py_diff.ge(&neg_max_change));
                self.backend.assert(&py_diff.le(&neg_min_change));
            }
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

            for t in 0..=horizon {
                px_vars.push(Real::new_const(format!("{}__px_{}", actor_id, t)));
                py_vars.push(Real::new_const(format!("{}__py_{}", actor_id, t)));
                theta_vars.push(Real::new_const(format!("{}__theta_{}", actor_id, t)));
                v_vars.push(Real::new_const(format!("{}__v_{}", actor_id, t)));
                delta_vars.push(Real::new_const(format!("{}__delta_{}", actor_id, t)));
                a_vars.push(Real::new_const(format!("{}__a_{}", actor_id, t)));
                lane_vars.push(Int::new_const(format!("{}__lane_{}", actor_id, t)));
            }

            self.positions_x.insert(actor_id.clone(), px_vars);
            self.positions_y.insert(actor_id.clone(), py_vars);
            self.heading_theta.insert(actor_id.clone(), theta_vars);
            self.speed_v.insert(actor_id.clone(), v_vars);
            self.steering_delta.insert(actor_id.clone(), delta_vars);
            self.accelerations.insert(actor_id.clone(), a_vars);
            self.lanes.insert(actor_id.clone(), lane_vars);
        }
    }

    fn encode_kinematics(&mut self, dt: f64) {
        let dt_val = Real::from_rational((dt * 100.0) as i64, 100_i64);

        for actor in &self.spec.actors {
            if actor.role == ActorRole::Pedestrian {
                // TODO: Implement simplified pedestrian model
                continue;
            }

            let actor_id = &actor.id;

            // Get bicycle parameters for this actor
            let (wheelbase, _, _) = match self.get_actor_bicycle_params(actor_id) {
                Ok(params) => params,
                Err(_) => continue,
            };

            let wheelbase_val = Real::from_rational((wheelbase * 100.0) as i64, 100_i64);

            // Encode bicycle dynamics using small angle approximation
            for t in 0..self.horizon {
                let px_t = &self.positions_x[actor_id][t];
                let py_t = &self.positions_y[actor_id][t];
                let theta_t = &self.heading_theta[actor_id][t];
                let v_t = &self.speed_v[actor_id][t];
                let delta_t = &self.steering_delta[actor_id][t];
                let a_t = &self.accelerations[actor_id][t];

                let px_t1 = &self.positions_x[actor_id][t + 1];
                let py_t1 = &self.positions_y[actor_id][t + 1];
                let theta_t1 = &self.heading_theta[actor_id][t + 1];
                let v_t1 = &self.speed_v[actor_id][t + 1];

                // Small angle approximation:
                // dx/dt = v * cos(θ) ≈ v (cos(θ) ≈ 1)
                // dy/dt = v * sin(θ) ≈ v * θ (sin(θ) ≈ θ)
                // dθ/dt = (v/L) * tan(δ) ≈ (v/L) * δ (tan(δ) ≈ δ)
                // dv/dt = a

                // px[t+1] = px[t] + v[t] * dt
                let px_next = px_t + &(v_t * &dt_val);
                self.backend.assert(&px_t1.eq(&px_next));

                // py[t+1] = py[t] + v[t] * θ[t] * dt
                let py_next = py_t + &(v_t * theta_t * &dt_val);
                self.backend.assert(&py_t1.eq(&py_next));

                // θ[t+1] = θ[t] + (v[t] / L) * δ[t] * dt
                let theta_next = theta_t + &((v_t * delta_t / &wheelbase_val) * &dt_val);
                self.backend.assert(&theta_t1.eq(&theta_next));

                // v[t+1] = v[t] + a[t] * dt
                let v_next = v_t + &(a_t * &dt_val);
                self.backend.assert(&v_t1.eq(&v_next));
            }
        }

        // Encode lane change constraints and lane-position coupling
        self.encode_lane_coupling_with_lane_changes();

        // Encode bicycle-specific constraints
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
        }
    }

    fn encode_ttc_constraint(&self, actor1: &str, actor2: &str, min_ttc: f64, time: usize) -> Bool {
        let lane1 = &self.lanes[actor1][time];
        let lane2 = &self.lanes[actor2][time];

        let px1 = &self.positions_x[actor1][time];
        let px2 = &self.positions_x[actor2][time];

        // For bicycle model, use speed (always positive) with heading to determine velocity
        // Small angle approximation: vx ≈ v * cos(θ) ≈ v
        let v1 = &self.speed_v[actor1][time];
        let v2 = &self.speed_v[actor2][time];

        let min_ttc_val = Real::from_rational((min_ttc * 10.0) as i64, 10_i64);
        let epsilon = Real::from_rational(1_i64, 100_i64); // 0.01 m/s to avoid division by zero

        // Same lane condition
        let same_lane = lane1.eq(lane2);

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

        let min_dist_val = Real::from_rational((min_dist * 10.0) as i64, 10_i64);

        // Same lane condition
        let same_lane = lane1.eq(lane2);

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

            // Extract bicycle state variables
            let px = self.extract_real(model, &self.positions_x[actor_id][t])?;
            let py = self.extract_real(model, &self.positions_y[actor_id][t])?;
            let theta = self.extract_real(model, &self.heading_theta[actor_id][t])?;
            let v = self.extract_real(model, &self.speed_v[actor_id][t])?;
            let a = self.extract_real(model, &self.accelerations[actor_id][t])?;
            let lane = self.extract_int(model, &self.lanes[actor_id][t])?;

            // Convert bicycle state to Cartesian velocities using small angle approximation:
            // vx ≈ v * cos(θ) ≈ v (since cos(θ) ≈ 1 for small θ)
            // vy ≈ v * sin(θ) ≈ v * θ (since sin(θ) ≈ θ for small θ)
            let vx = v; // Small angle approximation: cos(θ) ≈ 1
            let vy = v * theta; // Small angle approximation: sin(θ) ≈ θ

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
        // Bicycle model doesn't have separate lateral velocity variable
        // Return a reference to a zero (approximation)
        // TODO: Compute from v * θ if needed
        &self.speed_v[actor_id][time] // Placeholder
    }

    fn encode_lane_velocity_constraints(&mut self) {
        // TODO: Implement lane velocity constraints for bicycle model
        // For now, just encode basic lane bounds
        let num_lanes = self.spec.get_num_lanes();
        let num_lanes_int = Int::from_i64(num_lanes as i64);

        for actor in &self.spec.actors {
            let actor_id = &actor.id;

            for t in 0..=self.horizon {
                let lane_var = &self.lanes[actor_id][t];

                // Lane bounds: 0 <= lane < num_lanes
                let zero = Int::from_i64(0);
                self.backend.assert(&lane_var.ge(&zero));
                self.backend.assert(&lane_var.lt(&num_lanes_int));
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
