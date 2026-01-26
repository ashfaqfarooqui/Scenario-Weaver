//! Cartesian coordinate system encoder
//!
//! Implements the CoordinateEncoder trait for Cartesian (x, y) coordinates.
//! This encoder handles vehicle kinematics in 2D Cartesian space with
//! lane-based constraints.

use std::collections::HashMap;
use std::ops::Add;
use z3::ast::{Bool, Int, Real};
use z3::Model;

use crate::dsl::types::{ActorRole, ScenarioSpec};
use crate::dsl::types::{
    PEDESTRIAN_MAX_ACCELERATION, PEDESTRIAN_MAX_DECELERATION, PEDESTRIAN_RUN_MAX_SPEED,
    PEDESTRIAN_WALK_MAX_SPEED,
};
use crate::error::{Result, ScenarioGenError};
use crate::scenario::model::{
    Acceleration, ActorTrajectory, CartesianState, Position, State, Velocity,
};
use crate::solver::backend::Z3Backend;
use crate::solver::coordinate_encoder::CoordinateEncoder;

/// Cartesian coordinate system encoder
///
/// Uses (x, y) position variables and manages lane-based constraints
/// for vehicle motion.
pub struct CartesianEncoder<B: Z3Backend> {
    /// Z3 backend (Solver or Optimizer)
    backend: B,

    /// Scenario specification
    spec: ScenarioSpec,

    /// Number of time steps
    horizon: usize,

    // Variable maps: actor_id -> Vec<variable> (one per time step)
    /// Longitudinal positions (m)
    positions_x: HashMap<String, Vec<Real>>,

    /// Lateral positions (m)
    positions_y: HashMap<String, Vec<Real>>,

    /// Longitudinal velocities (m/s)
    velocities_x: HashMap<String, Vec<Real>>,

    /// Lateral velocities (m/s)
    velocities_y: HashMap<String, Vec<Real>>,

    /// Lane numbers (integer)
    lanes: HashMap<String, Vec<Int>>,

    /// Longitudinal accelerations (m/s²)
    accelerations_x: HashMap<String, Vec<Real>>,

    /// Lateral accelerations (m/s²)
    accelerations_y: HashMap<String, Vec<Real>>,
}

impl<B: Z3Backend> CartesianEncoder<B> {
    /// Create a new Cartesian encoder
    pub fn new(spec: ScenarioSpec, backend: B) -> Self {
        let horizon = spec.num_time_steps();

        Self {
            backend,
            spec,
            horizon,
            positions_x: HashMap::new(),
            positions_y: HashMap::new(),
            velocities_x: HashMap::new(),
            velocities_y: HashMap::new(),
            lanes: HashMap::new(),
            accelerations_x: HashMap::new(),
            accelerations_y: HashMap::new(),
        }
    }

    /// Encode lane-position coupling at a specific time step
    ///
    /// Constrains py = lane * lane_width + lane_width/2
    fn encode_lane_position_coupling_at_time(&mut self, actor_id: &str, t: usize) {
        let lane_var = &self.lanes[actor_id][t];
        let py_var = &self.positions_y[actor_id][t];

        let lane_width = self.spec.lane_width;
        let lane_width_real = Real::from_rational((lane_width * 10.0) as i64, 10_i64);
        let half_width = Real::from_rational((lane_width * 5.0) as i64, 10_i64);

        // py = lane * lane_width + lane_width/2
        let lane_real = lane_var.to_real();
        let expected_py = lane_real * &lane_width_real + &half_width;
        self.backend.assert(&py_var.eq(&expected_py));
    }

    /// Encode initial state for a single actor (Cartesian-specific)
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
        role: ActorRole,
        direction: i32,
    ) {
        // Lane at t=0
        let lane_var = &self.lanes[actor_id][0];
        let lane_val = Int::from_i64(lane as i64);
        self.backend.assert(&lane_var.eq(&lane_val));

        // Position at t=0
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

        // Velocity at t=0
        // Use actor direction: speed is magnitude, vx sign depends on direction
        let vx_var = &self.velocities_x[actor_id][0];

        if (speed_min - speed_max).abs() < 1e-6 {
            // Fixed value
            let speed = if direction == 1 {
                speed_min
            } else {
                -speed_min
            };
            let speed_val = Real::from_rational((speed * 10.0) as i64, 10_i64);
            self.backend.assert(&vx_var.eq(&speed_val));
        } else {
            // Range
            if direction == 1 {
                // Forward: vx in [speed_min, speed_max]
                let min_val = Real::from_rational((speed_min * 10.0) as i64, 10_i64);
                let max_val = Real::from_rational((speed_max * 10.0) as i64, 10_i64);
                self.backend.assert(&vx_var.ge(&min_val));
                self.backend.assert(&vx_var.le(&max_val));
            } else {
                // Backward: vx in [-speed_max, -speed_min]
                let min_val = Real::from_rational((-speed_max * 10.0) as i64, 10_i64);
                let max_val = Real::from_rational((-speed_min * 10.0) as i64, 10_i64);
                self.backend.assert(&vx_var.ge(&min_val));
                self.backend.assert(&vx_var.le(&max_val));
            }
        }

        // Initial lateral velocity
        // For vehicles: zero (not changing lanes initially)
        // For pedestrians: unconstrained (they need to cross laterally)
        let vy_var = &self.velocities_y[actor_id][0];
        let zero = Real::from_rational(0_i64, 1_i64);
        if role != ActorRole::Pedestrian {
            self.backend.assert(&vy_var.eq(&zero));
        }

        // Initial acceleration at t=0
        let ax_var = &self.accelerations_x[actor_id][0];
        if (accel_min - accel_max).abs() < 1e-6 {
            // Fixed acceleration
            let accel_val = Real::from_rational((accel_min * 10.0) as i64, 10_i64);
            self.backend.assert(&ax_var.eq(&accel_val));
        } else {
            // Acceleration range
            let min_val = Real::from_rational((accel_min * 10.0) as i64, 10_i64);
            let max_val = Real::from_rational((accel_max * 10.0) as i64, 10_i64);
            self.backend.assert(&ax_var.ge(&min_val));
            self.backend.assert(&ax_var.le(&max_val));
        }

        // Initial lateral acceleration
        // For vehicles: zero (not changing lanes initially)
        // For pedestrians: unconstrained (they need to accelerate laterally to cross)
        let ay_var = &self.accelerations_y[actor_id][0];
        if role != ActorRole::Pedestrian {
            self.backend.assert(&ay_var.eq(&zero));
        }

        // Encode initial lane-position coupling
        self.encode_lane_position_coupling_at_time(actor_id, 0);
    }

    /// Extract a real value from Z3 model
    fn extract_real(&self, model: &Model, var: &Real) -> Result<f64> {
        let ast = model.eval(var, true).ok_or_else(|| {
            ScenarioGenError::Z3ModelParsing("Failed to evaluate real variable".to_string())
        })?;

        if let Some(rational) = ast.as_rational() {
            let (num, denom) = rational;
            Ok(num as f64 / denom as f64)
        } else {
            Err(ScenarioGenError::Z3ModelParsing(format!(
                "Expected rational value, got: {}",
                ast
            )))
        }
    }

    /// Extract an integer value from Z3 model
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

    /// Encode smooth lane transition over multiple time steps
    ///
    /// Constrains lateral position to gradually transition from source lane center
    /// to target lane center over the specified time window.
    fn encode_smooth_lane_transition(
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
        let target_lane_int = source_lane.add(&Int::from_i64(lane_delta));

        // Get source and target lane centers
        let half_width = Real::from_rational((lane_width * 5.0) as i64, 10_i64);
        let source_center = source_lane.to_real() * &lane_width_real + &half_width;
        let target_center = target_lane_int.to_real() * &lane_width_real + &half_width;

        // Number of transition steps
        let num_steps = end_step - start_step;
        if num_steps == 0 {
            return;
        }

        // Soft constraints: constrain lateral position near source at start and target at end
        // Allow Z3 to discover smooth curve trajectory instead of forcing linear interpolation
        let tolerance = Real::from_rational(5_i64, 10_i64); // 0.5m tolerance

        // Constrain lateral position near source at start (soft)
        let py_start = &self.positions_y[actor_id][start_step];
        self.backend
            .assert(&py_start.ge(&(&source_center - &tolerance)));
        self.backend
            .assert(&py_start.le(&(&source_center + &tolerance)));

        // Constrain lateral position near target at end (soft)
        let py_end = &self.positions_y[actor_id][end_step.min(self.horizon)];
        self.backend
            .assert(&py_end.ge(&(&target_center - &tolerance)));
        self.backend
            .assert(&py_end.le(&(&target_center + &tolerance)));

        // Constrain lateral velocity for smooth motion during transition
        let max_vy = Real::from_rational(20_i64, 10_i64); // 2.0 m/s
        for t in start_step..=end_step.min(self.horizon) {
            let vy_t = &self.velocities_y[actor_id][t];
            self.backend.assert(&vy_t.ge(&-&max_vy));
            self.backend.assert(&vy_t.le(&max_vy));
        }

        // Update lane variable during transition
        for t in start_step..=end_step.min(self.horizon) {
            if t < end_step {
                // Keep lane variable at source during transition
                self.backend
                    .assert(&self.lanes[actor_id][t].eq(source_lane));
            } else {
                // At end of transition, lane equals target
                self.backend
                    .assert(&self.lanes[actor_id][t].eq(&target_lane_int));
            }
        }

        // Encode lateral acceleration bounds for smoothness
        self.encode_lateral_acceleration_bounds(actor_id, start_step, end_step);
    }

    /// Encode lateral acceleration bounds during lane changes
    fn encode_lateral_acceleration_bounds(
        &mut self,
        actor_id: &str,
        start_step: usize,
        end_step: usize,
    ) {
        // Use configurable max lateral acceleration from spec
        let max_ay_value = self.spec.max_lateral_acceleration;
        let max_ay = Real::from_rational((max_ay_value * 10.0) as i64, 10_i64);

        for t in start_step..=end_step.min(self.horizon) {
            let ay_t = &self.accelerations_y[actor_id][t];
            self.backend.assert(&ay_t.le(&max_ay));
            self.backend.assert(&ay_t.ge(&-&max_ay));
        }
    }
}

impl<B: Z3Backend> CoordinateEncoder<B> for CartesianEncoder<B> {
    fn create_variables(&mut self, horizon: usize, spec: &ScenarioSpec) {
        for actor in &spec.actors {
            let actor_id = &actor.id;

            let mut px_vars = Vec::new();
            let mut py_vars = Vec::new();
            let mut vx_vars = Vec::new();
            let mut vy_vars = Vec::new();
            let mut lane_vars = Vec::new();
            let mut ax_vars = Vec::new();
            let mut ay_vars = Vec::new();

            // Create variables for each time step
            for t in 0..=horizon {
                px_vars.push(Real::new_const(format!("{}_px_{}", actor_id, t)));
                py_vars.push(Real::new_const(format!("{}_py_{}", actor_id, t)));
                vx_vars.push(Real::new_const(format!("{}_vx_{}", actor_id, t)));
                vy_vars.push(Real::new_const(format!("{}_vy_{}", actor_id, t)));
                lane_vars.push(Int::new_const(format!("{}_lane_{}", actor_id, t)));
                ax_vars.push(Real::new_const(format!("{}_ax_{}", actor_id, t)));
                ay_vars.push(Real::new_const(format!("{}_ay_{}", actor_id, t)));
            }

            self.positions_x.insert(actor_id.clone(), px_vars);
            self.positions_y.insert(actor_id.clone(), py_vars);
            self.velocities_x.insert(actor_id.clone(), vx_vars);
            self.velocities_y.insert(actor_id.clone(), vy_vars);
            self.lanes.insert(actor_id.clone(), lane_vars);
            self.accelerations_x.insert(actor_id.clone(), ax_vars);
            self.accelerations_y.insert(actor_id.clone(), ay_vars);
        }
    }

    fn encode_kinematics(&mut self, dt: f64) {
        let dt_real = Real::from_rational((dt * 10.0) as i64, 10_i64);
        let zero = Real::from_rational(0_i64, 1_i64);

        for actor in &self.spec.actors {
            let actor_id = &actor.id;

            // Get acceleration bounds from actor spec
            // For pedestrians, clamp to pedestrian-specific physics limits
            let (ax_min, ax_max) = if actor.role == ActorRole::Pedestrian {
                let spec_min = actor.acceleration.min();
                let spec_max = actor.acceleration.max();
                (
                    spec_min.max(PEDESTRIAN_MAX_DECELERATION),
                    spec_max.min(PEDESTRIAN_MAX_ACCELERATION),
                )
            } else {
                (actor.acceleration.min(), actor.acceleration.max())
            };
            let ax_min_real = Real::from_rational((ax_min * 10.0) as i64, 10_i64);
            let ax_max_real = Real::from_rational((ax_max * 10.0) as i64, 10_i64);

            for t in 0..self.horizon {
                // ========== LONGITUDINAL DYNAMICS ==========

                // Acceleration bounds at each timestep
                let ax_t = &self.accelerations_x[actor_id][t];
                self.backend.assert(&ax_t.ge(&ax_min_real));
                self.backend.assert(&ax_t.le(&ax_max_real));

                // Velocity update: vx[t+1] = vx[t] + ax[t] * dt
                let vx_t = &self.velocities_x[actor_id][t];
                let vx_t1 = &self.velocities_x[actor_id][t + 1];
                let expected_vx = vx_t + &(ax_t * &dt_real);
                self.backend.assert(&vx_t1.eq(&expected_vx));

                // Position update: px[t+1] = px[t] + vx[t] * dt
                let px_t = &self.positions_x[actor_id][t];
                let px_t1 = &self.positions_x[actor_id][t + 1];
                let expected_px = px_t + &(vx_t * &dt_real);
                self.backend.assert(&px_t1.eq(&expected_px));

                // ========== LATERAL DYNAMICS ==========

                // Lateral acceleration bounds (for pedestrians)
                if actor.role == ActorRole::Pedestrian {
                    let ay_t = &self.accelerations_y[actor_id][t];
                    self.backend.assert(&ay_t.ge(&ax_min_real));
                    self.backend.assert(&ay_t.le(&ax_max_real));

                    // Lateral velocity update for pedestrians: vy[t+1] = vy[t] + ay[t] * dt
                    let vy_t = &self.velocities_y[actor_id][t];
                    let vy_t1 = &self.velocities_y[actor_id][t + 1];
                    let expected_vy = vy_t + &(ay_t * &dt_real);
                    self.backend.assert(&vy_t1.eq(&expected_vy));
                }

                // Lateral position update: py[t+1] = py[t] + vy[t] * dt
                let py_t = &self.positions_y[actor_id][t];
                let py_t1 = &self.positions_y[actor_id][t + 1];
                let vy_t = &self.velocities_y[actor_id][t];
                let expected_py = py_t + &(vy_t * &dt_real);
                self.backend.assert(&py_t1.eq(&expected_py));

                // Ego never changes lanes (vy = 0)
                if actor.role == ActorRole::Ego {
                    self.backend.assert(&vy_t.eq(&zero));
                }

                // Pedestrian speed magnitude constraints
                //
                // LINEARIZED VERSION (Phase 2): Replaced quadratic disk constraint
                // (vx^2 + vy^2 <= max^2) with linear box constraint (|vx| <= max AND |vy| <= max)
                //
                // Trade-off: Box is over-conservative (contains disk), so diagonal speeds up
                // to sqrt(2) * max are allowed. Compensated by reducing max speeds by sqrt(2)
                // in constants to maintain semantic correctness.
                //
                // Performance: Eliminates QF_NRA (nonlinear) solver requirement, keeps Z3 in
                // QF_LRA (linear) theory for 10-20x speedup. Multi-solve now works reliably.
                if actor.role == ActorRole::Pedestrian {
                    let max_speed = actor
                        .behavior
                        .get("walking_mode")
                        .map(|mode| match mode.as_str() {
                            Some("run") => PEDESTRIAN_RUN_MAX_SPEED,
                            _ => PEDESTRIAN_WALK_MAX_SPEED,
                        })
                        .unwrap_or(PEDESTRIAN_WALK_MAX_SPEED);

                    let max_speed_real = Real::from_rational((max_speed * 10.0) as i64, 10_i64);
                    let neg_max_speed = -max_speed;
                    let neg_max_speed_real =
                        Real::from_rational((neg_max_speed * 10.0) as i64, 10_i64);

                    // Linear box constraint: |vx| <= max_speed AND |vy| <= max_speed
                    // vx >= -max_speed AND vx <= max_speed
                    self.backend.assert(&vx_t.ge(&neg_max_speed_real));
                    self.backend.assert(&vx_t.le(&max_speed_real));

                    // vy >= -max_speed AND vy <= max_speed
                    self.backend.assert(&vy_t.ge(&neg_max_speed_real));
                    self.backend.assert(&vy_t.le(&max_speed_real));
                }
            }
        }

        // Lane-position coupling with smooth lane change support
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

                    // Use midpoint for now (TODO: make solver variables)
                    let start_step = (start_step_min + start_step_max) / 2;
                    let duration_steps = (duration_steps_min + duration_steps_max) / 2;
                    let end_step = (start_step + duration_steps).min(self.horizon);

                    (a.id.clone(), lc.direction.clone(), start_step, end_step)
                })
            })
            .collect();

        // Collect actor IDs and roles to avoid borrow checker issues
        let actor_data: Vec<_> = self
            .spec
            .actors
            .iter()
            .filter(|a| a.role != ActorRole::Pedestrian)
            .map(|a| (a.id.clone(), a.role))
            .collect();

        for (actor_id, _role) in actor_data {
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
                self.encode_smooth_lane_transition(&actor_id, *start_step, *end_step, direction);

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

    fn encode_initial_conditions(&mut self) {
        // Collect all actor data upfront to avoid borrow checker issues
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
            acc_min,
            acc_max,
            role,
            direction,
        ) in actor_data
        {
            self.encode_actor_initial_state(
                &actor_id, lane, pos_min, pos_max, speed_min, speed_max, acc_min, acc_max, role,
                direction,
            );
        }
    }

    fn encode_velocity_constraints(&mut self) {
        // In Cartesian system, velocity direction constraints are based on actor direction
        let zero = Real::from_rational(0_i64, 1_i64);

        for actor in &self.spec.actors {
            let actor_id = &actor.id;

            // Skip pedestrians - they don't follow lane-based kinematics
            if actor.role == ActorRole::Pedestrian {
                continue;
            }

            for t in 0..=self.horizon {
                let vx_t = &self.velocities_x[actor_id][t];

                // Constrain velocity direction based on actor direction
                if actor.direction == 1 {
                    // Forward direction: vx >= 0
                    self.backend.assert(&vx_t.ge(&zero));
                } else if actor.direction == -1 {
                    // Backward direction: vx <= 0
                    self.backend.assert(&vx_t.le(&zero));
                }
            }
        }
    }

    fn encode_acceleration_constraints(&mut self) {
        // Acceleration constraints are already encoded in encode_kinematics()
        // This method is a no-op for Cartesian encoder
    }

    fn encode_ttc_constraint(&self, actor1: &str, actor2: &str, min_ttc: f64, time: usize) -> Bool {
        let lane1 = &self.lanes[actor1][time];
        let lane2 = &self.lanes[actor2][time];

        let px1 = &self.positions_x[actor1][time];
        let px2 = &self.positions_x[actor2][time];

        let vx1 = &self.velocities_x[actor1][time];
        let vx2 = &self.velocities_x[actor2][time];

        let min_ttc_val = Real::from_rational((min_ttc * 10.0) as i64, 10_i64);
        let epsilon = Real::from_rational(1_i64, 100_i64); // 0.01 m/s to avoid division by zero

        // Same lane condition
        let same_lane = lane1.eq(lane2);

        // Determine who is ahead and who is behind
        // If px1 > px2, then actor1 is ahead (lead), actor2 is behind (follow)
        // If px2 > px1, then actor2 is ahead (lead), actor1 is behind (follow)

        // Case 1: actor1 ahead, actor2 behind, actor2 faster
        // TTC = (px1 - px2) / (vx2 - vx1)
        let actor1_ahead = px1.gt(px2);
        let actor2_faster = vx2.gt(vx1);
        let rel_vel_1 = vx2 - vx1;
        let distance_1 = px1 - px2;
        let collision_possible_1 =
            Bool::and(&[&actor1_ahead, &actor2_faster, &rel_vel_1.gt(&epsilon)]);
        // TTC > min_ttc means: distance / rel_vel > min_ttc
        // Equivalent to: distance > min_ttc * rel_vel
        let ttc_safe_1 = distance_1.gt(&(&min_ttc_val * &rel_vel_1));

        // Case 2: actor2 ahead, actor1 behind, actor1 faster
        // TTC = (px2 - px1) / (vx1 - vx2)
        let actor2_ahead = px2.gt(px1);
        let actor1_faster = vx1.gt(vx2);
        let rel_vel_2 = vx1 - vx2;
        let distance_2 = px2 - px1;
        let collision_possible_2 =
            Bool::and(&[&actor2_ahead, &actor1_faster, &rel_vel_2.gt(&epsilon)]);
        let ttc_safe_2 = distance_2.gt(&(&min_ttc_val * &rel_vel_2));

        // Overall constraint:
        // If same_lane AND collision_possible_1, then ttc_safe_1
        // If same_lane AND collision_possible_2, then ttc_safe_2
        // Otherwise, true (no collision risk)

        let case1 = Bool::and(&[&same_lane, &collision_possible_1]).implies(&ttc_safe_1);
        let case2 = Bool::and(&[&same_lane, &collision_possible_2]).implies(&ttc_safe_2);

        Bool::and(&[&case1, &case2])
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
        let distance_safe = Bool::or(&[&dist_fwd, &dist_bwd]);

        // If same lane, enforce distance constraint
        same_lane.implies(&distance_safe)
    }

    fn extract_actor_trajectory(
        &self,
        model: &Model,
        actor_id: &str,
        role: &str,
    ) -> Result<ActorTrajectory> {
        let mut trajectory = ActorTrajectory::new(actor_id.to_string(), role.to_string());

        for t in 0..=self.horizon {
            let time = t as f64 * self.spec.time_step;

            // Extract Cartesian values
            let px = self.extract_real(model, &self.positions_x[actor_id][t])?;
            let py = self.extract_real(model, &self.positions_y[actor_id][t])?;
            let vx = self.extract_real(model, &self.velocities_x[actor_id][t])?;
            let vy = self.extract_real(model, &self.velocities_y[actor_id][t])?;
            let ax = self.extract_real(model, &self.accelerations_x[actor_id][t])?;
            let ay = self.extract_real(model, &self.accelerations_y[actor_id][t])?;
            let lane = self.extract_int(model, &self.lanes[actor_id][t])?;

            let state = State {
                time,
                cartesian: Some(CartesianState {
                    position: Position::new(px, py),
                    velocity: Velocity::new(vx, vy),
                    acceleration: Acceleration::new(ax, ay),
                    lane,
                }),
            };

            trajectory.add_state(state);
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
        &self.velocities_x[actor_id][time]
    }

    fn get_lane_var(&self, actor_id: &str, time: usize) -> &Int {
        &self.lanes[actor_id][time]
    }

    fn get_lateral_vel(&self, actor_id: &str, time: usize) -> &Real {
        &self.velocities_y[actor_id][time]
    }

    fn encode_lane_velocity_constraints(&mut self) {
        let zero = Real::from_rational(0_i64, 1_i64);

        for actor in &self.spec.actors {
            let actor_id = &actor.id;

            // Skip pedestrians - they don't follow lane-based kinematics
            if actor.role == ActorRole::Pedestrian {
                continue;
            }

            // Use actor's direction (independent of lane direction)
            let direction = actor.direction;

            for t in 0..=self.horizon {
                let vx_t = &self.velocities_x[actor_id][t];

                if direction == 1 {
                    // Forward: vx >= 0
                    self.backend.assert(&vx_t.ge(&zero));
                } else {
                    // Backward: vx <= 0
                    self.backend.assert(&vx_t.le(&zero));
                }
            }
        }

        // Add lane bounds: 0 <= lane < num_lanes for all actors and time steps
        let num_lanes = self.spec.get_num_lanes();
        let max_lane = Int::from_i64((num_lanes - 1) as i64);
        let zero_lane = Int::from_i64(0);

        for actor in &self.spec.actors {
            let actor_id = &actor.id;
            for t in 0..=self.horizon {
                let lane_var = &self.lanes[actor_id][t];
                self.backend.assert(&lane_var.ge(&zero_lane));
                self.backend.assert(&lane_var.le(&max_lane));
            }
        }

        // Add single-lane-jump constraint: |lane[t+1] - lane[t]| <= 1
        // Prevents vehicles from jumping multiple lanes at once
        let one = Int::from_i64(1);
        let neg_one = Int::from_i64(-1);

        for actor in &self.spec.actors {
            if actor.role != ActorRole::Ego {
                // Ego already constrained (vy = 0, lane never changes)
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
        // Use realistic lateral velocity bound for vehicles
        // For 3.5m lane change over 3s with dt=0.1s: vy ≈ 1.17 m/s (realistic)
        // Setting max to 2.0 m/s allows for smooth lane changes
        let max_vy = 2.0; // m/s
        let max_vy_real = Real::from_rational((max_vy * 10.0) as i64, 10_i64);
        let neg_max_vy_real = Real::from_rational((-max_vy * 10.0) as i64, 10_i64);

        for actor in &self.spec.actors {
            if actor.role != ActorRole::Ego {
                // Ego vy already constrained to 0 (no lane changes)
                let actor_id = &actor.id;
                for t in 0..=self.horizon {
                    let vy_t = &self.velocities_y[actor_id][t];
                    self.backend.assert(&vy_t.ge(&neg_max_vy_real));
                    self.backend.assert(&vy_t.le(&max_vy_real));
                }
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
