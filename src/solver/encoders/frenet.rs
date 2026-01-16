//! Frenet coordinate system encoder (without polynomial pre-computation)
//!
//! Implements the CoordinateEncoder trait for Frenet (s, t) coordinates.
//! This encoder lets the Z3 solver discover lane change trajectories dynamically
//! rather than pre-computing them with polynomials.

use std::collections::HashMap;
use z3::ast::{Bool, Int, Real};
use z3::Model;

use crate::dsl::types::{ActorRole, CoordinateSystem, ScenarioSpec, ValueOrRange};
use crate::error::{Result, ScenarioGenError};
use crate::geometry::FrenetPoint;
use crate::scenario::model::{
    Acceleration, ActorTrajectory, CartesianState, FrenetState, Position, State, Velocity,
};
use crate::solver::backend::Z3Backend;
use crate::solver::coordinate_encoder::CoordinateEncoder;

/// Frenet coordinate system encoder
///
/// Uses (s, t) position variables where:
/// - s: longitudinal position along reference line
/// - t: lateral offset from reference line
///
/// Lane changes are discovered by the solver with smoothness constraints,
/// not pre-computed using polynomials.
pub struct FrenetEncoder<B: Z3Backend> {
    /// Z3 backend (Solver or Optimizer)
    backend: B,

    /// Scenario specification
    spec: ScenarioSpec,

    /// Number of time steps
    horizon: usize,

    // Variable maps: actor_id -> Vec<variable> (one per time step)
    /// Longitudinal positions along reference line (m)
    frenet_s: HashMap<String, Vec<Real>>,

    /// Lateral offsets from reference line (m)
    frenet_t: HashMap<String, Vec<Real>>,

    /// Longitudinal velocities (m/s)
    frenet_vs: HashMap<String, Vec<Real>>,

    /// Lateral velocities (m/s)
    frenet_vt: HashMap<String, Vec<Real>>,

    /// Longitudinal accelerations (m/s²)
    frenet_as: HashMap<String, Vec<Real>>,

    /// Lateral accelerations (m/s²)
    frenet_at: HashMap<String, Vec<Real>>,

    /// Lane numbers (integer) - for compatibility
    lanes: HashMap<String, Vec<Int>>,
}

impl<B: Z3Backend> FrenetEncoder<B> {
    /// Create a new Frenet encoder
    pub fn new(spec: ScenarioSpec, backend: B) -> Self {
        let horizon = spec.num_time_steps();

        Self {
            backend,
            spec,
            horizon,
            frenet_s: HashMap::new(),
            frenet_t: HashMap::new(),
            frenet_vs: HashMap::new(),
            frenet_vt: HashMap::new(),
            frenet_as: HashMap::new(),
            frenet_at: HashMap::new(),
            lanes: HashMap::new(),
        }
    }

    /// Encode smoothness constraints for lane changes
    ///
    /// During a lane change, constrain lateral acceleration and velocity
    /// to ensure smooth, realistic trajectories.
    fn encode_lane_change_smoothness(
        &mut self,
        actor_id: &str,
        start_min: f64,
        start_max: f64,
        duration_min: f64,
        duration_max: f64,
    ) {
        let dt = self.spec.time_step;

        // Convert time ranges to step ranges
        let start_step_min = (start_min / dt) as usize;
        let start_step_max = (start_max / dt) as usize;
        let duration_steps_min = (duration_min / dt) as usize;
        let duration_steps_max = (duration_max / dt) as usize;

        // For simplicity in the first implementation, use the midpoint of ranges
        // TODO: Make these solver variables for full flexibility
        let start_step = (start_step_min + start_step_max) / 2;
        let duration_steps = (duration_steps_min + duration_steps_max) / 2;
        let end_step = start_step + duration_steps;

        // Smoothness constraints during lane change
        for t in start_step..end_step.min(self.horizon) {
            // Constrain lateral acceleration for smoothness
            let at_t = &self.frenet_at[actor_id][t];
            let max_at = Real::from_rational(20_i64, 10_i64); // 2.0 m/s²
            self.backend.assert(&at_t.le(&max_at));
            self.backend.assert(&at_t.ge(&-&max_at));

            // Constrain lateral velocity
            let vt_t = &self.frenet_vt[actor_id][t];
            let max_vt = Real::from_rational(25_i64, 10_i64); // 2.5 m/s
            self.backend.assert(&vt_t.le(&max_vt));
            self.backend.assert(&vt_t.ge(&-&max_vt));
        }
    }

    /// Encode initial state for a single actor (Frenet-specific)
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
        _direction: i32,
    ) {
        let zero = Real::from_rational(0_i64, 1_i64);

        // Lane at t=0 (for compatibility)
        let lane_var = &self.lanes[actor_id][0];
        let lane_val = Int::from_i64(lane as i64);
        self.backend.assert(&lane_var.eq(&lane_val));

        // Calculate initial Frenet coordinates from lane
        let lane_width = self.spec.lane_width;

        // Initial lateral position t: center of the lane
        // Lane 0 is at 0.5*lane_width, Lane 1 is at 1.5*lane_width, etc.
        let t_initial = (lane as f64 + 0.5) * lane_width;
        let t_val = Real::from_rational((t_initial * 10.0) as i64, 10_i64);
        let t_var = &self.frenet_t[actor_id][0];
        self.backend.assert(&t_var.eq(&t_val));

        // Initial longitudinal position s: convert from x position
        let s_min = Real::from_rational((pos_min * 10.0) as i64, 10_i64);
        let s_max = Real::from_rational((pos_max * 10.0) as i64, 10_i64);
        let s_var = &self.frenet_s[actor_id][0];
        if (pos_min - pos_max).abs() < 1e-6 {
            self.backend.assert(&s_var.eq(&s_min));
        } else {
            self.backend.assert(&s_var.ge(&s_min));
            self.backend.assert(&s_var.le(&s_max));
        }

        // Initial longitudinal velocity vs: same as speed magnitude
        if (speed_min - speed_max).abs() < 1e-6 {
            let vs_val = Real::from_rational((speed_min * 10.0) as i64, 10_i64);
            let vs_var = &self.frenet_vs[actor_id][0];
            self.backend.assert(&vs_var.eq(&vs_val));
        } else {
            let vs_min = Real::from_rational((speed_min * 10.0) as i64, 10_i64);
            let vs_max = Real::from_rational((speed_max * 10.0) as i64, 10_i64);
            let vs_var = &self.frenet_vs[actor_id][0];
            self.backend.assert(&vs_var.ge(&vs_min));
            self.backend.assert(&vs_var.le(&vs_max));
        }

        // Initial lateral velocity vt: zero (not changing lanes initially)
        let vt_var = &self.frenet_vt[actor_id][0];
        self.backend.assert(&vt_var.eq(&zero));

        // Initial longitudinal acceleration as: use acceleration range
        let as_var = &self.frenet_as[actor_id][0];
        if (accel_min - accel_max).abs() < 1e-6 {
            let as_val = Real::from_rational((accel_min * 10.0) as i64, 10_i64);
            self.backend.assert(&as_var.eq(&as_val));
        } else {
            let as_min = Real::from_rational((accel_min * 10.0) as i64, 10_i64);
            let as_max = Real::from_rational((accel_max * 10.0) as i64, 10_i64);
            self.backend.assert(&as_var.ge(&as_min));
            self.backend.assert(&as_var.le(&as_max));
        }

        // Initial lateral acceleration at: zero (not changing lanes initially)
        let at_var = &self.frenet_at[actor_id][0];
        if role != ActorRole::Pedestrian {
            self.backend.assert(&at_var.eq(&zero));
        }
    }

    /// Extract a real value from Z3 model
    fn extract_real(&self, model: &Model, var: &Real) -> Result<f64> {
        let ast = model.eval(var, true).ok_or_else(|| {
            ScenarioGenError::Z3ModelParsing("Failed to evaluate real variable".to_string())
        })?;

        if let Some(rational) = ast.as_real() {
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
}

impl<B: Z3Backend> CoordinateEncoder<B> for FrenetEncoder<B> {
    fn create_variables(&mut self, horizon: usize, spec: &ScenarioSpec) {
        for actor in &spec.actors {
            let actor_id = &actor.id;

            let mut s_vars = Vec::new();
            let mut t_vars = Vec::new();
            let mut vs_vars = Vec::new();
            let mut vt_vars = Vec::new();
            let mut as_vars = Vec::new();
            let mut at_vars = Vec::new();
            let mut lane_vars = Vec::new();

            // Create variables for each time step
            for t in 0..=horizon {
                s_vars.push(Real::new_const(format!("{}_s_{}", actor_id, t)));
                t_vars.push(Real::new_const(format!("{}_t_{}", actor_id, t)));
                vs_vars.push(Real::new_const(format!("{}_vs_{}", actor_id, t)));
                vt_vars.push(Real::new_const(format!("{}_vt_{}", actor_id, t)));
                as_vars.push(Real::new_const(format!("{}_as_{}", actor_id, t)));
                at_vars.push(Real::new_const(format!("{}_at_{}", actor_id, t)));
                lane_vars.push(Int::new_const(format!("{}_lane_{}", actor_id, t)));
            }

            self.frenet_s.insert(actor_id.clone(), s_vars);
            self.frenet_t.insert(actor_id.clone(), t_vars);
            self.frenet_vs.insert(actor_id.clone(), vs_vars);
            self.frenet_vt.insert(actor_id.clone(), vt_vars);
            self.frenet_as.insert(actor_id.clone(), as_vars);
            self.frenet_at.insert(actor_id.clone(), at_vars);
            self.lanes.insert(actor_id.clone(), lane_vars);
        }
    }

    fn encode_kinematics(&mut self, dt: f64) {
        let dt_real = Real::from_rational((dt * 10.0) as i64, 10_i64);
        let zero = Real::from_rational(0_i64, 1_i64);

        for actor in &self.spec.actors {
            let actor_id = &actor.id;

            // Get acceleration bounds
            let ax_min = actor.acceleration.min();
            let ax_max = actor.acceleration.max();
            let ax_min_real = Real::from_rational((ax_min * 10.0) as i64, 10_i64);
            let ax_max_real = Real::from_rational((ax_max * 10.0) as i64, 10_i64);

            // Longitudinal kinematics (s)
            for t in 0..self.horizon {
                // Longitudinal acceleration bounds
                let as_t = &self.frenet_as[actor_id][t];
                self.backend.assert(&as_t.ge(&ax_min_real));
                self.backend.assert(&as_t.le(&ax_max_real));

                // Longitudinal velocity update: vs[t+1] = vs[t] + as[t] * dt
                let vs_t = &self.frenet_vs[actor_id][t];
                let vs_t1 = &self.frenet_vs[actor_id][t + 1];
                let expected_vs = vs_t + &(as_t * &dt_real);
                self.backend.assert(&vs_t1.eq(&expected_vs));

                // Longitudinal position update: s[t+1] = s[t] + vs[t] * dt
                let s_t = &self.frenet_s[actor_id][t];
                let s_t1 = &self.frenet_s[actor_id][t + 1];
                let expected_s = s_t + &(vs_t * &dt_real);
                self.backend.assert(&s_t1.eq(&expected_s));
            }

            // Lateral kinematics (t, vt) - NEW APPROACH WITHOUT POLYNOMIAL
            for t in 0..self.horizon {
                let vt_t = &self.frenet_vt[actor_id][t];
                let vt_t1 = &self.frenet_vt[actor_id][t + 1];
                let t_t = &self.frenet_t[actor_id][t];
                let t_t1 = &self.frenet_t[actor_id][t + 1];
                let at_t = &self.frenet_at[actor_id][t];

                if actor.role == ActorRole::Pedestrian {
                    // Pedestrians: full lateral dynamics
                    self.backend.assert(&at_t.ge(&ax_min_real));
                    self.backend.assert(&at_t.le(&ax_max_real));

                    // Lateral velocity update: vt[t+1] = vt[t] + at[t] * dt
                    let expected_vt = vt_t + &(at_t * &dt_real);
                    self.backend.assert(&vt_t1.eq(&expected_vt));
                } else if actor.role == ActorRole::Ego {
                    // Ego: no lane changes
                    self.backend.assert(&vt_t.eq(&zero));
                } else {
                    // Other vehicles: allow lateral movement if lane change enabled
                    if let Some(lc) = &actor.lane_change {
                        if lc.enabled {
                            // Lateral dynamics with smoothness constraints
                            let expected_vt = vt_t + &(at_t * &dt_real);
                            self.backend.assert(&vt_t1.eq(&expected_vt));

                            // Note: Smoothness constraints applied separately
                        } else {
                            // No lane change: vt = 0
                            self.backend.assert(&vt_t.eq(&zero));
                        }
                    } else {
                        // No lane change config: vt = 0
                        self.backend.assert(&vt_t.eq(&zero));
                    }
                }

                // Lateral position update: t[t+1] = t[t] + vt[t] * dt
                let expected_t = t_t + &(vt_t * &dt_real);
                self.backend.assert(&t_t1.eq(&expected_t));
            }
        }

        // Apply smoothness constraints for lane changes
        // Collect lane change data upfront to avoid borrow checker issues
        let lane_change_data: Vec<_> = self
            .spec
            .actors
            .iter()
            .filter_map(|actor| {
                if let Some(lc) = &actor.lane_change {
                    if lc.enabled
                        && actor.role != ActorRole::Ego
                        && actor.role != ActorRole::Pedestrian
                    {
                        // Extract min/max from ValueOrRange
                        Some((
                            actor.id.clone(),
                            lc.start_time.min(),
                            lc.start_time.max(),
                            lc.duration.min(),
                            lc.duration.max(),
                        ))
                    } else {
                        None
                    }
                } else {
                    None
                }
            })
            .collect();

        for (actor_id, start_min, start_max, duration_min, duration_max) in lane_change_data {
            self.encode_lane_change_smoothness(
                &actor_id,
                start_min,
                start_max,
                duration_min,
                duration_max,
            );
        }

        // Add explicit bounds on lateral position t to ensure vehicles stay on road
        let road_width = (self.spec.get_num_lanes() as f64) * self.spec.lane_width;
        let road_width_real = Real::from_rational((road_width * 10.0) as i64, 10_i64);
        let zero_t = Real::from_rational(0_i64, 1_i64);

        for actor in &self.spec.actors {
            let actor_id = &actor.id;
            for t in 0..=self.horizon {
                let t_var = &self.frenet_t[actor_id][t];
                // 0 <= t <= road_width
                self.backend.assert(&t_var.ge(&zero_t));
                self.backend.assert(&t_var.le(&road_width_real));
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
        // In Frenet system, velocity constraints are mostly handled in kinematics
        // Direction constraints based on actor direction
        let zero = Real::from_rational(0_i64, 1_i64);

        for actor in &self.spec.actors {
            if actor.role == ActorRole::Pedestrian {
                continue;
            }

            let actor_id = &actor.id;
            for t in 0..=self.horizon {
                let vs_t = &self.frenet_vs[actor_id][t];

                // Constrain velocity direction (assuming forward motion)
                if actor.direction == 1 {
                    self.backend.assert(&vs_t.ge(&zero));
                } else if actor.direction == -1 {
                    self.backend.assert(&vs_t.le(&zero));
                }
            }
        }
    }

    fn encode_acceleration_constraints(&mut self) {
        // Acceleration constraints are already encoded in encode_kinematics()
        // This method is a no-op for Frenet encoder
    }

    fn encode_ttc_constraint(
        &self,
        actor1: &str,
        actor2: &str,
        min_ttc: f64,
        time: usize,
    ) -> Bool {
        // TTC constraint using Frenet coordinates (similar to Cartesian but with s/vs)
        let lane1 = &self.lanes[actor1][time];
        let lane2 = &self.lanes[actor2][time];

        let s1 = &self.frenet_s[actor1][time];
        let s2 = &self.frenet_s[actor2][time];

        let vs1 = &self.frenet_vs[actor1][time];
        let vs2 = &self.frenet_vs[actor2][time];

        let min_ttc_val = Real::from_rational((min_ttc * 10.0) as i64, 10_i64);
        let epsilon = Real::from_rational(1_i64, 100_i64); // 0.01 m/s

        // Same lane condition
        let same_lane = lane1.eq(lane2);

        // Case 1: actor1 ahead, actor2 behind, actor2 faster
        let actor1_ahead = s1.gt(s2);
        let actor2_faster = vs2.gt(vs1);
        let rel_vel_1 = vs2 - vs1;
        let distance_1 = s1 - s2;
        let collision_possible_1 =
            Bool::and(&[&actor1_ahead, &actor2_faster, &rel_vel_1.gt(&epsilon)]);
        let ttc_safe_1 = distance_1.gt(&(&min_ttc_val * &rel_vel_1));

        // Case 2: actor2 ahead, actor1 behind, actor1 faster
        let actor2_ahead = s2.gt(s1);
        let actor1_faster = vs1.gt(vs2);
        let rel_vel_2 = vs1 - vs2;
        let distance_2 = s2 - s1;
        let collision_possible_2 =
            Bool::and(&[&actor2_ahead, &actor1_faster, &rel_vel_2.gt(&epsilon)]);
        let ttc_safe_2 = distance_2.gt(&(&min_ttc_val * &rel_vel_2));

        // Overall constraint
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

        let s1 = &self.frenet_s[actor1][time];
        let s2 = &self.frenet_s[actor2][time];

        let min_dist_val = Real::from_rational((min_dist * 10.0) as i64, 10_i64);

        // Same lane condition
        let same_lane = lane1.eq(lane2);

        // Distance constraint: |s1 - s2| >= min_dist
        let dist_fwd = (s1 - s2).ge(&min_dist_val);
        let dist_bwd = (s2 - s1).ge(&min_dist_val);
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

        // Get reference line for Frenet conversion
        let ref_line = self.spec.reference_line.as_ref();

        for t in 0..=self.horizon {
            let time = t as f64 * self.spec.time_step;

            // Extract Frenet values
            let s = self.extract_real(model, &self.frenet_s[actor_id][t])?;
            let t_val = self.extract_real(model, &self.frenet_t[actor_id][t])?;
            let vs = self.extract_real(model, &self.frenet_vs[actor_id][t])?;
            let vt = self.extract_real(model, &self.frenet_vt[actor_id][t])?;
            let as_ = self.extract_real(model, &self.frenet_as[actor_id][t])?;
            let at = self.extract_real(model, &self.frenet_at[actor_id][t])?;

            // Calculate theta (heading) from road heading
            let theta = ref_line.map(|rl| rl.heading).unwrap_or(0.0);

            // Compute Cartesian from Frenet
            let cartesian = if let Some(rl) = ref_line {
                let frenet_point = FrenetPoint::new(s, t_val);
                let cart_point = rl.frenet_to_cartesian(&frenet_point);

                // Convert Frenet velocity to Cartesian velocity
                let vx = vs * theta.cos() - vt * theta.sin();
                let vy = vs * theta.sin() + vt * theta.cos();

                Some(CartesianState {
                    position: Position::new(cart_point.x, cart_point.y),
                    velocity: Velocity::new(vx, vy),
                    acceleration: Acceleration::new(as_, at),
                    lane: (t_val / self.spec.lane_width).round() as usize,
                })
            } else {
                None
            };

            let state = State {
                time,
                frenet: Some(FrenetState {
                    s,
                    t: t_val,
                    theta,
                    vs,
                    vt,
                    as_,
                    at,
                }),
                cartesian,
            };

            trajectory.add_state(state);
        }

        Ok(trajectory)
    }

    fn get_longitudinal_pos(&self, actor_id: &str, time: usize) -> &Real {
        &self.frenet_s[actor_id][time]
    }

    fn get_lateral_pos(&self, actor_id: &str, time: usize) -> &Real {
        &self.frenet_t[actor_id][time]
    }

    fn get_longitudinal_vel(&self, actor_id: &str, time: usize) -> &Real {
        &self.frenet_vs[actor_id][time]
    }

    fn get_lane_var(&self, actor_id: &str, time: usize) -> &Int {
        &self.lanes[actor_id][time]
    }

    fn get_lateral_vel(&self, actor_id: &str, time: usize) -> &Real {
        &self.frenet_vt[actor_id][time]
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
                let vs_t = &self.frenet_vs[actor_id][t];

                if direction == 1 {
                    // Forward: vs >= 0
                    self.backend.assert(&vs_t.ge(&zero));
                } else {
                    // Backward: vs <= 0
                    self.backend.assert(&vs_t.le(&zero));
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
                // Ego already constrained (vt = 0, lane never changes)
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
        // max_vt allows single-timestep lane changes with 10% buffer
        // This is much more reasonable than unconstrained (which produced 14+ m/s)
        let max_vt = (self.spec.lane_width / self.spec.time_step) * 1.1;
        let max_vt_real = Real::from_rational((max_vt * 10.0) as i64, 10_i64);
        let neg_max_vt_real = Real::from_rational((-max_vt * 10.0) as i64, 10_i64);

        for actor in &self.spec.actors {
            if actor.role != ActorRole::Ego {
                // Ego vt already constrained to 0 (no lane changes)
                let actor_id = &actor.id;
                for t in 0..=self.horizon {
                    let vt_t = &self.frenet_vt[actor_id][t];
                    self.backend.assert(&vt_t.ge(&neg_max_vt_real));
                    self.backend.assert(&vt_t.le(&max_vt_real));
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
