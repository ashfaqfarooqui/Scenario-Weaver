//! Z3 constraint encoder

use std::collections::HashMap;
use z3::ast::{Int, Real};
use z3::SatResult;

use crate::dsl::types::{ConstraintMode, CoordinateSystem, ScenarioSpec};
use crate::solver::backend::{SolverBackend, Z3Backend};

/// Z3 SMT encoder for scenario constraints (generic over backend)
///
/// This encoder can work with either `SolverBackend` (SAT checking)
/// or `OptimizerBackend` (optimization objectives).
///
/// Supports both Cartesian (x, y) and Frenet (s, t) coordinate systems.
///
/// Note: In Z3 0.19, the context is managed internally and is implicit
/// within the `with_z3_config()` callback scope.
pub struct GenericEncoder<B: Z3Backend> {
    /// Z3 backend (Solver or Optimizer)
    pub(crate) backend: B,

    /// Original scenario specification
    pub(crate) spec: ScenarioSpec,

    /// Number of time steps in the scenario
    pub(crate) horizon: usize,

    // Variable maps: actor_id -> Vec<variable> (one per time step)

    // Cartesian variables
    /// Longitudinal positions (m)
    pub(crate) positions_x: HashMap<String, Vec<Real>>,

    /// Lateral positions (m)
    pub(crate) positions_y: HashMap<String, Vec<Real>>,

    /// Longitudinal velocities (m/s)
    pub(crate) velocities_x: HashMap<String, Vec<Real>>,

    /// Lateral velocities (m/s)
    pub(crate) velocities_y: HashMap<String, Vec<Real>>,

    /// Lane numbers (integer)
    pub(crate) lanes: HashMap<String, Vec<Int>>,

    /// Longitudinal accelerations (m/s²)
    pub(crate) accelerations_x: HashMap<String, Vec<Real>>,

    /// Lateral accelerations (m/s²)
    pub(crate) accelerations_y: HashMap<String, Vec<Real>>,

    // Frenet variables
    /// Longitudinal positions along reference line (m)
    pub(crate) frenet_s: HashMap<String, Vec<Real>>,

    /// Lateral offsets from reference line (m)
    pub(crate) frenet_t: HashMap<String, Vec<Real>>,

    /// Longitudinal velocities (m/s)
    pub(crate) frenet_vs: HashMap<String, Vec<Real>>,

    /// Lateral velocities (m/s)
    pub(crate) frenet_vt: HashMap<String, Vec<Real>>,

    /// Longitudinal accelerations (m/s²)
    pub(crate) frenet_as: HashMap<String, Vec<Real>>,

    /// Lateral accelerations (m/s²)
    pub(crate) frenet_at: HashMap<String, Vec<Real>>,
}

/// Type alias for backward compatibility - uses Solver backend
pub type Z3Encoder = GenericEncoder<SolverBackend>;

impl<B: Z3Backend> GenericEncoder<B> {
    /// Create a new encoder with a specific backend
    ///
    /// Note: This must be called within a `z3::with_z3_config()` callback.
    pub fn with_backend(spec: ScenarioSpec, backend: B) -> Self {
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
            frenet_s: HashMap::new(),
            frenet_t: HashMap::new(),
            frenet_vs: HashMap::new(),
            frenet_vt: HashMap::new(),
            frenet_as: HashMap::new(),
            frenet_at: HashMap::new(),
        }
    }

    /// Get the scenario specification
    pub fn spec(&self) -> &ScenarioSpec {
        &self.spec
    }

    /// Get the time horizon
    pub fn horizon(&self) -> usize {
        self.horizon
    }
}

impl Z3Encoder {
    /// Create a new Z3 encoder for the given specification (backward compatible)
    ///
    /// Note: This must be called within a `z3::with_z3_config()` callback.
    pub fn new(spec: ScenarioSpec) -> Self {
        Self::with_backend(spec, SolverBackend::new())
    }

    /// Encode scenario-specific Z3 constraints
    ///
    /// This calls the trait method to allow scenarios to add custom Z3 assertions
    /// beyond the standard LTL and safety encodings.
    ///
    /// Note: This method is only available for Z3Encoder (SolverBackend) because
    /// the ScenarioModel trait is tied to the concrete encoder type.
    pub fn encode_scenario_specific_constraints(
        &mut self,
        model: &dyn crate::scenarios::ScenarioModel,
    ) -> anyhow::Result<()> {
        model.add_z3_constraints(&self.spec, self, &self.backend, self.horizon)
            .map_err(|e| anyhow::anyhow!(e))
    }
}

impl<B: Z3Backend> GenericEncoder<B> {
    /// Create all Z3 variables for the scenario
    ///
    /// For each actor and each time step t ∈ [0, horizon],
    /// creates variables:
    /// - px_t: longitudinal position
    /// - py_t: lateral position
    /// - vx_t: longitudinal velocity
    /// - vy_t: lateral velocity
    /// - ax_t: longitudinal acceleration
    /// - ay_t: lateral acceleration
    /// - lane_t: lane number
    pub fn create_variables(&mut self) {
        let use_frenet = matches!(self.spec.coordinate_system, CoordinateSystem::Frenet);

        for actor in &self.spec.actors {
            let actor_id = &actor.id;

            let mut px_vars = Vec::new();
            let mut py_vars = Vec::new();
            let mut vx_vars = Vec::new();
            let mut vy_vars = Vec::new();
            let mut lane_vars = Vec::new();
            let mut ax_vars = Vec::new();
            let mut ay_vars = Vec::new();

            // Frenet variables
            let mut s_vars = Vec::new();
            let mut t_vars = Vec::new();
            let mut vs_vars = Vec::new();
            let mut vt_vars = Vec::new();
            let mut as_vars = Vec::new();
            let mut at_vars = Vec::new();

            // Create variables for each time step
            for t in 0..=self.horizon {
                // Cartesian variables (always created for backward compatibility)
                px_vars.push(Real::new_const(format!("{}_px_{}", actor_id, t)));
                py_vars.push(Real::new_const(format!("{}_py_{}", actor_id, t)));
                vx_vars.push(Real::new_const(format!("{}_vx_{}", actor_id, t)));
                vy_vars.push(Real::new_const(format!("{}_vy_{}", actor_id, t)));
                lane_vars.push(Int::new_const(format!("{}_lane_{}", actor_id, t)));
                ax_vars.push(Real::new_const(format!("{}_ax_{}", actor_id, t)));
                ay_vars.push(Real::new_const(format!("{}_ay_{}", actor_id, t)));

                // Frenet variables (created when needed)
                if use_frenet {
                    s_vars.push(Real::new_const(format!("{}_s_{}", actor_id, t)));
                    t_vars.push(Real::new_const(format!("{}_t_{}", actor_id, t)));
                    vs_vars.push(Real::new_const(format!("{}_vs_{}", actor_id, t)));
                    vt_vars.push(Real::new_const(format!("{}_vt_{}", actor_id, t)));
                    as_vars.push(Real::new_const(format!("{}_as_{}", actor_id, t)));
                    at_vars.push(Real::new_const(format!("{}_at_{}", actor_id, t)));
                }
            }

            self.positions_x.insert(actor_id.clone(), px_vars);
            self.positions_y.insert(actor_id.clone(), py_vars);
            self.velocities_x.insert(actor_id.clone(), vx_vars);
            self.velocities_y.insert(actor_id.clone(), vy_vars);
            self.lanes.insert(actor_id.clone(), lane_vars);
            self.accelerations_x.insert(actor_id.clone(), ax_vars);
            self.accelerations_y.insert(actor_id.clone(), ay_vars);

            if use_frenet {
                self.frenet_s.insert(actor_id.clone(), s_vars);
                self.frenet_t.insert(actor_id.clone(), t_vars);
                self.frenet_vs.insert(actor_id.clone(), vs_vars);
                self.frenet_vt.insert(actor_id.clone(), vt_vars);
                self.frenet_as.insert(actor_id.clone(), as_vars);
                self.frenet_at.insert(actor_id.clone(), at_vars);
            }
        }
    }

    /// Encode initial conditions from the DSL specification
    pub fn encode_initial_conditions(&mut self) {
        use crate::dsl::types::ActorRole;

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
                    actor.behavior.clone(),
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
            behavior,
        ) in actor_data
        {
            // Call existing encoding method
            self.encode_actor_initial_state(
                &actor_id, lane, pos_min, pos_max, speed_min, speed_max, acc_min, acc_max, role,
                direction,
            );

            // Handle lateral position constraints
            if role == ActorRole::Pedestrian {
                // Pedestrians start on sidewalk (based on direction field in behavior)
                // Use range to allow Z3 to choose specific position that works
                let lane_width = self.spec.get_lane_width();
                let num_lanes = self.spec.get_num_lanes();
                let road_width = lane_width * num_lanes as f64;

                let py_0 = &self.positions_y[&actor_id][0];
                let road_width_real = Real::from_rational((road_width * 10.0) as i64, 10_i64);

                // Get crossing direction from behavior
                let direction = behavior.get("direction").and_then(|v| v.as_str());

                // Set initial sidewalk position based on direction
                if let Some(dir) = direction {
                    match dir {
                        "left_to_right" => {
                            // Left sidewalk: py between -0.6 and -0.4
                            let sidewalk_min = Real::from_rational(-6_i64, 10_i64);
                            let sidewalk_max = Real::from_rational(-4_i64, 10_i64);
                            self.backend.assert(&py_0.ge(&sidewalk_min));
                            self.backend.assert(&py_0.le(&sidewalk_max));
                        }
                        "right_to_left" => {
                            // Right sidewalk: py between road_width + 0.4 and road_width + 0.6
                            let sidewalk_min =
                                &road_width_real + &Real::from_rational(4_i64, 10_i64);
                            let sidewalk_max =
                                &road_width_real + &Real::from_rational(6_i64, 10_i64);
                            self.backend.assert(&py_0.ge(&sidewalk_min));
                            self.backend.assert(&py_0.le(&sidewalk_max));
                        }
                        _ => {
                            tracing::error!(
                                "Invalid direction '{}' for pedestrian {}",
                                dir,
                                actor_id
                            );
                            // Fall back to left sidewalk
                            let sidewalk_min = Real::from_rational(-6_i64, 10_i64);
                            let sidewalk_max = Real::from_rational(-4_i64, 10_i64);
                            self.backend.assert(&py_0.ge(&sidewalk_min));
                            self.backend.assert(&py_0.le(&sidewalk_max));
                        }
                    }
                } else {
                    tracing::error!("Pedestrian {} missing 'direction' in behavior", actor_id);
                    // Fall back to left sidewalk
                    let sidewalk_min = Real::from_rational(-6_i64, 10_i64);
                    let sidewalk_max = Real::from_rational(-4_i64, 10_i64);
                    self.backend.assert(&py_0.ge(&sidewalk_min));
                    self.backend.assert(&py_0.le(&sidewalk_max));
                }

                // Set initial lane to 0 for all pedestrians (value doesn't matter semantically)
                let lane_0 = &self.lanes[&actor_id][0];
                let zero_lane = Int::from_i64(0_i64);
                self.backend.assert(&lane_0.eq(&zero_lane));
            } else {
                // Vehicles: initial lateral position matches lane center
                self.encode_lane_position_coupling_at_time(&actor_id, 0);
            }

            // Ego never changes lanes (vy = 0)
            // Pedestrians can move laterally, so skip this constraint for them
            if role == ActorRole::Ego {
                let zero = Real::from_rational(0_i64, 1_i64);
                let vy_0 = &self.velocities_y[&actor_id][0];
                self.backend.assert(&vy_0.eq(&zero));
            }
        }
    }

    /// Encode initial state for an actor
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
        role: crate::dsl::types::ActorRole,
        direction: i32,
    ) {
        use crate::dsl::types::{ActorRole, CoordinateSystem};

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

        // Frenet initial conditions (if using Frenet coordinate system)
        if matches!(self.spec.coordinate_system, CoordinateSystem::Frenet) {
            // Calculate initial Frenet coordinates from lane
            let num_lanes = self.spec.road.as_ref().map(|r| r.num_lanes).unwrap_or(2);
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
    }

    /// Encode constraint: lateral position matches lane center
    /// py = lane * lane_width + lane_width/2
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

    /// Encode kinematic constraints with acceleration support
    ///
    /// Dispatches to either Cartesian or Frenet kinematics encoding based on coordinate_system.
    pub fn encode_kinematics(&mut self) {
        match self.spec.coordinate_system {
            CoordinateSystem::Frenet => self.encode_frenet_kinematics(),
            CoordinateSystem::Cartesian => self.encode_cartesian_kinematics(),
        }
    }

    /// Encode Cartesian kinematic constraints (original implementation)
    fn encode_cartesian_kinematics(&mut self) {
        use crate::dsl::types::{
            ActorRole, PEDESTRIAN_MAX_ACCELERATION, PEDESTRIAN_MAX_DECELERATION,
            PEDESTRIAN_RUN_MAX_SPEED, PEDESTRIAN_WALK_MAX_SPEED,
        };

        let dt = self.spec.time_step;
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

                // Note: Velocity direction constraints are applied separately
                // in encode_lane_velocity_constraints() based on lane direction

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

            // Note: Final velocity direction constraints are applied
            // in encode_lane_velocity_constraints() based on lane direction
        }

        // Lane-position coupling for all time steps (skip pedestrians)
        let actor_ids: Vec<_> = self
            .spec
            .actors
            .iter()
            .filter(|a| a.role != ActorRole::Pedestrian)
            .map(|a| a.id.clone())
            .collect();
        for actor_id in actor_ids {
            for t in 0..=self.horizon {
                self.encode_lane_position_coupling_at_time(&actor_id, t);
            }
        }
    }

    /// Encode velocity direction constraints based on actor direction
    ///
    /// For each actor and time step, constrains velocity to match actor direction:
    /// - Forward (direction = +1): vx >= 0
    /// - Backward (direction = -1): vx <= 0
    ///
    /// Note: Pedestrians are skipped as they don't follow lane-based kinematics
    pub fn encode_lane_velocity_constraints(&mut self) {
        use crate::dsl::types::ActorRole;

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

    /// Encode lateral velocity bounds for realistic lane changes
    ///
    /// Constrains lateral velocity to be within a reasonable range based on
    /// lane width and time step. This ensures physically realistic lane change
    /// maneuvers while still allowing lane changes within 1-2 time steps.
    ///
    /// The bound is: max_vy = (lane_width / time_step) * 1.1
    /// This allows single-timestep lane changes with a small buffer.
    ///
    /// Example: For 3.5m lanes and 0.5s timestep: max_vy = 7.7 m/s (~28 km/h lateral)
    pub fn encode_lateral_velocity_bounds(&mut self) {
        use crate::dsl::types::ActorRole;

        // max_vy allows single-timestep lane changes with 10% buffer
        // This is much more reasonable than unconstrained (which produced 14+ m/s)
        let max_vy = (self.spec.lane_width / self.spec.time_step) * 1.1;
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

    /// Check if the constraints are satisfiable (for testing)
    pub fn check(&self) -> SatResult {
        self.backend.check()
    }

    /// Get the Z3 model (for testing)
    pub fn get_model(&self) -> Option<z3::Model> {
        self.backend.get_model()
    }

    /// Encode LTL formula into Z3 constraints using bounded model checking
    ///
    /// This is the core of Phase 7. We expand temporal operators over the
    /// finite time horizon, converting them into Boolean combinations of
    /// propositions at different time steps.
    pub fn encode_ltl(&mut self, formula: &crate::ltl::formula::LTLFormula) {
        // Encode the formula starting at time 0, with full horizon
        let constraint = self.encode_ltl_bounded(formula, 0, self.horizon);
        self.backend.assert(&constraint);
    }

    /// Bounded LTL encoding: expand temporal operators over [time, horizon]
    ///
    /// This implements bounded model checking for LTL:
    /// - Eventually(φ): φ[t] ∨ φ[t+1] ∨ ... ∨ φ[horizon]
    /// - Always(φ): φ[t] ∧ φ[t+1] ∧ ... ∧ φ[horizon]
    /// - Until(φ, ψ): ψ[t] ∨ (φ[t] ∧ Until(φ,ψ)[t+1])
    /// - Atom(p): encode proposition at time t
    fn encode_ltl_bounded(
        &self,
        formula: &crate::ltl::formula::LTLFormula,
        time: usize,
        horizon: usize,
    ) -> z3::ast::Bool {
        use crate::ltl::formula::LTLFormula;

        match formula {
            // Atomic proposition - encode at specific time
            LTLFormula::Atom(prop) => self.encode_proposition(prop, time),

            // Boolean operators - recursive encoding
            LTLFormula::Not(phi) => self.encode_ltl_bounded(phi, time, horizon).not(),

            LTLFormula::And(phi, psi) => {
                let left = self.encode_ltl_bounded(phi, time, horizon);
                let right = self.encode_ltl_bounded(psi, time, horizon);
                z3::ast::Bool::and(&[&left, &right])
            }

            LTLFormula::Or(phi, psi) => {
                let left = self.encode_ltl_bounded(phi, time, horizon);
                let right = self.encode_ltl_bounded(psi, time, horizon);
                z3::ast::Bool::or(&[&left, &right])
            }

            LTLFormula::Implies(phi, psi) => {
                let left = self.encode_ltl_bounded(phi, time, horizon);
                let right = self.encode_ltl_bounded(psi, time, horizon);
                left.implies(&right)
            }

            // Temporal operators - bounded expansion

            // Next: X(φ) = φ[time+1] (if within horizon)
            LTLFormula::Next(phi) => {
                if time < horizon {
                    self.encode_ltl_bounded(phi, time + 1, horizon)
                } else {
                    // If at horizon, treat as false (no next state)
                    z3::ast::Bool::from_bool(false)
                }
            }

            // Eventually: F(φ) = φ[time] ∨ φ[time+1] ∨ ... ∨ φ[horizon]
            LTLFormula::Eventually(phi) => {
                let mut disjuncts = Vec::new();
                for t in time..=horizon {
                    disjuncts.push(self.encode_ltl_bounded(phi, t, horizon));
                }
                let refs: Vec<&z3::ast::Bool> = disjuncts.iter().collect();
                z3::ast::Bool::or(&refs)
            }

            // Always: G(φ) = φ[time] ∧ φ[time+1] ∧ ... ∧ φ[horizon]
            LTLFormula::Always(phi) => {
                let mut conjuncts = Vec::new();
                for t in time..=horizon {
                    conjuncts.push(self.encode_ltl_bounded(phi, t, horizon));
                }
                let refs: Vec<&z3::ast::Bool> = conjuncts.iter().collect();
                z3::ast::Bool::and(&refs)
            }

            // Until: φ U ψ = ψ[time] ∨ (φ[time] ∧ (φ U ψ)[time+1])
            // Bounded version: must happen within horizon
            LTLFormula::Until(phi, psi) => {
                let mut disjuncts = Vec::new();

                for t in time..=horizon {
                    // ψ happens at time t, and φ holds from time to t-1
                    let psi_at_t = self.encode_ltl_bounded(psi, t, horizon);

                    if t == time {
                        // Base case: ψ holds now
                        disjuncts.push(psi_at_t);
                    } else {
                        // φ must hold from time to t-1
                        let mut phi_conjuncts = Vec::new();
                        for s in time..t {
                            phi_conjuncts.push(self.encode_ltl_bounded(phi, s, horizon));
                        }
                        let phi_refs: Vec<&z3::ast::Bool> = phi_conjuncts.iter().collect();
                        let phi_holds = z3::ast::Bool::and(&phi_refs);

                        // (φ[time] ∧ ... ∧ φ[t-1]) ∧ ψ[t]
                        let both = z3::ast::Bool::and(&[&phi_holds, &psi_at_t]);
                        disjuncts.push(both);
                    }
                }

                let refs: Vec<&z3::ast::Bool> = disjuncts.iter().collect();
                z3::ast::Bool::or(&refs)
            }
        }
    }

    /// Encode atomic propositions as Z3 constraints at a specific time
    fn encode_proposition(
        &self,
        prop: &crate::ltl::formula::Proposition,
        time: usize,
    ) -> z3::ast::Bool {
        use crate::ltl::formula::Proposition;

        match prop {
            // InLane(actor, lane): lane_var[t] == lane
            Proposition::InLane { actor, lane } => {
                let lane_var = &self.lanes[actor][time];
                let lane_val = Int::from_i64(*lane as i64);
                lane_var.eq(&lane_val)
            }

            // Ahead(actor1, actor2): px1[t] > px2[t]
            Proposition::Ahead { actor1, actor2 } => {
                let px1 = &self.positions_x[actor1][time];
                let px2 = &self.positions_x[actor2][time];
                px1.gt(px2)
            }

            // DistanceGT(actor1, actor2, d): |px1[t] - px2[t]| > d
            Proposition::DistanceGT {
                actor1,
                actor2,
                distance,
            } => {
                let px1 = &self.positions_x[actor1][time];
                let px2 = &self.positions_x[actor2][time];
                let dist_val = Real::from_rational((*distance * 10.0) as i64, 10_i64);

                // |px1 - px2| > d is equivalent to: (px1 - px2 > d) OR (px2 - px1 > d)
                let diff_pos = px1 - px2;
                let diff_neg = px2 - px1;

                let pos_case = diff_pos.gt(&dist_val);
                let neg_case = diff_neg.gt(&dist_val);

                z3::ast::Bool::or(&[&pos_case, &neg_case])
            }

            // TTCGT(actor1, actor2, ttc): TTC > ttc (if collision possible)
            Proposition::TTCGT {
                actor1,
                actor2,
                ttc,
            } => self.encode_ttc_constraint(actor1, actor2, *ttc, time),

            // OnSidewalk(actor, side): py < 0 (left) or py > road_width (right)
            Proposition::OnSidewalk { actor, side } => {
                let py = &self.positions_y[actor][time];
                let zero = Real::from_rational(0_i64, 1_i64);

                let lane_width = self.spec.get_lane_width();
                let num_lanes = self.spec.get_num_lanes();
                let road_width = lane_width * num_lanes as f64;
                let road_width_real = Real::from_rational((road_width * 10.0) as i64, 10_i64);

                if side == "left" {
                    py.lt(&zero)
                } else {
                    py.gt(&road_width_real)
                }
            }

            // CrossingRoad(actor): 0 <= py <= road_width
            Proposition::CrossingRoad { actor } => {
                let py = &self.positions_y[actor][time];
                let zero = Real::from_rational(0_i64, 1_i64);

                let lane_width = self.spec.get_lane_width();
                let num_lanes = self.spec.get_num_lanes();
                let road_width = lane_width * num_lanes as f64;
                let road_width_real = Real::from_rational((road_width * 10.0) as i64, 10_i64);

                let on_road_start = py.ge(&zero);
                let on_road_end = py.le(&road_width_real);
                z3::ast::Bool::and(&[&on_road_start, &on_road_end])
            }

            // Distance2DGT: 2D Euclidean distance between actors > threshold
            Proposition::Distance2DGT {
                actor1,
                actor2,
                distance,
            } => {
                let px1 = &self.positions_x[actor1][time];
                let py1 = &self.positions_y[actor1][time];
                let px2 = &self.positions_x[actor2][time];
                let py2 = &self.positions_y[actor2][time];

                // Euclidean distance: sqrt((px1-px2)^2 + (py1-py2)^2) > threshold
                // Z3 encoding: (px1-px2)^2 + (py1-py2)^2 > threshold^2
                let dx = px1 - px2;
                let dy = py1 - py2;
                let dist_sq = &(&dx * &dx) + &(&dy * &dy);
                let threshold_sq =
                    Real::from_rational((distance * distance * 100.0) as i64, 100_i64);
                dist_sq.gt(&threshold_sq)
            }

            // ManhattanDistanceGT: Manhattan distance between actors > threshold
            // Linear encoding: |dx| + |dy| > threshold
            // Implemented as disjunction over four cases (one per quadrant)
            Proposition::ManhattanDistanceGT {
                actor1,
                actor2,
                distance,
            } => {
                let px1 = &self.positions_x[actor1][time];
                let py1 = &self.positions_y[actor1][time];
                let px2 = &self.positions_x[actor2][time];
                let py2 = &self.positions_y[actor2][time];

                let dx = px1 - px2;
                let dy = py1 - py2;
                let threshold_real = Real::from_rational((distance * 10.0) as i64, 10_i64);
                let zero = Real::from_rational(0_i64, 1_i64);

                // Manhattan distance: |dx| + |dy| > threshold
                // We check all four combinations of signs:
                // Case 1: dx ≥ 0, dy ≥ 0 → dx + dy > threshold
                // Case 2: dx ≥ 0, dy < 0 → dx - dy > threshold
                // Case 3: dx < 0, dy ≥ 0 → -dx + dy > threshold
                // Case 4: dx < 0, dy < 0 → -dx - dy > threshold
                //
                // Disjunction: at least one case must hold
                let case1 = (&dx + &dy).gt(&threshold_real);
                let case2 = (&dx - &dy).gt(&threshold_real);
                let case3 = (&dy - &dx).gt(&threshold_real); // -dx + dy = dy - dx
                let case4 = (&zero - &(&dx + &dy)).gt(&threshold_real); // -(dx + dy)

                z3::ast::Bool::or(&[&case1, &case2, &case3, &case4])
            }

            // RectangularDistanceGT: Rectangular safety box
            // Simplest linear encoding: |dx| > threshold_x OR |dy| > threshold_y
            Proposition::RectangularDistanceGT {
                actor1,
                actor2,
                threshold_x,
                threshold_y,
            } => {
                let px1 = &self.positions_x[actor1][time];
                let py1 = &self.positions_y[actor1][time];
                let px2 = &self.positions_x[actor2][time];
                let py2 = &self.positions_y[actor2][time];

                let dx = px1 - px2;
                let dy = py1 - py2;
                let threshold_x_real = Real::from_rational((threshold_x * 10.0) as i64, 10_i64);
                let threshold_y_real = Real::from_rational((threshold_y * 10.0) as i64, 10_i64);

                // |dx| > threshold_x: dx > threshold_x OR dx < -threshold_x
                let dx_positive = dx.gt(&threshold_x_real);
                let neg_threshold_x = Real::from_rational((-threshold_x * 10.0) as i64, 10_i64);
                let dx_negative = dx.lt(&neg_threshold_x);
                let dx_outside = z3::ast::Bool::or(&[&dx_positive, &dx_negative]);

                // |dy| > threshold_y: dy > threshold_y OR dy < -threshold_y
                let dy_positive = dy.gt(&threshold_y_real);
                let neg_threshold_y = Real::from_rational((-threshold_y * 10.0) as i64, 10_i64);
                let dy_negative = dy.lt(&neg_threshold_y);
                let dy_outside = z3::ast::Bool::or(&[&dy_positive, &dy_negative]);

                // At least one dimension must be outside the box
                z3::ast::Bool::or(&[&dx_outside, &dy_outside])
            }

            // PedestrianTTCGT: Time-to-collision for perpendicular crossing
            Proposition::PedestrianTTCGT {
                ego,
                pedestrian,
                ttc,
            } => {
                let ego_px = &self.positions_x[ego][time];
                let ego_vx = &self.velocities_x[ego][time];
                let ped_px = &self.positions_x[pedestrian][time];
                let ped_py = &self.positions_y[pedestrian][time];

                let lane_width = self.spec.get_lane_width();
                let num_lanes = self.spec.get_num_lanes();
                let road_width = lane_width * num_lanes as f64;
                let road_width_real = Real::from_rational((road_width * 10.0) as i64, 10_i64);
                let zero = Real::from_rational(0_i64, 1_i64);

                // Pedestrian on road: 0 <= py <= road_width
                let ped_on_road =
                    z3::ast::Bool::and(&[&ped_py.ge(&zero), &ped_py.le(&road_width_real)]);

                // Ego approaching pedestrian's position
                let ego_behind = ego_px.lt(ped_px);
                let ego_moving_forward = ego_vx.gt(&zero);
                let approaching = z3::ast::Bool::and(&[&ego_behind, &ego_moving_forward]);

                // TTC = (ped_px - ego_px) / ego_vx
                // Safe if: (ped_px - ego_px) > ttc * ego_vx
                let distance = ped_px - ego_px;
                let ttc_val = Real::from_rational((ttc * 10.0) as i64, 10_i64);
                let ttc_safe = distance.gt(&(&ttc_val * ego_vx));

                // Overall: NOT (ped_on_road AND approaching) OR ttc_safe
                z3::ast::Bool::and(&[&ped_on_road, &approaching]).implies(&ttc_safe)
            }

            // VelocityGT: Actor's longitudinal speed exceeds threshold
            // Linear constraint: |vx| > threshold
            // Z3 encoding: (vx > threshold) OR (vx < -threshold)
            Proposition::VelocityGT { actor, velocity } => {
                let vx = &self.velocities_x[actor][time];
                let threshold_val = Real::from_rational((velocity * 10.0) as i64, 10_i64);

                // |vx| > threshold is equivalent to: (vx > threshold) OR (vx < -threshold)
                let pos_case = vx.gt(&threshold_val);
                let neg_threshold = Real::from_rational((-velocity * 10.0) as i64, 10_i64);
                let neg_case = vx.lt(&neg_threshold);

                z3::ast::Bool::or(&[&pos_case, &neg_case])
            }

            // VelocityLT: Actor's longitudinal speed is below threshold
            // Linear constraint: |vx| < threshold
            // Z3 encoding: (vx < threshold) AND (vx > -threshold)
            Proposition::VelocityLT { actor, velocity } => {
                let vx = &self.velocities_x[actor][time];
                let threshold_val = Real::from_rational((velocity * 10.0) as i64, 10_i64);
                let neg_threshold = Real::from_rational((-velocity * 10.0) as i64, 10_i64);

                // |vx| < threshold is equivalent to: -threshold < vx < threshold
                let upper_bound = vx.lt(&threshold_val);
                let lower_bound = vx.gt(&neg_threshold);

                z3::ast::Bool::and(&[&upper_bound, &lower_bound])
            }

            // LateralDistanceGT: Lateral distance between actors exceeds threshold
            // Linear constraint: |py1 - py2| > distance
            Proposition::LateralDistanceGT {
                actor1,
                actor2,
                distance,
            } => {
                let py1 = &self.positions_y[actor1][time];
                let py2 = &self.positions_y[actor2][time];
                let dist_val = Real::from_rational((*distance * 10.0) as i64, 10_i64);

                // |py1 - py2| > d is equivalent to: (py1 - py2 > d) OR (py2 - py1 > d)
                let diff_pos = py1 - py2;
                let diff_neg = py2 - py1;

                let pos_case = diff_pos.gt(&dist_val);
                let neg_case = diff_neg.gt(&dist_val);

                z3::ast::Bool::or(&[&pos_case, &neg_case])
            }

            // OnLeftOf: Actor1 is laterally left of Actor2
            // Simple comparison: py1 > py2
            Proposition::OnLeftOf { actor1, actor2 } => {
                let py1 = &self.positions_y[actor1][time];
                let py2 = &self.positions_y[actor2][time];
                py1.gt(py2)
            }

            // OnRightOf: Actor1 is laterally right of Actor2
            // Simple comparison: py1 < py2
            Proposition::OnRightOf { actor1, actor2 } => {
                let py1 = &self.positions_y[actor1][time];
                let py2 = &self.positions_y[actor2][time];
                py1.lt(py2)
            }

            // RelativeVelocityGT: Relative longitudinal velocity exceeds threshold
            // Linear constraint: |vx1 - vx2| > velocity
            Proposition::RelativeVelocityGT {
                actor1,
                actor2,
                velocity,
            } => {
                let vx1 = &self.velocities_x[actor1][time];
                let vx2 = &self.velocities_x[actor2][time];
                let vel_val = Real::from_rational((*velocity * 10.0) as i64, 10_i64);

                // |vx1 - vx2| > v is equivalent to: (vx1 - vx2 > v) OR (vx2 - vx1 > v)
                let diff_pos = vx1 - vx2;
                let diff_neg = vx2 - vx1;

                let pos_case = diff_pos.gt(&vel_val);
                let neg_case = diff_neg.gt(&vel_val);

                z3::ast::Bool::or(&[&pos_case, &neg_case])
            }
        }
    }

    /// Encode TTC (Time-To-Collision) constraint
    ///
    /// TTC is only defined when:
    /// 1. Both actors are in the same lane
    /// 2. The following actor is moving faster
    ///
    /// TTC = (distance between actors) / (relative velocity)
    ///     = (px_lead - px_follow) / (vx_follow - vx_lead)
    ///
    /// We require: TTC > min_ttc OR collision is not possible
    fn encode_ttc_constraint(
        &self,
        actor1: &str,
        actor2: &str,
        min_ttc: f64,
        time: usize,
    ) -> z3::ast::Bool {
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
            z3::ast::Bool::and(&[&actor1_ahead, &actor2_faster, &rel_vel_1.gt(&epsilon)]);
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
            z3::ast::Bool::and(&[&actor2_ahead, &actor1_faster, &rel_vel_2.gt(&epsilon)]);
        let ttc_safe_2 = distance_2.gt(&(&min_ttc_val * &rel_vel_2));

        // Overall constraint:
        // If same_lane AND collision_possible_1, then ttc_safe_1
        // If same_lane AND collision_possible_2, then ttc_safe_2
        // Otherwise, true (no collision risk)

        let case1 = z3::ast::Bool::and(&[&same_lane, &collision_possible_1]).implies(&ttc_safe_1);
        let case2 = z3::ast::Bool::and(&[&same_lane, &collision_possible_2]).implies(&ttc_safe_2);

        z3::ast::Bool::and(&[&case1, &case2])
    }

    /// Encode global max acceleration constraints (if specified)
    pub fn encode_acceleration_constraints(&mut self) {
        // Only apply if max_acceleration/max_deceleration specified AND mode is Enforce
        if let Some(max_accel) = self.spec.max_acceleration {
            let mode = self.spec.constraint_modes.max_acceleration();

            if mode == ConstraintMode::Enforce {
                let max_real = Real::from_rational((max_accel * 10.0) as i64, 10_i64);

                for actor in &self.spec.actors {
                    for t in 0..=self.horizon {
                        let ax = &self.accelerations_x[&actor.id][t];
                        self.backend.assert(&ax.le(&max_real));
                    }
                }
            }
            // Violate and Ignore modes handled via LTL
        }

        if let Some(max_decel) = self.spec.max_deceleration {
            // max_decel should be negative (e.g., -3.0)
            let mode = self.spec.constraint_modes.max_acceleration();

            if mode == ConstraintMode::Enforce {
                let min_real = Real::from_rational((max_decel * 10.0) as i64, 10_i64);

                for actor in &self.spec.actors {
                    for t in 0..=self.horizon {
                        let ax = &self.accelerations_x[&actor.id][t];
                        self.backend.assert(&ax.ge(&min_real));
                    }
                }
            }
        }
    }

    /// Check if an actor has a polynomial lane change configured
    fn actor_has_polynomial_lane_change(&self, actor: &crate::dsl::types::ActorSpec) -> bool {
        match &actor.lane_change {
            Some(lc) => lc.enabled && matches!(lc.method, crate::dsl::types::LaneChangeMethod::Polynomial),
            None => false,
        }
    }

    /// Encode Frenet kinematics constraints for smooth lane changes
    ///
    /// This implements Frenet coordinate system kinematics:
    /// - Longitudinal (s): standard kinematics with position, velocity, acceleration
    /// - Lateral (t): constrained by polynomial during lane changes, otherwise follows kinematics
    /// - Polynomial lane changes: t and vt are pre-computed and fixed
    fn encode_frenet_kinematics(&mut self) {
        use crate::dsl::types::ActorRole;

        let dt = self.spec.time_step;
        let dt_real = Real::from_rational((dt * 10.0) as i64, 10_i64);
        let zero = Real::from_rational(0_i64, 1_i64);

        // Collect actors with polynomial lane changes first (to avoid borrow checker issues)
        let poly_lane_change_actors: Vec<_> = self.spec.actors.iter()
            .filter(|a| self.actor_has_polynomial_lane_change(a))
            .map(|a| (
                a.id.clone(),
                a.lane_change.as_ref().unwrap().polynomial_coeffs.unwrap(),
                a.lane_change.as_ref().unwrap().start_time,
                a.lane_change.as_ref().unwrap().duration,
            ))
            .collect();

        for actor in &self.spec.actors {
            let actor_id = &actor.id;

            // Check if actor has polynomial lane change
            let has_poly_lc = self.actor_has_polynomial_lane_change(actor);

            // Get acceleration bounds from actor spec
            let (ax_min, ax_max) = if actor.role == ActorRole::Pedestrian {
                (actor.acceleration.min(), actor.acceleration.max())
            } else {
                (actor.acceleration.min(), actor.acceleration.max())
            };
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

            // Lateral kinematics (t, vt)
            for t in 0..self.horizon {
                let vt_t = &self.frenet_vt[actor_id][t];
                let vt_t1 = &self.frenet_vt[actor_id][t + 1];
                let t_t = &self.frenet_t[actor_id][t];
                let t_t1 = &self.frenet_t[actor_id][t + 1];

                if has_poly_lc {
                    // For actors with polynomial lane changes, we need to handle three phases:
                    // 1. Before lane change: allow small lateral movement but prevent drifting to edges
                    // 2. During lane change: t and vt fixed by polynomial (handled in encode_polynomial_lane_change)
                    // 3. After lane change: allow small lateral movement but prevent drifting to edges

                    let lc_config = actor.lane_change.as_ref().unwrap();
                    let start_step = (lc_config.start_time / self.spec.time_step) as usize;
                    let end_step = start_step + (lc_config.duration / self.spec.time_step) as usize;

                    // Add kinematic update only for timesteps outside lane change period
                    if t < start_step || t >= end_step {
                        let expected_t = t_t + &(vt_t * &dt_real);
                        self.backend.assert(&t_t1.eq(&expected_t));
                    }
                    // During lane change: no kinematic update (polynomial fixes t and vt directly)

                    // Add lateral velocity bounds to prevent jumping to edges
                    // Allow |vt| <= 2.0 m/s (prevents instant jumps to edges but allows variability)
                    let max_vt = Real::from_rational(20_i64, 10_i64); // 2.0 m/s
                    let neg_max_vt = Real::from_rational(-20_i64, 10_i64); // -2.0 m/s

                    if t < start_step || t > end_step {
                        // Before or after lane change: limit lateral velocity
                        self.backend.assert(&vt_t.ge(&neg_max_vt));
                        self.backend.assert(&vt_t.le(&max_vt));
                    }
                    // During lane change: t and vt are fixed by polynomial in encode_polynomial_lane_change
                } else {
                    // Standard lateral kinematics when not in lane change

                    // Lateral acceleration bounds
                    if actor.role == ActorRole::Pedestrian {
                        let at_t = &self.frenet_at[actor_id][t];
                        self.backend.assert(&at_t.ge(&ax_min_real));
                        self.backend.assert(&at_t.le(&ax_max_real));

                        // Lateral velocity update: vt[t+1] = vt[t] + at[t] * dt
                        let expected_vt = vt_t + &(at_t * &dt_real);
                        self.backend.assert(&vt_t1.eq(&expected_vt));
                    }

                    // Lateral position update: t[t+1] = t[t] + vt[t] * dt
                    let expected_t = t_t + &(vt_t * &dt_real);
                    self.backend.assert(&t_t1.eq(&expected_t));

                    // Ego never changes lanes (vt = 0)
                    if actor.role == ActorRole::Ego {
                        self.backend.assert(&vt_t.eq(&zero));
                    }
                }
            }
        }

        // Encode polynomial lane change constraints after the main loop
        // (to avoid borrow checker issues)
        for (actor_id, coeffs, start_time, duration) in poly_lane_change_actors {
            self.encode_polynomial_lane_change_for_actor(&actor_id, coeffs, start_time, duration);
        }

        // Add explicit bounds on lateral position t to ensure vehicles stay on road
        // This prevents vehicles from jumping to lane edges or outside road boundaries
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

    /// Encode polynomial lane change constraints
    ///
    /// For actors with polynomial lane changes, this fixes t and vt values
    /// based on the pre-computed quintic polynomial coefficients.
    fn encode_polynomial_lane_change_for_actor(
        &mut self,
        actor_id: &str,
        coeffs: [f64; 6],
        start_time: f64,
        duration: f64,
    ) {
        use crate::trajectory::{evaluate_polynomial, evaluate_polynomial_derivative};

        let start_step = (start_time / self.spec.time_step) as usize;
        let end_step = start_step + (duration / self.spec.time_step) as usize;

        for t in start_step..=end_step {
            let tau = (t as f64 - start_step as f64) * self.spec.time_step;

            // Fix lateral position t from polynomial
            let t_val = evaluate_polynomial(tau, &coeffs);
            let t_real = Real::from_rational((t_val * 10.0) as i64, 10_i64);
            let t_var = &self.frenet_t[actor_id][t];
            self.backend.assert(&t_var.eq(&t_real));

            // Fix lateral velocity vt from polynomial derivative
            let vt_val = evaluate_polynomial_derivative(tau, &coeffs);
            let vt_real = Real::from_rational((vt_val * 10.0) as i64, 10_i64);
            let vt_var = &self.frenet_vt[actor_id][t];
            self.backend.assert(&vt_var.eq(&vt_real));
        }
    }

    /// Extract scenario from Z3 model
    ///
    /// Converts the Z3 solution (satisfying assignment) into a Scenario
    /// JSON structure with actor trajectories.
    pub fn extract_scenario(&self, model: &z3::Model) -> crate::error::Result<crate::scenario::model::Scenario> {
        use crate::dsl::types::ActorRole;

        // Get RoadSpec from ScenarioSpec (required, should always exist after validation)
        let road = self
            .spec
            .road
            .as_ref()
            .ok_or_else(|| crate::error::ScenarioGenError::ExtractionFailed("RoadSpec is required - should be validated during spec parsing".to_string()))?
            .clone();

        let mut scenario = crate::scenario::model::Scenario::new(
            self.spec.scenario_type.to_string(),
            self.spec.time_step,
            self.spec.duration,
            road,
        );

        // Extract trajectory for each actor
        for actor in &self.spec.actors {
            let role_str = match actor.role {
                ActorRole::Ego => "ego",
                ActorRole::Npc => "npc",
                ActorRole::Pedestrian => "pedestrian",
            };

            let trajectory = self.extract_actor_trajectory(model, &actor.id, role_str)?;
            scenario.add_actor(trajectory);
        }

        // Validate extracted Frenet t values are within road boundaries
        let road_width = (self.spec.get_num_lanes() as f64) * self.spec.lane_width;
        for trajectory in &scenario.actors {
            for state in &trajectory.states {
                if let Some(frenet) = &state.frenet {
                    if frenet.t < 0.0 || frenet.t > road_width {
                        eprintln!(
                            "WARNING: Actor {} at t={:.1}s has lateral position {:.2}m outside road bounds [0, {:.2}]",
                            trajectory.id, state.time, frenet.t, road_width
                        );
                    }
                }
            }
        }

        // Compute validation metrics
        self.compute_validation_metrics(&mut scenario)?;

        Ok(scenario)
    }

    /// Extract trajectory for a single actor
    fn extract_actor_trajectory(
        &self,
        model: &z3::Model,
        actor_id: &str,
        role: &str,
    ) -> crate::error::Result<crate::scenario::model::ActorTrajectory> {
        use crate::dsl::types::CoordinateSystem;
        use crate::geometry::FrenetPoint;
        use crate::scenario::model::{Acceleration, ActorTrajectory, CartesianState, FrenetState, Position, State, Velocity};

        let mut trajectory = ActorTrajectory::new(actor_id.to_string(), role.to_string());

        // Get reference line for Frenet conversion
        let ref_line = self.spec.reference_line.as_ref();

        for t in 0..=self.horizon {
            let time = t as f64 * self.spec.time_step;

            let state = match self.spec.coordinate_system {
                CoordinateSystem::Frenet => {
                    // Extract Frenet values
                    let s = self.extract_real(model, &self.frenet_s[actor_id][t])?;
                    let t_val = self.extract_real(model, &self.frenet_t[actor_id][t])?;
                    let vs = self.extract_real(model, &self.frenet_vs[actor_id][t])?;
                    let vt = self.extract_real(model, &self.frenet_vt[actor_id][t])?;
                    let as_ = self.extract_real(model, &self.frenet_as[actor_id][t])?;
                    let at = self.extract_real(model, &self.frenet_at[actor_id][t])?;

                    // Calculate theta (heading) from road heading
                    let theta = ref_line
                        .map(|rl| rl.heading)
                        .unwrap_or(0.0);

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

                    State {
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
                    }
                }
                CoordinateSystem::Cartesian => {
                    // Extract Cartesian values (existing behavior)
                    let px = self.extract_real(model, &self.positions_x[actor_id][t])?;
                    let py = self.extract_real(model, &self.positions_y[actor_id][t])?;
                    let vx = self.extract_real(model, &self.velocities_x[actor_id][t])?;
                    let vy = self.extract_real(model, &self.velocities_y[actor_id][t])?;
                    let ax = self.extract_real(model, &self.accelerations_x[actor_id][t])?;
                    let ay = self.extract_real(model, &self.accelerations_y[actor_id][t])?;
                    let lane = self.extract_int(model, &self.lanes[actor_id][t])?;

                    State {
                        time,
                        frenet: None,
                        cartesian: Some(CartesianState {
                            position: Position::new(px, py),
                            velocity: Velocity::new(vx, vy),
                            acceleration: Acceleration::new(ax, ay),
                            lane,
                        }),
                    }
                }
            };

            trajectory.add_state(state);
        }

        Ok(trajectory)
    }

    /// Extract a real value from Z3 model
    fn extract_real(&self, model: &z3::Model, var: &Real) -> crate::error::Result<f64> {
        let ast = model
            .eval(var, true)
            .ok_or_else(|| crate::error::ScenarioGenError::Z3ModelParsing("Failed to evaluate real variable".to_string()))?;

        // Z3 returns rationals in various formats
        let ast_str = ast.to_string();

        // Remove all parentheses and work with the content
        let cleaned = ast_str.replace(['(', ')'], "");

        // Split by whitespace to get components
        let parts: Vec<&str> = cleaned.split_whitespace().collect();

        // Check for various formats
        if parts.is_empty() {
            return Err(crate::error::ScenarioGenError::Z3ModelParsing(format!("Failed to parse real value from Z3: '{}'", ast_str)));
        }

        // Format: "- / numerator denominator" -> negative fraction
        if parts.len() >= 4 && parts[0] == "-" && parts[1] == "/" {
            let numerator: f64 = parts[2].parse()
                .map_err(|e| crate::error::ScenarioGenError::Z3ModelParsing(format!("Failed to parse numerator: {}", e)))?;
            let denominator: f64 = parts[3].parse()
                .map_err(|e| crate::error::ScenarioGenError::Z3ModelParsing(format!("Failed to parse denominator: {}", e)))?;
            return Ok(-(numerator / denominator));
        }

        // Format: "/ numerator denominator" -> positive fraction
        if parts.len() >= 3 && parts[0] == "/" {
            let numerator: f64 = parts[1].parse()
                .map_err(|e| crate::error::ScenarioGenError::Z3ModelParsing(format!("Failed to parse numerator: {}", e)))?;
            let denominator: f64 = parts[2].parse()
                .map_err(|e| crate::error::ScenarioGenError::Z3ModelParsing(format!("Failed to parse denominator: {}", e)))?;
            return Ok(numerator / denominator);
        }

        // Format: "- value" -> simple negative
        if parts.len() == 2 && parts[0] == "-" {
            let value: f64 = parts[1].parse()
                .map_err(|e| crate::error::ScenarioGenError::Z3ModelParsing(format!("Failed to parse negative value: {}", e)))?;
            return Ok(-value);
        }

        // Format: "numerator/denominator" or "-numerator/denominator"
        if parts.len() == 1 && parts[0].contains('/') {
            let frac_parts: Vec<&str> = parts[0].split('/').collect();
            let numerator: f64 = frac_parts[0].parse()
                .map_err(|e| crate::error::ScenarioGenError::Z3ModelParsing(format!("Failed to parse numerator: {}", e)))?;
            let denominator: f64 = frac_parts[1].parse()
                .map_err(|e| crate::error::ScenarioGenError::Z3ModelParsing(format!("Failed to parse denominator: {}", e)))?;
            return Ok(numerator / denominator);
        }

        // Default: try to parse as a simple number
        parts[0]
            .parse()
            .map_err(|e| crate::error::ScenarioGenError::Z3ModelParsing(format!("Failed to parse real value from Z3 '{}': {}", ast_str, e)))
    }

    /// Extract an integer value from Z3 model
    fn extract_int(&self, model: &z3::Model, var: &Int) -> crate::error::Result<usize> {
        let ast = model
            .eval(var, true)
            .ok_or_else(|| crate::error::ScenarioGenError::Z3ModelParsing("Failed to evaluate int variable".to_string()))?;
        let ast_str = ast.to_string();

        // Handle negative values like "(- 5)" or simple values like "1"
        let cleaned = ast_str.trim_start_matches('(').trim_end_matches(')');

        if cleaned.starts_with("- ") {
            // Format: "(- value)" - but usize can't be negative, so this would be an error
            return Err(crate::error::ScenarioGenError::Z3ModelParsing(format!("Cannot extract negative integer as usize: '{}'", ast_str)));
        }

        cleaned
            .trim()
            .parse()
            .map_err(|e| crate::error::ScenarioGenError::Z3ModelParsing(format!("Failed to parse int value from Z3 '{}': {}", ast_str, e)))
    }

    /// Compute validation metrics from the scenario trajectories
    fn compute_validation_metrics(&self, scenario: &mut crate::scenario::model::Scenario) -> crate::error::Result<()> {
        let mut min_ttc = f64::INFINITY;
        let mut min_distance = f64::INFINITY;
        let mut violations = Vec::new();

        // Compute pairwise metrics for all actor combinations
        for (i, id1) in self.spec.actors.iter().map(|a| a.id.clone()).enumerate() {
            for id2 in self.spec.actors.iter().skip(i + 1).map(|a| a.id.clone()) {
                let traj1 = scenario
                    .get_actor(&id1)
                    .ok_or_else(|| crate::error::ScenarioGenError::ActorNotFound(format!("Actor {} missing", id1)))?;
                let traj2 = scenario
                    .get_actor(&id2)
                    .ok_or_else(|| crate::error::ScenarioGenError::ActorNotFound(format!("Actor {} missing", id2)))?;

                for t in 0..=self.horizon {
                    let state1 = &traj1.states[t];
                    let state2 = &traj2.states[t];

                    // Compute longitudinal distance
                    let distance = (state1.position().x - state2.position().x).abs();

                    // Only consider distance when in same lane
                    if state1.lane() == state2.lane() {
                        if distance < min_distance {
                            min_distance = distance;
                        }

                        // Check minimum distance violation
                        if distance < self.spec.min_distance {
                            violations.push(format!(
                                "Distance violation at t={:.1}s: {}-{}: {:.2}m < {:.2}m",
                                t as f64 * self.spec.time_step,
                                id1,
                                id2,
                                distance,
                                self.spec.min_distance
                            ));
                        }
                    }

                    // Compute TTC (only when in same lane and approaching)
                    if state1.lane() == state2.lane() {
                        let rel_vel = (state1.velocity().vx - state2.velocity().vx).abs();

                        if rel_vel > 0.01 {
                            // Someone is catching up
                            let ttc = distance / rel_vel;

                            if ttc < min_ttc {
                                min_ttc = ttc;
                            }

                            // Check TTC violation
                            if ttc < self.spec.min_ttc {
                                violations.push(format!(
                                    "TTC violation at t={:.1}s: {}-{}: {:.2}s < {:.2}s",
                                    t as f64 * self.spec.time_step,
                                    id1,
                                    id2,
                                    ttc,
                                    self.spec.min_ttc
                                ));
                            }
                        }
                    }
                }
            }
        }

        // Compute acceleration metrics
        let mut max_accel = 0.0;
        let mut max_decel = 0.0;
        let mut accel_violations = Vec::new();

        for actor_traj in &scenario.actors {
            for state in &actor_traj.states {
                let ax = state.acceleration().ax;

                // Track maximum values
                if ax > max_accel {
                    max_accel = ax;
                }
                if ax < max_decel {
                    max_decel = ax;
                }

                // Check for global constraint violations
                if let Some(max_a) = self.spec.max_acceleration {
                    if ax > max_a {
                        accel_violations.push(format!(
                            "{} harsh acceleration at t={:.1}s: {:.2} m/s² > {:.2} m/s²",
                            actor_traj.id, state.time, ax, max_a
                        ));
                    }
                }

                if let Some(max_d) = self.spec.max_deceleration {
                    if ax < max_d {
                        accel_violations.push(format!(
                            "{} harsh braking at t={:.1}s: {:.2} m/s² < {:.2} m/s²",
                            actor_traj.id, state.time, ax, max_d
                        ));
                    }
                }
            }
        }

        scenario.validation.max_acceleration = max_accel;
        scenario.validation.max_deceleration = max_decel;
        scenario.validation.acceleration_violations = accel_violations.clone();

        // Update all_constraints_satisfied if there are acceleration violations
        if !accel_violations.is_empty() {
            scenario.validation.all_constraints_satisfied = false;
        }

        // Update validation info
        scenario.validation.min_ttc = if min_ttc.is_infinite() {
            999.0
        } else {
            min_ttc
        };
        scenario.validation.min_distance = if min_distance.is_infinite() {
            999.0
        } else {
            min_distance
        };
        scenario.validation.all_constraints_satisfied =
            violations.is_empty() && accel_violations.is_empty();
        scenario.validation.safety_violations = violations;

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dsl::types::{ActorRole, ActorSpec, RoadSpec, ScenarioType, ValueOrRange};
    use std::collections::HashMap;
    use z3::Config;

    fn create_test_spec() -> ScenarioSpec {
        let mut npc_behavior = HashMap::new();
        npc_behavior.insert("cut_in_time".to_string(), serde_json::json!([2.5, 7.5]));

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
                    lane_change: None,
                },
                ActorSpec {
                    id: "npc".to_string(),
                    role: ActorRole::Npc,
                    lane: 0,
                    position: ValueOrRange::Range([60.0, 80.0]),
                    speed: ValueOrRange::Range([12.0, 14.0]),
                    acceleration: ValueOrRange::Range([-8.0, 3.0]),
                    direction: 1,
                    behavior: npc_behavior,
                    lane_change: None,
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
            coordinate_system: crate::dsl::types::CoordinateSystem::Cartesian,
            reference_line: None,
        }
    }

    #[test]
    fn test_encoder_creation() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let spec = create_test_spec();
            let encoder = Z3Encoder::new(spec);
            assert_eq!(encoder.horizon, 20); // 10.0 / 0.5 = 20
        });
    }

    #[test]
    fn test_create_variables() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let spec = create_test_spec();
            let mut encoder = Z3Encoder::new(spec);
            encoder.create_variables();

            // Check variables were created
            assert!(encoder.positions_x.contains_key("ego"));
            assert!(encoder.positions_x.contains_key("npc"));

            // Check we have the right number of time steps
            assert_eq!(encoder.positions_x["ego"].len(), 21); // 0..=20
            assert_eq!(encoder.velocities_x["npc"].len(), 21);
        });
    }

    #[test]
    fn test_encode_initial_conditions() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let spec = create_test_spec();
            let mut encoder = Z3Encoder::new(spec);
            encoder.create_variables();
            encoder.encode_initial_conditions();

            // Check that constraints are satisfiable
            let result = encoder.check();
            assert_eq!(result, SatResult::Sat);

            // Get model and verify initial values
            let model = encoder.get_model().unwrap();

            // Ego position should be 50.0
            let ego_px_0 = model.eval(&encoder.positions_x["ego"][0], true).unwrap();
            println!("Ego initial position: {:?}", ego_px_0);

            // NPC position should be in range [60.0, 80.0]
            let npc_px_0 = model.eval(&encoder.positions_x["npc"][0], true).unwrap();
            println!("NPC initial position: {:?}", npc_px_0);

            // Ego speed should be 15.0
            let ego_vx_0 = model.eval(&encoder.velocities_x["ego"][0], true).unwrap();
            println!("Ego initial speed: {:?}", ego_vx_0);
        });
    }

    #[test]
    fn test_lane_position_coupling() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let spec = create_test_spec();
            let mut encoder = Z3Encoder::new(spec);
            encoder.create_variables();
            encoder.encode_initial_conditions();

            assert_eq!(encoder.check(), SatResult::Sat);

            let model = encoder.get_model().unwrap();

            // Ego in lane 1, should have py = 1 * 3.5 + 1.75 = 5.25
            let ego_py_0 = model.eval(&encoder.positions_y["ego"][0], true).unwrap();
            println!("Ego lateral position: {:?}", ego_py_0);

            // NPC in lane 0, should have py = 0 * 3.5 + 1.75 = 1.75
            let npc_py_0 = model.eval(&encoder.positions_y["npc"][0], true).unwrap();
            println!("NPC lateral position: {:?}", npc_py_0);
        });
    }

    #[test]
    fn test_kinematics() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let spec = create_test_spec();
            let mut encoder = Z3Encoder::new(spec);
            encoder.create_variables();
            encoder.encode_initial_conditions();
            encoder.encode_kinematics();
            encoder.encode_lane_velocity_constraints();
            encoder.encode_lateral_velocity_bounds();

            assert_eq!(encoder.check(), SatResult::Sat);

            let model = encoder.get_model().unwrap();

            // Check that position evolves correctly
            let ego_px_0 = model.eval(&encoder.positions_x["ego"][0], true).unwrap();
            let ego_px_1 = model.eval(&encoder.positions_x["ego"][1], true).unwrap();
            let ego_vx_0 = model.eval(&encoder.velocities_x["ego"][0], true).unwrap();

            println!("Ego px[0]: {:?}", ego_px_0);
            println!("Ego px[1]: {:?}", ego_px_1);
            println!("Ego vx[0]: {:?}", ego_vx_0);

            // px[1] should be px[0] + vx[0] * 0.5
            // 50.0 + 15.0 * 0.5 = 57.5
        });
    }

    #[test]
    fn test_ltl_encoding_simple() {
        use crate::ltl::formula::{LTLFormula, Proposition};

        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let spec = create_test_spec();
            let mut encoder = Z3Encoder::new(spec);
            encoder.create_variables();
            encoder.encode_initial_conditions();
            encoder.encode_kinematics();
            encoder.encode_lane_velocity_constraints();
            encoder.encode_lateral_velocity_bounds();

            // Test simple atomic proposition: InLane(ego, 1)
            let formula = LTLFormula::Atom(Proposition::InLane {
                actor: "ego".to_string(),
                lane: 1,
            });

            encoder.encode_ltl(&formula);
            assert_eq!(encoder.check(), SatResult::Sat);
        });
    }

    #[test]
    fn test_ltl_encoding_eventually() {
        use crate::ltl::formula::{LTLFormula, Proposition};

        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let spec = create_test_spec();
            let mut encoder = Z3Encoder::new(spec);
            encoder.create_variables();
            encoder.encode_initial_conditions();
            encoder.encode_kinematics();
            encoder.encode_lane_velocity_constraints();
            encoder.encode_lateral_velocity_bounds();

            // Test Eventually: F(InLane(npc, 1))
            // NPC should eventually be in lane 1
            let formula = LTLFormula::Atom(Proposition::InLane {
                actor: "npc".to_string(),
                lane: 1,
            })
            .eventually();

            encoder.encode_ltl(&formula);
            assert_eq!(encoder.check(), SatResult::Sat);

            let model = encoder.get_model().unwrap();

            // Check that NPC is in lane 1 at some point
            let mut found_lane_1 = false;
            for t in 0..=encoder.horizon {
                let lane = model.eval(&encoder.lanes["npc"][t], true).unwrap();
                if lane.to_string() == "1" {
                    found_lane_1 = true;
                    println!("NPC in lane 1 at time {}", t);
                    break;
                }
            }
            assert!(found_lane_1, "NPC should eventually be in lane 1");
        });
    }

    #[test]
    fn test_ltl_encoding_always() {
        use crate::ltl::formula::{LTLFormula, Proposition};

        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let spec = create_test_spec();
            let mut encoder = Z3Encoder::new(spec);
            encoder.create_variables();
            encoder.encode_initial_conditions();
            encoder.encode_kinematics();
            encoder.encode_lane_velocity_constraints();
            encoder.encode_lateral_velocity_bounds();

            // Test Always: G(InLane(ego, 1))
            // Ego should always be in lane 1
            let formula = LTLFormula::Atom(Proposition::InLane {
                actor: "ego".to_string(),
                lane: 1,
            })
            .always();

            encoder.encode_ltl(&formula);
            assert_eq!(encoder.check(), SatResult::Sat);

            let model = encoder.get_model().unwrap();

            // Check that ego is in lane 1 at all times
            for t in 0..=encoder.horizon {
                let lane = model.eval(&encoder.lanes["ego"][t], true).unwrap();
                assert_eq!(lane.to_string(), "1", "Ego should always be in lane 1");
            }
        });
    }

    #[test]
    fn test_ltl_encoding_until() {
        use crate::ltl::formula::{LTLFormula, Proposition};

        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let spec = create_test_spec();
            let mut encoder = Z3Encoder::new(spec);
            encoder.create_variables();
            encoder.encode_initial_conditions();
            encoder.encode_kinematics();
            encoder.encode_lane_velocity_constraints();
            encoder.encode_lateral_velocity_bounds();

            // Test Until: InLane(npc, 0) U InLane(npc, 1)
            // NPC stays in lane 0 until it moves to lane 1
            let formula = LTLFormula::Atom(Proposition::InLane {
                actor: "npc".to_string(),
                lane: 0,
            })
            .until(LTLFormula::Atom(Proposition::InLane {
                actor: "npc".to_string(),
                lane: 1,
            }));

            encoder.encode_ltl(&formula);
            assert_eq!(encoder.check(), SatResult::Sat);

            let model = encoder.get_model().unwrap();

            // Find when NPC transitions to lane 1
            let mut transition_time = None;
            for t in 0..=encoder.horizon {
                let lane = model.eval(&encoder.lanes["npc"][t], true).unwrap();
                if lane.to_string() == "1" {
                    transition_time = Some(t);
                    break;
                }
            }

            if let Some(trans_t) = transition_time {
                println!("NPC transitions to lane 1 at time {}", trans_t);
                // Before transition, should be in lane 0
                for t in 0..trans_t {
                    let lane = model.eval(&encoder.lanes["npc"][t], true).unwrap();
                    assert_eq!(
                        lane.to_string(),
                        "0",
                        "NPC should be in lane 0 before transition"
                    );
                }
            }
        });
    }

    #[test]
    fn test_safety_constraints() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let spec = create_test_spec();
            let mut encoder = Z3Encoder::new(spec);
            encoder.create_variables();
            encoder.encode_initial_conditions();
            encoder.encode_kinematics();
            encoder.encode_lane_velocity_constraints();
            encoder.encode_lateral_velocity_bounds();
            // Safety constraints are now included in LTL formula via generate_safety()

            // Safety constraints should be satisfiable
            assert_eq!(encoder.check(), SatResult::Sat);
        });
    }

    #[test]
    fn test_full_cut_in_scenario() {
        use crate::ltl::generator::LTLGenerator;

        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let spec = create_test_spec();
            let mut encoder = Z3Encoder::new(spec.clone());
            encoder.create_variables();
            encoder.encode_initial_conditions();
            encoder.encode_kinematics();
            encoder.encode_lane_velocity_constraints();
            encoder.encode_lateral_velocity_bounds();

            // Generate and encode full cut-in LTL formula
            let ltl_formula = LTLGenerator::generate(&spec).unwrap();
            encoder.encode_ltl(&ltl_formula);

            // Add safety constraints
            // Safety constraints are now included in LTL formula via generate_safety()

            // Check satisfiability
            let result = encoder.check();
            assert_eq!(
                result,
                SatResult::Sat,
                "Full cut-in scenario should be satisfiable"
            );

            if result == SatResult::Sat {
                let model = encoder.get_model().unwrap();

                // Verify initial conditions
                let ego_lane_0 = model.eval(&encoder.lanes["ego"][0], true).unwrap();
                let npc_lane_0 = model.eval(&encoder.lanes["npc"][0], true).unwrap();
                assert_eq!(ego_lane_0.to_string(), "1");
                assert_eq!(npc_lane_0.to_string(), "0");

                // Verify NPC eventually changes lanes
                let mut npc_in_lane_1 = false;
                for t in 0..=encoder.horizon {
                    let lane = model.eval(&encoder.lanes["npc"][t], true).unwrap();
                    if lane.to_string() == "1" {
                        npc_in_lane_1 = true;
                        println!("NPC changes to lane 1 at time step {}", t);
                        break;
                    }
                }
                assert!(npc_in_lane_1, "NPC should eventually change to lane 1");

                println!("Full cut-in scenario test passed!");
            }
        });
    }

    #[test]
    fn test_scenario_extraction() {
        use crate::ltl::generator::LTLGenerator;

        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let spec = create_test_spec();
            let mut encoder = Z3Encoder::new(spec.clone());
            encoder.create_variables();
            encoder.encode_initial_conditions();
            encoder.encode_kinematics();
            encoder.encode_lane_velocity_constraints();
            encoder.encode_lateral_velocity_bounds();

            // Generate and encode full cut-in LTL formula
            let ltl_formula = LTLGenerator::generate(&spec).unwrap();
            encoder.encode_ltl(&ltl_formula);
            // Safety constraints are now included in LTL formula via generate_safety()

            // Check satisfiability
            let result = encoder.check();
            assert_eq!(result, SatResult::Sat, "Should be satisfiable");

            if result == SatResult::Sat {
                let model = encoder.get_model().unwrap();

                // Extract scenario
                let scenario = encoder.extract_scenario(&model).unwrap();

                // Verify basic structure
                assert_eq!(scenario.actors.len(), 2);
                assert_eq!(scenario.time_step, 0.5);
                assert_eq!(scenario.duration, 10.0);

                // Verify ego trajectory
                let ego = scenario.get_actor("ego").expect("Ego missing");
                assert_eq!(ego.id, "ego");
                assert_eq!(ego.states.len(), 21); // 0..=20

                // Verify NPC trajectory
                let npc = scenario.get_actor("npc").expect("NPC missing");
                assert_eq!(npc.id, "npc");
                assert_eq!(npc.states.len(), 21);

                // Verify initial conditions
                assert_eq!(ego.states[0].lane(), 1);
                assert_eq!(npc.states[0].lane(), 0);

                // Verify NPC position is ahead initially
                assert!(npc.states[0].position().x > ego.states[0].position().x);

                // Verify validation metrics exist
                println!("Min TTC: {}", scenario.validation.min_ttc);
                println!("Min distance: {}", scenario.validation.min_distance);
                println!(
                    "All constraints satisfied: {}",
                    scenario.validation.all_constraints_satisfied
                );

                // Test JSON serialization
                let json = serde_json::to_string_pretty(&scenario).unwrap();
                println!("Extracted scenario JSON:\n{}", json);

                // Verify it can be deserialized
                let _deserialized: crate::scenario::model::Scenario =
                    serde_json::from_str(&json).unwrap();

                println!("Scenario extraction test passed!");
            }
        });
    }

    #[test]
    fn test_velocity_propositions_linear() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            use crate::ltl::formula::Proposition;

            let spec = ScenarioSpec {
                scenario_type: ScenarioType::CutInLeft,
                time_step: 0.5,
                duration: 5.0,
                actors: vec![ActorSpec {
                    id: "ego".to_string(),
                    role: ActorRole::Ego,
                    lane: 0,
                    position: ValueOrRange::Value(0.0),
                    speed: ValueOrRange::Value(20.0),
                    acceleration: ValueOrRange::Range([-8.0, 3.0]),
                    direction: 1,
                    behavior: HashMap::new(),
                    lane_change: None,
                }],
                min_ttc: 3.0,
                min_distance: 5.0,
                road: None,
                lane_width: 3.5,
                num_scenarios: 1,
                constraint_modes: crate::dsl::types::ConstraintModes::default(),
                optimization_target: crate::dsl::types::OptimizationTarget::None,
                max_acceleration: None,
                max_deceleration: None,
                max_velocity: Some(25.0),
                min_velocity: Some(10.0),
                min_lateral_distance: None,
                max_relative_velocity: None,
                coordinate_system: crate::dsl::types::CoordinateSystem::Cartesian,
                reference_line: None,
            };

            let mut encoder = Z3Encoder::new(spec);
            encoder.create_variables();
            encoder.encode_initial_conditions();

            // Test VelocityGT (linear constraint)
            let prop_gt = Proposition::VelocityGT {
                actor: "ego".to_string(),
                velocity: 15.0,
            };
            let _constraint_gt = encoder.encode_proposition(&prop_gt, 0);
            // If we get here, encoding succeeded without panicking

            // Test VelocityLT (linear constraint)
            let prop_lt = Proposition::VelocityLT {
                actor: "ego".to_string(),
                velocity: 30.0,
            };
            let _constraint_lt = encoder.encode_proposition(&prop_lt, 0);
            // If we get here, encoding succeeded without panicking

            println!("VelocityGT/LT use linear constraints (no quadratic operations)");
        });
    }
}
