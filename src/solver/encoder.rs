//! Z3 constraint encoder

use std::collections::HashMap;
use z3::ast::{Ast, Int, Real};
use z3::{Context, SatResult, Solver};

use crate::dsl::types::{ConstraintMode, ScenarioSpec};

/// Z3 SMT encoder for scenario constraints
///
/// Lifetime 'ctx is the Z3 context lifetime - all Z3 AST nodes must live
/// as long as the context.
pub struct Z3Encoder<'ctx> {
    /// Z3 context (must outlive all AST nodes)
    ctx: &'ctx Context,

    /// Z3 solver instance
    pub(crate) solver: Solver<'ctx>,

    /// Original scenario specification
    spec: ScenarioSpec,

    /// Number of time steps in the scenario
    horizon: usize,

    // Variable maps: actor_id -> Vec<variable> (one per time step)
    /// Longitudinal positions (m)
    pub(crate) positions_x: HashMap<String, Vec<Real<'ctx>>>,

    /// Lateral positions (m)
    positions_y: HashMap<String, Vec<Real<'ctx>>>,

    /// Longitudinal velocities (m/s)
    pub(crate) velocities_x: HashMap<String, Vec<Real<'ctx>>>,

    /// Lateral velocities (m/s)
    velocities_y: HashMap<String, Vec<Real<'ctx>>>,

    /// Lane numbers (integer)
    lanes: HashMap<String, Vec<Int<'ctx>>>,

    /// Longitudinal accelerations (m/s²)
    accelerations_x: HashMap<String, Vec<Real<'ctx>>>,

    /// Lateral accelerations (m/s²)
    accelerations_y: HashMap<String, Vec<Real<'ctx>>>,
}

impl<'ctx> Z3Encoder<'ctx> {
    /// Create a new Z3 encoder for the given specification
    pub fn new(ctx: &'ctx Context, spec: ScenarioSpec) -> Self {
        let solver = Solver::new(ctx);
        let horizon = spec.num_time_steps();

        Self {
            ctx,
            solver,
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
        for actor in &self.spec.actors {
            let actor_id = &actor.id;

            let mut px_vars = Vec::new();
            let mut py_vars = Vec::new();
            let mut vx_vars = Vec::new();
            let mut vy_vars = Vec::new();
            let mut lane_vars = Vec::new();
            let mut ax_vars = Vec::new();
            let mut ay_vars = Vec::new();

            // Create variables for each time step
            for t in 0..=self.horizon {
                px_vars.push(Real::new_const(self.ctx, format!("{}_px_{}", actor_id, t)));
                py_vars.push(Real::new_const(self.ctx, format!("{}_py_{}", actor_id, t)));
                vx_vars.push(Real::new_const(self.ctx, format!("{}_vx_{}", actor_id, t)));
                vy_vars.push(Real::new_const(self.ctx, format!("{}_vy_{}", actor_id, t)));
                lane_vars.push(Int::new_const(self.ctx, format!("{}_lane_{}", actor_id, t)));
                ax_vars.push(Real::new_const(self.ctx, format!("{}_ax_{}", actor_id, t)));
                ay_vars.push(Real::new_const(self.ctx, format!("{}_ay_{}", actor_id, t)));
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

    /// Encode initial conditions from the DSL specification
    pub fn encode_initial_conditions(&mut self) {
        use crate::dsl::types::ActorRole;

        // Collect all actor data upfront to avoid borrow checker issues
        let actor_data: Vec<_> = self.spec.actors.iter().map(|actor| {
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
            )
        }).collect();

        for (actor_id, lane, pos_min, pos_max, speed_min, speed_max, acc_min, acc_max, role) in actor_data {
            // Call existing encoding method
            self.encode_actor_initial_state(
                &actor_id,
                lane,
                pos_min,
                pos_max,
                speed_min,
                speed_max,
                acc_min,
                acc_max,
            );

            // Initial lateral position matches lane center
            self.encode_lane_position_coupling_at_time(&actor_id, 0);

            // Ego never changes lanes (vy = 0)
            if role == ActorRole::Ego {
                let zero = Real::from_real(self.ctx, 0, 1);
                let vy_0 = &self.velocities_y[&actor_id][0];
                self.solver.assert(&vy_0._eq(&zero));
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
    ) {
        // Lane at t=0
        let lane_var = &self.lanes[actor_id][0];
        let lane_val = Int::from_i64(self.ctx, lane as i64);
        self.solver.assert(&lane_var._eq(&lane_val));

        // Position at t=0
        let px_var = &self.positions_x[actor_id][0];
        if (pos_min - pos_max).abs() < 1e-6 {
            // Fixed value
            let pos_val = Real::from_real(self.ctx, (pos_min * 10.0) as i32, 10);
            self.solver.assert(&px_var._eq(&pos_val));
        } else {
            // Range
            let min_val = Real::from_real(self.ctx, (pos_min * 10.0) as i32, 10);
            let max_val = Real::from_real(self.ctx, (pos_max * 10.0) as i32, 10);
            self.solver.assert(&px_var.ge(&min_val));
            self.solver.assert(&px_var.le(&max_val));
        }

        // Velocity at t=0
        let vx_var = &self.velocities_x[actor_id][0];
        if (speed_min - speed_max).abs() < 1e-6 {
            // Fixed value
            let speed_val = Real::from_real(self.ctx, (speed_min * 10.0) as i32, 10);
            self.solver.assert(&vx_var._eq(&speed_val));
        } else {
            // Range
            let min_val = Real::from_real(self.ctx, (speed_min * 10.0) as i32, 10);
            let max_val = Real::from_real(self.ctx, (speed_max * 10.0) as i32, 10);
            self.solver.assert(&vx_var.ge(&min_val));
            self.solver.assert(&vx_var.le(&max_val));
        }

        // Initial lateral velocity is zero (not changing lanes initially)
        let vy_var = &self.velocities_y[actor_id][0];
        let zero = Real::from_real(self.ctx, 0, 1);
        self.solver.assert(&vy_var._eq(&zero));

        // Initial acceleration at t=0
        let ax_var = &self.accelerations_x[actor_id][0];
        if (accel_min - accel_max).abs() < 1e-6 {
            // Fixed acceleration
            let accel_val = Real::from_real(self.ctx, (accel_min * 10.0) as i32, 10);
            self.solver.assert(&ax_var._eq(&accel_val));
        } else {
            // Acceleration range
            let min_val = Real::from_real(self.ctx, (accel_min * 10.0) as i32, 10);
            let max_val = Real::from_real(self.ctx, (accel_max * 10.0) as i32, 10);
            self.solver.assert(&ax_var.ge(&min_val));
            self.solver.assert(&ax_var.le(&max_val));
        }

        // Initial lateral acceleration is zero
        let ay_var = &self.accelerations_y[actor_id][0];
        self.solver.assert(&ay_var._eq(&zero));
    }

    /// Encode constraint: lateral position matches lane center
    /// py = lane * lane_width + lane_width/2
    fn encode_lane_position_coupling_at_time(&mut self, actor_id: &str, t: usize) {
        let lane_var = &self.lanes[actor_id][t];
        let py_var = &self.positions_y[actor_id][t];

        let lane_width = self.spec.lane_width;
        let lane_width_real = Real::from_real(self.ctx, (lane_width * 10.0) as i32, 10);
        let half_width = Real::from_real(self.ctx, (lane_width * 5.0) as i32, 10);

        // py = lane * lane_width + lane_width/2
        let lane_real = lane_var.to_real();
        let expected_py = lane_real * &lane_width_real + &half_width;
        self.solver.assert(&py_var._eq(&expected_py));
    }

    /// Encode kinematic constraints with acceleration support
    pub fn encode_kinematics(&mut self) {
        use crate::dsl::types::ActorRole;

        let dt = self.spec.time_step;
        let dt_real = Real::from_real(self.ctx, (dt * 10.0) as i32, 10);
        let zero = Real::from_real(self.ctx, 0, 1);

        for actor in &self.spec.actors {
            let actor_id = &actor.id;

            // Get acceleration bounds directly from actor spec
            let ax_min = actor.acceleration.min();
            let ax_max = actor.acceleration.max();
            let ax_min_real = Real::from_real(self.ctx, (ax_min * 10.0) as i32, 10);
            let ax_max_real = Real::from_real(self.ctx, (ax_max * 10.0) as i32, 10);

            for t in 0..self.horizon {
                // ========== LONGITUDINAL DYNAMICS ==========

                // Acceleration bounds at each timestep
                let ax_t = &self.accelerations_x[actor_id][t];
                self.solver.assert(&ax_t.ge(&ax_min_real));
                self.solver.assert(&ax_t.le(&ax_max_real));

                // Velocity update: vx[t+1] = vx[t] + ax[t] * dt
                let vx_t = &self.velocities_x[actor_id][t];
                let vx_t1 = &self.velocities_x[actor_id][t + 1];
                let expected_vx = vx_t + &(ax_t * &dt_real);
                self.solver.assert(&vx_t1._eq(&expected_vx));

                // Non-negative velocity (no reversing)
                self.solver.assert(&vx_t.ge(&zero));

                // Position update: px[t+1] = px[t] + vx[t] * dt
                let px_t = &self.positions_x[actor_id][t];
                let px_t1 = &self.positions_x[actor_id][t + 1];
                let expected_px = px_t + &(vx_t * &dt_real);
                self.solver.assert(&px_t1._eq(&expected_px));

                // ========== LATERAL DYNAMICS ==========

                // Lateral position update: py[t+1] = py[t] + vy[t] * dt
                let py_t = &self.positions_y[actor_id][t];
                let py_t1 = &self.positions_y[actor_id][t + 1];
                let vy_t = &self.velocities_y[actor_id][t];
                let expected_py = py_t + &(vy_t * &dt_real);
                self.solver.assert(&py_t1._eq(&expected_py));

                // Ego never changes lanes (vy = 0)
                if actor.role == ActorRole::Ego {
                    self.solver.assert(&vy_t._eq(&zero));
                }
            }

            // Ensure final velocity is non-negative
            let vx_final = &self.velocities_x[actor_id][self.horizon];
            self.solver.assert(&vx_final.ge(&zero));
        }

        // Lane-position coupling for all time steps
        let actor_ids: Vec<_> = self.spec.actors.iter().map(|a| a.id.clone()).collect();
        for actor_id in actor_ids {
            for t in 0..=self.horizon {
                self.encode_lane_position_coupling_at_time(&actor_id, t);
            }
        }
    }

    /// Check if the constraints are satisfiable (for testing)
    pub fn check(&self) -> SatResult {
        self.solver.check()
    }

    /// Get the Z3 model (for testing)
    pub fn get_model(&self) -> Option<z3::Model<'ctx>> {
        self.solver.get_model()
    }

    /// Encode scenario-specific Z3 constraints
    ///
    /// This calls the trait method to allow scenarios to add custom Z3 assertions
    /// beyond the standard LTL and safety encodings.
    pub fn encode_scenario_specific_constraints(
        &mut self,
        model: &dyn crate::scenarios::ScenarioModel,
    ) -> anyhow::Result<()> {
        model.add_z3_constraints(&self.spec, self, &self.solver, self.horizon)
    }

    /// Encode LTL formula into Z3 constraints using bounded model checking
    ///
    /// This is the core of Phase 7. We expand temporal operators over the
    /// finite time horizon, converting them into Boolean combinations of
    /// propositions at different time steps.
    pub fn encode_ltl(&mut self, formula: &crate::ltl::formula::LTLFormula) {
        // Encode the formula starting at time 0, with full horizon
        let constraint = self.encode_ltl_bounded(formula, 0, self.horizon);
        self.solver.assert(&constraint);
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
    ) -> z3::ast::Bool<'ctx> {
        use crate::ltl::formula::LTLFormula;

        match formula {
            // Atomic proposition - encode at specific time
            LTLFormula::Atom(prop) => self.encode_proposition(prop, time),

            // Boolean operators - recursive encoding
            LTLFormula::Not(phi) => self.encode_ltl_bounded(phi, time, horizon).not(),

            LTLFormula::And(phi, psi) => {
                let left = self.encode_ltl_bounded(phi, time, horizon);
                let right = self.encode_ltl_bounded(psi, time, horizon);
                z3::ast::Bool::and(self.ctx, &[&left, &right])
            }

            LTLFormula::Or(phi, psi) => {
                let left = self.encode_ltl_bounded(phi, time, horizon);
                let right = self.encode_ltl_bounded(psi, time, horizon);
                z3::ast::Bool::or(self.ctx, &[&left, &right])
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
                    z3::ast::Bool::from_bool(self.ctx, false)
                }
            }

            // Eventually: F(φ) = φ[time] ∨ φ[time+1] ∨ ... ∨ φ[horizon]
            LTLFormula::Eventually(phi) => {
                let mut disjuncts = Vec::new();
                for t in time..=horizon {
                    disjuncts.push(self.encode_ltl_bounded(phi, t, horizon));
                }
                let refs: Vec<&z3::ast::Bool> = disjuncts.iter().collect();
                z3::ast::Bool::or(self.ctx, &refs)
            }

            // Always: G(φ) = φ[time] ∧ φ[time+1] ∧ ... ∧ φ[horizon]
            LTLFormula::Always(phi) => {
                let mut conjuncts = Vec::new();
                for t in time..=horizon {
                    conjuncts.push(self.encode_ltl_bounded(phi, t, horizon));
                }
                let refs: Vec<&z3::ast::Bool> = conjuncts.iter().collect();
                z3::ast::Bool::and(self.ctx, &refs)
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
                        let phi_holds = z3::ast::Bool::and(self.ctx, &phi_refs);

                        // (φ[time] ∧ ... ∧ φ[t-1]) ∧ ψ[t]
                        let both = z3::ast::Bool::and(self.ctx, &[&phi_holds, &psi_at_t]);
                        disjuncts.push(both);
                    }
                }

                let refs: Vec<&z3::ast::Bool> = disjuncts.iter().collect();
                z3::ast::Bool::or(self.ctx, &refs)
            }
        }
    }

    /// Encode atomic propositions as Z3 constraints at a specific time
    fn encode_proposition(
        &self,
        prop: &crate::ltl::formula::Proposition,
        time: usize,
    ) -> z3::ast::Bool<'ctx> {
        use crate::ltl::formula::Proposition;

        match prop {
            // InLane(actor, lane): lane_var[t] == lane
            Proposition::InLane { actor, lane } => {
                let lane_var = &self.lanes[actor][time];
                let lane_val = Int::from_i64(self.ctx, *lane as i64);
                lane_var._eq(&lane_val)
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
                let dist_val = Real::from_real(self.ctx, (*distance * 10.0) as i32, 10);

                // |px1 - px2| > d is equivalent to: (px1 - px2 > d) OR (px2 - px1 > d)
                let diff_pos = px1 - px2;
                let diff_neg = px2 - px1;

                let pos_case = diff_pos.gt(&dist_val);
                let neg_case = diff_neg.gt(&dist_val);

                z3::ast::Bool::or(self.ctx, &[&pos_case, &neg_case])
            }

            // TTCGT(actor1, actor2, ttc): TTC > ttc (if collision possible)
            Proposition::TTCGT {
                actor1,
                actor2,
                ttc,
            } => self.encode_ttc_constraint(actor1, actor2, *ttc, time),
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
    ) -> z3::ast::Bool<'ctx> {
        let lane1 = &self.lanes[actor1][time];
        let lane2 = &self.lanes[actor2][time];

        let px1 = &self.positions_x[actor1][time];
        let px2 = &self.positions_x[actor2][time];

        let vx1 = &self.velocities_x[actor1][time];
        let vx2 = &self.velocities_x[actor2][time];

        let min_ttc_val = Real::from_real(self.ctx, (min_ttc * 10.0) as i32, 10);
        let epsilon = Real::from_real(self.ctx, 1, 100); // 0.01 m/s to avoid division by zero

        // Same lane condition
        let same_lane = lane1._eq(lane2);

        // Determine who is ahead and who is behind
        // If px1 > px2, then actor1 is ahead (lead), actor2 is behind (follow)
        // If px2 > px1, then actor2 is ahead (lead), actor1 is behind (follow)

        // Case 1: actor1 ahead, actor2 behind, actor2 faster
        // TTC = (px1 - px2) / (vx2 - vx1)
        let actor1_ahead = px1.gt(px2);
        let actor2_faster = vx2.gt(vx1);
        let rel_vel_1 = vx2 - vx1;
        let distance_1 = px1 - px2;
        let collision_possible_1 = z3::ast::Bool::and(
            self.ctx,
            &[&actor1_ahead, &actor2_faster, &rel_vel_1.gt(&epsilon)],
        );
        // TTC > min_ttc means: distance / rel_vel > min_ttc
        // Equivalent to: distance > min_ttc * rel_vel
        let ttc_safe_1 = distance_1.gt(&(&min_ttc_val * &rel_vel_1));

        // Case 2: actor2 ahead, actor1 behind, actor1 faster
        // TTC = (px2 - px1) / (vx1 - vx2)
        let actor2_ahead = px2.gt(px1);
        let actor1_faster = vx1.gt(vx2);
        let rel_vel_2 = vx1 - vx2;
        let distance_2 = px2 - px1;
        let collision_possible_2 = z3::ast::Bool::and(
            self.ctx,
            &[&actor2_ahead, &actor1_faster, &rel_vel_2.gt(&epsilon)],
        );
        let ttc_safe_2 = distance_2.gt(&(&min_ttc_val * &rel_vel_2));

        // Overall constraint:
        // If same_lane AND collision_possible_1, then ttc_safe_1
        // If same_lane AND collision_possible_2, then ttc_safe_2
        // Otherwise, true (no collision risk)

        let case1 =
            z3::ast::Bool::and(self.ctx, &[&same_lane, &collision_possible_1]).implies(&ttc_safe_1);
        let case2 =
            z3::ast::Bool::and(self.ctx, &[&same_lane, &collision_possible_2]).implies(&ttc_safe_2);

        z3::ast::Bool::and(self.ctx, &[&case1, &case2])
    }

    /// Encode all safety constraints
    ///
    /// When constraints are "enforced", we add direct Z3 assertions at each timestep.
    /// When "violated" or "ignored", we rely on LTL encoding only.
    ///
    /// Generates pairwise safety constraints for all actor combinations.
    pub fn encode_safety(&mut self) {
        let min_ttc = self.spec.min_ttc;
        let min_distance = self.spec.min_distance;

        // Generate pairwise safety constraints for all actor combinations
        for (i, actor1) in self.spec.actors.iter().enumerate() {
            for actor2 in self.spec.actors.iter().skip(i + 1) {
                let id1 = &actor1.id;
                let id2 = &actor2.id;

                // Only add direct safety assertions if constraints are enforced
                // For violate/ignore modes, the LTL formula handles it

                if self.spec.constraint_modes.min_ttc() == ConstraintMode::Enforce {
                    for t in 0..=self.horizon {
                        let ttc_constraint = self.encode_ttc_constraint(id1, id2, min_ttc, t);
                        self.solver.assert(&ttc_constraint);
                    }
                }

                if self.spec.constraint_modes.min_distance() == ConstraintMode::Enforce {
                    for t in 0..=self.horizon {
                        let distance_constraint =
                            self.encode_min_distance_constraint(id1, id2, min_distance, t);
                        self.solver.assert(&distance_constraint);
                    }
                }
            }
        }
    }

    /// Encode minimum distance constraint between two actors
    ///
    /// When actors are in the same lane, they must maintain minimum distance.
    fn encode_min_distance_constraint(
        &self,
        actor1: &str,
        actor2: &str,
        min_distance: f64,
        time: usize,
    ) -> z3::ast::Bool<'ctx> {
        let lane1 = &self.lanes[actor1][time];
        let lane2 = &self.lanes[actor2][time];

        let px1 = &self.positions_x[actor1][time];
        let px2 = &self.positions_x[actor2][time];

        let min_dist_val = Real::from_real(self.ctx, (min_distance * 10.0) as i32, 10);

        // Same lane condition
        let same_lane = lane1._eq(lane2);

        // Distance: |px1 - px2|
        // We need: |px1 - px2| > min_distance
        // This is: (px1 - px2 > min_distance) OR (px2 - px1 > min_distance)
        let diff_1 = px1 - px2;
        let diff_2 = px2 - px1;

        let dist_case_1 = diff_1.gt(&min_dist_val);
        let dist_case_2 = diff_2.gt(&min_dist_val);

        let distance_ok = z3::ast::Bool::or(self.ctx, &[&dist_case_1, &dist_case_2]);

        // If same lane, then distance must be OK
        same_lane.implies(&distance_ok)
    }

    /// Encode global max acceleration constraints (if specified)
    pub fn encode_acceleration_constraints(&mut self) {
        // Only apply if max_acceleration/max_deceleration specified AND mode is Enforce
        if let Some(max_accel) = self.spec.max_acceleration {
            let mode = self.spec.constraint_modes.max_acceleration();

            if mode == ConstraintMode::Enforce {
                let max_real = Real::from_real(self.ctx, (max_accel * 10.0) as i32, 10);

                for actor in &self.spec.actors {
                    for t in 0..=self.horizon {
                        let ax = &self.accelerations_x[&actor.id][t];
                        self.solver.assert(&ax.le(&max_real));
                    }
                }
            }
            // Violate and Ignore modes handled via LTL
        }

        if let Some(max_decel) = self.spec.max_deceleration {
            // max_decel should be negative (e.g., -3.0)
            let mode = self.spec.constraint_modes.max_acceleration();

            if mode == ConstraintMode::Enforce {
                let min_real = Real::from_real(self.ctx, (max_decel * 10.0) as i32, 10);

                for actor in &self.spec.actors {
                    for t in 0..=self.horizon {
                        let ax = &self.accelerations_x[&actor.id][t];
                        self.solver.assert(&ax.ge(&min_real));
                    }
                }
            }
        }
    }

    /// Extract scenario from Z3 model
    ///
    /// Converts the Z3 solution (satisfying assignment) into a Scenario
    /// JSON structure with actor trajectories.
    pub fn extract_scenario(&self, model: &z3::Model<'ctx>) -> crate::scenario::model::Scenario {
        use crate::dsl::types::ActorRole;

        let mut scenario = crate::scenario::model::Scenario::new(
            self.spec.scenario_type.to_string(),
            self.spec.time_step,
            self.spec.duration,
        );

        // Extract trajectory for each actor
        for actor in &self.spec.actors {
            let role_str = match actor.role {
                ActorRole::Ego => "ego",
                ActorRole::Npc => "npc",
            };

            let trajectory = self.extract_actor_trajectory(model, &actor.id, role_str);
            scenario.add_actor(trajectory);
        }

        // Compute validation metrics
        self.compute_validation_metrics(&mut scenario);

        scenario
    }

    /// Extract trajectory for a single actor
    fn extract_actor_trajectory(
        &self,
        model: &z3::Model<'ctx>,
        actor_id: &str,
        role: &str,
    ) -> crate::scenario::model::ActorTrajectory {
        use crate::scenario::model::{Acceleration, ActorTrajectory, Position, State, Velocity};

        let mut trajectory = ActorTrajectory::new(actor_id.to_string(), role.to_string());

        for t in 0..=self.horizon {
            let time = t as f64 * self.spec.time_step;

            // Extract position
            let px = self.extract_real(model, &self.positions_x[actor_id][t]);
            let py = self.extract_real(model, &self.positions_y[actor_id][t]);

            // Extract velocity
            let vx = self.extract_real(model, &self.velocities_x[actor_id][t]);
            let vy = self.extract_real(model, &self.velocities_y[actor_id][t]);

            // Extract acceleration
            let ax = self.extract_real(model, &self.accelerations_x[actor_id][t]);
            let ay = self.extract_real(model, &self.accelerations_y[actor_id][t]);

            // Extract lane
            let lane = self.extract_int(model, &self.lanes[actor_id][t]);

            let state = State::new(
                time,
                Position::new(px, py),
                Velocity::new(vx, vy),
                Acceleration::new(ax, ay),
                lane,
            );

            trajectory.add_state(state);
        }

        trajectory
    }

    /// Extract a real value from Z3 model
    fn extract_real(&self, model: &z3::Model<'ctx>, var: &Real<'ctx>) -> f64 {
        let ast = model
            .eval(var, true)
            .expect("Failed to evaluate real variable");

        // Z3 returns rationals in various formats
        let ast_str = ast.to_string();

        // Remove all parentheses and work with the content
        let cleaned = ast_str.replace('(', "").replace(')', "");

        // Split by whitespace to get components
        let parts: Vec<&str> = cleaned.split_whitespace().collect();

        // Check for various formats
        if parts.is_empty() {
            panic!("Failed to parse real value from Z3: '{}'", ast_str);
        }

        // Format: "- / numerator denominator" -> negative fraction
        if parts.len() >= 4 && parts[0] == "-" && parts[1] == "/" {
            let numerator: f64 = parts[2].parse().expect("Failed to parse numerator");
            let denominator: f64 = parts[3].parse().expect("Failed to parse denominator");
            return -(numerator / denominator);
        }

        // Format: "/ numerator denominator" -> positive fraction
        if parts.len() >= 3 && parts[0] == "/" {
            let numerator: f64 = parts[1].parse().expect("Failed to parse numerator");
            let denominator: f64 = parts[2].parse().expect("Failed to parse denominator");
            return numerator / denominator;
        }

        // Format: "- value" -> simple negative
        if parts.len() == 2 && parts[0] == "-" {
            let value: f64 = parts[1].parse().expect("Failed to parse negative value");
            return -value;
        }

        // Format: "numerator/denominator" or "-numerator/denominator"
        if parts.len() == 1 && parts[0].contains('/') {
            let frac_parts: Vec<&str> = parts[0].split('/').collect();
            let numerator: f64 = frac_parts[0].parse().expect("Failed to parse numerator");
            let denominator: f64 = frac_parts[1].parse().expect("Failed to parse denominator");
            return numerator / denominator;
        }

        // Default: try to parse as a simple number
        parts[0]
            .parse()
            .unwrap_or_else(|_| panic!("Failed to parse real value from Z3: '{}'", ast_str))
    }

    /// Extract an integer value from Z3 model
    fn extract_int(&self, model: &z3::Model<'ctx>, var: &Int<'ctx>) -> usize {
        let ast = model
            .eval(var, true)
            .expect("Failed to evaluate int variable");
        let ast_str = ast.to_string();

        // Handle negative values like "(- 5)" or simple values like "1"
        let cleaned = ast_str.trim_start_matches('(').trim_end_matches(')');

        if cleaned.starts_with("- ") {
            // Format: "(- value)" - but usize can't be negative, so this would be an error
            panic!("Cannot extract negative integer as usize: '{}'", ast_str);
        }

        cleaned
            .trim()
            .parse()
            .unwrap_or_else(|_| panic!("Failed to parse int value from Z3: '{}'", ast_str))
    }

    /// Compute validation metrics from the scenario trajectories
    fn compute_validation_metrics(&self, scenario: &mut crate::scenario::model::Scenario) {
        let mut min_ttc = f64::INFINITY;
        let mut min_distance = f64::INFINITY;
        let mut violations = Vec::new();

        // Compute pairwise metrics for all actor combinations
        for (i, id1) in self.spec.actors.iter().map(|a| a.id.clone()).enumerate() {
            for id2 in self.spec.actors.iter().skip(i + 1).map(|a| a.id.clone()) {
                let traj1 = scenario
                    .get_actor(&id1)
                    .expect(&format!("Actor {} missing", id1));
                let traj2 = scenario
                    .get_actor(&id2)
                    .expect(&format!("Actor {} missing", id2));

                for t in 0..=self.horizon {
                    let state1 = &traj1.states[t];
                    let state2 = &traj2.states[t];

                    // Compute longitudinal distance
                    let distance = (state1.position.x - state2.position.x).abs();

                    // Only consider distance when in same lane
                    if state1.lane == state2.lane {
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
                    if state1.lane == state2.lane {
                        let rel_vel = (state1.velocity.vx - state2.velocity.vx).abs();

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
                let ax = state.acceleration.ax;

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
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dsl::types::{ActorSpec, ActorRole, ScenarioType, ValueOrRange};
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
                    behavior: HashMap::new(),
                },
                ActorSpec {
                    id: "npc".to_string(),
                    role: ActorRole::Npc,
                    lane: 0,
                    position: ValueOrRange::Range([60.0, 80.0]),
                    speed: ValueOrRange::Range([12.0, 14.0]),
                    acceleration: ValueOrRange::Range([-8.0, 3.0]),
                    behavior: npc_behavior,
                },
            ],
            min_ttc: 3.0,
            min_distance: 5.0,
            road: None,
            lane_width: 3.5,
            num_scenarios: 1,
            constraint_modes: crate::dsl::types::ConstraintModes::default(),
            max_acceleration: None,
            max_deceleration: None,
        }
    }

    #[test]
    fn test_encoder_creation() {
        let cfg = Config::new();
        let ctx = Context::new(&cfg);
        let spec = create_test_spec();

        let encoder = Z3Encoder::new(&ctx, spec);
        assert_eq!(encoder.horizon, 20); // 10.0 / 0.5 = 20
    }

    #[test]
    fn test_create_variables() {
        let cfg = Config::new();
        let ctx = Context::new(&cfg);
        let spec = create_test_spec();

        let mut encoder = Z3Encoder::new(&ctx, spec);
        encoder.create_variables();

        // Check variables were created
        assert!(encoder.positions_x.contains_key("ego"));
        assert!(encoder.positions_x.contains_key("npc"));

        // Check we have the right number of time steps
        assert_eq!(encoder.positions_x["ego"].len(), 21); // 0..=20
        assert_eq!(encoder.velocities_x["npc"].len(), 21);
    }

    #[test]
    fn test_encode_initial_conditions() {
        let cfg = Config::new();
        let ctx = Context::new(&cfg);
        let spec = create_test_spec();

        let mut encoder = Z3Encoder::new(&ctx, spec);
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
    }

    #[test]
    fn test_lane_position_coupling() {
        let cfg = Config::new();
        let ctx = Context::new(&cfg);
        let spec = create_test_spec();

        let mut encoder = Z3Encoder::new(&ctx, spec);
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
    }

    #[test]
    fn test_kinematics() {
        let cfg = Config::new();
        let ctx = Context::new(&cfg);
        let spec = create_test_spec();

        let mut encoder = Z3Encoder::new(&ctx, spec);
        encoder.create_variables();
        encoder.encode_initial_conditions();
        encoder.encode_kinematics();

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
    }

    #[test]
    fn test_ltl_encoding_simple() {
        use crate::ltl::formula::{LTLFormula, Proposition};

        let cfg = Config::new();
        let ctx = Context::new(&cfg);
        let spec = create_test_spec();

        let mut encoder = Z3Encoder::new(&ctx, spec);
        encoder.create_variables();
        encoder.encode_initial_conditions();
        encoder.encode_kinematics();

        // Test simple atomic proposition: InLane(ego, 1)
        let formula = LTLFormula::Atom(Proposition::InLane {
            actor: "ego".to_string(),
            lane: 1,
        });

        encoder.encode_ltl(&formula);
        assert_eq!(encoder.check(), SatResult::Sat);
    }

    #[test]
    fn test_ltl_encoding_eventually() {
        use crate::ltl::formula::{LTLFormula, Proposition};

        let cfg = Config::new();
        let ctx = Context::new(&cfg);
        let spec = create_test_spec();

        let mut encoder = Z3Encoder::new(&ctx, spec);
        encoder.create_variables();
        encoder.encode_initial_conditions();
        encoder.encode_kinematics();

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
    }

    #[test]
    fn test_ltl_encoding_always() {
        use crate::ltl::formula::{LTLFormula, Proposition};

        let cfg = Config::new();
        let ctx = Context::new(&cfg);
        let spec = create_test_spec();

        let mut encoder = Z3Encoder::new(&ctx, spec);
        encoder.create_variables();
        encoder.encode_initial_conditions();
        encoder.encode_kinematics();

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
    }

    #[test]
    fn test_ltl_encoding_until() {
        use crate::ltl::formula::{LTLFormula, Proposition};

        let cfg = Config::new();
        let ctx = Context::new(&cfg);
        let spec = create_test_spec();

        let mut encoder = Z3Encoder::new(&ctx, spec);
        encoder.create_variables();
        encoder.encode_initial_conditions();
        encoder.encode_kinematics();

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
    }

    #[test]
    fn test_safety_constraints() {
        let cfg = Config::new();
        let ctx = Context::new(&cfg);
        let spec = create_test_spec();

        let mut encoder = Z3Encoder::new(&ctx, spec);
        encoder.create_variables();
        encoder.encode_initial_conditions();
        encoder.encode_kinematics();
        encoder.encode_safety();

        // Safety constraints should be satisfiable
        assert_eq!(encoder.check(), SatResult::Sat);
    }

    #[test]
    fn test_full_cut_in_scenario() {
        use crate::ltl::generator::LTLGenerator;

        let cfg = Config::new();
        let ctx = Context::new(&cfg);
        let spec = create_test_spec();

        let mut encoder = Z3Encoder::new(&ctx, spec.clone());
        encoder.create_variables();
        encoder.encode_initial_conditions();
        encoder.encode_kinematics();

        // Generate and encode full cut-in LTL formula
        let ltl_formula = LTLGenerator::generate(&spec);
        encoder.encode_ltl(&ltl_formula);

        // Add safety constraints
        encoder.encode_safety();

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
    }

    #[test]
    fn test_scenario_extraction() {
        use crate::ltl::generator::LTLGenerator;

        let cfg = Config::new();
        let ctx = Context::new(&cfg);
        let spec = create_test_spec();

        let mut encoder = Z3Encoder::new(&ctx, spec.clone());
        encoder.create_variables();
        encoder.encode_initial_conditions();
        encoder.encode_kinematics();

        // Generate and encode full cut-in LTL formula
        let ltl_formula = LTLGenerator::generate(&spec);
        encoder.encode_ltl(&ltl_formula);
        encoder.encode_safety();

        // Check satisfiability
        let result = encoder.check();
        assert_eq!(result, SatResult::Sat, "Should be satisfiable");

        if result == SatResult::Sat {
            let model = encoder.get_model().unwrap();

            // Extract scenario
            let scenario = encoder.extract_scenario(&model);

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
            assert_eq!(ego.states[0].lane, 1);
            assert_eq!(npc.states[0].lane, 0);

            // Verify NPC position is ahead initially
            assert!(npc.states[0].position.x > ego.states[0].position.x);

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
    }
}
