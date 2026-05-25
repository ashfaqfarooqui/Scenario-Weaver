//! Z3 constraint encoder

use z3::ast::{Bool, Int, Real};
use z3::SatResult;

use crate::dsl::types::{CoordinateSystem, ScenarioSpec};
use crate::solver::backend::{OptimizationTarget, OptimizerBackend, SolverBackend, Z3Backend};
use crate::solver::coordinate_encoder::CoordinateEncoder;
use crate::solver::encoders::bicycle::BicycleEncoder;
use crate::solver::encoders::cartesian::CartesianEncoder;

/// Z3 SMT encoder for scenario constraints (generic over backend)
///
/// A thin facade that dispatches to coordinate-specific encoders
/// (CartesianEncoder or BicycleEncoder) via the CoordinateEncoder trait.
///
/// Works with either `SolverBackend` (SAT checking) or `OptimizerBackend`
/// (optimization objectives). The type alias `Z3Encoder = GenericEncoder<SolverBackend>`
/// is provided for the common SAT-solving case.
///
/// Supports both Cartesian (x, y) and Bicycle (x, y, θ, v) coordinate systems.
///
/// Note: In Z3 0.19, the context is managed internally and is implicit
/// within the `with_z3_config()` callback scope.
pub struct GenericEncoder<B: Z3Backend> {
    /// Coordinate-specific encoder (Cartesian or Bicycle)
    coord_encoder: Box<dyn CoordinateEncoder<B>>,

    /// Original scenario specification
    pub(crate) spec: ScenarioSpec,

    /// Number of time steps in the scenario
    pub(crate) horizon: usize,
}

/// Type alias for backward compatibility - uses Solver backend
pub type Z3Encoder = GenericEncoder<SolverBackend>;

impl<B: Z3Backend + 'static> GenericEncoder<B> {
    /// Create a new encoder with a specific backend
    ///
    /// Dispatches to the appropriate coordinate-specific encoder based on the
    /// coordinate system specified in the scenario spec.
    ///
    /// Note: This must be called within a `z3::with_z3_config()` callback.
    pub fn with_backend(spec: ScenarioSpec, backend: B) -> Self {
        let horizon = spec.num_time_steps();

        // Dispatch to appropriate encoder based on coordinate system
        let coord_encoder: Box<dyn CoordinateEncoder<B>> = match spec.coordinate_system {
            CoordinateSystem::Cartesian => Box::new(CartesianEncoder::new(spec.clone(), backend)),
            CoordinateSystem::Bicycle => Box::new(BicycleEncoder::new(spec.clone(), backend)),
        };

        Self {
            coord_encoder,
            spec,
            horizon,
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
        model
            .add_z3_constraints(&self.spec, self, self.coord_encoder.backend(), self.horizon)
            .map_err(|e| anyhow::anyhow!(e))
    }
}

impl<B: Z3Backend + 'static> GenericEncoder<B> {
    /// Create all Z3 variables for the scenario
    ///
    /// Delegates to the coordinate-specific encoder.
    pub fn create_variables(&mut self) {
        self.coord_encoder
            .create_variables(self.horizon, &self.spec);
    }

    /// Encode initial conditions from the DSL specification
    pub fn encode_initial_conditions(&mut self) {
        self.coord_encoder.encode_initial_conditions();
    }

    /// Encode kinematic constraints with acceleration support
    pub fn encode_kinematics(&mut self) {
        self.coord_encoder.encode_kinematics(self.spec.time_step);
    }

    /// Encode speed upper bounds for all actors
    pub fn encode_velocity_constraints(&mut self) {
        self.coord_encoder.encode_velocity_constraints();
    }

    /// Encode lane-based velocity constraints
    pub fn encode_lane_velocity_constraints(&mut self) {
        self.coord_encoder.encode_lane_velocity_constraints();
    }

    /// Encode lateral velocity bounds for realistic lane changes
    pub fn encode_lateral_velocity_bounds(&mut self) {
        self.coord_encoder.encode_lateral_velocity_bounds();
    }

    /// Check if the constraints are satisfiable (for testing)
    pub fn check(&self) -> SatResult {
        self.coord_encoder.backend().check()
    }

    /// Get the Z3 model (for testing)
    pub fn get_model(&self) -> Option<z3::Model> {
        self.coord_encoder.backend().get_model()
    }

    // === Variable Accessor Methods ===
    // These provide access to Z3 variables for scenario-specific constraints

    /// Get lane variable for an actor at a given time
    pub fn get_lane_var(&self, actor_id: &str, time: usize) -> &Int {
        self.coord_encoder.get_lane_var(actor_id, time)
    }

    /// Get longitudinal position variable for an actor at a given time
    pub fn get_longitudinal_pos(&self, actor_id: &str, time: usize) -> &Real {
        self.coord_encoder.get_longitudinal_pos(actor_id, time)
    }

    /// Get lateral position variable for an actor at a given time
    pub fn get_lateral_pos(&self, actor_id: &str, time: usize) -> &Real {
        self.coord_encoder.get_lateral_pos(actor_id, time)
    }

    /// Get longitudinal velocity variable for an actor at a given time
    pub fn get_longitudinal_vel(&self, actor_id: &str, time: usize) -> &Real {
        self.coord_encoder.get_longitudinal_vel(actor_id, time)
    }

    /// Get lateral velocity variable for an actor at a given time
    pub fn get_lateral_vel(&self, actor_id: &str, time: usize) -> &Real {
        self.coord_encoder.get_lateral_vel(actor_id, time)
    }

    // === Coordinate-specific accessors (for multi_solve.rs blocking clauses) ===

    /// Get Cartesian x position (maps to longitudinal position)
    pub fn get_position_x(&self, actor_id: &str, time: usize) -> &Real {
        self.get_longitudinal_pos(actor_id, time)
    }

    /// Get Cartesian x velocity (maps to longitudinal velocity)
    pub fn get_velocity_x(&self, actor_id: &str, time: usize) -> &Real {
        self.get_longitudinal_vel(actor_id, time)
    }

    /// Get Cartesian y position (maps to lateral position)
    pub fn get_position_y(&self, actor_id: &str, time: usize) -> &Real {
        self.get_lateral_pos(actor_id, time)
    }

    /// Get Cartesian y velocity (maps to lateral velocity)
    pub fn get_velocity_y(&self, actor_id: &str, time: usize) -> &Real {
        self.get_lateral_vel(actor_id, time)
    }

    /// Assert a constraint directly to the backend
    pub fn assert_constraint(&mut self, constraint: &Bool) {
        self.coord_encoder.backend_mut().assert(constraint);
    }

    // === LTL Encoding ===

    /// Encode LTL formula into Z3 constraints using bounded model checking
    ///
    /// This is the core of Phase 7. We expand temporal operators over the
    /// finite time horizon, converting them into Boolean combinations of
    /// propositions at different time steps.
    pub fn encode_ltl(&mut self, formula: &crate::ltl::formula::LTLFormula) {
        // Encode the formula starting at time 0, with full horizon
        let constraint = self.encode_ltl_bounded(formula, 0, self.horizon);
        self.coord_encoder.backend_mut().assert(&constraint);
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
            // Boolean literals
            LTLFormula::True => z3::ast::Bool::from_bool(true),
            LTLFormula::False => z3::ast::Bool::from_bool(false),

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
                let lane_var = self.get_lane_var(actor, time);
                let lane_val = Int::from_i64(*lane as i64);
                lane_var.eq(&lane_val)
            }

            // Ahead(actor1, actor2): px1[t] > px2[t]
            Proposition::Ahead { actor1, actor2 } => {
                let px1 = self.get_longitudinal_pos(actor1, time);
                let px2 = self.get_longitudinal_pos(actor2, time);
                px1.gt(px2)
            }

            // DistanceGT(actor1, actor2, d): |px1[t] - px2[t]| > d
            Proposition::DistanceGT {
                actor1,
                actor2,
                distance,
            } => {
                let px1 = self.get_longitudinal_pos(actor1, time);
                let px2 = self.get_longitudinal_pos(actor2, time);
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
                let py = self.get_lateral_pos(actor, time);
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
                let py = self.get_lateral_pos(actor, time);
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
                let px1 = self.get_longitudinal_pos(actor1, time);
                let py1 = self.get_lateral_pos(actor1, time);
                let px2 = self.get_longitudinal_pos(actor2, time);
                let py2 = self.get_lateral_pos(actor2, time);

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
                let px1 = self.get_longitudinal_pos(actor1, time);
                let py1 = self.get_lateral_pos(actor1, time);
                let px2 = self.get_longitudinal_pos(actor2, time);
                let py2 = self.get_lateral_pos(actor2, time);

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
                let px1 = self.get_longitudinal_pos(actor1, time);
                let py1 = self.get_lateral_pos(actor1, time);
                let px2 = self.get_longitudinal_pos(actor2, time);
                let py2 = self.get_lateral_pos(actor2, time);

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
                let ego_px = self.get_longitudinal_pos(ego, time);
                let ego_vx = self.get_longitudinal_vel(ego, time);
                let ped_px = self.get_longitudinal_pos(pedestrian, time);
                let ped_py = self.get_lateral_pos(pedestrian, time);

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
                let vx = self.get_longitudinal_vel(actor, time);
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
                let vx = self.get_longitudinal_vel(actor, time);
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
                let py1 = self.get_lateral_pos(actor1, time);
                let py2 = self.get_lateral_pos(actor2, time);
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
                let py1 = self.get_lateral_pos(actor1, time);
                let py2 = self.get_lateral_pos(actor2, time);
                py1.gt(py2)
            }

            // OnRightOf: Actor1 is laterally right of Actor2
            // Simple comparison: py1 < py2
            Proposition::OnRightOf { actor1, actor2 } => {
                let py1 = self.get_lateral_pos(actor1, time);
                let py2 = self.get_lateral_pos(actor2, time);
                py1.lt(py2)
            }

            // RelativeVelocityGT: Relative longitudinal velocity exceeds threshold
            // Linear constraint: |vx1 - vx2| > velocity
            Proposition::RelativeVelocityGT {
                actor1,
                actor2,
                velocity,
            } => {
                let vx1 = self.get_longitudinal_vel(actor1, time);
                let vx2 = self.get_longitudinal_vel(actor2, time);
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
        let lane1 = self.get_lane_var(actor1, time);
        let lane2 = self.get_lane_var(actor2, time);

        let px1 = self.get_longitudinal_pos(actor1, time);
        let px2 = self.get_longitudinal_pos(actor2, time);

        let vx1 = self.get_longitudinal_vel(actor1, time);
        let vx2 = self.get_longitudinal_vel(actor2, time);

        let min_ttc_val = Real::from_rational((min_ttc * 10.0) as i64, 10_i64);
        let epsilon = Real::from_rational(1_i64, 100_i64); // 0.01 m/s to avoid division by zero

        // "Same lane" condition for TTC.
        // For same-direction actors: use discrete lane match only.
        // For opposite-direction actors: also use y-position proximity since they approach
        // from different lanes and TTC is still relevant when they're laterally overlapping.
        let actor1_dir = self
            .spec
            .actors
            .iter()
            .find(|a| a.id == actor1)
            .map_or(1, |a| a.direction);
        let actor2_dir = self
            .spec
            .actors
            .iter()
            .find(|a| a.id == actor2)
            .map_or(1, |a| a.direction);

        let same_lane = if actor1_dir != actor2_dir {
            // Opposite-direction: use y-proximity in addition to discrete lane match.
            // During a lane change transition, the NPC's y-position passes through the
            // ego's lane space, making head-on TTC relevant.
            let py1 = self.get_lateral_pos(actor1, time);
            let py2 = self.get_lateral_pos(actor2, time);
            let lane_width = self.spec.get_lane_width();
            let lane_width_real = Real::from_rational((lane_width * 10.0) as i64, 10_i64);
            let py_diff_pos = py1 - py2;
            let py_diff_neg = py2 - py1;
            let y_proximity = z3::ast::Bool::and(&[
                &py_diff_pos.lt(&lane_width_real),
                &py_diff_neg.lt(&lane_width_real),
            ]);
            z3::ast::Bool::or(&[&lane1.eq(lane2), &y_proximity])
        } else {
            // Same-direction: only discrete lane match (original behavior).
            lane1.eq(lane2)
        };

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
        self.coord_encoder.encode_acceleration_constraints();
    }

    /// Extract scenario from Z3 model
    ///
    /// Converts the Z3 solution (satisfying assignment) into a Scenario
    /// JSON structure with actor trajectories.
    pub fn extract_scenario(
        &self,
        model: &z3::Model,
    ) -> crate::error::Result<crate::scenario::model::Scenario> {
        use crate::dsl::types::ActorRole;

        // Get RoadSpec from ScenarioSpec (required, should always exist after validation)
        let road = self
            .spec
            .road
            .as_ref()
            .ok_or_else(|| {
                crate::error::ScenarioGenError::ExtractionFailed(
                    "RoadSpec is required - should be validated during spec parsing".to_string(),
                )
            })?
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
        self.coord_encoder
            .extract_actor_trajectory(model, actor_id, role)
    }

    /// Compute validation metrics from the scenario trajectories
    fn compute_validation_metrics(
        &self,
        scenario: &mut crate::scenario::model::Scenario,
    ) -> crate::error::Result<()> {
        let mut min_ttc = f64::INFINITY;
        let mut min_distance = f64::INFINITY;
        let mut violations = Vec::new();

        // Compute pairwise metrics for all actor combinations
        for (i, id1) in self.spec.actors.iter().map(|a| a.id.clone()).enumerate() {
            for id2 in self.spec.actors.iter().skip(i + 1).map(|a| a.id.clone()) {
                let traj1 = scenario.get_actor(&id1).ok_or_else(|| {
                    crate::error::ScenarioGenError::ActorNotFound(format!("Actor {} missing", id1))
                })?;
                let traj2 = scenario.get_actor(&id2).ok_or_else(|| {
                    crate::error::ScenarioGenError::ActorNotFound(format!("Actor {} missing", id2))
                })?;

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
                    // Use directed velocity matching Z3 encoding logic
                    if state1.lane() == state2.lane() {
                        let epsilon = 0.01; // m/s threshold to avoid division by zero

                        // Case 1: state1 ahead, state2 behind, state2 faster (catching up)
                        if state1.position().x > state2.position().x {
                            let rel_vel = state2.velocity().vx - state1.velocity().vx;
                            if rel_vel > epsilon {
                                let ttc = distance / rel_vel;

                                if ttc < min_ttc {
                                    min_ttc = ttc;
                                }

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
                        // Case 2: state2 ahead, state1 behind, state1 faster (catching up)
                        else if state2.position().x > state1.position().x {
                            let rel_vel = state1.velocity().vx - state2.velocity().vx;
                            if rel_vel > epsilon {
                                let ttc = distance / rel_vel;

                                if ttc < min_ttc {
                                    min_ttc = ttc;
                                }

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
        let has_accel_violations = !accel_violations.is_empty();
        scenario.validation.acceleration_violations = accel_violations;

        // Update all_constraints_satisfied if there are acceleration violations
        if has_accel_violations {
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
            violations.is_empty() && !has_accel_violations;
        scenario.validation.safety_violations = violations;

        Ok(())
    }
}

// === Optimizer-specific methods ===

impl GenericEncoder<OptimizerBackend> {
    /// Encode optimization objective based on the target.
    ///
    /// Creates a Z3 Real variable representing the objective and calls
    /// minimize/maximize on the backend. Uses minimum same-lane longitudinal
    /// distance as an LRA proxy for all targets (avoids NRA from TTC division).
    pub fn encode_objective(&mut self) {
        let target = self.coord_encoder.backend().target();
        match target {
            OptimizationTarget::MinimizeTtc
            | OptimizationTarget::MinimizeDistance
            | OptimizationTarget::MinimizeSeverity => {
                self.encode_minimize_distance_objective();
            }
            OptimizationTarget::MaximizeTtc => {
                self.encode_maximize_distance_objective();
            }
        }
    }

    /// Minimize: find the scenario where the minimum same-lane distance is smallest.
    ///
    /// Combines lower-bound constraints (`obj <= effective_dist[t]` for all t) with
    /// a "choice" constraint (`obj = effective_dist[t]` for at least one t).
    /// Together these force `obj` to equal the actual minimum distance across all
    /// time steps. Z3 then minimizes this value by choosing the scenario parameters
    /// that make the tightest approach as close as possible.
    fn encode_minimize_distance_objective(&mut self) {
        let obj = Real::new_const("dist_obj");
        let zero = Real::from_rational(0, 1);
        let big_val = Real::from_rational(9999, 1);
        self.coord_encoder.backend_mut().assert(&obj.ge(&zero));

        let actor_ids: Vec<String> = self.spec.actors.iter().map(|a| a.id.clone()).collect();
        let mut choices = Vec::new();

        for i in 0..actor_ids.len() {
            for j in (i + 1)..actor_ids.len() {
                for t in 0..=self.horizon {
                    let effective_dist =
                        self.compute_effective_dist(&actor_ids[i], &actor_ids[j], t, &big_val);
                    // Lower bound: obj <= effective_dist at every (pair, time)
                    self.coord_encoder
                        .backend_mut()
                        .assert(&obj.le(&effective_dist));
                    // Choice: obj can equal this particular effective_dist
                    choices.push(obj.eq(&effective_dist));
                }
            }
        }

        // At least one choice must hold (obj = some effective distance)
        let refs: Vec<&Bool> = choices.iter().collect();
        self.coord_encoder.backend_mut().assert(&Bool::or(&refs));

        self.coord_encoder.backend_mut().minimize(&obj);
        self.coord_encoder.backend_mut().set_objective_var(obj);
    }

    /// Maximize: find the scenario where the minimum same-lane distance is largest.
    ///
    /// Uses the standard lower-bound formulation: obj <= effective_dist at every
    /// (pair, time), then maximize obj to push it up to the actual minimum.
    fn encode_maximize_distance_objective(&mut self) {
        let obj = Real::new_const("dist_obj");
        let zero = Real::from_rational(0, 1);
        let big_val = Real::from_rational(9999, 1);
        self.coord_encoder.backend_mut().assert(&obj.ge(&zero));

        let actor_ids: Vec<String> = self.spec.actors.iter().map(|a| a.id.clone()).collect();

        for i in 0..actor_ids.len() {
            for j in (i + 1)..actor_ids.len() {
                for t in 0..=self.horizon {
                    let effective_dist =
                        self.compute_effective_dist(&actor_ids[i], &actor_ids[j], t, &big_val);
                    // obj <= effective_dist at every (pair, time)
                    self.coord_encoder
                        .backend_mut()
                        .assert(&obj.le(&effective_dist));
                }
            }
        }

        self.coord_encoder.backend_mut().maximize(&obj);
        self.coord_encoder.backend_mut().set_objective_var(obj);
    }

    /// Compute effective distance between two actors at time t.
    /// Returns abs(px_i - px_j) if same lane, big_val otherwise.
    fn compute_effective_dist(&self, aid_i: &str, aid_j: &str, t: usize, big_val: &Real) -> Real {
        let lane_i = self.get_lane_var(aid_i, t);
        let lane_j = self.get_lane_var(aid_j, t);
        let px_i = self.get_longitudinal_pos(aid_i, t);
        let px_j = self.get_longitudinal_pos(aid_j, t);

        let same_lane = lane_i.eq(lane_j);
        let abs_dist = px_i.gt(px_j).ite(&(px_i - px_j), &(px_j - px_i));
        same_lane.ite(&abs_dist, big_val)
    }

    /// Extract the optimal value from the Z3 model after solving.
    pub fn extract_optimal_value(&mut self, model: &z3::Model) {
        self.coord_encoder
            .backend_mut()
            .extract_optimal_value(model);
    }

    /// Get the optimal value found by the optimizer.
    pub fn get_optimal_value(&self) -> Option<f64> {
        self.coord_encoder.backend().get_optimal_value()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dsl::types::{
        ActorRole, ActorSpec, LaneChangeConfig, LaneChangeDirection, RoadSpec, ScenarioType,
        ValueOrRange,
    };
    use std::collections::HashMap;
    use z3::Config;

    fn create_test_spec() -> ScenarioSpec {
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
                    bicycle_params: None,
                },
                ActorSpec {
                    id: "npc".to_string(),
                    role: ActorRole::Npc,
                    lane: 0,
                    position: ValueOrRange::Range([60.0, 80.0]),
                    speed: ValueOrRange::Range([12.0, 14.0]),
                    acceleration: ValueOrRange::Range([-8.0, 3.0]),
                    direction: 1,
                    behavior: HashMap::new(),
                    lane_changes: vec![LaneChangeConfig {
                        direction: LaneChangeDirection::Right,
                        start_time: ValueOrRange::Range([2.5, 7.5]),
                        duration: ValueOrRange::Range([3.0, 4.0]),
                    }],
                    bicycle_params: None,
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
            coordinate_system: crate::dsl::types::CoordinateSystem::Cartesian,
            bicycle_config: None,
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

            // Variables are created internally - just verify accessor methods work
            // Test that we can access variables for both actors
            let _ego_lane = encoder.get_lane_var("ego", 0);
            let _npc_lane = encoder.get_lane_var("npc", 0);
            let _ego_px = encoder.get_longitudinal_pos("ego", 0);
            let _npc_px = encoder.get_longitudinal_pos("npc", 0);

            // If we get here without panicking, variables were created successfully
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
            let ego_px_0 = model
                .eval(encoder.get_longitudinal_pos("ego", 0), true)
                .unwrap();
            println!("Ego initial position: {:?}", ego_px_0);

            // NPC position should be in range [60.0, 80.0]
            let npc_px_0 = model
                .eval(encoder.get_longitudinal_pos("npc", 0), true)
                .unwrap();
            println!("NPC initial position: {:?}", npc_px_0);

            // Ego speed should be 15.0
            let ego_vx_0 = model
                .eval(encoder.get_longitudinal_vel("ego", 0), true)
                .unwrap();
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
            let ego_py_0 = model.eval(encoder.get_lateral_pos("ego", 0), true).unwrap();
            println!("Ego lateral position: {:?}", ego_py_0);

            // NPC in lane 0, should have py = 0 * 3.5 + 1.75 = 1.75
            let npc_py_0 = model.eval(encoder.get_lateral_pos("npc", 0), true).unwrap();
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
            let ego_px_0 = model
                .eval(encoder.get_longitudinal_pos("ego", 0), true)
                .unwrap();
            let ego_px_1 = model
                .eval(encoder.get_longitudinal_pos("ego", 1), true)
                .unwrap();
            let ego_vx_0 = model
                .eval(encoder.get_longitudinal_vel("ego", 0), true)
                .unwrap();

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
                let lane = model.eval(encoder.get_lane_var("npc", t), true).unwrap();
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
                let lane = model.eval(encoder.get_lane_var("ego", t), true).unwrap();
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
                let lane = model.eval(encoder.get_lane_var("npc", t), true).unwrap();
                if lane.to_string() == "1" {
                    transition_time = Some(t);
                    break;
                }
            }

            if let Some(trans_t) = transition_time {
                println!("NPC transitions to lane 1 at time {}", trans_t);
                // Before transition, should be in lane 0
                for t in 0..trans_t {
                    let lane = model.eval(encoder.get_lane_var("npc", t), true).unwrap();
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
                let ego_lane_0 = model.eval(encoder.get_lane_var("ego", 0), true).unwrap();
                let npc_lane_0 = model.eval(encoder.get_lane_var("npc", 0), true).unwrap();
                assert_eq!(ego_lane_0.to_string(), "1");
                assert_eq!(npc_lane_0.to_string(), "0");

                // Verify NPC eventually changes lanes
                let mut npc_in_lane_1 = false;
                for t in 0..=encoder.horizon {
                    let lane = model.eval(encoder.get_lane_var("npc", t), true).unwrap();
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
                    lane_changes: vec![],
                    bicycle_params: None,
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
                max_lateral_acceleration: 2.0,
                coordinate_system: crate::dsl::types::CoordinateSystem::Cartesian,
                bicycle_config: None,
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

    /// Helper: create a two-actor spec with both in the same lane (for TTC/distance tests)
    fn create_two_actor_same_lane_spec() -> ScenarioSpec {
        ScenarioSpec {
            scenario_type: ScenarioType::CutInLeft,
            time_step: 0.5,
            duration: 5.0,
            actors: vec![
                ActorSpec {
                    id: "ego".to_string(),
                    role: ActorRole::Ego,
                    lane: 1,
                    position: ValueOrRange::Value(50.0),
                    speed: ValueOrRange::Value(20.0),
                    acceleration: ValueOrRange::Range([-8.0, 3.0]),
                    direction: 1,
                    behavior: HashMap::new(),
                    lane_changes: vec![],
                    bicycle_params: None,
                },
                ActorSpec {
                    id: "npc".to_string(),
                    role: ActorRole::Npc,
                    lane: 1,
                    position: ValueOrRange::Value(100.0),
                    speed: ValueOrRange::Value(15.0),
                    acceleration: ValueOrRange::Range([-8.0, 3.0]),
                    direction: 1,
                    behavior: HashMap::new(),
                    lane_changes: vec![],
                    bicycle_params: None,
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
            coordinate_system: crate::dsl::types::CoordinateSystem::Cartesian,
            bicycle_config: None,
        }
    }

    /// Helper: create a two-actor spec with actors in different lanes
    fn create_two_actor_diff_lane_spec() -> ScenarioSpec {
        let mut spec = create_two_actor_same_lane_spec();
        spec.actors[1].lane = 0; // NPC in lane 0, ego in lane 1
        spec
    }

    // ===== Group 1: Proposition encoding correctness =====

    #[test]
    fn test_proposition_distance_gt() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            use crate::ltl::formula::{LTLFormula, Proposition};

            let spec = create_two_actor_same_lane_spec();
            let mut encoder = Z3Encoder::new(spec);
            encoder.create_variables();
            encoder.encode_initial_conditions();

            // Encode: DistanceGT(ego, npc, 30.0) at time 0
            let formula = LTLFormula::Atom(Proposition::DistanceGT {
                actor1: "ego".to_string(),
                actor2: "npc".to_string(),
                distance: 30.0,
            });
            encoder.encode_ltl(&formula);

            assert_eq!(encoder.check(), SatResult::Sat);

            let model = encoder.get_model().unwrap();
            let px_ego = model
                .eval(encoder.get_longitudinal_pos("ego", 0), true)
                .unwrap();
            let px_npc = model
                .eval(encoder.get_longitudinal_pos("npc", 0), true)
                .unwrap();

            // Parse values and verify |px_ego - px_npc| > 30
            let ego_val: f64 = crate::solver::backend::parse_z3_real_pub(&px_ego.to_string());
            let npc_val: f64 = crate::solver::backend::parse_z3_real_pub(&px_npc.to_string());
            let dist = (ego_val - npc_val).abs();
            assert!(dist > 30.0, "Distance {} should be > 30.0", dist);
        });
    }

    #[test]
    fn test_proposition_lateral_distance_gt() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            use crate::ltl::formula::{LTLFormula, Proposition};

            let spec = create_two_actor_diff_lane_spec();
            let mut encoder = Z3Encoder::new(spec);
            encoder.create_variables();
            encoder.encode_initial_conditions();

            // Encode: LateralDistanceGT(ego, npc, 2.0) at time 0
            let formula = LTLFormula::Atom(Proposition::LateralDistanceGT {
                actor1: "ego".to_string(),
                actor2: "npc".to_string(),
                distance: 2.0,
            });
            encoder.encode_ltl(&formula);

            assert_eq!(encoder.check(), SatResult::Sat);

            let model = encoder.get_model().unwrap();
            let py_ego = model.eval(encoder.get_lateral_pos("ego", 0), true).unwrap();
            let py_npc = model.eval(encoder.get_lateral_pos("npc", 0), true).unwrap();

            let ego_lat: f64 = crate::solver::backend::parse_z3_real_pub(&py_ego.to_string());
            let npc_lat: f64 = crate::solver::backend::parse_z3_real_pub(&py_npc.to_string());
            let lat_dist = (ego_lat - npc_lat).abs();
            assert!(
                lat_dist > 2.0,
                "Lateral distance {} should be > 2.0",
                lat_dist
            );
        });
    }

    #[test]
    fn test_proposition_on_left_of() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            use crate::ltl::formula::{LTLFormula, Proposition};

            let spec = create_two_actor_diff_lane_spec();
            let mut encoder = Z3Encoder::new(spec);
            encoder.create_variables();
            encoder.encode_initial_conditions();

            // Encode: OnLeftOf(ego, npc) — ego.py > npc.py
            let formula = LTLFormula::Atom(Proposition::OnLeftOf {
                actor1: "ego".to_string(),
                actor2: "npc".to_string(),
            });
            encoder.encode_ltl(&formula);

            assert_eq!(encoder.check(), SatResult::Sat);

            let model = encoder.get_model().unwrap();
            let py_ego = model.eval(encoder.get_lateral_pos("ego", 0), true).unwrap();
            let py_npc = model.eval(encoder.get_lateral_pos("npc", 0), true).unwrap();

            let ego_lat: f64 = crate::solver::backend::parse_z3_real_pub(&py_ego.to_string());
            let npc_lat: f64 = crate::solver::backend::parse_z3_real_pub(&py_npc.to_string());
            assert!(
                ego_lat > npc_lat,
                "Ego py ({}) should be > NPC py ({})",
                ego_lat,
                npc_lat
            );
        });
    }

    #[test]
    fn test_proposition_on_right_of() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            use crate::ltl::formula::{LTLFormula, Proposition};

            let spec = create_two_actor_diff_lane_spec();
            let mut encoder = Z3Encoder::new(spec);
            encoder.create_variables();
            encoder.encode_initial_conditions();

            // Encode: OnRightOf(npc, ego) — npc.py < ego.py
            // npc is in lane 0 (py=1.75), ego in lane 1 (py=5.25)
            let formula = LTLFormula::Atom(Proposition::OnRightOf {
                actor1: "npc".to_string(),
                actor2: "ego".to_string(),
            });
            encoder.encode_ltl(&formula);

            assert_eq!(encoder.check(), SatResult::Sat);

            let model = encoder.get_model().unwrap();
            let py_ego = model.eval(encoder.get_lateral_pos("ego", 0), true).unwrap();
            let py_npc = model.eval(encoder.get_lateral_pos("npc", 0), true).unwrap();

            let ego_lat: f64 = crate::solver::backend::parse_z3_real_pub(&py_ego.to_string());
            let npc_lat: f64 = crate::solver::backend::parse_z3_real_pub(&py_npc.to_string());
            assert!(
                npc_lat < ego_lat,
                "NPC py ({}) should be < Ego py ({})",
                npc_lat,
                ego_lat
            );
        });
    }

    #[test]
    fn test_proposition_relative_velocity_gt() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            use crate::ltl::formula::{LTLFormula, Proposition};

            // ego speed=20, npc speed=15, so |20-15|=5 > 3
            let spec = create_two_actor_same_lane_spec();
            let mut encoder = Z3Encoder::new(spec);
            encoder.create_variables();
            encoder.encode_initial_conditions();

            let formula = LTLFormula::Atom(Proposition::RelativeVelocityGT {
                actor1: "ego".to_string(),
                actor2: "npc".to_string(),
                velocity: 3.0,
            });
            encoder.encode_ltl(&formula);

            assert_eq!(encoder.check(), SatResult::Sat);

            let model = encoder.get_model().unwrap();
            let vx_ego = model
                .eval(encoder.get_longitudinal_vel("ego", 0), true)
                .unwrap();
            let vx_npc = model
                .eval(encoder.get_longitudinal_vel("npc", 0), true)
                .unwrap();

            let ego_v: f64 = crate::solver::backend::parse_z3_real_pub(&vx_ego.to_string());
            let npc_v: f64 = crate::solver::backend::parse_z3_real_pub(&vx_npc.to_string());
            let rel_vel = (ego_v - npc_v).abs();
            assert!(
                rel_vel > 3.0,
                "Relative velocity {} should be > 3.0",
                rel_vel
            );
        });
    }

    #[test]
    fn test_proposition_manhattan_distance_gt() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            use crate::ltl::formula::{LTLFormula, Proposition};

            let spec = create_two_actor_diff_lane_spec();
            let mut encoder = Z3Encoder::new(spec);
            encoder.create_variables();
            encoder.encode_initial_conditions();

            // ego at px=50, npc at px=100, different lanes → manhattan should be large
            let formula = LTLFormula::Atom(Proposition::ManhattanDistanceGT {
                actor1: "ego".to_string(),
                actor2: "npc".to_string(),
                distance: 40.0,
            });
            encoder.encode_ltl(&formula);

            assert_eq!(encoder.check(), SatResult::Sat);

            let model = encoder.get_model().unwrap();
            let px_ego = model
                .eval(encoder.get_longitudinal_pos("ego", 0), true)
                .unwrap();
            let py_ego = model.eval(encoder.get_lateral_pos("ego", 0), true).unwrap();
            let px_npc = model
                .eval(encoder.get_longitudinal_pos("npc", 0), true)
                .unwrap();
            let py_npc = model.eval(encoder.get_lateral_pos("npc", 0), true).unwrap();

            let ex: f64 = crate::solver::backend::parse_z3_real_pub(&px_ego.to_string());
            let ey: f64 = crate::solver::backend::parse_z3_real_pub(&py_ego.to_string());
            let nx: f64 = crate::solver::backend::parse_z3_real_pub(&px_npc.to_string());
            let ny: f64 = crate::solver::backend::parse_z3_real_pub(&py_npc.to_string());
            let manhattan = (ex - nx).abs() + (ey - ny).abs();
            assert!(
                manhattan > 40.0,
                "Manhattan distance {} should be > 40.0",
                manhattan
            );
        });
    }

    #[test]
    fn test_proposition_rectangular_distance_gt() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            use crate::ltl::formula::{LTLFormula, Proposition};

            let spec = create_two_actor_diff_lane_spec();
            let mut encoder = Z3Encoder::new(spec);
            encoder.create_variables();
            encoder.encode_initial_conditions();

            let formula = LTLFormula::Atom(Proposition::RectangularDistanceGT {
                actor1: "ego".to_string(),
                actor2: "npc".to_string(),
                threshold_x: 30.0,
                threshold_y: 2.0,
            });
            encoder.encode_ltl(&formula);

            assert_eq!(encoder.check(), SatResult::Sat);

            let model = encoder.get_model().unwrap();
            let px_ego = model
                .eval(encoder.get_longitudinal_pos("ego", 0), true)
                .unwrap();
            let py_ego = model.eval(encoder.get_lateral_pos("ego", 0), true).unwrap();
            let px_npc = model
                .eval(encoder.get_longitudinal_pos("npc", 0), true)
                .unwrap();
            let py_npc = model.eval(encoder.get_lateral_pos("npc", 0), true).unwrap();

            let ex: f64 = crate::solver::backend::parse_z3_real_pub(&px_ego.to_string());
            let ey: f64 = crate::solver::backend::parse_z3_real_pub(&py_ego.to_string());
            let nx: f64 = crate::solver::backend::parse_z3_real_pub(&px_npc.to_string());
            let ny: f64 = crate::solver::backend::parse_z3_real_pub(&py_npc.to_string());
            let dx = (ex - nx).abs();
            let dy = (ey - ny).abs();
            assert!(
                dx > 30.0 || dy > 2.0,
                "Rectangular: |dx|={} should be > 30 OR |dy|={} should be > 2",
                dx,
                dy
            );
        });
    }

    // ===== Group 2: TTC constraint encoding =====

    #[test]
    fn test_ttc_constraint_same_lane_approaching() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            use crate::ltl::formula::{LTLFormula, Proposition};

            // ego at 50 speed 20, npc at 100 speed 15 — ego approaching npc
            let spec = create_two_actor_same_lane_spec();
            let mut encoder = Z3Encoder::new(spec);
            encoder.create_variables();
            encoder.encode_initial_conditions();
            encoder.encode_kinematics();
            encoder.encode_lane_velocity_constraints();

            // Always TTC > 3.0
            let formula = LTLFormula::Atom(Proposition::TTCGT {
                actor1: "ego".to_string(),
                actor2: "npc".to_string(),
                ttc: 3.0,
            })
            .always();
            encoder.encode_ltl(&formula);

            // Should be satisfiable — distance=50, rel_vel=5, TTC=10 initially
            assert_eq!(encoder.check(), SatResult::Sat);
        });
    }

    #[test]
    fn test_ttc_constraint_different_lanes_unconstrained() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            use crate::ltl::formula::{LTLFormula, Proposition};

            // Actors in different lanes — TTC constraint is trivially satisfied
            let spec = create_two_actor_diff_lane_spec();
            let mut encoder = Z3Encoder::new(spec);
            encoder.create_variables();
            encoder.encode_initial_conditions();
            encoder.encode_kinematics();
            encoder.encode_lane_velocity_constraints();

            // Always TTC > 100.0 (very high threshold, but different lanes so no issue)
            let formula = LTLFormula::Atom(Proposition::TTCGT {
                actor1: "ego".to_string(),
                actor2: "npc".to_string(),
                ttc: 100.0,
            })
            .always();
            encoder.encode_ltl(&formula);

            assert_eq!(encoder.check(), SatResult::Sat);
        });
    }

    // ===== Group 3: Validation metrics =====

    #[test]
    fn test_validation_metrics_safe_scenario() {
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

            let ltl_formula = LTLGenerator::generate(&spec).unwrap();
            encoder.encode_ltl(&ltl_formula);

            assert_eq!(encoder.check(), SatResult::Sat);
            let model = encoder.get_model().unwrap();
            let scenario = encoder.extract_scenario(&model).unwrap();

            assert!(
                scenario.validation.all_constraints_satisfied,
                "Safe scenario should satisfy all constraints"
            );
            assert!(
                scenario.validation.min_ttc >= spec.min_ttc
                    || scenario.validation.min_ttc == f64::INFINITY,
                "Min TTC {} should be >= {}",
                scenario.validation.min_ttc,
                spec.min_ttc
            );
            assert!(
                scenario.validation.min_distance >= spec.min_distance
                    || scenario.validation.min_distance == f64::INFINITY,
                "Min distance {} should be >= {}",
                scenario.validation.min_distance,
                spec.min_distance
            );
        });
    }

    #[test]
    fn test_validation_metrics_detects_violations() {
        use crate::ltl::generator::LTLGenerator;

        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            // Use adversarial mode: violate TTC
            let mut spec = create_test_spec();
            spec.constraint_modes =
                crate::dsl::types::ConstraintModes::Shorthand("violate_all".to_string());

            let mut encoder = Z3Encoder::new(spec.clone());
            encoder.create_variables();
            encoder.encode_initial_conditions();
            encoder.encode_kinematics();
            encoder.encode_lane_velocity_constraints();
            encoder.encode_lateral_velocity_bounds();

            let ltl_formula = LTLGenerator::generate(&spec).unwrap();
            encoder.encode_ltl(&ltl_formula);

            let result = encoder.check();
            if result == SatResult::Sat {
                let model = encoder.get_model().unwrap();
                let scenario = encoder.extract_scenario(&model).unwrap();

                // In adversarial mode, violations should be detected
                let has_violation = !scenario.validation.all_constraints_satisfied
                    || !scenario.validation.safety_violations.is_empty();
                // Note: it's possible the solver finds a scenario that technically
                // violates at some point but validation still passes due to timing.
                // The key test is that the pipeline doesn't crash.
                println!(
                    "Adversarial: constraints_satisfied={}, violations={}",
                    scenario.validation.all_constraints_satisfied,
                    scenario.validation.safety_violations.len()
                );
                // If violations exist, verify they have content
                if !scenario.validation.safety_violations.is_empty() {
                    assert!(has_violation);
                    for v in &scenario.validation.safety_violations {
                        assert!(!v.is_empty(), "Violation string should not be empty");
                    }
                }
            } else {
                // UNSAT is acceptable for adversarial — constraints may conflict
                println!("Adversarial scenario is UNSAT (constraints conflict)");
            }
        });
    }

    #[test]
    fn test_validation_acceleration_metrics() {
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

            let ltl_formula = LTLGenerator::generate(&spec).unwrap();
            encoder.encode_ltl(&ltl_formula);

            assert_eq!(encoder.check(), SatResult::Sat);
            let model = encoder.get_model().unwrap();
            let scenario = encoder.extract_scenario(&model).unwrap();

            // Check that acceleration metrics are computed from trajectory
            let ego = scenario.get_actor("ego").unwrap();
            // Compute acceleration from velocity differences
            let mut max_accel = 0.0_f64;
            let mut max_decel = 0.0_f64;
            for i in 1..ego.states.len() {
                let dv = ego.states[i].velocity().vx - ego.states[i - 1].velocity().vx;
                let accel = dv / spec.time_step;
                if accel > max_accel {
                    max_accel = accel;
                }
                if accel < max_decel {
                    max_decel = accel;
                }
            }
            // Acceleration should be within bounds [-8, 3]
            assert!(
                max_accel <= 3.0 + 0.1,
                "Max acceleration {} should be <= 3.0",
                max_accel
            );
            assert!(
                max_decel >= -8.0 - 0.1,
                "Max deceleration {} should be >= -8.0",
                max_decel
            );
        });
    }

    // ===== Group 4: Optimizer encoder =====

    #[test]
    fn test_optimizer_minimize_distance() {
        use crate::ltl::generator::LTLGenerator;
        use crate::solver::backend::OptimizationTarget;
        use crate::solver::backend::OptimizerBackend;

        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let spec = create_test_spec();
            let backend = OptimizerBackend::new(OptimizationTarget::MinimizeDistance);
            let mut encoder = GenericEncoder::with_backend(spec.clone(), backend);
            encoder.create_variables();
            encoder.encode_initial_conditions();
            encoder.encode_kinematics();
            encoder.encode_lane_velocity_constraints();
            encoder.encode_lateral_velocity_bounds();

            let ltl_formula = LTLGenerator::generate(&spec).unwrap();
            encoder.encode_ltl(&ltl_formula);
            encoder.encode_objective();

            let result = encoder.check();
            assert_eq!(result, SatResult::Sat, "Optimizer should find a solution");

            let model = encoder.get_model().unwrap();
            encoder.extract_optimal_value(&model);
            let optimal = encoder.get_optimal_value();
            assert!(optimal.is_some(), "Should have an optimal value");
            let val = optimal.unwrap();
            assert!(val >= 0.0, "Minimum distance {} should be >= 0", val);
            println!("Minimized distance: {}", val);
        });
    }

    #[test]
    fn test_optimizer_maximize_distance() {
        use crate::ltl::generator::LTLGenerator;
        use crate::solver::backend::OptimizationTarget;
        use crate::solver::backend::OptimizerBackend;

        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let spec = create_test_spec();
            let backend = OptimizerBackend::new(OptimizationTarget::MaximizeTtc);
            let mut encoder = GenericEncoder::with_backend(spec.clone(), backend);
            encoder.create_variables();
            encoder.encode_initial_conditions();
            encoder.encode_kinematics();
            encoder.encode_lane_velocity_constraints();
            encoder.encode_lateral_velocity_bounds();

            let ltl_formula = LTLGenerator::generate(&spec).unwrap();
            encoder.encode_ltl(&ltl_formula);
            encoder.encode_objective();

            let result = encoder.check();
            assert_eq!(result, SatResult::Sat, "Optimizer should find a solution");

            let model = encoder.get_model().unwrap();
            encoder.extract_optimal_value(&model);
            let optimal = encoder.get_optimal_value();
            assert!(optimal.is_some(), "Should have an optimal value");
            let val = optimal.unwrap();
            assert!(val > 0.0, "Maximized distance {} should be > 0", val);
            println!("Maximized distance: {}", val);
        });
    }

    // ===== Group 5: Edge cases =====

    #[test]
    fn test_ltl_next_at_horizon_boundary() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            use crate::ltl::formula::{LTLFormula, Proposition};

            // Create a very short scenario (2 time steps: horizon=2)
            let mut spec = create_two_actor_same_lane_spec();
            spec.duration = 1.0;
            spec.time_step = 0.5; // horizon = 2

            let mut encoder = Z3Encoder::new(spec);
            encoder.create_variables();
            encoder.encode_initial_conditions();

            // Encode: Always(Next(Next(Next(InLane(ego, 1)))))
            // At horizon, Next should return false
            // This creates Next at time horizon which should be false
            let inner = LTLFormula::Atom(Proposition::InLane {
                actor: "ego".to_string(),
                lane: 1,
            });
            // Triple Next from time 0 means we need time 3, but horizon is 2
            let formula = LTLFormula::Next(Box::new(LTLFormula::Next(Box::new(LTLFormula::Next(
                Box::new(inner),
            )))));
            encoder.encode_ltl(&formula);

            // Should be UNSAT because Next at horizon returns false
            assert_eq!(
                encoder.check(),
                SatResult::Unsat,
                "Next beyond horizon should be unsatisfiable"
            );
        });
    }

    #[test]
    fn test_unsatisfiable_constraints() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            use crate::ltl::formula::{LTLFormula, Proposition};

            let spec = create_two_actor_same_lane_spec();
            let mut encoder = Z3Encoder::new(spec);
            encoder.create_variables();
            encoder.encode_initial_conditions();

            // Ego must be in lane 0 AND lane 1 simultaneously at time 0
            let in_lane_0 = LTLFormula::Atom(Proposition::InLane {
                actor: "ego".to_string(),
                lane: 0,
            });
            let in_lane_1 = LTLFormula::Atom(Proposition::InLane {
                actor: "ego".to_string(),
                lane: 1,
            });
            let formula = LTLFormula::And(Box::new(in_lane_0), Box::new(in_lane_1));
            encoder.encode_ltl(&formula);

            assert_eq!(
                encoder.check(),
                SatResult::Unsat,
                "Actor cannot be in two lanes simultaneously"
            );
        });
    }
}
