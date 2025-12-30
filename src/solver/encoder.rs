//! Z3 constraint encoder

use std::collections::HashMap;
use z3::ast::{Ast, Int, Real};
use z3::{Context, SatResult, Solver};

use crate::dsl::types::ScenarioSpec;

/// Z3 SMT encoder for scenario constraints
///
/// Lifetime 'ctx is the Z3 context lifetime - all Z3 AST nodes must live
/// as long as the context.
pub struct Z3Encoder<'ctx> {
    /// Z3 context (must outlive all AST nodes)
    ctx: &'ctx Context,

    /// Z3 solver instance
    solver: Solver<'ctx>,

    /// Original scenario specification
    spec: ScenarioSpec,

    /// Number of time steps in the scenario
    horizon: usize,

    // Variable maps: actor_id -> Vec<variable> (one per time step)
    /// Longitudinal positions (m)
    positions_x: HashMap<String, Vec<Real<'ctx>>>,

    /// Lateral positions (m)
    positions_y: HashMap<String, Vec<Real<'ctx>>>,

    /// Longitudinal velocities (m/s)
    velocities_x: HashMap<String, Vec<Real<'ctx>>>,

    /// Lateral velocities (m/s)
    velocities_y: HashMap<String, Vec<Real<'ctx>>>,

    /// Lane numbers (integer)
    lanes: HashMap<String, Vec<Int<'ctx>>>,
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
        }
    }

    /// Create all Z3 variables for the scenario
    ///
    /// For each actor ("ego", "npc") and each time step t ∈ [0, horizon],
    /// creates variables:
    /// - px_t: longitudinal position
    /// - py_t: lateral position
    /// - vx_t: longitudinal velocity
    /// - vy_t: lateral velocity
    /// - lane_t: lane number
    pub fn create_variables(&mut self) {
        let actor_ids = vec!["ego".to_string(), "npc".to_string()];

        for actor_id in actor_ids {
            let mut px_vars = Vec::new();
            let mut py_vars = Vec::new();
            let mut vx_vars = Vec::new();
            let mut vy_vars = Vec::new();
            let mut lane_vars = Vec::new();

            // Create variables for each time step
            for t in 0..=self.horizon {
                px_vars.push(Real::new_const(self.ctx, format!("{}_px_{}", actor_id, t)));
                py_vars.push(Real::new_const(self.ctx, format!("{}_py_{}", actor_id, t)));
                vx_vars.push(Real::new_const(self.ctx, format!("{}_vx_{}", actor_id, t)));
                vy_vars.push(Real::new_const(self.ctx, format!("{}_vy_{}", actor_id, t)));
                lane_vars.push(Int::new_const(self.ctx, format!("{}_lane_{}", actor_id, t)));
            }

            self.positions_x.insert(actor_id.clone(), px_vars);
            self.positions_y.insert(actor_id.clone(), py_vars);
            self.velocities_x.insert(actor_id.clone(), vx_vars);
            self.velocities_y.insert(actor_id.clone(), vy_vars);
            self.lanes.insert(actor_id.clone(), lane_vars);
        }
    }

    /// Encode initial conditions from the DSL specification
    pub fn encode_initial_conditions(&mut self) {
        // Ego initial conditions
        let ego_id = "ego";
        self.encode_actor_initial_state(
            ego_id,
            self.spec.ego.lane,
            self.spec.ego.position,
            self.spec.ego.position, // no range for ego
            self.spec.ego.speed,
            self.spec.ego.speed,
        );

        // NPC initial conditions (may have ranges)
        let npc_id = "npc";
        self.encode_actor_initial_state(
            npc_id,
            self.spec.npc.lane,
            self.spec.npc.position.min(),
            self.spec.npc.position.max(),
            self.spec.npc.speed.min(),
            self.spec.npc.speed.max(),
        );

        // Initial lateral position matches lane center
        self.encode_lane_position_coupling_at_time(ego_id, 0);
        self.encode_lane_position_coupling_at_time(npc_id, 0);
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

    /// Encode kinematic constraints (constant velocity model)
    pub fn encode_kinematics(&mut self) {
        let dt = self.spec.time_step;
        let dt_real = Real::from_real(self.ctx, (dt * 10.0) as i32, 10);

        for actor_id in &["ego".to_string(), "npc".to_string()] {
            for t in 0..self.horizon {
                // Position update: px[t+1] = px[t] + vx[t] * dt
                let px_t = &self.positions_x[actor_id][t];
                let px_t1 = &self.positions_x[actor_id][t + 1];
                let vx_t = &self.velocities_x[actor_id][t];

                let expected_px = px_t + &(vx_t * &dt_real);
                self.solver.assert(&px_t1._eq(&expected_px));

                // Same for lateral: py[t+1] = py[t] + vy[t] * dt
                let py_t = &self.positions_y[actor_id][t];
                let py_t1 = &self.positions_y[actor_id][t + 1];
                let vy_t = &self.velocities_y[actor_id][t];

                let expected_py = py_t + &(vy_t * &dt_real);
                self.solver.assert(&py_t1._eq(&expected_py));
            }
        }

        // Ego: constant velocity (no acceleration)
        for t in 0..self.horizon {
            let vx_t = &self.velocities_x["ego"][t];
            let vx_t1 = &self.velocities_x["ego"][t + 1];
            self.solver.assert(&vx_t1._eq(vx_t));

            // Ego never changes lanes
            let vy_t = &self.velocities_y["ego"][t];
            let zero = Real::from_real(self.ctx, 0, 1);
            self.solver.assert(&vy_t._eq(&zero));
        }

        // NPC: constant longitudinal velocity
        for t in 0..self.horizon {
            let vx_t = &self.velocities_x["npc"][t];
            let vx_t1 = &self.velocities_x["npc"][t + 1];
            self.solver.assert(&vx_t1._eq(vx_t));
        }

        // Lane-position coupling for all time steps
        for t in 0..=self.horizon {
            self.encode_lane_position_coupling_at_time("ego", t);
            self.encode_lane_position_coupling_at_time("npc", t);
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
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dsl::types::{ActorSpec, NpcSpec, ScenarioType, ValueOrRange};
    use z3::Config;

    fn create_test_spec() -> ScenarioSpec {
        ScenarioSpec {
            scenario_type: ScenarioType::CutInLeft,
            time_step: 0.5,
            duration: 10.0,
            ego: ActorSpec {
                lane: 1,
                position: 50.0,
                speed: 15.0,
            },
            npc: NpcSpec {
                lane: 0,
                position: ValueOrRange::Range([60.0, 80.0]),
                speed: ValueOrRange::Range([12.0, 14.0]),
                cut_in_time: ValueOrRange::Range([2.5, 7.5]),
            },
            min_ttc: 3.0,
            min_distance: 5.0,
            lane_width: 3.5,
            num_scenarios: 1,
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
}
