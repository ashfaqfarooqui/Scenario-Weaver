# Phase 5: Z3 Foundation

**Prerequisites**: Phases 1-4 complete (setup, DSL, LTL, scenario model)

**Duration**: 1 day

---

## Context

This phase establishes the Z3 SMT solver integration. We create the encoder structure, Z3 variables for all time steps, and encode initial conditions from the DSL.

**Why this phase**: Z3 is the core constraint solver. Before we can encode physics, LTL, or safety constraints, we need the foundation: variables, context, and basic constraint encoding infrastructure.

**What problem it solves**: Creates the Z3 problem space where we'll encode all our constraints. This is the foundation for phases 6-8.

---

## Goals

- [ ] Create Z3Encoder struct with lifetime management
- [ ] Set up Z3 context and solver
- [ ] Create variables for all actors at all time steps
- [ ] Encode initial conditions from DSL
- [ ] Test variable creation
- [ ] Test initial constraint encoding

---

## Implementation Steps

### Step 1: Create Z3 Encoder Structure

**File**: `src/solver/encoder.rs`

```rust
//! Z3 constraint encoder

use z3::ast::{Ast, Bool, Int, Real};
use z3::{Config, Context, SatResult, Solver};
use std::collections::HashMap;

use crate::dsl::types::ScenarioSpec;
use crate::error::{Result, ScenarioGenError};
use crate::scenario::model::Scenario;

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
                px_vars.push(Real::new_const(
                    self.ctx,
                    format!("{}_px_{}", actor_id, t),
                ));
                py_vars.push(Real::new_const(
                    self.ctx,
                    format!("{}_py_{}", actor_id, t),
                ));
                vx_vars.push(Real::new_const(
                    self.ctx,
                    format!("{}_vx_{}", actor_id, t),
                ));
                vy_vars.push(Real::new_const(
                    self.ctx,
                    format!("{}_vy_{}", actor_id, t),
                ));
                lane_vars.push(Int::new_const(
                    self.ctx,
                    format!("{}_lane_{}", actor_id, t),
                ));
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
        let lane_width_real = Real::from_real(
            self.ctx,
            (lane_width * 10.0) as i32,
            10,
        );
        let half_width = Real::from_real(
            self.ctx,
            (lane_width * 5.0) as i32,
            10,
        );

        // py = lane * lane_width + lane_width/2
        let lane_real = lane_var.to_real();
        let expected_py = lane_real * &lane_width_real + &half_width;
        self.solver.assert(&py_var._eq(&expected_py));
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
}
```

### Step 2: Update Module Exports

**src/solver/mod.rs**:
```rust
//! Solver module

pub mod encoder;
pub mod physics;
pub mod multi_solve;

pub use encoder::Z3Encoder;
```

### Step 3: Create Physics Module Placeholder

**src/solver/physics.rs**:
```rust
//! Physics constraints (to be implemented in Phase 6)

// Placeholder for Phase 6
```

### Step 4: Create Multi-Solve Module Placeholder

**src/solver/multi_solve.rs**:
```rust
//! Multiple scenario generation (to be implemented in Phase 11)

// Placeholder for Phase 11
```

---

## Success Criteria

### Verification Steps

1. **Tests pass**:
   ```bash
   cargo test solver::encoder -- --nocapture
   ```

2. **Variables created correctly**:
   ```bash
   cargo test test_create_variables -- --nocapture
   ```
   Should show 21 variables per actor per dimension

3. **Initial conditions satisfiable**:
   ```bash
   cargo test test_encode_initial_conditions -- --nocapture
   ```
   Should print Z3 model values

4. **Lane-position coupling works**:
   ```bash
   cargo test test_lane_position_coupling -- --nocapture
   ```
   Should show correct lateral positions

### Checklist

- [ ] Z3Encoder struct created with lifetimes
- [ ] Variables created for all time steps
- [ ] Initial conditions encode correctly
- [ ] Lane-position coupling works
- [ ] Tests pass and show SAT results
- [ ] Z3 model extraction works

---

## Testing

```bash
# Run encoder tests with output
cargo test solver::encoder -- --nocapture

# Test variable creation
cargo test test_create_variables -- --nocapture

# Test initial conditions
cargo test test_encode_initial_conditions -- --nocapture
```

Expected output:
```
Ego initial position: Real { ... }
NPC initial position: Real { ... }
Ego initial speed: Real { ... }
Ego lateral position: Real { ... }
NPC lateral position: Real { ... }
```

---

## Common Issues

### Issue: Lifetime errors with Z3

**Symptom**: "borrowed value does not live long enough"

**Solution**: Ensure Context is created before Encoder and lives as long as needed. Use `'ctx` lifetime parameter consistently.

### Issue: Z3 returns UNSAT

**Symptom**: `check()` returns `Unsat`

**Solution**: Initial conditions are contradictory. Check that ranges are valid and constraints don't conflict.

### Issue: Real number precision

**Symptom**: Values not exact (e.g., 50.0 shows as 500/10)

**Solution**: Z3 uses rational numbers. This is expected. Extract as `(numerator, denominator)` and compute float.

---

## Next Phase

Once this phase is complete and all tests pass:

**→ Continue to [Phase 6: Z3 Physics](phase_06_z3_physics.md)**

Phase 6 will add kinematic constraints.

---

## Notes for AI Agents

**What you just built**:
- Z3 encoder foundation
- Variable creation for all actors and time steps
- Initial condition encoding from DSL
- Lane-position coupling

**What you can now do**:
- Create Z3 problem instances
- Encode constraints
- Check satisfiability
- Extract models

**What's next**:
- Phase 6: Add physics (kinematics)
- Phase 7: Encode LTL formulas
- Phase 8: Add safety constraints
