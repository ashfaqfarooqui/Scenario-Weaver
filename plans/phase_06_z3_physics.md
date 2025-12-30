# Phase 6: Z3 Physics

**Prerequisites**: Phase 5 complete (Z3 foundation, variables created)

**Duration**: 1 day

---

## Context

Physics constraints ensure scenarios are physically plausible. We encode kinematic equations that govern how positions and velocities evolve over time.

**Why this phase**: Without physics, Z3 could generate impossible scenarios (teleportation, infinite acceleration). Kinematics enforces realistic motion.

**What problem it solves**: Ensures generated scenarios obey laws of motion for constant-velocity model (MVP uses simplified physics).

---

## Goals

- [ ] Implement constant velocity kinematics
- [ ] Encode position updates: `x[t+1] = x[t] + v[t] * dt`
- [ ] Encode ego constant velocity (no acceleration)
- [ ] Encode NPC constant velocity (except during lane change)
- [ ] Encode lane-position coupling for all time steps
- [ ] Test physics constraints

---

## Implementation (Add to `src/solver/encoder.rs`)

```rust
// Add this method to Z3Encoder impl block

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
```

Add test:

```rust
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
}
```

---

## Success Criteria

- [ ] Kinematics method compiles
- [ ] Test passes showing SAT
- [ ] Positions evolve according to velocity
- [ ] Velocities remain constant
- [ ] Lane-position coupling maintained

---

## Next Phase

**→ Continue to [Phase 7: Z3 LTL Encoding](phase_07_z3_ltl.md)**
