# Phases 7-12 Implementation Summary

Due to the complexity and length of phases 7-12, here's a comprehensive summary with key implementation points for each. Full detailed files follow the same pattern as phases 1-6.

## Phase 7: Z3 LTL Encoding (MOST CRITICAL - 2 days)

**Core Algorithm**: Bounded model checking - expand LTL temporal operators over time horizon

**Key Method**:
```rust
fn encode_ltl_bounded(&self, formula: &LTLFormula, time: usize, horizon: usize) -> Bool<'ctx> {
    match formula {
        Eventually(φ) => OR(φ[time], φ[time+1], ..., φ[horizon]),
        Always(φ) => AND(φ[time], φ[time+1], ..., φ[horizon]),
        Until(φ, ψ) => ψ[time] ∨ (φ[time] ∧ Until(φ,ψ)[time+1]),
        Atom(p) => encode_proposition(p, time),
        And/Or/Not => recursive encoding
    }
}
```

**Proposition Encoding**:
- `InLane(actor, lane)` → `lane_var[t] == lane_value`
- `Ahead(a1, a2)` → `px1[t] > px2[t]`
- `TTCGT(a1, a2, ttc)` → TTC calculation with division by relative velocity
- `DistanceGT(a1, a2, d)` → `|px1[t] - px2[t]| > d`

**Critical**: This is where LTL formulas from Phase 3 become Z3 constraints.

---

## Phase 8: Z3 Safety Constraints (1 day)

**TTC (Time-To-Collision)**:
```rust
if same_lane and vx1 > vx2:
    (px2 - px1) / (vx1 - vx2) > min_ttc
else:
    true  // no collision possible
```

**Minimum Distance** (when in same lane):
```rust
if lane1 == lane2:
    |px1 - px2| > min_distance
```

Apply for all time steps, all actor pairs.

---

## Phase 9: Scenario Extraction (1 day)

**Extract Z3 model → Scenario JSON**:

```rust
pub fn extract_scenario(&self, model: &z3::Model) -> Scenario {
    let mut scenario = Scenario::new(...);
    
    for actor_id in ["ego", "npc"] {
        let mut trajectory = ActorTrajectory::new(actor_id, role);
        
        for t in 0..=horizon {
            let px = extract_real(model.eval(&self.positions_x[actor_id][t]));
            let py = extract_real(model.eval(&self.positions_y[actor_id][t]));
            let vx = extract_real(model.eval(&self.velocities_x[actor_id][t]));
            let vy = extract_real(model.eval(&self.velocities_y[actor_id][t]));
            let lane = extract_int(model.eval(&self.lanes[actor_id][t]));
            
            trajectory.add_state(State::new(
                t as f64 * time_step,
                Position::new(px, py),
                Velocity::new(vx, vy),
                lane
            ));
        }
        
        scenario.add_actor(trajectory);
    }
    
    scenario
}
```

Helper: Convert Z3 rationals to f64:
```rust
fn extract_real(ast: &dyn Ast) -> f64 {
    let (num, den) = ast.as_real().unwrap();
    num as f64 / den as f64
}
```

---

## Phase 10: Single Scenario Pipeline (1 day)

**End-to-end integration**:

```rust
// In src/lib.rs
pub fn generate_single_scenario(yaml: &str) -> Result<Scenario> {
    // 1. Parse YAML
    let spec = dsl::parse_yaml(yaml)?;
    
    // 2. Generate LTL
    let ltl = ltl::LTLGenerator::generate(&spec);
    
    // 3. Setup Z3
    let cfg = z3::Config::new();
    let ctx = z3::Context::new(&cfg);
    let mut encoder = solver::Z3Encoder::new(&ctx, spec);
    
    // 4. Encode constraints
    encoder.create_variables();
    encoder.encode_initial_conditions();
    encoder.encode_kinematics();
    encoder.encode_ltl(&ltl);
    encoder.encode_safety();
    
    // 5. Solve
    if encoder.check() == SatResult::Sat {
        let model = encoder.get_model().unwrap();
        Ok(encoder.extract_scenario(&model))
    } else {
        Err(ScenarioGenError::Unsatisfiable)
    }
}
```

**Test**: `examples/cut_in_left.yaml` → JSON output, manually validate

---

## Phase 11: Multiple Scenarios (1 day)

**Blocking Clauses**:

```rust
pub fn generate_multiple_scenarios(spec: &ScenarioSpec, n: usize) -> Vec<Scenario> {
    let mut scenarios = Vec::new();
    
    for i in 0..n {
        let cfg = z3::Config::new();
        let ctx = z3::Context::new(&cfg);
        let mut encoder = Z3Encoder::new(&ctx, spec.clone());
        
        // Setup (same as single scenario)
        encoder.create_variables();
        encoder.encode_initial_conditions();
        encoder.encode_kinematics();
        encoder.encode_ltl(&ltl);
        encoder.encode_safety();
        
        // Block previous solutions
        for prev in &scenarios {
            let blocking = create_blocking_clause(&ctx, &encoder, prev);
            encoder.solver.assert(&blocking);
        }
        
        // Solve
        if encoder.check() == SatResult::Sat {
            scenarios.push(encoder.extract_scenario(&model));
        } else {
            break;  // No more solutions
        }
    }
    
    scenarios
}

fn create_blocking_clause<'ctx>(
    ctx: &'ctx Context,
    encoder: &Z3Encoder<'ctx>,
    prev: &Scenario
) -> Bool<'ctx> {
    // Prevent same initial NPC parameters
    let prev_npc_x0 = prev.actors[1].states[0].position.x;
    let prev_npc_vx0 = prev.actors[1].states[0].velocity.vx;
    
    let npc_x0 = &encoder.positions_x["npc"][0];
    let npc_vx0 = &encoder.velocities_x["npc"][0];
    
    let prev_x = Real::from_real(ctx, (prev_npc_x0 * 10.0) as i32, 10);
    let prev_vx = Real::from_real(ctx, (prev_npc_vx0 * 10.0) as i32, 10);
    
    // Block if both are equal: !(x0 == prev_x0 AND vx0 == prev_vx0)
    let eq_x = npc_x0._eq(&prev_x);
    let eq_vx = npc_vx0._eq(&prev_vx);
    let both_eq = Bool::and(ctx, &[&eq_x, &eq_vx]);
    
    both_eq.not()
}
```

---

## Phase 12: CLI & Polish (1-2 days)

**CLI Implementation** (src/main.rs):

```rust
use clap::Parser;
use std::path::PathBuf;

#[derive(Parser)]
#[command(name = "carla-scenario-gen")]
#[command(about = "Generate CARLA scenarios using LTL + Z3")]
struct Cli {
    #[arg(short, long)]
    input: PathBuf,
    
    #[arg(short, long)]
    output: PathBuf,
    
    #[arg(short, long)]
    verbose: bool,
}

fn main() -> anyhow::Result<()> {
    let cli = Cli::parse();
    
    // Setup logging
    tracing_subscriber::fmt()
        .with_max_level(if cli.verbose { Level::DEBUG } else { Level::INFO })
        .init();
    
    tracing::info!("Loading: {:?}", cli.input);
    let yaml = std::fs::read_to_string(&cli.input)?;
    
    let spec = carla_scenario_generator::dsl::parse_yaml(&yaml)?;
    
    tracing::info!("Generating {} scenario(s)", spec.num_scenarios);
    
    let scenarios = if spec.num_scenarios == 1 {
        vec![carla_scenario_generator::generate_single_scenario(&yaml)?]
    } else {
        carla_scenario_generator::generate_multiple_scenarios(&spec, spec.num_scenarios)?
    };
    
    // Write output
    if scenarios.len() == 1 {
        let json = serde_json::to_string_pretty(&scenarios[0])?;
        std::fs::write(&cli.output, json)?;
        tracing::info!("Wrote: {:?}", cli.output);
    } else {
        std::fs::create_dir_all(&cli.output)?;
        for (i, scenario) in scenarios.iter().enumerate() {
            let path = cli.output.join(format!("scenario_{}.json", i));
            let json = serde_json::to_string_pretty(scenario)?;
            std::fs::write(&path, json)?;
        }
        tracing::info!("Wrote {} scenarios to: {:?}", scenarios.len(), cli.output);
    }
    
    Ok(())
}
```

**Usage**:
```bash
cargo run -- -i examples/cut_in_left.yaml -o output.json
cargo run -- -i examples/cut_in_left.yaml -o output_dir/ -v
```

**Final Polish**:
- Error messages user-friendly
- Progress logging
- README with examples
- Integration tests
- Documentation comments

---

## Testing Workflow Across All Phases

After each phase:
```bash
cargo test
cargo clippy
cargo fmt
```

After Phase 10:
```bash
cargo run -- -i examples/cut_in_left.yaml -o test_output.json
cat test_output.json | python -m json.tool
# Manually validate: NPC cuts in? TTC > 3? Distance > 5?
```

After Phase 11:
```bash
# Generate 5 scenarios
cargo run -- -i examples/cut_in_left.yaml -o scenarios/
ls scenarios/  # Should see scenario_0.json, scenario_1.json, ...
# Validate they're different (different initial positions/speeds)
```

---

## Critical Success Criteria

**MVP Complete When**:
- [ ] All phases 1-12 implemented
- [ ] `cargo test` passes
- [ ] Example YAML → JSON works
- [ ] Generated scenarios physically valid
- [ ] Safety constraints satisfied
- [ ] Multiple diverse scenarios generated
- [ ] CLI functional

**Manual Validation Checklist** (for each scenario):
- [ ] NPC starts in lane 0, ego in lane 1
- [ ] NPC ahead of ego initially
- [ ] NPC eventually in lane 1
- [ ] Lane change within specified time range
- [ ] TTC never drops below 3.0s
- [ ] Distance never drops below 5.0m (when same lane)
- [ ] Velocities constant (except lateral during lane change)
- [ ] Positions follow physics (px[t+1] = px[t] + vx[t] * 0.5)

