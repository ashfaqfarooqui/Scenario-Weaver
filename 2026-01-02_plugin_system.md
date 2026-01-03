# Implementation Specification: Plugin-Based Scenario System
**Date**: 2026-01-02
**Project**: CARLA Scenario Generator - Generalized LTL & Z3 Encoding
**Repository**: `/home/ashfaqf/playground/synergies/test-4`

---

## AI Agent Implementation Prompt

You are tasked with refactoring the CARLA scenario generator to support multiple scenario types through a plugin-based architecture. This will reduce the manual work required to add new scenarios from modifying 3+ files to implementing a single trait.

### Implementation Requirements

1. **Work incrementally**: Implement one phase at a time, testing thoroughly before proceeding
2. **Commit regularly**: Create git commits after completing each working phase (not after each file)
3. **Maintain backward compatibility**: Existing `cut_in_left.yaml` files must continue working
4. **Write professional code**: No emoji, casual comments, or tool-specific references
5. **Test continuously**: Run `cargo test` after each phase to catch regressions early
6. **Target timeline**: Complete core refactoring in ~5-7 days

---

## Current System Architecture

The existing codebase generates driving test scenarios using this pipeline:
```
YAML Input → DSL Parser → LTL Generator → Z3 Encoder → Z3 Solver → Scenario Extractor → JSON/XOSC Output
```

### Current Limitations

1. **Single scenario support**: Only `CutInLeft` is implemented
2. **Hardcoded actors**: Assumes exactly 2 actors named "ego" and "npc"
3. **Scenario-specific DSL**: `NpcSpec` struct has `cut_in_time` field that only applies to cut-in scenarios
4. **Fragmented logic**: Adding new scenarios requires changes in 3 separate modules

### Target Architecture

Transform to:
- **Plugin-based scenarios**: Each scenario type is a trait implementation
- **Multi-actor support**: 1 ego + N NPCs (specifically 2-3 NPCs)
- **Generic DSL**: `ActorSpec` with scenario-agnostic fields + generic `behavior` map
- **Expanded propositions**: Library of 16 reusable LTL building blocks

---

## Phase 1: Refactor DSL for Generic Actors

**Objective**: Remove scenario-specific fields from DSL types and support multiple actors.

### Step 1.1: Modify `src/dsl/types.rs`

**Remove** the `NpcSpec` struct (lines 134-142) and **replace** with a unified `ActorSpec`:

```rust
use std::collections::HashMap;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ActorSpec {
    pub id: String,  // "ego", "npc1", "npc2", etc.
    pub role: ActorRole,
    pub lane: usize,
    pub position: ValueOrRange,
    pub speed: ValueOrRange,
    pub acceleration: ValueOrRange,

    // Generic behavior parameters (scenario plugins parse this)
    #[serde(default)]
    pub behavior: HashMap<String, serde_json::Value>,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub enum ActorRole {
    Ego,  // Protagonist vehicle (typically constrained)
    Npc,  // Non-player character vehicle (typically scripted)
}
```

**Update** `ScenarioSpec` struct (lines 88-109):

```rust
pub struct ScenarioSpec {
    pub scenario_type: ScenarioType,
    pub time_step: f64,
    pub duration: f64,

    // Replace 'ego' and 'npc' fields with actor list
    pub actors: Vec<ActorSpec>,

    pub min_ttc: f64,
    pub min_distance: f64,
    pub lane_width: f64,
    pub constraint_modes: ConstraintModes,
    pub max_acceleration: Option<f64>,
    pub max_deceleration: Option<f64>,
    pub num_scenarios: usize,
}
```

**Add** helper methods to `ScenarioSpec`:

```rust
impl ScenarioSpec {
    /// Get the ego actor (must be exactly one)
    pub fn ego(&self) -> &ActorSpec {
        self.actors
            .iter()
            .find(|a| a.role == ActorRole::Ego)
            .expect("Scenario must have exactly one ego actor")
    }

    /// Get iterator over all NPC actors
    pub fn npcs(&self) -> impl Iterator<Item = &ActorSpec> {
        self.actors.iter().filter(|a| a.role == ActorRole::Npc)
    }
}
```

**Update** validation logic in `validate()` method (lines 175-274):

```rust
impl ScenarioSpec {
    pub fn validate(&self) -> Result<()> {
        // Existing validations...

        // NEW: Validate actor constraints
        let ego_count = self.actors.iter().filter(|a| a.role == ActorRole::Ego).count();
        if ego_count != 1 {
            anyhow::bail!("Scenario must have exactly 1 ego actor, found {}", ego_count);
        }

        let npc_count = self.actors.iter().filter(|a| a.role == ActorRole::Npc).count();
        if npc_count == 0 {
            anyhow::bail!("Scenario must have at least 1 NPC actor");
        }

        // Validate unique actor IDs
        let mut seen_ids = std::collections::HashSet::new();
        for actor in &self.actors {
            if !seen_ids.insert(&actor.id) {
                anyhow::bail!("Duplicate actor ID: {}", actor.id);
            }
        }

        Ok(())
    }
}
```

### Step 1.2: Update `src/dsl/parser.rs` for Backward Compatibility

The existing parser (lines 7-15) uses `serde_yaml::from_str` directly. Add logic to support both old and new YAML formats:

```rust
pub fn parse_yaml(yaml_content: &str) -> Result<ScenarioSpec> {
    // Try new format first
    match serde_yaml::from_str::<ScenarioSpec>(yaml_content) {
        Ok(spec) => {
            spec.validate()?;
            Ok(spec)
        }
        Err(_) => {
            // Fall back to legacy format
            let legacy: LegacyScenarioSpec = serde_yaml::from_str(yaml_content)?;
            let spec = ScenarioSpec::from(legacy);
            spec.validate()?;
            Ok(spec)
        }
    }
}

// Legacy format support
#[derive(Deserialize)]
struct LegacyScenarioSpec {
    scenario_type: ScenarioType,
    time_step: f64,
    duration: f64,
    ego: LegacyActorSpec,
    npc: LegacyNpcSpec,
    min_ttc: f64,
    min_distance: f64,
    lane_width: f64,
    constraint_modes: ConstraintModes,
    max_acceleration: Option<f64>,
    max_deceleration: Option<f64>,
    num_scenarios: usize,
}

#[derive(Deserialize)]
struct LegacyActorSpec {
    lane: usize,
    position: ValueOrRange,
    speed: ValueOrRange,
    acceleration: ValueOrRange,
}

#[derive(Deserialize)]
struct LegacyNpcSpec {
    lane: usize,
    position: ValueOrRange,
    speed: ValueOrRange,
    acceleration: ValueOrRange,
    cut_in_time: ValueOrRange,
}

impl From<LegacyScenarioSpec> for ScenarioSpec {
    fn from(legacy: LegacyScenarioSpec) -> Self {
        let ego_actor = ActorSpec {
            id: "ego".to_string(),
            role: ActorRole::Ego,
            lane: legacy.ego.lane,
            position: legacy.ego.position,
            speed: legacy.ego.speed,
            acceleration: legacy.ego.acceleration,
            behavior: HashMap::new(),
        };

        let mut npc_behavior = HashMap::new();
        npc_behavior.insert(
            "cut_in_time".to_string(),
            serde_json::to_value(legacy.npc.cut_in_time).unwrap(),
        );

        let npc_actor = ActorSpec {
            id: "npc".to_string(),
            role: ActorRole::Npc,
            lane: legacy.npc.lane,
            position: legacy.npc.position,
            speed: legacy.npc.speed,
            acceleration: legacy.npc.acceleration,
            behavior: npc_behavior,
        };

        ScenarioSpec {
            scenario_type: legacy.scenario_type,
            time_step: legacy.time_step,
            duration: legacy.duration,
            actors: vec![ego_actor, npc_actor],
            min_ttc: legacy.min_ttc,
            min_distance: legacy.min_distance,
            lane_width: legacy.lane_width,
            constraint_modes: legacy.constraint_modes,
            max_acceleration: legacy.max_acceleration,
            max_deceleration: legacy.max_deceleration,
            num_scenarios: legacy.num_scenarios,
        }
    }
}
```

### Step 1.3: Commit Phase 1

```bash
cargo test --lib
git add src/dsl/types.rs src/dsl/parser.rs
git commit -m "Refactor DSL to support generic actors with behavior maps

- Replace NpcSpec with unified ActorSpec supporting arbitrary actors
- Add ActorRole enum (Ego/Npc) and helper methods to ScenarioSpec
- Implement backward compatibility layer for legacy YAML format
- Add validation for actor constraints (1 ego, 1+ NPCs)

This enables multi-actor scenarios and removes scenario-specific
fields from the core DSL types."
```

---

## Phase 2: Expand Proposition Library

**Objective**: Add 12 new proposition types to enable richer scenario descriptions.

### Step 2.1: Modify `src/ltl/formula.rs`

Add new variants to the `Proposition` enum (currently lines 24-46):

```rust
#[derive(Debug, Clone, PartialEq)]
pub enum Proposition {
    // Existing propositions
    InLane { actor: String, lane: usize },
    Ahead { actor1: String, actor2: String },
    DistanceGT { actor1: String, actor2: String, distance: f64 },
    TTCGT { actor1: String, actor2: String, ttc: f64 },

    // Kinematic propositions
    SpeedGT { actor: String, speed: f64 },
    SpeedLT { actor: String, speed: f64 },
    SpeedInRange { actor: String, min: f64, max: f64 },
    ChangingLane { actor: String },  // lateral velocity != 0

    // Spatial propositions
    SameLane { actor1: String, actor2: String },
    PositionGT { actor: String, position: f64 },
    PositionLT { actor: String, position: f64 },
    LateralDistanceGT { actor1: String, actor2: String, distance: f64 },

    // Behavioral propositions
    Following { follower: String, leader: String, gap: f64 },
    Overtaking { actor1: String, actor2: String },
    DistanceLT { actor1: String, actor2: String, distance: f64 },
    Approaching { actor1: String, actor2: String },  // relative velocity < 0
}
```

### Step 2.2: Update `src/solver/encoder.rs` - Add Z3 Encoding

Add encoding logic for new propositions in the `encode_proposition` method (currently lines 430-479):

```rust
fn encode_proposition(&self, prop: &Proposition, t: usize) -> Bool<'ctx> {
    let ctx = &self.ctx;
    let zero = Real::from_real(ctx, 0, 1);

    match prop {
        // Existing encodings remain unchanged...

        // Kinematic propositions
        Proposition::SpeedGT { actor, speed } => {
            let vx = &self.actor_vars[actor].velocity_x[t];
            let threshold = Real::from_real(ctx, (*speed * 10.0) as i32, 10);
            vx._gt(&threshold)
        }

        Proposition::SpeedLT { actor, speed } => {
            let vx = &self.actor_vars[actor].velocity_x[t];
            let threshold = Real::from_real(ctx, (*speed * 10.0) as i32, 10);
            vx._lt(&threshold)
        }

        Proposition::SpeedInRange { actor, min, max } => {
            let vx = &self.actor_vars[actor].velocity_x[t];
            let min_threshold = Real::from_real(ctx, (*min * 10.0) as i32, 10);
            let max_threshold = Real::from_real(ctx, (*max * 10.0) as i32, 10);
            vx._gt(&min_threshold).and(&vx._lt(&max_threshold))
        }

        Proposition::ChangingLane { actor } => {
            let vy = &self.actor_vars[actor].velocity_y[t];
            let epsilon = Real::from_real(ctx, 1, 100);  // 0.01 m/s threshold
            vy._gt(&epsilon).or(&vy._lt(&(-&epsilon)))
        }

        // Spatial propositions
        Proposition::SameLane { actor1, actor2 } => {
            let lane1 = &self.actor_vars[actor1].lane[t];
            let lane2 = &self.actor_vars[actor2].lane[t];
            lane1._eq(lane2)
        }

        Proposition::PositionGT { actor, position } => {
            let px = &self.actor_vars[actor].position_x[t];
            let threshold = Real::from_real(ctx, (*position * 10.0) as i32, 10);
            px._gt(&threshold)
        }

        Proposition::PositionLT { actor, position } => {
            let px = &self.actor_vars[actor].position_x[t];
            let threshold = Real::from_real(ctx, (*position * 10.0) as i32, 10);
            px._lt(&threshold)
        }

        Proposition::LateralDistanceGT { actor1, actor2, distance } => {
            let py1 = &self.actor_vars[actor1].position_y[t];
            let py2 = &self.actor_vars[actor2].position_y[t];
            let diff = py1 - py2;
            let threshold = Real::from_real(ctx, (*distance * 10.0) as i32, 10);

            // Absolute value: (diff > threshold) OR (diff < -threshold)
            diff._gt(&threshold).or(&diff._lt(&(-&threshold)))
        }

        // Behavioral propositions
        Proposition::Following { follower, leader, gap } => {
            let px_follower = &self.actor_vars[follower].position_x[t];
            let px_leader = &self.actor_vars[leader].position_x[t];
            let distance = px_leader - px_follower;
            let target_gap = Real::from_real(ctx, (*gap * 10.0) as i32, 10);

            // Following: leader ahead, same lane, maintaining gap
            self.encode_proposition(
                &Proposition::Ahead {
                    actor1: leader.clone(),
                    actor2: follower.clone(),
                },
                t,
            )
            .and(&self.encode_proposition(
                &Proposition::SameLane {
                    actor1: leader.clone(),
                    actor2: follower.clone(),
                },
                t,
            ))
            .and(&distance._eq(&target_gap))
        }

        Proposition::Overtaking { actor1, actor2 } => {
            // Overtaking: actor1 faster than actor2, approaching from behind
            self.encode_proposition(
                &Proposition::Ahead {
                    actor1: actor2.clone(),
                    actor2: actor1.clone(),
                },
                t,
            )
            .and(&self.encode_proposition(
                &Proposition::Approaching {
                    actor1: actor1.clone(),
                    actor2: actor2.clone(),
                },
                t,
            ))
        }

        Proposition::DistanceLT { actor1, actor2, distance } => {
            let px1 = &self.actor_vars[actor1].position_x[t];
            let px2 = &self.actor_vars[actor2].position_x[t];
            let diff = px1 - px2;
            let threshold = Real::from_real(ctx, (*distance * 10.0) as i32, 10);
            let neg_threshold = Real::from_real(ctx, (-*distance * 10.0) as i32, 10);

            // |diff| < threshold => diff > -threshold AND diff < threshold
            diff._gt(&neg_threshold).and(&diff._lt(&threshold))
        }

        Proposition::Approaching { actor1, actor2 } => {
            let vx1 = &self.actor_vars[actor1].velocity_x[t];
            let vx2 = &self.actor_vars[actor2].velocity_x[t];
            let px1 = &self.actor_vars[actor1].position_x[t];
            let px2 = &self.actor_vars[actor2].position_x[t];

            // Approaching: relative velocity is closing the gap
            // If actor1 ahead: vx1 < vx2 (actor2 faster, catching up)
            // If actor2 ahead: vx1 > vx2 (actor1 faster, catching up)
            let ahead1 = px1._gt(px2);
            let ahead2 = px2._gt(px1);
            let rel_vel_closing1 = vx1._lt(vx2);
            let rel_vel_closing2 = vx1._gt(vx2);

            ahead1.and(&rel_vel_closing1).or(&ahead2.and(&rel_vel_closing2))
        }
    }
}
```

### Step 2.3: Commit Phase 2

```bash
cargo test --lib
git add src/ltl/formula.rs src/solver/encoder.rs
git commit -m "Expand proposition library with 12 new variants

Add kinematic, spatial, and behavioral propositions:
- SpeedGT/LT/InRange, ChangingLane
- SameLane, PositionGT/LT, LateralDistanceGT
- Following, Overtaking, DistanceLT, Approaching

Implement Z3 encoding for all new propositions in encoder.
This provides building blocks for more complex scenario types."
```

---

## Phase 3: Create Plugin System

**Objective**: Define plugin trait and registry for scenario-specific LTL generation.

### Step 3.1: Create New File `src/ltl/plugin.rs`

```rust
use crate::dsl::types::{ActorSpec, ScenarioSpec, ScenarioType};
use crate::ltl::formula::{LTLFormula, Proposition};
use anyhow::Result;
use std::collections::HashMap;
use std::sync::Mutex;

/// Trait for scenario-specific LTL generation
pub trait ScenarioPlugin: Send + Sync {
    /// Human-readable name for this scenario type
    fn name(&self) -> &str;

    /// Validate that the scenario spec is compatible with this plugin
    fn validate_spec(&self, spec: &ScenarioSpec) -> Result<()>;

    /// Generate the complete LTL formula for this scenario
    fn generate_ltl(&self, spec: &ScenarioSpec) -> Result<LTLFormula>;

    /// Generate initial conditions (default implementation provided)
    fn initial_conditions(&self, spec: &ScenarioSpec) -> LTLFormula {
        spec.actors
            .iter()
            .map(|actor| {
                LTLFormula::Atom(Proposition::InLane {
                    actor: actor.id.clone(),
                    lane: actor.lane,
                })
            })
            .reduce(|a, b| a.and(b))
            .unwrap_or(LTLFormula::Atom(Proposition::InLane {
                actor: "ego".to_string(),
                lane: 0,
            }))
    }

    /// Generate safety constraints (default implementation using spec constraints)
    fn safety_constraints(&self, spec: &ScenarioSpec) -> LTLFormula {
        use crate::dsl::types::ConstraintMode;

        let ego = spec.ego();
        let mut constraints = Vec::new();

        for npc in spec.npcs() {
            // TTC constraint
            match spec.constraint_modes.min_ttc() {
                ConstraintMode::Enforce => {
                    constraints.push(
                        LTLFormula::Atom(Proposition::TTCGT {
                            actor1: ego.id.clone(),
                            actor2: npc.id.clone(),
                            ttc: spec.min_ttc,
                        })
                        .always(),
                    );
                }
                ConstraintMode::Violate => {
                    constraints.push(
                        LTLFormula::Atom(Proposition::TTCGT {
                            actor1: ego.id.clone(),
                            actor2: npc.id.clone(),
                            ttc: spec.min_ttc,
                        })
                        .negate()
                        .eventually(),
                    );
                }
                ConstraintMode::Ignore => {}
            }

            // Distance constraint
            match spec.constraint_modes.min_distance() {
                ConstraintMode::Enforce => {
                    constraints.push(
                        LTLFormula::Atom(Proposition::DistanceGT {
                            actor1: ego.id.clone(),
                            actor2: npc.id.clone(),
                            distance: spec.min_distance,
                        })
                        .always(),
                    );
                }
                ConstraintMode::Violate => {
                    constraints.push(
                        LTLFormula::Atom(Proposition::DistanceGT {
                            actor1: ego.id.clone(),
                            actor2: npc.id.clone(),
                            distance: spec.min_distance,
                        })
                        .negate()
                        .eventually(),
                    );
                }
                ConstraintMode::Ignore => {}
            }
        }

        constraints
            .into_iter()
            .reduce(|a, b| a.and(b))
            .unwrap_or(LTLFormula::Atom(Proposition::InLane {
                actor: "ego".to_string(),
                lane: 0,
            }))
    }
}

/// Registry for scenario plugins
pub struct PluginRegistry {
    plugins: HashMap<ScenarioType, Box<dyn ScenarioPlugin>>,
}

impl PluginRegistry {
    pub fn new() -> Self {
        let mut registry = Self {
            plugins: HashMap::new(),
        };

        // Register built-in plugins
        registry.register(ScenarioType::CutInLeft, Box::new(CutInLeftPlugin));

        registry
    }

    pub fn register(&mut self, scenario_type: ScenarioType, plugin: Box<dyn ScenarioPlugin>) {
        self.plugins.insert(scenario_type, plugin);
    }

    pub fn generate(&self, spec: &ScenarioSpec) -> Result<LTLFormula> {
        let plugin = self
            .plugins
            .get(&spec.scenario_type)
            .ok_or_else(|| anyhow::anyhow!("No plugin registered for {:?}", spec.scenario_type))?;

        plugin.validate_spec(spec)?;
        plugin.generate_ltl(spec)
    }
}

// Global registry with lazy initialization
lazy_static::lazy_static! {
    pub static ref REGISTRY: Mutex<PluginRegistry> = Mutex::new(PluginRegistry::new());
}

// Plugin implementation for CutInLeft scenario
struct CutInLeftPlugin;

impl ScenarioPlugin for CutInLeftPlugin {
    fn name(&self) -> &str {
        "Cut-In Left"
    }

    fn validate_spec(&self, spec: &ScenarioSpec) -> Result<()> {
        let npc_count = spec.npcs().count();
        if npc_count != 1 {
            anyhow::bail!(
                "Cut-in left requires exactly 1 NPC, found {}",
                npc_count
            );
        }

        let ego = spec.ego();
        let npc = spec.npcs().next().unwrap();

        if ego.lane != 1 {
            anyhow::bail!(
                "Cut-in left requires ego in lane 1, found lane {}",
                ego.lane
            );
        }
        if npc.lane != 0 {
            anyhow::bail!(
                "Cut-in left requires NPC in lane 0, found lane {}",
                npc.lane
            );
        }

        Ok(())
    }

    fn generate_ltl(&self, spec: &ScenarioSpec) -> Result<LTLFormula> {
        let ego = spec.ego();
        let npc = spec.npcs().next().unwrap();

        // Initial conditions
        let init = self
            .initial_conditions(spec)
            .and(LTLFormula::Atom(Proposition::Ahead {
                actor1: npc.id.clone(),
                actor2: ego.id.clone(),
            }));

        // Behavior: NPC eventually changes to lane 1
        let behavior = LTLFormula::Atom(Proposition::InLane {
            actor: npc.id.clone(),
            lane: 1,
        })
        .eventually()
        .and(
            LTLFormula::Atom(Proposition::InLane {
                actor: npc.id.clone(),
                lane: 0,
            })
            .until(LTLFormula::Atom(Proposition::InLane {
                actor: npc.id.clone(),
                lane: 1,
            })),
        );

        // Safety constraints
        let safety = self.safety_constraints(spec);

        Ok(init.and(behavior).and(safety))
    }
}
```

### Step 3.2: Update `src/ltl/mod.rs`

Add the new module:

```rust
pub mod formula;
pub mod generator;
pub mod plugin;  // NEW
```

### Step 3.3: Update `src/ltl/generator.rs`

Refactor to use the plugin registry (modify lines 10-13):

```rust
use crate::dsl::types::ScenarioSpec;
use crate::ltl::formula::LTLFormula;
use crate::ltl::plugin::REGISTRY;

pub struct LTLGenerator;

impl LTLGenerator {
    pub fn generate(spec: &ScenarioSpec) -> LTLFormula {
        REGISTRY
            .lock()
            .unwrap()
            .generate(spec)
            .expect("Failed to generate LTL formula")
    }
}
```

Keep the old methods but mark them as deprecated:

```rust
#[deprecated(note = "Use plugin system via REGISTRY instead")]
impl LTLGenerator {
    fn generate_cut_in_left(spec: &ScenarioSpec) -> LTLFormula {
        // Keep original implementation for reference
        // ...
    }

    fn safety_constraints(spec: &ScenarioSpec) -> LTLFormula {
        // Now handled by plugin trait default method
        // ...
    }
}
```

### Step 3.4: Add Dependency

Update `Cargo.toml` to include `lazy_static`:

```toml
[dependencies]
lazy_static = "1.4"
# ... existing dependencies
```

### Step 3.5: Commit Phase 3

```bash
cargo test
git add src/ltl/plugin.rs src/ltl/mod.rs src/ltl/generator.rs Cargo.toml
git commit -m "Implement plugin system for scenario-specific LTL generation

- Define ScenarioPlugin trait with validate_spec and generate_ltl methods
- Create PluginRegistry for dynamic plugin dispatch
- Implement CutInLeftPlugin as reference implementation
- Refactor LTLGenerator to use plugin registry
- Move safety_constraints to trait default method for reusability

This enables adding new scenarios by implementing a single trait
instead of modifying multiple core modules."
```

---

## Phase 4: Multi-Actor Z3 Support

**Objective**: Remove hardcoded "ego"/"npc" assumptions and support N actors.

### Step 4.1: Update `src/solver/encoder.rs` - Variable Creation

Modify the `encode` method to loop over actors instead of hardcoding (around lines 81-112):

```rust
// OLD CODE (remove):
// let ego_vars = self.create_actor_variables("ego", horizon);
// let npc_vars = self.create_actor_variables("npc", horizon);
// self.actor_vars.insert("ego".to_string(), ego_vars);
// self.actor_vars.insert("npc".to_string(), npc_vars);

// NEW CODE:
for actor in &spec.actors {
    let vars = self.create_actor_variables(&actor.id, horizon);
    self.actor_vars.insert(actor.id.clone(), vars);
}
```

### Step 4.2: Update Kinematics Encoding

Modify `encode_kinematics` to work with actor IDs dynamically (around lines 232-292):

```rust
// Apply kinematics to all actors
for actor in &spec.actors {
    self.encode_kinematics_for_actor(&actor.id, spec, horizon, solver);
}

// Ego-specific constraint: never change lanes
let ego = spec.ego();
let zero = Real::from_real(&self.ctx, 0, 1);
for t in 0..=horizon {
    let vy = &self.actor_vars[&ego.id].velocity_y[t];
    solver.assert(&vy._eq(&zero));
}
```

Rename the existing `encode_kinematics` to `encode_kinematics_for_actor` and change its signature:

```rust
fn encode_kinematics_for_actor(
    &self,
    actor_id: &str,
    spec: &ScenarioSpec,
    horizon: usize,
    solver: &Solver,
) {
    let vars = &self.actor_vars[actor_id];
    let dt = Real::from_real(&self.ctx, (spec.time_step * 10.0) as i32, 10);
    let zero = Real::from_real(&self.ctx, 0, 1);

    // Position updates
    for t in 0..horizon {
        // px[t+1] = px[t] + vx[t] * dt
        let px_next = &vars.position_x[t] + &vars.velocity_x[t] * &dt;
        solver.assert(&vars.position_x[t + 1]._eq(&px_next));

        // py[t+1] = py[t] + vy[t] * dt
        let py_next = &vars.position_y[t] + &vars.velocity_y[t] * &dt;
        solver.assert(&vars.position_y[t + 1]._eq(&py_next));
    }

    // Velocity updates (with acceleration support)
    for t in 0..horizon {
        let vx_next = &vars.velocity_x[t] + &vars.acceleration_x[t] * &dt;
        solver.assert(&vars.velocity_x[t + 1]._eq(&vx_next));

        let vy_next = &vars.velocity_y[t] + &vars.acceleration_y[t] * &dt;
        solver.assert(&vars.velocity_y[t + 1]._eq(&vy_next));
    }

    // Non-negative velocity constraint
    for t in 0..=horizon {
        solver.assert(&vars.velocity_x[t]._ge(&zero));
    }

    // Lane-position coupling
    let lane_width = Real::from_real(&self.ctx, (spec.lane_width * 10.0) as i32, 10);
    let half_width = Real::from_real(&self.ctx, (spec.lane_width * 5.0) as i32, 10);
    for t in 0..=horizon {
        let lane_int = &vars.lane[t];
        let lane_real = Int::to_real(lane_int);
        let expected_py = &lane_real * &lane_width + &half_width;
        solver.assert(&vars.position_y[t]._eq(&expected_py));
    }
}
```

### Step 4.3: Update Safety Encoding

Modify `encode_safety` to handle pairwise constraints between ego and each NPC (around lines 560-584):

```rust
fn encode_safety(&self, spec: &ScenarioSpec, horizon: usize, solver: &Solver) {
    use crate::dsl::types::ConstraintMode;

    let ego = spec.ego();

    // Apply safety constraints between ego and each NPC
    for npc in spec.npcs() {
        self.encode_pairwise_safety(spec, &ego.id, &npc.id, horizon, solver);
    }
}

fn encode_pairwise_safety(
    &self,
    spec: &ScenarioSpec,
    actor1_id: &str,
    actor2_id: &str,
    horizon: usize,
    solver: &Solver,
) {
    use crate::dsl::types::ConstraintMode;

    // Only add direct assertions for Enforce mode
    // (LTL layer handles Violate/Ignore modes)

    if spec.constraint_modes.min_ttc() == ConstraintMode::Enforce {
        for t in 0..=horizon {
            let ttc_constraint = self.encode_ttc_constraint(actor1_id, actor2_id, spec.min_ttc, t);
            solver.assert(&ttc_constraint);
        }
    }

    if spec.constraint_modes.min_distance() == ConstraintMode::Enforce {
        for t in 0..=horizon {
            let dist_constraint = self.encode_distance_constraint(
                actor1_id,
                actor2_id,
                spec.min_distance,
                t,
            );
            solver.assert(&dist_constraint);
        }
    }
}
```

### Step 4.4: Update `src/scenario/extractor.rs`

Modify extraction to handle multiple actors (main changes in `extract_scenario` function):

```rust
pub fn extract_scenario(
    model: &Model,
    spec: &ScenarioSpec,
    encoder: &Z3Encoder,
) -> Result<Scenario> {
    let horizon = ((spec.duration / spec.time_step) as usize).max(1);
    let mut actor_trajectories = HashMap::new();

    // Extract trajectory for each actor
    for actor in &spec.actors {
        let trajectory = extract_actor_trajectory(model, &actor.id, spec, encoder, horizon)?;
        actor_trajectories.insert(actor.id.clone(), trajectory);
    }

    // Compute pairwise safety metrics (ego vs each NPC)
    let ego = spec.ego();
    let mut all_violations = Vec::new();
    let mut min_ttc = f64::INFINITY;
    let mut min_distance = f64::INFINITY;

    for npc in spec.npcs() {
        let (ttc, dist, violations) = compute_pairwise_metrics(
            &actor_trajectories[&ego.id],
            &actor_trajectories[&npc.id],
            spec,
        )?;

        min_ttc = min_ttc.min(ttc);
        min_distance = min_distance.min(dist);
        all_violations.extend(violations);
    }

    let all_satisfied = all_violations.is_empty();

    Ok(Scenario {
        scenario_id: uuid::Uuid::new_v4().to_string(),
        scenario_type: spec.scenario_type.clone(),
        duration: spec.duration,
        time_step: spec.time_step,
        actor_trajectories,
        min_ttc,
        min_distance,
        all_constraints_satisfied: all_satisfied,
        violations: all_violations,
    })
}

fn extract_actor_trajectory(
    model: &Model,
    actor_id: &str,
    spec: &ScenarioSpec,
    encoder: &Z3Encoder,
    horizon: usize,
) -> Result<ActorTrajectory> {
    let vars = &encoder.actor_vars[actor_id];
    let mut positions = Vec::new();
    let mut velocities = Vec::new();
    let mut accelerations = Vec::new();
    let mut lanes = Vec::new();

    for t in 0..=horizon {
        let px = extract_real_value(model, &vars.position_x[t])?;
        let py = extract_real_value(model, &vars.position_y[t])?;
        positions.push((px, py));

        let vx = extract_real_value(model, &vars.velocity_x[t])?;
        let vy = extract_real_value(model, &vars.velocity_y[t])?;
        velocities.push((vx, vy));

        let ax = extract_real_value(model, &vars.acceleration_x[t])?;
        let ay = extract_real_value(model, &vars.acceleration_y[t])?;
        accelerations.push((ax, ay));

        let lane = extract_int_value(model, &vars.lane[t])? as usize;
        lanes.push(lane);
    }

    Ok(ActorTrajectory {
        positions,
        velocities,
        accelerations,
        lanes,
    })
}
```

### Step 4.5: Update `src/scenario/model.rs`

Change `Scenario` struct to use `HashMap` for actor trajectories:

```rust
use std::collections::HashMap;

pub struct Scenario {
    pub scenario_id: String,
    pub scenario_type: ScenarioType,
    pub duration: f64,
    pub time_step: f64,

    // Multi-actor support
    pub actor_trajectories: HashMap<String, ActorTrajectory>,

    pub min_ttc: f64,
    pub min_distance: f64,
    pub all_constraints_satisfied: bool,
    pub violations: Vec<SafetyViolation>,
}
```

### Step 4.6: Commit Phase 4

```bash
cargo test
git add src/solver/encoder.rs src/scenario/extractor.rs src/scenario/model.rs
git commit -m "Add multi-actor support to Z3 encoder and extractor

- Remove hardcoded ego/npc actor IDs from encoder
- Support 1 ego + N NPC actors via dynamic loops over spec.actors
- Apply pairwise safety constraints between ego and each NPC
- Extract trajectories for all actors into HashMap
- Compute aggregate safety metrics across all NPC pairs

This enables scenarios with 2+ NPCs without code changes."
```

---

## Phase 5: Add New Scenario Types

**Objective**: Demonstrate plugin system by implementing cut-in right and following scenarios.

### Step 5.1: Add ScenarioType Variants

Update `src/dsl/types.rs`:

```rust
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq, Hash)]
pub enum ScenarioType {
    CutInLeft,
    CutInRight,   // NEW
    Following,    // NEW
}
```

### Step 5.2: Implement CutInRightPlugin

Add to `src/ltl/plugin.rs`:

```rust
struct CutInRightPlugin;

impl ScenarioPlugin for CutInRightPlugin {
    fn name(&self) -> &str {
        "Cut-In Right"
    }

    fn validate_spec(&self, spec: &ScenarioSpec) -> Result<()> {
        let npc_count = spec.npcs().count();
        if npc_count != 1 {
            anyhow::bail!("Cut-in right requires exactly 1 NPC, found {}", npc_count);
        }

        let ego = spec.ego();
        let npc = spec.npcs().next().unwrap();

        if ego.lane != 0 {
            anyhow::bail!(
                "Cut-in right requires ego in lane 0, found lane {}",
                ego.lane
            );
        }
        if npc.lane != 1 {
            anyhow::bail!(
                "Cut-in right requires NPC in lane 1, found lane {}",
                npc.lane
            );
        }

        Ok(())
    }

    fn generate_ltl(&self, spec: &ScenarioSpec) -> Result<LTLFormula> {
        let ego = spec.ego();
        let npc = spec.npcs().next().unwrap();

        // Initial conditions: NPC ahead of ego
        let init = self
            .initial_conditions(spec)
            .and(LTLFormula::Atom(Proposition::Ahead {
                actor1: npc.id.clone(),
                actor2: ego.id.clone(),
            }));

        // Behavior: NPC eventually changes from lane 1 to lane 0
        let behavior = LTLFormula::Atom(Proposition::InLane {
            actor: npc.id.clone(),
            lane: 0,
        })
        .eventually()
        .and(
            LTLFormula::Atom(Proposition::InLane {
                actor: npc.id.clone(),
                lane: 1,
            })
            .until(LTLFormula::Atom(Proposition::InLane {
                actor: npc.id.clone(),
                lane: 0,
            })),
        );

        let safety = self.safety_constraints(spec);

        Ok(init.and(behavior).and(safety))
    }
}
```

Register in `PluginRegistry::new()`:

```rust
registry.register(ScenarioType::CutInRight, Box::new(CutInRightPlugin));
```

### Step 5.3: Implement FollowingPlugin

Add to `src/ltl/plugin.rs`:

```rust
struct FollowingPlugin;

impl ScenarioPlugin for FollowingPlugin {
    fn name(&self) -> &str {
        "Following"
    }

    fn validate_spec(&self, spec: &ScenarioSpec) -> Result<()> {
        let npc_count = spec.npcs().count();
        if npc_count != 1 {
            anyhow::bail!("Following requires exactly 1 NPC, found {}", npc_count);
        }

        let ego = spec.ego();
        let npc = spec.npcs().next().unwrap();

        if ego.lane != npc.lane {
            anyhow::bail!("Following requires both actors in same lane");
        }

        // Validate behavior parameter
        npc.behavior
            .get("target_gap")
            .and_then(|v| v.as_f64())
            .ok_or_else(|| {
                anyhow::anyhow!("Following NPC requires 'target_gap' behavior parameter")
            })?;

        Ok(())
    }

    fn generate_ltl(&self, spec: &ScenarioSpec) -> Result<LTLFormula> {
        let ego = spec.ego();
        let npc = spec.npcs().next().unwrap();

        let target_gap = npc
            .behavior
            .get("target_gap")
            .and_then(|v| v.as_f64())
            .unwrap();

        // Initial: NPC ahead of ego
        let init = self
            .initial_conditions(spec)
            .and(LTLFormula::Atom(Proposition::Ahead {
                actor1: npc.id.clone(),
                actor2: ego.id.clone(),
            }));

        // Behavior: Ego always maintains gap behind NPC
        let behavior = LTLFormula::Atom(Proposition::Following {
            follower: ego.id.clone(),
            leader: npc.id.clone(),
            gap: target_gap,
        })
        .always();

        let safety = self.safety_constraints(spec);

        Ok(init.and(behavior).and(safety))
    }
}
```

Register in `PluginRegistry::new()`:

```rust
registry.register(ScenarioType::Following, Box::new(FollowingPlugin));
```

### Step 5.4: Create Example YAML Files

**File**: `examples/cut_in_right.yaml`

```yaml
scenario_type: cut_in_right

time_step: 0.5
duration: 10.0
lane_width: 3.5

actors:
  - id: ego
    role: ego
    lane: 0
    position: 50.0
    speed: 15.0
    acceleration: 0.0

  - id: npc
    role: npc
    lane: 1
    position: [60.0, 80.0]
    speed: [12.0, 14.0]
    acceleration: 0.0
    behavior:
      cut_in_time: [2.5, 7.5]

min_ttc: 3.0
min_distance: 5.0
constraint_modes: enforce_all
num_scenarios: 1
```

**File**: `examples/following.yaml`

```yaml
scenario_type: following

time_step: 0.5
duration: 10.0
lane_width: 3.5

actors:
  - id: ego
    role: ego
    lane: 1
    position: 50.0
    speed: [12.0, 14.0]
    acceleration: 0.0

  - id: npc
    role: npc
    lane: 1
    position: [60.0, 70.0]
    speed: [12.0, 14.0]
    acceleration: 0.0
    behavior:
      target_gap: 10.0

min_ttc: 3.0
min_distance: 5.0
constraint_modes: enforce_all
num_scenarios: 1
```

**File**: `examples/multi_npc_cut_in.yaml`

```yaml
scenario_type: cut_in_left

time_step: 0.5
duration: 10.0
lane_width: 3.5

actors:
  - id: ego
    role: ego
    lane: 1
    position: 50.0
    speed: 15.0
    acceleration: 0.0

  - id: npc1
    role: npc
    lane: 0
    position: [60.0, 70.0]
    speed: [12.0, 14.0]
    acceleration: 0.0
    behavior:
      cut_in_time: [2.5, 5.0]

  - id: npc2
    role: npc
    lane: 1
    position: [80.0, 100.0]
    speed: [10.0, 12.0]
    acceleration: 0.0
    behavior: {}

min_ttc: 3.0
min_distance: 5.0
constraint_modes: enforce_all
num_scenarios: 1
```

### Step 5.5: Add Integration Tests

Update `tests/integration_test.rs`:

```rust
#[test]
fn test_cut_in_right_scenario() {
    let yaml = std::fs::read_to_string("examples/cut_in_right.yaml").unwrap();
    let spec = parse_yaml(&yaml).unwrap();
    let scenario = generate_single_scenario(&spec).unwrap();

    assert_eq!(scenario.scenario_type, ScenarioType::CutInRight);
    assert!(scenario.actor_trajectories.contains_key("ego"));
    assert!(scenario.actor_trajectories.contains_key("npc"));

    // Verify lane change happened
    let npc_traj = &scenario.actor_trajectories["npc"];
    assert_eq!(npc_traj.lanes[0], 1);  // Starts in lane 1
    assert!(npc_traj.lanes.iter().any(|&lane| lane == 0));  // Eventually in lane 0
}

#[test]
fn test_following_scenario() {
    let yaml = std::fs::read_to_string("examples/following.yaml").unwrap();
    let spec = parse_yaml(&yaml).unwrap();
    let scenario = generate_single_scenario(&spec).unwrap();

    let ego_traj = &scenario.actor_trajectories["ego"];
    let npc_traj = &scenario.actor_trajectories["npc"];

    // Verify gap is maintained (approximately 10.0m)
    for i in 0..ego_traj.positions.len() {
        let gap = npc_traj.positions[i].0 - ego_traj.positions[i].0;
        assert!(
            gap > 8.0 && gap < 12.0,
            "Gap at timestep {} should be ~10.0m, got {}",
            i,
            gap
        );
    }
}

#[test]
fn test_multi_npc_scenario() {
    let yaml = std::fs::read_to_string("examples/multi_npc_cut_in.yaml").unwrap();
    let spec = parse_yaml(&yaml).unwrap();
    let scenario = generate_single_scenario(&spec).unwrap();

    assert_eq!(scenario.actor_trajectories.len(), 3);
    assert!(scenario.actor_trajectories.contains_key("ego"));
    assert!(scenario.actor_trajectories.contains_key("npc1"));
    assert!(scenario.actor_trajectories.contains_key("npc2"));
}
```

### Step 5.6: Commit Phase 5

```bash
cargo test
cargo run -- -i examples/cut_in_right.yaml -o test_output_right.json
cargo run -- -i examples/following.yaml -o test_output_following.json
git add src/dsl/types.rs src/ltl/plugin.rs examples/ tests/integration_test.rs
git commit -m "Add cut-in right and following scenario plugins

- Implement CutInRightPlugin (symmetric to cut-in left)
- Implement FollowingPlugin using Following proposition
- Add 3 example YAML files demonstrating new scenarios
- Add integration tests for new scenario types
- Verify multi-NPC support with example

Plugin system allows adding these scenarios with ~60 lines each,
demonstrating the effectiveness of the refactoring."
```

---

## Phase 6: Documentation and Final Testing

### Step 6.1: Update `CLAUDE.md`

Add section after "Common Commands":

```markdown
## Adding New Scenario Types

The plugin system allows you to add new scenario types by implementing a single trait.

### Quick Guide

1. **Add scenario type** to `ScenarioType` enum in `src/dsl/types.rs`:
   ```rust
   pub enum ScenarioType {
       CutInLeft,
       CutInRight,
       Following,
       MyNewScenario,  // Add here
   }
   ```

2. **Implement plugin** in `src/ltl/plugin.rs`:
   ```rust
   struct MyScenarioPlugin;

   impl ScenarioPlugin for MyScenarioPlugin {
       fn name(&self) -> &str { "My Scenario" }

       fn validate_spec(&self, spec: &ScenarioSpec) -> Result<()> {
           // Validate actor configuration
           Ok(())
       }

       fn generate_ltl(&self, spec: &ScenarioSpec) -> Result<LTLFormula> {
           let ego = spec.ego();
           let npc = spec.npcs().next().unwrap();

           // Build formula using propositions
           let init = self.initial_conditions(spec);
           let behavior = /* ... */;
           let safety = self.safety_constraints(spec);

           Ok(init.and(behavior).and(safety))
       }
   }
   ```

3. **Register plugin** in `PluginRegistry::new()`:
   ```rust
   registry.register(ScenarioType::MyNewScenario, Box::new(MyScenarioPlugin));
   ```

4. **Create example YAML** in `examples/my_scenario.yaml`

5. **Add test** in `tests/integration_test.rs`

### Available Propositions

Use these building blocks in `generate_ltl()`:

**Spatial**:
- `InLane { actor, lane }` - Actor is in specific lane
- `Ahead { actor1, actor2 }` - actor1 is ahead of actor2
- `SameLane { actor1, actor2 }` - Actors in same lane
- `PositionGT/LT { actor, position }` - Position comparison

**Distance**:
- `DistanceGT/LT { actor1, actor2, distance }` - Longitudinal distance
- `LateralDistanceGT { actor1, actor2, distance }` - Lateral distance

**Kinematic**:
- `SpeedGT/LT { actor, speed }` - Speed comparison
- `SpeedInRange { actor, min, max }` - Speed in range
- `ChangingLane { actor }` - Lateral velocity != 0

**Behavioral**:
- `Following { follower, leader, gap }` - Maintaining gap
- `Overtaking { actor1, actor2 }` - Overtaking maneuver
- `Approaching { actor1, actor2 }` - Closing gap

**Safety**:
- `TTCGT { actor1, actor2, ttc }` - Time-to-collision threshold

### LTL Temporal Operators

- `.always()` - G(φ) - Always true
- `.eventually()` - F(φ) - Eventually true
- `.until(ψ)` - φ U ψ - φ holds until ψ becomes true
- `.and(ψ)` - φ ∧ ψ - Both true
- `.or(ψ)` - φ ∨ ψ - At least one true
- `.negate()` - ¬φ - Negation

### Multi-Actor Scenarios

To support multiple NPCs:

```yaml
actors:
  - id: ego
    role: ego
    lane: 1
    # ...

  - id: npc1
    role: npc
    lane: 0
    # ...

  - id: npc2
    role: npc
    lane: 1
    # ...
```

Safety constraints are automatically applied pairwise between ego and each NPC.
```

### Step 6.2: Run Full Test Suite

```bash
# Run all tests
cargo test

# Test all example files
cargo run -- -i examples/cut_in_left.yaml -o output_left.json
cargo run -- -i examples/cut_in_right.yaml -o output_right.json
cargo run -- -i examples/following.yaml -o output_following.json
cargo run -- -i examples/multi_npc_cut_in.yaml -o output_multi.json

# Verify XOSC export works
ls -l output_*.xosc

# Test adversarial mode still works
cargo run -- -i examples/cut_in_left.yaml -o output_adv.json --adversarial

# Check code quality
cargo clippy
cargo fmt --check
```

### Step 6.3: Final Commit

```bash
git add CLAUDE.md
git commit -m "Update documentation for plugin-based scenario system

Add comprehensive guide for implementing new scenario types:
- Step-by-step plugin implementation instructions
- Available propositions reference
- LTL temporal operator documentation
- Multi-actor configuration examples

Developers can now add scenarios in <2 hours by following guide."
```

---

## Validation Checklist

Before considering the implementation complete, verify:

- [ ] `cargo test` passes all tests
- [ ] `cargo clippy` shows no warnings
- [ ] All example YAML files generate valid scenarios
- [ ] Backward compatibility: Old `cut_in_left.yaml` still works
- [ ] Multi-NPC example generates 3 actor trajectories
- [ ] New scenarios (cut-in right, following) produce expected behavior
- [ ] XOSC export works for all scenario types
- [ ] Adversarial mode works with new plugin system
- [ ] Documentation is complete and accurate
- [ ] Git history is clean with logical commits

---

## Success Metrics

**Achieved if**:

1. Adding a new scenario type (cut-in right) took < 2 hours
2. Plugin implementations are < 100 lines of code
3. Zero changes to core encoder/solver for new scenarios
4. Multi-actor support works with 2-3 NPCs
5. No performance degradation (< 5% slowdown on existing scenarios)

---

## Future Extensions

After completing this refactoring, consider:

1. **More scenario types**: Overtaking, emergency braking, merging
2. **Scenario composition**: Combine multiple behaviors (cut-in + deceleration)
3. **Advanced propositions**: Time-based constraints, complex spatial relationships
4. **Validation tools**: Satisfiability checker for behavior combinations
5. **Visualization**: Plot actor trajectories with matplotlib/plotly

---

## File Organization Summary

**Modified Files**:
- `src/dsl/types.rs` - Generic ActorSpec, multi-actor support
- `src/dsl/parser.rs` - Backward compatibility layer
- `src/ltl/formula.rs` - Expanded propositions (16 total)
- `src/ltl/generator.rs` - Refactored to use plugin system
- `src/solver/encoder.rs` - Multi-actor Z3 encoding
- `src/scenario/extractor.rs` - Multi-actor trajectory extraction
- `src/scenario/model.rs` - HashMap for actor trajectories
- `CLAUDE.md` - Plugin implementation guide

**New Files**:
- `src/ltl/plugin.rs` - Plugin trait, registry, 3 implementations
- `examples/cut_in_right.yaml`
- `examples/following.yaml`
- `examples/multi_npc_cut_in.yaml`

**Total Changed Files**: 11
**New Files**: 4
**Estimated Lines Changed**: ~800 lines

---

End of implementation specification.
