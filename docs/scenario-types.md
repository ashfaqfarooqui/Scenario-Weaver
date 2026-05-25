# Creating New Scenario Types

← [Back to README](../README.md)

This guide covers adding entirely new scenario types to ScenarioWeaver in Rust. For writing YAML specifications for existing types, see [CREATING_SCENARIOS.md](CREATING_SCENARIOS.md).

---

## Overview

Each scenario type implements the `ScenarioModel` trait:

```rust
pub trait ScenarioModel: Send + Sync {
    /// Validate scenario-specific requirements (actor count, required behavior keys, etc.)
    fn validate(&self, spec: &ScenarioSpec) -> Result<()>;

    /// Generate behavioral LTL formula (required)
    fn generate_ltl(&self, spec: &ScenarioSpec) -> Result<LTLFormula>;

    /// Generate safety constraints (optional — default: pairwise TTC + distance)
    fn generate_safety(&self, spec: &ScenarioSpec) -> Result<LTLFormula> {
        Ok(generate_default_safety(spec))
    }

    /// Add extra Z3 assertions (optional — default: none)
    fn add_z3_constraints(
        &self,
        spec: &ScenarioSpec,
        encoder: &crate::solver::Z3Encoder,
        solver: &z3::Solver,
        horizon: usize,
    ) -> Result<()> {
        Ok(())
    }
}
```

---

## Step-by-Step: Adding a Lane Change Scenario

### Step 1 — Create the implementation

Create `src/scenarios/lane_change.rs`:

```rust
//! Lane change scenario: NPC changes into ego's lane ahead of ego.

use crate::scenarios::ScenarioModel;
use crate::dsl::types::ScenarioSpec;
use crate::ltl::formula::{LTLFormula, Proposition};
use anyhow::Result;

pub struct LaneChangeModel;

impl ScenarioModel for LaneChangeModel {
    fn validate(&self, spec: &ScenarioSpec) -> Result<()> {
        if spec.actors.len() != 2 {
            anyhow::bail!("lane_change requires exactly 2 actors, found {}", spec.actors.len());
        }
        let npc = &spec.npcs()[0];
        if !npc.behavior.contains_key("lane_change_time") {
            anyhow::bail!("NPC missing 'lane_change_time' in behavior map");
        }
        Ok(())
    }

    fn generate_ltl(&self, spec: &ScenarioSpec) -> Result<LTLFormula> {
        let ego = spec.ego().map_err(|e| anyhow::anyhow!(e))?;
        let npc = &spec.npcs()[0];

        // Initial lane positions
        let init = LTLFormula::Atom(Proposition::InLane {
            actor: ego.id.clone(),
            lane: ego.lane,
        })
        .and(LTLFormula::Atom(Proposition::InLane {
            actor: npc.id.clone(),
            lane: npc.lane,
        }));

        // NPC eventually moves into target lane (computed from lane change deltas)
        let target_lane = npc.lane_changes.iter().fold(npc.lane as i64, |acc, lc| {
            acc + match lc.direction {
                LaneChangeDirection::Left => -1,
                LaneChangeDirection::Right => 1,
            }
        }).max(0) as usize;

        let behavior = LTLFormula::Atom(Proposition::InLane {
            actor: npc.id.clone(),
            lane: target_lane,
        })
        .eventually();

        Ok(init.and(behavior))
    }
}
```

### Step 2 — Register the scenario type

In `src/dsl/types.rs`, add the variant:

```rust
#[derive(Debug, Clone, Copy, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum ScenarioType {
    CutInLeft,
    CutInRight,
    LaneChange,  // NEW
}
```

Update `Display`:

```rust
ScenarioType::LaneChange => write!(f, "lane_change"),
```

Update `get_model()`:

```rust
ScenarioType::LaneChange =>
    Box::new(crate::scenarios::lane_change::LaneChangeModel),
```

### Step 3 — Export the module

In `src/scenarios/mod.rs`:

```rust
pub mod lane_change;  // NEW
```

That's it. The new type is now available.

---

## Create an Example YAML

`examples/lane_change.yaml`:

```yaml
scenario_type: lane_change
time_step: 0.5
duration: 10.0

actors:
  - id: ego
    role: ego
    lane: 0
    position: 50.0
    speed: 15.0
    acceleration: [-8.0, 3.0]

  - id: npc
    role: npc
    lane: 1
    position: [60.0, 80.0]
    speed: [12.0, 14.0]
    acceleration: [-8.0, 3.0]
    behavior:
      lane_change_time: [3.0, 7.0]

road:
  num_lanes: 2
  lane_width: 3.5
  lane_directions: [1, 1]

min_ttc: 3.0
min_distance: 5.0
```

Test it:

```bash
cargo run --release -- -i examples/lane_change.yaml -o output/ -n 5
cargo test lane_change
```

---

## Advanced: Custom Safety Constraints

Override `generate_safety()` to replace the default pairwise TTC + distance logic:

```rust
fn generate_safety(&self, spec: &ScenarioSpec) -> Result<LTLFormula> {
    // Example: only enforce distance, skip TTC
    Ok(custom_distance_constraint(spec))
}
```

## Advanced: Custom Z3 Assertions

Override `add_z3_constraints()` to inject raw Z3 assertions:

```rust
fn add_z3_constraints(
    &self,
    spec: &ScenarioSpec,
    encoder: &crate::solver::Z3Encoder,
    solver: &z3::Solver,
    horizon: usize,
) -> Result<()> {
    // Add scenario-specific Z3 constraints here
    Ok(())
}
```

---

## Multi-Actor Support

The system supports 1 ego + N NPCs:

```rust
let ego  = spec.ego()?;          // single ego actor
let npcs = spec.npcs();          // Vec<&ActorSpec>
let actor = spec.get_actor("npc1")?;  // by ID
```

Pairwise safety constraints are generated automatically for all actor combinations unless you override `generate_safety()`.

---

## Reference Implementations

| File | Scenario |
|------|----------|
| `src/scenarios/cut_in_left.rs` | NPC cuts in from left lane |
| `src/scenarios/cut_in_right.rs` | NPC cuts in from right lane |
| `src/scenarios/overtake_left.rs` | NPC overtakes via left lane (two lane changes) |
| `src/scenarios/pedestrian_crossing.rs` | Pedestrian crosses road |
