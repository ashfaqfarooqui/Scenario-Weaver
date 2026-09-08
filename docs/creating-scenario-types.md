# Creating New Scenario Types

← [Back to README](../README.md)

This guide covers adding a new scenario type to ScenarioWeaver in Rust. For writing YAML for the
scenario types that already exist, see [authoring-scenarios.md](authoring-scenarios.md) and
[yaml-reference.md](yaml-reference.md).

## The `ScenarioModel` trait

Every scenario type is a struct implementing `ScenarioModel` (`src/scenarios/mod.rs`):

```rust
pub trait ScenarioModel: Send + Sync {
    /// Validate scenario-specific requirements (actor count, required lane changes,
    /// required behavior keys, ...). Default: no extra checks.
    fn validate(&self, _spec: &ScenarioSpec) -> Result<()> {
        Ok(())
    }

    /// Generate the behavioral LTL formula. Required: this is the one method every
    /// scenario type must define. Covers initial conditions and scenario-specific
    /// behavior; safety is handled separately by generate_safety().
    fn generate_ltl(&self, spec: &ScenarioSpec) -> Result<LTLFormula>;

    /// Generate safety constraints. Default: generate_default_safety(spec), pairwise
    /// TTC, distance, lateral distance, relative velocity, and per-actor velocity
    /// constraints across every actor pair, one per constraint mode in the spec.
    fn generate_safety(&self, spec: &ScenarioSpec) -> Result<LTLFormula> {
        Ok(generate_default_safety(spec))
    }

    /// Add scenario-specific Z3 assertions beyond the LTL encoding. Default: none.
    fn add_z3_constraints(
        &self,
        _spec: &ScenarioSpec,
        _encoder: &dyn crate::solver::EncoderAccessor,
        _backend: &dyn crate::solver::Z3Backend,
        _horizon: usize,
    ) -> Result<()> {
        Ok(())
    }
}
```

Only `generate_ltl` is required. Most scenario types implement `validate` and `generate_ltl` and
take the default `generate_safety`; `add_z3_constraints` is rarely needed at all – none of the
five current types override it.

## LTL propositions and combinators

`generate_ltl` and `generate_safety` build a Linear Temporal Logic (LTL) formula, `LTLFormula`
(`src/ltl/formula.rs`), out of atomic propositions and the usual combinators.

**Combinators**, available as builder methods on `LTLFormula`:

| Method | Meaning |
|--------|---------|
| `.and(other)` | logical conjunction |
| `.or(other)` | logical disjunction |
| `.negate()` | logical negation |
| `.implies(other)` | material implication |
| `.always()` | `G φ`, holds at every remaining step |
| `.eventually()` | `F φ`, holds at some future step |
| `.until(other)` | `φ U ψ` |

`LTLFormula::True` and `LTLFormula::False` are the trivial cases; a scenario type with no
behavioral requirement beyond kinematics (`head_on` is one, see below) returns `Ok(LTLFormula::True)`.

**Propositions** (`Proposition` enum), the atoms these combinators wrap:

| Proposition | Meaning |
|-------------|---------|
| `InLane { actor, lane }` | actor occupies the given lane |
| `Ahead { actor1, actor2 }` | actor1 is longitudinally ahead of actor2 |
| `DistanceGT { actor1, actor2, distance }` | longitudinal gap exceeds the threshold |
| `TTCGT { actor1, actor2, ttc }` | time-to-collision exceeds the threshold |
| `Approaching { follower, leader }` | follower is behind leader and gaining (the antecedent that makes an `enforce`d TTC non-vacuous, see below) |
| `LateralDistanceGT { actor1, actor2, distance }` | lateral (side-by-side) gap exceeds the threshold |
| `RelativeVelocityGT { actor1, actor2, velocity }` | speed difference exceeds the threshold |
| `VelocityGT` / `VelocityLT { actor, velocity }` | longitudinal speed above/below a threshold |
| `OnSidewalk { actor, side }` | pedestrian is off the road, on the named side |
| `CrossingRoad { actor }` | pedestrian is actively crossing |
| `RectangularDistanceGT { actor1, actor2, threshold_x, threshold_y }` | `\|dx\| > threshold_x ∨ \|dy\| > threshold_y` (the pedestrian safety box) |
| `PedestrianTTCGT { ego, pedestrian, ttc }` | perpendicular-crossing TTC exceeds the threshold |
| `PedestrianTTCGuard { ego, pedestrian }` | the guard under which a pedestrian TTC is defined at all |

Two of these, `Approaching` and `PedestrianTTCGuard`, exist specifically to avoid a vacuity
trap. `TTCGT` is a guarded implication ("whenever this pair is converging, TTC exceeds the
threshold"), so `G(TTCGT(...))` is satisfied vacuously by a pair that never converges at all, and
an `enforce`d `min_ttc` then constrains nothing. Both `cut_in_left`/`cut_in_right` and
`pedestrian_crossing` instead assert `G(antecedent → Approaching/PedestrianTTCGuard)`, where the
antecedent is something the scenario's own behavior already forces true somewhere. If you are
writing a new scenario type where TTC matters, and the type has actors that might never actually
converge, follow the same pattern rather than asserting `TTCGT` bare, otherwise the validator
can silently report `min_ttc: null` for a scenario that "enforced" it.

## Reusable helpers in `scenarios/mod.rs`

- **`cut_in_conflict(ego_id, npc_id, ego_lane, target_lane, direction) -> LTLFormula`** builds
  exactly the guarded-`Approaching` conflict above for a merging NPC and an ego it should be
  closing on. It returns `LTLFormula::True` if the NPC's target lane isn't the ego's lane at all
  (no conflict to assert), and switches `follower`/`leader` based on `direction` so it works for
  actors traveling in either `+x` or `-x`. Both cut-in models call it; a new lane-change-based
  scenario type is the likely next caller.
- **`push_constraint(constraints, mode, atom, polarity)`** is the one place that turns a
  `ConstraintMode` (`Enforce`/`Violate`/`Ignore`) and an `AtomPolarity` (`Positive` if the atom
  itself is the safe condition, `Negated` if its negation is) into the right LTL formula:
  `G(atom)`, `F(¬atom)`, or nothing. `generate_default_safety` calls it once per constraint per
  actor pair; `head_on` and `pedestrian_crossing` call it directly from their own
  `generate_safety` overrides. Any custom safety logic should go through this rather than
  hand-rolling the enforce/violate/ignore match again; see
  [adversarial-generation.md](adversarial-generation.md) for what each mode means at the YAML
  level.
- **`generate_default_safety(spec) -> LTLFormula`** is what `ScenarioModel::generate_safety`'s
  default implementation calls: pairwise TTC and distance for every actor pair, plus lateral
  distance and relative velocity when the corresponding YAML fields are set, plus per-actor
  velocity bounds. Override `generate_safety` only when the default's "every pair, same
  predicate" shape doesn't fit, a scenario with a distinguished pair (`head_on`) or a
  non-longitudinal safety region (`pedestrian_crossing`).

## Reference walkthrough: `cut_in_left`

`src/scenarios/cut_in_left.rs` implements `CutInLeftModel`. Its shape is representative of the
lane-change scenario types:

- **`validate`** requires exactly two actors and at least one `lane_changes` entry on the NPC:
  cheap structural checks that catch a malformed spec before any Z3 encoding happens.
- **`generate_ltl`** delegates to two private helpers and conjoins their results:
  - **`initial_conditions`** asserts each actor's starting lane, and, only when both actors
    travel the same direction, that the NPC starts ahead of the ego. On a bidirectional road
    with actors in opposite-direction lanes, `Ahead` has no meaningful reading, so it's
    conditionally omitted rather than asserted vacuously true or false.
  - **`cut_in_behavior`** computes the NPC's target lane by summing its `lane_changes` deltas
    (accounting for direction: `Right` means a lower lane index for a backward-traveling actor,
    since "right" is relative to the actor, not the road frame), asserts that the NPC holds its
    initial lane *until* it reaches the target lane, and, if the ego and NPC share a direction,
    conjoins `cut_in_conflict` so the merge is required to happen in front of a closing ego,
    not into empty road behind it.
- **`generate_safety`** is not overridden: cut-in scenarios use `generate_default_safety`
  unmodified.
- **`add_z3_constraints`** is a no-op; lane-change kinematics themselves are handled by the
  encoder from the `lane_changes` config, not by this trait method.

`cut_in_right.rs` and `overtake_left.rs` follow the same shape with different lane geometry;
`overtake_left` composes two sequential lane changes (into the passing lane, then back) the way
`head_on`'s ego does.

## The five current scenario types

| Type | YAML value | One-line description |
|------|------------|----------------------|
| `CutInLeft` | `cut_in_left` | NPC starts in the lane to the ego's left, ahead, and cuts in front of the ego. |
| `CutInRight` | `cut_in_right` | NPC starts in the lane to the ego's right, ahead, and cuts in front of the ego. |
| `OvertakeLeft` | `overtake_left` | NPC starts behind the ego, passes via the left lane (two sequential lane changes), and ends up ahead. |
| `PedestrianCrossing` | `pedestrian_crossing` | A pedestrian crosses the road perpendicular to the ego's path while the ego approaches. |
| `HeadOn` | `head_on` | The ego overtakes a slower vehicle on a bidirectional road, briefly entering the oncoming lane where a third actor approaches head-on. |

## Step-by-step: adding a new scenario type

The steps below add a `lane_change` type, an NPC that changes lanes on a timer with no cut-in
conflict requirement, as a worked example. Substitute your own scenario's behavior.

### 1. Create `src/scenarios/lane_change.rs`

```rust
//! Lane change scenario: NPC changes lanes on a timer, independent of the ego.

use crate::dsl::types::{LaneChangeDirection, ScenarioSpec};
use crate::error::{Result, ScenarioGenError};
use crate::ltl::formula::{LTLFormula, Proposition};
use crate::scenarios::ScenarioModel;

pub(crate) struct LaneChangeModel;

impl ScenarioModel for LaneChangeModel {
    fn validate(&self, spec: &ScenarioSpec) -> Result<()> {
        if spec.actors.len() != 2 {
            return Err(ScenarioGenError::InvalidSpec(format!(
                "lane_change requires exactly 2 actors, found {}",
                spec.actors.len()
            )));
        }
        let npc = &spec.npcs()[0];
        if npc.lane_changes.is_empty() {
            return Err(ScenarioGenError::InvalidSpec(
                "lane_change requires at least one lane_change on the NPC".to_string(),
            ));
        }
        Ok(())
    }

    fn generate_ltl(&self, spec: &ScenarioSpec) -> Result<LTLFormula> {
        let ego = spec.ego().map_err(ScenarioGenError::InvalidSpec)?;
        let npc = &spec.npcs()[0];

        let init = LTLFormula::Atom(Proposition::InLane {
            actor: ego.id.clone(),
            lane: ego.lane,
        })
        .and(LTLFormula::Atom(Proposition::InLane {
            actor: npc.id.clone(),
            lane: npc.lane,
        }));

        let total_delta: i64 = npc
            .lane_changes
            .iter()
            .map(|lc| match lc.direction {
                LaneChangeDirection::Left => -(npc.direction as i64),
                LaneChangeDirection::Right => npc.direction as i64,
            })
            .sum();
        let target_lane = (npc.lane as i64 + total_delta).max(0) as usize;

        let ends_in_target_lane = LTLFormula::Atom(Proposition::InLane {
            actor: npc.id.clone(),
            lane: target_lane,
        })
        .eventually();

        Ok(init.and(ends_in_target_lane))
    }

    // generate_safety and add_z3_constraints take their defaults: pairwise
    // safety, no extra Z3 assertions.
}
```

`generate_safety` and `add_z3_constraints` are omitted entirely – the trait's defaults apply.

### 2. Register the module

In `src/scenarios/mod.rs`, add:

```rust
pub(crate) mod lane_change;
```

alongside the other five `pub(crate) mod` declarations at the bottom of the file.

### 3. Add the `ScenarioType` variant

In `src/dsl/types.rs`, add a variant to the enum:

```rust
#[derive(Debug, Clone, Copy, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum ScenarioType {
    CutInLeft,
    CutInRight,
    OvertakeLeft,
    PedestrianCrossing,
    HeadOn,
    LaneChange, // new: YAML value `lane_change` via #[serde(rename_all = "snake_case")]
}
```

Add the matching `Display` arm:

```rust
ScenarioType::LaneChange => write!(f, "lane_change"),
```

Add the matching `get_model` arm:

```rust
ScenarioType::LaneChange => Box::new(crate::scenarios::lane_change::LaneChangeModel),
```

`#[serde(rename_all = "snake_case")]` on the enum means the YAML value falls out of the variant
name automatically (`LaneChange` → `lane_change`) – there is no separate string to keep in sync
beyond the `Display` arm, which exists because the tool also prints the scenario type in
human-readable output.

### 4. Add an example YAML and a test

`examples/lane_change.yaml`:

```yaml
scenario_type: lane_change
time_step: 0.5
duration: 10.0

road:
  num_lanes: 2
  lane_width: 3.5
  lane_directions: [1, 1]

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
    lane_changes:
      - direction: left
        start_time: [3.0, 7.0]
        duration: [2.0, 3.0]

min_ttc: 3.0
min_distance: 5.0
```

```bash
cargo run --release -- -i examples/lane_change.yaml -o output/ -n 5
```

A unit test on the model itself (`validate`, `generate_ltl`) belongs in a `#[cfg(test)] mod
tests` block at the bottom of `lane_change.rs`, following the pattern in `cut_in_left.rs`,
`head_on.rs`, and the other four models: a `create_test_spec()` helper building a minimal valid
`ScenarioSpec`, then one test per validation failure mode and one confirming the generated
formula's shape.

That completes the addition: `cargo build` picks up the new variant, and
`cargo run -- -i examples/lane_change.yaml -o output/` exercises the full pipeline, parsing,
LTL generation, Z3 encoding, solving, metric extraction, and export, end to end.

## See also

- [architecture.md](architecture.md): how a `ScenarioModel`'s LTL formula reaches the Z3
  encoder and where in the pipeline `add_z3_constraints` runs relative to the LTL encoding.
- [yaml-reference.md](yaml-reference.md): the full `ScenarioSpec` schema a new type's YAML
  examples draw from.
- [authoring-scenarios.md](authoring-scenarios.md): writing YAML for existing types.
- [coordinate-systems.md](coordinate-systems.md): Cartesian vs. bicycle encoding; relevant if
  a new scenario type needs coordinate-system-specific behavior in `add_z3_constraints`.
- [adversarial-generation.md](adversarial-generation.md): constraint modes, and what a
  `generate_safety` override needs to respect if it doesn't use `generate_default_safety`.
