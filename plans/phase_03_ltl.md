# Phase 3: LTL Layer

**Prerequisites**: Phase 2 complete (DSL types defined)

**Duration**: 1-2 days

---

## Context

Linear Temporal Logic (LTL) is how we express temporal properties of scenarios: "eventually the NPC cuts in", "always maintain safe distance", "stay in left lane until lane change".

**Why this phase**: The DSL gives us structured data, but we need to translate that into formal temporal logic that can be solved. LTL is the bridge between high-level intent and low-level constraints.

**What problem it solves**: Provides a formal, unambiguous representation of scenario temporal behavior.

---

## Goals

- [ ] Define LTL formula AST
- [ ] Define atomic propositions for driving scenarios
- [ ] Implement builder methods for ergonomic formula construction
- [ ] Implement LTL generator for cut-in scenario
- [ ] Add Display trait for debugging
- [ ] Write unit tests

---

## Implementation Steps

### Step 1: Define LTL Formula AST

**File**: `src/ltl/formula.rs`

```rust
//! LTL (Linear Temporal Logic) formula AST

use std::fmt;

/// LTL Formula Abstract Syntax Tree
#[derive(Debug, Clone, PartialEq)]
pub enum LTLFormula {
    // Atomic propositions
    Atom(Proposition),

    // Boolean operators
    Not(Box<LTLFormula>),
    And(Box<LTLFormula>, Box<LTLFormula>),
    Or(Box<LTLFormula>, Box<LTLFormula>),
    Implies(Box<LTLFormula>, Box<LTLFormula>),

    // Temporal operators
    Next(Box<LTLFormula>),                       // X φ
    Eventually(Box<LTLFormula>),                 // F φ (◊φ)
    Always(Box<LTLFormula>),                     // G φ (□φ)
    Until(Box<LTLFormula>, Box<LTLFormula>),     // φ U ψ
}

/// Atomic propositions about scenario state
#[derive(Debug, Clone, PartialEq)]
pub enum Proposition {
    /// Actor is in a specific lane
    InLane {
        actor: String,
        lane: usize,
    },

    /// Actor1 is ahead of Actor2 (longitudinally)
    Ahead {
        actor1: String,
        actor2: String,
    },

    /// Longitudinal distance between actors > threshold
    DistanceGT {
        actor1: String,
        actor2: String,
        distance: f64,
    },

    /// Time-To-Collision between actors > threshold
    TTCGT {
        actor1: String,
        actor2: String,
        ttc: f64,
    },
}

// Builder methods for ergonomic formula construction
impl LTLFormula {
    /// Logical AND
    pub fn and(self, other: Self) -> Self {
        LTLFormula::And(Box::new(self), Box::new(other))
    }

    /// Logical OR
    pub fn or(self, other: Self) -> Self {
        LTLFormula::Or(Box::new(self), Box::new(other))
    }

    /// Logical NOT
    pub fn not(self) -> Self {
        LTLFormula::Not(Box::new(self))
    }

    /// Implication
    pub fn implies(self, other: Self) -> Self {
        LTLFormula::Implies(Box::new(self), Box::new(other))
    }

    /// Eventually (F φ) - will be true at some future point
    pub fn eventually(self) -> Self {
        LTLFormula::Eventually(Box::new(self))
    }

    /// Always (G φ) - will be true at all future points
    pub fn always(self) -> Self {
        LTLFormula::Always(Box::new(self))
    }

    /// Until (φ U ψ) - φ holds until ψ becomes true
    pub fn until(self, other: Self) -> Self {
        LTLFormula::Until(Box::new(self), Box::new(other))
    }

    /// Next (X φ) - true in next time step
    pub fn next(self) -> Self {
        LTLFormula::Next(Box::new(self))
    }
}

/// Display implementation for debugging
impl fmt::Display for LTLFormula {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            LTLFormula::Atom(p) => write!(f, "{:?}", p),
            LTLFormula::Not(phi) => write!(f, "¬({})", phi),
            LTLFormula::And(phi, psi) => write!(f, "({} ∧ {})", phi, psi),
            LTLFormula::Or(phi, psi) => write!(f, "({} ∨ {})", phi, psi),
            LTLFormula::Implies(phi, psi) => write!(f, "({} → {})", phi, psi),
            LTLFormula::Next(phi) => write!(f, "X({})", phi),
            LTLFormula::Eventually(phi) => write!(f, "F({})", phi),
            LTLFormula::Always(phi) => write!(f, "G({})", phi),
            LTLFormula::Until(phi, psi) => write!(f, "({} U {})", phi, psi),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_builder_methods() {
        let p1 = LTLFormula::Atom(Proposition::InLane {
            actor: "ego".to_string(),
            lane: 1,
        });
        let p2 = LTLFormula::Atom(Proposition::InLane {
            actor: "npc".to_string(),
            lane: 0,
        });

        // Test and
        let formula = p1.clone().and(p2.clone());
        assert!(matches!(formula, LTLFormula::And(_, _)));

        // Test eventually
        let formula = p1.clone().eventually();
        assert!(matches!(formula, LTLFormula::Eventually(_)));

        // Test always
        let formula = p1.clone().always();
        assert!(matches!(formula, LTLFormula::Always(_)));

        // Test complex formula
        let formula = p1.clone().until(p2.clone()).eventually();
        println!("Formula: {}", formula);
    }

    #[test]
    fn test_display() {
        let formula = LTLFormula::Atom(Proposition::InLane {
            actor: "ego".to_string(),
            lane: 1,
        })
        .eventually();

        let display = format!("{}", formula);
        assert!(display.contains("F("));
        println!("Display: {}", display);
    }
}
```

### Step 2: Implement LTL Generator

**File**: `src/ltl/generator.rs`

```rust
//! LTL formula generation from DSL specifications

use crate::dsl::types::{ScenarioSpec, ScenarioType};
use crate::ltl::formula::{LTLFormula, Proposition};

pub struct LTLGenerator;

impl LTLGenerator {
    /// Generate LTL formula from scenario specification
    pub fn generate(spec: &ScenarioSpec) -> LTLFormula {
        match spec.scenario_type {
            ScenarioType::CutInLeft => Self::generate_cut_in_left(spec),
        }
    }

    /// Generate LTL formula for cut-in from left scenario
    ///
    /// Formula structure:
    /// φ = φ_init ∧ φ_behavior ∧ φ_safety
    ///
    /// Where:
    /// - φ_init: Initial conditions (lanes, positions)
    /// - φ_behavior: Cut-in behavior (eventually changes lanes, stays left until change)
    /// - φ_safety: Safety constraints (always maintain TTC and distance)
    pub fn generate_cut_in_left(spec: &ScenarioSpec) -> LTLFormula {
        let ego = "ego";
        let npc = "npc";

        Self::initial_conditions(spec, ego, npc)
            .and(Self::cut_in_behavior(spec, ego, npc))
            .and(Self::safety_constraints(spec, ego, npc))
    }

    /// Initial conditions for cut-in scenario
    ///
    /// At t=0:
    /// - Ego in right lane (lane 1)
    /// - NPC in left lane (lane 0)
    /// - NPC ahead of ego
    fn initial_conditions(spec: &ScenarioSpec, ego: &str, npc: &str) -> LTLFormula {
        LTLFormula::Atom(Proposition::InLane {
            actor: ego.to_string(),
            lane: spec.ego.lane,
        })
        .and(LTLFormula::Atom(Proposition::InLane {
            actor: npc.to_string(),
            lane: spec.npc.lane,
        }))
        .and(LTLFormula::Atom(Proposition::Ahead {
            actor1: npc.to_string(),
            actor2: ego.to_string(),
        }))
    }

    /// Cut-in behavior
    ///
    /// - Eventually: NPC moves to ego's lane
    /// - Until: NPC stays in left lane until it changes
    fn cut_in_behavior(spec: &ScenarioSpec, ego: &str, npc: &str) -> LTLFormula {
        let target_lane = spec.ego.lane;
        let initial_lane = spec.npc.lane;

        // Eventually NPC moves to ego's lane: F(InLane(npc, 1))
        let eventually_in_lane = LTLFormula::Atom(Proposition::InLane {
            actor: npc.to_string(),
            lane: target_lane,
        })
        .eventually();

        // NPC stays in left lane UNTIL it changes: InLane(npc, 0) U InLane(npc, 1)
        let stay_until_change = LTLFormula::Atom(Proposition::InLane {
            actor: npc.to_string(),
            lane: initial_lane,
        })
        .until(LTLFormula::Atom(Proposition::InLane {
            actor: npc.to_string(),
            lane: target_lane,
        }));

        eventually_in_lane.and(stay_until_change)
    }

    /// Safety constraints
    ///
    /// - Always: TTC > min_ttc
    /// - Always: Distance > min_distance
    fn safety_constraints(spec: &ScenarioSpec, ego: &str, npc: &str) -> LTLFormula {
        // Always maintain minimum TTC: G(TTC > min_ttc)
        let ttc_constraint = LTLFormula::Atom(Proposition::TTCGT {
            actor1: ego.to_string(),
            actor2: npc.to_string(),
            ttc: spec.min_ttc,
        })
        .always();

        // Always maintain minimum distance: G(Distance > min_distance)
        let distance_constraint = LTLFormula::Atom(Proposition::DistanceGT {
            actor1: ego.to_string(),
            actor2: npc.to_string(),
            distance: spec.min_distance,
        })
        .always();

        ttc_constraint.and(distance_constraint)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dsl::types::{ActorSpec, NpcSpec, ValueOrRange};

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
    fn test_generate_cut_in_left() {
        let spec = create_test_spec();
        let formula = LTLGenerator::generate_cut_in_left(&spec);

        println!("Generated LTL formula:");
        println!("{}", formula);

        // Should be a conjunction (AND)
        assert!(matches!(formula, LTLFormula::And(_, _)));
    }

    #[test]
    fn test_initial_conditions() {
        let spec = create_test_spec();
        let formula = LTLGenerator::initial_conditions(&spec, "ego", "npc");

        println!("Initial conditions:");
        println!("{}", formula);

        // Should contain InLane propositions
        let formula_str = format!("{}", formula);
        assert!(formula_str.contains("InLane"));
        assert!(formula_str.contains("Ahead"));
    }

    #[test]
    fn test_cut_in_behavior() {
        let spec = create_test_spec();
        let formula = LTLGenerator::cut_in_behavior(&spec, "ego", "npc");

        println!("Cut-in behavior:");
        println!("{}", formula);

        // Should contain Eventually and Until
        let formula_str = format!("{}", formula);
        assert!(formula_str.contains("F("));
        assert!(formula_str.contains(" U "));
    }

    #[test]
    fn test_safety_constraints() {
        let spec = create_test_spec();
        let formula = LTLGenerator::safety_constraints(&spec, "ego", "npc");

        println!("Safety constraints:");
        println!("{}", formula);

        // Should contain Always operators
        let formula_str = format!("{}", formula);
        assert!(formula_str.contains("G("));
        assert!(formula_str.contains("TTCGT"));
        assert!(formula_str.contains("DistanceGT"));
    }
}
```

### Step 3: Update Module Exports

**src/ltl/mod.rs**:
```rust
//! LTL (Linear Temporal Logic) module

pub mod formula;
pub mod generator;

pub use formula::{LTLFormula, Proposition};
pub use generator::LTLGenerator;
```

---

## Success Criteria

### Verification Steps

1. **Unit tests pass**:
   ```bash
   cargo test ltl
   ```

2. **Formula generation works**:
   ```bash
   cargo test test_generate_cut_in_left -- --nocapture
   ```
   Should print a readable LTL formula

3. **Formula structure is correct**:
   - Contains initial conditions (InLane, Ahead)
   - Contains temporal operators (Eventually, Always, Until)
   - Contains safety constraints (TTCGT, DistanceGT)

### Checklist

- [ ] LTL formula AST defined
- [ ] All temporal operators implemented
- [ ] Builder methods work
- [ ] Display trait shows readable formulas
- [ ] LTL generator creates correct formula
- [ ] Tests pass
- [ ] Formula printed to console looks correct

---

## Testing

```bash
# Run LTL tests with output
cargo test ltl -- --nocapture

# Test specific function
cargo test test_generate_cut_in_left -- --nocapture

# Check the generated formula
cargo test test_cut_in_behavior -- --nocapture
```

Expected output should look like:
```
Generated LTL formula:
(((InLane { actor: "ego", lane: 1 } ∧ InLane { actor: "npc", lane: 0 }) ∧ Ahead { actor1: "npc", actor2: "ego" }) ∧ ((F(InLane { actor: "npc", lane: 1 }) ∧ (InLane { actor: "npc", lane: 0 } U InLane { actor: "npc", lane: 1 })) ∧ (G(TTCGT { actor1: "ego", actor2: "npc", ttc: 3.0 }) ∧ G(DistanceGT { actor1: "ego", actor2: "npc", distance: 5.0 }))))
```

---

## Next Phase

Once this phase is complete and all tests pass:

**→ Continue to [Phase 4: Scenario Model](phase_04_scenario_model.md)**

Phase 4 will define the output JSON data structures.

---

## Notes for AI Agents

**What you just built**:
- LTL formula AST (formal representation of temporal logic)
- Atomic propositions for driving scenarios
- LTL generator that translates DSL → LTL
- Human-readable display of formulas

**What you can now do**:
- Express temporal properties formally
- Generate formulas from DSL specs
- Debug formulas by printing them

**What's next**:
- Phase 4: Define JSON output format
- Phase 7: Encode these LTL formulas into Z3 constraints (bounded model checking)
