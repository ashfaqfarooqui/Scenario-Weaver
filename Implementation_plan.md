# CARLA Scenario Generator - Master Implementation Plan

## Project Overview

**Goal**: Build a tool that generates concrete driving test scenarios for CARLA simulator from high-level formal specifications using LTL (Linear Temporal Logic) + Z3 SMT solver.

**Approach**: Constraint-based scenario synthesis using bounded model checking

**Language**: Rust

**MVP Scope**: Support "cut-in from left" scenario only

---

## Quick Start

This is the master implementation plan. For detailed implementation steps:

1. Read this file for architecture overview and context
2. Follow phase-by-phase implementation in `plans/phase_XX_*.md` files
3. Each phase is self-contained with context, goals, steps, and validation
4. Complete phases sequentially (each depends on previous ones)

---

## Architecture Overview

### High-Level Pipeline

```
YAML DSL Input (high-level specification)
         ↓
    DSL Parser (serde_yaml)
         ↓
LTL Formula Generator (temporal logic)
         ↓
Z3 Constraint Encoder (bounded model checking)
         ↓
    Z3 SMT Solver
         ↓
 Scenario Extractor
         ↓
JSON Scenario Output (concrete trajectories)
```

### How It Works

**Input**: User writes a simple YAML file describing a scenario at high level
```yaml
scenario_type: cut_in_left
npc:
  position: [60.0, 80.0]  # range for solver to choose from
  speed: [12.0, 14.0]
min_ttc: 3.0              # safety constraint
```

**Processing**:
1. Parse YAML into structured types
2. Generate LTL formula capturing temporal behavior (e.g., "NPC eventually cuts into ego's lane")
3. Encode LTL + physics + safety as Z3 SMT constraints
4. Z3 solver finds concrete values satisfying all constraints

**Output**: JSON with complete trajectory data
```json
{
  "actors": [{
    "id": "npc",
    "states": [
      {"time": 0.0, "position": {"x": 72.5, "y": 1.75}, "lane": 0},
      {"time": 5.0, "position": {"x": 137.5, "y": 5.25}, "lane": 1}
    ]
  }]
}
```

---

## Key Components

### 1. DSL Layer (`src/dsl/`)
**Purpose**: Parse user input into structured data types

**Why**: Provides clean interface for scenario specification, hides complexity of LTL/Z3 from users

**Key Types**:
- `ScenarioSpec`: Root specification structure
- `ValueOrRange`: Supports both fixed values and ranges for Z3 to solve

### 2. LTL Layer (`src/ltl/`)
**Purpose**: Generate temporal logic formulas from high-level scenario descriptions

**Why**: LTL naturally expresses temporal properties like "eventually", "always", "until" that describe driving scenarios

**Key Types**:
- `LTLFormula`: AST for temporal logic (Eventually, Always, Until, etc.)
- `Proposition`: Atomic facts about scenario state (InLane, Ahead, TTC, etc.)

### 3. Solver Layer (`src/solver/`)
**Purpose**: Encode constraints into Z3 and find solutions

**Why**: Z3 can solve complex constraint systems mixing boolean logic, real arithmetic, and temporal properties

**Key Components**:
- **Encoder**: Translates LTL + physics + safety into Z3 constraints
- **Bounded Model Checking**: Expands temporal operators over fixed time horizon
- **Multi-solver**: Generates multiple diverse scenarios using blocking clauses

### 4. Scenario Layer (`src/scenario/`)
**Purpose**: Extract and format solutions as JSON

**Why**: Provides clean output format for consumption by other tools (visualization, CARLA, etc.)

---

## Example Workflow

### Input: `examples/cut_in_left.yaml`

```yaml
scenario_type: cut_in_left
time_step: 0.5
duration: 10.0

ego:
  lane: 1
  position: 50.0
  speed: 15.0

npc:
  lane: 0
  position: [60.0, 80.0]    # Z3 chooses value in this range
  speed: [12.0, 14.0]
  cut_in_time: [2.5, 7.5]

min_ttc: 3.0
min_distance: 5.0
lane_width: 3.5
num_scenarios: 1
```

### LTL Formula Generated

```
φ = InLane(ego, 1) ∧ InLane(npc, 0) ∧ Ahead(npc, ego)
    ∧ F(InLane(npc, 1))
    ∧ (InLane(npc, 0) U InLane(npc, 1))
    ∧ G(TTC(ego, npc) > 3.0)
    ∧ G(Distance(ego, npc) > 5.0)
```

**Translation**:
- Ego starts in right lane, NPC in left lane, NPC ahead
- Eventually (F) NPC moves to ego's lane
- NPC stays in left lane Until (U) it changes lanes
- Always (G) maintain min TTC and distance

### Z3 Variables Created

For each actor at each time step t ∈ [0, 20]:
- `px_t`, `py_t`: Position (Real)
- `vx_t`, `vy_t`: Velocity (Real)
- `lane_t`: Lane number (Int)

### Z3 Constraints Encoded

1. **Initial conditions**: `ego_px[0] = 50.0`, `npc_px[0] ∈ [60, 80]`, etc.
2. **Kinematics**: `px[t+1] = px[t] + vx[t] * dt`
3. **Lane coupling**: `py[t] = lane[t] * lane_width + lane_width/2`
4. **Bounded LTL**: `Eventually(NPC in lane 1)` → `(npc_lane[0]=1) ∨ (npc_lane[1]=1) ∨ ... ∨ (npc_lane[20]=1)`
5. **Safety**: `∀t: if same_lane then distance > 5.0`

### Solution Found by Z3

```
npc_px[0] = 72.5
npc_vx[0] = 13.0
cut_in_at_t = 10 (at 5.0 seconds)
... (all other variables)
```

### Output: `output.json`

Complete trajectory with positions, velocities, lanes at each 0.5s time step.

---

## Implementation Phases

Each phase builds on previous phases. Follow sequentially.

| Phase | File | Focus | Duration |
|-------|------|-------|----------|
| 1 | `plans/phase_01_setup.md` | Project setup, dependencies, structure | 1-2 days |
| 2 | `plans/phase_02_dsl.md` | DSL types and YAML parsing | 1 day |
| 3 | `plans/phase_03_ltl.md` | LTL formula AST and generation | 1-2 days |
| 4 | `plans/phase_04_scenario_model.md` | Output data structures | 0.5 day |
| 5 | `plans/phase_05_z3_foundation.md` | Z3 setup and variable creation | 1 day |
| 6 | `plans/phase_06_z3_physics.md` | Kinematic constraints | 1 day |
| 7 | `plans/phase_07_z3_ltl.md` | Bounded LTL encoding | 2 days |
| 8 | `plans/phase_08_z3_safety.md` | Safety constraints (TTC, distance) | 1 day |
| 9 | `plans/phase_09_extraction.md` | Extract scenarios from Z3 models | 1 day |
| 10 | `plans/phase_10_single_scenario.md` | End-to-end pipeline | 1 day |
| 11 | `plans/phase_11_multiple_scenarios.md` | Multi-scenario generation | 1 day |
| 12 | `plans/phase_12_cli.md` | CLI, logging, polish | 1-2 days |

**Total Estimated Time**: 10-15 days

---

## Project Structure

```
carla-scenario-generator/
├── Cargo.toml                           # Dependencies
├── README.md                            # User documentation
├── Implementation_plan.md               # This file
├── design_decisions.md                  # Design rationale
│
├── plans/                               # Phase-by-phase guides
│   ├── phase_01_setup.md
│   ├── phase_02_dsl.md
│   ├── ...
│   └── phase_12_cli.md
│
├── src/
│   ├── main.rs                          # CLI entry point
│   ├── lib.rs                           # Library exports
│   ├── error.rs                         # Error types
│   │
│   ├── dsl/                             # DSL parsing
│   │   ├── mod.rs
│   │   ├── types.rs                     # ScenarioSpec, ActorSpec, etc.
│   │   └── parser.rs                    # YAML → types
│   │
│   ├── ltl/                             # LTL formulas
│   │   ├── mod.rs
│   │   ├── formula.rs                   # LTLFormula AST
│   │   └── generator.rs                 # DSL → LTL
│   │
│   ├── solver/                          # Z3 integration
│   │   ├── mod.rs
│   │   ├── encoder.rs                   # Main Z3 encoding logic
│   │   ├── physics.rs                   # Kinematic constraints
│   │   └── multi_solve.rs               # Multiple scenario generation
│   │
│   └── scenario/                        # Output
│       ├── mod.rs
│       ├── model.rs                     # Scenario, ActorTrajectory, etc.
│       └── extractor.rs                 # Z3 model → Scenario
│
├── examples/
│   ├── cut_in_left.yaml                 # Example input
│   └── expected_output.json             # Example output
│
└── tests/
    ├── integration_test.rs
    └── fixtures/
```

---

## Key Technologies

### Rust
- **Why**: Memory safety, excellent tooling, good Z3 bindings, performance
- **Crates**: z3, serde, clap, anyhow, thiserror

### Z3 SMT Solver
- **Why**: Powerful constraint solver, handles mixed boolean/arithmetic/real constraints
- **Usage**: Encode all constraints (LTL, physics, safety) as SMT formulas, solve for concrete values

### Linear Temporal Logic (LTL)
- **Why**: Natural way to express temporal properties of driving scenarios
- **Operators**:
  - `F φ` (Eventually): "eventually NPC changes lanes"
  - `G φ` (Always): "always maintain safe distance"
  - `φ U ψ` (Until): "stay in left lane until lane change"

### Bounded Model Checking
- **Why**: Allows encoding infinite-horizon LTL into finite SMT constraints
- **How**: Fix time horizon (e.g., 20 time steps), expand temporal operators over this horizon

---

## Success Criteria

### MVP Completion Checklist

- [ ] Tool accepts YAML input file
- [ ] Generates valid JSON output
- [ ] JSON contains complete actor trajectories
- [ ] Generated scenarios are physically plausible:
  - [ ] NPC starts ahead in left lane
  - [ ] NPC cuts into ego's lane
  - [ ] Cut-in happens within specified time range
  - [ ] TTC never drops below minimum
  - [ ] Distance never drops below minimum
- [ ] Can generate multiple diverse scenarios from same spec
- [ ] CLI works: `cargo run -- -i input.yaml -o output.json`

### Validation Method

**Visual inspection** of generated JSON:
1. Plot trajectories (x vs time, lane vs time)
2. Verify temporal sequence makes sense
3. Check safety constraints are satisfied
4. Confirm diversity across multiple scenarios

---

## Next Steps After MVP

Once MVP is complete and validated:

1. **More Scenario Types**: Cut-in right, overtake, merge, pedestrian crossing
2. **Advanced Physics**: Acceleration, realistic lane changes
3. **OpenSCENARIO Export**: Generate XML for CARLA
4. **Optimization**: Find most challenging scenarios (minimize TTC)
5. **Visualization**: Tool to plot scenarios
6. **Real-World Data Integration**: Extract specs from driving logs

---

## Getting Started

1. Read `design_decisions.md` for architectural rationale
2. Start with `plans/phase_01_setup.md`
3. Complete each phase in order
4. Test thoroughly at each phase
5. Validate MVP with example scenarios

---

## Support Files

- **design_decisions.md**: Detailed explanation of design choices, alternative approaches, and rationale
- **plans/phase_XX_*.md**: Self-contained implementation guides for each phase
- **examples/**: Reference inputs and outputs
- **tests/**: Validation and testing strategy

---

## Implementation Philosophy

**For AI Agents Implementing This**:

1. **Follow phases sequentially** - each builds on previous work
2. **Test continuously** - verify each component before moving on
3. **Read phase context** - understand WHY before implementing WHAT
4. **Validate incrementally** - check outputs at each phase
5. **Reference examples** - use provided YAML/JSON as ground truth
6. **Ask for clarification** - if phase instructions are unclear, ask user

Each phase file is designed to be self-contained with:
- Prerequisites (what must be done first)
- Context (why this phase exists)
- Goals (what to achieve)
- Detailed steps (how to implement)
- Success criteria (how to validate)
- Next phase pointer (what comes after)

---

## Questions During Implementation

If you encounter issues:
1. Check if prerequisites from earlier phases are complete
2. Verify test cases pass
3. Review design_decisions.md for context
4. Consult example files
5. Ask user for clarification

---

## Timeline

**Week 1**: Phases 1-4 (Setup, DSL, LTL, Scenario model)
**Week 2**: Phases 5-9 (Z3 encoding - the core complexity)
**Week 3**: Phases 10-12 (Integration, multi-scenarios, polish)

Total: ~2-3 weeks for complete MVP
