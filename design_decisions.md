# CARLA Scenario Generator - Design Decisions

This document captures the complete design discussion, rationale for technical choices, and alternatives considered.

---

## Table of Contents

1. [Problem Statement](#problem-statement)
2. [Formal Methods Exploration](#formal-methods-exploration)
3. [Why LTL + Z3](#why-ltl--z3)
4. [Architecture Decisions](#architecture-decisions)
5. [Input/Output Design](#inputoutput-design)
6. [LTL and Z3 Division of Labor](#ltl-and-z3-division-of-labor)
7. [Bounded Model Checking Approach](#bounded-model-checking-approach)
8. [Key Trade-offs](#key-trade-offs)

---

## Problem Statement

### Goal
Build a tool for generating OpenSCENARIO files for testing autonomous vehicles in CARLA simulator.

### High-Level Idea
Instead of manually writing low-level OpenSCENARIO XML, users write high-level specifications in a formal modeling language. The tool automatically generates concrete, validated scenarios.

### Benefits
- **Higher abstraction**: Easier to specify intent
- **Verification**: Can validate properties before generation
- **Automation**: Generate many test scenarios from one spec
- **Formal guarantees**: Scenarios provably satisfy constraints

### Scope for MVP
- **Input**: Formal specification (high-level)
- **Output**: JSON scenario description (defer OpenSCENARIO XML to later)
- **Scenario types**: Start with "cut-in from left" only
- **Validation**: Visual inspection (manual)

---

## Formal Methods Exploration

### Formal Semantics Considered

We explored multiple formal modeling approaches:

#### 1. Temporal Logics

**LTL (Linear Temporal Logic)**
- **Strengths**: Simple, good tool support, qualitative temporal properties
- **Limitations**: No continuous time/values natively (needs discretization)
- **Use case**: Abstract requirements, safety properties

**STL/MTL (Signal/Metric Temporal Logic)**
- **Strengths**: Real-time bounds, continuous signals, metric constraints
- **Limitations**: More complex, synthesis computationally expensive
- **Use case**: Real-time embedded systems, tight timing requirements

**ATL (Alternating-Time Temporal Logic)**
- **Strengths**: Multi-agent strategic reasoning, game-theoretic
- **Limitations**: Complex solving, less tool support
- **Use case**: Adversarial testing, cooperative scenarios

#### 2. Hybrid/Continuous Logics

**Differential Dynamic Logic (dL)**
- **Strengths**: Handles continuous dynamics + discrete actions, formal proofs
- **Limitations**: Steep learning curve, synthesis harder than verification
- **Tool**: KeYmaera X

**Hybrid Automata / Timed Automata**
- **Strengths**: Visual modeling, mature tools (UPPAAL), complex timing
- **Limitations**: Not a direct specification language, manual modeling

#### 3. Spatial & Geometric Logics

**Spatial + Temporal Logic**
- **Strengths**: Natural for geometric relationships, qualitative reasoning
- **Limitations**: Limited tool support for combined spatio-temporal

#### 4. Process Algebras

**CSP (Communicating Sequential Processes)**
- **Strengths**: Concurrent behaviors, composition, refinement checking
- **Limitations**: Abstract (not metric), not designed for continuous space/time

#### 5. Probabilistic/Stochastic

**pSTL (Probabilistic STL)**
- **Strengths**: Uncertainty quantification, statistical model checking
- **Limitations**: Requires distribution knowledge, complex solving

**PRISM / MDPs**
- **Strengths**: Probabilistic model checking, strategy synthesis
- **Limitations**: Discrete state spaces, abstraction gap

#### 6. Scenario-Specific Languages

**SCENIC**
- **Strengths**: Domain-specific, intuitive, probabilistic sampling, Python-based
- **Limitations**: Less formal (no verification), sampling-based (not exhaustive)

**GeoScenario / M-SDL**
- Similar domain-specific approaches from research

#### 7. Logic Programming

**Answer Set Programming (ASP) / Clingo**
- **Strengths**: Declarative, efficient solving, combinatorial generation
- **Limitations**: Not designed for continuous dynamics, needs discretization

---

## Why LTL + Z3

### Decision: Hybrid Approach

After exploring alternatives, we chose **LTL (for temporal structure) + Z3 SMT solver (for quantitative constraints)**.

### Rationale

#### What LTL Provides
- **Temporal reasoning**: Naturally expresses "eventually", "always", "until"
- **Qualitative structure**: Captures sequence of events (NPC cuts in, ego maintains speed)
- **Simple semantics**: Easier than STL/MTL, sufficient for discrete time steps
- **Tool support**: Well-understood, mature theory

#### What Z3 Provides
- **Quantitative constraints**: Real numbers (positions, speeds), arithmetic
- **Physics**: Kinematic equations, continuous dynamics
- **Safety**: Numeric bounds (TTC > 3.0, distance > 5.0)
- **Powerful solving**: Mixed Boolean/arithmetic, optimization

#### Why Both Are Needed

| Aspect | LTL Alone | Z3 Alone | **LTL + Z3** |
|--------|-----------|----------|--------------|
| Temporal ordering | ✅ | ❌ | ✅ |
| "Eventually happens" | ✅ | ❌ | ✅ |
| Concrete values | ❌ | ✅ | ✅ |
| Physics constraints | ❌ | ✅ | ✅ |
| Safety properties | Partial | ✅ | ✅ |
| Scenario structure | ✅ | ❌ | ✅ |

**Example**:
- Without LTL: You'd manually specify "lane change at step 3" - no temporal reasoning
- Without Z3: You'd get abstract traces with no concrete positions/speeds

### Alternatives Not Chosen

**Pure STL/MTL + Synthesis**
- **Rejected because**: Overkill for simple scenarios, synthesis expensive, steep learning curve
- **When to reconsider**: If we need complex real-time properties or continuous-time verification

**Scenic (extend with backend)**
- **Rejected because**: Less formal, probabilistic sampling vs. deterministic synthesis
- **When to reconsider**: If we prioritize rapid prototyping over formal guarantees

**ASP/Clingo**
- **Rejected because**: Not natural for continuous dynamics, would need Z3 anyway for physics
- **When to reconsider**: If scenarios become highly combinatorial (many discrete choices)

**dL (KeYmaera X)**
- **Rejected because**: Focus on verification not synthesis, steep learning curve
- **When to reconsider**: If we need formal safety proofs for generated scenarios

---

## Architecture Decisions

### Approach: Custom DSL + Bounded Model Checking

#### Components

```
High-level DSL (YAML)
     ↓
LTL Formula Generator
     ↓
Z3 Constraint Encoder (bounded model checking)
     ↓
Z3 Solver
     ↓
Scenario Instance
```

#### Why This Architecture

1. **Simpler than full STL synthesis**: Declarative constraints instead of complex temporal formulas
2. **More formal than pure Scenic**: Z3 gives constraint solving and verification
3. **Incremental complexity**: Start simple (LTL basics), add features later (optimization, probabilities)
4. **Direct control**: We control the entire pipeline, no external LTL solvers needed
5. **Easier automation**: Extracting constraints from real-world data is straightforward

### Language Choice: Rust

**Decision**: Implement in Rust

**Rationale**:
- Memory safety without GC (important for performance)
- Excellent Z3 bindings (`z3` crate)
- Strong type system (catch errors at compile time)
- Great tooling (cargo, clippy, rustfmt)
- Good for production tools

**Alternatives**:
- **Python**: Easier prototyping, but slower, less type safety
- **TypeScript/Node**: Cross-platform, but Z3 bindings less mature

### Modular Design

**Decision**: Clear layer separation (DSL → LTL → Solver → Scenario)

**Rationale**:
- **Testable**: Each layer can be unit tested independently
- **Maintainable**: Changes to one layer don't cascade
- **Extensible**: Easy to add new scenario types or constraint types
- **Understandable**: Clear data flow, easy to reason about

---

## Input/Output Design

### Input Format

**Decision**: YAML DSL (simplified, flat structure)

#### Evolution of DSL Design

**Initial proposal**: Hierarchical YAML with sections for metadata, world, actors, behavior, constraints, generation

**User feedback**: "Simplify it - too verbose for MVP"

**Final decision**: Flatter structure with minimal nesting

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
  position: [60.0, 80.0]  # range
  speed: [12.0, 14.0]
  cut_in_time: [2.5, 7.5]

min_ttc: 3.0
min_distance: 5.0
lane_width: 3.5

num_scenarios: 5
```

**Rationale**:
- **Concise**: Easy to read and write
- **Ranges**: `[min, max]` syntax lets Z3 choose values
- **Extensible**: Can add fields without breaking structure
- **Rust-friendly**: Clean mapping to Rust structs with serde

**Alternatives considered**:
- **Python DSL**: More flexible but requires Python runtime
- **JSON**: Valid but less human-friendly than YAML
- **Custom syntax**: More work, harder to parse

### Output Format

**Decision**: JSON (not OpenSCENARIO XML for MVP)

**Rationale**:
- **Focus**: Get LTL + Z3 working first
- **Validation**: JSON easier to inspect manually
- **Defer complexity**: OpenSCENARIO mapping can come later
- **Intermediate format**: JSON can be transformed to other formats

**JSON Schema**:
```json
{
  "scenario_id": "uuid",
  "scenario_type": "cut_in_left",
  "actors": [
    {
      "id": "ego",
      "states": [
        {"time": 0.0, "position": {"x": 50, "y": 5.25}, "velocity": {...}, "lane": 1},
        ...
      ]
    }
  ],
  "validation": {
    "min_ttc": 3.8,
    "min_distance": 6.2,
    "all_constraints_satisfied": true
  }
}
```

### Scenario Generation Modes

**Decision**: Support both single and multiple scenario generation

**User requirement**: "Let the user decide how many scenarios to generate"

**Implementation**:
- `num_scenarios: 1` → Single scenario
- `num_scenarios: N` → Generate N diverse scenarios using blocking clauses

**Rationale**:
- **Flexibility**: Single for debugging, multiple for test coverage
- **Same interface**: No mode switching, just a parameter
- **Diversity**: Blocking clauses ensure scenarios are different

---

## LTL and Z3 Division of Labor

### What Happens on LTL Side

**Role**: Define **temporal structure** and **qualitative properties**

**Operators**:
- `F φ` (Eventually): "NPC eventually changes lanes"
- `G φ` (Always): "Always maintain safe distance"
- `φ U ψ` (Until): "Stay in left lane until lane change"
- `X φ` (Next): "In next time step, ..."

**Output**: LTL formula (AST)

**Example for Cut-In**:
```
φ = InLane(ego, 1) ∧ InLane(npc, 0) ∧ Ahead(npc, ego)
    ∧ F(InLane(npc, 1))
    ∧ (InLane(npc, 0) U InLane(npc, 1))
    ∧ G(TTC > 3.0)
    ∧ G(Distance > 5.0)
```

### What Happens on Z3 Side

**Role**: Solve for **concrete numeric values** satisfying constraints

**Variables**: For each actor at each time step:
- `px_t`, `py_t` (position, Real)
- `vx_t`, `vy_t` (velocity, Real)
- `lane_t` (lane number, Int)

**Constraints**:
1. **Initial conditions**: `ego_px[0] = 50.0`, `npc_px[0] ∈ [60, 80]`
2. **Physics**: `px[t+1] = px[t] + vx[t] * dt`
3. **LTL (bounded)**: Expand temporal operators over time horizon
4. **Safety**: `TTC[t] > 3.0`, `distance[t] > 5.0`

**Output**: Satisfying assignment (concrete values for all variables)

**Example Solution**:
```
npc_px[0] = 72.5
npc_vx[0] = 13.0
npc_lane[10] = 1  (lane change at t=10)
...
```

### Integration: Bounded Model Checking

**Key insight**: We don't use a separate LTL solver. Instead, we **encode LTL directly into Z3** using bounded model checking.

**How it works**:
1. Fix time horizon (e.g., N = 20 time steps)
2. Expand temporal operators over this horizon
3. Encode as Z3 Boolean/Real constraints

**Example**:

LTL: `Eventually(φ)` → "φ is true at some time step"

Z3 encoding:
```
φ[0] ∨ φ[1] ∨ φ[2] ∨ ... ∨ φ[N]
```

LTL: `Always(φ)` → "φ is true at all time steps"

Z3 encoding:
```
φ[0] ∧ φ[1] ∧ φ[2] ∧ ... ∧ φ[N]
```

LTL: `φ Until ψ` → "φ holds until ψ becomes true"

Z3 encoding:
```
ψ[0] ∨ (φ[0] ∧ ψ[1]) ∨ (φ[0] ∧ φ[1] ∧ ψ[2]) ∨ ...
```

**Benefits**:
- No separate LTL tool needed
- Z3 does both temporal and numeric solving
- Simpler tool chain

**Trade-offs**:
- Bounded only (can't handle unbounded/infinite horizon)
- No minimality guarantees (doesn't find shortest trace)
- Scalability depends on horizon length

---

## Bounded Model Checking Approach

### What is Bounded Model Checking?

**Traditional LTL model checking**: Verify if infinite-state system satisfies LTL property

**Bounded model checking**: Check if property holds for traces up to length N

**Our use case**: We don't just verify - we **synthesize** scenarios satisfying properties

### Why Bounded is Sufficient

**Driving scenarios have natural time bounds**:
- Test scenarios typically 5-30 seconds
- Discrete time steps (0.5s) give finite horizon
- Events happen within bounded time (cut-in within 10 seconds)

**No need for infinite traces**:
- Not looking for "system eventually fails" (unbounded)
- Looking for "concrete scenario within time budget" (bounded)

### How We Encode LTL into Z3

#### State Variables
For each time step `t ∈ [0, N]`, create Z3 variables for:
- Positions: `px[t]`, `py[t]`
- Velocities: `vx[t]`, `vy[t]`
- Discrete state: `lane[t]`

#### Atomic Propositions
Map to Z3 Boolean expressions:

`InLane(actor, lane)` at time `t` →
```rust
lane_var[t] == lane_value
```

`Ahead(actor1, actor2)` at time `t` →
```rust
px1[t] > px2[t]
```

`TTC(actor1, actor2) > ttc_min` at time `t` →
```rust
if vx1[t] > vx2[t]:
    (px2[t] - px1[t]) / (vx1[t] - vx2[t]) > ttc_min
else:
    true
```

#### Temporal Operators
Recursively expand over time:

```rust
fn encode_ltl_bounded(formula, time, horizon) -> Z3Bool {
    match formula {
        Atom(p) => encode_proposition(p, time),

        Eventually(φ) => {
            // φ[time] ∨ φ[time+1] ∨ ... ∨ φ[horizon]
            OR(
                encode_ltl_bounded(φ, time, horizon),
                encode_ltl_bounded(φ, time+1, horizon),
                ...
            )
        }

        Always(φ) => {
            // φ[time] ∧ φ[time+1] ∧ ... ∧ φ[horizon]
            AND(
                encode_ltl_bounded(φ, time, horizon),
                encode_ltl_bounded(φ, time+1, horizon),
                ...
            )
        }

        Until(φ, ψ) => {
            // ψ[time] ∨ (φ[time] ∧ Until(φ,ψ)[time+1])
            OR(
                encode_ltl_bounded(ψ, time, horizon),
                AND(
                    encode_ltl_bounded(φ, time, horizon),
                    encode_ltl_bounded(Until(φ, ψ), time+1, horizon)
                )
            )
        }
    }
}
```

### Complete Constraint Set

Z3 solves:
```
Satisfiable assignment for variables {px[0..N], py[0..N], vx[0..N], vy[0..N], lane[0..N]}

Such that:
    Initial_conditions ∧
    Physics_constraints ∧
    LTL_formula_encoded ∧
    Safety_constraints
```

---

## Key Trade-offs

### 1. Bounded vs Unbounded LTL

**Decision**: Bounded

**Trade-off**:
- ✅ Simpler encoding, finite constraints, known complexity
- ❌ Can't express "eventually without time bound"
- **Mitigation**: Driving scenarios have natural time limits

### 2. Constant Velocity vs Realistic Dynamics

**Decision**: Constant velocity for MVP

**Trade-off**:
- ✅ Simple, easy to encode, good starting point
- ❌ Not realistic (real cars accelerate)
- **Future**: Add acceleration profiles in post-MVP

### 3. Single Tool (Z3) vs Separate LTL Solver + SMT

**Decision**: Single tool (Z3 for both)

**Trade-off**:
- ✅ Simpler architecture, one solver, integrated solving
- ❌ Less specialized than dedicated LTL tools
- **Rationale**: For bounded case, Z3 is sufficient

### 4. YAML vs Python DSL

**Decision**: YAML

**Trade-off**:
- ✅ Language-agnostic, simple, readable
- ❌ Less flexible than embedded Python DSL
- **Future**: Could add Python API later

### 5. JSON vs OpenSCENARIO Output

**Decision**: JSON for MVP

**Trade-off**:
- ✅ Easier to validate, simpler to generate
- ❌ Not directly usable in CARLA
- **Future**: Add OpenSCENARIO XML export in Phase 13

### 6. Manual Validation vs Automated

**Decision**: Manual (visual inspection) for MVP

**Trade-off**:
- ✅ Faster to implement, good for prototyping
- ❌ Not scalable
- **Future**: Add automated validation, visualization

---

## Multiple Scenario Generation

### Problem
User wants to generate N diverse scenarios from one specification.

### Approach: Blocking Clauses

After finding a solution, add a constraint that **blocks that exact solution**, forcing Z3 to find a different one.

**Algorithm**:
```
scenarios = []
for i in 0..N:
    solver = new_solver()
    add_all_constraints(solver)

    for prev in scenarios:
        solver.add(NOT(same_as(prev)))  # blocking clause

    if solver.check() == SAT:
        scenarios.push(extract_scenario(solver.model()))
    else:
        break  # no more solutions
```

**Blocking Clause Example**:
```rust
// Block if NPC initial position and speed are too similar
!(npc_px[0] ≈ prev_npc_px[0] AND npc_vx[0] ≈ prev_npc_vx[0])
```

**Diversity Metric**: Block key parameters (initial position, speed, cut-in time)

---

## Why This Design is Good for AI Agents

### Self-Contained Phases
Each phase file has:
- Prerequisites (what must be done first)
- Context (why this phase exists)
- Goals (clear objectives)
- Implementation steps (detailed how-to)
- Success criteria (validation)

### Clear Data Flow
- Unidirectional: DSL → LTL → Z3 → Scenario
- No circular dependencies
- Each layer has well-defined inputs/outputs

### Incremental Validation
- Test each component independently
- Validate before moving to next phase
- Example files provide ground truth

### Extensibility
- Adding new scenario types: Add to `LTLGenerator`
- Adding new constraints: Add to `Z3Encoder`
- Adding new output formats: Add to `scenario/` module

---

## Future Enhancements

### Short Term (Post-MVP)
1. More scenario types (cut-in right, overtake, merge)
2. OpenSCENARIO XML export
3. Visualization (plot trajectories)

### Medium Term
4. Advanced physics (acceleration, realistic lane changes)
5. Optimization (find most challenging scenarios)
6. Probabilistic scenarios (pSTL)

### Long Term
7. Real-world data extraction (driving logs → DSL)
8. Multi-agent coordination scenarios (ATL)
9. Formal verification (prove safety properties)
10. Web UI for DSL editing and visualization

---

## Lessons Learned from Discussion

### 1. Start Simple, Build Up
- Don't over-engineer for MVP
- Constant velocity before acceleration
- JSON before OpenSCENARIO
- One scenario type before many

### 2. Hybrid Approaches Work
- LTL for structure, Z3 for values
- Best of both worlds
- More practical than pure approaches

### 3. User Input is Critical
- Simplified DSL based on feedback
- Full architecture but minimal complexity
- Focus on what matters (not over-abstraction)

### 4. Bounded is Often Sufficient
- Driving scenarios have natural bounds
- Bounded model checking is pragmatic
- Don't need theoretical unbounded LTL

### 5. Tool Integration > Purity
- Using Z3 for everything (not separate LTL tool)
- Pragmatic over theoretically pure
- Simpler architecture, easier to maintain

---

## Summary

**Problem**: Generate test scenarios for autonomous vehicles

**Solution**: LTL + Z3 with bounded model checking

**Why**:
- LTL for temporal structure
- Z3 for numeric constraints
- Bounded encoding integrates both
- Simpler than alternatives
- Sufficient for use case

**Key Innovation**: Encoding LTL directly into Z3 (no separate LTL solver)

**Implementation**: Rust tool with clean layer separation

**Output**: JSON scenarios (defer OpenSCENARIO to later)

**Validation**: Manual visual inspection for MVP

**Extensibility**: Designed for future enhancements

---

## References

- **Z3**: https://github.com/Z3Prover/z3
- **LTL**: https://en.wikipedia.org/wiki/Linear_temporal_logic
- **Bounded Model Checking**: Clarke et al., "Bounded Model Checking Using Satisfiability Solving"
- **OpenSCENARIO**: https://www.asam.net/standards/detail/openscenario/
- **CARLA**: https://carla.org/
