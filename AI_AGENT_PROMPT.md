# AI Agent Implementation Prompt

## Project Overview

You are tasked with implementing the **CARLA Scenario Generator**, a tool that generates driving test scenarios from high-level specifications using Linear Temporal Logic (LTL) and Z3 SMT solver.

**Current Status**: Complete documentation exists. Your job is to implement the system by following the phase-by-phase implementation plan.

**Location**: `/home/ashfaqf/playground/synergies/test-4/`

---

## Your Mission

Implement a Rust-based tool that:
1. Accepts YAML scenario specifications (user input)
2. Generates LTL formulas from specifications
3. Encodes LTL + physics + safety constraints into Z3 SMT solver
4. Solves for concrete scenario parameters
5. Outputs JSON with complete driving trajectories

**Technology**: Rust, Z3 SMT solver, LTL (Linear Temporal Logic), Bounded Model Checking

---

## Documentation Available

You have complete implementation documentation:

### Start Here (Read First)
1. **Implementation_plan.md** - Master plan, architecture, all phases
2. **design_decisions.md** - Design rationale, why we chose LTL + Z3
3. **QUICK_START.md** - Quick reference guide

### Implementation Guides
4. **plans/README.md** - How to use phase files
5. **plans/phase_01_setup.md** through **phase_06_*.md** - Detailed implementation guides
6. **plans/phases_7_to_12_summary.md** - Comprehensive guide for final phases

---

## Implementation Instructions

### Step 1: Read Documentation (REQUIRED)

Before writing any code:

```bash
# 1. Read master plan (understand architecture)
cat Implementation_plan.md

# 2. Read design decisions (understand rationale)
cat design_decisions.md

# 3. Read quick start
cat QUICK_START.md

# 4. Read phase files guide
cat plans/README.md
```

**Time**: Allocate 30-45 minutes to read and understand before coding.

### Step 2: Initialize Git Repository

```bash
# Initialize git
git init

# Create .gitignore
cat > .gitignore << 'EOF'
/target/
Cargo.lock
*.swp
*.swo
*~
.DS_Store
EOF

# Initial commit
git add .
git commit -m "Add project documentation and implementation plan

- Implementation plan with 12 phases
- Design decisions documenting LTL + Z3 approach
- Phase-by-phase implementation guides
- Example YAML and JSON schemas"
```

### Step 3: Execute Phases Sequentially

**CRITICAL**: Do phases in order. Each depends on previous work.

#### Phase 1: Project Setup

```bash
# Read the phase file
cat plans/phase_01_setup.md

# Follow ALL steps in the file
# The file contains:
# - Complete Cargo.toml with dependencies
# - Module structure to create
# - Error types to implement
# - Z3 integration test

# After implementing Phase 1, test:
cargo build
cargo test

# If tests pass, commit:
git add .
git commit -m "Implement Phase 1: Project setup

- Add Cargo.toml with all dependencies (z3, serde, etc)
- Create module structure (dsl, ltl, solver, scenario)
- Add error types with thiserror
- Verify Z3 integration with basic test
- All tests passing"
```

#### Phase 2: DSL Layer

```bash
# Read the phase file
cat plans/phase_02_dsl.md

# Implement all steps
# Create src/dsl/types.rs and src/dsl/parser.rs
# Follow the code in the phase file

# Test:
cargo test dsl

# Commit:
git add .
git commit -m "Implement Phase 2: DSL layer

- Define ScenarioSpec, ActorSpec, NpcSpec types
- Implement ValueOrRange for fixed/range values
- Add YAML parser with serde
- Implement validation logic
- Create example YAML file
- All DSL tests passing"
```

#### Phase 3: LTL Layer

```bash
cat plans/phase_03_ltl.md

# Implement LTL formula AST and generator

cargo test ltl

git add .
git commit -m "Implement Phase 3: LTL layer

- Define LTLFormula AST (Eventually, Always, Until, etc)
- Define Proposition types (InLane, Ahead, TTCGT, DistanceGT)
- Implement builder methods for ergonomic formula construction
- Add LTLGenerator for cut-in scenario
- Implement Display trait for debugging
- All LTL tests passing"
```

#### Phase 4: Scenario Model

```bash
cat plans/phase_04_scenario_model.md

# Implement JSON output schema

cargo test scenario

git add .
git commit -m "Implement Phase 4: Scenario model

- Define Scenario, ActorTrajectory, State structures
- Add Position and Velocity types with helper methods
- Implement ValidationInfo for constraint tracking
- Add JSON serialization with serde
- Create expected output example
- All scenario model tests passing"
```

#### Phase 5: Z3 Foundation

```bash
cat plans/phase_05_z3_foundation.md

# Implement Z3 encoder foundation

cargo test solver::encoder

git add .
git commit -m "Implement Phase 5: Z3 foundation

- Create Z3Encoder struct with lifetime management
- Implement variable creation for all time steps
- Encode initial conditions from DSL
- Add lane-position coupling constraints
- Verify Z3 returns SAT for initial conditions
- All encoder foundation tests passing"
```

#### Phase 6: Z3 Physics

```bash
cat plans/phase_06_z3_physics.md

# Add kinematic constraints

cargo test solver::encoder::test_kinematics

git add .
git commit -m "Implement Phase 6: Z3 physics constraints

- Add kinematic equations for position updates
- Encode constant velocity for ego and npc
- Implement lane-position coupling for all time steps
- Verify physics constraints with tests
- All physics tests passing"
```

#### Phases 7-12: Follow Summary Guide

```bash
cat plans/phases_7_to_12_summary.md

# Phase 7: Z3 LTL Encoding (CRITICAL - most complex)
# Implement bounded model checking
git add .
git commit -m "Implement Phase 7: Z3 LTL encoding

- Add bounded LTL encoding algorithm
- Implement Eventually, Always, Until expansion
- Encode propositions (InLane, Ahead, TTCGT, DistanceGT)
- Integrate LTL formulas into Z3 constraints
- All LTL encoding tests passing"

# Phase 8: Z3 Safety
git add .
git commit -m "Implement Phase 8: Z3 safety constraints

- Add TTC calculation constraints
- Implement minimum distance constraints
- Apply safety checks for all time steps
- All safety constraint tests passing"

# Phase 9: Extraction
git add .
git commit -m "Implement Phase 9: Scenario extraction

- Extract Z3 model to Scenario JSON
- Convert Z3 rationals to float values
- Build complete actor trajectories
- Compute validation metrics
- All extraction tests passing"

# Phase 10: Single Scenario Pipeline
git add .
git commit -m "Implement Phase 10: Single scenario pipeline

- Integrate all components (DSL -> LTL -> Z3 -> JSON)
- Add main generation function in lib.rs
- Test end-to-end with example YAML
- Verify generated JSON is valid
- Manual validation: scenario is physically plausible"

# Phase 11: Multiple Scenarios
git add .
git commit -m "Implement Phase 11: Multiple scenario generation

- Implement blocking clause generation
- Add multi-scenario loop with diversity
- Test generating 5+ scenarios from same spec
- Verify scenarios are different
- All multi-scenario tests passing"

# Phase 12: CLI & Polish
git add .
git commit -m "Implement Phase 12: CLI and polish

- Add clap-based command-line interface
- Implement logging with tracing
- Add error handling and user-friendly messages
- Write README with usage examples
- Add integration tests
- Project complete and ready for use"
```

---

## Git Workflow Requirements

### Commit Frequency

**REQUIRED**: Commit after EACH phase completion.

**Optional but recommended**: Commit during phases if you complete a significant sub-component.

### Commit Message Format

**REQUIRED FORMAT**:

```
<Short summary line (50 chars max)>
<blank line>
<Detailed description with bullet points>
```

**RULES**:
- ❌ NO emojis in commit messages
- ❌ NO "Generated with Claude" or similar
- ❌ NO "Co-Authored-By: Claude" tags
- ✅ DO use clear, descriptive messages
- ✅ DO include what was implemented
- ✅ DO mention test status

**Good Examples**:

```
Implement Phase 2: DSL layer

- Define ScenarioSpec, ActorSpec, NpcSpec types
- Implement ValueOrRange for fixed/range values
- Add YAML parser with serde
- Implement validation logic
- Create example YAML file
- All DSL tests passing
```

```
Add Z3 LTL encoding with bounded model checking

- Implement encode_ltl_bounded method
- Expand Eventually, Always, Until operators over horizon
- Encode atomic propositions to Z3 constraints
- Add comprehensive tests for temporal operators
- All tests passing
```

**Bad Examples** (DON'T DO THIS):

```
✨ Implement DSL layer 🚀  ❌ (has emojis)

Generated with Claude Code ❌ (mentions Claude)
```

### Checking Your Work

After each phase:

```bash
# Tests must pass
cargo test

# No warnings
cargo clippy

# Code formatted
cargo fmt

# Check git status
git status

# View commits
git log --oneline
```

---

## Success Criteria

### After Each Phase

Before moving to the next phase, verify:

- [ ] All steps in phase file completed
- [ ] `cargo test` passes (no failures)
- [ ] `cargo clippy` has no warnings
- [ ] Code is formatted with `cargo fmt`
- [ ] Git commit created with proper message
- [ ] Success criteria in phase file met

### After Phase 10 (End-to-End)

Manual validation required:

```bash
# Generate a scenario
cargo run -- -i examples/cut_in_left.yaml -o test_output.json

# Check it's valid JSON
cat test_output.json | python -m json.tool

# Manually verify the scenario:
# 1. NPC starts in lane 0, ego in lane 1
# 2. NPC is ahead of ego initially
# 3. NPC eventually changes to lane 1
# 4. Lane change happens within specified time range
# 5. TTC never drops below 3.0 seconds
# 6. Distance never drops below 5.0 meters (when same lane)
# 7. Velocities are constant (except lateral during lane change)
# 8. Positions follow physics: px[t+1] = px[t] + vx[t] * dt
```

If any manual check fails, debug before proceeding.

### After Phase 12 (MVP Complete)

Final validation:

```bash
# Build release
cargo build --release

# Generate single scenario
cargo run --release -- -i examples/cut_in_left.yaml -o output.json

# Generate multiple scenarios
cargo run --release -- -i examples/cut_in_left.yaml -o scenarios/ -v

# Verify:
# - CLI works
# - JSON outputs are valid
# - Scenarios are different (when multiple)
# - All manual validation checks pass
```

---

## Troubleshooting Guide

### If Phase Tests Fail

1. **Read the phase file carefully** - did you implement all steps?
2. **Check "Common Issues" section** in the phase file
3. **Review code against examples** in the phase file
4. **Check error messages** - what is failing?
5. **Read design_decisions.md** - understand the "why"
6. **Ask for help** - describe the issue, what you tried

### If Z3 Returns UNSAT

Means constraints are contradictory:

1. **Check initial conditions** - are ranges valid?
2. **Review constraints** - do they conflict?
3. **Simplify** - remove constraints one by one to find issue
4. **Print Z3 constraints** - debug what was added

### If Compilation Fails

1. **Check Cargo.toml** - all dependencies present?
2. **Check lifetimes** - Z3 requires careful lifetime management
3. **Read error messages** - Rust errors are usually helpful
4. **Consult phase file** - code examples should compile

---

## Important Notes

### About Phase Files

- **Phase files are self-contained** - all context needed is in the file
- **Code examples are complete** - you can copy/adapt them
- **Success criteria are explicit** - verify before moving on
- **Don't skip phases** - each builds on previous work

### About Testing

- **Test after every phase** - don't accumulate untested code
- **Unit tests validate components** - run `cargo test <module>`
- **Integration tests validate pipeline** - run `cargo test --test integration_test`
- **Manual validation is critical** - especially in Phase 10

### About Documentation

- **Read before coding** - understanding saves time
- **Reference during coding** - phase files have all details
- **design_decisions.md explains "why"** - consult if confused
- **Examples are your guide** - YAML and JSON schemas provided

---

## Expected Timeline

**Fast path**: ~10-12 days (if focused, experienced with Rust/Z3)
**Normal path**: ~15-20 days (learning Rust/Z3 as you go)
**Careful path**: ~20-25 days (thorough testing, documentation)

**Phase breakdown**:
- Phases 1-4 (Foundation): 3-4 days
- Phases 5-8 (Z3 encoding): 5-6 days
- Phases 9-12 (Integration): 3-4 days

---

## Final Deliverables

When you're done, the project should have:

```
carla-scenario-generator/
├── Cargo.toml
├── README.md (create in Phase 12)
├── src/
│   ├── main.rs (CLI)
│   ├── lib.rs (public API)
│   ├── error.rs
│   ├── dsl/ (types.rs, parser.rs)
│   ├── ltl/ (formula.rs, generator.rs)
│   ├── solver/ (encoder.rs, physics.rs, multi_solve.rs)
│   └── scenario/ (model.rs, extractor.rs)
├── examples/
│   ├── cut_in_left.yaml
│   └── expected_output.json
├── tests/
│   └── integration_test.rs
└── .git/ (with commits for each phase)
```

**Working CLI**:
```bash
cargo run -- -i examples/cut_in_left.yaml -o output.json
```

**Valid output**: JSON file with complete driving scenario trajectories

---

## Starting Now

```bash
# 1. Navigate to project directory
cd /home/ashfaqf/playground/synergies/test-4

# 2. Initialize git
git init
# Create .gitignore and initial commit (see above)

# 3. Read documentation
cat Implementation_plan.md
cat design_decisions.md
cat QUICK_START.md

# 4. Start Phase 1
cat plans/phase_01_setup.md
# Follow the phase file step by step

# 5. Test and commit
cargo test
git add .
git commit -m "Implement Phase 1: Project setup

- Add Cargo.toml with dependencies
- Create module structure
- Verify Z3 integration
- All tests passing"

# 6. Continue to Phase 2
cat plans/phase_02_dsl.md
# Repeat: implement, test, commit

# Keep going until Phase 12!
```

---

## Questions?

If you encounter issues:
1. Check phase file "Common Issues" section
2. Review design_decisions.md for context
3. Verify prerequisites from earlier phases
4. Ask user for clarification (describe what you tried)

---

## Summary

**Your task**: Implement CARLA Scenario Generator by following 12 phases

**Your guide**: Detailed phase files in `plans/` directory

**Your workflow**: Read → Implement → Test → Commit → Next phase

**Your output**: Working Rust tool that generates driving scenarios using LTL + Z3

**Start**: `cat plans/phase_01_setup.md` and begin!

Good luck! The documentation is comprehensive - you have everything you need to succeed.
