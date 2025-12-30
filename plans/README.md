# Implementation Phase Files

This directory contains 12 self-contained implementation phase files for the CARLA Scenario Generator project.

## Phase Overview

Each phase file is designed to be executed by an AI agent independently, with complete context and instructions.

| Phase | File | Focus | Est. Time |
|-------|------|-------|-----------|
| 1 | phase_01_setup.md | Project setup, dependencies | 1-2 days |
| 2 | phase_02_dsl.md | DSL types and YAML parsing | 1 day |
| 3 | phase_03_ltl.md | LTL formula AST and generation | 1-2 days |
| 4 | phase_04_scenario_model.md | Output JSON data structures | 0.5 day |
| 5 | phase_05_z3_foundation.md | Z3 setup and variables | 1 day |
| 6 | phase_06_z3_physics.md | Kinematic constraints | 1 day |
| 7 | phase_07_z3_ltl.md | Bounded LTL encoding | 2 days |
| 8 | phase_08_z3_safety.md | Safety constraints | 1 day |
| 9 | phase_09_extraction.md | Extract scenarios from Z3 | 1 day |
| 10 | phase_10_single_scenario.md | End-to-end pipeline | 1 day |
| 11 | phase_11_multiple_scenarios.md | Multi-scenario generation | 1 day |
| 12 | phase_12_cli.md | CLI, logging, polish | 1-2 days |

## How to Use These Files

### For AI Agents

1. Start with phase 1 and proceed sequentially
2. Each file is self-contained with:
   - Prerequisites (what must be done first)
   - Context (why this phase exists)
   - Goals (what to achieve)
   - Detailed implementation steps
   - Success criteria (how to validate)
   - Testing instructions
   - Next phase pointer

3. Do not skip phases - each builds on previous work
4. Run tests after each phase to verify correctness
5. If a test fails, debug before proceeding

### For Human Developers

- Use as a detailed implementation guide
- Each phase can be a sprint or work session
- Tests provide validation checkpoints
- Can be adapted based on team needs

## Phase Dependencies

```
Phase 1 (Setup)
    ↓
Phase 2 (DSL)
    ↓
Phase 3 (LTL)
    ↓
Phase 4 (Scenario Model)
    ↓
Phase 5 (Z3 Foundation)
    ↓
Phase 6 (Z3 Physics) ← Depends on Phase 5
    ↓
Phase 7 (Z3 LTL) ← Depends on Phase 3, 5
    ↓
Phase 8 (Z3 Safety) ← Depends on Phase 5
    ↓
Phase 9 (Extraction) ← Depends on Phase 4, 5
    ↓
Phase 10 (Single Scenario) ← Integrates Phases 2-9
    ↓
Phase 11 (Multiple Scenarios) ← Extends Phase 10
    ↓
Phase 12 (CLI) ← Polish Phase 10-11
```

## Completion Checklist

Track your progress:

- [ ] Phase 1: Project compiles, Z3 works
- [ ] Phase 2: YAML parsing works
- [ ] Phase 3: LTL formulas generate
- [ ] Phase 4: JSON schema defined
- [ ] Phase 5: Z3 variables created
- [ ] Phase 6: Physics constraints work
- [ ] Phase 7: LTL encoding works
- [ ] Phase 8: Safety constraints work
- [ ] Phase 9: Scenario extraction works
- [ ] Phase 10: End-to-end pipeline works
- [ ] Phase 11: Multiple scenarios generated
- [ ] Phase 12: CLI complete

## Testing Between Phases

After each phase:
```bash
cargo test        # All tests
cargo clippy      # Code quality
cargo fmt         # Formatting
```

## Getting Help

- Read `../design_decisions.md` for design rationale
- Check `../Implementation_plan.md` for architecture overview
- Review example files in `../examples/`
- Ask user for clarification if instructions are unclear
