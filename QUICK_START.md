# CARLA Scenario Generator - Quick Start Guide

## 🎯 For AI Agents Implementing This

### Start Here
1. Read `Implementation_plan.md` (15 min) - Get the big picture
2. Read `design_decisions.md` (30 min) - Understand the "why"
3. Execute phases sequentially starting with `plans/phase_01_setup.md`

### Implementation Checklist

```
□ Phase 1: Project Setup (1-2 days)
  └─ Create Rust project, add dependencies, verify Z3 works

□ Phase 2: DSL Layer (1 day)
  └─ Define YAML schema, parse into Rust structs

□ Phase 3: LTL Layer (1-2 days)
  └─ Define LTL AST, generate formulas from DSL

□ Phase 4: Scenario Model (0.5 day)
  └─ Define JSON output schema

□ Phase 5: Z3 Foundation (1 day)
  └─ Create Z3 encoder, variables, initial conditions

□ Phase 6: Z3 Physics (1 day)
  └─ Add kinematic constraints

□ Phase 7: Z3 LTL Encoding (2 days) ⚠️ CRITICAL
  └─ Bounded model checking: LTL → Z3 constraints

□ Phase 8: Z3 Safety (1 day)
  └─ TTC and distance constraints

□ Phase 9: Extraction (1 day)
  └─ Z3 model → JSON scenario

□ Phase 10: Single Scenario (1 day)
  └─ End-to-end pipeline, validate manually

□ Phase 11: Multiple Scenarios (1 day)
  └─ Blocking clauses for diversity

□ Phase 12: CLI & Polish (1-2 days)
  └─ Command-line interface, logging, docs
```

### After Each Phase

```bash
cargo test        # All tests must pass
cargo clippy      # No warnings
cargo fmt         # Format code
```

### Final Validation (Phase 10+)

```bash
# Generate scenario
cargo run -- -i examples/cut_in_left.yaml -o output.json

# Check it's valid
cat output.json | python -m json.tool

# Manually verify:
# - NPC starts in lane 0, ego in lane 1
# - NPC ahead initially
# - NPC eventually in lane 1
# - TTC > 3.0s maintained
# - Distance > 5.0m maintained
```

---

## 📚 For Human Developers

### Day 1: Understanding
- Read `Implementation_plan.md`
- Read `design_decisions.md`
- Review `plans/README.md`

### Day 2-3: Foundation
- Phase 1: Setup
- Phase 2: DSL
- Phase 3: LTL
- Phase 4: Scenario Model

### Week 2: Core (Hardest Part)
- Phase 5: Z3 Foundation
- Phase 6: Z3 Physics
- **Phase 7: Z3 LTL Encoding** ← Most complex
- Phase 8: Z3 Safety
- Phase 9: Extraction

### Week 3: Integration
- Phase 10: Single Scenario Pipeline
- Phase 11: Multiple Scenarios
- Phase 12: CLI & Polish

---

## 🔑 Key Files Reference

| Need to... | Read this... |
|------------|--------------|
| Understand architecture | `Implementation_plan.md` |
| Understand design choices | `design_decisions.md` |
| Navigate documentation | `DELIVERABLES.md` |
| Implement Phase 1 | `plans/phase_01_setup.md` |
| Implement Phase 2 | `plans/phase_02_dsl.md` |
| ... | `plans/phase_XX_*.md` |
| Implement Phases 7-12 | `plans/phases_7_to_12_summary.md` |

---

## ⚠️ Common Pitfalls

### 1. Skipping Phases
❌ Don't skip ahead
✅ Each phase builds on previous work

### 2. Not Testing
❌ Implement multiple phases then test
✅ Test after EACH phase

### 3. Ignoring Validation
❌ Assume it works
✅ Manually inspect JSON outputs

### 4. Missing Context
❌ Jump straight to code
✅ Read phase context and goals first

---

## 💡 Pro Tips

### For AI Agents
- Each phase file has COMPLETE context - don't need to reference others during implementation
- Success criteria are explicit - verify before moving on
- Common issues sections help debug
- Code examples are ready to use

### For Humans
- Phase files = sprint planning documents
- Can parallelize some phases (e.g., Phase 2 + Phase 4)
- Tests are your safety net
- Manual validation in Phase 10 is crucial

---

## 🎓 Core Concepts

### LTL (Linear Temporal Logic)
- `F φ` (Eventually): φ will be true at some point
- `G φ` (Always): φ is always true
- `φ U ψ` (Until): φ holds until ψ becomes true

### Z3 SMT Solver
- Finds values satisfying constraints
- Handles real numbers, integers, booleans
- Mix arithmetic + logic

### Bounded Model Checking
- Fix time horizon (e.g., 20 steps)
- Expand temporal operators over horizon
- `Eventually(φ)` → `φ[0] ∨ φ[1] ∨ ... ∨ φ[20]`

---

## 🚀 Getting Started NOW

```bash
# 1. Navigate to project directory
cd /home/ashfaqf/playground/synergies/test-4

# 2. Read master plan
cat Implementation_plan.md

# 3. Read design rationale
cat design_decisions.md

# 4. Start Phase 1
cat plans/phase_01_setup.md

# 5. Begin implementation!
# Follow the phase file step-by-step
```

---

## 📞 Help & Support

Stuck? Check this order:

1. **Phase file "Success Criteria"** - Did you meet them all?
2. **Phase file "Common Issues"** - Is your issue listed?
3. **design_decisions.md** - Understand the "why"
4. **Ask the user** - Clarify requirements

---

## ✅ MVP Success = All These True

- [ ] Project compiles: `cargo build`
- [ ] Tests pass: `cargo test`
- [ ] CLI works: `cargo run -- -i examples/cut_in_left.yaml -o output.json`
- [ ] JSON valid: `cat output.json | python -m json.tool`
- [ ] Scenario physically plausible (manual check)
- [ ] Safety satisfied: TTC > 3.0s, distance > 5.0m
- [ ] Multiple scenarios work and are diverse

---

*Quick Start Guide - CARLA Scenario Generator*
*Project Status: Documentation Complete, Ready for Implementation*
*Next: Begin Phase 1*
