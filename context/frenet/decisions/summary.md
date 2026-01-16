# Summary of Critical Decisions

## Overview
This document summarizes all key decisions for integrating Frenet coordinates into test-4.

## Decision Summary Table

| Decision | Recommendation | Implementation Status |
|----------|----------------|----------------------|
| Coordinate system mode | Hybrid with default Frenet | **COMPLETED** - Trait-based encoder architecture |
| Z3 solving strategy | Coordinate-specific solving (not hybrid) | **COMPLETED** - Separate encoders for each coordinate system |
| Polynomial coefficients | Not used (direct Z3 solving) | **CHANGED** - Z3 solves directly for variables |
| Lane change methods | Lateral velocity constraints | **COMPLETED** - Z3 bounds on lateral velocity |
| Reference lines | Single reference line per scenario | **COMPLETED** - Simple reference line implementation |
| Backward compatibility | Full backward compatibility | **COMPLETED** - Cartesian scenarios work unchanged |

## Detailed References

### Coordinate System Mode (**COMPLETED**)
→ **decisions/coordinate-mode.md**
- **Implemented:** Trait-based encoder architecture with `CoordinateEncoder` trait
- `CoordinateSystem` enum (Frenet/Cartesian) in DSL types
- `GenericEncoder` facade dispatches to `FrenetEncoder` or `CartesianEncoder`
- Default to Frenet for road scenarios
- Cartesian maintained for backward compatibility

### Z3 Constraint Solving (**COMPLETED**)
→ **decisions/z3-strategy.md**
- **Implemented:** Coordinate-specific solving (not hybrid within single solve)
- Frenet scenarios: Z3 variables are `frenet_s`, `frenet_t`, `frenet_vs`, `frenet_vt`
- Cartesian scenarios: Z3 variables are `positions_x`, `positions_y`, `velocities_x`, `velocities_y`
- No quintic pre-generation - Z3 directly solves for all variables
- Smoothness via lateral velocity constraints instead of polynomial generation

### Polynomial Coefficients (**CHANGED**)
→ **decisions/polynomial-coefficients.md**
- **Decision changed:** No pre-solved quintic coefficients
- Z3 solves directly for Frenet variables with smoothness constraints
- Lateral velocity bounds (`vt_min <= vt[t] <= vt_max`) enforce smooth lane changes
- Simpler architecture without pre-generation + refinement pipeline

### Lane Change Methods (**COMPLETED**)
→ **decisions/lane-change-methods.md**
- **Implemented:** Z3 constraints on lateral velocity
- Integer lane assignment variables constrain vehicles to discrete lanes
- Lateral velocity bounds prevent sudden lane changes
- No quintic polynomial trajectory generation

### Reference Line Management (**SIMPLIFIED**)
→ **decisions/reference-line-management.md**
- **Implemented:** Single straight reference line per scenario
- Lane centers computed as: `t = lane * lane_width + lane_width/2`
- No complex blending or switching logic
- Coordinate conversion handled during extraction, not during solving

### Backward Compatibility (**COMPLETED**)
→ **decisions/backward-compatibility.md**
- **Implemented:** Full backward compatibility via Cartesian encoder
- Existing Cartesian scenarios work without modification
- No auto-conversion or deprecation warnings needed
- Both coordinate systems coexist as first-class options

## Implementation Phases

### Phase 1: Core Frenet (Week 1-2)
1. Implement Frenet coordinate structures
2. Implement coordinate conversion functions
3. Implement quintic polynomial solver
4. Add unit tests

### Phase 2: Trajectory Generation (Week 2-3)
1. Build trajectory generation pipeline
2. Implement smoothness validation
3. Add integration tests for simple scenarios

### Phase 3: Z3 Integration (Week 3-4)
1. Design hybrid approach
2. Implement constraint adapters
3. Add Z3 refinement layer
4. Test collision avoidance

### Phase 4: System Integration (Week 4-5)
1. Integrate with existing test-4 code
2. Implement backward compatibility
3. Add E2E tests
4. Performance benchmarking

### Phase 5: Production Ready (Week 5-6)
1. Complete test coverage (>90%)
2. Optimize performance
3. Update documentation
4. Code review and validation

## Decision Rationale

### Why Trait-Based Architecture Instead of Hybrid?
- **Simpler:** Clean separation between coordinate systems (~700 lines each vs ~2360 monolithic)
- **Type-safe:** Enum-based dispatch at construction time
- **Extensible:** Add new coordinate systems by implementing trait
- **No runtime overhead:** Trait object indirection minimal compared to Z3 solving time
- **Easy to test:** Each encoder can be tested independently

### Why Coordinate-Specific Solving Instead of Hybrid Z3?
- **Simpler architecture:** No pre-generation + refinement pipeline
- **Direct solving:** All constraints encoded directly in Z3
- **No coordinate conversion during solving:** Only convert during extraction/output
- **Consistent variables:** All variables for a scenario use same coordinate system

### Why Direct Z3 Solving Instead of Quintic Pre-Generation?
- **Unified solving:** Single Z3 solve handles all constraints
- **More flexible:** Z3 can find solutions pre-generation might miss
- **Simpler code:** No need for trajectory generation + Z3 refinement
- **Good enough smoothness:** Lateral velocity bounds provide adequate smoothness

### Why Lateral Velocity Constraints Instead of Quintic Polynomials?
- **YAGNI:** Don't build trajectory generation if not needed
- **Adequate smoothness:** Velocity bounds prevent sudden lane changes
- **Simpler:** Direct Z3 constraints vs polynomial generation
- **Proven approach:** Similar to existing Cartesian implementation

## Risk Mitigation

### Risk: Z3 Breaks Smoothness
**Mitigation**: Tight waypoint tolerance (0.5m), validate C² continuity

### Risk: Backward Compatibility Issues
**Mitigation**: Auto-convert legacy scenarios, warn users, keep Cartesian mode

### Risk: Performance Degradation
**Mitigation**: Benchmark early, optimize hot paths, cache reference lines

### Risk: Collision Detection Inaccuracy
**Mitigation**: Convert to Cartesian for collision checks, use spatial indexing

## Context Dependencies
- All decisions depend on: **domain/core-concepts.md**
- Z3 decisions depend on: **processes/trajectory-generation.md**
- Lane change decisions depend on: **standards/smoothness-criteria.md**
- Integration decisions depend on: **processes/z3-integration.md**

## Next Steps

1. **Review** all decision documents
2. **Prototype** hybrid coordinate system
3. **Implement** quintic trajectory generator
4. **Test** with simple lane change scenarios
5. **Evaluate** Z3 integration approaches
6. **Decide** on final architecture based on testing results

## Questions to Resolve

1. Should we support multiple lane change methods from the start? **No, Phase 1: quintic only**
2. How tight should waypoint tolerance be for Z3? **0.5m recommended, tunable**
3. Should we deprecate Cartesian mode entirely? **No, keep for unstructured scenarios**
4. Should Z3 solve in Frenet or Cartesian space? **Cartesian for collision, waypoints as constraints**
5. When to add spline support? **Phase 2, if quintic proves insufficient**
