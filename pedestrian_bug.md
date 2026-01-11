# Pedestrian Multi-Solve Performance Issue

**Date**: 2026-01-11
**Status**: Open
**Severity**: Medium
**Component**: Multi-scenario generation for pedestrian crossing scenarios

---

## Summary

Pedestrian crossing scenarios exhibit severe performance degradation when generating multiple scenarios using the multi-solve functionality. While the first scenario generates successfully, subsequent scenarios take excessively long (50+ minutes) and often fail to complete within reasonable timeframes.

This issue is **separate** from the precision mismatch bug (fixed in commit 0615e9e) that affected vehicle-based scenarios.

---

## Observed Behavior

### Test Configuration
- **Scenario**: `examples/pedestrian_crossing.yaml`
- **Command**: `cargo run --release -- -i examples/pedestrian_crossing.yaml -o test_ped_multisolve/ -n 3 -v`
- **Expected**: 3 diverse scenarios generated in < 5 minutes
- **Actual**: Only 1 scenario generated after 51+ minutes

### Timing Breakdown
- **Scenario 0**: Generated successfully in ~30 seconds
- **Scenario 1**: Did not complete after 51+ minutes of computation
- **Scenario 2**: Never reached

### System Behavior During Hang
- CPU usage: 99.6% (single core saturated)
- Process: Active Z3 solver computation
- Debug logs: Continuous Z3 AST creation/dropping operations
- Output: No progress indicators after first scenario

---

## Root Cause Analysis

### 1. High Time Step Count

**Pedestrian scenarios** use much finer time resolution than vehicle scenarios:

```yaml
# Pedestrian crossing
time_step: 0.1
duration: 10.0
# Result: 100 time steps
```

```yaml
# Vehicle scenarios (cut-in, school zone)
time_step: 0.5
duration: 10.0
# Result: 20 time steps
```

**Impact**: 5x more variables and constraints for Z3 to solve
- Variables per actor: `(px, py, vx, vy, ax, ay, lane) × 101 time steps = 707 variables`
- Total variables: 1,414 (2 actors × 707)

### 2. Complex 2D Motion Constraints

Pedestrians require 2D motion physics while vehicles are mostly 1D:

**Pedestrian constraints**:
- Longitudinal motion: `vx[t+1] = vx[t] + ax[t] * dt`
- **Lateral motion**: `vy[t+1] = vy[t] + ay[t] * dt` (unconstrained)
- **Speed magnitude**: `vx² + vy² ≤ max_speed²` (quadratic constraint)
- **Rectangular distance**: `|dx| > threshold_x OR |dy| > threshold_y`
- **2D position tracking**: Both `px` and `py` vary over time

**Vehicle constraints**:
- Longitudinal motion only: `vx[t+1] = vx[t] + ax[t] * dt`
- Lateral velocity: `vy = 0` (fixed, no lane changes for ego)
- Lateral position: `py = lane × lane_width + lane_width/2` (coupled to discrete lane)

**Impact**: Quadratic constraints (speed magnitude) are much harder for Z3 to solve than linear constraints.

### 3. Constraint Mode Configuration

The example YAML uses `ignore` mode for safety constraints:

```yaml
# NOTE: Using 'ignore' mode due to performance issues with 'enforce' mode
# TODO: Investigate why enforce mode times out
constraint_modes:
  min_ttc: ignore
  min_distance: ignore
```

**Impact**:
- `ignore` mode means no LTL constraints for TTC/distance
- Z3 has more degrees of freedom, larger solution space
- Blocking clauses may be less effective without guiding constraints

### 4. Insufficient Blocking Diversity

Current blocking clause only blocks initial conditions:

```rust
// Block only px0 and vx0 at t=0
let prev_px0_z3 = Real::from_rational((prev_px0 * 10.0) as i64, 10_i64);
let prev_vx0_z3 = Real::from_rational((prev_vx0 * 10.0) as i64, 10_i64);
```

**For pedestrians**, initial `py0` and `vy0` are also critical diversity factors:
- `py0`: Starting sidewalk position (left vs right)
- `vy0`: Initial lateral velocity (affects crossing timing)

**Impact**: Blocking only `px0, vx0` may not sufficiently constrain the solution space, causing Z3 to explore similar trajectories.

---

## Performance Comparison

| Metric | Vehicle (Cut-in) | Pedestrian (Crossing) | Ratio |
|--------|------------------|----------------------|-------|
| Time steps | 21 | 101 | 4.8x |
| Variables per actor | ~147 | ~707 | 4.8x |
| Total variables | ~294 | ~1,414 | 4.8x |
| Constraint complexity | Linear (mostly) | Quadratic (speed) | Higher |
| Blocking variables | 2 (px0, vx0) | 2 (px0, vx0) | Same |
| Time for 3 scenarios | ~3 seconds | 51+ minutes (failed) | 1000x+ |

---

## Technical Details

### Z3 Solver Behavior

During the hang, Z3 continuously creates and drops AST nodes:

```
[DEBUG] new ast: id = 11403, pointer = 0x559b64628360
[DEBUG] new ast: id = 11404, pointer = 0x559b646283c0
[DEBUG] drop ast: id = 11403, pointer = 0x559b64628360
[DEBUG] drop ast: id = 11404, pointer = 0x559b646283c0
[DEBUG] assert: (not (and (= pedestrian_px_0 0.0) (= pedestrian_vx_0 (/ 4.0 5.0))))
```

This indicates Z3 is:
1. Building constraint clauses
2. Testing satisfiability
3. Backtracking extensively
4. Struggling to find a solution that differs from the blocked scenario

### Blocking Clause Example

For the first pedestrian scenario:
```rust
// First scenario: pedestrian_px_0 = 0.0, pedestrian_vx_0 = 0.8
// Blocking clause: NOT(px_0 == 0.0 AND vx_0 == 0.8)
```

However, the pedestrian's trajectory is heavily influenced by:
- Initial `py0` position (sidewalk location)
- Lateral velocity `vy0` and acceleration `ay`
- Ego vehicle speed (which varies in range [8.0, 12.0])

Z3 may find many similar solutions that satisfy the blocking clause but still result in nearly identical scenarios.

---

## Potential Solutions

### 1. Expand Blocking Clause (Quick Fix)

**Change**: Block more initial variables for pedestrians

```rust
// Current (insufficient for pedestrians):
let prev_px0_z3 = Real::from_rational((prev_px0 * 10.0) as i64, 10_i64);
let prev_vx0_z3 = Real::from_rational((prev_vx0 * 10.0) as i64, 10_i64);

// Proposed (better diversity):
let prev_px0_z3 = Real::from_rational((prev_px0 * 10.0) as i64, 10_i64);
let prev_py0_z3 = Real::from_rational((prev_py0 * 10.0) as i64, 10_i64);
let prev_vx0_z3 = Real::from_rational((prev_vx0 * 10.0) as i64, 10_i64);
let prev_vy0_z3 = Real::from_rational((prev_vy0 * 10.0) as i64, 10_i64);
```

**Expected impact**: May improve diversity but won't solve fundamental performance issue

### 2. Reduce Time Resolution (Medium Impact)

**Change**: Increase `time_step` from 0.1s to 0.2s or 0.25s

```yaml
time_step: 0.2  # Was 0.1
duration: 10.0
# Result: 50 time steps (was 100)
```

**Pros**:
- 50% reduction in variables and constraints
- Should significantly improve solve time

**Cons**:
- Lower trajectory resolution
- May miss fine-grained pedestrian behavior

### 3. Simplify Distance Constraints (Medium Impact)

**Change**: Replace `RectangularDistanceGT` with simpler `ManhattanDistanceGT`

Current constraint is very precise but computationally expensive. Manhattan distance uses only linear constraints instead of absolute value disjunctions.

### 4. Enable Safety Constraints (Counterintuitive)

**Change**: Switch from `ignore` to `enforce` mode for safety constraints

```yaml
constraint_modes:
  min_ttc: enforce  # Was ignore
  min_distance: enforce  # Was ignore
```

**Rationale**:
- Adding constraints can actually help Z3 by reducing solution space
- Provides guidance for trajectory search
- May prevent exploring unrealistic scenarios

**Risk**: Could make initial solve slower, needs testing

### 5. Optimize Z3 Solver Configuration (Advanced)

**Change**: Tune Z3 solver parameters for quadratic constraints

```rust
// Add solver configuration
let mut cfg = Config::new();
cfg.set_timeout_msec(60000);  // 60 second timeout per scenario
cfg.set_param_value("sat.random_seed", "42");
cfg.set_param_value("smt.arith.solver", "2");  // Use optimized arithmetic solver
```

### 6. Trajectory-Based Blocking (Advanced)

**Change**: Block based on trajectory characteristics, not just initial conditions

```rust
// Block based on crossing point, crossing time, or trajectory shape
let crossing_time = calculate_crossing_time(prev_scenario);
let crossing_point = calculate_crossing_point(prev_scenario);
// Require: new_crossing_time != prev_crossing_time OR new_crossing_point != prev_crossing_point
```

**Expected impact**: High - ensures truly diverse scenarios

---

## Reproduction Steps

1. Ensure all fixes from commit 0615e9e are applied
2. Run: `cargo run --release -- -i examples/pedestrian_crossing.yaml -o test_ped/ -n 3 -v`
3. Observe: First scenario completes in ~30 seconds
4. Observe: Second scenario hangs indefinitely (50+ minutes)

### Expected vs Actual

**Expected behavior**:
```
[INFO] Generated scenario 1/3
[INFO] Generated scenario 2/3
[INFO] Generated scenario 3/3
[INFO] Successfully generated 3 scenario(s)
Time: < 5 minutes
```

**Actual behavior**:
```
[INFO] Generated scenario 1/3
[DEBUG] <thousands of Z3 AST operations>
<hangs indefinitely>
Time: 51+ minutes, killed
```

---

## Recommended Action Plan

### Phase 1: Quick Improvements (1-2 hours)
1. Expand blocking clause to include `py0, vy0` for pedestrians
2. Test with increased `time_step` (0.2s instead of 0.1s)
3. Measure performance improvements

### Phase 2: Configuration Tuning (2-3 hours)
1. Enable safety constraints (switch to `enforce` mode)
2. Test different Z3 solver configurations
3. Profile Z3 solver time per constraint type

### Phase 3: Algorithm Improvements (1-2 days)
1. Implement trajectory-based blocking for pedestrians
2. Optimize distance constraint encodings
3. Consider actor-specific blocking strategies

### Phase 4: Verification
1. Generate 10 pedestrian scenarios successfully in < 10 minutes
2. Verify scenarios are meaningfully diverse (different crossing times, positions)
3. Document optimal configuration in CLAUDE.md

---

## Related Issues

- Fixed in commit 0615e9e: Precision mismatch bug (separate issue)
- TODO in `examples/pedestrian_crossing.yaml:30`: "Investigate why enforce mode times out"
- README_ADVERSARIAL.md mentions pedestrian scenarios use different constraint types

---

## Impact Assessment

**Current state**:
- Vehicle scenarios: ✅ Working (3 scenarios in ~3 seconds)
- Pedestrian scenarios: ❌ Broken (multi-solve not viable)

**User impact**:
- Medium severity: Pedestrian scenarios are less common than vehicle scenarios
- Workaround exists: Generate single pedestrian scenarios (works fine)
- Blocks: Testing edge cases with diverse pedestrian behaviors

**Priority**: Medium - Should be fixed but not blocking critical functionality
