# Optimizer

## Overview

The optimizer component finds *optimal* scenarios by replacing the standard Z3 `Solver` with Z3 `Optimize`. While the normal generation path finds any satisfying assignment, the optimizer steers the solver toward an extremal value of a chosen metric (e.g., minimum distance between actors, smallest time-to-collision).

Every objective is scored over **the same predicates the validator uses when it reports `min_distance` and `min_ttc`** — the same "same lane" test, the same closing-speed floor. That is not decoration: before SW-14 the objectives used a narrower same-lane test than the validator, so the tool optimised one quantity and reported another.

Use the optimizer when you need a specific boundary scenario rather than an arbitrary valid one — for example, the closest possible approach that still satisfies all constraints, or the safest scenario with the largest gap.

## Usage

### CLI

```sh
scenario-weaver -i scenario.yaml -o output/ --optimize <target>
```

### YAML field

The YAML field and the CLI flag take **different spellings of the same set**: the YAML
values are `snake_case` (serde), the CLI values are `kebab-case` (clap). Writing a CLI
spelling in YAML is a parse error.

```yaml
optimization_target: minimize_distance
```

### Available targets

| CLI value | YAML value | Rust variant | Description |
|-----------|------------|--------------|-------------|
| `min-distance` | `minimize_distance` | `MinimizeDistance` | Smallest same-lane longitudinal gap |
| `min-ttc` | `minimize_ttc` | `MinimizeTtc` | Smallest same-lane time-to-collision |
| `min-severity` | `minimize_severity` | `MinimizeSeverity` | **Maximises** the highest same-lane closing speed — the name is a misnomer, see below |
| `max-ttc` | `maximize_ttc` | `MaximizeTtc` | Largest same-lane time-to-collision (safest scenario) |
| — | `none` | `None` | No optimization; the plain solver path |

## Optimization Targets

| Target | CLI Value | Objective (LRA) | Direction | What It Finds |
|--------|-----------|-----------------|-----------|---------------|
| MinimizeDistance | `min-distance` | min\_t \|px\_i - px\_j\| over same-lane pairs | minimise | Closest physical approach between actors |
| MinimizeTtc | `min-ttc` | min\_t gap / closing\_speed over same-lane approaching pairs, read off the level ladder | minimise | Worst near-miss the constraints allow |
| MinimizeSeverity | `min-severity` | max\_t \|vx\_i - vx\_j\| over same-lane pairs | **maximise** | Highest relative approach speed |
| MaximizeTtc | `max-ttc` | min\_t gap / closing\_speed, read off the level ladder | maximise | Safest scenario by time-to-collision |

### `min-severity` is a maximiser

The `MinimizeSeverity` variant **maximises** the highest same-lane closing speed. Severity
correlates with relative impact speed, and driving that up is what an adversarial scenario
generator is for, so the *behaviour* is the useful one; the *name* is wrong. Renaming it to
`MaximizeSeverity` (`maximize_severity` / `--optimize max-severity`) is the recommended fix
and is recorded on the SW-14 issue; it reaches `src/lib.rs`, outside that issue's edit set,
so the identifier still lies while every prose surface now says "maximise".

**If you want the safest scenario, use `max-ttc`, not `min-severity`.**

### `max-ttc` maximises TTC, not gap

Until SW-14, `max-ttc` dispatched to a *distance* objective: it maximised the minimum gap
and reported that as TTC. It now maximises TTC, and the two are not the same thing — the
largest TTC is reached by *matching speeds*, not by separating. On
`examples/cut_in_left_optimize_max_ttc.yaml` the returned scenario's minimum gap fell from
113.50 m to 5.48 m while its TTC went from a measured 12.37 s to no measurable conflict at
all. If you want the largest gap, use the `min-distance` objective's mirror — which this
tool does not currently expose as a separate target.

## Design Decisions

### LRA only

All objectives are expressed in Linear Real Arithmetic. The whole encoding is deliberately
confined to QF_LRA — every multiplication in `src/solver/` is `constant × variable` — and
Z3's `Optimize` has essentially no support for non-linear objectives.

### How TTC is optimised inside LRA

TTC is `gap / closing_speed`, and division is not linear. Worse, **no linear function can
rank scenarios by TTC at all**: TTC is scale-invariant — `(d, v)` and `(λd, λv)` have the
same TTC — and no linear function of `d` and `v` is. So a single linear proxy cannot be
monotone in TTC, whichever proxy you pick.

The previous encoding tried anyway. It minimised `|Δpx| − dt·|Δvx|`, which with `dt = 0.5`
inverts the ranking it claims to produce:

| state | true TTC | old proxy |
|---|---|---|
| `d = 2 m, v = 0` | ∞ | 2.0 |
| `d = 50 m, v = 20 m/s` | 2.5 s | 40.0 |

Minimising that proxy prefers the **infinite**-TTC state.

The way out is that TTC only goes non-linear when the threshold is a *variable*. For a
**constant** `T`, both directions are linear:

```text
ttc(d, v) ≥ T   ⟺   d ≥ T · v        (v > 0)
ttc(d, v) ≤ T   ⟺   d ≤ T · v        (v > 0)
```

So both TTC objectives are expressed over a fixed ladder of constant levels
(`TTC_LEVELS` in `src/solver/encoder.rs`): 0.25 s steps below 1 s, widening to 10 s steps
beyond 30 s, topping out at 60 s. One Boolean per level says "every approaching step clears
this level" (`max-ttc`) or "some step falls below it" (`min-ttc`), and a linear
pseudo-Boolean sum reads the ladder off as a number of seconds. Every coefficient is a
constant, so the encoding stays in QF_LRA.

**The levels are nested one way only, and this makes the reported value a certified bound
rather than an equality:**

- `min-ttc` forces a step at or below its level to exist, so
  `measured min TTC ≤ optimal_value`. It never over-claims danger.
- `max-ttc` forces every approaching step to clear its level, so
  `optimal_value ≤ measured min TTC`. It never over-claims safety.

`tests/optimizer_test.rs::test_ttc_objectives_bound_the_reported_ttc` asserts both.

**Limits.** The measurement is at grid resolution, so an optimum of `3.0` means "somewhere
in the cell below 3.0 s", not exactly 3.0 s. `max-ttc` saturates at the top of the ladder:
a scenario with no closing pair at all has infinite TTC, and no finite ceiling can
distinguish "60 s" from "never". If your scenario lives beyond 60 s of TTC, `max-ttc` will
report 60.0 and stop caring.

### Single scalar objective

Each target optimizes exactly one scalar value. Lexicographic multi-priority objectives are
not used. Note that the obvious lexicographic route for TTC — minimise `d`, then maximise
closing speed — does **not** fix the monotonicity problem: it still prefers `(d = 2, v = 0)`
over `(d = 50, v = 20)`, because it compares `d` first. The level ladder is what fixes it.

### EncoderAccessor trait

The `EncoderAccessor` trait provides backend-agnostic access to Z3 variables (positions, velocities, time-step constants). This allows the same scenario-specific constraint logic to work with both the `Solver` and `Optimizer` backends without duplication.

## How It Works

1. The encoding pipeline runs identically to the normal path: kinematics, LTL temporal constraints, and scenario-specific constraints are all asserted.
2. After all constraints are in place, the optimizer adds an objective function corresponding to the chosen target.
3. Z3 `Optimize` searches for a satisfying assignment that extremizes the objective.
4. The optimal value is extracted from the resulting Z3 model and reported alongside the generated scenario.

## Known Limitations

- Each target optimizes a single scalar value.
- The two TTC targets measure TTC at the resolution of the `TTC_LEVELS` ladder, and
  `max-ttc` saturates at its top level (60 s).
- `MinimizeSeverity` is a maximiser; the identifier is a known misnomer.
- The optimizer may be slower than plain SAT solving for complex scenarios with many actors
  or long time horizons. `min-ttc` in particular adds one disjunction per ladder level
  (measured on `examples/cut_in_left_optimize_min_ttc.yaml`: 2 s → 6 s cartesian).

## Architecture

Key files:

| File | Contents |
|------|----------|
| `src/solver/backend.rs` | `OptimizerBackend` struct wrapping Z3 `Optimize` |
| `src/solver/encoder.rs` | `impl GenericEncoder<OptimizerBackend>` with objective encoding |
| `src/solver/encoder.rs` | `EncoderAccessor` trait for backend-agnostic variable access |
| `src/lib.rs` | `generate_with_optimizer()` orchestration entry point |
