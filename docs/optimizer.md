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

## Scaling with the time horizon (SW-26)

`--optimize` becomes unusable well before the step size the non-optimizer examples use.
`examples/cut_in_left.yaml` ships with `time_step: 0.1` (101 steps); every
`examples/*_optimize_*.yaml` is pinned at `time_step: 0.5` (21 steps), which is the only
reason the feature looks usable. Nothing in the test suite optimizes at a small step.

### Where the time goes

Measured on `examples/cut_in_left.yaml`, `duration: 10.0`, one scenario, cartesian,
`min-distance`, release build, Z3 4.16.0 (`z3` crate 0.19.7), AMD Ryzen AI MAX PRO 390,
Linux. All times are seconds. Repeated runs agree to ±0.05 % on the `Optimize` column and
±1 % on the others, so the differences below are real and not noise.

| steps | `time_step` | plain `Solver`, no objective | `Optimize`, **no objective** | `Optimize` + objective |
|---:|---:|---:|---:|---:|
| 21 | 0.5   | 0.06 | 0.09 | 0.70 |
| 41 | 0.25  | 0.28 | 0.49 | 2.45 |
| 51 | 0.2   | 0.29 | 1.84 | 10.11 |
| 81 | 0.125 | 0.52 | 5.69 | 46.95 |
| 101 | 0.1  | 4.46 | 13.80 | 335.44 |

Three things follow, and the first two rule out the obvious suspects.

**Constraint construction is not the cost.** Building and asserting the entire encoding
takes 2–8 ms at every horizon in the table, and building the objective takes under 1 ms.
100 % of the growth is inside `check()` — Z3's search.

**The objective's term count is not the cost either.** Asserting the same objective
constraints on a plain `Solver` and doing one satisfiability check costs 5.6 s at 101
steps, against 4.5 s without them — a factor of 1.26. The pairwise × per-step terms are
cheap; it is optimizing over them that is not.

**The cost is `Optimize`, in two independent factors.** With *no objective declared at
all*, Z3's `Optimize` is 3–11× slower than `Solver` on a byte-identical constraint set
(13.8 s vs 4.5 s at 101 steps). On top of that it runs a branch-and-bound loop — Z3's own
statistics report `num checks: 8` at every horizon — and the later iterations, which must
prove that no scenario beats the incumbent, are the expensive ones. The optimality *proof*
is the work, not finding the optimum: on this spec the optimum is 5.00 m at every horizon,
equal to the spec's `min_distance`, and a satisfying scenario at that value is found in the
first check.

The other three targets scale the same way, from the same cause (21 → 41 steps):
`max-ttc` 1.05 → 24.1 s, `min-severity` 2.57 → 39.8 s, `min-ttc` 6.29 → 118.2 s.

### What was tried and does not work

Each of these was measured, not reasoned about:

- **Plain `Solver` with a binary search on the objective bound.** Worse, in both forms. A
  fresh encoder per probe: 202 s at 81 steps against `Optimize`'s 47 s. One incremental
  solver with `push`/`pop` per probe: 80 s at 51 steps against `Optimize`'s 10 s. The
  `UNSAT` probes dominate — a single one cost 117 s at 81 steps. `Optimize` shares solver
  state across bounds and is already better than the naive alternative.
- **Z3 optimizer parameters.** At 51 steps, against a 10.1 s baseline:
  `opt.optsmt_engine=symba` 13.9 s, `opt.optsmt_engine=farkas` 14.0 s,
  `opt.incremental=true` 48.0 s, `opt.enable_sat=false` 10.0 s. The default engine is the
  best of them.
- **Removing the `ite` terms from `min-distance` and hoisting the choice into Boolean
  selectors** (SW-11's lesson: a disjunction in an antecedent propagates, in a consequent it
  is searched). 51 steps 3.2 s against 10.1 s — but 41 steps 4.5 s against 2.45 s, and 81
  steps did not finish in 570 s against 47 s.
- **Handing `Optimize` the lower bound the encoding already implies** (`dist_obj >= 5.0`,
  a logical consequence of the enforced `min_distance` constraint, which uses the same
  same-lane predicate). 81 steps 6.5 s against 47.0 s — but 101 steps did not finish in
  580 s against 335 s.

The last two are the important ones. Both are strict structural improvements — fewer terms,
strictly more information for the solver — and both make some horizons an order of magnitude
faster and others an order of magnitude slower. Runtime here is not a smooth function of the
encoding. Anyone proposing an encoding change for speed must measure it at **several**
horizons; a win at one step size says nothing about the next.

### Conclusion

This is inherent to running Z3's `Optimize` on a QF_LRA problem of this size. The optimality
proof requires case analysis over every (pair, step), which is linear in the horizon in term
count and far worse than linear in search. There is no encoding-level fix that is reliably
better across horizons, and the two candidates that looked most promising were each refuted
by measurement above.

Practical guidance: `--optimize` is usable up to roughly 50 steps. Past that, generate with
the plain solver (which handles 101 steps in 4.5 s) and treat optimization as a tool for
short or coarse horizons. Shortening `TTC_LEVELS` remains the lever for the two TTC targets
specifically.

## Known Limitations

- Each target optimizes a single scalar value.
- The two TTC targets measure TTC at the resolution of the `TTC_LEVELS` ladder, and
  `max-ttc` saturates at its top level (60 s).
- `MinimizeSeverity` is a maximiser; the identifier is a known misnomer.
- The optimizer does not scale past roughly 50 time steps; see
  [Scaling with the time horizon](#scaling-with-the-time-horizon-sw-26) for the measured
  numbers and the cause. `min-ttc` additionally adds one disjunction per ladder level
  (measured on `examples/cut_in_left_optimize_min_ttc.yaml`: 2 s → 6 s cartesian).

## Architecture

Key files:

| File | Contents |
|------|----------|
| `src/solver/backend.rs` | `OptimizerBackend` struct wrapping Z3 `Optimize` |
| `src/solver/encoder.rs` | `impl GenericEncoder<OptimizerBackend>` with objective encoding |
| `src/solver/encoder.rs` | `EncoderAccessor` trait for backend-agnostic variable access |
| `src/lib.rs` | `generate_with_optimizer()` orchestration entry point |
