# Optimizer

← [Back to README](../README.md)

## Overview

The optimizer replaces the standard Z3 `Solver` with Z3 `Optimize`. The plain generation path
finds any satisfying assignment; the optimizer steers the search toward an extremal value of a
chosen metric instead – the closest approach two actors ever make, or the worst time-to-collision (TTC)
a spec's constraints still allow.

Use it when you need a specific boundary scenario rather than an arbitrary valid one: the
tightest gap that still satisfies every constraint, or the safest scenario the spec admits.

## Usage

```bash
scenario-weaver -i scenario.yaml -o output/ --optimize <target>
```

or in YAML:

```yaml
optimization_target: minimize_distance
```

The YAML field and the CLI flag spell the same four targets differently on purpose: YAML uses
`snake_case` (serde's convention), the CLI uses `kebab-case` (clap's). Writing a CLI spelling in
YAML, or a YAML spelling on the command line, is a parse error rather than a silent no-op.

| CLI value | YAML value | What it finds |
|-----------|------------|---------------|
| `min-distance` | `minimize_distance` | Closest same-lane physical approach |
| `min-ttc` | `minimize_ttc` | Worst near-miss the constraints allow |
| `max-severity` | `maximize_severity` | Highest same-lane closing speed – the most severe interaction |
| `max-ttc` | `maximize_ttc` | Safest scenario by time-to-collision |

Omitting `--optimize` and `optimization_target` alike (the default, `none`) uses the plain
solver path with no objective at all.

`max-severity` **maximizes**: it drives the highest same-lane closing speed up, because a
severe interaction is a high relative-impact-speed one, and finding the worst interaction is
exactly what an adversarial generator is for. If what you want instead is the safest scenario,
use `max-ttc`, not `max-severity` – the two names describe opposite objectives despite both
containing "max".

## Same predicate as the validator

Every objective is scored over the same "same lane" test and the same closing-speed floor that
`compute_validation_metrics` uses when it reports `min_distance` and `min_ttc` for a generated
scenario. This is what makes the optimized number and the reported metric the same number – an
optimizer that quietly used a narrower or different same-lane test than the validator would
optimize one quantity and report another, which defeats the purpose of asking for a boundary
scenario at all. If you optimize for `min-distance` and the tool reports `min_distance: 5.0` in
the output, that 5.0 is the value the objective actually converged on, not an independent
re-measurement.

## Why TTC needs a level ladder

Time-to-collision is `gap / closing_speed`, and division is not linear. The whole encoding is
deliberately confined to Linear Real Arithmetic (QF_LRA): every multiplication anywhere in
`src/solver/` is a constant times a variable, never variable times variable, because Z3's
`Optimize` backend has essentially no support for non-linear objectives. Worse than the division
itself: TTC is scale-invariant, `(d, v)` and `(λd, λv)` have the same TTC for any `λ`, and no
linear function of `d` and `v` can be. There is no single linear proxy that ranks scenarios by
TTC correctly, whichever one you pick.

The way out is that TTC only becomes non-linear when the *threshold* is a variable. For a fixed
constant `T`, both directions stay linear:

```text
ttc(d, v) ≥ T   ⟺   d ≥ T · v    (v > 0)
ttc(d, v) ≤ T   ⟺   d ≤ T · v    (v > 0)
```

So both TTC objectives are expressed over a fixed ladder of constant levels – fine-grained below
1 s, coarsening to 10 s steps beyond 30 s, topping out at 60 s (`TTC_LEVELS` in
`src/solver/objectives.rs`). One Boolean per level asks "does every approaching step clear this
level" (`max-ttc`) or "does some step fall below it" (`min-ttc`), and a linear pseudo-Boolean
sum reads the ladder off as a number of seconds. Every coefficient in that sum is a constant, so
the whole thing stays inside QF_LRA.

This makes the reported optimum a **certified bound**, not an exact TTC: a value of `3.0`
means "somewhere in the cell below 3.0 s", at the ladder's resolution, not exactly 3.0 s. The
two directions bound opposite ways: `min-ttc` never over-claims danger (the true minimum TTC is
at or below the reported value), and `max-ttc` never over-claims safety (the true minimum TTC is
at or above the reported value). `max-ttc` also saturates at the ladder's top – a scenario with
no closing pair at all has infinite TTC, and no finite ceiling can tell "60 s" from "never", so
`max-ttc` reports 60.0 in that case and stops caring how much larger the true value is.

## Single scalar objective

Each target optimizes exactly one scalar value; there is no lexicographic multi-priority
objective. The obvious lexicographic route for TTC (minimize gap, then maximize closing speed as
a tiebreaker) does not fix the non-linearity problem: it still ranks `(d = 2, v = 0)` ahead of
`(d = 50, v = 20)` because it compares `d` first, and the first is the infinite-TTC state. The
level ladder above is what actually fixes the ranking; a second-priority objective on top of a
broken first-priority one does not.

## Interaction with `-n`

`--optimize` and `-n`/`num_scenarios` compose the way you would expect: each of the `n`
scenarios is optimized independently against the same objective, not jointly. There is no
notion of "the best of n" or "n scenarios spanning a range of the objective" – each solve
extremizes the objective from scratch, so with narrow parameter ranges the `n` results may be
close to identical.

The Z3 `Optimize` backend is substantially slower than the plain `Solver` on the same encoding,
and its cost grows quickly with the time horizon – the number of steps implied by
`duration / time_step`. In practice, `--optimize` stays comfortably usable at the coarse time
steps the shipped `*_optimize_*.yaml` examples use; a fine time step that is fine for plain
generation can make the optimizer slow enough to be impractical. If you need optimization over a
long or fine-grained horizon, coarsen `time_step` before reaching for other workarounds.

## Further reading

This document stays at the concept level on purpose. For the exact per-target Z3 encoding,
the `EncoderAccessor`/backend-agnostic machinery, and the underlying Satisfiability Modulo Theories (SMT) formulas, see
[z3_constraints.md](z3_constraints.md). For the full YAML schema including `optimization_target`
and its neighboring fields, see [yaml-reference.md](yaml-reference.md). For how the optimizer's
encoding pipeline relates to the plain solver path, see [architecture.md](architecture.md).
