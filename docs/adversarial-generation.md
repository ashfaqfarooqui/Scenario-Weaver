# Adversarial Scenario Generation

← [Back to README](../README.md)

Adversarial generation asks the solver for scenarios that break a safety property rather than
respect it – the worst-case trajectory a spec permits, not an arbitrary valid one. This is
useful for testing collision-avoidance logic, exercising emergency braking, and building
datasets that include near-misses rather than only clean traffic.

There is no separate adversarial code path. Every scenario is generated the same way: a Linear Temporal Logic (LTL)
formula is built from the spec's constraints, lowered to Z3, and solved. What changes is which
formula gets built. A constraint that is *enforced* becomes `G(safe)`, meaning the safe condition must
hold at every step. A constraint that is *violated* becomes `F(¬safe)`, meaning the safe condition must
fail at some step. Adversarial generation is ordinary solving over a negated safety formula, not
a different solver, encoder, or search strategy.

## Constraint modes

Each of the seven controllable constraints (`min_ttc`, `min_distance`, `max_velocity`,
`min_velocity`, `min_lateral_distance`, `max_relative_velocity`, and `max_acceleration`) takes
one of three modes:

| Mode | LTL shape | Meaning |
|------|-----------|---------|
| `enforce` | `G(safe)` | The condition holds at every time step (default). |
| `violate` | `F(¬safe)` | The condition fails at some time step. |
| `ignore` | – | The constraint is omitted from the formula entirely. |

`enforce` and `violate` are not complements of each other over the whole trajectory – `violate`
only asks for one bad instant, not a trajectory that is unsafe throughout. A scenario with
`min_ttc: violate` can still spend most of its duration at a safe following distance; it just
has to dip under the threshold somewhere. This is deliberate: it is what makes `violate` on one
constraint and `enforce` on another simultaneously satisfiable, which is the whole point of
per-constraint control (below).

For constraints whose safe condition is a *positive* comparison, time-to-collision (TTC) or distance exceeding a
threshold, this is exactly `enforce → G(atom)`, `violate → F(¬atom)`. `max_relative_velocity` is
the one exception: its safe condition is staying *under* a limit, so the atom itself
(`RelativeVelocityGT`) already names the unsafe state, and `enforce`/`violate` are lowered
against its negation rather than the atom directly. The effect at the YAML level is identical:
`enforce` always means "safe holds everywhere", and `violate` always means "safe fails
somewhere" – this is only an implementation detail worth knowing if you are reading
`push_constraint` in `src/scenarios/mod.rs`.

`max_deceleration` and `max_lateral_acceleration` are asserted directly by the encoder whenever
they are present in the YAML and have no `constraint_modes` entry of their own – they are always
enforced. `max_acceleration` at the top level is report-only (see
[yaml-reference.md](yaml-reference.md)); the bound the solver actually respects is each actor's
own `acceleration` range.

## The two YAML forms

`constraint_modes` accepts either a bulk shorthand string or a per-constraint mapping.

**Bulk shorthand** sets every constraint to the same mode:

```yaml
constraint_modes: violate_all   # every constraint must be violated somewhere
constraint_modes: ignore_all    # every constraint is omitted (maximum freedom)
constraint_modes: enforce_all   # every constraint is enforced (the default; can be omitted)
```

**Per-constraint mapping** sets each of the seven independently:

```yaml
constraint_modes:
  min_ttc: violate               # find TTC violations
  min_distance: enforce          # but keep a safe following distance
  max_velocity: ignore
  min_velocity: ignore
  min_lateral_distance: ignore
  max_relative_velocity: ignore
  max_acceleration: enforce
```

A constraint left out of the mapping falls back to `enforce`, so a short mapping that only
mentions the constraints you care about is fine. The mapping form is validated with
`deny_unknown_fields`, so a typo such as `min_tcc` is a parse error naming the bad key rather
than a silently-ignored setting.

The result of mixing modes is a scenario that is adversarial along exactly the axis you asked
for and safe everywhere else – a TTC violation with distance still respected, or a speeding ego
that never gets close enough to another actor to threaten a collision.

## `--adversarial`

```bash
cargo run --release -- -i examples/cut_in_left.yaml -o adversarial/ --adversarial
```

The `--adversarial` flag is a CLI override, applied after the YAML is parsed: it replaces
whatever `constraint_modes` the spec declared with `violate_all`. Nothing else about the pipeline
changes – the same encoder, the same LTL generation, the same solver call. If you need anything
other than "violate every constraint", write `constraint_modes` in the YAML instead; the flag
cannot express a mixed mode.

## Per-scenario overrides

`generate_default_safety` in `src/scenarios/mod.rs` builds pairwise safety constraints across
every actor pair, and most scenario types use it unmodified. Two do not:

- **`head_on`** overrides `generate_safety`. The scenario has three actors (ego, a slow vehicle
  the ego is overtaking, and an oncoming vehicle in the passing lane), and `enforce` still
  applies to every pair, since safety is a property of the whole scene. `violate`, though, is
  scoped to the ego-oncoming pair only: demanding that *every* pair breach its TTC or distance
  threshold, including the ego and the slow vehicle it is simply following, makes the scenario
  unsatisfiable. Only the ego-oncoming pair is what "head-on" refers to, so `violate` is
  scoped to that pair; the rest fall back to `ignore` for that constraint.
- **`pedestrian_crossing`** replaces the pairwise TTC/distance test with a rectangular safety
  box around the ego and the pedestrian (`RectangularDistanceGT`, longitudinal and lateral
  thresholds derived from `min_distance`), because "same lane" has no meaning for a pedestrian
  crossing perpendicular to the road. `min_ttc`'s mode still drives a guarded pedestrian-specific
  TTC atom on the same enforce/violate/ignore triple; it is just a different proposition than the
  one `generate_default_safety` would have built.

Any new scenario type that overrides `generate_safety` should keep this shape – respect the
declared mode per constraint, and be explicit in code (and in a comment) about which pairs
`violate` is scoped to when "every pair" would make the spec unsatisfiable. See
[creating-scenario-types.md](creating-scenario-types.md) for the trait this hangs off of.

## Use cases

- **Emergency system testing**: generate near-misses to validate braking and collision-avoidance
  logic under conditions the plain solver would never surface on its own.
- **Edge-case discovery**: search a parameter space for the worst case a spec allows, rather
  than sampling arbitrary valid scenarios and hoping to land on one.
- **Training and evaluation data**: build datasets that include violations alongside clean
  traffic, rather than only the latter.
- **Compliance and hazard documentation**: demonstrate system behavior under a specific,
  reproducible hazard rather than an informally described one.

## Worked example: violate TTC, keep distance

`examples/cut_in_left_adversarial_ttc.yaml` asks for a cut-in where the NPC's lane change forces
a TTC violation but the minimum distance is still respected throughout:

```yaml
scenario_type: cut_in_left
time_step: 0.5
duration: 10.0

road:
  num_lanes: 2
  lane_width: 3.5
  lane_directions: [1, 1]

actors:
  - id: ego
    role: ego
    lane: 1
    position: 50.0
    speed: 15.0
    direction: 1
    acceleration: [-8.0, 3.0]

  - id: npc
    role: npc
    lane: 0
    position: [60.0, 80.0]
    speed: [12.0, 14.0]
    direction: 1
    acceleration: [-8.0, 3.0]
    lane_changes:
      - direction: right
        start_time: [2.5, 4.0]
        duration: [3.0, 4.0]

min_ttc: 3.0
min_distance: 5.0

constraint_modes:
  min_ttc: violate       # find scenarios where TTC drops below 3.0 s
  min_distance: enforce  # but the 5.0 m gap is never breached

num_scenarios: 5
```

Run it directly, or use `--adversarial` if you want every constraint violated instead of just
TTC:

```bash
cargo run --release -- -i examples/cut_in_left_adversarial_ttc.yaml -o output/
cargo run --release -- -i examples/cut_in_left.yaml -o output/ --adversarial
```

For a full `violate_all` example, see `examples/cut_in_left_adversarial_all.yaml`; for a
single-threshold violation on a non-safety-pair constraint (the ego exceeding a posted speed
limit while TTC and distance stay enforced), see `examples/speed_limit_violation.yaml`.

## See also

- [yaml-reference.md](yaml-reference.md): full field-by-field schema, including every
  `constraint_modes`-adjacent threshold field.
- [authoring-scenarios.md](authoring-scenarios.md): end-to-end walkthrough of writing a YAML
  scenario, constraint modes included.
- [architecture.md](architecture.md): how the LTL formula reaches Z3.
- [z3_constraints.md](z3_constraints.md): the per-constraint SMT encoding, for contributors.
