# Authoring Scenarios

This guide walks through writing a ScenarioWeaver scenario from an empty file to
generated output. It is task-oriented: it builds a real scenario block by block
and explains why each piece is there. For the exhaustive schema (every field,
type, and default) see [yaml-reference.md](yaml-reference.md).

## The mental model

ScenarioWeaver is not a simulator you script. You do not tell an actor to move
two meters left and then accelerate for three seconds. Instead you write a
declarative specification: the actors, the road, the initial conditions, and the
constraints that must hold. The generator translates that specification into
temporal logic, hands it to the Z3 solver, and the solver finds concrete values
(positions, speeds, timings) that satisfy everything at once. What you author
is the shape of the scenario; the solver returns a concrete trajectory that
fits it.

You control how much freedom the solver gets through the choice between a fixed
value and a range. A fixed value is an equality the solver must honor
exactly. A range `[min, max]` is an interval the solver may choose from. A
specification built entirely from fixed values asks for one particular scenario;
one built from ranges asks for a whole family, and lets the solver find the
members that are actually feasible. Most useful specifications mix the two,
fixing the values that matter and leaving the rest as ranges.

## A worked example: a cut-in

We will build a cut-in scenario in which an NPC vehicle merges from the left
lane into the ego's lane ahead of it. This is the scenario shipped as
`examples/cut_in_left.yaml`; we assemble it here piece by piece.

### The scenario frame

Every file opens by naming the behavioral model and the time frame.

```yaml
scenario_type: cut_in_left

time_step: 0.1
duration: 10.0
```

`scenario_type` selects the model – `cut_in_left` generates the LTL that
defines what a left cut-in means and how it is validated. `time_step` and
`duration` set the discretization: a ten-second scenario sampled every 0.1 s is
100 steps. A finer step gives smoother trajectories and a more precise
interaction, at the cost of solve time; a coarser step solves faster. Ten
seconds at 0.1 s is a reasonable default for a highway maneuver.

### The road

```yaml
road:
  num_lanes: 3
  lane_width: 3.5
  lane_directions: [1, 1, -1]

coordinate_system: cartesian
```

The road is three lanes wide. `lane_directions` gives the direction of each
lane: the first two run forward (`+1`), the third runs backward (`-1`). The list
must have exactly `num_lanes` entries, and all forward lanes must come before
all backward lanes – an interleaved layout is rejected, because the exporter
places lanes by index and would otherwise put a lane where its traffic does not
drive. `coordinate_system: cartesian` selects the 2D point-mass model, which is
the default and the right choice unless you specifically need the steering
dynamics of the bicycle model (see [coordinate-systems.md](coordinate-systems.md)).

### The ego

```yaml
actors:
  - id: ego
    role: ego
    lane: 1
    position: [0.0, 55.0]
    speed: [14.0, 16.0]
    direction: 1
    acceleration: [-8.0, 3.0]
```

The ego starts in lane 1, a forward lane, travelling forward. Its starting
position and speed are ranges rather than fixed values: the solver may spawn it
anywhere in the first 55 m of road at any speed between 14 and 16 m/s. Its
`direction` must match the direction of the lane it occupies; lane 1 is `+1`, so
`direction: 1`. The `acceleration` range `[-8.0, 3.0]` is the real per-actor
acceleration bound – the solver keeps the ego's longitudinal acceleration inside
that interval, allowing a firm brake down to -8 m/s² and a gentle acceleration
up to 3 m/s².

### The NPC and its cut-in

```yaml
  - id: npc
    role: npc
    lane: 0
    position: [20.0, 80.0]
    speed: [16.0, 20.0]
    direction: 1
    acceleration: [-8.0, 3.0]

    lane_changes:
      - direction: right
        start_time: [1.4, 3.5]
        duration: [3.0, 4.0]
```

The NPC starts in lane 0, the left forward lane, moving a little faster than the
ego. The cut-in itself is the `lane_changes` block. `direction: right` moves the
NPC one lane in the direction of its own travel – for a forward actor, that
lowers nothing and raises the lane index from 0 to 1, into the ego's lane. The
`start_time` and `duration` are both ranges, so the solver chooses when the
merge begins (somewhere between 1.4 and 3.5 s) and how long it takes (three to
four seconds). The solver discovers the actual lateral trajectory under
smoothness constraints; you specify only the direction, the window, and how long
it lasts.

One rule matters here: for a cut-in, the NPC's lane changes must end in the
ego's lane. If they did not, the two vehicles would never share a lane, no
conflict would be asserted, and the safety constraints would never be evaluated
for the pair. The NPC ends in lane 1, the ego sits in lane 1, so the conflict is
real.

### The safety constraints and generation count

```yaml
min_ttc: 3.0
min_distance: 5.0
num_scenarios: 5
```

`min_ttc` and `min_distance` are the safety thresholds: at least three seconds
of time-to-collision and at least five meters of longitudinal gap. By default
these are enforced – the solver must keep them satisfied throughout. `num_scenarios: 5`
asks for five distinct scenarios rather than one; the generator adds blocking
clauses between solutions so that each is meaningfully different from the last,
which is how you get diversity out of the ranges you wrote.

### Generating it

With the file saved as `examples/cut_in_left.yaml`, run:

```bash
cargo run --release -- -i examples/cut_in_left.yaml -o output/
```

Because `num_scenarios` is 5, the generator writes five sets of output files
into `output/`, named `scenario_0` through `scenario_4`. Each set contains six
formats: the canonical JSON, an OpenSCENARIO `.xosc` and its companion
OpenDRIVE `.xodr`, a static SVG top-down view, an animated GIF, and an OpenLABEL
`.ol.json`. For what each format is and when to use it, see
[output-formats.md](output-formats.md). Each scenario is a concrete trajectory
the solver found inside the ranges you specified, with the cut-in performed and
the three-second TTC held throughout.

## Turning a scenario adversarial

The scenario above is safe by construction – the solver was told to enforce the
safety constraints, so every scenario it returns respects them. The point of
adversarial generation is the other way around: to ask for scenarios that
*break* a constraint, which is how you find the near-misses and edge cases worth
testing against.

This is what `constraint_modes` is for. Each safety constraint can be enforced
(must hold), violated (must be broken somewhere), or ignored (dropped from the
formula entirely). To make the cut-in above adversarial in its entirety, replace
the default enforcement with the `violate_all` shorthand:

```yaml
min_ttc: 3.0
min_distance: 5.0

constraint_modes: violate_all

num_scenarios: 5
```

Now the solver must find cut-ins that violate the safety constraints – the NPC
merges in a way that drops the TTC below three seconds or the gap below five
meters. The thresholds still matter: `violate` means "break this specific
bound", so the numbers define what counts as a violation.

You rarely want to violate everything at once. The per-constraint form lets you
be selective – violate the time-to-collision while still requiring a safe
distance, for instance, which produces a fast, close-timed merge that never
actually touches:

```yaml
constraint_modes:
  min_ttc: violate
  min_distance: enforce
```

Any constraint you omit from the mapping defaults to `enforce`. The same effect
as `violate_all` is also available from the command line without editing the
file, via the `--adversarial` flag. For the full treatment (what each mode
encodes, how violation interacts with the LTL, and when to reach for each) see
[adversarial-generation.md](adversarial-generation.md).

## A second example: ranges for diversity, and optimization

The cut-in above already uses ranges to get five different scenarios. It is
worth seeing why that works, and what the alternative gives you.

When you write `speed: [14.0, 16.0]` and ask for five scenarios, you are handing
the solver an interval and asking for five distinct points that each satisfy
every constraint. Before generation starts, every such range on every actor,
ego included, is cut into five strata, and a seeded Latin hypercube confines
each of the five scenarios to one stratum per range; a plain blocking clause
still runs underneath to rule out an exact repeat. On `cut_in_left.yaml` this
takes the NPC's initial position from 1.86 m of its declared 60 m range to
48 m, and the ego's from bit-identical to 33 m of its 55 m range (measured at
`e851b9b`, `-n 5`). The wider your ranges, the more room there is to spread;
a specification that fixes every value can only ever produce one scenario, no
matter what `num_scenarios` says, because there is nothing left to sample.

This spreads *where each scenario starts*, not *what each scenario does*.
`cut_in_left`'s lane change still happens at the same simulated time in all
five scenarios, because the timing range in the spec is collapsed to its
midpoint before the solver runs (tracked separately as SW-55) – so do not
expect `-n` alone to vary manoeuvre structure. Sampling is seeded (`--seed`,
default a fixed constant): the same seed reproduces the same batch, a
different seed a different one, and there is no separate YAML field for it:
the ranges you already declare are what gets sampled. A stratum can turn out
to have no solution at all (`cut_in_left` requires the NPC ahead of the ego,
so the ego-high/NPC-low corner is empty); the solver relaxes it and logs a
`warn!` rather than failing the run, so a batch can be less evenly spread than
requested without being wrong.

Diversity is one way to explore the feasible region. Optimization is the other.
Instead of asking for several scenarios spread across what is feasible, you can
ask for the single scenario that is extreme along some axis – the worst
near-miss, the closest approach, the most severe interaction. Set
`optimization_target`:

```yaml
min_ttc: 3.0
min_distance: 5.0

optimization_target: minimize_ttc
num_scenarios: 1
```

`minimize_ttc` tells the solver to find the scenario with the smallest
time-to-collision the constraints still allow: the worst near-miss inside the
feasible region. The available targets are `minimize_ttc`, `minimize_distance`,
`maximize_severity`, and `maximize_ttc`; the reference table in
[yaml-reference.md](yaml-reference.md#optimization-target) lists what each one
means. Note the spelling: the YAML field uses snake_case (`minimize_ttc`), while
the equivalent CLI flag `--optimize` uses kebab-case (`min-ttc`). The two are
not interchangeable, so `optimization_target: min-ttc` is a parse error. For
what each target certifies about the trajectory it returns (an optimum is a
proven bound, not just the best of a sample) see [optimizer.md](optimizer.md).

## Common pitfalls

A few mistakes come up often enough to name directly.

**Direction that disagrees with the lane.** An actor's `direction` is not a free
choice; it must equal the direction of the lane it starts in. Putting an actor
in a forward lane with `direction: -1` is rejected. The mistake comes from
treating direction as a property of the actor rather than of the lane. Set `direction`
to match `lane_directions[lane]`, or move the actor to a lane that runs the way
you want.

**Unknown fields from typos.** Every structure in the schema rejects keys it
does not recognize. A misspelled `min_tcc` or a `walking_moad` in a behavior
block is a parse error, not a silently-ignored extra. This is deliberate: it
turns a typo that would otherwise leave a constraint at its default into an
immediate, named failure, but it means the fix for an "unknown field" error is
usually a spelling correction or a field that has drifted to the wrong nesting
level.

**The report-only acceleration fields.** The top-level `max_acceleration` and
`max_deceleration` are not solver constraints. They are checked after the fact
and reported; the solver is free to exceed them. If you want the solver to
respect an acceleration bound, use the actor's own `acceleration` range, which
is the real per-actor bound. The lateral counterpart, `max_lateral_acceleration`,
*is* a genuine solver constraint. It is
covered in full in [yaml-reference.md](yaml-reference.md#acceleration-fields-report-only-versus-solver-enforced).

**A cut-in that never meets.** For `cut_in_left` and `cut_in_right`, the NPC's
lane changes must end in the ego's lane. If they end elsewhere, the vehicles
never share a lane and the safety constraints are never evaluated for the pair –
the scenario is reported as satisfying constraints it was never tested against.
Validation catches this, but it is worth checking the geometry yourself when a
cut-in does not behave as expected.

## Where to go next

- [yaml-reference.md](yaml-reference.md): the exhaustive, field-by-field schema.
- [coordinate-systems.md](coordinate-systems.md): Cartesian versus bicycle, and
  the pedestrian sub-model.
- [adversarial-generation.md](adversarial-generation.md): constraint modes,
  violation, and the `--adversarial` flag in depth.
- [optimizer.md](optimizer.md): the optimization targets and what they certify.
- [creating-scenario-types.md](creating-scenario-types.md): adding a new
  scenario type in Rust, beyond what the YAML exposes.
