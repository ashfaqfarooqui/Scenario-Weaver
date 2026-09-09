# YAML Reference

This document is the field-by-field reference for the YAML that ScenarioWeaver
reads. It describes every key the parser accepts, its type, whether it is
required, its default, and what it means. For a narrative walkthrough of how to
assemble these fields into a working scenario, see
[authoring-scenarios.md](authoring-scenarios.md); for the concepts behind the
coordinate systems, see [coordinate-systems.md](coordinate-systems.md).

A scenario file is a single YAML document that deserializes into the
`ScenarioSpec` structure. Every struct in the schema is declared with
`deny_unknown_fields`, so any key the parser does not recognize is a parse
error, not a silently-ignored extra. This is deliberate: a typo such as
`min_tcc` for `min_ttc` stops generation with a message naming the offending
key, rather than leaving the constraint at its default and producing a scenario
that quietly ignores what you asked for. The same discipline applies to the
constraint-mode block, the actor behavior map, and the value-or-range fields,
each of which rejects input it does not recognize.

## Value shapes

Two conventions recur throughout the schema and are defined here once.

**ValueOrRange.** Several numeric fields (an actor's `position`, `speed`, and
`acceleration`, and a lane change's `start_time` and `duration`) accept either
a scalar or a two-element `[min, max]` array. A scalar fixes the value: the
solver is handed an equality and must honor it exactly. An array gives the
solver a closed interval to choose from, and it will pick any value inside
`[min, max]` that satisfies every other constraint. Fixing a value and giving a
range are the two ways you control how much freedom the solver gets; a file
that fixes everything asks for one specific scenario, while a file built from
ranges asks for a family of them.

```yaml
speed: 15.0          # fixed: exactly 15 m/s
speed: [14.0, 16.0]  # range: solver chooses anywhere in [14, 16]
```

**deny_unknown_fields.** As above, unknown keys are rejected everywhere. When a
scenario fails to parse with a complaint about an unknown field, the cause is
almost always a misspelled key or a field placed at the wrong nesting level.

## Top-level: `ScenarioSpec`

The root of the document. The required fields have no default and must appear
in the file (or, for `road`, arrive through an import – see [Imports](#imports)).

| Key | Type | Required | Default | Meaning |
|---|---|---|---|---|
| `scenario_type` | enum | yes | n/a | Which behavioral model to generate. One of `cut_in_left`, `cut_in_right`, `overtake_left`, `pedestrian_crossing`, `head_on`. See [Scenario types](#scenario-types). |
| `time_step` | float (s) | yes | n/a | Discretization interval. Must be at least `0.001`, and `duration / time_step` must not exceed 100000 steps. Smaller steps give smoother trajectories at the cost of solve time. |
| `duration` | float (s) | yes | n/a | Total scenario length. Must be positive, at most `3600.0`, and at least `time_step`. |
| `actors` | list of `ActorSpec` | yes | n/a | Every actor in the scenario. Exactly one must have `role: ego`, and at least one NPC or pedestrian must be present. |
| `min_ttc` | float (s) | yes | n/a | Minimum time-to-collision threshold. Must be positive. How it is applied is governed by `constraint_modes.min_ttc`. |
| `min_distance` | float (m) | yes | n/a | Minimum longitudinal distance threshold. Must be positive. Governed by `constraint_modes.min_distance`. |
| `num_scenarios` | integer | yes | n/a | How many distinct scenarios to generate. Must be at least 1. Values above 1 use blocking clauses to force diversity between solutions. |
| `road` | `RoadSpec` | effectively yes | none | The road geometry. See [`RoadSpec`](#roadspec). Validation rejects a spec with no road, so this must be present either inline or via `imports`. |
| `coordinate_system` | enum | no | `cartesian` | Motion model: `cartesian` (2D point mass) or `bicycle` (kinematic bicycle with heading and steering). See [coordinate-systems.md](coordinate-systems.md). |
| `constraint_modes` | shorthand or mapping | no | `enforce_all` | How each safety constraint is treated. See [Constraint modes](#constraint-modes). |
| `optimization_target` | enum | no | `none` | When set, the solver searches for an optimal scenario rather than any satisfying one. See [Optimization target](#optimization-target). |
| `max_lateral_acceleration` | float (m/s²) | no | `2.0` | Comfort bound on lateral acceleration during lane changes. This is a genuine solver constraint. Must be positive. |
| `max_velocity` | float (m/s) | no | none | Optional upper speed bound. Must be positive if set. Governed by `constraint_modes.max_velocity`. |
| `min_velocity` | float (m/s) | no | none | Optional lower speed bound. Must be non-negative if set. Governed by `constraint_modes.min_velocity`. |
| `min_lateral_distance` | float (m) | no | none | Optional lateral separation bound. Must be positive if set. Governed by `constraint_modes.min_lateral_distance`. **Means something different on a pedestrian crossing** — see the note below. |
| `max_relative_velocity` | float (m/s) | no | none | Optional closing-speed bound. Must be positive if set. Governed by `constraint_modes.max_relative_velocity`. |
| `max_acceleration` | float (m/s²) | no | none | **Report-only.** Must be positive if set. See the caveat below. |
| `max_deceleration` | float (m/s²) | no | none | **Report-only.** Must be negative if set. See the caveat below. |
| `bicycle_config` | `BicycleConfig` | no | none | Scenario-level default bicycle parameters. Only valid when `coordinate_system: bicycle`; rejected otherwise. See [`BicycleConfig`](#bicycleconfig). |
| `lane_width` | float (m) | no | `3.5` | **Deprecated.** Use `road.lane_width` instead. Read only as a fallback when no `road` is given, and validation requires a road, so this field is effectively dead. |

### `min_lateral_distance` on a pedestrian crossing

For every scenario type except `pedestrian_crossing`, `min_lateral_distance` is
an unguarded floor: `|py1 - py2| >= min_lateral_distance` for every pair, at
every step. The actors are lane-following, so they never traverse each other's
lateral position and the bound is meaningful as written.

A crossing pedestrian is different. Its whole purpose is to walk *through* the
ego's lateral position, so the unguarded reading is unsatisfiable for any
`min_lateral_distance` larger than half the pedestrian's per-step lateral hop —
the constraint would hold only because the trajectory is sampled, and halving
`time_step` would turn an unchanged spec `Unsatisfiable`. Lateral separation
from a crossing pedestrian is a safety property only *while the two are
longitudinally close*.

So on a `pedestrian_crossing` the field is guarded on longitudinal relevance:

```text
at every step:   |dx| <= W   =>   |dy| >= min_lateral_distance
```

where `dx`/`dy` are the ego-pedestrian longitudinal/lateral separations and `W`
is the **longitudinal relevance window**

```text
W = max( min_ttc * (the ego's declared top speed),  min_distance / 2 )
```

— the distance the ego can cover inside its own stated time-to-collision
budget, floored at the pedestrian safety box's own longitudinal half-width so
the two constraints never disagree about what "longitudinally close" means. `W`
is derived from fields you already set; there is no separate key for it.

What this means in practice:

- **A physically meaningful value now works.** `min_lateral_distance: 2.0` on
  `examples/pedestrian_wide_road.yaml` (`W = 18.0 m`) is satisfiable: the
  pedestrian crosses the ego's lane while the ego is still more than 18 m away,
  or waits at the kerb until it has passed.
- **`enforce`** requires the clearance only inside the window. Outside it the
  pedestrian may pass straight through `py_ego`; that is the approach phase,
  where lateral separation carries no safety meaning.
- **`violate`** asks for the negation, `|dx| < W AND |dy| < min_lateral_distance`
  at some step — a genuine near miss, in which the ego must actually *be* inside
  the window. It cannot be satisfied by keeping the pedestrian on the kerb while
  the ego is far away.
- **Raising `min_ttc` widens the window**, even when `constraint_modes.min_ttc`
  is `ignore`. A window wide enough to cover the whole scenario collapses the
  guard back into the unguarded bound, and the spec becomes unsatisfiable for
  any realistic clearance.

`compute_validation_metrics` measures exactly this guarded property, so a
generated scenario's `all_constraints_satisfied` reflects what the solver was
asked for rather than the unguarded reading.

### Acceleration fields: report-only versus solver-enforced

Three fields carry the word "acceleration" at the top level, and they do not
all mean the same thing. They differ in whether the solver sees them at all, so
the distinction is set out here.

- `max_acceleration` and `max_deceleration` are **report-only**. They are not
  handed to the solver. After a scenario is generated, an independent
  validation pass compares the trajectory's acceleration against these bounds
  and records any excursion in the report. The solver is free to produce a
  scenario that exceeds them; you will simply be told that it did. If you want
  the solver to respect an acceleration bound, that is not the field to use.
- The **actual per-actor acceleration bound** is the actor's own
  `acceleration` range (see [`ActorSpec`](#actorspec)). Writing
  `acceleration: [-8.0, 3.0]` on an actor constrains that actor's longitudinal
  acceleration to `[-8, 3] m/s²` inside the solver, and the solver honors it.
- `max_lateral_acceleration` **is** a genuine solver constraint. It bounds the
  lateral acceleration during lane changes and defaults to `2.0 m/s²`.

In short: use the actor's `acceleration` range for a real longitudinal bound,
`max_lateral_acceleration` for a real lateral bound, and treat the top-level
`max_acceleration`/`max_deceleration` pair as post-hoc reporting only.

## `RoadSpec`

A single straight road with a fixed number of lanes, each carrying traffic in
one direction. Nested under the top-level `road` key.

| Key | Type | Required | Default | Meaning |
|---|---|---|---|---|
| `num_lanes` | integer | yes | n/a | Total number of lanes across both directions. Must be at least 1. |
| `lane_width` | float (m) | yes | n/a | Width of every lane. Must be within `[1.0, 20.0]`. |
| `lane_directions` | list of int | no | `[1, 1]` | One entry per lane: `+1` for forward (+x), `-1` for backward (-x). Its length must equal `num_lanes`. See the rule below. |
| `road_length` | float (m) | no | auto | Length of the road. If omitted, it is computed as `duration * 30.0` (assuming a 30 m/s ceiling). Must be positive if set. |

**The `lane_directions` rule.** The list must have exactly `num_lanes` entries,
each `+1` or `-1`, and all forward lanes must come before all backward lanes –
a single forward block followed by a single backward block. `[1, 1, -1, -1]`
is valid; `[-1, -1, 1, 1]` and any interleaving such as `[1, -1, 1, -1]` are
rejected. This is not an arbitrary restriction: the OpenDRIVE exporter maps a
lane index to a physical position independently of direction, and splits
forward and backward lanes onto opposite sides of the road by index. An
interleaved layout would place a lane a full lane-width away from where its
trajectories actually drive, so the parser rejects it rather than emit a road
that disagrees with the motion on it.

The default `[1, 1]` describes two forward lanes. If your road has any other
lane count, you must set `lane_directions` explicitly – an omitted list on a
three-lane road is a length mismatch and fails validation.

## `ActorSpec`

One entry in the top-level `actors` list. Every actor's position and speed can
be fixed or given as a range; ranges let the solver choose concrete values that
satisfy the scenario's constraints.

| Key | Type | Required | Default | Meaning |
|---|---|---|---|---|
| `id` | string | yes | n/a | Unique identifier for the actor. Duplicate IDs are rejected. |
| `role` | enum | yes | n/a | `ego`, `npc`, or `pedestrian`. Exactly one `ego` is required; pedestrians use a simplified point-mass model with no steering. |
| `lane` | integer | yes | n/a | Starting lane index, `0`-based. Must be less than `num_lanes`. |
| `position` | ValueOrRange (m) | yes | n/a | Initial longitudinal position along the road. A range gives the solver a spawn window. |
| `speed` | ValueOrRange (m/s) | yes | n/a | Initial speed. Must be non-negative. |
| `acceleration` | ValueOrRange (m/s²) | yes | n/a | Longitudinal acceleration bound for this actor. This is the real per-actor acceleration constraint the solver enforces. |
| `direction` | int | yes | n/a | `+1` forward or `-1` backward. Must equal `lane_directions[lane]`; see the rule below. |
| `behavior` | map | no | `{}` | Scenario-specific behavior parameters. Only three keys are accepted; see [Behavior keys](#behavior-keys). |
| `lane_changes` | list of `LaneChangeConfig` | no | `[]` | Zero or more lane changes, applied in order. Presence in the list means enabled, with no separate on/off flag. |
| `bicycle_params` | `BicycleParams` | no | none | Per-actor bicycle parameters, overriding `bicycle_config`. Only valid under `coordinate_system: bicycle`; rejected otherwise. See [`BicycleParams`](#bicycleparams). |

**Direction must agree with the lane.** An actor's `direction` is not chosen
independently – it is determined by the lane it occupies. Validation rejects a
spec whose `actor.direction` differs from `lane_directions[actor.lane]`. An NPC
declaring `direction: -1` while sitting in a `+1` lane is an error, not a
silent correction. Set `direction` to match the lane, or move the actor to a
lane whose direction you want.

### Behavior keys

The `behavior` map is untyped, so it is validated against a small allowlist
rather than a per-scenario-type schema. Only three keys are accepted; any other
key is a parse error, which turns a typo such as `walking_moad` into an
immediate failure rather than a silently-defaulted behavior. Each accepted key
is range-checked too, so a well-spelled key holding a nonsense value is also an
error rather than a silent default.

| Key | Values | Meaning |
|---|---|---|
| `walking_mode` | `walk`, `run`, `hesitate` | Pedestrian gait. `walk` caps speed at 2.0 m/s, `run` at 5.0 m/s, `hesitate` introduces pauses during the crossing. |
| `direction` | `left_to_right`, `right_to_left` | Pedestrian crossing direction. |
| `speed_retention` | number in `(0, 1]` | Vehicles only. Hold the declared `speed:` band for the whole horizon instead of only at `t = 0`. |

The first two keys are meaningful only for pedestrian actors; `speed_retention`
is rejected on a pedestrian. For vehicles, the behavioral pattern otherwise
comes from `scenario_type` and `lane_changes`, not from `behavior`.

#### `speed_retention`

`speed:` is an **initial condition**: it pins the actor's speed at `t = 0` and
nothing else refers to it again. Between the first and last step an actor's
speed is bounded only by its own `acceleration:` band, by the net-displacement
floor, and by the terminal-step floor (see
[`docs/z3_constraints.md`](z3_constraints.md)), so the solver is free to return
a vehicle that drifts far outside the band it declared — an oncoming car that
sheds speed to 41% of its declared minimum, or a "slow" vehicle that ends up
69% above its declared maximum.

`speed_retention: f` says the band means *hold this*, for this actor only. At
every step the actor's along-track speed must satisfy

```text
f * speed.min()  <=  direction * v_long[t]  <=  speed.max() / f
```

`f = 1.0` is the declared band exactly. Smaller values widen it by the same
relative factor on each side: `speed_retention: 0.8` on `speed: [10.0, 12.0]`
admits `[8.0, 15.0]`.

It is **off by default and opt-in per actor**, deliberately. A speed floor on
every actor at every step would forbid braking hard — to a standstill, briefly —
for a pedestrian in the road, which is legitimate driving and the point of an
entire scenario type. Per actor, you state which vehicles are background traffic
holding a speed and which is the vehicle under test that may do anything. Both
head-on examples set it on `slow_npc` and `oncoming_npc` and leave the ego free.

Both bounds are compile-time constants against a single solver variable, so this
stays in QF_LRA. Note that it can make a spec unsatisfiable, exactly as intended:
if the retained band contradicts what the scenario's safety constraints require,
the generator reports UNSAT rather than quietly returning a vehicle that ignores
its declared speed.

## `LaneChangeConfig`

One entry in an actor's `lane_changes` list. Multiple entries describe
sequential lane changes, each applied to the lane the actor reached after the
previous one.

| Key | Type | Required | Default | Meaning |
|---|---|---|---|---|
| `direction` | enum | yes | n/a | `left` or `right`, relative to the actor's own direction of travel. See the note below. |
| `start_time` | ValueOrRange (s) | yes | n/a | When the change begins. A range lets the solver schedule it; the encoder uses the midpoint of the window. Must be non-negative and start before the horizon. |
| `duration` | ValueOrRange (s) | yes | n/a | How long the change takes. Must be positive and at least `time_step`, otherwise it spans no simulated step and would be discarded. |

**Left and right are actor-relative.** `right` moves one lane in the direction
of the actor's own `direction`; `left` moves one lane against it. For a
forward-travelling actor (`direction: 1`) this matches the road frame – `left`
lowers the lane index and `right` raises it. For a backward-travelling actor
the two are mirrored in the road frame. The solver discovers the actual
trajectory of each change dynamically under smoothness constraints; you specify
only the direction, the window, and the duration.

A lane change must stay on the road. A change whose target lane index falls
outside `0 .. num_lanes` is rejected. For `cut_in_left`/`cut_in_right`
scenarios, an NPC's lane changes must also end in the ego's lane – otherwise
the two never share a lane, no conflict is asserted, and `min_ttc`/`min_distance`
are never evaluated for the pair. Validation catches this and tells you which
lane the actors end up in.

## `BicycleParams`

Per-actor kinematic bicycle parameters, given under an actor's `bicycle_params`
key. Only meaningful when `coordinate_system: bicycle`. All three fields are
required when the struct is present.

| Key | Type | Required | Default | Meaning |
|---|---|---|---|---|
| `wheelbase` | float (m) | yes | n/a | Distance between the front and rear axles. Must be positive. |
| `max_steering_angle` | float (rad) | yes | n/a | Maximum steering angle at the front wheels. Must be positive and below π/2. |
| `max_steering_rate` | float (rad/s) | yes | n/a | Maximum rate of change of the steering angle. Must be positive. |

The minimum turn radius is the exact kinematic relation
`wheelbase / tan(max_steering_angle)`, so these parameters directly bound how
tightly the vehicle can turn.

## `BicycleConfig`

Scenario-level default bicycle parameters, given under the top-level
`bicycle_config` key. Any actor without its own `bicycle_params` inherits these.
Valid only when `coordinate_system: bicycle`, and rejected otherwise. All three
fields are required when the struct is present.

| Key | Type | Required | Default | Meaning |
|---|---|---|---|---|
| `default_wheelbase` | float (m) | yes | n/a | Wheelbase for actors without their own `bicycle_params`. Must be positive. |
| `default_max_steering_angle` | float (rad) | yes | n/a | Maximum steering angle for such actors. Must be positive and below π/2. |
| `default_max_steering_rate` | float (rad/s) | yes | n/a | Maximum steering rate for such actors. Must be positive. |

Under the bicycle model, every non-pedestrian actor must have parameters from
one source or the other. A vehicle with neither its own `bicycle_params` nor a
scenario-level `bicycle_config` is a validation error.

## Scenario types

The `scenario_type` field selects one of five behavioral models. Each generates
its own LTL and validation rules.

| Value | Meaning |
|---|---|
| `cut_in_left` | An NPC in a left lane changes into the ego's lane ahead of it. |
| `cut_in_right` | An NPC in a right lane changes into the ego's lane ahead of it. |
| `overtake_left` | The ego overtakes a slower vehicle via the left lane, using two sequential lane changes. |
| `pedestrian_crossing` | A pedestrian crosses the road while the ego approaches. |
| `head_on` | The ego overtakes a slow vehicle on a bidirectional road, entering the oncoming lane. |

To add a scenario type of your own, see
[creating-scenario-types.md](creating-scenario-types.md).

## Constraint modes

`constraint_modes` controls how each safety constraint is treated during
generation. There are three modes:

- **enforce**: the constraint must hold at all times (`G(constraint)`).
- **violate**: the constraint must be broken at some point (`F(¬constraint)`).
  This is how adversarial, near-miss scenarios are generated.
- **ignore**: the constraint is not added to the formula at all.

The field takes one of two forms. The **shorthand** form is a single string
that applies one mode to every constraint:

```yaml
constraint_modes: violate_all   # or enforce_all, or ignore_all
```

The **per-constraint** form is a mapping that sets each of the seven
controllable constraints independently. Any constraint omitted from the mapping
defaults to `enforce`.

```yaml
constraint_modes:
  min_ttc: violate
  min_distance: enforce
```

The seven controllable constraints are:

| Key | Governs |
|---|---|
| `min_ttc` | the `min_ttc` threshold |
| `min_distance` | the `min_distance` threshold |
| `max_acceleration` | the acceleration bound |
| `max_velocity` | the optional `max_velocity` bound |
| `min_velocity` | the optional `min_velocity` bound |
| `min_lateral_distance` | the optional `min_lateral_distance` bound |
| `max_relative_velocity` | the optional `max_relative_velocity` bound |

A mode keyed to an optional scalar that is not set is inert. Four of these
constraints (`max_velocity`, `min_velocity`, `min_lateral_distance`, and
`max_relative_velocity`) correspond to top-level fields that default to unset.
Setting `min_velocity: violate` in `constraint_modes` does nothing unless you
also give a `min_velocity` value for it to act on. The default when the whole
block is omitted is `enforce_all`.

The shorthand form is validated strictly: a typo such as `violate-all` (with a
hyphen) is a parse error naming the valid values, not a silently-accepted
string. The per-constraint form rejects unknown keys the same way.

For a fuller treatment of adversarial generation and the `--adversarial` flag,
see [adversarial-generation.md](adversarial-generation.md).

## Optimization target

By default the solver returns any scenario that satisfies the constraints. Set
`optimization_target` and it instead searches for an optimal one, using Z3's
optimizing solver.

| YAML value | Meaning |
|---|---|
| `none` | No optimization (default). Any satisfying scenario. |
| `minimize_ttc` | The worst near-miss the constraints allow: the smallest same-lane time-to-collision. |
| `minimize_distance` | The closest approach: the smallest same-lane longitudinal gap. |
| `maximize_severity` | The most severe interaction: the highest same-lane closing speed. |
| `maximize_ttc` | The safest scenario: the largest same-lane time-to-collision. |

Note the spelling. The YAML field uses **snake_case** (`minimize_ttc`,
`maximize_severity`, …), while the CLI's `--optimize` flag uses **kebab-case**
(`min-ttc`, `max-severity`, …) for the same set. The two are not
interchangeable: `optimization_target: min-ttc` in a YAML file is a parse
error. See [optimizer.md](optimizer.md) for what each target certifies about
the trajectory it ships.

## Imports

A scenario file may pull its road definition from another file, which lets a
library of roads be reused across scenarios. The top-level `imports` key is a
list of paths, each resolved relative to the importing file's own directory.

```yaml
imports:
  - ../roads/4_lane_bidirectional.yaml

scenario_type: cut_in_left
# ... the rest of the scenario, with no inline `road` block
```

Imports are preprocessed before the document is parsed. Only the road
specification is merged, and only when the main document has no `road` of its
own – an inline `road` always wins over an imported one. An imported file may
contain either a nested `road:` block or a flat road spec (a document whose top
level is `num_lanes`, `lane_width`, and `lane_directions`); both are recognized.
The `imports` key itself is stripped before parsing, so it does not collide with
`deny_unknown_fields`.

This is the one mechanism by which the effectively-required `road` may be absent
from the main document: it can arrive entirely through an import.
