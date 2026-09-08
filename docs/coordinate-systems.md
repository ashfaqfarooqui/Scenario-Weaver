# Coordinate Systems

← [Back to docs](./README.md)

Every actor in a scenario moves according to one of two motion models: a
**Cartesian** point mass or a **kinematic bicycle** model. The choice is
scenario-wide and is made with a single YAML field, `coordinate_system`. This
document explains what each model can and cannot express, how to configure them,
and the one place where the bicycle model trades exact dynamics for a linear
encoding the solver can decide.

Pedestrians are a special case that sits underneath both models. They are covered
in [Pedestrians](#pedestrians) below.

For where the encoders fit in the generation pipeline, see
[architecture.md](./architecture.md); for the full YAML field list, see
[yaml-reference.md](./yaml-reference.md).

---

## The two models at a glance

| | Cartesian (default) | Bicycle |
|---|---|---|
| Internal state per actor per step | `x`, `y`, `vx`, `vy`, `ax`, `ay`, `lane` | the Cartesian state **plus** a real heading `θ` and a steering angle `δ` |
| Motion model | 2D point mass, with longitudinal and lateral motion independent | kinematic bicycle, where lateral motion is driven by heading and heading by steering |
| Steering / turn radius | none: no steering angle, no curvature, no turn-radius limit | a real steering angle `δ`, a steering-rate limit, and a genuine minimum turn radius from wheelbase and maximum steering angle |
| Heading | not modeled; the actor is a point with a velocity vector | tracked explicitly as `θ` and coupled to lateral motion |
| Extra configuration | none | requires `bicycle_config` or per-actor `bicycle_params` |
| Solver cost | lower | higher, since the heading/steering coupling adds variables and a case split on speed |
| Exported trajectory fields | `x`, `y`, `vx`, `vy`, `ax`, `ay`, `lane` | **the same** `x`, `y`, `vx`, `vy`, `ax`, `ay`, `lane` |

The final row is the important one. Both models export the identical set of
fields. The heading `θ` and steering angle `δ` are internal to the bicycle
encoder and are **not** part of the output; there is no heading column. Their
effect appears instead in the exported fields: a bicycle-model actor's `x`, `y`,
`vx`, and `vy` are the trajectory that a bounded heading and a limited steering
rate produce. The output format is documented in
[output-formats.md](./output-formats.md).

In practice, Cartesian can express *where* an actor is and *how fast* it moves in
each axis, but nothing about *how* a vehicle would have to steer to get there. A
Cartesian lane change is a lateral velocity that respects a lateral-speed cap; it
carries no notion of a turning circle. The bicycle model adds exactly that notion.
If a maneuver would require a passenger car to turn inside its own minimum radius,
the bicycle model rejects it and the Cartesian model does not.

---

## Choosing a model

Use **Cartesian** unless you have a specific reason not to. It is the default, it
needs no extra parameters, and it solves faster. For common scenarios such as
cut-ins, overtakes, following, and pedestrian crossings, a point mass with a
lateral-speed cap is an adequate model of where the vehicles are and how they
interact.

Use the **bicycle** model when the plausibility of the *maneuver itself* matters:
when you want steering realism, when a scenario hinges on whether a vehicle can
physically make a turn at a given speed, or when you are exercising a system under
test that is sensitive to turn-radius-feasible trajectories. The cost is real but
modest, a little extra YAML and more solver work, and in return the trajectories
are ones a real vehicle with the declared wheelbase and steering lock could drive.

The two models are close but not identical for the same YAML. Both route a lane
change through the same lateral-speed envelope, so neither one permits gross
motion the other forbids. The difference is that the bicycle model additionally
ties that lateral motion to a bounded heading and a limited steering rate, so its
lane changes are smoother and its turns respect a minimum radius.

---

## Selecting a model in YAML

The model is chosen once, at the top level of the scenario:

```yaml
coordinate_system: cartesian   # the default; may be omitted entirely
```

```yaml
coordinate_system: bicycle
```

Cartesian needs nothing more. The bicycle model needs vehicle geometry, supplied
in one of two ways.

**Scenario-level defaults** apply to every vehicle that does not override them:

```yaml
coordinate_system: bicycle

bicycle_config:
  default_wheelbase: 2.7            # meters, front-to-rear axle (typical sedan)
  default_max_steering_angle: 0.6   # radians (~34°)
  default_max_steering_rate: 0.5    # radians per second
```

**Per-actor parameters** override the defaults for a single vehicle:

```yaml
actors:
  - id: npc
    role: npc
    # ...
    bicycle_params:
      wheelbase: 2.9            # a larger vehicle
      max_steering_angle: 0.5   # less maneuverable
      max_steering_rate: 0.4    # slower steering
```

The wheelbase and the maximum steering angle together fix the **minimum turn
radius**, `R = wheelbase / tan(max_steering_angle)`. With the defaults above that
is `2.7 / tan(0.6) ≈ 3.95 m`, the tightest circle that vehicle is allowed to
turn, at any speed.

Validation is strict and symmetric. A bicycle scenario must supply geometry: if
`coordinate_system: bicycle` is set and an actor has neither a scenario-level
`bicycle_config` default nor its own `bicycle_params`, the parser reports an error
naming that actor. The reverse is rejected too. A `bicycle_config` or
`bicycle_params` block on a scenario that is not `bicycle` is an error, rather
than a silently ignored block. See [yaml-reference.md](./yaml-reference.md) for
the full field list and [authoring-scenarios.md](./authoring-scenarios.md) for
worked examples.

Working bicycle-model scenarios ship with the tool:

- `examples/bicycle_lane_change.yaml` is a highway cut-in driven by bicycle dynamics
- `examples/cut_in_right_bicycle.yaml` is the mirror-image cut-in on a three-lane road
- `examples/head_on_near_miss_bicycle.yaml` is a head-on near miss with steering realism

---

## The linearization and its error bounds

The bicycle model's heading and steering are real and load-bearing. The heading
drives lateral motion, the steering drives the heading, and the steering rate and
turn radius are enforced against them. Remove the heading and the turn-radius
guarantee goes with it. The model is accurate up to a single, small, documented
approximation of the dynamics, described below.

The exact kinematic bicycle model is nonlinear. Its lateral velocity is
`v · sin(θ)` and its turn rate is `(v / L) · tan(δ)`, each a product of a variable
speed with a trigonometric function of a variable angle. ScenarioWeaver generates
scenarios by handing the whole problem to a solver that works in **linear real
arithmetic**: it decides systems of linear constraints exactly, but it has no
decision procedure for the variable-times-variable products those two equations
contain. To keep the model inside that linear world, the encoder makes two
approximations, and only two.

**A small-angle approximation.** For small angles, `sin(θ) ≈ θ` and
`tan(δ) ≈ δ`, which removes the trigonometry. This holds only while the angles
stay small, so the encoder bounds the heading to about **8.5°**. At that bound the
approximation error on lateral velocity is at most about **0.37%**, and in a
typical highway lane change the steering angle is far smaller still, where the
error is smaller again. The bound is a genuine restriction: a scenario that needs
a sharper heading than 8.5° is outside this encoder's scope. Within the bound, the
heading is an accurate stand-in for the real thing.

**A constant reference speed.** Removing the trigonometry still leaves a speed
multiplying an angle, a variable times a variable. The encoder replaces the
variable speed in those two couplings with a **constant** reference speed, so that
a constant times a variable stays linear. It does this without pretending the
actor moves at one fixed speed. Each actor's reachable speed range is partitioned
into **buckets**, 5 m/s wide and at most eight per actor, and the solver is free
to place the actor in whichever bucket its true speed falls into at each step,
using that bucket's midpoint as the reference. The approximation error is
therefore at most half a bucket's worth of speed, and it is **exactly zero** for
any actor whose whole reachable speed span fits inside a single bucket.

The result is an accurate *kinematic* bicycle model, exact up to a small, bounded,
and documented approximation of the dynamics. The approximation is of the
*dynamics*, not of the exported trajectory's internal consistency: within a single
scenario the positions, velocities, and accelerations agree with each other,
because the lateral position is integrated from the same lateral velocity the
heading defines. The exported numbers are the solver's own, mutually consistent,
and drivable by a vehicle with the geometry you declared.

---

## Pedestrians

A pedestrian is **not** a third coordinate system. Pedestrians share one 2D
point-mass sub-model that both the Cartesian and bicycle encoders delegate to for
any actor with `role: pedestrian`. Whichever coordinate system a scenario
declares, its pedestrians move the same way.

The pedestrian model is deliberately simple. It has no heading, no steering, and
no lane-following: a pedestrian is a point that can move in any direction across
the road. Its speed is capped by a walking mode, selected with
`behavior.walking_mode`, walking by default or running when set to `run`. A
pedestrian may also begin the scenario already in motion across the road, rather
than starting from the curb, which lets you author a crossing that is already
underway at the first time step.

The speed cap is enforced as an **octagon** rather than a disk. A true circular
speed limit is nonlinear and would take the problem out of linear real arithmetic;
a plain box would be too loose, letting a pedestrian move faster along a diagonal
than straight across. The octagon is the linear compromise that keeps the cap
close to circular in every direction, including the perpendicular-crossing
direction typical of a pedestrian scenario.

One implementation detail is useful to know, because it explains why the two
coordinate systems agree so closely: the Cartesian encoder reuses this same
point-mass integration for its **vehicles** as well. In the Cartesian world every
actor, vehicle or pedestrian, is a 2D point mass integrated by one shared step.
The bicycle encoder is the only one that adds heading and steering on top, and
does so only for vehicles.

---

## See also

- [architecture.md](./architecture.md) for where the encoders sit in the pipeline
- [yaml-reference.md](./yaml-reference.md) for every YAML field, including `coordinate_system`, `bicycle_config`, and `bicycle_params`
- [authoring-scenarios.md](./authoring-scenarios.md) for writing a scenario end to end
- [output-formats.md](./output-formats.md) for what the exported trajectory contains
