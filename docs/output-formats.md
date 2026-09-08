# Output Formats

← [Back to README](../README.md)

ScenarioWeaver produces the same **six formats** for every scenario it solves. All six are
exported from one solved `Scenario` value (see `src/scenario/model.rs`) after the LTL/Z3 pipeline
finishes – none of them re-derives its own data, so the JSON, the `.xosc`, and the SVG can never
disagree about what actually happened in a scenario. If you only need one format, generating all
six still costs nothing extra: the solve dominates the runtime, not the export.

For how a scenario spec turns into that `Scenario` value in the first place, see
[architecture.md](architecture.md); for the coordinate systems the trajectories are expressed in,
see [coordinate-systems.md](coordinate-systems.md).

## File Naming

**Single scenario mode** (`-o output/`):

```
output/scenario.json
output/scenario.xosc
output/scenario.xodr
output/scenario.svg
output/scenario.gif
output/scenario.ol.json
```

**Multiple scenario mode** (`-o scenarios/ -n 5`):

```
scenarios/scenario_0.json  scenario_0.xosc  scenario_0.xodr  scenario_0.svg  scenario_0.gif  scenario_0.ol.json
scenarios/scenario_1.json  scenario_1.xosc  scenario_1.xodr  scenario_1.svg  scenario_1.gif  scenario_1.ol.json
...
```

The `.xodr` is written before the `.xosc` in both modes, since the `.xosc`'s `RoadNetwork`
reference needs the sibling filename to already be settled.

---

## JSON (.json)

The canonical serialization of the `Scenario` model – the source of truth every other format is
derived from. It round-trips: `serde_json::from_str` on this file reconstructs exactly what the
solver produced, with nothing lost to a lossy export step.

```json
{
  "scenario_id": "uuid-here",
  "scenario_type": "cut_in_left",
  "time_step": 0.5,
  "duration": 10.0,
  "road": { "num_lanes": 2, "lane_width": 3.5, "lane_directions": [1, 1] },
  "actors": [
    {
      "id": "ego",
      "role": "ego",
      "states": [
        {
          "time": 0.0,
          "cartesian": {
            "position": { "x": 50.0, "y": 5.25 },
            "velocity": { "vx": 15.0, "vy": 0.0 },
            "acceleration": { "ax": 0.0, "ay": 0.0 },
            "lane": 1
          }
        }
      ]
    }
  ],
  "validation": {
    "min_ttc": 3.5,
    "min_distance": 8.2,
    "all_constraints_satisfied": true,
    "safety_violations": []
  }
}
```

- **Top level**: `scenario_id` (a UUID), `scenario_type`, `time_step`, `duration`, the `road` spec,
  the per-actor `actors` list, a `validation` block, and, only when the run used `--optimize`, an
  `optimization` block with the requested `target` and the `optimal_value` Z3 found.
- **Per actor**: `id`, `role` (`ego`, `npc`, or `pedestrian`), and `states`, one entry per time
  step from `0.0` to `duration`.
- **Per state**: `time`, plus a `cartesian` block with `position` (`x`, `y`), `velocity` (`vx`,
  `vy`), `acceleration` (`ax`, `ay`), and the current `lane` index. This is a flat 2D point-mass
  representation regardless of which coordinate system solved the scenario: a bicycle-model run's
  heading θ and steering angle δ are load-bearing during solving but are not exported here. Their
  effect shows up only through `x`, `y`, `vx`, and `vy`. See
  [coordinate-systems.md](coordinate-systems.md) for why.
- **`validation`**: `min_ttc` and `min_distance` are `Option<f64>`. `null` in the JSON means the
  metric was never evaluated (no pair of actors was ever in a comparable configuration), which is
  a distinct case from "very safe" and should not be read as one. `all_constraints_satisfied` and
  `safety_violations` (with timestamps) matter most for adversarial runs, where some violation is
  expected and the JSON is how you find exactly when it happened.

Use this format for post-processing, analysis pipelines, or feeding trajectories into your own
tools – it is the only format that carries every field the solver produced.

---

## OpenSCENARIO 1.3 (.xosc)

A runnable OpenSCENARIO 1.3 file (the revision is declared explicitly via
`ScenarioBuilder::with_revision`, not left to the `openscenario-rs` default), built with:

- One entity per actor: a `PassengerCar` for `ego`/`npc` roles, a `Pedestrian` for `role:
  pedestrian` – each typed correctly rather than represented as a vehicle.
- `InitAction`s setting each actor's starting position and speed.
- A storyboard with one `Act`/`Maneuver` per actor, each carrying a `FollowTrajectoryAction` built
  from that actor's full state sequence, plus a stop trigger keyed to the scenario's `duration`.
- A `RoadNetwork/LogicFile` reference to the sibling `.xodr` (via
  `export_to_xosc_with_road_file`, which is what `main.rs` calls) so the two files load together
  in a simulator.

Two limitations worth knowing about before you feed a `.xosc` into strict tooling: vehicle and
pedestrian bounding boxes are fixed constants (4.5m × 1.8m × 1.4m for vehicles, 0.6m × 0.6m × 1.8m
for pedestrians, matching `openscenario-rs`'s own presets) rather than per-actor dimensions – the
DSL does not yet carry per-actor sizing. As a direct consequence, `min_distance` in the validation
block is centre-to-centre, not bumper-to-bumper; a reported minimum distance of 5m does not mean 5m
of physical clearance.

**Programmatic export:**

```rust
use scenario_weaver::{generate_single_scenario, export_scenario_to_xosc};

let yaml = std::fs::read_to_string("scenario.yaml")?;
let scenario = generate_single_scenario(&yaml)?;
let xosc_xml = export_scenario_to_xosc(&scenario)?;
std::fs::write("scenario.xosc", xosc_xml)?;
```

To reference a specific road file (this is what the CLI does):

```rust
use scenario_weaver::export_scenario_to_xosc_with_road_file;
let xosc_xml = export_scenario_to_xosc_with_road_file(&scenario, "scenario.xodr")?;
```

---

## OpenDRIVE 1.7 (.xodr)

A single straight-road OpenDRIVE 1.7 network matching the YAML `road:` spec: lane count, lane
widths, per-lane travel directions, lane road marks, and a speed limit per lane. The road's `s=0`
does not always coincide with world `x=0` – if any actor's trajectory reaches negative `x` (a
backward-direction actor, typically), the road's start is pulled back far enough to cover the most
negative `x` any actor reaches, with padding, so every actor's position lands at a valid,
non-negative `s`. The `.xosc`'s `LanePosition`s are computed against this same offset, which is why
the two files are meant to be used together rather than independently.

The exporter also adds a `Sidewalk`-type lane strip on each side, wide enough to cover the actual
lateral excursion of any actor in the scenario (including a pedestrian who drifts past the road's
nominal edge) rather than a fixed width that a wide crossing could exceed.

**Programmatic export:**

```rust
use scenario_weaver::{generate_single_scenario, export_scenario_to_xodr};

let yaml = std::fs::read_to_string("scenario.yaml")?;
let scenario = generate_single_scenario(&yaml)?;
let xodr_xml = export_scenario_to_xodr(&scenario)?;
std::fs::write("scenario.xodr", xodr_xml)?;
```

---

## SVG (.svg)

A static, 1200×600 top-down vector rendering of the whole scenario:

- A metrics bar across the top with min TTC, min distance, and overall pass/fail status.
- The road surface with lane markings, drawn from the same `road` spec as the `.xodr`.
- Every actor's complete trajectory from `t=0` to `t=duration`, plus explicit start and end
  markers so direction of travel is legible without an animation.
- A legend identifying each actor.
- Violation markers where a safety constraint was breached (relevant mainly for adversarial runs).

Being plain SVG, it opens in any browser or image viewer and scales to any zoom level without
artifacts – a reasonable default for reports and documentation where a GIF would be overkill.

**Programmatic export:**

```rust
use scenario_weaver::{generate_single_scenario, export_scenario_to_svg};

let yaml = std::fs::read_to_string("scenario.yaml")?;
let scenario = generate_single_scenario(&yaml)?;
let svg = export_scenario_to_svg(&scenario)?;
std::fs::write("scenario.svg", svg)?;
```

---

## GIF (.gif)

An animated top-down rendering of the same scenario at 10 FPS (one frame per time step, 100ms
frame delay), looping indefinitely. Each frame shows:

- Every actor as a rectangle at its current position, with a heading arrow derived from its
  instantaneous velocity vector (the arrow disappears once an actor is nearly stationary, since a
  near-zero velocity has no meaningful heading).
- A live metrics overlay (current time, TTC, distance, and constraint status) updated frame by
  frame rather than fixed to the scenario's overall summary.
- Red highlighting on a violation frame, where applicable.

Resolution is selectable: `Resolution::High` (1200×600), `Resolution::Medium` (900×450, the
default used by both `export_to_gif` and the CLI), and `Resolution::Low` (600×300). A typical
10-second scenario at the default resolution runs close to 900KB, small enough to drop straight
into a browser, Slack, GitHub, or email without a hosting step.

**Programmatic export:**

```rust
use scenario_weaver::{generate_single_scenario, export_scenario_to_gif};

let yaml = std::fs::read_to_string("scenario.yaml")?;
let scenario = generate_single_scenario(&yaml)?;
let gif_bytes = export_scenario_to_gif(&scenario)?;
std::fs::write("scenario.gif", gif_bytes)?;
```

Custom resolution:

```rust
use scenario_weaver::{export_scenario_to_gif_with_resolution, Resolution};
let gif_bytes = export_scenario_to_gif_with_resolution(&scenario, Resolution::High)?;
```

Implementation notes, in case you are debugging the renderer itself: it is built on the `image`,
`gif`, `imageproc`, and `ab_glyph` crates, with an embedded font (`assets/DejaVuSans.ttf`) so text
rendering does not depend on fonts installed on the host.

---

## OpenLABEL 1.0.0 (.ol.json)

OpenLABEL 1.0.0 JSON describing the scenario as `objects` and `frames`, plus a set of semantic
tags – meant for cataloging, search, and filtering across a dataset of generated scenarios rather
than for re-deriving the trajectories themselves.

- **`objects`**: one entry per actor, typed `"vehicle"` or `"pedestrian"` by role, with a `name`
  that matches the actor's id exactly as it appears in the sibling `.xosc`'s `<ScenarioObject
  name="...">`. This shared name is what lets you join the `.ol.json` and the `.xosc` (and, through
  it, the `.json`) for the same scenario.
- **`frames`**: one entry per timestep, keyed by frame index, with each present actor's `(x, y)`
  position recorded under `object_data.vec` and a `frame_properties.timestamp`. Frames are built
  defensively against actors with mismatched state counts: the frame count is the max across
  actors, and a frame's timestamp is read from whichever actor has a state at that index first.
  Every bundled example advances all actors in lockstep, so in practice each frame carries every
  actor.
- **`tags`**: scenario category, lane-change direction, per-actor roles and behaviors, and lane
  count, all derived from data actually present in the trajectories.
- **`metadata`**: the real ASAM OpenLABEL 1.0.0 schema fields (`schema_version`, `file_version`,
  `annotator`, `comment`) sit at the top level. Everything else this project needs, such as
  scenario id, timestamps, and generator info, is namespaced under a `scenario_weaver` extension object
  (`metadata.scenario_weaver`) instead of being added as bare, non-schema keys, so a strict
  OpenLABEL consumer can ignore it cleanly.

Road-type tags (`RoadTypeMotorway`/`Distributor`/`Minor`) that used to be guessed from lane count
and directionality alone have been removed: they were never derived from a real classification,
and a wrong guess is worse than no tag at all. A travel-direction tag is emitted only when every
lane agrees on one direction; a bidirectional road gets none, on the same principle – an honest
absence of a tag beats a tag that might be lying.

**Programmatic export:**

```rust
use scenario_weaver::{generate_single_scenario, export_scenario_to_openlabel};

let yaml = std::fs::read_to_string("scenario.yaml")?;
let scenario = generate_single_scenario(&yaml)?;
let openlabel_json = export_scenario_to_openlabel(&scenario)?;
std::fs::write("scenario.ol.json", openlabel_json)?;
```

---

## Parsing a spec without re-reading YAML

If you already have a parsed `ScenarioSpec` (for example, one you built or modified
programmatically), you can generate directly from it and skip the YAML round-trip:

```rust
use scenario_weaver::generate_single_scenario_from_spec;
use scenario_weaver::dsl::parser::parse_yaml;

let yaml = std::fs::read_to_string("scenario.yaml")?;
let spec = parse_yaml(&yaml)?;
// Modify spec programmatically if needed...
let scenario = generate_single_scenario_from_spec(spec)?;
```
