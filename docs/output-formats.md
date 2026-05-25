# Output Formats

← [Back to README](../README.md)

ScenarioWeaver automatically produces scenarios in **six formats** for every run.

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
scenarios/scenario_0.json  scenario_0.xosc  scenario_0.xodr  ...
scenarios/scenario_1.json  scenario_1.xosc  scenario_1.xodr  ...
```

---

## JSON (.json)

Complete actor trajectories with validation metrics.

```json
{
  "scenario_id": "uuid-here",
  "scenario_type": "cut_in_left",
  "time_step": 0.5,
  "duration": 10.0,
  "actors": [
    {
      "id": "ego",
      "role": "ego",
      "states": [
        {
          "time": 0.0,
          "position": { "x": 50.0, "y": 5.25 },
          "velocity": { "vx": 15.0, "vy": 0.0 },
          "lane": 1
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

---

## OpenSCENARIO (.xosc)

Valid OpenSCENARIO 1.0+ XML for simulator compatibility.

- File header with scenario metadata
- Vehicle entities for all actors
- Trajectory data embedded in description field
- Compatible with CARLA and other OpenSCENARIO-compliant simulators

**Programmatic export:**

```rust
use scenario_weaver::{generate_single_scenario, export_scenario_to_xosc};

let yaml = std::fs::read_to_string("scenario.yaml")?;
let scenario = generate_single_scenario(&yaml)?;
let xosc_xml = export_scenario_to_xosc(&scenario)?;
std::fs::write("scenario.xosc", xosc_xml)?;
```

To include a road file reference:

```rust
use scenario_weaver::export_scenario_to_xosc_with_road_file;
let xosc_xml = export_scenario_to_xosc_with_road_file(&scenario, "scenario.xodr")?;
```

> **Note:** Pedestrians are exported as vehicles due to a limitation in the openscenario-rs library.

---

## OpenDRIVE (.xodr)

OpenDRIVE 1.7 road network XML describing the scenario's road geometry.

- Single straight road with lane definitions
- Lane widths and directions matching the YAML `road` spec
- Compatible with OpenSCENARIO-based simulators that require a road network file

**Programmatic export:**

```rust
use scenario_weaver::{generate_single_scenario, export_scenario_to_xodr};

let yaml = std::fs::read_to_string("scenario.yaml")?;
let scenario = generate_single_scenario(&yaml)?;
let xodr_xml = export_scenario_to_xodr(&scenario)?;
std::fs::write("scenario.xodr", xodr_xml)?;
```

---

## SVG Visualization (.svg)

Static vector graphic showing the complete scenario trajectory.

- Road layout with lane markings
- Complete trajectories for all actors from start to end
- Vehicle positions at initial and final states
- Safety metrics in header (Min TTC, Min Distance, Status)
- Violation markers if safety constraints were violated
- Opens in any web browser or image viewer; scales to any zoom level

**Programmatic export:**

```rust
use scenario_weaver::{generate_single_scenario, export_scenario_to_svg};

let yaml = std::fs::read_to_string("scenario.yaml")?;
let scenario = generate_single_scenario(&yaml)?;
let svg = export_scenario_to_svg(&scenario)?;
std::fs::write("scenario.svg", svg)?;
```

---

## GIF Animation (.gif)

Animated visualization showing vehicles moving through the scenario.

- 10 FPS animation showing trajectory evolution over time
- Fading trajectory trails showing motion history
- Real-time metrics overlay (current time, TTC, distance, status)
- Violation highlighting with red circles
- Road surface with lane markings and vehicle rectangles with heading arrows
- Infinite loop playback; ~900 KB for a typical 10-second scenario

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

Available variants: `Resolution::High` (1200×600), `Resolution::Medium` (900×450), `Resolution::Low` (600×300).

**Implementation notes:**
- Uses `image`, `gif`, `imageproc`, and `ab_glyph` crates
- Embedded font: `assets/DejaVuSans.ttf`
- One frame per time step; frame delay 100 ms

---

## OpenLabel (.ol.json)

OpenLabel 1.0.0 JSON metadata describing scenario semantics.

- Scenario metadata (type, duration, time step)
- Semantic tags: road type, scenario category, actor roles, behaviors
- Frame-level object data with positions and velocities
- Validation metadata (TTC, distance, constraint satisfaction)
- Useful for scenario cataloging, search, and filtering

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

To avoid a YAML round-trip when generating programmatically:

```rust
use scenario_weaver::generate_single_scenario_from_spec;
use scenario_weaver::dsl::parser::parse_yaml;

let yaml = std::fs::read_to_string("scenario.yaml")?;
let spec = parse_yaml(&yaml)?;
// Modify spec programmatically if needed...
let scenario = generate_single_scenario_from_spec(spec)?;
```
