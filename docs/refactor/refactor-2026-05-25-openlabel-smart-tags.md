---
created: 2026-05-25
file: src/scenario/openlabel_exporter.rs
---

# Refactor: openlabel-smart-tags

## Before

`build_tags` emitted several tags unconditionally regardless of the actual scenario content:

- `VehicleCar` was always added, even for pedestrian-only scenarios
- `RoadTypeMotorway` was always added regardless of lane count or road directionality
- No road travel direction tags were emitted despite bidirectional road support
- No `MotionAccelerate` / `MotionDecelerate` tags despite trajectory ax data being available
- No `SpecialStructurePedestrianCrossing` or `ZoneSchool` tags despite scenario type being available

## Changes

- **`VehicleCar`** — now only emitted when at least one actor with `role != "pedestrian"` exists
- **Road type** — inferred from `scenario.road`: `RoadTypeMotorway` (≥3 lanes, unidirectional), `RoadTypeDistributor` (2 lanes, unidirectional), `RoadTypeMinor` (1 lane or bidirectional); exactly one tag emitted
- **Travel direction** — always emits `LaneSpecificationTravelDirection`; emits `TravelDirectionRight` if any lane has direction `+1`, `TravelDirectionLeft` if any lane has direction `-1`
- **`MotionAccelerate`** — emitted when any actor state has `ax > 0.5`
- **`MotionDecelerate`** — emitted when any actor state has `ax < -0.5`
- **`SpecialStructurePedestrianCrossing`** — emitted when `scenario_type` contains `"pedestrian"`
- **`ZoneSchool`** — emitted when `scenario_type` contains `"school"`
- Added `has_acceleration()` and `has_deceleration()` helper functions
- Updated `test_ontology_tags_always_present` to assert `RoadTypeDistributor` (2-lane unidirectional) instead of `RoadTypeMotorway`
- Added 8 new targeted tests covering all new conditional logic

## After

Tags accurately reflect the actual scenario content. A pedestrian-only scenario no longer claims `VehicleCar`. A 2-lane road no longer claims motorway classification. Bidirectional roads correctly emit both travel direction tags. Acceleration/deceleration events are captured from trajectory data.
