# Architecture

← [Back to README](../README.md)

## Pipeline

```
YAML Input
  → DSL Parser       (src/dsl/)         — validate and deserialise spec
  → Scenario Model   (src/scenarios/)   — generate behavioural LTL
  → LTL Generator    (src/ltl/)         — expand temporal operators
  → Z3 Encoder       (src/solver/)      — encode physics + LTL as SMT
  → Z3 Solver                           — find satisfying assignment
  → Scenario Extractor (src/scenario/)  — pull trajectories, compute metrics
  → Output Writers                      — JSON, XOSC, XODR, SVG, GIF, OpenLabel
```

---

## Encoder Architecture

The solver layer uses a **trait-based plugin system** to support multiple coordinate systems without duplicating constraint logic.

### `CoordinateEncoder<B>` trait (`src/solver/coordinate_encoder.rs`)

Defines the interface every coordinate-specific encoder must implement:

| Method | Purpose |
|--------|---------|
| `create_variables()` | Allocate Z3 variables for all actors × time steps |
| `encode_kinematics()` | Motion equations (position/velocity updates) |
| `encode_initial_conditions()` | Starting positions and velocities |
| `encode_velocity_constraints()` | Velocity bounds |
| `encode_acceleration_constraints()` | Acceleration bounds |
| `get_longitudinal_pos(actor, t)` | Position variable accessor |
| `get_lateral_pos(actor, t)` | Lateral position accessor |
| `get_longitudinal_vel(actor, t)` | Velocity accessor |
| `get_lateral_vel(actor, t)` | Lateral velocity accessor |
| `get_lane_var(actor, t)` | Lane assignment accessor |
| `extract_actor_trajectory()` | Pull trajectory from Z3 model |

Always use accessor methods — never access encoder fields directly:

```rust
// Correct
let px = encoder.get_longitudinal_pos("ego", t);

// Wrong — fields no longer public
let px = &encoder.positions_x["ego"][t];
```

### `GenericEncoder<B>` (`src/solver/encoder.rs`)

Thin facade that:
- Holds `Box<dyn CoordinateEncoder<B>>`
- Selects the concrete encoder at construction time based on `spec.coordinate_system`
- Provides coordinate-agnostic methods: `encode_ltl()`, `encode_safety()`, `extract_scenario()`, `compute_validation_metrics()`

### Coordinate-specific encoders (`src/solver/encoders/`)

| File | Coordinate system | Variables |
|------|-------------------|-----------|
| `cartesian.rs` | (x, y) | `x`, `y`, `vx`, `vy`, `lane` |
| `bicycle.rs` | (x, y, θ, v) | `x`, `y`, `θ`, `v`, `δ`, `a`, `lane` |

See [Coordinate Systems](coordinate-systems.md) for dynamics details.

### Adding a new coordinate system

1. Create `src/solver/encoders/mycoords.rs` and implement `CoordinateEncoder<B>`
2. Add a variant to `CoordinateSystem` enum in `src/dsl/types.rs`
3. Dispatch to it in `GenericEncoder::with_backend` in `src/solver/encoder.rs`
4. Add tests and update this document

---

## ScenarioModel Trait (`src/scenarios/`)

All scenario types implement:

```rust
pub trait ScenarioModel: Send + Sync {
    fn validate(&self, spec: &ScenarioSpec) -> Result<()>;
    fn generate_ltl(&self, spec: &ScenarioSpec) -> Result<LTLFormula>;

    // Optional overrides
    fn generate_safety(&self, spec: &ScenarioSpec) -> Result<LTLFormula> {
        Ok(generate_default_safety(spec))  // pairwise TTC + distance
    }
    fn add_z3_constraints(&self, ...) -> Result<()> { Ok(()) }
}
```

See [Creating New Scenario Types](scenario-types.md) for a step-by-step guide.

---

## Key Design Decisions

**`Z3Encoder` type alias**
`Z3Encoder` is a convenience type alias for `GenericEncoder<SolverBackend>` defined in `src/solver/encoder.rs`. It is the default encoder used in standard SAT-solving mode. The optimizer path uses `GenericEncoder<OptimizerBackend>` directly.

**Optimizer limitation**
When `--optimize` is used, the encoder runs `generate_with_optimizer()` in `src/lib.rs`, which skips the `encode_scenario_specific_constraints()` call. For all built-in scenario types this is a no-op, so it has no practical effect. Any custom scenario type that overrides `add_z3_constraints()` will not have those assertions applied in optimizer mode.

**Constraint modes (Enforce / Violate / Ignore)**
- `Enforce` → `G(constraint)` added to LTL *and* as a direct Z3 assertion
- `Violate` → `F(NOT constraint)` added to LTL only; no direct assertion
- `Ignore` → constraint omitted entirely

**Multi-scenario diversity**
Blocking clauses exclude previous solutions, forcing Z3 to find structurally different trajectories on each call. Diversity focuses on NPC position and cut-in timing.

**Lane change timing**
`start_time` and `duration` ranges are resolved to their midpoint before encoding (see `encoder_utils.rs`). The solver cannot currently explore different timings for diversity — tracked as a TODO.

**Two-layer encoding**
1. LTL temporal operators expanded over all time steps (all modes)
2. Direct Z3 assertions for `Enforce` mode only — avoids conflicting with `Violate`/`Ignore`

---

## Test Files

| File | Coverage |
|------|----------|
| `tests/integration_test.rs` | Full pipeline YAML → JSON for all scenario types |
| `tests/cartesian_physics_test.rs` | Velocity ratio constraints, heading angle bounds |
| `tests/bidirectional_test.rs` | Bidirectional lane scenarios, backward velocity constraints |
| `tests/comprehensive_test.rs` | Broad constraint mode combinations, edge cases |

---

## Module Map

```
src/
  main.rs                    CLI entry point
  lib.rs                     Public API
  error.rs                   Error types
  dsl/
    types.rs                 ScenarioSpec, ActorSpec, RoadSpec, ConstraintModes
    parser.rs                YAML parsing and validation
  ltl/
    formula.rs               LTL AST (Always, Eventually, And, Or, Proposition)
    generator.rs             LTL generation from ScenarioSpec
  solver/
    encoder.rs               GenericEncoder facade
    coordinate_encoder.rs    CoordinateEncoder trait
    encoders/
      cartesian.rs           CartesianEncoder
      bicycle.rs             BicycleEncoder
    encoder_utils.rs         Shared helpers: lane change resolution, Z3 value extraction
    multi_solve.rs           Blocking-clause diversity
    backend.rs               SolverBackend / OptimizerBackend traits
  scenarios/
    mod.rs                   ScenarioModel trait + default safety
    cut_in_left.rs
    cut_in_right.rs
    overtake_left.rs
    pedestrian_crossing.rs
  scenario/
    model.rs                 Scenario, ActorTrajectory, ValidationInfo
    extractor.rs             Z3 model → trajectory + metrics
    xosc_exporter.rs         OpenSCENARIO export
    xodr_exporter.rs         OpenDRIVE export
    openlabel_exporter.rs    OpenLabel export
    svg_visualizer.rs        SVG static visualisation
    gif_animator.rs          GIF animation export
```
