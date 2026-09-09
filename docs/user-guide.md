# User Guide

← [Back to README](../README.md)

ScenarioWeaver generates diverse, safety-critical driving test scenarios from declarative YAML
specifications. You describe the actors, the road, and the constraints; the tool encodes the
description as Linear Temporal Logic (LTL) over a Z3 SMT model and asks the solver for concrete
trajectories that satisfy what you asked for, or, in adversarial mode, deliberately violate it.

This guide covers installation, the quick-start commands, and the full CLI reference. For how to
actually write a scenario spec, see [authoring-scenarios.md](authoring-scenarios.md) (a worked
walkthrough) and [yaml-reference.md](yaml-reference.md) (the complete field-by-field schema). For
how the pipeline fits together internally, see [architecture.md](architecture.md); for the
difference between the two coordinate systems, see
[coordinate-systems.md](coordinate-systems.md).

---

## Installation

### Prerequisites

- A `C` toolchain, Z3, and libxml2. On Ubuntu 24.04:

  ```bash
  sudo apt install build-essential clang libclang-dev pkg-config libz3-dev libxml2-dev
  ```

  `libclang-dev` is needed by `z3-sys`'s bindgen; `libxml2-dev` is needed transitively via
  `openscenario-rs` → `libxml`. On macOS, `brew install z3` covers the solver, though you may
  still need Xcode's command-line tools for the rest of the toolchain.
- Rust 1.90+ and Cargo. Install from
  [doc.rust-lang.org/cargo/getting-started/installation.html](https://doc.rust-lang.org/cargo/getting-started/installation.html).

### Build

```bash
git clone <repo-url>
cd <repo-name>
cargo build --release
```

Verify the build:

```bash
cargo test
```

The binary lands at `target/release/scenario-weaver`. The examples below use `cargo run
--release --` in its place; swap in the binary path once you have built it, or drop `--release`
for a faster (but slower-running) debug build during iteration.

If Cargo reports `could not find native static library z3` during the build, the Z3 development
library is missing – reinstall it per the prerequisites above rather than looking further; this is
the one error the build reliably surfaces this way.

---

## Quick Start

Generate a single scenario from one of the bundled examples:

```bash
cargo run --release -- -i examples/cut_in_left.yaml -o output/
```

This solves the spec once and writes six files to `output/`:

```
output/scenario.json
output/scenario.xosc
output/scenario.xodr
output/scenario.svg
output/scenario.gif
output/scenario.ol.json
```

Open `scenario.svg` in a browser for the complete top-down trajectory, or `scenario.gif` to watch
the scenario play out. See [output-formats.md](output-formats.md) for what each file actually
contains and how the formats relate to one another.

A few more entry points into the same spec:

```bash
# Ten structurally distinct scenarios from one spec (blocking clauses force diversity)
cargo run --release -- -i examples/cut_in_left.yaml -o scenarios/ -n 10

# Adversarial: override every constraint mode to "violate"
cargo run --release -- -i examples/cut_in_left.yaml -o adversarial/ --adversarial

# The kinematic bicycle model instead of the default Cartesian point-mass
cargo run --release -- -i examples/bicycle_lane_change.yaml -o output/

# Verbose logging, useful when a spec comes back UNSAT
cargo run --release -- -i examples/cut_in_left.yaml -o output/ -v
```

In multi-scenario mode (`-n N`), the six files above are named `scenario_0.*` .. `scenario_{N-1}.*`
in the output directory rather than `scenario.*`.

---

## CLI Reference

```
scenario-weaver [OPTIONS] --input <FILE> --output <DIR>

Options:
  -i, --input <FILE>       Input YAML specification file (required)
  -o, --output <DIR>       Output directory for generated scenarios (required)
  -n, --num <N>            Number of scenarios to generate (overrides the YAML's num_scenarios)
  -v, --verbose            Enable debug-level logging
      --adversarial        Override all constraint modes to violate
      --optimize <TARGET>  Find an optimal scenario instead of any satisfying one:
                            min-ttc, min-distance, max-severity, max-ttc
      --seed <SEED>        Seed for multi-scenario diversity (`-n` > 1). Defaults to a
                            fixed constant.
  -h, --help                Print help
  -V, --version              Print version
```

Every option maps directly onto the `Cli` struct in `src/main.rs`; there is no hidden
configuration file or environment variable behind any of these flags.

- `-i/--input` and `-o/--output` are the only required flags. The output directory is created if
  it does not already exist.
- `-n/--num`, when given, overrides the spec's own `num_scenarios:` field. Omit it to use whatever
  the YAML says.
- `-v/--verbose` raises the log level from `INFO` to `DEBUG`. Reach for it first when a spec
  returns UNSAT and you need to see which constraint the solver could not satisfy.
- `--adversarial` sets `constraint_modes` to the `violate_all` shorthand for this run, regardless
  of what the spec says. It is a CLI-level override, not a separate code path: the same
  `enforce`/`violate`/`ignore` machinery documented in
  [adversarial-generation.md](adversarial-generation.md) handles it.
- `--optimize <TARGET>` switches from Z3's plain Solver to its Optimize solver and returns the
  scenario that best matches `TARGET` instead of the first one found. The CLI spelling is
  kebab-case (`min-ttc`); the equivalent YAML field, `optimization_target:`, uses snake_case
  (`minimize_ttc`) – the two are not interchangeable, and one spelling is a parse error in the
  other's slot. See [optimizer.md](optimizer.md) for what each target actually optimizes and the
  direction of its bound.
- `--seed <SEED>` governs how a batch (`-n` > 1) is spread across its declared ranges – it seeds
  the Latin-hypercube strata described in
  [architecture.md](architecture.md#constraint-modes-and-multi-scenario-diversity), not Z3 itself.
  The same seed reproduces the same batch (modulo each scenario's UUID); a different seed gives a
  different, equally valid one. Omit it and re-runs stay byte-identical, as they always have.

---

## What You Get

Every run, single or multi-scenario, standard or adversarial or optimized, produces the same six
files per scenario: a canonical JSON record of the solved trajectories, an OpenSCENARIO `.xosc`
and a matching OpenDRIVE `.xodr` for simulator import, a static SVG and an animated GIF for quick
visual inspection, and an OpenLABEL `.ol.json` for dataset cataloging. All six are derived from the
same solved `Scenario`, so they never disagree with one another. The full description of each
format (exact contents, JSON shape, and known limitations) lives in
[output-formats.md](output-formats.md).

---

## Writing a Scenario Spec

This guide stops at running the tool. To actually write a spec (the five parts of a
`ScenarioSpec`, fixed values versus ranges, lane changes, constraint modes), see
[authoring-scenarios.md](authoring-scenarios.md) for a worked cut-in walkthrough and
[yaml-reference.md](yaml-reference.md) for the complete schema. If you are choosing between the
Cartesian and bicycle coordinate systems, [coordinate-systems.md](coordinate-systems.md) covers
the trade-off and the honest limits of the bicycle model's linearization.
