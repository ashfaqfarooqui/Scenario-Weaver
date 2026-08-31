//! Shared helpers for the integration-test crates.
//!
//! Rust builds each file in `tests/` as its own crate, so `tests/common/mod.rs`
//! is the standard place to share fixtures. Every integration test that loads an
//! example YAML or generates a scenario goes through here, so there is exactly
//! one definition of "load an example" and one definition of "generate or fail".
//!
//! The central rule this module enforces: **a solver result is never accepted
//! without a committed expectation.** `Expect` says, per scenario, whether the
//! constraints are meant to be solvable; a mismatch is a test failure, not a
//! `println!`.

#![allow(dead_code)] // each test crate uses a different subset
#![allow(unused_imports)] // ditto for the invariants re-exports

use std::path::{Path, PathBuf};

pub mod invariants;

pub use invariants::{
    assert_all_scenario_invariants, assert_invariant, assert_scenario_invariants,
    check_scenario_invariants, is_known_broken, Invariant, Violation, KNOWN_BROKEN_INVARIANTS, TOL,
};

use scenario_weaver::dsl::types::{ConstraintMode, ScenarioSpec};
use scenario_weaver::error::ScenarioGenError;
use scenario_weaver::scenario::model::Scenario;

// ---------------------------------------------------------------------------
// Locating example inputs
// ---------------------------------------------------------------------------

/// Repository root (the directory holding `Cargo.toml`).
#[must_use]
pub fn project_root() -> &'static Path {
    Path::new(env!("CARGO_MANIFEST_DIR"))
}

/// `<repo>/examples`.
#[must_use]
pub fn examples_dir() -> PathBuf {
    project_root().join("examples")
}

/// Every `examples/*.yaml`, discovered by reading the directory, sorted by name.
///
/// Nothing in the suite may hard-code a list of examples: a new example file
/// must be picked up automatically, and must fail the expectation-coverage test
/// until someone commits an expectation for it.
#[must_use]
pub fn example_paths() -> Vec<PathBuf> {
    let dir = examples_dir();
    let mut paths: Vec<PathBuf> = std::fs::read_dir(&dir)
        .unwrap_or_else(|e| panic!("cannot read {}: {e}", dir.display()))
        .map(|entry| entry.expect("directory entry").path())
        .filter(|p| p.extension().is_some_and(|ext| ext == "yaml"))
        .collect();
    paths.sort();
    assert!(
        !paths.is_empty(),
        "no examples/*.yaml found under {}",
        dir.display()
    );
    paths
}

/// File names (`"cut_in_left.yaml"`) of every example on disk, sorted.
#[must_use]
pub fn example_names() -> Vec<String> {
    example_paths()
        .iter()
        .map(|p| {
            p.file_name()
                .expect("example path has a file name")
                .to_string_lossy()
                .into_owned()
        })
        .collect()
}

/// Absolute path of one example by file name.
#[must_use]
pub fn example_path(name: &str) -> PathBuf {
    examples_dir().join(name)
}

/// Read one `examples/<name>` as text. Panics with the path on failure.
#[must_use]
pub fn load_example(name: &str) -> String {
    let path = example_path(name);
    std::fs::read_to_string(&path)
        .unwrap_or_else(|e| panic!("cannot read example {}: {e}", path.display()))
}

/// Parse one `examples/<name>` into a spec, resolving `imports:` relative to the
/// example's own directory. Panics with the path on failure.
#[must_use]
pub fn parse_example(name: &str) -> ScenarioSpec {
    let path = example_path(name);
    scenario_weaver::dsl::parse_yaml_file(&path)
        .unwrap_or_else(|e| panic!("cannot parse example {}: {e}", path.display()))
}

/// Parse one `examples/<name>` from its text (no `imports:` resolution).
#[must_use]
pub fn parse_example_yaml(name: &str) -> ScenarioSpec {
    let yaml = load_example(name);
    scenario_weaver::dsl::parser::parse_yaml(&yaml)
        .unwrap_or_else(|e| panic!("cannot parse example {name}: {e}"))
}

// ---------------------------------------------------------------------------
// Generation with a committed expectation
// ---------------------------------------------------------------------------

/// What a scenario's constraints are committed to be.
///
/// Deliberately not defaulted and not inferred: every call site states which one
/// it expects, so an input that flips from solvable to infeasible (or back) is a
/// test failure rather than a log line.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Expect {
    /// The solver must find a model.
    Solvable,
    /// The solver must prove no model exists.
    Infeasible,
}

/// True when the error is the solver's "no model exists" verdict, as opposed to
/// a parse, encoding or extraction failure.
///
/// This is the single place in the whole test tree that names that variant;
/// everywhere else asks this question through `Expect`.
#[must_use]
pub fn is_infeasible(err: &ScenarioGenError) -> bool {
    matches!(err, ScenarioGenError::Unsatisfiable)
}

/// Generate from YAML text, panicking with the underlying error on failure.
///
/// Replaces the three divergent per-file `generate_from_file` helpers, one of
/// which silently discarded the error by returning `Option`.
///
/// Every scenario that leaves this module has been through
/// [`invariants::assert_scenario_invariants`]. Routing the check through the
/// three shared generators, rather than pasting a call into forty tests, means
/// there is one definition of "a scenario that is allowed to exist" and a new
/// test cannot forget to apply it.
#[must_use]
pub fn generate_or_fail(yaml: &str) -> Scenario {
    let spec = scenario_weaver::dsl::parser::parse_yaml(yaml)
        .unwrap_or_else(|e| panic!("cannot parse scenario YAML: {e}"));
    generate_spec_or_fail(spec)
}

/// Generate from a spec, panicking with the underlying error on failure.
#[must_use]
pub fn generate_spec_or_fail(spec: ScenarioSpec) -> Scenario {
    let scenario = scenario_weaver::generate_single_scenario_from_spec(spec.clone())
        .unwrap_or_else(|e| panic!("expected a solvable scenario, solver returned: {e}"));
    invariants::assert_scenario_invariants(&scenario, &spec);
    scenario
}

/// Generate `examples/<name>`, panicking on failure.
#[must_use]
pub fn generate_example(name: &str) -> Scenario {
    let spec = parse_example(name);
    let scenario = scenario_weaver::generate_single_scenario_from_spec(spec.clone())
        .unwrap_or_else(|e| panic!("expected {name} to be solvable, solver returned: {e}"));
    invariants::assert_scenario_invariants(&scenario, &spec);
    scenario
}

/// Assert that a spec is infeasible *because the solver proved it*, and say why
/// that is the committed expectation.
pub fn assert_infeasible(spec: ScenarioSpec, why: &str) {
    match scenario_weaver::generate_single_scenario_from_spec(spec) {
        Ok(scenario) => panic!(
            "expected no model to exist ({why}), but the solver produced one: \
             {} actors, min_ttc={:?}, min_distance={:?}",
            scenario.actors.len(),
            scenario.validation.min_ttc,
            scenario.validation.min_distance
        ),
        Err(e) if is_infeasible(&e) => {}
        Err(e) => panic!("expected the solver's no-model verdict ({why}), got: {e}"),
    }
}

/// Run one spec against a committed [`Expect`], returning a one-line description
/// of the outcome on success and an explanatory message on mismatch.
///
/// # Errors
/// Returns a human-readable mismatch description when the outcome differs from
/// `expect`, or when generation failed for a reason other than infeasibility.
pub fn check_expectation(
    label: &str,
    spec: ScenarioSpec,
    expect: Expect,
) -> Result<String, String> {
    match (
        scenario_weaver::generate_single_scenario_from_spec(spec.clone()),
        expect,
    ) {
        (Ok(s), Expect::Solvable) => {
            if s.actors.is_empty() {
                return Err(format!("{label}: solvable but produced no actors"));
            }
            if s.scenario_id.is_empty() {
                return Err(format!("{label}: solvable but scenario_id is empty"));
            }
            if s.time_step <= 0.0 {
                return Err(format!("{label}: invalid time_step {}", s.time_step));
            }
            if s.actors.iter().any(|a| a.states.is_empty()) {
                return Err(format!("{label}: an actor has no trajectory states"));
            }
            let breaches: Vec<String> = invariants::check_scenario_invariants(&s, &spec)
                .into_iter()
                .filter(|v| !invariants::is_known_broken(v.invariant))
                .map(|v| v.to_string())
                .collect();
            if !breaches.is_empty() {
                return Err(format!("{label}: {}", breaches.join("; ")));
            }
            Ok(format!(
                "{label}: solvable ({} actors, {} steps, min_ttc={:?}, min_distance={:?})",
                s.actors.len(),
                s.actors[0].states.len(),
                s.validation.min_ttc,
                s.validation.min_distance
            ))
        }
        (Ok(s), Expect::Infeasible) => Err(format!(
            "{label}: expected no model to exist, but got one \
             (min_ttc={:?}, min_distance={:?})",
            s.validation.min_ttc, s.validation.min_distance
        )),
        (Err(e), Expect::Infeasible) if is_infeasible(&e) => Ok(format!("{label}: infeasible")),
        (Err(e), Expect::Infeasible) => Err(format!(
            "{label}: expected the solver's no-model verdict, got a different error: {e}"
        )),
        (Err(e), Expect::Solvable) => Err(format!("{label}: expected a model, got: {e}")),
    }
}

// ---------------------------------------------------------------------------
// Committed expectations for the example corpus
// ---------------------------------------------------------------------------

/// Every `examples/*.yaml` and the outcome it is committed to produce.
///
/// `examples_smoke_test::test_expectations_cover_every_example` asserts that
/// this table plus [`KNOWN_BROKEN_EXAMPLES`] partitions the directory exactly,
/// so a new example cannot be silently uncovered.
pub const EXAMPLE_EXPECTATIONS: &[(&str, Expect)] = &[
    ("bicycle_lane_change.yaml", Expect::Solvable),
    ("cut_in_left.yaml", Expect::Solvable),
    ("cut_in_left_adversarial_all.yaml", Expect::Solvable),
    ("cut_in_left_adversarial_ttc.yaml", Expect::Solvable),
    ("cut_in_left_optimize_max_ttc.yaml", Expect::Solvable),
    ("cut_in_left_optimize_min_distance.yaml", Expect::Solvable),
    ("cut_in_left_optimize_min_severity.yaml", Expect::Solvable),
    ("cut_in_left_optimize_min_ttc.yaml", Expect::Solvable),
    ("cut_in_right.yaml", Expect::Solvable),
    ("cut_in_right_bicycle.yaml", Expect::Solvable),
    ("head_on_collision.yaml", Expect::Solvable),
    ("head_on_near_miss.yaml", Expect::Solvable),
    ("multi_lane_safety.yaml", Expect::Solvable),
    ("overtake_left.yaml", Expect::Solvable),
    ("overtake_with_opposite.yaml", Expect::Solvable),
    ("pedestrian_crossing.yaml", Expect::Solvable),
    ("pedestrian_running.yaml", Expect::Solvable),
    ("pedestrian_wide_road.yaml", Expect::Solvable),
    ("simple_bidirectional.yaml", Expect::Solvable),
    ("speed_limit_violation.yaml", Expect::Solvable),
    ("unsafe_following.yaml", Expect::Solvable),
];

/// Examples that cannot currently be loaded at all, each with the issue that
/// owns the fix. They are excluded from the corpus sweep and each has its own
/// `#[ignore = "SW-NN"]` test that fails for the documented reason.
pub const KNOWN_BROKEN_EXAMPLES: &[(&str, &str)] = &[(
    "with_import.yaml",
    "SW-21: `imports: roads/...` resolves relative to the YAML's own directory, \
     but roads/ lives at the repo root, so the shipped example cannot be loaded \
     from examples/ as its own header documents",
)];

// ---------------------------------------------------------------------------
// Corpus slices, derived from the expectation table
// ---------------------------------------------------------------------------

/// Every solvable example, paired with its parsed spec.
///
/// Derived from [`EXAMPLE_EXPECTATIONS`], never from a hand-written list, so a
/// new example is swept automatically the moment it has an expectation.
#[must_use]
pub fn solvable_examples() -> Vec<(&'static str, ScenarioSpec)> {
    EXAMPLE_EXPECTATIONS
        .iter()
        .filter(|(_, expect)| *expect == Expect::Solvable)
        .map(|(name, _)| (*name, parse_example(name)))
        .collect()
}

/// Every solvable example whose spec declares `mode` for the constraint picked
/// out by `select`, paired with its parsed spec.
///
/// The same derive-don't-hardcode rule as [`solvable_examples`]: an example that
/// changes its constraint modes, or a new one that declares `enforce`, is picked
/// up without editing a list.
#[must_use]
pub fn examples_declaring(
    select: fn(&ScenarioSpec) -> ConstraintMode,
    mode: ConstraintMode,
) -> Vec<(&'static str, ScenarioSpec)> {
    solvable_examples()
        .into_iter()
        .filter(|(_, spec)| select(spec) == mode)
        .collect()
}

/// Generate `examples/<name>` and hand back the spec it came from, so the
/// caller can assert invariants against the bounds the spec actually declares
/// rather than against transcribed literals.
#[must_use]
pub fn generate_example_with_spec(name: &str) -> (Scenario, ScenarioSpec) {
    let spec = parse_example(name);
    (generate_spec_or_fail(spec.clone()), spec)
}

/// Generate from YAML text and hand back the spec it parsed to.
#[must_use]
pub fn generate_yaml_with_spec(yaml: &str) -> (Scenario, ScenarioSpec) {
    let spec = scenario_weaver::dsl::parser::parse_yaml(yaml)
        .unwrap_or_else(|e| panic!("cannot parse scenario YAML: {e}"));
    (generate_spec_or_fail(spec.clone()), spec)
}

/// Generate from a spec and hand back both, so invariants can be asserted
/// against the same spec that drove generation.
#[must_use]
pub fn generate_spec_with_spec(spec: ScenarioSpec) -> (Scenario, ScenarioSpec) {
    (generate_spec_or_fail(spec.clone()), spec)
}

/// Generate `n` diverse scenarios from YAML text, panicking on failure, with
/// every scenario checked against the invariants.
///
/// The multi-scenario API takes a callback type parameter that every no-callback
/// call site had to spell out as a nine-line turbofish; this hides that and adds
/// the invariant check the single-scenario helpers already apply.
#[must_use]
pub fn generate_multiple_or_fail(yaml: &str, n: usize) -> Vec<Scenario> {
    let spec = scenario_weaver::dsl::parser::parse_yaml(yaml)
        .unwrap_or_else(|e| panic!("cannot parse scenario YAML: {e}"));
    let scenarios = scenario_weaver::generate_multiple_scenarios_from_spec(
        spec.clone(),
        n,
        None::<fn(usize, &Scenario) -> scenario_weaver::error::Result<()>>,
    )
    .unwrap_or_else(|e| panic!("expected {n} solvable scenarios, solver returned: {e}"));
    for scenario in &scenarios {
        invariants::assert_scenario_invariants(scenario, &spec);
    }
    scenarios
}
