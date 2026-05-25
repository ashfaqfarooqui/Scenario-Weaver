//! ScenarioWeaver
//!
//! Generate driving test scenarios from high-level specifications using
//! Linear Temporal Logic (LTL) + Z3 SMT solver.

pub mod dsl;
pub mod error;
pub mod ltl;
pub mod scenario;
pub mod scenarios;
pub mod solver;

use dsl::types::OptimizationTarget;
use error::{Result, ScenarioGenError};
use scenario::model::Scenario;
use z3::SatResult;

/// Generate a single scenario from YAML specification
///
/// This is the main entry point for scenario generation.
///
/// # Arguments
/// * `yaml_content` - YAML specification string
///
/// # Returns
/// A single generated scenario with actor trajectories
///
/// # Errors
/// Returns error if:
/// - YAML parsing fails
/// - Specification is invalid
/// - Z3 solver returns UNSAT (no solution exists)
pub fn generate_single_scenario(yaml_content: &str) -> Result<Scenario> {
    let spec = dsl::parser::parse_yaml(yaml_content)?;
    generate_single_scenario_from_spec(spec)
}

/// Generate a single scenario from a pre-parsed specification
///
/// Use this when you have already parsed and possibly modified the spec,
/// to avoid a fragile YAML round-trip.
///
/// Validation is performed inside [`ltl::generator::LTLGenerator::generate`],
/// which is the single canonical validation point for all generation paths.
pub fn generate_single_scenario_from_spec(spec: dsl::types::ScenarioSpec) -> Result<Scenario> {
    let scenario_model = spec.scenario_type.get_model();

    // Validation is intentionally omitted here; LTLGenerator::generate is the
    // canonical validation point and runs unconditionally on all code paths.
    let ltl_formula = ltl::generator::LTLGenerator::generate(&spec)?;

    match spec.optimization_target {
        OptimizationTarget::None => generate_with_solver(spec, &ltl_formula, &*scenario_model),
        target => {
            tracing::info!("Optimization target: {:?}", target);
            generate_with_optimizer(spec, &ltl_formula, target)
        }
    }
}

/// Generate scenario using standard Z3 Solver (SAT checking)
fn generate_with_solver(
    spec: dsl::types::ScenarioSpec,
    ltl_formula: &ltl::formula::LTLFormula,
    scenario_model: &dyn scenarios::ScenarioModel,
) -> Result<Scenario> {
    let cfg = z3::Config::new();
    z3::with_z3_config(&cfg, || {
        let mut encoder = solver::Z3Encoder::new(spec);

        encoder.create_variables();
        encoder.encode_initial_conditions();
        encoder.encode_kinematics();
        encoder.encode_velocity_constraints();
        encoder.encode_acceleration_constraints();
        encoder.encode_lane_velocity_constraints();
        encoder.encode_lateral_velocity_bounds();
        encoder.encode_ltl(ltl_formula);

        // Safety constraints are encoded via LTL propositions inside encode_ltl();
        // there is no separate encode_safety() call to avoid duplicate assertions.
        encoder.encode_scenario_specific_constraints(scenario_model)?;

        match encoder.check() {
            SatResult::Sat => {
                let model = encoder.get_model().ok_or_else(|| {
                    ScenarioGenError::ExtractionFailed("Failed to get Z3 model".to_string())
                })?;
                encoder.extract_scenario(&model)
            }
            SatResult::Unsat => Err(ScenarioGenError::Unsatisfiable),
            SatResult::Unknown => Err(ScenarioGenError::Z3Encoding(
                "Z3 solver returned UNKNOWN".to_string(),
            )),
        }
    })
}

/// Generate scenario using Z3 Optimize (objective optimization)
fn generate_with_optimizer(
    spec: dsl::types::ScenarioSpec,
    ltl_formula: &ltl::formula::LTLFormula,
    target: dsl::types::OptimizationTarget,
) -> Result<Scenario> {
    use solver::backend::{OptimizationTarget as BackendTarget, OptimizerBackend};

    let backend_target = match target {
        dsl::types::OptimizationTarget::MinimizeTtc => BackendTarget::MinimizeTtc,
        dsl::types::OptimizationTarget::MinimizeDistance => BackendTarget::MinimizeDistance,
        dsl::types::OptimizationTarget::MinimizeSeverity => BackendTarget::MinimizeSeverity,
        dsl::types::OptimizationTarget::MaximizeTtc => BackendTarget::MaximizeTtc,
        dsl::types::OptimizationTarget::None => {
            return Err(ScenarioGenError::InvalidSpec(
                "OptimizationTarget::None passed to optimizer path".to_string(),
            ));
        }
    };

    let cfg = z3::Config::new();
    z3::with_z3_config(&cfg, || {
        let backend = OptimizerBackend::new(backend_target);
        let mut encoder = solver::GenericEncoder::with_backend(spec, backend);

        // Full encoding pipeline (same as solver path)
        encoder.create_variables();
        encoder.encode_initial_conditions();
        encoder.encode_kinematics();
        encoder.encode_velocity_constraints();
        encoder.encode_acceleration_constraints();
        encoder.encode_lane_velocity_constraints();
        encoder.encode_lateral_velocity_bounds();
        encoder.encode_ltl(ltl_formula);

        // Scenario-specific Z3 constraints are skipped in optimizer mode:
        // add_z3_constraints() requires a SolverBackend, and all built-in
        // scenario types have no-op implementations anyway.
        tracing::warn!("Scenario-specific Z3 constraints skipped in optimizer mode");

        encoder.encode_objective();

        match encoder.check() {
            SatResult::Sat => {
                let model = encoder.get_model().ok_or_else(|| {
                    ScenarioGenError::ExtractionFailed("Failed to get Z3 model".to_string())
                })?;

                encoder.extract_optimal_value(&model);
                let opt_val = encoder.get_optimal_value();

                let mut scenario = encoder.extract_scenario(&model)?;

                scenario.optimization = Some(scenario::model::OptimizationInfo {
                    target: format!("{:?}", target),
                    optimal_value: opt_val,
                });

                Ok(scenario)
            }
            SatResult::Unsat => Err(ScenarioGenError::Unsatisfiable),
            SatResult::Unknown => Err(ScenarioGenError::Z3Encoding(
                "Z3 optimizer returned UNKNOWN".to_string(),
            )),
        }
    })
}

/// Generate multiple diverse scenarios from YAML specification
///
/// Uses blocking clauses to ensure each generated scenario is structurally
/// different from the previous ones (diversity based on NPC initial position
/// and velocity).
///
/// # Arguments
/// * `yaml_content` - YAML specification string
/// * `num_scenarios` - Number of scenarios to generate
/// * `callback` - Optional callback invoked after each scenario is generated
///
/// # Returns
/// A vector of generated scenarios
///
/// # Errors
/// Returns error if YAML parsing or specification validation fails
pub fn generate_multiple_scenarios<F>(
    yaml_content: &str,
    num_scenarios: usize,
    callback: Option<F>,
) -> Result<Vec<Scenario>>
where
    F: FnMut(usize, &Scenario) -> Result<()>,
{
    let spec = dsl::parser::parse_yaml(yaml_content)?;
    generate_multiple_scenarios_from_spec(spec, num_scenarios, callback)
}

/// Generate multiple diverse scenarios from a pre-parsed specification
///
/// Use this when you have already parsed and possibly modified the spec,
/// to avoid a fragile YAML round-trip.
pub fn generate_multiple_scenarios_from_spec<F>(
    spec: dsl::types::ScenarioSpec,
    num_scenarios: usize,
    callback: Option<F>,
) -> Result<Vec<Scenario>>
where
    F: FnMut(usize, &Scenario) -> Result<()>,
{
    let ltl_formula = ltl::generator::LTLGenerator::generate(&spec)?;
    solver::multi_solve::generate_scenarios(&spec, &ltl_formula, num_scenarios, callback)
}

/// Export a scenario to OpenSCENARIO XML format
///
/// Converts an internally generated scenario to OpenSCENARIO (.xosc) format
/// for use with OpenSCENARIO-compliant simulation platforms (e.g. CARLA).
///
/// # Arguments
/// * `scenario` - The scenario to export
///
/// # Returns
/// XML string in OpenSCENARIO format
///
/// # Errors
/// Returns error if XML serialization fails
///
/// # Example
/// ```no_run
/// use scenario_weaver::{generate_single_scenario, export_scenario_to_xosc};
///
/// let yaml = std::fs::read_to_string("scenario.yaml").unwrap();
/// let scenario = generate_single_scenario(&yaml).unwrap();
/// let xosc_xml = export_scenario_to_xosc(&scenario).unwrap();
/// std::fs::write("scenario.xosc", xosc_xml).unwrap();
/// ```
pub fn export_scenario_to_xosc(scenario: &Scenario) -> Result<String> {
    scenario::export_to_xosc(scenario)
}

/// Export a scenario to OpenSCENARIO XML format with an OpenDRIVE road reference
///
/// Embeds a `<RoadNetwork><LogicFile>` reference to `xodr_path` inside the
/// generated .xosc.  Use a relative path so the two files stay portable.
pub fn export_scenario_to_xosc_with_road_file(
    scenario: &Scenario,
    xodr_path: &str,
) -> Result<String> {
    scenario::export_to_xosc_with_road_file(scenario, xodr_path)
}

/// Export a scenario to SVG format for visualization
///
/// Generates an SVG file showing vehicle trajectories, lane layout, and safety metrics.
///
/// # Example
/// ```no_run
/// use scenario_weaver::{generate_single_scenario, export_scenario_to_svg};
///
/// let yaml = std::fs::read_to_string("scenario.yaml").unwrap();
/// let scenario = generate_single_scenario(&yaml).unwrap();
/// let svg = export_scenario_to_svg(&scenario).unwrap();
/// std::fs::write("scenario.svg", svg).unwrap();
/// ```
pub fn export_scenario_to_svg(scenario: &Scenario) -> Result<String> {
    scenario::export_to_svg(scenario)
}

/// Export a scenario to animated GIF format
///
/// Generates a GIF animation showing vehicle trajectories evolving over time
/// at 10 FPS with real-time metrics displayed as text overlay.
///
/// # Example
/// ```no_run
/// use scenario_weaver::{generate_single_scenario, export_scenario_to_gif};
///
/// let yaml = std::fs::read_to_string("scenario.yaml").unwrap();
/// let scenario = generate_single_scenario(&yaml).unwrap();
/// let gif_bytes = export_scenario_to_gif(&scenario).unwrap();
/// std::fs::write("scenario.gif", gif_bytes).unwrap();
/// ```
pub fn export_scenario_to_gif(scenario: &Scenario) -> Result<Vec<u8>> {
    scenario::export_to_gif(scenario)
}

/// Export a scenario to animated GIF format with custom resolution
///
/// # Example
/// ```no_run
/// use scenario_weaver::{generate_single_scenario, export_scenario_to_gif_with_resolution, Resolution};
///
/// let yaml = std::fs::read_to_string("scenario.yaml").unwrap();
/// let scenario = generate_single_scenario(&yaml).unwrap();
/// let gif_bytes = export_scenario_to_gif_with_resolution(&scenario, Resolution::High).unwrap();
/// std::fs::write("scenario.gif", gif_bytes).unwrap();
/// ```
pub fn export_scenario_to_gif_with_resolution(
    scenario: &Scenario,
    resolution: scenario::Resolution,
) -> Result<Vec<u8>> {
    scenario::export_to_gif_with_resolution(scenario, resolution)
}

/// Export a scenario to OpenDRIVE XML format
///
/// Generates an OpenDRIVE 1.7 road network file (.xodr) describing the
/// single straight road from the scenario's `RoadSpec`.
///
/// # Errors
/// Returns an error if XML serialization fails.
pub fn export_scenario_to_xodr(scenario: &Scenario) -> Result<String> {
    scenario::export_to_xodr(scenario)
}

/// Export a scenario to OpenLabel 1.0.0 JSON format
///
/// Generates a minimal OpenLabel file containing scenario metadata and
/// semantic tags (road type, scenario category, actor roles, behaviors).
///
/// # Example
/// ```no_run
/// use scenario_weaver::{generate_single_scenario, export_scenario_to_openlabel};
///
/// let yaml = std::fs::read_to_string("scenario.yaml").unwrap();
/// let scenario = generate_single_scenario(&yaml).unwrap();
/// let ol_json = export_scenario_to_openlabel(&scenario).unwrap();
/// std::fs::write("scenario.ol.json", ol_json).unwrap();
/// ```
pub fn export_scenario_to_openlabel(scenario: &Scenario) -> Result<String> {
    scenario::export_to_openlabel(scenario)
}

/// Re-export the Resolution type for GIF export configuration
pub use scenario::Resolution;
