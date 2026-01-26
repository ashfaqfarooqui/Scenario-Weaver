//!  Scenario Generator
//!
//! Generate driving test scenarios from high-level specifications using
//! Linear Temporal Logic (LTL) + Z3 SMT solver.

pub mod dsl;
pub mod error;
pub mod ltl;
pub mod scenario;
pub mod scenarios;
pub mod solver;
pub mod trajectory;

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
    // Parse YAML into DSL specification
    let spec = dsl::parser::parse_yaml(yaml_content)?;

    // Get scenario model and validate scenario-specific requirements
    let scenario_model = spec.scenario_type.get_model();
    scenario_model.validate(&spec)?;

    // Generate LTL using trait (behavior + safety combined)
    let ltl_formula = ltl::generator::LTLGenerator::generate(&spec)?;

    // Check if optimization is requested
    use dsl::types::OptimizationTarget;
    match spec.optimization_target {
        OptimizationTarget::None => {
            // Standard SAT solving - find any satisfying solution
            generate_with_solver(spec, &ltl_formula, &*scenario_model)
        }
        target => {
            // Optimization mode - find optimal scenario
            tracing::info!("Optimization target: {:?}", target);
            // TODO: Full optimization with numerical TTC/distance objectives
            // For now, fall back to standard solving with a warning
            tracing::warn!(
                "Optimization mode {:?} is experimental. \
                Full numerical optimization requires additional encoding. \
                Falling back to standard SAT solving.",
                target
            );
            generate_with_solver(spec, &ltl_formula, &*scenario_model)
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

        // Create variables
        encoder.create_variables();

        // Encode initial conditions and kinematics
        encoder.encode_initial_conditions();
        encoder.encode_kinematics();
        encoder.encode_lane_velocity_constraints();
        encoder.encode_lateral_velocity_bounds();

        // Encode LTL formula
        encoder.encode_ltl(ltl_formula);

        // Call scenario-specific Z3 constraints (if any)
        encoder.encode_scenario_specific_constraints(scenario_model)?;

        // Note: Safety constraints are now handled via LTL propositions in generate_safety()
        // No need for direct encode_safety() call - avoids redundant constraints

        // Solve and extract scenario
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

/// Generate multiple diverse scenarios from YAML specification
///
/// This is the main entry point for Phase 11 - multiple scenario generation.
/// Uses blocking clauses to generate diverse scenarios.
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
    // Parse specification
    let spec = dsl::parser::parse_yaml(yaml_content)?;

    // Generate LTL formula (same for all scenarios)
    let ltl_formula = ltl::generator::LTLGenerator::generate(&spec)?;

    // Use multi-solve module to generate multiple scenarios
    solver::multi_solve::generate_scenarios(&spec, &ltl_formula, num_scenarios, callback)
}

/// Export a scenario to OpenSCENARIO XML format
///
/// Converts an internally generated scenario to OpenSCENARIO (.xosc) format
/// for use with simulation platforms like .
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
/// use scenario_generator::{generate_single_scenario, export_scenario_to_xosc};
///
/// let yaml = std::fs::read_to_string("scenario.yaml").unwrap();
/// let scenario = generate_single_scenario(&yaml).unwrap();
/// let xosc_xml = export_scenario_to_xosc(&scenario).unwrap();
/// std::fs::write("scenario.xosc", xosc_xml).unwrap();
/// ```
pub fn export_scenario_to_xosc(scenario: &Scenario) -> Result<String> {
    scenario::export_to_xosc(scenario)
}

/// Export a scenario to SVG format for visualization
///
/// Generates an SVG file showing vehicle trajectories, lane layout, and safety metrics.
///
/// # Example
/// ```no_run
/// use scenario_generator::{generate_single_scenario, export_scenario_to_svg};
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
/// use scenario_generator::{generate_single_scenario, export_scenario_to_gif};
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
/// use scenario_generator::{generate_single_scenario, export_scenario_to_gif_with_resolution, Resolution};
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

/// Re-export the Resolution type for GIF export configuration
pub use scenario::Resolution;
