//! CARLA Scenario Generator
//!
//! Generate driving test scenarios from high-level specifications using
//! Linear Temporal Logic (LTL) + Z3 SMT solver.

pub mod dsl;
pub mod error;
pub mod ltl;
pub mod scenario;
pub mod scenarios;
pub mod solver;

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
    let ltl_formula = ltl::generator::LTLGenerator::generate(&spec);

    // Setup Z3 and solve
    let cfg = z3::Config::new();
    let ctx = z3::Context::new(&cfg);
    let mut encoder = solver::Z3Encoder::new(&ctx, spec);

    // Create variables
    encoder.create_variables();

    // Encode initial conditions and kinematics
    encoder.encode_initial_conditions();
    encoder.encode_kinematics();
    encoder.encode_lane_velocity_constraints();

    // Encode LTL formula
    encoder.encode_ltl(&ltl_formula);

    // Call scenario-specific Z3 constraints (if any)
    encoder.encode_scenario_specific_constraints(&*scenario_model)?;

    // Encode safety constraints
    encoder.encode_safety();

    // Solve and extract scenario
    match encoder.check() {
        SatResult::Sat => {
            let model = encoder
                .get_model()
                .ok_or_else(|| {
                    ScenarioGenError::ExtractionFailed("Failed to get Z3 model".to_string())
                })?;
            Ok(encoder.extract_scenario(&model))
        }
        SatResult::Unsat => Err(ScenarioGenError::Unsatisfiable),
        SatResult::Unknown => Err(ScenarioGenError::Z3Encoding(
            "Z3 solver returned UNKNOWN".to_string(),
        )),
    }
}

/// Generate multiple diverse scenarios from YAML specification
///
/// This is the main entry point for Phase 11 - multiple scenario generation.
/// Uses blocking clauses to generate diverse scenarios.
///
/// # Arguments
/// * `yaml_content` - YAML specification string
/// * `num_scenarios` - Number of scenarios to generate
///
/// # Returns
/// A vector of generated scenarios
///
/// # Errors
/// Returns error if YAML parsing or specification validation fails
pub fn generate_multiple_scenarios(
    yaml_content: &str,
    num_scenarios: usize,
) -> Result<Vec<Scenario>> {
    // Parse specification
    let spec = dsl::parser::parse_yaml(yaml_content)?;

    // Generate LTL formula (same for all scenarios)
    let ltl_formula = ltl::generator::LTLGenerator::generate(&spec);

    // Use multi-solve module to generate multiple scenarios
    solver::multi_solve::generate_scenarios(&spec, &ltl_formula, num_scenarios)
}

/// Export a scenario to OpenSCENARIO XML format
///
/// Converts an internally generated scenario to OpenSCENARIO (.xosc) format
/// for use with simulation platforms like CARLA.
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
/// use carla_scenario_generator::{generate_single_scenario, export_scenario_to_xosc};
///
/// let yaml = std::fs::read_to_string("scenario.yaml").unwrap();
/// let scenario = generate_single_scenario(&yaml).unwrap();
/// let xosc_xml = export_scenario_to_xosc(&scenario).unwrap();
/// std::fs::write("scenario.xosc", xosc_xml).unwrap();
/// ```
pub fn export_scenario_to_xosc(scenario: &Scenario) -> Result<String> {
    scenario::export_to_xosc(scenario)
}
