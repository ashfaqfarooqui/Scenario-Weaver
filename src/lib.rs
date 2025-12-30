//! CARLA Scenario Generator
//!
//! Generate driving test scenarios from high-level specifications using
//! Linear Temporal Logic (LTL) + Z3 SMT solver.

pub mod dsl;
pub mod error;
pub mod ltl;
pub mod scenario;
pub mod solver;

use error::{Result, ScenarioGenError};
use scenario::model::Scenario;
use z3::SatResult;

/// Generate a single scenario from YAML specification
///
/// This is the main entry point for Phase 10 - single scenario generation.
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
    // Phase 1-2: Parse YAML into DSL specification
    let spec = dsl::parser::parse_yaml(yaml_content)?;

    // Phase 3-4: Generate LTL formula from specification
    let ltl_formula = ltl::generator::LTLGenerator::generate(&spec);

    // Phase 5-9: Setup Z3 and solve
    let cfg = z3::Config::new();
    let ctx = z3::Context::new(&cfg);
    let mut encoder = solver::Z3Encoder::new(&ctx, spec);

    // Phase 6: Create Z3 variables
    encoder.create_variables();

    // Phase 6: Encode initial conditions
    encoder.encode_initial_conditions();

    // Phase 6: Encode kinematic constraints
    encoder.encode_kinematics();

    // Phase 7: Encode LTL formula
    encoder.encode_ltl(&ltl_formula);

    // Phase 8: Encode safety constraints
    encoder.encode_safety();

    // Phase 9: Solve and extract scenario
    match encoder.check() {
        SatResult::Sat => {
            let model = encoder.get_model().ok_or_else(|| {
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
