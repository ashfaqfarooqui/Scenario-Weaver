//! CLI for  Scenario Generator

use anyhow::Result;
use clap::Parser;
use std::path::PathBuf;
use tracing::Level;

/// Optimization target options
#[derive(Clone, Debug, clap::ValueEnum)]
enum OptimizeTarget {
    /// Minimize time-to-collision (find worst-case TTC)
    MinTtc,
    /// Minimize distance (find closest approach)
    MinDistance,
    /// Minimize both (weighted severity)
    MinSeverity,
    /// Maximize TTC (find safest scenario)
    MaxTtc,
}

#[derive(Parser)]
#[command(name = "scenario-gen")]
#[command(about = "Generate  driving scenarios using LTL + Z3", long_about = None)]
#[command(version)]
struct Cli {
    /// Input YAML specification file
    #[arg(short, long, value_name = "FILE")]
    input: PathBuf,

    /// Output directory for generated scenarios
    #[arg(short, long, value_name = "DIR")]
    output: PathBuf,

    /// Number of scenarios to generate (overrides YAML file)
    #[arg(short, long)]
    num: Option<usize>,

    /// Enable verbose logging
    #[arg(short, long)]
    verbose: bool,

    /// Override constraint modes to violate all safety constraints
    #[arg(long)]
    adversarial: bool,

    /// Optimization target: find optimal scenarios instead of any satisfying solution
    #[arg(long, value_enum)]
    optimize: Option<OptimizeTarget>,
}

fn main() -> Result<()> {
    let cli = Cli::parse();

    // Setup logging
    let log_level = if cli.verbose {
        Level::DEBUG
    } else {
        Level::INFO
    };

    tracing_subscriber::fmt()
        .with_max_level(log_level)
        .with_target(false)
        .init();

    tracing::info!(" Scenario Generator");
    tracing::info!("Loading specification from: {:?}", cli.input);

    // Read YAML file
    let yaml_content = std::fs::read_to_string(&cli.input)?;

    // Parse specification
    let mut spec = scenario_generator::dsl::parser::parse_yaml(&yaml_content)?;

    // Apply CLI override for adversarial mode
    if cli.adversarial {
        use scenario_generator::dsl::types::ConstraintModes;
        tracing::warn!("CLI override: Setting all constraints to VIOLATE mode");
        spec.constraint_modes = ConstraintModes::Shorthand("violate_all".to_string());
    }

    // Apply CLI override for optimization target
    if let Some(optimize) = &cli.optimize {
        use scenario_generator::dsl::types::OptimizationTarget;
        let target = match optimize {
            OptimizeTarget::MinTtc => OptimizationTarget::MinimizeTtc,
            OptimizeTarget::MinDistance => OptimizationTarget::MinimizeDistance,
            OptimizeTarget::MinSeverity => OptimizationTarget::MinimizeSeverity,
            OptimizeTarget::MaxTtc => OptimizationTarget::MaximizeTtc,
        };
        tracing::info!("CLI override: Optimization target set to {:?}", target);
        spec.optimization_target = target;
    }

    let num_scenarios = cli.num.unwrap_or(spec.num_scenarios);

    tracing::info!("Generating {} scenario(s)...", num_scenarios);

    // Re-serialize spec to YAML if modified
    let final_yaml = if cli.adversarial || cli.optimize.is_some() {
        serde_yml::to_string(&spec)?
    } else {
        yaml_content
    };

    // Generate scenarios
    let scenarios = if num_scenarios == 1 {
        vec![scenario_generator::generate_single_scenario(&final_yaml)?]
    } else {
        // Create callback to write each scenario immediately after generation
        let output_dir = cli.output.clone();
        let callback = |i: usize,
                        scenario: &scenario_generator::scenario::model::Scenario|
         -> scenario_generator::error::Result<()> {
            write_scenario(scenario, &output_dir, i, num_scenarios).map_err(|e| {
                scenario_generator::error::ScenarioGenError::ExtractionFailed(e.to_string())
            })
        };
        scenario_generator::generate_multiple_scenarios(&final_yaml, num_scenarios, Some(callback))?
    };

    tracing::info!("Successfully generated {} scenario(s)", scenarios.len());

    // Write output for single scenario (multiple scenarios written via callback)
    if num_scenarios == 1 {
        write_scenarios(&scenarios, &cli.output)?;
    } else {
        // For multiple scenarios, print summary since files already written
        tracing::info!(
            "Wrote {} scenario quadruplet(s) (JSON+XOSC+SVG+GIF+XODR) to directory: {:?}",
            scenarios.len(),
            cli.output
        );
    }

    if scenarios.len() == 1 {
        print_scenario_summary(&scenarios[0]);
    } else {
        for (i, scenario) in scenarios.iter().enumerate() {
            println!("\n--- Scenario {} ---", i);
            print_scenario_summary(scenario);
        }
    }

    tracing::info!("Done!");
    Ok(())
}

/// Write a single scenario to a directory
fn write_scenario(
    scenario: &scenario_generator::scenario::model::Scenario,
    output_dir: &PathBuf,
    index: usize,
    total_scenarios: usize,
) -> Result<()> {
    std::fs::create_dir_all(output_dir)?;

    // For single scenario, use "scenario" as base name
    // For multiple scenarios, use "scenario_0", "scenario_1", etc.
    let base = if total_scenarios == 1 {
        "scenario".to_string()
    } else {
        format!("scenario_{}", index)
    };

    // Write JSON
    let json_path = output_dir.join(format!("{}.json", base));
    let json = serde_json::to_string_pretty(scenario)?;
    std::fs::write(&json_path, json)?;
    tracing::debug!("Wrote JSON to: {:?}", json_path);

    // Write XODR first so the filename is ready for the XOSC reference
    let xodr_filename = format!("{}.xodr", base);
    let xodr_path = output_dir.join(&xodr_filename);
    let xodr_xml = scenario_generator::scenario::export_to_xodr(scenario)?;
    std::fs::write(&xodr_path, xodr_xml)?;
    tracing::debug!("Wrote XODR to: {:?}", xodr_path);

    // Write XOSC with a relative reference to the companion .xodr file
    let xosc_path = output_dir.join(format!("{}.xosc", base));
    let xosc_xml =
        scenario_generator::scenario::export_to_xosc_with_road_file(scenario, &xodr_filename)?;
    std::fs::write(&xosc_path, xosc_xml)?;
    tracing::debug!("Wrote XOSC to: {:?}", xosc_path);

    // Write SVG
    let svg_path = output_dir.join(format!("{}.svg", base));
    let svg = scenario_generator::scenario::export_to_svg(scenario)?;
    std::fs::write(&svg_path, svg)?;
    tracing::debug!("Wrote SVG to: {:?}", svg_path);

    // Write GIF
    let gif_path = output_dir.join(format!("{}.gif", base));
    let gif_bytes = scenario_generator::scenario::export_to_gif(scenario)?;
    std::fs::write(&gif_path, gif_bytes)?;
    tracing::debug!("Wrote GIF to: {:?}", gif_path);

    Ok(())
}

/// Write scenarios to a directory (handles both single and multiple scenarios)
fn write_scenarios(
    scenarios: &[scenario_generator::scenario::model::Scenario],
    output_dir: &PathBuf,
) -> Result<()> {
    for (i, scenario) in scenarios.iter().enumerate() {
        write_scenario(scenario, output_dir, i, scenarios.len())?;
    }

    tracing::info!(
        "Wrote {} scenario quadruplet(s) (JSON+XOSC+SVG+GIF+XODR) to directory: {:?}",
        scenarios.len(),
        output_dir
    );

    Ok(())
}

/// Print a summary of the scenario
fn print_scenario_summary(scenario: &scenario_generator::scenario::model::Scenario) {
    println!("Scenario ID: {}", scenario.scenario_id);
    println!("Type: {}", scenario.scenario_type);
    println!("Duration: {:.1}s", scenario.duration);
    println!("Time steps: {}", scenario.actors[0].states.len());

    // Print actor initial conditions
    for actor in &scenario.actors {
        let initial = &actor.states[0];
        println!(
            "  {} [{}]: lane={}, pos=({:.2}, {:.2}), vel=({:.2}, {:.2})",
            actor.id,
            actor.role,
            initial.lane(),
            initial.position().x,
            initial.position().y,
            initial.velocity().vx,
            initial.velocity().vy
        );
    }

    // Print validation metrics
    println!("\nValidation Metrics:");
    println!("  Min TTC: {:.2}s", scenario.validation.min_ttc);
    println!("  Min Distance: {:.2}m", scenario.validation.min_distance);
    println!(
        "  All Constraints Satisfied: {}",
        scenario.validation.all_constraints_satisfied
    );

    if !scenario.validation.safety_violations.is_empty() {
        println!("\n  Safety Violations:");
        for violation in &scenario.validation.safety_violations {
            println!("    - {}", violation);
        }
    }
}
