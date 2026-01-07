//! CLI for CARLA Scenario Generator

use anyhow::Result;
use clap::Parser;
use std::path::PathBuf;
use tracing::Level;

#[derive(Parser)]
#[command(name = "carla-scenario-gen")]
#[command(about = "Generate CARLA driving scenarios using LTL + Z3", long_about = None)]
#[command(version)]
struct Cli {
    /// Input YAML specification file
    #[arg(short, long, value_name = "FILE")]
    input: PathBuf,

    /// Output JSON file (or directory for multiple scenarios)
    #[arg(short, long, value_name = "PATH")]
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

    tracing::info!("CARLA Scenario Generator");
    tracing::info!("Loading specification from: {:?}", cli.input);

    // Read YAML file
    let yaml_content = std::fs::read_to_string(&cli.input)?;

    // Parse specification
    let mut spec = carla_scenario_generator::dsl::parser::parse_yaml(&yaml_content)?;

    // Apply CLI override for adversarial mode
    if cli.adversarial {
        use carla_scenario_generator::dsl::types::ConstraintModes;
        tracing::warn!("CLI override: Setting all constraints to VIOLATE mode");
        spec.constraint_modes = ConstraintModes::Shorthand("violate_all".to_string());
    }

    let num_scenarios = cli.num.unwrap_or(spec.num_scenarios);

    tracing::info!("Generating {} scenario(s)...", num_scenarios);

    // Re-serialize spec to YAML if modified
    let final_yaml = if cli.adversarial {
        serde_yaml::to_string(&spec)?
    } else {
        yaml_content
    };

    // Generate scenarios
    let scenarios = if num_scenarios == 1 {
        vec![carla_scenario_generator::generate_single_scenario(
            &final_yaml,
        )?]
    } else {
        carla_scenario_generator::generate_multiple_scenarios(&final_yaml, num_scenarios)?
    };

    tracing::info!("Successfully generated {} scenario(s)", scenarios.len());

    // Write output - BOTH JSON and XOSC formats
    if scenarios.len() == 1 {
        write_single_scenario(&scenarios[0], &cli.output)?;
        print_scenario_summary(&scenarios[0]);
    } else {
        write_multiple_scenarios(&scenarios, &cli.output)?;
        for (i, scenario) in scenarios.iter().enumerate() {
            println!("\n--- Scenario {} ---", i);
            print_scenario_summary(scenario);
        }
    }

    tracing::info!("Done!");
    Ok(())
}

/// Write a single scenario in both JSON and XOSC formats
fn write_single_scenario(
    scenario: &carla_scenario_generator::scenario::model::Scenario,
    output_path: &PathBuf,
) -> Result<()> {
    let base_path = output_path.with_extension("");

    // Write JSON
    let json_path = base_path.with_extension("json");
    let json = serde_json::to_string_pretty(scenario)?;
    std::fs::write(&json_path, json)?;
    tracing::info!("Wrote JSON to: {:?}", json_path);

    // Write XOSC
    let xosc_path = base_path.with_extension("xosc");
    let xosc_xml = carla_scenario_generator::scenario::export_to_xosc(scenario)?;
    std::fs::write(&xosc_path, xosc_xml)?;
    tracing::info!("Wrote XOSC to: {:?}", xosc_path);

    // Write SVG
    let svg_path = base_path.with_extension("svg");
    let svg = carla_scenario_generator::scenario::export_to_svg(scenario)?;
    std::fs::write(&svg_path, svg)?;
    tracing::info!("Wrote SVG to: {:?}", svg_path);

    Ok(())
}

/// Write multiple scenarios in both JSON and XOSC formats
fn write_multiple_scenarios(
    scenarios: &[carla_scenario_generator::scenario::model::Scenario],
    output_dir: &PathBuf,
) -> Result<()> {
    std::fs::create_dir_all(output_dir)?;

    for (i, scenario) in scenarios.iter().enumerate() {
        let base = format!("scenario_{}", i);

        // Write JSON
        let json_path = output_dir.join(format!("{}.json", base));
        let json = serde_json::to_string_pretty(scenario)?;
        std::fs::write(&json_path, json)?;
        tracing::debug!("Wrote JSON {} to: {:?}", i, json_path);

        // Write XOSC
        let xosc_path = output_dir.join(format!("{}.xosc", base));
        let xosc_xml = carla_scenario_generator::scenario::export_to_xosc(scenario)?;
        std::fs::write(&xosc_path, xosc_xml)?;
        tracing::debug!("Wrote XOSC {} to: {:?}", i, xosc_path);

        // Write SVG
        let svg_path = output_dir.join(format!("{}.svg", base));
        let svg = carla_scenario_generator::scenario::export_to_svg(scenario)?;
        std::fs::write(&svg_path, svg)?;
        tracing::debug!("Wrote SVG {} to: {:?}", i, svg_path);
    }

    tracing::info!(
        "Wrote {} scenario triplets (JSON+XOSC+SVG) to directory: {:?}",
        scenarios.len(),
        output_dir
    );

    Ok(())
}

/// Print a summary of the scenario
fn print_scenario_summary(scenario: &carla_scenario_generator::scenario::model::Scenario) {
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
            initial.lane,
            initial.position.x,
            initial.position.y,
            initial.velocity.vx,
            initial.velocity.vy
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
