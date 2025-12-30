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

    // Parse to get num_scenarios (either from CLI or YAML)
    let spec = carla_scenario_generator::dsl::parser::parse_yaml(&yaml_content)?;
    let num_scenarios = cli.num.unwrap_or(spec.num_scenarios);

    tracing::info!("Generating {} scenario(s)...", num_scenarios);

    // Generate scenarios
    let scenarios = if num_scenarios == 1 {
        vec![carla_scenario_generator::generate_single_scenario(
            &yaml_content,
        )?]
    } else {
        carla_scenario_generator::generate_multiple_scenarios(&yaml_content, num_scenarios)?
    };

    tracing::info!("Successfully generated {} scenario(s)", scenarios.len());

    // Write output
    if scenarios.len() == 1 {
        // Single scenario: write to single JSON file
        let json = serde_json::to_string_pretty(&scenarios[0])?;
        std::fs::write(&cli.output, json)?;
        tracing::info!("Wrote scenario to: {:?}", cli.output);

        // Print summary
        print_scenario_summary(&scenarios[0]);
    } else {
        // Multiple scenarios: create directory and write files
        std::fs::create_dir_all(&cli.output)?;

        for (i, scenario) in scenarios.iter().enumerate() {
            let filename = format!("scenario_{}.json", i);
            let path = cli.output.join(filename);
            let json = serde_json::to_string_pretty(scenario)?;
            std::fs::write(&path, json)?;
            tracing::debug!("Wrote scenario {} to: {:?}", i, path);
        }

        tracing::info!(
            "Wrote {} scenarios to directory: {:?}",
            scenarios.len(),
            cli.output
        );

        // Print summaries
        for (i, scenario) in scenarios.iter().enumerate() {
            println!("\n--- Scenario {} ---", i);
            print_scenario_summary(scenario);
        }
    }

    tracing::info!("Done!");
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
