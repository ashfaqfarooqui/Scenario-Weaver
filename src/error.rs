//! Error types for the scenario generator

use thiserror::Error;

#[derive(Error, Debug)]
pub enum ScenarioGenError {
    #[error("Failed to parse YAML: {0}")]
    YamlParse(#[from] serde_yml::Error),

    #[error("Failed to serialize JSON: {0}")]
    JsonSerialize(#[from] serde_json::Error),

    #[error("Z3 solver returned UNSAT - no valid scenario exists")]
    Unsatisfiable,

    #[error("Invalid DSL specification: {0}")]
    InvalidSpec(String),

    #[error("LTL formula generation failed: {0}")]
    LTLGeneration(String),

    #[error("Z3 encoding failed: {0}")]
    Z3Encoding(String),

    #[error("Scenario extraction failed: {0}")]
    ExtractionFailed(String),

    #[error("OpenSCENARIO export failed: {0}")]
    XoscExport(String),

    #[error("GIF export failed: {0}")]
    GifExport(String),

    #[error("OpenDRIVE export failed: {0}")]
    XodrExport(String),

    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),

    #[error("Scenario validation failed: {0}")]
    ValidationFailed(#[from] anyhow::Error),
}

pub type Result<T> = std::result::Result<T, ScenarioGenError>;
