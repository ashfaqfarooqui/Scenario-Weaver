//! Error types for ScenarioWeaver

use thiserror::Error;

/// All errors that can occur during scenario generation, export, or validation.
#[derive(Error, Debug)]
pub enum ScenarioGenError {
    /// The input YAML could not be deserialized into a [`ScenarioSpec`](crate::dsl::types::ScenarioSpec).
    #[error("Failed to parse YAML: {0}")]
    YamlParse(#[from] serde_yml::Error),

    /// JSON serialization of the output scenario failed.
    #[error("Failed to serialize JSON: {0}")]
    JsonSerialize(#[from] serde_json::Error),

    /// The Z3 solver proved no solution exists for the given constraints.
    /// This typically means the specification is over-constrained (e.g., impossible
    /// lane change duration at the given speed).
    #[error("Z3 solver returned UNSAT - no valid scenario exists")]
    Unsatisfiable,

    /// The parsed YAML is syntactically valid but semantically invalid
    /// (e.g., missing ego actor, lane index out of range).
    #[error("Invalid DSL specification: {0}")]
    InvalidSpec(String),

    /// LTL formula construction failed, usually due to missing actor references.
    #[error("LTL formula generation failed: {0}")]
    LTLGeneration(String),

    /// A constraint could not be encoded into Z3 (e.g., unknown proposition type).
    #[error("Z3 encoding failed: {0}")]
    Z3Encoding(String),

    /// The Z3 model was SAT but trajectory extraction failed
    /// (e.g., a variable could not be evaluated).
    #[error("Scenario extraction failed: {0}")]
    ExtractionFailed(String),

    /// OpenSCENARIO XML generation failed.
    #[error("OpenSCENARIO export failed: {0}")]
    XoscExport(String),

    /// GIF animation encoding failed (frame rendering or encoding error).
    #[error("GIF export failed: {0}")]
    GifExport(String),

    /// OpenLabel JSON export failed.
    #[error("OpenLabel export failed: {0}")]
    OpenLabelExport(String),

    /// File system I/O error (reading YAML, writing output files).
    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),

    /// Post-generation validation detected constraint violations.
    #[error("Scenario validation failed: {0}")]
    ValidationFailed(#[from] anyhow::Error),

    /// A Z3 model value could not be parsed to a numeric type.
    #[error("Z3 model parsing failed: {0}")]
    Z3ModelParsing(String),

    /// The embedded font for GIF text rendering could not be loaded.
    #[error("Font loading failed: {0}")]
    FontLoading(String),

    /// The YAML has correct syntax but unexpected structure for import merging.
    #[error("YAML structure error: {0}")]
    YamlStructure(String),

    /// An actor ID referenced in a constraint or query does not exist in the spec.
    #[error("Actor not found: {0}")]
    ActorNotFound(String),
}

/// Convenience alias used throughout the crate.
pub type Result<T> = std::result::Result<T, ScenarioGenError>;
