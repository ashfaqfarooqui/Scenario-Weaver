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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn display_unsatisfiable() {
        let e = ScenarioGenError::Unsatisfiable;
        let msg = e.to_string();
        assert!(msg.to_lowercase().contains("unsat"), "got: {msg}");
    }

    #[test]
    fn display_invalid_spec() {
        let e = ScenarioGenError::InvalidSpec("msg".into());
        assert!(e.to_string().contains("msg"));
    }

    #[test]
    fn display_z3_encoding() {
        let e = ScenarioGenError::Z3Encoding("detail".into());
        assert!(e.to_string().contains("detail"));
    }

    #[test]
    fn display_extraction_failed() {
        let e = ScenarioGenError::ExtractionFailed("reason".into());
        assert!(e.to_string().contains("reason"));
    }

    #[test]
    fn display_xosc_export() {
        let e = ScenarioGenError::XoscExport("err".into());
        assert!(e.to_string().contains("err"));
    }

    #[test]
    fn display_gif_export() {
        let e = ScenarioGenError::GifExport("err".into());
        assert!(e.to_string().contains("err"));
    }

    #[test]
    fn display_open_label_export() {
        let e = ScenarioGenError::OpenLabelExport("err".into());
        assert!(e.to_string().contains("err"));
    }

    #[test]
    fn display_ltl_generation() {
        let e = ScenarioGenError::LTLGeneration("err".into());
        assert!(e.to_string().contains("err"));
    }

    #[test]
    fn display_z3_model_parsing() {
        let e = ScenarioGenError::Z3ModelParsing("err".into());
        assert!(e.to_string().contains("err"));
    }

    #[test]
    fn display_font_loading() {
        let e = ScenarioGenError::FontLoading("err".into());
        assert!(e.to_string().contains("err"));
    }

    #[test]
    fn display_yaml_structure() {
        let e = ScenarioGenError::YamlStructure("err".into());
        assert!(e.to_string().contains("err"));
    }

    #[test]
    fn display_actor_not_found() {
        let e = ScenarioGenError::ActorNotFound("actor1".into());
        assert!(e.to_string().contains("actor1"));
    }

    #[test]
    fn from_io_error() {
        let io_err = std::io::Error::new(std::io::ErrorKind::NotFound, "file missing");
        let e: ScenarioGenError = io_err.into();
        assert!(e.to_string().contains("file missing"));
        assert!(matches!(e, ScenarioGenError::Io(_)));
    }

    #[test]
    fn result_type_alias() {
        let ok: Result<i32> = Ok(42);
        assert_eq!(ok.unwrap(), 42);

        let err: Result<i32> = Err(ScenarioGenError::Unsatisfiable);
        assert!(err.is_err());
    }

    #[test]
    fn debug_all_variants() {
        let variants: Vec<ScenarioGenError> = vec![
            ScenarioGenError::Unsatisfiable,
            ScenarioGenError::InvalidSpec("x".into()),
            ScenarioGenError::LTLGeneration("x".into()),
            ScenarioGenError::Z3Encoding("x".into()),
            ScenarioGenError::ExtractionFailed("x".into()),
            ScenarioGenError::XoscExport("x".into()),
            ScenarioGenError::GifExport("x".into()),
            ScenarioGenError::OpenLabelExport("x".into()),
            ScenarioGenError::Io(std::io::Error::new(std::io::ErrorKind::Other, "x")),
            ScenarioGenError::Z3ModelParsing("x".into()),
            ScenarioGenError::FontLoading("x".into()),
            ScenarioGenError::YamlStructure("x".into()),
            ScenarioGenError::ActorNotFound("x".into()),
        ];
        for v in &variants {
            let _ = format!("{:?}", v);
        }
    }

    #[test]
    fn pattern_matching() {
        let e = ScenarioGenError::ActorNotFound("ego".into());
        match e {
            ScenarioGenError::ActorNotFound(name) => assert_eq!(name, "ego"),
            _ => panic!("wrong variant"),
        }
    }
}
