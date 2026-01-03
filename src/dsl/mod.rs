//! DSL (Domain-Specific Language) module
//!
//! Parses YAML specifications into structured types

pub mod parser;
pub mod types;

pub use parser::{parse_yaml, parse_yaml_file};
pub use types::{ActorRole, ActorSpec, ScenarioSpec, ScenarioType, ValueOrRange};
