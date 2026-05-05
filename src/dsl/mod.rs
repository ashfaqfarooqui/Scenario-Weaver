//! DSL (Domain-Specific Language) module for scenario specification.
//!
//! Parses YAML files describing driving scenarios into typed Rust structures.
//! Supports actor definitions (ego, NPC, pedestrian), road geometry, lane changes,
//! constraint modes (enforce/violate/ignore), and multiple coordinate systems.

pub mod parser;
pub mod types;

pub use parser::{parse_yaml, parse_yaml_file};
pub use types::{ActorRole, ActorSpec, ScenarioSpec, ScenarioType, ValueOrRange};
