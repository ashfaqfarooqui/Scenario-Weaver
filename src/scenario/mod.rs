//! Scenario module
//!
//! Output data structures and extraction from Z3 models

pub mod extractor;
pub mod model;
pub mod xosc_exporter;

pub use model::{ActorTrajectory, Position, Scenario, State, ValidationInfo, Velocity};
pub use xosc_exporter::export_to_xosc;
