//! Scenario module
//!
//! Output data structures and extraction from Z3 models

pub mod extractor;
pub mod model;

pub use model::{ActorTrajectory, Position, Scenario, State, ValidationInfo, Velocity};
