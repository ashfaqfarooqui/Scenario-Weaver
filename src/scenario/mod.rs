//! Scenario module
//!
//! Output data structures and extraction from Z3 models

pub mod extractor;
pub mod model;
pub mod svg_visualizer;
pub mod xosc_exporter;

pub use model::{ActorTrajectory, Position, Scenario, State, ValidationInfo, Velocity};
pub use svg_visualizer::export_to_svg;
pub use xosc_exporter::export_to_xosc;
