//! Scenario output, extraction, and export.
//!
//! Contains the output data model ([`Scenario`], [`ActorTrajectory`], [`State`]),
//! Z3 model extraction, and exporters for OpenSCENARIO, OpenDRIVE, SVG, GIF, and OpenLabel.

pub mod extractor;
pub mod gif_animator;
pub mod model;
pub mod openlabel_exporter;
pub mod svg_visualizer;
pub mod visualization_common;
pub mod xodr_exporter;
pub mod xosc_exporter;

pub use gif_animator::{export_to_gif, export_to_gif_with_resolution, Resolution};
pub use model::{ActorTrajectory, Position, Scenario, State, ValidationInfo, Velocity};
pub use openlabel_exporter::export_to_openlabel;
pub use svg_visualizer::export_to_svg;
pub use xodr_exporter::export_to_xodr;
pub use xosc_exporter::{export_to_xosc, export_to_xosc_with_road_file};
