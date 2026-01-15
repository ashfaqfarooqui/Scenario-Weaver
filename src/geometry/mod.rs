//! Geometry module for Frenet coordinate system
//!
//! Provides types and conversions between Frenet and Cartesian coordinate systems.

pub mod frenet;
pub mod reference_line;

pub use frenet::{CartesianPoint, FrenetPoint};
pub use reference_line::ReferenceLine;
