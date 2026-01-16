//! Coordinate system-specific encoder implementations

pub mod cartesian;
pub mod frenet;

pub use cartesian::CartesianEncoder;
pub use frenet::FrenetEncoder;
