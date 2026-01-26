//! Coordinate system-specific encoder implementations

pub mod bicycle;
pub mod cartesian;

pub use bicycle::BicycleEncoder;
pub use cartesian::CartesianEncoder;
