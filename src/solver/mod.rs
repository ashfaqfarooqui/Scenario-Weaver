//! Solver module
//!
//! Z3 SMT solver integration and constraint encoding

pub mod backend;
pub mod coordinate_encoder;
pub mod encoder;
pub mod encoders;
pub mod multi_solve;

pub use backend::OptimizationTarget as BackendOptimizationTarget;
pub use backend::{OptimizerBackend, SolverBackend, Z3Backend};
pub use coordinate_encoder::CoordinateEncoder;
pub use encoder::Z3Encoder;
pub use encoders::{BicycleEncoder, CartesianEncoder};
