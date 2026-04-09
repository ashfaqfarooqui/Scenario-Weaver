//! Solver module
//!
//! Z3 SMT solver integration and constraint encoding

pub mod backend;
pub mod coordinate_encoder;
pub mod encoder;
pub mod encoder_utils;
pub mod encoders;
pub mod multi_solve;

pub use backend::OptimizationTarget as BackendOptimizationTarget;
pub use backend::{OptimizerBackend, SolverBackend, Z3Backend};
pub use coordinate_encoder::CoordinateEncoder;
pub use encoder::GenericEncoder;
pub use encoder::Z3Encoder;
pub use encoder_utils::{collect_lane_change_data, extract_int, extract_real, LaneChangeSteps};
pub use encoders::{BicycleEncoder, CartesianEncoder};
