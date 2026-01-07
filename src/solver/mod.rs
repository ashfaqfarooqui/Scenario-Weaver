//! Solver module
//!
//! Z3 SMT solver integration and constraint encoding

pub mod backend;
pub mod encoder;
pub mod multi_solve;

pub use backend::{OptimizerBackend, SolverBackend, Z3Backend};
pub use backend::OptimizationTarget as BackendOptimizationTarget;
pub use encoder::Z3Encoder;
