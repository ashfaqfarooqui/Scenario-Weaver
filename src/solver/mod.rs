//! Solver module
//!
//! Z3 SMT solver integration and constraint encoding

pub mod encoder;
pub mod multi_solve;
pub mod physics;

pub use encoder::Z3Encoder;
