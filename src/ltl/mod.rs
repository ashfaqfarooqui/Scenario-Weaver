//! LTL (Linear Temporal Logic) module

pub mod formula;
pub mod generator;

pub use formula::{LTLFormula, Proposition};
pub use generator::LTLGenerator;
