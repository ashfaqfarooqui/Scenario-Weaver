//! Linear Temporal Logic (LTL) formula construction and generation.
//!
//! Provides an AST for LTL formulas ([`LTLFormula`]) and a generator ([`LTLGenerator`])
//! that converts a [`ScenarioSpec`](crate::dsl::types::ScenarioSpec) into temporal
//! constraints suitable for bounded model checking over a finite time horizon.

pub mod formula;
pub mod generator;

pub use formula::{LTLFormula, Proposition};
pub use generator::LTLGenerator;
