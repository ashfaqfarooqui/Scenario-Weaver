//! Trajectory generation module
//!
//! Provides polynomial-based smooth lane change trajectory generation.

pub mod polynomial;

pub use polynomial::{
    evaluate_polynomial,
    evaluate_polynomial_acceleration,
    evaluate_polynomial_derivative,
    solve_quintic_polynomial,
};
