//! Z3 backend abstraction
//!
//! This module provides a trait abstraction over Z3's `Solver` and `Optimize` types,
//! allowing the encoder to work with either backend without code duplication.

use z3::ast::{Bool, Real};
use z3::{Model, Optimize, SatResult, Solver};

/// Trait abstracting common Z3 solver/optimizer operations
///
/// Both `Solver` and `Optimize` implement this interface, allowing
/// the encoder to be generic over the backend.
pub trait Z3Backend {
    /// Assert a constraint (takes reference to match z3 API)
    fn assert(&self, constraint: &Bool);

    /// Check satisfiability
    fn check(&self) -> SatResult;

    /// Get the model (satisfying assignment) if SAT
    fn get_model(&self) -> Option<Model>;
}

/// Extension trait to allow asserting owned Bool values
pub trait Z3BackendExt: Z3Backend {
    /// Assert an owned constraint (convenience method)
    fn assert_owned(&self, constraint: Bool) {
        self.assert(&constraint);
    }
}

/// Backend using Z3's standard Solver (SAT checking only)
pub struct SolverBackend {
    solver: Solver,
}

impl SolverBackend {
    /// Create a new solver backend
    pub fn new() -> Self {
        Self {
            solver: Solver::new(),
        }
    }
}

impl Default for SolverBackend {
    fn default() -> Self {
        Self::new()
    }
}

impl Z3Backend for SolverBackend {
    fn assert(&self, constraint: &Bool) {
        self.solver.assert(constraint);
    }

    fn check(&self) -> SatResult {
        self.solver.check()
    }

    fn get_model(&self) -> Option<Model> {
        self.solver.get_model()
    }
}

/// Optimization target for the optimizer backend
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OptimizationTarget {
    /// Minimize time-to-collision (find worst-case TTC)
    MinimizeTtc,
    /// Minimize distance (find closest approach)
    MinimizeDistance,
    /// Minimize both (weighted combination)
    MinimizeSeverity,
    /// Maximize TTC (find safest scenario)
    MaximizeTtc,
}

/// Backend using Z3's Optimize (supports optimization objectives)
pub struct OptimizerBackend {
    optimizer: Optimize,
    /// Optimization target
    target: OptimizationTarget,
    /// The objective variable being optimized
    objective_var: Option<Real>,
    /// Optimal value found (after solving)
    optimal_value: Option<f64>,
}

impl OptimizerBackend {
    /// Create a new optimizer backend with the given target
    pub fn new(target: OptimizationTarget) -> Self {
        Self {
            optimizer: Optimize::new(),
            target,
            objective_var: None,
            optimal_value: None,
        }
    }

    /// Get the optimization target
    pub fn target(&self) -> OptimizationTarget {
        self.target
    }

    /// Set the objective variable to minimize
    pub fn minimize(&mut self, objective: &Real) {
        self.optimizer.minimize(objective);
    }

    /// Set the objective variable to maximize
    pub fn maximize(&mut self, objective: &Real) {
        self.optimizer.maximize(objective);
    }

    /// Store the objective variable for later extraction
    pub fn set_objective_var(&mut self, var: Real) {
        self.objective_var = Some(var);
    }

    /// Get the optimal value after solving
    pub fn get_optimal_value(&self) -> Option<f64> {
        self.optimal_value
    }

    /// Extract optimal value from model
    pub fn extract_optimal_value(&mut self, model: &Model) {
        if let Some(ref obj_var) = self.objective_var {
            if let Some(val) = model.eval(obj_var, true) {
                let val_str = val.to_string();
                self.optimal_value = parse_z3_real(&val_str);
            }
        }
    }
}

impl Z3Backend for OptimizerBackend {
    fn assert(&self, constraint: &Bool) {
        self.optimizer.assert(constraint);
    }

    fn check(&self) -> SatResult {
        self.optimizer.check(&[])
    }

    fn get_model(&self) -> Option<Model> {
        self.optimizer.get_model()
    }
}

/// Parse a Z3 real value string to f64
/// Handles formats like "5.0", "5/2", "(/ 5 2)", "(- 5)", "(- / 5 2)"
fn parse_z3_real(s: &str) -> Option<f64> {
    let cleaned = s.replace(['(', ')'], "");
    let parts: Vec<&str> = cleaned.split_whitespace().collect();

    if parts.is_empty() {
        return None;
    }

    // Format: "- / numerator denominator" -> negative fraction
    if parts.len() >= 4 && parts[0] == "-" && parts[1] == "/" {
        let numerator: f64 = parts[2].parse().ok()?;
        let denominator: f64 = parts[3].parse().ok()?;
        return Some(-(numerator / denominator));
    }

    // Format: "/ numerator denominator" -> positive fraction
    if parts.len() >= 3 && parts[0] == "/" {
        let numerator: f64 = parts[1].parse().ok()?;
        let denominator: f64 = parts[2].parse().ok()?;
        return Some(numerator / denominator);
    }

    // Format: "- value" -> simple negative
    if parts.len() == 2 && parts[0] == "-" {
        let value: f64 = parts[1].parse().ok()?;
        return Some(-value);
    }

    // Format: "numerator/denominator"
    if parts.len() == 1 && parts[0].contains('/') {
        let frac_parts: Vec<&str> = parts[0].split('/').collect();
        let numerator: f64 = frac_parts[0].parse().ok()?;
        let denominator: f64 = frac_parts[1].parse().ok()?;
        return Some(numerator / denominator);
    }

    // Simple number
    parts[0].parse().ok()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_z3_real() {
        assert_eq!(parse_z3_real("5"), Some(5.0));
        assert_eq!(parse_z3_real("5.5"), Some(5.5));
        assert_eq!(parse_z3_real("5/2"), Some(2.5));
        assert_eq!(parse_z3_real("(/ 5 2)"), Some(2.5));
        assert_eq!(parse_z3_real("(- 5)"), Some(-5.0));
        assert_eq!(parse_z3_real("(- / 5 2)"), Some(-2.5));
    }

    #[test]
    fn test_solver_backend() {
        let cfg = z3::Config::new();
        z3::with_z3_config(&cfg, || {
            let backend = SolverBackend::new();

            // Simple satisfiable constraint
            let x = Real::new_const("x");
            let five = Real::from_rational(5, 1);
            backend.assert(&x.gt(&five));

            assert_eq!(backend.check(), SatResult::Sat);
            assert!(backend.get_model().is_some());
        });
    }

    #[test]
    fn test_optimizer_backend() {
        let cfg = z3::Config::new();
        z3::with_z3_config(&cfg, || {
            let mut backend = OptimizerBackend::new(OptimizationTarget::MinimizeTtc);

            // x >= 5, x <= 10, minimize x
            let x = Real::new_const("x");
            let five = Real::from_rational(5, 1);
            let ten = Real::from_rational(10, 1);

            backend.assert(&x.ge(&five));
            backend.assert(&x.le(&ten));
            backend.minimize(&x);
            backend.set_objective_var(x);

            assert_eq!(backend.check(), SatResult::Sat);

            let model = backend.get_model().unwrap();
            backend.extract_optimal_value(&model);

            // Optimal value should be exactly 5 (or very close due to floating point)
            let opt = backend.get_optimal_value().unwrap();
            assert!((opt - 5.0).abs() < 0.01, "Expected 5.0, got {}", opt);
        });
    }
}
