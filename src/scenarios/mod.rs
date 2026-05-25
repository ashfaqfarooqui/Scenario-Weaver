//! Scenario model trait and per-scenario-type implementations.
//!
//! Each scenario type (cut-in, overtake, pedestrian crossing) implements
//! [`ScenarioModel`] to define its behavioral LTL formula and validation rules.

use crate::dsl::types::{ConstraintMode, ScenarioSpec};
use crate::error::Result;
use crate::ltl::formula::{LTLFormula, Proposition};

/// Trait for scenario-specific LTL generation and validation
///
/// Each scenario type implements this trait to define its behavior.
/// Most scenarios only need to implement validate() and generate_ltl().
/// The other methods have default implementations.
pub trait ScenarioModel: Send + Sync {
    /// Validate scenario-specific requirements
    ///
    /// Default implementation provides basic validation.
    /// Override to add scenario-specific checks (e.g., required behavior parameters).
    fn validate(&self, _spec: &ScenarioSpec) -> Result<()> {
        Ok(())
    }

    /// Generate behavioral LTL formula (required)
    ///
    /// Each scenario must implement this method to define its temporal logic.
    /// This should include initial conditions and scenario-specific behaviors,
    /// but NOT safety constraints (those are handled by generate_safety()).
    fn generate_ltl(&self, spec: &ScenarioSpec) -> Result<LTLFormula>;

    /// Generate safety constraints (optional, has default)
    ///
    /// Default implementation generates pairwise safety constraints for all actor pairs.
    /// Override if a scenario needs different safety behavior.
    fn generate_safety(&self, spec: &ScenarioSpec) -> Result<LTLFormula> {
        Ok(generate_default_safety(spec))
    }

    /// Add scenario-specific Z3 constraints (optional, has default)
    ///
    /// Default implementation does nothing.
    /// Override for scenarios needing custom Z3 assertions beyond the LTL encoding.
    fn add_z3_constraints(
        &self,
        _spec: &ScenarioSpec,
        _encoder: &crate::solver::Z3Encoder,
        _backend: &dyn crate::solver::Z3Backend,
        _horizon: usize,
    ) -> Result<()> {
        Ok(())
    }
}

/// Generate default safety constraints for all actor pairs
///
/// This function generates pairwise TTC and distance constraints based on
/// the constraint modes (Enforce/Violate/Ignore) specified in the scenario.
/// Also includes velocity and lateral distance constraints.
fn generate_default_safety(spec: &ScenarioSpec) -> LTLFormula {
    let mut constraints = Vec::new();

    // Generate pairwise safety for all actor combinations
    for (i, actor1) in spec.actors.iter().enumerate() {
        for actor2 in spec.actors.iter().skip(i + 1) {
            // TTC constraint
            match spec.constraint_modes.min_ttc() {
                ConstraintMode::Enforce => {
                    let ttc = LTLFormula::Atom(Proposition::TTCGT {
                        actor1: actor1.id.clone(),
                        actor2: actor2.id.clone(),
                        ttc: spec.min_ttc,
                    })
                    .always();
                    constraints.push(ttc);
                }
                ConstraintMode::Violate => {
                    let ttc_violation = LTLFormula::Atom(Proposition::TTCGT {
                        actor1: actor1.id.clone(),
                        actor2: actor2.id.clone(),
                        ttc: spec.min_ttc,
                    })
                    .negate()
                    .eventually();
                    constraints.push(ttc_violation);
                }
                ConstraintMode::Ignore => {}
            }

            // Distance constraint
            match spec.constraint_modes.min_distance() {
                ConstraintMode::Enforce => {
                    let dist = LTLFormula::Atom(Proposition::DistanceGT {
                        actor1: actor1.id.clone(),
                        actor2: actor2.id.clone(),
                        distance: spec.min_distance,
                    })
                    .always();
                    constraints.push(dist);
                }
                ConstraintMode::Violate => {
                    let dist_violation = LTLFormula::Atom(Proposition::DistanceGT {
                        actor1: actor1.id.clone(),
                        actor2: actor2.id.clone(),
                        distance: spec.min_distance,
                    })
                    .negate()
                    .eventually();
                    constraints.push(dist_violation);
                }
                ConstraintMode::Ignore => {}
            }

            // Lateral distance constraint
            if let Some(min_lat_dist) = spec.min_lateral_distance {
                match spec.constraint_modes.min_lateral_distance() {
                    ConstraintMode::Enforce => {
                        let lat_dist = LTLFormula::Atom(Proposition::LateralDistanceGT {
                            actor1: actor1.id.clone(),
                            actor2: actor2.id.clone(),
                            distance: min_lat_dist,
                        })
                        .always();
                        constraints.push(lat_dist);
                    }
                    ConstraintMode::Violate => {
                        let lat_dist_violation = LTLFormula::Atom(Proposition::LateralDistanceGT {
                            actor1: actor1.id.clone(),
                            actor2: actor2.id.clone(),
                            distance: min_lat_dist,
                        })
                        .negate()
                        .eventually();
                        constraints.push(lat_dist_violation);
                    }
                    ConstraintMode::Ignore => {}
                }
            }

            // Relative velocity constraint (note: we negate it for "enforce" mode)
            // Enforce means: |vx1 - vx2| <= max_relative_velocity
            // Which is: NOT (|vx1 - vx2| > max_relative_velocity)
            if let Some(max_rel_vel) = spec.max_relative_velocity {
                match spec.constraint_modes.max_relative_velocity() {
                    ConstraintMode::Enforce => {
                        let rel_vel = LTLFormula::Atom(Proposition::RelativeVelocityGT {
                            actor1: actor1.id.clone(),
                            actor2: actor2.id.clone(),
                            velocity: max_rel_vel,
                        })
                        .negate()
                        .always();
                        constraints.push(rel_vel);
                    }
                    ConstraintMode::Violate => {
                        let rel_vel_violation = LTLFormula::Atom(Proposition::RelativeVelocityGT {
                            actor1: actor1.id.clone(),
                            actor2: actor2.id.clone(),
                            velocity: max_rel_vel,
                        })
                        .eventually();
                        constraints.push(rel_vel_violation);
                    }
                    ConstraintMode::Ignore => {}
                }
            }
        }
    }

    // Generate per-actor velocity constraints
    for actor in &spec.actors {
        // Max velocity constraint
        if let Some(max_vel) = spec.max_velocity {
            match spec.constraint_modes.max_velocity() {
                ConstraintMode::Enforce => {
                    let vel = LTLFormula::Atom(Proposition::VelocityLT {
                        actor: actor.id.clone(),
                        velocity: max_vel,
                    })
                    .always();
                    constraints.push(vel);
                }
                ConstraintMode::Violate => {
                    let vel_violation = LTLFormula::Atom(Proposition::VelocityLT {
                        actor: actor.id.clone(),
                        velocity: max_vel,
                    })
                    .negate()
                    .eventually();
                    constraints.push(vel_violation);
                }
                ConstraintMode::Ignore => {}
            }
        }

        // Min velocity constraint
        if let Some(min_vel) = spec.min_velocity {
            match spec.constraint_modes.min_velocity() {
                ConstraintMode::Enforce => {
                    let vel = LTLFormula::Atom(Proposition::VelocityGT {
                        actor: actor.id.clone(),
                        velocity: min_vel,
                    })
                    .always();
                    constraints.push(vel);
                }
                ConstraintMode::Violate => {
                    let vel_violation = LTLFormula::Atom(Proposition::VelocityGT {
                        actor: actor.id.clone(),
                        velocity: min_vel,
                    })
                    .negate()
                    .eventually();
                    constraints.push(vel_violation);
                }
                ConstraintMode::Ignore => {}
            }
        }
    }

    LTLFormula::conjunction(constraints)
}

pub(crate) mod cut_in_left;
pub(crate) mod cut_in_right;
pub(crate) mod head_on;
pub(crate) mod overtake_left;
pub(crate) mod pedestrian_crossing;
