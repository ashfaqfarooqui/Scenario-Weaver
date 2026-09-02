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
        _encoder: &dyn crate::solver::EncoderAccessor,
        _backend: &dyn crate::solver::Z3Backend,
        _horizon: usize,
    ) -> Result<()> {
        Ok(())
    }
}

/// Whether a safety atom already states the *safe* condition, or states the
/// *unsafe* one (so the safe condition is its negation).
///
/// SW-21/item 3. Every safety atom below reduces to the same Enforce/Violate/
/// Ignore triple — `Enforce` keeps the safe formula, `Violate` demands its
/// negation eventually hold, `Ignore` adds nothing — except which of "the
/// atom" or "its negation" *is* the safe formula differs by atom. `TTCGT`,
/// `DistanceGT`, `LateralDistanceGT`, `VelocityLT` and `VelocityGT` all name
/// the safe condition directly (e.g. "TTC is greater than the minimum" is
/// safe). `RelativeVelocityGT` instead names the unsafe one — staying under
/// the limit is `NOT (relative velocity > max)` — so its polarity is
/// flipped. This type makes that one difference explicit at each call site
/// instead of six near-identical `match`es.
#[derive(Clone, Copy)]
enum AtomPolarity {
    /// The atom itself is the safe formula.
    Positive,
    /// The atom's negation is the safe formula.
    Negated,
}

/// The one real implementation behind the six per-constraint `match`es that
/// used to be copy-pasted through this function (`generate_default_safety`),
/// each varying only in which `Proposition` it built and whether the safe
/// condition was the atom or its negation.
fn push_constraint(
    constraints: &mut Vec<LTLFormula>,
    mode: ConstraintMode,
    atom: LTLFormula,
    polarity: AtomPolarity,
) {
    let formula = match (mode, polarity) {
        (ConstraintMode::Enforce, AtomPolarity::Positive) => atom.always(),
        (ConstraintMode::Enforce, AtomPolarity::Negated) => atom.negate().always(),
        (ConstraintMode::Violate, AtomPolarity::Positive) => atom.negate().eventually(),
        (ConstraintMode::Violate, AtomPolarity::Negated) => atom.eventually(),
        (ConstraintMode::Ignore, _) => return,
    };
    constraints.push(formula);
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
            push_constraint(
                &mut constraints,
                spec.constraint_modes.min_ttc(),
                LTLFormula::Atom(Proposition::TTCGT {
                    actor1: actor1.id.clone(),
                    actor2: actor2.id.clone(),
                    ttc: spec.min_ttc,
                }),
                AtomPolarity::Positive,
            );

            push_constraint(
                &mut constraints,
                spec.constraint_modes.min_distance(),
                LTLFormula::Atom(Proposition::DistanceGT {
                    actor1: actor1.id.clone(),
                    actor2: actor2.id.clone(),
                    distance: spec.min_distance,
                }),
                AtomPolarity::Positive,
            );

            if let Some(min_lat_dist) = spec.min_lateral_distance {
                push_constraint(
                    &mut constraints,
                    spec.constraint_modes.min_lateral_distance(),
                    LTLFormula::Atom(Proposition::LateralDistanceGT {
                        actor1: actor1.id.clone(),
                        actor2: actor2.id.clone(),
                        distance: min_lat_dist,
                    }),
                    AtomPolarity::Positive,
                );
            }

            // RelativeVelocityGT names the *unsafe* condition — staying under
            // the limit is `NOT (|vx1 - vx2| > max_relative_velocity)` — so
            // this one is `Negated`, not `Positive`.
            if let Some(max_rel_vel) = spec.max_relative_velocity {
                push_constraint(
                    &mut constraints,
                    spec.constraint_modes.max_relative_velocity(),
                    LTLFormula::Atom(Proposition::RelativeVelocityGT {
                        actor1: actor1.id.clone(),
                        actor2: actor2.id.clone(),
                        velocity: max_rel_vel,
                    }),
                    AtomPolarity::Negated,
                );
            }
        }
    }

    // Generate per-actor velocity constraints
    for actor in &spec.actors {
        if let Some(max_vel) = spec.max_velocity {
            push_constraint(
                &mut constraints,
                spec.constraint_modes.max_velocity(),
                LTLFormula::Atom(Proposition::VelocityLT {
                    actor: actor.id.clone(),
                    velocity: max_vel,
                }),
                AtomPolarity::Positive,
            );
        }

        if let Some(min_vel) = spec.min_velocity {
            push_constraint(
                &mut constraints,
                spec.constraint_modes.min_velocity(),
                LTLFormula::Atom(Proposition::VelocityGT {
                    actor: actor.id.clone(),
                    velocity: min_vel,
                }),
                AtomPolarity::Positive,
            );
        }
    }

    LTLFormula::conjunction(constraints)
}

pub(crate) mod cut_in_left;
pub(crate) mod cut_in_right;
pub(crate) mod head_on;
pub(crate) mod overtake_left;
pub(crate) mod pedestrian_crossing;
