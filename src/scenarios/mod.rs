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

/// The cut-in's *conflict*: while the NPC and the ego share the target lane, the NPC is
/// in front and the ego is gaining on it.
///
/// **Why a scenario model asserts anything about closing at all.** An `enforce`d
/// `min_ttc` lowers to `G(TTCGT(ego, npc, T))`, and `TTCGT` is a *guarded implication* —
/// "whenever this pair is converging, its TTC exceeds `T`". Nothing required a pair to
/// converge, so Z3 was free to answer with traffic that never does, and
/// `compute_validation_metrics` then reported `min_ttc: null`, "never evaluated" — every
/// `cut_in_left` and `cut_in_right` example in the corpus was affected, and no other
/// scenario type was.
///
/// The trajectories say why. In `cut_in_left` the ego *overtook* the NPC in the adjacent
/// lane and the NPC then merged in **behind** it, slower and finally stationary — a lane
/// change into empty road, with no conflict anywhere in it. That is a defect in the
/// scenario model, not in the TTC bound: `cut_in_left.rs`'s own module doc says the NPC
/// "changes lanes to cut in front of the ego vehicle", and only the *initial*
/// `Ahead(npc, ego)` ever said so.
///
/// **Why this shape and not `F(approaching)`.** An existential — a `Closing`
/// proposition under `F(⋁ over pairs)` — works but is unaffordable: `cut_in_left` went
/// from 4 s to 12 s for one scenario, and >500 s for the five it declares. A disjunction
/// over the horizon asks Z3 to *search* for the instant. Here the antecedent is one the
/// template already forces — `cut_in_behavior`'s `Until` makes `InLane(npc, target_lane)`
/// true at some step, and the ego, which has no lane changes, holds its lane — so
/// `G(antecedent → Approaching)` cannot be satisfied vacuously, and every conjunct is an
/// implication Z3 *propagates* rather than a disjunct it chooses between. That is the
/// same speed-bucket lesson (guards that partition, not selectors Z3 picks) applied to
/// the conflict itself.
///
/// **Both lane atoms are in the antecedent on purpose.** They are `Int` equalities, so as
/// hypotheses they are free, and together they imply `encode_same_lane_constraint`'s
/// discrete-match disjunct — which is what makes the validator measure a TTC here. Moving
/// either into the consequent turns it into something Z3 must *satisfy*: with the full
/// same-lane predicate as a consequent, `cut_in_left`'s five scenarios took 103 s against
/// a 15.0 s pre-fix baseline; this way they take 14.1 s, i.e. the conflict is free.
/// The price is a residual vacuity hole — an ego that left its lane would make the
/// antecedent false — which no cut-in spec can reach today, since the templates give the
/// ego no `lane_changes`, but which a future spec that did would silently re-open.
///
/// **Two cases add nothing, and both are deliberate.**
///
/// *The NPC does not end up in the ego's lane* (`target_lane != ego_lane`): then "cuts in
/// front of the ego" has no referent, and asserting a conflict under a lane the ego is not
/// in would be unsatisfiable rather than merely inapplicable. Such a spec leaves an
/// `enforce`d `min_ttc` vacuous — the two actors never share a lane, so no TTC is ever
/// defined — and nothing here can change that; it is a mis-specified cut-in, and
/// `tests/bidirectional_test.rs::test_backward_lane_velocity` is one.
///
/// *The two actors travel in opposite directions*: then the ego is not following the NPC
/// at all, it is meeting it, and the conflict is **structural** — an oncoming pair
/// approaches in every model there is, which is exactly why `head_on_near_miss.yaml` has
/// never needed help measuring a TTC. `initial_conditions` in both cut-in models already
/// skips its `Ahead` atom for the same reason and says so. Asserting the following
/// relation anyway makes those specs UNSAT: the ego and an oncoming NPC sharing a lane
/// must cross, and after crossing `px_npc > px_ego` is false, so the NPC would have to
/// leave the lane it was just required to merge into. Verified —
/// `test_lane_direction_consistency` and `test_narrow_rural_road` both went UNSAT before
/// this guard.
///
/// `direction` is the shared travel direction of the pair, `+1` or `-1`. The comparisons
/// are made in the raw `+x` frame, matching `compute_validation_metrics`, so for a pair
/// travelling in `-x` the *physically* following ego is the one at the larger `x` and the
/// roles swap.
fn cut_in_conflict(
    ego_id: &str,
    npc_id: &str,
    ego_lane: usize,
    target_lane: usize,
    direction: i32,
) -> LTLFormula {
    if target_lane != ego_lane {
        return LTLFormula::True;
    }

    let both_in_lane = LTLFormula::Atom(Proposition::InLane {
        actor: npc_id.to_string(),
        lane: target_lane,
    })
    .and(LTLFormula::Atom(Proposition::InLane {
        actor: ego_id.to_string(),
        lane: ego_lane,
    }));

    let (follower, leader) = if direction >= 0 {
        (ego_id, npc_id)
    } else {
        (npc_id, ego_id)
    };

    both_in_lane
        .implies(LTLFormula::Atom(Proposition::Approaching {
            follower: follower.to_string(),
            leader: leader.to_string(),
        }))
        .always()
}

/// Whether a safety atom already states the *safe* condition, or states the
/// *unsafe* one (so the safe condition is its negation).
///
/// Every safety atom below reduces to the same Enforce/Violate/
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
