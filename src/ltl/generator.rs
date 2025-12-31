//! LTL formula generation from DSL specifications

use crate::dsl::types::{ConstraintMode, ScenarioSpec, ScenarioType};
use crate::ltl::formula::{LTLFormula, Proposition};

pub struct LTLGenerator;

impl LTLGenerator {
    /// Generate LTL formula from scenario specification
    pub fn generate(spec: &ScenarioSpec) -> LTLFormula {
        match spec.scenario_type {
            ScenarioType::CutInLeft => Self::generate_cut_in_left(spec),
        }
    }

    /// Generate LTL formula for cut-in from left scenario
    ///
    /// Formula structure:
    /// φ = φ_init ∧ φ_behavior ∧ φ_safety
    ///
    /// Where:
    /// - φ_init: Initial conditions (lanes, positions)
    /// - φ_behavior: Cut-in behavior (eventually changes lanes, stays left until change)
    /// - φ_safety: Safety constraints (always maintain TTC and distance)
    pub fn generate_cut_in_left(spec: &ScenarioSpec) -> LTLFormula {
        let ego = "ego";
        let npc = "npc";

        Self::initial_conditions(spec, ego, npc)
            .and(Self::cut_in_behavior(spec, ego, npc))
            .and(Self::safety_constraints(spec, ego, npc))
    }

    /// Initial conditions for cut-in scenario
    ///
    /// At t=0:
    /// - Ego in right lane (lane 1)
    /// - NPC in left lane (lane 0)
    /// - NPC ahead of ego
    fn initial_conditions(spec: &ScenarioSpec, ego: &str, npc: &str) -> LTLFormula {
        LTLFormula::Atom(Proposition::InLane {
            actor: ego.to_string(),
            lane: spec.ego.lane,
        })
        .and(LTLFormula::Atom(Proposition::InLane {
            actor: npc.to_string(),
            lane: spec.npc.lane,
        }))
        .and(LTLFormula::Atom(Proposition::Ahead {
            actor1: npc.to_string(),
            actor2: ego.to_string(),
        }))
    }

    /// Cut-in behavior
    ///
    /// - Eventually: NPC moves to ego's lane
    /// - Until: NPC stays in left lane until it changes
    fn cut_in_behavior(spec: &ScenarioSpec, _ego: &str, npc: &str) -> LTLFormula {
        let target_lane = spec.ego.lane;
        let initial_lane = spec.npc.lane;

        // Eventually NPC moves to ego's lane: F(InLane(npc, 1))
        let eventually_in_lane = LTLFormula::Atom(Proposition::InLane {
            actor: npc.to_string(),
            lane: target_lane,
        })
        .eventually();

        // NPC stays in left lane UNTIL it changes: InLane(npc, 0) U InLane(npc, 1)
        let stay_until_change = LTLFormula::Atom(Proposition::InLane {
            actor: npc.to_string(),
            lane: initial_lane,
        })
        .until(LTLFormula::Atom(Proposition::InLane {
            actor: npc.to_string(),
            lane: target_lane,
        }));

        eventually_in_lane.and(stay_until_change)
    }

    /// Safety constraints with configurable modes
    ///
    /// - Enforce: G(constraint) - always maintain safety
    /// - Violate: F(NOT constraint) - eventually violate safety
    /// - Ignore: constraint not added
    fn safety_constraints(spec: &ScenarioSpec, ego: &str, npc: &str) -> LTLFormula {
        let mut constraints = Vec::new();

        // TTC constraint
        match spec.constraint_modes.min_ttc() {
            ConstraintMode::Enforce => {
                // G(TTC > min_ttc)
                let ttc_constraint = LTLFormula::Atom(Proposition::TTCGT {
                    actor1: ego.to_string(),
                    actor2: npc.to_string(),
                    ttc: spec.min_ttc,
                })
                .always();
                constraints.push(ttc_constraint);
            }
            ConstraintMode::Violate => {
                // F(NOT (TTC > min_ttc))
                let ttc_violation = LTLFormula::Atom(Proposition::TTCGT {
                    actor1: ego.to_string(),
                    actor2: npc.to_string(),
                    ttc: spec.min_ttc,
                })
                .negate()
                .eventually();
                constraints.push(ttc_violation);
            }
            ConstraintMode::Ignore => {
                // Don't add any constraint
            }
        }

        // Distance constraint
        match spec.constraint_modes.min_distance() {
            ConstraintMode::Enforce => {
                // G(Distance > min_distance)
                let distance_constraint = LTLFormula::Atom(Proposition::DistanceGT {
                    actor1: ego.to_string(),
                    actor2: npc.to_string(),
                    distance: spec.min_distance,
                })
                .always();
                constraints.push(distance_constraint);
            }
            ConstraintMode::Violate => {
                // F(NOT (Distance > min_distance))
                let distance_violation = LTLFormula::Atom(Proposition::DistanceGT {
                    actor1: ego.to_string(),
                    actor2: npc.to_string(),
                    distance: spec.min_distance,
                })
                .negate()
                .eventually();
                constraints.push(distance_violation);
            }
            ConstraintMode::Ignore => {
                // Don't add any constraint
            }
        }

        // Combine all constraints with AND
        if constraints.is_empty() {
            // Return a tautology (always true)
            LTLFormula::Atom(Proposition::InLane {
                actor: ego.to_string(),
                lane: spec.ego.lane,
            })
            .or(LTLFormula::Atom(Proposition::InLane {
                actor: ego.to_string(),
                lane: spec.ego.lane,
            })
            .negate())
        } else {
            constraints
                .into_iter()
                .reduce(|acc, c| acc.and(c))
                .unwrap()
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dsl::types::{ActorSpec, NpcSpec, ValueOrRange};

    fn create_test_spec() -> ScenarioSpec {
        ScenarioSpec {
            scenario_type: ScenarioType::CutInLeft,
            time_step: 0.5,
            duration: 10.0,
            ego: ActorSpec {
                lane: 1,
                position: ValueOrRange::Value(50.0),
                speed: ValueOrRange::Value(15.0),
            },
            npc: NpcSpec {
                lane: 0,
                position: ValueOrRange::Range([60.0, 80.0]),
                speed: ValueOrRange::Range([12.0, 14.0]),
                cut_in_time: ValueOrRange::Range([2.5, 7.5]),
            },
            min_ttc: 3.0,
            min_distance: 5.0,
            lane_width: 3.5,
            num_scenarios: 1,
            constraint_modes: crate::dsl::types::ConstraintModes::default(),
        }
    }

    #[test]
    fn test_generate_cut_in_left() {
        let spec = create_test_spec();
        let formula = LTLGenerator::generate_cut_in_left(&spec);

        println!("Generated LTL formula:");
        println!("{}", formula);

        // Should be a conjunction (AND)
        assert!(matches!(formula, LTLFormula::And(_, _)));
    }

    #[test]
    fn test_initial_conditions() {
        let spec = create_test_spec();
        let formula = LTLGenerator::initial_conditions(&spec, "ego", "npc");

        println!("Initial conditions:");
        println!("{}", formula);

        // Should contain InLane propositions
        let formula_str = format!("{}", formula);
        assert!(formula_str.contains("InLane"));
        assert!(formula_str.contains("Ahead"));
    }

    #[test]
    fn test_cut_in_behavior() {
        let spec = create_test_spec();
        let formula = LTLGenerator::cut_in_behavior(&spec, "ego", "npc");

        println!("Cut-in behavior:");
        println!("{}", formula);

        // Should contain Eventually and Until
        let formula_str = format!("{}", formula);
        assert!(formula_str.contains("F("));
        assert!(formula_str.contains(" U "));
    }

    #[test]
    fn test_safety_constraints() {
        let spec = create_test_spec();
        let formula = LTLGenerator::safety_constraints(&spec, "ego", "npc");

        println!("Safety constraints:");
        println!("{}", formula);

        // Should contain Always operators
        let formula_str = format!("{}", formula);
        assert!(formula_str.contains("G("));
        assert!(formula_str.contains("TTCGT"));
        assert!(formula_str.contains("DistanceGT"));
    }
}
