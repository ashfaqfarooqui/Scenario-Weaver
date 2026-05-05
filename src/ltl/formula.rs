//! LTL (Linear Temporal Logic) formula AST
//!
//! Formulas are expanded into Z3 constraints via bounded model checking:
//! temporal operators become conjunctions/disjunctions over discrete time steps.

use std::fmt;

/// AST node for an LTL formula used in bounded model checking.
///
/// Temporal operators (`Always`, `Eventually`, `Until`, `Next`) are expanded
/// over a finite time horizon during Z3 encoding.
#[derive(Debug, Clone, PartialEq)]
pub enum LTLFormula {
    /// Atomic proposition about scenario state at a single time step.
    Atom(Proposition),

    /// Logical negation.
    Not(Box<LTLFormula>),
    /// Logical conjunction.
    And(Box<LTLFormula>, Box<LTLFormula>),
    /// Logical disjunction.
    Or(Box<LTLFormula>, Box<LTLFormula>),
    /// Material implication (phi -> psi).
    Implies(Box<LTLFormula>, Box<LTLFormula>),

    /// Next (X phi) -- holds at the immediately following time step.
    Next(Box<LTLFormula>),
    /// Eventually (F phi) -- holds at some future time step.
    Eventually(Box<LTLFormula>),
    /// Always (G phi) -- holds at all remaining time steps.
    Always(Box<LTLFormula>),
    /// Until (phi U psi) -- phi holds until psi becomes true.
    Until(Box<LTLFormula>, Box<LTLFormula>),
}

/// Atomic propositions about scenario state
#[derive(Debug, Clone, PartialEq)]
pub enum Proposition {
    /// Actor is in a specific lane
    InLane { actor: String, lane: usize },

    /// Actor1 is ahead of Actor2 (longitudinally)
    Ahead { actor1: String, actor2: String },

    /// Longitudinal distance between actors > threshold
    DistanceGT {
        actor1: String,
        actor2: String,
        distance: f64,
    },

    /// Time-To-Collision between actors > threshold
    TTCGT {
        actor1: String,
        actor2: String,
        ttc: f64,
    },

    /// Pedestrian is on a specific side of the road ("left" or "right")
    OnSidewalk { actor: String, side: String },

    /// Pedestrian is actively crossing the road
    CrossingRoad { actor: String },

    /// 2D Euclidean distance between actors > threshold
    /// For pedestrian-vehicle scenarios where same-lane assumption doesn't apply
    /// WARNING: Creates nonlinear (quadratic) constraints - use ManhattanDistanceGT for linear alternative
    Distance2DGT {
        actor1: String,
        actor2: String,
        distance: f64,
    },

    /// Manhattan distance between actors > threshold
    /// Linear alternative to Distance2DGT: |dx| + |dy| > threshold
    /// For pedestrian-vehicle scenarios needing fast Z3 solving
    ManhattanDistanceGT {
        actor1: String,
        actor2: String,
        distance: f64,
    },

    /// Rectangular safety box: |dx| > threshold_x OR |dy| > threshold_y
    /// Simplest linear distance constraint - at least one dimension must exceed threshold
    /// Very fast Z3 solving, conservative safety
    RectangularDistanceGT {
        actor1: String,
        actor2: String,
        threshold_x: f64,
        threshold_y: f64,
    },

    /// Time-to-collision for perpendicular crossing
    /// Checks if ego will reach pedestrian's crossing point before pedestrian clears
    PedestrianTTCGT {
        ego: String,
        pedestrian: String,
        ttc: f64,
    },

    /// Actor's longitudinal speed exceeds threshold (linear constraint: |vx| > velocity)
    /// Uses absolute value of longitudinal velocity (vx), not vector magnitude.
    /// Matches YAML "speed" semantics which sets vx (signed by lane direction).
    VelocityGT { actor: String, velocity: f64 },

    /// Actor's longitudinal speed is below threshold (linear constraint: |vx| < velocity)
    /// Uses absolute value of longitudinal velocity (vx), not vector magnitude.
    /// Matches YAML "speed" semantics which sets vx (signed by lane direction).
    VelocityLT { actor: String, velocity: f64 },

    /// Lateral (perpendicular) distance between actors exceeds threshold
    /// Linear constraint: |py1 - py2| > distance
    LateralDistanceGT {
        actor1: String,
        actor2: String,
        distance: f64,
    },

    /// Actor1 is laterally left of Actor2 (py1 > py2)
    OnLeftOf { actor1: String, actor2: String },

    /// Actor1 is laterally right of Actor2 (py1 < py2)
    OnRightOf { actor1: String, actor2: String },

    /// Relative longitudinal velocity exceeds threshold
    /// Linear constraint: |vx1 - vx2| > velocity
    RelativeVelocityGT {
        actor1: String,
        actor2: String,
        velocity: f64,
    },
}

// Builder methods for ergonomic formula construction
impl LTLFormula {
    /// Logical AND
    pub fn and(self, other: Self) -> Self {
        LTLFormula::And(Box::new(self), Box::new(other))
    }

    /// Logical OR
    pub fn or(self, other: Self) -> Self {
        LTLFormula::Or(Box::new(self), Box::new(other))
    }

    /// Logical NOT
    pub fn negate(self) -> Self {
        LTLFormula::Not(Box::new(self))
    }

    /// Implication
    pub fn implies(self, other: Self) -> Self {
        LTLFormula::Implies(Box::new(self), Box::new(other))
    }

    /// Eventually (F φ) - will be true at some future point
    pub fn eventually(self) -> Self {
        LTLFormula::Eventually(Box::new(self))
    }

    /// Always (G φ) - will be true at all future points
    pub fn always(self) -> Self {
        LTLFormula::Always(Box::new(self))
    }

    /// Until (φ U ψ) - φ holds until ψ becomes true
    pub fn until(self, other: Self) -> Self {
        LTLFormula::Until(Box::new(self), Box::new(other))
    }

    /// Next (X φ) - true in next time step
    pub fn next(self) -> Self {
        LTLFormula::Next(Box::new(self))
    }
}

/// Display implementation for debugging
impl fmt::Display for LTLFormula {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            LTLFormula::Atom(p) => write!(f, "{:?}", p),
            LTLFormula::Not(phi) => write!(f, "¬({})", phi),
            LTLFormula::And(phi, psi) => write!(f, "({} ∧ {})", phi, psi),
            LTLFormula::Or(phi, psi) => write!(f, "({} ∨ {})", phi, psi),
            LTLFormula::Implies(phi, psi) => write!(f, "({} → {})", phi, psi),
            LTLFormula::Next(phi) => write!(f, "X({})", phi),
            LTLFormula::Eventually(phi) => write!(f, "F({})", phi),
            LTLFormula::Always(phi) => write!(f, "G({})", phi),
            LTLFormula::Until(phi, psi) => write!(f, "({} U {})", phi, psi),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_builder_methods() {
        let p1 = LTLFormula::Atom(Proposition::InLane {
            actor: "ego".to_string(),
            lane: 1,
        });
        let p2 = LTLFormula::Atom(Proposition::InLane {
            actor: "npc".to_string(),
            lane: 0,
        });

        // Test and
        let formula = p1.clone().and(p2.clone());
        assert!(matches!(formula, LTLFormula::And(_, _)));

        // Test eventually
        let formula = p1.clone().eventually();
        assert!(matches!(formula, LTLFormula::Eventually(_)));

        // Test always
        let formula = p1.clone().always();
        assert!(matches!(formula, LTLFormula::Always(_)));

        // Test complex formula
        let formula = p1.clone().until(p2.clone()).eventually();
        println!("Formula: {}", formula);
    }

    #[test]
    fn test_display() {
        let formula = LTLFormula::Atom(Proposition::InLane {
            actor: "ego".to_string(),
            lane: 1,
        })
        .eventually();

        let display = format!("{}", formula);
        assert!(display.contains("F("));
        println!("Display: {}", display);
    }
}
