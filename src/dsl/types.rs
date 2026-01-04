//! DSL data structures for scenario specification

use serde::{Deserialize, Serialize};

/// Constraint enforcement mode
#[derive(Debug, Clone, Copy, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum ConstraintMode {
    /// Enforce constraint: G(constraint) - must hold at all times
    Enforce,
    /// Violate constraint: F(NOT constraint) - must be violated at some point
    Violate,
    /// Ignore constraint: not added to the formula
    Ignore,
}

impl Default for ConstraintMode {
    fn default() -> Self {
        ConstraintMode::Enforce
    }
}

/// Configuration for how constraints should be enforced
#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(untagged)]
pub enum ConstraintModes {
    /// Detailed per-constraint configuration
    Detailed {
        #[serde(default)]
        min_ttc: ConstraintMode,
        #[serde(default)]
        min_distance: ConstraintMode,
        #[serde(default)]
        max_acceleration: ConstraintMode,
    },
    /// Shorthand: "violate_all", "ignore_all", "enforce_all"
    Shorthand(String),
}

impl Default for ConstraintModes {
    fn default() -> Self {
        ConstraintModes::Detailed {
            min_ttc: ConstraintMode::Enforce,
            min_distance: ConstraintMode::Enforce,
            max_acceleration: ConstraintMode::Enforce,
        }
    }
}

impl ConstraintModes {
    /// Get the mode for min_ttc constraint
    pub fn min_ttc(&self) -> ConstraintMode {
        match self {
            ConstraintModes::Detailed { min_ttc, .. } => *min_ttc,
            ConstraintModes::Shorthand(s) => match s.as_str() {
                "violate_all" => ConstraintMode::Violate,
                "ignore_all" => ConstraintMode::Ignore,
                _ => ConstraintMode::Enforce,
            },
        }
    }

    /// Get the mode for min_distance constraint
    pub fn min_distance(&self) -> ConstraintMode {
        match self {
            ConstraintModes::Detailed { min_distance, .. } => *min_distance,
            ConstraintModes::Shorthand(s) => match s.as_str() {
                "violate_all" => ConstraintMode::Violate,
                "ignore_all" => ConstraintMode::Ignore,
                _ => ConstraintMode::Enforce,
            },
        }
    }

    /// Get the mode for max_acceleration constraint
    pub fn max_acceleration(&self) -> ConstraintMode {
        match self {
            ConstraintModes::Detailed { max_acceleration, .. } => *max_acceleration,
            ConstraintModes::Shorthand(s) => match s.as_str() {
                "violate_all" => ConstraintMode::Violate,
                "ignore_all" => ConstraintMode::Ignore,
                _ => ConstraintMode::Enforce,
            },
        }
    }
}

/// Actor role (ego or NPC)
#[derive(Debug, Clone, Copy, Deserialize, Serialize, PartialEq, Eq)]
pub enum ActorRole {
    #[serde(rename = "ego")]
    Ego,
    #[serde(rename = "npc")]
    Npc,
}

#[derive(Debug, Clone, Copy, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum ScenarioType {
    CutInLeft,
    CutInRight,
}

impl std::fmt::Display for ScenarioType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ScenarioType::CutInLeft => write!(f, "cut_in_left"),
            ScenarioType::CutInRight => write!(f, "cut_in_right"),
        }
    }
}

impl ScenarioType {
    /// Get the scenario model for this scenario type
    pub fn get_model(&self) -> Box<dyn crate::scenarios::ScenarioModel> {
        match self {
            ScenarioType::CutInLeft => {
                Box::new(crate::scenarios::cut_in_left::CutInLeftModel)
            }
            ScenarioType::CutInRight => {
                Box::new(crate::scenarios::cut_in_right::CutInRightModel)
            }
        }
    }
}

/// Generic actor specification (supports both ego and NPCs)
#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct ActorSpec {
    pub id: String,
    pub role: ActorRole,
    pub lane: usize,
    pub position: ValueOrRange,
    pub speed: ValueOrRange,
    pub acceleration: ValueOrRange,
    /// Scenario-specific behavior parameters
    #[serde(default)]
    pub behavior: std::collections::HashMap<String, serde_json::Value>,
}

/// Root scenario specification
#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct ScenarioSpec {
    pub scenario_type: ScenarioType,
    pub time_step: f64, // seconds per discretization step
    pub duration: f64,  // total scenario duration (seconds)
    pub actors: Vec<ActorSpec>,
    pub min_ttc: f64,         // minimum time-to-collision (seconds)
    pub min_distance: f64,    // minimum longitudinal distance (meters)
    pub lane_width: f64,      // meters
    pub num_scenarios: usize, // 1 for single, N for multiple
    /// Constraint enforcement modes (optional, defaults to enforce_all)
    #[serde(default)]
    pub constraint_modes: ConstraintModes,
    /// Optional global maximum acceleration constraint (m/s²)
    #[serde(default)]
    pub max_acceleration: Option<f64>,
    /// Optional global maximum deceleration constraint (m/s², should be negative)
    #[serde(default)]
    pub max_deceleration: Option<f64>,
}

/// Value that can be either fixed or a range for Z3 to solve
#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(untagged)]
pub enum ValueOrRange {
    Value(f64),
    Range([f64; 2]), // [min, max]
}

impl ValueOrRange {
    /// Get minimum value
    pub fn min(&self) -> f64 {
        match self {
            ValueOrRange::Value(v) => *v,
            ValueOrRange::Range([min, _]) => *min,
        }
    }

    /// Get maximum value
    pub fn max(&self) -> f64 {
        match self {
            ValueOrRange::Value(v) => *v,
            ValueOrRange::Range([_, max]) => *max,
        }
    }

    /// Check if this is a fixed value
    pub fn is_fixed(&self) -> bool {
        matches!(self, ValueOrRange::Value(_))
    }
}

impl ScenarioSpec {
    /// Get the ego actor (expects exactly one)
    pub fn ego(&self) -> Result<&ActorSpec, String> {
        self.actors
            .iter()
            .find(|a| a.role == ActorRole::Ego)
            .ok_or_else(|| "No ego actor found".to_string())
    }

    /// Get all NPC actors
    pub fn npcs(&self) -> Vec<&ActorSpec> {
        self.actors
            .iter()
            .filter(|a| a.role == ActorRole::Npc)
            .collect()
    }

    /// Get actor by ID
    pub fn get_actor(&self, id: &str) -> Option<&ActorSpec> {
        self.actors.iter().find(|a| a.id == id)
    }

    /// Validate the specification
    pub fn validate(&self) -> Result<(), String> {
        // Time parameters
        if self.time_step <= 0.0 {
            return Err("time_step must be positive".to_string());
        }
        if self.duration <= 0.0 {
            return Err("duration must be positive".to_string());
        }
        if self.duration < self.time_step {
            return Err("duration must be >= time_step".to_string());
        }

        // Safety constraints
        if self.min_ttc <= 0.0 {
            return Err("min_ttc must be positive".to_string());
        }
        if self.min_distance <= 0.0 {
            return Err("min_distance must be positive".to_string());
        }
        if self.lane_width <= 0.0 {
            return Err("lane_width must be positive".to_string());
        }

        // Generation parameters
        if self.num_scenarios == 0 {
            return Err("num_scenarios must be at least 1".to_string());
        }

        // NEW: Validate exactly one ego
        let ego_count = self.actors.iter().filter(|a| a.role == ActorRole::Ego).count();
        if ego_count != 1 {
            return Err(format!("Expected exactly 1 ego actor, found {}", ego_count));
        }

        // NEW: Validate at least one NPC
        let npc_count = self.actors.iter().filter(|a| a.role == ActorRole::Npc).count();
        if npc_count == 0 {
            return Err("At least one NPC actor required".to_string());
        }

        // NEW: Validate unique actor IDs
        let mut seen_ids = std::collections::HashSet::new();
        for actor in &self.actors {
            if !seen_ids.insert(&actor.id) {
                return Err(format!("Duplicate actor ID: {}", actor.id));
            }
        }

        // NEW: Validate all actor parameters
        for actor in &self.actors {
            if actor.speed.min() <= 0.0 {
                return Err(format!("{} speed must be positive", actor.id));
            }
            if actor.acceleration.min() >= actor.acceleration.max() {
                return Err(format!("{} acceleration range invalid", actor.id));
            }
            if let ValueOrRange::Range([min, max]) = actor.position {
                if min >= max {
                    return Err(format!("{} position range invalid: min >= max", actor.id));
                }
            }
            if let ValueOrRange::Range([min, max]) = actor.speed {
                if min >= max {
                    return Err(format!("{} speed range invalid: min >= max", actor.id));
                }
            }
        }

        // Validate acceleration ranges
        if let Some(max_accel) = self.max_acceleration {
            if max_accel <= 0.0 {
                return Err("max_acceleration must be positive".to_string());
            }
        }

        if let Some(max_decel) = self.max_deceleration {
            if max_decel >= 0.0 {
                return Err("max_deceleration must be negative".to_string());
            }
        }

        // Warn if violating constraints
        if self.constraint_modes.min_ttc() == ConstraintMode::Violate
            || self.constraint_modes.min_distance() == ConstraintMode::Violate
        {
            eprintln!("WARNING: Adversarial mode enabled - constraints will be violated");
        }

        Ok(())
    }

    /// Get the number of time steps in the scenario
    pub fn num_time_steps(&self) -> usize {
        (self.duration / self.time_step).ceil() as usize
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_value_or_range_fixed() {
        let val = ValueOrRange::Value(10.0);
        assert_eq!(val.min(), 10.0);
        assert_eq!(val.max(), 10.0);
        assert!(val.is_fixed());
    }

    #[test]
    fn test_value_or_range_range() {
        let val = ValueOrRange::Range([5.0, 15.0]);
        assert_eq!(val.min(), 5.0);
        assert_eq!(val.max(), 15.0);
        assert!(!val.is_fixed());
    }

    #[test]
    fn test_num_time_steps() {
        let spec = create_test_spec();
        assert_eq!(spec.num_time_steps(), 20); // 10.0 / 0.5 = 20
    }

    #[test]
    fn test_actor_spec_helpers() {
        let spec = create_test_spec();

        assert!(spec.ego().is_ok());
        assert_eq!(spec.ego().unwrap().id, "ego");
        assert_eq!(spec.npcs().len(), 1);
        assert_eq!(spec.npcs()[0].id, "npc");
        assert!(spec.get_actor("ego").is_some());
        assert!(spec.get_actor("npc").is_some());
        assert!(spec.get_actor("unknown").is_none());
    }

    fn create_test_spec() -> ScenarioSpec {
        use std::collections::HashMap;

        ScenarioSpec {
            scenario_type: ScenarioType::CutInLeft,
            time_step: 0.5,
            duration: 10.0,
            actors: vec![
                ActorSpec {
                    id: "ego".to_string(),
                    role: ActorRole::Ego,
                    lane: 1,
                    position: ValueOrRange::Value(50.0),
                    speed: ValueOrRange::Value(15.0),
                    acceleration: ValueOrRange::Range([-8.0, 3.0]),
                    behavior: HashMap::new(),
                },
                ActorSpec {
                    id: "npc".to_string(),
                    role: ActorRole::Npc,
                    lane: 0,
                    position: ValueOrRange::Range([60.0, 80.0]),
                    speed: ValueOrRange::Range([12.0, 14.0]),
                    acceleration: ValueOrRange::Range([-8.0, 3.0]),
                    behavior: {
                        let mut map = HashMap::new();
                        map.insert("cut_in_time".to_string(), serde_json::json!([2.5, 7.5]));
                        map
                    },
                },
            ],
            min_ttc: 3.0,
            min_distance: 5.0,
            lane_width: 3.5,
            num_scenarios: 1,
            constraint_modes: ConstraintModes::default(),
            max_acceleration: None,
            max_deceleration: None,
        }
    }
}
