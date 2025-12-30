//! DSL data structures for scenario specification

use serde::Deserialize;

/// Root scenario specification
#[derive(Debug, Clone, Deserialize)]
pub struct ScenarioSpec {
    pub scenario_type: ScenarioType,
    pub time_step: f64, // seconds per discretization step
    pub duration: f64,  // total scenario duration (seconds)
    pub ego: ActorSpec,
    pub npc: NpcSpec,
    pub min_ttc: f64,         // minimum time-to-collision (seconds)
    pub min_distance: f64,    // minimum longitudinal distance (meters)
    pub lane_width: f64,      // meters
    pub num_scenarios: usize, // 1 for single, N for multiple
}

#[derive(Debug, Clone, Copy, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum ScenarioType {
    CutInLeft,
}

impl std::fmt::Display for ScenarioType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ScenarioType::CutInLeft => write!(f, "cut_in_left"),
        }
    }
}

/// Ego vehicle specification (fixed parameters)
#[derive(Debug, Clone, Deserialize)]
pub struct ActorSpec {
    pub lane: usize,
    pub position: f64, // meters from start
    pub speed: f64,    // m/s
}

/// NPC vehicle specification (with ranges for solver)
#[derive(Debug, Clone, Deserialize)]
pub struct NpcSpec {
    pub lane: usize,
    pub position: ValueOrRange,    // starting position
    pub speed: ValueOrRange,       // velocity
    pub cut_in_time: ValueOrRange, // when to perform lane change (seconds)
}

/// Value that can be either fixed or a range for Z3 to solve
#[derive(Debug, Clone, Deserialize)]
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

        // Actor parameters
        if self.ego.speed <= 0.0 {
            return Err("ego speed must be positive".to_string());
        }
        if self.npc.speed.min() <= 0.0 {
            return Err("npc speed must be positive".to_string());
        }

        // Range validity
        if let ValueOrRange::Range([min, max]) = self.npc.position {
            if min >= max {
                return Err("npc position range invalid: min >= max".to_string());
            }
        }
        if let ValueOrRange::Range([min, max]) = self.npc.speed {
            if min >= max {
                return Err("npc speed range invalid: min >= max".to_string());
            }
        }
        if let ValueOrRange::Range([min, max]) = self.npc.cut_in_time {
            if min >= max {
                return Err("npc cut_in_time range invalid: min >= max".to_string());
            }
            if max > self.duration {
                return Err("npc cut_in_time max exceeds scenario duration".to_string());
            }
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

    fn create_test_spec() -> ScenarioSpec {
        ScenarioSpec {
            scenario_type: ScenarioType::CutInLeft,
            time_step: 0.5,
            duration: 10.0,
            ego: ActorSpec {
                lane: 1,
                position: 50.0,
                speed: 15.0,
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
        }
    }
}
