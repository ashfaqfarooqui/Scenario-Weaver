//! DSL data structures for scenario specification

use serde::{Deserialize, Serialize};

// Pedestrian physics constants
//
// NOTE: Max speeds are adjusted for linear box constraint (|vx| <= max AND |vy| <= max)
// instead of quadratic disk constraint (vx² + vy² <= max²). The box contains the disk,
// allowing diagonal speeds up to sqrt(2) * max. To maintain original semantic max speeds,
// we divide by sqrt(2). Result: diagonal movement matches original speed limits.
//
/// Maximum walking speed for pedestrians (m/s) - normal walk
/// Adjusted: 2.0 / sqrt(2) ≈ 1.41 m/s to maintain diagonal speed semantics with box constraint
pub const PEDESTRIAN_WALK_MAX_SPEED: f64 = 1.41;

/// Minimum walking speed for pedestrians (m/s)
pub const PEDESTRIAN_WALK_MIN_SPEED: f64 = 0.5;

/// Maximum running speed for pedestrians (m/s)
/// Adjusted: 5.0 / sqrt(2) ≈ 3.54 m/s to maintain diagonal speed semantics with box constraint
pub const PEDESTRIAN_RUN_MAX_SPEED: f64 = 3.54;

/// Minimum running speed for pedestrians (m/s)
pub const PEDESTRIAN_RUN_MIN_SPEED: f64 = 2.0;

/// Maximum acceleration for pedestrians (m/s²)
pub const PEDESTRIAN_MAX_ACCELERATION: f64 = 1.0;

/// Maximum deceleration for pedestrians (m/s²) - negative value
pub const PEDESTRIAN_MAX_DECELERATION: f64 = -1.0;

/// Constraint enforcement mode
#[derive(Debug, Clone, Copy, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
#[derive(Default)]
pub enum ConstraintMode {
    /// Enforce constraint: G(constraint) - must hold at all times
    #[default]
    Enforce,
    /// Violate constraint: F(NOT constraint) - must be violated at some point
    Violate,
    /// Ignore constraint: not added to the formula
    Ignore,
}

/// Optimization target for finding worst-case or best-case scenarios
#[derive(Debug, Clone, Copy, Deserialize, Serialize, PartialEq, Eq, Default)]
#[serde(rename_all = "snake_case")]
pub enum OptimizationTarget {
    /// No optimization - find any satisfying solution (default, backward compatible)
    #[default]
    None,
    /// Minimize TTC - find worst-case time-to-collision scenario
    MinimizeTtc,
    /// Minimize distance - find closest approach scenario
    MinimizeDistance,
    /// Minimize both TTC and distance (weighted combination)
    MinimizeSeverity,
    /// Maximize TTC - find safest scenario
    MaximizeTtc,
}

/// Coordinate system for scenario generation
#[derive(Debug, Clone, Copy, Deserialize, Serialize, PartialEq, Eq, Default)]
#[serde(rename_all = "snake_case")]
pub enum CoordinateSystem {
    /// Cartesian coordinates (x, y) with discrete lane assignments (default, for backward compatibility)
    #[default]
    Cartesian,
    /// Bicycle model (x, y, θ, v) with heading tracking and steering constraints
    Bicycle,
}

/// Lane change direction
#[derive(Debug, Clone, Copy, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum LaneChangeDirection {
    Left,
    Right,
}

/// Lane change configuration
///
/// The solver discovers lane change trajectories dynamically using smoothness
/// constraints, rather than pre-computing them with polynomials.
#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct LaneChangeConfig {
    pub enabled: bool,
    pub direction: LaneChangeDirection,
    /// Start time (can be a fixed value or range for solver to choose)
    pub start_time: ValueOrRange,
    /// Duration (can be a fixed value or range for solver to choose)
    pub duration: ValueOrRange,
}

/// Bicycle model parameters for a specific actor
#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct BicycleParams {
    /// Wheelbase in meters (distance between front and rear axles)
    pub wheelbase: f64,
    /// Maximum steering angle in radians (at front wheels)
    pub max_steering_angle: f64,
    /// Maximum steering rate in radians per second
    pub max_steering_rate: f64,
}

impl BicycleParams {
    /// Validate bicycle parameters
    pub fn validate(&self) -> Result<(), String> {
        if self.wheelbase <= 0.0 {
            return Err("wheelbase must be positive".to_string());
        }
        if self.max_steering_angle <= 0.0 {
            return Err("max_steering_angle must be positive".to_string());
        }
        if self.max_steering_rate <= 0.0 {
            return Err("max_steering_rate must be positive".to_string());
        }
        Ok(())
    }

    /// Get minimum turn radius (R = L / tan(δ_max) ≈ L / δ_max for small angles)
    pub fn min_turn_radius(&self) -> f64 {
        self.wheelbase / self.max_steering_angle
    }
}

/// Scenario-level bicycle configuration (default parameters for all actors)
#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct BicycleConfig {
    /// Default wheelbase for actors without specific bicycle_params
    pub default_wheelbase: f64,
    /// Default max steering angle for actors without specific bicycle_params
    pub default_max_steering_angle: f64,
    /// Default max steering rate for actors without specific bicycle_params
    pub default_max_steering_rate: f64,
}

impl BicycleConfig {
    /// Validate bicycle configuration
    pub fn validate(&self) -> Result<(), String> {
        if self.default_wheelbase <= 0.0 {
            return Err("default_wheelbase must be positive".to_string());
        }
        if self.default_max_steering_angle <= 0.0 {
            return Err("default_max_steering_angle must be positive".to_string());
        }
        if self.default_max_steering_rate <= 0.0 {
            return Err("default_max_steering_rate must be positive".to_string());
        }
        Ok(())
    }

    /// Convert to BicycleParams
    pub fn to_params(&self) -> BicycleParams {
        BicycleParams {
            wheelbase: self.default_wheelbase,
            max_steering_angle: self.default_max_steering_angle,
            max_steering_rate: self.default_max_steering_rate,
        }
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
        #[serde(default)]
        max_velocity: ConstraintMode,
        #[serde(default)]
        min_velocity: ConstraintMode,
        #[serde(default)]
        min_lateral_distance: ConstraintMode,
        #[serde(default)]
        max_relative_velocity: ConstraintMode,
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
            max_velocity: ConstraintMode::Enforce,
            min_velocity: ConstraintMode::Ignore,
            min_lateral_distance: ConstraintMode::Ignore,
            max_relative_velocity: ConstraintMode::Ignore,
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
            ConstraintModes::Detailed {
                max_acceleration, ..
            } => *max_acceleration,
            ConstraintModes::Shorthand(s) => match s.as_str() {
                "violate_all" => ConstraintMode::Violate,
                "ignore_all" => ConstraintMode::Ignore,
                _ => ConstraintMode::Enforce,
            },
        }
    }

    /// Get the mode for max_velocity constraint
    pub fn max_velocity(&self) -> ConstraintMode {
        match self {
            ConstraintModes::Detailed { max_velocity, .. } => *max_velocity,
            ConstraintModes::Shorthand(s) => match s.as_str() {
                "violate_all" => ConstraintMode::Violate,
                "ignore_all" => ConstraintMode::Ignore,
                _ => ConstraintMode::Enforce,
            },
        }
    }

    /// Get the mode for min_velocity constraint
    pub fn min_velocity(&self) -> ConstraintMode {
        match self {
            ConstraintModes::Detailed { min_velocity, .. } => *min_velocity,
            ConstraintModes::Shorthand(s) => match s.as_str() {
                "violate_all" => ConstraintMode::Violate,
                "ignore_all" => ConstraintMode::Ignore,
                _ => ConstraintMode::Enforce,
            },
        }
    }

    /// Get the mode for min_lateral_distance constraint
    pub fn min_lateral_distance(&self) -> ConstraintMode {
        match self {
            ConstraintModes::Detailed {
                min_lateral_distance,
                ..
            } => *min_lateral_distance,
            ConstraintModes::Shorthand(s) => match s.as_str() {
                "violate_all" => ConstraintMode::Violate,
                "ignore_all" => ConstraintMode::Ignore,
                _ => ConstraintMode::Enforce,
            },
        }
    }

    /// Get the mode for max_relative_velocity constraint
    pub fn max_relative_velocity(&self) -> ConstraintMode {
        match self {
            ConstraintModes::Detailed {
                max_relative_velocity,
                ..
            } => *max_relative_velocity,
            ConstraintModes::Shorthand(s) => match s.as_str() {
                "violate_all" => ConstraintMode::Violate,
                "ignore_all" => ConstraintMode::Ignore,
                _ => ConstraintMode::Enforce,
            },
        }
    }
}

/// Actor role (ego, NPC vehicle, or pedestrian)
#[derive(Debug, Clone, Copy, Deserialize, Serialize, PartialEq, Eq)]
pub enum ActorRole {
    #[serde(rename = "ego")]
    Ego,
    #[serde(rename = "npc")]
    Npc,
    #[serde(rename = "pedestrian")]
    Pedestrian,
}

#[derive(Debug, Clone, Copy, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum ScenarioType {
    CutInLeft,
    CutInRight,
    OvertakeLeft,
    PedestrianCrossing,
}

impl std::fmt::Display for ScenarioType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ScenarioType::CutInLeft => write!(f, "cut_in_left"),
            ScenarioType::CutInRight => write!(f, "cut_in_right"),
            ScenarioType::OvertakeLeft => write!(f, "overtake_left"),
            ScenarioType::PedestrianCrossing => write!(f, "pedestrian_crossing"),
        }
    }
}

impl ScenarioType {
    /// Get the scenario model for this scenario type
    pub fn get_model(&self) -> Box<dyn crate::scenarios::ScenarioModel> {
        match self {
            ScenarioType::CutInLeft => Box::new(crate::scenarios::cut_in_left::CutInLeftModel),
            ScenarioType::CutInRight => Box::new(crate::scenarios::cut_in_right::CutInRightModel),
            ScenarioType::OvertakeLeft => {
                Box::new(crate::scenarios::overtake_left::OvertakeLeftModel)
            }
            ScenarioType::PedestrianCrossing => {
                Box::new(crate::scenarios::pedestrian_crossing::PedestrianCrossingModel)
            }
        }
    }
}

/// Road specification with lane directions for bidirectional traffic
#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct RoadSpec {
    /// Number of lanes (total, both directions)
    pub num_lanes: usize,

    /// Width of each lane in meters
    pub lane_width: f64,

    /// Direction of each lane: +1 for forward (+x), -1 for backward (-x)
    /// Length must equal num_lanes
    /// Example: [1, 1, -1, -1] for 4 lanes (2 forward, 2 backward)
    #[serde(default = "default_lane_directions")]
    pub lane_directions: Vec<i32>,

    /// Length of the road in meters (optional, will be calculated if not provided)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub road_length: Option<f64>,
}

impl RoadSpec {
    /// Get the direction of a specific lane
    pub fn get_lane_direction(&self, lane: usize) -> i32 {
        if lane < self.lane_directions.len() {
            self.lane_directions[lane]
        } else {
            // Default: all lanes go forward (backward compatible)
            1
        }
    }

    /// Validate road specification
    pub fn validate(&self) -> Result<(), String> {
        if self.lane_directions.len() != self.num_lanes {
            return Err(format!(
                "lane_directions length ({}) must equal num_lanes ({})",
                self.lane_directions.len(),
                self.num_lanes
            ));
        }

        for (i, &dir) in self.lane_directions.iter().enumerate() {
            if dir != 1 && dir != -1 {
                return Err(format!("lane_directions[{}] = {} must be +1 or -1", i, dir));
            }
        }

        if self.num_lanes == 0 {
            return Err("num_lanes must be at least 1".to_string());
        }

        if self.lane_width <= 0.0 {
            return Err("lane_width must be positive".to_string());
        }

        if let Some(length) = self.road_length {
            if length <= 0.0 {
                return Err("road_length must be positive".to_string());
            }
        }

        Ok(())
    }
}

/// Default lane directions: all forward (backward compatible)
fn default_lane_directions() -> Vec<i32> {
    vec![1; 4] // Default to 4 forward lanes
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
    /// Direction of travel: +1 for forward (+x), -1 for backward (-x)
    pub direction: i32,
    /// Scenario-specific behavior parameters
    #[serde(default)]
    pub behavior: std::collections::HashMap<String, serde_json::Value>,
    /// Lane change configuration (optional, for smooth lane changes)
    #[serde(default)]
    pub lane_change: Option<LaneChangeConfig>,
    /// Bicycle model parameters (optional, overrides scenario-level bicycle_config)
    #[serde(default)]
    pub bicycle_params: Option<BicycleParams>,
}

/// Root scenario specification
#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct ScenarioSpec {
    pub scenario_type: ScenarioType,
    pub time_step: f64, // seconds per discretization step
    pub duration: f64,  // total scenario duration (seconds)
    pub actors: Vec<ActorSpec>,
    pub min_ttc: f64,      // minimum time-to-collision (seconds)
    pub min_distance: f64, // minimum longitudinal distance (meters)
    /// Road specification (optional, for bidirectional traffic)
    #[serde(default)]
    pub road: Option<RoadSpec>,
    /// Lane width (deprecated, use road.lane_width instead)
    #[serde(default = "default_lane_width")]
    pub lane_width: f64, // meters
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
    /// Optimization target (optional, defaults to None for backward compatibility)
    /// When set, uses Z3 Optimize instead of Solver to find optimal scenarios
    #[serde(default)]
    pub optimization_target: OptimizationTarget,
    /// Optional maximum velocity constraint (m/s)
    #[serde(default)]
    pub max_velocity: Option<f64>,
    /// Optional minimum velocity constraint (m/s)
    #[serde(default)]
    pub min_velocity: Option<f64>,
    /// Optional minimum lateral distance constraint (m)
    #[serde(default)]
    pub min_lateral_distance: Option<f64>,
    /// Optional maximum relative velocity constraint (m/s)
    #[serde(default)]
    pub max_relative_velocity: Option<f64>,
    /// Maximum lateral acceleration during lane changes (m/s²)
    /// Default: 2.0 m/s² for comfortable driving
    #[serde(default = "default_max_lateral_acceleration")]
    pub max_lateral_acceleration: f64,
    /// Coordinate system (default: Cartesian)
    #[serde(default)]
    pub coordinate_system: CoordinateSystem,
    /// Bicycle model configuration (optional, provides default parameters for all actors)
    #[serde(default)]
    pub bicycle_config: Option<BicycleConfig>,
}

/// Default lane width for backward compatibility
fn default_lane_width() -> f64 {
    3.5
}

/// Default maximum lateral acceleration for lane changes
fn default_max_lateral_acceleration() -> f64 {
    2.0 // 2.0 m/s² - comfortable driving
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

    /// Get all NPC actors (includes pedestrians)
    pub fn npcs(&self) -> Vec<&ActorSpec> {
        self.actors
            .iter()
            .filter(|a| a.role == ActorRole::Npc || a.role == ActorRole::Pedestrian)
            .collect()
    }

    /// Get actor by ID
    pub fn get_actor(&self, id: &str) -> Option<&ActorSpec> {
        self.actors.iter().find(|a| a.id == id)
    }

    /// Get lane width (backward compatible)
    pub fn get_lane_width(&self) -> f64 {
        self.road
            .as_ref()
            .map(|r| r.lane_width)
            .unwrap_or(self.lane_width)
    }

    /// Get lane direction (backward compatible)
    /// Returns +1 for forward lanes, -1 for backward lanes
    pub fn get_lane_direction(&self, lane: usize) -> i32 {
        self.road
            .as_ref()
            .map(|r| r.get_lane_direction(lane))
            .unwrap_or(1) // Default: all forward
    }

    /// Get number of lanes (backward compatible)
    pub fn get_num_lanes(&self) -> usize {
        self.road.as_ref().map(|r| r.num_lanes).unwrap_or(2) // Default: 2 lanes
    }

    /// Get bicycle parameters for an actor (uses actor-specific params or scenario defaults)
    pub fn get_bicycle_params(&self, actor: &ActorSpec) -> Option<BicycleParams> {
        actor
            .bicycle_params
            .clone()
            .or_else(|| self.bicycle_config.as_ref().map(|cfg| cfg.to_params()))
    }

    /// Validate the specification
    pub fn validate(&self) -> Result<(), String> {
        // Ensure road specification is present
        if self.road.is_none() {
            return Err("road specification is required".to_string());
        }

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

        // Validate road specification
        if let Some(road) = &self.road {
            road.validate()?;
        }

        // Generation parameters
        if self.num_scenarios == 0 {
            return Err("num_scenarios must be at least 1".to_string());
        }

        // NEW: Validate exactly one ego
        let ego_count = self
            .actors
            .iter()
            .filter(|a| a.role == ActorRole::Ego)
            .count();
        if ego_count != 1 {
            return Err(format!("Expected exactly 1 ego actor, found {}", ego_count));
        }

        // NEW: Validate at least one NPC or pedestrian
        let npc_count = self
            .actors
            .iter()
            .filter(|a| a.role == ActorRole::Npc || a.role == ActorRole::Pedestrian)
            .count();
        if npc_count == 0 {
            return Err("At least one NPC or pedestrian actor required".to_string());
        }

        // NEW: Validate unique actor IDs
        let mut seen_ids = std::collections::HashSet::new();
        for actor in &self.actors {
            if !seen_ids.insert(&actor.id) {
                return Err(format!("Duplicate actor ID: {}", actor.id));
            }
        }

        // NEW: Validate all actor parameters
        let num_lanes = self.get_num_lanes();
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
            // Validate lane number
            if actor.lane >= num_lanes {
                return Err(format!(
                    "Actor {} lane {} exceeds num_lanes {}",
                    actor.id, actor.lane, num_lanes
                ));
            }
            // Validate direction
            if actor.direction != 1 && actor.direction != -1 {
                return Err(format!(
                    "Actor {} direction must be +1 (forward) or -1 (backward), got {}",
                    actor.id, actor.direction
                ));
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

        // Validate bicycle configuration
        if self.coordinate_system == CoordinateSystem::Bicycle {
            // Validate scenario-level bicycle config if present
            if let Some(ref bicycle_config) = self.bicycle_config {
                bicycle_config.validate()?;
            }

            // Ensure all actors have bicycle params (either from actor or scenario defaults)
            for actor in &self.actors {
                if actor.role != ActorRole::Pedestrian {
                    let params = self.get_bicycle_params(actor);
                    if params.is_none() {
                        return Err(format!(
                            "Actor {} requires bicycle_params when coordinate_system is bicycle \
                             (either specify bicycle_params for the actor or bicycle_config at scenario level)",
                            actor.id
                        ));
                    }
                    // Validate actor-specific params if present
                    if let Some(ref params) = actor.bicycle_params {
                        params.validate()?;
                    }
                }
            }
        }

        // Validate bicycle params are only used with bicycle coordinate system
        if self.coordinate_system != CoordinateSystem::Bicycle {
            if self.bicycle_config.is_some() {
                return Err(
                    "bicycle_config can only be used with coordinate_system: bicycle".to_string(),
                );
            }
            for actor in &self.actors {
                if actor.bicycle_params.is_some() {
                    return Err(format!(
                        "Actor {} has bicycle_params but coordinate_system is not bicycle",
                        actor.id
                    ));
                }
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
                    direction: 1,
                    behavior: HashMap::new(),
                    lane_change: None,
                    bicycle_params: None,
                },
                ActorSpec {
                    id: "npc".to_string(),
                    role: ActorRole::Npc,
                    lane: 0,
                    position: ValueOrRange::Range([60.0, 80.0]),
                    speed: ValueOrRange::Range([12.0, 14.0]),
                    acceleration: ValueOrRange::Range([-8.0, 3.0]),
                    direction: 1,
                    behavior: HashMap::new(),
                    lane_change: Some(LaneChangeConfig {
                        enabled: true,
                        direction: LaneChangeDirection::Right,
                        start_time: ValueOrRange::Range([2.5, 7.5]),
                        duration: ValueOrRange::Range([3.0, 4.0]),
                    }),
                    bicycle_params: None,
                },
            ],
            min_ttc: 3.0,
            min_distance: 5.0,
            road: None,
            lane_width: 3.5,
            num_scenarios: 1,
            constraint_modes: ConstraintModes::default(),
            max_acceleration: None,
            max_deceleration: None,
            optimization_target: OptimizationTarget::None,
            max_velocity: None,
            min_velocity: None,
            min_lateral_distance: None,
            max_relative_velocity: None,
            max_lateral_acceleration: 2.0,
            coordinate_system: CoordinateSystem::Cartesian,
            bicycle_config: None,
        }
    }

    #[test]
    fn test_road_spec_validation() {
        let valid_road = RoadSpec {
            num_lanes: 4,
            lane_width: 3.5,
            lane_directions: vec![1, 1, -1, -1],
            road_length: None,
        };
        assert!(valid_road.validate().is_ok());

        let invalid_road = RoadSpec {
            num_lanes: 4,
            lane_width: 3.5,
            lane_directions: vec![1, 1, -1], // Wrong length
            road_length: None,
        };
        assert!(invalid_road.validate().is_err());
    }

    #[test]
    fn test_road_spec_invalid_direction() {
        let road = RoadSpec {
            num_lanes: 4,
            lane_width: 3.5,
            lane_directions: vec![1, 2, -1, -1], // 2 is invalid
            road_length: None,
        };
        assert!(road.validate().is_err());
    }

    #[test]
    fn test_get_lane_direction() {
        let road = RoadSpec {
            num_lanes: 4,
            lane_width: 3.5,
            lane_directions: vec![1, 1, -1, -1],
            road_length: None,
        };

        assert_eq!(road.get_lane_direction(0), 1);
        assert_eq!(road.get_lane_direction(2), -1);
        assert_eq!(road.get_lane_direction(10), 1); // Out of bounds, default
    }

    #[test]
    fn test_scenario_spec_backward_compat() {
        let yaml = r#"
    scenario_type: cut_in_left
    time_step: 0.5
    duration: 10.0
    actors:
      - id: ego
        role: ego
        lane: 0
        position: 50.0
        speed: 15.0
        direction: 1
        acceleration: [-8.0, 3.0]
      - id: npc
        role: npc
        lane: 1
        position: 60.0
        speed: 13.0
        direction: 1
        acceleration: [-8.0, 3.0]
        behavior:
          cut_in_time: 5.0
    min_ttc: 3.0
    min_distance: 5.0
    lane_width: 3.5
    num_scenarios: 1
"#;

        let spec: ScenarioSpec = serde_yml::from_str(yaml).unwrap();
        assert_eq!(spec.get_lane_width(), 3.5);
        assert_eq!(spec.get_num_lanes(), 2); // Default
        assert_eq!(spec.get_lane_direction(0), 1); // All forward
    }

    #[test]
    fn test_parse_road_with_directions() {
        let yaml = r#"
scenario_type: cut_in_left
time_step: 0.5
duration: 10.0
road:
  num_lanes: 4
  lane_width: 3.5
  lane_directions: [1, 1, -1, -1]
actors:
  - id: ego
    role: ego
    lane: 0
    position: 50.0
    speed: 20.0
    direction: 1
    acceleration: [-8.0, 3.0]
  - id: npc
    role: npc
    lane: 2
    position: 150.0
    speed: 20.0
    direction: 1
    acceleration: [-8.0, 3.0]
min_ttc: 3.0
min_distance: 5.0
num_scenarios: 1
"#;

        let spec: ScenarioSpec = serde_yml::from_str(yaml).unwrap();
        assert_eq!(spec.get_num_lanes(), 4);
        assert_eq!(spec.get_lane_width(), 3.5);
        assert_eq!(spec.get_lane_direction(0), 1);
        assert_eq!(spec.get_lane_direction(2), -1);
    }
}
