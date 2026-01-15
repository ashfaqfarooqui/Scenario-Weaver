//! Scenario output data structures

use crate::dsl::types::RoadSpec;
use crate::geometry::{CartesianPoint, FrenetPoint, ReferenceLine};
use serde::{Deserialize, Serialize};

/// Complete scenario with all actor trajectories
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Scenario {
    /// Unique scenario identifier
    pub scenario_id: String,

    /// Type of scenario (e.g., "cut_in_left")
    pub scenario_type: String,

    /// Time discretization step (seconds)
    pub time_step: f64,

    /// Total duration (seconds)
    pub duration: f64,

    /// Road specification
    pub road: RoadSpec,

    /// All actors and their trajectories
    pub actors: Vec<ActorTrajectory>,

    /// Validation information
    pub validation: ValidationInfo,

    /// Reference line for coordinate conversion (optional, for Frenet scenarios)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub reference_line: Option<ReferenceLine>,
}

/// Trajectory of a single actor through the scenario
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ActorTrajectory {
    /// Actor identifier (e.g., "ego", "npc")
    pub id: String,

    /// Actor role
    pub role: String,

    /// Sequence of states over time
    pub states: Vec<State>,
}

/// State of an actor at a specific time
///
/// Supports both Frenet and Cartesian coordinates for A/B testing.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct State {
    /// Time (seconds from start)
    pub time: f64,

    /// Frenet coordinates (primary when using Frenet system)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub frenet: Option<FrenetState>,

    /// Cartesian coordinates (primary when using Cartesian system, also computed from Frenet)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub cartesian: Option<CartesianState>,
}

/// Frenet coordinate state
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FrenetState {
    /// Longitudinal position along reference line (meters)
    pub s: f64,

    /// Lateral offset from reference line (meters)
    pub t: f64,

    /// Heading angle (radians)
    pub theta: f64,

    /// Longitudinal velocity (m/s)
    pub vs: f64,

    /// Lateral velocity (m/s)
    pub vt: f64,

    /// Longitudinal acceleration (m/s²)
    #[serde(rename = "as")]
    pub as_: f64,

    /// Lateral acceleration (m/s²)
    pub at: f64,
}

/// Cartesian coordinate state
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CartesianState {
    /// Position in world coordinates
    pub position: Position,

    /// Velocity
    pub velocity: Velocity,

    /// Acceleration
    pub acceleration: Acceleration,

    /// Current lane (derived from y-position or t-offset)
    pub lane: usize,
}

/// 2D position
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Position {
    /// Longitudinal position (along road, meters)
    pub x: f64,

    /// Lateral position (across lanes, meters)
    pub y: f64,
}

/// 2D velocity
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Velocity {
    /// Longitudinal velocity (m/s)
    pub vx: f64,

    /// Lateral velocity (m/s, non-zero during lane changes)
    pub vy: f64,
}

/// 2D acceleration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Acceleration {
    /// Longitudinal acceleration (m/s²)
    pub ax: f64,

    /// Lateral acceleration (m/s², for lane changes)
    pub ay: f64,
}

/// Validation information for the scenario
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ValidationInfo {
    /// Minimum time-to-collision observed (seconds)
    pub min_ttc: f64,

    /// Minimum longitudinal distance observed (meters)
    pub min_distance: f64,

    /// Whether all constraints were satisfied
    pub all_constraints_satisfied: bool,

    /// List of any safety violations detected
    #[serde(default)]
    pub safety_violations: Vec<String>,

    /// Maximum acceleration observed (m/s²)
    #[serde(default)]
    pub max_acceleration: f64,

    /// Maximum deceleration observed (m/s², negative value)
    #[serde(default)]
    pub max_deceleration: f64,

    /// List of acceleration constraint violations
    #[serde(default)]
    pub acceleration_violations: Vec<String>,
}

impl Scenario {
    /// Create a new scenario with basic metadata
    pub fn new(scenario_type: String, time_step: f64, duration: f64, road: RoadSpec) -> Self {
        Self {
            scenario_id: uuid::Uuid::new_v4().to_string(),
            scenario_type,
            time_step,
            duration,
            road,
            actors: Vec::new(),
            validation: ValidationInfo {
                min_ttc: 999.0, // Using large value instead of INFINITY for JSON compatibility
                min_distance: 999.0,
                all_constraints_satisfied: false,
                safety_violations: Vec::new(),
                max_acceleration: 0.0,
                max_deceleration: 0.0,
                acceleration_violations: Vec::new(),
            },
            reference_line: None,
        }
    }

    /// Add an actor trajectory to the scenario
    pub fn add_actor(&mut self, trajectory: ActorTrajectory) {
        self.actors.push(trajectory);
    }

    /// Get trajectory for a specific actor
    pub fn get_actor(&self, id: &str) -> Option<&ActorTrajectory> {
        self.actors.iter().find(|a| a.id == id)
    }

    /// Compute validation metrics from trajectories
    pub fn compute_validation(&mut self, _min_ttc_required: f64, _min_dist_required: f64) {
        // This will be implemented in Phase 9 when we have actual trajectories
        // For now, just placeholder
        self.validation.all_constraints_satisfied = true;
    }
}

impl ActorTrajectory {
    /// Create a new actor trajectory
    pub fn new(id: String, role: String) -> Self {
        Self {
            id,
            role,
            states: Vec::new(),
        }
    }

    /// Add a state to the trajectory
    pub fn add_state(&mut self, state: State) {
        self.states.push(state);
    }

    /// Get state at a specific time index
    pub fn state_at(&self, index: usize) -> Option<&State> {
        self.states.get(index)
    }

    /// Get the number of time steps
    pub fn num_steps(&self) -> usize {
        self.states.len()
    }
}

impl State {
    /// Create a new state with Cartesian coordinates (backward compatible)
    pub fn new(
        time: f64,
        position: Position,
        velocity: Velocity,
        acceleration: Acceleration,
        lane: usize,
    ) -> Self {
        Self {
            time,
            frenet: None,
            cartesian: Some(CartesianState {
                position,
                velocity,
                acceleration,
                lane,
            }),
        }
    }

    /// Create a new state with Frenet coordinates
    pub fn new_frenet(
        time: f64,
        s: f64,
        t: f64,
        theta: f64,
        vs: f64,
        vt: f64,
        as_: f64,
        at: f64,
        lane: usize,
    ) -> Self {
        Self {
            time,
            frenet: Some(FrenetState {
                s,
                t,
                theta,
                vs,
                vt,
                as_,
                at,
            }),
            cartesian: None,
        }
    }

    /// Create a new state with both Frenet and Cartesian coordinates
    pub fn new_both(
        time: f64,
        frenet: FrenetState,
        cartesian: CartesianState,
    ) -> Self {
        Self {
            time,
            frenet: Some(frenet),
            cartesian: Some(cartesian),
        }
    }

    /// Get lane from Frenet t offset
    pub fn get_lane_from_frenet(&self, lane_width: f64) -> Option<usize> {
        self.frenet.as_ref().map(|f| (f.t / lane_width).round() as usize)
    }

    /// Convert Frenet to Cartesian (requires ReferenceLine)
    pub fn to_cartesian(&self, ref_line: &ReferenceLine) -> Option<CartesianState> {
        self.frenet.as_ref().map(|f| {
            let frenet_point = FrenetPoint::new(f.s, f.t);
            let cart_point = ref_line.frenet_to_cartesian(&frenet_point);

            CartesianState {
                position: Position::new(cart_point.x, cart_point.y),
                velocity: Velocity::new(
                    f.vs * f.theta.cos() - f.vt * f.theta.sin(),
                    f.vs * f.theta.sin() + f.vt * f.theta.cos(),
                ),
                acceleration: Acceleration::new(f.as_, f.at),
                lane: (f.t / 3.5).round() as usize, // TODO: use actual lane_width
            }
        })
    }

    /// Get Cartesian position (fallback to Frenet conversion if needed)
    pub fn get_position(&self, ref_line: &ReferenceLine) -> Option<Position> {
        if let Some(cart) = &self.cartesian {
            Some(cart.position.clone())
        } else if let Some(_) = &self.frenet {
            self.to_cartesian(ref_line).map(|c| c.position)
        } else {
            None
        }
    }

    /// Get Cartesian velocity (fallback to Frenet conversion if needed)
    pub fn get_velocity(&self, ref_line: &ReferenceLine) -> Option<Velocity> {
        if let Some(cart) = &self.cartesian {
            Some(cart.velocity.clone())
        } else if let Some(_) = &self.frenet {
            self.to_cartesian(ref_line).map(|c| c.velocity)
        } else {
            None
        }
    }

    /// Get lane number (from either coordinate system)
    pub fn get_lane(&self, lane_width: f64, _ref_line: &ReferenceLine) -> Option<usize> {
        if let Some(cart) = &self.cartesian {
            Some(cart.lane)
        } else if let Some(f) = &self.frenet {
            Some((f.t / lane_width).round() as usize)
        } else {
            None
        }
    }

    // Convenience methods for backward compatibility
    // These allow existing code to work while we transition to Frenet

    /// Get position (backward compatible - will panic if neither cartesian nor frenet is set)
    pub fn position(&self) -> &Position {
        self.cartesian.as_ref().map(|c| &c.position).expect("State must have cartesian or frenet data")
    }

    /// Get velocity (backward compatible - will panic if neither cartesian nor frenet is set)
    pub fn velocity(&self) -> &Velocity {
        self.cartesian.as_ref().map(|c| &c.velocity).expect("State must have cartesian or frenet data")
    }

    /// Get acceleration (backward compatible - will panic if neither cartesian nor frenet is set)
    pub fn acceleration(&self) -> &Acceleration {
        self.cartesian.as_ref().map(|c| &c.acceleration).expect("State must have cartesian or frenet data")
    }

    /// Get lane (backward compatible - will panic if neither cartesian nor frenet is set)
    pub fn lane(&self) -> usize {
        self.cartesian.as_ref().map(|c| c.lane).expect("State must have cartesian or frenet data")
    }
}

impl Position {
    /// Create a new position
    pub fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    /// Compute Euclidean distance to another position
    pub fn distance_to(&self, other: &Position) -> f64 {
        ((self.x - other.x).powi(2) + (self.y - other.y).powi(2)).sqrt()
    }

    /// Longitudinal distance (x-axis only)
    pub fn longitudinal_distance_to(&self, other: &Position) -> f64 {
        (self.x - other.x).abs()
    }

    /// Lateral distance (y-axis only)
    pub fn lateral_distance_to(&self, other: &Position) -> f64 {
        (self.y - other.y).abs()
    }
}

impl Velocity {
    /// Create a new velocity
    pub fn new(vx: f64, vy: f64) -> Self {
        Self { vx, vy }
    }

    /// Compute speed (magnitude of velocity)
    pub fn speed(&self) -> f64 {
        (self.vx.powi(2) + self.vy.powi(2)).sqrt()
    }
}

impl Acceleration {
    /// Create a new acceleration
    pub fn new(ax: f64, ay: f64) -> Self {
        Self { ax, ay }
    }

    /// Compute acceleration magnitude
    pub fn magnitude(&self) -> f64 {
        (self.ax.powi(2) + self.ay.powi(2)).sqrt()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_scenario_creation() {
        let road = RoadSpec {
            num_lanes: 2,
            lane_width: 3.5,
            lane_directions: vec![1, 1],
        };
        let scenario = Scenario::new("cut_in_left".to_string(), 0.5, 10.0, road);

        assert_eq!(scenario.scenario_type, "cut_in_left");
        assert_eq!(scenario.time_step, 0.5);
        assert_eq!(scenario.duration, 10.0);
        assert_eq!(scenario.road.num_lanes, 2);
        assert_eq!(scenario.road.lane_width, 3.5);
        assert!(!scenario.scenario_id.is_empty());
    }

    #[test]
    fn test_actor_trajectory() {
        let mut traj = ActorTrajectory::new("ego".to_string(), "ego".to_string());

        let state = State::new(
            0.0,
            Position::new(50.0, 5.25),
            Velocity::new(15.0, 0.0),
            Acceleration::new(0.0, 0.0),
            1,
        );

        traj.add_state(state);
        assert_eq!(traj.num_steps(), 1);
        assert_eq!(traj.state_at(0).unwrap().time, 0.0);
    }

    #[test]
    fn test_position_distance() {
        let p1 = Position::new(0.0, 0.0);
        let p2 = Position::new(3.0, 4.0);

        assert_eq!(p1.distance_to(&p2), 5.0);
        assert_eq!(p1.longitudinal_distance_to(&p2), 3.0);
        assert_eq!(p1.lateral_distance_to(&p2), 4.0);
    }

    #[test]
    fn test_velocity_speed() {
        let v = Velocity::new(3.0, 4.0);
        assert_eq!(v.speed(), 5.0);
    }

    #[test]
    fn test_json_serialization() {
        let road = RoadSpec {
            num_lanes: 2,
            lane_width: 3.5,
            lane_directions: vec![1, 1],
        };
        let mut scenario = Scenario::new("cut_in_left".to_string(), 0.5, 10.0, road);

        let mut ego_traj = ActorTrajectory::new("ego".to_string(), "ego".to_string());
        ego_traj.add_state(State::new(
            0.0,
            Position::new(50.0, 5.25),
            Velocity::new(15.0, 0.0),
            Acceleration::new(0.0, 0.0),
            1,
        ));

        scenario.add_actor(ego_traj);

        // Test serialization
        let json = serde_json::to_string_pretty(&scenario).unwrap();
        println!("Serialized scenario:\n{}", json);

        // Test deserialization
        let deserialized: Scenario = serde_json::from_str(&json).unwrap();
        assert_eq!(deserialized.scenario_type, "cut_in_left");
        assert_eq!(deserialized.actors.len(), 1);
        assert_eq!(deserialized.road.num_lanes, 2);
        assert_eq!(deserialized.road.lane_width, 3.5);
    }
}
