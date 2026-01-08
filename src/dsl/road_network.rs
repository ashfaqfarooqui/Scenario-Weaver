//! Road network data structures for multi-road scenarios

use serde::{Deserialize, Serialize};

use super::types::RoadSpec;

/// A network of connected roads
#[derive(Debug, Clone, Deserialize, Serialize, Default)]
pub struct RoadNetwork {
    /// Named roads in the network
    #[serde(default)]
    pub roads: Vec<ExtendedRoadSpec>,

    /// Connections between roads (Phase 3)
    #[serde(default)]
    pub connections: Vec<RoadConnection>,

    /// Junction definitions (Phase 4)
    #[serde(default)]
    pub junctions: Vec<Junction>,
}

/// Road specification with ID and length
#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct ExtendedRoadSpec {
    /// Unique identifier for this road
    pub id: String,

    /// Number of lanes (total, both directions)
    pub num_lanes: usize,

    /// Width of each lane in meters
    pub lane_width: f64,

    /// Direction of each lane: +1 for forward (+x), -1 for backward (-x)
    #[serde(default = "default_lane_directions")]
    pub lane_directions: Vec<i32>,

    /// Road length in meters
    pub length: f64,

    /// Origin position in world coordinates (optional)
    #[serde(default)]
    pub origin: Option<WorldPosition>,

    /// Heading at origin in radians (optional, default 0 = East)
    #[serde(default)]
    pub heading: Option<f64>,
}

fn default_lane_directions() -> Vec<i32> {
    vec![1; 2] // Default to 2 forward lanes
}

impl ExtendedRoadSpec {
    /// Convert to base RoadSpec
    pub fn to_road_spec(&self) -> RoadSpec {
        RoadSpec {
            num_lanes: self.num_lanes,
            lane_width: self.lane_width,
            lane_directions: self.lane_directions.clone(),
            length: Some(self.length),
        }
    }

    /// Get the direction of a specific lane
    pub fn get_lane_direction(&self, lane: usize) -> i32 {
        if lane < self.lane_directions.len() {
            self.lane_directions[lane]
        } else {
            1 // Default: forward
        }
    }

    /// Validate road specification
    pub fn validate(&self) -> Result<(), String> {
        if self.id.is_empty() {
            return Err("road id cannot be empty".to_string());
        }

        if self.lane_directions.len() != self.num_lanes {
            return Err(format!(
                "road {} lane_directions length ({}) must equal num_lanes ({})",
                self.id,
                self.lane_directions.len(),
                self.num_lanes
            ));
        }

        for (i, &dir) in self.lane_directions.iter().enumerate() {
            if dir != 1 && dir != -1 {
                return Err(format!(
                    "road {} lane_directions[{}] = {} must be +1 or -1",
                    self.id, i, dir
                ));
            }
        }

        if self.num_lanes == 0 {
            return Err(format!("road {} num_lanes must be at least 1", self.id));
        }

        if self.lane_width <= 0.0 {
            return Err(format!("road {} lane_width must be positive", self.id));
        }

        if self.length <= 0.0 {
            return Err(format!("road {} length must be positive", self.id));
        }

        Ok(())
    }
}

/// World position for road placement
#[derive(Debug, Clone, Deserialize, Serialize, Default)]
pub struct WorldPosition {
    pub x: f64,
    pub y: f64,
}

/// Connection between roads
#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct RoadConnection {
    pub from_road: String,
    pub to_road: String,
    #[serde(default)]
    pub from_lane: Option<usize>,
    #[serde(default)]
    pub to_lane: Option<usize>,
    #[serde(default)]
    pub connection_type: ConnectionType,
}

impl RoadConnection {
    /// Validate the connection against a road network
    pub fn validate(&self, network: &RoadNetwork) -> Result<(), String> {
        let from_road = network.get_road(&self.from_road).ok_or_else(|| {
            format!("Connection references unknown from_road: {}", self.from_road)
        })?;
        let to_road = network.get_road(&self.to_road).ok_or_else(|| {
            format!("Connection references unknown to_road: {}", self.to_road)
        })?;

        // Validate lane references if specified
        if let Some(from_lane) = self.from_lane {
            if from_lane >= from_road.num_lanes {
                return Err(format!(
                    "from_lane {} exceeds {} num_lanes ({})",
                    from_lane, self.from_road, from_road.num_lanes
                ));
            }
        }

        if let Some(to_lane) = self.to_lane {
            if to_lane >= to_road.num_lanes {
                return Err(format!(
                    "to_lane {} exceeds {} num_lanes ({})",
                    to_lane, self.to_road, to_road.num_lanes
                ));
            }
        }

        Ok(())
    }
}

#[derive(Debug, Clone, Copy, Deserialize, Serialize, Default, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum ConnectionType {
    #[default]
    Predecessor,
    Successor,
    Junction,
}

/// Coordinate transformation utilities for multi-road scenarios
pub mod transforms {
    use super::*;

    /// Transform a position from road-local coordinates to world coordinates
    pub fn road_to_world(
        road: &ExtendedRoadSpec,
        road_position: f64, // s-coordinate along road
        lateral_offset: f64, // t-coordinate (lateral offset from center)
    ) -> (f64, f64) {
        let origin = road.origin.clone().unwrap_or_default();
        let heading = road.heading.unwrap_or(0.0);

        // Road-local coordinates: s along road, t lateral
        // Transform to world coordinates using heading
        let cos_h = heading.cos();
        let sin_h = heading.sin();

        let world_x = origin.x + road_position * cos_h - lateral_offset * sin_h;
        let world_y = origin.y + road_position * sin_h + lateral_offset * cos_h;

        (world_x, world_y)
    }

    /// Transform a position from world coordinates to road-local coordinates
    pub fn world_to_road(
        road: &ExtendedRoadSpec,
        world_x: f64,
        world_y: f64,
    ) -> (f64, f64) {
        let origin = road.origin.clone().unwrap_or_default();
        let heading = road.heading.unwrap_or(0.0);

        // Translate to road origin
        let dx = world_x - origin.x;
        let dy = world_y - origin.y;

        // Rotate by -heading to get road-local coordinates
        let cos_h = heading.cos();
        let sin_h = heading.sin();

        let road_s = dx * cos_h + dy * sin_h;
        let road_t = -dx * sin_h + dy * cos_h;

        (road_s, road_t)
    }

    /// Get lane center y-coordinate (lateral offset from road center)
    pub fn lane_center_offset(road: &ExtendedRoadSpec, lane: usize) -> f64 {
        // OpenDRIVE convention: lane 0 is at center, positive to left, negative to right
        // Our convention: lane indices 0, 1, 2, ... from left to right
        let total_width = road.num_lanes as f64 * road.lane_width;
        let half_width = total_width / 2.0;
        let lane_center = (lane as f64 + 0.5) * road.lane_width;

        // Convert to lateral offset (positive = left of center)
        half_width - lane_center
    }

    /// Check if a position is within a road's bounds
    pub fn is_on_road(road: &ExtendedRoadSpec, s: f64, t: f64) -> bool {
        let total_width = road.num_lanes as f64 * road.lane_width;
        let half_width = total_width / 2.0;

        s >= 0.0 && s <= road.length && t.abs() <= half_width
    }

    /// Get the road endpoint position in world coordinates
    pub fn road_endpoint(road: &ExtendedRoadSpec, at_start: bool) -> (f64, f64) {
        if at_start {
            let origin = road.origin.clone().unwrap_or_default();
            (origin.x, origin.y)
        } else {
            road_to_world(road, road.length, 0.0)
        }
    }
}

/// Junction definition (Phase 4 placeholder)
#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct Junction {
    pub id: String,
    #[serde(default)]
    pub junction_type: JunctionType,
    #[serde(default)]
    pub main_road: Option<String>,
    #[serde(default)]
    pub incoming_roads: Vec<String>,
    #[serde(default)]
    pub position: Option<f64>,
    #[serde(default)]
    pub side: Option<JunctionSide>,
}

#[derive(Debug, Clone, Copy, Deserialize, Serialize, Default)]
#[serde(rename_all = "snake_case")]
pub enum JunctionType {
    #[default]
    TJunction,
    Crossroads,
}

#[derive(Debug, Clone, Copy, Deserialize, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum JunctionSide {
    Left,
    Right,
}

impl RoadNetwork {
    /// Create a new road network
    pub fn new(roads: Vec<ExtendedRoadSpec>) -> Self {
        Self {
            roads,
            connections: vec![],
            junctions: vec![],
        }
    }

    /// Check if the road network is empty
    pub fn is_empty(&self) -> bool {
        self.roads.is_empty()
    }

    /// Get road by ID
    pub fn get_road(&self, id: &str) -> Option<&ExtendedRoadSpec> {
        self.roads.iter().find(|r| r.id == id)
    }

    /// Get the first road in the network (primary road)
    pub fn primary_road(&self) -> Option<&ExtendedRoadSpec> {
        self.roads.first()
    }

    /// Validate the road network
    pub fn validate(&self) -> Result<(), String> {
        if self.roads.is_empty() {
            return Err("Road network must have at least one road".to_string());
        }

        // Check for duplicate IDs
        let mut seen_ids = std::collections::HashSet::new();
        for road in &self.roads {
            if !seen_ids.insert(&road.id) {
                return Err(format!("Duplicate road ID: {}", road.id));
            }
            road.validate()?;
        }

        // Validate connections reference existing roads
        for conn in &self.connections {
            if self.get_road(&conn.from_road).is_none() {
                return Err(format!(
                    "Connection references unknown road: {}",
                    conn.from_road
                ));
            }
            if self.get_road(&conn.to_road).is_none() {
                return Err(format!(
                    "Connection references unknown road: {}",
                    conn.to_road
                ));
            }
        }

        // Validate junctions reference existing roads
        for junction in &self.junctions {
            if let Some(main) = &junction.main_road {
                if self.get_road(main).is_none() {
                    return Err(format!(
                        "Junction {} references unknown main road: {}",
                        junction.id, main
                    ));
                }
            }
            for road_id in &junction.incoming_roads {
                if self.get_road(road_id).is_none() {
                    return Err(format!(
                        "Junction {} references unknown road: {}",
                        junction.id, road_id
                    ));
                }
            }
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_extended_road_spec_validation() {
        let valid_road = ExtendedRoadSpec {
            id: "main".to_string(),
            num_lanes: 4,
            lane_width: 3.5,
            lane_directions: vec![1, 1, -1, -1],
            length: 500.0,
            origin: None,
            heading: None,
        };
        assert!(valid_road.validate().is_ok());
    }

    #[test]
    fn test_extended_road_spec_invalid_id() {
        let road = ExtendedRoadSpec {
            id: "".to_string(),
            num_lanes: 2,
            lane_width: 3.5,
            lane_directions: vec![1, 1],
            length: 500.0,
            origin: None,
            heading: None,
        };
        assert!(road.validate().is_err());
    }

    #[test]
    fn test_road_network_validation() {
        let network = RoadNetwork::new(vec![ExtendedRoadSpec {
            id: "main".to_string(),
            num_lanes: 2,
            lane_width: 3.5,
            lane_directions: vec![1, 1],
            length: 300.0,
            origin: None,
            heading: None,
        }]);
        assert!(network.validate().is_ok());
    }

    #[test]
    fn test_road_network_duplicate_ids() {
        let network = RoadNetwork::new(vec![
            ExtendedRoadSpec {
                id: "main".to_string(),
                num_lanes: 2,
                lane_width: 3.5,
                lane_directions: vec![1, 1],
                length: 300.0,
                origin: None,
                heading: None,
            },
            ExtendedRoadSpec {
                id: "main".to_string(), // Duplicate
                num_lanes: 2,
                lane_width: 3.5,
                lane_directions: vec![1, 1],
                length: 200.0,
                origin: None,
                heading: None,
            },
        ]);
        assert!(network.validate().is_err());
    }

    #[test]
    fn test_road_network_empty() {
        let network = RoadNetwork::default();
        assert!(network.validate().is_err());
    }

    #[test]
    fn test_get_road() {
        let network = RoadNetwork::new(vec![
            ExtendedRoadSpec {
                id: "highway".to_string(),
                num_lanes: 4,
                lane_width: 3.5,
                lane_directions: vec![1, 1, -1, -1],
                length: 500.0,
                origin: None,
                heading: None,
            },
            ExtendedRoadSpec {
                id: "side_road".to_string(),
                num_lanes: 2,
                lane_width: 3.0,
                lane_directions: vec![1, -1],
                length: 150.0,
                origin: None,
                heading: None,
            },
        ]);

        assert!(network.get_road("highway").is_some());
        assert!(network.get_road("side_road").is_some());
        assert!(network.get_road("unknown").is_none());
    }

    #[test]
    fn test_to_road_spec() {
        let extended = ExtendedRoadSpec {
            id: "main".to_string(),
            num_lanes: 4,
            lane_width: 3.5,
            lane_directions: vec![1, 1, -1, -1],
            length: 500.0,
            origin: None,
            heading: None,
        };

        let road_spec = extended.to_road_spec();
        assert_eq!(road_spec.num_lanes, 4);
        assert_eq!(road_spec.lane_width, 3.5);
        assert_eq!(road_spec.length, Some(500.0));
    }

    #[test]
    fn test_connection_validation() {
        let mut network = RoadNetwork::new(vec![
            ExtendedRoadSpec {
                id: "road_a".to_string(),
                num_lanes: 2,
                lane_width: 3.5,
                lane_directions: vec![1, 1],
                length: 300.0,
                origin: None,
                heading: None,
            },
            ExtendedRoadSpec {
                id: "road_b".to_string(),
                num_lanes: 2,
                lane_width: 3.5,
                lane_directions: vec![1, 1],
                length: 200.0,
                origin: Some(WorldPosition { x: 300.0, y: 0.0 }),
                heading: None,
            },
        ]);

        // Valid connection
        let valid_conn = RoadConnection {
            from_road: "road_a".to_string(),
            to_road: "road_b".to_string(),
            from_lane: Some(0),
            to_lane: Some(0),
            connection_type: ConnectionType::Successor,
        };
        assert!(valid_conn.validate(&network).is_ok());

        // Invalid: unknown from_road
        let invalid_conn = RoadConnection {
            from_road: "unknown".to_string(),
            to_road: "road_b".to_string(),
            from_lane: None,
            to_lane: None,
            connection_type: ConnectionType::Successor,
        };
        assert!(invalid_conn.validate(&network).is_err());

        // Invalid: lane out of bounds
        let invalid_lane_conn = RoadConnection {
            from_road: "road_a".to_string(),
            to_road: "road_b".to_string(),
            from_lane: Some(5), // Only 2 lanes exist
            to_lane: None,
            connection_type: ConnectionType::Successor,
        };
        assert!(invalid_lane_conn.validate(&network).is_err());

        // Add connection to network and validate
        network.connections.push(valid_conn);
        assert!(network.validate().is_ok());
    }

    #[test]
    fn test_road_to_world_transform() {
        use super::transforms::*;

        // Road at origin pointing east (heading = 0)
        let road = ExtendedRoadSpec {
            id: "test".to_string(),
            num_lanes: 2,
            lane_width: 3.5,
            lane_directions: vec![1, -1],
            length: 100.0,
            origin: Some(WorldPosition { x: 0.0, y: 0.0 }),
            heading: Some(0.0),
        };

        // Point at s=50, t=0 (on centerline)
        let (wx, wy) = road_to_world(&road, 50.0, 0.0);
        assert!((wx - 50.0).abs() < 0.001);
        assert!(wy.abs() < 0.001);

        // Point at s=0, t=3.5 (left of center by one lane width)
        let (wx, wy) = road_to_world(&road, 0.0, 3.5);
        assert!(wx.abs() < 0.001);
        assert!((wy - 3.5).abs() < 0.001);
    }

    #[test]
    fn test_road_to_world_rotated() {
        use super::transforms::*;
        use std::f64::consts::FRAC_PI_2;

        // Road at origin pointing north (heading = pi/2)
        let road = ExtendedRoadSpec {
            id: "test".to_string(),
            num_lanes: 2,
            lane_width: 3.5,
            lane_directions: vec![1, -1],
            length: 100.0,
            origin: Some(WorldPosition { x: 10.0, y: 20.0 }),
            heading: Some(FRAC_PI_2),
        };

        // Point at s=50 along road (should be 50 units north of origin)
        let (wx, wy) = road_to_world(&road, 50.0, 0.0);
        assert!((wx - 10.0).abs() < 0.001);
        assert!((wy - 70.0).abs() < 0.001);
    }

    #[test]
    fn test_world_to_road_transform() {
        use super::transforms::*;

        let road = ExtendedRoadSpec {
            id: "test".to_string(),
            num_lanes: 2,
            lane_width: 3.5,
            lane_directions: vec![1, -1],
            length: 100.0,
            origin: Some(WorldPosition { x: 0.0, y: 0.0 }),
            heading: Some(0.0),
        };

        // World point at (50, 0) should map to s=50, t=0
        let (s, t) = world_to_road(&road, 50.0, 0.0);
        assert!((s - 50.0).abs() < 0.001);
        assert!(t.abs() < 0.001);

        // World point at (0, 3.5) should map to s=0, t=3.5
        let (s, t) = world_to_road(&road, 0.0, 3.5);
        assert!(s.abs() < 0.001);
        assert!((t - 3.5).abs() < 0.001);
    }

    #[test]
    fn test_lane_center_offset() {
        use super::transforms::*;

        let road = ExtendedRoadSpec {
            id: "test".to_string(),
            num_lanes: 4,
            lane_width: 3.5,
            lane_directions: vec![1, 1, -1, -1],
            length: 100.0,
            origin: None,
            heading: None,
        };

        // Total width = 4 * 3.5 = 14m, half = 7m
        // Lane 0 center = 0.5 * 3.5 = 1.75m from left edge
        // Offset = 7 - 1.75 = 5.25m (left of center)
        let offset0 = lane_center_offset(&road, 0);
        assert!((offset0 - 5.25).abs() < 0.001);

        // Lane 1 center = 1.5 * 3.5 = 5.25m from left edge
        // Offset = 7 - 5.25 = 1.75m (left of center)
        let offset1 = lane_center_offset(&road, 1);
        assert!((offset1 - 1.75).abs() < 0.001);

        // Lane 2 center = 2.5 * 3.5 = 8.75m from left edge
        // Offset = 7 - 8.75 = -1.75m (right of center)
        let offset2 = lane_center_offset(&road, 2);
        assert!((offset2 - (-1.75)).abs() < 0.001);
    }

    #[test]
    fn test_is_on_road() {
        use super::transforms::*;

        let road = ExtendedRoadSpec {
            id: "test".to_string(),
            num_lanes: 2,
            lane_width: 3.5,
            lane_directions: vec![1, -1],
            length: 100.0,
            origin: None,
            heading: None,
        };

        // Total width = 2 * 3.5 = 7m, half = 3.5m

        // Valid positions
        assert!(is_on_road(&road, 50.0, 0.0));
        assert!(is_on_road(&road, 0.0, 3.4));
        assert!(is_on_road(&road, 100.0, -3.4));

        // Invalid positions
        assert!(!is_on_road(&road, -1.0, 0.0)); // Before road start
        assert!(!is_on_road(&road, 101.0, 0.0)); // After road end
        assert!(!is_on_road(&road, 50.0, 4.0)); // Too far left
        assert!(!is_on_road(&road, 50.0, -4.0)); // Too far right
    }

    #[test]
    fn test_road_endpoint() {
        use super::transforms::*;

        let road = ExtendedRoadSpec {
            id: "test".to_string(),
            num_lanes: 2,
            lane_width: 3.5,
            lane_directions: vec![1, -1],
            length: 100.0,
            origin: Some(WorldPosition { x: 10.0, y: 20.0 }),
            heading: Some(0.0), // East
        };

        let (start_x, start_y) = road_endpoint(&road, true);
        assert!((start_x - 10.0).abs() < 0.001);
        assert!((start_y - 20.0).abs() < 0.001);

        let (end_x, end_y) = road_endpoint(&road, false);
        assert!((end_x - 110.0).abs() < 0.001);
        assert!((end_y - 20.0).abs() < 0.001);
    }
}
