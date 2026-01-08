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

/// Connection between roads (Phase 3 placeholder)
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

#[derive(Debug, Clone, Copy, Deserialize, Serialize, Default)]
#[serde(rename_all = "snake_case")]
pub enum ConnectionType {
    #[default]
    Predecessor,
    Successor,
    Junction,
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
}
