//! SVG visualization export functionality
//!
//! Converts internal Scenario data structures to SVG static images
//! showing vehicle trajectories, lane layout, and safety metrics.

use crate::error::Result;
use crate::scenario::model::Scenario;
use svg::node::element::{Circle, Group, Line, Path, Rectangle, Text};
use svg::Document;

// Canvas dimensions (matched with GIF animator for consistency)
const CANVAS_WIDTH: f64 = 1200.0;
const CANVAS_HEIGHT: f64 = 600.0;
const MARGIN: f64 = 80.0;
const ROAD_MARGIN_TOP: f64 = 120.0;

// Colors
const COLOR_EGO: &str = "#4CAF50"; // Green
const COLOR_NPC: &str = "#2196F3"; // Blue
const COLOR_PEDESTRIAN: &str = "#FF9800"; // Orange
const COLOR_VIOLATION: &str = "#F44336"; // Red
const COLOR_EGO_PATH: &str = "#8BC34A"; // Light green
const COLOR_NPC_PATH: &str = "#64B5F6"; // Light blue
const COLOR_PEDESTRIAN_PATH: &str = "#FFB74D"; // Light orange
const COLOR_ROAD: &str = "#2A2A2A"; // Dark gray
const COLOR_LANE_MARKING: &str = "#FFFFFF"; // White
const COLOR_TEXT: &str = "#333333"; // Dark gray text
const COLOR_BACKGROUND: &str = "#F5F5F5"; // Light gray background

// Vehicle dimensions (in pixels)
const VEHICLE_LENGTH: f64 = 12.0;
const VEHICLE_WIDTH: f64 = 6.0;

/// Export a scenario to SVG format for visualization
///
/// Generates an SVG file with:
/// - Header with scenario metadata
/// - Metrics bar with safety information
/// - Road surface with lane markings
/// - Vehicle trajectories as polylines
/// - Vehicle markers at initial and final positions
/// - Legend with color key
///
/// # Example
/// ```no_run
/// use scenario_generator::{generate_single_scenario, export_scenario_to_svg};
///
/// let yaml = std::fs::read_to_string("scenario.yaml").unwrap();
/// let scenario = generate_single_scenario(&yaml).unwrap();
/// let svg = export_scenario_to_svg(&scenario).unwrap();
/// std::fs::write("scenario.svg", svg).unwrap();
/// ```
pub fn export_to_svg(scenario: &Scenario) -> Result<String> {
    let visualizer = SvgVisualizer::new(scenario);
    visualizer.generate()
}

/// Configuration for SVG visualization
#[derive(Debug)]
struct VisualizerConfig {
    canvas_width: f64,
    canvas_height: f64,
    margin: f64,
    road_margin_top: f64,
    x_scale: f64,
    y_scale: f64,
    x_min: f64,
    y_max: f64,
}

impl VisualizerConfig {
    fn from_scenario(scenario: &Scenario) -> Self {
        // Find bounds of all trajectories
        let (mut x_min, mut x_max, mut y_min, mut y_max) = (f64::MAX, f64::MIN, f64::MAX, f64::MIN);

        for actor in &scenario.actors {
            for state in &actor.states {
                x_min = x_min.min(state.position().x);
                x_max = x_max.max(state.position().x);
                y_min = y_min.min(state.position().y);
                y_max = y_max.max(state.position().y);
            }
        }

        // Include road extent in bounds to ensure all lanes are visible
        y_min = y_min.min(0.0);
        y_max = y_max.max(scenario.road.num_lanes as f64 * scenario.road.lane_width);

        // Add padding (10% on each side)
        let x_range = x_max - x_min;
        let y_range = y_max - y_min;
        x_min -= x_range * 0.1;
        x_max += x_range * 0.1;
        y_min -= y_range * 0.1;
        y_max += y_range * 0.1;

        // Compute scales
        let drawable_width = CANVAS_WIDTH - 2.0 * MARGIN;
        let drawable_height = CANVAS_HEIGHT - ROAD_MARGIN_TOP - MARGIN;
        let x_scale = drawable_width / (x_max - x_min);
        let y_scale = drawable_height / (y_max - y_min);

        Self {
            canvas_width: CANVAS_WIDTH,
            canvas_height: CANVAS_HEIGHT,
            margin: MARGIN,
            road_margin_top: ROAD_MARGIN_TOP,
            x_scale,
            y_scale,
            x_min,
            y_max,
        }
    }
}

/// Main SVG visualizer
struct SvgVisualizer<'a> {
    scenario: &'a Scenario,
    config: VisualizerConfig,
}

impl<'a> SvgVisualizer<'a> {
    fn new(scenario: &'a Scenario) -> Self {
        let config = VisualizerConfig::from_scenario(scenario);
        Self { scenario, config }
    }

    /// Generate the complete SVG document
    fn generate(&self) -> Result<String> {
        let mut document = Document::new()
            .set("width", self.config.canvas_width)
            .set("height", self.config.canvas_height)
            .set(
                "viewBox",
                (0, 0, self.config.canvas_width, self.config.canvas_height),
            );

        document = self.add_background(document);
        document = self.add_header(document);
        document = self.add_metrics_bar(document);
        document = self.add_road_surface(document);
        document = self.add_lane_markings(document);
        document = self.add_trajectories(document);
        document = self.add_vehicles(document);
        document = self.add_legend(document);

        Ok(document.to_string())
    }

    /// Transform scenario coordinates to SVG viewport coordinates
    fn transform_coords(&self, scenario_x: f64, scenario_y: f64) -> (f64, f64) {
        let svg_x = self.config.margin + (scenario_x - self.config.x_min) * self.config.x_scale;
        // Flip Y-axis: higher scenario Y should be at top (lower SVG Y)
        let svg_y =
            self.config.road_margin_top + (self.config.y_max - scenario_y) * self.config.y_scale;
        (svg_x, svg_y)
    }

    /// Get the SVG Y coordinate for a lane center
    fn get_lane_center_y(&self, lane_y: f64) -> f64 {
        let (_svg_x, svg_y) = self.transform_coords(0.0, lane_y);
        svg_y
    }

    /// Get color for an actor based on role
    fn get_actor_color(&self, actor_id: &str) -> &'static str {
        if actor_id.to_lowercase().contains("ego") {
            COLOR_EGO
        } else if actor_id.to_lowercase().contains("pedestrian") {
            COLOR_PEDESTRIAN
        } else {
            COLOR_NPC
        }
    }

    /// Get trajectory path color for an actor
    fn get_actor_path_color(&self, actor_id: &str) -> &'static str {
        if actor_id.to_lowercase().contains("ego") {
            COLOR_EGO_PATH
        } else if actor_id.to_lowercase().contains("pedestrian") {
            COLOR_PEDESTRIAN_PATH
        } else {
            COLOR_NPC_PATH
        }
    }

    /// Add background rectangle
    fn add_background(&self, document: Document) -> Document {
        let bg = Rectangle::new()
            .set("width", self.config.canvas_width)
            .set("height", self.config.canvas_height)
            .set("fill", COLOR_BACKGROUND);
        document.add(bg)
    }

    /// Add header with scenario information
    fn add_header(&self, document: Document) -> Document {
        let mut group = Group::new().set("id", "header");

        // Title
        let title = Text::new(format!("Scenario: {}", self.scenario.scenario_type))
            .set("x", self.config.canvas_width / 2.0)
            .set("y", 30)
            .set("text-anchor", "middle")
            .set("font-family", "Arial, sans-serif")
            .set("font-size", 24)
            .set("font-weight", "bold")
            .set("fill", COLOR_TEXT);
        group = group.add(title);

        // Subtitle with ID and duration
        let subtitle = Text::new(format!(
            "ID: {} | Duration: {:.1}s | Time Step: {:.2}s",
            &self.scenario.scenario_id[..8], // First 8 chars of UUID
            self.scenario.duration,
            self.scenario.time_step
        ))
        .set("x", self.config.canvas_width / 2.0)
        .set("y", 55)
        .set("text-anchor", "middle")
        .set("font-family", "Arial, sans-serif")
        .set("font-size", 12)
        .set("fill", COLOR_TEXT);
        group = group.add(subtitle);

        document.add(group)
    }

    /// Add metrics bar with safety information
    fn add_metrics_bar(&self, document: Document) -> Document {
        let mut group = Group::new().set("id", "metrics");

        let val = &self.scenario.validation;
        let metrics_y = 80;

        // Determine color based on constraint satisfaction
        let status_color = if val.all_constraints_satisfied {
            COLOR_EGO
        } else {
            COLOR_VIOLATION
        };

        let status_text = if val.all_constraints_satisfied {
            "✓ SAFE"
        } else {
            "✗ VIOLATED"
        };

        // Metrics text
        let text = Text::new(format!(
            "Min TTC: {:.2}s | Min Distance: {:.2}m | Max Accel: {:.2} m/s² | Max Decel: {:.2} m/s² | Constraints: ",
            val.min_ttc,
            val.min_distance,
            val.max_acceleration,
            val.max_deceleration
        ))
            .set("x", self.config.canvas_width / 2.0)
            .set("y", metrics_y)
            .set("text-anchor", "middle")
            .set("font-family", "Arial, sans-serif")
            .set("font-size", 14)
            .set("fill", COLOR_TEXT);
        group = group.add(text);

        // Status text with color
        let status = Text::new(status_text.to_string())
            .set("x", self.config.canvas_width / 2.0 + 320.0)
            .set("y", metrics_y)
            .set("text-anchor", "start")
            .set("font-family", "Arial, sans-serif")
            .set("font-size", 14)
            .set("font-weight", "bold")
            .set("fill", status_color);
        group = group.add(status);

        document.add(group)
    }

    /// Add road surface
    fn add_road_surface(&self, document: Document) -> Document {
        let road_start_y = self.config.road_margin_top;
        let road_height =
            self.config.canvas_height - self.config.road_margin_top - self.config.margin;

        let road = Rectangle::new()
            .set("x", self.config.margin)
            .set("y", road_start_y)
            .set("width", self.config.canvas_width - 2.0 * self.config.margin)
            .set("height", road_height)
            .set("fill", COLOR_ROAD);

        document.add(road)
    }

    /// Add lane markings
    fn add_lane_markings(&self, document: Document) -> Document {
        let mut group = Group::new().set("id", "lane_markings");

        // Use static road geometry from road spec
        let num_lanes = self.scenario.road.num_lanes;
        let lane_width = self.scenario.road.lane_width;
        let road_width = num_lanes as f64 * lane_width;

        // Draw lane dividers between lanes (dashed lines)
        // Lane dividers are at y = lane_width * i for i in 1..num_lanes
        for i in 1..num_lanes {
            let lane_y = lane_width * i as f64;
            let svg_y = self.get_lane_center_y(lane_y);

            let line = Line::new()
                .set("x1", self.config.margin)
                .set("y1", svg_y)
                .set("x2", self.config.canvas_width - self.config.margin)
                .set("y2", svg_y)
                .set("stroke", COLOR_LANE_MARKING)
                .set("stroke-width", 2)
                .set("stroke-dasharray", "10,10")
                .set("opacity", 0.6);
            group = group.add(line);
        }

        // Draw road edges (solid lines)
        let top_edge = road_width; // y = road_width is the top edge
        let bottom_edge = 0.0; // y = 0 is the bottom edge

        let top_line = Line::new()
            .set("x1", self.config.margin)
            .set("y1", self.get_lane_center_y(top_edge))
            .set("x2", self.config.canvas_width - self.config.margin)
            .set("y2", self.get_lane_center_y(top_edge))
            .set("stroke", COLOR_LANE_MARKING)
            .set("stroke-width", 3);
        group = group.add(top_line);

        let bottom_line = Line::new()
            .set("x1", self.config.margin)
            .set("y1", self.get_lane_center_y(bottom_edge))
            .set("x2", self.config.canvas_width - self.config.margin)
            .set("y2", self.get_lane_center_y(bottom_edge))
            .set("stroke", COLOR_LANE_MARKING)
            .set("stroke-width", 3);
        group = group.add(bottom_line);

        document.add(group)
    }

    /// Add vehicle trajectories as polylines
    fn add_trajectories(&self, document: Document) -> Document {
        let mut group = Group::new().set("id", "trajectories");

        for actor in &self.scenario.actors {
            // Build path data
            let mut path_data = String::new();
            for (i, state) in actor.states.iter().enumerate() {
                let (svg_x, svg_y) = self.transform_coords(state.position().x, state.position().y);
                if i == 0 {
                    path_data.push_str(&format!("M {} {} ", svg_x, svg_y));
                } else {
                    path_data.push_str(&format!("L {} {} ", svg_x, svg_y));
                }
            }

            // Draw trajectory path
            let color = self.get_actor_path_color(&actor.id);
            let path = Path::new()
                .set("d", path_data)
                .set("stroke", color)
                .set("stroke-width", 3)
                .set("fill", "none")
                .set("opacity", 0.7);
            group = group.add(path);

            // Add small circles at each time step
            for state in &actor.states {
                let (svg_x, svg_y) = self.transform_coords(state.position().x, state.position().y);
                let circle = Circle::new()
                    .set("cx", svg_x)
                    .set("cy", svg_y)
                    .set("r", 2)
                    .set("fill", color)
                    .set("opacity", 0.5);
                group = group.add(circle);
            }
        }

        // Add violation highlighting if present
        if !self.scenario.validation.safety_violations.is_empty() {
            for violation in &self.scenario.validation.safety_violations {
                // Parse time from violation string (format: "at t=X.Xs")
                if let Some(time_str) = violation.split("t=").nth(1) {
                    if let Some(time_val) = time_str.split('s').next() {
                        if let Ok(time) = time_val.parse::<f64>() {
                            // Find positions at this time for all actors
                            // Use dynamic tolerance based on time_step (matching GIF animator)
                            let tolerance = self.scenario.time_step / 2.0;
                            for actor in &self.scenario.actors {
                                if let Some(state) = actor
                                    .states
                                    .iter()
                                    .find(|s| (s.time - time).abs() < tolerance)
                                {
                                    let (svg_x, svg_y) =
                                        self.transform_coords(state.position().x, state.position().y);
                                    let marker = Circle::new()
                                        .set("cx", svg_x)
                                        .set("cy", svg_y)
                                        .set("r", 8)
                                        .set("fill", "none")
                                        .set("stroke", COLOR_VIOLATION)
                                        .set("stroke-width", 3);
                                    group = group.add(marker);
                                }
                            }
                        }
                    }
                }
            }
        }

        document.add(group)
    }

    /// Add vehicle markers at initial and final positions
    fn add_vehicles(&self, document: Document) -> Document {
        let mut group = Group::new().set("id", "vehicles");

        for actor in &self.scenario.actors {
            let color = self.get_actor_color(&actor.id);
            let is_pedestrian = actor.role.to_lowercase() == "pedestrian";

            // Initial position
            if let Some(first_state) = actor.states.first() {
                let (svg_x, svg_y) =
                    self.transform_coords(first_state.position().x, first_state.position().y);

                if is_pedestrian {
                    // Pedestrian rendered as circle
                    let circle = Circle::new()
                        .set("cx", svg_x)
                        .set("cy", svg_y)
                        .set("r", VEHICLE_WIDTH / 2.0)
                        .set("fill", color)
                        .set("stroke", COLOR_TEXT)
                        .set("stroke-width", 1);
                    group = group.add(circle);

                    // Label
                    let label = Text::new(format!("{} (t=0)", actor.id))
                        .set("x", svg_x)
                        .set("y", svg_y - VEHICLE_WIDTH / 2.0 - 5.0)
                        .set("text-anchor", "middle")
                        .set("font-family", "Arial, sans-serif")
                        .set("font-size", 10)
                        .set("font-weight", "bold")
                        .set("fill", COLOR_TEXT);
                    group = group.add(label);
                } else {
                    // Vehicle rendered as rectangle
                    let rect = Rectangle::new()
                        .set("x", svg_x - VEHICLE_LENGTH / 2.0)
                        .set("y", svg_y - VEHICLE_WIDTH / 2.0)
                        .set("width", VEHICLE_LENGTH)
                        .set("height", VEHICLE_WIDTH)
                        .set("fill", color)
                        .set("stroke", COLOR_TEXT)
                        .set("stroke-width", 1)
                        .set("rx", 1);
                    group = group.add(rect);

                    // Label
                    let label = Text::new(format!("{} (t=0)", actor.id))
                        .set("x", svg_x)
                        .set("y", svg_y - VEHICLE_WIDTH / 2.0 - 5.0)
                        .set("text-anchor", "middle")
                        .set("font-family", "Arial, sans-serif")
                        .set("font-size", 10)
                        .set("font-weight", "bold")
                        .set("fill", COLOR_TEXT);
                    group = group.add(label);
                }
            }

            // Final position
            if let Some(last_state) = actor.states.last() {
                let (svg_x, svg_y) =
                    self.transform_coords(last_state.position().x, last_state.position().y);

                if is_pedestrian {
                    // Pedestrian rendered as circle
                    let circle = Circle::new()
                        .set("cx", svg_x)
                        .set("cy", svg_y)
                        .set("r", VEHICLE_WIDTH / 2.0)
                        .set("fill", color)
                        .set("stroke", COLOR_TEXT)
                        .set("stroke-width", 2)
                        .set("stroke-dasharray", "3,3")
                        .set("opacity", 0.8);
                    group = group.add(circle);

                    // Label
                    let label = Text::new(format!("{} (t={:.1})", actor.id, last_state.time))
                        .set("x", svg_x)
                        .set("y", svg_y + VEHICLE_WIDTH / 2.0 + 15.0)
                        .set("text-anchor", "middle")
                        .set("font-family", "Arial, sans-serif")
                        .set("font-size", 10)
                        .set("font-weight", "bold")
                        .set("fill", COLOR_TEXT);
                    group = group.add(label);
                } else {
                    // Vehicle rendered as rectangle
                    let rect = Rectangle::new()
                        .set("x", svg_x - VEHICLE_LENGTH / 2.0)
                        .set("y", svg_y - VEHICLE_WIDTH / 2.0)
                        .set("width", VEHICLE_LENGTH)
                        .set("height", VEHICLE_WIDTH)
                        .set("fill", color)
                        .set("stroke", COLOR_TEXT)
                        .set("stroke-width", 2)
                        .set("stroke-dasharray", "3,3")
                        .set("rx", 1)
                        .set("opacity", 0.8);
                    group = group.add(rect);

                    // Label
                    let label = Text::new(format!("{} (t={:.1})", actor.id, last_state.time))
                        .set("x", svg_x)
                        .set("y", svg_y + VEHICLE_WIDTH / 2.0 + 15.0)
                        .set("text-anchor", "middle")
                        .set("font-family", "Arial, sans-serif")
                        .set("font-size", 10)
                        .set("font-weight", "bold")
                        .set("fill", COLOR_TEXT);
                    group = group.add(label);
                }
            }
        }

        document.add(group)
    }

    /// Add legend with color key
    fn add_legend(&self, document: Document) -> Document {
        let mut group = Group::new().set("id", "legend");

        let legend_x = self.config.canvas_width - self.config.margin - 150.0;
        let legend_y = self.config.canvas_height - self.config.margin - 100.0;

        // Legend background
        let bg = Rectangle::new()
            .set("x", legend_x)
            .set("y", legend_y)
            .set("width", 150)
            .set("height", 100)
            .set("fill", "white")
            .set("stroke", COLOR_TEXT)
            .set("stroke-width", 1)
            .set("rx", 3);
        group = group.add(bg);

        // Title
        let title = Text::new("Legend")
            .set("x", legend_x + 75.0)
            .set("y", legend_y + 15.0)
            .set("text-anchor", "middle")
            .set("font-family", "Arial, sans-serif")
            .set("font-size", 12)
            .set("font-weight", "bold")
            .set("fill", COLOR_TEXT);
        group = group.add(title);

        // Ego vehicle
        let ego_marker = Rectangle::new()
            .set("x", legend_x + 10.0)
            .set("y", legend_y + 25.0)
            .set("width", 12)
            .set("height", 8)
            .set("fill", COLOR_EGO);
        group = group.add(ego_marker);

        let ego_text = Text::new("Ego Vehicle")
            .set("x", legend_x + 30.0)
            .set("y", legend_y + 32.0)
            .set("text-anchor", "start")
            .set("font-family", "Arial, sans-serif")
            .set("font-size", 10)
            .set("fill", COLOR_TEXT);
        group = group.add(ego_text);

        // NPC vehicle
        let npc_marker = Rectangle::new()
            .set("x", legend_x + 10.0)
            .set("y", legend_y + 40.0)
            .set("width", 12)
            .set("height", 8)
            .set("fill", COLOR_NPC);
        group = group.add(npc_marker);

        let npc_text = Text::new("NPC Vehicle")
            .set("x", legend_x + 30.0)
            .set("y", legend_y + 47.0)
            .set("text-anchor", "start")
            .set("font-family", "Arial, sans-serif")
            .set("font-size", 10)
            .set("fill", COLOR_TEXT);
        group = group.add(npc_text);

        // Pedestrian
        let pedestrian_marker = Circle::new()
            .set("cx", legend_x + 16.0)
            .set("cy", legend_y + 59.0)
            .set("r", 4)
            .set("fill", COLOR_PEDESTRIAN);
        group = group.add(pedestrian_marker);

        let pedestrian_text = Text::new("Pedestrian")
            .set("x", legend_x + 30.0)
            .set("y", legend_y + 62.0)
            .set("text-anchor", "start")
            .set("font-family", "Arial, sans-serif")
            .set("font-size", 10)
            .set("fill", COLOR_TEXT);
        group = group.add(pedestrian_text);

        // Violation marker
        let violation_marker = Circle::new()
            .set("cx", legend_x + 16.0)
            .set("cy", legend_y + 78.0)
            .set("r", 6)
            .set("fill", "none")
            .set("stroke", COLOR_VIOLATION)
            .set("stroke-width", 2);
        group = group.add(violation_marker);

        let violation_text = Text::new("Safety Violation")
            .set("x", legend_x + 30.0)
            .set("y", legend_y + 81.0)
            .set("text-anchor", "start")
            .set("font-family", "Arial, sans-serif")
            .set("font-size", 10)
            .set("fill", COLOR_TEXT);
        group = group.add(violation_text);

        document.add(group)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scenario::model::{
        Acceleration, ActorTrajectory, Position, Scenario, State, ValidationInfo, Velocity,
    };

    fn create_test_scenario() -> Scenario {
        let ego_states = vec![
            State::new(
                0.0,
                Position { x: 0.0, y: 5.0 },
                Velocity { vx: 10.0, vy: 0.0 },
                Acceleration { ax: 0.0, ay: 0.0 },
                1,
            ),
            State::new(
                1.0,
                Position { x: 10.0, y: 5.0 },
                Velocity { vx: 10.0, vy: 0.0 },
                Acceleration { ax: 0.0, ay: 0.0 },
                1,
            ),
        ];

        let npc_states = vec![
            State::new(
                0.0,
                Position { x: 5.0, y: 1.5 },
                Velocity { vx: 10.0, vy: 0.0 },
                Acceleration { ax: 0.0, ay: 0.0 },
                0,
            ),
            State::new(
                1.0,
                Position { x: 15.0, y: 5.0 },
                Velocity { vx: 10.0, vy: 0.0 },
                Acceleration { ax: 0.0, ay: 0.0 },
                1,
            ),
        ];

        Scenario {
            scenario_id: "test-scenario-123".to_string(),
            scenario_type: "cut_in_left".to_string(),
            time_step: 1.0,
            duration: 1.0,
            road: crate::dsl::types::RoadSpec {
                num_lanes: 2,
                lane_width: 3.5,
                lane_directions: vec![1, 1],
            },
            actors: vec![
                ActorTrajectory {
                    id: "ego".to_string(),
                    role: "ego".to_string(),
                    states: ego_states,
                },
                ActorTrajectory {
                    id: "npc".to_string(),
                    role: "npc".to_string(),
                    states: npc_states,
                },
            ],
            validation: ValidationInfo {
                min_ttc: 3.5,
                min_distance: 10.0,
                all_constraints_satisfied: true,
                safety_violations: vec![],
                max_acceleration: 2.0,
                max_deceleration: -3.0,
                acceleration_violations: vec![],
            },
            reference_line: None,
        }
    }

    #[test]
    fn test_export_to_svg_basic() {
        let scenario = create_test_scenario();
        let svg = export_to_svg(&scenario).unwrap();

        // Verify SVG structure
        assert!(svg.contains("<svg"));
        assert!(svg.contains("</svg>"));
        assert!(svg.contains("width=\"1200\""));
        assert!(svg.contains("height=\"600\""));
    }

    #[test]
    fn test_coordinate_transformation() {
        let scenario = create_test_scenario();
        let visualizer = SvgVisualizer::new(&scenario);

        // Test transformation exists and produces reasonable values
        let (svg_x, svg_y) = visualizer.transform_coords(5.0, 3.0);
        assert!(svg_x >= visualizer.config.margin);
        assert!(svg_x <= visualizer.config.canvas_width - visualizer.config.margin);
        assert!(svg_y >= visualizer.config.road_margin_top);
        assert!(svg_y <= visualizer.config.canvas_height - visualizer.config.margin);
    }

    #[test]
    fn test_vehicle_color_coding() {
        let scenario = create_test_scenario();
        let visualizer = SvgVisualizer::new(&scenario);

        assert_eq!(visualizer.get_actor_color("ego"), COLOR_EGO);
        assert_eq!(visualizer.get_actor_color("npc"), COLOR_NPC);
        assert_eq!(visualizer.get_actor_color("ego_vehicle"), COLOR_EGO);
    }

    #[test]
    fn test_svg_contains_scenario_elements() {
        let scenario = create_test_scenario();
        let svg = export_to_svg(&scenario).unwrap();

        // Verify key elements are present
        assert!(svg.contains("ego"));
        assert!(svg.contains("npc"));
        assert!(svg.contains("cut_in_left"));
        assert!(svg.contains("Min TTC"));
        assert!(svg.contains("Legend"));
    }
}
