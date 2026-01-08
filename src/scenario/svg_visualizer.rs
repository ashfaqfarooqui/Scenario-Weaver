//! SVG visualization export functionality
//!
//! Converts internal Scenario data structures to SVG static images
//! showing vehicle trajectories, lane layout, and safety metrics.
//!
//! Supports both single-road and multi-road scenarios with roads at
//! different positions and headings.

use crate::dsl::road_network::RoadNetwork;
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
const COLOR_VIOLATION: &str = "#F44336"; // Red
const COLOR_EGO_PATH: &str = "#8BC34A"; // Light green
const COLOR_NPC_PATH: &str = "#64B5F6"; // Light blue
const COLOR_ROAD: &str = "#2A2A2A"; // Dark gray
const COLOR_LANE_MARKING: &str = "#FFFFFF"; // White
const COLOR_TEXT: &str = "#333333"; // Dark gray text
const COLOR_BACKGROUND: &str = "#F5F5F5"; // Light gray background
const COLOR_JUNCTION: &str = "#3D3D3D"; // Slightly lighter than road for junctions

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
/// use carla_scenario_generator::{generate_single_scenario, export_scenario_to_svg};
///
/// let yaml = std::fs::read_to_string("scenario.yaml").unwrap();
/// let scenario = generate_single_scenario(&yaml).unwrap();
/// let svg = export_scenario_to_svg(&scenario).unwrap();
/// std::fs::write("scenario.svg", svg).unwrap();
/// ```
pub fn export_to_svg(scenario: &Scenario) -> Result<String> {
    let visualizer = SvgVisualizer::new(scenario, None);
    visualizer.generate()
}

/// Export a scenario with road network to SVG format
///
/// This function renders multi-road scenarios with roads at different
/// positions and headings. Each road is drawn with its proper geometry.
pub fn export_to_svg_with_network(scenario: &Scenario, network: &RoadNetwork) -> Result<String> {
    let visualizer = SvgVisualizer::new(scenario, Some(network));
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
    fn from_scenario(scenario: &Scenario, network: Option<&RoadNetwork>) -> Self {
        // Find bounds of all trajectories
        let (mut x_min, mut x_max, mut y_min, mut y_max) = (f64::MAX, f64::MIN, f64::MAX, f64::MIN);

        for actor in &scenario.actors {
            for state in &actor.states {
                x_min = x_min.min(state.position.x);
                x_max = x_max.max(state.position.x);
                y_min = y_min.min(state.position.y);
                y_max = y_max.max(state.position.y);
            }
        }

        // Also consider road network geometry if present
        if let Some(network) = network {
            for road in &network.roads {
                let origin_x = road.origin.as_ref().map(|o| o.x).unwrap_or(0.0);
                let origin_y = road.origin.as_ref().map(|o| o.y).unwrap_or(0.0);
                let heading = road.heading.unwrap_or(0.0);
                let half_width = road.num_lanes as f64 * road.lane_width / 2.0;

                // Road start point
                x_min = x_min.min(origin_x - half_width * heading.sin().abs());
                x_max = x_max.max(origin_x + half_width * heading.sin().abs());
                y_min = y_min.min(origin_y - half_width * heading.cos().abs());
                y_max = y_max.max(origin_y + half_width * heading.cos().abs());

                // Road end point
                let end_x = origin_x + road.length * heading.cos();
                let end_y = origin_y + road.length * heading.sin();
                x_min = x_min.min(end_x - half_width * heading.sin().abs());
                x_max = x_max.max(end_x + half_width * heading.sin().abs());
                y_min = y_min.min(end_y - half_width * heading.cos().abs());
                y_max = y_max.max(end_y + half_width * heading.cos().abs());
            }
        }

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
    network: Option<&'a RoadNetwork>,
}

impl<'a> SvgVisualizer<'a> {
    fn new(scenario: &'a Scenario, network: Option<&'a RoadNetwork>) -> Self {
        let config = VisualizerConfig::from_scenario(scenario, network);
        Self {
            scenario,
            config,
            network,
        }
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
        document = self.add_junctions(document);
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
        } else {
            COLOR_NPC
        }
    }

    /// Get trajectory path color for an actor
    fn get_actor_path_color(&self, actor_id: &str) -> &'static str {
        if actor_id.to_lowercase().contains("ego") {
            COLOR_EGO_PATH
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
        // If we have a road network, render each road with proper geometry
        if let Some(network) = self.network {
            if !network.roads.is_empty() {
                return self.add_multi_road_surface(document, network);
            }
        }

        // Fallback to single rectangular road
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

    /// Add multiple roads from a road network
    fn add_multi_road_surface(&self, document: Document, network: &RoadNetwork) -> Document {
        let mut group = Group::new().set("id", "roads");

        for (idx, road) in network.roads.iter().enumerate() {
            let origin_x = road.origin.as_ref().map(|o| o.x).unwrap_or(0.0);
            let origin_y = road.origin.as_ref().map(|o| o.y).unwrap_or(0.0);
            let heading = road.heading.unwrap_or(0.0);
            let road_width = road.num_lanes as f64 * road.lane_width;
            let half_width = road_width / 2.0;

            // Calculate road corners in world coordinates
            let cos_h = heading.cos();
            let sin_h = heading.sin();

            // Road is a rectangle: from origin to origin + length in heading direction
            // with width perpendicular to heading
            // Corners: (origin - perp*half_width) to (origin + length*dir - perp*half_width)
            //          and (origin + perp*half_width) to (origin + length*dir + perp*half_width)

            // Perpendicular vector (90 degrees left of heading)
            let perp_x = -sin_h;
            let perp_y = cos_h;

            // Four corners of the road
            let corners = [
                (
                    origin_x + perp_x * half_width,
                    origin_y + perp_y * half_width,
                ),
                (
                    origin_x - perp_x * half_width,
                    origin_y - perp_y * half_width,
                ),
                (
                    origin_x + road.length * cos_h - perp_x * half_width,
                    origin_y + road.length * sin_h - perp_y * half_width,
                ),
                (
                    origin_x + road.length * cos_h + perp_x * half_width,
                    origin_y + road.length * sin_h + perp_y * half_width,
                ),
            ];

            // Transform to SVG coordinates and build path
            let svg_corners: Vec<(f64, f64)> = corners
                .iter()
                .map(|&(x, y)| self.transform_coords(x, y))
                .collect();

            // Build polygon path (0 -> 1 -> 2 -> 3 -> 0)
            let path_data = format!(
                "M {} {} L {} {} L {} {} L {} {} Z",
                svg_corners[0].0,
                svg_corners[0].1,
                svg_corners[1].0,
                svg_corners[1].1,
                svg_corners[2].0,
                svg_corners[2].1,
                svg_corners[3].0,
                svg_corners[3].1
            );

            let road_path = Path::new()
                .set("d", path_data)
                .set("fill", COLOR_ROAD)
                .set("id", format!("road_{}", idx));
            group = group.add(road_path);

            // Add road name label at the center
            let center_x = origin_x + road.length * cos_h / 2.0;
            let center_y = origin_y + road.length * sin_h / 2.0;
            let (svg_cx, svg_cy) = self.transform_coords(center_x, center_y);

            let label = Text::new(&road.id)
                .set("x", svg_cx)
                .set("y", svg_cy - 5.0)
                .set("text-anchor", "middle")
                .set("font-family", "Arial, sans-serif")
                .set("font-size", 10)
                .set("fill", COLOR_LANE_MARKING)
                .set("opacity", 0.7)
                .set(
                    "transform",
                    format!(
                        "rotate({} {} {})",
                        heading.to_degrees(),
                        svg_cx,
                        svg_cy - 5.0
                    ),
                );
            group = group.add(label);
        }

        document.add(group)
    }

    /// Add junctions to the visualization
    fn add_junctions(&self, document: Document) -> Document {
        // Only render junctions if we have a road network
        let network = match self.network {
            Some(n) if !n.junctions.is_empty() => n,
            _ => return document,
        };

        use crate::dsl::road_network::{
            CrossroadsGeometry, JunctionType, TJunctionGeometry,
        };

        let mut group = Group::new().set("id", "junctions");

        for (idx, junction) in network.junctions.iter().enumerate() {
            // Get junction corners based on type
            let corners: [(f64, f64); 4] = match junction.junction_type {
                JunctionType::TJunction => {
                    if let Ok(geom) = TJunctionGeometry::from_junction(junction, network) {
                        geom.get_corners()
                    } else {
                        continue;
                    }
                }
                JunctionType::Crossroads => {
                    if let Ok(geom) = CrossroadsGeometry::from_junction(junction, network) {
                        geom.get_corners()
                    } else {
                        continue;
                    }
                }
            };

            // Transform corners to SVG coordinates
            let svg_corners: Vec<(f64, f64)> = corners
                .iter()
                .map(|&(x, y)| self.transform_coords(x, y))
                .collect();

            // Build polygon path
            let path_data = format!(
                "M {} {} L {} {} L {} {} L {} {} Z",
                svg_corners[0].0, svg_corners[0].1,
                svg_corners[1].0, svg_corners[1].1,
                svg_corners[2].0, svg_corners[2].1,
                svg_corners[3].0, svg_corners[3].1
            );

            let junction_path = Path::new()
                .set("d", path_data)
                .set("fill", COLOR_JUNCTION)
                .set("id", format!("junction_{}", idx));
            group = group.add(junction_path);

            // Add junction label
            let center_x = corners.iter().map(|(x, _)| x).sum::<f64>() / 4.0;
            let center_y = corners.iter().map(|(_, y)| y).sum::<f64>() / 4.0;
            let (svg_cx, svg_cy) = self.transform_coords(center_x, center_y);

            let label = Text::new(&junction.id)
                .set("x", svg_cx)
                .set("y", svg_cy)
                .set("text-anchor", "middle")
                .set("dominant-baseline", "middle")
                .set("font-family", "Arial, sans-serif")
                .set("font-size", 9)
                .set("fill", COLOR_LANE_MARKING)
                .set("opacity", 0.8);
            group = group.add(label);
        }

        document.add(group)
    }

    /// Add lane markings
    fn add_lane_markings(&self, document: Document) -> Document {
        // If we have a road network, render lane markings for each road
        if let Some(network) = self.network {
            if !network.roads.is_empty() {
                return self.add_multi_road_lane_markings(document, network);
            }
        }

        let mut group = Group::new().set("id", "lane_markings");

        // Get unique lane Y positions from all actor states
        let mut lane_ys = std::collections::HashSet::new();
        for actor in &self.scenario.actors {
            for state in &actor.states {
                // Round to avoid floating point issues
                let y_rounded = (state.position.y * 10.0).round() / 10.0;
                lane_ys.insert((y_rounded * 1000.0) as i64);
            }
        }

        let mut lane_y_sorted: Vec<f64> = lane_ys.iter().map(|&y| y as f64 / 1000.0).collect();
        lane_y_sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());

        // Estimate lane width from spacing
        let lane_width = if lane_y_sorted.len() > 1 {
            (lane_y_sorted[1] - lane_y_sorted[0]).abs()
        } else {
            3.5 // Default lane width
        };

        // Draw lane dividers between lanes (dashed lines)
        for i in 1..lane_y_sorted.len() {
            let lane_y = (lane_y_sorted[i] + lane_y_sorted[i - 1]) / 2.0;
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
        if !lane_y_sorted.is_empty() {
            let top_edge = lane_y_sorted.last().unwrap() + lane_width / 2.0;
            let bottom_edge = lane_y_sorted.first().unwrap() - lane_width / 2.0;

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
        }

        document.add(group)
    }

    /// Add lane markings for multi-road networks
    fn add_multi_road_lane_markings(&self, document: Document, network: &RoadNetwork) -> Document {
        let mut group = Group::new().set("id", "lane_markings");

        for road in &network.roads {
            let origin_x = road.origin.as_ref().map(|o| o.x).unwrap_or(0.0);
            let origin_y = road.origin.as_ref().map(|o| o.y).unwrap_or(0.0);
            let heading = road.heading.unwrap_or(0.0);

            let cos_h = heading.cos();
            let sin_h = heading.sin();

            // Perpendicular vector (90 degrees left of heading)
            let perp_x = -sin_h;
            let perp_y = cos_h;

            let road_width = road.num_lanes as f64 * road.lane_width;
            let half_width = road_width / 2.0;

            // Draw lane dividers (dashed lines between lanes)
            for lane_idx in 1..road.num_lanes {
                // Calculate offset from center
                let offset = half_width - lane_idx as f64 * road.lane_width;

                // Start and end points of lane divider
                let start_x = origin_x + perp_x * offset;
                let start_y = origin_y + perp_y * offset;
                let end_x = origin_x + road.length * cos_h + perp_x * offset;
                let end_y = origin_y + road.length * sin_h + perp_y * offset;

                let (svg_x1, svg_y1) = self.transform_coords(start_x, start_y);
                let (svg_x2, svg_y2) = self.transform_coords(end_x, end_y);

                let line = Line::new()
                    .set("x1", svg_x1)
                    .set("y1", svg_y1)
                    .set("x2", svg_x2)
                    .set("y2", svg_y2)
                    .set("stroke", COLOR_LANE_MARKING)
                    .set("stroke-width", 2)
                    .set("stroke-dasharray", "10,10")
                    .set("opacity", 0.6);
                group = group.add(line);
            }

            // Draw road edges (solid lines)
            // Left edge
            let left_start_x = origin_x + perp_x * half_width;
            let left_start_y = origin_y + perp_y * half_width;
            let left_end_x = origin_x + road.length * cos_h + perp_x * half_width;
            let left_end_y = origin_y + road.length * sin_h + perp_y * half_width;

            let (svg_lx1, svg_ly1) = self.transform_coords(left_start_x, left_start_y);
            let (svg_lx2, svg_ly2) = self.transform_coords(left_end_x, left_end_y);

            let left_line = Line::new()
                .set("x1", svg_lx1)
                .set("y1", svg_ly1)
                .set("x2", svg_lx2)
                .set("y2", svg_ly2)
                .set("stroke", COLOR_LANE_MARKING)
                .set("stroke-width", 3);
            group = group.add(left_line);

            // Right edge
            let right_start_x = origin_x - perp_x * half_width;
            let right_start_y = origin_y - perp_y * half_width;
            let right_end_x = origin_x + road.length * cos_h - perp_x * half_width;
            let right_end_y = origin_y + road.length * sin_h - perp_y * half_width;

            let (svg_rx1, svg_ry1) = self.transform_coords(right_start_x, right_start_y);
            let (svg_rx2, svg_ry2) = self.transform_coords(right_end_x, right_end_y);

            let right_line = Line::new()
                .set("x1", svg_rx1)
                .set("y1", svg_ry1)
                .set("x2", svg_rx2)
                .set("y2", svg_ry2)
                .set("stroke", COLOR_LANE_MARKING)
                .set("stroke-width", 3);
            group = group.add(right_line);
        }

        document.add(group)
    }

    /// Add vehicle trajectories as polylines
    fn add_trajectories(&self, document: Document) -> Document {
        let mut group = Group::new().set("id", "trajectories");

        for actor in &self.scenario.actors {
            // Build path data
            let mut path_data = String::new();
            for (i, state) in actor.states.iter().enumerate() {
                let (svg_x, svg_y) = self.transform_coords(state.position.x, state.position.y);
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
                let (svg_x, svg_y) = self.transform_coords(state.position.x, state.position.y);
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
                                        self.transform_coords(state.position.x, state.position.y);
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

            // Initial position (rectangle)
            if let Some(first_state) = actor.states.first() {
                let (svg_x, svg_y) =
                    self.transform_coords(first_state.position.x, first_state.position.y);
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

            // Final position (rectangle with different styling)
            if let Some(last_state) = actor.states.last() {
                let (svg_x, svg_y) =
                    self.transform_coords(last_state.position.x, last_state.position.y);
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

        document.add(group)
    }

    /// Add legend with color key
    fn add_legend(&self, document: Document) -> Document {
        let mut group = Group::new().set("id", "legend");

        let legend_x = self.config.canvas_width - self.config.margin - 150.0;
        let legend_y = self.config.canvas_height - self.config.margin - 80.0;

        // Legend background
        let bg = Rectangle::new()
            .set("x", legend_x)
            .set("y", legend_y)
            .set("width", 150)
            .set("height", 80)
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

        // Violation marker
        let violation_marker = Circle::new()
            .set("cx", legend_x + 16.0)
            .set("cy", legend_y + 59.0)
            .set("r", 6)
            .set("fill", "none")
            .set("stroke", COLOR_VIOLATION)
            .set("stroke-width", 2);
        group = group.add(violation_marker);

        let violation_text = Text::new("Safety Violation")
            .set("x", legend_x + 30.0)
            .set("y", legend_y + 62.0)
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
            State {
                time: 0.0,
                position: Position { x: 0.0, y: 5.0 },
                velocity: Velocity { vx: 10.0, vy: 0.0 },
                acceleration: Acceleration { ax: 0.0, ay: 0.0 },
                lane: 1,
                road_id: None,
            },
            State {
                time: 1.0,
                position: Position { x: 10.0, y: 5.0 },
                velocity: Velocity { vx: 10.0, vy: 0.0 },
                acceleration: Acceleration { ax: 0.0, ay: 0.0 },
                lane: 1,
                road_id: None,
            },
        ];

        let npc_states = vec![
            State {
                time: 0.0,
                position: Position { x: 5.0, y: 1.5 },
                velocity: Velocity { vx: 10.0, vy: 0.0 },
                acceleration: Acceleration { ax: 0.0, ay: 0.0 },
                lane: 0,
                road_id: None,
            },
            State {
                time: 1.0,
                position: Position { x: 15.0, y: 5.0 },
                velocity: Velocity { vx: 10.0, vy: 0.0 },
                acceleration: Acceleration { ax: 0.0, ay: 0.0 },
                lane: 1,
                road_id: None,
            },
        ];

        Scenario {
            scenario_id: "test-scenario-123".to_string(),
            scenario_type: "cut_in_left".to_string(),
            time_step: 1.0,
            duration: 1.0,
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
        let visualizer = SvgVisualizer::new(&scenario, None);

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
        let visualizer = SvgVisualizer::new(&scenario, None);

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

    #[test]
    fn test_export_multi_road_svg() {
        use crate::dsl::road_network::{ExtendedRoadSpec, RoadNetwork, WorldPosition};

        let road_a = ExtendedRoadSpec {
            id: "road_a".to_string(),
            num_lanes: 2,
            lane_width: 3.5,
            lane_directions: vec![1, 1],
            length: 100.0,
            origin: Some(WorldPosition { x: 0.0, y: 0.0 }),
            heading: Some(0.0),
        };

        let road_b = ExtendedRoadSpec {
            id: "road_b".to_string(),
            num_lanes: 2,
            lane_width: 3.5,
            lane_directions: vec![1, 1],
            length: 100.0,
            origin: Some(WorldPosition { x: 100.0, y: 0.0 }),
            heading: Some(std::f64::consts::FRAC_PI_4), // 45 degrees
        };

        let network = RoadNetwork::new(vec![road_a, road_b]);
        let scenario = create_test_scenario();

        let svg = export_to_svg_with_network(&scenario, &network).unwrap();

        // Verify basic structure
        assert!(svg.contains("<svg"));
        assert!(svg.contains("</svg>"));

        // Verify roads group exists
        assert!(svg.contains("id=\"roads\""));

        // Verify road names are present
        assert!(svg.contains("road_a"));
        assert!(svg.contains("road_b"));

        // Verify lane markings group exists
        assert!(svg.contains("id=\"lane_markings\""));
    }

    #[test]
    fn test_multi_road_config_bounds() {
        use crate::dsl::road_network::{ExtendedRoadSpec, RoadNetwork, WorldPosition};

        let road = ExtendedRoadSpec {
            id: "road".to_string(),
            num_lanes: 2,
            lane_width: 3.5,
            lane_directions: vec![1, 1],
            length: 200.0,
            origin: Some(WorldPosition { x: 50.0, y: 50.0 }),
            heading: Some(0.0),
        };

        let network = RoadNetwork::new(vec![road]);
        let scenario = create_test_scenario();

        let config = VisualizerConfig::from_scenario(&scenario, Some(&network));

        // The config should consider the road geometry
        // Road goes from (50, 50) to (250, 50), so bounds should include this
        assert!(config.x_min < 50.0);
    }

    #[test]
    fn test_export_svg_with_junction() {
        use crate::dsl::road_network::{
            ExtendedRoadSpec, Junction, JunctionSide, JunctionType, RoadNetwork, WorldPosition,
        };

        let main_road = ExtendedRoadSpec {
            id: "main".to_string(),
            num_lanes: 4,
            lane_width: 3.5,
            lane_directions: vec![1, 1, -1, -1],
            length: 400.0,
            origin: Some(WorldPosition { x: 0.0, y: 0.0 }),
            heading: Some(0.0),
        };

        let side_road = ExtendedRoadSpec {
            id: "side".to_string(),
            num_lanes: 2,
            lane_width: 3.0,
            lane_directions: vec![1, -1],
            length: 150.0,
            origin: Some(WorldPosition { x: 200.0, y: -50.0 }),
            heading: Some(std::f64::consts::FRAC_PI_2),
        };

        let junction = Junction {
            id: "t_junction".to_string(),
            junction_type: JunctionType::TJunction,
            main_road: Some("main".to_string()),
            incoming_roads: vec!["side".to_string()],
            position: Some(200.0),
            side: Some(JunctionSide::Right),
        };

        let network =
            RoadNetwork::new(vec![main_road, side_road]).with_junctions(vec![junction]);

        let scenario = create_test_scenario();
        let svg = export_to_svg_with_network(&scenario, &network).unwrap();

        // Verify basic structure
        assert!(svg.contains("<svg"));
        assert!(svg.contains("</svg>"));

        // Verify junction is rendered
        assert!(svg.contains("id=\"junctions\""));
        assert!(svg.contains("junction_0"));
        assert!(svg.contains("t_junction")); // Junction label
    }
}
