//! GIF animation export functionality
//!
//! Converts internal Scenario data structures to animated GIF showing
//! vehicle trajectories evolving over time with real-time metrics overlay.

use crate::dsl::road_network::RoadNetwork;
use crate::error::{Result, ScenarioGenError};
use crate::scenario::model::{Scenario, Velocity};
use ab_glyph::{FontArc, PxScale};
use gif::{Encoder, Frame, Repeat};
use image::{Rgb, RgbImage};
use imageproc::drawing::{
    draw_filled_circle_mut, draw_filled_rect_mut, draw_hollow_circle_mut, draw_line_segment_mut,
    draw_text_mut,
};
use imageproc::rect::Rect;

// Canvas dimensions
const CANVAS_WIDTH: u32 = 1200;
const CANVAS_HEIGHT: u32 = 600;
const MARGIN: u32 = 80;
const METRICS_OVERLAY_HEIGHT: u32 = 120;
const ROAD_AREA_TOP: u32 = METRICS_OVERLAY_HEIGHT;

// Animation settings
const _TARGET_FPS: u16 = 10; // For reference: 10 FPS
const FRAME_DELAY_CENTISECONDS: u16 = 10; // 100ms per frame = 10 FPS

// Colors (match SVG visualizer)
const COLOR_EGO: Rgb<u8> = Rgb([76, 175, 80]); // #4CAF50 Green
const COLOR_NPC: Rgb<u8> = Rgb([33, 150, 243]); // #2196F3 Blue
const COLOR_VIOLATION: Rgb<u8> = Rgb([244, 67, 54]); // #F44336 Red
const COLOR_EGO_TRAIL: Rgb<u8> = Rgb([139, 195, 74]); // #8BC34A Light green
const COLOR_NPC_TRAIL: Rgb<u8> = Rgb([100, 181, 246]); // #64B5F6 Light blue
const COLOR_ROAD: Rgb<u8> = Rgb([42, 42, 42]); // #2A2A2A Dark gray
const COLOR_LANE_MARKING: Rgb<u8> = Rgb([255, 255, 255]); // #FFFFFF White
const COLOR_BACKGROUND: Rgb<u8> = Rgb([245, 245, 245]); // #F5F5F5 Light gray
const COLOR_TEXT: Rgb<u8> = Rgb([51, 51, 51]); // #333333 Dark gray text
const COLOR_JUNCTION: Rgb<u8> = Rgb([61, 61, 61]); // #3D3D3D Slightly lighter than road

// Vehicle dimensions (in pixels)
const VEHICLE_LENGTH: u32 = 12;
const VEHICLE_WIDTH: u32 = 6;

/// Export a scenario to animated GIF format
///
/// Generates a GIF animation showing vehicle trajectories evolving over time
/// at 10 FPS with real-time metrics displayed as text overlay.
///
/// # Example
/// ```no_run
/// use carla_scenario_generator::{generate_single_scenario, export_scenario_to_gif};
///
/// let yaml = std::fs::read_to_string("scenario.yaml").unwrap();
/// let scenario = generate_single_scenario(&yaml).unwrap();
/// let gif_bytes = export_scenario_to_gif(&scenario).unwrap();
/// std::fs::write("scenario.gif", gif_bytes).unwrap();
/// ```
pub fn export_to_gif(scenario: &Scenario) -> Result<Vec<u8>> {
    let animator = GifAnimator::new(scenario, None);
    animator.generate()
}

/// Export a scenario with road network to animated GIF format
///
/// This function renders multi-road scenarios with junctions at different
/// positions and headings.
pub fn export_to_gif_with_network(scenario: &Scenario, network: &RoadNetwork) -> Result<Vec<u8>> {
    let animator = GifAnimator::new(scenario, Some(network));
    animator.generate()
}

/// Configuration for GIF animation
#[derive(Debug)]
struct AnimatorConfig {
    canvas_width: u32,
    canvas_height: u32,
    margin: u32,
    road_area_top: u32,
    x_scale: f64,
    y_scale: f64,
    x_min: f64,
    y_max: f64,
    num_frames: usize,
}

impl AnimatorConfig {
    fn from_scenario(scenario: &Scenario) -> Self {
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

        // Add padding (10% on each side)
        let x_range = x_max - x_min;
        let y_range = y_max - y_min;
        x_min -= x_range * 0.1;
        x_max += x_range * 0.1;
        y_min -= y_range * 0.1;
        y_max += y_range * 0.1;

        // Compute scales
        let drawable_width = (CANVAS_WIDTH - 2 * MARGIN) as f64;
        let drawable_height = (CANVAS_HEIGHT - ROAD_AREA_TOP - MARGIN) as f64;
        let x_scale = drawable_width / (x_max - x_min);
        let y_scale = drawable_height / (y_max - y_min);

        // Number of frames = number of states
        let num_frames = if !scenario.actors.is_empty() {
            scenario.actors[0].states.len()
        } else {
            0
        };

        Self {
            canvas_width: CANVAS_WIDTH,
            canvas_height: CANVAS_HEIGHT,
            margin: MARGIN,
            road_area_top: ROAD_AREA_TOP,
            x_scale,
            y_scale,
            x_min,
            y_max,
            num_frames,
        }
    }
}

/// Main GIF animator
struct GifAnimator<'a> {
    scenario: &'a Scenario,
    config: AnimatorConfig,
    font: FontArc,
    network: Option<&'a RoadNetwork>,
}

impl<'a> GifAnimator<'a> {
    fn new(scenario: &'a Scenario, network: Option<&'a RoadNetwork>) -> Self {
        let config = AnimatorConfig::from_scenario(scenario);

        // Load embedded font
        let font_data: &[u8] = include_bytes!("../../assets/DejaVuSans.ttf");
        let font = FontArc::try_from_slice(font_data).expect("Failed to load embedded font");

        Self {
            scenario,
            network,
            config,
            font,
        }
    }

    /// Generate the complete GIF animation
    fn generate(&self) -> Result<Vec<u8>> {
        let mut buffer = Vec::new();
        let mut encoder = Encoder::new(
            &mut buffer,
            self.config.canvas_width as u16,
            self.config.canvas_height as u16,
            &[],
        )
        .map_err(|e| ScenarioGenError::GifExport(format!("Failed to create GIF encoder: {}", e)))?;

        encoder
            .set_repeat(Repeat::Infinite)
            .map_err(|e| ScenarioGenError::GifExport(format!("Failed to set repeat: {}", e)))?;

        // Pre-render static background once (road, lanes) for performance
        let static_background = self.render_static_background();

        // Generate frame for each time step
        for frame_idx in 0..self.config.num_frames {
            let image = self.render_frame(frame_idx, &static_background);
            let mut frame = Frame::from_rgb_speed(
                self.config.canvas_width as u16,
                self.config.canvas_height as u16,
                &image.into_raw(),
                10,
            );
            frame.delay = FRAME_DELAY_CENTISECONDS;
            encoder.write_frame(&frame).map_err(|e| {
                ScenarioGenError::GifExport(format!("Failed to write frame: {}", e))
            })?;
        }

        drop(encoder); // Finalize GIF
        Ok(buffer)
    }

    /// Render static background elements (background, road, lane markings)
    /// This is rendered once and cloned for each frame for performance
    fn render_static_background(&self) -> RgbImage {
        let mut image = RgbImage::new(self.config.canvas_width, self.config.canvas_height);

        self.draw_background(&mut image);
        self.draw_road_surface(&mut image);
        self.draw_junctions(&mut image);
        self.draw_lane_markings(&mut image);

        image
    }

    /// Render a single frame at given time index
    fn render_frame(&self, frame_idx: usize, static_background: &RgbImage) -> RgbImage {
        // Clone the pre-rendered static background
        let mut image = static_background.clone();

        // Only render dynamic elements
        self.draw_trajectory_trails(&mut image, frame_idx);
        self.draw_vehicles(&mut image, frame_idx);
        self.draw_violations(&mut image, frame_idx);
        self.draw_metrics_overlay(&mut image, frame_idx);

        image
    }

    /// Transform scenario coordinates to image pixel coordinates
    fn transform_coords(&self, scenario_x: f64, scenario_y: f64) -> (i32, i32) {
        let px = self.config.margin as f64 + (scenario_x - self.config.x_min) * self.config.x_scale;
        // Flip Y-axis: higher scenario Y should be at top (lower pixel Y)
        let py = self.config.road_area_top as f64
            + (self.config.y_max - scenario_y) * self.config.y_scale;
        (px as i32, py as i32)
    }

    /// Get color for an actor based on role
    fn get_actor_color(&self, actor_id: &str) -> Rgb<u8> {
        if actor_id.to_lowercase().contains("ego") {
            COLOR_EGO
        } else {
            COLOR_NPC
        }
    }

    /// Get trail color for an actor
    fn get_trail_color(&self, actor_id: &str) -> Rgb<u8> {
        if actor_id.to_lowercase().contains("ego") {
            COLOR_EGO_TRAIL
        } else {
            COLOR_NPC_TRAIL
        }
    }

    /// Add background rectangle
    fn draw_background(&self, image: &mut RgbImage) {
        for pixel in image.pixels_mut() {
            *pixel = COLOR_BACKGROUND;
        }
    }

    /// Add road surface
    fn draw_road_surface(&self, image: &mut RgbImage) {
        let road_rect = Rect::at(self.config.margin as i32, self.config.road_area_top as i32)
            .of_size(
                self.config.canvas_width - 2 * self.config.margin,
                self.config.canvas_height - self.config.road_area_top - self.config.margin,
            );
        draw_filled_rect_mut(image, road_rect, COLOR_ROAD);
    }

    /// Add junctions to the image
    fn draw_junctions(&self, image: &mut RgbImage) {
        // Only render junctions if we have a road network
        let network = match self.network {
            Some(n) if !n.junctions.is_empty() => n,
            _ => return,
        };

        use crate::dsl::road_network::{
            CrossroadsGeometry, JunctionType, TJunctionGeometry,
        };

        for junction in &network.junctions {
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

            // Transform corners to image coordinates
            let img_corners: Vec<(i32, i32)> = corners
                .iter()
                .map(|&(x, y)| self.transform_coords(x, y))
                .collect();

            // Find bounding box for the junction
            let min_x = img_corners.iter().map(|(x, _)| *x).min().unwrap_or(0);
            let max_x = img_corners.iter().map(|(x, _)| *x).max().unwrap_or(0);
            let min_y = img_corners.iter().map(|(_, y)| *y).min().unwrap_or(0);
            let max_y = img_corners.iter().map(|(_, y)| *y).max().unwrap_or(0);

            // Draw a filled rectangle for the junction (simplified from polygon)
            if max_x > min_x && max_y > min_y {
                let junction_rect = Rect::at(min_x, min_y)
                    .of_size((max_x - min_x) as u32, (max_y - min_y) as u32);
                draw_filled_rect_mut(image, junction_rect, COLOR_JUNCTION);
            }
        }
    }

    /// Add lane markings
    fn draw_lane_markings(&self, image: &mut RgbImage) {
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
            let (_, py) = self.transform_coords(0.0, lane_y);

            // Draw dashed line
            let x_start = self.config.margin as i32;
            let x_end = (self.config.canvas_width - self.config.margin) as i32;
            self.draw_dashed_line(image, x_start, py, x_end, py, COLOR_LANE_MARKING);
        }

        // Draw road edges (solid lines)
        if !lane_y_sorted.is_empty() {
            let top_edge = lane_y_sorted.last().unwrap() + lane_width / 2.0;
            let bottom_edge = lane_y_sorted.first().unwrap() - lane_width / 2.0;

            let (_, top_py) = self.transform_coords(0.0, top_edge);
            let (_, bottom_py) = self.transform_coords(0.0, bottom_edge);

            let x_start = self.config.margin as f32;
            let x_end = (self.config.canvas_width - self.config.margin) as f32;

            // Top edge
            draw_line_segment_mut(
                image,
                (x_start, top_py as f32),
                (x_end, top_py as f32),
                COLOR_LANE_MARKING,
            );

            // Bottom edge
            draw_line_segment_mut(
                image,
                (x_start, bottom_py as f32),
                (x_end, bottom_py as f32),
                COLOR_LANE_MARKING,
            );
        }
    }

    /// Draw dashed line
    fn draw_dashed_line(
        &self,
        image: &mut RgbImage,
        x1: i32,
        y1: i32,
        x2: i32,
        y2: i32,
        color: Rgb<u8>,
    ) {
        let dash_length = 10;
        let gap_length = 10;
        let mut x = x1;

        while x < x2 {
            let end_x = (x + dash_length).min(x2);
            draw_line_segment_mut(
                image,
                (x as f32, y1 as f32),
                (end_x as f32, y2 as f32),
                color,
            );
            x = end_x + gap_length;
        }
    }

    /// Draw trajectory trails with fading effect
    fn draw_trajectory_trails(&self, image: &mut RgbImage, current_frame: usize) {
        for actor in &self.scenario.actors {
            let trail_color = self.get_trail_color(&actor.id);

            // Draw trail from start to current frame
            for t in 0..=current_frame {
                if t >= actor.states.len() {
                    break;
                }

                let state = &actor.states[t];
                let (px, py) = self.transform_coords(state.position.x, state.position.y);

                // Calculate alpha for fading effect (range 0.3 to 1.0 so oldest positions are visible)
                let alpha = if current_frame > 0 {
                    0.3 + 0.7 * (t as f64) / (current_frame as f64)
                } else {
                    1.0
                };

                // Blend trail color with background
                let faded_color = self.blend_color(trail_color, alpha);

                // Draw small circle at this position
                if px >= 0
                    && py >= 0
                    && px < self.config.canvas_width as i32
                    && py < self.config.canvas_height as i32
                {
                    draw_filled_circle_mut(image, (px, py), 2, faded_color);
                }
            }
        }
    }

    /// Blend color with background using alpha
    fn blend_color(&self, color: Rgb<u8>, alpha: f64) -> Rgb<u8> {
        let bg = COLOR_BACKGROUND;
        Rgb([
            (color[0] as f64 * alpha + bg[0] as f64 * (1.0 - alpha)) as u8,
            (color[1] as f64 * alpha + bg[1] as f64 * (1.0 - alpha)) as u8,
            (color[2] as f64 * alpha + bg[2] as f64 * (1.0 - alpha)) as u8,
        ])
    }

    /// Draw vehicles at current frame position
    fn draw_vehicles(&self, image: &mut RgbImage, frame_idx: usize) {
        for actor in &self.scenario.actors {
            if frame_idx >= actor.states.len() {
                continue;
            }

            let state = &actor.states[frame_idx];
            let color = self.get_actor_color(&actor.id);
            let (px, py) = self.transform_coords(state.position.x, state.position.y);

            // Draw vehicle rectangle
            let rect = Rect::at(
                px - VEHICLE_LENGTH as i32 / 2,
                py - VEHICLE_WIDTH as i32 / 2,
            )
            .of_size(VEHICLE_LENGTH, VEHICLE_WIDTH);
            draw_filled_rect_mut(image, rect, color);

            // Draw heading arrow
            self.draw_heading_arrow(image, px, py, &state.velocity, color);
        }
    }

    /// Draw heading arrow based on velocity
    fn draw_heading_arrow(
        &self,
        image: &mut RgbImage,
        px: i32,
        py: i32,
        velocity: &Velocity,
        _color: Rgb<u8>,
    ) {
        // Calculate heading from velocity
        let speed = (velocity.vx * velocity.vx + velocity.vy * velocity.vy).sqrt();
        if speed < 0.1 {
            return; // Don't draw arrow if nearly stationary
        }

        // Normalize velocity
        let vx_norm = velocity.vx / speed;
        let vy_norm = velocity.vy / speed;

        // Arrow length
        let arrow_len = 8.0;

        // Arrow tip position
        let tip_x = px as f32 + (vx_norm * arrow_len) as f32;
        let tip_y = py as f32 - (vy_norm * arrow_len) as f32; // Flip Y

        // Draw arrow line
        draw_line_segment_mut(image, (px as f32, py as f32), (tip_x, tip_y), COLOR_TEXT);
    }

    /// Draw violation markers
    fn draw_violations(&self, image: &mut RgbImage, frame_idx: usize) {
        let current_time = frame_idx as f64 * self.scenario.time_step;

        for violation in &self.scenario.validation.safety_violations {
            if let Some(violation_time) = self.parse_violation_time(violation) {
                // Check if this frame is at the violation time
                if (violation_time - current_time).abs() < self.scenario.time_step * 0.5 {
                    // Parse which actors are involved in this violation
                    let involved_actors = self.parse_violation_actors(violation);

                    // Only draw circles around actors involved in the violation
                    for actor in &self.scenario.actors {
                        // Check if this actor is involved in the violation
                        let is_involved = involved_actors.is_empty()
                            || involved_actors
                                .iter()
                                .any(|a| actor.id.to_lowercase().contains(&a.to_lowercase()));

                        if is_involved && frame_idx < actor.states.len() {
                            let state = &actor.states[frame_idx];
                            let (px, py) =
                                self.transform_coords(state.position.x, state.position.y);
                            draw_hollow_circle_mut(image, (px, py), 15, COLOR_VIOLATION);
                        }
                    }
                }
            }
        }
    }

    /// Parse violation time from violation string
    fn parse_violation_time(&self, violation: &str) -> Option<f64> {
        // Parse format: "... at t=X.Xs ..."
        violation
            .split("t=")
            .nth(1)?
            .split('s')
            .next()?
            .parse::<f64>()
            .ok()
    }

    /// Parse actor names from violation string
    /// Format: "TTC violation at t=3.5s: ego-npc: 2.1s < 3.0s"
    fn parse_violation_actors(&self, violation: &str) -> Vec<String> {
        // Look for pattern "actor1-actor2:" after the time
        if let Some(after_time) = violation.split("s: ").nth(1) {
            if let Some(actors_pair) = after_time.split(':').next() {
                return actors_pair
                    .split('-')
                    .map(|s| s.trim().to_string())
                    .collect();
            }
        }
        vec![]
    }

    /// Draw metrics overlay
    fn draw_metrics_overlay(&self, image: &mut RgbImage, frame_idx: usize) {
        let current_time = frame_idx as f64 * self.scenario.time_step;
        let (current_ttc, current_distance) = self.compute_frame_metrics(frame_idx);

        let scale = PxScale::from(24.0);

        // Time display
        let time_text = format!(
            "Time: {:.1}s / {:.1}s",
            current_time, self.scenario.duration
        );
        draw_text_mut(image, COLOR_TEXT, 20, 20, scale, &self.font, &time_text);

        // Current TTC
        let ttc_text = if current_ttc < 999.0 {
            format!("TTC: {:.2}s", current_ttc)
        } else {
            "TTC: N/A".to_string()
        };
        draw_text_mut(image, COLOR_TEXT, 20, 50, scale, &self.font, &ttc_text);

        // Current distance
        let dist_text = format!("Distance: {:.2}m", current_distance);
        draw_text_mut(image, COLOR_TEXT, 20, 80, scale, &self.font, &dist_text);

        // Overall status (right side)
        let status_text = if self.scenario.validation.all_constraints_satisfied {
            "SAFE"
        } else {
            "VIOLATED"
        };
        let status_color = if self.scenario.validation.all_constraints_satisfied {
            COLOR_EGO
        } else {
            COLOR_VIOLATION
        };
        let status_scale = PxScale::from(32.0);
        draw_text_mut(
            image,
            status_color,
            (self.config.canvas_width - 150) as i32,
            40,
            status_scale,
            &self.font,
            status_text,
        );
    }

    /// Compute TTC and distance for vehicles at specific frame
    fn compute_frame_metrics(&self, frame_idx: usize) -> (f64, f64) {
        let mut min_ttc = f64::INFINITY;
        let mut min_distance = f64::INFINITY;

        // Pairwise comparison
        for i in 0..self.scenario.actors.len() {
            for j in (i + 1)..self.scenario.actors.len() {
                if frame_idx >= self.scenario.actors[i].states.len()
                    || frame_idx >= self.scenario.actors[j].states.len()
                {
                    continue;
                }

                let state1 = &self.scenario.actors[i].states[frame_idx];
                let state2 = &self.scenario.actors[j].states[frame_idx];

                // Only compute metrics if in same lane
                if state1.lane == state2.lane {
                    // Compute distance
                    let distance = (state1.position.x - state2.position.x).abs();
                    min_distance = min_distance.min(distance);

                    // Compute TTC
                    let rel_vel = (state1.velocity.vx - state2.velocity.vx).abs();
                    if rel_vel > 0.01 {
                        let ttc = distance / rel_vel;
                        min_ttc = min_ttc.min(ttc);
                    }
                }
            }
        }

        (min_ttc, min_distance)
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
    fn test_animator_config_creation() {
        let scenario = create_test_scenario();
        let config = AnimatorConfig::from_scenario(&scenario);

        assert_eq!(config.canvas_width, CANVAS_WIDTH);
        assert_eq!(config.num_frames, 2); // 2 states
        assert!(config.x_scale > 0.0);
        assert!(config.y_scale > 0.0);
    }

    #[test]
    fn test_coordinate_transformation() {
        let scenario = create_test_scenario();
        let animator = GifAnimator::new(&scenario, None);

        let (px, py) = animator.transform_coords(5.0, 3.0);
        assert!(px >= animator.config.margin as i32);
        assert!(py >= animator.config.road_area_top as i32);
    }

    #[test]
    fn test_vehicle_color_coding() {
        let scenario = create_test_scenario();
        let animator = GifAnimator::new(&scenario, None);

        assert_eq!(animator.get_actor_color("ego"), COLOR_EGO);
        assert_eq!(animator.get_actor_color("npc"), COLOR_NPC);
        assert_eq!(animator.get_actor_color("ego_vehicle"), COLOR_EGO);
    }

    #[test]
    fn test_frame_metrics_computation() {
        let scenario = create_test_scenario();
        let animator = GifAnimator::new(&scenario, None);

        let (ttc, distance) = animator.compute_frame_metrics(0);
        // Different lanes at frame 0, so metrics should be infinity
        assert!(ttc == f64::INFINITY);
        assert!(distance == f64::INFINITY);
    }

    #[test]
    fn test_violation_time_parsing() {
        let scenario = create_test_scenario();
        let animator = GifAnimator::new(&scenario, None);

        let violation = "TTC violation at t=3.5s: ego-npc: 2.1s < 3.0s";
        let time = animator.parse_violation_time(violation);
        assert_eq!(time, Some(3.5));
    }

    #[test]
    fn test_export_to_gif_basic() {
        let scenario = create_test_scenario();
        let gif_bytes = export_to_gif(&scenario).unwrap();

        // Verify GIF header (GIF89a)
        assert_eq!(&gif_bytes[0..3], b"GIF");
        assert_eq!(&gif_bytes[3..6], b"89a");
        assert!(gif_bytes.len() > 1024); // Should be at least 1KB
    }
}
