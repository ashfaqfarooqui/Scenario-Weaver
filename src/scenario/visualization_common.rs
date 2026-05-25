//! Shared visualization utilities for GIF and SVG exporters.
//!
//! Contains common logic for viewport computation, actor color mapping,
//! and violation time parsing.

use crate::scenario::model::Scenario;

/// Viewport bounds computed from scenario trajectory data.
///
/// Used by both GIF and SVG visualizers to determine coordinate transformations.
#[derive(Debug, Clone)]
pub struct ViewportBounds {
    pub x_min: f64,
    pub x_max: f64,
    pub y_min: f64,
    pub y_max: f64,
}

impl ViewportBounds {
    /// Compute viewport bounds from all actor trajectories in a scenario.
    ///
    /// Includes road extent and 10% padding on all sides.
    pub fn from_scenario(scenario: &Scenario) -> Self {
        let (mut x_min, mut x_max, mut y_min, mut y_max) =
            (f64::MAX, f64::MIN, f64::MAX, f64::MIN);

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

        Self { x_min, x_max, y_min, y_max }
    }

    /// Width of the viewport
    pub fn width(&self) -> f64 {
        self.x_max - self.x_min
    }

    /// Height of the viewport
    pub fn height(&self) -> f64 {
        self.y_max - self.y_min
    }
}

/// Actor role classification for color assignment.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ActorVisualRole {
    Ego,
    Npc,
    Pedestrian,
}

/// Determine an actor's visual role from its ID string.
///
/// Checks for "ego" or "pedestrian" substring (case-insensitive).
pub fn classify_actor(actor_id: &str) -> ActorVisualRole {
    let lower = actor_id.to_lowercase();
    if lower.contains("ego") {
        ActorVisualRole::Ego
    } else if lower.contains("pedestrian") {
        ActorVisualRole::Pedestrian
    } else {
        ActorVisualRole::Npc
    }
}

/// Parse the time value from a violation string.
///
/// Expects format: `"... t=X.Xs ..."` (e.g., `"TTC violation at t=3.5s: ego-npc: 2.1s < 3.0s"`)
pub fn parse_violation_time(violation: &str) -> Option<f64> {
    violation
        .split("t=")
        .nth(1)?
        .split('s')
        .next()?
        .parse::<f64>()
        .ok()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_classify_actor() {
        assert_eq!(classify_actor("ego"), ActorVisualRole::Ego);
        assert_eq!(classify_actor("Ego_vehicle"), ActorVisualRole::Ego);
        assert_eq!(classify_actor("npc"), ActorVisualRole::Npc);
        assert_eq!(classify_actor("npc_1"), ActorVisualRole::Npc);
        assert_eq!(classify_actor("pedestrian_0"), ActorVisualRole::Pedestrian);
        assert_eq!(classify_actor("Pedestrian"), ActorVisualRole::Pedestrian);
        assert_eq!(classify_actor("car_1"), ActorVisualRole::Npc);
    }

    #[test]
    fn test_parse_violation_time() {
        assert_eq!(
            parse_violation_time("TTC violation at t=3.5s: ego-npc: 2.1s < 3.0s"),
            Some(3.5)
        );
        assert_eq!(
            parse_violation_time("Distance violation at t=7.0s: ego-npc: 3.2m < 5.0m"),
            Some(7.0)
        );
        assert_eq!(parse_violation_time("no time here"), None);
        assert_eq!(parse_violation_time(""), None);
    }
}
