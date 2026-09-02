//! Shared visualization utilities for GIF and SVG exporters.
//!
//! Contains common logic for viewport computation, actor color mapping,
//! projection into pixel/canvas space, and violation time parsing.
//!
//! `svg_visualizer.rs` and `gif_animator.rs` render the same trajectory data
//! into two different targets (an SVG `&str` colour and an `image::Rgb<u8>`
//! pixel, an `f64` and an `i32` coordinate space). Every value with visual
//! meaning is defined exactly once here, in a representation-neutral form,
//! and each renderer adapts it to its own type at the call site rather than
//! keeping a second copy of the literal.

use crate::scenario::model::Scenario;

/// A colour defined once, in both forms its two consumers need:
/// an SVG hex string and an `[u8; 3]` RGB triple (the `image` crate's `Rgb<u8>`
/// is just `Rgb([u8; 3])`, so callers construct it with one wrapping step).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ColorDef {
    pub hex: &'static str,
    pub rgb: [u8; 3],
}

// The full visualization palette. Every colour used by either renderer is
// defined here exactly once; `svg_visualizer.rs` reads `.hex`, `gif_animator.rs`
// reads `.rgb`. Do not add a colour to either renderer directly.
pub const COLOR_EGO: ColorDef = ColorDef {
    hex: "#4CAF50",
    rgb: [76, 175, 80],
};
pub const COLOR_NPC: ColorDef = ColorDef {
    hex: "#2196F3",
    rgb: [33, 150, 243],
};
pub const COLOR_PEDESTRIAN: ColorDef = ColorDef {
    hex: "#FF9800",
    rgb: [255, 152, 0],
};
pub const COLOR_VIOLATION: ColorDef = ColorDef {
    hex: "#F44336",
    rgb: [244, 67, 54],
};
pub const COLOR_EGO_PATH: ColorDef = ColorDef {
    hex: "#8BC34A",
    rgb: [139, 195, 74],
};
pub const COLOR_NPC_PATH: ColorDef = ColorDef {
    hex: "#64B5F6",
    rgb: [100, 181, 246],
};
pub const COLOR_PEDESTRIAN_PATH: ColorDef = ColorDef {
    hex: "#FFB74D",
    rgb: [255, 183, 77],
};
pub const COLOR_ROAD: ColorDef = ColorDef {
    hex: "#2A2A2A",
    rgb: [42, 42, 42],
};
pub const COLOR_LANE_MARKING: ColorDef = ColorDef {
    hex: "#FFFFFF",
    rgb: [255, 255, 255],
};
pub const COLOR_TEXT: ColorDef = ColorDef {
    hex: "#333333",
    rgb: [51, 51, 51],
};
pub const COLOR_BACKGROUND: ColorDef = ColorDef {
    hex: "#F5F5F5",
    rgb: [245, 245, 245],
};

/// Vehicle marker footprint, in canvas/pixel units, shared by both renderers.
pub const VEHICLE_LENGTH: u32 = 12;
pub const VEHICLE_WIDTH: u32 = 6;

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

        Self {
            x_min,
            x_max,
            y_min,
            y_max,
        }
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

/// Determine an actor's visual role from its `role` field.
///
/// Keys off the actor's declared role ("ego" / "npc" / "pedestrian",
/// case-insensitive), **not** the actor id. An id is free text chosen by the
/// scenario author — an actor named `npc_ego_follower` has "ego" as an id
/// substring while its role is `"npc"`; classifying by id substring drew it
/// green (as ego) while every other exporter tagged it NPC from its role.
/// Classifying by role keeps the visualizers consistent with
/// `openlabel_exporter.rs`, which has always read `actor.role`.
pub fn classify_actor(role: &str) -> ActorVisualRole {
    match role.to_lowercase().as_str() {
        "ego" => ActorVisualRole::Ego,
        "pedestrian" => ActorVisualRole::Pedestrian,
        _ => ActorVisualRole::Npc,
    }
}

/// Scenario-space-to-render-space affine projection, shared by both
/// renderers. `x_scale`/`y_scale` map scenario metres to canvas units;
/// `x_min`/`y_max` are the scenario-space origin the projection is anchored
/// to (Y is flipped: increasing scenario `y` moves toward the canvas top).
#[derive(Debug, Clone, Copy)]
pub struct Projection {
    pub x_min: f64,
    pub y_max: f64,
    pub x_scale: f64,
    pub y_scale: f64,
}

impl Projection {
    /// Build a projection that maps `bounds` onto a `drawable_width` x
    /// `drawable_height` render-space rectangle anchored at the caller's own
    /// origin (margins differ between the SVG and GIF canvases, so the
    /// origin is supplied at transform time, not baked in here).
    pub fn new(bounds: &ViewportBounds, drawable_width: f64, drawable_height: f64) -> Self {
        Self {
            x_min: bounds.x_min,
            y_max: bounds.y_max,
            x_scale: drawable_width / bounds.width(),
            y_scale: drawable_height / bounds.height(),
        }
    }

    /// Map a scenario-space `(x, y)` into render space, offset from
    /// `(origin_x, origin_y)`.
    pub fn transform(&self, x: f64, y: f64, origin_x: f64, origin_y: f64) -> (f64, f64) {
        let out_x = origin_x + (x - self.x_min) * self.x_scale;
        // Flip Y-axis: higher scenario Y should be at the top (lower render Y).
        let out_y = origin_y + (self.y_max - y) * self.y_scale;
        (out_x, out_y)
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
    fn test_classify_actor_keys_off_role() {
        assert_eq!(classify_actor("ego"), ActorVisualRole::Ego);
        assert_eq!(classify_actor("Ego"), ActorVisualRole::Ego);
        assert_eq!(classify_actor("npc"), ActorVisualRole::Npc);
        assert_eq!(classify_actor("pedestrian"), ActorVisualRole::Pedestrian);
        assert_eq!(classify_actor("Pedestrian"), ActorVisualRole::Pedestrian);
        assert_eq!(classify_actor("unknown"), ActorVisualRole::Npc);
    }

    /// Regression test for M15: classification must not be fooled by an id
    /// substring. An actor named `npc_ego_follower` with role `"npc"` used to
    /// be drawn green (as ego) because `classify_actor` matched "ego" inside
    /// the id, while `openlabel_exporter.rs` tagged the same actor NPC from
    /// its role — the two artifacts disagreed about what the actor was.
    #[test]
    fn test_classify_actor_not_fooled_by_ego_substring_in_id() {
        // The id contains "ego", but the role does not — role must win.
        let id = "npc_ego_follower";
        assert!(id.contains("ego"));
        assert_eq!(classify_actor("npc"), ActorVisualRole::Npc);
    }

    #[test]
    fn test_projection_round_trip() {
        let bounds = ViewportBounds {
            x_min: 0.0,
            x_max: 100.0,
            y_min: 0.0,
            y_max: 10.0,
        };
        let projection = Projection::new(&bounds, 1000.0, 100.0);
        assert!((projection.x_scale - 10.0).abs() < 1e-9);
        assert!((projection.y_scale - 10.0).abs() < 1e-9);

        // x increases rightward from the origin.
        let (x0, _) = projection.transform(0.0, 0.0, 5.0, 5.0);
        assert!((x0 - 5.0).abs() < 1e-9);
        let (x1, _) = projection.transform(10.0, 0.0, 5.0, 5.0);
        assert!((x1 - 105.0).abs() < 1e-9);

        // y is flipped: scenario y_max maps to the origin (canvas top).
        let (_, y_top) = projection.transform(0.0, 10.0, 5.0, 5.0);
        assert!((y_top - 5.0).abs() < 1e-9);
        let (_, y_bottom) = projection.transform(0.0, 0.0, 5.0, 5.0);
        assert!((y_bottom - 105.0).abs() < 1e-9);
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
