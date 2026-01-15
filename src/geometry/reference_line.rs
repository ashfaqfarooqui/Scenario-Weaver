//! Reference line for Frenet coordinate conversion
//!
//! Provides conversion between Frenet and Cartesian coordinate systems.
//! Currently supports straight roads only.

use crate::geometry::{CartesianPoint, FrenetPoint};
use serde::{Deserialize, Serialize};

/// Reference line for Frenet coordinate conversion
///
/// For straight roads, the reference line is defined by a start point, heading, and length.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ReferenceLine {
    pub start_x: f64,
    pub start_y: f64,
    pub heading: f64,  // Radians
    pub length: f64,   // Meters
}

impl ReferenceLine {
    /// Create a straight reference line
    pub fn straight(start_x: f64, start_y: f64, length: f64, heading: f64) -> Self {
        Self {
            start_x,
            start_y,
            heading,
            length,
        }
    }

    /// Convert Frenet to Cartesian coordinates (straight road)
    ///
    /// # Formula
    /// ```text
    /// x = start_x + s * cos(heading) - t * sin(heading)
    /// y = start_y + s * sin(heading) + t * cos(heading)
    /// ```
    pub fn frenet_to_cartesian(&self, frenet: &FrenetPoint) -> CartesianPoint {
        let x = self.start_x + frenet.s * self.heading.cos() - frenet.t * self.heading.sin();
        let y = self.start_y + frenet.s * self.heading.sin() + frenet.t * self.heading.cos();
        CartesianPoint::new(x, y)
    }

    /// Convert Cartesian to Frenet coordinates (straight road)
    ///
    /// # Formula
    /// ```text
    /// s = (x - start_x) * cos(heading) + (y - start_y) * sin(heading)
    /// t = -(x - start_x) * sin(heading) + (y - start_y) * cos(heading)
    /// ```
    pub fn cartesian_to_frenet(&self, cart: &CartesianPoint) -> FrenetPoint {
        let dx = cart.x - self.start_x;
        let dy = cart.y - self.start_y;
        let s = dx * self.heading.cos() + dy * self.heading.sin();
        let t = -dx * self.heading.sin() + dy * self.heading.cos();
        FrenetPoint::new(s, t)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_reference_line_creation() {
        let ref_line = ReferenceLine::straight(0.0, 0.0, 100.0, 0.0);
        assert_eq!(ref_line.start_x, 0.0);
        assert_eq!(ref_line.start_y, 0.0);
        assert_eq!(ref_line.length, 100.0);
        assert_eq!(ref_line.heading, 0.0);
    }

    #[test]
    fn test_frenet_to_cartesian_zero_heading() {
        let ref_line = ReferenceLine::straight(0.0, 0.0, 100.0, 0.0);

        // At origin, s=0, t=0 should give (0, 0)
        let frenet = FrenetPoint::new(0.0, 0.0);
        let cart = ref_line.frenet_to_cartesian(&frenet);
        assert!((cart.x - 0.0).abs() < 1e-9);
        assert!((cart.y - 0.0).abs() < 1e-9);

        // s=10, t=0 should give (10, 0) (along x-axis)
        let frenet = FrenetPoint::new(10.0, 0.0);
        let cart = ref_line.frenet_to_cartesian(&frenet);
        assert!((cart.x - 10.0).abs() < 1e-9);
        assert!((cart.y - 0.0).abs() < 1e-9);

        // s=0, t=3.5 should give (0, 3.5) (lateral offset)
        let frenet = FrenetPoint::new(0.0, 3.5);
        let cart = ref_line.frenet_to_cartesian(&frenet);
        assert!((cart.x - 0.0).abs() < 1e-9);
        assert!((cart.y - 3.5).abs() < 1e-9);
    }

    #[test]
    fn test_cartesian_to_frenet_zero_heading() {
        let ref_line = ReferenceLine::straight(0.0, 0.0, 100.0, 0.0);

        // (10, 0) should give s=10, t=0
        let cart = CartesianPoint::new(10.0, 0.0);
        let frenet = ref_line.cartesian_to_frenet(&cart);
        assert!((frenet.s - 10.0).abs() < 1e-9);
        assert!((frenet.t - 0.0).abs() < 1e-9);

        // (0, 3.5) should give s=0, t=3.5
        let cart = CartesianPoint::new(0.0, 3.5);
        let frenet = ref_line.cartesian_to_frenet(&cart);
        assert!((frenet.s - 0.0).abs() < 1e-9);
        assert!((frenet.t - 3.5).abs() < 1e-9);
    }

    #[test]
    fn test_round_trip_conversion() {
        let ref_line = ReferenceLine::straight(0.0, 0.0, 100.0, 0.0);

        // Test various points
        let test_cases = vec![
            (0.0, 0.0),
            (10.0, 0.0),
            (50.0, 3.5),
            (100.0, -3.5),
        ];

        for (s, t) in test_cases {
            let original = FrenetPoint::new(s, t);
            let cart = ref_line.frenet_to_cartesian(&original);
            let converted = ref_line.cartesian_to_frenet(&cart);

            assert!((converted.s - s).abs() < 1e-9, "s mismatch: {} vs {}", converted.s, s);
            assert!((converted.t - t).abs() < 1e-9, "t mismatch: {} vs {}", converted.t, t);
        }
    }
}
