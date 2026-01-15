//! Frenet and Cartesian coordinate point types

use serde::{Deserialize, Serialize};

/// Frenet coordinates: longitudinal (s) and lateral (t) positions
///
/// In the Frenet frame:
/// - `s` is the longitudinal position along the reference line (meters)
/// - `t` is the lateral offset from the reference line (meters)
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct FrenetPoint {
    pub s: f64,
    pub t: f64,
}

impl FrenetPoint {
    pub fn new(s: f64, t: f64) -> Self {
        Self { s, t }
    }
}

/// Cartesian coordinates: x and y positions
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct CartesianPoint {
    pub x: f64,
    pub y: f64,
}

impl CartesianPoint {
    pub fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_frenet_point_creation() {
        let point = FrenetPoint::new(10.0, 2.5);
        assert_eq!(point.s, 10.0);
        assert_eq!(point.t, 2.5);
    }

    #[test]
    fn test_cartesian_point_creation() {
        let point = CartesianPoint::new(5.0, 3.0);
        assert_eq!(point.x, 5.0);
        assert_eq!(point.y, 3.0);
    }
}
