//! Trait definition for coordinate system encoders
//!
//! This module defines the `CoordinateEncoder` trait that abstracts over
//! different coordinate systems (Cartesian, Frenet, etc.).

use z3::ast::{Bool, Int, Real};
use z3::Model;

use crate::dsl::types::ScenarioSpec;
use crate::error::Result;
use crate::scenario::ActorTrajectory;
use crate::solver::backend::Z3Backend;

/// Trait for coordinate-system-specific encoding logic
///
/// Each coordinate system (Cartesian, Frenet) implements this trait
/// to provide its own variable creation, kinematics, and constraint encoding.
///
/// # Method Usage Notes
///
/// Some methods defined in this trait are optional or may be no-ops depending
/// on the coordinate system implementation:
///
/// - `encode_velocity_constraints()`: Not currently called by the main encoder.
///   CartesianEncoder duplicates this logic in `encode_lane_velocity_constraints()`.
///   Implementations may leave this as a no-op.
///
/// - `encode_acceleration_constraints()`: CartesianEncoder is a no-op because
///   acceleration constraints are encoded in `encode_kinematics()`. BicycleEncoder
///   uses this to enforce acceleration bounds separately.
///
/// - `encode_lateral_velocity_bounds()`: BicycleEncoder is a no-op because lateral
///   velocity is implicitly constrained by steering angle and heading bounds.
pub trait CoordinateEncoder<B: Z3Backend> {
    // === Core Encoding ===

    /// Create Z3 variables for all actors across the time horizon
    fn create_variables(&mut self, horizon: usize, spec: &ScenarioSpec);

    /// Encode kinematic equations (velocity/acceleration integration)
    fn encode_kinematics(&mut self, dt: f64);

    /// Encode initial conditions from scenario specification
    fn encode_initial_conditions(&mut self);

    /// Encode velocity constraints (min/max bounds)
    ///
    /// Note: This method is not currently called by the main encoder pipeline.
    /// Velocity constraints are typically encoded in `encode_lane_velocity_constraints()`
    /// or within `encode_kinematics()`. Implementations may leave this as a no-op.
    fn encode_velocity_constraints(&mut self);

    /// Encode acceleration constraints (min/max bounds)
    ///
    /// Note: For CartesianEncoder, this is a no-op because acceleration constraints
    /// are encoded within `encode_kinematics()`. For BicycleEncoder, this method
    /// enforces acceleration bounds on the `accelerations` variable.
    fn encode_acceleration_constraints(&mut self);

    // === Collision Detection (coordinate-specific) ===

    /// Generate time-to-collision constraint between two actors
    ///
    /// Returns a Bool constraint that is true when TTC >= min_ttc
    fn encode_ttc_constraint(&self, actor1: &str, actor2: &str, min_ttc: f64, time: usize) -> Bool;

    /// Generate distance constraint between two actors
    ///
    /// Returns a Bool constraint that is true when distance >= min_dist
    fn encode_distance_constraint(
        &self,
        actor1: &str,
        actor2: &str,
        min_dist: f64,
        time: usize,
    ) -> Bool;

    // === Extraction ===

    /// Extract actor trajectory from Z3 model
    ///
    /// Converts Z3 variable values into an ActorTrajectory object
    fn extract_actor_trajectory(
        &self,
        model: &Model,
        actor_id: &str,
        role: &str,
    ) -> Result<ActorTrajectory>;

    // === Accessors ===

    /// Get longitudinal position variable for an actor at a given time
    fn get_longitudinal_pos(&self, actor_id: &str, time: usize) -> &Real;

    /// Get lateral position variable for an actor at a given time
    fn get_lateral_pos(&self, actor_id: &str, time: usize) -> &Real;

    /// Get longitudinal velocity variable for an actor at a given time
    fn get_longitudinal_vel(&self, actor_id: &str, time: usize) -> &Real;

    /// Get lane variable for an actor at a given time
    fn get_lane_var(&self, actor_id: &str, time: usize) -> &Int;

    /// Get lateral velocity variable for an actor at a given time
    fn get_lateral_vel(&self, actor_id: &str, time: usize) -> &Real;

    // === Lane Constraints ===

    /// Encode lane-based velocity direction constraints
    ///
    /// Constrains velocity direction based on actor direction (forward/backward lanes)
    /// Also adds lane bounds and single-lane-jump constraints
    fn encode_lane_velocity_constraints(&mut self);

    /// Encode lateral velocity bounds for realistic lane changes
    ///
    /// Constrains lateral velocity to allow single-timestep lane changes
    fn encode_lateral_velocity_bounds(&mut self);

    // === Backend Access ===

    /// Get reference to the Z3 backend
    fn backend(&self) -> &B;

    /// Get mutable reference to the Z3 backend
    fn backend_mut(&mut self) -> &mut B;

    /// Get reference to the scenario specification
    fn spec(&self) -> &ScenarioSpec;
}
