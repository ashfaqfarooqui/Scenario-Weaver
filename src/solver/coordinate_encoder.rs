//! Trait definition for coordinate system encoders
//!
//! This module defines the `CoordinateEncoder` trait that abstracts over
//! different coordinate systems (Cartesian, Bicycle, etc.).

use z3::ast::{Int, Real};
use z3::Model;

use crate::dsl::types::ScenarioSpec;
use crate::error::Result;
use crate::scenario::ActorTrajectory;
use crate::solver::backend::Z3Backend;

/// Trait for coordinate-system-specific encoding logic
///
/// Each coordinate system (Cartesian, Bicycle) implements this trait
/// to provide its own variable creation, kinematics, and constraint encoding.
///
/// # Method usage notes (verified against a fresh grep for call sites)
///
/// `encode_velocity_constraints()` and `encode_acceleration_constraints()` are
/// both called unconditionally from every entry point that builds a
/// `GenericEncoder` — `src/lib.rs` (the main generation path),
/// `src/solver/multi_solve.rs`, and `src/solver/objectives.rs` (the optimizer
/// paths) — precisely *because* `GenericEncoder` is coordinate-system-generic
/// and none of those callers know or care which concrete encoder they hold.
/// That is what the trait is for, and it is why leaving one implementor a
/// no-op is not itself a defect: a doc comment that calls the same methods
/// "not currently called" would hide that `BicycleEncoder`'s implementation
/// is load-bearing.
///
/// - `encode_velocity_constraints()`: a no-op for `CartesianEncoder` — the
///   direction-sign half of velocity is asserted by
///   `encode_lane_velocity_constraints()` instead, and the `max_velocity`
///   ceiling (when the spec declares one) is enforced coordinate-system-
///   agnostically as a `VelocityLT` proposition
///   (`src/scenarios/mod.rs` lowers `spec.max_velocity`, `src/ltl/encode.rs`
///   encodes it against `get_longitudinal_vel`, which both encoders
///   implement). For `BicycleEncoder` this method asserts that same
///   `max_velocity` ceiling directly on `speed_v` — redundant with the
///   proposition when one fires, but this is the encoder's only unconditional
///   enforcement of it, since not every spec reaches a `VelocityLT` atom.
///
/// - `encode_acceleration_constraints()`: a no-op for `CartesianEncoder`,
///   which asserts the acceleration band inline in `encode_kinematics()`
///   instead. For `BicycleEncoder` this method is where that band is
///   asserted; `encode_kinematics()` does not do it there.
///
/// - `encode_lateral_velocity_bounds()`: real in both implementors.
///   `CartesianEncoder` applies a flat `|vy| <= 2.0` m/s cap.
///   `BicycleEncoder` additionally derives a tighter bound from the heading
///   coupling (`vy = v̄*θ`, `|θ| <= atan(0.15)`) before applying the same
///   2.0 m/s absolute cap — it is not a no-op: steering constraints alone
///   do not relate `θ`/`δ` to `vy`.
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
    /// Called unconditionally from every generation and optimizer entry
    /// point. See the trait-level doc comment for what each implementor
    /// actually does with it — it is not a no-op for either coordinate
    /// system, only redundant with other enforcement for one of them.
    fn encode_velocity_constraints(&mut self);

    /// Encode acceleration constraints (min/max bounds)
    ///
    /// Called unconditionally from every generation and optimizer entry
    /// point. A no-op for `CartesianEncoder` (asserted inline in
    /// `encode_kinematics()` instead); for `BicycleEncoder` this is where
    /// the acceleration band is asserted. See the trait-level doc comment.
    fn encode_acceleration_constraints(&mut self);

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
    /// Constrains lateral velocity to allow single-timestep lane changes.
    /// Real, and not a no-op, in both implementors — see the trait-level doc
    /// comment.
    fn encode_lateral_velocity_bounds(&mut self);

    // === Backend Access ===

    /// Get reference to the Z3 backend
    fn backend(&self) -> &B;

    /// Get mutable reference to the Z3 backend
    fn backend_mut(&mut self) -> &mut B;

    /// Get reference to the scenario specification
    fn spec(&self) -> &ScenarioSpec;
}
