//! Cartesian coordinate system encoder
//!
//! Implements the CoordinateEncoder trait for Cartesian (x, y) coordinates.
//! This encoder handles vehicle kinematics in 2D Cartesian space with
//! lane-based constraints.

use std::collections::HashMap;
use std::ops::Add;
use z3::ast::{Int, Real};
use z3::Model;

use crate::dsl::types::{ActorRole, ActorSpec, ScenarioSpec};
use crate::error::Result;
use crate::scenario::model::{
    Acceleration, ActorTrajectory, CartesianState, Position, State, Velocity,
};
use crate::solver::backend::Z3Backend;
use crate::solver::coordinate_encoder::CoordinateEncoder;
use crate::solver::encoder_utils::{
    collect_lane_change_data, extract_int, extract_real, real_from_f64,
};
use crate::solver::encoders::pedestrian::{
    encode_pedestrian_bounds_step, encode_pedestrian_initial_state,
    encode_pedestrian_kinematics_step, encode_pedestrian_lateral_containment,
    extract_pedestrian_trajectory,
};

/// Cartesian coordinate system encoder
///
/// Uses (x, y) position variables and manages lane-based constraints
/// for vehicle motion.
pub struct CartesianEncoder<B: Z3Backend> {
    /// Z3 backend (Solver or Optimizer)
    backend: B,

    /// Scenario specification
    spec: ScenarioSpec,

    /// Number of time steps
    horizon: usize,

    // Variable maps: actor_id -> Vec<variable> (one per time step)
    /// Longitudinal positions (m)
    positions_x: HashMap<String, Vec<Real>>,

    /// Lateral positions (m)
    positions_y: HashMap<String, Vec<Real>>,

    /// Longitudinal velocities (m/s)
    velocities_x: HashMap<String, Vec<Real>>,

    /// Lateral velocities (m/s)
    velocities_y: HashMap<String, Vec<Real>>,

    /// Lane numbers (integer)
    lanes: HashMap<String, Vec<Int>>,

    /// Longitudinal accelerations (m/s²)
    accelerations_x: HashMap<String, Vec<Real>>,

    /// Lateral accelerations (m/s²)
    accelerations_y: HashMap<String, Vec<Real>>,
}

impl<B: Z3Backend> CartesianEncoder<B> {
    /// Create a new Cartesian encoder
    pub fn new(spec: ScenarioSpec, backend: B) -> Self {
        let horizon = spec.num_time_steps();

        Self {
            backend,
            spec,
            horizon,
            positions_x: HashMap::new(),
            positions_y: HashMap::new(),
            velocities_x: HashMap::new(),
            velocities_y: HashMap::new(),
            lanes: HashMap::new(),
            accelerations_x: HashMap::new(),
            accelerations_y: HashMap::new(),
        }
    }

    /// Encode lane-position coupling at a specific time step
    ///
    /// Constrains py = lane * lane_width + lane_width/2
    fn encode_lane_position_coupling_at_time(&mut self, actor_id: &str, t: usize) {
        let lane_var = &self.lanes[actor_id][t];
        let py_var = &self.positions_y[actor_id][t];

        let lane_width = self.spec.get_lane_width();
        let lane_width_real = real_from_f64(lane_width);
        let half_width = real_from_f64(lane_width / 2.0);

        // py = lane * lane_width + lane_width/2
        let lane_real = lane_var.to_real();
        let expected_py = lane_real * &lane_width_real + &half_width;
        self.backend.assert(&py_var.eq(&expected_py));
    }

    /// Encode lane-position *bracketing* at a specific time step:
    /// `|py - (lane*lane_width + lane_width/2)| <= lane_width/2`.
    ///
    /// This is the mid-manoeuvre counterpart of
    /// [`Self::encode_lane_position_coupling_at_time`] (SW-10/H2). While a
    /// vehicle is between two lane centres `py` cannot equal a centre, so the
    /// equality coupling cannot be asserted — but `lane` must still name the
    /// lane the vehicle is *physically in*. Bracketing says exactly that:
    /// `lane` is the index of the lane whose 3.5 m-wide strip contains `py`.
    ///
    /// This replaces the old schedule (`lane == source` for every step of the
    /// window, `lane == target` only at its last step), under which `py` was
    /// free to reach the target centre seconds before `lane` acknowledged it,
    /// and every consumer keyed on `lane` — the TTC proposition's
    /// `same_lane_discrete`, `compute_effective_dist`,
    /// `compute_validation_metrics` — read the actors as separated during
    /// exactly the window a cut-in is about.
    ///
    /// `lane.to_real() * lane_width` is constant x variable, so this stays
    /// linear; it does move the window steps from pure QF_LRA into mixed
    /// integer-real linear arithmetic, which is still decidable and still fine
    /// for `Optimize`. Note the surrounding code was already mixed: the
    /// source/target lane centres a few lines below are built from the same
    /// `Int::to_real()` coercion.
    fn encode_lane_position_bracket_at_time(&mut self, actor_id: &str, t: usize) {
        let lane_var = &self.lanes[actor_id][t];
        let py_var = &self.positions_y[actor_id][t];

        let lane_width = self.spec.get_lane_width();
        let lane_width_real = real_from_f64(lane_width);
        let half_width = real_from_f64(lane_width / 2.0);

        let centre = lane_var.to_real() * &lane_width_real + &half_width;
        let offset = py_var - &centre;
        self.backend.assert(&offset.le(&half_width));
        self.backend.assert(&offset.ge(&-&half_width));
    }

    /// A step in which a vehicle is not changing lanes: pinned to the lane
    /// centre, and not moving laterally at all.
    ///
    /// The `vy = 0` half is new with SW-09 and is the counterpart of the
    /// bicycle encoder's stable-phase rule. Pinning `py` alone leaves a
    /// sawtooth: `py[t+1] = py[t] + (vy[t] + vy[t+1])*dt/2` with `py` fixed
    /// admits any `vy[t+1] = -vy[t]`, so with `|ay| <= 2` and `dt = 0.1` the
    /// solver would alternate `vy` between ±0.1 m/s while parked on the lane
    /// centre — kinematically consistent, physically nonsense, and enough to
    /// break the `vy/vx <= 0.15` heading-ratio tests once the vehicle has
    /// braked and `vx` is small. `vy = 0` on a pinned step forces `ay = 0`
    /// there through the velocity chain.
    fn encode_stable_lateral_state_at_time(&mut self, actor_id: &str, t: usize) {
        self.encode_lane_position_coupling_at_time(actor_id, t);
        let zero = Real::from_rational(0_i64, 1_i64);
        let vy_var = &self.velocities_y[actor_id][t];
        self.backend.assert(&vy_var.eq(&zero));
    }

    /// Encode initial state for a single actor (Cartesian-specific)
    fn encode_actor_initial_state(
        &mut self,
        actor_id: &str,
        lane: usize,
        pos_min: f64,
        pos_max: f64,
        speed_min: f64,
        speed_max: f64,
        accel_min: f64,
        accel_max: f64,
        _role: ActorRole,
        direction: i32,
    ) {
        // Lane at t=0
        let lane_var = &self.lanes[actor_id][0];
        let lane_val = Int::from_i64(lane as i64);
        self.backend.assert(&lane_var.eq(&lane_val));

        // Position at t=0
        let px_var = &self.positions_x[actor_id][0];
        if (pos_min - pos_max).abs() < 1e-6 {
            // Fixed value
            let pos_val = real_from_f64(pos_min);
            self.backend.assert(&px_var.eq(&pos_val));
        } else {
            // Range
            let min_val = real_from_f64(pos_min);
            let max_val = real_from_f64(pos_max);
            self.backend.assert(&px_var.ge(&min_val));
            self.backend.assert(&px_var.le(&max_val));
        }

        // Velocity at t=0
        // Use actor direction: speed is magnitude, vx sign depends on direction
        let vx_var = &self.velocities_x[actor_id][0];

        if (speed_min - speed_max).abs() < 1e-6 {
            // Fixed value
            let speed = if direction == 1 {
                speed_min
            } else {
                -speed_min
            };
            let speed_val = real_from_f64(speed);
            self.backend.assert(&vx_var.eq(&speed_val));
        } else {
            // Range
            if direction == 1 {
                // Forward: vx in [speed_min, speed_max]
                let min_val = real_from_f64(speed_min);
                let max_val = real_from_f64(speed_max);
                self.backend.assert(&vx_var.ge(&min_val));
                self.backend.assert(&vx_var.le(&max_val));
            } else {
                // Backward: vx in [-speed_max, -speed_min]
                let min_val = real_from_f64(-speed_max);
                let max_val = real_from_f64(-speed_min);
                self.backend.assert(&vx_var.ge(&min_val));
                self.backend.assert(&vx_var.le(&max_val));
            }
        }

        // Initial lateral velocity: zero (not changing lanes initially).
        let vy_var = &self.velocities_y[actor_id][0];
        let zero = Real::from_rational(0_i64, 1_i64);
        self.backend.assert(&vy_var.eq(&zero));

        // Initial acceleration at t=0
        let ax_var = &self.accelerations_x[actor_id][0];
        if (accel_min - accel_max).abs() < 1e-6 {
            // Fixed acceleration
            let accel_val = real_from_f64(accel_min);
            self.backend.assert(&ax_var.eq(&accel_val));
        } else {
            // Acceleration range
            let min_val = real_from_f64(accel_min);
            let max_val = real_from_f64(accel_max);
            self.backend.assert(&ax_var.ge(&min_val));
            self.backend.assert(&ax_var.le(&max_val));
        }

        // Initial lateral acceleration: zero (not changing lanes initially).
        let ay_var = &self.accelerations_y[actor_id][0];
        self.backend.assert(&ay_var.eq(&zero));

        // Encode initial lane-position coupling
        self.encode_lane_position_coupling_at_time(actor_id, 0);
    }

    /// Encode smooth lane transition over multiple time steps
    ///
    /// Constrains lateral position to gradually transition from source lane center
    /// to target lane center over the specified time window.
    ///
    /// Returns `true` if the window was encoded. `false` means the manoeuvre
    /// spans no simulated step and nothing at all was asserted over
    /// `[start_step, end_step]` — the caller must then cover those steps
    /// itself (SW-25; see `encode_lane_coupling_with_lane_changes`).
    fn encode_smooth_lane_transition(
        &mut self,
        actor_id: &str,
        start_step: usize,
        end_step: usize,
        direction: &crate::dsl::types::LaneChangeDirection,
    ) -> bool {
        // Defense-in-depth: skip encoding if lane change is beyond horizon.
        //
        // SW-25: this used to return `()`, so the caller could not tell an
        // encoded window from a discarded one — and its stable-phase loops
        // skip `[start_step, end_step]` either way. A discarded window
        // therefore left those steps with *no* lateral constraint: `py` was
        // held only by the kinematic chain and `lane` by nothing at all, so
        // the two were free to disagree, which is what `cut_in_right.yaml`
        // truncated to `duration: 5.0` did at its final step (py = 5.00,
        // inside lane 1, lane = 0 — and the `MinimizeTtc` optimiser picked
        // that lane precisely because a free `lane` let it claim to share the
        // ego's). `ScenarioSpec::validate` now rejects that spec outright, so
        // this branch should be unreachable from the public API; reporting it
        // rather than silently swallowing it is what keeps it defensive.
        if start_step >= self.horizon || start_step >= end_step {
            return false;
        }

        let lane_width = self.spec.get_lane_width();
        let lane_width_real = real_from_f64(lane_width);

        // Get source lane from step before transition starts
        let source_lane = &self.lanes[actor_id][start_step.saturating_sub(1)];

        // Calculate target lane, accounting for actor's direction of travel.
        // For a forward actor (direction=1): Right=lane+1, Left=lane-1 (road-frame).
        // For a backward actor (direction=-1): Right=lane-1, Left=lane+1 (road-frame),
        // because the actor's "right" is the opposite road-frame direction.
        let actor_direction = self
            .spec
            .actors
            .iter()
            .find(|a| a.id == actor_id)
            .map_or(1, |a| a.direction);
        let lane_delta = match direction {
            crate::dsl::types::LaneChangeDirection::Right => actor_direction as i64,
            crate::dsl::types::LaneChangeDirection::Left => -(actor_direction as i64),
        };
        let target_lane_int = source_lane.add(&Int::from_i64(lane_delta));

        // Get source and target lane centers
        let half_width = real_from_f64(lane_width / 2.0);
        let source_center = source_lane.to_real() * &lane_width_real + &half_width;
        let target_center = target_lane_int.to_real() * &lane_width_real + &half_width;

        // (The `num_steps == 0` guard that used to sit here was dead: the
        // `start_step >= end_step` return above already covers it, and it was
        // the second of two early returns that told the caller nothing.)

        // Soft constraints: constrain lateral position near source at start and target at end
        // Allow Z3 to discover smooth curve trajectory instead of forcing linear interpolation
        let tolerance = Real::from_rational(5_i64, 10_i64); // 0.5m tolerance

        // Constrain lateral position near source at start (soft)
        let py_start = &self.positions_y[actor_id][start_step];
        self.backend
            .assert(&py_start.ge(&(&source_center - &tolerance)));
        self.backend
            .assert(&py_start.le(&(&source_center + &tolerance)));

        // Constrain lateral position near target at end (soft)
        let py_end = &self.positions_y[actor_id][end_step.min(self.horizon)];
        self.backend
            .assert(&py_end.ge(&(&target_center - &tolerance)));
        self.backend
            .assert(&py_end.le(&(&target_center + &tolerance)));

        // Constrain lateral velocity using velocity ratio: |vy| <= k * |vx|
        // This ensures realistic heading angles and prevents sideways-only motion
        // k = 0.15 corresponds to ~8.5° max heading angle (arctan(0.15) ≈ 8.53°)
        let k = Real::from_rational(15_i64, 100_i64); // 0.15 ratio

        // Get actor direction to handle forward/backward lanes
        let actor = self
            .spec
            .actors
            .iter()
            .find(|a| a.id == actor_id)
            .expect("Actor must exist");

        // From start_step - 1, not start_step: py is pinned to the lane centre
        // for every t < start_step, and py[start_step] = py[start_step-1] +
        // vy[start_step-1]*dt, so vy[start_step-1] is the first lateral
        // velocity that is free to be non-zero. Bounding only from start_step
        // left that one step covered by nothing but the |vy| <= 2 m/s cap; Z3
        // used to pick 0 there and the ratio tests passed by luck, until the
        // corrected lane centres moved the solution and it picked -2.0
        // (vy/vx = 0.41 against a 0.15 limit) on cut_in_left.
        let ratio_start = start_step.saturating_sub(1);
        for t in ratio_start..=end_step.min(self.horizon) {
            let vx_t = &self.velocities_x[actor_id][t];
            let vy_t = &self.velocities_y[actor_id][t];

            // Compute |vx| * k for ratio bound
            // For forward lanes (direction = 1): |vx| = vx (positive)
            // For backward lanes (direction = -1): |vx| = -vx (make positive)
            let abs_vx = if actor.direction == 1 {
                vx_t.clone()
            } else {
                -vx_t // Negate to get positive magnitude
            };

            // Velocity ratio constraint: |vy| <= k * |vx|
            let max_vy = &abs_vx * &k;
            self.backend.assert(&vy_t.ge(&-&max_vy));
            self.backend.assert(&vy_t.le(&max_vy));
        }

        // Lane variable during the transition (SW-10/H2).
        //
        // The old encoding pinned `lane` on a schedule — `source` for every
        // step of the window, `target` only at the last one — while `py` was
        // constrained only at the two endpoints. Z3 was therefore free to sit
        // on the target lane centre for seconds while `lane` still read
        // `source`; `overtake_left` flipped `lane` at py = 2.12 and 4.88,
        // neither of which is a lane centre.
        //
        // Instead, derive `lane` from `py`: at every step of the window `lane`
        // must be the index of the lane physically containing `py`. The
        // timing is *not* lost with the schedule — it was never carried by it.
        // `py[start_step]` is pinned within 0.5 m of the source centre and
        // `py[end_step]` within 0.5 m of the target centre (above), and 0.5 m
        // is inside the 1.75 m half-width, so the bracket forces
        // `lane[start_step] == source` and `lane[end_step] == target`
        // exactly as the schedule's two endpoints did. What the schedule
        // additionally asserted — `lane == source` strictly *inside* the
        // window — is the bug, not the timing.
        for t in start_step..=end_step.min(self.horizon) {
            self.encode_lane_position_bracket_at_time(actor_id, t);
        }

        // `max_lateral_acceleration` used to be applied only here, over the
        // lane-change window. Now that `ay` is chained to `vy` for every actor
        // (SW-09/C2) it is a real envelope everywhere, so `encode_kinematics`
        // applies it at every step and this window-local copy is gone.

        true
    }
}

impl<B: Z3Backend> CoordinateEncoder<B> for CartesianEncoder<B> {
    fn create_variables(&mut self, horizon: usize, spec: &ScenarioSpec) {
        for actor in &spec.actors {
            let actor_id = &actor.id;

            let mut px_vars = Vec::new();
            let mut py_vars = Vec::new();
            let mut vx_vars = Vec::new();
            let mut vy_vars = Vec::new();
            let mut lane_vars = Vec::new();
            let mut ax_vars = Vec::new();
            let mut ay_vars = Vec::new();

            // Create variables for each time step
            for t in 0..=horizon {
                px_vars.push(Real::new_const(format!("{}_px_{}", actor_id, t)));
                py_vars.push(Real::new_const(format!("{}_py_{}", actor_id, t)));
                vx_vars.push(Real::new_const(format!("{}_vx_{}", actor_id, t)));
                vy_vars.push(Real::new_const(format!("{}_vy_{}", actor_id, t)));
                lane_vars.push(Int::new_const(format!("{}_lane_{}", actor_id, t)));
                ax_vars.push(Real::new_const(format!("{}_ax_{}", actor_id, t)));
                ay_vars.push(Real::new_const(format!("{}_ay_{}", actor_id, t)));
            }

            self.positions_x.insert(actor_id.clone(), px_vars);
            self.positions_y.insert(actor_id.clone(), py_vars);
            self.velocities_x.insert(actor_id.clone(), vx_vars);
            self.velocities_y.insert(actor_id.clone(), vy_vars);
            self.lanes.insert(actor_id.clone(), lane_vars);
            self.accelerations_x.insert(actor_id.clone(), ax_vars);
            self.accelerations_y.insert(actor_id.clone(), ay_vars);
        }
    }

    fn encode_kinematics(&mut self, dt: f64) {
        let dt_real = real_from_f64(dt);
        // Half of dt, for the trapezoidal position updates in the shared
        // point-mass step below.
        let half_dt = real_from_f64(dt / 2.0);
        let zero = Real::from_rational(0_i64, 1_i64);

        // `max_lateral_acceleration` is a hard envelope, not a safety metric:
        // it has no `ConstraintMode` of its own and `tests/common/invariants.rs`
        // treats it as unconditional. Before SW-09 it was applied only over
        // lane-change windows, which cost nothing because `ay` was a free
        // variable that no equation read. It now bounds the acceleration that
        // actually drives `vy`, at every step.
        let max_ay = real_from_f64(self.spec.max_lateral_acceleration);
        let neg_max_ay = real_from_f64(-self.spec.max_lateral_acceleration);

        // Road width for the pedestrian lateral-containment bound below
        // (SW-23): a per-spec constant, computed once for the whole horizon.
        let road_width = self.spec.get_lane_width() * self.spec.get_num_lanes() as f64;

        for actor in &self.spec.actors {
            let actor_id = &actor.id;
            let is_pedestrian = actor.role == ActorRole::Pedestrian;

            let ax_min_real = real_from_f64(actor.acceleration.min());
            let ax_max_real = real_from_f64(actor.acceleration.max());

            // Bounds run to the horizon inclusive. The trapezoidal position
            // update reads v[t+1], so v[horizon] and — through the velocity
            // chain — a[horizon] appear in the encoding; a bound that stopped
            // at horizon-1 would leave the last extracted acceleration free to
            // be anything at all, which is what the reported `ay` used to be
            // for every vehicle at every step.
            for t in 0..=self.horizon {
                let vx_t = &self.velocities_x[actor_id][t];
                let vy_t = &self.velocities_y[actor_id][t];
                let ax_t = &self.accelerations_x[actor_id][t];
                let ay_t = &self.accelerations_y[actor_id][t];

                if is_pedestrian {
                    // Acceleration clamped to the pedestrian limits on both
                    // axes, plus the linearised speed *octagon*
                    // |vx| <= v, |vy| <= v, |vx| + |vy| <= sqrt(2)*v.
                    //
                    // The octagon replaces the quadratic disk
                    // vx^2 + vy^2 <= v^2 so the encoding stays in QF_LRA
                    // (10-20x faster, and the optimiser works at all). It used
                    // to be a plain box, over-conservative by sqrt(2) on the
                    // diagonal, which the speed constants "compensated" for by
                    // shrinking — capping a pedestrian crossing perpendicular
                    // to the road, the dominant case in this corpus, at
                    // 1.41 m/s instead of 2.0 (SW-12/M8).
                    encode_pedestrian_bounds_step(&self.backend, vx_t, vy_t, ax_t, ay_t, actor);

                    // `py` bounded to the drivable surface plus the sidewalk
                    // margin, at every step — not only where `OnSidewalk`
                    // happens to pin one instant (SW-23). Without this, Z3 is
                    // free to place a pedestrian arbitrarily far past the
                    // sidewalk strip everywhere `OnSidewalk` isn't literally
                    // asserted; SW-16 measured up to 2.60 m of drift with the
                    // proposition-only bound in place.
                    encode_pedestrian_lateral_containment(
                        &self.backend,
                        &self.positions_y[actor_id][t],
                        road_width,
                    );
                } else {
                    self.backend.assert(&ax_t.ge(&ax_min_real));
                    self.backend.assert(&ax_t.le(&ax_max_real));
                    self.backend.assert(&ay_t.ge(&neg_max_ay));
                    self.backend.assert(&ay_t.le(&max_ay));
                }

                // Ego without lane changes never changes lanes (vy = 0, and
                // therefore ay = 0 through the chain below).
                if actor.role == ActorRole::Ego && actor.lane_changes.is_empty() {
                    self.backend.assert(&vy_t.eq(&zero));
                }

                if t == self.horizon {
                    continue;
                }

                // One integration step, both axes, for every actor.
                //
                // This is the C2 fix. `vy[t+1] = vy[t] + ay[t]*dt` used to sit
                // inside an `if role == Pedestrian` branch while the `py`
                // update below it applied to everyone, so for a vehicle
                // nothing connected `ay` to `vy`: `max_lateral_acceleration`
                // bounded a variable that constrained nothing, the `ay` in
                // scenario.json and the .xosc was fiction, and `vy` sign-flipped
                // step to step (cut_in_left's npc ran 1.28 -> -1.73 -> 1.12 with
                // ay = 0.000 reported throughout). The equations are the same
                // 2D point-mass equations for vehicles and pedestrians alike in
                // this coordinate system, so there is now one copy of them, in
                // `encoders::pedestrian`.
                //
                // With `vy` chained the lateral position update is trapezoidal
                // for everyone too — the null space that forced vehicles onto
                // forward Euler (py[t+1] = py[t] for any vy[t+1] = -vy[t], so a
                // parked vehicle could sawtooth vy between +2 and -2 at no
                // cost) is closed by the bounded `ay` behind `vy`.
                encode_pedestrian_kinematics_step(
                    &self.backend,
                    &self.positions_x[actor_id][t],
                    &self.positions_x[actor_id][t + 1],
                    &self.positions_y[actor_id][t],
                    &self.positions_y[actor_id][t + 1],
                    vx_t,
                    &self.velocities_x[actor_id][t + 1],
                    vy_t,
                    &self.velocities_y[actor_id][t + 1],
                    ax_t,
                    ay_t,
                    &dt_real,
                    &half_dt,
                );
            }
        }

        // Lane-position coupling with smooth lane change support
        // Use shared utility to collect lane change data
        let lane_changes_data = collect_lane_change_data(&self.spec, self.horizon);

        // Collect actor IDs and roles to avoid borrow checker issues
        let actor_data: Vec<_> = self
            .spec
            .actors
            .iter()
            .filter(|a| a.role != ActorRole::Pedestrian)
            .map(|a| (a.id.clone(), a.role))
            .collect();

        for (actor_id, _role) in actor_data {
            // Check if this actor has lane_changes
            if let Some(changes) = lane_changes_data.get(&actor_id) {
                if changes.is_empty() {
                    // No lane changes: enforce coupling at all time steps
                    for t in 0..=self.horizon {
                        self.encode_stable_lateral_state_at_time(&actor_id, t);
                    }
                } else {
                    // Multiple lane changes: encode phases
                    let first_start = changes[0].start_step;

                    // Before first lane change: enforce lane-position coupling
                    for t in 0..first_start.min(self.horizon) {
                        self.encode_stable_lateral_state_at_time(&actor_id, t);
                    }

                    // Process each lane change and intermediate phases
                    for (i, lc) in changes.iter().enumerate() {
                        // Encode smooth transition for this lane change
                        let encoded = self.encode_smooth_lane_transition(
                            &actor_id,
                            lc.start_step,
                            lc.end_step,
                            &lc.direction,
                        );

                        // A window the transition declined to encode is not a
                        // manoeuvre, so its steps belong to the surrounding
                        // stable phase rather than to nobody (SW-25). Without
                        // this the loops below skip them and they end up with
                        // no lateral constraint at all.
                        if !encoded {
                            for t in lc.start_step..=lc.end_step.min(self.horizon) {
                                self.encode_stable_lateral_state_at_time(&actor_id, t);
                            }
                        }

                        // After this lane change: enforce coupling until next change or end
                        let next_start = if i + 1 < changes.len() {
                            changes[i + 1].start_step
                        } else {
                            self.horizon + 1
                        };

                        for t in (lc.end_step + 1)..next_start.min(self.horizon + 1) {
                            self.encode_stable_lateral_state_at_time(&actor_id, t);
                        }
                    }
                }
            } else {
                // No lane changes: enforce coupling at all time steps
                for t in 0..=self.horizon {
                    self.encode_stable_lateral_state_at_time(&actor_id, t);
                }
            }
        }
    }

    fn encode_initial_conditions(&mut self) {
        // Pedestrians go through the shared point-mass helper: same px range,
        // same lane-centre py, the spec's speed range capped by the walking or
        // running limit, the acceleration range clamped to the pedestrian
        // limits, and vy[0]/ay[0] left free so they can already be crossing.
        // The vehicle block below would instead force vy[0] = ay[0] = 0 and
        // lock vx[0] to the sign of `direction`.
        //
        // Cloned so the `&mut self` calls in the loop do not collide with a
        // borrow of `self.spec`, the same reason the vehicle path collects its
        // fields into a tuple first.
        let pedestrians: Vec<ActorSpec> = self
            .spec
            .actors
            .iter()
            .filter(|a| a.role == ActorRole::Pedestrian)
            .cloned()
            .collect();
        let lane_width = self.spec.get_lane_width();
        for actor in &pedestrians {
            let actor_id = &actor.id;
            let lane_var = &self.lanes[actor_id][0];
            // `try_from` rather than `as`: a lane index is a `usize` and the
            // wrapping cast the rest of this file still uses is a live lint.
            let lane_val = Int::from_i64(i64::try_from(actor.lane).unwrap_or(i64::MAX));
            self.backend.assert(&lane_var.eq(&lane_val));
            encode_pedestrian_initial_state(
                &self.backend,
                &self.positions_x[actor_id],
                &self.positions_y[actor_id],
                &self.velocities_x[actor_id],
                &self.velocities_y[actor_id],
                &self.accelerations_x[actor_id],
                &self.accelerations_y[actor_id],
                actor,
                lane_width,
            );
            self.encode_lane_position_coupling_at_time(actor_id, 0);
        }

        // Collect all actor data upfront to avoid borrow checker issues
        let actor_data: Vec<_> = self
            .spec
            .actors
            .iter()
            .filter(|actor| actor.role != ActorRole::Pedestrian)
            .map(|actor| {
                (
                    actor.id.clone(),
                    actor.lane,
                    actor.position.min(),
                    actor.position.max(),
                    actor.speed.min(),
                    actor.speed.max(),
                    actor.acceleration.min(),
                    actor.acceleration.max(),
                    actor.role,
                    actor.direction,
                )
            })
            .collect();

        for (
            actor_id,
            lane,
            pos_min,
            pos_max,
            speed_min,
            speed_max,
            acc_min,
            acc_max,
            role,
            direction,
        ) in actor_data
        {
            self.encode_actor_initial_state(
                &actor_id, lane, pos_min, pos_max, speed_min, speed_max, acc_min, acc_max, role,
                direction,
            );
        }
    }

    fn encode_velocity_constraints(&mut self) {
        // Velocity direction constraints are handled by encode_lane_velocity_constraints().
        // This trait method is intentionally a no-op for CartesianEncoder.
    }

    fn encode_acceleration_constraints(&mut self) {
        // NOTE: This is a no-op for CartesianEncoder because acceleration constraints
        // are already encoded within encode_kinematics() for each actor at each timestep.
        // BicycleEncoder uses this method to enforce acceleration bounds separately.
    }

    fn extract_actor_trajectory(
        &self,
        model: &Model,
        actor_id: &str,
        role: &str,
    ) -> Result<ActorTrajectory> {
        // Pedestrians go through the shared extractor. The Cartesian variables
        // already are (px, py, vx, vy, ax, ay), so this reads the same values
        // the loop below would; routing it through one function keeps the
        // Bicycle encoder — where the mapping is not the identity — honest.
        if self
            .spec
            .get_actor(actor_id)
            .is_some_and(|a| a.role == ActorRole::Pedestrian)
        {
            return extract_pedestrian_trajectory(
                model,
                actor_id,
                &self.positions_x[actor_id],
                &self.positions_y[actor_id],
                &self.velocities_x[actor_id],
                &self.velocities_y[actor_id],
                &self.accelerations_x[actor_id],
                &self.accelerations_y[actor_id],
                &self.lanes[actor_id],
                self.horizon,
                self.spec.time_step,
            );
        }

        let mut trajectory = ActorTrajectory::new(actor_id.to_string(), role.to_string());

        for t in 0..=self.horizon {
            let time = t as f64 * self.spec.time_step;

            // Extract Cartesian values using shared utilities
            let px = extract_real(model, &self.positions_x[actor_id][t])?;
            let py = extract_real(model, &self.positions_y[actor_id][t])?;
            let vx = extract_real(model, &self.velocities_x[actor_id][t])?;
            let vy = extract_real(model, &self.velocities_y[actor_id][t])?;
            let ax = extract_real(model, &self.accelerations_x[actor_id][t])?;
            let ay = extract_real(model, &self.accelerations_y[actor_id][t])?;
            let lane = extract_int(model, &self.lanes[actor_id][t])?;

            let state = State {
                time,
                cartesian: CartesianState {
                    position: Position::new(px, py),
                    velocity: Velocity::new(vx, vy),
                    acceleration: Acceleration::new(ax, ay),
                    lane,
                },
            };

            trajectory.add_state(state);
        }

        Ok(trajectory)
    }

    fn get_longitudinal_pos(&self, actor_id: &str, time: usize) -> &Real {
        &self.positions_x[actor_id][time]
    }

    fn get_lateral_pos(&self, actor_id: &str, time: usize) -> &Real {
        &self.positions_y[actor_id][time]
    }

    fn get_longitudinal_vel(&self, actor_id: &str, time: usize) -> &Real {
        &self.velocities_x[actor_id][time]
    }

    fn get_lane_var(&self, actor_id: &str, time: usize) -> &Int {
        &self.lanes[actor_id][time]
    }

    fn get_lateral_vel(&self, actor_id: &str, time: usize) -> &Real {
        &self.velocities_y[actor_id][time]
    }

    fn encode_lane_velocity_constraints(&mut self) {
        let zero = Real::from_rational(0_i64, 1_i64);

        for actor in &self.spec.actors {
            let actor_id = &actor.id;

            // Skip pedestrians - they don't follow lane-based kinematics
            if actor.role == ActorRole::Pedestrian {
                continue;
            }

            // Use actor's direction (independent of lane direction)
            let direction = actor.direction;

            for t in 0..=self.horizon {
                let vx_t = &self.velocities_x[actor_id][t];

                if direction == 1 {
                    // Forward: vx >= 0
                    self.backend.assert(&vx_t.ge(&zero));
                } else {
                    // Backward: vx <= 0
                    self.backend.assert(&vx_t.le(&zero));
                }
            }
        }

        // Add lane bounds: 0 <= lane < num_lanes for all actors and time steps
        let num_lanes = self.spec.get_num_lanes();
        let max_lane = Int::from_i64((num_lanes - 1) as i64);
        let zero_lane = Int::from_i64(0);

        for actor in &self.spec.actors {
            let actor_id = &actor.id;
            for t in 0..=self.horizon {
                let lane_var = &self.lanes[actor_id][t];
                self.backend.assert(&lane_var.ge(&zero_lane));
                self.backend.assert(&lane_var.le(&max_lane));
            }
        }

        // Add single-lane-jump constraint: |lane[t+1] - lane[t]| <= 1
        // Prevents vehicles from jumping multiple lanes at once
        let one = Int::from_i64(1);
        let neg_one = Int::from_i64(-1);

        for actor in &self.spec.actors {
            if actor.role != ActorRole::Ego || !actor.lane_changes.is_empty() {
                // Apply lane-jump constraint to NPCs and ego with lane changes
                let actor_id = &actor.id;
                for t in 0..self.horizon {
                    let lane_t = &self.lanes[actor_id][t];
                    let lane_t1 = &self.lanes[actor_id][t + 1];
                    let diff = lane_t1 - lane_t;
                    // -1 <= diff <= 1
                    self.backend.assert(&diff.ge(&neg_one));
                    self.backend.assert(&diff.le(&one));
                }
            }
        }
    }

    fn encode_lateral_velocity_bounds(&mut self) {
        // Use realistic lateral velocity bound for vehicles
        // For 3.5m lane change over 3s with dt=0.1s: vy ≈ 1.17 m/s (realistic)
        // Setting max to 2.0 m/s allows for smooth lane changes
        let max_vy = 2.0; // m/s
        let max_vy_real = real_from_f64(max_vy);
        let neg_max_vy_real = real_from_f64(-max_vy);

        for actor in &self.spec.actors {
            if actor.role != ActorRole::Ego || !actor.lane_changes.is_empty() {
                // Apply lateral velocity bounds to NPCs and ego with lane changes
                let actor_id = &actor.id;
                for t in 0..=self.horizon {
                    let vy_t = &self.velocities_y[actor_id][t];
                    self.backend.assert(&vy_t.ge(&neg_max_vy_real));
                    self.backend.assert(&vy_t.le(&max_vy_real));
                }
            }
        }
    }

    fn backend(&self) -> &B {
        &self.backend
    }

    fn backend_mut(&mut self) -> &mut B {
        &mut self.backend
    }

    fn spec(&self) -> &ScenarioSpec {
        &self.spec
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dsl::types::{
        ActorRole, ActorSpec, LaneChangeConfig, LaneChangeDirection, RoadSpec, ScenarioType,
        ValueOrRange,
    };
    use crate::solver::backend::SolverBackend;
    use crate::solver::coordinate_encoder::CoordinateEncoder;
    use z3::{Config, SatResult};

    fn create_test_spec() -> ScenarioSpec {
        ScenarioSpec {
            scenario_type: ScenarioType::CutInLeft,
            time_step: 0.5,
            duration: 10.0,
            actors: vec![
                ActorSpec {
                    id: "ego".to_string(),
                    role: ActorRole::Ego,
                    lane: 1,
                    position: ValueOrRange::Value(50.0),
                    speed: ValueOrRange::Value(15.0),
                    acceleration: ValueOrRange::Range([-8.0, 3.0]),
                    direction: 1,
                    behavior: HashMap::new(),
                    lane_changes: vec![],
                    bicycle_params: None,
                },
                ActorSpec {
                    id: "npc".to_string(),
                    role: ActorRole::Npc,
                    lane: 0,
                    position: ValueOrRange::Range([60.0, 80.0]),
                    speed: ValueOrRange::Range([10.0, 20.0]),
                    acceleration: ValueOrRange::Range([-8.0, 3.0]),
                    direction: 1,
                    behavior: HashMap::new(),
                    lane_changes: vec![LaneChangeConfig {
                        direction: LaneChangeDirection::Right,
                        start_time: ValueOrRange::Range([2.5, 7.5]),
                        duration: ValueOrRange::Range([3.0, 4.0]),
                    }],
                    bicycle_params: None,
                },
            ],
            min_ttc: 3.0,
            min_distance: 5.0,
            road: Some(RoadSpec {
                num_lanes: 2,
                lane_width: 3.5,
                lane_directions: vec![1, 1],
                road_length: None,
            }),
            lane_width: 3.5,
            num_scenarios: 1,
            constraint_modes: crate::dsl::types::ConstraintModes::default(),
            optimization_target: crate::dsl::types::OptimizationTarget::None,
            max_acceleration: None,
            max_deceleration: None,
            max_velocity: None,
            min_velocity: None,
            min_lateral_distance: None,
            max_relative_velocity: None,
            max_lateral_acceleration: 2.0,
            coordinate_system: crate::dsl::types::CoordinateSystem::Cartesian,
            bicycle_config: None,
        }
    }

    /// Helper to parse a Z3 real AST to f64
    fn eval_real(model: &z3::Model, var: &Real) -> f64 {
        crate::solver::encoder_utils::extract_real(model, var).unwrap()
    }

    /// Helper to parse a Z3 int AST to i64
    fn eval_int(model: &z3::Model, var: &Int) -> i64 {
        crate::solver::encoder_utils::extract_int(model, var).unwrap() as i64
    }

    fn approx_eq(a: f64, b: f64, eps: f64) -> bool {
        (a - b).abs() < eps
    }

    #[test]
    fn test_new_creates_empty_encoder() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let spec = create_test_spec();
            let backend = SolverBackend::new();
            let encoder = CartesianEncoder::new(spec, backend);

            assert_eq!(encoder.horizon, 20);
            assert!(encoder.positions_x.is_empty());
            assert!(encoder.positions_y.is_empty());
            assert!(encoder.velocities_x.is_empty());
            assert!(encoder.velocities_y.is_empty());
            assert!(encoder.lanes.is_empty());
        });
    }

    #[test]
    fn test_create_variables() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let spec = create_test_spec();
            let backend = SolverBackend::new();
            let mut encoder = CartesianEncoder::new(spec.clone(), backend);
            encoder.create_variables(20, &spec);

            // Each actor should have horizon+1 = 21 variables per type
            assert_eq!(encoder.positions_x["ego"].len(), 21);
            assert_eq!(encoder.positions_y["npc"].len(), 21);
            assert_eq!(encoder.velocities_x["ego"].len(), 21);
            assert_eq!(encoder.velocities_y["npc"].len(), 21);
            assert_eq!(encoder.lanes["ego"].len(), 21);

            // Accessor methods should not panic
            let _ = encoder.get_longitudinal_pos("ego", 0);
            let _ = encoder.get_lateral_pos("npc", 10);
            let _ = encoder.get_longitudinal_vel("ego", 20);
            let _ = encoder.get_lateral_vel("npc", 5);
            let _ = encoder.get_lane_var("ego", 0);
        });
    }

    #[test]
    fn test_encode_initial_conditions() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let spec = create_test_spec();
            let backend = SolverBackend::new();
            let mut encoder = CartesianEncoder::new(spec.clone(), backend);
            encoder.create_variables(20, &spec);
            encoder.encode_initial_conditions();

            assert_eq!(encoder.backend.check(), SatResult::Sat);
            let model = encoder.backend.get_model().unwrap();

            // Ego at fixed position 50.0
            let ego_px = eval_real(&model, &encoder.positions_x["ego"][0]);
            assert!(approx_eq(ego_px, 50.0, 0.01));

            // Ego speed 15.0
            let ego_vx = eval_real(&model, &encoder.velocities_x["ego"][0]);
            assert!(approx_eq(ego_vx, 15.0, 0.01));

            // Ego lane 1
            let ego_lane = eval_int(&model, &encoder.lanes["ego"][0]);
            assert_eq!(ego_lane, 1);

            // NPC position in [60, 80]
            let npc_px = eval_real(&model, &encoder.positions_x["npc"][0]);
            assert!(npc_px >= 59.99 && npc_px <= 80.01);

            // NPC speed in [10, 20]
            let npc_vx = eval_real(&model, &encoder.velocities_x["npc"][0]);
            assert!(npc_vx >= 9.99 && npc_vx <= 20.01);

            // NPC lane 0
            let npc_lane = eval_int(&model, &encoder.lanes["npc"][0]);
            assert_eq!(npc_lane, 0);
        });
    }

    #[test]
    fn test_encode_kinematics() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let spec = create_test_spec();
            let backend = SolverBackend::new();
            let mut encoder = CartesianEncoder::new(spec.clone(), backend);
            encoder.create_variables(20, &spec);
            encoder.encode_initial_conditions();
            encoder.encode_kinematics(0.5);
            encoder.encode_lane_velocity_constraints();

            assert_eq!(encoder.backend.check(), SatResult::Sat);
            let model = encoder.backend.get_model().unwrap();

            let dt = 0.5;
            let px0 = eval_real(&model, &encoder.positions_x["ego"][0]);
            let px1 = eval_real(&model, &encoder.positions_x["ego"][1]);
            let vx0 = eval_real(&model, &encoder.velocities_x["ego"][0]);
            let vx1 = eval_real(&model, &encoder.velocities_x["ego"][1]);
            let ax0 = eval_real(&model, &encoder.accelerations_x["ego"][0]);

            // Velocity is exact: vx[1] = vx[0] + ax[0]*dt.
            assert!(
                approx_eq(vx1, vx0 + ax0 * dt, 1e-9),
                "vx1={vx1} expected={}",
                vx0 + ax0 * dt
            );

            // Position carries the second-order term:
            //   px[1] = px[0] + vx[0]*dt + 0.5*ax[0]*dt^2
            // This test previously asserted the forward-Euler form
            // `px0 + vx0*dt` at a 0.01 tolerance, which is what the encoder
            // used to assert (SW-08/H1). The two differ by 0.5*ax*dt^2 —
            // 0.375 m per step at ax = 3, dt = 0.5 — so the old assertion
            // fails against the corrected update and the 1e-9 tolerance here
            // is the exactness Z3's rationals actually give.
            let expected_px1 = px0 + vx0 * dt + 0.5 * ax0 * dt * dt;
            assert!(
                approx_eq(px1, expected_px1, 1e-9),
                "px1={px1} expected={expected_px1} (px0={px0}, vx0={vx0}, ax0={ax0})"
            );
            // Equivalently, the trapezoidal form the encoder asserts.
            assert!(
                approx_eq(px1, px0 + (vx0 + vx1) * dt / 2.0, 1e-9),
                "px1={px1} disagrees with the trapezoidal form"
            );

            // The ego is a vehicle, so its lateral update stays forward Euler
            // until SW-09 chains vy to ay — see encode_kinematics.
            let py0 = eval_real(&model, &encoder.positions_y["ego"][0]);
            let py1 = eval_real(&model, &encoder.positions_y["ego"][1]);
            let vy0 = eval_real(&model, &encoder.velocities_y["ego"][0]);

            let expected_py1 = py0 + vy0 * dt;
            assert!(
                approx_eq(py1, expected_py1, 1e-9),
                "py1={py1} expected={expected_py1}"
            );
        });
    }

    #[test]
    fn test_lane_velocity_constraints_forward() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let mut spec = create_test_spec();
            spec.actors = vec![ActorSpec {
                id: "fwd".to_string(),
                role: ActorRole::Ego,
                lane: 0,
                position: ValueOrRange::Value(0.0),
                speed: ValueOrRange::Value(10.0),
                acceleration: ValueOrRange::Range([-2.0, 2.0]),
                direction: 1,
                behavior: HashMap::new(),
                lane_changes: vec![],
                bicycle_params: None,
            }];
            spec.duration = 2.0;
            spec.time_step = 0.5;

            let backend = SolverBackend::new();
            let mut encoder = CartesianEncoder::new(spec.clone(), backend);
            let horizon = spec.num_time_steps();
            encoder.create_variables(horizon, &spec);
            encoder.encode_initial_conditions();
            encoder.encode_kinematics(0.5);
            encoder.encode_lane_velocity_constraints();

            assert_eq!(encoder.backend.check(), SatResult::Sat);
            let model = encoder.backend.get_model().unwrap();

            // All vx should be >= 0 for forward actor
            for t in 0..=horizon {
                let vx = eval_real(&model, &encoder.velocities_x["fwd"][t]);
                assert!(vx >= -0.01, "vx[{}]={} should be >= 0", t, vx);
            }

            // Lane should be bounded [0, num_lanes-1]
            for t in 0..=horizon {
                let lane = eval_int(&model, &encoder.lanes["fwd"][t]);
                assert!(lane >= 0 && lane <= 1, "lane[{}]={} out of bounds", t, lane);
            }
        });
    }

    #[test]
    fn test_lane_velocity_constraints_backward() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let mut spec = create_test_spec();
            spec.actors = vec![ActorSpec {
                id: "bwd".to_string(),
                role: ActorRole::Ego,
                lane: 0,
                position: ValueOrRange::Value(100.0),
                speed: ValueOrRange::Value(10.0),
                acceleration: ValueOrRange::Range([-2.0, 2.0]),
                direction: -1,
                behavior: HashMap::new(),
                lane_changes: vec![],
                bicycle_params: None,
            }];
            spec.duration = 2.0;
            spec.time_step = 0.5;

            let backend = SolverBackend::new();
            let mut encoder = CartesianEncoder::new(spec.clone(), backend);
            let horizon = spec.num_time_steps();
            encoder.create_variables(horizon, &spec);
            encoder.encode_initial_conditions();
            encoder.encode_kinematics(0.5);
            encoder.encode_lane_velocity_constraints();

            assert_eq!(encoder.backend.check(), SatResult::Sat);
            let model = encoder.backend.get_model().unwrap();

            // All vx should be <= 0 for backward actor
            for t in 0..=horizon {
                let vx = eval_real(&model, &encoder.velocities_x["bwd"][t]);
                assert!(vx <= 0.01, "vx[{}]={} should be <= 0", t, vx);
            }
        });
    }

    #[test]
    fn test_smooth_lane_transition() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let mut spec = create_test_spec();
            // Single actor with a lane change from lane 0 to lane 1 (Right for direction=1)
            spec.actors = vec![ActorSpec {
                id: "lc".to_string(),
                role: ActorRole::Npc,
                lane: 0,
                position: ValueOrRange::Value(50.0),
                speed: ValueOrRange::Value(15.0),
                acceleration: ValueOrRange::Range([-2.0, 2.0]),
                direction: 1,
                behavior: HashMap::new(),
                lane_changes: vec![LaneChangeConfig {
                    direction: LaneChangeDirection::Right,
                    start_time: ValueOrRange::Value(1.0),
                    duration: ValueOrRange::Value(3.0),
                }],
                bicycle_params: None,
            }];
            spec.duration = 5.0;
            spec.time_step = 0.5;

            let backend = SolverBackend::new();
            let mut encoder = CartesianEncoder::new(spec.clone(), backend);
            let horizon = spec.num_time_steps();
            encoder.create_variables(horizon, &spec);
            encoder.encode_initial_conditions();
            encoder.encode_kinematics(0.5);
            encoder.encode_lane_velocity_constraints();
            encoder.encode_lateral_velocity_bounds();

            assert_eq!(encoder.backend.check(), SatResult::Sat);
            let model = encoder.backend.get_model().unwrap();

            // Before transition (t=0): lane should be 0
            let lane_0 = eval_int(&model, &encoder.lanes["lc"][0]);
            assert_eq!(lane_0, 0, "Source lane should be 0");

            // After transition (start=step2, duration=6 steps, end=step8): lane should be 1
            // start_time=1.0 / dt=0.5 => start_step=2, duration=3.0/0.5=6 => end_step=8
            let lane_end = eval_int(&model, &encoder.lanes["lc"][8]);
            assert_eq!(lane_end, 1, "Target lane should be 1 after transition");

            // During transition, check vy/vx ratio <= 0.15
            for t in 2..=8 {
                let vx = eval_real(&model, &encoder.velocities_x["lc"][t]);
                let vy = eval_real(&model, &encoder.velocities_y["lc"][t]);
                if vx.abs() > 0.1 {
                    let ratio = vy.abs() / vx.abs();
                    assert!(
                        ratio <= 0.16,
                        "vy/vx ratio at t={} is {} (vy={}, vx={})",
                        t,
                        ratio,
                        vy,
                        vx
                    );
                }
            }
        });
    }

    #[test]
    fn test_extract_actor_trajectory() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let mut spec = create_test_spec();
            spec.actors = vec![ActorSpec {
                id: "ego".to_string(),
                role: ActorRole::Ego,
                lane: 0,
                position: ValueOrRange::Value(0.0),
                speed: ValueOrRange::Value(10.0),
                acceleration: ValueOrRange::Range([0.0, 0.0]),
                direction: 1,
                behavior: HashMap::new(),
                lane_changes: vec![],
                bicycle_params: None,
            }];
            spec.duration = 3.0;
            spec.time_step = 0.5;

            let backend = SolverBackend::new();
            let mut encoder = CartesianEncoder::new(spec.clone(), backend);
            let horizon = spec.num_time_steps();
            encoder.create_variables(horizon, &spec);
            encoder.encode_initial_conditions();
            encoder.encode_kinematics(0.5);
            encoder.encode_lane_velocity_constraints();

            assert_eq!(encoder.backend.check(), SatResult::Sat);
            let model = encoder.backend.get_model().unwrap();

            let trajectory = encoder
                .extract_actor_trajectory(&model, "ego", "ego")
                .unwrap();

            // Should have horizon+1 states
            assert_eq!(trajectory.states.len(), horizon + 1);

            // Positions should be monotonically increasing for forward actor
            for i in 1..trajectory.states.len() {
                let prev_px = trajectory.states[i - 1].cartesian.position.x;
                let curr_px = trajectory.states[i].cartesian.position.x;
                assert!(
                    curr_px >= prev_px - 0.01,
                    "Position not monotonic: px[{}]={} < px[{}]={}",
                    i,
                    curr_px,
                    i - 1,
                    prev_px
                );
            }

            // Lane values should be valid
            for state in &trajectory.states {
                assert!(state.cartesian.lane >= 0 && state.cartesian.lane <= 1);
            }
        });
    }
}
