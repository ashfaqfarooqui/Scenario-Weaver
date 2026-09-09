//! Z3 constraint encoder
//!
//! `GenericEncoder` is a delegating façade over the coordinate-specific encoders
//! (`CartesianEncoder`/`BicycleEncoder`, via the `CoordinateEncoder` trait): variable
//! creation, kinematics, and the coordinate-specific constraints all go through it. This
//! file also keeps a handful of items shared by more than one of the modules split
//! out of it — `encode_ttc_constraint` (the encoder-side TTC lowering; the near-duplicate
//! in the validator lives in `src/scenario/metrics.rs`), `directed_conflict`/
//! `encode_approaching`/[`DirectedConflict`] (used by both `src/ltl/encode.rs`'s
//! `Approaching` proposition and `src/solver/objectives.rs`'s optimizer
//! objectives), and the `METRIC_TOL`/`TTC_CLOSING_SPEED_EPSILON` tolerances the encoder,
//! `src/ltl/encode.rs` and `src/scenario/metrics.rs` all have to agree on.

use z3::ast::{Bool, Int, Real};
use z3::SatResult;

use crate::dsl::types::{CoordinateSystem, ScenarioSpec};
use crate::solver::backend::{SolverBackend, Z3Backend};
use crate::solver::coordinate_encoder::CoordinateEncoder;
use crate::solver::encoder_utils::{encode_same_lane_constraint, real_from_f64};
use crate::solver::encoders::bicycle::BicycleEncoder;
use crate::solver::encoders::cartesian::CartesianEncoder;
/// Width, in metres, of the sidewalk strip on either side of the drivable road surface.
///
/// `OnSidewalk` (below) bounds the pedestrian half-plane to this strip rather than leaving
/// it unbounded, and `src/scenario/xodr_exporter.rs` emits a matching `LaneType::Sidewalk`
/// of this width so the two agree. 2.0 m is a conventional urban sidewalk width and
/// comfortably covers the corpus's measured excursions (pedestrian_running/pedestrian_crossing
/// ~2.0 m; pedestrian_wide_road ~0.44 m).
pub const SIDEWALK_WIDTH: f64 = 2.0;

/// Trait providing read-only access to Z3 variables for scenario-specific constraints.
///
/// This abstraction allows scenario models to work with any backend (Solver or Optimizer)
/// without being tied to a concrete encoder type.
pub trait EncoderAccessor {
    /// Get lane variable for an actor at a given time step
    fn get_lane_var(&self, actor_id: &str, time: usize) -> &Int;
    /// Get longitudinal position variable
    fn get_longitudinal_pos(&self, actor_id: &str, time: usize) -> &Real;
    /// Get lateral position variable
    fn get_lateral_pos(&self, actor_id: &str, time: usize) -> &Real;
    /// Get longitudinal velocity variable
    fn get_longitudinal_vel(&self, actor_id: &str, time: usize) -> &Real;
    /// Get lateral velocity variable
    fn get_lateral_vel(&self, actor_id: &str, time: usize) -> &Real;
}

impl<B: Z3Backend + 'static> EncoderAccessor for GenericEncoder<B> {
    fn get_lane_var(&self, actor_id: &str, time: usize) -> &Int {
        self.coord_encoder.get_lane_var(actor_id, time)
    }
    fn get_longitudinal_pos(&self, actor_id: &str, time: usize) -> &Real {
        self.coord_encoder.get_longitudinal_pos(actor_id, time)
    }
    fn get_lateral_pos(&self, actor_id: &str, time: usize) -> &Real {
        self.coord_encoder.get_lateral_pos(actor_id, time)
    }
    fn get_longitudinal_vel(&self, actor_id: &str, time: usize) -> &Real {
        self.coord_encoder.get_longitudinal_vel(actor_id, time)
    }
    fn get_lateral_vel(&self, actor_id: &str, time: usize) -> &Real {
        self.coord_encoder.get_lateral_vel(actor_id, time)
    }
}

/// Z3 SMT encoder for scenario constraints (generic over backend)
///
/// A thin facade that dispatches to coordinate-specific encoders
/// (CartesianEncoder or BicycleEncoder) via the CoordinateEncoder trait.
///
/// Works with either `SolverBackend` (SAT checking) or `OptimizerBackend`
/// (optimization objectives). The type alias `Z3Encoder = GenericEncoder<SolverBackend>`
/// is provided for the common SAT-solving case.
///
/// Supports both Cartesian (x, y) and Bicycle (x, y, θ, v) coordinate systems.
///
/// Note: In Z3 0.19, the context is managed internally and is implicit
/// within the `with_z3_config()` callback scope.
pub struct GenericEncoder<B: Z3Backend> {
    /// Coordinate-specific encoder (Cartesian or Bicycle)
    pub(crate) coord_encoder: Box<dyn CoordinateEncoder<B>>,

    /// Original scenario specification
    pub(crate) spec: ScenarioSpec,

    /// Number of time steps in the scenario
    pub(crate) horizon: usize,
}

/// Type alias for backward compatibility - uses Solver backend
pub type Z3Encoder = GenericEncoder<SolverBackend>;

impl<B: Z3Backend + 'static> GenericEncoder<B> {
    /// Create a new encoder with a specific backend
    ///
    /// Dispatches to the appropriate coordinate-specific encoder based on the
    /// coordinate system specified in the scenario spec.
    ///
    /// Note: This must be called within a `z3::with_z3_config()` callback.
    pub fn with_backend(spec: ScenarioSpec, backend: B) -> Self {
        let horizon = spec.num_time_steps();

        // Dispatch to appropriate encoder based on coordinate system
        let coord_encoder: Box<dyn CoordinateEncoder<B>> = match spec.coordinate_system {
            CoordinateSystem::Cartesian => Box::new(CartesianEncoder::new(spec.clone(), backend)),
            CoordinateSystem::Bicycle => Box::new(BicycleEncoder::new(spec.clone(), backend)),
        };

        Self {
            coord_encoder,
            spec,
            horizon,
        }
    }

    /// Get the scenario specification
    pub fn spec(&self) -> &ScenarioSpec {
        &self.spec
    }

    /// Get the time horizon
    pub fn horizon(&self) -> usize {
        self.horizon
    }
}

impl Z3Encoder {
    /// Create a new Z3 encoder for the given specification (backward compatible)
    ///
    /// Note: This must be called within a `z3::with_z3_config()` callback.
    pub fn new(spec: ScenarioSpec) -> Self {
        Self::with_backend(spec, SolverBackend::new())
    }
}

impl<B: Z3Backend + 'static> GenericEncoder<B> {
    /// Encode scenario-specific Z3 constraints
    ///
    /// This calls the trait method to allow scenarios to add custom Z3 assertions
    /// beyond the standard LTL and safety encodings.
    pub fn encode_scenario_specific_constraints(
        &self,
        model: &dyn crate::scenarios::ScenarioModel,
    ) -> anyhow::Result<()> {
        model
            .add_z3_constraints(&self.spec, self, self.coord_encoder.backend(), self.horizon)
            .map_err(|e| anyhow::anyhow!(e))
    }

    /// Create all Z3 variables for the scenario
    ///
    /// Delegates to the coordinate-specific encoder.
    pub fn create_variables(&mut self) {
        self.coord_encoder
            .create_variables(self.horizon, &self.spec);
    }

    /// Encode initial conditions from the DSL specification
    pub fn encode_initial_conditions(&mut self) {
        self.coord_encoder.encode_initial_conditions();
    }

    /// Encode kinematic constraints with acceleration support
    pub fn encode_kinematics(&mut self) {
        self.coord_encoder.encode_kinematics(self.spec.time_step);
    }

    /// Encode speed upper bounds for all actors
    pub fn encode_velocity_constraints(&mut self) {
        self.coord_encoder.encode_velocity_constraints();
    }

    /// Encode lane-based velocity constraints, plus the forward-progress floor
    /// and the opt-in per-actor speed-retention band.
    pub fn encode_lane_velocity_constraints(&mut self) {
        self.coord_encoder.encode_lane_velocity_constraints();
        self.encode_forward_progress();
        self.encode_speed_retention();
    }

    /// Require every vehicle to actually traverse the scenario.
    ///
    /// The coordinate encoders assert only a sign condition on `vx` (`vx >= 0`
    /// forward, `vx <= 0` backward) and `min_velocity` defaults to
    /// `ConstraintMode::Ignore`, so without this, "everybody stops" would be a
    /// legal answer to every specification — and a very attractive one, because a
    /// pair of parked cars satisfies every distance and TTC threshold
    /// trivially. Without it, on `cut_in_left` both vehicles could come to a dead
    /// stop 98 m apart and the result would still be emitted with
    /// `all_constraints_satisfied: true`; on all three pedestrian examples the
    /// only vehicle would stand still for a majority of the horizon.
    ///
    /// The constraint is on net displacement over the whole horizon:
    ///
    /// ```text
    /// dir * (px[horizon] - px[0])  >=  f * v_declared_min * duration
    /// ```
    ///
    /// One inequality per vehicle, `f * v * T` a compile-time constant, so it
    /// is a single linear bound that propagates rather than a case split.
    ///
    /// Displacement rather than a per-step speed floor, deliberately: braking
    /// hard — to a standstill, briefly — is legitimate driving, and is the
    /// whole point of a scenario with a pedestrian in the road. What is not
    /// legitimate is standing still for most of the scenario. A per-step floor
    /// would forbid the first to prevent the second.
    ///
    /// Pedestrians are excluded: they cross the road, so their longitudinal
    /// displacement is near zero by design, and `PEDESTRIAN_*` already bounds
    /// their speed on both axes.
    ///
    /// A second, narrower floor applies alongside the displacement one: the
    /// longitudinal speed at the *final* step must be back up to
    /// `TERMINAL_SPEED_FRACTION` of the actor's declared initial speed. The
    /// displacement floor alone only bounds an average over the horizon, and
    /// the cheapest way to satisfy an average is a monotone decay to zero —
    /// which is exactly the solution Z3 was finding: an oncoming vehicle that
    /// coasts to a dead stop and stays there is not a near miss, however
    /// satisfiable it is. Bounding only the terminal step (not every step)
    /// leaves interior steps free to dip to zero and recover, so a genuine
    /// emergency stop for a pedestrian in the road remains legal — it is a dip,
    /// not an ending state. Both bounds are one linear inequality per actor
    /// against a compile-time constant, so this stays in QF_LRA.
    ///
    /// The terminal floor is narrowed to the actors that can meet it. "A dip,
    /// not an ending state" covers a stop in the middle of the horizon and says
    /// nothing about a stop *at* it, and a spec can demand exactly that: with
    /// `speed: 20.0` and `acceleration: [-5.0, -1.9]` the declared band forces
    /// the actor to shed at least 1.9 m/s² at every step, so it reaches the
    /// horizon at 1 m/s at the very fastest — a braking manoeuvre that ends at
    /// rest, which is the whole scenario. The displacement floor accepts it
    /// (braking at the declared -1.9 m/s² covers 105 m against the 100 m it
    /// requires, and the shipped trajectory rides that floor exactly); the
    /// terminal floor, which wants 10 m/s, made it UNSAT on its own. So the
    /// terminal bound is emitted only when
    /// `speed.max() + a_along_max * duration >= TERMINAL_SPEED_FRACTION *
    /// speed.min()`, i.e. only when the actor's declared dynamics can reach it
    /// — see the `a_along_max` comment below. Nothing is relaxed for an actor
    /// that *could* hold its speed and simply chose not to; the terminal floor
    /// still applies to that case.
    fn encode_forward_progress(&mut self) {
        use crate::dsl::types::{
            ActorRole, CoordinateSystem, MIN_FORWARD_PROGRESS_FRACTION, TERMINAL_SPEED_FRACTION,
        };

        let duration = self.spec.duration;
        let horizon = self.horizon;
        let coordinate_system = self.spec.coordinate_system;

        let bounds: Vec<(String, f64, Option<f64>)> = self
            .spec
            .actors
            .iter()
            .filter(|a| a.role != ActorRole::Pedestrian)
            .filter_map(|a| {
                let dir = f64::from(a.direction);
                let displacement_required =
                    MIN_FORWARD_PROGRESS_FRACTION * a.speed.min() * duration * dir;

                // The terminal-speed floor is asserted only where the
                // actor's own declared dynamics can actually meet it. The
                // velocity chain is `v[t+1] = v[t] + a[t]*dt` with
                // `a[t]` in the declared band at every step, so the fastest
                // the actor can be travelling *along its direction of
                // travel* at the horizon is
                // `speed.max() + a_along_max * duration` — the same envelope
                // `BicycleEncoder::reachable_speed_span` already computes for
                // its speed buckets. When a spec pins the band strictly
                // negative (`acceleration: [-5.0, -1.9]` — a declared braking
                // manoeuvre), that ceiling is below the floor and the floor
                // is unsatisfiable *by construction*: it makes the spec UNSAT
                // on its own, whatever else the scenario says. That is not a
                // near-miss being rejected, it is a bound contradicting the
                // dynamics the user declared, so it is not asserted at all.
                //
                // `a_along_max` is frame-dependent: Cartesian bounds the
                // world-frame `ax`, so along-track acceleration is `dir * ax`
                // and a backward actor's maximum is `-acceleration.min()`;
                // the bicycle model bounds `a` along the heading already
                // (`speed_v` is an unsigned magnitude, `longitudinal_vel` is
                // `direction * speed_v`), so its maximum is
                // `acceleration.max()` for either direction.
                let a_along_max =
                    if coordinate_system == CoordinateSystem::Bicycle || a.direction >= 0 {
                        a.acceleration.max()
                    } else {
                        -a.acceleration.min()
                    };
                let terminal_speed_required = TERMINAL_SPEED_FRACTION * a.speed.min();
                let terminal_speed_reachable = a.speed.max() + a_along_max * duration;
                let terminal_bound = (terminal_speed_reachable >= terminal_speed_required)
                    .then_some(terminal_speed_required * dir);

                (a.speed.min() > 0.0).then(|| (a.id.clone(), displacement_required, terminal_bound))
            })
            .collect();

        for (actor_id, displacement_required, terminal_speed_required) in bounds {
            let start = self.get_longitudinal_pos(&actor_id, 0).clone();
            let end = self.get_longitudinal_pos(&actor_id, horizon).clone();
            let travelled = &end - &start;
            let displacement_bound = real_from_f64(displacement_required);
            let displacement_constraint = if displacement_required >= 0.0 {
                travelled.ge(&displacement_bound)
            } else {
                travelled.le(&displacement_bound)
            };
            self.coord_encoder
                .backend_mut()
                .assert(&displacement_constraint);

            let Some(terminal_speed_required) = terminal_speed_required else {
                continue;
            };
            let final_vel = self.get_longitudinal_vel(&actor_id, horizon).clone();
            let terminal_bound = real_from_f64(terminal_speed_required);
            let terminal_constraint = if terminal_speed_required >= 0.0 {
                final_vel.ge(&terminal_bound)
            } else {
                final_vel.le(&terminal_bound)
            };
            self.coord_encoder
                .backend_mut()
                .assert(&terminal_constraint);
        }
    }

    /// Hold an actor inside its declared `speed:` band for the whole horizon,
    /// where — and only where — the actor asked for it.
    ///
    /// `speed:` is an initial condition everywhere else: it pins `v[0]` and is
    /// never mentioned again. `encode_forward_progress` above bounds the
    /// *average* over the horizon and the *final* step, and nothing bounds the
    /// steps in between, so the cheapest satisfying assignment rides whichever
    /// of those bounds is closest. Measured on the shipped corpus at `2c7c8fc`,
    /// every non-pedestrian actor in 16 of the 19 vehicle examples left its
    /// declared band at some step: `head_on_near_miss`'s oncoming vehicle decayed
    /// to 4.1 m/s against `[10.0, 12.0]` (riding `TERMINAL_SPEED_FRACTION`), and
    /// its "slow vehicle motivating overtake" reached 13.5 m/s against
    /// `[6.0, 8.0]` — the overtake was still enforced, but the reason for it had
    /// evaporated.
    ///
    /// With `behavior.speed_retention: f` (`0 < f <= 1`) on an actor, every step
    /// gets two constant-vs-variable inequalities:
    ///
    /// ```text
    /// direction * v_long[t]  >=  f * speed.min()
    /// direction * v_long[t]  <=  speed.max() / f
    /// ```
    ///
    /// Both bounds are compile-time constants and `direction` is `±1`, so this
    /// is a pair of linear bounds per actor per step — QF_LRA, exactly like the
    /// two floors above; no products of variables are introduced.
    ///
    /// The ceiling is not decoration. The floor half is bounded below by a
    /// bound that at least exists (the terminal floor); the ceiling half is
    /// unbounded above except by the acceleration band, which is why the
    /// slow vehicle could exceed its declared maximum by 69% and outrun the
    /// ego's own declared speed.
    ///
    /// **Opt-in per actor, defaulted off.** A per-step floor asserted on every
    /// actor is precisely what `encode_forward_progress`'s doc comment refuses:
    /// braking to a standstill for a pedestrian in the road is legitimate
    /// driving, and SW-40 already had to narrow the terminal floor for a
    /// declared braking manoeuvre. Per actor, the author states which vehicles
    /// are background traffic that holds a speed and which is the vehicle under
    /// test that may do anything. No corpus example changes unless it opts in.
    ///
    /// `get_longitudinal_vel` is the *signed* along-track velocity in both
    /// coordinate systems (Cartesian `vx`; the bicycle model's
    /// `longitudinal_vel = direction * speed_v`), so multiplying the bound by
    /// `direction` is frame-independent — unlike `a_along_max` above, which
    /// has to know which frame bounds the acceleration.
    ///
    /// Pedestrians are excluded, and `ScenarioSpec::validate` rejects the key on
    /// one: since SW-45 a pedestrian's authored speed governs `vy` and its `vx`
    /// is pinned to zero, so an along-track band would be unsatisfiable.
    fn encode_speed_retention(&mut self) {
        use crate::dsl::types::ActorRole;

        let horizon = self.horizon;
        let bounds: Vec<(String, f64, f64)> = self
            .spec
            .actors
            .iter()
            .filter(|a| a.role != ActorRole::Pedestrian)
            .filter_map(|a| {
                let fraction = a.speed_retention()?;
                let dir = f64::from(a.direction);
                Some((
                    a.id.clone(),
                    fraction * a.speed.min() * dir,
                    (a.speed.max() / fraction) * dir,
                ))
            })
            .collect();

        for (actor_id, floor, ceiling) in bounds {
            // Signed bounds: for a backward actor the along-track *floor* is an
            // upper bound on a negative `vx`, and the ceiling a lower one, so
            // the pair swaps rather than the comparison flipping.
            let (lower, upper) = if floor <= ceiling {
                (real_from_f64(floor), real_from_f64(ceiling))
            } else {
                (real_from_f64(ceiling), real_from_f64(floor))
            };
            for t in 0..=horizon {
                let vel = self.get_longitudinal_vel(&actor_id, t).clone();
                let lower_constraint = vel.ge(&lower);
                let upper_constraint = vel.le(&upper);
                self.coord_encoder.backend_mut().assert(&lower_constraint);
                self.coord_encoder.backend_mut().assert(&upper_constraint);
            }
        }
    }

    /// Encode lateral velocity bounds for realistic lane changes
    pub fn encode_lateral_velocity_bounds(&mut self) {
        self.coord_encoder.encode_lateral_velocity_bounds();
    }

    /// Check if the constraints are satisfiable (for testing)
    pub fn check(&self) -> SatResult {
        self.coord_encoder.backend().check()
    }

    /// Get the Z3 model (for testing)
    pub fn get_model(&self) -> Option<z3::Model> {
        self.coord_encoder.backend().get_model()
    }

    // === Variable Accessor Methods ===
    // These provide access to Z3 variables for scenario-specific constraints

    /// Get lane variable for an actor at a given time
    pub fn get_lane_var(&self, actor_id: &str, time: usize) -> &Int {
        self.coord_encoder.get_lane_var(actor_id, time)
    }

    /// Get longitudinal position variable for an actor at a given time
    pub fn get_longitudinal_pos(&self, actor_id: &str, time: usize) -> &Real {
        self.coord_encoder.get_longitudinal_pos(actor_id, time)
    }

    /// Get lateral position variable for an actor at a given time
    pub fn get_lateral_pos(&self, actor_id: &str, time: usize) -> &Real {
        self.coord_encoder.get_lateral_pos(actor_id, time)
    }

    /// Get longitudinal velocity variable for an actor at a given time
    pub fn get_longitudinal_vel(&self, actor_id: &str, time: usize) -> &Real {
        self.coord_encoder.get_longitudinal_vel(actor_id, time)
    }

    /// Get lateral velocity variable for an actor at a given time
    pub fn get_lateral_vel(&self, actor_id: &str, time: usize) -> &Real {
        self.coord_encoder.get_lateral_vel(actor_id, time)
    }

    // === Coordinate-specific accessors (for multi_solve.rs blocking clauses) ===

    /// Get Cartesian x position (maps to longitudinal position)
    pub fn get_position_x(&self, actor_id: &str, time: usize) -> &Real {
        self.get_longitudinal_pos(actor_id, time)
    }

    /// Get Cartesian x velocity (maps to longitudinal velocity)
    pub fn get_velocity_x(&self, actor_id: &str, time: usize) -> &Real {
        self.get_longitudinal_vel(actor_id, time)
    }

    /// Get Cartesian y position (maps to lateral position)
    pub fn get_position_y(&self, actor_id: &str, time: usize) -> &Real {
        self.get_lateral_pos(actor_id, time)
    }

    /// Get Cartesian y velocity (maps to lateral velocity)
    pub fn get_velocity_y(&self, actor_id: &str, time: usize) -> &Real {
        self.get_lateral_vel(actor_id, time)
    }

    /// Assert a constraint directly to the backend
    pub fn assert_constraint(&mut self, constraint: &Bool) {
        self.coord_encoder.backend_mut().assert(constraint);
    }

    // === LTL Encoding ===

    /// Encode LTL formula into Z3 constraints using bounded model checking
    ///
    /// This is the core of Phase 7. We expand temporal operators over the
    /// finite time horizon, converting them into Boolean combinations of
    /// propositions at different time steps.
    pub fn encode_ltl(&mut self, formula: &crate::ltl::formula::LTLFormula) {
        // Encode the formula starting at time 0, with full horizon
        let constraint =
            crate::ltl::encode::encode_ltl_bounded(self, &self.spec, formula, 0, self.horizon);
        self.coord_encoder.backend_mut().assert(&constraint);
    }
}

pub(crate) fn encode_ttc_constraint(
    accessor: &dyn EncoderAccessor,
    spec: &ScenarioSpec,
    actor1: &str,
    actor2: &str,
    min_ttc: f64,
    time: usize,
) -> z3::ast::Bool {
    let lane1 = accessor.get_lane_var(actor1, time);
    let lane2 = accessor.get_lane_var(actor2, time);

    let px1 = accessor.get_longitudinal_pos(actor1, time);
    let px2 = accessor.get_longitudinal_pos(actor2, time);

    let vx1 = accessor.get_longitudinal_vel(actor1, time);
    let vx2 = accessor.get_longitudinal_vel(actor2, time);

    let min_ttc_val = real_from_f64(min_ttc);
    let epsilon = Real::from_rational(1_i64, 100_i64); // 0.01 m/s to avoid division by zero

    // "Same lane" condition for TTC — one predicate, shared with the
    // `DistanceGT` lowering and with `compute_validation_metrics`:
    // `lane1 == lane2 OR |py1 - py2| < lane_width`.
    //
    // The predicate does not special-case direction: it does not use
    // y-proximity for opposite-direction pairs and a bare discrete lane
    // match for same-direction pairs. Such a split would only be safe if
    // `lane` were trustworthy for both, but `lane` is derived from `py`, so
    // the disjunct means something precise for every pair: either the two
    // occupy the same lane strip, or one of them is mid-manoeuvre and they
    // are laterally within a lane width of each other. Both are conflicts.
    //
    // A discrete-lane-only same-direction case would leave the validator
    // stricter than the encoder: with the widened validator predicate,
    // `overtake_left` reports `TTC 0.28 s < 2.00 s` at t=7.5 on a spec that
    // declares min_ttc *enforce* — a violation the encoder must assert
    // against even while the merge happens with the discrete lanes still
    // differing.
    let same_lane = encode_same_lane_constraint(
        lane1,
        lane2,
        accessor.get_lateral_pos(actor1, time),
        accessor.get_lateral_pos(actor2, time),
        spec.get_lane_width(),
    );

    // Determine who is ahead and who is behind
    // If px1 > px2, then actor1 is ahead (lead), actor2 is behind (follow)
    // If px2 > px1, then actor2 is ahead (lead), actor1 is behind (follow)

    // Case 1: actor1 ahead, actor2 behind, actor2 faster
    // TTC = (px1 - px2) / (vx2 - vx1)
    let actor1_ahead = px1.gt(px2);
    let actor2_faster = vx2.gt(vx1);
    let rel_vel_1 = vx2 - vx1;
    let distance_1 = px1 - px2;
    let collision_possible_1 =
        z3::ast::Bool::and(&[&actor1_ahead, &actor2_faster, &rel_vel_1.gt(&epsilon)]);
    // TTC >= min_ttc means: distance / rel_vel >= min_ttc, i.e.
    // distance >= min_ttc * rel_vel (rel_vel > 0 on this branch).
    //
    // Non-strict for the same reason as `DistanceGT` above:
    // `compute_validation_metrics` calls `ttc < min_ttc` a violation, so
    // safe is `ttc >= min_ttc`, and `Violate` mode — the negation of what
    // the encoder asserts — then means `ttc < min_ttc` strictly rather
    // than the boundary-satisfying `ttc <= min_ttc`. A strict form would
    // wrongly reject `cut_in_left_adversarial_all`, which reports
    // `min_ttc = 3.0` against a threshold of exactly 3.0.
    let ttc_safe_1 = distance_1.ge(&(&min_ttc_val * &rel_vel_1));

    // Case 2: actor2 ahead, actor1 behind, actor1 faster
    // TTC = (px2 - px1) / (vx1 - vx2)
    let actor2_ahead = px2.gt(px1);
    let actor1_faster = vx1.gt(vx2);
    let rel_vel_2 = vx1 - vx2;
    let distance_2 = px2 - px1;
    let collision_possible_2 =
        z3::ast::Bool::and(&[&actor2_ahead, &actor1_faster, &rel_vel_2.gt(&epsilon)]);
    let ttc_safe_2 = distance_2.ge(&(&min_ttc_val * &rel_vel_2));

    // Overall constraint:
    // If same_lane AND collision_possible_1, then ttc_safe_1
    // If same_lane AND collision_possible_2, then ttc_safe_2
    // Otherwise, true (no collision risk)

    let case1 = z3::ast::Bool::and(&[&same_lane, &collision_possible_1]).implies(&ttc_safe_1);
    let case2 = z3::ast::Bool::and(&[&same_lane, &collision_possible_2]).implies(&ttc_safe_2);

    z3::ast::Bool::and(&[&case1, &case2])
}
/// The lane-free half of a directed conflict: `follower` is strictly behind `leader`
/// and gaining on it.
///
/// Two inequalities between existing variables and nothing else — in particular **no
/// `same_lane` disjunction**, which is what makes it cheap enough to assert 101 times.
/// The lane test belongs in the *antecedent* of whatever implication uses this: a
/// disjunction in a consequent Z3 must satisfy is a choice it searches over, one per
/// step, while the same disjunction as a hypothesis is propagation.
/// Measured on `cut_in_left`'s five scenarios against a 15.0 s pre-fix baseline: with
/// `same_lane` in the consequent, 103 s; with the lane match hoisted into the
/// antecedent, 14.1 s.
///
/// Both comparisons are strict, matching `compute_validation_metrics`'s
/// `state1.x > state2.x` and `rel_vel > epsilon` exactly. A non-strict `>=` on the
/// closing floor would let Z3 answer *on* it, which the validator then declines to
/// measure — the same enforce-one-thing/report-another boundary mismatch that
/// `DistanceGT` and `METRIC_TOL` document elsewhere.
pub(crate) fn encode_approaching(
    accessor: &dyn EncoderAccessor,
    follower: &str,
    leader: &str,
    time: usize,
) -> Bool {
    let px_follow = accessor.get_longitudinal_pos(follower, time);
    let px_lead = accessor.get_longitudinal_pos(leader, time);
    let closing =
        accessor.get_longitudinal_vel(follower, time) - accessor.get_longitudinal_vel(leader, time);

    Bool::and(&[
        &px_lead.gt(px_follow),
        &closing.gt(real_from_f64(TTC_CLOSING_SPEED_EPSILON)),
    ])
}
/// The single directed conflict `follower` → `leader` at step `time`.
///
/// This is the one place the "a TTC exists here" predicate is built, and it is the
/// encoder-side twin of the TTC block in `compute_validation_metrics`: the same
/// "same lane" test ([`encode_same_lane_constraint`] — discrete lane match *or*
/// lateral overlap), the same requirement that the follower be strictly behind, and
/// the same closing-speed floor. Both [`Self::collect_directed_conflicts`] (the
/// optimizer objectives) and the `Converging` proposition go through
/// it, so an objective, an assertion and a reported metric cannot drift apart.
///
/// Every term is a difference of existing variables — no products, so QF_LRA.
pub(crate) fn directed_conflict(
    accessor: &dyn EncoderAccessor,
    spec: &ScenarioSpec,
    follower: &str,
    leader: &str,
    time: usize,
) -> DirectedConflict {
    let same_lane = encode_same_lane_constraint(
        accessor.get_lane_var(follower, time),
        accessor.get_lane_var(leader, time),
        accessor.get_lateral_pos(follower, time),
        accessor.get_lateral_pos(leader, time),
        spec.get_lane_width(),
    );

    let px_follow = accessor.get_longitudinal_pos(follower, time);
    let px_lead = accessor.get_longitudinal_pos(leader, time);
    let gap = px_lead - px_follow;
    let closing =
        accessor.get_longitudinal_vel(follower, time) - accessor.get_longitudinal_vel(leader, time);

    let approaching = encode_approaching(accessor, follower, leader, time);
    let guard = Bool::and(&[&same_lane, &approaching]);

    DirectedConflict {
        guard,
        gap,
        closing,
    }
}
/// Closing-speed floor, in m/s, below which a pair is not treated as approaching.
///
/// Mirrors the `epsilon = 0.01` in `compute_validation_metrics`
/// (`src/scenario/metrics.rs`): the objective must define TTC over exactly the states
/// the validator then measures it on, or the tool optimises one quantity and reports
/// another.
pub(crate) const TTC_CLOSING_SPEED_EPSILON: f64 = 0.01;

/// One directed conflict: actor `follow` is behind actor `lead`, in the same lane, and
/// gaining on it.
///
/// `gap` and `closing` are only meaningful under `guard`; outside it the pair contributes
/// no TTC at all, exactly as in `compute_validation_metrics`.
pub(crate) struct DirectedConflict {
    /// `same_lane ∧ px_lead > px_follow ∧ (vx_follow - vx_lead) ≥ ε`
    pub(crate) guard: Bool,
    /// `px_lead - px_follow` — positive under `guard`
    pub(crate) gap: Real,
    /// `vx_follow - vx_lead` — at least `ε` under `guard`
    pub(crate) closing: Real,
}
/// A breach is only a breach beyond the representation error.
///
/// The encoder asserts `distance >= min_distance` and
/// `gap >= min_ttc * closing_speed` as exact rationals, and it
/// asserts them non-strictly, so Z3 answers *on* the boundary:
/// `min_distance` comes back as exactly 5, `gap` as exactly
/// `3 * closing_speed`. Rendering those rationals as `f64` and then
/// *dividing* to recover a TTC does not round back to the same number — an
/// exactly-satisfying solution reports `2.999999999999999 < 3.00`, and the
/// validator called that a violation of a constraint the solver had
/// proved.
///
/// 1e-6 is the tolerance `tests/common/invariants.rs` uses for the same
/// reason and for the same quantities. It is the rational-to-double
/// rounding error, not an allowance for modelling error: the discrepancies
/// it excuses are ~1e-15.
pub(crate) const METRIC_TOL: f64 = 1e-6;
impl<B: Z3Backend + 'static> GenericEncoder<B> {
    /// Encode global max acceleration constraints (if specified)
    pub fn encode_acceleration_constraints(&mut self) {
        self.coord_encoder.encode_acceleration_constraints();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dsl::types::{
        ActorRole, ActorSpec, LaneChangeConfig, LaneChangeDirection, RoadSpec, ScenarioType,
        ValueOrRange,
    };
    use std::collections::HashMap;
    use z3::Config;

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
                    speed: ValueOrRange::Range([12.0, 14.0]),
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

    fn create_two_actor_same_lane_spec() -> ScenarioSpec {
        ScenarioSpec {
            scenario_type: ScenarioType::CutInLeft,
            time_step: 0.5,
            duration: 5.0,
            actors: vec![
                ActorSpec {
                    id: "ego".to_string(),
                    role: ActorRole::Ego,
                    lane: 1,
                    position: ValueOrRange::Value(50.0),
                    speed: ValueOrRange::Value(20.0),
                    acceleration: ValueOrRange::Range([-8.0, 3.0]),
                    direction: 1,
                    behavior: HashMap::new(),
                    lane_changes: vec![],
                    bicycle_params: None,
                },
                ActorSpec {
                    id: "npc".to_string(),
                    role: ActorRole::Npc,
                    lane: 1,
                    position: ValueOrRange::Value(100.0),
                    speed: ValueOrRange::Value(15.0),
                    acceleration: ValueOrRange::Range([-8.0, 3.0]),
                    direction: 1,
                    behavior: HashMap::new(),
                    lane_changes: vec![],
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

    /// Helper: create a two-actor spec with actors in different lanes
    fn create_two_actor_diff_lane_spec() -> ScenarioSpec {
        let mut spec = create_two_actor_same_lane_spec();
        spec.actors[1].lane = 0; // NPC in lane 0, ego in lane 1
        spec
    }

    /// Cartesian's `get_longitudinal_vel` returns a signed velocity
    /// (`velocities_x`, `cartesian.rs:768`); Bicycle's returns `speed_v`, which
    /// `bicycle.rs:1098` documents as non-negative and direction-locked. For a
    /// `direction: 1` actor the two coincide, which is exactly why the two
    /// shipped bicycle examples (`bicycle_lane_change.yaml`,
    /// `cut_in_right_bicycle.yaml`, both `direction: 1` throughout) never
    /// caught the divergence. This builds the *same* head-on situation — ego
    /// forward, npc oncoming (`direction: -1`), same lane slot so `same_lane`
    /// holds trivially — under both coordinate systems and checks that
    /// `get_longitudinal_vel` agrees on the closing speed, `|vx_ego - vx_npc|`.
    /// Physically that closing speed is `v_ego + v_npc` (they approach each
    /// other), which is what the signed cartesian encoder gives; the bicycle
    /// encoder must give the same number once its accessor returns a signed
    /// velocity too.
    fn oncoming_spec(coord: crate::dsl::types::CoordinateSystem) -> ScenarioSpec {
        let mut spec = create_two_actor_same_lane_spec();
        spec.coordinate_system = coord;
        spec.actors[0].speed = ValueOrRange::Value(10.0);
        spec.actors[0].direction = 1;
        spec.actors[1].speed = ValueOrRange::Value(8.0);
        spec.actors[1].direction = -1;
        spec.actors[1].position = ValueOrRange::Value(100.0);
        spec
    }

    /// `|vx_ego(0) - vx_npc(0)|` as reported through the `get_longitudinal_vel`
    /// accessor, for the [`oncoming_spec`] built under `coord`.
    fn oncoming_closing_speed(coord: crate::dsl::types::CoordinateSystem) -> f64 {
        let cfg = Config::new();
        let mut closing = None;
        z3::with_z3_config(&cfg, || {
            let spec = oncoming_spec(coord);
            let mut encoder = Z3Encoder::new(spec);
            encoder.create_variables();
            encoder.encode_initial_conditions();
            assert_eq!(encoder.check(), SatResult::Sat);
            let model = encoder.get_model().unwrap();
            let vx_ego = model
                .eval(encoder.get_longitudinal_vel("ego", 0), true)
                .unwrap();
            let vx_npc = model
                .eval(encoder.get_longitudinal_vel("npc", 0), true)
                .unwrap();
            let ego_v: f64 = crate::solver::backend::parse_z3_real_pub(&vx_ego.to_string());
            let npc_v: f64 = crate::solver::backend::parse_z3_real_pub(&vx_npc.to_string());
            closing = Some((ego_v - npc_v).abs());
        });
        closing.unwrap()
    }

    fn backward_lane_change_spec(coord: crate::dsl::types::CoordinateSystem) -> ScenarioSpec {
        ScenarioSpec {
            scenario_type: ScenarioType::CutInLeft,
            time_step: 0.5,
            duration: 10.0,
            actors: vec![
                ActorSpec {
                    id: "ego".to_string(),
                    role: ActorRole::Ego,
                    lane: 0,
                    position: ValueOrRange::Value(50.0),
                    speed: ValueOrRange::Value(15.0),
                    acceleration: ValueOrRange::Range([-8.0, 3.0]),
                    direction: -1,
                    behavior: HashMap::new(),
                    lane_changes: vec![],
                    bicycle_params: Some(crate::dsl::types::BicycleParams {
                        wheelbase: 2.7,
                        max_steering_angle: 0.5,
                        max_steering_rate: 0.5,
                    }),
                },
                ActorSpec {
                    id: "npc".to_string(),
                    role: ActorRole::Npc,
                    lane: 1,
                    position: ValueOrRange::Value(70.0),
                    speed: ValueOrRange::Value(15.0),
                    acceleration: ValueOrRange::Range([-8.0, 3.0]),
                    direction: -1,
                    behavior: HashMap::new(),
                    lane_changes: vec![LaneChangeConfig {
                        direction: LaneChangeDirection::Right,
                        start_time: ValueOrRange::Value(2.5),
                        duration: ValueOrRange::Value(3.0),
                    }],
                    bicycle_params: Some(crate::dsl::types::BicycleParams {
                        wheelbase: 2.7,
                        max_steering_angle: 0.5,
                        max_steering_rate: 0.5,
                    }),
                },
            ],
            min_ttc: 3.0,
            min_distance: 5.0,
            road: Some(RoadSpec {
                num_lanes: 3,
                lane_width: 3.5,
                lane_directions: vec![-1, -1, -1],
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
            coordinate_system: coord,
            bicycle_config: Some(crate::dsl::types::BicycleConfig {
                default_wheelbase: 2.7,
                default_max_steering_angle: 0.5,
                default_max_steering_rate: 0.5,
            }),
        }
    }

    /// `|py[H] - py[0]|`'s *signed* delta for `npc`, under `coord`, for the
    /// [`backward_lane_change_spec`] built with that coordinate system.
    fn backward_lane_change_delta(coord: crate::dsl::types::CoordinateSystem) -> f64 {
        let cfg = Config::new();
        let mut delta = None;
        z3::with_z3_config(&cfg, || {
            let spec = backward_lane_change_spec(coord);
            let mut encoder = Z3Encoder::new(spec);
            encoder.create_variables();
            encoder.encode_initial_conditions();
            encoder.encode_kinematics();
            encoder.encode_velocity_constraints();
            encoder.encode_acceleration_constraints();
            encoder.encode_lane_velocity_constraints();
            encoder.encode_lateral_velocity_bounds();
            assert_eq!(encoder.check(), SatResult::Sat, "{coord:?} should be SAT");
            let model = encoder.get_model().unwrap();
            let horizon = encoder.horizon();
            let py_start = model.eval(encoder.get_lateral_pos("npc", 0), true).unwrap();
            let py_end = model
                .eval(encoder.get_lateral_pos("npc", horizon), true)
                .unwrap();
            let py_start_f: f64 = crate::solver::backend::parse_z3_real_pub(&py_start.to_string());
            let py_end_f: f64 = crate::solver::backend::parse_z3_real_pub(&py_end.to_string());
            delta = Some(py_end_f - py_start_f);
        });
        delta.unwrap()
    }

    /// `LaneChangeDirection::Right`/`Left` are relative to the
    /// actor's own heading, not an absolute lane-index step (see the doc
    /// comment on `LaneChangeDirection` and
    /// `CartesianEncoder::encode_smooth_lane_transition`, both of which map
    /// `Right => actor.direction`, `Left => -actor.direction`). A hardcoded
    /// `Right => 1, Left => -1` in
    /// `BicycleEncoder::encode_lane_coupling_with_lane_changes` would agree
    /// with cartesian only for a forward actor. `backward_lane_change_spec`
    /// puts `npc` at
    /// `direction: -1` performing a `Right` lane change on the *same* YAML
    /// under both coordinate systems; the two must move it the same way.
    #[test]
    fn test_backward_actor_lane_change_direction_matches_across_coordinate_systems() {
        let cartesian_delta =
            backward_lane_change_delta(crate::dsl::types::CoordinateSystem::Cartesian);
        // A backward actor's "Right" is the opposite road-frame direction,
        // so this must be a *decrease* in lane index (one `lane_width` of
        // -3.5 m), not an increase.
        assert!(
            (cartesian_delta - (-3.5)).abs() < 1e-6,
            "cartesian: backward actor's Right lane change should step lane index down \
             (delta -3.5), got {cartesian_delta}"
        );

        let bicycle_delta =
            backward_lane_change_delta(crate::dsl::types::CoordinateSystem::Bicycle);
        assert!(
            (bicycle_delta - cartesian_delta).abs() < 1e-6,
            "bicycle's lane-change delta ({bicycle_delta}) must agree with cartesian's \
             ({cartesian_delta}) for the same backward-actor Right lane change: both must \
             treat Right/Left as relative to the actor's own heading"
        );
    }

    #[test]
    fn test_bicycle_oncoming_closing_speed_matches_cartesian() {
        let cartesian_closing =
            oncoming_closing_speed(crate::dsl::types::CoordinateSystem::Cartesian);
        assert!(
            (cartesian_closing - 18.0).abs() < 1e-6,
            "cartesian closing speed should be v_ego + v_npc = 18.0 m/s, got {cartesian_closing}"
        );

        let bicycle_closing = oncoming_closing_speed(crate::dsl::types::CoordinateSystem::Bicycle);
        assert!(
            (bicycle_closing - cartesian_closing).abs() < 1e-6,
            "bicycle closing speed ({bicycle_closing}) must agree with cartesian's \
             ({cartesian_closing}) for the same head-on situation: get_longitudinal_vel must \
             return a signed velocity, not speed_v's unsigned magnitude"
        );
    }

    #[test]
    fn test_encoder_creation() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let spec = create_test_spec();
            let encoder = Z3Encoder::new(spec);
            assert_eq!(encoder.horizon, 20); // 10.0 / 0.5 = 20
        });
    }

    #[test]
    fn test_create_variables() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let spec = create_test_spec();
            let mut encoder = Z3Encoder::new(spec);
            encoder.create_variables();

            // Variables are created internally - just verify accessor methods work
            // Test that we can access variables for both actors
            let _ego_lane = encoder.get_lane_var("ego", 0);
            let _npc_lane = encoder.get_lane_var("npc", 0);
            let _ego_px = encoder.get_longitudinal_pos("ego", 0);
            let _npc_px = encoder.get_longitudinal_pos("npc", 0);

            // If we get here without panicking, variables were created successfully
            assert_eq!(encoder.horizon(), 20);
        });
    }

    #[test]
    fn test_encode_initial_conditions() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let spec = create_test_spec();
            let mut encoder = Z3Encoder::new(spec);
            encoder.create_variables();
            encoder.encode_initial_conditions();

            // Check that constraints are satisfiable
            let result = encoder.check();
            assert_eq!(result, SatResult::Sat);

            // Get model and verify initial values
            let model = encoder.get_model().unwrap();

            // Ego position should be 50.0
            let ego_px_0 = model
                .eval(encoder.get_longitudinal_pos("ego", 0), true)
                .unwrap();
            println!("Ego initial position: {:?}", ego_px_0);

            // NPC position should be in range [60.0, 80.0]
            let npc_px_0 = model
                .eval(encoder.get_longitudinal_pos("npc", 0), true)
                .unwrap();
            println!("NPC initial position: {:?}", npc_px_0);

            // Ego speed should be 15.0
            let ego_vx_0 = model
                .eval(encoder.get_longitudinal_vel("ego", 0), true)
                .unwrap();
            println!("Ego initial speed: {:?}", ego_vx_0);

            let ego_pos_f64: f64 = crate::solver::backend::parse_z3_real_pub(&ego_px_0.to_string());
            let npc_pos_f64: f64 = crate::solver::backend::parse_z3_real_pub(&npc_px_0.to_string());
            let ego_spd_f64: f64 = crate::solver::backend::parse_z3_real_pub(&ego_vx_0.to_string());

            assert!(
                (ego_pos_f64 - 50.0).abs() < 0.1,
                "ego_pos should be ~50.0, got {}",
                ego_pos_f64
            );
            assert!(
                (ego_spd_f64 - 15.0).abs() < 0.1,
                "ego_speed should be ~15.0, got {}",
                ego_spd_f64
            );
            assert!(
                npc_pos_f64 >= 60.0 && npc_pos_f64 <= 80.0,
                "npc_pos should be in [60,80], got {}",
                npc_pos_f64
            );
        });
    }

    #[test]
    fn test_lane_position_coupling() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let spec = create_test_spec();
            let mut encoder = Z3Encoder::new(spec);
            encoder.create_variables();
            encoder.encode_initial_conditions();

            assert_eq!(encoder.check(), SatResult::Sat);

            let model = encoder.get_model().unwrap();

            // Ego in lane 1, should have py = 1 * 3.5 + 1.75 = 5.25
            let ego_py_0 = model.eval(encoder.get_lateral_pos("ego", 0), true).unwrap();
            println!("Ego lateral position: {:?}", ego_py_0);

            // NPC in lane 0, should have py = 0 * 3.5 + 1.75 = 1.75
            let npc_py_0 = model.eval(encoder.get_lateral_pos("npc", 0), true).unwrap();
            println!("NPC lateral position: {:?}", npc_py_0);

            let ego_py_f64: f64 = crate::solver::backend::parse_z3_real_pub(&ego_py_0.to_string());
            let npc_py_f64: f64 = crate::solver::backend::parse_z3_real_pub(&npc_py_0.to_string());

            // Exact lane centres: lane*width + width/2 with width = 3.5.
            // A tolerance as wide as 0.5 m would also accept 1.70 / 5.20, which
            // is what a `half_width` built as `(lane_width * 5.0) as i64 / 10`
            // (17.5 truncated to 17) produces instead. 1e-9 is the exactness
            // Z3's rationals give.
            assert!(
                (ego_py_f64 - 5.25).abs() < 1e-9,
                "ego_py should be exactly 5.25, got {}",
                ego_py_f64
            );
            assert!(
                (npc_py_f64 - 1.75).abs() < 1e-9,
                "npc_py should be exactly 1.75, got {}",
                npc_py_f64
            );
        });
    }

    #[test]
    fn test_kinematics() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let spec = create_test_spec();
            let mut encoder = Z3Encoder::new(spec);
            encoder.create_variables();
            encoder.encode_initial_conditions();
            encoder.encode_kinematics();
            encoder.encode_lane_velocity_constraints();
            encoder.encode_lateral_velocity_bounds();

            assert_eq!(encoder.check(), SatResult::Sat);

            let model = encoder.get_model().unwrap();

            // Check that position evolves correctly
            let ego_px_0 = model
                .eval(encoder.get_longitudinal_pos("ego", 0), true)
                .unwrap();
            let ego_px_1 = model
                .eval(encoder.get_longitudinal_pos("ego", 1), true)
                .unwrap();
            let ego_vx_0 = model
                .eval(encoder.get_longitudinal_vel("ego", 0), true)
                .unwrap();

            println!("Ego px[0]: {:?}", ego_px_0);
            println!("Ego px[1]: {:?}", ego_px_1);
            println!("Ego vx[0]: {:?}", ego_vx_0);

            // px[1] = px[0] + vx[0]*dt + 0.5*ax[0]*dt^2 with dt = 0.5.
            // A forward-Euler form (px0 + vx0*0.5, at a 0.1 tolerance) would be
            // short by 0.5*ax*dt^2 — 1.0 m per step at the ax = -8 the solver
            // picks here, ten times that tolerance.
            let ego_vx_1 = model
                .eval(encoder.get_longitudinal_vel("ego", 1), true)
                .unwrap();
            let px0_f64: f64 = crate::solver::backend::parse_z3_real_pub(&ego_px_0.to_string());
            let px1_f64: f64 = crate::solver::backend::parse_z3_real_pub(&ego_px_1.to_string());
            let vx0_f64: f64 = crate::solver::backend::parse_z3_real_pub(&ego_vx_0.to_string());
            let vx1_f64: f64 = crate::solver::backend::parse_z3_real_pub(&ego_vx_1.to_string());

            // ax[0] = (vx[1] - vx[0]) / dt, since the velocity update is exact.
            let dt = 0.5;
            let ax0_f64 = (vx1_f64 - vx0_f64) / dt;
            let expected = px0_f64 + vx0_f64 * dt + 0.5 * ax0_f64 * dt * dt;
            assert!(
                (px1_f64 - expected).abs() < 1e-9,
                "kinematic relation violated: px1={px1_f64}, expected {expected} \
                 (px0={px0_f64}, vx0={vx0_f64}, ax0={ax0_f64})"
            );
        });
    }

    #[test]
    fn test_safety_constraints() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let spec = create_test_spec();
            let mut encoder = Z3Encoder::new(spec);
            encoder.create_variables();
            encoder.encode_initial_conditions();
            encoder.encode_kinematics();
            encoder.encode_lane_velocity_constraints();
            encoder.encode_lateral_velocity_bounds();
            // Safety constraints are now included in LTL formula via generate_safety()

            // Safety constraints should be satisfiable
            assert_eq!(encoder.check(), SatResult::Sat);
        });
    }

    #[test]
    fn test_full_cut_in_scenario() {
        use crate::ltl::generator::LTLGenerator;

        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let spec = create_test_spec();
            let mut encoder = Z3Encoder::new(spec.clone());
            encoder.create_variables();
            encoder.encode_initial_conditions();
            encoder.encode_kinematics();
            encoder.encode_lane_velocity_constraints();
            encoder.encode_lateral_velocity_bounds();

            // Generate and encode full cut-in LTL formula
            let ltl_formula = LTLGenerator::generate(&spec).unwrap();
            encoder.encode_ltl(&ltl_formula);

            // Add safety constraints
            // Safety constraints are now included in LTL formula via generate_safety()

            // Check satisfiability
            let result = encoder.check();
            assert_eq!(
                result,
                SatResult::Sat,
                "Full cut-in scenario should be satisfiable"
            );

            if result == SatResult::Sat {
                let model = encoder.get_model().unwrap();

                // Verify initial conditions
                let ego_lane_0 = model.eval(encoder.get_lane_var("ego", 0), true).unwrap();
                let npc_lane_0 = model.eval(encoder.get_lane_var("npc", 0), true).unwrap();
                assert_eq!(ego_lane_0.to_string(), "1");
                assert_eq!(npc_lane_0.to_string(), "0");

                // Verify NPC eventually changes lanes
                let mut npc_in_lane_1 = false;
                for t in 0..=encoder.horizon {
                    let lane = model.eval(encoder.get_lane_var("npc", t), true).unwrap();
                    if lane.to_string() == "1" {
                        npc_in_lane_1 = true;
                        println!("NPC changes to lane 1 at time step {}", t);
                        break;
                    }
                }
                assert!(npc_in_lane_1, "NPC should eventually change to lane 1");

                println!("Full cut-in scenario test passed!");
            }
        });
    }

    #[test]
    fn test_unsatisfiable_constraints() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            use crate::ltl::formula::{LTLFormula, Proposition};

            let spec = create_two_actor_same_lane_spec();
            let mut encoder = Z3Encoder::new(spec);
            encoder.create_variables();
            encoder.encode_initial_conditions();

            // Ego must be in lane 0 AND lane 1 simultaneously at time 0
            let in_lane_0 = LTLFormula::Atom(Proposition::InLane {
                actor: "ego".to_string(),
                lane: 0,
            });
            let in_lane_1 = LTLFormula::Atom(Proposition::InLane {
                actor: "ego".to_string(),
                lane: 1,
            });
            let formula = LTLFormula::And(Box::new(in_lane_0), Box::new(in_lane_1));
            encoder.encode_ltl(&formula);

            assert_eq!(
                encoder.check(),
                SatResult::Unsat,
                "Actor cannot be in two lanes simultaneously"
            );
        });
    }

    // ===== Group 2: TTC constraint encoding =====

    #[test]
    fn test_ttc_constraint_same_lane_approaching() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            use crate::ltl::formula::{LTLFormula, Proposition};

            // ego at 50 speed 20, npc at 100 speed 15 — ego approaching npc
            let spec = create_two_actor_same_lane_spec();
            let mut encoder = Z3Encoder::new(spec);
            encoder.create_variables();
            encoder.encode_initial_conditions();
            encoder.encode_kinematics();
            encoder.encode_lane_velocity_constraints();

            // Always TTC > 3.0
            let formula = LTLFormula::Atom(Proposition::TTCGT {
                actor1: "ego".to_string(),
                actor2: "npc".to_string(),
                ttc: 3.0,
            })
            .always();
            encoder.encode_ltl(&formula);

            // Should be satisfiable — distance=50, rel_vel=5, TTC=10 initially
            assert_eq!(encoder.check(), SatResult::Sat);
        });
    }

    #[test]
    fn test_ttc_constraint_different_lanes_unconstrained() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            use crate::ltl::formula::{LTLFormula, Proposition};

            // Actors in different lanes — TTC constraint is trivially satisfied
            let spec = create_two_actor_diff_lane_spec();
            let mut encoder = Z3Encoder::new(spec);
            encoder.create_variables();
            encoder.encode_initial_conditions();
            encoder.encode_kinematics();
            encoder.encode_lane_velocity_constraints();

            // Always TTC > 100.0 (very high threshold, but different lanes so no issue)
            let formula = LTLFormula::Atom(Proposition::TTCGT {
                actor1: "ego".to_string(),
                actor2: "npc".to_string(),
                ttc: 100.0,
            })
            .always();
            encoder.encode_ltl(&formula);

            assert_eq!(encoder.check(), SatResult::Sat);
        });
    }
}
