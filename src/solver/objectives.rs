//! Optimizer objective encoding.
//!
//! Encodes each [`OptimizationTarget`] as a QF_LRA objective over the Z3 `Optimize`
//! backend: `MinimizeDistance`/`MaximizeSeverity` read the effective same-lane gap or
//! closing speed directly, while `MinimizeTtc`/`MaximizeTtc` walk the [`TTC_LEVELS`]
//! ladder (see its doc comment for why TTC itself cannot be a linear objective). Every
//! objective is scored over the same "same lane" predicate
//! `compute_validation_metrics` (`src/scenario/metrics.rs`) uses when reporting
//! `min_distance`/`min_ttc`, so what is optimised is what is reported.

use z3::ast::{Bool, Real};

use crate::solver::backend::{OptimizationTarget, OptimizerBackend, Z3Backend};
use crate::solver::encoder::{directed_conflict, DirectedConflict, GenericEncoder};
use crate::solver::encoder_utils::{encode_same_lane_constraint, real_from_f64};

/// Candidate time-to-collision levels, in seconds, used by the `MinimizeTtc` and
/// `MaximizeTtc` objectives.
///
/// TTC is `gap / closing_speed`. A *division* is non-linear, and the whole encoding is
/// deliberately confined to QF_LRA, so no single linear term can rank scenarios by TTC:
/// TTC is scale-invariant (`(d, v)` and `(λd, λv)` have the same TTC) and no linear
/// function of `d` and `v` is. The previous encoding tried anyway — it minimised
/// `d - dt·v` — and **inverted the ranking it claimed to produce**: with `dt = 0.5` it
/// scored `(d = 2 m, v = 0)`, whose TTC is infinite, at 2.0 and `(d = 50 m, v = 20 m/s)`,
/// whose TTC is 2.5 s, at 40.0, so *minimising* it preferred the infinite-TTC state.
///
/// The way out is that TTC only becomes non-linear when the threshold is a *variable*.
/// For a **constant** `T`, both `ttc ≥ T` and `ttc ≤ T` are linear:
///
/// ```text
/// ttc(d, v) ≥ T   ⟺   d ≥ T · v      (v > 0)
/// ttc(d, v) ≤ T   ⟺   d ≤ T · v      (v > 0)
/// ```
///
/// So the objective is expressed over a fixed ladder of constant levels: one Boolean per
/// level saying "the whole trajectory clears this level" (for `MaximizeTtc`) or "some step
/// falls below it" (for `MinimizeTtc`), and a linear pseudo-Boolean sum that reads the
/// ladder off as a number of seconds. Every product is `constant × variable`, so the
/// encoding stays in QF_LRA and `Optimize` keeps its decision procedure.
///
/// **What this costs:** the objective is a *grid-resolution* measurement of TTC, not an
/// exact one, and the ladder is deliberately fine where TTC matters (0.25 s steps below
/// 1 s) and coarse where it does not (10 s steps beyond 30 s). The objective value is
/// therefore a **certified bound**, not an equality — see `encode_minimize_ttc_objective`
/// and `encode_maximize_ttc_objective` for the exact direction of each bound.
const TTC_LEVELS: [f64; 21] = [
    0.25, 0.5, 0.75, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 5.0, 6.0, 8.0, 10.0, 12.5, 15.0, 20.0,
    25.0, 30.0, 40.0, 60.0,
];

impl GenericEncoder<OptimizerBackend> {
    /// Encode optimization objective based on the target.
    ///
    /// Every objective is Linear Real Arithmetic — the whole encoding is confined to
    /// QF_LRA and `Optimize` has essentially no support for non-linear objectives — and
    /// every one is scored over the *same* "same lane" predicate the validator uses when
    /// it reports `min_distance` and `min_ttc`, so the quantity optimised is the quantity
    /// reported.
    ///
    /// - **`MinimizeDistance`**: minimise the smallest same-lane longitudinal gap.
    /// - **`MinimizeTtc`**: minimise the smallest same-lane time-to-collision, over the
    ///   [`TTC_LEVELS`] ladder.
    /// - **`MaximizeSeverity`**: maximise the highest same-lane closing speed.
    /// - **`MaximizeTtc`**: maximise the smallest same-lane time-to-collision, over the
    ///   [`TTC_LEVELS`] ladder.
    pub fn encode_objective(&mut self) {
        let target = self.coord_encoder.backend().target();
        match target {
            OptimizationTarget::MinimizeDistance => {
                self.encode_minimize_distance_objective();
            }
            OptimizationTarget::MinimizeTtc => {
                self.encode_minimize_ttc_objective();
            }
            OptimizationTarget::MaximizeSeverity => {
                self.encode_maximize_severity_objective();
            }
            OptimizationTarget::MaximizeTtc => {
                self.encode_maximize_ttc_objective();
            }
        }
    }

    /// MinimizeDistance: find the scenario with the smallest same-lane gap.
    ///
    /// Combines lower-bound constraints (`obj <= effective_dist[t]` for all t) with
    /// a "choice" constraint (`obj = effective_dist[t]` for at least one t).
    /// Together these force `obj` to equal the actual minimum distance across all
    /// time steps. Z3 then minimizes this value by choosing the scenario parameters
    /// that make the tightest approach as close as possible.
    fn encode_minimize_distance_objective(&mut self) {
        let obj = Real::new_const("dist_obj");
        let zero = Real::from_rational(0, 1);
        let big_val = Real::from_rational(9999, 1);
        self.coord_encoder.backend_mut().assert(&obj.ge(&zero));

        let actor_ids: Vec<String> = self.spec.actors.iter().map(|a| a.id.clone()).collect();
        let mut choices = Vec::new();

        for i in 0..actor_ids.len() {
            for j in (i + 1)..actor_ids.len() {
                for t in 0..=self.horizon {
                    let effective_dist =
                        self.compute_effective_dist(&actor_ids[i], &actor_ids[j], t, &big_val);
                    // Lower bound: obj <= effective_dist at every (pair, time)
                    self.coord_encoder
                        .backend_mut()
                        .assert(&obj.le(&effective_dist));
                    // Choice: obj can equal this particular effective_dist
                    choices.push(obj.eq(&effective_dist));
                }
            }
        }

        // At least one choice must hold (obj = some effective distance)
        let refs: Vec<&Bool> = choices.iter().collect();
        self.coord_encoder.backend_mut().assert(&Bool::or(&refs));

        self.coord_encoder.backend_mut().minimize(&obj);
        self.coord_encoder.backend_mut().set_objective_var(obj);
    }

    /// MinimizeTtc: find the scenario with the smallest time-to-collision.
    ///
    /// One Boolean `b_k` per level of [`TTC_LEVELS`], meaning "some same-lane approaching
    /// pair, at some step, has `ttc ≤ T_k`":
    ///
    /// ```text
    /// b_k  ⇒  ⋁_{pair, t, direction} ( guard ∧ gap ≤ T_k · closing )
    /// b_k  ⇒  b_{k+1}                       (levels are nested upwards)
    /// obj  =  T_K − Σ_{k<K} (T_{k+1} − T_k) · [b_k]
    /// ```
    ///
    /// With `b_k` true exactly for `k ≥ m` the sum telescopes and `obj = T_m`, so
    /// minimising `obj` walks `m` down the ladder. Every coefficient is a constant, so
    /// this is linear.
    ///
    /// **The bound is one-sided and that is deliberate.** `b_m` is only an implication, so
    /// asserting it *forces* a step with `ttc ≤ T_m` to exist: the reported optimum is a
    /// **certified upper bound** on the min-TTC of the trajectory that ships with it —
    /// `measured min TTC ≤ optimal_value`. It is never an over-claim, only ever
    /// pessimistic by less than one grid cell.
    fn encode_minimize_ttc_objective(&mut self) {
        let conflicts = self.collect_directed_conflicts();
        let levels = &TTC_LEVELS;
        let top = levels[levels.len() - 1];

        let obj = Real::new_const("ttc_obj");
        let mut terms: Vec<Real> = vec![real_from_f64(top)];

        for (k, &level) in levels.iter().enumerate().take(levels.len() - 1) {
            let below = Bool::new_const(format!("ttc_below_{k}"));

            // below_k ⇒ ⋁ (guard ∧ gap ≤ T_k · closing)
            let level_real = real_from_f64(level);
            let witnesses: Vec<Bool> = conflicts
                .iter()
                .map(|c| Bool::and(&[&c.guard, &c.gap.le(&(&level_real * &c.closing))]))
                .collect();
            let witness_refs: Vec<&Bool> = witnesses.iter().collect();
            let some_witness = if witness_refs.is_empty() {
                Bool::from_bool(false)
            } else {
                Bool::or(&witness_refs)
            };
            self.coord_encoder
                .backend_mut()
                .assert(&below.implies(&some_witness));

            // Nesting: falling below T_k means falling below every higher level too.
            if k + 1 < levels.len() - 1 {
                let next = Bool::new_const(format!("ttc_below_{}", k + 1));
                self.coord_encoder
                    .backend_mut()
                    .assert(&below.implies(&next));
            }

            // obj = T_K − Σ (T_{k+1} − T_k) · [below_k]
            let step = real_from_f64(levels[k + 1] - level);
            let zero = Real::from_rational(0, 1);
            terms.push(-below.ite(&step, &zero));
        }

        let term_refs: Vec<&Real> = terms.iter().collect();
        self.coord_encoder
            .backend_mut()
            .assert(&obj.eq(Real::add(&term_refs)));

        self.coord_encoder.backend_mut().minimize(&obj);
        self.coord_encoder.backend_mut().set_objective_var(obj);
    }

    /// MaximizeSeverity: find the scenario with the highest same-lane closing speed.
    ///
    /// Severity correlates with relative impact speed, and this objective drives that up,
    /// which is what an adversarial scenario generator wants.
    ///
    /// Uses a "choice" pattern: obj can equal any effective_closing_speed value, and Z3
    /// maximizes it by choosing scenario parameters that produce the highest approach
    /// speed.
    fn encode_maximize_severity_objective(&mut self) {
        let obj = Real::new_const("severity_obj");
        let zero = Real::from_rational(0, 1);
        self.coord_encoder.backend_mut().assert(&obj.ge(&zero));

        let actor_ids: Vec<String> = self.spec.actors.iter().map(|a| a.id.clone()).collect();
        let mut choices = Vec::new();

        for i in 0..actor_ids.len() {
            for j in (i + 1)..actor_ids.len() {
                for t in 0..=self.horizon {
                    let effective_speed =
                        self.compute_effective_closing_speed(&actor_ids[i], &actor_ids[j], t);
                    // Choice: obj can equal this particular effective closing speed
                    choices.push(obj.eq(&effective_speed));
                }
            }
        }

        // At least one choice must hold (obj = some effective closing speed)
        let refs: Vec<&Bool> = choices.iter().collect();
        self.coord_encoder.backend_mut().assert(&Bool::or(&refs));

        self.coord_encoder.backend_mut().maximize(&obj);
        self.coord_encoder.backend_mut().set_objective_var(obj);
    }

    /// MaximizeTtc: find the scenario with the largest time-to-collision.
    ///
    /// The mirror of [`Self::encode_minimize_ttc_objective`]. One Boolean `g_k` per level
    /// of [`TTC_LEVELS`], meaning "*every* same-lane approaching pair, at *every* step,
    /// has `ttc ≥ T_k`":
    ///
    /// ```text
    /// g_k  ⇒  ⋀_{pair, t, direction} ( guard ⇒ gap ≥ T_k · closing )
    /// g_k  ⇒  g_{k−1}                       (levels are nested downwards)
    /// obj  =  Σ_k (T_k − T_{k−1}) · [g_k]   with T_{−1} = 0
    /// ```
    ///
    /// With `g_k` true exactly for `k ≤ M` the sum telescopes and `obj = T_M`. This is a
    /// conjunction of implications rather than a disjunction, so it *propagates* rather
    /// than branching.
    ///
    /// The bound runs the other way from the minimiser: `g_M` forces every approaching
    /// step to clear `T_M`, so the reported optimum is a **certified lower bound** —
    /// `optimal_value ≤ measured min TTC`.
    ///
    /// Dispatching this target to a *distance* objective instead — maximising the
    /// minimum gap and calling it TTC — would be wrong: on
    /// `examples/cut_in_left_optimize_max_ttc.yaml` that reports a 113.50 m gap in
    /// cartesian and 126.88 m in bicycle — 12 % apart — while the TTC it claims to
    /// maximise comes out 12.37 s and 44.76 s, a factor of 3.6.
    fn encode_maximize_ttc_objective(&mut self) {
        let conflicts = self.collect_directed_conflicts();
        let levels = &TTC_LEVELS;

        let obj = Real::new_const("ttc_obj");
        let mut terms: Vec<Real> = Vec::new();
        let mut previous = 0.0_f64;

        for (k, &level) in levels.iter().enumerate() {
            let clears = Bool::new_const(format!("ttc_clears_{k}"));

            // clears_k ⇒ ⋀ (guard ⇒ gap ≥ T_k · closing)
            let level_real = real_from_f64(level);
            let per_conflict: Vec<Bool> = conflicts
                .iter()
                .map(|c| c.guard.implies(c.gap.ge(&(&level_real * &c.closing))))
                .collect();
            let refs: Vec<&Bool> = per_conflict.iter().collect();
            let all_clear = if refs.is_empty() {
                Bool::from_bool(true)
            } else {
                Bool::and(&refs)
            };
            self.coord_encoder
                .backend_mut()
                .assert(&clears.implies(&all_clear));

            // Nesting: clearing T_k means clearing every lower level too.
            if k > 0 {
                let lower = Bool::new_const(format!("ttc_clears_{}", k - 1));
                self.coord_encoder
                    .backend_mut()
                    .assert(&clears.implies(&lower));
            }

            // obj = Σ (T_k − T_{k−1}) · [clears_k], T_{−1} = 0
            let step = real_from_f64(level - previous);
            let zero = Real::from_rational(0, 1);
            terms.push(clears.ite(&step, &zero));
            previous = level;
        }

        let term_refs: Vec<&Real> = terms.iter().collect();
        self.coord_encoder
            .backend_mut()
            .assert(&obj.eq(Real::add(&term_refs)));

        self.coord_encoder.backend_mut().maximize(&obj);
        self.coord_encoder.backend_mut().set_objective_var(obj);
    }

    // === Helper methods for objective encoding ===

    /// Every directed "A is behind B in the same lane and gaining" conflict in the
    /// encoding, one per (unordered pair, step, direction).
    ///
    /// This is the encoder-side twin of the TTC block in `compute_validation_metrics`: the
    /// same "same lane" predicate ([`encode_same_lane_constraint`] — discrete lane match
    /// *or* lateral overlap), the same requirement that the follower be strictly behind,
    /// and the same closing-speed floor. Sharing the predicate is the whole point: testing
    /// `lane_i == lane_j` alone would score a different set of states than the validator
    /// measures, and the two would diverge most in the bicycle coordinate system, where a
    /// lane change spends many steps laterally overlapping without a discrete lane match.
    fn collect_directed_conflicts(&self) -> Vec<DirectedConflict> {
        let actor_ids: Vec<String> = self.spec.actors.iter().map(|a| a.id.clone()).collect();

        let mut conflicts = Vec::new();
        for i in 0..actor_ids.len() {
            for j in (i + 1)..actor_ids.len() {
                for t in 0..=self.horizon {
                    for (follow, lead) in [
                        (&actor_ids[i], &actor_ids[j]),
                        (&actor_ids[j], &actor_ids[i]),
                    ] {
                        // The body of this loop lives in `directed_conflict`, so
                        // the `Converging` proposition and these objectives assert
                        // the same predicate by construction.
                        conflicts.push(directed_conflict(self, &self.spec, follow, lead, t));
                    }
                }
            }
        }
        conflicts
    }

    /// Compute effective distance between two actors at time t.
    ///
    /// Returns `|px_i - px_j|` when the pair is in the same lane, `big_val` otherwise.
    /// "Same lane" is [`encode_same_lane_constraint`] — the same predicate
    /// `compute_validation_metrics` uses to decide which gaps enter the reported
    /// `min_distance`. Using `lane_i == lane_j` alone here would score a narrower
    /// set of states than the tool reports on.
    fn compute_effective_dist(&self, aid_i: &str, aid_j: &str, t: usize, big_val: &Real) -> Real {
        let px_i = self.get_longitudinal_pos(aid_i, t);
        let px_j = self.get_longitudinal_pos(aid_j, t);

        let same_lane = self.same_lane_at(aid_i, aid_j, t);
        let abs_dist = px_i.gt(px_j).ite(&(px_i - px_j), &(px_j - px_i));
        same_lane.ite(&abs_dist, big_val)
    }

    /// The validator's "same lane" predicate, as a Z3 term.
    fn same_lane_at(&self, aid_i: &str, aid_j: &str, t: usize) -> Bool {
        encode_same_lane_constraint(
            self.get_lane_var(aid_i, t),
            self.get_lane_var(aid_j, t),
            self.get_lateral_pos(aid_i, t),
            self.get_lateral_pos(aid_j, t),
            self.spec.get_lane_width(),
        )
    }

    /// Compute closing speed (absolute relative velocity) between two actors at time t.
    /// Returns |vx_i - vx_j| — purely linear (LRA), no division involved.
    fn compute_closing_speed(&self, aid_i: &str, aid_j: &str, t: usize) -> Real {
        let vx_i = self.get_longitudinal_vel(aid_i, t);
        let vx_j = self.get_longitudinal_vel(aid_j, t);
        let v_rel = vx_i - vx_j;
        let zero = Real::from_rational(0, 1);
        v_rel.gt(&zero).ite(&v_rel, &(-&v_rel))
    }

    /// Compute effective closing speed: |vx_i - vx_j| when same lane, 0 otherwise.
    ///
    /// Used by the severity objective to maximize approach speed. Same-lane is the
    /// validator's predicate (see [`Self::compute_effective_dist`]).
    fn compute_effective_closing_speed(&self, aid_i: &str, aid_j: &str, t: usize) -> Real {
        let closing_speed = self.compute_closing_speed(aid_i, aid_j, t);
        let zero = Real::from_rational(0, 1);

        self.same_lane_at(aid_i, aid_j, t)
            .ite(&closing_speed, &zero)
    }

    /// Extract the optimal value from the Z3 model after solving.
    pub fn extract_optimal_value(&mut self, model: &z3::Model) {
        self.coord_encoder
            .backend_mut()
            .extract_optimal_value(model);
    }

    /// Get the optimal value found by the optimizer.
    pub fn get_optimal_value(&self) -> Option<f64> {
        self.coord_encoder.backend().get_optimal_value()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dsl::types::{
        ActorRole, ActorSpec, LaneChangeConfig, LaneChangeDirection, RoadSpec, ScenarioSpec,
        ScenarioType, ValueOrRange,
    };
    use std::collections::HashMap;
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

    // ===== Group 4: Optimizer encoder =====

    #[test]
    fn test_optimizer_minimize_distance() {
        use crate::ltl::generator::LTLGenerator;
        use crate::solver::backend::OptimizationTarget;
        use crate::solver::backend::OptimizerBackend;

        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let spec = create_test_spec();
            let backend = OptimizerBackend::new(OptimizationTarget::MinimizeDistance);
            let mut encoder = GenericEncoder::with_backend(spec.clone(), backend);
            encoder.create_variables();
            encoder.encode_initial_conditions();
            encoder.encode_kinematics();
            encoder.encode_lane_velocity_constraints();
            encoder.encode_lateral_velocity_bounds();

            let ltl_formula = LTLGenerator::generate(&spec).unwrap();
            encoder.encode_ltl(&ltl_formula);
            encoder.encode_objective();

            let result = encoder.check();
            assert_eq!(result, SatResult::Sat, "Optimizer should find a solution");

            let model = encoder.get_model().unwrap();
            encoder.extract_optimal_value(&model);
            let optimal = encoder.get_optimal_value();
            assert!(optimal.is_some(), "Should have an optimal value");
            let val = optimal.unwrap();
            assert!(val >= 0.0, "Minimum distance {} should be >= 0", val);
            println!("Minimized distance: {}", val);
        });
    }

    #[test]
    fn test_optimizer_maximize_distance() {
        use crate::ltl::generator::LTLGenerator;
        use crate::solver::backend::OptimizationTarget;
        use crate::solver::backend::OptimizerBackend;

        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let spec = create_test_spec();
            let backend = OptimizerBackend::new(OptimizationTarget::MaximizeTtc);
            let mut encoder = GenericEncoder::with_backend(spec.clone(), backend);
            encoder.create_variables();
            encoder.encode_initial_conditions();
            encoder.encode_kinematics();
            encoder.encode_lane_velocity_constraints();
            encoder.encode_lateral_velocity_bounds();

            let ltl_formula = LTLGenerator::generate(&spec).unwrap();
            encoder.encode_ltl(&ltl_formula);
            encoder.encode_objective();

            let result = encoder.check();
            assert_eq!(result, SatResult::Sat, "Optimizer should find a solution");

            let model = encoder.get_model().unwrap();
            encoder.extract_optimal_value(&model);
            let optimal = encoder.get_optimal_value();
            assert!(optimal.is_some(), "Should have an optimal value");
            let val = optimal.unwrap();
            assert!(val > 0.0, "Maximized distance {} should be > 0", val);
            println!("Maximized distance: {}", val);
        });
    }

    // ===== Group 6: Optimizer regression tests =====

    #[test]
    fn test_optimizer_single_actor_returns_unsat() {
        use crate::solver::backend::OptimizationTarget;
        use crate::solver::backend::OptimizerBackend;

        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let spec = ScenarioSpec {
                scenario_type: ScenarioType::CutInLeft,
                time_step: 0.5,
                duration: 5.0,
                actors: vec![ActorSpec {
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
                }],
                min_ttc: 3.0,
                min_distance: 5.0,
                road: None,
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
            };

            let backend = OptimizerBackend::new(OptimizationTarget::MinimizeDistance);
            let mut encoder = GenericEncoder::with_backend(spec, backend);
            encoder.create_variables();
            encoder.encode_initial_conditions();
            encoder.encode_kinematics();
            encoder.encode_velocity_constraints();
            encoder.encode_acceleration_constraints();
            encoder.encode_objective();

            let result = encoder.check();
            assert_eq!(
                result,
                SatResult::Unsat,
                "Single-actor optimization should return UNSAT (no actor pairs exist)"
            );
        });
    }

    #[test]
    fn test_compute_closing_speed() {
        use crate::solver::backend::OptimizationTarget;
        use crate::solver::backend::OptimizerBackend;
        use z3::ast::Ast;

        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let spec = create_two_actor_same_lane_spec();
            let backend = OptimizerBackend::new(OptimizationTarget::MinimizeDistance);
            let mut encoder = GenericEncoder::with_backend(spec, backend);
            encoder.create_variables();
            encoder.encode_initial_conditions();

            // ego speed=20, npc speed=15 → closing speed = |20-15| = 5
            let closing_speed = encoder.compute_closing_speed("ego", "npc", 0);

            let expected = Real::from_rational(5, 1);
            encoder.assert_constraint(&closing_speed._eq(&expected));

            let result = encoder.check();
            assert_eq!(
                result,
                SatResult::Sat,
                "Closing speed at t=0 should be 5 (|20-15|)"
            );
        });
    }

    /// The directed conflicts the TTC objectives are scored over.
    ///
    /// Replaces `test_compute_ttc_proxy_same_lane`, which pinned the old
    /// `|Δpx| − dt·|Δvx|` proxy at 47.5 — a quantity that was never a TTC and
    /// that ranked an infinite TTC as worse than a 2.5 s one. What matters now
    /// is that `gap` and `closing` are the two halves of the validator's own
    /// TTC quotient, under the validator's own guard.
    #[test]
    fn test_collect_directed_conflicts_matches_the_validator_quotient() {
        use crate::solver::backend::OptimizationTarget;
        use crate::solver::backend::OptimizerBackend;
        use z3::ast::Ast;

        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let spec = create_two_actor_same_lane_spec();
            let backend = OptimizerBackend::new(OptimizationTarget::MinimizeTtc);
            let mut encoder = GenericEncoder::with_backend(spec, backend);
            encoder.create_variables();
            encoder.encode_initial_conditions();

            let conflicts = encoder.collect_directed_conflicts();
            assert!(
                !conflicts.is_empty(),
                "two actors over a horizon must yield directed conflicts"
            );

            // ego is at px=50 doing 20 m/s, npc at px=100 doing 15 m/s, same
            // lane: ego is the follower and is gaining, so at t=0 the ego->npc
            // conflict is guarded, with gap 50 m and closing speed 5 m/s —
            // TTC 10 s, exactly what `compute_validation_metrics` would report.
            let c = &conflicts[0];
            encoder.assert_constraint(&c.guard);
            encoder.assert_constraint(&c.gap._eq(&Real::from_rational(50, 1)));
            encoder.assert_constraint(&c.closing._eq(&Real::from_rational(5, 1)));

            assert_eq!(
                encoder.check(),
                SatResult::Sat,
                "the first directed conflict should be ego closing on npc with a 50 m gap \
                 at 5 m/s"
            );
        });
    }

    #[test]
    fn test_compute_effective_closing_speed_different_lanes() {
        use crate::solver::backend::OptimizationTarget;
        use crate::solver::backend::OptimizerBackend;
        use z3::ast::Ast;

        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let spec = create_two_actor_diff_lane_spec();
            let backend = OptimizerBackend::new(OptimizationTarget::MaximizeSeverity);
            let mut encoder = GenericEncoder::with_backend(spec, backend);
            encoder.create_variables();
            encoder.encode_initial_conditions();

            let eff_speed = encoder.compute_effective_closing_speed("ego", "npc", 0);

            // Different lanes → effective closing speed = 0
            let zero = Real::from_rational(0, 1);
            encoder.assert_constraint(&eff_speed._eq(&zero));

            let result = encoder.check();
            assert_eq!(
                result,
                SatResult::Sat,
                "Effective closing speed should be 0 for different lanes"
            );
        });
    }

    #[test]
    fn test_encoder_accessor_with_optimizer_backend() {
        use crate::solver::backend::OptimizationTarget;
        use crate::solver::backend::OptimizerBackend;
        use crate::solver::encoder::EncoderAccessor;

        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let spec = create_two_actor_same_lane_spec();
            let backend = OptimizerBackend::new(OptimizationTarget::MinimizeDistance);
            let mut encoder = GenericEncoder::with_backend(spec, backend);
            encoder.create_variables();

            let accessor: &dyn EncoderAccessor = &encoder;

            // The trait objects must hand back the encoder's own variables,
            // named per actor and per time step.
            assert_eq!(accessor.get_lane_var("ego", 0).to_string(), "ego_lane_0");
            assert_eq!(
                accessor.get_longitudinal_pos("ego", 0).to_string(),
                "ego_px_0"
            );
            assert_eq!(
                accessor.get_longitudinal_vel("ego", 0).to_string(),
                "ego_vx_0"
            );
            assert_eq!(accessor.get_lateral_pos("ego", 0).to_string(), "ego_py_0");
            assert_eq!(accessor.get_lateral_vel("ego", 0).to_string(), "ego_vy_0");

            assert_eq!(accessor.get_lane_var("npc", 0).to_string(), "npc_lane_0");
            assert_eq!(
                accessor.get_longitudinal_pos("npc", 0).to_string(),
                "npc_px_0"
            );

            // Distinct time steps must be distinct variables.
            assert_ne!(
                accessor.get_longitudinal_pos("ego", 0).to_string(),
                accessor.get_longitudinal_pos("ego", 1).to_string()
            );
        });
    }

    #[test]
    fn test_optimizer_different_lanes_sentinel_values() {
        use crate::solver::backend::OptimizationTarget;
        use crate::solver::backend::OptimizerBackend;

        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let mut spec = create_two_actor_same_lane_spec();
            spec.actors[1].lane = 3;
            spec.road = Some(RoadSpec {
                num_lanes: 5,
                lane_width: 3.5,
                lane_directions: vec![1, 1, 1, 1, 1],
                road_length: None,
            });

            let backend = OptimizerBackend::new(OptimizationTarget::MinimizeDistance);
            let mut encoder = GenericEncoder::with_backend(spec.clone(), backend);
            encoder.create_variables();
            encoder.encode_initial_conditions();
            encoder.encode_kinematics();
            encoder.encode_velocity_constraints();
            encoder.encode_acceleration_constraints();
            encoder.encode_lane_velocity_constraints();
            encoder.encode_lateral_velocity_bounds();
            encoder.encode_objective();

            let result = encoder.check();
            if result == SatResult::Sat {
                let model = encoder.get_model().unwrap();
                encoder.extract_optimal_value(&model);
                let val = encoder.get_optimal_value();
                if let Some(v) = val {
                    assert!(
                        (v - 9999.0).abs() < 1.0,
                        "Different-lanes distance should be sentinel 9999, got {}",
                        v
                    );
                }
            }
        });

        let cfg2 = Config::new();
        z3::with_z3_config(&cfg2, || {
            let mut spec = create_two_actor_same_lane_spec();
            spec.actors[1].lane = 3;
            spec.road = Some(RoadSpec {
                num_lanes: 5,
                lane_width: 3.5,
                lane_directions: vec![1, 1, 1, 1, 1],
                road_length: None,
            });

            let backend = OptimizerBackend::new(OptimizationTarget::MaximizeSeverity);
            let mut encoder = GenericEncoder::with_backend(spec, backend);
            encoder.create_variables();
            encoder.encode_initial_conditions();
            encoder.encode_kinematics();
            encoder.encode_velocity_constraints();
            encoder.encode_acceleration_constraints();
            encoder.encode_lane_velocity_constraints();
            encoder.encode_lateral_velocity_bounds();
            encoder.encode_objective();

            let result = encoder.check();
            if result == SatResult::Sat {
                let model = encoder.get_model().unwrap();
                encoder.extract_optimal_value(&model);
                let val = encoder.get_optimal_value();
                if let Some(v) = val {
                    assert!(
                        v.abs() < 1.0,
                        "Different-lanes severity should be ~0, got {}",
                        v
                    );
                }
            }
        });
    }
}
