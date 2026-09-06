//! Pedestrian crossing scenario model
//!
//! Simplified model: A pedestrian crosses perpendicular to the ego vehicle's path.
//! The pedestrian starts at an initial lateral position and moves to cross the road.
//! No complex sidewalk or timing constraints - just basic crossing behavior.

use crate::dsl::types::ScenarioSpec;
use crate::error::{Result, ScenarioGenError};
use crate::ltl::formula::{LTLFormula, Proposition};
use crate::scenarios::ScenarioModel;
use crate::solver::encoder_utils::real_from_f64;

/// Divisors for the pedestrian safety box asserted by `generate_safety`
/// (`threshold_x = min_distance / LONGITUDINAL`, `threshold_y = min_distance
/// / LATERAL`) and reproduced by `compute_validation_metrics`
/// (`solver/encoder.rs`, the `pedestrian_pair` branch) to measure exactly
/// what got asserted. SW-34: these used to be independent literals in the
/// two files — the SW-27/SW-30 shape, one edit from disagreeing — hoisted
/// here so there is exactly one copy of each number.
///
/// Deliberately divisors, not multiplied ratios: `x / 2.0` and `x * 0.5` are
/// bit-identical in IEEE-754, but `x / 1.5` and `x * (1.0 / 1.5)` are not —
/// `1.0 / 1.5` is not exactly representable. Keeping the division form
/// preserves the exact arithmetic both files already did; a multiplication
/// form would have moved the pedestrian snapshot in the last bit for no
/// reason, which is the whole story behind SW-27.
pub(crate) const PEDESTRIAN_BOX_LONGITUDINAL_DIVISOR: f64 = 2.0;
pub(crate) const PEDESTRIAN_BOX_LATERAL_DIVISOR: f64 = 1.5;

/// Pedestrian crossing scenario model
pub(crate) struct PedestrianCrossingModel;

impl ScenarioModel for PedestrianCrossingModel {
    fn validate(&self, spec: &ScenarioSpec) -> Result<()> {
        use crate::dsl::types::ActorRole;

        // Validate exactly 2 actors (1 ego vehicle, 1 pedestrian)
        if spec.actors.len() != 2 {
            return Err(ScenarioGenError::InvalidSpec(format!(
                "Pedestrian crossing requires exactly 2 actors, found {}",
                spec.actors.len()
            )));
        }

        // Validate roles
        let _ego = spec.ego().map_err(ScenarioGenError::InvalidSpec)?;
        let pedestrian = &spec.npcs()[0];

        if pedestrian.role != ActorRole::Pedestrian {
            return Err(ScenarioGenError::InvalidSpec(format!(
                "Second actor must be pedestrian, found {:?}",
                pedestrian.role
            )));
        }

        // Validate direction field exists and is valid
        let direction = pedestrian
            .behavior
            .get("direction")
            .and_then(|v| v.as_str())
            .ok_or_else(|| {
                ScenarioGenError::InvalidSpec(
                    "Pedestrian missing 'direction' in behavior".to_string(),
                )
            })?;

        match direction {
            "left_to_right" | "right_to_left" => Ok(()),
            _ => Err(ScenarioGenError::InvalidSpec(format!(
                "Invalid direction '{}': must be 'left_to_right' or 'right_to_left'",
                direction
            ))),
        }
    }

    fn generate_safety(&self, spec: &ScenarioSpec) -> Result<LTLFormula> {
        use crate::dsl::types::ActorRole;

        let ego = spec.ego().map_err(ScenarioGenError::InvalidSpec)?;
        let npcs = spec.npcs();
        let pedestrian = npcs
            .iter()
            .find(|a| a.role == ActorRole::Pedestrian)
            .ok_or_else(|| ScenarioGenError::InvalidSpec("No pedestrian found".to_string()))?;

        let mut constraints = Vec::new();

        // Rectangular safety box (simplest linear constraint, very fast Z3 solving)
        // For perpendicular crossing: lateral distance is more critical than longitudinal
        // Using threshold/1.5 gives conservative safety (~1.3m for 2m threshold)
        //
        // SW-33. Used to be gated on `Enforce` only, so `Violate` (and
        // `Ignore`, though that already meant "assert nothing") asserted
        // nothing at all — a `violate`d pedestrian `min_distance` silently
        // produced an ordinary scenario, the same failure SW-32 fixed for
        // `min_lateral_distance` a few lines below. Routed through
        // `push_constraint` like every other constraint in this file and in
        // `generate_default_safety`. The negation is meaningful: `atom` is
        // the disjunction `|dx| > tx OR |dy| > ty`, so `atom.negate()` is the
        // conjunction `|dx| <= tx AND |dy| <= ty` — an ordinary box interior,
        // satisfiable, not vacuous.
        super::push_constraint(
            &mut constraints,
            spec.constraint_modes.min_distance(),
            LTLFormula::Atom(Proposition::RectangularDistanceGT {
                actor1: ego.id.clone(),
                actor2: pedestrian.id.clone(),
                threshold_x: spec.min_distance / PEDESTRIAN_BOX_LONGITUDINAL_DIVISOR, // Longitudinal: half the threshold
                threshold_y: spec.min_distance / PEDESTRIAN_BOX_LATERAL_DIVISOR, // Lateral: slightly more conservative
            }),
            super::AtomPolarity::Positive,
        );

        // SW-32. `min_lateral_distance` used to be parsed, documented and
        // validated, then silently dropped here: this override replaced
        // `generate_default_safety`'s per-pair loop wholesale and never once
        // read the field, so an `enforce`d `min_lateral_distance` changed
        // nothing about the encoding. Lowered the same way
        // `generate_default_safety` (`scenarios/mod.rs`) does for every other
        // scenario type — `push_constraint` with `AtomPolarity::Positive` —
        // so `Enforce`/`Violate`/`Ignore` all mean what they mean everywhere
        // else, and `compute_validation_metrics` (which checks this field
        // generically, for any scenario type) actually has something to
        // measure.
        //
        // This is a second, independent lateral constraint alongside the
        // box's own `threshold_y = min_distance / 1.5` above — not a
        // replacement for it, and not in tension with it. The box asserts
        // `|dx| > threshold_x OR |dy| > threshold_y`: an actor pair may
        // satisfy it on the *longitudinal* branch alone, with `|dy|`
        // arbitrarily small. `LateralDistanceGT` instead asserts `|dy| >=
        // min_lateral_distance` unconditionally, so it closes exactly the
        // gap the box's disjunction leaves open on the lateral axis — it
        // does not fight the box, it tightens the one case the box does not
        // cover. A `min_lateral_distance` looser than `threshold_y` adds
        // nothing new (the box's own lateral branch already implies it
        // whenever that branch is the one satisfied, and the constraint is
        // trivially satisfiable whenever the longitudinal branch is used
        // instead); one tighter than `threshold_y` is a real additional
        // restriction, verified satisfiable end-to-end in
        // `pedestrian_lateral_distance_test.rs`.
        if let Some(min_lat_dist) = spec.min_lateral_distance {
            super::push_constraint(
                &mut constraints,
                spec.constraint_modes.min_lateral_distance(),
                LTLFormula::Atom(Proposition::LateralDistanceGT {
                    actor1: ego.id.clone(),
                    actor2: pedestrian.id.clone(),
                    distance: min_lat_dist,
                }),
                super::AtomPolarity::Positive,
            );
        }

        // Pedestrian-specific TTC (perpendicular crossing).
        //
        // SW-33. Same defect as the box above: gated on `Enforce` only, so
        // `Violate` asserted nothing. `PedestrianTTCGT` lowers to the guarded
        // implication `ped_on_road ∧ approaching ⟹ ttc_safe`
        // (`encoder.rs`), whose negation is
        // `ped_on_road ∧ approaching ∧ ¬ttc_safe` — it requires the
        // antecedent to actually hold, not just any state, so this is the
        // one of the two negations in this issue worth checking rather than
        // assuming. Verified satisfiable end-to-end (not vacuous, not
        // UNSAT): with SW-35 landed first, `ttc_safe` is non-strict, so
        // `¬ttc_safe` is the strict `distance < ttc * ego_vx` — an ordinary
        // close call, not a boundary condition — and a `violate`d
        // `pedestrian_crossing.yaml` produces a real breach (see the SW-33
        // report/tests for the numbers).
        super::push_constraint(
            &mut constraints,
            spec.constraint_modes.min_ttc(),
            LTLFormula::Atom(Proposition::PedestrianTTCGT {
                ego: ego.id.clone(),
                pedestrian: pedestrian.id.clone(),
                ttc: spec.min_ttc,
            }),
            super::AtomPolarity::Positive,
        );

        // SW-43. The crossing must be a *conflict*, or the bound above
        // constrains nothing.
        //
        // `PedestrianTTCGT` is a guarded implication — "whenever the pedestrian
        // is on the road with the ego behind it and closing, the TTC exceeds
        // `min_ttc`" — and the pedestrian's `py` is a solver variable, not an
        // input. `generate_ltl` asks only that the pedestrian reach the far
        // sidewalk *eventually*, so Z3 could satisfy `G(PedestrianTTCGT(..))`
        // by keeping the pedestrian clear of the road at exactly the steps
        // where the ego is bearing down on it and crossing once the ego was
        // past. Measured on an `enforce`d two-lane spec before this constraint
        // existed: the pedestrian starts at the lane centre (`py[0]` is pinned
        // there by `encode_pedestrian_initial_state`, so the guard is true at
        // `t=0` whatever else happens), steps *off* the road by `t=3` while the
        // ego is still 22 m away, waits on the near kerb through the ego's
        // whole approach, and crosses over steps 19-30 with the ego already
        // past it. The `enforce`d 2 s bound was evaluated only at the three
        // opening steps, 33 m out — an `enforce` that no crossing could ever
        // fail. This is SW-22's defect one scenario type over; see
        // `scenarios::cut_in_conflict` for the precedent and the cost argument.
        //
        // **The shape, and why not the obvious one.** Not `F(guard)`: SW-12
        // built that existential for the cut-in and measured it at >500 s for
        // five scenarios, because a disjunction over the horizon asks Z3 to
        // *search* for the instant. This is `G(antecedent → conflict)` with an
        // antecedent the template already forces — `generate_ltl` asserts
        // `F(CrossingRoad(ped))`, and SW-42's `Invariant::Liveness` confirms the
        // shipped trajectory really does cross — so every conjunct is an
        // implication Z3 propagates. Unlike the cut-in's `same_lane`, the
        // consequent here contains no disjunction at all, which is the half of
        // SW-22's 103 s → 14.1 s measurement that did the damage.
        //
        // **Why the consequent is the TTC's own guard atom** rather than a
        // hand-written "and the ego is approaching": a conflict formula weaker
        // than the guard would force something the TTC is not conditioned on
        // and prove nothing. `PedestrianTTCGuard` and `PedestrianTTCGT`'s
        // antecedent are lowered by one function (`encode_pedestrian_ttc_guard`),
        // so they cannot drift.
        //
        // **`Enforce` only, deliberately.** Under `Violate`, `push_constraint`
        // already asserts `F(¬(guard → ttc_safe))` = `F(guard ∧ ¬ttc_safe)`,
        // which forces the conflict on its own — adding this would be
        // redundant and could only make a `violate` spec harder to solve.
        // Under `Ignore` the user has said the TTC is not being tested, and
        // there is no vacuous `enforce` to protect. So the constraint attaches
        // to the mode whose promise it restores, and the three pedestrian
        // examples in `examples/` — all `min_ttc: ignore` — are bit-identical
        // across this change.
        //
        // A pedestrian who waits for a car to pass is correct road behaviour,
        // not a defect; what was wrong was that *nothing ever required the
        // conflict to exist*. This says: if you `enforce` a pedestrian TTC,
        // the crossing this scenario generates is one where that TTC is
        // actually at stake.
        if spec.constraint_modes.min_ttc() == crate::dsl::types::ConstraintMode::Enforce {
            constraints.push(
                LTLFormula::Atom(Proposition::CrossingRoad {
                    actor: pedestrian.id.clone(),
                })
                .implies(LTLFormula::Atom(Proposition::PedestrianTTCGuard {
                    ego: ego.id.clone(),
                    pedestrian: pedestrian.id.clone(),
                }))
                .always(),
            );
        }

        Ok(LTLFormula::conjunction(constraints))
    }

    fn generate_ltl(&self, spec: &ScenarioSpec) -> Result<LTLFormula> {
        use crate::dsl::types::ActorRole;

        let ego = spec.ego().map_err(ScenarioGenError::InvalidSpec)?;
        let ego_id = ego.id.as_str();

        // Ego stays in its lane
        let ego_in_lane = LTLFormula::Atom(Proposition::InLane {
            actor: ego_id.to_string(),
            lane: ego.lane,
        });

        // Get pedestrian
        let npcs = spec.npcs();
        let pedestrian = npcs
            .iter()
            .find(|a| a.role == ActorRole::Pedestrian)
            .ok_or_else(|| {
                ScenarioGenError::InvalidSpec("No pedestrian found in spec".to_string())
            })?;
        let ped_id = &pedestrian.id;

        // Determine crossing direction from behavior field
        let direction = pedestrian
            .behavior
            .get("direction")
            .and_then(|v| v.as_str())
            .ok_or_else(|| {
                ScenarioGenError::InvalidSpec(
                    "Pedestrian missing 'direction' in behavior".to_string(),
                )
            })?;

        let opposite_side = match direction {
            "left_to_right" => "right",
            "right_to_left" => "left",
            _ => {
                return Err(ScenarioGenError::InvalidSpec(format!(
                    "Invalid direction '{}': must be 'left_to_right' or 'right_to_left'",
                    direction
                )))
            }
        };

        // Multi-stage crossing using sequential implications
        // Stage 1: Eventually starts crossing (enters road)
        let crossing_road = LTLFormula::Atom(Proposition::CrossingRoad {
            actor: ped_id.clone(),
        });

        // Stage 2: Eventually reaches opposite sidewalk
        let on_opposite_sidewalk = LTLFormula::Atom(Proposition::OnSidewalk {
            actor: ped_id.clone(),
            side: opposite_side.to_string(),
        });

        // Combine: Eventually cross road AND eventually reach opposite side
        // This allows: start on initial → move to road → move to opposite
        let enters_road = crossing_road.clone().eventually();
        let reaches_opposite = on_opposite_sidewalk.eventually();

        let full_crossing = enters_road.and(reaches_opposite);

        // Combine ego constraint with pedestrian crossing behavior
        Ok(ego_in_lane.and(full_crossing))
    }

    fn add_z3_constraints(
        &self,
        spec: &ScenarioSpec,
        encoder: &dyn crate::solver::EncoderAccessor,
        backend: &dyn crate::solver::Z3Backend,
        horizon: usize,
    ) -> Result<()> {
        use crate::dsl::ActorRole;
        use z3::ast::Int;

        // Get pedestrian actor
        let npcs = spec.npcs();
        let pedestrian = npcs
            .iter()
            .find(|a| a.role == ActorRole::Pedestrian)
            .ok_or_else(|| {
                ScenarioGenError::InvalidSpec("No pedestrian actor found".to_string())
            })?;

        let pedestrian_id = &pedestrian.id;

        // Fix pedestrian lane to 0 throughout the scenario
        // The lane field has no semantic meaning for pedestrians - only lateral
        // position (py) matters for crossing detection. Fixing it to 0 prevents
        // Z3 from generating spurious lane values (e.g., 2, 3, 4).
        let zero_lane = Int::from_i64(0_i64);
        for t in 0..=horizon {
            let lane_t = encoder.get_lane_var(pedestrian_id, t);
            backend.assert(&lane_t.eq(&zero_lane));
        }

        // SW-44. The safety box is *sampled*, so the ego can drive through it
        // between two steps.
        //
        // `RectangularDistanceGT` asserts `|dx| >= threshold_x OR |dy| >=
        // threshold_y` at each discrete step, and `threshold_x` is
        // `min_distance / PEDESTRIAN_BOX_LONGITUDINAL_DIVISOR` — a half-box
        // 1 m long for the usual `min_distance: 2.0`. An ego at 20 m/s covers
        // 10 m in a `time_step: 0.5`, so it can sit at `dx = -3.0` at one step
        // and `dx = +7.4` at the next, satisfy the box at every sampled step,
        // and have driven straight through the pedestrian in between. Measured
        // before this constraint existed, on a single-lane spec with
        // `min_distance: 6.0` where lateral clearance is geometrically
        // impossible: every step reported `boxOK`, `all_constraints_satisfied`
        // came back `true`, and `dx` went `-3.000 → +7.375` across one step
        // with `|dy| = 1.875` against a `threshold_y` of 4.0. The box is
        // smaller than one step of travel at any realistic speed, so this is
        // the normal case rather than a corner one.
        //
        // The guard: **the pair may not swap longitudinal order between two
        // steps unless it is laterally clear at both of them, on the same
        // side.** `dx` is continuous, so a sign change means the ego passed
        // through `dx = 0` somewhere inside the step, where the box demands
        // `|dy| >= threshold_y`; requiring that clearance at both endpoints,
        // with a common sign, is the strongest linear statement available
        // about an instant that is not a variable.
        //
        // **Cost.** One assertion per step per scenario, over the one
        // ego-pedestrian pair a `pedestrian_crossing` spec has — the shape
        // SW-22 established as affordable (an implication Z3 propagates), not
        // the `F(⋁ over the horizon)` shape SW-12 measured at >500 s. It is a
        // disjunction, so Z3 case-splits, but over four fixed alternatives at
        // one step rather than over the horizon. Measured: the three
        // `examples/pedestrian_*.yaml` are `min_distance: ignore` and so are
        // untouched; a constructed `enforce` spec solves in the same tenths of
        // a second it did before.
        //
        // **Still QF_LRA.** Four comparisons between existing variables and a
        // disjunction of conjunctions of them. No product of two variables,
        // and in particular no attempt at an exact swept-volume test against
        // the trapezoidal position update, which is quadratic in time and
        // would leave the fragment.
        //
        // **`Enforce` only**, matching the box it protects: under `Violate`
        // `push_constraint` asks for `F(|dx| < tx AND |dy| < ty)` and this
        // would fight it; under `Ignore` no box is asserted, so there is
        // nothing to tunnel through.
        //
        // **What it does not cover, stated rather than implied.** `dx` is
        // quadratic in time within a step, so it can in principle dip below
        // `threshold_x` and recover without changing sign — the relative
        // longitudinal velocity would have to reverse inside one step. And a
        // `dy` that dips toward zero and returns within a single step would
        // pass the both-endpoints test. Both need an acceleration reversal
        // inside `time_step`; neither is expressible as a linear condition on
        // the sampled variables, which is the boundary this fix stops at.
        // `compute_validation_metrics` re-derives exactly this guard on the
        // shipped trajectory (`scenario/metrics.rs`, the `pedestrian_pair`
        // branch), so the two agree on which crossings count.
        if spec.constraint_modes.min_distance() == crate::dsl::types::ConstraintMode::Enforce {
            let ego = spec.ego().map_err(ScenarioGenError::InvalidSpec)?;
            let threshold_y = spec.min_distance / PEDESTRIAN_BOX_LATERAL_DIVISOR;
            let ty = real_from_f64(threshold_y);
            let neg_ty = real_from_f64(-threshold_y);
            let zero = real_from_f64(0.0);

            for t in 0..horizon {
                let dx_t = encoder.get_longitudinal_pos(&ego.id, t)
                    - encoder.get_longitudinal_pos(pedestrian_id, t);
                let dx_next = encoder.get_longitudinal_pos(&ego.id, t + 1)
                    - encoder.get_longitudinal_pos(pedestrian_id, t + 1);
                let dy_t =
                    encoder.get_lateral_pos(&ego.id, t) - encoder.get_lateral_pos(pedestrian_id, t);
                let dy_next = encoder.get_lateral_pos(&ego.id, t + 1)
                    - encoder.get_lateral_pos(pedestrian_id, t + 1);

                // The ego does not pass the pedestrian between t and t+1.
                // `dx == 0` at a sampled step satisfies both halves, and is
                // already covered by the box at that step.
                let no_pass = z3::ast::Bool::or(&[
                    &z3::ast::Bool::and(&[&dx_t.ge(&zero), &dx_next.ge(&zero)]),
                    &z3::ast::Bool::and(&[&dx_t.le(&zero), &dx_next.le(&zero)]),
                ]);

                // ... or it is clear of the lateral half-box at both ends, on
                // the same side of the pedestrian.
                let laterally_clear = z3::ast::Bool::or(&[
                    &z3::ast::Bool::and(&[&dy_t.ge(&ty), &dy_next.ge(&ty)]),
                    &z3::ast::Bool::and(&[&dy_t.le(&neg_ty), &dy_next.le(&neg_ty)]),
                ]);

                backend.assert(&z3::ast::Bool::or(&[&no_pass, &laterally_clear]));
            }
        }

        // Check if pedestrian has "hesitate" walking mode

        if let Some(walking_mode) = pedestrian.behavior.get("walking_mode") {
            if walking_mode == "hesitate" {
                let pedestrian_id = &pedestrian.id;

                // For hesitate mode: Force pedestrian to slow down significantly
                // at some point during the middle of the scenario (e.g., 40%-60% through)
                let start_hesitate = (horizon as f64 * 0.4) as usize;
                let end_hesitate = (horizon as f64 * 0.6) as usize;

                // At least one time step in this range should have very low speed
                // Create a disjunction: at least one time step has speed < 0.2 m/s
                // Linear box constraint to avoid NRA (quadratic) overhead
                let slow_threshold = 0.2; // m/s
                let threshold_real = real_from_f64(slow_threshold);
                let neg_threshold_real = real_from_f64(-slow_threshold);

                let mut slow_constraints = vec![];
                for t in start_hesitate..end_hesitate {
                    let vx_t = encoder.get_longitudinal_vel(pedestrian_id, t);
                    let vy_t = encoder.get_lateral_vel(pedestrian_id, t);
                    // |vx| < threshold AND |vy| < threshold
                    let vx_lt = vx_t.lt(&threshold_real);
                    let vx_gt = vx_t.gt(&neg_threshold_real);
                    let vy_lt = vy_t.lt(&threshold_real);
                    let vy_gt = vy_t.gt(&neg_threshold_real);
                    slow_constraints.push(z3::ast::Bool::and(&[&vx_lt, &vx_gt, &vy_lt, &vy_gt]));
                }

                // At least one of these must be true (OR them together)
                if !slow_constraints.is_empty() {
                    let slow_constraint_refs: Vec<_> = slow_constraints.iter().collect();
                    let hesitate_constraint = z3::ast::Bool::or(&slow_constraint_refs);
                    backend.assert(&hesitate_constraint);
                }
            }
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dsl::types::{
        ActorRole, ActorSpec, ConstraintModes, OptimizationTarget, ValueOrRange,
    };
    use std::collections::HashMap;

    fn create_test_spec() -> ScenarioSpec {
        let ego_behavior = HashMap::new();
        let mut pedestrian_behavior = HashMap::new();
        pedestrian_behavior.insert("direction".to_string(), serde_json::json!("left_to_right"));

        ScenarioSpec {
            scenario_type: crate::dsl::types::ScenarioType::PedestrianCrossing,
            time_step: 0.5,
            duration: 10.0,
            actors: vec![
                ActorSpec {
                    id: "ego".to_string(),
                    role: ActorRole::Ego,
                    lane: 0,
                    position: ValueOrRange::Value(0.0),
                    speed: ValueOrRange::Value(10.0),
                    acceleration: ValueOrRange::Range([-3.0, 2.0]),
                    direction: 1,
                    behavior: ego_behavior,
                    lane_changes: vec![],
                    bicycle_params: None,
                },
                ActorSpec {
                    id: "pedestrian".to_string(),
                    role: ActorRole::Pedestrian,
                    lane: 0,
                    position: ValueOrRange::Value(50.0),
                    speed: ValueOrRange::Range([0.8, 1.5]),
                    acceleration: ValueOrRange::Range([-1.0, 1.0]),
                    direction: 1,
                    behavior: pedestrian_behavior,
                    lane_changes: vec![],
                    bicycle_params: None,
                },
            ],
            min_ttc: 2.0,
            min_distance: 2.0,
            road: None,
            lane_width: 3.5,
            num_scenarios: 1,
            constraint_modes: ConstraintModes::default(),
            optimization_target: OptimizationTarget::None,
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

    #[test]
    fn test_pedestrian_validate_success() {
        let model = PedestrianCrossingModel;
        let spec = create_test_spec();
        assert!(model.validate(&spec).is_ok());
    }

    #[test]
    fn test_pedestrian_validate_wrong_role() {
        let model = PedestrianCrossingModel;
        let mut spec = create_test_spec();
        spec.actors[1].role = ActorRole::Npc; // Change pedestrian to NPC
        assert!(model.validate(&spec).is_err());
    }

    #[test]
    fn test_pedestrian_generate_ltl() {
        let model = PedestrianCrossingModel;
        let spec = create_test_spec();
        let formula = model.generate_ltl(&spec);
        assert!(formula.is_ok());

        let formula_str = format!("{}", formula.unwrap());
        assert!(formula_str.contains("InLane"));
    }

    /// SW-33. Both `RectangularDistanceGT` (`min_distance`) and
    /// `PedestrianTTCGT` (`min_ttc`) used to be gated on
    /// `ConstraintMode::Enforce` only, so `Violate` fell through and asserted
    /// nothing — `generate_safety` returned `LTLFormula::True` for either
    /// field's contribution. Routed through `push_constraint` like every
    /// other constraint in this module, `Violate` now asserts
    /// `F(¬(atom))` (`AtomPolarity::Positive`, `ConstraintMode::Violate`).
    /// Against the pre-SW-33 code this assertion fails: the formula string
    /// contains no `RectangularDistanceGT`/`PedestrianTTCGT` at all, since
    /// nothing was pushed.
    #[test]
    fn test_pedestrian_min_distance_violate_asserts_the_negation() {
        use crate::dsl::types::{Constraint, ConstraintMode, ConstraintModes};

        let model = PedestrianCrossingModel;
        let mut spec = create_test_spec();
        spec.constraint_modes = ConstraintModes::Detailed {
            min_ttc: ConstraintMode::Ignore,
            min_distance: ConstraintMode::Violate,
            max_acceleration: ConstraintMode::Ignore,
            max_velocity: ConstraintMode::Ignore,
            min_velocity: ConstraintMode::Ignore,
            min_lateral_distance: ConstraintMode::Ignore,
            max_relative_velocity: ConstraintMode::Ignore,
        };
        assert_eq!(
            spec.constraint_modes.mode_for(Constraint::MinDistance),
            ConstraintMode::Violate
        );

        let formula = model.generate_safety(&spec).unwrap();
        let formula_str = format!("{formula}");

        assert!(
            formula_str.contains("RectangularDistanceGT"),
            "Violate must still assert something about RectangularDistanceGT, got {formula_str}"
        );
        assert!(
            formula_str.contains("F(¬("),
            "Violate must assert the atom's negation, eventually — got {formula_str}"
        );
    }

    /// SW-43. An `enforce`d pedestrian `min_ttc` must come with the conflict
    /// that makes it evaluable: `G(CrossingRoad(ped) → PedestrianTTCGuard(..))`,
    /// where the guard atom is `PedestrianTTCGT`'s own antecedent. Without it,
    /// `G(PedestrianTTCGT(..))` is satisfiable with the antecedent false at
    /// every step that matters — a pedestrian who waits on the kerb for the ego
    /// to pass and crosses behind it — so the bound cannot fail. Against the
    /// pre-SW-43 code this assertion fails: the formula string contains no
    /// `PedestrianTTCGuard` at all. `tests/pedestrian_conflict_test.rs` is the
    /// end-to-end half, on the trajectory rather than on the formula.
    #[test]
    fn test_pedestrian_min_ttc_enforce_also_asserts_the_conflict() {
        use crate::dsl::types::{Constraint, ConstraintMode};

        let model = PedestrianCrossingModel;
        let spec = create_test_spec();
        assert_eq!(
            spec.constraint_modes.mode_for(Constraint::MinTtc),
            ConstraintMode::Enforce,
            "this test needs the default modes to enforce min_ttc"
        );

        let formula_str = format!("{}", model.generate_safety(&spec).unwrap());
        assert!(
            formula_str.contains("PedestrianTTCGuard"),
            "Enforce must also force the crossing to be a conflict, got {formula_str}"
        );
        assert!(
            formula_str.contains("G((CrossingRoad"),
            "the conflict must be a G(antecedent → guard), not an existential — got \
             {formula_str}"
        );
    }

    /// The conflict attaches to `Enforce` only. `Violate` already forces it —
    /// `push_constraint` asserts `F(¬(guard → ttc_safe))`, i.e.
    /// `F(guard ∧ ¬ttc_safe)` — and under `Ignore` the spec has said the TTC is
    /// not under test, so there is no vacuous `enforce` to protect and nothing
    /// to justify constraining the trajectory. This is also why the three
    /// `examples/pedestrian_*.yaml`, all `min_ttc: ignore`, are unchanged by
    /// SW-43.
    #[test]
    fn test_pedestrian_min_ttc_violate_and_ignore_do_not_add_the_conflict() {
        use crate::dsl::types::{ConstraintMode, ConstraintModes};

        let model = PedestrianCrossingModel;
        for mode in [ConstraintMode::Violate, ConstraintMode::Ignore] {
            let mut spec = create_test_spec();
            spec.constraint_modes = ConstraintModes::Detailed {
                min_ttc: mode,
                min_distance: ConstraintMode::Ignore,
                max_acceleration: ConstraintMode::Ignore,
                max_velocity: ConstraintMode::Ignore,
                min_velocity: ConstraintMode::Ignore,
                min_lateral_distance: ConstraintMode::Ignore,
                max_relative_velocity: ConstraintMode::Ignore,
            };
            let formula_str = format!("{}", model.generate_safety(&spec).unwrap());
            assert!(
                !formula_str.contains("PedestrianTTCGuard"),
                "{mode:?} must not carry the Enforce-only conflict, got {formula_str}"
            );
        }
    }

    #[test]
    fn test_pedestrian_min_ttc_violate_asserts_the_negation() {
        use crate::dsl::types::{Constraint, ConstraintMode, ConstraintModes};

        let model = PedestrianCrossingModel;
        let mut spec = create_test_spec();
        spec.constraint_modes = ConstraintModes::Detailed {
            min_ttc: ConstraintMode::Violate,
            min_distance: ConstraintMode::Ignore,
            max_acceleration: ConstraintMode::Ignore,
            max_velocity: ConstraintMode::Ignore,
            min_velocity: ConstraintMode::Ignore,
            min_lateral_distance: ConstraintMode::Ignore,
            max_relative_velocity: ConstraintMode::Ignore,
        };
        assert_eq!(
            spec.constraint_modes.mode_for(Constraint::MinTtc),
            ConstraintMode::Violate
        );

        let formula = model.generate_safety(&spec).unwrap();
        let formula_str = format!("{formula}");

        assert!(
            formula_str.contains("PedestrianTTCGT"),
            "Violate must still assert something about PedestrianTTCGT, got {formula_str}"
        );
        assert!(
            formula_str.contains("F(¬("),
            "Violate must assert the atom's negation, eventually — got {formula_str}"
        );
    }
}
