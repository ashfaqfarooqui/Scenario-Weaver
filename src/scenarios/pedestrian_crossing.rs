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
/// what got asserted. Keeping them as independent literals in the two files
/// would be one edit from disagreeing, so they are hoisted here so there is
/// exactly one copy of each number.
///
/// Deliberately divisors, not multiplied ratios: `x / 2.0` and `x * 0.5` are
/// bit-identical in IEEE-754, but `x / 1.5` and `x * (1.0 / 1.5)` are not —
/// `1.0 / 1.5` is not exactly representable. Keeping the division form
/// preserves the exact arithmetic both files already do; a multiplication
/// form would move the pedestrian snapshot in the last bit for no reason.
pub(crate) const PEDESTRIAN_BOX_LONGITUDINAL_DIVISOR: f64 = 2.0;
pub(crate) const PEDESTRIAN_BOX_LATERAL_DIVISOR: f64 = 1.5;

/// The longitudinal window inside which lateral separation between the ego and
/// a crossing pedestrian is a *safety* property, in metres.
///
/// `min_lateral_distance` used to be lowered here as an **unguarded**
/// `LateralDistanceGT` — `|py_ego − py_ped| >= d` at every step. For a
/// perpendicular crossing that is self-contradictory: the pedestrian's path
/// runs through `py_ego`, so any continuous crossing has `|dy| = 0` at some
/// instant, and the only reason the constraint was ever satisfiable is that the
/// trajectory is *sampled*. The largest `d` that could hold was
/// `(v_ped × time_step) / 2` — the half-hop the pedestrian takes across the
/// forbidden band between two samples. Measured at HEAD before this changed:
/// `pedestrian_crossing.yaml` (1.5 m/s, `time_step: 0.3`) was SAT at
/// `d = 0.22` and UNSAT at `d = 0.25`; halving `time_step` to `0.15` made
/// `0.22` UNSAT and only `0.11` SAT. A safety threshold whose satisfiability is
/// a function of the discretisation is not measuring safety, and every value a
/// user would plausibly author — `2.0`, the same order as the `min_distance:
/// 2.0` sitting beside it in the example — was UNSAT by construction.
///
/// Lateral separation from a crossing pedestrian only means anything while the
/// two are *longitudinally* close. Outside that window the pedestrian is
/// walking across a road the ego is nowhere near, and demanding clearance there
/// is a statement about geometry the scenario cannot honour.
///
/// **Where the number comes from.** `min_ttc × the ego's declared top speed` is
/// the distance the ego can cover in its own stated time-to-collision budget,
/// so "longitudinally close" here means exactly "less than `min_ttc` away from
/// the pedestrian's crossing point at the fastest the spec allows". That is the
/// same notion of relevance `PedestrianTTCGT` already uses
/// (`distance >= ttc · vx_ego`), with the *declared* speed bound substituted
/// for the solver's `vx` so the window is a constant: a product of two
/// variables would leave QF_LRA and break `Optimize`.
///
/// **Floored at the safety box's own `threshold_x`** so the two constraints can
/// never disagree about what "longitudinally close" means, and so a spec with
/// `min_ttc: 0` still gets a window rather than a silently dropped constraint.
///
/// **Why not simply reuse `threshold_x`.** Because then the guarded constraint
/// would be vacuous whenever the box is enforced: the box already gives
/// `|dx| >= threshold_x ∨ |dy| >= threshold_y`, so with `W = threshold_x` every
/// state that satisfies the box on its longitudinal branch satisfies the
/// guarded lateral one too, and the only states left are ones the box was
/// already policing. `W` has to be strictly wider than `threshold_x` to add
/// anything — that is the SW-22 "live but vacuous" failure mode. On the shipped
/// examples it is: 24.0 m against a `threshold_x` of 1.0 m for
/// `pedestrian_crossing.yaml`, 18.0 m against 0.75 m for
/// `pedestrian_wide_road.yaml`.
///
/// `compute_validation_metrics` (`scenario/metrics.rs`) calls this same
/// function rather than re-deriving the arithmetic, for the reason the box
/// divisors above are shared: two copies of a number are one edit from
/// disagreeing.
pub(crate) fn lateral_relevance_window(spec: &ScenarioSpec) -> Result<f64> {
    let ego = spec.ego().map_err(ScenarioGenError::InvalidSpec)?;
    let reachable_in_ttc = spec.min_ttc * ego.speed.max();
    let box_threshold_x = spec.min_distance / PEDESTRIAN_BOX_LONGITUDINAL_DIVISOR;
    Ok(reachable_in_ttc.max(box_threshold_x))
}

/// How many steps at the end of the horizon the pedestrian must be
/// *settled* on the far kerb for (the final step plus a short tail before it).
///
/// The crossing goal `F(OnSidewalk(far))` only pins the far kerb at *some* step
/// and leaves it free everywhere else, so the pedestrian reached the far kerb
/// for a single step and walked back into the road (ending mid-road). Requiring
/// the last few steps to lie on the far kerb — past the kerb centre, not merely
/// a millimetre onto the sidewalk — makes "arrives and stays" a hard bound.
///
/// A fixed suffix is deliberate: it is a per-step containment Z3 *propagates*,
/// not the `F(G(...))` / `∃k.∀t≥k` shape that makes Z3 *search* over where the
/// settled tail begins. Two steps is enough because the pedestrian's speed is
/// capped (`encoders::pedestrian`): it cannot hover off the kerb and jump onto
/// it within one step, so pinning the final steps drags the approach with it.
const SETTLE_TAIL_STEPS: usize = 2;

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
        // Gating this on `Enforce` only would mean `Violate` (and `Ignore`,
        // though that already meant "assert nothing") asserted nothing at
        // all — a `violate`d pedestrian `min_distance` would silently
        // produce an ordinary scenario, the same failure that has to be
        // avoided for `min_lateral_distance` a few lines below. Routed
        // through `push_constraint` like every other constraint in this file
        // and in `generate_default_safety`. The negation is meaningful: `atom` is
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

        // `min_lateral_distance`, **guarded on longitudinal relevance**.
        //
        // The field is parsed, documented and validated, so it must not be
        // silently dropped here: this override replaces
        // `generate_default_safety`'s per-pair loop wholesale, and if it never
        // read the field an `enforce`d `min_lateral_distance` would change
        // nothing about the encoding. It still goes through the same
        // `push_constraint` with `AtomPolarity::Positive`, so
        // `Enforce`/`Violate`/`Ignore` mean here what they mean everywhere
        // else. What changed is the *atom*.
        //
        // It used to be an unguarded `LateralDistanceGT` — `|dy| >= d` at
        // every step. See `lateral_relevance_window` above for why that was
        // satisfiable only as an artifact of the sampling rate. The property
        // the field is actually trying to express is
        //
        //     G( |dx| <= W  =>  |dy| >= min_lateral_distance )
        //
        // and that implication is, term for term, a rectangular exclusion box
        // `W` long and `min_lateral_distance` tall:
        //
        //     |dx| < W  =>  |dy| >= d      ==      |dx| >= W  OR  |dy| >= d
        //
        // which is exactly what `RectangularDistanceGT { threshold_x: W,
        // threshold_y: d }` lowers to (`ltl/encode.rs`). So the guard needs no
        // new proposition variant and no new lowering — it is the box shape
        // the very same function already asserts for `min_distance`, sized by
        // the lateral field instead. Reusing the atom also inherits SW-33's
        // boundary fix (both comparisons non-strict) for free.
        //
        // **This does not weaken `min_distance`'s box, and is not weakened by
        // it.** The two are independent boxes over the same pair: the
        // `min_distance` one is short and squat (`threshold_x = md/2`,
        // `threshold_y = md/1.5`), this one is long and thin (`W` ≫
        // `threshold_x`, `threshold_y = d`). Their conjunction is the union of
        // the two exclusion regions, and neither implies the other as long as
        // `W > min_distance / PEDESTRIAN_BOX_LONGITUDINAL_DIVISOR` — which
        // `lateral_relevance_window` guarantees by flooring at exactly that
        // value. A state with `|dx| = threshold_x` and `|dy| = 0` satisfies
        // the `min_distance` box and violates this one, so this constraint is
        // not vacuous in the presence of the box.
        //
        // **`Violate` is `A AND NOT B`, and that is the point.** `negate()`
        // on this atom gives `|dx| < W AND |dy| < d`: the antecedent must
        // *actually hold*, so the solver cannot satisfy a `violate`d
        // `min_lateral_distance` by parking the pedestrian on the kerb while
        // the ego is 60 m away. That is a strictly stronger obligation than
        // the unguarded form's `|dy| < d`, and it is the near-miss the field
        // is for. Verified end-to-end (not assumed) in
        // `tests/pedestrian_lateral_distance_test.rs`.
        //
        // **Still QF_LRA.** Four comparisons between existing position
        // variables and two constant thresholds; `W` is folded to an `f64`
        // before it reaches Z3 rather than being `min_ttc * vx_ego`, which
        // would be a product of a variable and leave the fragment.
        if let Some(min_lat_dist) = spec.min_lateral_distance {
            let window = lateral_relevance_window(spec)?;
            super::push_constraint(
                &mut constraints,
                spec.constraint_modes.min_lateral_distance(),
                LTLFormula::Atom(Proposition::RectangularDistanceGT {
                    actor1: ego.id.clone(),
                    actor2: pedestrian.id.clone(),
                    threshold_x: window,
                    threshold_y: min_lat_dist,
                }),
                super::AtomPolarity::Positive,
            );
        }

        // Pedestrian-specific TTC (perpendicular crossing).
        //
        // Same concern as the box above: gating this on `Enforce` only would
        // mean `Violate` asserts nothing. `PedestrianTTCGT` lowers to the guarded
        // implication `ped_on_road ∧ approaching ⟹ ttc_safe`
        // (`encoder.rs`), whose negation is
        // `ped_on_road ∧ approaching ∧ ¬ttc_safe` — it requires the
        // antecedent to actually hold, not just any state, so this negation is
        // worth checking rather than assuming. Verified satisfiable end-to-end
        // (not vacuous, not UNSAT): `ttc_safe` is non-strict, so `¬ttc_safe` is
        // the strict `distance < ttc * ego_vx` — an ordinary close call, not a
        // boundary condition — and a `violate`d `pedestrian_crossing.yaml`
        // produces a real breach.
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

        // The crossing must be a *conflict*, or the bound above constrains
        // nothing.
        //
        // `PedestrianTTCGT` is a guarded implication — "whenever the pedestrian
        // is on the road with the ego behind it and closing, the TTC exceeds
        // `min_ttc`" — and the pedestrian's `py` is a solver variable, not an
        // input. `generate_ltl` asks only that the pedestrian reach the far
        // sidewalk *eventually*, so without this constraint Z3 could satisfy
        // `G(PedestrianTTCGT(..))` by keeping the pedestrian clear of the road
        // at exactly the steps where the ego is bearing down on it and
        // crossing once the ego was past. For example, on an `enforce`d
        // two-lane spec: the pedestrian steps onto the road early, is *off*
        // it again by `t=3` while the ego is still 22 m away, waits on the
        // near kerb through the
        // ego's whole approach, and crosses over steps 19-30 with the ego
        // already past it. The `enforce`d 2 s bound would then be evaluated
        // only at the three opening steps, 33 m out — an `enforce` that no
        // crossing could ever fail. This is the same defect one scenario type
        // over from `scenarios::cut_in_conflict`; see there for the precedent
        // and the cost argument.
        //
        // **The shape, and why not the obvious one.** Not `F(guard)`: that
        // existential, built for the cut-in, measures at >500 s for five
        // scenarios, because a disjunction over the horizon asks Z3 to
        // *search* for the instant. This is `G(antecedent → conflict)` with an
        // antecedent the template already forces — `generate_ltl` asserts
        // `F(CrossingRoad(ped))`, and `Invariant::Liveness` confirms the
        // shipped trajectory really does cross — so every conjunct is an
        // implication Z3 propagates. Unlike the cut-in's `same_lane`, the
        // consequent here contains no disjunction at all, which is the half of
        // the cut-in's 103 s → 14.1 s measurement that did the damage.
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

        // The safety box is *sampled*, so without this constraint the ego
        // could drive through it between two steps.
        //
        // `RectangularDistanceGT` asserts `|dx| >= threshold_x OR |dy| >=
        // threshold_y` at each discrete step, and `threshold_x` is
        // `min_distance / PEDESTRIAN_BOX_LONGITUDINAL_DIVISOR` — a half-box
        // 1 m long for the usual `min_distance: 2.0`. An ego at 20 m/s covers
        // 10 m in a `time_step: 0.5`, so it can sit at `dx = -3.0` at one step
        // and `dx = +7.4` at the next, satisfy the box at every sampled step,
        // and have driven straight through the pedestrian in between. For
        // example, on a single-lane spec with `min_distance: 6.0` where
        // lateral clearance is geometrically impossible: every step would
        // report `boxOK`, `all_constraints_satisfied` would come back `true`,
        // and `dx` would go `-3.000 → +7.375` across one step with `|dy| =
        // 1.875` against a `threshold_y` of 4.0. The box is smaller than one
        // step of travel at any realistic speed, so this is the normal case
        // rather than a corner one.
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
        // established elsewhere as affordable (an implication Z3 propagates), not
        // the `F(⋁ over the horizon)` shape that measures at >500 s. It is a
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

        // Settle on the far kerb and stay there.
        //
        // `generate_ltl` asks only `F(OnSidewalk(far))`, which pins the far
        // kerb at one step and leaves `py` free at every other; combined with a
        // pedestrian that can now only move forward across the road,
        // the loosest witness reaches the far kerb for a single step at the very
        // end — and `OnSidewalk` is satisfied by `py = road_width + ε`, a
        // millimetre onto the sidewalk. This adds, alongside (not instead of)
        // the liveness goal, a hard requirement that the last `SETTLE_TAIL_STEPS`
        // steps lie *past the kerb centre* on the far sidewalk:
        //
        //   left_to_right  → far = right kerb: road_width + SIDEWALK/2 <= py <= road_width + SIDEWALK
        //   right_to_left  → far =  left kerb:      -SIDEWALK <= py <= -SIDEWALK/2
        //
        // The kerb-centre margin (`SIDEWALK/2`) is symmetric with where the
        // pedestrian's *start* is seated on the near kerb, and folds in the
        // "arrives by a hair and hovers" case. All bounds are linear on the
        // existing `py` variables, so this stays in QF_LRA.
        let direction = pedestrian
            .behavior
            .get("direction")
            .and_then(|v| v.as_str())
            .unwrap_or("left_to_right");
        let crosses_left_to_right = direction != "right_to_left";

        let road_width = spec.get_lane_width() * spec.get_num_lanes() as f64;
        let sidewalk = crate::solver::encoder::SIDEWALK_WIDTH;
        // Far-kerb band [lo, hi], past the kerb centre in the crossing direction.
        let (lo, hi) = if crosses_left_to_right {
            (road_width + sidewalk / 2.0, road_width + sidewalk)
        } else {
            (-sidewalk, -sidewalk / 2.0)
        };
        let lo_real = real_from_f64(lo);
        let hi_real = real_from_f64(hi);

        let settle_from = horizon.saturating_sub(SETTLE_TAIL_STEPS.saturating_sub(1));
        for t in settle_from..=horizon {
            let py = encoder.get_lateral_pos(pedestrian_id, t);
            backend.assert(&py.ge(&lo_real));
            backend.assert(&py.le(&hi_real));
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

    /// Both `RectangularDistanceGT` (`min_distance`) and `PedestrianTTCGT`
    /// (`min_ttc`) must not be gated on `ConstraintMode::Enforce` only, or
    /// `Violate` falls through and asserts nothing — `generate_safety` would
    /// return `LTLFormula::True` for either field's contribution. Routed
    /// through `push_constraint` like every other constraint in this module,
    /// `Violate` asserts `F(¬(atom))` (`AtomPolarity::Positive`,
    /// `ConstraintMode::Violate`). This test guards against the regression
    /// where the formula string contains no
    /// `RectangularDistanceGT`/`PedestrianTTCGT` at all, since nothing was
    /// pushed.
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

    /// An `enforce`d pedestrian `min_ttc` must come with the conflict that
    /// makes it evaluable: `G(CrossingRoad(ped) → PedestrianTTCGuard(..))`,
    /// where the guard atom is `PedestrianTTCGT`'s own antecedent. Without it,
    /// `G(PedestrianTTCGT(..))` is satisfiable with the antecedent false at
    /// every step that matters — a pedestrian who waits on the kerb for the ego
    /// to pass and crosses behind it — so the bound cannot fail. This test
    /// guards against that regression: the formula string must contain
    /// `PedestrianTTCGuard`. `tests/pedestrian_conflict_test.rs` is the
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
    /// `examples/pedestrian_*.yaml`, all `min_ttc: ignore`, are unaffected.
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
