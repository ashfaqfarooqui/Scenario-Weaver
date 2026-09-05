//! Bounded LTL encoding and atomic-proposition lowering.
//!
//! This is the "encode an LTL formula/proposition as Z3 constraints" half of
//! `src/solver/encoder.rs`, split out by SW-19: bounded model checking expands
//! temporal operators over the finite time horizon, and proposition lowering
//! converts each atomic [`Proposition`] into a Z3 boolean at a specific time
//! step. Both only need read-only access to the encoder's Z3 variables (the
//! [`EncoderAccessor`] getters) and the scenario spec, so they are free
//! functions rather than methods on `GenericEncoder`.

use z3::ast::{Int, Real};

use crate::dsl::types::ScenarioSpec;
use crate::ltl::formula::{LTLFormula, Proposition};
use crate::solver::encoder::{EncoderAccessor, SIDEWALK_WIDTH};
use crate::solver::encoder_utils::{encode_same_lane_constraint, real_from_f64};

/// Bounded LTL encoding: expand temporal operators over [time, horizon]
///
/// This implements bounded model checking for LTL:
/// - Eventually(φ): φ[t] ∨ φ[t+1] ∨ ... ∨ φ[horizon]
/// - Always(φ): φ[t] ∧ φ[t+1] ∧ ... ∧ φ[horizon]
/// - Until(φ, ψ): ψ[t] ∨ (φ[t] ∧ Until(φ,ψ)[t+1])
/// - Atom(p): encode proposition at time t
pub(crate) fn encode_ltl_bounded(
    accessor: &dyn EncoderAccessor,
    spec: &ScenarioSpec,
    formula: &crate::ltl::formula::LTLFormula,
    time: usize,
    horizon: usize,
) -> z3::ast::Bool {
    match formula {
        // Boolean literals
        LTLFormula::True => z3::ast::Bool::from_bool(true),
        LTLFormula::False => z3::ast::Bool::from_bool(false),

        // Atomic proposition - encode at specific time
        LTLFormula::Atom(prop) => encode_proposition(accessor, spec, prop, time),

        // Boolean operators - recursive encoding
        LTLFormula::Not(phi) => encode_ltl_bounded(accessor, spec, phi, time, horizon).not(),

        LTLFormula::And(phi, psi) => {
            let left = encode_ltl_bounded(accessor, spec, phi, time, horizon);
            let right = encode_ltl_bounded(accessor, spec, psi, time, horizon);
            z3::ast::Bool::and(&[&left, &right])
        }

        LTLFormula::Or(phi, psi) => {
            let left = encode_ltl_bounded(accessor, spec, phi, time, horizon);
            let right = encode_ltl_bounded(accessor, spec, psi, time, horizon);
            z3::ast::Bool::or(&[&left, &right])
        }

        LTLFormula::Implies(phi, psi) => {
            let left = encode_ltl_bounded(accessor, spec, phi, time, horizon);
            let right = encode_ltl_bounded(accessor, spec, psi, time, horizon);
            left.implies(&right)
        }

        // Temporal operators - bounded expansion

        // Next: X(φ) = φ[time+1] (if within horizon)
        LTLFormula::Next(phi) => {
            if time < horizon {
                encode_ltl_bounded(accessor, spec, phi, time + 1, horizon)
            } else {
                // SW-12/L1. At the horizon there is no next state, and
                // this used to yield `false`. Because `Always` expands
                // over `time..=horizon` *inclusive*, that made every
                // `G(X phi)` unsatisfiable regardless of `phi` — the
                // conjunct at `t = horizon` was the literal `false`.
                //
                // Bounded model checking has no information about what
                // happens after the bound, so the honest reading of `X`
                // past the end is "not refuted by this trace". `true` is
                // the standard weak/optimistic semantics for safety
                // properties and is what makes `G(X phi)` mean "phi holds
                // at every step from 1 to the horizon", which is what a
                // caller writing it means. It is unsound for liveness —
                // `F(G(X phi))` becomes trivially satisfiable at the last
                // step — but bounded `F(G(...))` already has that defect
                // independently (see the note in `cut_in_left.rs`).
                z3::ast::Bool::from_bool(true)
            }
        }

        // Eventually: F(φ) = φ[time] ∨ φ[time+1] ∨ ... ∨ φ[horizon]
        LTLFormula::Eventually(phi) => {
            let mut disjuncts = Vec::new();
            for t in time..=horizon {
                disjuncts.push(encode_ltl_bounded(accessor, spec, phi, t, horizon));
            }
            let refs: Vec<&z3::ast::Bool> = disjuncts.iter().collect();
            z3::ast::Bool::or(&refs)
        }

        // Always: G(φ) = φ[time] ∧ φ[time+1] ∧ ... ∧ φ[horizon]
        LTLFormula::Always(phi) => {
            let mut conjuncts = Vec::new();
            for t in time..=horizon {
                conjuncts.push(encode_ltl_bounded(accessor, spec, phi, t, horizon));
            }
            let refs: Vec<&z3::ast::Bool> = conjuncts.iter().collect();
            z3::ast::Bool::and(&refs)
        }

        // Until: φ U ψ = ψ[time] ∨ (φ[time] ∧ (φ U ψ)[time+1])
        // Bounded version: must happen within horizon
        LTLFormula::Until(phi, psi) => {
            let mut disjuncts = Vec::new();

            for t in time..=horizon {
                // ψ happens at time t, and φ holds from time to t-1
                let psi_at_t = encode_ltl_bounded(accessor, spec, psi, t, horizon);

                if t == time {
                    // Base case: ψ holds now
                    disjuncts.push(psi_at_t);
                } else {
                    // φ must hold from time to t-1
                    let mut phi_conjuncts = Vec::new();
                    for s in time..t {
                        phi_conjuncts.push(encode_ltl_bounded(accessor, spec, phi, s, horizon));
                    }
                    let phi_refs: Vec<&z3::ast::Bool> = phi_conjuncts.iter().collect();
                    let phi_holds = z3::ast::Bool::and(&phi_refs);

                    // (φ[time] ∧ ... ∧ φ[t-1]) ∧ ψ[t]
                    let both = z3::ast::Bool::and(&[&phi_holds, &psi_at_t]);
                    disjuncts.push(both);
                }
            }

            let refs: Vec<&z3::ast::Bool> = disjuncts.iter().collect();
            z3::ast::Bool::or(&refs)
        }
    }
}
/// The longitudinal frame in which `Ahead(actor1, actor2)` is judged.
///
/// A function of the unordered pair, never of one member of it: the shared
/// travel direction when the two actors agree, and the fixed road frame
/// (+1) when they do not. See the `Ahead` arm of `encode_proposition` for
/// why (SW-12/M4).
fn ahead_frame(spec: &ScenarioSpec, actor1: &str, actor2: &str) -> i32 {
    let dir = |id: &str| {
        spec.actors
            .iter()
            .find(|a| a.id == id)
            .map_or(1, |a| a.direction)
    };
    let (d1, d2) = (dir(actor1), dir(actor2));
    if d1 == d2 {
        d1
    } else {
        1
    }
}
/// Encode atomic propositions as Z3 constraints at a specific time
fn encode_proposition(
    accessor: &dyn EncoderAccessor,
    spec: &ScenarioSpec,
    prop: &crate::ltl::formula::Proposition,
    time: usize,
) -> z3::ast::Bool {
    match prop {
        // InLane(actor, lane): lane_var[t] == lane
        Proposition::InLane { actor, lane } => {
            let lane_var = accessor.get_lane_var(actor, time);
            let lane_val = Int::from_i64(*lane as i64);
            lane_var.eq(&lane_val)
        }

        // Ahead(actor1, actor2): actor1 is ahead of actor2.
        //
        // SW-12/M4. The frame used to be read off *actor1* alone, which
        // made the relation symmetric instead of antisymmetric whenever
        // the two actors travelled in opposite directions. For ego
        // (dir = +1) against an oncoming NPC (dir = -1):
        //
        //     Ahead{ego, onc}  =>  px_ego > px_onc
        //     Ahead{onc, ego}  =>  px_onc < px_ego
        //
        // — the same constraint, so both directions of the relation were
        // simultaneously satisfiable. `head_on.rs` and
        // `overtake_left.rs` both apply `Ahead` to mixed-direction pairs.
        //
        // The frame is now a function of the *pair*, not of actor1: the
        // shared travel direction when the two agree, and the fixed road
        // frame (+x) when they do not. Being pair-symmetric, the same
        // comparison direction is used for `Ahead(a,b)` and `Ahead(b,a)`,
        // so `Ahead(a,b) => !Ahead(b,a)` holds by construction.
        Proposition::Ahead { actor1, actor2 } => {
            let px1 = accessor.get_longitudinal_pos(actor1, time);
            let px2 = accessor.get_longitudinal_pos(actor2, time);
            if ahead_frame(spec, actor1, actor2) >= 0 {
                px1.gt(px2)
            } else {
                px1.lt(px2)
            }
        }

        // DistanceGT(actor1, actor2, d): same_lane ⟹ |px1[t] - px2[t]| > d
        //
        // SW-10/H7. This used to be a bare `|px1 - px2| > d` with no lane
        // guard, so two vehicles in physically different lanes still had
        // to keep `min_distance` apart longitudinally — over-constraining
        // every multi-lane spec, and disagreeing with
        // `compute_validation_metrics`, which has always gated the metric
        // on the actors sharing a lane. The guarded form is the one the
        // docs describe and the one the validator checks; it is also what
        // the (previously dead) `encode_distance_constraint` in both
        // coordinate encoders already implemented.
        //
        // The guard matters just as much in `Violate` mode: the negation
        // `¬(same_lane ⟹ safe)` is `same_lane ∧ ¬safe`, i.e. "get close
        // *in the same lane*", which is the adversarial event the feature
        // is for. The unguarded negation was satisfiable by two cars
        // passing in adjacent lanes.
        Proposition::DistanceGT {
            actor1,
            actor2,
            distance,
        } => {
            let px1 = accessor.get_longitudinal_pos(actor1, time);
            let px2 = accessor.get_longitudinal_pos(actor2, time);
            let dist_val = real_from_f64(*distance);

            // |px1 - px2| >= d, as (px1 - px2 >= d) OR (px2 - px1 >= d).
            //
            // SW-12. The comparison is deliberately non-strict, and this
            // is the whole of the violate-mode fix. `compute_validation_metrics`
            // reports a breach when `distance < min_distance`, so "safe"
            // for the validator is `distance >= min_distance`. The encoder
            // asserted the strict `>`, and `Violate` mode is the negation
            // of whatever the encoder asserted: `!(d > 5)` is `d <= 5`,
            // which is satisfied by `d == 5` exactly — a solution the
            // validator then reports as *satisfying* the constraint the
            // spec asked to have violated.
            // `test_violate_mode_negates_constraint` measured exactly
            // 5.0000 against a threshold of 5.00 and passed only because
            // its assertion was `<=`.
            //
            // With `>=` here the two agree: enforce asserts what the
            // validator calls safe, and violate asserts its exact
            // negation, `distance < min_distance`, strictly.
            let diff_pos = px1 - px2;
            let diff_neg = px2 - px1;

            let pos_case = diff_pos.ge(&dist_val);
            let neg_case = diff_neg.ge(&dist_val);
            let distance_safe = z3::ast::Bool::or(&[&pos_case, &neg_case]);

            let same_lane = encode_same_lane_constraint(
                accessor.get_lane_var(actor1, time),
                accessor.get_lane_var(actor2, time),
                accessor.get_lateral_pos(actor1, time),
                accessor.get_lateral_pos(actor2, time),
                spec.get_lane_width(),
            );

            same_lane.implies(&distance_safe)
        }

        // TTCGT(actor1, actor2, ttc): TTC > ttc (if collision possible)
        Proposition::TTCGT {
            actor1,
            actor2,
            ttc,
        } => crate::solver::encoder::encode_ttc_constraint(
            accessor, spec, actor1, actor2, *ttc, time,
        ),

        // Approaching(follower, leader): leader in front, follower gaining — the
        // lane-free half of the state in which a TTC exists at all. See the
        // proposition's doc comment (SW-22) for why `TTCGT` needs this to mean
        // anything, and why the lane test is the caller's antecedent, not part of
        // this atom.
        Proposition::Approaching { follower, leader } => {
            crate::solver::encoder::encode_approaching(accessor, follower, leader, time)
        }

        // OnSidewalk(actor, side): -SIDEWALK_WIDTH <= py < 0 (left) or
        // road_width < py <= road_width + SIDEWALK_WIDTH (right).
        //
        // Previously an unbounded half-plane (`py < 0` / `py > road_width`), which let
        // Z3 park a pedestrian arbitrarily far off the road — up to 10.7 m measured on
        // pedestrian_wide_road (SW-16 E3, re-attributed from SW-10). Bounded to a strip
        // matching the `LaneType::Sidewalk` the xodr exporter now emits.
        Proposition::OnSidewalk { actor, side } => {
            let py = accessor.get_lateral_pos(actor, time);
            let zero = Real::from_rational(0_i64, 1_i64);

            let lane_width = spec.get_lane_width();
            let num_lanes = spec.get_num_lanes();
            let road_width = lane_width * num_lanes as f64;
            let road_width_real = real_from_f64(road_width);
            let sidewalk_outer_real = real_from_f64(-SIDEWALK_WIDTH);
            let road_sidewalk_outer_real = real_from_f64(road_width + SIDEWALK_WIDTH);

            if side == "left" {
                z3::ast::Bool::and(&[&py.lt(&zero), &py.ge(&sidewalk_outer_real)])
            } else {
                z3::ast::Bool::and(&[&py.gt(&road_width_real), &py.le(&road_sidewalk_outer_real)])
            }
        }

        // CrossingRoad(actor): 0 <= py <= road_width
        Proposition::CrossingRoad { actor } => {
            let py = accessor.get_lateral_pos(actor, time);
            let zero = Real::from_rational(0_i64, 1_i64);

            let lane_width = spec.get_lane_width();
            let num_lanes = spec.get_num_lanes();
            let road_width = lane_width * num_lanes as f64;
            let road_width_real = real_from_f64(road_width);

            let on_road_start = py.ge(&zero);
            let on_road_end = py.le(&road_width_real);
            z3::ast::Bool::and(&[&on_road_start, &on_road_end])
        }

        // Distance2DGT: 2D Euclidean distance between actors > threshold
        Proposition::Distance2DGT {
            actor1,
            actor2,
            distance,
        } => {
            let px1 = accessor.get_longitudinal_pos(actor1, time);
            let py1 = accessor.get_lateral_pos(actor1, time);
            let px2 = accessor.get_longitudinal_pos(actor2, time);
            let py2 = accessor.get_lateral_pos(actor2, time);

            // Euclidean distance: sqrt((px1-px2)^2 + (py1-py2)^2) > threshold
            // Z3 encoding: (px1-px2)^2 + (py1-py2)^2 > threshold^2
            let dx = px1 - px2;
            let dy = py1 - py2;
            let dist_sq = &(&dx * &dx) + &(&dy * &dy);
            let threshold_sq = real_from_f64(distance * distance);
            dist_sq.gt(&threshold_sq)
        }

        // ManhattanDistanceGT: Manhattan distance between actors > threshold
        // Linear encoding: |dx| + |dy| > threshold
        // Implemented as disjunction over four cases (one per quadrant)
        Proposition::ManhattanDistanceGT {
            actor1,
            actor2,
            distance,
        } => {
            let px1 = accessor.get_longitudinal_pos(actor1, time);
            let py1 = accessor.get_lateral_pos(actor1, time);
            let px2 = accessor.get_longitudinal_pos(actor2, time);
            let py2 = accessor.get_lateral_pos(actor2, time);

            let dx = px1 - px2;
            let dy = py1 - py2;
            let threshold_real = real_from_f64(*distance);
            let zero = Real::from_rational(0_i64, 1_i64);

            // Manhattan distance: |dx| + |dy| > threshold
            // We check all four combinations of signs:
            // Case 1: dx ≥ 0, dy ≥ 0 → dx + dy > threshold
            // Case 2: dx ≥ 0, dy < 0 → dx - dy > threshold
            // Case 3: dx < 0, dy ≥ 0 → -dx + dy > threshold
            // Case 4: dx < 0, dy < 0 → -dx - dy > threshold
            //
            // Disjunction: at least one case must hold
            let case1 = (&dx + &dy).gt(&threshold_real);
            let case2 = (&dx - &dy).gt(&threshold_real);
            let case3 = (&dy - &dx).gt(&threshold_real); // -dx + dy = dy - dx
            let case4 = (&zero - &(&dx + &dy)).gt(&threshold_real); // -(dx + dy)

            z3::ast::Bool::or(&[&case1, &case2, &case3, &case4])
        }

        // RectangularDistanceGT: Rectangular safety box
        // Simplest linear encoding: |dx| > threshold_x OR |dy| > threshold_y
        Proposition::RectangularDistanceGT {
            actor1,
            actor2,
            threshold_x,
            threshold_y,
        } => {
            let px1 = accessor.get_longitudinal_pos(actor1, time);
            let py1 = accessor.get_lateral_pos(actor1, time);
            let px2 = accessor.get_longitudinal_pos(actor2, time);
            let py2 = accessor.get_lateral_pos(actor2, time);

            let dx = px1 - px2;
            let dy = py1 - py2;
            let threshold_x_real = real_from_f64(*threshold_x);
            let threshold_y_real = real_from_f64(*threshold_y);

            // |dx| >= threshold_x: dx >= threshold_x OR dx <= -threshold_x
            //
            // SW-33. Found while checking this negation is not vacuous
            // for `Violate`: with the strict `>`/`<` this used to be,
            // `.negate()` (De Morgan) gives the *non-strict* conjunction
            // `dx <= tx AND dy <= ty` — satisfiable with `dx == tx`
            // exactly, which `compute_validation_metrics`'s
            // `box_safe = dx > tx - METRIC_TOL || ...` calls safe (the
            // same tolerance-buffered-toward-safe boundary every other
            // proposition in this file uses). Z3 does return that exact
            // witness in practice (verified: an isolated `min_distance:
            // violate` spec settled on `dx == threshold_x` for several
            // consecutive steps), so `Violate` could produce a scenario
            // reporting no box violation at all — the SW-12/30/35
            // boundary shape, for this proposition too, just not named
            // in the original issue. Non-strict here makes the negation
            // strict (`dx < tx AND dy < ty`), so `Violate` can no longer
            // settle on the exact boundary the validator calls safe.
            let dx_positive = dx.ge(&threshold_x_real);
            let neg_threshold_x = real_from_f64(-threshold_x);
            let dx_negative = dx.le(&neg_threshold_x);
            let dx_outside = z3::ast::Bool::or(&[&dx_positive, &dx_negative]);

            // |dy| >= threshold_y: dy >= threshold_y OR dy <= -threshold_y
            let dy_positive = dy.ge(&threshold_y_real);
            let neg_threshold_y = real_from_f64(-threshold_y);
            let dy_negative = dy.le(&neg_threshold_y);
            let dy_outside = z3::ast::Bool::or(&[&dy_positive, &dy_negative]);

            // At least one dimension must be outside the box
            z3::ast::Bool::or(&[&dx_outside, &dy_outside])
        }

        // PedestrianTTCGT: Time-to-collision for perpendicular crossing
        //
        // SW-35. This is the SW-12/SW-30 boundary defect a third time,
        // in a guarded implication rather than a plain comparison.
        // `¬(A ⟹ B)` is `A ∧ ¬B`; the antecedent `A`
        // (`ped_on_road ∧ approaching`) is unaffected by the strictness
        // of the consequent, so the question reduces to the same one
        // SW-12/SW-30 answered for `B` (`ttc_safe`) alone.
        // `compute_validation_metrics` (below, the `pedestrian_pair`
        // branch) calls `ttc == min_ttc` safe (it flags a violation only
        // at `ttc < min_ttc - METRIC_TOL`). `ttc_safe` was the strict
        // `distance > ttc * ego_vx`, so `Violate`'s negation
        // `A ∧ ¬ttc_safe` had `¬ttc_safe` as the non-strict
        // `distance <= ttc * ego_vx`, satisfiable at exactly
        // `distance == ttc * ego_vx` — the boundary the validator calls
        // safe. Non-strict here (`.ge`) makes `¬ttc_safe` strict
        // (`distance < ttc * ego_vx`), matching the validator's boundary
        // exactly, as SW-12/SW-30 did for their propositions.
        Proposition::PedestrianTTCGT {
            ego,
            pedestrian,
            ttc,
        } => {
            let ego_px = accessor.get_longitudinal_pos(ego, time);
            let ego_vx = accessor.get_longitudinal_vel(ego, time);
            let ped_px = accessor.get_longitudinal_pos(pedestrian, time);
            let ped_py = accessor.get_lateral_pos(pedestrian, time);

            let lane_width = spec.get_lane_width();
            let num_lanes = spec.get_num_lanes();
            let road_width = lane_width * num_lanes as f64;
            let road_width_real = real_from_f64(road_width);
            let zero = Real::from_rational(0_i64, 1_i64);

            // Pedestrian on road: 0 <= py <= road_width
            let ped_on_road =
                z3::ast::Bool::and(&[&ped_py.ge(&zero), &ped_py.le(&road_width_real)]);

            // Ego approaching pedestrian's position
            let ego_behind = ego_px.lt(ped_px);
            let ego_moving_forward = ego_vx.gt(&zero);
            let approaching = z3::ast::Bool::and(&[&ego_behind, &ego_moving_forward]);

            // TTC = (ped_px - ego_px) / ego_vx
            // Safe if: (ped_px - ego_px) > ttc * ego_vx
            let distance = ped_px - ego_px;
            let ttc_val = real_from_f64(*ttc);
            let ttc_safe = distance.ge(&(&ttc_val * ego_vx));

            // Overall: NOT (ped_on_road AND approaching) OR ttc_safe
            z3::ast::Bool::and(&[&ped_on_road, &approaching]).implies(&ttc_safe)
        }

        // VelocityGT: Actor's longitudinal speed exceeds threshold (min_velocity).
        // Linear constraint: |vx| >= threshold
        //
        // SW-41. `generate_default_safety` (scenarios/mod.rs) uses this atom
        // as `AtomPolarity::Positive` for `min_velocity`: the atom itself is
        // the safe condition, so `Violate` asserts its negation eventually.
        // `compute_validation_metrics` calls a step safe whenever
        // `vx_abs >= min_vel - METRIC_TOL`, i.e. it treats `vx_abs == min_vel`
        // exactly as satisfying the bound. This used to assert the strict
        // `>`, so `Violate`'s negation was the non-strict `|vx| <= velocity`,
        // satisfiable at `vx == velocity` exactly — a point the validator
        // does not flag as a violation. Non-strict here (`.ge`/`.le`) makes
        // the negation strict, the same fix SW-12/30/33/35 made for their
        // propositions.
        Proposition::VelocityGT { actor, velocity } => {
            let vx = accessor.get_longitudinal_vel(actor, time);
            let threshold_val = real_from_f64(*velocity);

            // |vx| >= threshold is equivalent to: (vx >= threshold) OR (vx <= -threshold)
            let pos_case = vx.ge(&threshold_val);
            let neg_threshold = real_from_f64(-velocity);
            let neg_case = vx.le(&neg_threshold);

            z3::ast::Bool::or(&[&pos_case, &neg_case])
        }

        // VelocityLT: Actor's longitudinal speed is below threshold (max_velocity).
        // Linear constraint: |vx| <= threshold
        //
        // SW-41. Same reasoning as `VelocityGT` above, mirrored: this atom is
        // `AtomPolarity::Positive` for `max_velocity`, and
        // `compute_validation_metrics` calls a step safe whenever
        // `vx_abs <= max_vel + METRIC_TOL`. The strict `<` this used to
        // assert made `Violate`'s negation the non-strict `|vx| >= velocity`,
        // satisfiable at `vx == velocity` exactly, which the validator does
        // not flag. Non-strict here makes the negation strict.
        Proposition::VelocityLT { actor, velocity } => {
            let vx = accessor.get_longitudinal_vel(actor, time);
            let threshold_val = real_from_f64(*velocity);
            let neg_threshold = real_from_f64(-velocity);

            // |vx| <= threshold is equivalent to: -threshold <= vx <= threshold
            let upper_bound = vx.le(&threshold_val);
            let lower_bound = vx.ge(&neg_threshold);

            z3::ast::Bool::and(&[&upper_bound, &lower_bound])
        }

        // LateralDistanceGT: Lateral distance between actors exceeds threshold
        // Linear constraint: |py1 - py2| >= distance
        //
        // SW-30. Same defect SW-12 fixed for `DistanceGT`, in the one
        // proposition it did not touch. `compute_validation_metrics`
        // reports a breach only at `lateral < min_lat - METRIC_TOL`
        // (encoder.rs, `compute_validation_metrics`), i.e. it calls
        // `lateral >= min_lat` safe. This used to assert the strict `>`,
        // so under `Violate` the negation `!(d > min_lat)` is `d <=
        // min_lat`, satisfiable at `d == min_lat` exactly — a point the
        // validator does not consider a breach. Non-strict here makes
        // `Violate`'s negation `d < min_lat` strictly, matching the
        // validator's own boundary exactly, as SW-12 did for
        // `DistanceGT`.
        Proposition::LateralDistanceGT {
            actor1,
            actor2,
            distance,
        } => {
            let py1 = accessor.get_lateral_pos(actor1, time);
            let py2 = accessor.get_lateral_pos(actor2, time);
            let dist_val = real_from_f64(*distance);

            // |py1 - py2| >= d is equivalent to: (py1 - py2 >= d) OR (py2 - py1 >= d)
            let diff_pos = py1 - py2;
            let diff_neg = py2 - py1;

            let pos_case = diff_pos.ge(&dist_val);
            let neg_case = diff_neg.ge(&dist_val);

            z3::ast::Bool::or(&[&pos_case, &neg_case])
        }

        // OnLeftOf: Actor1 is laterally left of Actor2
        // Simple comparison: py1 > py2
        Proposition::OnLeftOf { actor1, actor2 } => {
            let py1 = accessor.get_lateral_pos(actor1, time);
            let py2 = accessor.get_lateral_pos(actor2, time);
            py1.gt(py2)
        }

        // OnRightOf: Actor1 is laterally right of Actor2
        // Simple comparison: py1 < py2
        Proposition::OnRightOf { actor1, actor2 } => {
            let py1 = accessor.get_lateral_pos(actor1, time);
            let py2 = accessor.get_lateral_pos(actor2, time);
            py1.lt(py2)
        }

        // RelativeVelocityGT: Relative longitudinal velocity exceeds threshold
        // Linear constraint: |vx1 - vx2| > velocity
        Proposition::RelativeVelocityGT {
            actor1,
            actor2,
            velocity,
        } => {
            let vx1 = accessor.get_longitudinal_vel(actor1, time);
            let vx2 = accessor.get_longitudinal_vel(actor2, time);
            let vel_val = real_from_f64(*velocity);

            // |vx1 - vx2| > v is equivalent to: (vx1 - vx2 > v) OR (vx2 - vx1 > v)
            let diff_pos = vx1 - vx2;
            let diff_neg = vx2 - vx1;

            let pos_case = diff_pos.gt(&vel_val);
            let neg_case = diff_neg.gt(&vel_val);

            z3::ast::Bool::or(&[&pos_case, &neg_case])
        }
    }
}

// SW-41. Test-only: the classification below exists purely to drive
// `test_strictness_coverage_is_exhaustive_and_documented` and
// `test_measured_propositions_agree_with_validator_at_the_boundary`, so it is
// `#[cfg(test)]` rather than `pub(crate)` in the production build — the
// no-wildcard-`match` guarantee only needs to hold when the test suite
// compiles, and gating it out of non-test builds keeps it off the clippy
// ratchet.
#[cfg(test)]
use self::strictness_coverage_impl::{strictness_coverage, StrictnessCoverage};

#[cfg(test)]
mod strictness_coverage_impl {
    use super::Proposition;

    /// Whether a `Proposition` variant's Enforce/Violate boundary is
    /// checked against `compute_validation_metrics` at all, and if so, whether
    /// that check has been confirmed to agree with the encoder's strictness at
    /// the exact numeric boundary — the shape SW-12/30/33/35 each found and
    /// fixed independently (`Violate` settling on `distance == threshold`
    /// exactly, and the validator, testing non-strictly, calling that safe).
    ///
    /// This `match` has **no wildcard arm** on purpose: adding a 13th
    /// `Proposition` variant without adding a line here is a compile error, not
    /// a silently-skipped case. See `test_strictness_coverage_is_exhaustive_and_documented`
    /// for the classification of every current variant, and
    /// `test_measured_propositions_agree_with_validator_at_the_boundary` for the
    /// actual boundary check on every `Measured` one.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum StrictnessCoverage {
    /// `compute_validation_metrics` derives a numeric safe/unsafe boundary
    /// from this proposition's threshold, and the boundary test confirms the
    /// encoder's safe formula (the one `Violate` negates) holds — non-strict
    /// — exactly at that boundary, agreeing with the validator.
    Measured,
    /// No validator-side check exists for this proposition's threshold at
    /// all. Not a defect by itself — `Violate` still asserts something, Z3
    /// still solves it — but the strict/non-strict question is *unreachable*
    /// here: nothing independently measures whether the encoder and the
    /// reported scenario agree, which is exactly where SW-12/30/33/35's
    /// defect shape hid for as long as it did. This is the SW-31/SW-41 gap;
    /// see `SW-41-strictness-sweep.md` for the follow-up issue.
    Unmeasured,
    /// Not a thresholded safety comparison that a spec-level `Enforce`/
    /// `Violate` polarity is ever applied to (a discrete equality, a
    /// positional ordering, or a guard term used only as an antecedent) —
    /// the "boundary satisfied exactly, reported as safe" defect shape does
    /// not apply because there is no min/max threshold whose boundary the
    /// validator could disagree with the encoder about.
    NotApplicable,
}

/// The exhaustive classification driving [`StrictnessCoverage`]. Matches on
/// the variant shape only (`{ .. }`), so it says nothing about a
/// proposition's field values — only whether *this kind* of proposition is
/// in scope for the boundary invariant.
pub(crate) fn strictness_coverage(prop: &Proposition) -> StrictnessCoverage {
    use StrictnessCoverage::{Measured, NotApplicable, Unmeasured};
    match prop {
        // Discrete equality / positional ordering: no numeric threshold, so
        // no boundary for `Violate` to land on exactly.
        Proposition::InLane { .. }
        | Proposition::Ahead { .. }
        | Proposition::OnLeftOf { .. }
        | Proposition::OnRightOf { .. } => NotApplicable,

        // Region-membership predicates used only inside `eventually()` goals
        // in `pedestrian_crossing.rs` (reach the sidewalk / cross the road),
        // never through `generate_default_safety`'s Enforce/Violate
        // machinery, and never independently re-measured by
        // `compute_validation_metrics`.
        Proposition::OnSidewalk { .. } | Proposition::CrossingRoad { .. } => Unmeasured,

        // Dead code: neither is lowered by any scenario type (verified by
        // grep — `Proposition::Distance2DGT`/`Proposition::ManhattanDistanceGT`
        // do not appear outside this enum and `encode_proposition`), so
        // there is nothing for `compute_validation_metrics` to have ever
        // measured.
        Proposition::Distance2DGT { .. } | Proposition::ManhattanDistanceGT { .. } => Unmeasured,

        // The lane-free guard half of a directed conflict (SW-22). Its own
        // boundary (closing speed exactly at `TTC_CLOSING_SPEED_EPSILON`) is
        // deliberately strict and matches `compute_validation_metrics`'s own
        // `rel_vel > epsilon` exactly (see `encode_approaching`'s doc
        // comment) — but it is never itself the subject of an Enforce/
        // Violate polarity; it is always an antecedent. The invariant this
        // sweep is about does not apply to an antecedent.
        Proposition::Approaching { .. } => NotApplicable,

        // Every one of these has a spec-level threshold, is asserted through
        // `generate_default_safety`'s `push_constraint` (Enforce/Violate
        // polarity) or the pedestrian equivalent in `pedestrian_crossing.rs`,
        // and has a corresponding numeric check in
        // `compute_validation_metrics`. See the boundary test for the
        // per-proposition setup and the doc comment on each arm of
        // `encode_proposition` for why its strictness is what it is.
        Proposition::DistanceGT { .. }
        | Proposition::TTCGT { .. }
        | Proposition::LateralDistanceGT { .. }
        | Proposition::RelativeVelocityGT { .. }
        | Proposition::RectangularDistanceGT { .. }
        | Proposition::PedestrianTTCGT { .. }
        | Proposition::VelocityGT { .. }
        | Proposition::VelocityLT { .. } => Measured,
    }
}
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dsl::types::{
        ActorRole, ActorSpec, LaneChangeConfig, LaneChangeDirection, RoadSpec, ScenarioType,
        ValueOrRange,
    };
    use crate::solver::encoder::Z3Encoder;
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

    #[test]
    fn test_ltl_encoding_simple() {
        use crate::ltl::formula::{LTLFormula, Proposition};

        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let spec = create_test_spec();
            let mut encoder = Z3Encoder::new(spec);
            encoder.create_variables();
            encoder.encode_initial_conditions();
            encoder.encode_kinematics();
            encoder.encode_lane_velocity_constraints();
            encoder.encode_lateral_velocity_bounds();

            // Test simple atomic proposition: InLane(ego, 1)
            let formula = LTLFormula::Atom(Proposition::InLane {
                actor: "ego".to_string(),
                lane: 1,
            });

            encoder.encode_ltl(&formula);
            assert_eq!(encoder.check(), SatResult::Sat);
        });
    }

    #[test]
    fn test_ltl_encoding_eventually() {
        use crate::ltl::formula::{LTLFormula, Proposition};

        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let spec = create_test_spec();
            let mut encoder = Z3Encoder::new(spec);
            encoder.create_variables();
            encoder.encode_initial_conditions();
            encoder.encode_kinematics();
            encoder.encode_lane_velocity_constraints();
            encoder.encode_lateral_velocity_bounds();

            // Test Eventually: F(InLane(npc, 1))
            // NPC should eventually be in lane 1
            let formula = LTLFormula::Atom(Proposition::InLane {
                actor: "npc".to_string(),
                lane: 1,
            })
            .eventually();

            encoder.encode_ltl(&formula);
            assert_eq!(encoder.check(), SatResult::Sat);

            let model = encoder.get_model().unwrap();

            // Check that NPC is in lane 1 at some point
            let mut found_lane_1 = false;
            for t in 0..=encoder.horizon {
                let lane = model.eval(encoder.get_lane_var("npc", t), true).unwrap();
                if lane.to_string() == "1" {
                    found_lane_1 = true;
                    println!("NPC in lane 1 at time {}", t);
                    break;
                }
            }
            assert!(found_lane_1, "NPC should eventually be in lane 1");
        });
    }

    #[test]
    fn test_ltl_encoding_always() {
        use crate::ltl::formula::{LTLFormula, Proposition};

        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let spec = create_test_spec();
            let mut encoder = Z3Encoder::new(spec);
            encoder.create_variables();
            encoder.encode_initial_conditions();
            encoder.encode_kinematics();
            encoder.encode_lane_velocity_constraints();
            encoder.encode_lateral_velocity_bounds();

            // Test Always: G(InLane(ego, 1))
            // Ego should always be in lane 1
            let formula = LTLFormula::Atom(Proposition::InLane {
                actor: "ego".to_string(),
                lane: 1,
            })
            .always();

            encoder.encode_ltl(&formula);
            assert_eq!(encoder.check(), SatResult::Sat);

            let model = encoder.get_model().unwrap();

            // Check that ego is in lane 1 at all times
            for t in 0..=encoder.horizon {
                let lane = model.eval(encoder.get_lane_var("ego", t), true).unwrap();
                assert_eq!(lane.to_string(), "1", "Ego should always be in lane 1");
            }
        });
    }

    #[test]
    fn test_ltl_encoding_until() {
        use crate::ltl::formula::{LTLFormula, Proposition};

        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let spec = create_test_spec();
            let mut encoder = Z3Encoder::new(spec);
            encoder.create_variables();
            encoder.encode_initial_conditions();
            encoder.encode_kinematics();
            encoder.encode_lane_velocity_constraints();
            encoder.encode_lateral_velocity_bounds();

            // Test Until: InLane(npc, 0) U InLane(npc, 1)
            // NPC stays in lane 0 until it moves to lane 1
            let formula = LTLFormula::Atom(Proposition::InLane {
                actor: "npc".to_string(),
                lane: 0,
            })
            .until(LTLFormula::Atom(Proposition::InLane {
                actor: "npc".to_string(),
                lane: 1,
            }));

            encoder.encode_ltl(&formula);
            assert_eq!(encoder.check(), SatResult::Sat);

            let model = encoder.get_model().unwrap();

            // Find when NPC transitions to lane 1
            let mut transition_time = None;
            for t in 0..=encoder.horizon {
                let lane = model.eval(encoder.get_lane_var("npc", t), true).unwrap();
                if lane.to_string() == "1" {
                    transition_time = Some(t);
                    break;
                }
            }

            if let Some(trans_t) = transition_time {
                println!("NPC transitions to lane 1 at time {}", trans_t);
                // Before transition, should be in lane 0
                for t in 0..trans_t {
                    let lane = model.eval(encoder.get_lane_var("npc", t), true).unwrap();
                    assert_eq!(
                        lane.to_string(),
                        "0",
                        "NPC should be in lane 0 before transition"
                    );
                }
            }
        });
    }

    #[test]
    fn test_velocity_propositions_linear() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            use crate::ltl::formula::Proposition;

            let spec = ScenarioSpec {
                scenario_type: ScenarioType::CutInLeft,
                time_step: 0.5,
                duration: 5.0,
                actors: vec![ActorSpec {
                    id: "ego".to_string(),
                    role: ActorRole::Ego,
                    lane: 0,
                    position: ValueOrRange::Value(0.0),
                    speed: ValueOrRange::Value(20.0),
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
                max_velocity: Some(25.0),
                min_velocity: Some(10.0),
                min_lateral_distance: None,
                max_relative_velocity: None,
                max_lateral_acceleration: 2.0,
                coordinate_system: crate::dsl::types::CoordinateSystem::Cartesian,
                bicycle_config: None,
            };

            let mut encoder = Z3Encoder::new(spec);
            encoder.create_variables();
            encoder.encode_initial_conditions();

            // Test VelocityGT (linear constraint)
            let prop_gt = Proposition::VelocityGT {
                actor: "ego".to_string(),
                velocity: 15.0,
            };
            let constraint_gt = encode_proposition(&encoder, &encoder.spec, &prop_gt, 0);

            // Test VelocityLT (linear constraint)
            let prop_lt = Proposition::VelocityLT {
                actor: "ego".to_string(),
                velocity: 30.0,
            };
            let constraint_lt = encode_proposition(&encoder, &encoder.spec, &prop_lt, 0);

            // Both must constrain the ego's longitudinal velocity variable at t=0,
            // and they must be different constraints (not the same comparison).
            let gt = constraint_gt.to_string();
            let lt = constraint_lt.to_string();
            assert!(
                gt.contains("ego_vx_0"),
                "VelocityGT should reference ego_vx_0, got: {gt}"
            );
            assert!(
                lt.contains("ego_vx_0"),
                "VelocityLT should reference ego_vx_0, got: {lt}"
            );
            assert_ne!(gt, lt, "VelocityGT and VelocityLT must encode differently");

            // Linear means the velocity variable never multiplies another variable:
            // the only product allowed is by a literal coefficient.
            assert!(
                !gt.contains("(* ego_vx_0"),
                "VelocityGT must stay linear in ego_vx_0, got: {gt}"
            );
            assert!(
                !lt.contains("(* ego_vx_0"),
                "VelocityLT must stay linear in ego_vx_0, got: {lt}"
            );
        });
    }

    /// Helper: create a two-actor spec with both in the same lane (for TTC/distance tests)
    /// `Ahead(a, b)` and `Ahead(b, a)` must not both hold (SW-12/M4).
    ///
    /// The failing case is a *mixed-direction* pair, because the frame used to
    /// be read off actor1 alone. For ego (dir = +1) against an oncoming npc
    /// (dir = -1) the lowering produced `px_ego > px_onc` for one direction of
    /// the relation and `px_onc < px_ego` for the other — the same constraint,
    /// so asserting both was satisfiable and "ahead" was not a strict order.
    /// `head_on.rs` and `overtake_left.rs` both apply `Ahead` to exactly this
    /// kind of pair.
    ///
    /// Asserting the conjunction and requiring UNSAT is the test: it holds for
    /// any lowering that is genuinely antisymmetric, and fails for any that
    /// collapses the two directions into one comparison.
    #[test]
    fn test_ahead_is_antisymmetric_for_a_mixed_direction_pair() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            use crate::ltl::formula::Proposition;

            let mut spec = create_two_actor_same_lane_spec();
            spec.actors[1].direction = -1;
            spec.road = Some(RoadSpec {
                num_lanes: 2,
                lane_width: 3.5,
                lane_directions: vec![1, -1],
                road_length: None,
            });
            spec.actors[1].lane = 1;

            let mut encoder = Z3Encoder::new(spec);
            encoder.create_variables();

            let ahead = |a: &str, b: &str| Proposition::Ahead {
                actor1: a.to_string(),
                actor2: b.to_string(),
            };
            let fwd = encode_proposition(&encoder, &encoder.spec, &ahead("ego", "npc"), 0);
            let rev = encode_proposition(&encoder, &encoder.spec, &ahead("npc", "ego"), 0);
            let both = z3::ast::Bool::and(&[&fwd, &rev]);
            encoder.assert_constraint(&both);

            assert_eq!(
                encoder.check(),
                SatResult::Unsat,
                "Ahead(ego, npc) and Ahead(npc, ego) must not be simultaneously \
                 satisfiable for a mixed-direction pair"
            );
        });
    }

    /// The same-direction case must keep working, and must still be decided in
    /// the actors' own travel direction rather than the road frame.
    #[test]
    fn test_ahead_is_antisymmetric_for_a_same_direction_pair() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            use crate::ltl::formula::Proposition;

            let mut encoder = Z3Encoder::new(create_two_actor_same_lane_spec());
            encoder.create_variables();

            let ahead = |a: &str, b: &str| Proposition::Ahead {
                actor1: a.to_string(),
                actor2: b.to_string(),
            };
            let fwd = encode_proposition(&encoder, &encoder.spec, &ahead("ego", "npc"), 0);
            let rev = encode_proposition(&encoder, &encoder.spec, &ahead("npc", "ego"), 0);
            encoder.assert_constraint(&z3::ast::Bool::and(&[&fwd, &rev]));

            assert_eq!(encoder.check(), SatResult::Unsat);
        });
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

    // ===== Group 1: Proposition encoding correctness =====

    #[test]
    fn test_proposition_distance_gt() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            use crate::ltl::formula::{LTLFormula, Proposition};

            let spec = create_two_actor_same_lane_spec();
            let mut encoder = Z3Encoder::new(spec);
            encoder.create_variables();
            encoder.encode_initial_conditions();

            // Encode: DistanceGT(ego, npc, 30.0) at time 0
            let formula = LTLFormula::Atom(Proposition::DistanceGT {
                actor1: "ego".to_string(),
                actor2: "npc".to_string(),
                distance: 30.0,
            });
            encoder.encode_ltl(&formula);

            assert_eq!(encoder.check(), SatResult::Sat);

            let model = encoder.get_model().unwrap();
            let px_ego = model
                .eval(encoder.get_longitudinal_pos("ego", 0), true)
                .unwrap();
            let px_npc = model
                .eval(encoder.get_longitudinal_pos("npc", 0), true)
                .unwrap();

            // Parse values and verify |px_ego - px_npc| > 30
            let ego_val: f64 = crate::solver::backend::parse_z3_real_pub(&px_ego.to_string());
            let npc_val: f64 = crate::solver::backend::parse_z3_real_pub(&px_npc.to_string());
            let dist = (ego_val - npc_val).abs();
            assert!(dist > 30.0, "Distance {} should be > 30.0", dist);
        });
    }

    #[test]
    fn test_proposition_lateral_distance_gt() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            use crate::ltl::formula::{LTLFormula, Proposition};

            let spec = create_two_actor_diff_lane_spec();
            let mut encoder = Z3Encoder::new(spec);
            encoder.create_variables();
            encoder.encode_initial_conditions();

            // Encode: LateralDistanceGT(ego, npc, 2.0) at time 0
            let formula = LTLFormula::Atom(Proposition::LateralDistanceGT {
                actor1: "ego".to_string(),
                actor2: "npc".to_string(),
                distance: 2.0,
            });
            encoder.encode_ltl(&formula);

            assert_eq!(encoder.check(), SatResult::Sat);

            let model = encoder.get_model().unwrap();
            let py_ego = model.eval(encoder.get_lateral_pos("ego", 0), true).unwrap();
            let py_npc = model.eval(encoder.get_lateral_pos("npc", 0), true).unwrap();

            let ego_lat: f64 = crate::solver::backend::parse_z3_real_pub(&py_ego.to_string());
            let npc_lat: f64 = crate::solver::backend::parse_z3_real_pub(&py_npc.to_string());
            let lat_dist = (ego_lat - npc_lat).abs();
            assert!(
                lat_dist > 2.0,
                "Lateral distance {} should be > 2.0",
                lat_dist
            );
        });
    }

    /// SW-30. `min_lateral_distance` had SW-12's defect in the one
    /// constraint SW-12 did not touch: `LateralDistanceGT` lowered to a
    /// *strict* `|py1 - py2| > d`, so `Violate` mode asserted its negation
    /// `|py1 - py2| <= d` — satisfiable **at exactly `d`**, a point
    /// `compute_validation_metrics` (`lateral < min_lat - METRIC_TOL`) does
    /// not consider a breach.
    ///
    /// `create_two_actor_diff_lane_spec` places ego in lane 1 and npc in lane
    /// 0 on a 3.5 m-wide road, and with no `lane_changes` configured for
    /// either actor, `py` is pinned to its lane centre at every time step
    /// (`encode_lane_position_coupling_at_time`) rather than left free — so
    /// `|py_ego - py_npc|` is exactly `3.5` by construction, not a value Z3
    /// chooses. That rigidity is what makes this deterministic: the atom's
    /// negation is tested at a distance equal to that fixed gap, so whether
    /// it is satisfiable is entirely decided by whether the lowering is
    /// strict or not, with no solver freedom to launder the result either
    /// way.
    ///
    /// This is deliberately a single time step, not `eventually` over a full
    /// scenario. Every scenario type wiring `min_lateral_distance` through
    /// `generate_default_safety` (`CutInLeft`, `CutInRight`, `OvertakeLeft`,
    /// `HeadOn`) also mandates at least one `lane_changes` entry, which
    /// drives `|py1 - py2|` towards 0 regardless of this constraint — so a
    /// corpus example cannot isolate this defect from that unrelated
    /// convergence (see the SW-30 report for the sweep this comes from).
    #[test]
    fn test_lateral_distance_violate_mode_is_strict_at_the_boundary() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            use crate::ltl::formula::{LTLFormula, Proposition};

            let spec = create_two_actor_diff_lane_spec();
            let mut encoder = Z3Encoder::new(spec);
            encoder.create_variables();
            encoder.encode_initial_conditions();

            // ego (lane 1, py=5.25) and npc (lane 0, py=1.75) are exactly
            // 3.5 m apart — the road's lane_width — at every time step.
            let boundary = 3.5;

            // `Violate` asserts the negation of `LateralDistanceGT`. Before
            // SW-30 that negation was `|py1 - py2| <= boundary`, satisfied by
            // the fixed 3.5 m gap exactly.
            let formula = LTLFormula::Atom(Proposition::LateralDistanceGT {
                actor1: "ego".to_string(),
                actor2: "npc".to_string(),
                distance: boundary,
            })
            .negate();
            encoder.encode_ltl(&formula);

            // Fixed: the negation is now `|py1 - py2| < boundary`, strict,
            // which the rigid 3.5 m gap cannot satisfy — correctly UNSAT,
            // rather than accepting the boundary as a violation the
            // validator would go on to call safe.
            assert_eq!(
                encoder.check(),
                SatResult::Unsat,
                "SW-30: violate mode must not be satisfiable by the exact \
                 threshold distance ({boundary:.4} m) — that is the boundary \
                 the validator calls safe, not a breach"
            );
        });
    }

    /// SW-35. `PedestrianTTCGT` lowers to a guarded implication,
    /// `ped_on_road ∧ approaching ⟹ ttc_safe`, and `ttc_safe` was the
    /// strict `distance > ttc * ego_vx` (encoder.rs `encode_proposition`).
    /// The guard changes the *shape* of the negation relative to SW-12
    /// (`DistanceGT`) and SW-30 (`LateralDistanceGT`), which negate plain
    /// comparisons, but not the conclusion: `¬(A ⟹ B)` is `A ∧ ¬B`, and the
    /// antecedent `A` (`ped_on_road ∧ approaching`) is untouched by the
    /// strictness of `B` (`ttc_safe`) — so once `A` holds, whether the
    /// negation is satisfiable at the boundary reduces to exactly the same
    /// question SW-12/SW-30 answered: is `¬ttc_safe` `<=` (non-strict,
    /// satisfiable at the boundary) or `<` (strict, not)?
    ///
    /// `compute_validation_metrics` (encoder.rs, the `pedestrian_pair`
    /// branch) flags a TTC violation only at `ttc < min_ttc - METRIC_TOL`,
    /// i.e. it calls `ttc == min_ttc` safe. Pre-fix, `ttc_safe` was strict
    /// `>`, so `Violate`'s negation `¬ttc_safe` was the non-strict
    /// `distance <= ttc * ego_vx`, satisfiable at exactly `distance == ttc *
    /// ego_vx` — the point the validator calls safe. Same defect, third
    /// instance, different proposition shape.
    ///
    /// Every quantity here is pinned by `ValueOrRange::Value`, so the
    /// antecedent (`ped_on_road`, `ego_behind`, `ego_moving_forward`) is true
    /// by construction and the boundary is exact, with no solver freedom to
    /// launder the result: ego at `px=0`, `vx=10`; pedestrian at `px=20`,
    /// lane 0 (so `py = 1.75`, inside the 3.5 m road); `min_ttc = 2.0`, so
    /// `distance (20) == ttc * ego_vx (2.0 * 10)` exactly.
    #[test]
    fn test_pedestrian_ttc_violate_mode_is_strict_at_the_boundary() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            use crate::ltl::formula::{LTLFormula, Proposition};

            let spec = ScenarioSpec {
                scenario_type: ScenarioType::PedestrianCrossing,
                time_step: 1.0,
                duration: 1.0,
                actors: vec![
                    ActorSpec {
                        id: "ego".to_string(),
                        role: ActorRole::Ego,
                        lane: 0,
                        position: ValueOrRange::Value(0.0),
                        speed: ValueOrRange::Value(10.0),
                        acceleration: ValueOrRange::Range([-3.0, 2.0]),
                        direction: 1,
                        behavior: HashMap::new(),
                        lane_changes: vec![],
                        bicycle_params: None,
                    },
                    ActorSpec {
                        id: "pedestrian".to_string(),
                        role: ActorRole::Pedestrian,
                        lane: 0,
                        position: ValueOrRange::Value(20.0),
                        speed: ValueOrRange::Value(1.0),
                        acceleration: ValueOrRange::Range([-1.0, 1.0]),
                        direction: 1,
                        behavior: HashMap::new(),
                        lane_changes: vec![],
                        bicycle_params: None,
                    },
                ],
                min_ttc: 2.0,
                min_distance: 2.0,
                road: Some(RoadSpec {
                    num_lanes: 1,
                    lane_width: 3.5,
                    lane_directions: vec![1],
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
            };

            let mut encoder = Z3Encoder::new(spec);
            encoder.create_variables();
            encoder.encode_initial_conditions();

            // `Violate` asserts the negation of `PedestrianTTCGT`. Before
            // SW-35 that negation's `¬ttc_safe` half was
            // `distance <= ttc * ego_vx`, satisfied by the pinned boundary
            // (`20 <= 2.0 * 10`) exactly.
            let formula = LTLFormula::Atom(Proposition::PedestrianTTCGT {
                ego: "ego".to_string(),
                pedestrian: "pedestrian".to_string(),
                ttc: 2.0,
            })
            .negate();
            encoder.encode_ltl(&formula);

            // Fixed: `¬ttc_safe` is now `distance < ttc * ego_vx`, strict,
            // which the pinned boundary (`20 == 2.0 * 10`) cannot satisfy —
            // correctly UNSAT, rather than accepting the exact threshold as
            // a violation the validator would go on to call safe.
            assert_eq!(
                encoder.check(),
                SatResult::Unsat,
                "SW-35: violate mode must not be satisfiable by the exact \
                 TTC boundary — that is the boundary the validator calls \
                 safe, not a breach"
            );
        });
    }

    #[test]
    fn test_proposition_on_left_of() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            use crate::ltl::formula::{LTLFormula, Proposition};

            let spec = create_two_actor_diff_lane_spec();
            let mut encoder = Z3Encoder::new(spec);
            encoder.create_variables();
            encoder.encode_initial_conditions();

            // Encode: OnLeftOf(ego, npc) — ego.py > npc.py
            let formula = LTLFormula::Atom(Proposition::OnLeftOf {
                actor1: "ego".to_string(),
                actor2: "npc".to_string(),
            });
            encoder.encode_ltl(&formula);

            assert_eq!(encoder.check(), SatResult::Sat);

            let model = encoder.get_model().unwrap();
            let py_ego = model.eval(encoder.get_lateral_pos("ego", 0), true).unwrap();
            let py_npc = model.eval(encoder.get_lateral_pos("npc", 0), true).unwrap();

            let ego_lat: f64 = crate::solver::backend::parse_z3_real_pub(&py_ego.to_string());
            let npc_lat: f64 = crate::solver::backend::parse_z3_real_pub(&py_npc.to_string());
            assert!(
                ego_lat > npc_lat,
                "Ego py ({}) should be > NPC py ({})",
                ego_lat,
                npc_lat
            );
        });
    }

    #[test]
    fn test_proposition_on_right_of() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            use crate::ltl::formula::{LTLFormula, Proposition};

            let spec = create_two_actor_diff_lane_spec();
            let mut encoder = Z3Encoder::new(spec);
            encoder.create_variables();
            encoder.encode_initial_conditions();

            // Encode: OnRightOf(npc, ego) — npc.py < ego.py
            // npc is in lane 0 (py=1.75), ego in lane 1 (py=5.25)
            let formula = LTLFormula::Atom(Proposition::OnRightOf {
                actor1: "npc".to_string(),
                actor2: "ego".to_string(),
            });
            encoder.encode_ltl(&formula);

            assert_eq!(encoder.check(), SatResult::Sat);

            let model = encoder.get_model().unwrap();
            let py_ego = model.eval(encoder.get_lateral_pos("ego", 0), true).unwrap();
            let py_npc = model.eval(encoder.get_lateral_pos("npc", 0), true).unwrap();

            let ego_lat: f64 = crate::solver::backend::parse_z3_real_pub(&py_ego.to_string());
            let npc_lat: f64 = crate::solver::backend::parse_z3_real_pub(&py_npc.to_string());
            assert!(
                npc_lat < ego_lat,
                "NPC py ({}) should be < Ego py ({})",
                npc_lat,
                ego_lat
            );
        });
    }

    #[test]
    fn test_proposition_relative_velocity_gt() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            use crate::ltl::formula::{LTLFormula, Proposition};

            // ego speed=20, npc speed=15, so |20-15|=5 > 3
            let spec = create_two_actor_same_lane_spec();
            let mut encoder = Z3Encoder::new(spec);
            encoder.create_variables();
            encoder.encode_initial_conditions();

            let formula = LTLFormula::Atom(Proposition::RelativeVelocityGT {
                actor1: "ego".to_string(),
                actor2: "npc".to_string(),
                velocity: 3.0,
            });
            encoder.encode_ltl(&formula);

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
            let rel_vel = (ego_v - npc_v).abs();
            assert!(
                rel_vel > 3.0,
                "Relative velocity {} should be > 3.0",
                rel_vel
            );
        });
    }

    #[test]
    fn test_proposition_manhattan_distance_gt() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            use crate::ltl::formula::{LTLFormula, Proposition};

            let spec = create_two_actor_diff_lane_spec();
            let mut encoder = Z3Encoder::new(spec);
            encoder.create_variables();
            encoder.encode_initial_conditions();

            // ego at px=50, npc at px=100, different lanes → manhattan should be large
            let formula = LTLFormula::Atom(Proposition::ManhattanDistanceGT {
                actor1: "ego".to_string(),
                actor2: "npc".to_string(),
                distance: 40.0,
            });
            encoder.encode_ltl(&formula);

            assert_eq!(encoder.check(), SatResult::Sat);

            let model = encoder.get_model().unwrap();
            let px_ego = model
                .eval(encoder.get_longitudinal_pos("ego", 0), true)
                .unwrap();
            let py_ego = model.eval(encoder.get_lateral_pos("ego", 0), true).unwrap();
            let px_npc = model
                .eval(encoder.get_longitudinal_pos("npc", 0), true)
                .unwrap();
            let py_npc = model.eval(encoder.get_lateral_pos("npc", 0), true).unwrap();

            let ex: f64 = crate::solver::backend::parse_z3_real_pub(&px_ego.to_string());
            let ey: f64 = crate::solver::backend::parse_z3_real_pub(&py_ego.to_string());
            let nx: f64 = crate::solver::backend::parse_z3_real_pub(&px_npc.to_string());
            let ny: f64 = crate::solver::backend::parse_z3_real_pub(&py_npc.to_string());
            let manhattan = (ex - nx).abs() + (ey - ny).abs();
            assert!(
                manhattan > 40.0,
                "Manhattan distance {} should be > 40.0",
                manhattan
            );
        });
    }

    #[test]
    fn test_proposition_rectangular_distance_gt() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            use crate::ltl::formula::{LTLFormula, Proposition};

            let spec = create_two_actor_diff_lane_spec();
            let mut encoder = Z3Encoder::new(spec);
            encoder.create_variables();
            encoder.encode_initial_conditions();

            let formula = LTLFormula::Atom(Proposition::RectangularDistanceGT {
                actor1: "ego".to_string(),
                actor2: "npc".to_string(),
                threshold_x: 30.0,
                threshold_y: 2.0,
            });
            encoder.encode_ltl(&formula);

            assert_eq!(encoder.check(), SatResult::Sat);

            let model = encoder.get_model().unwrap();
            let px_ego = model
                .eval(encoder.get_longitudinal_pos("ego", 0), true)
                .unwrap();
            let py_ego = model.eval(encoder.get_lateral_pos("ego", 0), true).unwrap();
            let px_npc = model
                .eval(encoder.get_longitudinal_pos("npc", 0), true)
                .unwrap();
            let py_npc = model.eval(encoder.get_lateral_pos("npc", 0), true).unwrap();

            let ex: f64 = crate::solver::backend::parse_z3_real_pub(&px_ego.to_string());
            let ey: f64 = crate::solver::backend::parse_z3_real_pub(&py_ego.to_string());
            let nx: f64 = crate::solver::backend::parse_z3_real_pub(&px_npc.to_string());
            let ny: f64 = crate::solver::backend::parse_z3_real_pub(&py_npc.to_string());
            let dx = (ex - nx).abs();
            let dy = (ey - ny).abs();
            assert!(
                dx > 30.0 || dy > 2.0,
                "Rectangular: |dx|={} should be > 30 OR |dy|={} should be > 2",
                dx,
                dy
            );
        });
    }

    /// SW-33. Found while checking `RectangularDistanceGT`'s negation is not
    /// vacuous for `Violate` — same SW-12/SW-30/SW-35 boundary shape,
    /// unnamed in the original issue. The box's `dx`/`dy` comparisons were
    /// strict `>`/`<`, so `.negate()` (De Morgan) gave the non-strict
    /// conjunction `dx <= tx AND dy <= ty`, satisfiable with `dx == tx`
    /// exactly — a point `compute_validation_metrics`'s
    /// `box_safe = dx > tx - METRIC_TOL || ...` calls safe.
    ///
    /// Both actors pinned to lane 0 (so `dy = 0`, comfortably inside `ty`)
    /// and `px` pinned by `ValueOrRange::Value` so `dx == threshold_x`
    /// exactly, with no solver freedom: `min_distance = 2.0` gives
    /// `threshold_x = 1.0` (`PEDESTRIAN_BOX_LONGITUDINAL_DIVISOR`), and
    /// `ego.px = 0.0`, `pedestrian.px = 1.0`.
    #[test]
    fn test_rectangular_distance_violate_mode_is_strict_at_the_boundary() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            use crate::ltl::formula::{LTLFormula, Proposition};

            let spec = ScenarioSpec {
                scenario_type: ScenarioType::PedestrianCrossing,
                time_step: 1.0,
                duration: 1.0,
                actors: vec![
                    ActorSpec {
                        id: "ego".to_string(),
                        role: ActorRole::Ego,
                        lane: 0,
                        position: ValueOrRange::Value(0.0),
                        speed: ValueOrRange::Value(10.0),
                        acceleration: ValueOrRange::Range([-3.0, 2.0]),
                        direction: 1,
                        behavior: HashMap::new(),
                        lane_changes: vec![],
                        bicycle_params: None,
                    },
                    ActorSpec {
                        id: "pedestrian".to_string(),
                        role: ActorRole::Pedestrian,
                        lane: 0,
                        position: ValueOrRange::Value(1.0),
                        speed: ValueOrRange::Value(1.0),
                        acceleration: ValueOrRange::Range([-1.0, 1.0]),
                        direction: 1,
                        behavior: HashMap::new(),
                        lane_changes: vec![],
                        bicycle_params: None,
                    },
                ],
                min_ttc: 2.0,
                min_distance: 2.0,
                road: Some(RoadSpec {
                    num_lanes: 1,
                    lane_width: 3.5,
                    lane_directions: vec![1],
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
            };

            let mut encoder = Z3Encoder::new(spec);
            encoder.create_variables();
            encoder.encode_initial_conditions();

            // `Violate` asserts the negation of `RectangularDistanceGT`.
            // Before SW-33's boundary fix, that negation was
            // `dx <= 1.0 AND dy <= 1.333`, satisfied by the pinned boundary
            // (`dx == 1.0` exactly, `dy == 0`).
            let formula = LTLFormula::Atom(Proposition::RectangularDistanceGT {
                actor1: "ego".to_string(),
                actor2: "pedestrian".to_string(),
                threshold_x: 1.0,
                threshold_y: 2.0 / 1.5,
            })
            .negate();
            encoder.encode_ltl(&formula);

            // Fixed: the negation is now `dx < 1.0 AND dy < 1.333`, strict,
            // which the pinned `dx == 1.0` cannot satisfy — correctly
            // UNSAT, rather than accepting the exact threshold as a
            // violation the validator would go on to call safe.
            assert_eq!(
                encoder.check(),
                SatResult::Unsat,
                "SW-33: violate mode must not be satisfiable by the exact \
                 box boundary — that is the boundary the validator calls \
                 safe, not a breach"
            );
        });
    }

    // ===== Group 5: Edge cases =====

    #[test]
    fn test_ltl_next_at_horizon_boundary() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            use crate::ltl::formula::{LTLFormula, Proposition};

            // Create a very short scenario (2 time steps: horizon=2)
            let mut spec = create_two_actor_same_lane_spec();
            spec.duration = 1.0;
            spec.time_step = 0.5; // horizon = 2

            let mut encoder = Z3Encoder::new(spec);
            encoder.create_variables();
            encoder.encode_initial_conditions();

            // Encode: X(X(X(InLane(ego, 1)))) with horizon = 2, so the
            // innermost X is taken at the horizon and has no next state.
            let inner = LTLFormula::Atom(Proposition::InLane {
                actor: "ego".to_string(),
                lane: 1,
            });
            let formula = LTLFormula::Next(Box::new(LTLFormula::Next(Box::new(LTLFormula::Next(
                Box::new(inner),
            )))));
            encoder.encode_ltl(&formula);

            // SW-12/L1. This test asserted `Unsat`, which is the defect rather
            // than the specification: `X` past the bound yielded the literal
            // `false`, and because `Always` expands over `time..=horizon`
            // *inclusive*, every `G(X phi)` was unsatisfiable no matter what
            // `phi` said — see `test_always_next_is_satisfiable` below.
            // Bounded model checking knows nothing about states after the
            // bound, so the honest reading is "not refuted by this trace".
            assert_eq!(
                encoder.check(),
                SatResult::Sat,
                "X past the horizon must not refute the formula by itself"
            );
        });
    }

    /// `G(X phi)` must be satisfiable (SW-12/L1).
    ///
    /// The direct consequence of the bug above: `Always` expands over
    /// `time..=horizon` inclusive, so its last conjunct is `X` evaluated *at*
    /// the horizon. With `X` past the bound lowered to `false`, that conjunct
    /// was the literal `false` and every `G(X phi)` in the language was
    /// unsatisfiable regardless of `phi`. Nothing in the shipped scenario
    /// templates uses `X` today, which is why this went unnoticed.
    #[test]
    fn test_always_next_is_satisfiable() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            use crate::ltl::formula::{LTLFormula, Proposition};

            let mut spec = create_two_actor_same_lane_spec();
            spec.duration = 1.0;
            spec.time_step = 0.5; // horizon = 2

            let ego_lane = spec.ego().unwrap().lane;
            let mut encoder = Z3Encoder::new(spec);
            encoder.create_variables();
            encoder.encode_initial_conditions();

            let formula = LTLFormula::Atom(Proposition::InLane {
                actor: "ego".to_string(),
                lane: ego_lane,
            })
            .next()
            .always();
            encoder.encode_ltl(&formula);

            assert_eq!(
                encoder.check(),
                SatResult::Sat,
                "G(X phi) must be satisfiable; it was unsat for every phi before SW-12/L1"
            );
        });
    }

    // SW-41 -----------------------------------------------------------------
    //
    // The durable check for the SW-12/30/33/35 defect shape: an encoder
    // comparison lowered strictly where `compute_validation_metrics` tests it
    // non-strictly, so `Violate` mode can satisfy its negation exactly on the
    // boundary (`distance == threshold`) and the validator — testing `>=` —
    // reports no violation. Every `Measured` proposition (per
    // `strictness_coverage`) gets one boundary setup below: pin every free
    // variable the proposition reads to an exact value that puts the raw
    // quantity (a distance, a TTC, a relative speed) exactly at its
    // threshold, then assert the formula `Violate` would actually use — the
    // *unsafe* one, `atom.negate()` for `AtomPolarity::Positive` propositions
    // and the bare atom for `AtomPolarity::Negated` ones (`RelativeVelocityGT`,
    // see `scenarios::mod::AtomPolarity`) — and require it UNSAT. If it is
    // satisfiable, the encoder let `Violate` land exactly where the validator
    // calls the state safe.
    //
    // `test_strictness_coverage_is_exhaustive_and_documented` is the other
    // half: it pins down, per variant, whether this test below applies at
    // all — so a 13th `Proposition` variant either gets a `Measured` boundary
    // case here or is explicitly filed as `Unmeasured`/`NotApplicable`,
    // never silently skipped.

    /// Pin every per-actor Z3 variable this file's boundary tests read to an
    /// exact concrete value at `time`. Deliberately does not call
    /// `encode_initial_conditions`/`encode_kinematics`: the boundary tests
    /// want *only* the values below constrained, nothing else, so a
    /// proposition that (incorrectly) read a variable this helper did not
    /// pin would leave it free rather than silently inheriting some other
    /// encoder's defaults.
    #[allow(clippy::too_many_arguments)]
    fn pin_actor(
        encoder: &mut Z3Encoder,
        actor: &str,
        time: usize,
        lane: i64,
        px: f64,
        py: f64,
        vx: f64,
    ) {
        let lane_eq = encoder
            .get_lane_var(actor, time)
            .eq(&Int::from_i64(lane));
        let px_eq = encoder.get_longitudinal_pos(actor, time).eq(&real_from_f64(px));
        let py_eq = encoder.get_lateral_pos(actor, time).eq(&real_from_f64(py));
        let vx_eq = encoder.get_longitudinal_vel(actor, time).eq(&real_from_f64(vx));
        encoder.assert_constraint(&lane_eq);
        encoder.assert_constraint(&px_eq);
        encoder.assert_constraint(&py_eq);
        encoder.assert_constraint(&vx_eq);
    }

    /// The `Violate`-mode formula for a `Measured` proposition: the atom's
    /// negation for `AtomPolarity::Positive`, the bare atom for
    /// `AtomPolarity::Negated`. Mirrors `scenarios::mod::push_constraint`'s
    /// `(ConstraintMode::Violate, polarity)` arms exactly (minus the
    /// `.eventually()`, irrelevant to a single-step ground check).
    fn violate_formula(atom: z3::ast::Bool, negated_polarity: bool) -> z3::ast::Bool {
        if negated_polarity {
            atom
        } else {
            atom.not()
        }
    }

    #[test]
    fn test_measured_propositions_agree_with_validator_at_the_boundary() {
        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            // (name, proposition, negated_polarity, pins)
            struct Case {
                name: &'static str,
                prop: Proposition,
                negated_polarity: bool,
                pins: Vec<(&'static str, i64, f64, f64, f64)>, // actor, lane, px, py, vx
            }

            let cases = vec![
                // DistanceGT (min_distance, Positive). Same lane, |px1-px2|
                // exactly at the threshold.
                Case {
                    name: "DistanceGT",
                    prop: Proposition::DistanceGT {
                        actor1: "ego".to_string(),
                        actor2: "npc".to_string(),
                        distance: 5.0,
                    },
                    negated_polarity: false,
                    pins: vec![("ego", 0, 105.0, 0.0, 0.0), ("npc", 0, 100.0, 0.0, 0.0)],
                },
                // TTCGT (min_ttc, Positive). Same lane, ego ahead by 9 m,
                // npc closing at 3 m/s: TTC = 9/3 = 3.0 s exactly.
                Case {
                    name: "TTCGT",
                    prop: Proposition::TTCGT {
                        actor1: "ego".to_string(),
                        actor2: "npc".to_string(),
                        ttc: 3.0,
                    },
                    negated_polarity: false,
                    pins: vec![("ego", 0, 109.0, 0.0, 10.0), ("npc", 0, 100.0, 0.0, 13.0)],
                },
                // LateralDistanceGT (min_lateral_distance, Positive).
                // Unguarded; |py1-py2| exactly at the threshold.
                Case {
                    name: "LateralDistanceGT",
                    prop: Proposition::LateralDistanceGT {
                        actor1: "ego".to_string(),
                        actor2: "npc".to_string(),
                        distance: 2.0,
                    },
                    negated_polarity: false,
                    pins: vec![("ego", 0, 0.0, 2.0, 0.0), ("npc", 0, 0.0, 0.0, 0.0)],
                },
                // RelativeVelocityGT (max_relative_velocity, Negated — the
                // atom names the unsafe condition). |vx1-vx2| exactly at the
                // threshold.
                Case {
                    name: "RelativeVelocityGT",
                    prop: Proposition::RelativeVelocityGT {
                        actor1: "ego".to_string(),
                        actor2: "npc".to_string(),
                        velocity: 5.0,
                    },
                    negated_polarity: true,
                    pins: vec![("ego", 0, 0.0, 0.0, 15.0), ("npc", 0, 0.0, 0.0, 10.0)],
                },
                // RectangularDistanceGT (pedestrian box, Positive). dx
                // exactly at threshold_x, dy well inside threshold_y.
                Case {
                    name: "RectangularDistanceGT",
                    prop: Proposition::RectangularDistanceGT {
                        actor1: "ego".to_string(),
                        actor2: "npc".to_string(),
                        threshold_x: 1.0,
                        threshold_y: 1.5,
                    },
                    negated_polarity: false,
                    pins: vec![("ego", 0, 1.0, 0.0, 0.0), ("npc", 0, 0.0, 0.0, 0.0)],
                },
                // PedestrianTTCGT (min_ttc, pedestrian pair, Positive). Ego
                // (misused here as the vehicle role) 10 m behind the
                // pedestrian, closing at 5 m/s: TTC = 10/5 = 2.0 s exactly.
                // Pedestrian on the road (0 <= py <= road_width = 7.0).
                Case {
                    name: "PedestrianTTCGT",
                    prop: Proposition::PedestrianTTCGT {
                        ego: "ego".to_string(),
                        pedestrian: "npc".to_string(),
                        ttc: 2.0,
                    },
                    negated_polarity: false,
                    pins: vec![("ego", 0, 90.0, 0.0, 5.0), ("npc", 0, 100.0, 3.5, 0.0)],
                },
                // VelocityGT (min_velocity, Positive). |vx| exactly at the
                // threshold.
                Case {
                    name: "VelocityGT",
                    prop: Proposition::VelocityGT {
                        actor: "ego".to_string(),
                        velocity: 10.0,
                    },
                    negated_polarity: false,
                    pins: vec![("ego", 0, 0.0, 0.0, 10.0)],
                },
                // VelocityLT (max_velocity, Positive). |vx| exactly at the
                // threshold.
                Case {
                    name: "VelocityLT",
                    prop: Proposition::VelocityLT {
                        actor: "ego".to_string(),
                        velocity: 10.0,
                    },
                    negated_polarity: false,
                    pins: vec![("ego", 0, 0.0, 0.0, 10.0)],
                },
            ];

            for case in cases {
                assert_eq!(
                    strictness_coverage(&case.prop),
                    StrictnessCoverage::Measured,
                    "{}: test case is for a proposition `strictness_coverage` does not call \
                     Measured — fix the classification or the test case",
                    case.name
                );

                let spec = create_two_actor_same_lane_spec();
                let mut encoder = Z3Encoder::new(spec);
                encoder.create_variables();

                for (actor, lane, px, py, vx) in &case.pins {
                    pin_actor(&mut encoder, actor, 0, *lane, *px, *py, *vx);
                }

                let atom = encode_proposition(&encoder, &encoder.spec, &case.prop, 0);
                let violate = violate_formula(atom, case.negated_polarity);
                encoder.assert_constraint(&violate);

                assert_eq!(
                    encoder.check(),
                    SatResult::Unsat,
                    "{}: Violate settled exactly on the boundary the validator calls safe \
                     (SAT when it must be UNSAT) — the encoder's comparison is strict where \
                     compute_validation_metrics tests it non-strictly",
                    case.name
                );
            }
        });
    }

    /// Every `Proposition` variant must be classified — this is the
    /// compiler-enforced half of the invariant. If this test compiles, the
    /// `match` in `strictness_coverage` has no wildcard arm covering it, so a
    /// 13th variant added without a corresponding line here is a build
    /// failure, not a silent gap. The assertions pin down *today's*
    /// classification so a change to it is a deliberate, reviewed edit.
    #[test]
    fn test_strictness_coverage_is_exhaustive_and_documented() {
        use StrictnessCoverage::{Measured, NotApplicable, Unmeasured};

        let s = |s: &str| s.to_string();
        let cases: Vec<(Proposition, StrictnessCoverage)> = vec![
            (
                Proposition::InLane {
                    actor: s("a"),
                    lane: 0,
                },
                NotApplicable,
            ),
            (
                Proposition::Ahead {
                    actor1: s("a"),
                    actor2: s("b"),
                },
                NotApplicable,
            ),
            (
                Proposition::DistanceGT {
                    actor1: s("a"),
                    actor2: s("b"),
                    distance: 1.0,
                },
                Measured,
            ),
            (
                Proposition::TTCGT {
                    actor1: s("a"),
                    actor2: s("b"),
                    ttc: 1.0,
                },
                Measured,
            ),
            (
                Proposition::OnSidewalk {
                    actor: s("a"),
                    side: s("left"),
                },
                Unmeasured,
            ),
            (Proposition::CrossingRoad { actor: s("a") }, Unmeasured),
            (
                Proposition::Distance2DGT {
                    actor1: s("a"),
                    actor2: s("b"),
                    distance: 1.0,
                },
                Unmeasured,
            ),
            (
                Proposition::ManhattanDistanceGT {
                    actor1: s("a"),
                    actor2: s("b"),
                    distance: 1.0,
                },
                Unmeasured,
            ),
            (
                Proposition::RectangularDistanceGT {
                    actor1: s("a"),
                    actor2: s("b"),
                    threshold_x: 1.0,
                    threshold_y: 1.0,
                },
                Measured,
            ),
            (
                Proposition::PedestrianTTCGT {
                    ego: s("a"),
                    pedestrian: s("b"),
                    ttc: 1.0,
                },
                Measured,
            ),
            (
                Proposition::VelocityGT {
                    actor: s("a"),
                    velocity: 1.0,
                },
                Measured,
            ),
            (
                Proposition::VelocityLT {
                    actor: s("a"),
                    velocity: 1.0,
                },
                Measured,
            ),
            (
                Proposition::LateralDistanceGT {
                    actor1: s("a"),
                    actor2: s("b"),
                    distance: 1.0,
                },
                Measured,
            ),
            (
                Proposition::OnLeftOf {
                    actor1: s("a"),
                    actor2: s("b"),
                },
                NotApplicable,
            ),
            (
                Proposition::OnRightOf {
                    actor1: s("a"),
                    actor2: s("b"),
                },
                NotApplicable,
            ),
            (
                Proposition::Approaching {
                    follower: s("a"),
                    leader: s("b"),
                },
                NotApplicable,
            ),
            (
                Proposition::RelativeVelocityGT {
                    actor1: s("a"),
                    actor2: s("b"),
                    velocity: 1.0,
                },
                Measured,
            ),
        ];

        for (prop, expected) in &cases {
            assert_eq!(
                strictness_coverage(prop),
                *expected,
                "{prop:?} classified as {:?}, expected {:?}",
                strictness_coverage(prop),
                expected
            );
        }

        let measured_count = cases
            .iter()
            .filter(|(_, c)| *c == Measured)
            .count();
        assert_eq!(
            measured_count, 8,
            "expected exactly 8 Measured propositions today (DistanceGT, TTCGT, \
             LateralDistanceGT, RelativeVelocityGT, RectangularDistanceGT, \
             PedestrianTTCGT, VelocityGT, VelocityLT) — SW-31 reported 3; if this \
             assertion fails because that number changed, update it deliberately, \
             not by deleting the assertion"
        );
    }
}
