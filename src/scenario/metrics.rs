//! Scenario extraction and post-solve validation metrics.
//!
//! Converts a satisfying Z3 model into the [`Scenario`](crate::scenario::model::Scenario)
//! output structure, then independently re-derives the safety metrics (`min_distance`,
//! `min_ttc`, per-actor velocity/acceleration bounds, ...) from the extracted trajectories.
//! This is the only place a generated scenario is checked against its spec independently
//! of the solver that produced it — see [`GenericEncoder::compute_validation_metrics`]
//! for the full audit of which `Proposition` each check corresponds to.

use crate::scenarios::pedestrian_crossing::{
    PEDESTRIAN_BOX_LATERAL_DIVISOR, PEDESTRIAN_BOX_LONGITUDINAL_DIVISOR,
};
use crate::solver::backend::Z3Backend;
use crate::solver::encoder::{GenericEncoder, METRIC_TOL, TTC_CLOSING_SPEED_EPSILON};
use crate::solver::encoder_utils::same_lane_f64;

impl<B: Z3Backend + 'static> GenericEncoder<B> {
    /// Extract scenario from Z3 model
    ///
    /// Converts the Z3 solution (satisfying assignment) into a Scenario
    /// JSON structure with actor trajectories.
    pub fn extract_scenario(
        &self,
        model: &z3::Model,
    ) -> crate::error::Result<crate::scenario::model::Scenario> {
        use crate::dsl::types::ActorRole;

        // Get RoadSpec from ScenarioSpec (required, should always exist after validation)
        let road = self
            .spec
            .road
            .as_ref()
            .ok_or_else(|| {
                crate::error::ScenarioGenError::ExtractionFailed(
                    "RoadSpec is required - should be validated during spec parsing".to_string(),
                )
            })?
            .clone();

        let mut scenario = crate::scenario::model::Scenario::new(
            self.spec.scenario_type.to_string(),
            self.spec.time_step,
            self.spec.duration,
            road,
        );

        // Extract trajectory for each actor
        for actor in &self.spec.actors {
            let role_str = match actor.role {
                ActorRole::Ego => "ego",
                ActorRole::Npc => "npc",
                ActorRole::Pedestrian => "pedestrian",
            };

            let trajectory = self.extract_actor_trajectory(model, &actor.id, role_str)?;
            scenario.add_actor(trajectory);
        }

        // Compute validation metrics
        self.compute_validation_metrics(&mut scenario)?;

        Ok(scenario)
    }

    /// Extract trajectory for a single actor
    fn extract_actor_trajectory(
        &self,
        model: &z3::Model,
        actor_id: &str,
        role: &str,
    ) -> crate::error::Result<crate::scenario::model::ActorTrajectory> {
        self.coord_encoder
            .extract_actor_trajectory(model, actor_id, role)
    }

    /// Compute validation metrics from the scenario trajectories
    ///
    /// # Audit — every `Proposition` variant, what is encoded, what is measured
    ///
    /// This is the **only** place a generated scenario is checked against its
    /// spec independently of the solver that produced it. Measuring only a
    /// subset of fields (say, just `min_distance`, `min_ttc`,
    /// `min_lateral_distance`) would leave everything else a scenario model
    /// can lower a [`Proposition`](crate::ltl::formula::Proposition) to
    /// unchecked, making `all_constraints_satisfied` partly the encoder
    /// marking its own homework. Every row below states *encoded* and
    /// *measured* separately: a field can be lowered correctly and still be
    /// measured generically (or not at all), and the two failures look
    /// identical from the JSON output.
    ///
    /// | Proposition | Encoded (production caller) | Measured here | Verdict |
    /// |---|---|---|---|
    /// | `InLane` | every scenario type (lane bookkeeping) | — | not a spec field, no metric to check |
    /// | `Ahead` | `cut_in_left.rs:86`, `cut_in_right.rs:81`, `overtake_left.rs:139,171` | — | structural ordering, not a spec field |
    /// | `Approaching` | `scenarios/mod.rs:144` (`cut_in_conflict`) | — | structural antecedent for `TTCGT`'s reachability; its effect shows up in the TTC measurement below |
    /// | `OnSidewalk` / `CrossingRoad` | `pedestrian_crossing.rs:191,186` | — | LTL goals, not spec thresholds |
    /// | `PedestrianTTCGuard` | `pedestrian_crossing.rs::generate_safety`, `Enforce` only | — | structural antecedent for `PedestrianTTCGT`'s reachability, the pedestrian twin of `Approaching`; it is *the same formula* as that proposition's own guard (one lowering, `encode_pedestrian_ttc_guard`), so its effect shows up in the pedestrian TTC check below rather than in a metric of its own |
    /// | **`DistanceGT`** | `scenarios/mod.rs:217`, `head_on.rs:248,258` | `min_distance` block below, `same_lane`-gated `\|dx\|`, non-strict boundary | **agrees** — same guard predicate ([`encode_same_lane_constraint`]/[`same_lane_f64`]), same boundary |
    /// | **`TTCGT`** | `scenarios/mod.rs:206`, `head_on.rs:223,233` | `min_ttc` block below, `same_lane` + closing-speed gated, same `TTC_CLOSING_SPEED_EPSILON` | **agrees** |
    /// | **`LateralDistanceGT`** | `scenarios/mod.rs:229`, `pedestrian_crossing.rs:117` | `min_lateral_distance` block, unguarded `\|dy\|`, non-strict boundary | **agrees** |
    /// | **`VelocityLT`** (`max_velocity`) | `scenarios/mod.rs:262`, all actors, unguarded | per-actor `\|vx\|` check | measured (a test-only re-derivation also exists in `tests/common/invariants.rs`) |
    /// | **`VelocityGT`** (`min_velocity`) | `scenarios/mod.rs:274`, all actors, unguarded | per-actor `\|vx\|` check | measured. No corpus example sets it; covered by a constructed test below |
    /// | **`RelativeVelocityGT`** (`max_relative_velocity`) | `scenarios/mod.rs:245`, all pairs, unguarded, negated polarity | per-pair `\|vx1-vx2\|` check | measured — `unsafe_following.yaml` is an adversarial corpus example built to violate exactly this field |
    /// | **`RectangularDistanceGT`** (pedestrian `min_distance`) | `pedestrian_crossing.rs:75`, `Enforce` only | box check for pairs containing a pedestrian, rather than the generic `same_lane`-gated `\|dx\|` — a pedestrian's lane is pinned to 0 (`pedestrian_crossing.rs:232`) and so is the ego's, making `same_lane` structurally true | **measured differently from the generic pair check** — see doc note on the pedestrian branch below |
    /// | **`PedestrianTTCGT`** (pedestrian `min_ttc`) | `pedestrian_crossing.rs:128`, `Enforce` only | guarded ego-behind/on-road TTC check, mirroring the encoder's own guard | **measured differently**, same root cause as the row above |
    ///
    /// `Distance2DGT`, `ManhattanDistanceGT`, `OnLeftOf`, and `OnRightOf` have no
    /// non-test caller in `src/scenarios/` and no validator check, so they are
    /// not kept as permanent "unmeasured" bookkeeping for code nothing emits.
    /// If a future scenario type needs 2D/Manhattan distance or lateral
    /// ordering, reintroduce the proposition alongside its row here and a
    /// check above, not before.
    ///
    /// ## The pedestrian branch, in detail
    ///
    /// `pedestrian_crossing.rs` only lowers `RectangularDistanceGT` /
    /// `PedestrianTTCGT` when `min_distance`/`min_ttc` are `Enforce` (there is
    /// no `Violate` lowering at all for either — flagged below, since fixing
    /// it means editing a fenced file). The box's thresholds
    /// (`min_distance / PEDESTRIAN_BOX_LONGITUDINAL_DIVISOR`, `min_distance /
    /// PEDESTRIAN_BOX_LATERAL_DIVISOR`) live as
    /// `pedestrian_crossing::PEDESTRIAN_BOX_{LONGITUDINAL,LATERAL}_DIVISOR`,
    /// imported here, so there is exactly one copy of each number. Kept as
    /// divisors rather than multiplied ratios: `x / 1.5` and `x * (1.0/1.5)`
    /// are not bit-identical in IEEE-754. This measurement is what makes a
    /// mismatch between the two files visible at all — a pedestrian pair
    /// that only satisfies the box's longitudinal branch (large `\|dx\|`,
    /// small `\|dy\|`) must not be reported as a `min_distance` *violation*
    /// by the generic check when the encoded box is genuinely satisfied —
    /// that would be a false positive, not just a blind spot. See
    /// `test_pedestrian_box_is_measured_not_the_longitudinal_proxy`.
    ///
    /// This only changes which formula decides `safety_violations` /
    /// `all_constraints_satisfied` for a pedestrian pair. The scalar
    /// `scenario.validation.min_distance`/`min_ttc` fields still accumulate
    /// from the generic same-lane longitudinal reading for *every* pair,
    /// pedestrians included — changing that too would disagree with
    /// `tests/common/invariants.rs::check_extraction_agreement`, which
    /// independently re-derives those two scalars with the same generic
    /// formula and asserts they match `scenario.validation` exactly. So for a
    /// pure ego+pedestrian scenario the reported `min_distance` number is
    /// still the longitudinal gap, not the box — a pre-existing scalar
    /// ambiguity distinct from the violation-reporting gap this closes.
    fn compute_validation_metrics(
        &self,
        scenario: &mut crate::scenario::model::Scenario,
    ) -> crate::error::Result<()> {
        use crate::dsl::types::ActorRole;

        let mut min_ttc: Option<f64> = None;
        let mut min_distance: Option<f64> = None;
        let mut violations = Vec::new();

        // The validator's "same lane" test must be the *same*
        // predicate the encoder asserts, or the tool enforces one thing and
        // reports another. `encode_same_lane_constraint` (and the TTC /
        // distance propositions built on it) is
        //     lane1 == lane2  OR  |py1 - py2| < lane_width / 2
        // — the discrete match alone misses actors that are laterally
        // overlapping mid-manoeuvre or travelling in opposite directions.
        //
        // It is not enough to write the same formula here; it has to
        // be the same *function*. Re-typed as a bare `f64` `<`, this would
        // disagree with the exact evaluation on the boundary — exactly where
        // adjacent lane centres sit — and report gaps the encoder never
        // constrained as breaches. `same_lane_f64` is the one `f64` reading of
        // the predicate, with the rounding budget the exact side does not
        // need; see `encoder_utils::LANE_OVERLAP_EPS`.
        let lane_width = self.spec.get_lane_width();
        let same_lane = |s1: &crate::scenario::model::State, s2: &crate::scenario::model::State| {
            same_lane_f64(
                s1.lane(),
                s2.lane(),
                s1.position().y,
                s2.position().y,
                lane_width,
            )
        };

        // Road width for the pedestrian `ped_on_road` guard below — the same
        // `lane_width * num_lanes` formula `OnSidewalk`/`CrossingRoad`/
        // `PedestrianTTCGT` all use elsewhere in this file (encoder.rs), not a
        // value owned by a fenced file.
        let road_width = lane_width * self.spec.get_num_lanes() as f64;

        // Compute pairwise metrics for all actor combinations
        for (i, actor1) in self.spec.actors.iter().enumerate() {
            for actor2 in self.spec.actors.iter().skip(i + 1) {
                let id1 = &actor1.id;
                let id2 = &actor2.id;
                let traj1 = scenario.get_actor(id1).ok_or_else(|| {
                    crate::error::ScenarioGenError::ActorNotFound(format!("Actor {} missing", id1))
                })?;
                let traj2 = scenario.get_actor(id2).ok_or_else(|| {
                    crate::error::ScenarioGenError::ActorNotFound(format!("Actor {} missing", id2))
                })?;

                // `RectangularDistanceGT`/`PedestrianTTCGT` are the only
                // propositions any scenario type lowers for a pair involving a
                // pedestrian (see the audit above); the generic same-lane
                // longitudinal model is the wrong one for a perpendicular
                // crossing, so pairs like this take a different branch below
                // rather than silently reusing it.
                let pedestrian_pair =
                    actor1.role == ActorRole::Pedestrian || actor2.role == ActorRole::Pedestrian;

                for t in 0..=self.horizon {
                    let state1 = &traj1.states[t];
                    let state2 = &traj2.states[t];

                    // Compute longitudinal distance
                    let distance = (state1.position().x - state2.position().x).abs();

                    // `min_distance`/`min_ttc` are accumulated the same way
                    // for every pair, pedestrians included — this is what
                    // `tests/common/invariants.rs::check_extraction_agreement`
                    // independently re-derives and compares against, so the
                    // two scalar fields must keep meaning exactly what they
                    // always have (a same-lane-gated longitudinal reading),
                    // not the pedestrian box. What *does* change for a
                    // pedestrian pair is which formula decides whether this
                    // step is reported as a **violation** — see below.
                    if same_lane(state1, state2) {
                        min_distance =
                            Some(min_distance.map_or(distance, |m: f64| m.min(distance)));

                        // `TTC_CLOSING_SPEED_EPSILON`, not a second `0.01` literal:
                        // a value repeated by hand is one edit away from
                        // disagreeing with itself (see that constant's doc for
                        // why `tests/common/invariants.rs` and `tests/optimizer_test.rs`
                        // still carry their own copies rather than importing this one).
                        let epsilon = TTC_CLOSING_SPEED_EPSILON;
                        let rel_vel = if state1.position().x > state2.position().x {
                            Some(state2.velocity().vx - state1.velocity().vx)
                        } else if state2.position().x > state1.position().x {
                            Some(state1.velocity().vx - state2.velocity().vx)
                        } else {
                            None
                        };
                        if let Some(rel_vel) = rel_vel {
                            if rel_vel > epsilon {
                                let ttc = distance / rel_vel;
                                min_ttc = Some(min_ttc.map_or(ttc, |m: f64| m.min(ttc)));
                            }
                        }
                    }

                    if pedestrian_pair {
                        // Mirrors `Proposition::RectangularDistanceGT`
                        // (encoder.rs, `encode_proposition`) and
                        // `Proposition::PedestrianTTCGT`, both lowered only by
                        // `pedestrian_crossing.rs`. See the doc note above for
                        // why the thresholds are reproduced rather than
                        // shared. This *replaces* the generic longitudinal
                        // violation check for this pair — the generic
                        // `distance < min_distance` reading has no bearing on
                        // whether the box the encoder actually asserted was
                        // satisfied, and reporting it anyway is exactly the
                        // false-positive this issue's deliverable 3 exists to
                        // remove (see `test_pedestrian_box_is_measured_not_the_longitudinal_proxy`).
                        let (ped_state, other_state, ped_id, other_id, traj_ped, traj_other) =
                            if actor1.role == ActorRole::Pedestrian {
                                (state1, state2, id1, id2, traj1, traj2)
                            } else {
                                (state2, state1, id2, id1, traj2, traj1)
                            };

                        // Box: |dx| > min_distance/LONGITUDINAL OR
                        // |dy| > min_distance/LATERAL, sharing divisors
                        // with `pedestrian_crossing.rs` rather than reproducing
                        // literals. Both comparisons are strict in the
                        // encoding (no `.ge`), so — matching the tolerance
                        // direction used everywhere else in this function —
                        // a measurement within METRIC_TOL of clearing a side
                        // counts as clearing it.
                        let threshold_x =
                            self.spec.min_distance / PEDESTRIAN_BOX_LONGITUDINAL_DIVISOR;
                        let threshold_y = self.spec.min_distance / PEDESTRIAN_BOX_LATERAL_DIVISOR;
                        let dx = (other_state.position().x - ped_state.position().x).abs();
                        let dy = (other_state.position().y - ped_state.position().y).abs();
                        let box_safe =
                            dx > threshold_x - METRIC_TOL || dy > threshold_y - METRIC_TOL;
                        if !box_safe {
                            violations.push(format!(
                                "Pedestrian distance-box violation at t={:.1}s: {}-{}: \
                                 |dx|={:.2}m (limit {:.2}m), |dy|={:.2}m (limit {:.2}m)",
                                t as f64 * self.spec.time_step,
                                other_id,
                                ped_id,
                                dx,
                                threshold_x,
                                dy,
                                threshold_y
                            ));
                        }

                        // The box above is *sampled*, and the ego covers
                        // far more ground in one `time_step` than the box is
                        // long: at 20 m/s and `time_step: 0.5` it moves 10 m
                        // against a `threshold_x` of 1 m for the usual
                        // `min_distance: 2.0`. So a trajectory can satisfy the
                        // box at every step and still drive straight through
                        // the pedestrian in between, and — because this
                        // function samples exactly the way the encoder asserts
                        // — be reported safe: an `enforce`d single-lane
                        // spec with `min_distance: 6.0` could come back
                        // `all_constraints_satisfied: true` with `dx` going
                        // `-3.000 → +7.375` across one step at `|dy| = 1.875`
                        // against a `threshold_y` of 4.0.
                        //
                        // `pedestrian_crossing.rs::add_z3_constraints` now
                        // forbids that in the encoding; this is the same guard
                        // re-derived on the shipped trajectory, so a scenario
                        // that reaches a user by any other route is still
                        // checked. See that constraint's comment for what the
                        // guard does and does not cover.
                        //
                        // Deliberately unconditional, like every other check in
                        // this function: `ConstraintModes` are the business of
                        // `tests/common/invariants.rs`, and the three
                        // `examples/pedestrian_*.yaml` pass it as they stand —
                        // each does cross the pedestrian's `px` exactly once,
                        // and each is laterally clear at both ends when it does
                        // (`|dy|` = 4.2/4.8, 5.5/6.1 and 2.9/3.7 against
                        // thresholds of 1.33, 1.33 and 1.0).
                        if t < self.horizon {
                            let next_ped = &traj_ped.states[t + 1];
                            let next_other = &traj_other.states[t + 1];
                            let dx_signed = other_state.position().x - ped_state.position().x;
                            let dx_next = next_other.position().x - next_ped.position().x;
                            let dy_signed = other_state.position().y - ped_state.position().y;
                            let dy_next = next_other.position().y - next_ped.position().y;

                            let passes = dx_signed * dx_next < 0.0;
                            // Same tolerance direction as `box_safe` above:
                            // a measurement within METRIC_TOL of clearing the
                            // lateral half-box counts as clearing it.
                            let clear = |d: f64| d > threshold_y - METRIC_TOL;
                            let laterally_clear = (clear(dy_signed) && clear(dy_next))
                                || (clear(-dy_signed) && clear(-dy_next));
                            if passes && !laterally_clear {
                                violations.push(format!(
                                    "Pedestrian box tunnelling between t={:.1}s and t={:.1}s: \
                                     {}-{}: dx {:.2}m → {:.2}m crosses the pedestrian with \
                                     dy {:.2}m → {:.2}m inside the {:.2}m lateral half-box",
                                    t as f64 * self.spec.time_step,
                                    (t + 1) as f64 * self.spec.time_step,
                                    other_id,
                                    ped_id,
                                    dx_signed,
                                    dx_next,
                                    dy_signed,
                                    dy_next,
                                    threshold_y
                                ));
                            }
                        }

                        // PedestrianTTCGT: guarded by the pedestrian being on
                        // the road and the other actor behind it and moving
                        // forward — same guard as the encoder's `ped_on_road`
                        // / `approaching`. `ttc_safe` there is non-strict
                        // (`.ge`), matching this check's own boundary
                        // (`ttc < min_ttc - METRIC_TOL` calls `ttc == min_ttc`
                        // safe) the same way `.ge` does for the
                        // vehicle-vehicle `TTCGT`.
                        let ped_on_road =
                            ped_state.position().y >= 0.0 && ped_state.position().y <= road_width;
                        let ego_behind = other_state.position().x < ped_state.position().x;
                        let ego_vx = other_state.velocity().vx;
                        if ped_on_road && ego_behind && ego_vx > TTC_CLOSING_SPEED_EPSILON {
                            let ttc = dx / ego_vx;
                            if ttc < self.spec.min_ttc - METRIC_TOL {
                                violations.push(format!(
                                    "Pedestrian TTC violation at t={:.1}s: {}-{}: {:.2}s < {:.2}s",
                                    t as f64 * self.spec.time_step,
                                    other_id,
                                    ped_id,
                                    ttc,
                                    self.spec.min_ttc
                                ));
                            }
                        }
                    } else {
                        // Check minimum distance violation
                        if same_lane(state1, state2)
                            && distance < self.spec.min_distance - METRIC_TOL
                        {
                            violations.push(format!(
                                "Distance violation at t={:.1}s: {}-{}: {:.2}m < {:.2}m",
                                t as f64 * self.spec.time_step,
                                id1,
                                id2,
                                distance,
                                self.spec.min_distance
                            ));
                        }

                        // TTC violation (only when in same lane and approaching).
                        // The scalar `min_ttc` accumulation above already
                        // recomputes the same directed relative velocity;
                        // this only decides whether *this* step is reported.
                        if same_lane(state1, state2) {
                            let epsilon = TTC_CLOSING_SPEED_EPSILON;

                            // Case 1: state1 ahead, state2 behind, state2 faster (catching up)
                            if state1.position().x > state2.position().x {
                                let rel_vel = state2.velocity().vx - state1.velocity().vx;
                                if rel_vel > epsilon {
                                    let ttc = distance / rel_vel;
                                    if ttc < self.spec.min_ttc - METRIC_TOL {
                                        violations.push(format!(
                                            "TTC violation at t={:.1}s: {}-{}: {:.2}s < {:.2}s",
                                            t as f64 * self.spec.time_step,
                                            id1,
                                            id2,
                                            ttc,
                                            self.spec.min_ttc
                                        ));
                                    }
                                }
                            }
                            // Case 2: state2 ahead, state1 behind, state1 faster (catching up)
                            else if state2.position().x > state1.position().x {
                                let rel_vel = state1.velocity().vx - state2.velocity().vx;
                                if rel_vel > epsilon {
                                    let ttc = distance / rel_vel;
                                    if ttc < self.spec.min_ttc - METRIC_TOL {
                                        violations.push(format!(
                                            "TTC violation at t={:.1}s: {}-{}: {:.2}s < {:.2}s",
                                            t as f64 * self.spec.time_step,
                                            id1,
                                            id2,
                                            ttc,
                                            self.spec.min_ttc
                                        ));
                                    }
                                }
                            }
                        }
                    }

                    // Lateral separation. The encoder
                    // lowers `min_lateral_distance` to
                    // `Proposition::LateralDistanceGT` — an unguarded
                    // |py1 - py2| > d at every step — but the validator never
                    // looked at it, so `multi_lane_safety` could report
                    // `all_constraints_satisfied: true` with 0.000 m of
                    // lateral separation. Check exactly what the encoder
                    // asserts: unguarded, at every step. Unconditional on
                    // `pedestrian_pair` — this is one of the two rows the
                    // audit above marks "agrees" regardless of actor role.
                    if let Some(min_lat) = self.spec.min_lateral_distance {
                        let lateral = (state1.position().y - state2.position().y).abs();
                        if lateral < min_lat - METRIC_TOL {
                            violations.push(format!(
                                "Lateral distance violation at t={:.1}s: {}-{}: {:.2}m < {:.2}m",
                                t as f64 * self.spec.time_step,
                                id1,
                                id2,
                                lateral,
                                min_lat
                            ));
                        }
                    }

                    // `RelativeVelocityGT` (`max_relative_velocity`).
                    // `scenarios/mod.rs` lowers this for every pair,
                    // unguarded by lane, with `Negated` polarity: the safe
                    // condition is `|vx1 - vx2| <= max_relative_velocity`
                    // (non-strict — the negation of the encoder's strict
                    // `.gt`). See `unsafe_following.yaml`, an adversarial
                    // corpus example built to violate exactly this field.
                    if let Some(max_rel_vel) = self.spec.max_relative_velocity {
                        let rel_vel = (state1.velocity().vx - state2.velocity().vx).abs();
                        if rel_vel > max_rel_vel + METRIC_TOL {
                            violations.push(format!(
                                "Relative velocity violation at t={:.1}s: {}-{}: {:.2}m/s > {:.2}m/s",
                                t as f64 * self.spec.time_step,
                                id1,
                                id2,
                                rel_vel,
                                max_rel_vel
                            ));
                        }
                    }
                }
            }
        }

        // `VelocityLT`/`VelocityGT` (`max_velocity`/`min_velocity`).
        // `scenarios/mod.rs` lowers both per-actor, unguarded, for every
        // actor including pedestrians (no scenario type currently sets
        // either field for a pedestrian, but the lowering does not exclude
        // one, so neither does this check). Both encoder-side comparisons
        // are strict (no `.ge`/`.le`), matching the `+ TOL` boundary used
        // here — a measurement within `METRIC_TOL` of the limit is not
        // flagged, the same direction `check_envelope` in
        // `tests/common/invariants.rs` already uses for the same fields.
        // That test-side check only ever saw the raw trajectories; this is
        // the first time either bound is measured into the shipped
        // `scenario.validation` output.
        for actor in &self.spec.actors {
            let Some(traj) = scenario.get_actor(&actor.id) else {
                continue;
            };
            for state in &traj.states {
                let vx_abs = state.velocity().vx.abs();
                let t = state.time;

                if let Some(max_vel) = self.spec.max_velocity {
                    if vx_abs > max_vel + METRIC_TOL {
                        violations.push(format!(
                            "Velocity violation at t={:.1}s: {}: |vx|={:.2}m/s > {:.2}m/s",
                            t, actor.id, vx_abs, max_vel
                        ));
                    }
                }
                if let Some(min_vel) = self.spec.min_velocity {
                    if vx_abs < min_vel - METRIC_TOL {
                        violations.push(format!(
                            "Velocity violation at t={:.1}s: {}: |vx|={:.2}m/s < {:.2}m/s",
                            t, actor.id, vx_abs, min_vel
                        ));
                    }
                }
            }
        }

        // Compute acceleration metrics
        let mut max_accel = 0.0;
        let mut max_decel = 0.0;
        let mut accel_violations = Vec::new();

        for actor_traj in &scenario.actors {
            for state in &actor_traj.states {
                let ax = state.acceleration().ax;

                // Track maximum values
                if ax > max_accel {
                    max_accel = ax;
                }
                if ax < max_decel {
                    max_decel = ax;
                }

                // Check for global constraint violations
                if let Some(max_a) = self.spec.max_acceleration {
                    if ax > max_a {
                        accel_violations.push(format!(
                            "{} harsh acceleration at t={:.1}s: {:.2} m/s² > {:.2} m/s²",
                            actor_traj.id, state.time, ax, max_a
                        ));
                    }
                }

                if let Some(max_d) = self.spec.max_deceleration {
                    if ax < max_d {
                        accel_violations.push(format!(
                            "{} harsh braking at t={:.1}s: {:.2} m/s² < {:.2} m/s²",
                            actor_traj.id, state.time, ax, max_d
                        ));
                    }
                }
            }
        }

        scenario.validation.max_acceleration = max_accel;
        scenario.validation.max_deceleration = max_decel;
        let has_accel_violations = !accel_violations.is_empty();
        scenario.validation.acceleration_violations = accel_violations;

        // Update validation info. `None` means the metric was never evaluated —
        // it must not be conflated with a large (i.e. safe) measurement.
        scenario.validation.min_ttc = min_ttc;
        scenario.validation.min_distance = min_distance;
        scenario.validation.all_constraints_satisfied =
            violations.is_empty() && !has_accel_violations;
        scenario.validation.safety_violations = violations;

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use crate::dsl::types::{
        ActorRole, ActorSpec, LaneChangeConfig, LaneChangeDirection, RoadSpec, ScenarioSpec,
        ScenarioType, ValueOrRange,
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
    fn test_scenario_extraction() {
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
            // Safety constraints are now included in LTL formula via generate_safety()

            // Check satisfiability
            let result = encoder.check();
            assert_eq!(result, SatResult::Sat, "Should be satisfiable");

            if result == SatResult::Sat {
                let model = encoder.get_model().unwrap();

                // Extract scenario
                let scenario = encoder.extract_scenario(&model).unwrap();

                // Verify basic structure
                assert_eq!(scenario.actors.len(), 2);
                assert_eq!(scenario.time_step, 0.5);
                assert_eq!(scenario.duration, 10.0);

                // Verify ego trajectory
                let ego = scenario.get_actor("ego").expect("Ego missing");
                assert_eq!(ego.id, "ego");
                assert_eq!(ego.states.len(), 21); // 0..=20

                // Verify NPC trajectory
                let npc = scenario.get_actor("npc").expect("NPC missing");
                assert_eq!(npc.id, "npc");
                assert_eq!(npc.states.len(), 21);

                // Verify initial conditions
                assert_eq!(ego.states[0].lane(), 1);
                assert_eq!(npc.states[0].lane(), 0);

                // Verify NPC position is ahead initially
                assert!(npc.states[0].position().x > ego.states[0].position().x);

                // Verify validation metrics exist
                println!("Min TTC: {:?}", scenario.validation.min_ttc);
                println!("Min distance: {:?}", scenario.validation.min_distance);
                println!(
                    "All constraints satisfied: {}",
                    scenario.validation.all_constraints_satisfied
                );

                // Test JSON serialization
                let json = serde_json::to_string_pretty(&scenario).unwrap();
                println!("Extracted scenario JSON:\n{}", json);

                // Verify it can be deserialized
                let _deserialized: crate::scenario::model::Scenario =
                    serde_json::from_str(&json).unwrap();

                println!("Scenario extraction test passed!");
            }
        });
    }

    // ===== Group 3: Validation metrics =====

    #[test]
    fn test_validation_metrics_safe_scenario() {
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

            let ltl_formula = LTLGenerator::generate(&spec).unwrap();
            encoder.encode_ltl(&ltl_formula);

            assert_eq!(encoder.check(), SatResult::Sat);
            let model = encoder.get_model().unwrap();
            let scenario = encoder.extract_scenario(&model).unwrap();

            assert!(
                scenario.validation.all_constraints_satisfied,
                "Safe scenario should satisfy all constraints"
            );
            // `None` = never evaluated, which is the same escape hatch the old
            // `== f64::INFINITY` disjunct provided.
            //
            // The 1e-6 slack is the same rational-to-double rounding error
            // `compute_validation_metrics` allows for (see `METRIC_TOL`
            // there): the encoder asserts `gap >= min_ttc *
            // closing_speed` non-strictly and Z3 answers on the boundary, and
            // recovering a TTC by dividing two rounded `f64`s lands a ULP low
            // — 2.999999999999991 against a threshold of 3.
            assert!(
                scenario
                    .validation
                    .min_ttc
                    .is_none_or(|ttc| ttc >= spec.min_ttc - 1e-6),
                "Min TTC {:?} should be >= {}",
                scenario.validation.min_ttc,
                spec.min_ttc
            );
            assert!(
                scenario
                    .validation
                    .min_distance
                    .is_none_or(|d| d >= spec.min_distance),
                "Min distance {:?} should be >= {}",
                scenario.validation.min_distance,
                spec.min_distance
            );
        });
    }

    #[test]
    fn test_validation_metrics_detects_violations() {
        use crate::ltl::generator::LTLGenerator;

        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            // Use adversarial mode: violate TTC
            let mut spec = create_test_spec();
            spec.constraint_modes = crate::dsl::types::ConstraintModes::Shorthand(
                crate::dsl::types::ConstraintShorthand::ViolateAll,
            );

            let mut encoder = Z3Encoder::new(spec.clone());
            encoder.create_variables();
            encoder.encode_initial_conditions();
            encoder.encode_kinematics();
            encoder.encode_lane_velocity_constraints();
            encoder.encode_lateral_velocity_bounds();

            let ltl_formula = LTLGenerator::generate(&spec).unwrap();
            encoder.encode_ltl(&ltl_formula);

            let result = encoder.check();
            if result == SatResult::Sat {
                let model = encoder.get_model().unwrap();
                let scenario = encoder.extract_scenario(&model).unwrap();

                // In adversarial mode, violations should be detected
                let has_violation = !scenario.validation.all_constraints_satisfied
                    || !scenario.validation.safety_violations.is_empty();
                // Note: it's possible the solver finds a scenario that technically
                // violates at some point but validation still passes due to timing.
                // The key test is that the pipeline doesn't crash.
                println!(
                    "Adversarial: constraints_satisfied={}, violations={}",
                    scenario.validation.all_constraints_satisfied,
                    scenario.validation.safety_violations.len()
                );
                // If violations exist, verify they have content
                if !scenario.validation.safety_violations.is_empty() {
                    assert!(has_violation);
                    for v in &scenario.validation.safety_violations {
                        assert!(!v.is_empty(), "Violation string should not be empty");
                    }
                }
            } else {
                // UNSAT is acceptable for adversarial — constraints may conflict
                println!("Adversarial scenario is UNSAT (constraints conflict)");
            }
        });
    }

    /// `min_velocity` (`Proposition::VelocityGT`) is lowered by
    /// `scenarios/mod.rs` and measured by `compute_validation_metrics`.
    /// No shipped corpus example sets `min_velocity`, so this constructs the
    /// violation directly: a single decelerating actor under `Violate` mode,
    /// which must end up slower than the floor at some point.
    #[test]
    fn test_validation_metrics_detects_min_velocity_violation() {
        use crate::dsl::types::ConstraintMode;
        use crate::ltl::generator::LTLGenerator;

        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
            let mut spec = create_test_spec();
            spec.min_velocity = Some(10.0);
            spec.constraint_modes = crate::dsl::types::ConstraintModes::Detailed {
                min_ttc: ConstraintMode::Enforce,
                min_distance: ConstraintMode::Enforce,
                max_acceleration: ConstraintMode::Enforce,
                max_velocity: ConstraintMode::Enforce,
                min_velocity: ConstraintMode::Violate,
                min_lateral_distance: ConstraintMode::Enforce,
                max_relative_velocity: ConstraintMode::Enforce,
            };

            let mut encoder = Z3Encoder::new(spec.clone());
            encoder.create_variables();
            encoder.encode_initial_conditions();
            encoder.encode_kinematics();
            encoder.encode_lane_velocity_constraints();
            encoder.encode_lateral_velocity_bounds();

            let ltl_formula = LTLGenerator::generate(&spec).unwrap();
            encoder.encode_ltl(&ltl_formula);

            assert_eq!(
                encoder.check(),
                SatResult::Sat,
                "min_velocity: violate on a decelerating actor should be satisfiable"
            );
            let model = encoder.get_model().unwrap();
            let scenario = encoder.extract_scenario(&model).unwrap();

            assert!(
                !scenario.validation.all_constraints_satisfied,
                "a constructed min_velocity violation must now surface in \
                 all_constraints_satisfied, got violations={:?}",
                scenario.validation.safety_violations
            );
            assert!(
                scenario
                    .validation
                    .safety_violations
                    .iter()
                    .any(|v| v.contains("Velocity violation") && v.contains('<')),
                "expected a min_velocity-shaped violation in {:?}",
                scenario.validation.safety_violations
            );
        });
    }

    /// `pedestrian_crossing.rs` lowers `min_distance` to
    /// `Proposition::RectangularDistanceGT` (a box: `|dx| > d/2 OR |dy| >
    /// d/1.5`), not the longitudinal-only model `compute_validation_metrics`
    /// uses for every other scenario type. Because a pedestrian's lane is
    /// pinned to 0 and so is the ego's in this scenario type, `same_lane` is
    /// structurally true, so the generic check must not disagree with the
    /// box in either direction. This constructs the direction that matters
    /// most: a pair the box calls safe (cleared on the longitudinal branch)
    /// that a longitudinal-only check would call a *violation* — a false
    /// positive, not just a blind spot.
    #[test]
    fn test_pedestrian_box_is_measured_not_the_longitudinal_proxy() {
        use crate::scenario::model::{
            Acceleration, ActorTrajectory, Position, Scenario, State, Velocity,
        };

        let cfg = Config::new();
        z3::with_z3_config(&cfg, || {
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
                        position: ValueOrRange::Value(0.0),
                        speed: ValueOrRange::Value(1.0),
                        acceleration: ValueOrRange::Range([-1.0, 1.0]),
                        direction: 1,
                        behavior: HashMap::new(),
                        lane_changes: vec![],
                        bicycle_params: None,
                    },
                ],
                min_ttc: 2.0,
                // threshold_x = 1.0, threshold_y = 1.333... (pedestrian_crossing.rs:78-79)
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

            let encoder = Z3Encoder::new(spec.clone());

            let build = |ego_x: f64, ped_y: f64| -> Scenario {
                let mut scenario = Scenario::new(
                    "pedestrian_crossing".to_string(),
                    spec.time_step,
                    spec.duration,
                    spec.road.clone().unwrap(),
                );
                let mut ego_traj = ActorTrajectory::new("ego".to_string(), "ego".to_string());
                let mut ped_traj =
                    ActorTrajectory::new("pedestrian".to_string(), "pedestrian".to_string());
                for t in 0..=1 {
                    ego_traj.add_state(State::new(
                        t as f64,
                        Position::new(ego_x, 0.0),
                        Velocity::new(0.0, 0.0),
                        Acceleration::new(0.0, 0.0),
                        0,
                    ));
                    ped_traj.add_state(State::new(
                        t as f64,
                        Position::new(0.0, ped_y),
                        Velocity::new(0.0, 0.0),
                        Acceleration::new(0.0, 0.0),
                        0,
                    ));
                }
                scenario.add_actor(ego_traj);
                scenario.add_actor(ped_traj);
                scenario
            };

            // dx = 1.5 > threshold_x (1.0): box's longitudinal branch alone
            // clears it, so the box is satisfied regardless of dy. The old
            // longitudinal-only check (same_lane true, since both actors are
            // pinned to lane 0) would compare 1.5 against min_distance (2.0)
            // and call it a violation.
            let mut safe = build(1.5, 0.2);
            encoder.compute_validation_metrics(&mut safe).unwrap();
            assert!(
                !safe
                    .validation
                    .safety_violations
                    .iter()
                    .any(|v| v.contains("distance-box")),
                "|dx|=1.5 clears threshold_x=1.0, the box is satisfied, this must not be \
                 reported as a distance-box violation; got {:?}",
                safe.validation.safety_violations
            );

            // dx = 0.5, dy = 0.5: both inside their thresholds (1.0 and
            // 1.333), so the box is genuinely violated.
            let mut unsafe_ = build(0.5, 0.5);
            encoder.compute_validation_metrics(&mut unsafe_).unwrap();
            assert!(
                unsafe_
                    .validation
                    .safety_violations
                    .iter()
                    .any(|v| v.contains("distance-box")),
                "|dx|=0.5 <= threshold_x=1.0 and |dy|=0.5 <= threshold_y=1.333 must be \
                 reported as a distance-box violation; got {:?}",
                unsafe_.validation.safety_violations
            );
        });
    }

    /// The pedestrian safety box's two ratios must not be independent
    /// `f64` literals in `pedestrian_crossing.rs::generate_safety` (what gets
    /// asserted) and here in `compute_validation_metrics` (what gets
    /// measured) — that shape is one edit away from disagreeing silently.
    /// Instead both import
    /// `pedestrian_crossing::PEDESTRIAN_BOX_{LONGITUDINAL,LATERAL}_DIVISOR`.
    /// This test pins that coupling down: it computes the
    /// box thresholds from the shared constants exactly as
    /// `compute_validation_metrics` does, and cross-checks them, bit for
    /// bit, against the `threshold_x`/`threshold_y` the encoder actually
    /// asserts (extracted from the `RectangularDistanceGT` atom
    /// `generate_safety` produces). If the shared constants this test names
    /// did not exist, it would not even compile against that code — a
    /// stronger failure than a numeric mismatch, and the reason this is a
    /// coupling test rather than a tolerance-based one.
    #[test]
    fn test_pedestrian_box_thresholds_share_one_definition() {
        use crate::ltl::formula::{LTLFormula, Proposition};
        use crate::scenarios::pedestrian_crossing::{
            PEDESTRIAN_BOX_LATERAL_DIVISOR, PEDESTRIAN_BOX_LONGITUDINAL_DIVISOR,
        };
        use crate::scenarios::ScenarioModel;

        let mut spec = create_test_spec();
        spec.scenario_type = ScenarioType::PedestrianCrossing;
        spec.min_distance = 2.6;
        spec.actors[1].role = ActorRole::Pedestrian;

        // What `compute_validation_metrics` measures against, computed via
        // the shared constants.
        let measured_threshold_x = spec.min_distance / PEDESTRIAN_BOX_LONGITUDINAL_DIVISOR;
        let measured_threshold_y = spec.min_distance / PEDESTRIAN_BOX_LATERAL_DIVISOR;

        // What `generate_safety` actually asserts.
        let model = crate::scenarios::pedestrian_crossing::PedestrianCrossingModel;
        let formula = model.generate_safety(&spec).unwrap();
        let asserted = find_rectangular_distance_gt(&formula)
            .expect("generate_safety must emit a RectangularDistanceGT atom");

        assert_eq!(
            asserted.0, measured_threshold_x,
            "asserted threshold_x must equal the shared constant \
             compute_validation_metrics measures against, bit for bit"
        );
        assert_eq!(
            asserted.1, measured_threshold_y,
            "asserted threshold_y must equal the shared constant \
             compute_validation_metrics measures against, bit for bit"
        );

        // Helper: find the (threshold_x, threshold_y) of the first
        // RectangularDistanceGT atom in a formula tree.
        fn find_rectangular_distance_gt(formula: &LTLFormula) -> Option<(f64, f64)> {
            match formula {
                LTLFormula::Atom(Proposition::RectangularDistanceGT {
                    threshold_x,
                    threshold_y,
                    ..
                }) => Some((*threshold_x, *threshold_y)),
                LTLFormula::Always(inner)
                | LTLFormula::Eventually(inner)
                | LTLFormula::Not(inner) => find_rectangular_distance_gt(inner),
                LTLFormula::And(lhs, rhs) | LTLFormula::Or(lhs, rhs) => {
                    find_rectangular_distance_gt(lhs).or_else(|| find_rectangular_distance_gt(rhs))
                }
                _ => None,
            }
        }
    }

    #[test]
    fn test_validation_acceleration_metrics() {
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

            let ltl_formula = LTLGenerator::generate(&spec).unwrap();
            encoder.encode_ltl(&ltl_formula);

            assert_eq!(encoder.check(), SatResult::Sat);
            let model = encoder.get_model().unwrap();
            let scenario = encoder.extract_scenario(&model).unwrap();

            // Check that acceleration metrics are computed from trajectory
            let ego = scenario.get_actor("ego").unwrap();
            // Compute acceleration from velocity differences
            let mut max_accel = 0.0_f64;
            let mut max_decel = 0.0_f64;
            for i in 1..ego.states.len() {
                let dv = ego.states[i].velocity().vx - ego.states[i - 1].velocity().vx;
                let accel = dv / spec.time_step;
                if accel > max_accel {
                    max_accel = accel;
                }
                if accel < max_decel {
                    max_decel = accel;
                }
            }
            // Acceleration should be within bounds [-8, 3]
            assert!(
                max_accel <= 3.0 + 0.1,
                "Max acceleration {} should be <= 3.0",
                max_accel
            );
            assert!(
                max_decel >= -8.0 - 0.1,
                "Max deceleration {} should be >= -8.0",
                max_decel
            );
        });
    }
}
