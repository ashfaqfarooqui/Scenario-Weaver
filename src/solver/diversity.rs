//! Stratified (Latin-hypercube) sampling of the declared initial-condition ranges.
//!
//! The blocking clauses in [`crate::solver::multi_solve`] say only "this solution is
//! not a near-duplicate of that one". Z3's LRA simplex answers a satisfiable query
//! with a *vertex* of the feasible polytope, so each successive solve steps just far
//! enough to clear the blocking band and stops: `-n 5` on `cut_in_left.yaml` covered
//! 1.86 m of a declared 60 m position range, and left the ego bit-identical in all
//! five scenarios.
//!
//! This module replaces "not a duplicate" with **spread by construction**. Every free
//! initial-condition dimension across every actor — the ego included — is cut into `N`
//! equal strata, and scenario `i` is confined to one stratum per dimension. The
//! stratum assignment for each dimension is an independent permutation of `0..N`, which
//! is what makes the design a Latin hypercube rather than a diagonal: every stratum of
//! every dimension is used exactly once across the batch, but the dimensions are not
//! correlated with one another.
//!
//! The permutations come from a seeded [`SplitMix64`] written out inline rather than
//! pulled from `rand`, so the sequence is identical on every platform and toolchain —
//! which is exactly what `--seed` reproducibility means here.
//!
//! # What is *not* stratified
//!
//! - **Pedestrian `speed:`.** For a pedestrian the declared speed binds to the *lateral*
//!   velocity `vy[0]` (`solver::encoders::pedestrian::encode_pedestrian_initial_state`),
//!   signed by the crossing direction and clamped to the walking/running band, not to
//!   the longitudinal `vx[0]` this plan asserts on — `vx[0]` is pinned to 0 for a
//!   crossing pedestrian. Stratifying it here would assert a band the encoder has
//!   already contradicted and drive every solve down the fallback ladder. Pedestrian
//!   `position:` *is* stratified: it binds to `px[0]` exactly as it does for a vehicle.
//! - **Accelerations, lane-change timing, and any state at `t > 0`.** Manoeuvre timing is
//!   out of scope here: it is fixed before the solver runs (see
//!   `collect_lane_change_data`), so there is nothing to stratify over yet.
//!
//! # Infeasible cells
//!
//! A cell of the hypercube can be genuinely empty: `cut_in_left` requires the NPC ahead
//! of the ego, so pairing the ego's top position stratum with the NPC's bottom one has
//! no solution at all. Callers are expected to walk [`DiversityPlan::max_relaxation`]
//! levels, dropping one dimension's stratum at a time (narrowest declared range first,
//! since the narrowest dimension buys the least spread per unit of constraint) and
//! finally dropping all of them, before concluding that a solve is really `Unsat`.

use crate::dsl::types::{ActorRole, ScenarioSpec, ValueOrRange};

/// Seed used when the caller does not pass `--seed`.
///
/// Fixed, not drawn from the clock: with a fixed seed the whole tool stays
/// deterministic, so two runs of the same command still produce byte-identical
/// batches (modulo the per-scenario UUID) as they did before stratification existed.
pub const DEFAULT_DIVERSITY_SEED: u64 = 0x5745_4156_4552; // "WEAVER" in ASCII

/// Which Z3 variable at `t = 0` a stratum constrains.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DimensionKind {
    /// The actor's longitudinal position, `encoder.get_position_x(id, 0)`.
    LongitudinalPosition,
    /// The actor's longitudinal velocity, `encoder.get_velocity_x(id, 0)`.
    ///
    /// Signed: the encoder puts `vx[0]` in `[-speed_max, -speed_min]` for an actor
    /// with `direction: -1`, so the bounds carried here are signed to match.
    LongitudinalVelocity,
}

/// One free initial-condition dimension of one actor.
#[derive(Debug, Clone)]
struct Dimension {
    actor_id: String,
    kind: DimensionKind,
    /// Lower bound of the declared range, in the variable's own (signed) units.
    lo: f64,
    /// Upper bound of the declared range, in the variable's own (signed) units.
    hi: f64,
}

impl Dimension {
    fn width(&self) -> f64 {
        self.hi - self.lo
    }
}

/// A closed interval to assert on one actor's `t = 0` variable.
#[derive(Debug, Clone)]
pub struct StratumBound {
    /// The actor whose variable is constrained.
    pub actor_id: String,
    /// Which variable to constrain.
    pub kind: DimensionKind,
    /// Inclusive lower bound.
    pub lo: f64,
    /// Inclusive upper bound.
    pub hi: f64,
}

/// A batch-wide assignment of scenarios to strata.
///
/// Built once, before the generation loop, and consulted per solve attempt via
/// [`DiversityPlan::strata_for`].
#[derive(Debug, Clone)]
pub struct DiversityPlan {
    dimensions: Vec<Dimension>,
    /// `assignments[dim][scenario]` is the stratum index in `0..num_scenarios`.
    assignments: Vec<Vec<usize>>,
    num_scenarios: usize,
    /// Dimension indices ordered narrowest declared range first — the order in which
    /// the fallback ladder gives strata up.
    relaxation_order: Vec<usize>,
}

impl DiversityPlan {
    /// Build the plan for `num_scenarios` scenarios of `spec`, seeded by `seed`.
    ///
    /// Dimensions are collected in spec order (actors as declared, position before
    /// speed) so the plan — and therefore the output — depends only on the spec and
    /// the seed, never on iteration order of a hash map.
    #[must_use]
    pub fn new(spec: &ScenarioSpec, num_scenarios: usize, seed: u64) -> Self {
        let mut dimensions = Vec::new();

        for actor in &spec.actors {
            if let ValueOrRange::Range([lo, hi]) = actor.position {
                if hi - lo > f64::EPSILON {
                    dimensions.push(Dimension {
                        actor_id: actor.id.clone(),
                        kind: DimensionKind::LongitudinalPosition,
                        lo,
                        hi,
                    });
                }
            }

            // See the module docs: a pedestrian's `speed:` does not reach `vx[0]`.
            if actor.role == ActorRole::Pedestrian {
                continue;
            }

            if let ValueOrRange::Range([lo, hi]) = actor.speed {
                if hi - lo > f64::EPSILON {
                    // `direction: -1` puts vx[0] in [-hi, -lo]; stratify in the
                    // variable's own signed units so the cells line up with the
                    // interval the encoder actually asserted.
                    let (lo, hi) = if actor.direction >= 0 {
                        (lo, hi)
                    } else {
                        (-hi, -lo)
                    };
                    dimensions.push(Dimension {
                        actor_id: actor.id.clone(),
                        kind: DimensionKind::LongitudinalVelocity,
                        lo,
                        hi,
                    });
                }
            }
        }

        // One independent permutation per dimension. The per-dimension sub-seed is
        // derived from the batch seed and the dimension's ordinal so that adding an
        // actor to a spec does not reshuffle the dimensions before it.
        let assignments: Vec<Vec<usize>> = (0..dimensions.len())
            .map(|d| {
                let mut rng =
                    SplitMix64::new(seed ^ (d as u64).wrapping_mul(0x9E37_79B9_7F4A_7C15));
                rng.permutation(num_scenarios)
            })
            .collect();

        let mut relaxation_order: Vec<usize> = (0..dimensions.len()).collect();
        relaxation_order.sort_by(|&a, &b| {
            dimensions[a]
                .width()
                .partial_cmp(&dimensions[b].width())
                .unwrap_or(std::cmp::Ordering::Equal)
                .then(a.cmp(&b))
        });

        Self {
            dimensions,
            assignments,
            num_scenarios,
            relaxation_order,
        }
    }

    /// Number of free dimensions the plan stratifies.
    #[must_use]
    pub fn dimension_count(&self) -> usize {
        self.dimensions.len()
    }

    /// Highest relaxation level. Level `0` asserts every stratum; level `k` drops the
    /// `k` narrowest dimensions' strata; level `max_relaxation()` asserts none at all,
    /// leaving only the blocking clauses — the behaviour before stratification existed.
    #[must_use]
    pub fn max_relaxation(&self) -> usize {
        self.dimensions.len()
    }

    /// The strata to assert for scenario `scenario_index` at relaxation `level`.
    ///
    /// Returns an empty vector once `level >= max_relaxation()`, or when the spec
    /// declares no free dimension at all (everything pinned to a `Value`).
    #[must_use]
    pub fn strata_for(&self, scenario_index: usize, level: usize) -> Vec<StratumBound> {
        if self.num_scenarios == 0 || scenario_index >= self.num_scenarios {
            return Vec::new();
        }

        let dropped: Vec<usize> = self
            .relaxation_order
            .iter()
            .take(level.min(self.relaxation_order.len()))
            .copied()
            .collect();

        self.dimensions
            .iter()
            .enumerate()
            .filter(|(d, _)| !dropped.contains(d))
            .map(|(d, dim)| {
                let stratum = self.assignments[d][scenario_index];
                let (lo, hi) = stratum_bounds(dim.lo, dim.hi, stratum, self.num_scenarios);
                StratumBound {
                    actor_id: dim.actor_id.clone(),
                    kind: dim.kind,
                    lo,
                    hi,
                }
            })
            .collect()
    }

    /// Human-readable name of the dimension dropped when moving from relaxation
    /// `level` to `level + 1`, for the `warn!` the caller logs on each rung.
    #[must_use]
    pub fn dropped_at(&self, level: usize) -> Option<String> {
        let d = *self.relaxation_order.get(level)?;
        let dim = self.dimensions.get(d)?;
        let what = match dim.kind {
            DimensionKind::LongitudinalPosition => "position",
            DimensionKind::LongitudinalVelocity => "speed",
        };
        Some(format!(
            "{}.{what} (declared width {:.3})",
            dim.actor_id,
            dim.width()
        ))
    }
}

/// Closed bounds of stratum `k` of `n` over `[lo, hi]`.
#[allow(clippy::cast_precision_loss)] // n and k are small batch counts; f64 is exact to 2^53
fn stratum_bounds(lo: f64, hi: f64, k: usize, n: usize) -> (f64, f64) {
    if n <= 1 {
        return (lo, hi);
    }
    let width = (hi - lo) / n as f64;
    let cell_lo = lo + k as f64 * width;
    // The topmost cell takes `hi` exactly rather than `lo + n*width`, which floating
    // point can leave a few ulps short of the declared upper bound.
    let cell_hi = if k + 1 >= n { hi } else { cell_lo + width };
    (cell_lo, cell_hi)
}

/// SplitMix64 — the fixed-increment generator Java's `SplittableRandom` uses.
///
/// Inline rather than a `rand` dependency: the point of `--seed` is that the same seed
/// produces the same batch on every machine, and an inline generator makes that a
/// property of this file rather than of a dependency's version resolution. Quality
/// requirements here are modest — the generator draws a handful of small permutations
/// per run, not a simulation.
struct SplitMix64 {
    state: u64,
}

impl SplitMix64 {
    fn new(seed: u64) -> Self {
        Self { state: seed }
    }

    fn next_u64(&mut self) -> u64 {
        self.state = self.state.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.state;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    /// A uniform-enough index in `0..n`. Modulo bias is bounded by `n / 2^64` and `n`
    /// here is a batch size, so it is unobservable.
    fn below(&mut self, n: usize) -> usize {
        if n == 0 {
            return 0;
        }
        #[allow(clippy::cast_possible_truncation)] // reduced mod n, which is a usize
        {
            (self.next_u64() % n as u64) as usize
        }
    }

    /// Fisher-Yates shuffle of `0..n`.
    fn permutation(&mut self, n: usize) -> Vec<usize> {
        let mut v: Vec<usize> = (0..n).collect();
        for i in (1..n).rev() {
            let j = self.below(i + 1);
            v.swap(i, j);
        }
        v
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dsl::types::{
        ConstraintModes, CoordinateSystem, OptimizationTarget, RoadSpec, ScenarioType,
    };
    use std::collections::HashMap;

    fn actor(
        id: &str,
        role: ActorRole,
        position: ValueOrRange,
        speed: ValueOrRange,
        direction: i32,
    ) -> crate::dsl::types::ActorSpec {
        crate::dsl::types::ActorSpec {
            id: id.to_string(),
            role,
            lane: 0,
            position,
            speed,
            acceleration: ValueOrRange::Range([-8.0, 3.0]),
            direction,
            behavior: HashMap::new(),
            lane_changes: vec![],
            bicycle_params: None,
        }
    }

    fn spec_with(actors: Vec<crate::dsl::types::ActorSpec>) -> ScenarioSpec {
        ScenarioSpec {
            scenario_type: ScenarioType::CutInLeft,
            time_step: 0.5,
            duration: 10.0,
            actors,
            min_ttc: 3.0,
            min_distance: 5.0,
            road: Some(RoadSpec {
                num_lanes: 2,
                lane_width: 3.5,
                lane_directions: vec![1, 1],
                road_length: None,
            }),
            lane_width: 3.5,
            num_scenarios: 5,
            constraint_modes: ConstraintModes::default(),
            optimization_target: OptimizationTarget::None,
            max_acceleration: None,
            max_deceleration: None,
            max_velocity: None,
            min_velocity: None,
            min_lateral_distance: None,
            max_relative_velocity: None,
            max_lateral_acceleration: 2.0,
            coordinate_system: CoordinateSystem::Cartesian,
            bicycle_config: None,
        }
    }

    /// The ego is stratified like anybody else — the defect this fixes was
    /// the ego being bit-identical across every scenario of a batch.
    #[test]
    fn ego_ranges_become_dimensions() {
        let spec = spec_with(vec![
            actor(
                "ego",
                ActorRole::Ego,
                ValueOrRange::Range([0.0, 55.0]),
                ValueOrRange::Range([14.0, 16.0]),
                1,
            ),
            actor(
                "npc",
                ActorRole::Npc,
                ValueOrRange::Range([20.0, 80.0]),
                ValueOrRange::Range([16.0, 20.0]),
                1,
            ),
        ]);
        let plan = DiversityPlan::new(&spec, 5, DEFAULT_DIVERSITY_SEED);
        assert_eq!(plan.dimension_count(), 4);

        let ego_dims: Vec<_> = plan
            .strata_for(0, 0)
            .into_iter()
            .filter(|s| s.actor_id == "ego")
            .collect();
        assert_eq!(ego_dims.len(), 2, "ego must be stratified, not skipped");
    }

    /// Pinned `Value` initial conditions have nothing to spread and must not become
    /// dimensions — a stratum on them would be an unsatisfiable band.
    #[test]
    fn pinned_values_are_not_dimensions() {
        let spec = spec_with(vec![actor(
            "ego",
            ActorRole::Ego,
            ValueOrRange::Value(10.0),
            ValueOrRange::Value(15.0),
            1,
        )]);
        let plan = DiversityPlan::new(&spec, 5, DEFAULT_DIVERSITY_SEED);
        assert_eq!(plan.dimension_count(), 0);
        assert_eq!(plan.max_relaxation(), 0);
        assert!(plan.strata_for(0, 0).is_empty());
    }

    /// A pedestrian's declared speed binds to `vy[0]`, not the `vx[0]` a stratum would
    /// constrain, so only its position is stratified.
    #[test]
    fn pedestrian_speed_is_not_stratified() {
        let spec = spec_with(vec![actor(
            "walker",
            ActorRole::Pedestrian,
            ValueOrRange::Range([10.0, 40.0]),
            ValueOrRange::Range([1.0, 2.0]),
            1,
        )]);
        let plan = DiversityPlan::new(&spec, 4, DEFAULT_DIVERSITY_SEED);
        assert_eq!(plan.dimension_count(), 1);
        assert_eq!(
            plan.strata_for(0, 0)[0].kind,
            DimensionKind::LongitudinalPosition
        );
    }

    /// The defining property of a Latin hypercube: across the batch, each dimension
    /// visits every stratum exactly once, and the strata tile the declared range.
    #[test]
    fn every_stratum_is_used_exactly_once_per_dimension() {
        let spec = spec_with(vec![actor(
            "npc",
            ActorRole::Npc,
            ValueOrRange::Range([20.0, 80.0]),
            ValueOrRange::Range([16.0, 20.0]),
            1,
        )]);
        let n = 5;
        let plan = DiversityPlan::new(&spec, n, DEFAULT_DIVERSITY_SEED);

        let mut seen: Vec<Vec<(f64, f64)>> = vec![Vec::new(); plan.dimension_count()];
        for i in 0..n {
            for (d, s) in plan.strata_for(i, 0).into_iter().enumerate() {
                seen[d].push((s.lo, s.hi));
            }
        }
        for cells in &seen {
            let mut sorted = cells.clone();
            sorted.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
            for w in sorted.windows(2) {
                assert!(
                    (w[0].1 - w[1].0).abs() < 1e-9,
                    "strata must tile the range without gaps or overlap: {sorted:?}"
                );
            }
            assert_eq!(sorted.len(), n, "each scenario gets one cell per dimension");
        }
    }

    /// A backward actor's `vx[0]` lives in `[-speed_max, -speed_min]`; the strata must
    /// be expressed in those same signed units or every cell is empty.
    #[test]
    fn backward_actors_get_negated_velocity_strata() {
        let spec = spec_with(vec![actor(
            "oncoming",
            ActorRole::Npc,
            ValueOrRange::Value(50.0),
            ValueOrRange::Range([10.0, 20.0]),
            -1,
        )]);
        let plan = DiversityPlan::new(&spec, 2, DEFAULT_DIVERSITY_SEED);
        for i in 0..2 {
            for s in plan.strata_for(i, 0) {
                assert!(
                    s.lo >= -20.0 && s.hi <= -10.0,
                    "stratum {s:?} outside the signed declared band"
                );
            }
        }
    }

    /// The ladder gives up the narrowest declared range first, and its top rung has no
    /// strata at all — that rung is what makes an `Unsat` after it a real `Unsat`.
    #[test]
    fn ladder_drops_narrowest_first_and_ends_empty() {
        let spec = spec_with(vec![actor(
            "npc",
            ActorRole::Npc,
            ValueOrRange::Range([20.0, 80.0]), // width 60
            ValueOrRange::Range([16.0, 20.0]), // width 4 — narrower, dropped first
            1,
        )]);
        let plan = DiversityPlan::new(&spec, 3, DEFAULT_DIVERSITY_SEED);
        assert_eq!(plan.max_relaxation(), 2);

        let after_one = plan.strata_for(0, 1);
        assert_eq!(after_one.len(), 1);
        assert_eq!(after_one[0].kind, DimensionKind::LongitudinalPosition);

        assert!(plan.strata_for(0, 2).is_empty());
    }

    /// `--seed` reproducibility: the same seed gives the same plan, a different seed
    /// gives a different one. If this stops holding the CLI flag is a lie.
    #[test]
    fn seeds_are_reproducible_and_distinguishable() {
        let spec = spec_with(vec![
            actor(
                "ego",
                ActorRole::Ego,
                ValueOrRange::Range([0.0, 55.0]),
                ValueOrRange::Range([14.0, 16.0]),
                1,
            ),
            actor(
                "npc",
                ActorRole::Npc,
                ValueOrRange::Range([20.0, 80.0]),
                ValueOrRange::Range([16.0, 20.0]),
                1,
            ),
        ]);

        let bounds = |seed: u64| -> Vec<(f64, f64)> {
            (0..8)
                .flat_map(|i| {
                    DiversityPlan::new(&spec, 8, seed)
                        .strata_for(i, 0)
                        .into_iter()
                        .map(|s| (s.lo, s.hi))
                        .collect::<Vec<_>>()
                })
                .collect()
        };

        assert_eq!(bounds(7), bounds(7), "same seed must give the same plan");
        assert_ne!(
            bounds(7),
            bounds(8),
            "different seeds must give different plans"
        );
    }

    /// SplitMix64 against the reference sequence for seed 0 (Vigna's `splitmix64.c`),
    /// so a typo in a constant cannot silently degrade the generator to something that
    /// still "looks random".
    #[test]
    fn splitmix64_matches_the_reference_sequence() {
        let mut rng = SplitMix64::new(0);
        assert_eq!(rng.next_u64(), 0xE220_A839_7B1D_CDAF);
        assert_eq!(rng.next_u64(), 0x6E78_9E6A_A1B9_65F4);
        assert_eq!(rng.next_u64(), 0x06C4_5D18_8009_454F);
    }
}
