//! Property-based tests over valid `ScenarioSpec`s (SW-07, item 4).
//!
//! Perturbs `examples/cut_in_left.yaml` (a cartesian cut-in with a lane
//! change) along one numeric axis at a time — lane width, ego speed, NPC
//! speed, the longitudinal gap between them, `min_ttc` or `min_distance` —
//! and asserts all six scenario invariants. It asserted only
//! [`common::Invariant::Envelope`] and
//! [`common::Invariant::ExtractionAgreement`] while the other four were on
//! `tests/common/invariants.rs::KNOWN_BROKEN_INVARIANTS`; that list emptied
//! with SW-22, so `Kinematics`, `Containment` and `ForwardProgress` became
//! live here too, and SW-27 fixed the exact-vs-`f64` same-lane divergence
//! that had kept `ConstraintModes` filtered on the `LaneWidth(3.2)` axis —
//! see the test below.
//!
//! ## Why perturb a real example rather than build a spec from scratch
//!
//! `ScenarioSpec` has enough cross-field structure (`road.lane_directions`
//! must match `road.num_lanes`, lane indices must be in range, a lane
//! change direction must be reachable from the actor's starting lane) that
//! a from-scratch generator would spend most of its logic re-deriving
//! `ScenarioSpec::validate`'s rules rather than testing anything — exactly
//! the "generator that emits invalid specs just tests the validator" trap
//! the issue calls out. Starting from a known-valid, known-solvable example
//! and perturbing only the free numeric parameters keeps every generated
//! spec valid *by construction*.
//!
//! ## Why discrete vetted values, not continuous ranges
//!
//! Two things were found while building this generator, in order:
//!
//! 1. Varying all six axes together in one `(Range, Range, ...).prop_map`
//!    hit *combinations* — e.g. NPC speed close to ego speed together with a
//!    small gap — that took the solver from ~1s to still not having returned
//!    after 20s+, even though each axis value in isolation solved in about a
//!    second. Fixed by restricting to one perturbed axis at a time.
//! 2. That alone was not enough: sweeping `lane_width` alone over
//!    `3.0..=4.0` in steps of 0.05 (holding every other field at the base
//!    example's value) found narrow **pathological bands** —
//!    `[3.1, 3.15]`, `[3.3, 3.35]`, `[3.9, 3.95]` — where the solve did not
//!    return within an 8s per-case timeout, sitting between fast (~1s)
//!    values on both sides. `ego_speed_min`, `npc_speed_min`, `gap`,
//!    `min_ttc` and `min_distance` showed no such cliffs when swept the same
//!    way. This means a *continuous* `proptest` range over `lane_width`
//!    would eventually sample a value in one of these bands and stall the
//!    suite — not flakiness (Z3 is deterministic here), a genuine
//!    input-sensitive solver-performance cliff. That is a solver finding
//!    worth its own issue (see the SW-07 report's "Out of scope, found
//!    anyway"), not something to paper over with `prop_assume` inside a test
//!    that is supposed to prove the generator's cases actually solve.
//!
//! The fix applied here: every axis draws from a short list of **concrete,
//! individually-timed values** (`prop::sample::select`) rather than a
//! continuous range, so the generator can never sample a value from an
//! unvetted region of the input space. This is still a property test — it
//! varies the spec across many valid combinations and asserts a checkable
//! postcondition — it just does not treat "any `f64` in a range" as the
//! population to draw from, because for `lane_width` on this solver that
//! population is not uniformly safe.
//!
//! ## Non-vacuousness
//!
//! Every case calls [`scenario_weaver::generate_single_scenario_from_spec`]
//! and `.expect()`s success rather than `prop_assume`-ing it away: if a
//! vetted value ever produced an infeasible spec, this test fails loudly
//! instead of silently discarding the case. A green run is therefore proof
//! that every one of the configured cases both solved and satisfied the two
//! invariants — see the SW-07 report for the observed case count and timing.

mod common;

use proptest::prelude::*;
use proptest::sample::select;

use scenario_weaver::dsl::types::{ScenarioSpec, ValueOrRange};

use common::check_scenario_invariants;

/// One axis of `examples/cut_in_left.yaml` to perturb, holding every other
/// field at the base example's value.
///
/// Every value below was individually timed with the solver (not just
/// assumed from its neighbours) — see the module docs, and in particular
/// why `LaneWidth`'s list excludes several values inside `3.0..=4.0`.
#[derive(Debug, Clone, Copy)]
enum Axis {
    /// Base 3.5 m. Excludes the measured-pathological bands
    /// `[3.1, 3.15]`, `[3.3, 3.35]`, `[3.9, 3.95]`.
    LaneWidth(f64),
    /// Base ego speed range is `[14, 16]`; this perturbs the lower bound,
    /// keeping the 2 m/s span.
    EgoSpeedMin(f64),
    /// Base NPC speed range is `[16, 20]`; this perturbs the lower bound,
    /// keeping the 4 m/s span.
    NpcSpeedMin(f64),
    /// Base NPC position range is `[20, 80]` (i.e. a 20 m longitudinal gap
    /// ahead of ego's `[0, 55]`); this perturbs that gap, keeping the 60 m
    /// span.
    Gap(f64),
    /// Base is exactly `3.0`; this only ever loosens it (`<= 3.0`).
    MinTtc(f64),
    /// Base is exactly `5.0`; this only ever loosens it (`<= 5.0`).
    MinDistance(f64),
}

/// `lane_width` values confirmed (individually, with the solver) to return
/// in about a second, avoiding the pathological bands documented above.
const LANE_WIDTHS: &[f64] = &[3.0, 3.2, 3.5, 3.6, 3.7, 3.8, 4.0];
const EGO_SPEED_MINS: &[f64] = &[13.0, 13.5, 14.0, 14.5, 15.0];
const NPC_SPEED_MINS: &[f64] = &[16.0, 17.0, 18.0, 19.0, 20.0];
const GAPS: &[f64] = &[20.0, 22.5, 25.0, 27.5, 30.0];
const MIN_TTCS: &[f64] = &[2.5, 2.6, 2.7, 2.8, 2.9, 3.0];
const MIN_DISTANCES: &[f64] = &[4.0, 4.25, 4.5, 4.75, 5.0];

fn axis_strategy() -> impl Strategy<Value = Axis> {
    prop_oneof![
        select(LANE_WIDTHS).prop_map(Axis::LaneWidth),
        select(EGO_SPEED_MINS).prop_map(Axis::EgoSpeedMin),
        select(NPC_SPEED_MINS).prop_map(Axis::NpcSpeedMin),
        select(GAPS).prop_map(Axis::Gap),
        select(MIN_TTCS).prop_map(Axis::MinTtc),
        select(MIN_DISTANCES).prop_map(Axis::MinDistance),
    ]
}

/// Apply one [`Axis`] perturbation to a fresh parse of `cut_in_left.yaml`.
fn apply_axis(axis: Axis) -> ScenarioSpec {
    let mut spec = common::parse_example("cut_in_left.yaml");

    match axis {
        Axis::LaneWidth(lane_width) => {
            if let Some(road) = spec.road.as_mut() {
                road.lane_width = lane_width;
            }
            spec.lane_width = lane_width;
        }
        Axis::EgoSpeedMin(ego_speed_min) => {
            let ego = spec
                .actors
                .iter_mut()
                .find(|a| a.id == "ego")
                .expect("cut_in_left.yaml has an 'ego' actor");
            ego.speed = ValueOrRange::Range([ego_speed_min, ego_speed_min + 2.0]);
        }
        Axis::NpcSpeedMin(npc_speed_min) => {
            let npc = spec
                .actors
                .iter_mut()
                .find(|a| a.id == "npc")
                .expect("cut_in_left.yaml has an 'npc' actor");
            npc.speed = ValueOrRange::Range([npc_speed_min, npc_speed_min + 4.0]);
        }
        Axis::Gap(gap) => {
            let npc = spec
                .actors
                .iter_mut()
                .find(|a| a.id == "npc")
                .expect("cut_in_left.yaml has an 'npc' actor");
            npc.position = ValueOrRange::Range([gap, gap + 60.0]);
        }
        Axis::MinTtc(min_ttc) => spec.min_ttc = min_ttc,
        Axis::MinDistance(min_distance) => spec.min_distance = min_distance,
    }

    spec
}

proptest! {
    // Kept low so the whole suite stays well under two minutes: each case
    // is one full Z3 solve plus invariant checking (~1-1.6s warm, all
    // individually vetted — see the module docs), so 30 cases is
    // comfortably under a minute for this file alone.
    //
    // `failure_persistence: None` disables proptest's regression-file
    // writing (it would otherwise try to write next to a `src/lib.rs` this
    // `tests/` crate does not have, and warn every run).
    #![proptest_config(ProptestConfig {
        cases: 30,
        failure_persistence: None,
        .. ProptestConfig::default()
    })]

    /// Every single-axis perturbation of `cut_in_left`, drawn from a vetted
    /// discrete value list per axis, solves — and its generated scenario
    /// satisfies **all six** scenario invariants.
    ///
    /// It used to assert only [`common::Invariant::Envelope`] and
    /// [`common::Invariant::ExtractionAgreement`], because the other four were on
    /// `tests/common/invariants.rs::KNOWN_BROKEN_INVARIANTS` and failed by
    /// construction. SW-22 emptied that list, so `Kinematics`, `Containment` and
    /// `ForwardProgress` became live here.
    ///
    /// [`common::Invariant::ConstraintModes`] was the last one filtered, on the
    /// `LaneWidth(3.2)` axis, and **SW-27 retired that filter** — this test is
    /// that fix's ratchet, so the filter must not come back. The defect it held
    /// back: at `lane_width = 3.2` the two lane centres are 1.6 and 4.8, so
    /// adjacent-lane actors sit at `|Δpy| = lane_width` *exactly*, the boundary
    /// of the shared "same lane" predicate. The encoder evaluated it over Z3's
    /// exact rationals and got `false`, so it asserted no `min_distance`
    /// between them; `compute_validation_metrics` evaluated the same expression
    /// in `f64`, where the subtraction lands on 3.1999999999999997, got `true`,
    /// and reported the untouched gap as a breach — `min_distance: 2.02 m`
    /// against an enforced 5.0 m, `all_constraints_satisfied: false`, measured
    /// at `93f2ab2` (and 1.82 m at `894409b`, before SW-22's conflict
    /// requirement, so it predated SW-22).
    ///
    /// Both sides now compare against `encoder_utils::lane_overlap_threshold`,
    /// one constant, with the boundary excluded: two actors exactly one lane
    /// width apart are in adjacent lanes and are not a conflict pair.
    ///
    /// The corpus-wide ratchet is unaffected and stays at full strength:
    /// `examples_smoke_test::test_known_broken_invariants_are_still_broken`
    /// reports zero breaches of any invariant across all 22 examples.
    #[test]
    fn scenario_invariants_hold_over_perturbed_cut_in_left(
        axis in axis_strategy(),
    ) {
        let spec = apply_axis(axis);

        spec.validate()
            .unwrap_or_else(|e| panic!("generator must only emit valid specs, got: {e}"));

        let scenario = scenario_weaver::generate_single_scenario_from_spec(spec.clone())
            .unwrap_or_else(|e| {
                panic!(
                    "generator's vetted values are chosen to stay solvable, but the \
                     solver returned: {e} for axis {axis:?}"
                )
            });

        let found: Vec<String> = check_scenario_invariants(&scenario, &spec)
            .into_iter()
            .map(|v| v.to_string())
            .collect();
        prop_assert!(
            found.is_empty(),
            "axis {axis:?}: {} scenario invariant violation(s):\n  {}",
            found.len(),
            found.join("\n  ")
        );
    }
}
