//! Physical and semantic invariants that every generated scenario must satisfy.
//!
//! Before this module the suite asserted that generation *returned* something.
//! `tests/pedestrian_physics_test.rs` was the only file that looked at the
//! numbers, and only for pedestrians, in one scenario type, at a `1e-1`
//! tolerance against `dt = 0.5` — roughly ten percent of slack, enough to hide
//! a sign error on a small `vy`.
//!
//! # Tolerance
//!
//! [`TOL`] is `1e-6`. Z3 works in exact rationals and extraction is
//! deterministic (verified: byte-identical output across repeated runs, no RNG,
//! default `Config::new()`), so the only error between an asserted constraint
//! and an extracted `f64` is the rational-to-double rounding. There is no
//! physical source of slack, and a tolerance wide enough to absorb a modelling
//! error is a tolerance that hides one. **Never widen this to make something
//! pass** — if a check fails, either the invariant is wrong or the code is, and
//! the failing case belongs behind `#[ignore = "SW-NN"]` until the owning issue
//! lands.
//!
//! # Envelope bounds come from the spec
//!
//! Every bound checked here is read off the [`ScenarioSpec`] (or, for
//! pedestrians, off the `PEDESTRIAN_*` constants the encoder itself uses).
//! Nothing is transcribed as a literal. The old
//! `pedestrian_physics_test::test_pedestrian_crossing_acceleration_bounds`
//! hardcoded `1.0` for the acceleration bound while correctly importing
//! `PEDESTRIAN_WALK_MAX_SPEED` for the speed bound; a hardcoded bound stops
//! testing the code the moment the code changes.
//!
//! # Constraint modes
//!
//! A bound is only checked when its [`ConstraintMode`] is `Enforce`. Under
//! `Violate` the scenario is *supposed* to breach it, and under `Ignore`
//! nothing was asserted at all, so a check would be testing the solver's
//! arbitrary choice rather than the specification.

use scenario_weaver::dsl::types::{
    ActorRole, ActorSpec, ConstraintMode, ScenarioSpec, PEDESTRIAN_MAX_ACCELERATION,
    PEDESTRIAN_MAX_DECELERATION, PEDESTRIAN_RUN_MAX_SPEED, PEDESTRIAN_WALK_MAX_SPEED,
};
use scenario_weaver::scenario::model::{ActorTrajectory, Scenario};
use scenario_weaver::solver::encoder::SIDEWALK_WIDTH;
use scenario_weaver::solver::encoder_utils::same_lane_f64;

/// Numeric tolerance for every comparison in this module.
///
/// See the module docs: this is the rounding error of an exact rational
/// rendered as `f64`, not an allowance for modelling error.
pub const TOL: f64 = 1e-6;

/// The seven properties a generated scenario is committed to.
///
/// Named so a failure can be attributed to one property rather than to "the
/// invariants failed", and so a test can assert one property in isolation while
/// the rest are still known-broken.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Invariant {
    /// `p[i+1] = p[i] + v[i]·dt + ½·a[i]·dt²` and `v[i+1] = v[i] + a[i]·dt`,
    /// longitudinally *and* laterally, for every actor at every step.
    Kinematics,
    /// Speed, acceleration and lateral-acceleration bounds, sourced from the spec.
    Envelope,
    /// `Enforce ⟹ metric ≥ threshold`, `Violate ⟹ metric < threshold` strictly,
    /// `Ignore ⟹ nothing asserted`.
    ConstraintModes,
    /// Lateral position inside the road surface, and `lane` consistent with `y`.
    Containment,
    /// Not every vehicle is stationary for a majority of the horizon.
    ForwardProgress,
    /// `scenario.validation` re-derives from the extracted trajectories.
    ExtractionAgreement,
    /// A `pedestrian_crossing` pedestrian's shipped trajectory actually
    /// reaches the sidewalk opposite its declared crossing direction
    /// (SW-42) — the goal `generate_ltl` asserts as `on_opposite_sidewalk.
    /// eventually()`, confirmed here independently of the encoder rather
    /// than trusted from it.
    Liveness,
}

impl std::fmt::Display for Invariant {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let s = match self {
            Invariant::Kinematics => "kinematic consistency",
            Invariant::Envelope => "envelope compliance",
            Invariant::ConstraintModes => "constraint-mode semantics",
            Invariant::Containment => "lane/road containment",
            Invariant::ForwardProgress => "forward progress",
            Invariant::ExtractionAgreement => "model-vs-extraction agreement",
            Invariant::Liveness => "pedestrian crossing liveness",
        };
        f.write_str(s)
    }
}

/// One concrete breach: which property, and the numbers that broke it.
#[derive(Debug, Clone)]
pub struct Violation {
    pub invariant: Invariant,
    pub detail: String,
}

impl std::fmt::Display for Violation {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "[{}] {}", self.invariant, self.detail)
    }
}

// ---------------------------------------------------------------------------
// The known-broken baseline
// ---------------------------------------------------------------------------

/// Invariants that no scenario in this repository satisfies today, each with
/// the issue that owns the fix. **Empty since SW-22** — all seven now hold
/// across the corpus (SW-42 added the seventh, [`Invariant::Liveness`],
/// already holding on every example), and [`assert_scenario_invariants`]
/// asserts all seven at every call site.
///
/// This is a **ratchet**, deliberately shaped like `scripts/check.sh`'s clippy
/// baseline. [`assert_scenario_invariants`] is called from every test that
/// generates a scenario; when all six were broken it would have failed on every
/// one of them, so it would have had to be `#[ignore]`d everywhere and would
/// have asserted nothing at all. Instead it asserts everything *except* the
/// entries below, so each invariant becomes live the moment its entry goes.
///
/// The list is not an escape hatch:
///
/// - `test_known_broken_invariants_are_still_broken` fails if an entry stops
///   being violated anywhere in the corpus, forcing it to be deleted rather
///   than left to rot. That is how each of the five retirements below was
///   found — the ratchet fired first, and the firing *is* the proof.
/// - `test_all_scenario_invariants` (ignored) asserts all six at full strength,
///   and its output is the standing record of what is broken.
///
/// Never add an entry to silence a new failure. An entry means "an open issue
/// owns this and the fix is scheduled".
///
/// [`Invariant::ForwardProgress`] was retired by SW-12, [`Invariant::Kinematics`]
/// by SW-09 and [`Invariant::Containment`] by SW-23.
///
/// [`Invariant::ConstraintModes`] was retired by SW-22 and the list is now
/// **empty**: every invariant is asserted at every call site. Its two halves
/// went separately. `violate`-at-equality was SW-12's (`DistanceGT` lowered
/// strictly, so its negation was satisfied *at* the threshold the validator
/// calls safe). `enforce`-on-an-unevaluated-metric was this one, and the entry
/// text blamed the wrong thing: it said "the lane variable lags lateral
/// position so no same-lane step exists", which SW-10 had already fixed. The
/// real cause is that `Always(TTCGT(..))` is a guarded implication and nothing
/// required a pair to converge, so `min_ttc` came back `None` — neither
/// enforced nor violated — on every `cut_in_left` / `cut_in_right` example in
/// the corpus. `scenarios::cut_in_conflict` supplies the missing antecedent.
///
/// Kept as an empty constant rather than deleted: it is the documented place a
/// future agent would be tempted to add an entry, and the ratchet around it
/// (`test_known_broken_invariants_are_still_broken`) is what makes adding one
/// cost something.
pub const KNOWN_BROKEN_INVARIANTS: &[(Invariant, &str)] = &[];

/// True when `invariant` is on the known-broken baseline.
#[must_use]
pub fn is_known_broken(invariant: Invariant) -> bool {
    KNOWN_BROKEN_INVARIANTS.iter().any(|(i, _)| *i == invariant)
}

// ---------------------------------------------------------------------------
// Entry points
// ---------------------------------------------------------------------------

/// Assert every invariant that is not on the [`KNOWN_BROKEN_INVARIANTS`]
/// baseline, panicking with all breaches at once.
///
/// This is the call that belongs in every test that generates a scenario; the
/// shared generators in `tests/common/mod.rs` make it automatic. Breaches are
/// collected before the panic so one broken property does not hide the others.
///
/// # Panics
/// If any invariant outside the baseline is violated.
pub fn assert_scenario_invariants(scenario: &Scenario, spec: &ScenarioSpec) {
    let found: Vec<Violation> = check_scenario_invariants(scenario, spec)
        .into_iter()
        .filter(|v| !is_known_broken(v.invariant))
        .collect();
    report(scenario, &found);
}

/// Assert all six invariants at full strength, baseline ignored.
///
/// Fails today. Its output is the record of what SW-08, SW-10 and SW-12 have
/// left to fix.
///
/// # Panics
/// If any invariant is violated.
pub fn assert_all_scenario_invariants(scenario: &Scenario, spec: &ScenarioSpec) {
    report(scenario, &check_scenario_invariants(scenario, spec));
}

/// Assert a single invariant in isolation.
///
/// Used where the other properties are known-broken against an open issue: the
/// property under test still gets asserted at full strength.
///
/// # Panics
/// If `invariant` is violated.
pub fn assert_invariant(scenario: &Scenario, spec: &ScenarioSpec, invariant: Invariant) {
    let found: Vec<Violation> = check_scenario_invariants(scenario, spec)
        .into_iter()
        .filter(|v| v.invariant == invariant)
        .collect();
    report(scenario, &found);
}

fn report(scenario: &Scenario, found: &[Violation]) {
    assert!(
        found.is_empty(),
        "{}: {} scenario invariant violation(s):\n  {}",
        scenario.scenario_type,
        found.len(),
        found
            .iter()
            .map(ToString::to_string)
            .collect::<Vec<_>>()
            .join("\n  ")
    );
}

/// Evaluate every invariant and return the breaches, in `Invariant` order.
#[must_use]
pub fn check_scenario_invariants(scenario: &Scenario, spec: &ScenarioSpec) -> Vec<Violation> {
    let mut v = Vec::new();
    check_kinematics(scenario, spec, &mut v);
    check_envelope(scenario, spec, &mut v);
    check_constraint_modes(scenario, spec, &mut v);
    check_containment(scenario, spec, &mut v);
    check_forward_progress(scenario, spec, &mut v);
    check_extraction_agreement(scenario, spec, &mut v);
    check_pedestrian_crossing_liveness(scenario, spec, &mut v);
    v.sort_by_key(|x| x.invariant);
    v
}

fn push(out: &mut Vec<Violation>, invariant: Invariant, detail: String) {
    out.push(Violation { invariant, detail });
}

/// The spec entry for an extracted trajectory. Extraction is driven by the
/// spec's actor list, so a missing entry is itself a defect worth reporting.
fn spec_actor<'a>(spec: &'a ScenarioSpec, traj: &ActorTrajectory) -> Option<&'a ActorSpec> {
    spec.get_actor(&traj.id)
}

// ---------------------------------------------------------------------------
// 1. Kinematic consistency
// ---------------------------------------------------------------------------

/// Position and velocity must integrate the accelerations that are reported
/// alongside them, on both axes.
///
/// The position form asserted here is the exact constant-acceleration update
/// `p + v·dt + ½·a·dt²`, not the forward-Euler `p + v·dt` the encoder currently
/// writes (finding H1). Forward Euler is not a discretisation choice here: the
/// velocity update keeps the full `a·dt` term, so `p` and `v` are integrated
/// under two different assumptions and the trajectory in the JSON is not the
/// trajectory the velocities describe. At `dt = 0.5, a = 3` the gap is 0.375 m
/// per step, about 7.5 m over twenty steps — larger than a typical
/// `min_distance` threshold, so it corrupts the very margins being enforced.
/// The correct form is still linear in `a` and costs the solver nothing.
///
/// The failure message reports both residuals, so a breach shows at a glance
/// whether the trajectory is forward-Euler (½a·dt² residual) or incoherent in
/// some other way.
fn check_kinematics(scenario: &Scenario, _spec: &ScenarioSpec, out: &mut Vec<Violation>) {
    let dt = scenario.time_step;
    let half_dt2 = 0.5 * dt * dt;

    for traj in &scenario.actors {
        for w in traj.states.windows(2) {
            let (s, next) = (&w[0], &w[1]);
            let (p, v, a) = (s.position(), s.velocity(), s.acceleration());
            let np = next.position();
            let nv = next.velocity();

            for (axis, got, from_p, from_v, from_a) in
                [("x", np.x, p.x, v.vx, a.ax), ("y", np.y, p.y, v.vy, a.ay)]
            {
                let expected = from_p + from_v * dt + from_a * half_dt2;
                let euler = from_p + from_v * dt;
                if (got - expected).abs() > TOL {
                    push(
                        out,
                        Invariant::Kinematics,
                        format!(
                            "{}: p{axis}[t={:.2}s] = {got:.9}, but p{axis} + v{axis}·dt + \
                             ½·a{axis}·dt² = {expected:.9} (p={from_p:.9}, v={from_v:.9}, \
                             a={from_a:.9}, dt={dt}); residual {:.3e}, forward-Euler residual \
                             {:.3e}",
                            traj.id,
                            next.time,
                            got - expected,
                            got - euler,
                        ),
                    );
                }
            }

            for (axis, got, from_v, from_a) in [("x", nv.vx, v.vx, a.ax), ("y", nv.vy, v.vy, a.ay)]
            {
                let expected = from_v + from_a * dt;
                if (got - expected).abs() > TOL {
                    push(
                        out,
                        Invariant::Kinematics,
                        format!(
                            "{}: v{axis}[t={:.2}s] = {got:.9}, but v{axis} + a{axis}·dt = \
                             {expected:.9} (v={from_v:.9}, a={from_a:.9}, dt={dt}); residual \
                             {:.3e}",
                            traj.id,
                            next.time,
                            got - expected,
                        ),
                    );
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// 2. Envelope compliance
// ---------------------------------------------------------------------------

/// A pedestrian's speed ceiling, read from the same `walking_mode` behaviour key
/// the encoder reads (`src/solver/encoders/pedestrian.rs`).
fn pedestrian_max_speed(actor: &ActorSpec) -> f64 {
    actor
        .behavior
        .get("walking_mode")
        .map_or(PEDESTRIAN_WALK_MAX_SPEED, |mode| match mode.as_str() {
            Some("run") => PEDESTRIAN_RUN_MAX_SPEED,
            _ => PEDESTRIAN_WALK_MAX_SPEED,
        })
}

/// Every bound here is read from the spec (or from the pedestrian constants the
/// encoder itself uses), and each is checked only in `Enforce` mode.
fn check_envelope(scenario: &Scenario, spec: &ScenarioSpec, out: &mut Vec<Violation>) {
    let modes = &spec.constraint_modes;

    for traj in &scenario.actors {
        let Some(actor) = spec_actor(spec, traj) else {
            continue;
        };
        let is_ped = actor.role == ActorRole::Pedestrian;

        for st in &traj.states {
            let (v, a) = (st.velocity(), st.acceleration());
            let at = format!("{} at t={:.2}s", traj.id, st.time);

            if is_ped {
                // The encoder bounds each pedestrian axis independently (a box,
                // not a disk); the constants are pre-divided by sqrt(2) to keep
                // the diagonal speed semantics. Check the same box.
                let vmax = pedestrian_max_speed(actor);
                for (axis, val) in [("vx", v.vx), ("vy", v.vy)] {
                    if val.abs() > vmax + TOL {
                        push(
                            out,
                            Invariant::Envelope,
                            format!(
                                "{at}: |{axis}| = {:.9} exceeds pedestrian ceiling {vmax}",
                                val.abs()
                            ),
                        );
                    }
                }
                for (axis, val) in [("ax", a.ax), ("ay", a.ay)] {
                    if val > PEDESTRIAN_MAX_ACCELERATION + TOL
                        || val < PEDESTRIAN_MAX_DECELERATION - TOL
                    {
                        push(
                            out,
                            Invariant::Envelope,
                            format!(
                                "{at}: {axis} = {val:.9} outside pedestrian band \
                                 [{PEDESTRIAN_MAX_DECELERATION}, {PEDESTRIAN_MAX_ACCELERATION}]"
                            ),
                        );
                    }
                }
                continue;
            }

            if let (Some(vmax), ConstraintMode::Enforce) = (spec.max_velocity, modes.max_velocity())
            {
                if v.vx.abs() > vmax + TOL {
                    push(
                        out,
                        Invariant::Envelope,
                        format!(
                            "{at}: |vx| = {:.9} exceeds spec.max_velocity {vmax} \
                             (declared max_velocity: enforce)",
                            v.vx.abs()
                        ),
                    );
                }
            }
            if let (Some(vmin), ConstraintMode::Enforce) = (spec.min_velocity, modes.min_velocity())
            {
                if v.vx.abs() < vmin - TOL {
                    push(
                        out,
                        Invariant::Envelope,
                        format!(
                            "{at}: |vx| = {:.9} below spec.min_velocity {vmin} \
                             (declared min_velocity: enforce)",
                            v.vx.abs()
                        ),
                    );
                }
            }
            if modes.max_acceleration() == ConstraintMode::Enforce {
                if let Some(amax) = spec.max_acceleration {
                    if a.ax > amax + TOL {
                        push(
                            out,
                            Invariant::Envelope,
                            format!(
                                "{at}: ax = {:.9} exceeds spec.max_acceleration {amax}",
                                a.ax
                            ),
                        );
                    }
                }
                if let Some(amin) = spec.max_deceleration {
                    if a.ax < amin - TOL {
                        push(
                            out,
                            Invariant::Envelope,
                            format!("{at}: ax = {:.9} below spec.max_deceleration {amin}", a.ax),
                        );
                    }
                }
            }

            // `max_lateral_acceleration` has no ConstraintMode of its own: the
            // spec field is unconditional (`default_max_lateral_acceleration`),
            // so it is a hard envelope for every vehicle at every step.
            let ay_max = spec.max_lateral_acceleration;
            if a.ay.abs() > ay_max + TOL {
                push(
                    out,
                    Invariant::Envelope,
                    format!(
                        "{at}: |ay| = {:.9} exceeds spec.max_lateral_acceleration {ay_max}",
                        a.ay.abs()
                    ),
                );
            }
        }
    }
}

// ---------------------------------------------------------------------------
// 3. Constraint-mode semantics
// ---------------------------------------------------------------------------

/// The property `ConstraintMode` exists to express, and which nothing tested.
///
/// `Violate` is checked **strictly**: `metric < threshold`. Equality is the
/// defect this catches — `cut_in_left_adversarial_*` reports
/// `min_distance == 5.0` against a threshold of exactly 5.0 and calls it a
/// violation.
///
/// A metric of `None` (SW-04: never evaluated) is a breach under both `Enforce`
/// and `Violate`: a constraint the validator never evaluates is neither
/// enforced nor violated, whatever the YAML says.
fn check_constraint_modes(scenario: &Scenario, spec: &ScenarioSpec, out: &mut Vec<Violation>) {
    // SW-44. A `pedestrian_crossing` spec is checked against the constraints it
    // was actually given, which are not the two scalars below.
    //
    // `validation.min_ttc` and `validation.min_distance` are accumulated by
    // `compute_validation_metrics` from a `same_lane`-gated *longitudinal*
    // reading, for every pair including a pedestrian's. For a perpendicular
    // crossing that reading means nothing — and it is never empty either, since
    // `pedestrian_crossing.rs::add_z3_constraints` pins the pedestrian's lane to
    // 0 and the ego is usually in lane 0, so `same_lane` is structurally true
    // and every step contributes. `metrics.rs` says so itself, in the doc note
    // on its pedestrian branch: "the generic `distance < min_distance` reading
    // has no bearing on whether the box the encoder actually asserted was
    // satisfied", which is why SW-31 stopped *reporting* it for these pairs.
    // What `pedestrian_crossing.rs::generate_safety` asserts instead is
    // `RectangularDistanceGT` for `min_distance` and `PedestrianTTCGT` for
    // `min_ttc`, and `compute_validation_metrics` measures exactly those.
    //
    // Applying the scalar check here was therefore a false positive waiting for
    // the right model: an ego stopped 5.3 m to the side of a pedestrian on the
    // far sidewalk has a *longitudinal* separation of 0 m and is entirely safe
    // by every constraint the encoder asserted, and this invariant called it an
    // `enforce` breach. Every `enforce`d pedestrian spec fails the scalar check
    // — measured at `76dd7e8`, before any of this wave's edits — and
    // `test_declared_braking_manoeuvre_may_end_at_rest` passed only because Z3
    // happened to return a model with a large longitudinal gap; adding SW-44's
    // between-steps guard, which that model satisfies, was enough to move Z3 to
    // a different and equally valid one.
    //
    // The replacement reads the validator's own verdict rather than
    // re-deriving the box here: the thresholds are
    // `min_distance / PEDESTRIAN_BOX_{LONGITUDINAL,LATERAL}_DIVISOR`, and those
    // divisors are `pub(crate)` precisely so there is one copy of each number
    // (SW-34). Copying them into the test suite to re-derive the box would
    // recreate the divergence they were hoisted to prevent. It is a stronger
    // check than the one it replaces, not a weaker one: the scalar never looked
    // at the box, the lateral axis, the TTC guard or the between-steps guard,
    // and this looks at all four.
    if spec.scenario_type == scenario_weaver::dsl::types::ScenarioType::PedestrianCrossing {
        check_pedestrian_constraint_modes(scenario, spec, out);
        return;
    }

    let checks = [
        (
            "min_ttc",
            spec.constraint_modes.min_ttc(),
            spec.min_ttc,
            scenario.validation.min_ttc,
            "s",
        ),
        (
            "min_distance",
            spec.constraint_modes.min_distance(),
            spec.min_distance,
            scenario.validation.min_distance,
            "m",
        ),
    ];

    for (name, mode, threshold, measured, unit) in checks {
        match (mode, measured) {
            (ConstraintMode::Ignore, _) => {}
            (ConstraintMode::Enforce, Some(m)) if m >= threshold - TOL => {}
            (ConstraintMode::Violate, Some(m)) if m < threshold - TOL => {}
            (mode, Some(m)) => push(
                out,
                Invariant::ConstraintModes,
                format!(
                    "{name}: declared {mode:?} against threshold {threshold}{unit}, \
                     measured {m:.9}{unit} — {}",
                    match mode {
                        ConstraintMode::Enforce => "enforce requires metric >= threshold",
                        _ => "violate requires metric < threshold strictly",
                    }
                ),
            ),
            (mode, None) => push(
                out,
                Invariant::ConstraintModes,
                format!(
                    "{name}: declared {mode:?} against threshold {threshold}{unit}, but the \
                     metric was never evaluated — a constraint the validator never evaluates \
                     is neither enforced nor violated \
                     (all_constraints_satisfied={}, safety_violations={:?})",
                    scenario.validation.all_constraints_satisfied,
                    scenario.validation.safety_violations
                ),
            ),
        }
    }
}

/// [`check_constraint_modes`] for `pedestrian_crossing`, against the
/// propositions that scenario type actually lowers.
///
/// `Enforce` means `compute_validation_metrics` reported no violation of the
/// field's own proposition; `Violate` means it reported at least one. The
/// strings are the ones `metrics.rs`'s `pedestrian_pair` branch emits — the
/// distance box, its SW-44 between-steps guard, and the guarded pedestrian TTC.
fn check_pedestrian_constraint_modes(
    scenario: &Scenario,
    spec: &ScenarioSpec,
    out: &mut Vec<Violation>,
) {
    let violations = &scenario.validation.safety_violations;
    let matching = |needles: &[&str]| -> Vec<String> {
        violations
            .iter()
            .filter(|v| needles.iter().any(|n| v.contains(n)))
            .cloned()
            .collect()
    };

    for (name, mode, found) in [
        (
            "min_distance",
            spec.constraint_modes.min_distance(),
            matching(&[
                "Pedestrian distance-box violation",
                "Pedestrian box tunnelling",
            ]),
        ),
        (
            "min_ttc",
            spec.constraint_modes.min_ttc(),
            matching(&["Pedestrian TTC violation"]),
        ),
    ] {
        match mode {
            ConstraintMode::Ignore => {}
            ConstraintMode::Enforce if found.is_empty() => {}
            ConstraintMode::Violate if !found.is_empty() => {}
            ConstraintMode::Enforce => push(
                out,
                Invariant::ConstraintModes,
                format!(
                    "{name}: declared Enforce, and the validator reported {} breach(es) of the \
                     proposition pedestrian_crossing.rs lowers for it: {found:?}",
                    found.len()
                ),
            ),
            ConstraintMode::Violate => push(
                out,
                Invariant::ConstraintModes,
                format!(
                    "{name}: declared Violate, and the validator reported no breach of the \
                     proposition pedestrian_crossing.rs lowers for it — a constraint the \
                     validator never finds broken is not violated, whatever the YAML says \
                     (safety_violations={violations:?})"
                ),
            ),
        }
    }
}

// ---------------------------------------------------------------------------
// 4. Lane / road containment
// ---------------------------------------------------------------------------

/// Every actor stays on the road surface, and the discrete `lane` it is
/// recorded in matches where it actually is.
///
/// The lane-centre form `|py − lane·w − w/2| ≤ w/2` is the relation finding H2
/// proposes: today `lane` is pinned on a schedule while `py` is pinned only at
/// the endpoints of a lane change, so for one to three seconds an actor is
/// physically in the ego's lane while `lane` still reads the old one. Every
/// consumer keyed on `lane` — the TTC proposition's same-lane test,
/// `compute_effective_dist`, `compute_validation_metrics` — then treats the two
/// actors as separated during exactly the window a cut-in scenario is about.
///
/// # Pedestrians (SW-23)
///
/// A pedestrian legitimately leaves `[0, road_top]`: that is what
/// `CrossingRoad`/`OnSidewalk` mean, and `src/solver/encoder.rs`'s
/// `SIDEWALK_WIDTH` strip is the bound on how far. So a pedestrian's `py` is
/// checked against `[-SIDEWALK_WIDTH, road_top + SIDEWALK_WIDTH]` instead —
/// the same envelope `encode_pedestrian_lateral_containment`
/// (`src/solver/encoders/pedestrian.rs`) now asserts at every step, and the
/// same floor `xodr_exporter::sidewalk_widths` measures the `.xodr` against.
/// The lane-centre check is skipped for pedestrians entirely: `lane` has no
/// semantic meaning for them (`pedestrian_crossing.rs` pins it to 0
/// throughout — only `py` carries position), so comparing it to a lane centre
/// tests an artifact of the encoding, not a property of the scenario.
fn check_containment(scenario: &Scenario, spec: &ScenarioSpec, out: &mut Vec<Violation>) {
    let w = spec.get_lane_width();
    let num_lanes = spec.get_num_lanes();
    let road_top = num_lanes as f64 * w;

    for traj in &scenario.actors {
        let is_ped = spec_actor(spec, traj).is_some_and(|a| a.role == ActorRole::Pedestrian);
        let (lower, upper) = if is_ped {
            (-SIDEWALK_WIDTH, road_top + SIDEWALK_WIDTH)
        } else {
            (0.0, road_top)
        };

        for st in &traj.states {
            let py = st.position().y;
            let lane = st.lane();
            let at = format!("{} at t={:.2}s", traj.id, st.time);

            if py < lower - TOL || py > upper + TOL {
                push(
                    out,
                    Invariant::Containment,
                    format!(
                        "{at}: py = {py:.9} outside [{lower}, {upper}] \
                         ({num_lanes} lanes x {w} m{})",
                        if is_ped { " + sidewalk margin" } else { "" }
                    ),
                );
            }

            if is_ped {
                // `lane` carries no position information for pedestrians;
                // only `py`, already checked above.
                continue;
            }

            if lane >= num_lanes {
                push(
                    out,
                    Invariant::Containment,
                    format!("{at}: lane = {lane} but the road has {num_lanes} lanes"),
                );
                continue;
            }

            let centre = lane as f64 * w + w / 2.0;
            if (py - centre).abs() > w / 2.0 + TOL {
                push(
                    out,
                    Invariant::Containment,
                    format!(
                        "{at}: recorded lane {lane} (centre {centre:.9}, half-width {:.9}) \
                         but py = {py:.9}; |py - lane·w - w/2| = {:.9}",
                        w / 2.0,
                        (py - centre).abs()
                    ),
                );
            }
        }
    }
}

// ---------------------------------------------------------------------------
// 5. Forward progress
// ---------------------------------------------------------------------------

/// A scenario in which every vehicle is parked for most of the horizon is not
/// the scenario the YAML describes, however satisfiable it is.
///
/// `cut_in_left` currently produces exactly that — both vehicles at `vx = 0`
/// for seven seconds — and reports `all_constraints_satisfied: true`, because a
/// stationary pair trivially satisfies every distance and TTC threshold.
///
/// Pedestrians are excluded: a pedestrian crossing perpendicular to the road
/// legitimately has `vx = 0` throughout.
fn check_forward_progress(scenario: &Scenario, spec: &ScenarioSpec, out: &mut Vec<Violation>) {
    let vehicles: Vec<&ActorTrajectory> = scenario
        .actors
        .iter()
        .filter(|t| spec_actor(spec, t).is_some_and(|a| a.role != ActorRole::Pedestrian))
        .collect();
    if vehicles.is_empty() {
        return;
    }

    let steps = vehicles[0].states.len();
    let stalled = (0..steps)
        .filter(|&i| {
            vehicles.iter().all(|t| {
                t.states
                    .get(i)
                    .is_some_and(|s| s.velocity().vx.abs() <= TOL)
            })
        })
        .count();

    if stalled * 2 > steps {
        let per_actor: Vec<String> = vehicles
            .iter()
            .map(|t| {
                let n = t
                    .states
                    .iter()
                    .filter(|s| s.velocity().vx.abs() <= TOL)
                    .count();
                format!("{}: {n}/{} steps at vx=0", t.id, t.states.len())
            })
            .collect();
        push(
            out,
            Invariant::ForwardProgress,
            format!(
                "every vehicle is stationary for {stalled} of {steps} steps \
                 ({:.1} s of a {:.1} s horizon), a majority — a parked pair satisfies every \
                 distance and TTC threshold trivially; all_constraints_satisfied={} [{}]",
                stalled as f64 * scenario.time_step,
                scenario.duration,
                scenario.validation.all_constraints_satisfied,
                per_actor.join(", ")
            ),
        );
    }
}

// ---------------------------------------------------------------------------
// 6. Model-vs-extraction agreement
// ---------------------------------------------------------------------------

/// Re-derive `scenario.validation` from the extracted trajectories and compare.
///
/// The point is *where* the numbers come from. `compute_validation_metrics`
/// runs inside the encoder with the Z3 model in hand; this re-derivation sees
/// only the `Scenario` struct that every output format is built from. Anything
/// `src/scenario/extractor.rs` loses, reorders or rounds between the two shows
/// up as a disagreement here. That file sits between the solver and every
/// exporter and is otherwise covered only by its own inline unit tests.
fn check_extraction_agreement(scenario: &Scenario, spec: &ScenarioSpec, out: &mut Vec<Violation>) {
    let mut min_ttc: Option<f64> = None;
    let mut min_distance: Option<f64> = None;

    for (i, a1) in scenario.actors.iter().enumerate() {
        for a2 in scenario.actors.iter().skip(i + 1) {
            let steps = a1.states.len().min(a2.states.len());
            for t in 0..steps {
                let (s1, s2) = (&a1.states[t], &a2.states[t]);
                // The library's own `f64` twin of
                // `encoder_utils::encode_same_lane_constraint`, which is what
                // both the encoder's propositions and
                // `compute_validation_metrics` use since SW-10:
                //     lane1 == lane2  OR  |py1 - py2| < lane_overlap_threshold
                // The discrete half alone misses an actor that is physically
                // straddling the lane line mid-manoeuvre, and misses
                // opposite-direction actors entirely (their lanes never match).
                //
                // Called rather than re-typed (SW-27): this check exists to
                // catch the extractor disagreeing with the encoder, so writing
                // out a third copy of the predicate here would make it a test
                // of whether three hand-copies of one formula still match.
                let same_lane = same_lane_f64(
                    s1.lane(),
                    s2.lane(),
                    s1.position().y,
                    s2.position().y,
                    spec.get_lane_width(),
                );
                if !same_lane {
                    continue;
                }
                let distance = (s1.position().x - s2.position().x).abs();
                min_distance = Some(min_distance.map_or(distance, |m: f64| m.min(distance)));

                // Mirrors the encoder: the actor behind must be closing on the
                // actor ahead by more than `epsilon` for TTC to be defined.
                let epsilon = 0.01;
                let rel_vel = if s1.position().x > s2.position().x {
                    s2.velocity().vx - s1.velocity().vx
                } else if s2.position().x > s1.position().x {
                    s1.velocity().vx - s2.velocity().vx
                } else {
                    continue;
                };
                if rel_vel > epsilon {
                    let ttc = distance / rel_vel;
                    min_ttc = Some(min_ttc.map_or(ttc, |m: f64| m.min(ttc)));
                }
            }
        }
    }

    for (name, recomputed, reported) in [
        ("min_ttc", min_ttc, scenario.validation.min_ttc),
        (
            "min_distance",
            min_distance,
            scenario.validation.min_distance,
        ),
    ] {
        match (recomputed, reported) {
            (Some(r), Some(g)) if (r - g).abs() <= TOL => {}
            (None, None) => {}
            (r, g) => push(
                out,
                Invariant::ExtractionAgreement,
                format!(
                    "validation.{name} = {g:?} but re-deriving it from the extracted \
                     trajectories gives {r:?}"
                ),
            ),
        }
    }

    let max_accel = scenario
        .actors
        .iter()
        .flat_map(|t| t.states.iter())
        .map(|s| s.acceleration().ax)
        .fold(0.0_f64, f64::max);
    let max_decel = scenario
        .actors
        .iter()
        .flat_map(|t| t.states.iter())
        .map(|s| s.acceleration().ax)
        .fold(0.0_f64, f64::min);

    for (name, recomputed, reported) in [
        (
            "max_acceleration",
            max_accel,
            scenario.validation.max_acceleration,
        ),
        (
            "max_deceleration",
            max_decel,
            scenario.validation.max_deceleration,
        ),
    ] {
        if (recomputed - reported).abs() > TOL {
            push(
                out,
                Invariant::ExtractionAgreement,
                format!(
                    "validation.{name} = {reported:.9} but the extracted trajectories give \
                     {recomputed:.9}"
                ),
            );
        }
    }

    // `all_constraints_satisfied` must be the conjunction of the two violation
    // lists actually attached to the scenario, not an independent claim.
    let expected = scenario.validation.safety_violations.is_empty()
        && scenario.validation.acceleration_violations.is_empty();
    if expected != scenario.validation.all_constraints_satisfied {
        push(
            out,
            Invariant::ExtractionAgreement,
            format!(
                "validation.all_constraints_satisfied = {} but safety_violations={:?} \
                 and acceleration_violations={:?}",
                scenario.validation.all_constraints_satisfied,
                scenario.validation.safety_violations,
                scenario.validation.acceleration_violations
            ),
        );
    }

    // The spec drives extraction, so the two actor lists must correspond.
    for traj in &scenario.actors {
        if spec_actor(spec, traj).is_none() {
            push(
                out,
                Invariant::ExtractionAgreement,
                format!("extracted actor {:?} is not in the spec", traj.id),
            );
        }
    }
    for actor in &spec.actors {
        if scenario.get_actor(&actor.id).is_none() {
            push(
                out,
                Invariant::ExtractionAgreement,
                format!("spec actor {:?} has no extracted trajectory", actor.id),
            );
        }
    }
}

// ---------------------------------------------------------------------------
// 7. Pedestrian crossing liveness (SW-42)
// ---------------------------------------------------------------------------

/// A `pedestrian_crossing` pedestrian must actually reach the sidewalk
/// opposite its declared crossing direction somewhere in the shipped
/// trajectory.
///
/// `pedestrian_crossing.rs`'s `generate_ltl` asserts exactly this as a hard
/// LTL goal (`on_opposite_sidewalk.eventually()`), so Z3 cannot return a
/// model that fails to satisfy it — but nothing before this check has ever
/// independently confirmed that the *shipped* trajectory (the JSON/XOSC a
/// user actually gets, built by `src/scenario/extractor.rs` from whatever
/// model Z3 found) is the same trajectory the encoder assumed it was
/// asserting. [`check_extraction_agreement`] catches the encoder and the
/// extractor disagreeing about a *number* (TTC, distance, acceleration);
/// this is the same category of gap for a *goal*. It belongs here rather
/// than in `compute_validation_metrics` for the same reason `Containment`
/// and `ForwardProgress` do: it is an unconditional structural property of
/// the scenario, not a spec threshold gated by `ConstraintMode`, and
/// `compute_validation_metrics`'s shape (a metric, a threshold, a polarity)
/// has nowhere to put a property that is simply "did this happen anywhere in
/// the trajectory".
///
/// [`check_containment`] alone cannot catch a pedestrian that never leaves
/// its starting kerb: it only bounds `py` to
/// `[-SIDEWALK_WIDTH, road_top + SIDEWALK_WIDTH]`, an envelope a stationary
/// pedestrian satisfies perfectly. This check is deliberately narrower and
/// scenario-type-specific — it reads the pedestrian's declared `direction`
/// (via its spec entry, the same field `pedestrian_crossing.rs::generate_ltl`
/// reads) and requires `py` to enter the region [`OnSidewalk`]'s encoding
/// defines for the *opposite* side at at least one time step, mirroring the
/// encoder's own region boundaries (`src/ltl/encode.rs`,
/// `Proposition::OnSidewalk`) exactly: `[-SIDEWALK_WIDTH, 0)` for "left",
/// `(road_width, road_width + SIDEWALK_WIDTH]` for "right".
///
/// Only scoped to `ScenarioType::PedestrianCrossing`: it is the only scenario
/// type whose `generate_ltl` asserts this goal at all, and the only one
/// where `direction`/`OnSidewalk` are meaningful.
fn check_pedestrian_crossing_liveness(
    scenario: &Scenario,
    spec: &ScenarioSpec,
    out: &mut Vec<Violation>,
) {
    use scenario_weaver::dsl::types::ScenarioType;

    if spec.scenario_type != ScenarioType::PedestrianCrossing {
        return;
    }

    let lane_width = spec.get_lane_width();
    let num_lanes = spec.get_num_lanes();
    let road_width = lane_width * num_lanes as f64;

    for traj in &scenario.actors {
        let Some(actor) = spec_actor(spec, traj) else {
            continue;
        };
        if actor.role != ActorRole::Pedestrian {
            continue;
        }
        let Some(direction) = actor.behavior.get("direction").and_then(|v| v.as_str()) else {
            continue;
        };
        let opposite_side = match direction {
            "left_to_right" => "right",
            "right_to_left" => "left",
            _ => continue,
        };

        let reached = traj.states.iter().any(|s| {
            let py = s.position().y;
            if opposite_side == "left" {
                py < -TOL && py >= -SIDEWALK_WIDTH - TOL
            } else {
                py > road_width + TOL && py <= road_width + SIDEWALK_WIDTH + TOL
            }
        });

        if !reached {
            let py_values: Vec<f64> = traj.states.iter().map(|s| s.position().y).collect();
            let py_min = py_values.iter().copied().fold(f64::INFINITY, f64::min);
            let py_max = py_values.iter().copied().fold(f64::NEG_INFINITY, f64::max);
            let region = if opposite_side == "left" {
                format!("[{:.9}, 0)", -SIDEWALK_WIDTH)
            } else {
                format!("({road_width:.9}, {:.9}]", road_width + SIDEWALK_WIDTH)
            };
            push(
                out,
                Invariant::Liveness,
                format!(
                    "{}: direction={direction:?} so it must reach the \"{opposite_side}\" \
                     sidewalk (py in {region}) at some point, but py stayed in \
                     [{py_min:.9}, {py_max:.9}] over the whole trajectory — it never crossed",
                    traj.id
                ),
            );
        }
    }
}
