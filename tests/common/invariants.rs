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

/// Numeric tolerance for every comparison in this module.
///
/// See the module docs: this is the rounding error of an exact rational
/// rendered as `f64`, not an allowance for modelling error.
pub const TOL: f64 = 1e-6;

/// The six properties a generated scenario is committed to.
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
/// the issue that owns the fix.
///
/// This is a **ratchet**, deliberately shaped like `scripts/check.sh`'s clippy
/// baseline. [`assert_scenario_invariants`] is called from every test that
/// generates a scenario; if it asserted all six it would fail on every one of
/// them, so it would have to be `#[ignore]`d everywhere and would assert
/// nothing at all. Instead it asserts everything *except* the entries below,
/// which means the two properties that do hold today —
/// [`Invariant::Envelope`] and [`Invariant::ExtractionAgreement`] — are
/// enforced at every call site right now, and cannot regress unnoticed.
///
/// The list is not an escape hatch:
///
/// - `test_known_broken_invariants_are_still_broken` fails if an entry stops
///   being violated anywhere in the corpus, forcing it to be deleted rather
///   than left to rot.
/// - `test_all_scenario_invariants` (ignored) asserts all six at full strength,
///   and its output is the standing record of what is broken.
///
/// Never add an entry to silence a new failure. An entry means "an open issue
/// owns this and the fix is scheduled".
pub const KNOWN_BROKEN_INVARIANTS: &[(Invariant, &str)] = &[
    (
        Invariant::ConstraintModes,
        "SW-12 (violate) and SW-10 (enforce): `violate` is satisfied by equality — \
         cut_in_left_adversarial_all reports min_ttc = 3.0 against a threshold of exactly \
         3.0 — and `enforce` passes on examples where the metric was never evaluated at \
         all, because the lane variable lags lateral position so no same-lane step exists.",
    ),
    (
        Invariant::Containment,
        "SW-16 (E3): pedestrians only. The SW-10 half — the lane variable pinned on a \
         schedule while py was pinned only at the endpoints of a lane change, observed \
         |py - lane*w - w/2| up to 3.46 m against a half-width of 1.75 — is fixed: `lane` \
         is now derived from `py` at every step and every vehicle example is clean. What \
         is left is the pedestrian encoder: `OnSidewalk` is the unbounded half-plane \
         py > lane_width * num_lanes, so a crossing pedestrian parks up to 6.1 m outside \
         the road surface (pedestrian_crossing: py = 7.85 on a road of [0, 7]) while its \
         `lane` stays at the lane it started in. The SW-08 half — cartesian lane centres \
         5 cm short — is also fixed; the centres are exactly 1.75 / 5.25 now.",
    ),
    (
        Invariant::ForwardProgress,
        "SW-12: on all three pedestrian examples the only vehicle brakes to a standstill \
         and stays there for a majority of the horizon (pedestrian_crossing: 29 of 35 \
         steps), which trivially satisfies every distance and TTC threshold.",
    ),
];

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
fn check_containment(scenario: &Scenario, spec: &ScenarioSpec, out: &mut Vec<Violation>) {
    let w = spec.get_lane_width();
    let num_lanes = spec.get_num_lanes();
    let road_top = num_lanes as f64 * w;

    for traj in &scenario.actors {
        for st in &traj.states {
            let py = st.position().y;
            let lane = st.lane();
            let at = format!("{} at t={:.2}s", traj.id, st.time);

            if py < -TOL || py > road_top + TOL {
                push(
                    out,
                    Invariant::Containment,
                    format!(
                        "{at}: py = {py:.9} outside the road surface [0, {road_top}] \
                         ({num_lanes} lanes x {w} m)"
                    ),
                );
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
                // Mirrors `encoder_utils::encode_same_lane_constraint`, which
                // is what both the encoder's propositions and
                // `compute_validation_metrics` use since SW-10:
                //     lane1 == lane2  OR  |py1 - py2| < lane_width
                // The discrete half alone misses an actor that is physically
                // straddling the lane line mid-manoeuvre, and misses
                // opposite-direction actors entirely (their lanes never match).
                let same_lane = s1.lane() == s2.lane()
                    || (s1.position().y - s2.position().y).abs() < spec.get_lane_width();
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
