//! Core data structures for the scenario specification DSL.
//!
//! These types are deserialized from YAML input and drive the entire generation pipeline.

use serde::de::{self, MapAccess, SeqAccess, Visitor};
use serde::{Deserialize, Deserializer, Serialize};
use std::fmt;

// Pedestrian physics constants
//
// SW-12/M8. These are the true physical speeds, not a compensation for the
// encoding. They used to be divided by sqrt(2) (2.0 -> 1.41, 5.0 -> 3.54)
// because the encoder bounded velocity with a *box* — `|vx| <= v` and
// `|vy| <= v` — which contains the disk and so permits `sqrt(2)*v` on the
// diagonal. Shrinking the constant fixed the diagonal by breaking every other
// direction: a pedestrian crossing perpendicular to the road, which is the
// dominant case in this corpus, was capped at 1.41 m/s instead of 2.0. The
// compensation also never reached `PEDESTRIAN_WALK_MIN_SPEED`, so the
// min/max ratio moved as well.
//
// The bound is now an *octagon* (`encoders::pedestrian::encode_pedestrian_bounds_step`):
// `|vx| <= v`, `|vy| <= v` and `|vx| + |vy| <= sqrt(2)*v`. Still linear — eight
// half-planes, no case split, so the encoding stays in QF_LRA and the
// optimiser keeps working. It is exact on both axes and exact on the
// 45-degree diagonal (`|vx| = |vy| = v/sqrt(2)`, speed `v`), and its worst
// over-approximation of the disk is at the corner `(v, (sqrt(2)-1)v)`, whose
// speed is `1.0824*v` — 8.2 %, against the box's 41.4 %.

/// Maximum walking speed for pedestrians (m/s) - normal walk
pub const PEDESTRIAN_WALK_MAX_SPEED: f64 = 2.0;

/// Minimum walking speed for pedestrians (m/s)
pub const PEDESTRIAN_WALK_MIN_SPEED: f64 = 0.5;

/// Maximum running speed for pedestrians (m/s)
pub const PEDESTRIAN_RUN_MAX_SPEED: f64 = 5.0;

/// Minimum running speed for pedestrians (m/s)
pub const PEDESTRIAN_RUN_MIN_SPEED: f64 = 2.0;

/// Maximum acceleration for pedestrians (m/s²)
pub const PEDESTRIAN_MAX_ACCELERATION: f64 = 1.0;

/// Maximum deceleration for pedestrians (m/s²) - negative value
pub const PEDESTRIAN_MAX_DECELERATION: f64 = -1.0;

// Scenario envelope bounds (SW-12/L2)
//
// `ScenarioSpec::validate` bounded `time_step` and `duration` from one side
// only, so `time_step: 1e-9` with `duration: 10.0` was accepted and then hung
// the process building 10^10 Z3 variables. These name the accepted range so
// the error message can quote it.

/// Smallest accepted `time_step` (s).
pub const MIN_TIME_STEP: f64 = 0.001;

/// Largest accepted `duration` (s).
pub const MAX_DURATION: f64 = 3600.0;

/// Largest accepted `duration / time_step`.
pub const MAX_TIME_STEPS: usize = 100_000;

/// Smallest accepted lane width (m).
pub const MIN_LANE_WIDTH: f64 = 1.0;

/// Largest accepted lane width (m).
pub const MAX_LANE_WIDTH: f64 = 20.0;

/// Whether a lane change scheduled to start at `start_step` has anywhere to
/// go within a `horizon`-step scenario (SW-25/SW-29).
///
/// `horizon` steps means the last valid index is `horizon`, but that index is
/// the *final* state — nothing simulates past it — so a change starting there
/// spans no time step at all and is discarded by the encoder
/// (`CartesianEncoder::encode_smooth_lane_transition`'s own
/// `start_step >= self.horizon` guard). The comparison is therefore `>=`, not
/// `>`; SW-25 found the two disagreeing by exactly one between
/// [`ScenarioSpec::validate`] and the encoder, with `collect_lane_change_data`
/// in `solver::encoder_utils` a third, independently-drifted copy (still `>`
/// until SW-29). `validate` and `collect_lane_change_data` now both call this
/// function. `src/solver/encoders/cartesian.rs` is outside SW-29's file list
/// (`src/solver/encoders/` is fenced off entirely), so its own
/// `start_step >= self.horizon` is left as a separate literal — numerically
/// identical to this predicate today, but a fourth call site should route
/// through it too if that fence is ever lifted.
#[must_use]
pub const fn lane_change_start_past_horizon(start_step: usize, horizon: usize) -> bool {
    start_step >= horizon
}

/// Fraction of the distance implied by an actor's declared initial speed that
/// a vehicle must actually cover over the scenario duration (SW-12/M5).
///
/// This is the forward-progress floor. It is a bound on *net displacement*
/// over the whole horizon, not a per-step speed floor, so a vehicle may still
/// brake hard — including to a full stop, which is exactly what a scenario
/// with a pedestrian in the road is for — but it cannot answer the whole
/// specification by parking. A per-step floor would forbid the emergency stop
/// outright; a "stopped for at most half the steps" cardinality constraint
/// would say precisely the right thing but needs one Boolean indicator per
/// step and turns propagation into search. One inequality per actor does not.
pub const MIN_FORWARD_PROGRESS_FRACTION: f64 = 0.5;

/// Fraction of an actor's declared initial speed that its longitudinal speed
/// must be back up to *at the final step of the horizon* (SW-39).
///
/// The displacement floor above bounds only the average speed over the whole
/// horizon, and the cheapest way to satisfy an average is a monotone decay to
/// zero — `MIN_FORWARD_PROGRESS_FRACTION` alone lets an oncoming vehicle coast
/// to a dead stop by the end of the scenario and stay there, which every
/// distance/TTC check downstream is happy to call a near miss. A per-step
/// floor is still the wrong tool (see the constant above): it would forbid a
/// vehicle braking hard for a pedestrian in the road, which must stay legal.
///
/// A floor on the terminal speed only — not on any interior step — forbids
/// exactly the "ends stopped, or near it" case while leaving every interior
/// step free to dip to zero and recover: an emergency stop is a dip, not an
/// ending state. It is also still one linear inequality per actor
/// (`v[H] * dir` compared against `f * speed.min() * dir`, all compile-time
/// constants), so it stays in QF_LRA exactly like the displacement floor.
pub const TERMINAL_SPEED_FRACTION: f64 = 0.5;

/// The only `ActorSpec.behavior` keys anything in the crate reads (SW-21/M10).
///
/// `behavior` is an untyped `HashMap<String, serde_json::Value>` — there is no
/// per-scenario-type schema to validate against — so this whitelist is the
/// "at minimum" option: reject any key that is not one of these two, so a
/// typo (`walking_moad` for `walking_mode`) is a parse-time error rather than
/// a silently-defaulted behavior. Grep the crate before adding to this list.
pub const ALLOWED_BEHAVIOR_KEYS: &[&str] = &["direction", "walking_mode"];

/// Constraint enforcement mode
#[derive(Debug, Clone, Copy, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
#[derive(Default)]
pub enum ConstraintMode {
    /// Enforce constraint: G(constraint) - must hold at all times
    #[default]
    Enforce,
    /// Violate constraint: F(NOT constraint) - must be violated at some point
    Violate,
    /// Ignore constraint: not added to the formula
    Ignore,
}

/// Optimization target for finding worst-case or best-case scenarios.
///
/// Every variant is scored over the same "same lane" predicate and closing-speed floor
/// that `compute_validation_metrics` uses when it reports `min_distance` and `min_ttc`,
/// so the quantity optimised is the quantity reported. `docs/optimizer.md` documents the
/// encoding; the objective region of `src/solver/encoder.rs` implements it.
///
/// YAML spells these in `snake_case` (`minimize_ttc`); the CLI's `--optimize` spells the
/// same set in `kebab-case` (`min-ttc`). The two are not interchangeable.
#[derive(Debug, Clone, Copy, Deserialize, Serialize, PartialEq, Eq, Default)]
#[serde(rename_all = "snake_case")]
pub enum OptimizationTarget {
    /// No optimization - find any satisfying solution (default, backward compatible)
    #[default]
    None,
    /// Minimize the smallest same-lane time-to-collision — the worst near-miss the
    /// constraints allow.
    ///
    /// The reported optimum is a certified **upper** bound on the min-TTC of the
    /// trajectory shipped with it.
    MinimizeTtc,
    /// Minimize the smallest same-lane longitudinal gap — the closest approach.
    MinimizeDistance,
    /// **Maximize** the highest same-lane closing speed — the most severe interaction.
    ///
    /// Severity correlates with relative impact speed, and the encoder drives that *up*.
    /// There is no weighted combination of TTC and distance anywhere, and never was —
    /// which is why this was called `MinimizeSeverity` until SW-14's rename: the old name
    /// described an encoding that did not exist. Use `MaximizeTtc` if what you want is the
    /// *safest* scenario; this one asks for the worst.
    MaximizeSeverity,
    /// Maximize the smallest same-lane time-to-collision — the safest scenario.
    ///
    /// This maximises **TTC**, not gap: the largest TTC is reached by matching speeds
    /// rather than by separating, so the returned scenario may have a small gap. The
    /// reported optimum is a certified **lower** bound on the min-TTC of the trajectory,
    /// and saturates at the top of `TTC_LEVELS` (60 s).
    MaximizeTtc,
}

/// Coordinate system for scenario generation
#[derive(Debug, Clone, Copy, Deserialize, Serialize, PartialEq, Eq, Default)]
#[serde(rename_all = "snake_case")]
pub enum CoordinateSystem {
    /// Cartesian coordinates (x, y) with discrete lane assignments (default, for backward compatibility)
    #[default]
    Cartesian,
    /// Bicycle model (x, y, θ, v) with heading tracking and steering constraints
    Bicycle,
}

/// Direction of a lane change maneuver relative to the actor's current lane.
#[derive(Debug, Clone, Copy, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum LaneChangeDirection {
    /// Move to the lane with a lower index (lower y-coordinate), for a
    /// forward-travelling actor (`direction: 1`, the default). Lane position
    /// is `y = lane * lane_width + lane_width / 2`
    /// (`cartesian.rs::encode_lane_position_coupling_at_time`), so lane index
    /// and y-coordinate move together — index 0 is always the lower-y lane.
    /// For a backward-travelling actor this flips, since the actor's own
    /// "left" is the opposite road-frame direction (see
    /// `cartesian.rs::encode_smooth_lane_transition`, which maps
    /// `Left => -actor.direction`).
    ///
    /// SW-17 E5: this used to claim the opposite (higher index / lower y);
    /// it was corrected to match `cartesian.rs`, the encoder that actually
    /// implements it, rather than the other way around.
    Left,
    /// Move to the lane with a higher index (higher y-coordinate), for a
    /// forward-travelling actor. See [`Left`](LaneChangeDirection::Left) for
    /// the y-coordinate formula and the backward-actor caveat
    /// (`cartesian.rs::encode_smooth_lane_transition` maps
    /// `Right => actor.direction`).
    Right,
}

/// Lane change configuration
///
/// The solver discovers lane change trajectories dynamically using smoothness
/// constraints, rather than pre-computing them with polynomials.
///
/// Note: Multiple lane changes can be specified as a `Vec<LaneChangeConfig>`.
/// Presence in the vec implies enabled (no explicit enabled field needed).
#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct LaneChangeConfig {
    pub direction: LaneChangeDirection,
    /// Start time (can be a fixed value or range for solver to choose)
    pub start_time: ValueOrRange,
    /// Duration (can be a fixed value or range for solver to choose)
    pub duration: ValueOrRange,
}

/// Kinematic bicycle model parameters for a single actor.
///
/// Models vehicle dynamics with front-wheel steering. The minimum turn radius
/// is the exact kinematic one, `wheelbase / tan(max_steering_angle)` — see
/// [`BicycleParams::min_turn_radius`].
#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct BicycleParams {
    /// Wheelbase in meters (distance between front and rear axles)
    pub wheelbase: f64,
    /// Maximum steering angle in radians (at front wheels)
    pub max_steering_angle: f64,
    /// Maximum steering rate in radians per second
    pub max_steering_rate: f64,
}

impl BicycleParams {
    /// Validate bicycle parameters
    pub fn validate(&self) -> Result<(), String> {
        if self.wheelbase <= 0.0 {
            return Err("wheelbase must be positive".to_string());
        }
        if self.max_steering_angle <= 0.0 {
            return Err("max_steering_angle must be positive".to_string());
        }
        // `min_turn_radius` divides by `tan(max_steering_angle)`, which is
        // singular at pi/2 and negative beyond it. A steering angle at or past
        // 90 deg is not a vehicle in any case.
        if self.max_steering_angle >= std::f64::consts::FRAC_PI_2 {
            return Err(format!(
                "max_steering_angle must be below pi/2 ({:.4} rad), got {}",
                std::f64::consts::FRAC_PI_2,
                self.max_steering_angle
            ));
        }
        if self.max_steering_rate <= 0.0 {
            return Err("max_steering_rate must be positive".to_string());
        }
        Ok(())
    }

    /// Minimum turn radius of the kinematic bicycle model, `R = L / tan(δ_max)`.
    ///
    /// This is the exact geometric relation, not the small-angle
    /// `L / δ_max` this used to return. The two differ by
    /// `tan(δ_max) / δ_max`, which at the 0.6 rad default steering lock of
    /// `examples/bicycle_lane_change.yaml` is 14 %: 2.7 / tan(0.6) = 3.95 m,
    /// against the 4.5 m the old formula (and `docs/coordinate-systems.md`)
    /// advertised.
    ///
    /// `BicycleParams::validate` keeps `max_steering_angle` in (0, pi/2), so
    /// the tangent is finite and positive here.
    #[must_use]
    pub fn min_turn_radius(&self) -> f64 {
        self.wheelbase / self.max_steering_angle.tan()
    }
}

/// Scenario-level defaults for bicycle model parameters.
///
/// Applied to any actor that does not specify its own [`BicycleParams`].
#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct BicycleConfig {
    /// Default wheelbase for actors without specific bicycle_params
    pub default_wheelbase: f64,
    /// Default max steering angle for actors without specific bicycle_params
    pub default_max_steering_angle: f64,
    /// Default max steering rate for actors without specific bicycle_params
    pub default_max_steering_rate: f64,
}

impl BicycleConfig {
    /// Validate bicycle configuration
    pub fn validate(&self) -> Result<(), String> {
        if self.default_wheelbase <= 0.0 {
            return Err("default_wheelbase must be positive".to_string());
        }
        if self.default_max_steering_angle <= 0.0 {
            return Err("default_max_steering_angle must be positive".to_string());
        }
        if self.default_max_steering_angle >= std::f64::consts::FRAC_PI_2 {
            return Err(format!(
                "default_max_steering_angle must be below pi/2 ({:.4} rad), got {}",
                std::f64::consts::FRAC_PI_2,
                self.default_max_steering_angle
            ));
        }
        if self.default_max_steering_rate <= 0.0 {
            return Err("default_max_steering_rate must be positive".to_string());
        }
        Ok(())
    }

    /// Convert to BicycleParams
    pub fn to_params(&self) -> BicycleParams {
        BicycleParams {
            wheelbase: self.default_wheelbase,
            max_steering_angle: self.default_max_steering_angle,
            max_steering_rate: self.default_max_steering_rate,
        }
    }
}

/// Which of the seven per-constraint modes is being asked for.
///
/// SW-21/D1+item 3: `ConstraintModes` used to expose seven public accessors
/// (`min_ttc()`, `min_distance()`, ...) that were the same 4-line match
/// copy-pasted seven times, once per field. This enum plus
/// [`ConstraintModes::mode_for`] is the single real implementation; the seven
/// named methods below are kept as one-line wrappers only because they are
/// public API called from `src/scenarios/{head_on,pedestrian_crossing}.rs`
/// and elsewhere outside this issue's file list — removing them would be a
/// breaking rename this issue is not scoped to make.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Constraint {
    MinTtc,
    MinDistance,
    MaxAcceleration,
    MaxVelocity,
    MinVelocity,
    MinLateralDistance,
    MaxRelativeVelocity,
}

/// Per-constraint enforcement configuration.
///
/// Controls how each safety constraint is treated during generation:
/// enforced (must hold), violated (adversarial), or ignored.
///
/// SW-21/D1+D2: `Deserialize` is hand-written rather than derived (see
/// `impl<'de> Deserialize<'de> for ConstraintModes` below) for two reasons
/// that `#[serde(untagged)]` cannot give us:
///   1. The `Detailed { .. }` mapping form needs `#[serde(deny_unknown_fields)]`
///      so `{min_tcc: violate}` (typo of `min_ttc`) is a parse error naming
///      the bad key, not a silently-ignored field that leaves the constraint
///      at its default. Serde does not support `#[serde(deny_unknown_fields)]`
///      on one variant of an enum (confirmed: `unknown serde variant
///      attribute`), so the mapping form is deserialized through a private
///      `DetailedConstraintModesRaw` struct that does carry the attribute.
///   2. The shorthand string form needs a custom error naming the valid
///      values when it does not match one of them, e.g. `violate-all` (a
///      hyphen typo of `violate_all`) used to deserialize successfully as an
///      arbitrary `String` and then silently fall through the accessors'
///      `_ => ConstraintMode::Enforce` arm — the exact inverse of the
///      adversarial intent the user wrote, with no error and no warning.
#[derive(Debug, Clone, Serialize)]
#[serde(untagged)]
pub enum ConstraintModes {
    /// Individual mode per constraint (e.g., enforce TTC but violate distance).
    Detailed {
        min_ttc: ConstraintMode,
        min_distance: ConstraintMode,
        max_acceleration: ConstraintMode,
        max_velocity: ConstraintMode,
        min_velocity: ConstraintMode,
        min_lateral_distance: ConstraintMode,
        max_relative_velocity: ConstraintMode,
    },
    /// Bulk mode string: `"violate_all"`, `"ignore_all"`, or `"enforce_all"`.
    Shorthand(ConstraintShorthand),
}

/// The three bulk values `constraint_modes` accepts as a single string.
///
/// SW-19: previously the payload of `ConstraintModes::Shorthand` was a bare
/// `String`. SW-21's custom `Deserialize` already turned a typo'd shorthand
/// into a parse error naming the valid values, but the type still allowed
/// constructing an invalid one directly (`ConstraintModes::Shorthand("bogus".into())`
/// compiled fine); `mode_for`'s `Shorthand` arm below could only guard
/// against that with a runtime `unreachable!()`. A three-variant enum makes
/// the invalid state unrepresentable instead.
#[derive(Debug, Clone, Copy, Serialize, PartialEq, Eq)]
pub enum ConstraintShorthand {
    #[serde(rename = "violate_all")]
    ViolateAll,
    #[serde(rename = "ignore_all")]
    IgnoreAll,
    #[serde(rename = "enforce_all")]
    EnforceAll,
}

/// Deserialization target for the `Detailed` mapping form only. Exists
/// purely so `#[serde(deny_unknown_fields)]` can be attached to it (serde
/// rejects that attribute on an individual enum variant), then its fields are
/// moved into the real `ConstraintModes::Detailed { .. }` — see the impl
/// below.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct DetailedConstraintModesRaw {
    #[serde(default)]
    min_ttc: ConstraintMode,
    #[serde(default)]
    min_distance: ConstraintMode,
    #[serde(default)]
    max_acceleration: ConstraintMode,
    #[serde(default)]
    max_velocity: ConstraintMode,
    #[serde(default)]
    min_velocity: ConstraintMode,
    #[serde(default)]
    min_lateral_distance: ConstraintMode,
    #[serde(default)]
    max_relative_velocity: ConstraintMode,
}

impl<'de> Deserialize<'de> for ConstraintModes {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        struct ConstraintModesVisitor;

        impl<'de> Visitor<'de> for ConstraintModesVisitor {
            type Value = ConstraintModes;

            fn expecting(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
                write!(
                    f,
                    "a mapping of individual constraint modes (min_ttc, min_distance, \
                     max_acceleration, max_velocity, min_velocity, min_lateral_distance, \
                     max_relative_velocity), or one of the strings \"violate_all\", \
                     \"ignore_all\", \"enforce_all\""
                )
            }

            fn visit_str<E>(self, v: &str) -> Result<Self::Value, E>
            where
                E: de::Error,
            {
                match v {
                    "violate_all" => {
                        Ok(ConstraintModes::Shorthand(ConstraintShorthand::ViolateAll))
                    }
                    "ignore_all" => Ok(ConstraintModes::Shorthand(ConstraintShorthand::IgnoreAll)),
                    "enforce_all" => {
                        Ok(ConstraintModes::Shorthand(ConstraintShorthand::EnforceAll))
                    }
                    other => Err(E::custom(format!(
                        "unknown constraint_modes value '{other}'. Valid values: \
                         violate_all, ignore_all, enforce_all (or a mapping of \
                         per-constraint modes)"
                    ))),
                }
            }

            fn visit_map<A>(self, map: A) -> Result<Self::Value, A::Error>
            where
                A: MapAccess<'de>,
            {
                let raw = DetailedConstraintModesRaw::deserialize(
                    de::value::MapAccessDeserializer::new(map),
                )?;
                Ok(ConstraintModes::Detailed {
                    min_ttc: raw.min_ttc,
                    min_distance: raw.min_distance,
                    max_acceleration: raw.max_acceleration,
                    max_velocity: raw.max_velocity,
                    min_velocity: raw.min_velocity,
                    min_lateral_distance: raw.min_lateral_distance,
                    max_relative_velocity: raw.max_relative_velocity,
                })
            }
        }

        deserializer.deserialize_any(ConstraintModesVisitor)
    }
}

impl Default for ConstraintModes {
    fn default() -> Self {
        // SW-21/M7: this used to disagree with the per-field `#[serde(default)]`
        // (which resolves to `ConstraintMode::default()` = `Enforce` for every
        // field), so omitting `constraint_modes` entirely and writing a
        // *partial* `Detailed` block gave different semantics for
        // `min_velocity`, `min_lateral_distance` and `max_relative_velocity`
        // — the first left them `Ignore`, the second silently flipped them to
        // `Enforce`. Reconciled here in favour of `Enforce`: these three are
        // still `Option<f64>` scalars on `ScenarioSpec` that default to
        // `None`, so an omitted `constraint_modes` block was already inert
        // for them in every shipped example and test fixture (verified: grep
        // finds no example that sets `min_velocity`/`min_lateral_distance`/
        // `max_relative_velocity` without also spelling out `constraint_modes`
        // explicitly). `Enforce` is also the safer default in general: a user
        // who sets one of these numeric bounds and does not touch
        // `constraint_modes` at all almost certainly means for it to hold,
        // not to be silently inert — the same "wrong silently" failure mode
        // this issue exists to remove from the shorthand string.
        ConstraintModes::Detailed {
            min_ttc: ConstraintMode::Enforce,
            min_distance: ConstraintMode::Enforce,
            max_acceleration: ConstraintMode::Enforce,
            max_velocity: ConstraintMode::Enforce,
            min_velocity: ConstraintMode::Enforce,
            min_lateral_distance: ConstraintMode::Enforce,
            max_relative_velocity: ConstraintMode::Enforce,
        }
    }
}

impl ConstraintModes {
    /// The one real implementation behind the seven named accessors below.
    pub fn mode_for(&self, constraint: Constraint) -> ConstraintMode {
        match self {
            ConstraintModes::Detailed {
                min_ttc,
                min_distance,
                max_acceleration,
                max_velocity,
                min_velocity,
                min_lateral_distance,
                max_relative_velocity,
            } => match constraint {
                Constraint::MinTtc => *min_ttc,
                Constraint::MinDistance => *min_distance,
                Constraint::MaxAcceleration => *max_acceleration,
                Constraint::MaxVelocity => *max_velocity,
                Constraint::MinVelocity => *min_velocity,
                Constraint::MinLateralDistance => *min_lateral_distance,
                Constraint::MaxRelativeVelocity => *max_relative_velocity,
            },
            // SW-21/D1: this used to be duplicated seven times, once per
            // accessor, with a `_ => ConstraintMode::Enforce` fallback that
            // made a typo'd shorthand silently enforce rather than error.
            // SW-19 finished the job: `ConstraintShorthand` has exactly these
            // three variants, so this match is exhaustive with no `_` arm and
            // no runtime fallback — a future shorthand value nobody taught
            // this function about is a compile error, not a silent `Enforce`.
            ConstraintModes::Shorthand(shorthand) => match shorthand {
                ConstraintShorthand::ViolateAll => ConstraintMode::Violate,
                ConstraintShorthand::IgnoreAll => ConstraintMode::Ignore,
                ConstraintShorthand::EnforceAll => ConstraintMode::Enforce,
            },
        }
    }

    /// Get the mode for min_ttc constraint
    pub fn min_ttc(&self) -> ConstraintMode {
        self.mode_for(Constraint::MinTtc)
    }

    /// Get the mode for min_distance constraint
    pub fn min_distance(&self) -> ConstraintMode {
        self.mode_for(Constraint::MinDistance)
    }

    /// Get the mode for max_acceleration constraint
    pub fn max_acceleration(&self) -> ConstraintMode {
        self.mode_for(Constraint::MaxAcceleration)
    }

    /// Get the mode for max_velocity constraint
    pub fn max_velocity(&self) -> ConstraintMode {
        self.mode_for(Constraint::MaxVelocity)
    }

    /// Get the mode for min_velocity constraint
    pub fn min_velocity(&self) -> ConstraintMode {
        self.mode_for(Constraint::MinVelocity)
    }

    /// Get the mode for min_lateral_distance constraint
    pub fn min_lateral_distance(&self) -> ConstraintMode {
        self.mode_for(Constraint::MinLateralDistance)
    }

    /// Get the mode for max_relative_velocity constraint
    pub fn max_relative_velocity(&self) -> ConstraintMode {
        self.mode_for(Constraint::MaxRelativeVelocity)
    }
}

/// Role of an actor in the scenario.
#[derive(Debug, Clone, Copy, Deserialize, Serialize, PartialEq, Eq)]
pub enum ActorRole {
    /// The autonomous vehicle under test.
    #[serde(rename = "ego")]
    Ego,
    /// A non-player-character vehicle (other traffic).
    #[serde(rename = "npc")]
    Npc,
    /// A pedestrian with simplified physics (no steering model).
    #[serde(rename = "pedestrian")]
    Pedestrian,
}

impl std::fmt::Display for ActorRole {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ActorRole::Ego => write!(f, "ego"),
            ActorRole::Npc => write!(f, "npc"),
            ActorRole::Pedestrian => write!(f, "pedestrian"),
        }
    }
}

/// The type of driving scenario to generate.
///
/// Each variant maps to a [`ScenarioModel`](crate::scenarios::ScenarioModel) implementation
/// that defines behavioral LTL and validation rules.
#[derive(Debug, Clone, Copy, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum ScenarioType {
    /// NPC cuts in from the left lane ahead of ego.
    CutInLeft,
    /// NPC cuts in from the right lane ahead of ego.
    CutInRight,
    /// NPC overtakes ego via the left lane (two sequential lane changes).
    OvertakeLeft,
    /// Pedestrian crosses the road while ego approaches.
    PedestrianCrossing,
    /// Ego overtakes a slow vehicle on a bidirectional road, entering the oncoming lane.
    HeadOn,
}

impl std::fmt::Display for ScenarioType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ScenarioType::CutInLeft => write!(f, "cut_in_left"),
            ScenarioType::CutInRight => write!(f, "cut_in_right"),
            ScenarioType::OvertakeLeft => write!(f, "overtake_left"),
            ScenarioType::PedestrianCrossing => write!(f, "pedestrian_crossing"),
            ScenarioType::HeadOn => write!(f, "head_on"),
        }
    }
}

impl ScenarioType {
    /// Get the scenario model for this scenario type
    pub fn get_model(&self) -> Box<dyn crate::scenarios::ScenarioModel> {
        match self {
            ScenarioType::CutInLeft => Box::new(crate::scenarios::cut_in_left::CutInLeftModel),
            ScenarioType::CutInRight => Box::new(crate::scenarios::cut_in_right::CutInRightModel),
            ScenarioType::OvertakeLeft => {
                Box::new(crate::scenarios::overtake_left::OvertakeLeftModel)
            }
            ScenarioType::PedestrianCrossing => {
                Box::new(crate::scenarios::pedestrian_crossing::PedestrianCrossingModel)
            }
            ScenarioType::HeadOn => Box::new(crate::scenarios::head_on::HeadOnModel),
        }
    }
}

/// Single straight road with configurable lanes, widths, and per-lane directions.
///
/// Bidirectional traffic is modeled by assigning `+1` (forward) or `-1` (backward)
/// to each lane in `lane_directions`.
#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct RoadSpec {
    /// Number of lanes (total, both directions)
    pub num_lanes: usize,

    /// Width of each lane in meters
    pub lane_width: f64,

    /// Direction of each lane: +1 for forward (+x), -1 for backward (-x)
    /// Length must equal num_lanes
    /// Example: [1, 1, -1, -1] for 4 lanes (2 forward, 2 backward)
    #[serde(default = "default_lane_directions")]
    pub lane_directions: Vec<i32>,

    /// Length of the road in meters (optional, will be calculated if not provided)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub road_length: Option<f64>,
}

impl RoadSpec {
    /// Get the direction of a specific lane
    pub fn get_lane_direction(&self, lane: usize) -> i32 {
        if lane < self.lane_directions.len() {
            self.lane_directions[lane]
        } else {
            // Default: all lanes go forward (backward compatible)
            1
        }
    }

    /// Validate road specification
    pub fn validate(&self) -> Result<(), String> {
        if self.lane_directions.len() != self.num_lanes {
            return Err(format!(
                "lane_directions length ({}) must equal num_lanes ({}). \
                 If lane_directions was omitted it defaults to 2 forward lanes; \
                 set it explicitly for roads with != 2 lanes",
                self.lane_directions.len(),
                self.num_lanes
            ));
        }

        for (i, &dir) in self.lane_directions.iter().enumerate() {
            if dir != 1 && dir != -1 {
                return Err(format!("lane_directions[{}] = {} must be +1 or -1", i, dir));
            }
        }

        // The xodr exporter (`xodr_exporter::build_lane_section`) maps a scenario lane
        // index `i` to a physical y-position `[i*lane_width, (i+1)*lane_width]` — that
        // mapping is direction-independent (see `cartesian.rs`'s `py = lane*lane_width +
        // lane_width/2`). To place forward lanes on OpenDRIVE's right side (negative ids)
        // and backward lanes on the left (positive ids) while keeping ids consistent with
        // physical y-position, every forward lane must sit at a lower index than every
        // backward lane: `[1,1,-1,-1]` is fine, `[-1,-1,1,1]` and any interleaving such as
        // `[1,-1,1,-1]` are not — the exporter would silently place a lane a full
        // lane-width off from its scenario y-position (SW-16 E2). Reject rather than emit
        // a road that disagrees with the trajectories driving on it.
        if self
            .lane_directions
            .windows(2)
            .any(|pair| pair[0] == -1 && pair[1] == 1)
        {
            return Err(format!(
                "lane_directions {:?} is not a single forward-then-backward block \
                 (e.g. [1,1,-1,-1]) — once a backward (-1) lane appears, no forward \
                 (+1) lane may follow it. The xodr exporter's right/left lane-id split \
                 assumes this ordering; an interleaved or reversed layout would place \
                 lanes at the wrong y-position. Reorder the road's lanes, or track the \
                 general fix under SW-16 (drop the direction-based id split entirely and \
                 assign ids purely from lane index).",
                self.lane_directions
            ));
        }

        if self.num_lanes == 0 {
            return Err("num_lanes must be at least 1".to_string());
        }

        if self.lane_width <= 0.0 {
            return Err("lane_width must be positive".to_string());
        }

        if let Some(length) = self.road_length {
            if length <= 0.0 {
                return Err("road_length must be positive".to_string());
            }
        }

        Ok(())
    }
}

/// Default lane directions: all forward (backward compatible)
fn default_lane_directions() -> Vec<i32> {
    vec![1; 2] // Default to 2 forward lanes
}

/// Specification of a single actor (ego vehicle, NPC vehicle, or pedestrian).
///
/// Position and speed can be fixed values or ranges; ranges let the Z3 solver
/// choose concrete values that satisfy all constraints.
#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct ActorSpec {
    pub id: String,
    pub role: ActorRole,
    pub lane: usize,
    pub position: ValueOrRange,
    pub speed: ValueOrRange,
    pub acceleration: ValueOrRange,
    /// Direction of travel: +1 for forward (+x), -1 for backward (-x)
    pub direction: i32,
    /// Scenario-specific behavior parameters
    #[serde(default)]
    pub behavior: std::collections::HashMap<String, serde_json::Value>,
    /// Lane change configurations (multiple sequential lane changes supported)
    /// Empty vec means no lane changes. Presence in vec implies enabled.
    #[serde(default)]
    pub lane_changes: Vec<LaneChangeConfig>,
    /// Bicycle model parameters (optional, overrides scenario-level bicycle_config)
    #[serde(default)]
    pub bicycle_params: Option<BicycleParams>,
}

/// Root configuration parsed from a YAML scenario file.
///
/// Contains all information needed to generate one or more concrete scenarios:
/// actors, road geometry, timing, safety thresholds, constraint modes, and
/// coordinate system selection.
#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct ScenarioSpec {
    pub scenario_type: ScenarioType,
    pub time_step: f64, // seconds per discretization step
    pub duration: f64,  // total scenario duration (seconds)
    pub actors: Vec<ActorSpec>,
    pub min_ttc: f64,      // minimum time-to-collision (seconds)
    pub min_distance: f64, // minimum longitudinal distance (meters)
    /// Road specification (optional, for bidirectional traffic)
    #[serde(default)]
    pub road: Option<RoadSpec>,
    /// Lane width (deprecated, use road.lane_width instead)
    #[serde(default = "default_lane_width")]
    pub lane_width: f64, // meters
    pub num_scenarios: usize, // 1 for single, N for multiple
    /// Constraint enforcement modes (optional, defaults to enforce_all)
    #[serde(default)]
    pub constraint_modes: ConstraintModes,
    /// Optional global maximum acceleration constraint (m/s²)
    #[serde(default)]
    pub max_acceleration: Option<f64>,
    /// Optional global maximum deceleration constraint (m/s², should be negative)
    #[serde(default)]
    pub max_deceleration: Option<f64>,
    /// Optimization target (optional, defaults to None for backward compatibility)
    /// When set, uses Z3 Optimize instead of Solver to find optimal scenarios
    #[serde(default)]
    pub optimization_target: OptimizationTarget,
    /// Optional maximum velocity constraint (m/s)
    #[serde(default)]
    pub max_velocity: Option<f64>,
    /// Optional minimum velocity constraint (m/s)
    #[serde(default)]
    pub min_velocity: Option<f64>,
    /// Optional minimum lateral distance constraint (m)
    #[serde(default)]
    pub min_lateral_distance: Option<f64>,
    /// Optional maximum relative velocity constraint (m/s)
    #[serde(default)]
    pub max_relative_velocity: Option<f64>,
    /// Maximum lateral acceleration during lane changes (m/s²)
    /// Default: 2.0 m/s² for comfortable driving
    #[serde(default = "default_max_lateral_acceleration")]
    pub max_lateral_acceleration: f64,
    /// Coordinate system (default: Cartesian)
    #[serde(default)]
    pub coordinate_system: CoordinateSystem,
    /// Bicycle model configuration (optional, provides default parameters for all actors)
    #[serde(default)]
    pub bicycle_config: Option<BicycleConfig>,
}

/// Default lane width for backward compatibility
fn default_lane_width() -> f64 {
    3.5
}

/// Default maximum lateral acceleration for lane changes
fn default_max_lateral_acceleration() -> f64 {
    2.0 // 2.0 m/s² - comfortable driving
}

/// A numeric value that is either fixed or a `[min, max]` range for the solver to explore.
///
/// SW-21/M11. `Deserialize` is hand-written rather than derived from
/// `#[serde(untagged)]`. The derived untagged form buffers the value into a
/// generic `Content` tree and retries each variant against it, which loses
/// the original deserializer's position tracking — `speed: "fast"` used to
/// report `data did not match any variant of untagged enum ValueOrRange` at
/// the line/column of the *start of the actor mapping*, not the `speed:` key,
/// and named an internal type the user has never heard of. Visiting the value
/// directly (no buffering) keeps the deserializer's real position, and a
/// custom `expecting` message states the accepted shapes in DSL terms instead
/// of the type name.
#[derive(Debug, Clone, Serialize)]
#[serde(untagged)]
pub enum ValueOrRange {
    /// Exact value (encoded as an equality constraint in Z3).
    Value(f64),
    /// Inclusive range `[min, max]` (encoded as inequality constraints in Z3).
    Range([f64; 2]), // [min, max]
}

impl<'de> Deserialize<'de> for ValueOrRange {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        struct ValueOrRangeVisitor;

        impl<'de> Visitor<'de> for ValueOrRangeVisitor {
            type Value = ValueOrRange;

            fn expecting(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
                write!(f, "a number, or a two-element [min, max] array")
            }

            fn visit_f64<E>(self, v: f64) -> Result<Self::Value, E>
            where
                E: de::Error,
            {
                Ok(ValueOrRange::Value(v))
            }

            fn visit_i64<E>(self, v: i64) -> Result<Self::Value, E>
            where
                E: de::Error,
            {
                #[allow(clippy::cast_precision_loss)]
                Ok(ValueOrRange::Value(v as f64))
            }

            fn visit_u64<E>(self, v: u64) -> Result<Self::Value, E>
            where
                E: de::Error,
            {
                #[allow(clippy::cast_precision_loss)]
                Ok(ValueOrRange::Value(v as f64))
            }

            fn visit_seq<A>(self, mut seq: A) -> Result<Self::Value, A::Error>
            where
                A: SeqAccess<'de>,
            {
                let min: f64 = seq.next_element()?.ok_or_else(|| {
                    de::Error::invalid_length(0, &"a two-element [min, max] array")
                })?;
                let max: f64 = seq.next_element()?.ok_or_else(|| {
                    de::Error::invalid_length(1, &"a two-element [min, max] array")
                })?;
                if seq.next_element::<f64>()?.is_some() {
                    return Err(de::Error::invalid_length(
                        3,
                        &"a two-element [min, max] array",
                    ));
                }
                Ok(ValueOrRange::Range([min, max]))
            }
        }

        deserializer.deserialize_any(ValueOrRangeVisitor)
    }
}

impl ValueOrRange {
    /// Get minimum value
    pub fn min(&self) -> f64 {
        match self {
            ValueOrRange::Value(v) => *v,
            ValueOrRange::Range([min, _]) => *min,
        }
    }

    /// Get maximum value
    pub fn max(&self) -> f64 {
        match self {
            ValueOrRange::Value(v) => *v,
            ValueOrRange::Range([_, max]) => *max,
        }
    }

    /// Check if this is a fixed value
    pub fn is_fixed(&self) -> bool {
        matches!(self, ValueOrRange::Value(_))
    }
}

impl ScenarioSpec {
    /// Get the ego actor (expects exactly one)
    pub fn ego(&self) -> Result<&ActorSpec, String> {
        self.actors
            .iter()
            .find(|a| a.role == ActorRole::Ego)
            .ok_or_else(|| "No ego actor found".to_string())
    }

    /// Get all NPC actors (includes pedestrians)
    pub fn npcs(&self) -> Vec<&ActorSpec> {
        self.actors
            .iter()
            .filter(|a| a.role == ActorRole::Npc || a.role == ActorRole::Pedestrian)
            .collect()
    }

    /// Get actor by ID
    pub fn get_actor(&self, id: &str) -> Option<&ActorSpec> {
        self.actors.iter().find(|a| a.id == id)
    }

    /// Get lane width (backward compatible)
    pub fn get_lane_width(&self) -> f64 {
        self.road.as_ref().map_or(self.lane_width, |r| r.lane_width)
    }

    /// Get lane direction (backward compatible)
    /// Returns +1 for forward lanes, -1 for backward lanes
    pub fn get_lane_direction(&self, lane: usize) -> i32 {
        self.road.as_ref().map_or(1, |r| r.get_lane_direction(lane)) // Default: all forward
    }

    /// Get number of lanes (backward compatible)
    pub fn get_num_lanes(&self) -> usize {
        self.road.as_ref().map_or(2, |r| r.num_lanes) // Default: 2 lanes
    }

    /// Get bicycle parameters for an actor (uses actor-specific params or scenario defaults)
    pub fn get_bicycle_params(&self, actor: &ActorSpec) -> Option<BicycleParams> {
        actor
            .bicycle_params
            .clone()
            .or_else(|| self.bicycle_config.as_ref().map(|cfg| cfg.to_params()))
    }

    /// Validate the specification
    pub fn validate(&self) -> Result<(), String> {
        // Ensure road specification is present
        if self.road.is_none() {
            return Err("road specification is required".to_string());
        }

        // Time parameters
        if self.time_step <= 0.0 {
            return Err("time_step must be positive".to_string());
        }
        if self.duration <= 0.0 {
            return Err("duration must be positive".to_string());
        }
        if self.duration < self.time_step {
            return Err("duration must be >= time_step".to_string());
        }
        // SW-12/L2. `time_step` was bounded below only by `> 0.0`, so
        // `time_step: 1e-9` with `duration: 10` gave `num_time_steps() = 10^10`
        // and the process hung building Z3 variables before it ever reached the
        // solver. The floor is the smallest step that yields a tractable
        // horizon; the ceiling on `duration` bounds the same product from the
        // other side.
        if self.time_step < MIN_TIME_STEP {
            return Err(format!(
                "time_step must be at least {MIN_TIME_STEP} s (got {});                  smaller steps produce an intractable number of time steps",
                self.time_step
            ));
        }
        if self.duration > MAX_DURATION {
            return Err(format!(
                "duration must be at most {MAX_DURATION} s (got {})",
                self.duration
            ));
        }
        if self.num_time_steps() > MAX_TIME_STEPS {
            return Err(format!(
                "duration {} / time_step {} yields {} time steps, above the {MAX_TIME_STEPS}                  supported",
                self.duration,
                self.time_step,
                self.num_time_steps()
            ));
        }

        // Safety constraints
        if self.min_ttc <= 0.0 {
            return Err("min_ttc must be positive".to_string());
        }
        if self.min_distance <= 0.0 {
            return Err("min_distance must be positive".to_string());
        }

        // Validate road specification
        if let Some(road) = &self.road {
            road.validate()?;
        }

        // SW-12/L2. `lane_width` had a lower bound but no upper one, and the
        // three optional safety scalars below were never checked for sign at
        // all — a negative `min_lateral_distance` is a constraint no pair can
        // fail, silently disabling the check it looks like it enables.
        let lane_width = self.get_lane_width();
        if !(MIN_LANE_WIDTH..=MAX_LANE_WIDTH).contains(&lane_width) {
            return Err(format!(
                "lane_width must be within [{MIN_LANE_WIDTH}, {MAX_LANE_WIDTH}] m, got {lane_width}"
            ));
        }
        if let Some(v) = self.max_velocity {
            if v <= 0.0 {
                return Err(format!("max_velocity must be positive, got {v}"));
            }
        }
        if let Some(v) = self.min_velocity {
            if v < 0.0 {
                return Err(format!("min_velocity must be non-negative, got {v}"));
            }
        }
        if let (Some(lo), Some(hi)) = (self.min_velocity, self.max_velocity) {
            if lo > hi {
                return Err(format!("min_velocity {lo} exceeds max_velocity {hi}"));
            }
        }
        if let Some(d) = self.min_lateral_distance {
            if d <= 0.0 {
                return Err(format!("min_lateral_distance must be positive, got {d}"));
            }
        }
        if let Some(v) = self.max_relative_velocity {
            if v <= 0.0 {
                return Err(format!("max_relative_velocity must be positive, got {v}"));
            }
        }
        if self.max_lateral_acceleration <= 0.0 {
            return Err(format!(
                "max_lateral_acceleration must be positive, got {}",
                self.max_lateral_acceleration
            ));
        }

        // Generation parameters
        if self.num_scenarios == 0 {
            return Err("num_scenarios must be at least 1".to_string());
        }

        // NEW: Validate exactly one ego
        let ego_count = self
            .actors
            .iter()
            .filter(|a| a.role == ActorRole::Ego)
            .count();
        if ego_count != 1 {
            return Err(format!("Expected exactly 1 ego actor, found {}", ego_count));
        }

        // NEW: Validate at least one NPC or pedestrian
        let npc_count = self
            .actors
            .iter()
            .filter(|a| a.role == ActorRole::Npc || a.role == ActorRole::Pedestrian)
            .count();
        if npc_count == 0 {
            return Err("At least one NPC or pedestrian actor required".to_string());
        }

        // NEW: Validate unique actor IDs
        let mut seen_ids = std::collections::HashSet::new();
        for actor in &self.actors {
            if !seen_ids.insert(&actor.id) {
                return Err(format!("Duplicate actor ID: {}", actor.id));
            }
        }

        // NEW: Validate all actor parameters
        let num_lanes = self.get_num_lanes();
        for actor in &self.actors {
            if actor.speed.min() < 0.0 {
                return Err(format!("{} speed must be non-negative", actor.id));
            }
            if actor.acceleration.min() > actor.acceleration.max() {
                return Err(format!("{} acceleration range invalid", actor.id));
            }
            if let ValueOrRange::Range([min, max]) = actor.position {
                if min >= max {
                    return Err(format!("{} position range invalid: min >= max", actor.id));
                }
            }
            if let ValueOrRange::Range([min, max]) = actor.speed {
                if min >= max {
                    return Err(format!("{} speed range invalid: min >= max", actor.id));
                }
            }
            // Validate lane number
            if actor.lane >= num_lanes {
                return Err(format!(
                    "Actor {} lane {} exceeds num_lanes {}",
                    actor.id, actor.lane, num_lanes
                ));
            }
            // Validate direction
            if actor.direction != 1 && actor.direction != -1 {
                return Err(format!(
                    "Actor {} direction must be +1 (forward) or -1 (backward), got {}",
                    actor.id, actor.direction
                ));
            }
            // SW-21/M9. `parser::parse_yaml` used to validate `actor.direction`
            // here and then unconditionally *overwrite* it from
            // `road.lane_directions[actor.lane]` a few lines later, discarding
            // whatever the user wrote. An npc declaring `direction: -1` in a
            // `[1, 1]` road was silently flipped to `+1` with no error — the
            // validated field never actually took effect. Lane direction does
            // genuinely determine an actor's direction of travel (that is why
            // the overwrite existed), so the field is not removed from the
            // DSL; instead it is required to already agree with its lane,
            // which makes the overwrite in `parser::parse_yaml` a no-op
            // consistency check rather than a silent correction. (`self.road`
            // is `Some` here: the very first check in this function returns
            // early otherwise. `actor.lane < num_lanes` and
            // `road.lane_directions.len() == num_lanes` are both already
            // guaranteed by the checks above and by `RoadSpec::validate`, so
            // this index cannot panic.)
            if let Some(road) = &self.road {
                let lane_dir = road.lane_directions[actor.lane];
                if actor.direction != lane_dir {
                    return Err(format!(
                        "Actor {} declares direction {} but is in lane {}, whose \
                         lane_directions entry is {lane_dir}. An actor's direction \
                         is determined by its lane, not chosen independently \
                         (previously this mismatch was silently overwritten to \
                         match the lane) — set direction to {lane_dir}, or move \
                         the actor to a lane whose direction is {}",
                        actor.id, actor.direction, actor.lane, actor.direction
                    ));
                }
            }
            // SW-21/M10. `behavior` is an untyped `HashMap<String, Value>`
            // with no schema, so `walking_moad: run` (typo of `walking_mode`)
            // used to deserialize successfully and the reader
            // (`solver/encoders/pedestrian.rs`, `pedestrian_crossing.rs`)
            // fell back to its `"walk"` default, silently producing a walking
            // pedestrian for a scenario that asked for a runner. There is no
            // typed schema per scenario type, so this is the "at minimum"
            // option the issue names: reject any key that nothing in the
            // crate reads. `cut_in_time` was in this set until this issue
            // (grep confirms it: zero non-test readers; the actual cut-in
            // timing is `ActorSpec::lane_changes`) and has been dropped from
            // the two test fixtures that still had it, matching every shipped
            // example already having moved off it (see the "replaces
            // behavior.cut_in_time" comments in cut_in_left.yaml /
            // cut_in_right.yaml).
            for key in actor.behavior.keys() {
                if !ALLOWED_BEHAVIOR_KEYS.contains(&key.as_str()) {
                    return Err(format!(
                        "Actor {}: unknown behavior key '{key}'. Allowed keys: {}",
                        actor.id,
                        ALLOWED_BEHAVIOR_KEYS.join(", ")
                    ));
                }
            }
            // Validate lane changes.
            //
            // SW-12/M6. All three of these used to be silent: an out-of-range
            // target clamped to the actor's current lane and the encoder then
            // encoded a "transition" from lane N to lane N; a `start_time`
            // past the horizon was dropped by `collect_lane_change_data`'s
            // `filter_map`; and a non-positive `duration` gave
            // `duration_steps = 0`, so `start_step >= end_step` and
            // `encode_smooth_lane_transition` returned early. In every case
            // the requested manoeuvre disappeared and the scenario was emitted
            // as if it had been performed.
            let horizon = self.num_time_steps();
            let mut current_lane = i64::try_from(actor.lane).unwrap_or(i64::MAX);
            let lane_count = i64::try_from(num_lanes).unwrap_or(i64::MAX);
            for lc in &actor.lane_changes {
                if lc.start_time.min() > lc.start_time.max() {
                    return Err(format!(
                        "Lane change start_time range is invalid (min > max) for actor {}",
                        actor.id
                    ));
                }
                if lc.duration.min() > lc.duration.max() {
                    return Err(format!(
                        "Lane change duration range is invalid (min > max) for actor {}",
                        actor.id
                    ));
                }
                if lc.start_time.min() < 0.0 {
                    return Err(format!(
                        "Actor {}: lane change start_time must be non-negative, got {}",
                        actor.id,
                        lc.start_time.min()
                    ));
                }
                if lc.duration.min() <= 0.0 {
                    return Err(format!(
                        "Actor {}: lane change duration must be positive, got {}                          (a zero-length lane change is silently discarded by the encoder)",
                        actor.id,
                        lc.duration.min()
                    ));
                }
                if lc.duration.min() < self.time_step {
                    return Err(format!(
                        "Actor {}: lane change duration {} is shorter than time_step {},                          so it spans no time steps and would be discarded",
                        actor.id,
                        lc.duration.min(),
                        self.time_step
                    ));
                }
                // The encoder schedules the change at the midpoint of the
                // declared start window (`collect_lane_change_data`); a
                // midpoint past the horizon is a change that never happens.
                //
                // SW-25: the bound is `>=`, not `>`. Step `horizon` is the
                // *last* state in the trajectory, so a change scheduled to
                // begin there spans no simulated step at all — the same
                // "requested manoeuvre disappeared" class this check exists
                // to reject, and exactly what `CartesianEncoder::
                // encode_smooth_lane_transition` discards with its own
                // `start_step >= self.horizon` guard. With `>` the two
                // disagreed on precisely one value, and a spec landing on it
                // reached the encoder with a lane change that was dropped
                // while the step it was scheduled for was left with no
                // lateral constraint at all: `lane` floated free of `py`
                // (`cut_in_right.yaml` truncated to `duration: 5.0` gave
                // py = 5.00, inside lane 1, with lane = 0 at t = 10).
                let start_step = usize::midpoint(
                    (lc.start_time.min() / self.time_step) as usize,
                    (lc.start_time.max() / self.time_step) as usize,
                );
                if lane_change_start_past_horizon(start_step, horizon) {
                    return Err(format!(
                        "Actor {}: lane change starts at step {} (t = {:.3} s), at or past \
                         the scenario horizon of {} steps ({} s) — the last step is the \
                         final state, so the change would span no time step at all and \
                         be discarded",
                        actor.id,
                        start_step,
                        start_step as f64 * self.time_step,
                        horizon,
                        self.duration
                    ));
                }
                // Right is +1 lane in the actor's own travel direction, Left
                // is -1; for a backward actor that is mirrored in the road
                // frame. Same rule as `CartesianEncoder::encode_lane_change`
                // and `CutInLeftModel::cut_in_behavior`.
                let delta = match lc.direction {
                    LaneChangeDirection::Right => actor.direction as i64,
                    LaneChangeDirection::Left => -(actor.direction as i64),
                };
                let target = current_lane + delta;
                if target < 0 || target >= lane_count {
                    return Err(format!(
                        "Actor {}: lane change {:?} from lane {} leaves the road                          (target lane {}, road has {} lanes 0..{})",
                        actor.id,
                        lc.direction,
                        current_lane,
                        target,
                        num_lanes,
                        num_lanes - 1
                    ));
                }
                current_lane = target;
            }

            // SW-28. `scenarios::cut_in_conflict` already detects a cut-in
            // whose NPC never reaches the ego's lane and returns `True` for
            // it — correctly declining to assert a conflict that cannot
            // exist — but that leaves the spec's `enforce`d `min_ttc` /
            // `min_distance` never evaluated for the pair: no metric is ever
            // computed, so the scenario is reported as satisfying constraints
            // that were never once tested. `current_lane` here is the same
            // accumulated-delta target `cut_in_left.rs`/`cut_in_right.rs`'s
            // `cut_in_behavior` computes to find the NPC's merge target, so
            // this reuses that arithmetic instead of duplicating it.
            if matches!(
                self.scenario_type,
                ScenarioType::CutInLeft | ScenarioType::CutInRight
            ) && matches!(actor.role, ActorRole::Npc | ActorRole::Pedestrian)
                && !actor.lane_changes.is_empty()
            {
                if let Ok(ego) = self.ego() {
                    let ego_lane = i64::try_from(ego.lane).unwrap_or(i64::MAX);
                    if current_lane != ego_lane {
                        return Err(format!(
                            "{} names a cut-in, but actor {}'s lane change(s) end in \
                             lane {current_lane} while the ego is in lane {ego_lane} — \
                             the two never share a lane, so no conflict is ever \
                             asserted and min_ttc/min_distance are never evaluated for \
                             this pair even when `enforce`d. Either change {}'s \
                             lane_changes to end in lane {ego_lane} (to match the ego), \
                             or move the ego to lane {current_lane} (to match where {} \
                             actually ends up)",
                            self.scenario_type, actor.id, actor.id, actor.id
                        ));
                    }
                }
            }
        }

        // Validate acceleration ranges
        if let Some(max_accel) = self.max_acceleration {
            if max_accel <= 0.0 {
                return Err("max_acceleration must be positive".to_string());
            }
        }

        if let Some(max_decel) = self.max_deceleration {
            if max_decel >= 0.0 {
                return Err("max_deceleration must be negative".to_string());
            }
        }

        // A bad constraint_modes shorthand is now a compile-time-unrepresentable
        // state (ConstraintShorthand, SW-19) enforced at parse time by
        // ConstraintModes's custom Deserialize (SW-21), so there is nothing
        // left to validate here.

        // Warn if violating constraints
        if self.constraint_modes.min_ttc() == ConstraintMode::Violate
            || self.constraint_modes.min_distance() == ConstraintMode::Violate
        {
            tracing::warn!("Adversarial mode enabled - constraints will be violated");
        }

        // Validate bicycle configuration
        if self.coordinate_system == CoordinateSystem::Bicycle {
            // Validate scenario-level bicycle config if present
            if let Some(ref bicycle_config) = self.bicycle_config {
                bicycle_config.validate()?;
            }

            // Ensure all actors have bicycle params (either from actor or scenario defaults)
            for actor in &self.actors {
                if actor.role != ActorRole::Pedestrian {
                    let params = self.get_bicycle_params(actor);
                    if params.is_none() {
                        return Err(format!(
                            "Actor {} requires bicycle_params when coordinate_system is bicycle \
                             (either specify bicycle_params for the actor or bicycle_config at scenario level)",
                            actor.id
                        ));
                    }
                    // Validate actor-specific params if present
                    if let Some(ref params) = actor.bicycle_params {
                        params.validate()?;
                    }
                }
            }
        }

        // Validate bicycle params are only used with bicycle coordinate system
        if self.coordinate_system != CoordinateSystem::Bicycle {
            if self.bicycle_config.is_some() {
                return Err(
                    "bicycle_config can only be used with coordinate_system: bicycle".to_string(),
                );
            }
            for actor in &self.actors {
                if actor.bicycle_params.is_some() {
                    return Err(format!(
                        "Actor {} has bicycle_params but coordinate_system is not bicycle",
                        actor.id
                    ));
                }
            }
        }

        Ok(())
    }

    /// Get the number of time steps in the scenario
    pub fn num_time_steps(&self) -> usize {
        (self.duration / self.time_step).ceil() as usize
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_value_or_range_fixed() {
        let val = ValueOrRange::Value(10.0);
        assert_eq!(val.min(), 10.0);
        assert_eq!(val.max(), 10.0);
        assert!(val.is_fixed());
    }

    #[test]
    fn test_value_or_range_range() {
        let val = ValueOrRange::Range([5.0, 15.0]);
        assert_eq!(val.min(), 5.0);
        assert_eq!(val.max(), 15.0);
        assert!(!val.is_fixed());
    }

    #[test]
    fn test_num_time_steps() {
        let spec = create_test_spec();
        assert_eq!(spec.num_time_steps(), 20); // 10.0 / 0.5 = 20
    }

    #[test]
    fn test_actor_spec_helpers() {
        let spec = create_test_spec();

        assert!(spec.ego().is_ok());
        assert_eq!(spec.ego().unwrap().id, "ego");
        assert_eq!(spec.npcs().len(), 1);
        assert_eq!(spec.npcs()[0].id, "npc");
        assert!(spec.get_actor("ego").is_some());
        assert!(spec.get_actor("npc").is_some());
        assert!(spec.get_actor("unknown").is_none());
    }

    /// A spec that passes `validate()`, so each test below can break exactly
    /// one thing. `create_test_spec` has `road: None`, which `validate`
    /// rejects on its own.
    fn create_valid_spec() -> ScenarioSpec {
        let mut spec = create_test_spec();
        spec.road = Some(RoadSpec {
            num_lanes: 2,
            lane_width: 3.5,
            lane_directions: vec![1, 1],
            road_length: None,
        });
        // Right from lane 0 lands in lane 1, which exists.
        spec.actors[1].lane_changes[0].direction = LaneChangeDirection::Right;
        assert!(
            spec.validate().is_ok(),
            "fixture must be valid: {:?}",
            spec.validate()
        );
        spec
    }

    /// SW-12/M6: a lane change off the edge of the road is an error, not a clamp.
    ///
    /// `bicycle.rs` clamps the target with
    /// `(current_lane + lane_delta).clamp(0, num_lanes - 1)`, so `direction:
    /// left` from lane 0 used to become a "transition" from lane 0 to lane 0 —
    /// the requested manoeuvre silently disappeared and the scenario was
    /// emitted as though it had been performed.
    #[test]
    fn test_validate_rejects_lane_change_off_the_road() {
        let mut spec = create_valid_spec();
        spec.actors[1].lane = 0;
        spec.actors[1].lane_changes[0].direction = LaneChangeDirection::Left;

        let err = spec
            .validate()
            .expect_err("left from lane 0 leaves the road");
        assert!(
            err.contains("leaves the road") && err.contains("npc"),
            "error must name the actor and the problem, got: {err}"
        );
    }

    /// SW-12/M6: a lane change scheduled past the horizon is an error.
    ///
    /// `encoder_utils::collect_lane_change_data` dropped these with a
    /// `filter_map` returning `None`, with no warning.
    #[test]
    fn test_validate_rejects_lane_change_past_the_horizon() {
        let mut spec = create_valid_spec();
        // duration 10.0, so the horizon is t = 10.0.
        spec.actors[1].lane_changes[0].start_time = ValueOrRange::Value(30.0);

        let err = spec
            .validate()
            .expect_err("a lane change at t = 30 in a 10 s scenario cannot happen");
        assert!(
            err.contains("past the") && err.contains("horizon"),
            "error must say the change is past the horizon, got: {err}"
        );
    }

    /// SW-25: the boundary case the SW-12 check let through.
    ///
    /// `start_step == horizon` schedules the change to begin at the *final*
    /// state of the trajectory, so it spans no simulated step — the same
    /// silently-discarded manoeuvre the check above rejects, and exactly what
    /// `CartesianEncoder::encode_smooth_lane_transition` discards with its own
    /// `start_step >= self.horizon` guard. The check said `>` and so disagreed
    /// with the encoder on precisely this one value.
    #[test]
    fn test_validate_rejects_lane_change_starting_at_the_last_step() {
        let mut spec = create_valid_spec();
        // horizon = duration / time_step steps; put the change on the last one.
        let horizon = spec.num_time_steps();
        spec.actors[1].lane_changes[0].start_time =
            ValueOrRange::Value(horizon as f64 * spec.time_step);

        let err = spec
            .validate()
            .expect_err("a lane change beginning at the final state spans no time step");
        assert!(
            err.contains("past the") && err.contains("horizon"),
            "error must say the change is at or past the horizon, got: {err}"
        );
    }

    /// SW-12/M6: a non-positive lane-change duration is an error.
    ///
    /// `duration: 0` gave `duration_steps = 0`, so `start_step >= end_step`
    /// and `encode_smooth_lane_transition` returned early at
    /// `cartesian.rs:208` — again silently discarding the change. Only
    /// `min > max` was ever checked.
    #[test]
    fn test_validate_rejects_zero_lane_change_duration() {
        let mut spec = create_valid_spec();
        spec.actors[1].lane_changes[0].duration = ValueOrRange::Value(0.0);

        let err = spec
            .validate()
            .expect_err("a zero-length lane change is not a lane change");
        assert!(
            err.contains("duration must be positive"),
            "error must name the field and the accepted range, got: {err}"
        );
    }

    /// SW-12/L2: `time_step: 1e-9` is rejected instead of hanging.
    ///
    /// With `duration: 10` it gives `num_time_steps() = 10^10`, and the
    /// process died building Z3 variables long before it reached the solver.
    /// `validate` only ever checked `time_step > 0.0`.
    #[test]
    fn test_validate_rejects_a_time_step_that_would_hang() {
        let mut spec = create_valid_spec();
        spec.time_step = 1e-9;

        let err = spec
            .validate()
            .expect_err("1e-9 s steps over 10 s is 10^10 time steps");
        assert!(
            err.contains("time_step must be at least"),
            "error must name the field and the accepted range, got: {err}"
        );
    }

    /// SW-12/L2: the optional safety scalars are checked for sign.
    ///
    /// A negative `min_lateral_distance` is a constraint no pair can fail, so
    /// it silently disables the check it looks like it enables.
    #[test]
    fn test_validate_rejects_negative_optional_bounds() {
        for (name, apply) in [
            (
                "min_lateral_distance",
                (|s: &mut ScenarioSpec| s.min_lateral_distance = Some(-1.0))
                    as fn(&mut ScenarioSpec),
            ),
            ("max_velocity", |s: &mut ScenarioSpec| {
                s.max_velocity = Some(-1.0)
            }),
            ("max_relative_velocity", |s: &mut ScenarioSpec| {
                s.max_relative_velocity = Some(-1.0)
            }),
        ] {
            let mut spec = create_valid_spec();
            apply(&mut spec);
            let err = spec
                .validate()
                .expect_err("negative bound must be rejected");
            assert!(err.contains(name), "error must name {name}, got: {err}");
        }
    }

    /// SW-12/L2: `lane_width` is bounded from above as well as below.
    #[test]
    fn test_validate_rejects_an_absurd_lane_width() {
        let mut spec = create_valid_spec();
        if let Some(road) = spec.road.as_mut() {
            road.lane_width = 500.0;
        }
        let err = spec.validate().expect_err("a 500 m lane is not a lane");
        assert!(
            err.contains("lane_width must be within"),
            "error must name the field and the accepted range, got: {err}"
        );
    }

    fn create_test_spec() -> ScenarioSpec {
        use std::collections::HashMap;

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
            road: None,
            lane_width: 3.5,
            num_scenarios: 1,
            constraint_modes: ConstraintModes::default(),
            max_acceleration: None,
            max_deceleration: None,
            optimization_target: OptimizationTarget::None,
            max_velocity: None,
            min_velocity: None,
            min_lateral_distance: None,
            max_relative_velocity: None,
            max_lateral_acceleration: 2.0,
            coordinate_system: CoordinateSystem::Cartesian,
            bicycle_config: None,
        }
    }

    #[test]
    fn test_road_spec_validation() {
        let valid_road = RoadSpec {
            num_lanes: 4,
            lane_width: 3.5,
            lane_directions: vec![1, 1, -1, -1],
            road_length: None,
        };
        assert!(valid_road.validate().is_ok());

        let invalid_road = RoadSpec {
            num_lanes: 4,
            lane_width: 3.5,
            lane_directions: vec![1, 1, -1], // Wrong length
            road_length: None,
        };
        assert!(invalid_road.validate().is_err());
    }

    #[test]
    fn test_road_spec_invalid_direction() {
        let road = RoadSpec {
            num_lanes: 4,
            lane_width: 3.5,
            lane_directions: vec![1, 2, -1, -1], // 2 is invalid
            road_length: None,
        };
        assert!(road.validate().is_err());
    }

    #[test]
    fn test_road_spec_rejects_interleaved_directions() {
        // SW-16 E2: [1,-1,1,-1] would place scenario lane 1 (y centre 5.25) at
        // xodr's left lane +1 (y in [7.0, 10.5]) — a full lane-width off.
        let interleaved = RoadSpec {
            num_lanes: 4,
            lane_width: 3.5,
            lane_directions: vec![1, -1, 1, -1],
            road_length: None,
        };
        let err = interleaved
            .validate()
            .expect_err("interleaved must be rejected");
        assert!(err.contains("forward-then-backward"), "{err}");
    }

    #[test]
    fn test_road_spec_rejects_backward_first() {
        // Backward-first also breaks the exporter's right=forward/left=backward split
        // (see the comment on RoadSpec::validate), even though it is not "interleaved".
        let backward_first = RoadSpec {
            num_lanes: 4,
            lane_width: 3.5,
            lane_directions: vec![-1, -1, 1, 1],
            road_length: None,
        };
        assert!(backward_first.validate().is_err());
    }

    #[test]
    fn test_road_spec_accepts_forward_then_backward_block() {
        let ok = RoadSpec {
            num_lanes: 4,
            lane_width: 3.5,
            lane_directions: vec![1, 1, -1, -1],
            road_length: None,
        };
        assert!(ok.validate().is_ok());

        let all_forward = RoadSpec {
            num_lanes: 3,
            lane_width: 3.5,
            lane_directions: vec![1, 1, 1],
            road_length: None,
        };
        assert!(all_forward.validate().is_ok());

        let all_backward = RoadSpec {
            num_lanes: 2,
            lane_width: 3.5,
            lane_directions: vec![-1, -1],
            road_length: None,
        };
        assert!(all_backward.validate().is_ok());
    }

    #[test]
    fn test_get_lane_direction() {
        let road = RoadSpec {
            num_lanes: 4,
            lane_width: 3.5,
            lane_directions: vec![1, 1, -1, -1],
            road_length: None,
        };

        assert_eq!(road.get_lane_direction(0), 1);
        assert_eq!(road.get_lane_direction(2), -1);
        assert_eq!(road.get_lane_direction(10), 1); // Out of bounds, default
    }

    #[test]
    fn test_scenario_spec_backward_compat() {
        let yaml = r#"
    scenario_type: cut_in_left
    time_step: 0.5
    duration: 10.0
    actors:
      - id: ego
        role: ego
        lane: 0
        position: 50.0
        speed: 15.0
        direction: 1
        acceleration: [-8.0, 3.0]
      - id: npc
        role: npc
        lane: 1
        position: 60.0
        speed: 13.0
        direction: 1
        acceleration: [-8.0, 3.0]
    min_ttc: 3.0
    min_distance: 5.0
    lane_width: 3.5
    num_scenarios: 1
"#;

        let spec: ScenarioSpec = serde_yml::from_str(yaml).unwrap();
        assert_eq!(spec.get_lane_width(), 3.5);
        assert_eq!(spec.get_num_lanes(), 2); // Default
        assert_eq!(spec.get_lane_direction(0), 1); // All forward
    }

    #[test]
    fn test_parse_road_with_directions() {
        let yaml = r#"
scenario_type: cut_in_left
time_step: 0.5
duration: 10.0
road:
  num_lanes: 4
  lane_width: 3.5
  lane_directions: [1, 1, -1, -1]
actors:
  - id: ego
    role: ego
    lane: 0
    position: 50.0
    speed: 20.0
    direction: 1
    acceleration: [-8.0, 3.0]
  - id: npc
    role: npc
    lane: 2
    position: 150.0
    speed: 20.0
    direction: 1
    acceleration: [-8.0, 3.0]
min_ttc: 3.0
min_distance: 5.0
num_scenarios: 1
"#;

        let spec: ScenarioSpec = serde_yml::from_str(yaml).unwrap();
        assert_eq!(spec.get_num_lanes(), 4);
        assert_eq!(spec.get_lane_width(), 3.5);
        assert_eq!(spec.get_lane_direction(0), 1);
        assert_eq!(spec.get_lane_direction(2), -1);
    }

    /// SW-11: the minimum turn radius is the exact kinematic
    /// `L / tan(δ_max)`, not the small-angle `L / δ_max`.
    #[test]
    fn test_min_turn_radius_is_exact() {
        let params = BicycleParams {
            wheelbase: 2.7,
            max_steering_angle: 0.6,
            max_steering_rate: 0.5,
        };
        let exact = 2.7 / 0.6_f64.tan();
        assert!((params.min_turn_radius() - exact).abs() < 1e-12);
        // The value docs/coordinate-systems.md used to advertise, 14 % out.
        let small_angle: f64 = 2.7 / 0.6;
        assert!((small_angle - 4.5).abs() < 1e-9);
        assert!((params.min_turn_radius() - 3.9466).abs() < 1e-4);
        // 14 % apart, which is the error the docs advertised.
        assert!(((small_angle / params.min_turn_radius()) - 1.140).abs() < 1e-3);
    }

    /// `tan` is singular at pi/2, so the steering lock has to stay below it.
    #[test]
    fn test_steering_angle_must_be_below_a_right_angle() {
        let params = BicycleParams {
            wheelbase: 2.7,
            max_steering_angle: std::f64::consts::FRAC_PI_2,
            max_steering_rate: 0.5,
        };
        assert!(params.validate().is_err());

        let cfg = BicycleConfig {
            default_wheelbase: 2.7,
            default_max_steering_angle: 2.0,
            default_max_steering_rate: 0.5,
        };
        assert!(cfg.validate().is_err());
    }

    #[test]
    fn test_constraint_modes_unknown_shorthand() {
        // SW-21/D1: `enforce_al` (typo of `enforce_all`) used to deserialize
        // successfully as `Shorthand("enforce_al")` and the accessor's
        // `_ => ConstraintMode::Enforce` fallback made it behave as
        // `enforce_all` with no error and no warning — the exact inverse of
        // adversarial intent when the typo was e.g. `violate_al`. It is now a
        // parse error at deserialize time, naming the valid values.
        let err = serde_yml::from_str::<ConstraintModes>("enforce_al")
            .expect_err("a typo'd shorthand must not deserialize");
        let msg = err.to_string();
        assert!(
            msg.contains("enforce_al")
                && msg.contains("violate_all")
                && msg.contains("ignore_all")
                && msg.contains("enforce_all"),
            "error must name the bad value and the valid ones, got: {msg}"
        );

        // The three real values still deserialize and resolve correctly.
        for (input, expected) in [
            ("violate_all", ConstraintMode::Violate),
            ("ignore_all", ConstraintMode::Ignore),
            ("enforce_all", ConstraintMode::Enforce),
        ] {
            let modes: ConstraintModes = serde_yml::from_str(input).unwrap();
            assert_eq!(modes.min_ttc(), expected, "input {input}");
        }
    }

    /// SW-21/D1: a hyphenated typo of a shorthand value is a parse error
    /// naming the valid values — not a silent `enforce_all`, which is what a
    /// typo'd `violate_all` used to become (the exact inverse of the
    /// adversarial intent the user wrote).
    #[test]
    fn test_constraint_modes_hyphen_typo_is_a_parse_error() {
        let err = serde_yml::from_str::<ConstraintModes>("violate-all")
            .expect_err("'violate-all' (hyphen) must not silently become enforce_all");
        let msg = err.to_string();
        assert!(
            msg.contains("violate-all") && msg.contains("violate_all"),
            "error must name the bad value and suggest the real one, got: {msg}"
        );
    }

    /// SW-21/D2: a typo'd key inside the `Detailed` mapping form (`min_tcc`
    /// for `min_ttc`) used to deserialize successfully — untagged +
    /// `#[serde(default)]` on every field meant *any* mapping matched, and
    /// the misspelled key's real target silently kept its default mode. It is
    /// now a parse error naming the bad key.
    #[test]
    fn test_constraint_modes_detailed_rejects_unknown_key() {
        let err = serde_yml::from_str::<ConstraintModes>("min_tcc: violate")
            .expect_err("'min_tcc' (typo of min_ttc) must be rejected");
        let msg = err.to_string();
        assert!(
            msg.contains("min_tcc"),
            "error must name the offending key, got: {msg}"
        );
    }
}
