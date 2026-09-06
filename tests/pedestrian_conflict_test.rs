//! SW-43 — an `enforce`d pedestrian `min_ttc` must be a bound the crossing can
//! actually fail.
//!
//! `PedestrianTTCGT` lowers to the guarded implication
//! `(ped_on_road ∧ ego_behind ∧ ego_vx > 0) ⟹ ttc_safe`, and the pedestrian's
//! `py` is a solver variable. `pedestrian_crossing.rs::generate_ltl` asks only
//! that the pedestrian reach the far sidewalk *eventually*, so Z3 was free to
//! keep it clear of the road for exactly the steps where the ego was bearing
//! down on it and to cross once the ego had gone past — satisfying
//! `G(PedestrianTTCGT(..))` with its antecedent false wherever it mattered. A
//! vacuously-true constraint and a genuinely satisfied one are
//! indistinguishable from the outside, which is why the two tests here assert
//! the antecedent's *truth* and the bound's *bite* rather than the suite being
//! green.
//!
//! Both fail at the pre-SW-43 tree: the first because the pedestrian steps off
//! the road while the ego approaches, the second because the evasion made an
//! otherwise impossible bound satisfiable.

mod common;

/// A pedestrian crossing a two-lane road in front of an ego with an `enforce`d
/// 2 s TTC. Nothing here is unusual — it is `examples/pedestrian_crossing.yaml`
/// with `min_ttc` promoted from `ignore` to `enforce`.
const ENFORCED_TTC: &str = r#"
scenario_type: pedestrian_crossing
time_step: 0.3
duration: 10.0
road:
  num_lanes: 2
  lane_width: 3.5
  lane_directions: [1, 1]
actors:
  - id: ego
    role: ego
    lane: 0
    position: [0.0, 40.0]
    speed: [8.0, 12.0]
    direction: 1
    acceleration: [-5.0, 2.0]
  - id: pedestrian
    role: pedestrian
    lane: 0
    position: [10.0, 60.0]
    speed: [0.8, 1.5]
    direction: 1
    acceleration: [-1.0, 1.0]
    behavior:
      walking_mode: walk
      direction: left_to_right
constraint_modes:
  min_ttc: enforce
  min_distance: ignore
min_ttc: 2.0
min_distance: 2.0
num_scenarios: 1
"#;

/// Whenever the pedestrian is on the road, the ego must be behind it and still
/// moving toward it — the exact antecedent `PedestrianTTCGT` is conditioned on
/// (`encode_pedestrian_ttc_guard`), re-derived here from the shipped
/// trajectory rather than read back out of the encoding.
///
/// At the pre-SW-43 tree this fails: the pedestrian starts at the lane centre
/// (`py[0]` is pinned there), leaves the road while the ego is still ~22 m
/// away, waits out the whole approach on the kerb, and crosses over the last
/// third of the horizon with the ego already past it — so the enforced 2 s
/// bound was evaluated only at the opening steps, 33 m out.
#[test]
fn test_enforced_pedestrian_ttc_makes_the_crossing_a_conflict() {
    // Generated directly rather than through `common::generate_yaml_with_spec`
    // so the per-step conflict assertion below is what fires, not the shared
    // invariant set — which at the pre-SW-43 tree fails first, on the same
    // trajectory, with `min_ttc: declared Enforce against threshold 2s,
    // measured 0.129s`. The full invariant set still runs, at the end.
    let spec = scenario_weaver::dsl::parser::parse_yaml(ENFORCED_TTC)
        .unwrap_or_else(|e| panic!("cannot parse scenario YAML: {e}"));
    let scenario = scenario_weaver::generate_single_scenario_from_spec(spec.clone())
        .unwrap_or_else(|e| panic!("expected a solvable scenario, solver returned: {e}"));
    let road_width = spec.get_lane_width() * spec.get_num_lanes() as f64;

    let ego = scenario.get_actor("ego").expect("ego trajectory");
    let ped = scenario
        .get_actor("pedestrian")
        .expect("pedestrian trajectory");

    let mut on_road_steps = 0_usize;
    let mut crossing_steps = 0_usize;
    for (t, (e, p)) in ego.states.iter().zip(ped.states.iter()).enumerate() {
        let py = p.position().y;
        if py < 0.0 || py > road_width {
            continue;
        }
        on_road_steps += 1;
        if py > 0.0 && py < road_width {
            crossing_steps += 1;
        }
        assert!(
            e.position().x < p.position().x && e.velocity().vx > 0.0,
            "step {t}: the pedestrian is on the road (py={py:.3}, road_width={road_width}) \
             while the ego is not approaching it (ego_px={:.3}, ped_px={:.3}, ego_vx={:.3}) — \
             an enforced min_ttc that the crossing can never put at risk",
            e.position().x,
            p.position().x,
            e.velocity().vx
        );
    }

    // The property above is vacuous if the pedestrian never reaches the road,
    // and near-vacuous if it only ever touches the boundary; both are ruled
    // out here so a trajectory that dodges the road entirely cannot pass.
    assert!(
        on_road_steps > 1,
        "the pedestrian was on the road at {on_road_steps} step(s) — nothing was tested"
    );
    assert!(
        crossing_steps > 0,
        "the pedestrian never got strictly inside the road, so it never crossed in front \
         of the ego"
    );

    common::invariants::assert_scenario_invariants(&scenario, &spec);
}

/// The same crossing across a *three*-lane road, which the pedestrian cannot
/// complete inside the window the enforced TTC leaves it.
///
/// The ego runs at a fixed 12 m/s with zero acceleration, so `ttc >= 2 s` is
/// exactly `ped_px - ego_px >= 24 m` and the window in which the pedestrian may
/// be on the road at all closes at `t = (ped_px - 24)/12 <= 3 s`. Crossing
/// 10.5 m of road from the lane-0 centre takes at least 4.4 s at the 2 m/s
/// walking cap, and the crossing goal (`F(OnSidewalk(right))`) forbids
/// retreating to the near kerb instead. No model exists.
///
/// This is the inverted-property half: at the pre-SW-43 tree this spec is
/// **satisfiable**, because the pedestrian is free to stand on the kerb until
/// the ego has driven past and cross behind it — which is precisely the
/// evasion that made the bound unfailable. A test that only checked the
/// spec still solved could not tell the two trees apart.
#[test]
fn test_enforced_pedestrian_ttc_a_crossing_cannot_meet_is_infeasible() {
    let yaml = ENFORCED_TTC
        .replace("num_lanes: 2", "num_lanes: 3")
        .replace("lane_directions: [1, 1]", "lane_directions: [1, 1, 1]")
        .replace("time_step: 0.3", "time_step: 0.5")
        .replace("    position: [0.0, 40.0]\n", "    position: 0.0\n")
        .replace("    speed: [8.0, 12.0]\n", "    speed: 12.0\n")
        .replace("    acceleration: [-5.0, 2.0]\n", "    acceleration: 0.0\n");

    let spec = scenario_weaver::dsl::parser::parse_yaml(&yaml)
        .unwrap_or_else(|e| panic!("cannot parse scenario YAML: {e}"));
    common::assert_infeasible(
        spec,
        "an enforced 2 s TTC against a 12 m/s ego leaves under 3 s in which the pedestrian \
         may be on the road, and crossing three lanes takes at least 4.4 s",
    );
}
