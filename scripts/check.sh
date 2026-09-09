#!/usr/bin/env bash
#
# check.sh — local pre-commit gate for scenario-weaver (D2, supersedes SW-02).
#
# There is deliberately no `.github/workflows/ci.yml` here: `openscenario-rs` is
# consumed as a local path dependency at `../../Workspace_OpenScenario-rs/main`
# while the two crates are developed together, so a cloud CI runner cannot
# resolve the build. This script is the substitute — run it before committing.
#
# TODO(SW-02):
#   (a) Once the `cargo clippy --all-targets` warning count (currently 693,
#       concentrated in tests/) is burned down, add `-D warnings` to that step
#       below so it gates like the --lib --bins step already does.
#       That figure is NOT comparable to the ~446 and 564 this comment and the
#       summary line reported before SW-56: those counted printed `^warning: `
#       lines across every crate, dependency included. It now sums clippy's own
#       per-target totals for `scenario-weaver` alone, which include the
#       duplicates clippy suppresses from its printed output (e.g. the lib-test
#       target's "308 warnings (58 duplicates)"). The count went up because the
#       measure changed, not because the tree got worse.
#   (a2) Drive LIB_BINS_BASELINE below to 0 and switch that step back to a plain
#       `-D warnings` gate. Deferred to wave 5 deliberately: 23 of the 60 are
#       wrap-around cast lints in solver code that SW-08 (exact f64->Real
#       conversion) rewrites anyway, so fixing them now is duplicated work.
#   (b) Once openscenario-rs 0.3.3 is published to crates.io and the path
#       dependency in Cargo.toml is replaced with a version dependency,
#       promote this script to a real `.github/workflows/ci.yml`.
#   Also: SW-05 will add an XSD-validation step for emitted .xosc/.xodr
#   artifacts, and SW-07 will add nextest config (profiles, per-test
#   timeouts) — update the `cargo nextest run` invocation below when it lands.
#
# Usage:
#   scripts/check.sh          # run everything
#   scripts/check.sh --fast   # skip both clippy steps, just fmt + tests

set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

FAST=0
for arg in "$@"; do
  case "$arg" in
    --fast) FAST=1 ;;
    *)
      echo "check.sh: unknown argument: $arg" >&2
      echo "usage: $0 [--fast]" >&2
      exit 2
      ;;
  esac
done

# --- Gate ownership guard -------------------------------------------------
#
# This script takes 4-6 minutes (the full nextest suite dominates; the single
# test `scenario_invariants_hold_over_perturbed_cut_in_left` alone runs 132-215s
# against a 360s per-test timeout). That duration is exactly why it must not be
# launched detached: a backgrounded run cannot be waited on usefully, and twelve
# separate remediation sessions have stalled polling one.
#
# The gate is therefore owned by the human/orchestrator, not by task agents.
# If you are working on a single SW issue, do NOT run this script. Use the
# targeted commands instead — they answer the same question in seconds:
#
#   cargo fmt --check
#   # ratchet (baseline LIB_BINS_BASELINE below); count it exactly this way --
#   # `openscenario-rs` is a local path dependency (D1), so an unscoped
#   # `grep -c '^warning'` also counts ITS warnings on a cold cache, and a
#   # plain `grep -c '^warning'` additionally matches the "generated N
#   # warnings" summary lines and reads high on top of that. Sum the number(s)
#   # off the `scenario-weaver` (lib)/(bin ...) summary line(s) instead:
#   cargo clean -p scenario-weaver >/dev/null 2>&1
#   cargo clippy --lib --bins 2>&1 | grep -oE '^warning: `scenario-weaver` \((lib|bin)[^)]*\) generated [0-9]+ warning' | grep -oE '[0-9]+' | awk '{s+=$1} END{print s+0}'
#   cargo nextest run -E 'test(<the test you changed>)'
#   cargo nextest run -E 'test(test_kinematic_consistency_across_the_corpus)'
#
# and ask the orchestrator to run the full gate when you believe you are done.
#
if [[ "${SCENARIO_WEAVER_GATE:-}" != "1" ]]; then
  cat >&2 <<'GATE_EOF'
check.sh: refusing to run.

This is the full pre-commit gate (4-6 minutes) and it is owned by the
orchestrator, not by per-issue task agents. It must never be backgrounded.

If you are a task agent: do not run this script. Run the targeted commands
documented at the top of the guard in scripts/check.sh, and ask the
orchestrator to run the gate for you.

If you are the orchestrator and you are running this in the FOREGROUND:

    SCENARIO_WEAVER_GATE=1 scripts/check.sh

GATE_EOF
  exit 3
fi

# Each entry: "name|status|gating(0/1)|detail"
declare -a RESULTS=()
OVERALL_FAIL=0

run_step() {
  local name="$1" gating="$2"
  shift 2
  echo
  echo "==> $name"
  local output status
  output="$("$@" 2>&1)"
  status=$?
  echo "$output"
  if [[ $status -eq 0 ]]; then
    RESULTS+=("$name|pass|$gating|")
  else
    RESULTS+=("$name|fail|$gating|exit code $status")
    if [[ "$gating" -eq 1 ]]; then
      OVERALL_FAIL=1
    fi
  fi
}

# 1. cargo fmt --check (gating)
run_step "cargo fmt --check" 1 cargo fmt --check

if [[ "$FAST" -eq 0 ]]; then
  # 2. cargo clippy --lib --bins (gating, as a RATCHET)
  #
  # The production lib is not clean: it carries LIB_BINS_BASELINE warnings today.
  # Rather than gate on zero (which would fail every run and so gate nothing) or
  # drop to reporting-only (which would let new warnings in unnoticed), this step
  # fails if the count goes UP. Ratcheting down is free; ratcheting up is caught.
  # When you legitimately reduce the count, lower the baseline in the same commit.
  # 62 -> 60 in the SW-11 commit: the two `manual_let_else` findings in
  # `src/solver/encoders/bicycle.rs` are gone, one because the `match` around
  # `get_actor_bicycle_params` became a `let ... else`, the other because the
  # block that held it was rewritten. Nothing was suppressed or allow()d.
  # 60 -> 59 in the SW-20 commit adopting `lane_ids::lane_index_to_xodr_id`
  # in `xodr_exporter::build_lane_section`: the `n_forward as i64` cast in the
  # hand-rolled counters it replaced was clippy::cast_possible_wrap; the
  # helper's own `usize::try_into().unwrap_or(i64::MAX)` doesn't trigger it.
  # Verified by diffing `cargo clippy --lib --bins` (each run preceded by
  # `cargo clean -p scenario-weaver`) with and without this commit's change to
  # `src/scenario/{xodr_exporter,xosc_exporter,lane_ids}.rs`: that one line is
  # the only difference in the warning set, 60 lines vs 59.
  LIB_BINS_BASELINE=59

  # Cargo fingerprints a clippy unit like any other build unit: on a warm cache
  # it reports "Finished" and re-emits NOTHING. Both clippy steps below then
  # count 0 findings. For the ratchet that is not a harmless nuisance — 0 is
  # BELOW the baseline, so the step passes and advises lowering the baseline to
  # 0, which would retire the ratchet permanently. A gate that checks nothing is
  # exactly the failure this project exists to remove, so force re-emission by
  # invalidating this crate's own artifacts (deps are untouched, so the cost is
  # recompiling scenario-weaver alone).
  cargo clean -p scenario-weaver >/dev/null 2>&1 || true

  echo
  echo "==> cargo clippy --lib --bins (ratchet, baseline ${LIB_BINS_BASELINE})"
  lib_bins_output="$(cargo clippy --lib --bins 2>&1)"
  echo "$lib_bins_output"
  # `openscenario-rs` is a local path dependency (D1): on a cold dependency
  # clippy cache it emits its own "warning: ..." lines and its own per-crate
  # summary line in this same output, and a crate-unscoped count picks those
  # up too. Read the count off the `scenario-weaver` (lib)/(bin ...) summary
  # line(s) instead of counting individual `^warning:` lines. `--lib --bins`
  # can produce a summary line per target (a lib one and a bin one), so sum
  # them rather than taking the first match. The summary line can also carry
  # a trailing parenthetical ("(2 duplicates)", "(run cargo clippy --fix
  # ...)"), so extract the number by pattern, not by fixed field position.
  lib_bins_summary_lines="$(grep -E '^warning: `scenario-weaver` \((lib|bin)' <<<"$lib_bins_output" || true)"
  lib_bins_count=0
  if [[ -n "$lib_bins_summary_lines" ]]; then
    while IFS= read -r summary_line; do
      finding_n="$(grep -oE 'generated [0-9]+ warning' <<<"$summary_line" | grep -oE '[0-9]+')"
      lib_bins_count=$((lib_bins_count + finding_n))
    done <<<"$lib_bins_summary_lines"
  fi

  # A zero count means clippy re-emitted nothing, not that the lib became clean:
  # the crate has never been at zero. Treat it as a broken measurement and fail,
  # rather than silently passing and inviting the baseline down to 0.
  if [[ "$lib_bins_count" -eq 0 && "$LIB_BINS_BASELINE" -gt 0 ]]; then
    RESULTS+=("cargo clippy --lib --bins|fail|1|clippy emitted 0 findings against a baseline of ${LIB_BINS_BASELINE} — stale cache, measurement is not trustworthy; run 'cargo clean -p scenario-weaver' and retry")
    OVERALL_FAIL=1
  elif [[ "$lib_bins_count" -gt "$LIB_BINS_BASELINE" ]]; then
    RESULTS+=("cargo clippy --lib --bins|fail|1|${lib_bins_count} warnings, ABOVE baseline ${LIB_BINS_BASELINE} — your change added $((lib_bins_count - LIB_BINS_BASELINE))")
    OVERALL_FAIL=1
  elif [[ "$lib_bins_count" -lt "$LIB_BINS_BASELINE" ]]; then
    RESULTS+=("cargo clippy --lib --bins|pass|1|${lib_bins_count} warnings, BELOW baseline ${LIB_BINS_BASELINE} — lower LIB_BINS_BASELINE to ${lib_bins_count} in this commit")
  else
    RESULTS+=("cargo clippy --lib --bins|pass|1|${lib_bins_count} warnings, at baseline")
  fi

  # 3. cargo clippy --all-targets (reporting only, never gating)
  echo
  echo "==> cargo clippy --all-targets (reporting only, not gating)"
  cargo clean -p scenario-weaver >/dev/null 2>&1 || true
  all_targets_output="$(cargo clippy --all-targets 2>&1)"
  echo "$all_targets_output"
  # Reporting-only (see TODO(SW-02) above), but scope it the same way as the
  # --lib --bins ratchet above: `openscenario-rs` is a local path dependency
  # (D1) and its warnings otherwise inflate this count on a cold cache too.
  all_targets_summary_lines="$(grep -E '^warning: `scenario-weaver` \((lib|bin|test)' <<<"$all_targets_output" || true)"
  all_targets_warn_count=0
  if [[ -n "$all_targets_summary_lines" ]]; then
    while IFS= read -r summary_line; do
      finding_n="$(grep -oE 'generated [0-9]+ warning' <<<"$summary_line" | grep -oE '[0-9]+')"
      all_targets_warn_count=$((all_targets_warn_count + finding_n))
    done <<<"$all_targets_summary_lines"
  fi
  RESULTS+=("cargo clippy --all-targets|report|0|${all_targets_warn_count} warning(s) (scenario-weaver only) — not gating, see TODO(SW-02)")
else
  RESULTS+=("cargo clippy --lib --bins|skipped|1|--fast")
  RESULTS+=("cargo clippy --all-targets|skipped|0|--fast")
fi

# 4. tests: prefer nextest, fall back to cargo test (gating)
if command -v cargo-nextest >/dev/null 2>&1; then
  run_step "cargo nextest run" 1 cargo nextest run
else
  run_step "cargo test (cargo-nextest not found, fell back)" 1 cargo test
fi

# ---- summary ----
echo
echo "================ check.sh summary ================"
printf '%-55s %-8s %-9s %s\n' "STEP" "STATUS" "GATING" "DETAIL"
for entry in "${RESULTS[@]}"; do
  IFS='|' read -r name status gating detail <<<"$entry"
  gate_label="report"
  [[ "$gating" -eq 1 ]] && gate_label="gating"
  printf '%-55s %-8s %-9s %s\n' "$name" "$status" "$gate_label" "$detail"
done
echo "===================================================="

if [[ "$OVERALL_FAIL" -eq 1 ]]; then
  echo "RESULT: FAIL — one or more gating steps failed. See details above."
  exit 1
else
  echo "RESULT: PASS — all gating steps passed."
  exit 0
fi
