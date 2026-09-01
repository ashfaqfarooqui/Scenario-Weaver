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
#   (a) Once the `cargo clippy --all-targets` warning count (currently ~446,
#       concentrated in tests/) is burned down, add `-D warnings` to that step
#       below so it gates like the --lib --bins step already does.
#   (a2) Drive LIB_BINS_BASELINE below to 0 and switch that step back to a plain
#       `-D warnings` gate. Deferred to wave 5 deliberately: 23 of the 62 are
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
  LIB_BINS_BASELINE=62

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
  # count findings, dropping the trailing "... generated N warnings" summary lines
  lib_bins_count="$(grep -E '^warning: ' <<<"$lib_bins_output" | grep -vc 'generated .* warning' || true)"

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
  all_targets_warn_count="$(grep -c '^warning:' <<<"$all_targets_output" || true)"
  RESULTS+=("cargo clippy --all-targets|report|0|${all_targets_warn_count} warning(s) — not gating, see TODO(SW-02)")
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
