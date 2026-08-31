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
  # 2. cargo clippy --lib --bins -D warnings (gating)
  echo
  echo "==> cargo clippy --lib --bins -- -D warnings"
  lib_bins_output="$(cargo clippy --lib --bins -- -D warnings 2>&1)"
  lib_bins_status=$?
  echo "$lib_bins_output"
  if [[ $lib_bins_status -eq 0 ]]; then
    RESULTS+=("cargo clippy --lib --bins -- -D warnings|pass|1|")
  else
    # -D warnings promotes each finding to an "error:" line, plus one summary
    # line ("could not compile ... due to N previous errors"); prefer that
    # summary count when present, else fall back to counting error: lines.
    lib_bins_warn_count="$(grep -oE 'due to [0-9]+ previous error' <<<"$lib_bins_output" | grep -oE '[0-9]+' | tail -1)"
    if [[ -z "$lib_bins_warn_count" ]]; then
      lib_bins_warn_count="$(grep -c '^error:' <<<"$lib_bins_output" || true)"
    fi
    RESULTS+=("cargo clippy --lib --bins -- -D warnings|fail|1|${lib_bins_warn_count} warning(s) — see output above; the production lib is expected to be near-clean, this is a real regression to fix")
    OVERALL_FAIL=1
  fi

  # 3. cargo clippy --all-targets (reporting only, never gating)
  echo
  echo "==> cargo clippy --all-targets (reporting only, not gating)"
  all_targets_output="$(cargo clippy --all-targets 2>&1)"
  echo "$all_targets_output"
  all_targets_warn_count="$(grep -c '^warning:' <<<"$all_targets_output" || true)"
  RESULTS+=("cargo clippy --all-targets|report|0|${all_targets_warn_count} warning(s) — not gating, see TODO(SW-02)")
else
  RESULTS+=("cargo clippy --lib --bins -- -D warnings|skipped|1|--fast")
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
