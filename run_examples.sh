#!/usr/bin/env bash
# run_examples.sh
#
# Build ScenarioWeaver and run all (or selected) examples, writing each
# scenario's outputs to output/<example_name>/.
#
# Usage:
#   ./run_examples.sh                      # run all examples
#   ./run_examples.sh cut_in_left          # run one example by name (no .yaml)
#   ./run_examples.sh cut_in_left overtake_left  # run several
#   ./run_examples.sh --no-build           # skip cargo build
#
# Output:
#   output/<example_name>/scenario_*.json
#   output/<example_name>/scenario_*.xosc
#   output/<example_name>/scenario_*.xodr
#   output/<example_name>/scenario_*.svg
#   output/<example_name>/scenario_*.gif
#   output/<example_name>/scenario_*.ol.json

set -euo pipefail

EXAMPLES_DIR="examples"
OUTPUT_DIR="output"
BINARY="./target/release/scenario-weaver"

# ── Colour helpers ────────────────────────────────────────────────────────────
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
RESET='\033[0m'

ok()   { echo -e "${GREEN}[ok]${RESET}  $*"; }
fail() { echo -e "${RED}[fail]${RESET} $*"; }
info() { echo -e "${CYAN}[info]${RESET} $*"; }
warn() { echo -e "${YELLOW}[warn]${RESET} $*"; }

# ── Parse arguments ───────────────────────────────────────────────────────────
BUILD=true
SELECTED=()

for arg in "$@"; do
    case "$arg" in
        --no-build) BUILD=false ;;
        *)          SELECTED+=("$arg") ;;
    esac
done

# ── Build ─────────────────────────────────────────────────────────────────────
if $BUILD; then
    info "Building release binary..."
    cargo build --release 2>&1 | tail -3
fi

if [[ ! -x "$BINARY" ]]; then
    fail "Binary not found at $BINARY. Run without --no-build."
    exit 1
fi

# ── Collect examples to run ───────────────────────────────────────────────────
if [[ ${#SELECTED[@]} -gt 0 ]]; then
    EXAMPLES=()
    for name in "${SELECTED[@]}"; do
        yaml="$EXAMPLES_DIR/${name%.yaml}.yaml"
        if [[ ! -f "$yaml" ]]; then
            warn "Example not found: $yaml (skipping)"
        else
            EXAMPLES+=("$yaml")
        fi
    done
else
    mapfile -t EXAMPLES < <(ls "$EXAMPLES_DIR"/*.yaml 2>/dev/null | sort)
fi

if [[ ${#EXAMPLES[@]} -eq 0 ]]; then
    fail "No examples found."
    exit 1
fi

# ── Run each example ──────────────────────────────────────────────────────────
PASS=0
FAIL=0
FAIL_NAMES=()

echo ""
info "Running ${#EXAMPLES[@]} example(s)..."
echo ""

for yaml in "${EXAMPLES[@]}"; do
    name=$(basename "$yaml" .yaml)
    out="$OUTPUT_DIR/$name"
    mkdir -p "$out"

    printf "  %-45s" "$name"

    start=$SECONDS
    if "$BINARY" -i "$yaml" -o "$out/" > "$out/run.log" 2>&1; then
        elapsed=$(( SECONDS - start ))
        count=$(ls "$out"/*.json 2>/dev/null | wc -l | tr -d ' ')
        ok "${count} scenario(s) in ${elapsed}s"
        (( PASS++ ))
    else
        elapsed=$(( SECONDS - start ))
        fail "FAILED after ${elapsed}s  (see $out/run.log)"
        (( FAIL++ ))
        FAIL_NAMES+=("$name")
    fi
done

# ── Summary ───────────────────────────────────────────────────────────────────
echo ""
echo "──────────────────────────────────────────"
info "Results: ${PASS} passed, ${FAIL} failed"
if [[ ${#FAIL_NAMES[@]} -gt 0 ]]; then
    for n in "${FAIL_NAMES[@]}"; do
        fail "  $n"
    done
    exit 1
fi
echo ""
info "Outputs written to: $OUTPUT_DIR/"
