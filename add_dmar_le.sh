#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${BUILD_DIR:-"$ROOT_DIR/build"}"
INSTANCES="${INSTANCES:-5}"
RUNS="${RUNS:-3}"
MAX_ROUNDS="${MAX_ROUNDS:-5000}"
MAX_SENSING_RADIUS="${MAX_SENSING_RADIUS:-8}"
TEMP_OUTPUT="$(mktemp)"
trap 'rm -f "$TEMP_OUTPUT"' EXIT

alphas=(0.25 0.5 0.75)
expected_rows=$((INSTANCES * RUNS * 3 * MAX_SENSING_RADIUS))

cmake -S "$ROOT_DIR" -B "$BUILD_DIR"
cmake --build "$BUILD_DIR" --target dmar_benchmark

for side in 10 20 30; do
    missing=0
    for alpha in "${alphas[@]}"; do
        output="$ROOT_DIR/benchmark-${side}x${side}-alpha-${alpha}.csv"
        if [[ ! -f "$output" ]]; then
            echo "Missing benchmark file: $output" >&2
            exit 1
        fi
        count="$(awk -F, '$1 == "dmar_le" { count++ } END { print count + 0 }' "$output")"
        if [[ "$count" -gt "$expected_rows" ]]; then
            echo "Too many dmar_le rows in $output: $count (expected $expected_rows)" >&2
            exit 1
        fi
        if [[ "$count" -lt "$expected_rows" ]]; then
            missing=1
        fi
    done

    if [[ "$missing" -eq 0 ]]; then
        echo "dmar_le already complete for side $side; skipping." >&2
        continue
    fi

    echo "Generating dmar_le rows for side $side." >&2
    "$BUILD_DIR/dmar_benchmark" \
        "$side" \
        "$INSTANCES" \
        "$RUNS" \
        "$MAX_ROUNDS" \
        "$MAX_SENSING_RADIUS" \
        0.25 \
        1 \
        0 \
        0 \
        0 \
        0 \
        -1 \
        0 \
        3 > "$TEMP_OUTPUT"

    for alpha in "${alphas[@]}"; do
        output="$ROOT_DIR/benchmark-${side}x${side}-alpha-${alpha}.csv"
        count="$(awk -F, '$1 == "dmar_le" { count++ } END { print count + 0 }' "$output")"
        if [[ "$count" -eq "$expected_rows" ]]; then
            echo "  $output already complete; skipping." >&2
            continue
        fi
        awk -F, -v OFS=, -v side="$side" -v alpha="$alpha" \
            'NR > 1 && $1 == "dmar_le" && $2 == side {$17 = alpha; print}' \
            "$TEMP_OUTPUT" >> "$output"
        count="$(awk -F, '$1 == "dmar_le" { count++ } END { print count + 0 }' "$output")"
        if [[ "$count" -ne "$expected_rows" ]]; then
            echo "Unexpected dmar_le row count in $output: $count (expected $expected_rows)" >&2
            exit 1
        fi
    done
done

echo "dmar_le rows are complete." >&2
