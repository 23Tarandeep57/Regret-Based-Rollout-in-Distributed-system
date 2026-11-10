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

cmake -S "$ROOT_DIR" -B "$BUILD_DIR"
cmake --build "$BUILD_DIR" --target dmar_benchmark

for alpha in 0.25 0.5 0.75; do
    for side in 10 20 30; do
        output="$ROOT_DIR/benchmark-${side}x${side}-alpha-${alpha}.csv"
        echo "Writing $output" >&2
        "$BUILD_DIR/dmar_benchmark" \
            "$side" \
            "$INSTANCES" \
            "$RUNS" \
            "$MAX_ROUNDS" \
            "$MAX_SENSING_RADIUS" \
            "$alpha" \
            1 \
            0 > "$TEMP_OUTPUT"
        awk -F, -v side="$side" 'NR == 1 || $2 == side' "$TEMP_OUTPUT" > "$output"
    done
done

echo "Benchmark complete." >&2
