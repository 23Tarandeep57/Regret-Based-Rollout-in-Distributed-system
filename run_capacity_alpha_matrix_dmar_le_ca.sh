#!/usr/bin/env bash
set -euo pipefail

cmake --build build --target dmar_benchmark -j2

INSTANCES="${INSTANCES:-10}"
RUNS="${RUNS:-3}"
MAX_ROUNDS="${MAX_ROUNDS:-5000}"
MAX_RADIUS="${MAX_RADIUS:-8}"

for PROFILE in 1 2 3; do
  case "$PROFILE" in
    1) PROFILE_LABEL="heterogeneous" ;;
    2) PROFILE_LABEL="moderate-spread" ;;
    3) PROFILE_LABEL="high-spread" ;;
  esac

  OUTPUT="benchmark-30x30-${PROFILE_LABEL}-capacity-alpha-matrix-5-tasks-per-agent-10-agents-dmar-le-ca.csv"
  TEMP_OUTPUT="${OUTPUT}.tmp"
  first_alpha=1
  : > "$OUTPUT"

  for ALPHA in 0 0.5 1; do
    ./build/dmar_benchmark \
      30 \
      "$INSTANCES" \
      "$RUNS" \
      "$MAX_ROUNDS" \
      "$MAX_RADIUS" \
      "$ALPHA" \
      1 \
      0 \
      0 \
      0 \
      0 \
      -1 \
      0 \
      4 \
      5 \
      30 \
      10 \
      "$PROFILE" \
      > "$TEMP_OUTPUT"

    if [[ "$first_alpha" -eq 1 ]]; then
      cat "$TEMP_OUTPUT" > "$OUTPUT"
      first_alpha=0
    else
      tail -n +2 "$TEMP_OUTPUT" >> "$OUTPUT"
    fi
  done

  rm -f "$TEMP_OUTPUT"
done
