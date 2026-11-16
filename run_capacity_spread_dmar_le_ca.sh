#!/usr/bin/env bash
set -euo pipefail

cmake --build build --target dmar_benchmark -j2

INSTANCES="${INSTANCES:-10}"
RUNS="${RUNS:-3}"
MAX_ROUNDS="${MAX_ROUNDS:-5000}"
MAX_RADIUS="${MAX_RADIUS:-8}"

for PROFILE in 1 2 3; do
  case "$PROFILE" in
    1) LABEL="heterogeneous" ;;
    2) LABEL="moderate-spread" ;;
    3) LABEL="high-spread" ;;
  esac

  ./build/dmar_benchmark \
    30 \
    "$INSTANCES" \
    "$RUNS" \
    "$MAX_ROUNDS" \
    "$MAX_RADIUS" \
    0.5 \
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
    > "benchmark-30x30-${LABEL}-5-tasks-per-agent-10-agents-dmar-le-ca.csv"
done
