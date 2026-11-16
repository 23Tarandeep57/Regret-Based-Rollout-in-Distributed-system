#!/usr/bin/env bash
set -euo pipefail

cmake --build build --target dmar_benchmark -j2

INSTANCES="${INSTANCES:-10}"
RUNS="${RUNS:-3}"
MAX_ROUNDS="${MAX_ROUNDS:-5000}"
MAX_RADIUS="${MAX_RADIUS:-8}"

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
  0 \
  5 \
  30 \
  10 \
  1 \
  > benchmark-30x30-heterogeneous-5-tasks-per-agent-10-agents.csv
