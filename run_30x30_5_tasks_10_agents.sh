#!/usr/bin/env bash
set -euo pipefail

cmake --build build --target dmar_benchmark -j2

INSTANCES="${INSTANCES:-2}"
RUNS="${RUNS:-1}"
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
  > benchmark-30x30-5-tasks-per-agent-10-agents.csv
