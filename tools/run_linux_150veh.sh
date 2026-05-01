#!/usr/bin/env bash
# Run the 10 missing 150-vehicle cells inside a Linux Docker container.
# macOS libsystem_malloc hits a 5G-LENA heap-corruption bug at this
# density; Linux glibc is the reference runtime.
# Output goes into the same results_web/full_matrix_<TS>/ directory that
# aggregate_full_matrix.py reads.

set -uo pipefail

# /workspace inside the container is bind-mounted to the host's
# /Users/wsj/Documents/ns3_project tree.
CONTAINER="${CONTAINER:-ns3-dev}"
SIM_BIN="${SIM_BIN:-/workspace/ns-3.46/build/scratch/ns3.46-v2x-5g-sumo-optimized}"
LD_LIB="${LD_LIB:-/workspace/ns-3.46/build/lib}"
TRACE_FILE="${TRACE_FILE:-scratch/manchester_city_ns3.tcl}"
NUM_GNBS="${NUM_GNBS:-3}"
SIM_TIME="${SIM_TIME:-+100s}"
SINR_THRESHOLD_DB="${SINR_THRESHOLD_DB:--5}"
RNG_SEED="${RNG_SEED:-1}"

# Use the most recent matrix dir unless OUT_ROOT is set.
HOST_OUT_ROOT="${OUT_ROOT:-$(ls -td results_web/full_matrix_* 2>/dev/null | head -1)}"
if [[ -z "$HOST_OUT_ROOT" || ! -d "$HOST_OUT_ROOT" ]]; then
  echo "ERROR: no full_matrix_* directory found"
  exit 1
fi
CONTAINER_OUT_ROOT="/workspace/ns-3.46/$HOST_OUT_ROOT"
echo "Filling 150-veh cells in: $HOST_OUT_ROOT"

run_one() {
  local arch="$1" run="$2"
  local label="150veh_${arch}_seed${run}"
  local host_dir="$HOST_OUT_ROOT/$label"
  local container_dir="$CONTAINER_OUT_ROOT/$label"

  if [[ -f "$host_dir/task_counters.csv" ]]; then
    echo "  [$label] already complete, skipping"
    return 0
  fi

  local num_rsus num_buses test_mode
  if [[ "$arch" == "CFN" ]]; then
    num_rsus=2; num_buses=0; test_mode=0
  else
    num_rsus=0; num_buses=2; test_mode=1
  fi

  echo "  [$label] starting in container"
  rm -rf "$host_dir"
  mkdir -p "$host_dir"
  local start=$(date +%s)
  docker exec "$CONTAINER" bash -c "cd /workspace/ns-3.46 && \
    LD_LIBRARY_PATH=$LD_LIB $SIM_BIN \
      --traceFile=$TRACE_FILE \
      --numVehicles=150 \
      --numRsus=$num_rsus \
      --numBuses=$num_buses \
      --numGnbs=$NUM_GNBS \
      --testMode=$test_mode \
      --sinrThresholdDb=$SINR_THRESHOLD_DB \
      --simTime=$SIM_TIME \
      --rngSeed=$RNG_SEED \
      --rngRun=$run \
      --simTag=v2x \
      --outputDir=$container_dir" \
    >"$host_dir/runner.log" 2>&1
  local rc=$?
  local end=$(date +%s)
  if [[ -f "$host_dir/task_counters.csv" ]]; then
    printf "  [$label] OK in %ds\n" $((end-start))
    return 0
  fi
  printf "  [$label] FAILED rc=%d in %ds\n" "$rc" $((end-start))
  return 1
}

failed=()
for arch in CFN VFN; do
  for run in 1 2 3 4 5; do
    if ! run_one "$arch" "$run"; then
      failed+=("150veh_${arch}_seed${run}")
    fi
  done
done

echo ""
if [[ ${#failed[@]} -eq 0 ]]; then
  echo "All 10 cells filled in $HOST_OUT_ROOT"
else
  echo "Cells that failed under Linux: ${failed[*]}"
  exit 2
fi
