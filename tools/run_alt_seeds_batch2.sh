#!/usr/bin/env bash
# Second batch of substream replacements for 150-veh VFN cells that hit
# the 5G-LENA heap-corruption bug. Tries rngRun 9..15 in order and stops
# once seeds {1, 2, 5} are filled.

set -uo pipefail

CONTAINER="${CONTAINER:-ns3-dev}"
SIM_BIN="${SIM_BIN:-/workspace/ns-3.46/build/scratch/ns3.46-v2x-5g-sumo-optimized}"
LD_LIB="${LD_LIB:-/workspace/ns-3.46/build/lib}"
TRACE_FILE="${TRACE_FILE:-scratch/manchester_city_ns3.tcl}"
SINR_THRESHOLD_DB="${SINR_THRESHOLD_DB:--5}"
RNG_SEED="${RNG_SEED:-1}"

HOST_OUT_ROOT="${OUT_ROOT:-$(ls -td results_web/full_matrix_* 2>/dev/null | head -1)}"
CONTAINER_OUT_ROOT="/workspace/ns-3.46/$HOST_OUT_ROOT"

target_slots=(1 2 5)
attempt_runs=(9 10 11 12 13 14 15)

slot_idx=0
for alt in "${attempt_runs[@]}"; do
  if [[ $slot_idx -ge ${#target_slots[@]} ]]; then
    echo "All slots filled; stopping at rngRun=$alt"
    break
  fi
  orig=${target_slots[$slot_idx]}
  label="150veh_VFN_seed${orig}"
  host_dir="$HOST_OUT_ROOT/$label"
  container_dir="$CONTAINER_OUT_ROOT/$label"

  if [[ -f "$host_dir/task_counters.csv" ]]; then
    slot_idx=$((slot_idx+1))
    continue
  fi

  echo "  [$label slot] trying rngRun=$alt"
  rm -rf "$host_dir"
  mkdir -p "$host_dir"
  start=$(date +%s)
  docker exec "$CONTAINER" bash -c "cd /workspace/ns-3.46 && \
    LD_LIBRARY_PATH=$LD_LIB $SIM_BIN \
      --traceFile=$TRACE_FILE \
      --numVehicles=150 --numRsus=0 --numBuses=2 --numGnbs=3 \
      --testMode=1 \
      --sinrThresholdDb=$SINR_THRESHOLD_DB \
      --simTime=+100s \
      --rngSeed=$RNG_SEED --rngRun=$alt \
      --simTag=v2x \
      --outputDir=$container_dir" \
    >"$host_dir/runner.log" 2>&1
  rc=$?
  end=$(date +%s)
  if [[ -f "$host_dir/task_counters.csv" ]]; then
    echo "rngRun_used=$alt" > "$host_dir/replacement_seed.txt"
    echo "rngRun_failed_originals=$orig" >> "$host_dir/replacement_seed.txt"
    printf "  [$label] OK in %ds (rngRun=%d)\n" $((end-start)) "$alt"
    slot_idx=$((slot_idx+1))
  else
    printf "  [$label] FAILED rc=%d in %ds (rngRun=%d)\n" "$rc" $((end-start)) "$alt"
  fi
done

still_missing=()
for orig in "${target_slots[@]}"; do
  d="$HOST_OUT_ROOT/150veh_VFN_seed${orig}"
  [[ -f "$d/task_counters.csv" ]] || still_missing+=("seed${orig}")
done

echo ""
if [[ ${#still_missing[@]} -eq 0 ]]; then
  echo "All 3 missing VFN cells now filled."
else
  echo "Still missing: ${still_missing[*]}"
  echo "Partial coverage n=$((5 - ${#still_missing[@]})) for 150-veh VFN."
  exit 2
fi
