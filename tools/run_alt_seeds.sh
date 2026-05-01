#!/usr/bin/env bash
# Re-run 150-veh VFN cells that hit the 5G-LENA UE-uplink heap-corruption
# bug. rngRun selects an RNG substream; trying alternative substreams
# typically dodges the (ordering-dependent) bug. Failed seeds {1, 2, 5}
# are replaced by {6, 7, 8} from the same seed=1 pool.

set -uo pipefail

CONTAINER="${CONTAINER:-ns3-dev}"
SIM_BIN="${SIM_BIN:-/workspace/ns-3.46/build/scratch/ns3.46-v2x-5g-sumo-optimized}"
LD_LIB="${LD_LIB:-/workspace/ns-3.46/build/lib}"
TRACE_FILE="${TRACE_FILE:-scratch/manchester_city_ns3.tcl}"
SINR_THRESHOLD_DB="${SINR_THRESHOLD_DB:--5}"
RNG_SEED="${RNG_SEED:-1}"

HOST_OUT_ROOT="${OUT_ROOT:-$(ls -td results_web/full_matrix_* 2>/dev/null | head -1)}"
if [[ -z "$HOST_OUT_ROOT" || ! -d "$HOST_OUT_ROOT" ]]; then
  echo "ERROR: no full_matrix_* directory found"
  exit 1
fi
CONTAINER_OUT_ROOT="/workspace/ns-3.46/$HOST_OUT_ROOT"

# failed_seed -> replacement rngRun. We keep the cell label with the
# original failed seed so aggregation stays unified, and write the
# substream actually used into the cell directory.
declare -A REPLACEMENTS=(
  [1]=6
  [2]=7
  [5]=8
)

failed=()
for orig in 1 2 5; do
  alt=${REPLACEMENTS[$orig]}
  label="150veh_VFN_seed${orig}"
  host_dir="$HOST_OUT_ROOT/$label"
  container_dir="$CONTAINER_OUT_ROOT/$label"

  echo "  [$label] retrying with rngRun=$alt (seed $orig hit the heap bug)"
  rm -rf "$host_dir"
  mkdir -p "$host_dir"
  start=$(date +%s)
  docker exec "$CONTAINER" bash -c "cd /workspace/ns-3.46 && \
    LD_LIBRARY_PATH=$LD_LIB $SIM_BIN \
      --traceFile=$TRACE_FILE \
      --numVehicles=150 \
      --numRsus=0 --numBuses=2 --numGnbs=3 \
      --testMode=1 \
      --sinrThresholdDb=$SINR_THRESHOLD_DB \
      --simTime=+100s \
      --rngSeed=$RNG_SEED \
      --rngRun=$alt \
      --simTag=v2x \
      --outputDir=$container_dir" \
    >"$host_dir/runner.log" 2>&1
  rc=$?
  end=$(date +%s)
  if [[ -f "$host_dir/task_counters.csv" ]]; then
    echo "rngRun_used=$alt" > "$host_dir/replacement_seed.txt"
    echo "rngRun_failed_originals=$orig" >> "$host_dir/replacement_seed.txt"
    printf "  [$label] OK in %ds (rngRun=%d)\n" $((end-start)) "$alt"
  else
    printf "  [$label] FAILED rc=%d in %ds (rngRun=%d, %s)\n" \
      "$rc" $((end-start)) "$alt" "$(tail -1 $host_dir/runner.log | head -c 60)"
    failed+=("$label")
  fi
done

echo ""
if [[ ${#failed[@]} -eq 0 ]]; then
  echo "All replacement seeds succeeded"
else
  echo "Cells still missing: ${failed[*]}"
  exit 2
fi
