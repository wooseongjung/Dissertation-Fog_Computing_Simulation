#!/usr/bin/env bash
# Resume an interrupted run_full_matrix.sh sweep.
# Walks the existing full_matrix directory and re-runs every cell whose
# task_counters.csv is missing. Up to 2 attempts per cell.

set -uo pipefail

SIM_BIN="${SIM_BIN:-./build-mac-out/scratch/ns3.46-v2x-5g-sumo-default}"
TRACE_FILE="${TRACE_FILE:-scratch/manchester_city_ns3.tcl}"
NUM_GNBS="${NUM_GNBS:-3}"
SIM_TIME="${SIM_TIME:-+100s}"
SINR_THRESHOLD_DB="${SINR_THRESHOLD_DB:--5}"
RNG_SEED="${RNG_SEED:-1}"

OUT_ROOT="${OUT_ROOT:-$(ls -td results_web/full_matrix_* 2>/dev/null | head -1)}"
if [[ -z "$OUT_ROOT" || ! -d "$OUT_ROOT" ]]; then
  echo "ERROR: no full_matrix_* directory found"
  exit 1
fi
echo "Resuming matrix in: $OUT_ROOT"

densities=(50 100 150)
seeds=(1 2 3 4 5)
architectures=(CFN VFN)

retry_cell() {
  local veh="$1" arch="$2" run="$3"
  local out_dir="$OUT_ROOT/${veh}veh_${arch}_seed${run}"
  if [[ -f "$out_dir/task_counters.csv" ]]; then
    return 0
  fi
  local num_rsus num_buses test_mode
  if [[ "$arch" == "CFN" ]]; then
    num_rsus=2; num_buses=0; test_mode=0
  else
    num_rsus=0; num_buses=2; test_mode=1
  fi

  mkdir -p "$out_dir"
  for attempt in 1 2; do
    echo "  [retry attempt $attempt] $veh veh / $arch / run=$run"
    # Keep each attempt's log so a persistent failure leaves a trail.
    log_file="$out_dir/runner_attempt${attempt}.log"
    "$SIM_BIN" \
      --traceFile="$TRACE_FILE" \
      --numVehicles="$veh" \
      --numRsus="$num_rsus" \
      --numBuses="$num_buses" \
      --numGnbs="$NUM_GNBS" \
      --testMode="$test_mode" \
      --sinrThresholdDb="$SINR_THRESHOLD_DB" \
      --simTime="$SIM_TIME" \
      --rngSeed="$RNG_SEED" \
      --rngRun="$run" \
      --simTag=v2x \
      --outputDir="$out_dir" \
      >"$log_file" 2>&1
    cp "$log_file" "$out_dir/runner.log"
    if [[ -f "$out_dir/task_counters.csv" ]]; then
      echo "    ok"
      return 0
    fi
    echo "    failed (log preserved as $log_file), will retry"
  done
  echo "    PERSISTENT FAILURE for $veh / $arch / run=$run"
  return 1
}

failed=()
for veh in "${densities[@]}"; do
  for arch in "${architectures[@]}"; do
    for run in "${seeds[@]}"; do
      if ! retry_cell "$veh" "$arch" "$run"; then
        failed+=("${veh}veh_${arch}_seed${run}")
      fi
    done
  done
done

echo ""
if [[ ${#failed[@]} -eq 0 ]]; then
  echo "All cells present and accounted for in $OUT_ROOT"
else
  echo "Cells that still fail after 2 attempts: ${failed[*]}"
  exit 2
fi
