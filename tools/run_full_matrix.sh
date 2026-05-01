#!/usr/bin/env bash
set -euo pipefail

# Full dissertation matrix.
# sinrThresholdDb=-5 dB, 2 fog nodes, simTime=100 s.
# 50/100/150 vehicles x CFN/VFN x 5 substreams (rngSeed=1, rngRun=1..5).
# 30 runs total, around 60 min on Apple Silicon.

SIM_BIN="${SIM_BIN:-./build-mac-out/scratch/ns3.46-v2x-5g-sumo-default}"
OUT_ROOT="${OUT_ROOT:-results_web/full_matrix_$(date +%Y%m%d%H%M%S)}"
TRACE_FILE="${TRACE_FILE:-scratch/manchester_city_ns3.tcl}"
NUM_GNBS="${NUM_GNBS:-3}"
SIM_TIME="${SIM_TIME:-+100s}"
SINR_THRESHOLD_DB="${SINR_THRESHOLD_DB:--5}"
RNG_SEED="${RNG_SEED:-1}"

densities=(50 100 150)
seeds=(1 2 3 4 5)
architectures=(CFN VFN)

mkdir -p "$OUT_ROOT"

total=$((${#densities[@]} * ${#architectures[@]} * ${#seeds[@]}))
n=0
for veh in "${densities[@]}"; do
  for arch in "${architectures[@]}"; do
    for run in "${seeds[@]}"; do
      n=$((n+1))
      if [[ "$arch" == "CFN" ]]; then
        num_rsus=2
        num_buses=0
        test_mode=0
      else
        num_rsus=0
        num_buses=2
        test_mode=1
      fi

      out_dir="$OUT_ROOT/${veh}veh_${arch}_seed${run}"
      mkdir -p "$out_dir"
      echo "[$n/$total] $arch / $veh veh / seed=$RNG_SEED run=$run / thr=${SINR_THRESHOLD_DB} dB"

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
        >"$out_dir/runner.log" 2>&1
    done
  done
done

echo "Full-matrix sweep complete: $OUT_ROOT"
