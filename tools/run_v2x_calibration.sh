#!/usr/bin/env bash
set -euo pipefail

# Calibration sweep run before the full matrix.
# 50 vehicles, 50 s, CFN/VFN, SNR thresholds {0, -5, -10} dB,
# fog counts {2, 4} -> 12 runs total.

SIM_BIN="${SIM_BIN:-./build-mac-out/scratch/ns3.46-v2x-5g-sumo-default}"
OUT_ROOT="${OUT_ROOT:-results_web/calibration_$(date +%Y%m%d%H%M%S)}"
TRACE_FILE="${TRACE_FILE:-scratch/manchester_city_ns3.tcl}"
NUM_VEHICLES="${NUM_VEHICLES:-50}"
SIM_TIME="${SIM_TIME:-+50s}"
RNG_SEED="${RNG_SEED:-1}"
RNG_RUN="${RNG_RUN:-1}"
NUM_GNBS="${NUM_GNBS:-3}"

thresholds=(0 -5 -10)
fog_counts=(2 4)
architectures=(CFN VFN)

mkdir -p "$OUT_ROOT"

for arch in "${architectures[@]}"; do
  for fogs in "${fog_counts[@]}"; do
    for threshold in "${thresholds[@]}"; do
      if [[ "$arch" == "CFN" ]]; then
        num_rsus="$fogs"
        num_buses=0
        test_mode=0
      else
        num_rsus=0
        num_buses="$fogs"
        test_mode=1
      fi

      safe_threshold="${threshold/-/m}"
      out_dir="$OUT_ROOT/${arch}_${fogs}fog_${safe_threshold}dB"
      mkdir -p "$out_dir"

      "$SIM_BIN" \
        --traceFile="$TRACE_FILE" \
        --numVehicles="$NUM_VEHICLES" \
        --numRsus="$num_rsus" \
        --numBuses="$num_buses" \
        --numGnbs="$NUM_GNBS" \
        --testMode="$test_mode" \
        --sinrThresholdDb="$threshold" \
        --simTime="$SIM_TIME" \
        --rngSeed="$RNG_SEED" \
        --rngRun="$RNG_RUN" \
        --simTag=v2x \
        --outputDir="$out_dir" \
        >"$out_dir/runner.log" 2>&1
    done
  done
done

echo "Calibration complete: $OUT_ROOT"
