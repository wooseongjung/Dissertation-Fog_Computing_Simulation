#!/usr/bin/env bash
# SUMO trace generation for the Manchester city-centre scenario.
#
# Inputs:
#   OSM bounding box for central Manchester
#   Fixed SUMO seed
#   randomTrips parameters
#
# Outputs (in scratch/):
#   manchester_city.osm            raw OSM
#   manchester_city.net.xml        SUMO network
#   manchester_city.trips.xml      trip definitions
#   manchester_city.rou.xml        routed trips
#   manchester_city.fcd.xml        SUMO FCD trace
#   manchester_city_ns3.tcl        NS-2 mobility for ns-3
#   manchester_city_meta.txt       run metadata
#
# Tested with SUMO 1.26.0 on macOS (bash 5.x).
# Runs in roughly 60 s on Apple Silicon.

set -euo pipefail

SUMO_HOME="${SUMO_HOME:-/Library/Frameworks/EclipseSUMO.framework/Versions/Current/EclipseSUMO/share/sumo}"
SUMO_TOOLS="${SUMO_TOOLS:-/opt/homebrew/share/sumo/tools}"

# Manchester city-centre bounding box, ~2.0 km x 1.5 km.
OSM_BBOX_MIN_LON="-2.255"
OSM_BBOX_MIN_LAT="53.470"
OSM_BBOX_MAX_LON="-2.225"
OSM_BBOX_MAX_LAT="53.485"

SUMO_SEED=42
SIM_END=200
TRIPS_END=200
TRIPS_PERIOD=0.20
NS3_TARGET_DURATION=100
MIN_REQUIRED_ALIVE=150

SCRATCH_DIR="${SCRATCH_DIR:-scratch}"
WORK_DIR="${WORK_DIR:-$SCRATCH_DIR/sumo_work}"
OSM_FILE="$SCRATCH_DIR/manchester_city.osm"
NET_FILE="$SCRATCH_DIR/manchester_city.net.xml"
TRIPS_FILE="$SCRATCH_DIR/manchester_city.trips.xml"
ROUTES_FILE="$SCRATCH_DIR/manchester_city.rou.xml"
FCD_FILE="$SCRATCH_DIR/manchester_city.fcd.xml"
RAW_TCL="$WORK_DIR/manchester_city_raw.tcl"
FINAL_TCL="$SCRATCH_DIR/manchester_city_ns3.tcl"
META_FILE="$SCRATCH_DIR/manchester_city_meta.txt"

mkdir -p "$WORK_DIR"

# 1. Download OSM
echo "[1/6] Downloading OSM data for Manchester city centre via Overpass"
if [[ ! -s "$OSM_FILE" ]] || [[ "$(wc -c < "$OSM_FILE")" -lt 10000 ]]; then
  query="[out:xml][timeout:90];(way[\"highway\"](${OSM_BBOX_MIN_LAT},${OSM_BBOX_MIN_LON},${OSM_BBOX_MAX_LAT},${OSM_BBOX_MAX_LON}););(._;>;);out;"
  curl -sS --get \
    --data-urlencode "data=${query}" \
    -o "$OSM_FILE" \
    "https://overpass-api.de/api/interpreter"
  echo "    saved $OSM_FILE ($(wc -c < "$OSM_FILE") bytes)"
  if [[ "$(wc -c < "$OSM_FILE")" -lt 100000 ]]; then
    echo "ERROR: OSM extract is suspiciously small. Contents:"
    head -c 300 "$OSM_FILE"
    exit 1
  fi
else
  echo "    using existing $OSM_FILE ($(wc -c < "$OSM_FILE") bytes)"
fi

# 2. netconvert: OSM -> SUMO network
echo "[2/6] netconvert -> $NET_FILE"
netconvert \
  --osm-files "$OSM_FILE" \
  --output-file "$NET_FILE" \
  --geometry.remove \
  --roundabouts.guess \
  --ramps.guess \
  --junctions.join \
  --tls.guess-signals \
  --tls.discard-simple \
  --tls.join \
  --no-warnings true \
  --no-internal-links true \
  --keep-edges.by-vclass passenger \
  --remove-edges.isolated \
  > "$WORK_DIR/netconvert.log" 2>&1

# 3. randomTrips.py
# (--validate skipped: randomTrips.py 1.26 calls duarouter with implicit
# working-directory paths that fail here; duarouter runs explicitly below.)
echo "[3/6] randomTrips period=$TRIPS_PERIOD end=$TRIPS_END seed=$SUMO_SEED"
/opt/homebrew/bin/python3.13 "$SUMO_TOOLS/randomTrips.py" \
  -n "$NET_FILE" \
  -o "$TRIPS_FILE" \
  -e "$TRIPS_END" \
  --period "$TRIPS_PERIOD" \
  --seed "$SUMO_SEED" \
  --random-departpos \
  --vehicle-class passenger \
  > "$WORK_DIR/randomTrips.log" 2>&1
TRIP_COUNT=$(grep -c "<trip " "$TRIPS_FILE" || true)
echo "    generated $TRIP_COUNT trips"

# 4. duarouter: trips -> routes
echo "[4/6] duarouter -> $ROUTES_FILE"
duarouter \
  --net-file "$NET_FILE" \
  --route-files "$TRIPS_FILE" \
  --output-file "$ROUTES_FILE" \
  --ignore-errors \
  --no-warnings true \
  > "$WORK_DIR/duarouter.log" 2>&1

# 5. SUMO -> FCD
echo "[5/6] sumo (no GUI) producing FCD output"
sumo \
  --net-file "$NET_FILE" \
  --route-files "$ROUTES_FILE" \
  --fcd-output "$FCD_FILE" \
  --begin 0 \
  --end "$SIM_END" \
  --step-length 1.0 \
  --seed "$SUMO_SEED" \
  --no-warnings true \
  > "$WORK_DIR/sumo.log" 2>&1

# 6. traceExporter.py: FCD -> NS-2 .tcl
echo "[6/6] traceExporter -> raw $RAW_TCL"
/opt/homebrew/bin/python3.13 "$SUMO_TOOLS/traceExporter.py" \
  --fcd-input "$FCD_FILE" \
  --ns2mobility-output "$RAW_TCL" \
  --begin 0 \
  --end "$NS3_TARGET_DURATION" \
  > "$WORK_DIR/traceExporter.log" 2>&1

# 7. Keep only vehicles alive throughout [0, NS3_TARGET_DURATION] and renumber
echo "[7/7] selecting continuously-alive vehicles, renumbering 0..N-1"
PYTHONPATH="" RAW_TCL="$RAW_TCL" FINAL_TCL="$FINAL_TCL" \
  TARGET_T="$NS3_TARGET_DURATION" MIN_REQUIRED="$MIN_REQUIRED_ALIVE" \
  /opt/homebrew/bin/python3.13 - <<'PYEOF'
import os
import re
import sys
import collections

raw = os.environ["RAW_TCL"]
final = os.environ["FINAL_TCL"]
target_T = float(os.environ["TARGET_T"])
min_required = int(os.environ["MIN_REQUIRED"])

# Parse the raw .tcl: per-vehicle initial positions plus setdest events.
init_x = {}
init_y = {}
events = collections.defaultdict(list)

re_init  = re.compile(r'\$node_\((\d+)\)\s+set\s+(X|Y|Z)_\s+([-\d\.]+)')
re_event = re.compile(r'\$ns_\s+at\s+([\d\.]+)\s+"\$node_\((\d+)\)\s+setdest\s+([-\d\.]+)\s+([-\d\.]+)\s+([-\d\.]+)')

with open(raw) as f:
    for line in f:
        m = re_init.search(line)
        if m:
            vid = int(m.group(1)); axis = m.group(2); val = float(m.group(3))
            if axis == 'X':
                init_x[vid] = val
            elif axis == 'Y':
                init_y[vid] = val
            continue
        m = re_event.search(line)
        if m:
            t = float(m.group(1)); vid = int(m.group(2))
            x = float(m.group(3)); y = float(m.group(4)); s = float(m.group(5))
            events[vid].append((t, x, y, s))

# A vehicle is alive across [0, T] if it has an initial X/Y entry and its
# last setdest event time is >= T - eps.
EPS = 1.0
candidates = []
for vid in sorted(set(init_x) & set(init_y)):
    if not events.get(vid):
        candidates.append(vid)
        continue
    last_t = max(t for t, *_ in events[vid])
    if last_t >= target_T - EPS:
        candidates.append(vid)

selected = candidates
if len(selected) < min_required:
    sys.exit(f"ERROR: only {len(selected)} continuously-alive vehicles, need {min_required}. "
             "Increase --period or extend OSM bbox.")

N = min(len(selected), min_required + 100)
selected = selected[:N]
print(f"    candidates continuously alive in [0,{target_T}]: {len(candidates)}; emitting N={N}")

remap = {old: new for new, old in enumerate(selected)}

with open(final, 'w') as f:
    for old in selected:
        new = remap[old]
        f.write(f"$node_({new}) set X_ {init_x[old]:.6f}\n")
        f.write(f"$node_({new}) set Y_ {init_y[old]:.6f}\n")
        f.write(f"$node_({new}) set Z_ 0.000000\n")
    all_events = []
    for old in selected:
        new = remap[old]
        for t, x, y, s in events.get(old, []):
            all_events.append((t, new, x, y, s))
    all_events.sort()
    for t, new, x, y, s in all_events:
        f.write(f'$ns_ at {t:.2f} "$node_({new}) setdest {x:.2f} {y:.2f} {s:.2f}"\n')

print(f"    wrote {final} with {N} vehicles, IDs 0..{N-1} contiguous")
PYEOF

# Metadata
cat > "$META_FILE" <<META
SUMO pipeline metadata for $FINAL_TCL
=======================================
Generated: $(date -u +"%Y-%m-%dT%H:%M:%SZ")
SUMO version: $(sumo --version 2>/dev/null | grep -i "Eclipse SUMO" | head -1 || echo "(could not detect)")
Pipeline script: tools/sumo_pipeline.sh

OSM bounding box (lonMin, latMin, lonMax, latMax):
  $OSM_BBOX_MIN_LON, $OSM_BBOX_MIN_LAT, $OSM_BBOX_MAX_LON, $OSM_BBOX_MAX_LAT

Random seed (netconvert/randomTrips/sumo): $SUMO_SEED
randomTrips parameters: period=$TRIPS_PERIOD end=$TRIPS_END --random-departpos
  (--validate not passed; duarouter runs explicitly in step 4.)
duarouter: --ignore-errors --no-warnings true
sumo: --step-length 1.0 --begin 0 --end $SIM_END --seed $SUMO_SEED
traceExporter: NS-2 mobility output, begin=0 end=$NS3_TARGET_DURATION
Post-processing: kept only vehicles continuously alive in [0, $NS3_TARGET_DURATION] s, renumbered to contiguous IDs 0..N-1.

Trips generated: $TRIP_COUNT
Final vehicle count in $FINAL_TCL:
$(grep -c "set X_" "$FINAL_TCL" 2>/dev/null || echo "?") vehicles
META

echo "Pipeline complete: $FINAL_TCL"
echo "Metadata: $META_FILE"
