# Dissertation — Fog Computing Simulation

Simulation source code, mobility traces, and analysis scripts for the BEng Electronic Engineering dissertation **"Comparative Analysis of Vehicular and Cellular Fog Computing Architectures over 5G NR in Urban Environments"** (University of Manchester, 2025/26).

---

## 1. Introduction

This repository contains the discrete-event simulation framework used to compare **Cellular Fog Nodes (CFN)** mounted on roadside units with **Vehicular Fog Nodes (VFN)** mounted on transit buses, under matched 5G New Radio (NR) conditions. The simulation couples **ns-3.46** (network stack), the **5G-LENA NR module** (5G PHY/MAC), and **SUMO** (microscopic vehicular mobility) to measure per-task offloading reliability across 6 scenarios: 3 vehicle densities (50, 100, 150) × 2 architectures (CFN, VFN), each repeated with 5 random-number substreams for 95% Student-*t* confidence intervals.

The repository provides everything needed to (a) regenerate the Manchester city-centre SUMO mobility trace, (b) re-run the full 30-cell simulation matrix, and (c) reproduce the tables and figures of Section 4 of the dissertation.

---

## 2. Contextual overview

```
                      ┌────────────────────────────────────────┐
                      │  EPC / Core Network (SGW, PGW, MME)    │
                      └────────────┬─────────┬─────────────────┘
                                   │ S1-U    │ S1-U
                            ┌──────┴──┐    ┌─┴────┐
                            │  gNB 1  │ X2 │ gNB 2 │  ...  (3 gNBs)
                            └────┬────┘    └──┬───┘
                                 │ 5G NR Uu   │
                  ┌──────────────┴────────────┴──────────────┐
                  │                                          │
            ┌─────▼─────┐                              ┌─────▼─────┐
            │ CFN (RSU) │      OR (per-scenario)      │ VFN (bus) │
            │  fixed    │                              │  mobile   │
            └─────┬─────┘                              └─────┬─────┘
                  │ Task uplink / Response                  │
                  │                                          │
            ┌─────▼──────────────────────────────────────────▼─────┐
            │  Vehicle UEs (50 / 100 / 150) — SUMO mobility trace  │
            └──────────────────────────────────────────────────────┘
```

A higher-resolution rendering of the same diagram is at [`docs/system_topology.png`](docs/system_topology.png). The same diagram appears as Fig. 1 in the dissertation.

The simulation runtime classifies every task into one of four mutually-exclusive outcomes — **success**, **no-association** (no fog above the SNR floor), **pre-fog loss** (HARQ exhausted on the uplink), or **deadline miss** — and records the result to a per-cell CSV. The aggregator then computes per-stage reliability rates with 95% confidence intervals across substreams.

---

## 3. Installation

The simulation builds and runs inside a stock ns-3.46 tree with the 5G-LENA NR module present.

### Required software

| Tool | Tested version | Notes |
|---|---|---|
| Operating system | macOS 14, Ubuntu 22.04 / Debian 12 | both verified |
| C++ toolchain | clang 15+ or g++ 11+ | C++17 |
| ns-3 | **3.46** | fresh checkout from [nsnam.org](https://www.nsnam.org/) |
| 5G-LENA NR module | **NR-v4.1.1** | placed in `contrib/nr/` of the ns-3 tree |
| Python | 3.11+ | for SUMO pipeline + analysis scripts |
| SUMO | 1.20+ | `netconvert`, `randomTrips.py`, `duarouter`, `traceExporter.py` on `$PATH` |
| GNU Make + CMake | latest | ns-3 build dependencies |

Python packages required by the analysis scripts:

```bash
pip install pandas numpy scipy matplotlib
```

### Drop-in installation into ns-3.46

```bash
# 1. Get a clean ns-3.46 with the 5G-LENA NR module
git clone https://gitlab.com/nsnam/ns-3-dev.git ns-3.46          # checkout v3.46
cd ns-3.46
git clone https://gitlab.com/cttc-lena/nr.git contrib/nr         # checkout v4.1.1

# 2. Drop the simulation source and tools into the ns-3 tree
cp -r /path/to/Dissertation-Fog_Computing_Simulation/scratch/*  scratch/
cp -r /path/to/Dissertation-Fog_Computing_Simulation/tools/    .
cp /path/to/Dissertation-Fog_Computing_Simulation/traces/*.tcl  scratch/
cp /path/to/Dissertation-Fog_Computing_Simulation/traces/*.osm  scratch/

# 3. Configure and build
./ns3 configure --enable-examples --enable-modules=nr
./ns3 build
```

---

## 4. How to run

### Regenerate the SUMO trace from OpenStreetMap

```bash
bash tools/sumo_pipeline.sh
# produces scratch/manchester_city_ns3.tcl (250 vehicles continuously alive over 0–100 s)
```

### Single-cell smoke test (50 vehicles, CFN, single seed)

```bash
./build-mac-out/scratch/ns3.46-v2x-5g-sumo-default \
    --traceFile=scratch/manchester_city_ns3.tcl \
    --numVehicles=50 --numRsus=2 --numBuses=0 --numGnbs=3 \
    --testMode=0 --sinrThresholdDb=-5 \
    --simTime=+100s --rngSeed=1 --rngRun=1 \
    --simTag=v2x --outputDir=results/smoke_test
```

### Full 30-cell experimental matrix (3 densities × 2 architectures × 5 seeds, ~60 min)

```bash
bash tools/run_full_matrix.sh
# output to results_web/full_matrix_<timestamp>/
```

### Substream replacement for the 150-vehicle VFN cell (5G-LENA stability bug)

```bash
bash tools/run_alt_seeds.sh           # batch 1: rngRun ∈ {6,7,8}
bash tools/run_alt_seeds_batch2.sh    # batch 2: rngRun ∈ {9,10,11,12,13}
```

### Aggregate results and produce dissertation figures

```bash
python tools/aggregate_full_matrix.py results_web/full_matrix_<timestamp>/
python tools/plot_aggregated.py       results_web/full_matrix_<timestamp>/
python tools/plot_velocity_v2.py      results_web/full_matrix_<timestamp>/
```

### Calibration sweep (Appendix F of the dissertation)

```bash
bash tools/run_v2x_calibration.sh
python tools/analyze_calibration.py
```

---

## 5. Technical details

### Key parameters (locked unless noted)

| Quantity | Value |
|---|---|
| Carrier frequency | 28 GHz (mmWave) |
| Bandwidth | 100 MHz |
| Numerology μ | 2 (60 kHz subcarrier spacing, 0.25 ms slot) |
| Channel model | 3GPP UMi-LOS, shadowing disabled |
| MAC scheduler | TDMA round-robin, HARQ enabled |
| gNB TX / antenna | 40 dBm, 2×2 array, 10 m height |
| Fog TX / antenna | 23 dBm (CFN and VFN identical), 3 m / 2 m |
| Handover | A3-RSRP, 1.5 dB hyst, 128 ms TTT |
| Vehicle UEs | 50 / 100 / 150 (sub-sampled from a 250-vehicle pool) |
| Fog nodes | 2 per scenario (single architecture per scenario) |
| Task model | Poisson λ = 2 /s, 800 B UDP, 100 ms deadline |
| Fog service | 5 ms deterministic, FIFO single server, queue ≤ 50 |
| Sim time | 100 s per cell, 5 substreams per cell |

### SNR-gated fog selection (the only algorithmic decision in the simulation)

Each vehicle re-evaluates its fog choice every Δ_r = 500 ms using a free-space SNR scoring function:

```
SNR_dB = P_tx − 20·log10(4πd/λ_c) − P_noise
P_noise = −174 + 10·log10(B) + NF
```

with P_tx = 23 dBm, B = 100 MHz, NF = 7 dB. Candidates whose SNR < −5 dB or whose queue exceeds 50 are rejected. The vehicle associates to arg-max over surviving candidates. The free-space form is a deliberately lightweight proxy for the cell-selection decision; the actual radio link uses the full 3GPP UMi-LOS PHY/MAC under 5G-LENA, with HARQ retransmissions, scheduling contention, and beamforming alignment computed per packet.

### Per-task reliability decomposition

Every Poisson tick at every vehicle is classified into exactly one of four buckets:

- **Success** — UDP ACK received within 100 ms
- **No-association** — no candidate fog above the SNR floor at the moment of the tick
- **Pre-fog loss** — task was sent over NR but uplink HARQ exhausted before reaching the fog
- **Deadline miss** — task reached the fog and was acknowledged, but the round trip exceeded 100 ms

Pipeline percentages sum to 100% within rounding error.

### Output schema

Each run writes the following CSVs to `--outputDir`:

| File | Content |
|---|---|
| `results_tasks.csv` | per-task event log (one row per Poisson tick) |
| `results_reliability.csv` | aggregated per-stage rates with CIs |
| `results_methodology_a.csv` | Tx / Rx / Dropped events per vehicle |
| `results_handover.csv` | A3-RSRP handover events |
| `results_response_path.csv` | downlink ACK delivery events |
| `results_downlink_radio.csv` | RLC-layer downlink events |
| `assoc_diagnostics.csv` | per-association SNR scores and selection decisions |
| `replacement_seed.txt` | (only on substream re-runs) the rngRun used and which originals it replaces |

---

## 6. Repository layout

```
Dissertation-Fog_Computing_Simulation/
├── README.md                          this file
├── docs/
│   └── system_topology.png            contextual overview (= Fig. 1)
├── scratch/
│   └── v2x-5g-sumo.cc                 main ns-3 scenario (~2,750 LoC C++17)
├── tools/
│   ├── sumo_pipeline.sh               OSM → SUMO → NS-2 trace generator
│   ├── run_full_matrix.sh             30-cell experimental runner
│   ├── run_alt_seeds.sh               substream replacement (batch 1)
│   ├── run_alt_seeds_batch2.sh        substream replacement (batch 2)
│   ├── run_missing_cells.sh           re-run helper for partial matrices
│   ├── run_linux_150veh.sh            Docker-targeted 150-veh runner
│   ├── run_v2x_calibration.sh         12-cell calibration sweep
│   ├── aggregate_full_matrix.py       CSV aggregator (mean ± 95% CI)
│   ├── analyze_calibration.py         calibration-sweep analyser
│   ├── plot_aggregated.py             dissertation-quality figures
│   └── plot_velocity_v2.py            velocity-binned reliability figure
└── traces/
    ├── manchester_city.osm            OSM extract from Overpass API
    ├── manchester_city_ns3.tcl        post-processed NS-2 mobility trace
    └── manchester_city_meta.txt       SUMO pipeline metadata
```

---

## 7. Known issues and future improvements

- **5G-LENA UE-uplink stability** — under sustained high event load (150 vehicles, VFN), the 5G-LENA NR module triggers a non-deterministic heap-corruption fault inside its UE-uplink path. The bug is independent of this work and surfaces on both macOS `libsystem_malloc` and Linux `glibc` with identical stack traces. The 150-vehicle VFN cell uses 4 substreams instead of 5 after one substream slot could not be filled even after eight further `rngRun` attempts. The full audit trail is in `replacement_seed.txt` files inside each cell's output directory.
- **Two-path-loss disclosure** — the SNR-gated fog selection uses Friis free-space path loss as a lightweight proxy for the selection decision, while the actual NR PHY/MAC uses the 3GPP UMi-LOS curve. This is documented in §3.6.2 of the dissertation.
- **Channel idealisation** — shadow fading is disabled and LOS is forced for the entire simulation. NLOS state transitions and the `BuildingsHelper` module are not enabled. A more realistic channel model is identified as future work.
- **Sidelink (PC5) is not used** — direct V2V via NR Sidelink is not supported by the version of 5G-LENA employed. The cellular-Uu routing assumption is conservative for VFN performance.
- **Single SUMO trace** — all scenarios use one Manchester city-centre trace generated with random seed 42. A multi-trace robustness study across other urban morphologies is identified as future work.
- **Adaptive fog-node placement** — a control loop that periodically repositions CFN candidates based on observed demand could close the static-deployment gap by design but is not implemented.

---

## 8. Third-party code and licences

- **ns-3** (GNU GPLv2) — used unmodified at v3.46 from [nsnam.org](https://www.nsnam.org/).
- **5G-LENA NR module** (GNU GPLv2) — used unmodified at v4.1.1 from [CTTC GitLab](https://gitlab.com/cttc-lena/nr).
- **SUMO** (Eclipse Public License 2.0) — invoked via the `tools/sumo_pipeline.sh` shell script.
- **OpenStreetMap data** — © OpenStreetMap contributors, available under the Open Database License (ODbL).

All code under `scratch/` and `tools/` in this repository is the author's own original work, written for this dissertation. Where short snippets were adapted from upstream ns-3 / 5G-LENA examples, the adaptation is identified at the corresponding source-code site by an in-line comment naming the upstream file.

---

## Author and citation

**Wooseong Jung**
BEng Electronic Engineering, University of Manchester, 2025/26
Supervisor: [Project Supervisor Name]

If you use this code, please cite the dissertation:

```
W. Jung, "Comparative Analysis of Vehicular and Cellular Fog Computing
Architectures over 5G NR in Urban Environments," BEng Dissertation,
University of Manchester, 2026.
```
