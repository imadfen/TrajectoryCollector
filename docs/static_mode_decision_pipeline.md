# Static Mode Decision Pipeline — Implementation Plan

## Overview

This document describes **every change** that must be made to the OMNeT++ C++ simulation
so that it implements the Static Mode "Run 2" flow defined in
[docs/omnetpp_integration.md](omnetpp_integration.md) (§ 3, Step 4).

When the simulation starts, [DataCollectorApp](file:///home/massi/Documents/omnetpp-5.6.2/samples/TrajectoryCollector/src/trajectories/DataCollectorApp.h#24-49) will:

1. Check whether a `decisions.json` file exists in the output directory.
2. If **found** → enter **Decision Mode**: beacon interval and MAC backoff are read from
   the JSON file for each vehicle at every timestep instead of using the flat `beaconInterval`
   parameter.
3. If **not found** → fall back to the current **Baseline Mode** (flat 10 Hz beaconing,
   no changes).

---

## Architecture Diagram

```
omnetpp.ini
  └─ decisions_file = "decisions.json"   ← new INI parameter

DataCollectorApp::initialize()
  └─ DecisionLoader::load("decisions.json")
       ├─ file found?  → parse JSON, fill decision map
       └─ file missing? → leave map empty (baseline mode)

DataCollectorApp::handleSelfMsg()  (beacon timer fires)
  ├─ [Decision Mode]  lookup (vehicleId, simTime) → beacon_hz, mac_wait_ms
  │     ├─ apply beacon_hz  → scheduleAt(simTime + 1/beacon_hz, sendBeaconEvt)
  │     └─ apply mac_wait   → par("contentionWindow") = slots
  └─ [Baseline Mode]  scheduleAt(simTime + beaconIntervalVal, sendBeaconEvt)
```

---

## Files to Create

### 1. `src/trajectories/DecisionLoader.h`  *(new)*

**Purpose:** Header-only struct + class declaration for loading and querying `decisions.json`.

**Contents:**
- `struct Decision { double beacon_hz; double mac_wait_ms; };`
- `class DecisionLoader` with:
  - `bool load(const std::string& path)` — parses JSON, returns `true` on success
  - `bool hasDecisions() const`
  - `Decision lookup(const std::string& vehicleId, double simTimeSec) const`
  - Internal map: `std::map<std::string, std::map<double, Decision>>`

**Key design choices:**
- Uses only the C++ standard library (`<fstream>`, `<sstream>`, `<map>`, `<string>`) and a
  minimal hand-written JSON parser — **no external dependencies** (avoids build-system changes).
- Timestep keys in `decisions.json` are strings like `"18003.1"`; they are parsed as `double`
  for numeric nearest-key lookup.
- If no exact timestep match is found, the loader performs a **floor lookup** (use the last
  decision at or before the current sim-time).

---

### 2. `src/trajectories/DecisionLoader.cc`  *(new)*

**Purpose:** Implementation of `DecisionLoader::load()` and `DecisionLoader::lookup()`.

**Parsing strategy (no external JSON lib):**
- Read the file line-by-line.
- Detect `"car_XX"` keys with a regex-free string search.
- Detect `"TTTTT.T"` sub-keys and `beacon_hz` / `mac_wait_ms` values.
- Populate the nested map.

---

## Files to Modify

### 3. [src/trajectories/DataCollectorApp.h](file:///home/massi/Documents/omnetpp-5.6.2/samples/TrajectoryCollector/src/trajectories/DataCollectorApp.h)

| What | Change |
|---|---|
| Add include | `#include "DecisionLoader.h"` |
| Add member | `DecisionLoader decisionLoader;` |
| Add member | `bool decisionModeActive;` |
| Add member | `std::string myVehicleId;` (SUMO vehicle ID string, e.g. `"car_32"`) |

---

### 4. [src/trajectories/DataCollectorApp.cc](file:///home/massi/Documents/omnetpp-5.6.2/samples/TrajectoryCollector/src/trajectories/DataCollectorApp.cc)

#### [initialize()](file:///home/massi/Documents/omnetpp-5.6.2/samples/TrajectoryCollector/src/trajectories/DataCollectorApp.cc#7-38) — stage 0

```cpp
// Read the decisions_file INI parameter
std::string decisionsPath = par("decisionsFile").stringValue();
decisionModeActive = decisionLoader.load(decisionsPath);
if (decisionModeActive)
    EV << "[DecisionMode] Loaded decisions from " << decisionsPath << endl;
else
    EV << "[BaselineMode] No decisions file found at " << decisionsPath << endl;

// Resolve SUMO vehicle id (node index → "car_<index>")
myVehicleId = "car_" + std::to_string(getParentModule()->getIndex());
```

#### [handleSelfMsg()](file:///home/massi/Documents/omnetpp-5.6.2/samples/TrajectoryCollector/src/trajectories/DataCollectorApp.cc#39-69) — inside the `if (msg == sendBeaconEvt)` branch

Replace the fixed `scheduleAt(simTime() + beaconIntervalVal, sendBeaconEvt)` at the end with:

```cpp
if (decisionModeActive) {
    Decision d = decisionLoader.lookup(myVehicleId, simTime().dbl());
    double interval = (d.beacon_hz > 0) ? (1.0 / d.beacon_hz) : beaconIntervalVal;
    scheduleAt(simTime() + interval, sendBeaconEvt);
    // MAC backoff: set contention window (slots of 15.625 µs)
    if (d.mac_wait_ms > 0) {
        auto* mac = FindModule<Mac1609_4*>::findSubModule(getParentModule());
        if (mac) mac->par("contentionWindow") = (int)(d.mac_wait_ms / 0.015625);
    }
} else {
    scheduleAt(simTime() + beaconIntervalVal, sendBeaconEvt);
}
```

---

### 5. [src/trajectories/DataCollectorApp.ned](file:///home/massi/Documents/omnetpp-5.6.2/samples/TrajectoryCollector/src/trajectories/DataCollectorApp.ned)

Add a new string parameter with a sensible default:

```ned
parameters:
    ...
    string decisionsFile = default("decisions.json");
```

---

### 6. [omnetpp.ini](file:///home/massi/Documents/omnetpp-5.6.2/samples/TrajectoryCollector/omnetpp.ini)

Add to the `[General]` section:

```ini
# Static Mode — path to decisions.json produced by the Python pipeline
# Leave as "decisions.json" for auto-detection; set "" to force baseline mode.
*.node[*].appl.decisionsFile = "decisions.json"
```

---

## Decision Lookup Logic (Floor Lookup)

Because the Python pipeline writes timestep keys at irregular intervals, the lookup must find
the **most recent key ≤ current sim-time**:

```cpp
Decision DecisionLoader::lookup(const std::string& vehicleId, double t) const {
    auto vit = decisions_.find(vehicleId);
    if (vit == decisions_.end()) return defaultDecision();   // no entry → baseline

    const auto& timeMap = vit->second;
    // upper_bound gives first key > t; go one step back
    auto it = timeMap.upper_bound(t);
    if (it == timeMap.begin()) return defaultDecision();     // t is before first entry
    --it;
    return it->second;
}
```

---

## Build Changes

No new external libraries are required.  
The two new [.cc](file:///home/massi/Documents/omnetpp-5.6.2/samples/TrajectoryCollector/src/trajectories/DataCollectorApp.cc) / [.h](file:///home/massi/Documents/omnetpp-5.6.2/samples/TrajectoryCollector/src/trajectories/DataCollectorApp.h) files in `src/trajectories/` are automatically picked up by the
existing `Makefile` (OMNeT++ uses directory-level wildcard compilation for `src/`).

---

## Testing Checklist

- [ ] Baseline run: `decisions.json` absent → simulation behaves identically to today.
- [ ] Decision run: copy a valid `decisions.json` to the working directory → `[DecisionMode]`
      log line appears; beacon intervals vary per vehicle.
- [ ] Decision run: vehicle IDs in JSON that don't match any node → floor-lookup returns
      default (10 Hz), no crash.
- [ ] Decision run: `mac_wait_ms` values applied → verify via OMNeT++ Qtenv MAC stats.
- [ ] Metrics comparison: extract `results/General-#0.sca` from both runs and compare PDR /
      delay with `opp_scavetool`.

---

## Order of Changes

1. Create `DecisionLoader.h`
2. Create `DecisionLoader.cc`
3. Edit `DataCollectorApp.h` (add includes + members)
4. Edit `DataCollectorApp.cc` (initialize + handleSelfMsg)
5. Edit `DataCollectorApp.ned` (add `decisionsFile` parameter)
6. Edit `omnetpp.ini` (add `decisionsFile` line)
