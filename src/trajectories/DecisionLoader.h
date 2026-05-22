#pragma once

#include <string>
#include <map>

/**
 * A single AI decision entry produced by the Python pipeline
 * (src/deploy/export_decisions.py) for one vehicle at one timestep.
 */
struct Decision {
    bool   found;        // true if this is a real JSON entry, false if defaulted (no entry)
    int    flag;         // Loop A beacon mode: 0 = 2 Hz (stable/suppress), 1 = 10 Hz (emergency)
    double mac_wait_ms;  // Loop B MAC backoff wait (ms); 0 means "use default"
};

/**
 * DecisionLoader
 *
 * Loads a decisions.json file written by the Python pipeline and provides
 * per-vehicle, per-timestep lookups during the OMNeT++ Static Mode "Run 2".
 *
 * JSON format expected:
 * {
 *   "car_32": {
 *     "18003.1": {"flag": 0, "mac_wait_ms": 100.0},
 *     "18003.2": {"flag": 1, "mac_wait_ms": 1.0}
 *   },
 *   "car_211": {
 *     "18133.4": {"flag": 0, "mac_wait_ms": 100.0}
 *   }
 * }
 *
 * flag semantics (Loop A — beacon suppression):
 *   0  →  suppressed  :  2 Hz  (beaconInterval = 0.5 s)
 *   1  →  normal rate : 10 Hz  (beaconInterval = 0.1 s)
 *
 * mac_wait_ms (Loop B — MAC contention window backoff):
 *   Contention window slots = mac_wait_ms / 0.015625
 *
 * No external JSON library is required — the parser is stdlib-only.
 */
class DecisionLoader {
public:
    DecisionLoader() : loaded_(false) {}

    /**
     * Load and parse decisions.json from the given path.
     * Returns true if the file was found and parsed successfully,
     * false if the file is missing or cannot be parsed.
     */
    bool load(const std::string& path);

    /** True after a successful load(). */
    bool hasDecisions() const { return loaded_; }

    /**
     * Retrieve the decision that applies to 'vehicleId' at simulation time 't'.
     *
     * Uses a floor lookup: returns the entry whose timestep key is the largest
     * value <= t.  If no entry exists for the vehicle, or t is before the
     * first entry, returns a default Decision with found=false (caller falls back
     * to the flat beaconIntervalVal).
     */
    Decision lookup(const std::string& vehicleId, double t) const;

private:
    // decisions_[vehicleId][timestep] = Decision
    std::map<std::string, std::map<double, Decision>> decisions_;
    bool loaded_;

    static Decision defaultDecision() { return {false, 0, 0.0}; }  // found=false → fallback to defaults beaconIntervalVal
};
