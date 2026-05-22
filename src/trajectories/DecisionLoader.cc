#include "DecisionLoader.h"

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <cstdlib>   // strtod

// ---------------------------------------------------------------------------
// Minimal, dependency-free JSON parser for the decisions.json format.
//
// The format produced by export_decisions.py is well-structured and we only
// need to extract three kinds of values:
//   1. Vehicle ID keys  → strings like  "car_32"
//   2. Timestep keys    → strings like  "18003.1"
//   3. Leaf values      → flag (int 0/1) and mac_wait_ms (double)
//
// Strategy: iterate line-by-line.  Each line falls into one of:
//   • top-level vehicle key  (contains "car_")
//   • timestep sub-key       (numeric string, no alpha chars)
//   • flag value
//   • mac_wait_ms value
// ---------------------------------------------------------------------------

namespace {

// Extract the string inside the first pair of double-quotes on a line.
// Returns "" if no quoted string is found.
std::string extractQuotedString(const std::string& line) {
    std::size_t first = line.find('"');
    if (first == std::string::npos) return "";
    std::size_t second = line.find('"', first + 1);
    if (second == std::string::npos) return "";
    return line.substr(first + 1, second - first - 1);
}

// True if 's' looks like a numeric timestep key (digits, at most one dot,
// no letters).
bool isNumericKey(const std::string& s) {
    if (s.empty()) return false;
    bool hasDot = false;
    for (char c : s) {
        if (c == '.') {
            if (hasDot) return false;
            hasDot = true;
        } else if (c < '0' || c > '9') {
            return false;
        }
    }
    return true;
}

// Extract the numeric value after a colon on a line, e.g.  "beacon_hz": 2.0
// Returns 0 on failure.
double extractColonValue(const std::string& line) {
    std::size_t colon = line.rfind(':');
    if (colon == std::string::npos) return 0.0;
    const char* start = line.c_str() + colon + 1;
    char* end = nullptr;
    double v = std::strtod(start, &end);
    if (end == start) return 0.0;
    return v;
}

} // anonymous namespace

// ---------------------------------------------------------------------------

bool DecisionLoader::load(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open()) return false;          // file not found → baseline mode

    std::string currentVehicle;
    double      currentTimestep = -1.0;
    Decision    currentDecision{true, 0, 0.0};
    bool        inTimestep = false;

    auto commitTimestep = [&]() {
        if (!currentVehicle.empty() && currentTimestep >= 0 && inTimestep) {
            decisions_[currentVehicle][currentTimestep] = currentDecision;
        }
        currentTimestep = -1.0;
        currentDecision = {true, 0, 0.0};
        inTimestep = false;
    };

    std::string line;
    while (std::getline(f, line)) {
        // Skip blank lines and pure-brace lines
        if (line.find_first_not_of(" \t\r\n{},") == std::string::npos) continue;

        // ── vehicle-level key:  "car_32": { or "data_car_294_t18183": {
        if (line.find("car_") != std::string::npos && line.find('{') != std::string::npos) {
            commitTimestep();
            std::string rawVehicle = extractQuotedString(line);
            // Normalize "data_car_<index>_t<timestamp>" to "car_<index>"
            if (rawVehicle.rfind("data_car_", 0) == 0) {
                std::size_t startIdx = 9; // length of "data_car_"
                std::size_t endIdx = rawVehicle.find('_', startIdx);
                if (endIdx != std::string::npos) {
                    currentVehicle = "car_" + rawVehicle.substr(startIdx, endIdx - startIdx);
                } else {
                    currentVehicle = rawVehicle;
                }
            } else {
                currentVehicle = rawVehicle;
            }
            continue;
        }

        // ── timestep key:  "18003.1": {
        std::string q = extractQuotedString(line);
        if (!q.empty() && isNumericKey(q) && line.find('{') != std::string::npos) {
            commitTimestep();
            currentTimestep = std::strtod(q.c_str(), nullptr);
            inTimestep = true;
            continue;
        }

        // ── flag value  (0 = 10 Hz normal, 1 = 2 Hz suppressed)
        if (line.find("\"flag\"") != std::string::npos) {
            currentDecision.flag = static_cast<int>(extractColonValue(line));
            continue;
        }

        // ── mac_wait_ms value
        if (line.find("\"mac_wait_ms\"") != std::string::npos) {
            currentDecision.mac_wait_ms = extractColonValue(line);
            continue;
        }
    }

    commitTimestep(); // flush the last entry
    loaded_ = !decisions_.empty();
    return loaded_;
}

// ---------------------------------------------------------------------------

Decision DecisionLoader::lookup(const std::string& vehicleId, double t) const {
    auto vit = decisions_.find(vehicleId);
    if (vit == decisions_.end()) return defaultDecision(); // unknown vehicle

    const auto& timeMap = vit->second;
    if (timeMap.empty()) return defaultDecision();

    // upper_bound(t) → first key strictly greater than t
    auto it = timeMap.upper_bound(t);
    if (it == timeMap.begin()) return defaultDecision(); // t is before first entry
    --it; // step back to the largest key <= t
    return it->second;
}
