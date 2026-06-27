#pragma once

#include <string>
#include <map>

struct Decision {
    bool   found;        
    int    flag;        
    double mac_wait_ms;  
};


class DecisionLoader {
public:
    DecisionLoader() : loaded_(false) {}

    bool load(const std::string& path);

    bool hasDecisions() const { return loaded_; }

    Decision lookup(const std::string& vehicleId, double t) const;

private:
    std::map<std::string, std::map<double, Decision>> decisions_;
    bool loaded_;

    static Decision defaultDecision() { return {false, 0, 0.0}; }
};
