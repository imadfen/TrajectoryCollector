#pragma once

#include "veins/modules/application/ieee80211p/DemoBaseApplLayer.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"
#include "veins/modules/messages/DemoSafetyMessage_m.h"

using namespace veins;

struct NeighborData {
    Coord pos;
    Coord speed;
    simtime_t lastSeen;
};

class DataCollectorApp : public DemoBaseApplLayer {
public:
    virtual void initialize(int stage) override;
    virtual void finish() override;

protected:
    virtual void onWSM(BaseFrame1609_4* frame) override;
    virtual void handlePositionUpdate(cObject* obj) override;

    std::ofstream csvFile;
    double lastSpeed;
    simtime_t lastSpeedTime;
    std::map<long, NeighborData> neighborTable;
};
