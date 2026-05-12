#pragma once

#include "veins/modules/application/ieee80211p/DemoBaseApplLayer.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"
#include "veins/modules/messages/DemoSafetyMessage_m.h"
#include "TrajSafetyMessage_m.h" // Import our generated message

using namespace veins;

struct NeighborData {
    long id;
    Coord pos;
    Coord speed;
    double heading;
    simtime_t lastSeen;
    unsigned long lastSeqNum;
    int totalPacketsLost;
    int totalPacketsReceived;
    double lastDelay;
    double distToSender;
    bool initialized = false; 
};

class DataCollectorApp : public DemoBaseApplLayer {
public:
    virtual void initialize(int stage) override;
    virtual void finish() override;

protected:
    virtual void onWSM(BaseFrame1609_4* frame) override;
    virtual void handlePositionUpdate(cObject* obj) override;
    virtual void handleLowerMsg(cMessage* msg) override;

    // Timer handling
    virtual void handleSelfMsg(cMessage* msg) override;

    std::ofstream csvFile;
    double lastSpeed;
    double lastHeading;
    simtime_t lastStepTime;

    // Beaconing Logic
    cMessage* sendBeaconEvt;
    double beaconIntervalVal;   // read from NED beaconInterval parameter
    unsigned long mySequenceNumber;

    std::map<long, NeighborData> neighborTable;
};
