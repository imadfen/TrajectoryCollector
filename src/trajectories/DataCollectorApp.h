#pragma once

#include "veins/modules/application/ieee80211p/DemoBaseApplLayer.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"
#include "veins/modules/messages/DemoSafetyMessage_m.h"
#include "TrajSafetyMessage_m.h" // Import our generated message
#include "DecisionLoader.h"         // Static Mode: AI decision lookup
#include <set>
#include <utility>

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

    // Emergency Alert Logic
    void triggerEmergencyAlert();

    std::ofstream csvFile;
    double lastSpeed;
    double lastHeading;
    simtime_t lastStepTime;

    // Beaconing Logic
    cMessage* sendBeaconEvt;
    double beaconIntervalVal;      // read from NED beaconInterval parameter
    unsigned long mySequenceNumber;

    // Static Mode Decision Pipeline
    DecisionLoader decisionLoader;  // loads decisions.json once at startup
    bool decisionModeActive;        // true when a valid decisions.json was found
    std::string myVehicleId;        // "car_<index>" — key into decisions.json

    std::map<long, NeighborData> neighborTable;

    // Loop B: Selective Forwarding alert structures
    std::set<std::pair<int, unsigned long>> receivedAlerts;
    std::map<std::pair<int, unsigned long>, cMessage*> scheduledRelays;
};
