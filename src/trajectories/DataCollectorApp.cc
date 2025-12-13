#include "DataCollectorApp.h"
#include <iomanip> // Needed for exact time formatting

Define_Module(DataCollectorApp);

void DataCollectorApp::initialize(int stage) {
    DemoBaseApplLayer::initialize(stage);
    if (stage == 0) {
        lastSpeed = 0;
        lastHeading = 0;
        lastStepTime = 0;
        mySequenceNumber = 0;

        // Initialize custom beacon timer
        sendBeaconEvt = new cMessage("sendBeacon");
        scheduleAt(simTime() + uniform(1.0, 60.0), sendBeaconEvt);

        // --- FIX 1: UNIQUE FILENAME ---
        // We use GetIndex() directly + Creation Time to ensure uniqueness
        int myIndex = getParentModule()->getIndex();
        std::string filename = "results/data_car_" + std::to_string(myIndex) +
                               "_t" + std::to_string((int)simTime().dbl()) + ".csv";

        csvFile.open(filename);

        // --- FIX 2: EXPLICIT HEADER ---
        csvFile << "Time,X,Y,Speed,Acceleration,Heading,AngularVelocity,"
                << "LaneID,LaneDist,"
                << "Neigh1_Rx,Neigh1_Ry,Neigh1_RSpeed,Neigh1_RHeading,"
                << "Neigh2_Rx,Neigh2_Ry,Neigh2_RSpeed,Neigh2_RHeading,"
                << "Neigh3_Rx,Neigh3_Ry,Neigh3_RSpeed,Neigh3_RHeading,"
                << "AvgDistToSender,AvgMsgDelay,PacketLossRate"
                << std::endl;
    }
}

void DataCollectorApp::handleSelfMsg(cMessage* msg) {
    if (msg == sendBeaconEvt) {
        // 1. Create message
        veins::TrajSafetyMessage* wsm = new veins::TrajSafetyMessage();

        // 2. Fill Fields
        wsm->setChannelNumber(static_cast<int>(Channel::cch));
        wsm->setRecipientAddress(LAddress::L2BROADCAST());
        wsm->setUserPriority(7);
        wsm->setBitLength(256);
        wsm->setSequenceNumber(mySequenceNumber++);

        // 3. Set Motion Data
        wsm->setSenderSpeed(curSpeed);

        TraCIMobility* mobility = check_and_cast<TraCIMobility*>(getParentModule()->getSubmodule("veinsmobility"));
        wsm->setSenderPos(mobility->getPositionAt(simTime()));

        // 4. Log
        EV << " [APP] Car " << myId << " sending beacon (Seq " << mySequenceNumber << ")" << endl;

        sendDown(wsm);
        scheduleAt(simTime() + 1.0, sendBeaconEvt);
    } else {
        DemoBaseApplLayer::handleSelfMsg(msg);
    }
}

void DataCollectorApp::onWSM(BaseFrame1609_4* frame) {
    // --- DEBUG: THE WITNESS LOG ---
    // This runs for EVERY message, no matter what type it is.
    EV << " [WITNESS] Radio received a message from Node " << frame->getSenderModuleId() << endl;
    // ------------------------------

    // 1. Try to cast to OUR custom message
    veins::TrajSafetyMessage* wsm = dynamic_cast<veins::TrajSafetyMessage*>(frame);

    // 2. If cast fails, check if it's a standard DemoSafetyMessage
    if (!wsm) {
        EV << "   -> It is NOT a TrajSafetyMessage. Checking standard message..." << endl;

        // Check standard type
        veins::DemoSafetyMessage* standardWsm = dynamic_cast<veins::DemoSafetyMessage*>(frame);
        if (standardWsm) {
             EV << "   -> It IS a standard DemoSafetyMessage! (Success)" << endl;
        } else {
             EV << "   -> Unknown message type." << endl;
        }

        // Pass to parent and EXIT
        DemoBaseApplLayer::onWSM(frame);
        return;
    }

    EV << "   -> It IS a TrajSafetyMessage. Processing..." << endl;

    // ... (Rest of your metrics logic) ...
    long senderId = wsm->getSenderModuleId();
    // ...
}

void DataCollectorApp::handlePositionUpdate(cObject* obj) {
    DemoBaseApplLayer::handlePositionUpdate(obj);

    TraCIMobility* mobility = check_and_cast<TraCIMobility*>(getParentModule()->getSubmodule("veinsmobility"));
    TraCICommandInterface::Vehicle* traci = mobility->getVehicleCommandInterface();

    Coord pos = mobility->getPositionAt(simTime());
    double speed = mobility->getSpeed();
    double heading = traci->getAngle();

    double accel = 0.0;
    double angularVelocity = 0.0;
    double dt = (simTime() - lastStepTime).dbl();

    if (dt > 0) {
        accel = (speed - lastSpeed) / dt;
        angularVelocity = (heading - lastHeading) / dt;
    }

    lastSpeed = speed;
    lastHeading = heading;
    lastStepTime = simTime();

    // Safe String Handling
    std::string laneId = traci->getLaneId();
    if (laneId.empty()) laneId = "junction";
    double laneDist = traci->getLanePosition();

    // --- FIX: RELAXED TIMEOUT ---
    struct SortableNeigh { long id; double distSq; };
    std::vector<SortableNeigh> sorted;

    for (auto it = neighborTable.begin(); it != neighborTable.end(); ) {
        // Change 1.0 to 5.0 to prevent deleting neighbors too quickly
        if (simTime() - it->second.lastSeen > 5.0) {
            neighborTable.erase(it++);
        } else {
            double dx = it->second.pos.x - pos.x;
            double dy = it->second.pos.y - pos.y;
            sorted.push_back({it->first, dx*dx + dy*dy});
            ++it;
        }
    }

    std::sort(sorted.begin(), sorted.end(), [](const SortableNeigh &a, const SortableNeigh &b) {
        return a.distSq < b.distSq;
    });

    // Aggregate Stats
    double avgDelay = 0;
    double avgDist = 0;
    double totalPLR = 0;
    int count = 0;

    for(const auto& item : sorted) {
        NeighborData& n = neighborTable[item.id];
        avgDelay += n.lastDelay;
        avgDist += n.distToSender;
        if (n.totalPacketsReceived + n.totalPacketsLost > 0) {
            totalPLR += (double)n.totalPacketsLost / (n.totalPacketsReceived + n.totalPacketsLost);
        }
        count++;
    }
    if (count > 0) {
        avgDelay /= count;
        avgDist /= count;
        totalPLR /= count;
    }

    // --- WRITE TO CSV ---
    csvFile << std::fixed << std::setprecision(4);
    csvFile << simTime() << "," << pos.x << "," << pos.y << ","
            << speed << "," << accel << "," << heading << "," << angularVelocity << ",";
    csvFile << "\"" << laneId << "\"," << laneDist << ",";

    for(int i=0; i<3; i++) {
        if(i < (int)sorted.size()) {
            NeighborData& n = neighborTable[sorted[i].id];
            double relX = n.pos.x - pos.x;
            double relY = n.pos.y - pos.y;
            double relSpeed = sqrt(pow(n.speed.x, 2) + pow(n.speed.y, 2)) - speed;
            double relHeading = n.heading - (heading * M_PI / 180.0);
            csvFile << relX << "," << relY << "," << relSpeed << "," << relHeading << ",";
        } else {
            csvFile << "0,0,0,0,";
        }
    }

    csvFile << avgDist << "," << avgDelay << "," << totalPLR << std::endl << std::flush;
}

void DataCollectorApp::finish() {
    DemoBaseApplLayer::finish();
    cancelAndDelete(sendBeaconEvt);
    csvFile.close();
}
