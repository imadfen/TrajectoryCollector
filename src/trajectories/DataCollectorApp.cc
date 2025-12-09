#include "DataCollectorApp.h"

Define_Module(DataCollectorApp);

void DataCollectorApp::initialize(int stage) {
    DemoBaseApplLayer::initialize(stage);
    if (stage == 0) {
        lastSpeed = 0;
        lastSpeedTime = 0;

        // Open a CSV file for this specific car
        std::string filename = "results/car_" + std::to_string(myId) + ".csv";
        csvFile.open(filename);

        // Write Header
        csvFile << "Time,X,Y,Speed,Accel,Heading,LaneID,"
                << "N1_Rx,N1_Ry,N1_Dist,"
                << "N2_Rx,N2_Ry,N2_Dist,"
                << "N3_Rx,N3_Ry,N3_Dist" << std::endl;
    }
}

void DataCollectorApp::onWSM(BaseFrame1609_4* frame) {
    // 1. Cast to the standard Veins 5 Safety Message
    DemoSafetyMessage* wsm = check_and_cast<DemoSafetyMessage*>(frame);

    // 2. Store Neighbor Info
    NeighborData n;
    n.pos = wsm->getSenderPos();
    n.speed = wsm->getSenderSpeed();
    n.lastSeen = simTime();

    neighborTable[wsm->getSenderModuleId()] = n;

    // Optional: Pass to parent
    DemoBaseApplLayer::onWSM(frame);
}

void DataCollectorApp::handlePositionUpdate(cObject* obj) {
    DemoBaseApplLayer::handlePositionUpdate(obj);

    // 1. Get Mobility & TraCI info
    TraCIMobility* mobility = check_and_cast<TraCIMobility*>(getParentModule()->getSubmodule("veinsmobility"));
    TraCICommandInterface::Vehicle* traciVehicle = mobility->getVehicleCommandInterface();

    Coord pos = mobility->getPositionAt(simTime());
    double speed = mobility->getSpeed();
    double heading = traciVehicle->getAngle();
    std::string laneId = traciVehicle->getLaneId();

    // 2. Calculate Acceleration
    double accel = 0.0;
    if ((simTime() - lastSpeedTime) > 0)
        accel = (speed - lastSpeed) / (simTime() - lastSpeedTime).dbl();
    lastSpeed = speed;
    lastSpeedTime = simTime();

    // 3. Process Neighbors (Sort by distance)
    // We define the struct here
    struct RelNeigh { double distSq, rx, ry; };
    std::vector<RelNeigh> sorted;

    for (auto it = neighborTable.begin(); it != neighborTable.end(); ) {
        if (simTime() - it->second.lastSeen > 1.0) neighborTable.erase(it++);
        else {
            double dx = it->second.pos.x - pos.x;
            double dy = it->second.pos.y - pos.y;
            double dSq = dx*dx + dy*dy;
            if (dSq > 0.1) sorted.push_back({dSq, dx, dy});
            ++it;
        }
    }

    // FIX: Replaced 'auto' with 'const RelNeigh' for C++11 compatibility
    std::sort(sorted.begin(), sorted.end(), [](const RelNeigh &a, const RelNeigh &b){
        return a.distSq < b.distSq;
    });

    // 4. Write Data Line
    csvFile << simTime() << "," << pos.x << "," << pos.y << "," << speed << ","
            << accel << "," << heading << "," << laneId << ",";

    for(int i=0; i<3; i++) {
        if(i < (int)sorted.size())
            csvFile << sorted[i].rx << "," << sorted[i].ry << "," << sqrt(sorted[i].distSq) << ",";
        else csvFile << "0,0,0,";
    }
    csvFile << std::endl;
}

void DataCollectorApp::finish() {
    DemoBaseApplLayer::finish();
    csvFile.close();
}
