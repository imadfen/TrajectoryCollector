#include "DataCollectorApp.h"
#include <iomanip>
#include <sys/stat.h>
#include <sstream>
#include "veins/modules/mac/ieee80211p/Mac1609_4.h"


Define_Module(DataCollectorApp);

void DataCollectorApp::initialize(int stage) {
    DemoBaseApplLayer::initialize(stage);
    if (stage == 0) {
        lastSpeed = 0;
        lastHeading = 0;
        lastStepTime = 0;
        mySequenceNumber = 0;
        beaconIntervalVal = par("beaconInterval").doubleValue();

        // ── Static Mode Decision Pipeline ────────────────────────────────────
        // Try to load decisions.json (produced by the Python pipeline).
        // If the file exists we enter Decision Mode; otherwise Baseline Mode.
        std::string decisionsPath = par("decisionsFile").stringValue();
        decisionModeActive = decisionLoader.load(decisionsPath);
        
        static bool _firstInitializionPrint = true;
        if (_firstInitializionPrint) {
            _firstInitializionPrint = false;
            std::cout << "\n============================================\n";
            if (decisionModeActive) {
                std::cout << " [APPLICATION MODE] -> DECISION API LOADED! \n";
                std::cout << " File: " << decisionsPath << "\n";
            } else {
                std::cout << " [APPLICATION MODE] -> BASELINE BEACONING \n";
                std::cout << " (Decisions file missing or disabled)\n";
            }
            std::cout << "============================================\n\n";
        }

        // Build our vehicle ID as it appears in the decisions JSON: "car_<nodeIndex>"
        myVehicleId = "car_" + std::to_string(getParentModule()->getIndex());
        if (decisionModeActive)
            EV << "[DecisionMode] Loaded '" << decisionsPath
               << "' for vehicle '" << myVehicleId << "'" << endl;
        else
            EV << "[BaselineMode] No decisions file at '" << decisionsPath
               << "' — using flat " << (1.0/beaconIntervalVal) << " Hz beaconing" << endl;
        // ─────────────────────────────────────────────────────────────────────

        sendBeaconEvt = new cMessage("sendBeacon");
        scheduleAt(simTime() + uniform(1.0, 60.0), sendBeaconEvt);

        int myIndex = getParentModule()->getIndex();
        mkdir("results", 0777);
        mkdir("results/raw", 0777);
        std::string filename = "results/raw/data_car_" + std::to_string(myIndex) +
                               "_t" + std::to_string((int)simTime().dbl()) + ".csv";

        csvFile.open(filename);

        csvFile << "Time,X,Y,Speed,Acceleration,Heading,AngularVelocity,"
                << "LaneID,LaneDist,";
        for (int i = 1; i <= 10; ++i) {
            csvFile << "Neigh" << i << "_Rx,Neigh" << i << "_Ry,Neigh" << i << "_RSpeed,Neigh" << i << "_RHeading,";
        }
        csvFile << "AvgDistToSender,AvgMsgDelay,PacketLossRate" << std::endl;
    }
}

void DataCollectorApp::handleSelfMsg(cMessage* msg) {
    if (msg == sendBeaconEvt) {
        
        veins::TrajSafetyMessage* wsm = new veins::TrajSafetyMessage();
        EV << "[DEBUG] Created message type: " << wsm->getClassName() << endl;

        populateWSM(wsm);
        
        wsm->setSenderId(myId);
        wsm->setChannelNumber(static_cast<int>(Channel::cch));
        wsm->setRecipientAddress(LAddress::L2BROADCAST());
        wsm->setUserPriority(7);
        wsm->setBitLength(8000);  // Stress-test v2: max realistic BSM (IEEE 1609.2 full cert ~1000 bytes)
        wsm->setSequenceNumber(mySequenceNumber++);

        
        wsm->setSenderSpeed(curSpeed);

        TraCIMobility* mobility = check_and_cast<TraCIMobility*>(getParentModule()->getSubmodule("veinsmobility"));
        wsm->setSenderPos(mobility->getPositionAt(simTime()));

        // Default selective forwarding parameters:
        wsm->setIsEmergency(false);
        wsm->setOriginalSenderId(myId);
        wsm->setAlertSeqNum(wsm->getSequenceNumber());

        // ── Static Mode Decision Pipeline: choose next beacon interval ───────
        double interval = beaconIntervalVal;
        if (decisionModeActive) {
            Decision d = decisionLoader.lookup(myVehicleId, simTime().dbl());

            if (d.found) {
                // Loop A — Beacon rate driven by flag:
                //   flag = 1  →  normal suppression OFF  → 10 Hz (0.1 s interval)
                //   flag = 0  →  beacon suppressed        →  2 Hz (0.5 s interval)
                interval = (d.flag == 1) ? 0.1 : 0.5;

                EV << "[DecisionMode] " << myVehicleId
                   << " t=" << simTime()
                   << " flag=" << d.flag
                   << " beacon=" << (1.0 / interval) << "Hz"
                   << " mac_wait_ms=" << d.mac_wait_ms << endl;
            } else {
                interval = beaconIntervalVal;
                EV << "[DecisionMode] " << myVehicleId
                   << " t=" << simTime()
                   << " no decision found, defaulting to " << (1.0 / interval) << "Hz" << endl;
            }

        } else {
            // Baseline Mode: flat beaconing as configured in omnetpp.ini
            interval = beaconIntervalVal;
        }

        scheduleAt(simTime() + interval, sendBeaconEvt);
        // ─────────────────────────────────────────────────────────────────────

        EV << " [APP] Car " << myId << " sending periodic BSM (Seq " << wsm->getSequenceNumber() << ")" << endl;
        wsm->setGenerationTime(simTime());
        sendDown(wsm);

    } else {
        std::string msgName = msg->getName();
        if (msgName.rfind("relay:", 0) == 0) {
            // Loop B Selective Forwarding: parse origId and seqNum from "relay:origId:seqNum"
            std::stringstream ss(msgName);
            std::string prefix, origIdStr, seqNumStr;
            std::getline(ss, prefix, ':');
            std::getline(ss, origIdStr, ':');
            std::getline(ss, seqNumStr, ':');
            int origId = std::stoi(origIdStr);
            unsigned long seqNum = std::stoul(seqNumStr);

            // Create a relayed emergency alert packet
            veins::TrajSafetyMessage* wsm = new veins::TrajSafetyMessage();
            populateWSM(wsm);
            wsm->setSenderId(myId);
            wsm->setChannelNumber(static_cast<int>(Channel::cch));
            wsm->setRecipientAddress(LAddress::L2BROADCAST());
            wsm->setUserPriority(7);
            wsm->setBitLength(8000);  // Same size as BSM
            wsm->setSequenceNumber(mySequenceNumber++);
            wsm->setSenderSpeed(curSpeed);
            TraCIMobility* mobility = check_and_cast<TraCIMobility*>(getParentModule()->getSubmodule("veinsmobility"));
            wsm->setSenderPos(mobility->getPositionAt(simTime()));
            wsm->setGenerationTime(simTime());

            wsm->setIsEmergency(true);
            wsm->setOriginalSenderId(origId);
            wsm->setAlertSeqNum(seqNum);

            EV << " [LOOP B RELAY] Car " << myId << " relaying emergency alert from original sender Node " << origId << " (Seq " << seqNum << ")" << endl;
            sendDown(wsm);

            scheduledRelays.erase({origId, seqNum});
            delete msg;
        } else {
            DemoBaseApplLayer::handleSelfMsg(msg);
        }
    }
}

void DataCollectorApp::handleLowerMsg(cMessage* msg) {
    EV << "========================================" << endl;
    EV << "[LOWER MSG] Received at application layer!" << endl;
    EV << "[LOWER MSG] Message type: " << msg->getClassName() << endl;
    EV << "[LOWER MSG] Message name: " << msg->getName() << endl;
    EV << "========================================" << endl;
    
    
    BaseFrame1609_4* frame = dynamic_cast<BaseFrame1609_4*>(msg);
    
    if (frame) {
        EV << "[LOWER MSG] Successfully cast to BaseFrame1609_4, calling onWSM()" << endl;
        
        onWSM(frame);
        delete msg;
        return;
    }
    
    
    EV << "[LOWER MSG] Not a BaseFrame1609_4, passing to parent class" << endl;
    DemoBaseApplLayer::handleLowerMsg(msg);
}

void DataCollectorApp::onWSM(BaseFrame1609_4* frame) {
    EV << "[DEBUG] Received message from Node " << frame->getSenderModuleId() << endl;
    EV << "[DEBUG] Message class: " << frame->getClassName() << endl;
    EV << "[DEBUG] Expected: veins::TrajSafetyMessage" << endl;

    
    veins::TrajSafetyMessage* wsm = dynamic_cast<veins::TrajSafetyMessage*>(frame);

    // 2. If cast fails, check if it's a standard DemoSafetyMessage
    if (!wsm) {
        EV << "   -> It is NOT a TrajSafetyMessage. Checking standard message..." << endl;

        
        veins::DemoSafetyMessage* standardWsm = dynamic_cast<veins::DemoSafetyMessage*>(frame);
        if (standardWsm) {
             EV << "   -> It IS a standard DemoSafetyMessage! (Success)" << endl;
        } else {
             EV << "   -> Unknown message type." << endl;
        }

        
        DemoBaseApplLayer::onWSM(frame);
        return;
    }

    EV << "   -> It IS a TrajSafetyMessage. Processing..." << endl;

    
    long senderId = wsm->getSenderId();
    Coord senderPos = wsm->getSenderPos();
    Coord senderSpeed = wsm->getSenderSpeed();
    unsigned long seqNum = wsm->getSequenceNumber();
    
    
    TraCIMobility* mobility = check_and_cast<TraCIMobility*>(getParentModule()->getSubmodule("veinsmobility"));
    Coord myPos = mobility->getPositionAt(simTime());
    
    
    double dx = senderPos.x - myPos.x;
    double dy = senderPos.y - myPos.y;
    double distance = sqrt(dx*dx + dy*dy);
    
    
    simtime_t delay = simTime() - wsm->getGenerationTime();
    
    
    NeighborData& neighbor = neighborTable[senderId];
    
    
    if (neighbor.initialized && seqNum > neighbor.lastSeqNum + 1) {
    int lost = seqNum - neighbor.lastSeqNum - 1;
    neighbor.totalPacketsLost += lost;
    EV << "   [PACKET LOSS] Expected seq " << (neighbor.lastSeqNum + 1) 
       << " but got " << seqNum << " (" << lost << " packets lost)" << endl;
    }
    neighbor.initialized = true;
    neighbor.lastSeqNum = seqNum;
    neighbor.id = senderId;
    neighbor.pos = senderPos;
    neighbor.speed = senderSpeed;
    neighbor.heading = atan2(senderSpeed.y, senderSpeed.x); 
    neighbor.lastSeen = simTime();
    neighbor.totalPacketsReceived++;
    neighbor.lastDelay = delay.dbl();
    neighbor.distToSender = distance;
    
    EV << "   [NEIGHBOR UPDATE] ID=" << senderId 
       << " Pos=(" << senderPos.x << "," << senderPos.y << ")"
       << " Dist=" << distance << "m"
       << " Seq=" << seqNum
       << " Received=" << neighbor.totalPacketsReceived
       << " Lost=" << neighbor.totalPacketsLost << endl;

    // Loop B: Selective Forwarding emergency alert logic
    if (wsm->getIsEmergency()) {
        int origId = wsm->getOriginalSenderId();
        unsigned long alertSeq = wsm->getAlertSeqNum();
        std::pair<int, unsigned long> alertKey(origId, alertSeq);

        EV << " [LOOP B ALERT] Received emergency alert (" << origId << ", " << alertSeq << ") from Node " << senderId << endl;

        if (origId != myId) {
            auto rit = scheduledRelays.find(alertKey);
            if (rit != scheduledRelays.end()) {
                // Someone else relayed it first -> suppress our redundant transmission!
                EV << " [LOOP B SUPPRESSION] Node " << senderId << " relayed it first. Suppressing our redundant relay timer." << endl;
                cancelAndDelete(rit->second);
                scheduledRelays.erase(rit);
            } else {
                // If it is a new alert we haven't seen yet, schedule to relay it after mac_wait_ms delay
                if (receivedAlerts.count(alertKey) == 0) {
                    receivedAlerts.insert(alertKey);

                    double macWait = 1.0; // Default wait is 1ms
                    if (decisionModeActive) {
                        Decision d = decisionLoader.lookup(myVehicleId, simTime().dbl());
                        if (d.found && d.mac_wait_ms > 0.0) {
                            macWait = d.mac_wait_ms;
                        }
                    }

                    double delaySec = macWait / 1000.0; // Convert ms to seconds
                    EV << " [LOOP B SCHEDULING] Scheduling relay for alert (" << origId << ", " << alertSeq << ") in " << macWait << " ms" << endl;

                    std::string timerName = "relay:" + std::to_string(origId) + ":" + std::to_string(alertSeq);
                    cMessage* relayTimer = new cMessage(timerName.c_str());
                    scheduledRelays[alertKey] = relayTimer;
                    scheduleAt(simTime() + delaySec, relayTimer);
                }
            }
        }
    }
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

        // Auto-trigger an emergency alert if hard braking occurs
        if (accel < -4.0) {
            triggerEmergencyAlert();
        }
    }

    lastSpeed = speed;
    lastHeading = heading;
    lastStepTime = simTime();

    
    std::string laneId = traci->getLaneId();
    if (laneId.empty()) laneId = "junction";
    double laneDist = traci->getLanePosition();

    
    struct SortableNeigh { long id; double distSq; };
    std::vector<SortableNeigh> sorted;

    for (auto it = neighborTable.begin(); it != neighborTable.end(); ) {
        
        if (simTime() - it->second.lastSeen > 1.0) {
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

    
    csvFile << std::fixed << std::setprecision(4);
    csvFile << simTime() << "," << pos.x << "," << pos.y << ","
            << speed << "," << accel << "," << heading << "," << angularVelocity << ",";
    csvFile << "\"" << laneId << "\"," << laneDist << ",";

    for(int i=0; i<10; i++) {
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
    for (auto& pair : scheduledRelays) {
        cancelAndDelete(pair.second);
    }
    scheduledRelays.clear();
    csvFile.close();
}

void DataCollectorApp::triggerEmergencyAlert() {
    // Prevent spamming alerts; only send if we haven't sent one recently for this sequence
    unsigned long alertSeq = mySequenceNumber++;
    std::pair<int, unsigned long> alertKey(myId, alertSeq);
    if (receivedAlerts.count(alertKey) > 0) return;
    receivedAlerts.insert(alertKey);

    veins::TrajSafetyMessage* wsm = new veins::TrajSafetyMessage();
    populateWSM(wsm);
    wsm->setSenderId(myId);
    wsm->setChannelNumber(static_cast<int>(Channel::cch));
    wsm->setRecipientAddress(LAddress::L2BROADCAST());
    wsm->setUserPriority(7);
    wsm->setBitLength(1000);  // Alerts can be smaller than BSMs
    wsm->setSequenceNumber(alertSeq);
    wsm->setSenderSpeed(curSpeed);
    
    TraCIMobility* mobility = check_and_cast<TraCIMobility*>(getParentModule()->getSubmodule("veinsmobility"));
    wsm->setSenderPos(mobility->getPositionAt(simTime()));
    wsm->setGenerationTime(simTime());

    wsm->setIsEmergency(true);
    wsm->setOriginalSenderId(myId);
    wsm->setAlertSeqNum(alertSeq);

    EV << " [EMERGENCY] Car " << myId << " hard braking detected! Generating Emergency Alert (Seq " << alertSeq << ")" << endl;
    sendDown(wsm);
}
