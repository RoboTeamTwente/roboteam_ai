//
// Created by mrlukasbos on 14-1-19.
//

#ifndef ROBOTEAM_AI_APPLICATIONMANAGER_H
#define ROBOTEAM_AI_APPLICATIONMANAGER_H

#include "io/IOManager.h"
#include "treeinterp/BTFactory.h"

class ApplicationManager {
private:
    // init IOManager and subscribe to all topics immediately
    rtt::ai::io::IOManager IOManager;
    roboteam_msgs::World worldMsg;
    roboteam_msgs::GeometryData geometryMsg;
    roboteam_msgs::RefereeData refereeMsg;
    bt::BehaviorTree::Ptr strategy;
    BTFactory factory;
    df::DangerData dangerData;

    void updateROSData();
    void updateDangerfinder();
    void handleRefData();
    void notifyTreeStatus(bt::Node::Status status);

public:
    void setup();
    void loop();
    void checkForShutdown();
};

#endif //ROBOTEAM_AI_APPLICATIONMANAGER_H
