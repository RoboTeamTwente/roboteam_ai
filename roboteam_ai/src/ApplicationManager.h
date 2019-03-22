//
// Created by mrlukasbos on 14-1-19.
//

#ifndef ROBOTEAM_AI_APPLICATIONMANAGER_H
#define ROBOTEAM_AI_APPLICATIONMANAGER_H

#include <gtest/gtest_prod.h>
#include <roboteam_ai/src/dangerfinder/DangerFinder.h>
#include "io/IOManager.h"
#include "treeinterp/BTFactory.h"
#include "ros/ros.h"

namespace df = rtt::ai::dangerfinder;

namespace rtt {

class ApplicationManager {
private:
    FRIEND_TEST(ApplicationManagerTest, it_handles_ROS_data);
    rtt::ai::io::IOManager * IOManager;
    roboteam_msgs::World worldMsg;
    roboteam_msgs::GeometryData geometryMsg;
    roboteam_msgs::RefereeData refereeMsg;
    bt::BehaviorTree::Ptr strategy;
    bt::BehaviorTree::Ptr keeperTree;
    df::DangerData dangerData;
    void updateROSData();
    void updateDangerfinder();
    void handleRefData();
    void notifyTreeStatus(bt::Node::Status status);
    void runOneLoopCycle();

public:
    void setup();
    void loop();
    void checkForShutdown();
};

} // rtt

#endif //ROBOTEAM_AI_APPLICATIONMANAGER_H
