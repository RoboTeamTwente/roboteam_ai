//
// Created by mrlukasbos on 14-1-19.
//

#ifndef ROBOTEAM_AI_APPLICATIONMANAGER_H
#define ROBOTEAM_AI_APPLICATIONMANAGER_H

#include <gtest/gtest_prod.h>
#include <roboteam_ai/src/coach/OffensiveCoach.h>
#include <roboteam_ai/src/coach/PassCoach.h>
#include "io/IOManager.h"
#include "treeinterp/BTFactory.h"
#include "ros/ros.h"
#include <roboteam_ai/src/utilities/StrategyManager.h>

namespace rtt {

class ApplicationManager {
private:
    FRIEND_TEST(ApplicationManagerTest, it_handles_ROS_data);
    rtt::ai::io::IOManager * IOManager;

    bt::BehaviorTree::Ptr strategy;
    bt::BehaviorTree::Ptr keeperTree;

    void notifyTreeStatus(bt::Node::Status status);
    void runOneLoopCycle();

    ai::StrategyManager strategyManager;
    std::string oldKeeperTreeName = "";
    std::string oldStrategy = "";
public:
    void setup();
    void loop();
    void checkForShutdown();
};

} // rtt

#endif //ROBOTEAM_AI_APPLICATIONMANAGER_H
