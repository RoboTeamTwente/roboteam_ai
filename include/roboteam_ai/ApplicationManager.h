//
// Created by mrlukasbos on 14-1-19.
//

#ifndef ROBOTEAM_AI_APPLICATIONMANAGER_H
#define ROBOTEAM_AI_APPLICATIONMANAGER_H

#include <gtest/gtest_prod.h>
#include <utilities/StrategyManager.h>
#include "io/IOManager.h"
#include "treeinterp/BTFactory.h"

namespace rtt {

class ApplicationManager {
private:
    FRIEND_TEST(ApplicationManagerTest, it_handles_ROS_data);

    int ticksFree = 0;
    bt::BehaviorTree::Ptr strategy;
    bt::BehaviorTree::Ptr keeperTree;

    void notifyTreeStatus(bt::Node::Status status);
    void runOneLoopCycle();
    bool weHaveRobots = false;

    ai::StrategyManager strategyManager;
    std::string oldKeeperTreeName = "";
    std::string oldStrategyName = "";

    int publishSettingTicks= 0;

public:
    void start();
    void checkForShutdown();
  void checkForFreeRobots();
};

} // rtt

#endif //ROBOTEAM_AI_APPLICATIONMANAGER_H
