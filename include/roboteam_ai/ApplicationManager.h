//
// Created by mrlukasbos on 14-1-19.
//

#ifndef ROBOTEAM_AI_APPLICATIONMANAGER_H
#define ROBOTEAM_AI_APPLICATIONMANAGER_H

#include <gtest/gtest_prod.h>
#include <utilities/StrategyManager.h>
#include <roboteam_utils/Timer.h>
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
    std::string oldKeeperTreeName = "";
    std::string oldStrategyName = "";
    rtt::Settings* settings;

public:
    void start(Settings& settings);
    void checkForShutdown();
    void checkForFreeRobots();
    void updateCoaches() const;
    void updateTrees();
    bt::Node::Status runStrategyTree();
    void runKeeperTree();
};

} // rtt

#endif //ROBOTEAM_AI_APPLICATIONMANAGER_H
