//
// Created by mrlukasbos on 14-1-19.
//

#ifndef ROBOTEAM_AI_APPLICATIONMANAGER_H
#define ROBOTEAM_AI_APPLICATIONMANAGER_H

#include <gtest/gtest_prod.h>
#include <include/roboteam_ai/bt/BehaviorTree.hpp>
#include "world/World.h"
#include "world/Field.h"

namespace rtt {

class ApplicationManager {
private:
    FRIEND_TEST(ApplicationManagerTest, it_handles_ROS_data);

    int ticksFree = 0;
    bt::BehaviorTree::Ptr strategy;
    bt::BehaviorTree::Ptr keeperTree;

    void notifyTreeStatus(bt::Node::Status status);
    void runOneLoopCycle();

    std::string oldKeeperTreeName = "";
    std::string oldStrategyName = "";

    int publishSettingTicks= 0;

    ai::world::World actualWorld;
    ai::world::Field actualField;


public:
    void setup();
    void loop();
    void checkForShutdown();
};

} // rtt

#endif //ROBOTEAM_AI_APPLICATIONMANAGER_H
