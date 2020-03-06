//
// Created by mrlukasbos on 14-1-19.
//

#ifndef ROBOTEAM_AI_APPLICATIONMANAGER_H
#define ROBOTEAM_AI_APPLICATIONMANAGER_H

#include <gtest/gtest_prod.h>
#include <roboteam_utils/Timer.h>
#include <utilities/StrategyManager.h>
#include "analysis/play-utilities/PlayChecker.h"
#include "analysis/play-utilities/PlayDecider.h"
#include "treeinterp/BTFactory.h"
#include "utilities/IOManager.h"
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
    bool fieldInitialized = false;
    bool robotsInitialized = false;

    /**
     * Checks which plays are valid out of all the plays
     */
    rtt::ai::analysis::PlayChecker playChecker;
    /**
     * Checks, out of the valid plays, which play is the best to choose
     */
    rtt::ai::analysis::PlayDecider playDecider;
    /**
     * Function that decides whether to change plays given a world and field.
     * @param world the current world state
     * @param field the current field state
     */
    void decidePlay(rtt::ai::world::World* world, const rtt::ai::world::Field& field);

   public:
    void start();
    void checkForShutdown();
    void checkForFreeRobots();
    void updateCoaches(const ai::world::Field & field) const;
    void updateTrees();
    bt::Node::Status runStrategyTree(const ai::world::Field & field);
    void runKeeperTree(const ai::world::Field & field);
};

}  // namespace rtt

#endif  // ROBOTEAM_AI_APPLICATIONMANAGER_H
