//
// Created by mrlukasbos on 14-1-19.
//

#ifndef ROBOTEAM_AI_APPLICATIONMANAGER_H
#define ROBOTEAM_AI_APPLICATIONMANAGER_H

#include <gtest/gtest_prod.h>
#include <roboteam_utils/Timer.h>
#include <utilities/StrategyManager.h>

#include <include/roboteam_ai/stp/PlayChecker.hpp>
#include <include/roboteam_ai/stp/PlayDecider.hpp>
#include <include/roboteam_ai/interface/widgets/mainWindow.h>

#include "treeinterp/BTFactory.h"
#include "utilities/IOManager.h"
namespace rtt {

class ApplicationManager {
public:
    explicit ApplicationManager(ai::interface::MainWindow* mainWindow);

    std::vector<std::unique_ptr<rtt::ai::stp::Play>> plays;
    int programIndex{};

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
    ai::interface::MainWindow* mainWindow;

    /**
     * Current best play as picked by checker + decider
     */
    ai::stp::Play* currentPlay{nullptr};

    /**
     * Checks which plays are valid out of all the plays
     */
    rtt::ai::stp::PlayChecker playChecker;
    /**
     * Checks, out of the valid plays, which play is the best to choose
     */
    rtt::ai::stp::PlayDecider playDecider;
    /**
     * Function that decides whether to change plays given a world and field.
     * @param _world the current world state
     * @param field the current field state
     */
    void decidePlay(world_new::World* _world);

   public:
    void start();
    void checkForShutdown();
    void checkForFreeRobots();
    void updateCoaches(const ai::world::Field& field) const;
    void updateTrees();
    bt::Node::Status runStrategyTree(const ai::world::Field& field);
    void runKeeperTree(const ai::world::Field& field);
};

}  // namespace rtt

#endif  // ROBOTEAM_AI_APPLICATIONMANAGER_H
