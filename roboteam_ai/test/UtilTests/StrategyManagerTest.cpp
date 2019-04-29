//
// Created by mrlukasbos on 14-11-18.
//

#include <roboteam_msgs/RobotCommand.h>
#include "roboteam_ai/src/utilities/StrategyManager.h"
#include "gtest/gtest.h"

TEST(StrategyManagerTest, StrategyManagerTest) {
    rtt::ai::StrategyManager strategyManager;
    roboteam_msgs::RefereeCommand cmd;

    cmd.command = static_cast<int>(RefGameState::NORMAL_START);
    EXPECT_EQ(strategyManager.getCurrentStrategyName(cmd), "TestStrategy");

    cmd.command = static_cast<int>(RefGameState::HALT);
    EXPECT_EQ(strategyManager.getCurrentStrategyName(cmd), "halt_strategy");

    // prepare command followed up by normal start should trigger followUpCommand
    cmd.command = static_cast<int>(RefGameState::PREPARE_KICKOFF_US);
    EXPECT_EQ(strategyManager.getCurrentStrategyName(cmd), "kickoff_us_formation_strategy");
    cmd.command = static_cast<int>(RefGameState::NORMAL_START);
    EXPECT_EQ(strategyManager.getCurrentStrategyName(cmd), "kickoff_shoot_strategy");
    cmd.command = static_cast<int>(RefGameState::NORMAL_START);
    EXPECT_EQ(strategyManager.getCurrentStrategyName(cmd), "kickoff_shoot_strategy");

    // prepare command followed up by something else (i.e. command a) than normal start should trigger that command (command a).
    cmd.command = static_cast<int>(RefGameState::PREPARE_KICKOFF_US);
    EXPECT_EQ(strategyManager.getCurrentStrategyName(cmd), "kickoff_us_formation_strategy");
    cmd.command = static_cast<int>(RefGameState::HALT);
    EXPECT_EQ(strategyManager.getCurrentStrategyName(cmd), "halt_strategy");
}
