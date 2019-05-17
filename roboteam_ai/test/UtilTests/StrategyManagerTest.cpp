//
// Created by mrlukasbos on 14-11-18.
//

#include <roboteam_msgs/RobotCommand.h>
#include <roboteam_msgs/RefereeCommand.h>
#include "roboteam_ai/src/utilities/StrategyManager.h"
#include "gtest/gtest.h"

TEST(StrategyManagerTest, StrategyManagerTest) {
    rtt::ai::StrategyManager strategyManager;
    roboteam_msgs::RefereeCommand cmd;

    strategyManager.setCurrentRefGameState(RefCommand::NORMAL_START);
    EXPECT_EQ(strategyManager.getCurrentRefGameState().strategyName, "normal_play_strategy");

    strategyManager.setCurrentRefGameState(RefCommand::HALT);
    EXPECT_EQ(strategyManager.getCurrentRefGameState().strategyName, "halt_strategy");

    // prepare command followed up by normal start should trigger followUpCommand
    strategyManager.setCurrentRefGameState(RefCommand::PREPARE_KICKOFF_US);
    EXPECT_EQ(strategyManager.getCurrentRefGameState().strategyName, "kickoff_us_formation_strategy");

    strategyManager.setCurrentRefGameState(RefCommand::NORMAL_START);
    EXPECT_EQ(strategyManager.getCurrentRefGameState().strategyName, "kickoff_shoot_strategy");

    strategyManager.setCurrentRefGameState(RefCommand::NORMAL_START);
    EXPECT_EQ(strategyManager.getCurrentRefGameState().strategyName, "kickoff_shoot_strategy");

    // prepare command followed up by something else (i.e. command a) than normal start should trigger that command (command a).
    strategyManager.setCurrentRefGameState(RefCommand::PREPARE_KICKOFF_US);
    EXPECT_EQ(strategyManager.getCurrentRefGameState().strategyName, "kickoff_us_formation_strategy");

    strategyManager.setCurrentRefGameState(RefCommand::HALT);
    EXPECT_EQ(strategyManager.getCurrentRefGameState().strategyName, "halt_strategy");
}
