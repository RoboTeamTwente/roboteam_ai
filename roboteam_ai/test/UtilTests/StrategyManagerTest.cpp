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
    roboteam_msgs::RefereeStage stage;
    stage.stage=roboteam_msgs::RefereeStage::NORMAL_FIRST_HALF;

    strategyManager.setCurrentRefGameState(RefCommand::NORMAL_START,stage);
    EXPECT_EQ(strategyManager.getCurrentRefGameState().strategyName, "normal_play_strategy");

    strategyManager.setCurrentRefGameState(RefCommand::HALT,stage);
    EXPECT_EQ(strategyManager.getCurrentRefGameState().strategyName, "halt_strategy");

    // prepare command followed up by normal start should trigger followUpCommand
    strategyManager.setCurrentRefGameState(RefCommand::PREPARE_KICKOFF_US,stage);
    EXPECT_EQ(strategyManager.getCurrentRefGameState().strategyName, "kickoff_us_formation_strategy");

    strategyManager.setCurrentRefGameState(RefCommand::NORMAL_START,stage);
    EXPECT_EQ(strategyManager.getCurrentRefGameState().strategyName, "kickoff_shoot_strategy");

    strategyManager.setCurrentRefGameState(RefCommand::NORMAL_START,stage);
    EXPECT_EQ(strategyManager.getCurrentRefGameState().strategyName, "kickoff_shoot_strategy");

    // forcing a refgamestate should put it into normal start anyway
    strategyManager.forceCurrentRefGameState(RefCommand::NORMAL_START);
    EXPECT_EQ(strategyManager.getCurrentRefGameState().strategyName, "normal_play_strategy");

    // prepare command followed up by something else (i.e. command a) than normal start should trigger that command (command a).
    strategyManager.setCurrentRefGameState(RefCommand::PREPARE_KICKOFF_US,stage);
    EXPECT_EQ(strategyManager.getCurrentRefGameState().strategyName, "kickoff_us_formation_strategy");

    strategyManager.setCurrentRefGameState(RefCommand::HALT,stage);
    EXPECT_EQ(strategyManager.getCurrentRefGameState().strategyName, "halt_strategy");

    stage.stage=roboteam_msgs::RefereeStage::PENALTY_SHOOTOUT;
    strategyManager.setCurrentRefGameState(RefCommand::PREPARE_PENALTY_US,stage);
    EXPECT_EQ(strategyManager.getCurrentRefGameState().strategyName,"time_out_strategy");
    EXPECT_EQ(strategyManager.getCurrentRefGameState().keeperStrategyName,"shootout_prepare_tactic");

    strategyManager.setCurrentRefGameState(RefCommand::NORMAL_START,stage);
    EXPECT_EQ(strategyManager.getCurrentRefGameState().strategyName,"time_out_strategy");
    EXPECT_EQ(strategyManager.getCurrentRefGameState().keeperStrategyName,"shootout_shoot_tactic");

    strategyManager.setCurrentRefGameState(RefCommand::PREPARE_PENALTY_THEM,stage);
    EXPECT_EQ(strategyManager.getCurrentRefGameState().strategyName,"time_out_strategy");
    EXPECT_EQ(strategyManager.getCurrentRefGameState().keeperStrategyName,"keeper_penalty_prepare_tactic");

    strategyManager.setCurrentRefGameState(RefCommand::NORMAL_START,stage);
    EXPECT_EQ(strategyManager.getCurrentRefGameState().strategyName,"time_out_strategy");
    EXPECT_EQ(strategyManager.getCurrentRefGameState().keeperStrategyName,"keeper_default_tactic");




}
