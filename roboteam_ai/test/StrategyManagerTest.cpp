//
// Created by mrlukasbos on 14-11-18.
//

#include <roboteam_msgs/RobotCommand.h>
#include "../src/utilities/StrategyManager.h"
#include "gtest/gtest.h"

TEST(StrategyManagerTest, StrategyManagerTest) {
    rtt::ai::StrategyManager strategyManager;
    roboteam_msgs::RefereeCommand cmd;

    cmd.command = static_cast<int>(RefGameState::NORMAL_START);
    EXPECT_EQ(strategyManager.getCurrentStrategyName(cmd), "bigjson/NormalPlay");

    cmd.command = static_cast<int>(RefGameState::HALT);
    EXPECT_EQ(strategyManager.getCurrentStrategyName(cmd), "rtt_dennis/HaltStrategy");

    // prepare command followed up by normal start should trigger followUpCommand
    cmd.command = static_cast<int>(RefGameState::PREPARE_KICKOFF_US);
    EXPECT_EQ(strategyManager.getCurrentStrategyName(cmd), "rtt_emiel/PrepareKickoffUsStrategy");
    cmd.command = static_cast<int>(RefGameState::NORMAL_START);
    EXPECT_EQ(strategyManager.getCurrentStrategyName(cmd), "rtt_bob/KickoffWithChipStrategy");
    cmd.command = static_cast<int>(RefGameState::NORMAL_START);
    EXPECT_EQ(strategyManager.getCurrentStrategyName(cmd), "bigjson/NormalPlay");

    // prepare command followed up by something else (i.e. command a) than normal start should trigger that command (command a).
    cmd.command = static_cast<int>(RefGameState::PREPARE_KICKOFF_US);
    EXPECT_EQ(strategyManager.getCurrentStrategyName(cmd), "rtt_emiel/PrepareKickoffUsStrategy");
    cmd.command = static_cast<int>(RefGameState::HALT);
    EXPECT_EQ(strategyManager.getCurrentStrategyName(cmd), "rtt_dennis/HaltStrategy");
}
