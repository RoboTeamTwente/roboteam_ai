//
// Created by mrlukasbos on 14-11-18.
//

#include <roboteam_msgs/RefereeData.h>
#include <roboteam_ai/src/interface/api/Output.h>
#include "roboteam_ai/src/utilities/GameStateManager.hpp"
#include "gtest/gtest.h"

TEST(RefereeTest, it_gets_and_sets_the_ref) {
    roboteam_msgs::RefereeData refereeData;
    refereeData.command.command = 33;
    rtt::ai::GameStateManager::setRefereeData(refereeData);

    EXPECT_EQ(rtt::ai::GameStateManager::getRefereeData().command.command, 33);

    refereeData.command.command = 123;
    rtt::ai::GameStateManager::setRefereeData(refereeData);

    EXPECT_EQ(rtt::ai::GameStateManager::getRefereeData().command.command, 123);

    // this is necessary for this following test to work properly since it listens to the interface
    rtt::ai::interface::Output::setUseRefereeCommands(true);

    refereeData.stage.stage=roboteam_msgs::RefereeStage::PENALTY_SHOOTOUT;
    refereeData.command.command = roboteam_msgs::RefereeCommand::PREPARE_PENALTY_US;
    rtt::ai::GameStateManager::setRefereeData(refereeData);

    EXPECT_EQ(rtt::ai::GameStateManager::getCurrentGameState().strategyName,"time_out_strategy");
    EXPECT_EQ(rtt::ai::GameStateManager::getCurrentGameState().keeperStrategyName,"shootout_prepare_tactic");

    refereeData.stage.stage=roboteam_msgs::RefereeStage::PENALTY_SHOOTOUT;
    refereeData.command.command = roboteam_msgs::RefereeCommand::PREPARE_PENALTY_THEM;
    rtt::ai::GameStateManager::setRefereeData(refereeData);

    EXPECT_EQ(rtt::ai::GameStateManager::getCurrentGameState().strategyName,"time_out_strategy");
    EXPECT_EQ(rtt::ai::GameStateManager::getCurrentGameState().keeperStrategyName,"keeper_penalty_prepare_tactic");

}