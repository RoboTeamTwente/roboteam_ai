//
// Created by mrlukasbos on 14-11-18.
//

#include <roboteam_msgs/RefereeData.h>
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
    refereeData.stage.stage=roboteam_msgs::RefereeStage::PENALTY_SHOOTOUT;
    refereeData.command.command = roboteam_msgs::RefereeCommand::PREPARE_PENALTY_US;

    EXPECT_EQ(rtt::ai::GameStateManager::getCurrentGameState().strategyName,"time_out_strategy");
    EXPECT_EQ(rtt::ai::GameStateManager::getCurrentGameState().keeperStrategyName,"");

    refereeData.stage.stage=roboteam_msgs::RefereeStage::PENALTY_SHOOTOUT;
    refereeData.command.command = roboteam_msgs::RefereeCommand::PREPARE_PENALTY_THEM;

    EXPECT_EQ(rtt::ai::GameStateManager::getCurrentGameState().strategyName,"time_out_strategy");
    EXPECT_EQ(rtt::ai::GameStateManager::getCurrentGameState().keeperStrategyName,"");

}