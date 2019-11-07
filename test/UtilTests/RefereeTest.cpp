//
// Created by mrlukasbos on 14-11-18.
//

#include <roboteam_proto/messages_robocup_ssl_referee.pb.h>
#include <interface/api/Output.h>
#include "utilities/GameStateManager.hpp"
#include "gtest/gtest.h"

TEST(RefereeTest, it_gets_and_sets_the_ref) {
    proto::SSL_Referee refereeData;
    refereeData.set_command(proto::SSL_Referee_Command_PREPARE_KICKOFF_BLUE);
    rtt::ai::GameStateManager::setRefereeData(refereeData);

    EXPECT_EQ(rtt::ai::GameStateManager::getRefereeData().command(), proto::SSL_Referee_Command_PREPARE_KICKOFF_BLUE);

    refereeData.set_command(proto::SSL_Referee_Command_PREPARE_KICKOFF_YELLOW);
    rtt::ai::GameStateManager::setRefereeData(refereeData);

    EXPECT_EQ(rtt::ai::GameStateManager::getRefereeData().command(), proto::SSL_Referee_Command_PREPARE_KICKOFF_YELLOW);

    // this is necessary for this following test to work properly since it listens to the interface
    rtt::ai::interface::Output::setUseRefereeCommands(true);

    refereeData.set_stage(proto::SSL_Referee_Stage_PENALTY_SHOOTOUT);
    refereeData.set_command(proto::SSL_Referee_Command_PREPARE_PENALTY_YELLOW);
    rtt::ai::GameStateManager::setRefereeData(refereeData);

    EXPECT_EQ(rtt::ai::GameStateManager::getCurrentGameState().strategyName,"time_out_strategy");
    EXPECT_EQ(rtt::ai::GameStateManager::getCurrentGameState().keeperStrategyName,"keeper_penalty_prepare_tactic");

    refereeData.set_stage(proto::SSL_Referee_Stage_PENALTY_SHOOTOUT);
    refereeData.set_command(proto::SSL_Referee_Command_PREPARE_PENALTY_BLUE);
    rtt::ai::GameStateManager::setRefereeData(refereeData);

    EXPECT_EQ(rtt::ai::GameStateManager::getCurrentGameState().strategyName,"time_out_strategy");
    EXPECT_EQ(rtt::ai::GameStateManager::getCurrentGameState().keeperStrategyName,"shootout_prepare_tactic");

}