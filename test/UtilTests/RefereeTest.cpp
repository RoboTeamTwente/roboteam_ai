//
// Created by mrlukasbos on 14-11-18.
//

#include <gtest/gtest.h>
#include <interface/api/Output.h>
#include <proto/messages_robocup_ssl_referee.pb.h>

#include <world/World.hpp>

#include "helpers/FieldHelper.h"
#include "helpers/WorldHelper.h"
#include "utilities/GameStateManager.hpp"

TEST(RefereeTest, it_gets_and_sets_the_ref) {
    auto world = testhelpers::WorldHelper::getWorldMsg(11, 11, true, testhelpers::FieldHelper::generateField());
    auto const& [_, worldPtr] = rtt::world::World::instance();
    worldPtr->updateWorld(world);
    proto::SSL_Referee refereeData;
    refereeData.set_command(proto::SSL_Referee_Command_PREPARE_KICKOFF_BLUE);
    rtt::ai::GameStateManager::setRefereeData(refereeData, worldPtr);

    EXPECT_EQ(rtt::ai::GameStateManager::getRefereeData().command(), proto::SSL_Referee_Command_PREPARE_KICKOFF_BLUE);

    refereeData.set_command(proto::SSL_Referee_Command_PREPARE_KICKOFF_YELLOW);
    rtt::ai::GameStateManager::setRefereeData(refereeData, worldPtr);

    EXPECT_EQ(rtt::ai::GameStateManager::getRefereeData().command(), proto::SSL_Referee_Command_PREPARE_KICKOFF_YELLOW);

    // this is necessary for this following test to work properly since it listens to the interface
    rtt::ai::interface::Output::setUseRefereeCommands(true);

    refereeData.set_stage(proto::SSL_Referee_Stage_PENALTY_SHOOTOUT);
    refereeData.set_command(proto::SSL_Referee_Command_PREPARE_PENALTY_YELLOW);
    rtt::ai::GameStateManager::setRefereeData(refereeData, worldPtr);

    EXPECT_EQ(rtt::ai::GameStateManager::getCurrentGameState().getStrategyName(), "penalty_us_prepare");

    refereeData.set_stage(proto::SSL_Referee_Stage_PENALTY_SHOOTOUT);
    refereeData.set_command(proto::SSL_Referee_Command_PREPARE_PENALTY_BLUE);
    rtt::ai::GameStateManager::setRefereeData(refereeData, worldPtr);

    EXPECT_EQ(rtt::ai::GameStateManager::getCurrentGameState().getStrategyName(), "penalty_them_prepare");
}