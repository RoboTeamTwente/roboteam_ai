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
}