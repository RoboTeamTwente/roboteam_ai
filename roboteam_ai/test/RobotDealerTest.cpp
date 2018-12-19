//
// Created by baris on 24/11/18.
//

#include <gtest/gtest.h>
#include "../src/utilities/RobotDealer.h"

TEST(RobotDealerTest, RobotDealerTest) {

    // Make sure that there is a world

    using dealer = robotDealer::RobotDealer;
    using robot = dealer::RobotDealer::RobotType;

    ASSERT_TRUE(dealer::getAvailableRobots().empty() != 0);

    auto robot1 = dealer::claimRobotForTactic(robot::random, "testing1", "role1");

    ASSERT_TRUE(robot1 == dealer::findRobotForRole("role1"));

}