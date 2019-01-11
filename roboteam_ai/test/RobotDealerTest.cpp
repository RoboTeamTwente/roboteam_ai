//
// Created by baris on 24/11/18.
//

#include <gtest/gtest.h>
#include "../src/utilities/RobotDealer.h"
#include "../src/utilities/World.h"

namespace {
TEST(RobotDealerTest, RobotDealerTest) {
    using dealer = robotDealer::RobotDealer;
    using robot = robotDealer::RobotType;
    // Make sure that there is a world and that it is empty
    roboteam_msgs::World worldMsg;
    roboteam_msgs::WorldRobot robot1, robot2, robot3;
    rtt::ai::World::set_world(worldMsg);

    dealer::removeTactic("free"); // This is necessary because previous tests create free robots
    ASSERT_TRUE(dealer::getAvailableRobots().empty());

    //fill the world
    robot1.id = 1;
    robot2.id = 2;
    robot3.id = 3;
    worldMsg.us.push_back(robot1);
    worldMsg.us.push_back(robot2);
    worldMsg.us.push_back(robot3);
    rtt::ai::World::set_world(worldMsg);

    auto dealbot1 = dealer::claimRobotForTactic(robot::random, "testing1", "role1");

    EXPECT_TRUE(dealbot1==dealer::findRobotForRole("role1"));
    EXPECT_EQ(dealer::getTacticNameForRole("role1"), "testing1");
    EXPECT_EQ(dealer::getAvailableRobots().size(), (unsigned int) 2);
    dealer::claimRobotForTactic(robot::random, "testing1", "role2");
    EXPECT_EQ(dealer::getAvailableRobots().size(), (unsigned int) 1);
    auto claimedBots1 = dealer::getClaimedRobots()["testing1"];
    EXPECT_EQ(claimedBots1.size(), (unsigned int) 2);
    auto dealbot3 = dealer::claimRobotForTactic(robot::random, "testing1", "role3");
    dealer::releaseRobotForRole("role2");
    auto claimedBots2 = dealer::getClaimedRobots()["testing1"];
    EXPECT_NE(claimedBots1, claimedBots2);
    EXPECT_EQ(dealer::getRoleNameForId(dealbot3), "role3");
    EXPECT_EQ(dealbot3, dealer::findRobotForRole("role3"));
    std::set<int> set = {1, 3};
    EXPECT_EQ(set, dealer::findRobotsForTactic("testing1"));
    dealer::removeTactic("testing1");
    EXPECT_EQ(dealer::getAvailableRobots().size(), (unsigned int) 3);
}
}