//
// Created by baris on 24/11/18.
//

#include <gtest/gtest.h>
#include "roboteam_ai/src/utilities/RobotDealer.h"
#include "roboteam_ai/src/world/World.h"

TEST(RobotDealerTest, RobotDealerTest) {
    using robot = rtt::ai::robotDealer::RobotType;
    // Make sure that there is a world and that it is empty
    roboteam_msgs::World worldMsg;
    roboteam_msgs::WorldRobot robot1, robot2, robot3;
    rtt::ai::world::world->updateWorld(worldMsg);

    rtt::ai::robotDealer::robotDealer->removeTactic("free"); // This is necessary because previous tests create free robots
    EXPECT_TRUE(rtt::ai::robotDealer::robotDealer->getAvailableRobots().empty());

    //fill the world
    robot1.id = 1;
    robot2.id = 2;
    robot3.id = 3;
    worldMsg.us.push_back(robot1);
    worldMsg.us.push_back(robot2);
    worldMsg.us.push_back(robot3);
    rtt::ai::world::world->updateWorld(worldMsg);

    auto dealbot1 = rtt::ai::robotDealer::robotDealer->claimRobotForTactic(robot::random, "testing1", "role1");

    EXPECT_TRUE(dealbot1==rtt::ai::robotDealer::robotDealer->findRobotForRole("role1"));
    EXPECT_EQ(rtt::ai::robotDealer::robotDealer->getTacticNameForRole("role1"), "testing1");
    EXPECT_EQ(rtt::ai::robotDealer::robotDealer->getAvailableRobots().size(), (unsigned int) 2);
    rtt::ai::robotDealer::robotDealer->claimRobotForTactic(robot::random, "testing1", "role2");
    EXPECT_EQ(rtt::ai::robotDealer::robotDealer->getAvailableRobots().size(), (unsigned int) 1);
    auto claimedBots1 = rtt::ai::robotDealer::robotDealer->getClaimedRobots()["testing1"];
    EXPECT_EQ(claimedBots1.size(), (unsigned int) 2);
    auto dealbot3 = rtt::ai::robotDealer::robotDealer->claimRobotForTactic(robot::random, "testing1", "role3");
    rtt::ai::robotDealer::robotDealer->releaseRobotForRole("role2");
    auto claimedBots2 = rtt::ai::robotDealer::robotDealer->getClaimedRobots()["testing1"];
    EXPECT_NE(claimedBots1, claimedBots2);
    EXPECT_EQ(rtt::ai::robotDealer::robotDealer->getRoleNameForId(dealbot3), "role3");
    EXPECT_EQ(dealbot3, rtt::ai::robotDealer::robotDealer->findRobotForRole("role3"));
    std::set<int> set = {1, 3};
    EXPECT_EQ(set, rtt::ai::robotDealer::robotDealer->findRobotsForTactic("testing1"));
    EXPECT_EQ(rtt::ai::robotDealer::robotDealer->getTacticNameForId(1),"testing1");
    EXPECT_EQ(rtt::ai::robotDealer::robotDealer->getTacticNameForId(2),"free");
    EXPECT_EQ(rtt::ai::robotDealer::robotDealer->getRoleNameForId(2),"free");
    EXPECT_EQ(rtt::ai::robotDealer::robotDealer->getTacticNameForRole("role2"),"");
    rtt::ai::robotDealer::robotDealer->releaseRobotForRole("role1");
    rtt::ai::robotDealer::robotDealer->releaseRobotForRole("role3");

    //TODO: test if these functionalities actually pick the right robots
    dealbot1=rtt::ai::robotDealer::robotDealer->claimRobotForTactic(robot::betweenBallAndOurGoal,"testing1","role1");
    EXPECT_TRUE(dealbot1==rtt::ai::robotDealer::robotDealer->findRobotForRole("role1"));
    rtt::ai::robotDealer::robotDealer->releaseRobotForRole("role1");
    dealbot1=rtt::ai::robotDealer::robotDealer->claimRobotForTactic(robot::closeToBall,"testing1","role1");
    EXPECT_TRUE(dealbot1==rtt::ai::robotDealer::robotDealer->findRobotForRole("role1"));
    rtt::ai::robotDealer::robotDealer->releaseRobotForRole("role1");
    dealbot1=rtt::ai::robotDealer::robotDealer->claimRobotForTactic(robot::closeToOurGoal,"testing1","role1");
    EXPECT_TRUE(dealbot1==rtt::ai::robotDealer::robotDealer->findRobotForRole("role1"));
    rtt::ai::robotDealer::robotDealer->releaseRobotForRole("role1");
    dealbot1=rtt::ai::robotDealer::robotDealer->claimRobotForTactic(robot::closeToTheirGoal,"testing1","role1");
    EXPECT_TRUE(dealbot1==rtt::ai::robotDealer::robotDealer->findRobotForRole("role1"));
    rtt::ai::robotDealer::robotDealer->releaseRobotForRole("role1");
    dealbot1=rtt::ai::robotDealer::robotDealer->claimRobotForTactic(robot::farFromBall,"testing1","role1");
    EXPECT_TRUE(dealbot1==rtt::ai::robotDealer::robotDealer->findRobotForRole("role1"));
    rtt::ai::robotDealer::robotDealer->releaseRobotForRole("role1");

    rtt::ai::robotDealer::robotDealer->removeTactic("testing1");
    EXPECT_EQ(rtt::ai::robotDealer::robotDealer->getAvailableRobots().size(), (unsigned int) 3);
}
