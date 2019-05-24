//
// Created by baris on 24/11/18.
//

#include <gtest/gtest.h>
#include "roboteam_ai/src/utilities/RobotDealer.h"
#include "roboteam_ai/src/world/World.h"
#include "../helpers/FieldHelper.h"

TEST(RobotDealerTest, RobotDealerTest) {
    using robot = rtt::ai::robotDealer::RobotType;
    // Make sure that there is a world and that it is empty
    roboteam_msgs::World worldMsg;
    auto fieldMsg = testhelpers::FieldHelper::generateField();
    rtt::ai::world::field->set_field(fieldMsg);

    roboteam_msgs::WorldRobot robot1, robot2, robot3;
    roboteam_msgs::WorldBall ball;
    rtt::ai::world::world->updateWorld(worldMsg);
    rtt::ai::robotDealer::RobotDealer::refresh();

    rtt::ai::robotDealer::RobotDealer::removeTactic("free"); // This is necessary because previous tests create free robots
    EXPECT_TRUE(rtt::ai::robotDealer::RobotDealer::getAvailableRobots().empty());

    //fill the world
    robot1.id = 1;
    robot1.pos.x = 1;
    robot1.pos.y = 4;

    robot2.id = 2;
    robot2.pos.x = 1;
    robot2.pos.y = 2;

    robot3.id = 3;
    robot3.pos.x = 1;
    robot3.pos.y = -1;

    ball.pos.x = 0;
    ball.pos.y = 1;

    worldMsg.us.push_back(robot1);
    worldMsg.us.push_back(robot2);
    worldMsg.us.push_back(robot3);
    worldMsg.ball = ball;
    rtt::ai::world::world->updateWorld(worldMsg);
    rtt::ai::world::world->updateWorld(worldMsg);

    auto dealbot1 = rtt::ai::robotDealer::RobotDealer::claimRobotForTactic(robot::RANDOM, "role1", "testing1");
    EXPECT_TRUE(dealbot1==rtt::ai::robotDealer::RobotDealer::findRobotForRole("role1"));
    EXPECT_EQ(rtt::ai::robotDealer::RobotDealer::getTacticNameForRole("role1"), "testing1");
    EXPECT_EQ(rtt::ai::robotDealer::RobotDealer::getAvailableRobots().size(), (unsigned int) 2);

    rtt::ai::robotDealer::RobotDealer::claimRobotForTactic(robot::RANDOM, "role2", "testing1");
    EXPECT_EQ(rtt::ai::robotDealer::RobotDealer::getAvailableRobots().size(), (unsigned int) 1);
    auto claimedBots1 = rtt::ai::robotDealer::RobotDealer::getClaimedRobots()["testing1"];
    EXPECT_EQ(claimedBots1.size(), (unsigned int) 2);

    auto dealbot3 = rtt::ai::robotDealer::RobotDealer::claimRobotForTactic(robot::RANDOM, "role3", "testing1");
    rtt::ai::robotDealer::RobotDealer::releaseRobotForRole("role2");
    auto claimedBots2 = rtt::ai::robotDealer::RobotDealer::getClaimedRobots()["testing1"];
    EXPECT_NE(claimedBots1, claimedBots2);
    EXPECT_EQ(rtt::ai::robotDealer::RobotDealer::getRoleNameForId(dealbot3), "role3");
    EXPECT_EQ(dealbot3, rtt::ai::robotDealer::RobotDealer::findRobotForRole("role3"));
    std::set<int> set = {1, 3};
    EXPECT_EQ(set, rtt::ai::robotDealer::RobotDealer::findRobotsForTactic("testing1"));
    EXPECT_EQ(rtt::ai::robotDealer::RobotDealer::getTacticNameForId(1),"testing1");
    EXPECT_EQ(rtt::ai::robotDealer::RobotDealer::getTacticNameForId(2),"free");
    EXPECT_EQ(rtt::ai::robotDealer::RobotDealer::getRoleNameForId(2),"free");
    EXPECT_EQ(rtt::ai::robotDealer::RobotDealer::getTacticNameForRole("role2"),"");
    rtt::ai::robotDealer::RobotDealer::releaseRobotForRole("role1");
    rtt::ai::robotDealer::RobotDealer::releaseRobotForRole("role3");

    //TODO: test if these functionalities actually pick the right robots
    dealbot1=rtt::ai::robotDealer::RobotDealer::claimRobotForTactic(robot::BETWEEN_BALL_AND_OUR_GOAL, "role1", "testing1");
    EXPECT_TRUE(dealbot1==rtt::ai::robotDealer::RobotDealer::findRobotForRole("role1"));
    rtt::ai::robotDealer::RobotDealer::releaseRobotForRole("role1");
    dealbot1=rtt::ai::robotDealer::RobotDealer::claimRobotForTactic(robot::CLOSE_TO_BALL, "role1", "testing1");
    EXPECT_TRUE(dealbot1==rtt::ai::robotDealer::RobotDealer::findRobotForRole("role1"));
    rtt::ai::robotDealer::RobotDealer::releaseRobotForRole("role1");
    dealbot1=rtt::ai::robotDealer::RobotDealer::claimRobotForTactic(robot::CLOSE_TO_OUR_GOAL, "role1", "testing1");
    EXPECT_TRUE(dealbot1==rtt::ai::robotDealer::RobotDealer::findRobotForRole("role1"));
    rtt::ai::robotDealer::RobotDealer::releaseRobotForRole("role1");
    dealbot1=rtt::ai::robotDealer::RobotDealer::claimRobotForTactic(robot::CLOSE_TO_THEIR_GOAL, "role1", "testing1");
    EXPECT_TRUE(dealbot1==rtt::ai::robotDealer::RobotDealer::findRobotForRole("role1"));
    rtt::ai::robotDealer::RobotDealer::releaseRobotForRole("role1");
    dealbot1=rtt::ai::robotDealer::RobotDealer::claimRobotForTactic(robot::FAR_FROM_BALL, "role1", "testing1");
    EXPECT_TRUE(dealbot1==rtt::ai::robotDealer::RobotDealer::findRobotForRole("role1"));
    rtt::ai::robotDealer::RobotDealer::releaseRobotForRole("role1");

    rtt::ai::robotDealer::RobotDealer::removeTactic("testing1");
    EXPECT_EQ(rtt::ai::robotDealer::RobotDealer::getAvailableRobots().size(), (unsigned int) 3);
}
