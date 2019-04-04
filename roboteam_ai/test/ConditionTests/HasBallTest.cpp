//
// Created by rolf on 19-10-18.
//
#include <gtest/gtest.h>
#include "../../src/bt/bt.hpp"
#include "../../src/conditions/HasBall.hpp"
#include "../../src/world/World.h"
#include "../../src/utilities/RobotDealer.h"

TEST(BallTest, IHaveBallTest) {
    roboteam_msgs::World worldMsg;
    roboteam_msgs::WorldRobot robot;

    rtt::ai::world::world->updateWorld(worldMsg);

    std::string roleName = "test";
    rtt::ai::robotDealer::robotDealer->halt();
    auto BB = std::make_shared<bt::Blackboard>();
    BB->setInt("ROBOT_ID", 0);
    BB->setString("ROLE",roleName);
    BB->setBool("our_team", false);
    rtt::ai::HasBall node(roleName, BB);
    node.properties->setString("ROLE",roleName);
    EXPECT_EQ(node.node_name(), "HasBall");
    node.initialize();
    //First test should fail as the robot is not set in the world state yet.
    EXPECT_EQ(node.update(), bt::Node::Status::Failure);

    robot.id = 0;
    robot.pos.x = 0;
    robot.pos.y = 0;
    robot.angle = 0;
    worldMsg.us.push_back(robot);
    worldMsg.ball.pos.x = 0.1;
    worldMsg.ball.pos.y = 0.0;
    worldMsg.ball.visible = 1;
    worldMsg.ball.existence = 1;

    rtt::ai::world::world->updateWorld(worldMsg);

    rtt::ai::robotDealer::robotDealer->claimRobotForTactic(rtt::ai::robotDealer::RobotType::RANDOM, roleName, "IHaveBallTestTactic");
    std::cout << rtt::ai::robotDealer::robotDealer->findRobotForRole(roleName) << std::endl;
    EXPECT_EQ(node.update(), bt::Node::Status::Success);

    worldMsg.ball.pos.x = 0.0;
    worldMsg.ball.visible = 1;
    worldMsg.ball.existence = 1;

    rtt::ai::world::world->updateWorld(worldMsg);

    EXPECT_EQ(node.update(), bt::Node::Status::Failure);

    //Test if angle checking works
    worldMsg.ball.pos.x = 0;
    worldMsg.ball.pos.y = 0.1;
    worldMsg.ball.visible = 1;
    worldMsg.ball.existence = 99999;

    rtt::ai::world::world->updateWorld(worldMsg);

    EXPECT_EQ(node.update(), bt::Node::Status::Failure);

    worldMsg.ball.pos.x = 0;
    worldMsg.ball.pos.y = - 0.1;
    worldMsg.ball.visible = 1;
    worldMsg.ball.existence = 99999;
    rtt::ai::world::world->updateWorld(worldMsg);
    EXPECT_EQ(node.update(), bt::Node::Status::Failure);

    worldMsg.ball.pos.x = - 0.1;
    worldMsg.ball.pos.y = 0;
    worldMsg.ball.visible = 1;
    worldMsg.ball.existence = 99999;
    rtt::ai::world::world->updateWorld(worldMsg);
    EXPECT_EQ(node.update(),bt::Node::Status::Failure);

    rtt::ai::robotDealer::robotDealer->removeTactic("IHaveBallTestTactic");
}