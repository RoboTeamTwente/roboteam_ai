//
// Created by rolf on 19-10-18.
//
#include <gtest/gtest.h>
#include "include/roboteam_ai/conditions/HasBall.hpp"
#include "include/roboteam_ai/world/World.h"
#include "include/roboteam_ai/world/Ball.h"
#include "include/roboteam_ai/utilities/RobotDealer.h"

TEST(BallTest, IHaveBallTest) {
    roboteam_msgs::World worldMsg;
    roboteam_msgs::WorldRobot robot;
    rtt::ai::world::world->updateWorld(worldMsg);
    rtt::ai::world::Ball::exists = false;

    rtt::ai::robotDealer::RobotDealer::halt();
    auto BB = std::make_shared<bt::Blackboard>();
    BB->setInt("ROBOT_ID", 0);
    BB->setString("ROLE","test");
    BB->setBool("our_team", false);
    rtt::ai::HasBall node("HasBall", BB);

    EXPECT_EQ(node.node_name(), "HasBall");

    //First test should fail as the robot is not set in the world state yet.
    EXPECT_EQ(node.update(), bt::Node::Status::Failure);

    robot.id = 0;
    robot.pos.x = 0;
    robot.pos.y = 0;
    worldMsg.us.push_back(robot);
    worldMsg.ball.pos.x = 0.1;
    worldMsg.ball.pos.y = 0.0;
    worldMsg.ball.visible = 1;
    worldMsg.ball.existence = 99999;
    rtt::ai::world::world->updateWorld(worldMsg);
    rtt::ai::robotDealer::RobotDealer::claimRobotForTactic(
            rtt::ai::robotDealer::RobotType::RANDOM, "test", "IHaveBallTestTactic");
    node.initialize();
    EXPECT_EQ(node.update(), bt::Node::Status::Success);

    worldMsg.ball.pos.x = 0.0;
    worldMsg.ball.visible = 1;
    worldMsg.ball.existence = 99999;
    rtt::ai::world::world->updateWorld(worldMsg);
    EXPECT_EQ(node.update(), bt::Node::Status::Success);

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

    rtt::ai::robotDealer::RobotDealer::removeTactic("IHaveBallTestTactic");
}