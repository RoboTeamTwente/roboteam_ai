//
// Created by rolf on 19-10-18.
//
#include <gtest/gtest.h>
#include "../../src/bt/bt.hpp"
#include "../../src/conditions/HasBall.hpp"
#include "../../src/utilities/World.h"
#include "../../src/utilities/RobotDealer.h"

TEST(BallTest, IHaveBallTest) {
    robotDealer::RobotDealer::halt();
    auto BB = std::make_shared<bt::Blackboard>();
    BB->setInt("ROBOT_ID", 0);
    BB->setString("ROLE","test");
    BB->setBool("our_team", false);
    rtt::ai::HasBall node("Test", BB);

    EXPECT_EQ(node.node_name(), "HasBall");

    //First test should fail as the robot is not set in the world state yet.
    EXPECT_EQ(node.update(), bt::Node::Status::Failure);

    roboteam_msgs::World worldMsg;
    roboteam_msgs::WorldRobot robot;

    robot.id = 0;
    robot.pos.x = 0;
    robot.pos.y = 0;
    worldMsg.us.push_back(robot);
    worldMsg.ball.pos.x = 0.1;
    worldMsg.ball.pos.y = 0.0;
    worldMsg.ball.visible = 1;
    rtt::ai::World::set_world(worldMsg);
    robotDealer::RobotDealer::claimRobotForTactic(robotDealer::RobotType::random,"IHaveBallTestTactic","test");
    EXPECT_EQ(node.update(), bt::Node::Status::Success);

    worldMsg.ball.pos.x = 0.0;
    worldMsg.ball.visible = 1;
    rtt::ai::World::set_world(worldMsg);
    EXPECT_EQ(node.update(), bt::Node::Status::Failure);

    //Test if angle checking works
    worldMsg.ball.pos.x = 0;
    worldMsg.ball.pos.y = 0.1;
    worldMsg.ball.visible = 1;
    rtt::ai::World::set_world(worldMsg);
    EXPECT_EQ(node.update(), bt::Node::Status::Failure);

    worldMsg.ball.pos.x = 0;
    worldMsg.ball.pos.y = - 0.1;
    worldMsg.ball.visible = 1;
    rtt::ai::World::set_world(worldMsg);
    EXPECT_EQ(node.update(), bt::Node::Status::Failure);

    worldMsg.ball.pos.x = - 0.1;
    worldMsg.ball.pos.y = 0;
    worldMsg.ball.visible = 1;
    rtt::ai::World::set_world(worldMsg);
    EXPECT_EQ(node.update(),bt::Node::Status::Failure);

    robotDealer::RobotDealer::removeTactic("IHaveBallTestTactic");
}