//
// Created by rolf on 19-10-18.
//
#include <gtest/gtest.h>
#include "../../src/bt/bt.hpp"
#include "../../src/conditions/HasBall.hpp"

TEST(BallTest, IHaveBallTest) {
    auto BB = std::make_shared<bt::Blackboard>();
    BB->setInt("ROBOT_ID", 2);
    BB->setBool("our_team", false);
    rtt::ai::HasBall node("Test", BB);
    //First test should fail as the robot is not set in the world state yet.
    ASSERT_EQ(node.update(), bt::Node::Status::Failure);

    roboteam_msgs::World worldMsg;
    roboteam_msgs::WorldRobot robot;

    robot.id = 2;
    robot.pos.x = 0;
    robot.pos.y = 0;
    worldMsg.them.push_back(robot);
    worldMsg.ball.pos.x = 0.1;
    worldMsg.ball.pos.y = 0.0;
    rtt::ai::World::set_world(worldMsg);
    ASSERT_EQ(node.update(), bt::Node::Status::Success);

    worldMsg.ball.pos.x = 0.2;
    rtt::ai::World::set_world(worldMsg);
    ASSERT_EQ(node.update(), bt::Node::Status::Failure);

    //Test if angle checking works
    worldMsg.ball.pos.x = 0;
    worldMsg.ball.pos.y = 0.1;
    rtt::ai::World::set_world(worldMsg);
    ASSERT_EQ(node.update(), bt::Node::Status::Failure);

    worldMsg.ball.pos.x = 0;
    worldMsg.ball.pos.y = - 0.1;
    rtt::ai::World::set_world(worldMsg);
    ASSERT_EQ(node.update(), bt::Node::Status::Failure);

    worldMsg.ball.pos.x = - 0.1;
    worldMsg.ball.pos.y = 0;
    rtt::ai::World::set_world(worldMsg);
    ASSERT_EQ(node.update(),bt::Node::Status::Failure);


}