//
// Created by robzelluf on 10/25/18.
//

#include <gtest/gtest.h>
#include "../../src/conditions/WeHaveBall.h"

TEST(WeHaveBallTest, WeHaveBallTest) {
    auto BB = std::make_shared<bt::Blackboard>();
    rtt::ai::WeHaveBall node("Test", BB);

    roboteam_msgs::World worldMsg;
    roboteam_msgs::WorldRobot robot;

    rtt::ai::World::set_world(worldMsg);

    ASSERT_EQ(node.update(), bt::Node::Status::Failure);

    robot.id = 0;
    robot.pos.x = -2;
    robot.pos.y = -2;
    robot.angle = 0;
    worldMsg.us.push_back(robot);

    worldMsg.ball.pos.x = 0.13;
    worldMsg.ball.pos.y = 0.0;
    rtt::ai::World::set_world(worldMsg);

    ASSERT_EQ(node.update(), bt::Node::Status::Failure);

    roboteam_msgs::WorldRobot robot2;

    robot2.id = 1;
    robot2.pos.x = 0;
    robot2.pos.y = 0;
    robot2.angle = 0;
    worldMsg.us.push_back(robot2);

    rtt::ai::World::set_world(worldMsg);

    ASSERT_EQ(node.update(), bt::Node::Status::Success);
}