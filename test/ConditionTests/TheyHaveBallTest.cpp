//
// Created by robzelluf on 10/24/18.
//

#include <gtest/gtest.h>
#include "../../src/conditions/TheyHaveBall.h"
#include "../../src/world/World.h"

TEST(TheyHaveBallTest, TheyHaveBallTest) {
    auto BB = std::make_shared<bt::Blackboard>();
    rtt::ai::TheyHaveBall node("TheyHaveBall", BB);

    EXPECT_EQ(node.node_name(), "TheyHaveBall");

    roboteam_msgs::World worldMsg;
    roboteam_msgs::WorldRobot robot;

    rtt::ai::world::world->updateWorld(worldMsg);

    robot.id = 0;
    robot.pos.x = -2;
    robot.pos.y = -2;
    robot.angle = 0;
    worldMsg.them.push_back(robot);

    worldMsg.ball.pos.x = 0.04;
    worldMsg.ball.pos.y = 0.0;
    worldMsg.ball.visible = 1;
    worldMsg.ball.existence = 99999;
    rtt::ai::world::world->updateWorld(worldMsg);

    EXPECT_EQ(node.update(), bt::Node::Status::Failure);

    roboteam_msgs::WorldRobot robot2;

    robot2.id = 1;
    robot2.pos.x = 0;
    robot2.pos.y = 0;
    robot2.angle = 0;
    worldMsg.them.push_back(robot2);

    rtt::ai::world::world->updateWorld(worldMsg);

    EXPECT_EQ(node.update(), bt::Node::Status::Success);
}

