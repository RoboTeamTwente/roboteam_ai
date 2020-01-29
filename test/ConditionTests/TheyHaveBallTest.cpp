//
// Created by robzelluf on 10/24/18.
//

#include <gtest/gtest.h>
#include "conditions/TheyHaveBall.h"
#include "world/World.h"

TEST(TheyHaveBallTest, TheyHaveBallTest) {
    //    auto BB = std::make_shared<bt::Blackboard>();
    //    rtt::ai::TheyHaveBall node("TheyHaveBall", BB);
    //
    //    EXPECT_EQ(node.node_name(), "TheyHaveBall");
    //
    //    proto::World worldMsg;
    //    proto::WorldRobot robot;
    //
    //    rtt::ai::world::world->updateWorld(worldMsg);
    //
    //    robot.set_id(0);
    //    robot.mutable_pos()->set_x(-2);
    //    robot.mutable_pos()->set_y(-2);
    //    worldMsg.them.push_back(robot);
    //
    //    worldMsg.ball.pos.x = 0.04;
    //    worldMsg.ball.pos.y = 0.0;
    //    worldMsg.ball.visible = 1;
    //    worldMsg.ball.existence = 99999;
    //    rtt::ai::world::world->updateWorld(worldMsg);
    //
    //    EXPECT_EQ(node.update(), bt::Node::Status::Failure);
    //
    //    proto::WorldRobot robot2;
    //
    //    robot2.id = 1;
    //    robot2.pos.x = 0;
    //    robot2.pos.y = 0;
    //    robot2.angle = 0;
    //    worldMsg.them.push_back(robot2);
    //
    //    rtt::ai::world::world->updateWorld(worldMsg);
    //
    //    EXPECT_EQ(node.update(), bt::Node::Status::Success);
}
