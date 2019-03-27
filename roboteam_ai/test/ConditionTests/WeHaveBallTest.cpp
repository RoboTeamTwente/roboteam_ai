//
// Created by robzelluf on 10/25/18.
//

#include <gtest/gtest.h>
#include <roboteam_ai/test/helpers/WorldHelper.h>
#include "../../src/conditions/WeHaveBall.h"
#include "../../src/world/World.h"

TEST(WeHaveBallTest, WeHaveBallTest) {
    auto BB = std::make_shared<bt::Blackboard>();
    rtt::ai::WeHaveBall node("Test", BB);

    EXPECT_EQ(node.node_name(), "WeHaveBall");

    roboteam_msgs::World worldMsg;
    roboteam_msgs::WorldRobot robot;
    rtt::ai::world::world->setWorld(worldMsg);

    roboteam_msgs::GeometryFieldSize field;
    field.field_length = 12;
    field.field_width = 9;

    // expect failure when a world message is generated with a random ball
    rtt::ai::world::world->setWorld(testhelpers::WorldHelper::getWorldMsg(3, 3, true, field));
    EXPECT_EQ(node.update(), bt::Node::Status::Failure);

    // get world message where robot has ball.
    auto generated = testhelpers::WorldHelper::getWorldMsgWhereRobotHasBall(3, 3, true, field);
    auto msg = generated.first;
    rtt::ai::world::world->setWorld(msg);

    EXPECT_EQ(node.update(), bt::Node::Status::Success);
}