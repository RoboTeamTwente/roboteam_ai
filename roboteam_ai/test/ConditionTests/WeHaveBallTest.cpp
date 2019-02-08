//
// Created by robzelluf on 10/25/18.
//

#include <gtest/gtest.h>
#include <roboteam_ai/test/helpers/WorldHelper.h>
#include "../../src/conditions/WeHaveBall.h"
#include "../../src/utilities/World.h"

TEST(WeHaveBallTest, WeHaveBallTest) {
    auto BB = std::make_shared<bt::Blackboard>();
    rtt::ai::WeHaveBall node("Test", BB);

    EXPECT_EQ(node.node_name(), "WeHaveBall");

    roboteam_msgs::World worldMsg;
    roboteam_msgs::WorldRobot robot;
    rtt::ai::World::set_world(worldMsg);
    EXPECT_EQ(node.update(), bt::Node::Status::Failure);

    roboteam_msgs::GeometryFieldSize field;
    field.field_length = 12;
    field.field_width = 9;

    // expect failure when a world message is generated with a random ball
    rtt::ai::World::set_world(testhelpers::WorldHelper::getWorldMsg(3, 3, true, field));
    EXPECT_EQ(node.update(), bt::Node::Status::Failure);

    // get world message where robot has ball.
    auto generated = testhelpers::WorldHelper::getWorldMsgWhereRobotHasBall(3, 3, true, field);
    auto msg = generated.first;
    rtt::ai::World::set_world(msg);

    EXPECT_EQ(node.update(), bt::Node::Status::Success);
}