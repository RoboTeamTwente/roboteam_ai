//
// Created by robzelluf on 10/25/18.
//

#include <gtest/gtest.h>
#include "../../src/conditions/IsOnOurSide.h"

#include "../../src/utilities/Field.h"

TEST(DetectsBallOnOurSide, IsOnOurSideTest) {
    bt::Blackboard BB;
    BB.setBool("robot", false);

    auto blackBoard = std::make_shared<bt::Blackboard>(BB);
    rtt::ai::IsOnOurSide node("Test", blackBoard);

    ASSERT_EQ(node.update(),bt::Node::Status::Failure);

    roboteam_msgs::World worldMsg;

    roboteam_msgs::GeometryFieldSize field;

    field.field_length = 3.0;
    field.field_width = 2.0;

    rtt::ai::Field::set_field(field);

    worldMsg.ball.pos.x = -1.0;
    worldMsg.ball.pos.y = 0.0;

    rtt::ai::World::set_world(worldMsg);

    ASSERT_EQ(node.update(), bt::Node::Status::Success);

    worldMsg.ball.pos.x = 1.0;
    rtt::ai::World::set_world(worldMsg);

    ASSERT_EQ(node.update(), bt::Node::Status::Failure);
}

TEST(DetectsRobotOnOurSide, IsOnOurSideTest) {
    bt::Blackboard BB;
    BB.setInt("ROBOT_ID", 2);
    BB.setBool("robot", true);

    auto blackBoard = std::make_shared<bt::Blackboard>(BB);
    rtt::ai::IsOnOurSide node("Test", blackBoard);

    ASSERT_EQ(node.update(),bt::Node::Status::Failure);

    roboteam_msgs::World worldMsg;

    roboteam_msgs::GeometryFieldSize field;
    roboteam_msgs::WorldRobot robot;

    field.field_length = 3.0;
    field.field_width = 2.0;

    rtt::ai::Field::set_field(field);

    robot.id = 2;
    robot.pos.x = -1.0;
    robot.pos.y = 0.0;

    worldMsg.us.push_back(robot);

    rtt::ai::World::set_world(worldMsg);

    ASSERT_EQ(node.update(), bt::Node::Status::Success);

    worldMsg.us[0].pos.x = 1.0;
    rtt::ai::World::set_world(worldMsg);

    ASSERT_EQ(node.update(), bt::Node::Status::Failure);
}