//
// Created by baris on 22/10/18.
//

#include <gtest/gtest.h>
#include "../../src/conditions/IsInZone.h"

#include "../../src/utilities/Field.h"

TEST(IsBallInZoneTest, IsInZoneTest) {

    auto blackBoard = std::make_shared<bt::Blackboard>();
    rtt::ai::IsInZone node("Test", blackBoard);

    ASSERT_EQ(node.update(),bt::Node::Status::Failure);

    roboteam_msgs::World worldMsg;

    worldMsg.ball.pos.x=0.1;
    worldMsg.ball.pos.y=0.0;
    rtt::ai::World::set_world(worldMsg);

    blackBoard->setInt("zone", 3);

    ASSERT_EQ(node.update(),bt::Node::Status::Success);
}

TEST(IsRobotBallInZoneTest, IsInZoneTest) {
    bt::Blackboard BB;
    BB.setInt("ROBOT_ID", 2);
    BB.setBool("robot", true);

    auto blackBoard = std::make_shared<bt::Blackboard>(BB);
    rtt::ai::IsInZone node("Test", blackBoard);

    ASSERT_EQ(node.update(),bt::Node::Status::Failure);

    roboteam_msgs::World worldMsg;
    roboteam_msgs::WorldRobot robot;

    robot.id=2;
    robot.pos.x=-3.0;
    robot.pos.y=1.5;
    worldMsg.us.push_back(robot);
    rtt::ai::World::set_world(worldMsg);

    blackBoard->setInt("zone", 1);
    ASSERT_EQ(node.update(),bt::Node::Status::Success);

    blackBoard->setInt("zone", 2);
    ASSERT_EQ(node.update(),bt::Node::Status::Failure);

    blackBoard->setInt("zone", 4);
    roboteam_msgs::GeometryFieldSize field;

    worldMsg.us[0].pos.x = 0;
    worldMsg.us[0].pos.y = 0;
    rtt::ai::World::set_world(worldMsg);

    field.field_length = 3;
    field.field_width = 2;

    rtt::ai::Field::set_field(field);

    ASSERT_EQ(node.update(), bt::Node::Status::Success);
    worldMsg.us[0].pos.x = 0;
    worldMsg.us[0].pos.y = 0.7;
    rtt::ai::World::set_world(worldMsg);

    ASSERT_EQ(node.update(), bt::Node::Status::Success);

    blackBoard->setInt("zone", 5);
    ASSERT_EQ(node.update(),bt::Node::Status::Failure);

    blackBoard->setInt("zone", 99);
    ASSERT_EQ(node.update(),bt::Node::Status::Failure);
}