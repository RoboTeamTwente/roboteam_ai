//
// Created by baris on 22/10/18.
//

#include <gtest/gtest.h>
#include "../../src/conditions/IsInZone.h"

TEST(IsInZoneTest, IsInZoneTest) {

    auto blackBoard = std::make_shared<bt::Blackboard>();
    rtt::ai::IsInZone node("Test", blackBoard);

    ASSERT_EQ(node.Update(), bt::Node::Status::Failure);

    roboteam_msgs::World worldMsg;
    roboteam_msgs::WorldRobot robot;

    robot.id = 2;
    robot.pos.x = 0;
    robot.pos.y = 0;
    worldMsg.them.push_back(robot);
    worldMsg.ball.pos.x = 0.1;
    worldMsg.ball.pos.y = 0.0;
    rtt::ai::World::set_world(worldMsg);

    blackBoard->SetInt("zone", 3);

    ASSERT_EQ(node.Update(), bt::Node::Status::Success);


}