//
// Created by robzelluf on 6/14/19.
//

#include <gtest/gtest.h>
#include "roboteam_ai/src/conditions/CanReflectKick.h"
#include "../helpers/FieldHelper.h"
#include "roboteam_ai/src/world/World.h"
#include "roboteam_ai/src/world/Robot.h"
#include "roboteam_ai/src/utilities/Constants.h"
#include "roboteam_ai/src/utilities/RobotDealer.h"

TEST(CanReflectKickTest, can_reflect_kick) {
    auto field = testhelpers::FieldHelper::generateField(12, 9);
    roboteam_msgs::World world;

    roboteam_msgs::WorldRobot robot0;
    for (int i = 0; i<11; i++) {
        if (rtt::ai::Constants::ROBOT_HAS_WORKING_BALL_SENSOR(i)) {
            robot0.id = i;
            break;
        }
    }

    robot0.pos.x = 5.0;
    robot0.pos.y = -2.0;
    world.ball.pos = Vector2{4, 2};
    world.us.push_back(robot0);
    rtt::ai::world::world->updateWorld(world);

    rtt::ai::robotDealer::RobotDealer::claimRobotForTactic(rtt::ai::robotDealer::RobotType::RANDOM, "test0", "CanReflectKickTest");

    bt::Blackboard BB;
    BB.setString("ROLE", "test0");

    auto BBpointer = std::make_shared<bt::Blackboard>(BB);
    rtt::ai::CanReflectKick node("CanReflectKick",BBpointer);

    // Test a few cases for which the robot can and cannot ReflectKick
    node.initialize();
    EXPECT_EQ(node.node_name(), "CanReflectKick");
    EXPECT_EQ(node.update(), bt::Node::Status::Success);

    world.us[0].pos.y = 1.0;
    rtt::ai::world::world->updateWorld(world);

    node.initialize();
    EXPECT_EQ(node.update(), bt::Node::Status::Failure);

    world.ball.pos.y = -3;
    rtt::ai::world::world->updateWorld(world);

    node.initialize();
    EXPECT_EQ(node.update(), bt::Node::Status::Success);

    world.us[0].pos.y = -2;
    rtt::ai::world::world->updateWorld(world);

    node.initialize();
    EXPECT_EQ(node.update(), bt::Node::Status::Failure);

    world.us[0].pos.y = -4;
    rtt::ai::world::world->updateWorld(world);

    node.initialize();
    EXPECT_EQ(node.update(), bt::Node::Status::Success);
}

