//
// Created by robzelluf on 1/29/19.
//

#include <gtest/gtest.h>
#include <roboteam_ai/src/utilities/RobotDealer.h>
#include "../../src/skills/Pass.h"
#include "../../src/utilities/Field.h"
#include "../../src/coach/PassCoach.h"
#include "../../src/coach/GeneralPositionCoach.h"
#include "../../src/coach/OffensiveCoach.h"

TEST(PassTest, PassTest) {
    robotDealer::RobotDealer::halt();

    roboteam_msgs::GeometryFieldSize field;
    field.field_length = 20;
    field.field_width = 10;
    rtt::ai::Field::set_field(field);

    roboteam_msgs::World world;

    roboteam_msgs::WorldRobot robot1;
    robot1.id = 0;
    robot1.pos.x = 4;
    robot1.pos.y = 0;

    world.us.push_back(robot1);
    rtt::ai::World::set_world(world);

    ASSERT_EQ(rtt::ai::coach::g_pass.initiatePass(), robot1.id);

    roboteam_msgs::WorldRobot robot2;
    robot2.id = 1;
    rtt::ai::World::set_world(world);

    robot2.pos.x = 6;
    robot2.pos.y = 0;
    world.us.push_back(robot2);
    rtt::ai::World::set_world(world);

    ASSERT_EQ(rtt::ai::coach::g_pass.initiatePass(), robot2.id);

    roboteam_msgs::WorldBall ball;
    ball.pos.x = 3;
    ball.pos.y = -3;
    ball.visible = 1;
    ball.existence = 99999;
    world.ball = ball;
    rtt::ai::World::set_world(world);

    auto properties = std::make_shared<bt::Blackboard>();
    properties->setString("ROLE", "testPasser");

    // MIGHT NOT WORK
    robotDealer::RobotDealer::claimRobotForTactic(robotDealer::RobotType::closeToBall, "PassTest", "testPasser");

    rtt::ai::Pass pass("PassTest", properties);
    pass.initialize();

    ASSERT_EQ(pass.update(), bt::Leaf::Status::Running);

    robot1.pos = rtt::ai::coach::g_generalPositionCoach.getPositionBehindBallToPosition(0.05, robot2.pos);
    robot1.angle = static_cast<float>(((Vector2)robot1.pos - ball.pos).angle());

    roboteam_msgs::World world2;
    world2.us.push_back(robot1);
    world2.us.push_back(robot2);
    world2.ball.vel = (Vector2){5, 5};
    world2.ball.existence = 99999;
    rtt::ai::World::set_world(world2);

    ASSERT_EQ(pass.update(), bt::Leaf::Status::Running);

    world2.ball.pos = (Vector2){-3, 3};
    rtt::ai::World::set_world(world2);
    ASSERT_EQ(pass.update(), bt::Leaf::Status::Running);

}
