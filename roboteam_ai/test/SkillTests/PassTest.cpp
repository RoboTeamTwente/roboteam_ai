//
// Created by robzelluf on 1/29/19.
//

#include <gtest/gtest.h>
#include <roboteam_ai/src/utilities/RobotDealer.h>
#include "../../src/skills/Pass.h"
#include "../../src/coach/PassCoach.h"
#include "../../src/coach/GeneralPositionCoach.h"
#include "../../src/coach/OffensiveCoach.h"
#include "roboteam_ai/src/world/Field.h"
#include "roboteam_ai/src/world/World.h"


namespace rd = rtt::ai::robotDealer;
namespace w = rtt::ai::world;
using Vector2 = rtt::Vector2;
TEST(PassTest, PassTest) {
    rd::RobotDealer::halt();

    roboteam_msgs::GeometryFieldSize field;
    field.field_length = 20;
    field.field_width = 10;
    rtt::ai::world::field->set_field(field);
    roboteam_msgs::World world;

    roboteam_msgs::WorldRobot robot1;
    robot1.id = 0;
    robot1.pos.x = 4;
    robot1.pos.y = 0;
    roboteam_msgs::WorldBall ball;
    ball.existence = 11;
    ball.visible = static_cast<unsigned char>(true);
    ball.pos = Vector2(1.0,1.0);
    world.us.push_back(robot1);
    world.ball = ball;
    w::world->updateWorld(world);

    ASSERT_EQ(rtt::ai::coach::g_pass.initiatePass(), static_cast<int>(robot1.id));

    roboteam_msgs::WorldRobot robot2;
    robot2.id = 1;
    w::world->updateWorld(world);

    robot2.pos.x = 6;
    robot2.pos.y = 0;
    world.us.push_back(robot2);
    w::world->updateWorld(world);

    ASSERT_EQ(rtt::ai::coach::g_pass.initiatePass(), static_cast<int>(robot2.id));


    ball.pos.x = 3;
    ball.pos.y = -3;
    ball.visible = 1;
    ball.existence = 99999;
    world.ball = ball;
    w::world->updateWorld(world);

    auto properties = std::make_shared<bt::Blackboard>();
    properties->setString("ROLE", "testPasser");

    // MIGHT NOT WORK
    rd::RobotDealer::claimRobotForTactic(rd::RobotType::CLOSE_TO_BALL, "testPasser", "PassTest");

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
    w::world->updateWorld(world2);

    ASSERT_EQ(pass.update(), bt::Leaf::Status::Running);

    world2.ball.pos = (Vector2){-3, 3};
    w::world->updateWorld(world2);
    ASSERT_EQ(pass.update(), bt::Leaf::Status::Running);

}
