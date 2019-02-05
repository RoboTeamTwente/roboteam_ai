//
// Created by robzelluf on 1/29/19.
//

#include <gtest/gtest.h>
#include "../../src/skills/Pass.h"
#include "../../src/utilities/Field.h"
#include "../../src/utilities/Coach.h"

TEST(PassTest, PassTest) {
    roboteam_msgs::GeometryFieldSize field;
    field.field_length = 20;
    field.field_width = 10;
    rtt::ai::Field::set_field(field);

    roboteam_msgs::World world;

    roboteam_msgs::WorldRobot robot1;
    robot1.id = 1;
    robot1.pos.x = 0;
    robot1.pos.y = 0;

    world.us.push_back(robot1);
    rtt::ai::World::set_world(world);

    ASSERT_EQ(rtt::ai::coach::Coach::initiatePass(), robot1.id);

    roboteam_msgs::WorldRobot robot2;
    robot2.id = 2;
    robot2.pos.x = 3.0;
    robot2.pos.y = 0.0;
    world.us.push_back(robot2);
    rtt::ai::World::set_world(world);

    ASSERT_EQ(rtt::ai::coach::Coach::initiatePass(), robot2.id);

    roboteam_msgs::WorldBall ball;
    ball.pos.x = 3;
    ball.pos.y = -3;
    world.ball = ball;
    rtt::ai::World::set_world(world);

    auto properties = std::make_shared<bt::Blackboard>();
    properties->setString("ROLE", "testPasser");

    //TODO: Fix this part of the test, something with the robotdealer and integers segfaults
//    robotDealer::RobotDealer::claimRobotForTactic(robotDealer::RobotType::closeToBall, "PassTest", "testPasser");
//
//    rtt::ai::Pass pass("PassTest", properties);
//    pass.initialize();
//
//    ASSERT_EQ(pass.update(), bt::Leaf::Status::Running);
//
//    robot1.pos = rtt::ai::coach::Coach::getPositionBehindBallToPosition(0.05, robot2.pos);
//    robot1.angle = static_cast<float>(((Vector2)robot1.pos - ball.pos).angle());
//
//    roboteam_msgs::World world2;
//    world2.us.push_back(robot1);
//    world2.us.push_back(robot2);
//    world2.ball.vel = (Vector2){5, 5};
//    rtt::ai::World::set_world(world2);
//
//    ASSERT_EQ(pass.update(), bt::Leaf::Status::Running);
//
//    world2.ball.pos = (Vector2){-3, 3};
//    rtt::ai::World::set_world(world2);
//    ASSERT_EQ(pass.update(), bt::Leaf::Status::Running);

}
