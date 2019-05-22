//
// Created by robzelluf on 5/22/19.
//

#include <gtest/gtest.h>
#include <roboteam_ai/src/coach/midField/HarassRobotCoach.h>
#include "../helpers/FieldHelper.h"
#include "../helpers/WorldHelper.h"
#include "roboteam_ai/src/analysis/GameAnalyzer.h"

using Robot = rtt::ai::world::Robot;
using RobotPtr = std::shared_ptr<Robot>;

TEST(CoachTest, harass_robot_coach_initialize_test) {
    roboteam_msgs::GeometryFieldSize field = testhelpers::FieldHelper::generateField();
    rtt::ai::world::field->set_field(field);
    roboteam_msgs::World world;

    roboteam_msgs::WorldRobot robot0;
    robot0.id = 0;
    robot0 = testhelpers::WorldHelper::generateRandomRobot(0, field);
    world.us.emplace_back(robot0);
    int myIndex0 = -1;

    rtt::ai::world::world->updateWorld(world);

    rtt::ai::analysis::GameAnalyzer::getInstance().generateReportNow();

    auto harassTarget = rtt::ai::coach::g_harassRobotCoach.getHarassPosition(std::make_shared<Robot>(robot0), myIndex0);
    Vector2 bestPos = harassTarget.harassPosition;
    EXPECT_EQ(bestPos.x, 0);
    EXPECT_EQ(bestPos.y, robot0.pos.y);
}

TEST(CoachTest, we_have_ball_test) {
    roboteam_msgs::GeometryFieldSize field = testhelpers::FieldHelper::generateField();
    rtt::ai::world::field->set_field(field);
    roboteam_msgs::World world;

    auto worldData = testhelpers::WorldHelper::getWorldMsgWhereRobotHasBall(3, 3, true, field);
    world = worldData.first;
    roboteam_msgs::WorldRobot robot0 = world.us[0];
    int myIndex0 = -1;

    rtt::ai::world::world->updateWorld(world);
    world.time = world.time + 1.0;
    rtt::ai::world::world->updateWorld(world);

    rtt::ai::analysis::GameAnalyzer::getInstance().generateReportNow();

    auto harassTarget = rtt::ai::coach::g_harassRobotCoach.getHarassPosition(std::make_shared<Robot>(robot0), myIndex0);
    Vector2 bestPos = harassTarget.harassPosition;
    EXPECT_EQ(bestPos.x, 1.2);
    EXPECT_EQ(bestPos.y, robot0.pos.y);
}

