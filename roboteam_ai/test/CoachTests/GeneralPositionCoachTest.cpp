//
// Created by mrlukasbos on 20-3-19.
//

#include <gtest/gtest.h>
#include <roboteam_ai/src/coach/GeneralPositionCoach.h>
#include <roboteam_ai/src/world/Field.h>
#include <roboteam_ai/src/control/ControlUtils.h>
#include "../helpers/WorldHelper.h"

namespace w = rtt::ai::world;

TEST(CoachTest, get_position_behind_ball) {
    roboteam_msgs::GeometryFieldSize field;
    field.field_width = 12;
    field.field_length = 8;
    w::field->set_field(field);

    // create a world with one of our robots that has the ball
    auto world = testhelpers::WorldHelper::getWorldMsgWhereRobotHasBall(1, 0, true, field);
    w::world->updateWorld(world.first);

    // get position behind ball to our own goal
    auto pos = rtt::ai::coach::g_generalPositionCoach.getPositionBehindBallToGoal(1, true);

    // it should be one meter further than the ball
    EXPECT_FLOAT_EQ(pos.dist(w::world->getBall()->pos), 1);

    // it should be the same location as the test position
    auto ballPos = rtt::Vector2(w::world->getBall()->pos);
    auto testPos = ballPos + (ballPos - w::field->get_our_goal_center()).stretchToLength(1);
    EXPECT_FLOAT_EQ(pos.dist(w::world->getBall()->pos), 1);
    EXPECT_FLOAT_EQ(pos.x, testPos.x);
    EXPECT_FLOAT_EQ(pos.y, testPos.y);

    // create a world with 3v3 with a ball at given location
    auto worldMsg = testhelpers::WorldHelper::getWorldMsg(3, 3, false, field);
    worldMsg.ball.pos = Vector2(0, 0);
    worldMsg.ball.visible = 1;
    worldMsg.ball.existence = 9999;
    w::world->updateWorld(worldMsg);

    // set the robot on the horizontal line from the ball to the goal
    EXPECT_FALSE(rtt::ai::coach::g_generalPositionCoach.isRobotBehindBallToGoal(1.0, true, Vector2(0,0))); // robot on top of the ball: false
    EXPECT_TRUE(rtt::ai::coach::g_generalPositionCoach.isRobotBehindBallToGoal(1, true, Vector2(0.9, 0))); // robot 1 m behind the ball to goal:
    EXPECT_FALSE(rtt::ai::coach::g_generalPositionCoach.isRobotBehindBallToGoal(1, true, Vector2(-0.9, 0))); // robot 1m in front of the ball

    EXPECT_FALSE(rtt::ai::coach::g_generalPositionCoach.isRobotBehindBallToGoal(1, false, Vector2(1, 0))); // robot 1 m behind the ball to goal:
    EXPECT_TRUE(rtt::ai::coach::g_generalPositionCoach.isRobotBehindBallToGoal(1, false, Vector2(-1, 0))); // robot 1m in front of the

    // there should be some margins as well both behind the max position and on the y scale
    EXPECT_FALSE(rtt::ai::coach::g_generalPositionCoach.isRobotBehindBallToGoal(1, false, Vector2(1.2, 0.2)));
    EXPECT_TRUE(rtt::ai::coach::g_generalPositionCoach.isRobotBehindBallToGoal(1, false, Vector2(-1.2, 0.2)));

    // create an empty world
    worldMsg = testhelpers::WorldHelper::getWorldMsg(0, 0, false, field);
    worldMsg.ball.pos = Vector2(0, 0);
    worldMsg.ball.visible = 1;
    worldMsg.ball.existence = 9999;


        roboteam_msgs::WorldRobot robotToPointTo;
    robotToPointTo.id = 3;
    robotToPointTo.pos.x = -1;
    robotToPointTo.pos.y = -1;
    worldMsg.them.push_back(robotToPointTo);
    w::world->updateWorld(worldMsg);

    // check for position behind robottopointo, 1 m behind the ball
    // robottopointto is THEIR team
    EXPECT_TRUE(rtt::ai::coach::g_generalPositionCoach.isRobotBehindBallToRobot(1, false, 3, Vector2(1, 1))); // good
    EXPECT_TRUE(rtt::ai::coach::g_generalPositionCoach.isRobotBehindBallToRobot(1, false, 3, Vector2(1.2, 1))); // still good

    EXPECT_FALSE(rtt::ai::coach::g_generalPositionCoach.isRobotBehindBallToRobot(1, true, 3, Vector2(1, 1))); // wrong team, robot should not exist
    EXPECT_FALSE(rtt::ai::coach::g_generalPositionCoach.isRobotBehindBallToRobot(1, false, 2, Vector2(1.2, 1))); // wrong id, robot should not exist
    EXPECT_FALSE(rtt::ai::coach::g_generalPositionCoach.isRobotBehindBallToRobot(1, true, 3, Vector2(-1, 1))); // wrong location
    EXPECT_FALSE(rtt::ai::coach::g_generalPositionCoach.isRobotBehindBallToRobot(1, false, 2, Vector2(2, 2))); // too far behind ball

    auto testPos2 = rtt::ai::coach::g_generalPositionCoach.getPositionBehindBallToRobot(1, false, 3);
    EXPECT_FLOAT_EQ(testPos2.x, sin(M_PI/4));
    EXPECT_FLOAT_EQ(testPos2.y, cos(M_PI/4));
}
