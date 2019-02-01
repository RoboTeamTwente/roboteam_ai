#include <gtest/gtest.h>
#include <roboteam_ai/test/helpers/WorldHelper.h>
#include <roboteam_ai/src/utilities/Field.h>
#include <roboteam_ai/src/utilities/World.h>
#include <roboteam_ai/src/utilities/Coach.h>

namespace rtt {
namespace ai {
namespace coach {

TEST(CoachTest, robot_has_ball_tests) {
    roboteam_msgs::GeometryFieldSize field;
    field.field_width = 12;
    field.field_length = 8;
    Field::set_field(field);

    auto world = testhelpers::WorldHelper::getWorldMsgWhereRobotHasBall(3, 3, true, field);
    World::set_world(world.first);
    int robotThatHasBall = world.second;

    ASSERT_TRUE(Coach::doesRobotHaveBall(robotThatHasBall, true));
    ASSERT_FALSE(Coach::doesRobotHaveBall(robotThatHasBall, false));

    // the -1 gives an overflow on the unsigned int parameter, which makes it return false.
    ASSERT_FALSE(Coach::doesRobotHaveBall(robotThatHasBall - 1, false));
    ASSERT_FALSE(Coach::doesRobotHaveBall(robotThatHasBall + 1, true));

    ASSERT_EQ(Coach::whichRobotHasBall(true), robotThatHasBall);
    ASSERT_NE(Coach::whichRobotHasBall(false), robotThatHasBall);
}

TEST(CoachTest, get_position_behind_ball) {
    roboteam_msgs::GeometryFieldSize field;
    field.field_width = 12;
    field.field_length = 8;
    Field::set_field(field);

    // create a world with one of our robots that has the ball
    auto world = testhelpers::WorldHelper::getWorldMsgWhereRobotHasBall(1, 0, true, field);
    World::set_world(world.first);

    // get position behind ball to our own goal
    auto pos = Coach::getPositionBehindBallToGoal(1, true);

    // it should be one meter further than the ball
    EXPECT_FLOAT_EQ(pos.dist(World::getBall()->pos), 1);

    // it should be the same location as the test position
    auto ballPos = rtt::Vector2(World::getBall()->pos);
    auto testPos = ballPos + (ballPos - Field::get_our_goal_center()).stretchToLength(1);
    EXPECT_FLOAT_EQ(pos.dist(World::getBall()->pos), 1);
    EXPECT_FLOAT_EQ(pos.x, testPos.x);
    EXPECT_FLOAT_EQ(pos.y, testPos.y);

    // create a world with 3v3 with a ball at given location
    auto worldMsg = testhelpers::WorldHelper::getWorldMsg(3, 3, false, field);
    worldMsg.ball.pos = Vector2(0, 0);
    worldMsg.ball.visible = 1;
    World::set_world(worldMsg);

    // set the robot on the horizontal line from the ball to the goal
    EXPECT_FALSE(Coach::isRobotBehindBallToGoal(1.0, true, Vector2(0,0))); // robot on top of the ball: false
    EXPECT_TRUE(Coach::isRobotBehindBallToGoal(1, true, Vector2(0.9, 0))); // robot 1 m behind the ball to goal:
    EXPECT_FALSE(Coach::isRobotBehindBallToGoal(1, true, Vector2(-0.9, 0))); // robot 1m in front of the ball

    EXPECT_FALSE(Coach::isRobotBehindBallToGoal(1, false, Vector2(1, 0))); // robot 1 m behind the ball to goal:
    EXPECT_TRUE(Coach::isRobotBehindBallToGoal(1, false, Vector2(-1, 0))); // robot 1m in front of the

    // there should be some margins as well both behind the max position and on the y scale
    EXPECT_FALSE(Coach::isRobotBehindBallToGoal(1, false, Vector2(1.2, 0.2)));
    EXPECT_TRUE(Coach::isRobotBehindBallToGoal(1, false, Vector2(-1.2, 0.2)));

    // create an empty world
    worldMsg = testhelpers::WorldHelper::getWorldMsg(0, 0, false, field);
    worldMsg.ball.pos = Vector2(0, 0);
    worldMsg.ball.visible = 1;

    roboteam_msgs::WorldRobot robotToPointTo;
    robotToPointTo.id = 3;
    robotToPointTo.pos.x = -1;
    robotToPointTo.pos.y = -1;
    worldMsg.them.push_back(robotToPointTo);
    World::set_world(worldMsg);

    // check for position behind robottopointo, 1 m behind the ball
    // robottopointto is THEIR team
    EXPECT_TRUE(Coach::isRobotBehindBallToRobot(1, false, 3, Vector2(1, 1))); // good
    EXPECT_TRUE(Coach::isRobotBehindBallToRobot(1, false, 3, Vector2(1.2, 1))); // still good

    EXPECT_FALSE(Coach::isRobotBehindBallToRobot(1, true, 3, Vector2(1, 1))); // wrong team, robot should not exist
    EXPECT_FALSE(Coach::isRobotBehindBallToRobot(1, false, 2, Vector2(1.2, 1))); // wrong id, robot should not exist
    EXPECT_FALSE(Coach::isRobotBehindBallToRobot(1, true, 3, Vector2(-1, 1))); // wrong location
    EXPECT_FALSE(Coach::isRobotBehindBallToRobot(1, false, 2, Vector2(2, 2))); // too far behind ball
}

} // coach
} // ai
} // rtt