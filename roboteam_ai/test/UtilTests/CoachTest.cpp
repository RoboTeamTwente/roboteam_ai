#include <gtest/gtest.h>
#include <roboteam_ai/test/helpers/WorldHelper.h>
#include <roboteam_ai/src/utilities/Field.h>
#include <roboteam_ai/src/utilities/World.h>
#include <roboteam_ai/src/utilities/Coach.h>
#include <roboteam_ai/src/interface/InterfaceValues.h>

namespace rtt {
namespace ai {
namespace coach {

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

    auto testPos2 = Coach::getPositionBehindBallToRobot(1, false, 3);
    EXPECT_FLOAT_EQ(testPos2.x, sin(M_PI/4));
    EXPECT_FLOAT_EQ(testPos2.y, cos(M_PI/4));

}

TEST(CoachTest, get_robot_closest_to_ball) {
    roboteam_msgs::GeometryFieldSize field;
    field.field_width = 12;
    field.field_length = 8;
    Field::set_field(field);

    // create a world with 5v5 and one of our robots has the ball
    auto world = testhelpers::WorldHelper::getWorldMsgWhereRobotHasBall(5, 5, true, field);
    World::set_world(world.first);
    auto robotThatHasBallId = world.second;


    // the right robot is closest
    EXPECT_EQ(Coach::getRobotClosestToBall().first, robotThatHasBallId);

    // it is our robot
    EXPECT_TRUE(Coach::getRobotClosestToBall().second);
    EXPECT_EQ(Coach::getRobotClosestToBall(true)->id, robotThatHasBallId);

    // create a world with 5v5 and one of their robots has the ball
    world = testhelpers::WorldHelper::getWorldMsgWhereRobotHasBall(5, 5, false, field);
    World::set_world(world.first);
    robotThatHasBallId = world.second;

    // the right robot is closest
    EXPECT_EQ(Coach::getRobotClosestToBall().first, robotThatHasBallId);

    // it is our robot
    EXPECT_FALSE(Coach::getRobotClosestToBall().second);

    EXPECT_EQ(Coach::getRobotClosestToBall(false)->id, robotThatHasBallId);
}

TEST(CoachTest, it_adds_and_removes_defenders) {
    roboteam_msgs::GeometryFieldSize field;
    field.field_width = 12;
    field.field_length = 8;
    Field::set_field(field);

    Coach::defenders = {}; // empty the defenders to start the test

    EXPECT_TRUE(Coach::defenders.empty());

    Coach::addDefender(3);
    EXPECT_EQ(Coach::defenders.size(), 1);
    EXPECT_EQ(Coach::defenders.at(0), 3); // the id is correct

    // getting a defensive position adds a defender if it is not already there.
    // robot 3 is already there so it should not be added again.
    auto defensePosition = Coach::getDefensivePosition(3);
    EXPECT_EQ(Coach::defenders.size(), 1);
    EXPECT_EQ(Coach::defenders.at(0), 3); // the id is correct

    EXPECT_EQ(defensePosition.y, 0);
    EXPECT_EQ(defensePosition.x, -2);

    // now another defense robot + position should be there
    defensePosition = Coach::getDefensivePosition(5);
    EXPECT_EQ(Coach::defenders.size(), 2);

    EXPECT_EQ(defensePosition.y, abs(2));
    EXPECT_EQ(defensePosition.x, -2); // it is either -4 or 4

    Coach::removeDefender(6); // this should do nothing
    EXPECT_EQ(Coach::defenders.size(), 2);

    Coach::removeDefender(3); // this should do nothing
    EXPECT_EQ(Coach::defenders.size(), 1);
    EXPECT_EQ(Coach::defenders.at(0), 5); // the id is correct

    // TODO check. these might be wrong.
    EXPECT_EQ(defensePosition.y, 2);
    EXPECT_EQ(defensePosition.x, -2);

    Coach::removeDefender(5); // this should do nothing
    EXPECT_EQ(Coach::defenders.size(), 0);
}

TEST(CoachTest, it_adds_and_removes_formationrobots) {
    roboteam_msgs::GeometryFieldSize field;
    field.field_width = 12;
    field.field_length = 8;
    Field::set_field(field);

    EXPECT_TRUE(Coach::robotsInFormation.empty());

    Coach::addFormationRobot(3);
    EXPECT_EQ(Coach::robotsInFormation.size(), 1);
    EXPECT_EQ(Coach::robotsInFormation.at(0), 3); // the id is correct

    // getting a defensive position adds a defender if it is not already there.
    // robot 3 is already there so it should not be added again.
    auto formationPosition = Coach::getFormationPosition(3);
    EXPECT_EQ(Coach::robotsInFormation.size(), 1);
    EXPECT_EQ(Coach::robotsInFormation.at(0), 3); // the id is correct

    EXPECT_EQ(formationPosition.y, 0);
    EXPECT_EQ(formationPosition.x, -2);

    // now another defense robot + position should be there
    formationPosition = Coach::getFormationPosition(5);
    EXPECT_EQ(Coach::robotsInFormation.size(), 2);

    EXPECT_EQ(formationPosition.y, abs(2));
    EXPECT_EQ(formationPosition.x, -2); // it is either -4 or 4

    Coach::removeFormationRobot(6); // this should do nothing
    EXPECT_EQ(Coach::robotsInFormation.size(), 2);

    Coach::removeFormationRobot(3); // this should do nothing
    EXPECT_EQ(Coach::robotsInFormation.size(), 1);
    EXPECT_EQ(Coach::robotsInFormation.at(0), 5); // the id is correct

    // TODO check. these might be wrong.
    EXPECT_EQ(formationPosition.y, 2);
    EXPECT_EQ(formationPosition.x, -2);

    Coach::removeFormationRobot(5); // this should do nothing
    EXPECT_EQ(Coach::robotsInFormation.size(), 0);
}

TEST(CoachTest, it_handles_ballplacement_positions) {
    interface::InterfaceValues::setBallPlacementTarget({2.3, 0.3});

    auto ballplacementTarget = Coach::getBallPlacementPos();
    EXPECT_EQ(ballplacementTarget.x, 2.3);
    EXPECT_EQ(ballplacementTarget.y, 0.3);

    EXPECT_FLOAT_EQ(ballplacementTarget.dist(Coach::getBallPlacementAfterPos(0.2)), Constants::BP_MOVE_BACK_DIST());

    roboteam_msgs::World worldMsg;
    worldMsg.ball.pos = Vector2(0, 0);
    worldMsg.ball.visible = 1;
    World::set_world(worldMsg);

    EXPECT_FLOAT_EQ(Vector2(World::getBall()->pos).dist(Coach::getBallPlacementBeforePos(World::getBall()->pos)), Constants::BP_MOVE_TOWARDS_DIST());
}

} // coach
} // ai
} // rtt