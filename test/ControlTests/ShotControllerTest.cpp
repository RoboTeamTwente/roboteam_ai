//
// Created by mrlukasbos on 25-4-19.
//

#include <gtest/gtest.h>
#include <roboteam_ai/test/helpers/FieldHelper.h>
#include <roboteam_ai/test/helpers/WorldHelper.h>

#include "control/shot-controllers/ShotController.h"

namespace rtt {
namespace ai {
namespace control {

namespace cr = rtt::ai::control;
using Vector2 = rtt::Vector2;

TEST(ShotControllerTest, it_generates_robotcommands) {
    ShotController shotController;
    RobotCommand sd = shotController.getRobotCommand(*world::world->getUs().at(0), {1, 0});

    roboteam_msgs::WorldRobot robot;
    robot.id = 2;
    robot.pos.x = -1;
    robot.pos.y = 0;
    robot.angle = 0;

    roboteam_msgs::RobotCommand command = sd.makeROSCommand();
    command.id = robot.id;
    EXPECT_EQ(command.id, robot.id);
    EXPECT_FLOAT_EQ(command.x_vel, sd.vel.x);
    EXPECT_FLOAT_EQ(command.y_vel, sd.vel.y);
    EXPECT_FLOAT_EQ(command.w, sd.angle);
    EXPECT_EQ(command.chipper, sd.chipper);
    EXPECT_EQ(command.kicker, sd.kicker);
    EXPECT_EQ(command.kicker_forced, sd.kicker);
    EXPECT_EQ(command.chipper_forced, sd.kicker);
    EXPECT_DOUBLE_EQ(command.kicker_vel, sd.kickerVel);
}

TEST(ShotControllerTest, it_calculates_kickforce) {
    ShotController shotController;
    Constants::OVERWRITE_GRSIM(true);
    EXPECT_EQ(shotController.determineKickForce(10.0, BallSpeed::MAX_SPEED), Constants::MAX_KICK_POWER());
    EXPECT_EQ(shotController.determineKickForce(1.0, BallSpeed::MAX_SPEED), Constants::MAX_KICK_POWER());
    EXPECT_EQ(shotController.determineKickForce(10.0, BallSpeed::PASS), Constants::MAX_KICK_POWER());

    EXPECT_FLOAT_EQ(shotController.determineKickForce(1.0, BallSpeed::PASS), Constants::MAX_KICK_POWER() * 1.4 / 9.0);
    EXPECT_FLOAT_EQ(shotController.determineKickForce(4.0, BallSpeed::BALL_PLACEMENT), 4.01);
    EXPECT_FLOAT_EQ(shotController.determineKickForce(1.0, BallSpeed::BALL_PLACEMENT), 4.01);
    EXPECT_FLOAT_EQ(shotController.determineKickForce(4.0, BallSpeed::DRIBBLE_KICK), Constants::MAX_KICK_POWER() / 2.25);
    EXPECT_FLOAT_EQ(shotController.determineKickForce(1.0, BallSpeed::DRIBBLE_KICK), Constants::MAX_KICK_POWER() / 4.5);

    Constants::OVERWRITE_GRSIM(false);
    EXPECT_EQ(shotController.determineKickForce(10.0, BallSpeed::MAX_SPEED), Constants::MAX_KICK_POWER());
    EXPECT_EQ(shotController.determineKickForce(1.0, BallSpeed::MAX_SPEED), Constants::MAX_KICK_POWER());
    EXPECT_EQ(shotController.determineKickForce(10.0, BallSpeed::PASS), Constants::MAX_KICK_POWER());

    EXPECT_FLOAT_EQ(shotController.determineKickForce(1.0, BallSpeed::PASS), 1.01);
    EXPECT_FLOAT_EQ(shotController.determineKickForce(4.0, BallSpeed::BALL_PLACEMENT), 1.01);
    EXPECT_FLOAT_EQ(shotController.determineKickForce(1.0, BallSpeed::BALL_PLACEMENT), 1.01);
    EXPECT_FLOAT_EQ(shotController.determineKickForce(4.0, BallSpeed::DRIBBLE_KICK), Constants::MAX_KICK_POWER() / 2.25);
    EXPECT_FLOAT_EQ(shotController.determineKickForce(1.0, BallSpeed::DRIBBLE_KICK), Constants::MAX_KICK_POWER() / 4.5);
}

TEST(ShotControllerTest, it_locates_robots_properly) {
    // create an empty world message
    auto field = testhelpers::FieldHelper::generateField();
    auto world = testhelpers::WorldHelper::getWorldMsg(0, 0, 0, field);

    // give it one robot
    roboteam_msgs::WorldRobot robot;
    robot.id = 2;
    robot.pos.x = -1;
    robot.pos.y = 0;
    robot.angle = 0;
    world.us.push_back(robot);

    roboteam_msgs::WorldBall ball;
    ball.pos.x = 0;
    ball.pos.y = 0;
    world.ball = ball;

    rtt::world::world->updateWorld(world);
    ShotController shotController;

}

TEST(ShotControllerTest, it_sends_proper_shoot_commands) {
    // create an empty world message
    auto field = testhelpers::FieldHelper::generateField();
    auto world = testhelpers::WorldHelper::getWorldMsg(0, 0, 0, field);

    // give it one robot
    roboteam_msgs::WorldRobot robot;
    robot.id = 2;
    robot.pos.x = -1;
    robot.pos.y = 0;
    robot.angle = 0;
    world.us.push_back(robot);

    roboteam_msgs::WorldBall ball;
    ball.pos.x = 0;
    ball.pos.y = 0;
    world.ball = ball;

    rtt::world::world->updateWorld(world);

    ShotController shotController;
    RobotCommand shotdata;
    shotdata = shotController.shoot(shotdata, world::Robot(robot), {robot.pos, ball.pos}, {1, 0}, false, BallSpeed::MAX_SPEED);
    EXPECT_TRUE(shotdata.kicker);
    EXPECT_FALSE(shotdata.chipper);
    EXPECT_FLOAT_EQ(shotdata.angle, (Vector2(ball.pos) - Vector2(robot.pos)).toAngle());
    EXPECT_FLOAT_EQ(shotdata.kickerVel, Constants::MAX_KICK_POWER());

    shotdata = shotController.shoot(shotdata, world::Robot(robot), {robot.pos, ball.pos}, {1, 0}, true, BallSpeed::PASS);
    EXPECT_FALSE(shotdata.kicker);
    EXPECT_TRUE(shotdata.chipper);
    EXPECT_FLOAT_EQ(shotdata.angle, (Vector2(ball.pos) - Vector2(robot.pos)).toAngle());
    // EXPECT_FLOAT_EQ(shotdata.kickerVel, 3.2);

    shotdata = shotController.moveStraightToBall(world::Robot(robot), {robot.pos, ball.pos});
    EXPECT_FALSE(shotdata.kicker);
    EXPECT_FALSE(shotdata.chipper);
    EXPECT_FLOAT_EQ(shotdata.angle, (Vector2(ball.pos) - Vector2(robot.pos)).toAngle());

    shotdata = shotController.moveStraightToBall(world::Robot(robot), {robot.pos, ball.pos});
    EXPECT_FALSE(shotdata.kicker);
    EXPECT_FALSE(shotdata.chipper);
    EXPECT_FLOAT_EQ(shotdata.angle, (Vector2(ball.pos) - Vector2(robot.pos)).toAngle());
}

TEST(ShotControllerTest, getshotdata_test) {
    auto field = testhelpers::FieldHelper::generateField();
    auto worldRobotPair = testhelpers::WorldHelper::getWorldMsgWhereRobotHasBall(1, 0, true, field);
    int robotWithBallId = worldRobotPair.second;
    auto robotWithBall = world::world->getRobotForId(robotWithBallId, true);
    rtt::world::world->updateWorld(worldRobotPair.first);
    Vector2 shotVector = world::world->getBall()->pos - robotWithBall->pos;
    Vector2 simulatedShotTarget = world::world->getBall()->pos + shotVector.stretchToLength(1.0);

    // a world message has been configure that one robot has the ball and he is aiming directly towards the target pos
    // it should thus shoot.

    // kick test
    ShotController shotController;
    RobotCommand shotdata = shotController.getRobotCommand(*robotWithBall, simulatedShotTarget, false, BallSpeed::MAX_SPEED, false, ShotPrecision::LOW);
    EXPECT_TRUE(shotdata.kicker);
    EXPECT_FALSE(shotdata.chipper);
    EXPECT_FLOAT_EQ(shotdata.angle, (world::world->getBall()->pos - robotWithBall->pos).toAngle());

    // chip test
    shotdata = shotController.getRobotCommand(*robotWithBall, simulatedShotTarget, true, BallSpeed::MAX_SPEED, false, ShotPrecision::LOW);
    EXPECT_FALSE(shotdata.kicker);
    EXPECT_TRUE(shotdata.chipper);
    EXPECT_FLOAT_EQ(shotdata.angle, (world::world->getBall()->pos - robotWithBall->pos).toAngle());
-
}

}  // namespace control
}  // namespace ai
}  // namespace rtt