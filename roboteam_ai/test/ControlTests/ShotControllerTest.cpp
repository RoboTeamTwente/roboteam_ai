//
// Created by mrlukasbos on 25-4-19.
//

#include "roboteam_ai/src/control/shotControllers/ShotController.h"
#include <gtest/gtest.h>
#include <roboteam_ai/test/helpers/WorldHelper.h>
#include <roboteam_ai/test/helpers/FieldHelper.h>

namespace rtt {
namespace ai {
namespace control {

namespace cr=rtt::ai::control;
using Vector2 = rtt::Vector2;


TEST(ShotControllerTest, it_generates_robotcommands) {
    ShotController shotController;
    RobotCommand sd = shotController.getShotData(*world::world->getUs().at(0), {1, 0});

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
    EXPECT_EQ(command.geneva_state, sd.geneva);
}

TEST(ShotControllerTest, it_calculates_kickforce) {
    ShotController shotController;
    EXPECT_EQ(shotController.determineKickForce(10.0, BallSpeed::MAX_SPEED), Constants::MAX_KICK_POWER());
    EXPECT_EQ(shotController.determineKickForce(1.0, BallSpeed::MAX_SPEED), Constants::MAX_KICK_POWER());
    EXPECT_EQ(shotController.determineKickForce(10.0, BallSpeed::PASS), Constants::MAX_KICK_POWER());
    EXPECT_FLOAT_EQ(shotController.determineKickForce(1.0, BallSpeed::PASS), 3.2);
    EXPECT_FLOAT_EQ(shotController.determineKickForce(4.0, BallSpeed::LAY_STILL_AT_POSITION), Constants::MAX_KICK_POWER() / 2.25);
    EXPECT_FLOAT_EQ(shotController.determineKickForce(1.0, BallSpeed::LAY_STILL_AT_POSITION), Constants::MAX_KICK_POWER() / 4.5);
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

    rtt::ai::world::world->updateWorld(world);
    ShotController shotController;


    // check if the correct positions behind the ball are found
    // the robot is at -1, 0 and the ball is at 0,0 and the shottarget is 1,0
    Vector2 loc = shotController.getPlaceBehindBallForGenevaState(world::Robot(robot), {1, 0}, 1);
    Vector2 ballPos = ball.pos;
    EXPECT_FLOAT_EQ(loc.length(), 2.0 * Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS());
    EXPECT_FLOAT_EQ((ballPos - loc).angle(), toRadians(20));

    loc = shotController.getPlaceBehindBallForGenevaState(world::Robot(robot), {1, 0}, 2);
    EXPECT_FLOAT_EQ(loc.length(), 2.0 * Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS());
    EXPECT_FLOAT_EQ((ballPos - loc).angle(), toRadians(10));

    loc = shotController.getPlaceBehindBallForGenevaState(world::Robot(robot), {1, 0}, 3);
    EXPECT_FLOAT_EQ(loc.length(), 2.0 * Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS());
    EXPECT_FLOAT_EQ((ballPos - loc).angle(), toRadians(0));

    loc = shotController.getPlaceBehindBallForGenevaState(world::Robot(robot), {1, 0}, 4);
    EXPECT_FLOAT_EQ(loc.length(), 2.0 * Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS());
    EXPECT_FLOAT_EQ((ballPos - loc).angle(), toRadians(-10));

    loc = shotController.getPlaceBehindBallForGenevaState(world::Robot(robot), {1, 0}, 5);
    EXPECT_FLOAT_EQ(loc.length(), 2.0 * Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS());
    EXPECT_FLOAT_EQ((ballPos - loc).angle(), toRadians(-20));


    // the shotcontroller can determine a proper geneva state
    EXPECT_EQ(shotController.determineOptimalGenevaState(world::Robot(robot), {1, 0}), 3);
    EXPECT_EQ(shotController.determineOptimalGenevaState(world::Robot(robot), {-1, 1}), 4);
    EXPECT_EQ(shotController.determineOptimalGenevaState(world::Robot(robot), {-1, -1}), 2);

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

    rtt::ai::world::world->updateWorld(world);

    ShotController shotController;
    RobotCommand shotdata = shotController.shoot(world::Robot(robot), {robot.pos, ball.pos}, {1, 0}, false, BallSpeed::MAX_SPEED);
    EXPECT_TRUE(shotdata.kicker);
    EXPECT_FALSE(shotdata.chipper);
    EXPECT_FLOAT_EQ(shotdata.angle, (Vector2(ball.pos) - Vector2(robot.pos)).toAngle());
    EXPECT_FLOAT_EQ(shotdata.kickerVel, Constants::MAX_KICK_POWER());

    shotdata = shotController.shoot(world::Robot(robot), {robot.pos, ball.pos}, {1, 0}, true, BallSpeed::PASS);
    EXPECT_FALSE(shotdata.kicker);
    EXPECT_TRUE(shotdata.chipper);
    EXPECT_FLOAT_EQ(shotdata.angle, (Vector2(ball.pos) - Vector2(robot.pos)).toAngle());
    EXPECT_FLOAT_EQ(shotdata.kickerVel, 3.2);

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
    robotWithBall->setGenevaState(3);
    robotWithBall->setWorkingGeneva(true);

    rtt::ai::world::world->updateWorld(worldRobotPair.first);
    Vector2 shotVector = world::world->getBall()->pos - robotWithBall->pos;
    Vector2 simulatedShotTarget = world::world->getBall()->pos + shotVector.stretchToLength(1.0);

    // a world message has been configure that one robot has the ball and he is aiming directly towards the target pos
    // it should thus shoot.

    // kick test
    ShotController shotController;
    RobotCommand shotdata = shotController.getShotData(* robotWithBall, simulatedShotTarget, false, BallSpeed::MAX_SPEED, false, ShotPrecision::LOW);
    EXPECT_TRUE(shotdata.kicker);
    EXPECT_FALSE(shotdata.chipper);
    EXPECT_FLOAT_EQ(shotdata.angle, (Vector2(world::world->getBall()->pos) - Vector2(robotWithBall->pos)).toAngle());
    EXPECT_FLOAT_EQ(shotdata.geneva, 3);

    // chip test
    shotdata = shotController.getShotData(* robotWithBall, simulatedShotTarget, true, BallSpeed::MAX_SPEED, false, ShotPrecision::LOW);
    EXPECT_FALSE(shotdata.kicker);
    EXPECT_TRUE(shotdata.chipper);
    EXPECT_FLOAT_EQ(shotdata.angle, (Vector2(world::world->getBall()->pos) - Vector2(robotWithBall->pos)).toAngle());
    EXPECT_FLOAT_EQ(shotdata.geneva, 3);



    /*
     *
     * When the autogeneva is on we should test some things a lot -> in a loop
     * when we chip the geneva from the outgoing command should ALWAYS be 3
     * We alternate the target position to see the difference in behaviour
     */
    for (int i = 0; i < 100; i++) {
        auto world = testhelpers::WorldHelper::getWorldMsg(1, 0, true, field);
        rtt::ai::world::world->updateWorld(world);
        auto robot = world::world->getUs().at(0);
        robot->setGenevaState(3);
        robot->setWorkingGeneva(true);

        shotdata = shotController.getShotData(*robot, testhelpers::WorldHelper::getRandomFieldPosition(field),
                                              true, BallSpeed::MAX_SPEED, true, ShotPrecision::HIGH);
        EXPECT_FALSE(shotdata.kicker);
        EXPECT_FLOAT_EQ(shotdata.geneva, 3);
    }

    /*
     * we use autogeneva and kick
     * The geneva state should always stay between 0 and 5
     */
    for (int i = 0; i < 100; i++) {
        auto world = testhelpers::WorldHelper::getWorldMsg(1, 0, true, field);
        rtt::ai::world::world->updateWorld(world);
        auto robot = world::world->getUs().at(0);
        robot->setGenevaState(3);
        robot->setWorkingGeneva(true);

        shotdata = shotController.getShotData(*robot, testhelpers::WorldHelper::getRandomFieldPosition(field), false, BallSpeed::MAX_SPEED, true, ShotPrecision::HIGH);
        EXPECT_GE(shotdata.geneva, 0);
        EXPECT_LE(shotdata.geneva, 5);
    }

    /*
     * If the robot has no geneva it should ALWAYS keep its initial state
     */
    for (int i = 0; i < 100; i++) {
        auto world = testhelpers::WorldHelper::getWorldMsg(1, 0, true, field);
        rtt::ai::world::world->updateWorld(world);
        auto robot = world::world->getUs().at(0);
        robot->setGenevaState(4);
        robot->setWorkingGeneva(false);

        shotdata = shotController.getShotData(*robot, testhelpers::WorldHelper::getRandomFieldPosition(field), false, BallSpeed::MAX_SPEED, true, ShotPrecision::HIGH);
        EXPECT_EQ(shotdata.geneva, 4);
    }
}

TEST(ShotControllerTest, geneva_turning) {
    ShotController shotController;
    shotController.setGenevaDelay(2);
    EXPECT_FLOAT_EQ(shotController.secondsToTurnGeneva, 0.8);
    EXPECT_TRUE(shotController.genevaIsTurning);

    ShotController shotController2;
    shotController2.setGenevaDelay(0);
    EXPECT_FLOAT_EQ(shotController2.secondsToTurnGeneva, 0);
    EXPECT_FALSE(shotController2.genevaIsTurning);

    ShotController shotController3;
    shotController3.setGenevaDelay(4);
    EXPECT_FLOAT_EQ(shotController3.secondsToTurnGeneva, 1.6);
    EXPECT_TRUE(shotController3.genevaIsTurning);
}


} // control
} // ai
} // rtt