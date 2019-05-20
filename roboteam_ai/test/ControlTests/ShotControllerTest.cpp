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

TEST(ShotControllerTest, it_generates_proper_shots) {
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
    ShotData sd = shotController.getShotData(world::world->getUs().at(0), {1, 0});


    // test if it makes proper commands.
    roboteam_msgs::RobotCommand command;
    command.id = robot.id;
    shotController.makeCommand(sd, command);
    EXPECT_EQ(command.id, robot.id);
    EXPECT_EQ(command.x_vel, sd.vel.x);
    EXPECT_EQ(command.y_vel, sd.vel.y);
    EXPECT_EQ(command.w, sd.angle);
    EXPECT_EQ(command.chipper, sd.chip);
    EXPECT_EQ(command.kicker, sd.kick);
    EXPECT_EQ(command.kicker_forced, sd.kick);
    EXPECT_EQ(command.chipper_forced, sd.kick);
    EXPECT_EQ(command.kicker_vel, sd.kickSpeed);
    EXPECT_EQ(command.geneva_state, sd.genevaState);


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


    // the shotcontroller should determine a proper kickforce
    EXPECT_EQ(shotController.determineKickForce(10.0, BallSpeed::MAX_SPEED), Constants::MAX_KICK_POWER());
    EXPECT_EQ(shotController.determineKickForce(1.0, BallSpeed::MAX_SPEED), Constants::MAX_KICK_POWER());
    EXPECT_EQ(shotController.determineKickForce(10.0, BallSpeed::PASS), Constants::MAX_KICK_POWER());
    EXPECT_FLOAT_EQ(shotController.determineKickForce(1.0, BallSpeed::PASS), 3.2);
    EXPECT_FLOAT_EQ(shotController.determineKickForce(4.0, BallSpeed::LAY_STILL_AT_POSITION), Constants::MAX_KICK_POWER() / 2.25);
    EXPECT_FLOAT_EQ(shotController.determineKickForce(1.0, BallSpeed::LAY_STILL_AT_POSITION), Constants::MAX_KICK_POWER() / 4.5);
    EXPECT_FLOAT_EQ(shotController.determineKickForce(4.0, BallSpeed::DRIBBLE_KICK), Constants::MAX_KICK_POWER() / 2.25);
    EXPECT_FLOAT_EQ(shotController.determineKickForce(1.0, BallSpeed::DRIBBLE_KICK), Constants::MAX_KICK_POWER() / 4.5);


}

} // control
} // ai
} // rtt