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
    auto world = testhelpers::WorldHelper::getWorldMsg(0,0,0, field);

    // give it one robot
    roboteam_msgs::WorldRobot robot;
    robot.id = 2;
    robot.pos.x = 0;
    robot.pos.y = 0;
    robot.angle = 0;
    world.us.push_back(robot);

    // the robot is aimed towards the ball and the point
    world.ball = testhelpers::WorldHelper::generateBallAtLocation(testhelpers::WorldHelper::getLocationRightBeforeRobot(robot));

    rtt::ai::world::world->updateWorld(world);

    ShotController shotController;
    ShotData sd = shotController.getShotData(world::world->getUs().at(0), {1,0});

    // TODO improve this test
}




} // control
} // ai
} // rtt