#include <utility>

//
// Created by thijs on 10-12-18.
//


#include "ControlGoToPos.h"

namespace rtt {
namespace ai {
namespace control {

ControlGoToPos::ControlGoToPos() = default;

void ControlGoToPos::clear(GoToType goToType) {
    switch (goToType) {
    case noPreference:break;
    case ballControl:break;
    case basic:break;
    case lowLevel:break;
    case highLevel:break;
    case force:break;
    case luTh: {
        gtpLuth.clear();
        break;
    }
    case bezier:break;
    }
}

Vector2 ControlGoToPos::goToPos(RobotPtr robot, Vector2 &position) {
    GoToType goToType = basic;
    //TODO: do stuff that determines which gtp to use...

    return ControlGoToPos::goToPos(std::move(robot), position, goToType);
}

Vector2 ControlGoToPos::goToPos(RobotPtr robot, Vector2 &position, GoToType goToType) {

    switch (goToType) {
    case noPreference:
        return ControlGoToPos::goToPos(robot, position);

    case ballControl:
        return ControlGoToPos::goToPosBallControl(robot, position);

    case basic:
        return ControlGoToPos::goToPosBasic(std::move(robot), position);

    case force:
        return ControlGoToPos::goToPosForce(std::move(robot), position);


    case luTh:
        return ControlGoToPos::goToPosLuTh(std::move(robot), position);


    case lowLevel:
        return ControlGoToPos::goToPosLowLevel(std::move(robot), position);


    case highLevel:
        return ControlGoToPos::goToPosHighLevel(std::move(robot), position);


    case bezier:
        return ControlGoToPos::goToPosBezier(std::move(robot), position);


    }
    return goToPos(std::move(robot), position);
}
Vector2 ControlGoToPos::goToPosBallControl(RobotPtr robot, Vector2 &targetPos) {
    return gtpBallControl.goToPos(std::move(robot), targetPos);
}

Vector2 ControlGoToPos::goToPosBasic(RobotPtr robot, Vector2 &targetPos) {

    if (! robot) return {};

    Vector2 error;
    error.x = targetPos.x - robot->pos.x;
    error.y = targetPos.y - robot->pos.y;
    double dist = error.length();

    // Serial
    pid.setP(7.0);
    pid.setI(1.4);
    pid.setD(1.1);

//    // Grsim
//    pid.setP(3.0);
//    pid.setI(0.0);
//    pid.setD(1.5);

    if (dist < rtt::ai::constants::ROBOT_RADIUS) pid.setD(0.0);

    return pid.controlPIR(error, robot->vel);
}

Vector2 ControlGoToPos::goToPosForce(RobotPtr robot, Vector2 &targetPos) {
    return {};
}

Vector2 ControlGoToPos::goToPosLuTh(RobotPtr robot, Vector2 &targetPos) {

    return gtpLuth.goToPos(std::move(robot), targetPos);

}

Vector2 ControlGoToPos::goToPosLowLevel(RobotPtr robot, Vector2 &targetPos) {
    return {};
}

Vector2 ControlGoToPos::goToPosHighLevel(RobotPtr robot, Vector2 &targetPos) {
    return {};
}

Vector2 ControlGoToPos::goToPosBezier(RobotPtr robot, Vector2 &targetPos) {
    return {};
}

double ControlGoToPos::distanceToTarget(RobotPtr robot, Vector2 &targetPos) {

    double dx = targetPos.x - robot->pos.x;
    double dy = targetPos.y - robot->pos.y;
    Vector2 deltaPos = {dx, dy};
    return deltaPos.length();
}

void ControlGoToPos::setAvoidBall(bool _avoidBall) {
    // Add a function to avoid the ball for all goToPos's

    //gtpBallControl.setAvoidBall(true);
    gtpLuth.setAvoidBall(_avoidBall);
}

void ControlGoToPos::setCanGoOutsideField(bool _canGoOutsideField) {
    // Add a function to make sure the robot does not go out of the field for all goToPos's

    gtpLuth.setCanGoOutsideField(_canGoOutsideField);
}

} //control
} //ai
} //rtt