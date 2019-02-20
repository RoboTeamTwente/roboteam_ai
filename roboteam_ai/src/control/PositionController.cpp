//
// Created by thijs on 10-12-18.
//


#include "PositionController.h"

namespace rtt {
namespace ai {
namespace control {

ControlGoToPos::ControlGoToPos() = default;

void ControlGoToPos::clear(GoToType goToType) {
    PIDHasInitialized = false;

    switch (goToType) {
    case noPreference:break;
    case ballControl:break;
    case basic:break;
    case force:break;
    case numTree: {
        numTreeController.clear();
        break;
    }

    }
}

PosVelAngle ControlGoToPos::goToPos(RobotPtr robot, Vector2 &position) {
    GoToType goToType = basic;
    //TODO: do stuff that determines which gtp to use...

    return ControlGoToPos::goToPos(std::move(robot), position, goToType);
}

PosVelAngle ControlGoToPos::goToPos(RobotPtr robot, Vector2 &position, GoToType goToType) {
    if (!robot)
        return {};

    switch (goToType) {
    case noPreference:
        return ControlGoToPos::goToPos(robot, position);

    case ballControl:
        return ControlGoToPos::goToPosBallControl(robot, position);

    case basic:
        return ControlGoToPos::goToPosBasic(std::move(robot), position);

    case force:
        return ControlGoToPos::goToPosForce(std::move(robot), position);

    case numTree:
        return ControlGoToPos::numTreePosControl(std::move(robot), position);
    }
    return goToPos(std::move(robot), position);
}

PosVelAngle ControlGoToPos::goToPosBallControl(RobotPtr robot, Vector2 &targetPos) {
    return gtpBallControl.goToPos(std::move(robot), targetPos);
}

PosVelAngle ControlGoToPos::goToPosBasic(RobotPtr robot, Vector2 &targetPos) {

    if (! robot) return {};

    Vector2 error;
    error.x = targetPos.x - robot->pos.x;
    error.y = targetPos.y - robot->pos.y;
    double dist = error.length();
    static bool far = true;
    if (dist > rtt::ai::Constants::ROBOT_RADIUS() and ! far) {
        velPID.setD(1.5);
        far = true;
    }
    else {
        velPID.setD(0);
        far = false;
    }

    if (dist < rtt::ai::Constants::ROBOT_RADIUS()) velPID.setD(0.0);

    return PosVelAngle({0,0}, velPID.controlPIR(error, robot->vel), 0.0);
}

PosVelAngle ControlGoToPos::goToPosForce(RobotPtr robot, Vector2 &targetPos) {
    return {};
}

PosVelAngle ControlGoToPos::numTreePosControl(RobotPtr robot, Vector2 &targetPos) {
    PosVelAngle target = numTreeController.goToPos(robot,targetPos);
    return pidController(robot, target);
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
    numTreeController.setAvoidBall(_avoidBall);
}

void ControlGoToPos::setCanGoOutsideField(bool _canGoOutsideField) {
    // Add a function to make sure the robot does not go out of the field for all goToPos's
    numTreeController.setCanGoOutsideField(_canGoOutsideField);
}

PosVelAngle ControlGoToPos::pidController(const RobotPtr &robot, PosVelAngle target) {
    PosVelAngle pidCommand;
    if (!PIDHasInitialized)
        initializePID();
    checkInterfacePID();

    Vector2 pidP = Vector2();
    Vector2 pidV = Vector2();

    if (target.pos != Vector2()) {
        pidP = posPID.controlPIR(target.pos - robot->pos, robot->vel);
    }
    if (target.vel != Vector2()) {
        pidV = velPID.controlPIR(target.vel, robot->vel);
    }

    pidCommand.pos = target.pos;
    pidCommand.vel = (pidP + pidV).length() < Constants::MAX_VEL() ?
            (pidP + pidV) : (pidP + pidV).stretchToLength(Constants::MAX_VEL());
    pidCommand.angle = target.angle;
    return pidCommand;
}

/// start the PID for velocity and position control
void ControlGoToPos::initializePID() {
    posPID.reset();
    posPID.setPID(Constants::standardNumTreePosP(),
            Constants::standardNumTreePosP(),
            Constants::standardNumTreePosP());

    velPID.reset();
    velPID.setPID(Constants::standardNumTreeVelP(),
            Constants::standardNumTreeVelP(),
            Constants::standardNumTreeVelP());
}

/// compare current PID values to those set in the interface
void ControlGoToPos::checkInterfacePID() {
    if (velPID.getP() != interface::InterfaceValues::getNumTreeVelP() ||
            velPID.getI() != interface::InterfaceValues::getNumTreeVelI() ||
            velPID.getD() != interface::InterfaceValues::getNumTreeVelD()) {

        velPID.reset();
        velPID.setPID(interface::InterfaceValues::getNumTreeVelP(),
                interface::InterfaceValues::getNumTreeVelI(),
                interface::InterfaceValues::getNumTreeVelD());
    }

    if (posPID.getP() != interface::InterfaceValues::getNumTreePosP() ||
            posPID.getI() != interface::InterfaceValues::getNumTreePosI() ||
            posPID.getD() != interface::InterfaceValues::getNumTreePosD()) {

        posPID.reset();
        posPID.setPID(interface::InterfaceValues::getNumTreePosP(),
                interface::InterfaceValues::getNumTreePosI(),
                interface::InterfaceValues::getNumTreePosD());
    }
}

} //control
} //ai
} //rtt
