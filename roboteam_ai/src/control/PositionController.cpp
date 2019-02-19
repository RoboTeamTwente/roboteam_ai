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
    case luTh_OLD: {
        gtpLuth.clear();
        break;
    }
    }
}

Vector2 ControlGoToPos::goToPos(RobotPtr robot, Vector2 &position) {
    GoToType goToType = basic;
    //TODO: do stuff that determines which gtp to use...

    return ControlGoToPos::goToPos(std::move(robot), position, goToType);
}

Vector2 ControlGoToPos::goToPos(RobotPtr robot, Vector2 &position, GoToType goToType) {
    if (!robot)
        return Vector2();

    switch (goToType) {
    case noPreference:
        return ControlGoToPos::goToPos(robot, position);

    case ballControl:
        return ControlGoToPos::goToPosBallControl(robot, position);

    case basic:
        return ControlGoToPos::goToPosBasic(std::move(robot), position);

    case force:
        return ControlGoToPos::goToPosForce(std::move(robot), position);

    case luTh_OLD:
        return ControlGoToPos::goToPosLuTh(std::move(robot), position);

    case numTree:
        return ControlGoToPos::goToPosClean(std::move(robot),position);
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

    return velPID.controlPIR(error, robot->vel);
}

Vector2 ControlGoToPos::goToPosForce(RobotPtr robot, Vector2 &targetPos) {
    return {};
}

Vector2 ControlGoToPos::goToPosLuTh(RobotPtr robot, Vector2 &targetPos) {

    return gtpLuth.goToPos(std::move(robot), targetPos);

}

Vector2 ControlGoToPos::goToPosClean(RobotPtr robot, Vector2 &targetPos) {
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
    gtpLuth.setAvoidBall(_avoidBall);
    numTreeController.setAvoidBall(_avoidBall);
}

void ControlGoToPos::setCanGoOutsideField(bool _canGoOutsideField) {
    // Add a function to make sure the robot does not go out of the field for all goToPos's

    gtpLuth.setCanGoOutsideField(_canGoOutsideField);
    numTreeController.setCanGoOutsideField(_canGoOutsideField);

}

Vector2 ControlGoToPos::pidController(RobotPtr robot, PosVelAngle target) {
    if (!PIDHasInitialized)
        initializePID();
    checkInterfacePID();

    Vector2 pidV = velPID.controlPIR(target.vel, robot->vel);
    Vector2 pidP = posPID.controlPIR(target.pos - robot->pos, robot->vel);
    Vector2 total = pidV + pidP;
    return total.length() < Constants::MAX_VEL() ? total : total.stretchToLength(Constants::MAX_VEL());
}

/// start the PID for velocity and position control
void ControlGoToPos::initializePID() {
    posPID.reset();
    posPID.setPID(Constants::standard_luth_Pos_P(),
            Constants::standard_luth_Pos_P(),
            Constants::standard_luth_Pos_P());

    velPID.reset();
    velPID.setPID(Constants::standard_luth_Vel_P(),
            Constants::standard_luth_Vel_P(),
            Constants::standard_luth_Vel_P());
}

/// compare current PID values to those set in the interface
void ControlGoToPos::checkInterfacePID() {
    if (velPID.getP() != interface::InterfaceValues::getLuthVelP() ||
            velPID.getI() != interface::InterfaceValues::getLuthVelI() ||
            velPID.getD() != interface::InterfaceValues::getLuthVelD()) {

        velPID.reset();
        velPID.setPID(interface::InterfaceValues::getLuthVelP(),
                interface::InterfaceValues::getLuthVelI(),
                interface::InterfaceValues::getLuthVelD());
    }

    if (posPID.getP() != interface::InterfaceValues::getLuthPosP() ||
            posPID.getI() != interface::InterfaceValues::getLuthPosI() ||
            posPID.getD() != interface::InterfaceValues::getLuthPosD()) {

        posPID.reset();
        posPID.setPID(interface::InterfaceValues::getLuthPosP(),
                interface::InterfaceValues::getLuthPosI(),
                interface::InterfaceValues::getLuthPosD());
    }
}

} //control
} //ai
} //rtt
