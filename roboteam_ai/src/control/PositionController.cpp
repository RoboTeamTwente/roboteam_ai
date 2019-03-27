//
// Created by thijs on 10-12-18.
//


#include "PositionController.h"
#include "../utilities/Field.h"

namespace rtt {
namespace ai {
namespace control {

PositionController::PositionController() = default;

void PositionController::clear(PosControlType goToType) {
    PIDHasInitialized = false;

    switch (goToType) {
    case PosControlType::NO_PREFERENCE:break;
    case PosControlType::BALL_CONTROL:break;
    case PosControlType::BASIC:break;
    case PosControlType::FORCE:break;
    case PosControlType::NUMERIC_TREES: {
        numTreeController.clear();
        break;
    }
    default:break;
    }
}

PosVelAngle PositionController::goToPos(RobotPtr robot, Vector2 &position) {
    if (! robot) {
        ROS_ERROR("Error in PositionController->goToPos(robot %i): robot does not exist in world", robot->id);
        return {};
    }
    PosControlType goToType = PosControlType::NUMERIC_TREES;
    //TODO: do stuff that determines which gtp to use...

    return PositionController::goToPos(std::move(robot), position, goToType);
}

PosVelAngle PositionController::goToPos(RobotPtr robot, Vector2 &position, PosControlType goToType) {
    if (! robot)
        return {};

    switch (goToType) {
    case PosControlType::NO_PREFERENCE:
        return PositionController::goToPos(robot, position);
    case PosControlType::BALL_CONTROL:
        return PositionController::ballControl(robot, position);
    case PosControlType::BASIC:
        return PositionController::basic(robot, position);
    case PosControlType::FORCE:
        return PositionController::force(robot, position);
    case PosControlType::NUMERIC_TREES:
    default:
        return PositionController::numTree(robot, position);
    }
}

PosVelAngle PositionController::ballControl(RobotPtr robot, Vector2 &targetPos) {
    return ballControlController.goToPos(std::move(robot), targetPos);
}


PosVelAngle PositionController::numTree(RobotPtr robot, Vector2 &targetPos) {
    PosVelAngle target = numTreeController.goToPos(robot, targetPos);
    if (target.isZero()) {
        return force(robot, targetPos);
    }
    else {
        return pidController(robot, target);
    }
}

PosVelAngle PositionController::pidController(const RobotPtr &robot, PosVelAngle target, bool checkInterface) {
    PosVelAngle pidCommand;

    if (! PIDHasInitialized) initializePID();

    // force that we always check the interface to force the velPID to have proper values!
    // TODO this should be fixed!
    //if (checkInterface)
    checkInterfacePID();

    Vector2 pidP = Vector2();
    Vector2 pidV = Vector2();

    if (target.pos != Vector2() && ! (posPID.getP() == 0.0 && posPID.getI() == 0.0 && posPID.getD() == 0.0)) {
        pidP = posPID.controlPID(target.pos - robot->pos);
    }
    if (target.vel != Vector2() && ! (velPID.getP() == 0.0 && velPID.getI() == 0.0 && velPID.getD() == 0.0)) {
        pidV = velPID.controlPIR(target.vel, robot->vel);
    }

    pidCommand.pos = target.pos;
    pidCommand.vel = (pidP + pidV).length() < Constants::MAX_VEL() ?
                     (pidP + pidV) : (pidP + pidV).stretchToLength(Constants::MAX_VEL());
    pidCommand.angle = target.angle;
    return pidCommand;
}

/// start the PID for velocity and position control
void PositionController::initializePID() {
    posPID.reset();
    posPID.setPID(Constants::standardNumTreePosP(),
            Constants::standardNumTreePosP(),
            Constants::standardNumTreePosP());

    velPID.reset();
    velPID.setPID(Constants::standardNumTreeVelP(),
            Constants::standardNumTreeVelP(),
            Constants::standardNumTreeVelP());

    usingManualPID = false;
}

void PositionController::initializePID(double posP, double posI, double posD, double velP, double velI, double velD) {
    posPID.reset();
    posPID.setPID(posP, posI, posD);

    velPID.reset();
    velPID.setPID(velP, velI, velD);

    usingManualPID = true;
}

/// compare current PID values to those set in the interface
void PositionController::checkInterfacePID() {
    if (usingManualPID)
        return;

    using if_values = interface::InterfaceValues;

    if (velPID.getP() != if_values::getNumTreeVelP() ||
            velPID.getI() != if_values::getNumTreeVelI() ||
            velPID.getD() != if_values::getNumTreeVelD()) {

        velPID.reset();
        velPID.setPID(if_values::getNumTreeVelP(), if_values::getNumTreeVelI(), if_values::getNumTreeVelD());
    }

    if (posPID.getP() != if_values::getNumTreePosP() || posPID.getI() != if_values::getNumTreePosI() || posPID.getD() != if_values::getNumTreePosD()) {
        posPID.reset();
        posPID.setPID(if_values::setNumTreePosP(), if_values::getNumTreePosI(), if_values::getNumTreePosD());
    }
}

} //control
} //ai
} //rtt
