//
// Created by thijs on 10-12-18.
//


#include "PositionController.h"

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

PosVelAngle PositionController::basic(RobotPtr robot, Vector2 &targetPos) {

    PosVelAngle posVelAngle;
    Vector2 error;
    error.x = targetPos.x - robot->pos.x;
    error.y = targetPos.y - robot->pos.y;
    velPID.reset();
    posPID.reset();
    if (error.length() < rtt::ai::Constants::ROBOT_RADIUS())
        posPID.setPID(3.0, 1.0, 0.2);
    else
        posPID.setPID(3.0, 0.5, 1.5);

    PIDHasInitialized = true;
    posVelAngle.vel = error;
    return pidController(robot, posVelAngle, false);
}

PosVelAngle PositionController::force(RobotPtr robot, Vector2 &targetPos) {
    double forceRadius;
    if ((targetPos - robot->pos).length() < 0.1) {
        if (interface::InterfaceValues::showDebugNumTreeInfo())
            std::cout << "close to target, using basic gtp" << std::endl;
        return basic(robot, targetPos);
    }
    else if ((targetPos - robot->pos).length() < Constants::MIN_DISTANCE_FOR_FORCE()) {
        forceRadius = Constants::ROBOT_RADIUS_MAX()*2.0;
        initializePID(3.0, 1.0, 0.2);
    }
    else {
        forceRadius = Constants::ROBOT_RADIUS_MAX()*8.0;
        initializePID(3.0, 0.5, 1.5);
    }

    PosVelAngle target;
    auto w = world::world->getWorld();
    Vector2 force = (targetPos - robot->pos);
    force = (force.length() > 3.0) ?
            force.stretchToLength(3.0) : force;

    for (auto bot : w.us)
        force += ControlUtils::calculateForce((Vector2) robot->pos - bot.pos, 1, forceRadius);
    for (auto bot : w.them)
        force += ControlUtils::calculateForce((Vector2) robot->pos - bot.pos, 2, forceRadius);
    if (avoidBall)
        force += ControlUtils::calculateForce((Vector2) robot->pos - w.ball.pos, 1, forceRadius);
    if (!canGoOutsideField)
        force += world::field->pointIsInField(robot->pos, 0.5) ?
                Vector2() : ControlUtils::calculateForce(Vector2(-1.0,-1.0) / robot->pos, 1, 99.9);

    force = (force.length() > 3.0) ?
            force.stretchToLength(3.0) : force;

    target.vel = force;
    return pidController(robot, target);
}

PosVelAngle PositionController::numTree(RobotPtr robot, Vector2 &targetPos) {
    PosVelAngle target = numTreeController.goToPos(robot, targetPos);
    if (target.isZero())
        return force(robot, targetPos);
    else
        return pidController(robot, target);
}

void PositionController::setAvoidBall(bool _avoidBall) {
    // Add a function to avoid the ball for all goToPos's
    numTreeController.setAvoidBall(_avoidBall);
    avoidBall = _avoidBall;
}

void PositionController::setCanGoOutsideField(bool _canGoOutsideField) {
    // Add a function to make sure the robot does not go out of the field for all goToPos's
    numTreeController.setCanGoOutsideField(_canGoOutsideField);
    canGoOutsideField = _canGoOutsideField;
}

PosVelAngle PositionController::pidController(const RobotPtr &robot, PosVelAngle target, bool checkInterface) {
    PosVelAngle pidCommand;

    if (! PIDHasInitialized)
        initializePID();
    if (checkInterface)
        checkInterfacePID();

    Vector2 pidP = Vector2();
    Vector2 pidV = Vector2();

    if (target.pos != Vector2() && ! (posPID.getP() == 0.0 && posPID.getI() == 0.0 && posPID.getD() == 0.0)) {
        pidP = posPID.controlPID(target.pos - robot->pos);//, robot->vel);
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

    if (velPID.getP() != interface::InterfaceValues::getNumTreeVelP() ||
            velPID.getI() != interface::InterfaceValues::getNumTreeVelI() ||
            velPID.getD() != interface::InterfaceValues::getNumTreeVelD()) {

        velPID.reset();
        velPID.setPID(interface::InterfaceValues::getNumTreeVelP(),
                interface::InterfaceValues::getNumTreeVelI(),
                interface::InterfaceValues::getNumTreeVelD());
    }

    if (posPID.getP() != interface::InterfaceValues::setNumTreePosP() ||
            posPID.getI() != interface::InterfaceValues::getNumTreePosI() ||
            posPID.getD() != interface::InterfaceValues::getNumTreePosD()) {

        posPID.reset();
        posPID.setPID(interface::InterfaceValues::setNumTreePosP(),
                interface::InterfaceValues::getNumTreePosI(),
                interface::InterfaceValues::getNumTreePosD());
    }
}

} //control
} //ai
} //rtt
