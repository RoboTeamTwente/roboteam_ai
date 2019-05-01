//
// Created by mrlukasbos on 27-3-19.
//

#include <roboteam_ai/src/interface/InterfaceValues.h>
#include <roboteam_ai/src/control/ControlUtils.h>
#include "PosController.h"


namespace rtt {
namespace ai {
namespace control {

PosController::PosController(double avoidBall, bool canMoveOutOfField, bool canMoveInDefenseArea)
        : avoidBallDistance(avoidBall), canMoveOutOfField(canMoveOutOfField), canMoveInDefenseArea(canMoveInDefenseArea) {
    xpid.setOutputLimits(-8,8);
    xpid.setOutputRampRate(100);

    ypid.setOutputLimits(-8,8);
    ypid.setOutputRampRate(100);
}

/// apply a posPID and a velPID over a posVelAngle for better control
PosVelAngle PosController::controlWithPID(const RobotPtr &robot, PosVelAngle target) {
    if (getPIDFromInterface) checkInterfacePID();
    PosVelAngle pidCommand;
    pidCommand.pos = target.pos;
    pidCommand.angle = target.angle;

    // velocity limiter
    pidCommand.vel = calculatePIDs(robot, target);
    pidCommand.vel = control::ControlUtils::velocityLimiter(pidCommand.vel);

    // acceleration limiter
    double maxAcc = control::ControlUtils::calculateMaxAcceleration(pidCommand.vel, pidCommand.angle);
    if (prevVel == 0.0) prevVel = robot->vel.length();
    pidCommand.vel = control::ControlUtils::accelerationLimiter(pidCommand.vel, maxAcc, prevVel);

    // set previous velocity to the current velocity and return the command.
    prevVel = pidCommand.vel.length();
    return pidCommand;
}

// actually calculate the pids
Vector2 PosController::calculatePIDs(const PosController::RobotPtr &robot, PosVelAngle &target) {
    Vector2 pidP = Vector2();
    if (target.pos != Vector2()) {

        auto x = xpid.getOutput(robot->pos.x, target.pos.x);
        auto y = ypid.getOutput(robot->pos.y, target.pos.y);

        pidP = {x, y};
    }

    return pidP;
}

// Getters & Setters
bool PosController::getCanMoveOutOfField() const {
    return canMoveOutOfField;
}

void PosController::setCanMoveOutOfField(bool moveOutOfField) {
    canMoveOutOfField = moveOutOfField;
}

bool PosController::getCanMoveInDefenseArea() const {
    return canMoveInDefenseArea;
}

void PosController::setCanMoveInDefenseArea(bool moveInDefenseArea) {
     canMoveInDefenseArea = moveInDefenseArea;
}

double PosController::getAvoidBall() const {
    return avoidBallDistance;
}

void PosController::setAvoidBall(double ballDistance) {
    avoidBallDistance = ballDistance;
}

void PosController::updatePid(pidVals pid) {
    if (lastPid != pid) {
        xpid.setPID(pid);
        ypid.setPID(pid);
        lastPid = pid;
    }
}


} // control
} // ai
} // rtt