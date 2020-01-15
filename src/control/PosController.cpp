//
// Created by mrlukasbos on 27-3-19.
//

#include <interface/api/Output.h>
#include <control/ControlUtils.h>
#include <utilities/GameStateManager.hpp>
#include "control/PosController.h"
#include "world_old/Robot.h"

namespace rtt::ai::control {

PosController::PosController(double avoidBall, bool canMoveOutOfField, bool canMoveInDefenseArea)
        :customAvoidBallDistance(avoidBall), customCanMoveOutOfField(canMoveOutOfField), customCanMoveInDefenseArea(canMoveInDefenseArea) {
    xpid.setOutputLimits(- 8, 8);
    xpid.setOutputRampRate(100);

    ypid.setOutputLimits(- 8, 8);
    ypid.setOutputRampRate(100);
}

/// apply a posPID and a velPID over a posVelAngle for better control
RobotCommand PosController::controlWithPID(const RobotPtr &robot, RobotCommand target) {
    if (getPIDFromInterface) checkInterfacePID();
    RobotCommand pidCommand;
    pidCommand.pos = target.pos;
    pidCommand.angle = target.angle;
    pidCommand.vel = calculatePIDs(robot, target);

    // set previous velocity to the current velocity and return the command.
    return pidCommand;
}

// actually calculate the pids
Vector2 PosController::calculatePIDs(const PosController::RobotPtr &robot, RobotCommand &target) {
    auto x = xpid.getOutput(robot->pos.x, target.pos.x);
    auto y = ypid.getOutput(robot->pos.y, target.pos.y);
    Vector2 pidP(x, y);
    return pidP;
}

// Getters & Setters
bool PosController::getCanMoveOutOfField(int robotID) const {
    if (GameStateManager::canMoveOutsideField(robotID, *rtt::Settings::instance())){
        return customCanMoveOutOfField;
    }
    return false;
}

void PosController::setCanMoveOutOfField(bool moveOutOfField) {
    customCanMoveOutOfField = moveOutOfField;
}

bool PosController::getCanMoveInDefenseArea(int robotID) const {
    if (GameStateManager::canEnterDefenseArea(robotID, *rtt::Settings::instance())){
        return customCanMoveInDefenseArea;
    }
    return false;
}

void PosController::setCanMoveInDefenseArea(bool moveInDefenseArea) {
    customCanMoveInDefenseArea = moveInDefenseArea;
}

double PosController::getAvoidBallDistance() const {
    return std::max(customAvoidBallDistance, GameStateManager::getCurrentGameState().getRuleSet().minDistanceToBall);
}

void PosController::setAvoidBallDistance(double ballDistance) {
    customAvoidBallDistance = ballDistance;
}

///This function should NEVER be called except for the keeper. If you find yourself needing to do this you are probably doing something wrong
///This is a temporary fix for the keeperPID and keeper intercept PID's. In the future should probalby be properly refactored
void PosController::setAutoListenToInterface(bool listenToInterface) {
    getPIDFromInterface=listenToInterface;
}

void PosController::updatePid(pidVals pid) {
    if (lastPid != pid) {
        xpid.setPID(pid);
        ypid.setPID(pid);
        lastPid = pid;
    }
}

} // rtt