//
// Created by mrlukasbos on 27-3-19.
//

#include <interface/api/Output.h>
#include <utilities/GameStateManager.hpp>
#include "control/PosController.h"
#include "world/Robot.h"

namespace rtt {
namespace ai {
namespace control {

PosController::PosController(double avoidBall, bool canMoveOutOfField, bool canMoveInDefenseArea)
        :customAvoidBallDistance(avoidBall), customCanMoveOutOfField(canMoveOutOfField), customCanMoveInDefenseArea(canMoveInDefenseArea) {
    pid.setOutputLimits(- 8, 8, -8, 8);
    pid.setOutputRampRate(100, 100);
}

/// apply a posPID and a velPID over a posVelAngle for better control
RobotCommand PosController::controlWithPID(const RobotPtr &robot, const RobotCommand& target) {
    if (getPIDFromInterface) checkInterfacePID();
    RobotCommand pidCommand;
    pidCommand.pos = target.pos;
    pidCommand.angle = target.angle;
    pidCommand.vel = pid.computeOutput(target.vel, robot->vel);

    // set previous velocity to the current velocity and return the command.
    return pidCommand;
}

// Getters & Setters
bool PosController::getCanMoveOutOfField(int robotID) const {
    if (GameStateManager::canMoveOutsideField(robotID)){
        return customCanMoveOutOfField;
    }
    return false;
}

void PosController::setCanMoveOutOfField(bool moveOutOfField) {
    customCanMoveOutOfField = moveOutOfField;
}

bool PosController::getCanMoveInDefenseArea(int robotID) const {
    if (GameStateManager::canEnterDefenseArea(robotID)){
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
        //modify the PID controller with the new values; also, the F value is set to zero
        this->pid = PidTwoAxesController(std::tuple_cat(pid,std::make_tuple(0)));

        lastPid = pid;
    }
}

} // control
} // ai
} // rtt