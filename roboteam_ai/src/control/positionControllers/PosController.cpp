//
// Created by mrlukasbos on 27-3-19.
//

#include <roboteam_ai/src/interface/InterfaceValues.h>
#include <roboteam_ai/src/control/ControlUtils.h>
#include "PosController.h"


namespace rtt {
namespace ai {
namespace control {

PosController::PosController() { }

/// apply a posPID and a velPID over a posVelAngle for better control
PosVelAngle PosController::controlWithPID(const RobotPtr &robot, PosVelAngle target) {
    if (getPIDFromInterface) checkInterfacePID();

    PosVelAngle pidCommand;
    pidCommand.pos = target.pos;
    pidCommand.vel = control::ControlUtils::velocityLimiter(calculatePIDs(robot, target), Constants::MAX_VEL());
    pidCommand.angle = target.angle;
    return pidCommand;
}

// actually calculate the pids
Vector2 PosController::calculatePIDs(const PosController::RobotPtr &robot, PosVelAngle &target) {
    Vector2 pidP = Vector2();
    if (target.pos != Vector2()) {

        auto x = xpid.getOutput(0, target.pos.x - robot->pos.x);
        auto y = ypid.getOutput(0, target.pos.y - robot->pos.y);

        pidP = {x, y};
    }

    return pidP;
}

/// compare current PID values to those set in the interface
void PosController::checkInterfacePID() {
    using if_values = interface::InterfaceValues;
    //posPID.reset();
   // posPID.setPID(if_values::getNumTreePosP(), if_values::getNumTreePosI(), if_values::getNumTreePosD());
   PID newPid = PID(if_values::getNumTreePosP(), if_values::getNumTreePosI(), if_values::getNumTreePosD());
   newPid.setOutputLimits(-8,8);
   xpid = newPid;
   ypid = newPid;
}


// Getters & Setters
bool PosController::getCanMoveOutOfField() const {
    return canMoveOutOfField;
}

void PosController::setCanMoveOutOfField(bool canMoveOutOfField) {
    this->canMoveOutOfField = canMoveOutOfField;
}

bool PosController::getCanMoveInDefenseArea() const {
    return canMoveInDefenseArea;
}

void PosController::setCanMoveInDefenseArea(bool canMoveInDefenseArea) {
     this->canMoveInDefenseArea = canMoveInDefenseArea;
}

bool PosController::getAvoidBall() const {
    return avoidBall;
}

void PosController::setAvoidBall(bool avoidBall) {
    this->avoidBall = avoidBall;
}


} // control
} // ai
} // rtt