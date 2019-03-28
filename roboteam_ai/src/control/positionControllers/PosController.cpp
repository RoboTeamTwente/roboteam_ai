//
// Created by mrlukasbos on 27-3-19.
//

#include <roboteam_ai/src/interface/InterfaceValues.h>
#include "PosController.h"


namespace rtt {
namespace ai {
namespace control {

PosController::PosController() {
    posPID.reset();
}

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
    if (target.pos != Vector2() && !posPID.isZero()) {
        pidP = posPID.controlPID(target.pos - robot->pos);
    }

    Vector2 pidV = Vector2();
    if (target.vel != Vector2() && !velPID.isZero()) {
        pidV = velPID.controlPIR(target.vel, robot->vel);
    }

    return pidP + pidV;
}

/// compare current PID values to those set in the interface
void PosController::checkInterfacePID() {
    using if_values = interface::InterfaceValues;
    posPID.reset();
    posPID.setPID(if_values::getNumTreePosP(), if_values::getNumTreePosI(), if_values::getNumTreePosD());

    velPID.reset();
    velPID.setPID(if_values::getNumTreeVelP(), if_values::getNumTreeVelI(), if_values::getNumTreeVelD());
}


} // control
} // ai
} // rtt