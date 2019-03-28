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

PosVelAngle PosController::controlWithPID(const RobotPtr &robot, PosVelAngle target) {
    PosVelAngle pidCommand;

    if (getPIDFromInterface) checkInterfacePID();

    Vector2 pidP = Vector2();
    Vector2 pidV = Vector2();

    if (target.pos != Vector2() &&
        !(posPID.getP() == 0.0 && posPID.getI() == 0.0 && posPID.getD() == 0.0)) {
        pidP = posPID.controlPID(target.pos - robot->pos);
    }
    if (target.vel != Vector2() &&
        !(velPID.getP() == 0.0 && velPID.getI() == 0.0 && velPID.getD() == 0.0)) {
        pidV = velPID.controlPIR(target.vel, robot->vel);
    }

    pidCommand.pos = target.pos;
    pidCommand.vel = (pidP + pidV).length() < Constants::MAX_VEL() ?
                     (pidP + pidV) : (pidP + pidV).stretchToLength(Constants::MAX_VEL());
    pidCommand.angle = target.angle;
    return pidCommand;
}


/// compare current PID values to those set in the interface
void PosController::checkInterfacePID() {
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


}
}

