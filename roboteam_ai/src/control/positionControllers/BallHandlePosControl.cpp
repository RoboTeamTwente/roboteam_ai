//
// Created by thijs on 18-12-18.
//

#include <roboteam_ai/src/interface/InterfaceValues.h>
#include "BallHandlePosControl.h"
#include "PosVelAngle.h"

namespace rtt {
namespace ai {
namespace control {

BallHandlePosControl::BallHandlePosControl(bool canMoveInDefenseArea)
    :PosController(false, false, canMoveInDefenseArea) {}

PosVelAngle BallHandlePosControl::getPosVelAngle(const RobotPtr &robot, Vector2 &targetPos) {
    return {};
}

void BallHandlePosControl::checkInterfacePID() {
    auto newPid = interface::InterfaceValues::getBasicPid();
    updatePid(newPid);
}


} //control
} //ai
} //rtt