//
// Created by thijs on 18-12-18.
//

#include <roboteam_ai/src/interface/InterfaceValues.h>
#include "ControlGoToPosBallControl.h"
#include "PosVelAngle.h"

namespace rtt {
namespace ai {
namespace control {


ControlGoToPosBallControl::ControlGoToPosBallControl(bool avoidBall, bool canMoveOutsideField, bool canMoveInDefenseArea)
                                                     : PosController(avoidBall, canMoveOutsideField, canMoveInDefenseArea) {

}

PosVelAngle ControlGoToPosBallControl::getPosVelAngle(RobotPtr robot, Vector2 &targetPos) {
    return {};
}

void ControlGoToPosBallControl::checkInterfacePID() {
    //TODO use other pid than the numtree pid
    auto newPid = interface::InterfaceValues::getNumTreePid();
    updatePid(newPid);
}


} //control
} //ai
} //rtt