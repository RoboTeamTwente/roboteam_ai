//
// Created by thijs on 18-12-18.
//

#include "ControlGoToPosBallControl.h"
#include "PosVelAngle.h"

namespace rtt {
namespace ai {
namespace control {


ControlGoToPosBallControl::ControlGoToPosBallControl(bool avoidBall, bool canMoveOutsideField, bool canMoveInDefenseArea)
                                                     : PosController(avoidBall, canMoveOutsideField, canMoveInDefenseArea) {

}

PosVelAngle ControlGoToPosBallControl::getPosVelAngle(const RobotPtr &robot, Vector2 &targetPos) {
    return {};
}



} //control
} //ai
} //rtt