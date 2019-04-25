//
// Created by thijs on 18-12-18.
//

#include <roboteam_ai/src/interface/InterfaceValues.h>
#include "BallHandlePosControl.h"
#include "PosVelAngle.h"
#include "NumTreePosControl.h"

namespace rtt {
namespace ai {
namespace control {

BallHandlePosControl::BallHandlePosControl(bool canMoveInDefenseArea)
        :PosController(false, false, canMoveInDefenseArea) {

    numTreePosController->setAvoidBall(0.15);
}

PosVelAngle BallHandlePosControl::getPosVelAngle(const RobotPtr &robot,
        const Vector2 &targetPos, const Angle &targetAngle) {

    auto ball = world::world->getBall();
    if (! robot->hasBall() && (robot->pos - ball->pos).length2() > 0.5) {
        Vector2 target = ball->pos + (robot->pos - ball->pos).stretchToLength(0.25);
        return numTreePosController->getPosVelAngle(robot, target);
    }
    return {};
}

void BallHandlePosControl::checkInterfacePID() {
    auto newPid = interface::InterfaceValues::getBasicPid();
    updatePid(newPid);
}

PosVelAngle BallHandlePosControl::getPosVelAngle(const PosController::RobotPtr &robot, const Vector2 &target) {
    return PosController::getPosVelAngle(robot, target);
}

} //control
} //ai
} //rtt