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

//TODO: receive horizontal position of the ball using feedback <3

BallHandlePosControl::BallHandlePosControl(bool canMoveInDefenseArea)
        :PosController(false, false, canMoveInDefenseArea) {

    numTreePosController->setAvoidBall(0.15);
}

PosVelAngle BallHandlePosControl::getPosVelAngle(const PosController::RobotPtr &robot, const Vector2 &target) {
    return PosController::getPosVelAngle(robot, target);
}

PosVelAngle BallHandlePosControl::getPosVelAngle(const RobotPtr &robot,
        const Vector2 &targetPos, const Angle &targetAngle) {

    auto ball = world::world->getBall();


    // if the ball is visible, do some fancy stuff
    // TODO: do the fancy stuff
    if (ball->visible) {
        Angle ballAngleRelativeToRobot = (ball->pos - robot->pos).toAngle() - robot->angle;
        // TODO: stuff
    }

    // if we do not have the ball yet, go get it
    bool robotDoesNotHaveBall = ! robot->hasBall();
    bool robotIsTooFarFromBall = (robot->pos - ball->pos).length2() > maxBallDistance;

    if (robotDoesNotHaveBall || robotIsTooFarFromBall) {
        Vector2 target = ball->pos + (robot->pos - ball->pos).stretchToLength(maxBallDistance*0.8);
        return numTreePosController->getPosVelAngle(robot, target);
    }
    double deltaPosSquared = (targetPos - robot->pos).length2();
    if (deltaPosSquared > 1.0) {
        // rotate TOWARDS the target for large
        Angle newTargetAngle = targetPos - robot->pos;
        Angle deltaAngle = newTargetAngle - robot->angle;
        if (abs(deltaAngle) > angleErrorMargin) {
            return rotateWithBall(robot, newTargetAngle, defaultRotate);
        }
        else {
            return travelWithBall(robot, targetPos, forwards);
        }
    }
    else if (deltaPosSquared > errorMargin*errorMargin) {
        // rotate AWAY from the target for close distances
        Angle newTargetAngle = (robot->pos - targetPos).toAngle();
        Angle deltaAngle = newTargetAngle - robot->angle;
        if (abs(deltaAngle) > angleErrorMargin) {
            return rotateWithBall(robot, newTargetAngle, defaultRotate);
        }
        else {
            return travelWithBall(robot, targetPos, backwards);
        }
    }
    else {
        Angle deltaAngle = targetAngle - robot->angle;
        if (abs(deltaAngle) > angleErrorMargin) {
            return rotateWithBall(robot, targetAngle, rotateAroundBall);
        }
        else {
            return {};
        }
    }
}

void BallHandlePosControl::checkInterfacePID() {
    auto newPid = interface::InterfaceValues::getBasicPid();
    updatePid(newPid);
}

PosVelAngle BallHandlePosControl::rotateWithBall(const PosController::RobotPtr &robot,
        const Angle &targetAngle, RotateStrategy rotateStrategy) {

    return {};
}

PosVelAngle BallHandlePosControl::travelWithBall(const PosController::RobotPtr &robot,
        const Vector2 &targetPos, TravelStrategy travelStrategy) {

    return {};
}

} //control
} //ai
} //rtt