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

//TODO: receive horizontal position of the ball relative to the robot using feedback <3

BallHandlePosControl::BallHandlePosControl(bool canMoveInDefenseArea)
        :PosController(false, false, canMoveInDefenseArea) {

    numTreePosController->setAvoidBall(targetBallDistance*0.95);
}

PosVelAngle BallHandlePosControl::getPosVelAngle(const PosController::RobotPtr &robot, const Vector2 &target) {
    return PosController::getPosVelAngle(robot, target);
}

PosVelAngle BallHandlePosControl::getPosVelAngle(const RobotPtr &robot,
        const Vector2 &targetPos, const Angle &targetAngle) {

    BallHandlePosControl::targetPos = targetPos;
    BallHandlePosControl::targetAngle = targetAngle;

    auto ball = world::world->getBall();
    if (!ball) {
        std::cout << "Can't control the ball with no ball, you stupid" << std::endl;
        return {};
    }

    // if the ball is visible, do some fancy stuff
    // TODO: do the fancy stuff
    if (ball->visible) {
        Angle ballAngleRelativeToRobot = (ball->pos - robot->pos).toAngle() - robot->angle;
        // TODO: stuff
    }
    else {

    }

    //TODO: make seperate functions, low priority for now

    // if we do not have the ball yet, go get it
    {
        bool robotDoesNotHaveBall = ! robot->hasBall();
        double robotIsTooFarFromBall = (robot->pos - ball->pos).length2() > maxBallDistance;

        if (robotDoesNotHaveBall || robotIsTooFarFromBall) {
            Vector2 target = ball->pos + (robot->pos - ball->pos).stretchToLength(targetBallDistance);
            return numTreePosController->getPosVelAngle(robot, target);
        }
    }

    double deltaPosSquared = (targetPos - robot->pos).length2();

    // if the distance to the target is large, rotate with the ball forwards and move towards the target
    {
        if (deltaPosSquared > 1.0) {
            // rotate TOWARDS the target
            Angle newTargetAngle = targetPos - robot->pos;
            if (abs(newTargetAngle - robot->angle) > angleErrorMargin) {
                return rotateWithBall(robot, newTargetAngle, defaultRotate);
            }
            else {
                return travelWithBall(robot, targetPos, forwards);
            }
        }
    }

    // if the distance to the target is small (but d>errorMargin) and the robot cannot rotate around the ball
    {
        bool robotIsAtLocation = deltaPosSquared < errorMargin * errorMargin;
        bool robotCanRotateAroundBallToTarget = (targetPos - ball->pos).length2() < maxBallDistance;

        if (! robotCanRotateAroundBallToTarget && ! robotIsAtLocation) {
            // rotate AWAY from the target
            Angle newTargetAngle = (robot->pos - targetPos).toAngle();
            Angle deltaAngle = newTargetAngle - robot->angle;
            if (abs(deltaAngle) > angleErrorMargin) {
                return rotateWithBall(robot, newTargetAngle, fastest);
            }
            else {
                return travelWithBall(robot, targetPos, backwards);
            }
        }

        if (robotCanRotateAroundBallToTarget && !robotIsAtLocation) {
            // rotate AROUND the ball
            Angle newTargetAngle = (ball->pos - targetPos).toAngle();
            Angle deltaAngle = newTargetAngle - robot->angle;
            if (abs(deltaAngle) > angleErrorMargin) {
                return rotateWithBall(robot, newTargetAngle, rotateAroundBall);
            }
        }

        if (robotIsAtLocation) {
            Angle deltaAngle = targetAngle - robot->angle;
            if (abs(deltaAngle) > angleErrorMargin) {
                return rotateWithBall(robot, targetAngle, rotateAroundRobot);
            }
            else {
                return {};
            }
        }
    }
}

void BallHandlePosControl::checkInterfacePID() {
    auto newPid = interface::InterfaceValues::getBasicPid();
    updatePid(newPid);
}

PosVelAngle BallHandlePosControl::rotateWithBall(const PosController::RobotPtr &robot,
        const Angle &targetAngle, RotateStrategy rotateStrategy) {

    Angle deltaAngle = targetAngle - robot->angle;

    switch (rotateStrategy) {
    case rotateAroundBall: {
        PosVelAngle pva;
        pva.pos = robot->pos;
        pva.angle = (ball->pos - robot->pos).toAngle();
        if (deltaAngle > 0) pva.vel = (ball->pos - robot->pos).rotate(M_PI_2).toAngle();
        else                pva.vel = (ball->pos - robot->pos).rotate(-M_PI_2).toAngle();

        double distanceToBallSquared = (ball->pos - robot->pos).length2();
        if (distanceToBallSquared <= targetBallDistance) pva.vel += robot->pos - ball->pos;
        if (distanceToBallSquared > maxBallDistance) pva.vel += ball->pos - robot->pos;

        return pva;
    }
    }
    return {};
}

PosVelAngle BallHandlePosControl::travelWithBall(const PosController::RobotPtr &robot,
        const Vector2 &targetPos, TravelStrategy travelStrategy) {

    return {};
}

} //control
} //ai
} //rtt