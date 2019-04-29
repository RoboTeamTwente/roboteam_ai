//
// Created by thijs on 18-12-18.
//

#include <roboteam_ai/src/interface/InterfaceValues.h>
#include "BallHandlePosControl.h"

namespace rtt {
namespace ai {
namespace control {

//TODO: receive horizontal position of the ball relative to the robot using feedback <3

BallHandlePosControl::BallHandlePosControl(bool canMoveInDefenseArea)
        : canMoveInDefenseArea(canMoveInDefenseArea){
    numTreePosController->setCanMoveInDefenseArea(canMoveInDefenseArea);
    numTreePosController->setAvoidBall(targetBallDistance*0.95);

}

RobotCommand BallHandlePosControl::getPosVelAngle(const RobotPtr &robot,
        const Vector2 &targetP, const Angle &targetA) {

    auto b = world::world->getBall();
    BallHandlePosControl::ball = b;
    BallHandlePosControl::targetPos = targetP;
    BallHandlePosControl::targetAngle = targetA;

    if (! ball) {
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
        bool robotIsTooFarFromBall = (robot->pos - ball->pos).length2() > maxBallDistance;

        if (robotDoesNotHaveBall || robotIsTooFarFromBall) {
            Vector2 target = ball->pos + (robot->pos - ball->pos).stretchToLength(targetBallDistance);
            auto pva = numTreePosController->getPosVelAngle(robot, target);
            RobotCommand robotCommand;
            robotCommand.angle = pva.angle;
            robotCommand.vel = pva.vel;
            return robotCommand;
        }
    }

    double deltaPosSquared = (targetPos - robot->pos).length2();

    // if the distance to the target is large, rotate with the ball forwards and move towards the target
    {
        if (deltaPosSquared > 1.0) {
            // rotate TOWARDS the target
            targetAngle = targetPos - robot->pos;
            if (abs(targetAngle - robot->angle) > angleErrorMargin) {
                return rotateWithBall(robot, defaultRotate);
            }
            else {
                return travelWithBall(robot, forwards);
            }
        }
    }

    // if the distance to the target is small (but d>errorMargin) and the robot cannot rotate around the ball
    {
        bool robotIsAtLocation = deltaPosSquared < errorMargin*errorMargin;
        bool robotCanRotateAroundBallToTarget = (targetPos - ball->pos).length2() < maxBallDistance;

        if (! robotCanRotateAroundBallToTarget && ! robotIsAtLocation) {
            // rotate AWAY from the target
            targetAngle = (robot->pos - targetPos).toAngle();
            Angle deltaAngle = targetAngle - robot->angle;
            if (abs(deltaAngle) > angleErrorMargin) {
                return rotateWithBall(robot, fastest);
            }
            else {
                return travelWithBall(robot, backwards);
            }
        }

        if (robotCanRotateAroundBallToTarget && ! robotIsAtLocation) {
            // rotate AROUND the ball
            targetAngle = (ball->pos - targetPos).toAngle();
            Angle deltaAngle = targetAngle - robot->angle;
            if (abs(deltaAngle) > angleErrorMargin) {
                return rotateWithBall(robot, rotateAroundBall);
            }
        }

        if (robotIsAtLocation) {
            Angle deltaAngle = targetAngle - robot->angle;
            if (abs(deltaAngle) > angleErrorMargin) {
                return rotateWithBall(robot, rotateAroundRobot);
            }
            else {
                return {};
            }
        }
    }
}

RobotCommand BallHandlePosControl::rotateWithBall(const RobotPtr &robot, RotateStrategy rotateStrategy) {

    RobotCommand pva;
    Angle deltaAngle = targetAngle - robot->angle;

    switch (rotateStrategy) {
    case rotateAroundBall: {
        pva.angle = (ball->pos - robot->pos).toAngle();
        if (deltaAngle > 0) pva.vel = (ball->pos - robot->pos).rotate(  M_PI_2);
        else                pva.vel = (ball->pos - robot->pos).rotate(- M_PI_2);

        double distanceToBallSquared = (ball->pos - robot->pos).length2();
        if (distanceToBallSquared < targetBallDistance) pva.vel += robot->pos - ball->pos;
        if (distanceToBallSquared > maxBallDistance)    pva.vel += ball->pos - robot->pos;

        return pva;
    }
    case rotateAroundRobot:break;
    case fastest:break;
    case safest:break;
    case defaultRotate:break;
    }

    std::cout << "rotate case not handled" << std::endl;
    return {};
}

RobotCommand BallHandlePosControl::travelWithBall(const RobotPtr &robot, TravelStrategy travelStrategy) {

    std::cout << "travel case not handled" << std::endl;
    return {};
}

} //control
} //ai
} //rtt