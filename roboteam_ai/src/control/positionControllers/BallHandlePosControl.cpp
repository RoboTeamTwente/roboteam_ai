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
        :canMoveInDefenseArea(canMoveInDefenseArea) {
    numTreePosController.setCanMoveInDefenseArea(canMoveInDefenseArea);
    numTreePosController.setAvoidBall(targetBallDistance*0.95);

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

    //TODO: make seperate functions, low priority for now

    // if we do not have the ball yet, go get it
    {
        bool robotDoesNotHaveBall = ! robot->hasBall();
        bool robotIsTooFarFromBall = (robot->pos - ball->pos).length2() > maxBallDistance;

        if (robotDoesNotHaveBall || robotIsTooFarFromBall) {
            std::cout << "we do not have a ball yet idiot" << std::endl;
            Vector2 target = ball->pos + (robot->pos - ball->pos).stretchToLength(targetBallDistance);
            auto pva = numTreePosController.getPosVelAngle(robot, target);
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
            return rotateWithBall(robot, rotateAroundBall);
        }

        if (abs(targetAngle - robot->angle) > angleErrorMargin*0.25) {
            return rotateWithBall(robot, rotateAroundRobot);
        }
        else {
            return {};
        }
    }
}

RobotCommand BallHandlePosControl::rotateWithBall(const RobotPtr &robot, RotateStrategy rotateStrategy) {
    std::stringstream ss;
    ss << "rotating with strategy: ";
    switch (rotateStrategy) {
    case rotateAroundBall:
        ss << "aroundBall";
        break;
    case rotateAroundRobot:
        ss << "aroundRobot";
        break;
    case fastest:
        ss << "fastest";
        break;
    case safest:
        ss << "safest";
        break;
    case defaultRotate:
        ss << "default";
        break;
    }
    std::cout << ss.str() << std::endl;

    RobotCommand robotCommand;
    Angle deltaAngle = targetAngle - robot->angle;
    Angle compensationAngle = 0;
    if (ball->visible) {
        compensationAngle = (ball->pos - robot->pos).toAngle() - robot->angle;
    }

    switch (rotateStrategy) {
    case rotateAroundBall: {
        robotCommand.angle = (ball->pos - robot->pos).toAngle();
        if (deltaAngle > 0) robotCommand.vel = (ball->pos - robot->pos).rotate(M_PI_2);
        else robotCommand.vel = (ball->pos - robot->pos).rotate(- M_PI_2);

        double distanceToBallSquared = (ball->pos - robot->pos).length2();
        if (distanceToBallSquared < targetBallDistance) robotCommand.vel += robot->pos - ball->pos;
        if (distanceToBallSquared > maxBallDistance) robotCommand.vel += ball->pos - robot->pos;

        return robotCommand;
    }
    case rotateAroundRobot: {
        int direction = targetAngle - robot->angle > 0.0 ? 1 : -1;
        robotCommand.angle = Angle(robot->angle + maxAngularVelocity*direction);// + compensationAngle);
        robotCommand.dribbler = 1;
        return robotCommand;
    }
    case fastest: {
        int direction = targetAngle - robot->angle > 0 ? -1 : 1;
        robotCommand.angle = Angle(robot->angle + maxAngularVelocity*direction);// + compensationAngle);
        robotCommand.dribbler = 1;
        return robotCommand;
    }
    case safest:break;
    case defaultRotate:break;
    }

    std::cout << "rotate case not handled" << std::endl;
    return {};
}

RobotCommand BallHandlePosControl::travelWithBall(const RobotPtr &robot, TravelStrategy travelStrategy) {

    std::stringstream ss;
    ss << "travel with strategy: " << std::endl;
    switch (travelStrategy) {
    case forwards:
        ss << "forwards";
        break;
    case backwards:
        ss << "backwards";
        break;
    case defaultTravel:
        ss << "default";
        break;
    }
    std::cout << ss.str() << std::endl;

    RobotCommand robotCommand;
    Vector2 compensationVelocity = Vector2();
    Angle ballAngleRelativeToRobot = 0;
    if (ball->visible) {
        ballAngleRelativeToRobot = (ball->pos - robot->pos).toAngle() - robot->angle;
    }

    switch (travelStrategy) {
    case forwards: {

        Angle maxOffset = 0.02*M_PI; // in rad
        bool ballIsTooFarLeft = (ballAngleRelativeToRobot + maxOffset).getAngle() < 0;
        bool ballIsTooFarRight = (ballAngleRelativeToRobot - maxOffset).getAngle() > 0;
        if (ballIsTooFarLeft || ballIsTooFarRight) {
            compensationVelocity += (robot->angle + M_PI_2).toVector2(ballAngleRelativeToRobot);
        }
        robotCommand.vel = compensationVelocity + robot->angle.toVector2(maxForwardsVelocity);
        robotCommand.angle = robot->angle;
        return robotCommand;
    }

    case backwards: {
        Angle maxOffset = 0.02*M_PI; // in rad
        bool ballIsTooFarLeft = (ballAngleRelativeToRobot + maxOffset).getAngle() < 0;
        bool ballIsTooFarRight = (ballAngleRelativeToRobot - maxOffset).getAngle() > 0;
        if (ballIsTooFarLeft || ballIsTooFarRight) {
            compensationVelocity += (robot->angle + M_PI_2).toVector2(ballAngleRelativeToRobot);
        }
        robotCommand.vel = compensationVelocity + robot->angle.toVector2(maxForwardsVelocity);
        robotCommand.angle = robot->angle;
        robotCommand.dribbler = 1;
        return robotCommand;
    }
    case defaultTravel:break;
    }

    std::cout << "travel case not handled" << std::endl;
    return {
    };
}

} //control
} //ai
} //rtt