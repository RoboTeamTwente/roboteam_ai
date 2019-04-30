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

RobotCommand BallHandlePosControl::getPosVelAngle(const RobotPtr &r,
        const Vector2 &targetP, const Angle &targetA) {

    auto b = world::world->getBall();
    ball = b;
    robot = r;
    targetPos = targetP;
    finalTargetPos = targetP;
    targetAngle = targetA;
    finalTargetAngle = targetA;
    robotToBall = ball->pos - robot->pos;
    ballToRobot = robot->pos - ball->pos;

    if (! ball) {
        std::cout << "Can't control the ball with no ball, you stupid" << std::endl;
        return {};
    }

    //TODO: make seperate functions, low priority for now

    // if we do not have the ball yet, go get it
    {
        bool robotDoesNotHaveBall = ! robot->hasBall();
        bool robotIsTooFarFromBall = ballToRobot.length2() > maxBallDistance*maxBallDistance;

        if (robotDoesNotHaveBall || robotIsTooFarFromBall) {
            backwardsProgress = start;

            std::cout << "we do not have a ball yet idiot" << std::endl;
            Vector2 target = ball->pos + ballToRobot.stretchToLength(targetBallDistance);
            auto pva = numTreePosController.getPosVelAngle(robot, target);
            RobotCommand robotCommand;
            if (robotIsTooFarFromBall) robotCommand.angle = pva.angle;
            else robotCommand.angle = robotToBall.toAngle();
            robotCommand.vel = pva.vel;
            return robotCommand;
        }
    }

    if (backwardsProgress != start) return travelWithBall(backwards);
    lockedAngle = robotToBall.toAngle();

    double deltaPosSquared = (finalTargetPos - ball->pos).length2();

    // if the distance to the target is large, rotate with the ball forwards and move towards the target
    {
        if (deltaPosSquared > 1.0) {
            backwardsProgress = start;

            // rotate TOWARDS the target
            targetAngle = finalTargetPos - robot->pos;
            if (fabs(targetAngle - robot->angle) > angleErrorMargin) {
                return rotateWithBall(rotateAroundBall);
            }
            else {
                return travelWithBall(forwards);
            }
        }
    }

    // if the distance to the target is small (but d>errorMargin) and the robot cannot rotate around the ball
    {
        bool robotIsAtLocation = deltaPosSquared < errorMargin*errorMargin;

        if (! robotIsAtLocation) {
            // rotate AWAY from the target
            targetAngle = (robot->pos - finalTargetPos).toAngle();
            if (fabs(targetAngle + M_PI - robot->angle) < angleErrorMargin) {
                return travelWithBall(forwards);
            }
            Angle deltaAngle = targetAngle - robot->angle;

            if (fabs(deltaAngle) > angleErrorMargin) {
                return rotateWithBall(rotateAroundBall);
            }
            else {
                return travelWithBall(backwards);
            }
        }
        else {
            if (fabs(targetAngle - robot->angle) > angleErrorMargin*0.25) {
                return rotateWithBall(rotateAroundRobot);
            }
            else {
                return {};
            }
        }
    }
}

RobotCommand BallHandlePosControl::rotateWithBall(RotateStrategy rotateStrategy) {
    std::stringstream ss;
    ss << "rotating with strategy: ";
    switch (rotateStrategy) {
    case rotateAroundBall:ss << "aroundBall";
        break;
    case rotateAroundRobot:ss << "aroundRobot";
        break;
    case fastest:ss << "fastest";
        break;
    case safest:ss << "safest";
        break;
    case defaultRotate:ss << "default";
        break;
    }
    std::cout << ss.str() << std::endl;

    RobotCommand robotCommand;
    Angle deltaAngle = targetAngle - robot->angle;
    Angle compensationAngle = 0;
    Vector2 compensationVelocity = Vector2();
    if (ball->visible) {
        compensationAngle = (ball->pos - robot->pos).toAngle() - robot->angle;
        compensationVelocity = (robot->angle + M_PI_2).toVector2(compensationAngle);
    }

    switch (rotateStrategy) {
    case rotateAroundBall: {
        if (deltaAngle > 0) robotCommand.vel = robotToBall.rotate(- M_PI_2);
        else robotCommand.vel = robotToBall.rotate(M_PI_2);

        robotCommand.vel += robotToBall.stretchToLength(targetBallDistance) - robotToBall;
        robotCommand.vel.stretchToLength(fabs(deltaAngle));
        robotCommand.angle = robotToBall.toAngle();

        return robotCommand;
    }
    case rotateAroundRobot: {
        int direction = targetAngle - robot->angle > 0.0 ? 1 : - 1;
        robotCommand.angle = Angle(robot->angle + maxAngularVelocity*direction + compensationAngle);
        robotCommand.dribbler = 1;
        return robotCommand;
    }
    case fastest: {
        int direction = targetAngle - robot->angle > 0.0 ? 1 : - 1;
        robotCommand.angle = Angle(robot->angle + maxAngularVelocity*direction + compensationAngle);
        if ((finalTargetPos - robot->pos).length2() > errorMargin*errorMargin) {
            robotCommand.vel = compensationVelocity;
        }
        robotCommand.dribbler = 1;
        return robotCommand;
    }
    case safest:break;
    case defaultRotate:break;
    }

    std::cout << "rotate case not handled" << std::endl;
    return {};
}

RobotCommand BallHandlePosControl::travelWithBall(TravelStrategy travelStrategy) {

    std::stringstream ss;
    ss << "travel with strategy: ";
    switch (travelStrategy) {
    case forwards:ss << "forwards";
        break;
    case backwards:ss << "backwards";
        break;
    case defaultTravel:ss << "default";
        break;
    }
    std::cout << ss.str() << std::endl;

    RobotCommand robotCommand;
    Vector2 compensationVelocity = Vector2();
    Angle ballAngleRelativeToRobot = 0;
    if (ball->visible) {
        ballAngleRelativeToRobot = robotToBall.toAngle() - robot->angle;
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
        if (backwardsProgress != overshooting && backwardsProgress != dribbling) {
            approachPosition = ball->pos + ballToRobot.stretchToLength(0.05);
        }

        if (!robot->hasBall()) {
            lockedAngle = robotToBall.toAngle();
        }
        updateBackwardsProgress();

        switch (backwardsProgress) {
        case start: return startTravelBackwards();
        case turning:return sendTurnCommand();
        case approaching:return sendApproachCommand();
        case overshooting:return sendOvershootCommand();
        case dribbling:return sendDribblingCommand();
        case dribbleBackwards:return sendDribbleBackwardsCommand();
        case success: {
            return {};
        }
        case fail: {
            backwardsProgress = start;
            return {};
        }
        }
    }

    case defaultTravel:break;
    }

    std::cout << "travel case not handled" << std::endl;
    return {
    };
}

RobotCommand BallHandlePosControl::startTravelBackwards() {
    count = 0;
    backwardsProgress = turning;
    return sendTurnCommand();
}

RobotCommand BallHandlePosControl::sendTurnCommand() {
    targetAngle = (robot->pos - finalTargetPos).toAngle();
    if (fabs(targetAngle - robot->angle) < angleErrorMargin) {
        lockedAngle = targetAngle;
        backwardsProgress = approaching;
    }
    targetPos = finalTargetPos;
    return rotateWithBall(rotateAroundBall);
}

RobotCommand BallHandlePosControl::sendApproachCommand() {
    RobotCommand command;
    command.dribbler = 1;
    command.vel = robotToBall.stretchToLength(maxBackwardsVelocity);
    command.angle = lockedAngle;
    return command;
}

RobotCommand BallHandlePosControl::sendOvershootCommand() {
    RobotCommand command;
    command.dribbler = 1;
    command.vel = (approachPosition - robot->pos).stretchToLength(maxBackwardsVelocity);
    command.angle = lockedAngle;
    return command;
}

RobotCommand BallHandlePosControl::sendDribblingCommand() {
    RobotCommand command;
    command.dribbler = 1;
    command.angle = lockedAngle;
    return command;
}

RobotCommand BallHandlePosControl::sendDribbleBackwardsCommand() {
    Angle ballAngleRelativeToRobot = Angle();
    if (ball->visible) {
        ballAngleRelativeToRobot = robotToBall.toAngle() - robot->angle;
    }

    RobotCommand command;
    command.dribbler = 1;
    command.angle = lockedAngle;
    command.vel = lockedAngle.toVector2(maxBackwardsVelocity).rotate(M_PI);

    Angle maxOffset = 0.02*M_PI; // in rad
    bool ballIsTooFarLeft = (ballAngleRelativeToRobot + maxOffset).getAngle() < 0;
    bool ballIsTooFarRight = (ballAngleRelativeToRobot - maxOffset).getAngle() > 0;
    if (ballIsTooFarLeft || ballIsTooFarRight) {
        command.vel += (robot->angle + M_PI_2).toVector2(ballAngleRelativeToRobot);
    }
    return command;
}


void BallHandlePosControl::updateBackwardsProgress() {
    std::stringstream ss;
    ss << "backwards progress:                  ";
    switch (backwardsProgress) {
    case overshooting:ss << "overshooting";break;
    case dribbling:ss << "dribbling";break;
    case dribbleBackwards:ss << "dribbleBackwards";break;
    case success:ss << "success";break;
    case fail:ss << "fail";break;
    case start:ss << "start";break;
    case turning:ss << "turning";break;
    case approaching:ss << "approaching";break;
    }
    std::cout << ss.str() << std::endl;

    if (robotToBall.length2() > 0.5) {
        backwardsProgress = fail;
        return;
    }
    Angle angleDifference = robot->angle - robotToBall.toAngle();

    if (backwardsProgress != dribbling) count = 0;

    switch(backwardsProgress) {
    case turning: {
        if (fabs(angleDifference) < angleErrorMargin) {
            backwardsProgress = approaching;
            return;
        }
        return;
    }
    case approaching:{
        if (fabs(angleDifference) >= angleErrorMargin) {
            backwardsProgress = turning;
            return;
        }
        if (robot->hasBall()) {
            backwardsProgress = overshooting;
            return;
        }
        else {
            return;
        }
    }
    case overshooting:{
        if (! robot->hasBall()) {
            backwardsProgress = turning;
            return;
        }
        if (((approachPosition - robot->pos)).length() < 0.06) {
            backwardsProgress = dribbling;
            return;
        }
        return;
    }
    case dribbling:{
        if (! robot->hasBall()) {
            backwardsProgress = approaching;
            return;
        }
        count ++;
        if (count > 35) {
            backwardsProgress = dribbleBackwards;
            return;
        }
        return;
    }
    case dribbleBackwards:{
        if (! robot->hasBall()) {
            backwardsProgress = approaching;
            return;
        }
        Angle offsetAngle = (robot->pos - finalTargetPos).toAngle() - robot->angle;
        if (offsetAngle > M_PI * 0.05) {
            backwardsProgress = start;
            return;
        }
        if ((robot->pos - finalTargetPos).length2() < errorMargin*errorMargin) {
            backwardsProgress = success;
            return;
        }
        return;
    }
    case fail:
    case start:
    default:return;
    }
}

} //control
} //ai
} //rtt