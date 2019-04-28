//
// Created by robzelluf on 1/22/19.
//

#include <roboteam_ai/src/coach/PassCoach.h>
#include <roboteam_ai/src/coach/BallplacementCoach.h>
#include "Receive.h"

namespace rtt {
namespace ai {

Receive::Receive(string name, bt::Blackboard::Ptr blackboard) : Skill(std::move(name), std::move(blackboard)) {}

void Receive::onInitialize() {
    ballPlacement = properties->getBool("BallPlacement");
    isBallOnPassedSet = false;
}

Receive::Status Receive::onUpdate() {
    if (world::world->ourRobotHasBall(robot->id)) {
        return Status::Success;
    }

    if (coach::g_pass.getRobotBeingPassedTo() != robot->id) {
        return Status::Failure;
    }

    if (ballPlacement) {
        Vector2 ballPlacementTarget = coach::g_ballPlacement.getBallPlacementPos();
        auto behindTargetPos = control::PositionUtils::getPositionBehindPositionToPosition(
                Constants::ROBOT_RADIUS(),
                ballPlacementTarget,
                ball->pos);

        moveToCatchPosition(behindTargetPos);

        if (isInPosition(behindTargetPos)) {
            coach::g_pass.setReadyToReceivePass(true);
        }
    } else {
        // Check if robot is in position, otherwise turn towards ball
        if (isInPosition()) {
            coach::g_pass.setReadyToReceivePass(true);
        } else {
            command.w = static_cast<float>((Vector2(ball->pos) - robot->pos).angle());
            publishRobotCommand();
            return Status::Running;
        }
    }

    if (coach::g_pass.isPassed()) {
        // Remember the status of the ball at the moment of passing
        if(!isBallOnPassedSet) {
            ballOnPassed = ball;
            isBallOnPassedSet = true;
        }

        // Check if the ball was deflected
        if (isBallOnPassedSet && passFailed()) {
            publishRobotCommand();
            return Status::Failure;
        }

        intercept();
        return Status::Running;
    }

    return Status::Running;
}

void Receive::onTerminate(Status s) {
    command.x_vel = 0;
    command.y_vel = 0;
    command.dribbler = 0;
    publishRobotCommand();

    if (robot->id != -1) {
        coach::g_pass.resetPass();
    }
}


// Pick the closest point to the (predicted) line of the ball for any 'regular' interception
Vector2 Receive::computeInterceptPoint(const Vector2& startBall, const Vector2& endBall) {
    return Vector2(robot->pos).project(startBall, endBall);
}

// check if the robot is in the desired position to catch the ball
bool Receive::isInPosition(const Vector2& behindTargetPos) {
    bool isAimedAtBall = control::ControlUtils::robotIsAimedAtPoint(robot->id, true, ball->pos, 0.3*M_PI);

    if (ballPlacement) {
        bool isBehindTargetPos = behindTargetPos.dist(robot->pos) < 0.03;
        return isBehindTargetPos  && isAimedAtBall;
    }
    return isAimedAtBall;

}

void Receive::moveToCatchPosition(Vector2 position) {
    control::PosVelAngle pva = numTreeGtp.getPosVelAngle(robot, position);
    pva.vel = control::ControlUtils::velocityLimiter(pva.vel, rtt::ai::Constants::MAX_VEL());
    command.x_vel = static_cast<float>(pva.vel.x);
    command.y_vel = static_cast<float>(pva.vel.y);

    if (position.dist(robot->pos) < 0.6) {
        command.w = static_cast<float>((Vector2(ball->pos) - robot->pos).angle());
    } else {
        command.w = static_cast<float>((position - robot->pos).angle());

    }
    publishRobotCommand();
}

void Receive::intercept() {
    double ballAngle = ((Vector2) ball->pos - robot->pos).angle();

    ballStartPos = ball->pos;
    ballStartVel = ball->vel;
    ballEndPos = ballStartPos + ballStartVel * Constants::MAX_INTERCEPT_TIME();
    Vector2 interceptPoint = Receive::computeInterceptPoint(ballStartPos, ballEndPos);

    Vector2 velocities = basicGtp.getPosVelAngle(robot, interceptPoint).vel;

    velocities = control::ControlUtils::velocityLimiter(velocities);


    command.x_vel = static_cast<float>(velocities.x);
    command.y_vel = static_cast<float>(velocities.y);
    command.w = ballAngle;

    publishRobotCommand();
}

bool Receive::passFailed() {
    //TODO: Remove print statements and make 1 big if statement
    if (ballDeflected()) {
        return true;
    }

    if (ball->vel.length() < 0.1) {
        return true;
    }

    return false;

}
bool Receive::ballDeflected() {
    Angle robotToBallAngle = (robot->pos - ball->pos).toAngle();
    Angle ballVelocityAngle = (ball->vel).toAngle();

    return abs(robotToBallAngle - ballVelocityAngle) > BALL_DEFLECTION_ANGLE;

}

} // ai
} // rtt
