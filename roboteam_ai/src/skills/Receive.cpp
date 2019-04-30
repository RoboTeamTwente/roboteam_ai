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
    readyToPassSet = false;
}

Receive::Status Receive::onUpdate() {
    if (world::world->robotHasBall(robot->id, true)) {
        std::cout << robot->id << " received" << std::endl;
        return Status::Success;
    }

    if (coach::g_pass.getRobotBeingPassedTo() != robot->id) {
        std::cout << robot->id << " not being passed to anymore" << std::endl;
        return Status::Failure;
    }

    if (ballPlacement) {
        Vector2 ballPlacementTarget = coach::g_ballPlacement.getBallPlacementPos();
        auto behindTargetPos = control::PositionUtils::getPositionBehindPositionToPosition(
                Constants::ROBOT_RADIUS(),
                ballPlacementTarget,
                ball->pos);

        moveToCatchPosition(behindTargetPos);
        std::cerr << "BALL PLACEMENT HUH?" << std::endl;

        if (isInPosition(behindTargetPos)) {
            coach::g_pass.setReadyToReceivePass(true);
        }
    } else {
        if (coach::g_pass.isPassed()) {
            // Check if the ball was deflected
            if (passFailed()) {
                publishRobotCommand();
                return Status::Failure;
            }

            intercept();
            return Status::Running;
        }

        // Check if robot is in position, otherwise turn towards ball
        if (isInPosition()) {
            if (!readyToPassSet) {
                readyToPassSet = true;
                coach::g_pass.setReadyToReceivePass(true);
            }
        }

        command.w = (ball->pos - robot->pos).toAngle().getAngle();
        publishRobotCommand();
        return Status::Running;
    }
}

void Receive::onTerminate(Status s) {
    command.x_vel = 0;
    command.y_vel = 0;
    command.dribbler = 0;
    publishRobotCommand();

    if (robot->id != -1) {
        std::cout << robot->id << " receiver terminated pass" << std::endl;
        coach::g_pass.resetPass(robot->id);
    }
}


// Pick the closest point to the (predicted) line of the ball for any 'regular' interception
Vector2 Receive::computeInterceptPoint(const Vector2& startBall, const Vector2& endBall) {
    return robot->pos.project(startBall, endBall);
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
    double ballAngle = (ball->pos - robot->pos).toAngle().getAngle();

    ballStartPos = ball->pos;
    ballStartVel = ball->vel;
    ballEndPos = ballStartPos + ballStartVel * Constants::MAX_INTERCEPT_TIME();
    Vector2 interceptPoint = computeInterceptPoint(ballStartPos, ballEndPos);

    Vector2 velocities = basicGtp.getPosVelAngle(robot, interceptPoint).vel;
    velocities = control::ControlUtils::velocityLimiter(velocities);
    command.x_vel = static_cast<float>(velocities.x);
    command.y_vel = static_cast<float>(velocities.y);
    command.w = ballAngle;
    publishRobotCommand();
}

bool Receive::passFailed() {
    //TODO: Remove print statements and make 1 big if statement
//    if (ballDeflected()) {
//        return true;
//    }

    if (ball->vel.length() < 0.1) {
        std::cout << robot->id << " ball going too slow" << std::endl;
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
