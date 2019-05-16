//
// Created by robzelluf on 1/22/19.
//

#include <roboteam_ai/src/coach/PassCoach.h>
#include <roboteam_ai/src/coach/BallplacementCoach.h>
#include <roboteam_ai/src/interface/api/Input.h>
#include "Receive.h"

namespace rtt {
namespace ai {

Receive::Receive(string name, bt::Blackboard::Ptr blackboard) : Skill(std::move(name), std::move(blackboard)) {}

void Receive::onInitialize() {
    readyToPassSet = false;
   // basicGtp.setCanMoveInDefenseArea(false);
}

Receive::Status Receive::onUpdate() {
    if (world::world->robotHasBall(robot->id, true)) {
        return Status::Success;
    }

    if (coach::g_pass.getRobotBeingPassedTo() != robot->id) {
        return Status::Failure;
    }


    if (coach::g_pass.isPassed()) {
        // Check if the ball was deflected
        if (passFailed()) {
            publishRobotCommand();
            return Status::Failure;
        }

        intercept();
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

void Receive::onTerminate(Status s) {
    readyToPassSet = false;
    currentProgress = POSITIONING;
    if (passFailed() || coach::g_pass.getRobotBeingPassedTo() != robot->id) {
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
    return isAimedAtBall;
}


void Receive::intercept() {
    ball = world::world->getBall();
    double ballAngle = (ball->pos - robot->pos).toAngle().getAngle();

    ballStartPos = ball->pos;
    ballStartVel = ball->vel;
    ballEndPos = ballStartPos + ballStartVel * Constants::MAX_INTERCEPT_TIME();
    Vector2 interceptPoint = computeInterceptPoint(ballStartPos, ballEndPos);

    Vector2 velocities = robot->getBasicGtp()->getPosVelAngle(robot, interceptPoint).vel;
    velocities = control::ControlUtils::velocityLimiter(velocities);
    command.x_vel = static_cast<float>(velocities.x);
    command.y_vel = static_cast<float>(velocities.y);
    command.w = ball->vel.stretchToLength(-1).toAngle();

    interface::Input::drawData(interface::Visual::INTERCEPT, {ballStartPos, ballEndPos}, Qt::darkCyan, robot->id, interface::Drawing::LINES_CONNECTED);
    interface::Input::drawData(interface::Visual::INTERCEPT, {interceptPoint}, Qt::cyan, robot->id, interface::Drawing::DOTS, 5, 5);

}

bool Receive::passFailed() {
    return (ball->vel.length() < 0.3);
}


bool Receive::ballDeflected() {
    Angle robotToBallAngle = (robot->pos - ball->pos).toAngle();
    Angle ballVelocityAngle = (ball->vel).toAngle();

    return abs(robotToBallAngle - ballVelocityAngle) > BALL_DEFLECTION_ANGLE;

}

} // ai
} // rtt
