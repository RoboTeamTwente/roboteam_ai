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
    checkTicks = 0;
    initializedBall = false;
    ballPlacement = properties->getBool("BallPlacement");
};

Receive::Status Receive::onUpdate() {
    if (World::ourBotHasBall(robot->id)) {
        return Status::Success;
    }

    Vector2 ballPlacementTarget = coach::g_ballPlacement.getBallPlacementPos();
    auto behindTargetPos = coach::g_generalPositionCoach.getPositionBehindPositionToPosition(Constants::ROBOT_RADIUS(),
                                                                                             ballPlacementTarget,
                                                                                             ball->pos);

    ballStartPos = ball->pos;


    Vector2 ballVel = ball->vel;
    if (coach::g_pass.isPassed() && ballVel.length() < 0.1) {
        return Status::Success;
    }

    if (coach::g_pass.isPassed()) {
        intercept();
        return Status::Running;
    }


    if (isInPosition(behindTargetPos)) {
        std::cout << "receiver in position!" << std::endl;

        coach::g_pass.setReadyToReceivePass(true);
    } else {
        moveToCatchPosition(behindTargetPos);
    }
    return Status::Running;
}

void Receive::onTerminate(Status s) {
    command.x_vel = 0;
    command.y_vel = 0;
    command.dribbler = 0;
    publishRobotCommand();
}


// Pick the closest point to the (predicted) line of the ball for any 'regular' interception
Vector2 Receive::computeInterceptPoint(Vector2 startBall, Vector2 endBall) {
    return Vector2(robot->pos).project(startBall, endBall);
}

// check if the robot is in the desired position to catch the ball
bool Receive::isInPosition(Vector2 behindTargetPos) {
    bool isAimedAtBall = control::ControlUtils::robotIsAimedAtPoint(robot->id, true, ball->pos);

    if (ballPlacement) {
        bool isBehindTargetPos = behindTargetPos.dist(robot->pos) < 0.02;
        return isBehindTargetPos && isAimedAtBall;
    }
    return isAimedAtBall;

}

void Receive::moveToCatchPosition(Vector2 position) {
    std::cout << "moving to catch position" << std::endl;

    control::PosVelAngle pva = numTreeGtp.getPosVelAngle(robot, position);
    pva.vel = control::ControlUtils::velocityLimiter(pva.vel, rtt::ai::Constants::MAX_VEL(), 0.3);
    command.x_vel = static_cast<float>(pva.vel.x);
    command.y_vel = static_cast<float>(pva.vel.y);

    if (position.dist(robot->pos) < 0.2) {
        command.w = static_cast<float>((Vector2(ball->pos) - robot->pos).angle());
    } else {
        command.w = static_cast<float>((position - robot->pos).angle());

    }
    publishRobotCommand();
}

void Receive::intercept() {
    double ballAngle = ((Vector2) robot->pos - ball->pos).angle();

    Vector2 ballStartVel = ball->vel;
        Vector2 ballEndPos = ballStartPos + ballStartVel * Constants::MAX_INTERCEPT_TIME();
        Vector2 interceptPoint = Receive::computeInterceptPoint(ballStartPos, ballEndPos);

        std::cout << "intercepting at: " << interceptPoint << std::endl;

        Vector2 velocities = basicGtp.getPosVelAngle(robot, interceptPoint).vel;
        velocities = control::ControlUtils::velocityLimiter(velocities);
        command.x_vel = static_cast<float>(velocities.x);
        command.y_vel = static_cast<float>(velocities.y);
        command.w = ballAngle;
        command.dribbler = 1;

    publishRobotCommand();
}


} // ai
} // rtt
