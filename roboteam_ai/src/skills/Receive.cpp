//
// Created by robzelluf on 1/22/19.
//

#include <roboteam_ai/src/coach/PassCoach.h>
#include "Receive.h"

namespace rtt {
namespace ai {

Receive::Receive(string name, bt::Blackboard::Ptr blackboard)
    :Skill(std::move(name), std::move(blackboard)) {
}

void Receive::onInitialize() {
    checkTicks = 0;
    initializedBall = false;
    coach::g_pass.setReadyToReceivePass(true);
};

Vector2 Receive::computeInterceptPoint(Vector2 startBall, Vector2 endBall) {
    Vector2 interceptionPoint;

    // For now we pick the closest point to the (predicted) line of the ball for any 'regular' interception
    interceptionPoint = Vector2(robot->pos).project(startBall,endBall);

    return interceptionPoint;
}

Receive::Status Receive::onUpdate() {
    if (!World::ourBotHasBall(robot->id)) {
        if (Vector2(ball->vel).length() > 0.6 && !initializedBall) {
            initializedBall = true;
            ballStartPos = ball->pos;
            ballStartVel = ball->vel;
        }

        if (coach::g_pass.isPassed() && Vector2(ball->vel).length() < 0.01) {
            checkTicks++;
            if (checkTicks >= maxCheckTicks) return Status::Success;
        }

        command.w = static_cast<float>((Vector2(ball->pos) - Vector2(robot->pos)).angle()); //Rotates towards the ball
        double ballAngle = ((Vector2)robot->pos - ball->pos).angle();
        if (Vector2(ball->vel).length() > 0.6 && (ballAngle - Vector2(ball->vel).angle()) < 0.5) {
            Vector2 ballStartVel = ball->vel;
            Vector2 ballEndPos = ballStartPos + ballStartVel * Constants::MAX_INTERCEPT_TIME();
            Vector2 interceptPoint = Receive::computeInterceptPoint(ballStartPos, ballEndPos);

            control::PosVelAngle velocities = goToPos.goToPos(robot, interceptPoint, GoToType::BASIC);
            velocities.vel = control::ControlUtils::VelocityLimiter(velocities.vel);
            command.x_vel = static_cast<float>(velocities.vel.x);
            command.y_vel = static_cast<float>(velocities.vel.y);
            command.dribbler = 1;
        }
        publishRobotCommand();
        return Status::Running;
    } else {
        stopDribbleTick++;
        if (stopDribbleTick < stopDribbleTicks) return Status::Running;
        else return Status::Success;
    }
}
void Receive::onTerminate(Status s) {
    command.x_vel = 0;
    command.y_vel = 0;
    command.dribbler = 0;
    publishRobotCommand();
}

}
}
