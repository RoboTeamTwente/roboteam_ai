//
// Created by robzelluf on 1/22/19.
//

#include "Receive.h"

namespace rtt {
namespace ai {

Receive::Receive(string name, bt::Blackboard::Ptr blackboard)
    :Skill(std::move(name), std::move(blackboard)) {
}

void Receive::onInitialize() {
    checkTicks = 0;
    initializedBall = false;
    coach::Coach::setReadyToReceivePass(true);
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

        if (Coach::isPassed() && Vector2(ball->vel).length() < 0.01) {
            checkTicks++;
            if (checkTicks >= maxCheckTicks) return Status::Success;
        }
        roboteam_msgs::RobotCommand command;
        command.id = robot->id;
        command.w = static_cast<float>((Vector2(ball->pos) - Vector2(robot->pos)).angle()); //Rotates towards the ball
        command.use_angle = 1;
        double ballAngle = ((Vector2)robot->pos - ball->pos).angle();
        if (Vector2(ball->vel).length() > 0.6 && (ballAngle - Vector2(ball->vel).angle()) < 0.5) {
            Vector2 ballStartVel = ball->vel;
            Vector2 ballEndPos = ballStartPos + ballStartVel * Constants::MAX_INTERCEPT_TIME();
            Vector2 interceptPoint = Receive::computeInterceptPoint(ballStartPos, ballEndPos);

            Vector2 velocities = goToPos.goToPos(robot, interceptPoint, GoToType::basic);
            velocities=control::ControlUtils::VelocityLimiter(velocities);
            command.x_vel = static_cast<float>(velocities.x);
            command.y_vel = static_cast<float>(velocities.y);
            command.dribbler = 1;
        }
        publishRobotCommand(command);
        return Status::Running;
    } else {
        stopDribbleTick++;
        if (stopDribbleTick < stopDribbleTicks) return Status::Running;
        else return Status::Success;
    }
}
void Receive::onTerminate(Status s) {
    roboteam_msgs::RobotCommand command;
    command.x_vel = 0;
    command.y_vel = 0;
    command.id = robot->id;
    command.dribbler = 0;

    publishRobotCommand(command);
}

}
}
