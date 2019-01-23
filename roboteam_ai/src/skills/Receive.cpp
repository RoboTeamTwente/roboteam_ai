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
};

Vector2 Receive::computeInterceptPoint(Vector2 startBall, Vector2 endBall) {
    Vector2 interceptionPoint;

    // For now we pick the closest point to the (predicted) line of the ball for any 'regular' interception
    interceptionPoint = Vector2(robot->pos).project(startBall,endBall);

    return interceptionPoint;
}

Receive::Status Receive::onUpdate() {
    if (!coach::Coach::doesRobotHaveBall(robot->id, true)) {
        if (Coach::isPassed() && Vector2(ball->vel).length() < 0.01) {
            checkTicks++;
            if (checkTicks > maxCheckTicks) return Status::Success;
        }
        roboteam_msgs::RobotCommand command;
        command.id = robot->id;
        command.w = static_cast<float>((Vector2(ball->pos) - Vector2(robot->pos)).angle()); //Rotates towards the ball
        command.use_angle = 1;
        if (Vector2(ball->vel).length() > 1.0) {
            ballStartPos = ball->pos;
            Vector2 ballStartVel = ball->vel;
            Vector2 ballEndPos = ballStartPos + ballStartVel * constants::MAX_INTERCEPT_TIME;
            Vector2 interceptPoint = Receive::computeInterceptPoint(ballStartPos, ballEndPos);

            Vector2 velocities = goToPos.goToPos(robot, interceptPoint, GoToType::basic);

            command.x_vel = static_cast<float>(velocities.x);
            command.y_vel = static_cast<float>(velocities.y);
            command.dribbler = 1;
        }
        publishRobotCommand(command);
        return Status::Running;
    } else {
        return Status::Success;
    }
}

}
}
