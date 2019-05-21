//
// Created by robzelluf on 5/13/19.
//

#include "DribbleForward.h"

namespace rtt {
namespace ai {

DribbleForward::DribbleForward(string name, bt::Blackboard::Ptr blackboard)
        : Skill(std::move(name), std::move(blackboard)) {}

void DribbleForward::onInitialize() {
    initialBallPos = ball->pos;
    currentProgress = DRIBBLING;
    if (properties->hasDouble("dribbleDistance")) {
        dribbleDistance = properties->getDouble("dribbleDistance");
    } else {
        dribbleDistance = 0.9;
    }

    Angle angleToGoal = (world::field->get_their_goal_center() - ball->pos).toAngle();
    targetPos = ball->pos + Vector2{dribbleDistance, 0}.rotate(angleToGoal);
}


bt::Node::Status DribbleForward::onUpdate() {
    switch (currentProgress) {
        case GETTING_BALL: {
            if(robot->hasBall()) {
                currentProgress = DRIBBLING;
                targetPos = robot->pos + Vector2({dribbleDistance, 0}).rotate(robot->angle);
                return Status::Running;
            }

            auto pva = basicGtp.getPosVelAngle(robot, targetPos);
            command.x_vel = pva.vel.x;
            command.y_vel = pva.vel.y;
            command.w = (ball->pos - robot->pos).toAngle();
            break;
        }
        case DRIBBLING: {
            if ((ball->pos - targetPos).length() < 0.05 || (ball->pos - initialBallPos).length() >= dribbleDistance) {
                return Status::Success;
            }

            auto c = ballHandlePosControl.getRobotCommand(robot, targetPos, robot->angle);
            command = c.makeROSCommand();
            break;

        }
    }

    publishRobotCommand();
    return Status::Running;

}

} //ai
} //rtt
