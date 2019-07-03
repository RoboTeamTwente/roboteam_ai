//
// Created by rolf on 04/12/18.
//

#include "GetBall.h"
#include "../utilities/Constants.h"
#include "../control/ControlUtils.h"

namespace rtt {
namespace ai {

//TODO: do obstacle checking and return fail if there is an obstacle in the way.
//GetBall turns the robot to the ball and softly approaches with dribbler on in an attempt to get the ball.
GetBall::GetBall(string name, bt::Blackboard::Ptr blackboard)
        :Skill(std::move(name), std::move(blackboard)) { }

void GetBall::onInitialize() {
    ballHandlePosControl.setCanMoveInDefenseArea(properties->getBool("canMoveInDefenseArea"));
}

GetBall::Status GetBall::onUpdate() {
    if ((lockedTargetPos - ball->pos).length() > 0.2) {
        lockedTargetPos = ball->pos + (ball->pos - robot->pos).stretchToLength(0.1);
    }
    auto c = ballHandlePosControl.getRobotCommand(
            robot, lockedTargetPos, control::BallHandlePosControl::TravelStrategy::BACKWARDS);

    if (ballHandlePosControl.getStatus() == control::BallHandlePosControl::Status::SUCCESS) {
        return Status::Success;
    }

    command = c.makeROSCommand();
    publishRobotCommand();

    return Status::Running;
}

void GetBall::onTerminate(Status s) {
    if (properties->getBool("dribbleOnTerminate")) {
        command.dribbler = 20;
        command.x_vel = 0;
        command.y_vel = 0;
        command.w = robot->angle;
        publishRobotCommand();
    }
}

}//rtt
}//ai