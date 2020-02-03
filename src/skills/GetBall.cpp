//
// Created by rolf on 04/12/18.
//

#include "skills/GetBall.h"

#include "control/ControlUtils.h"
#include "utilities/Constants.h"

namespace rtt::ai {

// TODO: do obstacle checking and return fail if there is an obstacle in the way.
// GetBall turns the robot to the ball and softly approaches with dribbler on in an attempt to get the ball.
GetBall::GetBall(string name, bt::Blackboard::Ptr blackboard) : Skill(std::move(name), std::move(blackboard)) {}

void GetBall::onInitialize() { ballHandlePosControl.setCanMoveInDefenseArea(properties->getBool("canMoveInDefenseArea")); }

GetBall::Status GetBall::onUpdate() {
    if ((lockedTargetPos - ball->getPos()).length() > 0.2) {
        lockedTargetPos = ball->getPos() + (ball->getPos() - robot->pos).stretchToLength(0.1);
    }
    auto c = ballHandlePosControl.getRobotCommand(world, field, robot, lockedTargetPos, control::BallHandlePosControl::TravelStrategy::BACKWARDS);

    if (ballHandlePosControl.getStatus() == control::BallHandlePosControl::Status::SUCCESS) {
        return Status::Success;
    }

    command = c.makeROSCommand();
    publishRobotCommand();

    return Status::Running;
}

void GetBall::onTerminate(Status s) {
    if (properties->getBool("dribbleOnTerminate")) {
        command.set_dribbler(31);
        command.mutable_vel()->set_x(0);
        command.mutable_vel()->set_y(0);
        command.set_w(robot->angle);
        publishRobotCommand();
    }
}

}  // namespace rtt::ai