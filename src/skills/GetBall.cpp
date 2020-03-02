//
// Created by rolf on 04/12/18.
//

#include <skills/GetBall.h>

namespace rtt::ai {

// TODO: do obstacle checking and return fail if there is an obstacle in the way.
// GetBall turns the robot to the ball and softly approaches with dribbler on in an attempt to get the ball.
GetBall::GetBall(std::string name, bt::Blackboard::Ptr blackboard) : Skill(std::move(name), std::move(blackboard)) {}

void GetBall::onInitialize() { robot->getControllers().getBallHandlePosController()->setCanMoveInDefenseArea(properties->getBool("canMoveInDefenseArea")); }

GetBall::Status GetBall::onUpdate() {
    if ((lockedTargetPos - ball->get()->getPos()).length() > 0.2) {
        lockedTargetPos = ball->get()->getPos() + (ball->get()->getPos() - robot->get()->getPos()).stretchToLength(0.1);
    }
    auto c = robot->getControllers().getBallHandlePosController()->getRobotCommand(robot->get()->getId(), lockedTargetPos, control::BallHandlePosControl::TravelStrategy::BACKWARDS);

    if (robot->getControllers().getBallHandlePosController()->getStatus() == control::BallHandlePosControl::Status::SUCCESS) {
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
        command.set_w(robot->get()->getAngle());
        publishRobotCommand();
    }
}

}  // namespace rtt::ai